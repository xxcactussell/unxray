#include "StdAfx.h"
#include "PHWorld.h"
#include "PhysicsCommon.h"
#include "ExtendedGeom.h"
#include "PHCollideValidator.h"
#include "params.h"
#ifdef DEBUG
#include "debug_output.h"
#endif

#include "xrServerEntities/PHSynchronize.h"
#include "xrServerEntities/PHNetState.h"
#include "GeometryBits.h"
#include "console_vars.h"
#include "PHCommander.h"
#include "PHSimpleCalls.h"
#include "xrCore/FS_internal.h"
#include "xrCDB/xr_area.h"
#include "xrEngine/defines.h"
#include "xrEngine/device.h"
#include "xrEngine/GameFont.h"
#include "xrEngine/PerformanceAlert.hpp"

CPHWorld* ph_world = 0;

IPHWorld* physics_world() { return ph_world; }
void create_physics_world(bool mt, CObjectSpace* os, CObjectList* lo)
{
    ZoneScoped;
    ph_world = xr_new<CPHWorld>();
    VERIFY(os);
    ph_world->Create(mt, os, lo);
}

void destroy_physics_world()
{
    ZoneScoped;
    ph_world->Destroy();
    xr_delete(ph_world);
}

void destroy_object_space(CObjectSpace*& os) { xr_delete(os); }

void CPHMesh::Create()
{
    // Меш теперь инициализируется через IPhysicsCore
}

void CPHMesh::Destroy()
{
    if (m_mesh_handle) {
        GetPhysicsCore()->DestroyCDBModel(m_mesh_handle);
        m_mesh_handle = nullptr;
    }
}

#ifdef DEBUG
void CPHWorld::OnRender() { debug_output().PH_DBG_Render(); }
#endif

CPHWorld::CPHWorld()
    : m_object_space(0), m_level_objects(0), m_default_contact_shotmark(0),
      m_default_character_contact_shotmark(0), physics_step_time_callback(0)
{
    disable_count = 0;
    m_frame_time = 0.f;
    m_previous_frame_time = 0.f;
    b_frame_mark = false;
    m_steps_num = 0;
    m_steps_short_num = 0;
    m_frame_sum = 0.f;
    m_delay = 0;
    m_previous_delay = 0;
    m_reduce_delay = 0;
    m_update_delay_count = 0;
    b_world_freezed = false;
    b_processing = false;
    m_gravity = default_world_gravity;
    b_exist = false;
}

void CPHWorld::SetStep(float s)
{
    fixed_step = s;
    if (ph_world && ph_world->Exist())
    {
        float frame_time = Device.fTimeDelta;
        u32 it_number = iFloor(frame_time / fixed_step);
        frame_time -= it_number * fixed_step;
        ph_world->m_previous_frame_time = frame_time;
        ph_world->m_frame_time = frame_time;
    }
}

void CPHWorld::Create(bool mt, CObjectSpace* os, CObjectList* lo)
{
    ZoneScoped;
    GetPhysicsCore()->Initialize();
    LoadParams();
    m_object_space = os;
    m_level_objects = lo;
    Device.AddSeqFrame(this, mt);
    m_commander = xr_new<CPHCommander>();
    
    Mesh.Create();

    disable_count = 0;
    phBoundaries.set(inl_ph_world().ObjectSpace().GetBoundingVolume());
    phBoundaries.y1 -= 30.f;
    CPHCollideValidator::Init();
    b_exist = true;

    SetStep(ph_console::ph_step_time);
}

void CPHWorld::Destroy()
{
    ZoneScoped;
    r_spatial.clear();
    xr_delete(m_commander);
    Mesh.Destroy();
    
    // Очистка физического мира от объектов текущего уровня
    GetPhysicsCore()->Clear();

    Device.RemoveSeqFrame(this);
    b_exist = false;
}

void CPHWorld::SetGravity(float g)
{
    m_gravity = g;
}

void CPHWorld::OnFrame()
{
    ZoneScoped;
    stats.FrameStart();
    stats.MovCollision.Begin();
    FrameStep(Device.fTimeDelta);
    stats.MovCollision.End();
    stats.FrameEnd();
}

void CPHWorld::DumpStatistics(IGameFont& font, IPerformanceAlert* alert)
{
    stats.FrameEnd();
    float engineTotal = Device.GetStats().EngineTotal.result;
    float percentage = 100.0f * stats.MovCollision.result / engineTotal;
    font.OutNext("Physics:      %2.2fms, %2.1f%%", stats.MovCollision.result, percentage);
    font.OutNext("- collider:   %2.2fms", stats.Collision.result);
    font.OutNext("- solver:     %2.2fms, %d", stats.Core.result, stats.Core.count);
    if (alert && stats.MovCollision.result > 5.0f)
        alert->Print(font, "Physics   > 5ms:  %3.1f", stats.MovCollision.result);
}

static u32 start_time = 0;
void CPHWorld::Step()
{
    VERIFY(b_processing || IsFreezed());

    PH_OBJECT_I i_object;
    PH_UPDATE_OBJECT_I i_update_object;

    if (disable_count == 0)
    {
        disable_count = worldDisablingParams.objects_params.L2frames;
        for (i_object = m_recently_disabled_objects.begin(); m_recently_disabled_objects.end() != i_object;)
        {
            CPHObject* obj = (*i_object);
            obj->check_recently_deactivated();
            ++i_object;
        }
    }
    if (!IsFreezed())
        --disable_count;

    ++m_steps_num;
    ++m_steps_short_num;
    stats.Collision.Begin();

    // Jolt берет на себя коллизии, поэтому здесь просто базовая логика
    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        (*i_object)->Collide();
        ++i_object;
    }

    stats.Collision.End();

    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        (*i_object)->PhTune(fixed_step);
        ++i_object;
    }

    for (i_update_object = m_update_objects.begin(); m_update_objects.end() != i_update_object;)
    {
        (*i_update_object)->PhTune(fixed_step);
        ++i_update_object;
    }

    stats.Core.Begin();
    m_commander->update_threadsafety();
    
    // --- ГЛОБАЛЬНЫЙ ШАГ JOLT ---
    GetPhysicsCore()->Step(fixed_step);
    
    stats.Core.End();

    // Синхронизируем результаты обратно в движок
    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        CPHObject* obj = (*i_object);
        ++i_object;
        obj->PhDataUpdate(fixed_step);
        obj->spatial_move();
    }

    for (i_update_object = m_update_objects.begin(); m_update_objects.end() != i_update_object;)
    {
        CPHUpdateObject* obj = *i_update_object;
        ++i_update_object;
        obj->PhDataUpdate(fixed_step);
    }

    if (physics_step_time_callback)
    {
        physics_step_time_callback(start_time, start_time + u32(fixed_step * 1000));
        start_time += u32(fixed_step * 1000);
    };
}

void CPHWorld::StepTouch()
{
    PH_OBJECT_I i_object;
    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        (*i_object)->Collide();
        ++i_object;
    }

    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        CPHObject* obj = (*i_object);
        ++i_object;
        obj->spatial_move();
    }
}

u32 CPHWorld::CalcNumSteps(u32 dTime)
{
    if (dTime < m_frame_time * 1000)
        return 0;
    u32 res = iCeil((float(dTime) - m_frame_time * 1000) / (fixed_step * 1000));
    return res;
};

void CPHWorld::FrameStep(float step)
{
    if (IsFreezed())
        return;

    VERIFY(_valid(step));
    step *= phTimefactor;

    u32 it_number;
    float frame_time = m_frame_time;
    frame_time += step;

    if (!(frame_time < fixed_step))
    {
        it_number = iFloor(frame_time / fixed_step);
        frame_time -= it_number * fixed_step;
        m_previous_frame_time = m_frame_time;
        m_frame_time = frame_time;
        b_frame_mark = !b_frame_mark;
    }
    else
    {
        m_frame_time = frame_time;
        return;
    }

    b_processing = true;

    start_time = Device.dwTimeGlobal;
    if (ph_console::g_bDebugDumpPhysicsStep && it_number > 20)
        Msg("!!! TOO MANY PHYSICS STEPS PER FRAME = %d !!!", it_number);
        
    for (u32 i = 0; i < it_number; ++i)
        Step();
        
    b_processing = false;
}

void CPHWorld::AddObject(CPHObject* object) { m_objects.push_back(object); }
void CPHWorld::AddRecentlyDisabled(CPHObject* object) { m_recently_disabled_objects.push_back(object); }
void CPHWorld::RemoveFromRecentlyDisabled(PH_OBJECT_I i) { m_recently_disabled_objects.erase(i); }
void CPHWorld::AddUpdateObject(CPHUpdateObject* object) { m_update_objects.push_back(object); }
void CPHWorld::RemoveUpdateObject(PH_UPDATE_OBJECT_I i) { m_update_objects.erase(i); }
void CPHWorld::RemoveObject(PH_OBJECT_I i) { m_objects.erase((i)); };
void CPHWorld::AddFreezedObject(CPHObject* obj) { m_freezed_objects.push_back(obj); }
void CPHWorld::RemoveFreezedObject(PH_OBJECT_I i) { m_freezed_objects.erase(i); }

void CPHWorld::Freeze()
{
    R_ASSERT2(!b_world_freezed, "already freezed!!!");
    m_freezed_objects.move_items(m_objects);
    PH_OBJECT_I iter = m_freezed_objects.begin(), e = m_freezed_objects.end();
    for (; e != iter; ++iter)
        (*iter)->FreezeContent();
    m_freezed_update_objects.move_items(m_update_objects);
    b_world_freezed = true;
}

void CPHWorld::UnFreeze()
{
    R_ASSERT2(b_world_freezed, "is not freezed!!!");
    PH_OBJECT_I iter = m_freezed_objects.begin(), e = m_freezed_objects.end();
    for (; e != iter; ++iter)
        (*iter)->UnFreezeContent();
    m_objects.move_items(m_freezed_objects);
    m_update_objects.move_items(m_freezed_update_objects);
    b_world_freezed = false;
}

bool CPHWorld::IsFreezed() { return b_world_freezed; }

void CPHWorld::CutVelocity(float l_limit, float a_limit)
{
    PH_OBJECT_I i_object;
    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        (*i_object)->CutVelocity(l_limit, a_limit);
        ++i_object;
    }
}

void CPHWorld::NetRelcase(CPhysicsShell* s)
{
    CPHReqComparerHasShell c(s);
    m_commander->remove_calls_threadsafety(&c);

    PH_UPDATE_OBJECT_I i_update_object;
    for (i_update_object = m_update_objects.begin(); m_update_objects.end() != i_update_object;)
    {
        CPHUpdateObject* obj = (*i_update_object);
        ++i_update_object;
        obj->NetRelcase(s);
    }
}

void CPHWorld::AddCall(CPHCondition* c,CPHAction* a)
{
    m_commander->add_call_threadsafety(c, a);
}

u16 CPHWorld::ObjectsNumber() { return m_objects.count(); }
u16 CPHWorld::UpdateObjectsNumber() { return m_update_objects.count(); }

void CPHWorld::GetState(V_PH_WORLD_STATE& state)
{
    state.clear();
    PH_OBJECT_I i_object;
    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        CPHObject* obj = (*i_object);
        const u16 els = obj->get_elements_number();
        for (u16 i = 0; els > i; ++i)
        {
            std::pair<CPHSynchronize*, SPHNetState> s;
            s.first = obj->get_element_sync(i);
            s.first->get_State(s.second);
            state.push_back(s);
        }
        ++i_object;
    }
}

void CPHWorld::StepNumIterations(int num_it) { /* Пустышка для совместимости */ }
