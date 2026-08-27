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
#include "IPhysicsShellHolder.h"
#include "Include/xrRender/Kinematics.h"
#include "xrCore/Animation/Bone.hpp"
#include "PhysicsShell.h"
#include "PHShell.h"
#include "PHElement.h"

CPHWorld* ph_world = 0;

static void OnJoltBodyActivation(BodyHandle body, void* user_data, bool activated)
{
    if (!user_data) return;
    CPHElement* element = static_cast<CPHElement*>(user_data);
    CPHShell* shell = element->ph_shell();
    if (!shell) return;

    IPhysicsShellHolder* holder = element->PhysicsRefObject();
    if (!holder || holder->ObjectGetDestroy() || holder->has_parent_object())
        return;

    if (holder->IsActor() || holder->IsStalker())
        return;

    if (activated)
    {
        if (!shell->isActive())
        {
            shell->EnableObject(nullptr);
        }
    }
    else
    {
        if (shell->isActive())
        {
            shell->InterpolateGlobalTransform(&holder->ObjectXFORM());
            shell->DisableObject();
            holder->ObjectSpatialMove();
        }
    }
}

static u16 GetDefaultCreatureMaterial()
{
    static u16 s_creature_mtl = GAMEMTL_NONE_IDX;
    if (s_creature_mtl == GAMEMTL_NONE_IDX)
    {
        s_creature_mtl = GMLib.GetMaterialIdx("objects\\dead_body");
        if (s_creature_mtl == GAMEMTL_NONE_IDX)
            s_creature_mtl = GMLib.GetMaterialIdx("objects\\monster_body");
        if (s_creature_mtl == GAMEMTL_NONE_IDX)
            s_creature_mtl = GMLib.GetMaterialIdx("objects\\clothes");
        if (s_creature_mtl == GAMEMTL_NONE_IDX)
            s_creature_mtl = GMLib.GetMaterialIdx("creatures\\human");
        if (s_creature_mtl == GAMEMTL_NONE_IDX)
            s_creature_mtl = GMLib.GetMaterialIdx("materials\\cloth");
        if (s_creature_mtl == GAMEMTL_NONE_IDX)
            s_creature_mtl = GMLib.GetMaterialIdx("default");
    }
    return s_creature_mtl;
}

static void OnJoltRBContact(
    void* user_data_1, void* user_data_2, 
    u16 layer_1, u16 layer_2, 
    const Fvector& pos, const Fvector& norm, 
    float rel_vel, u16 mtl_1, u16 mtl_2)
{
    if (!ph_world || !ph_world->default_contact_shotmark()) return;

    IPhysicsShellHolder* holder_1 = nullptr;
    IPhysicsShellHolder* holder_2 = nullptr;
    CPhysicsGeom* geom_1 = nullptr;
    CPhysicsGeom* geom_2 = nullptr;

    if (user_data_1)
    {
        if (layer_1 == 1) // 1 = Layers::MOVING (Props / Boxes / Barrels)
        {
            CPHElement* elem1 = reinterpret_cast<CPHElement*>(user_data_1);
            holder_1 = elem1->PhysicsRefObject();
            if (elem1->numberOfGeoms() > 0) geom_1 = elem1->geometry(0);
            if (mtl_1 == GAMEMTL_NONE_IDX) {
                if (geom_1 && geom_1->material != GAMEMTL_NONE_IDX) mtl_1 = geom_1->material;
                else if (elem1->Material() != GAMEMTL_NONE_IDX) mtl_1 = elem1->Material();
            }
        }
        else if (layer_1 == 2) // 2 = Layers::RAGDOLL (Stalkers / Monsters)
        {
            holder_1 = reinterpret_cast<IPhysicsShellHolder*>(user_data_1);
            mtl_1 = GetDefaultCreatureMaterial();
        }
    }

    if (user_data_2)
    {
        if (layer_2 == 1) // 1 = Layers::MOVING (Props / Boxes / Barrels)
        {
            CPHElement* elem2 = reinterpret_cast<CPHElement*>(user_data_2);
            holder_2 = elem2->PhysicsRefObject();
            if (elem2->numberOfGeoms() > 0) geom_2 = elem2->geometry(0);
            if (mtl_2 == GAMEMTL_NONE_IDX) {
                if (geom_2 && geom_2->material != GAMEMTL_NONE_IDX) mtl_2 = geom_2->material;
                else if (elem2->Material() != GAMEMTL_NONE_IDX) mtl_2 = elem2->Material();
            }
        }
        else if (layer_2 == 2) // 2 = Layers::RAGDOLL (Stalkers / Monsters)
        {
            holder_2 = reinterpret_cast<IPhysicsShellHolder*>(user_data_2);
            mtl_2 = GetDefaultCreatureMaterial();
        }
    }

    if (holder_1 && holder_1->ObjectGetDestroy()) return;
    if (holder_2 && holder_2->ObjectGetDestroy()) return;

    void* primary_holder = holder_1 ? (void*)holder_1 : (void*)holder_2;
    if (primary_holder)
    {
        static xr_map<void*, u32> s_last_impact_time;
        u32 cur_time = Device.dwTimeGlobal;
        auto it = s_last_impact_time.find(primary_holder);
        if (it != s_last_impact_time.end())
        {
            if (cur_time - it->second < 80) // 80 ms cooldown per object
                return;
        }
        s_last_impact_time[primary_holder] = cur_time;
        if (s_last_impact_time.size() > 512)
            s_last_impact_time.clear();
    }

    Fsphere sph1, sph2;
    sph1.set(pos, 0.2f);
    sph2.set(pos, 0.2f);
    CSphereGeom dummy_g1(sph1);
    CSphereGeom dummy_g2(sph2);

    if (holder_1 && !geom_1) {
        dummy_g1.ph_ref_object = holder_1;
        geom_1 = &dummy_g1;
    }
    if (holder_2 && !geom_2) {
        dummy_g2.ph_ref_object = holder_2;
        geom_2 = &dummy_g2;
    }

    if (!geom_1 && !geom_2) return;

    if (mtl_1 == GAMEMTL_NONE_IDX || mtl_1 >= GMLib.CountMaterial())
        mtl_1 = GMLib.GetMaterialIdx("default_object");
    if (mtl_2 == GAMEMTL_NONE_IDX || mtl_2 >= GMLib.CountMaterial())
        mtl_2 = GMLib.GetMaterialIdx("default");

    SGameMtl* g_mtl1 = GMLib.GetMaterialByIdx(mtl_1);
    SGameMtl* g_mtl2 = GMLib.GetMaterialByIdx(mtl_2);

    if (!g_mtl1 || !g_mtl2) return;

    bool bo1 = (layer_1 == 1 || layer_1 == 2 || (holder_1 != nullptr && holder_2 == nullptr));

    bool do_collide = true;
    ph_world->default_contact_shotmark()(do_collide, bo1, geom_1, geom_2, norm, pos, g_mtl1, g_mtl2);
}

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
    GetPhysicsCore()->SetBodyActivationCallback(OnJoltBodyActivation);
    GetPhysicsCore()->SetRigidBodyContactCallback(OnJoltRBContact);
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
    GetPhysicsCore()->SetBodyActivationCallback(nullptr);
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

    for (i_object = m_objects.begin(); m_objects.end() != i_object;)
    {
        CPHObject* obj = (*i_object);
        ++i_object;
        obj->Collide();
        obj->PhTune(fixed_step);
    }

    stats.Collision.End();

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
}

void CPHWorld::UnFreeze()
{
}

bool CPHWorld::IsFreezed() { return false; }

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
