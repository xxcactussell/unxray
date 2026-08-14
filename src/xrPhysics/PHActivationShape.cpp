#include "StdAfx.h"
#include "PHActivationShape.h"

#include "Physics.h"
#include "MathUtils.h"
#include "PHValideValues.h"

#include "ExtendedGeom.h"
#include "SpaceUtils.h"

#include "PHWorld.h"
#include "xrPhysicsCore/IPhysicsCore.h"

#ifdef DEBUG
#include "debug_output.h"
#endif // DEBUG

#include "PHDynamicData.h"
#include "xrServerEntities/PHSynchronize.h"
#include "xrServerEntities/PHNetState.h"

namespace detail::activation_shape
{
    static float max_depth = 0.f;
}

#ifdef DEBUG
#define CHECK_POS(pos, msg, br)                   \
    if (!valid_pos(pos, phBoundaries))            \
    {                                             \
        Msg("pos:%f,%f,%f", pos.x, pos.y, pos.z); \
        Msg(msg);                                 \
        VERIFY(!br);                              \
    }
#else
#define CHECK_POS(pos, msg, br)
#endif

void RestoreVelocityState(V_PH_WORLD_STATE& state)
{
    for (auto& it : state)
    {
        CPHSynchronize& sync = *it.first;
        SPHNetState& old_s = it.second;
        SPHNetState new_s;
        sync.get_State(new_s);
        new_s.angular_vel.set(old_s.angular_vel);
        new_s.linear_vel.set(old_s.linear_vel);
        new_s.enabled = old_s.enabled;
        sync.set_State(new_s);
    }
}

CPHActivationShape::CPHActivationShape()
{
    m_geom = nullptr;
    m_body = INVALID_BODY_HANDLE;
    m_flags.zero();
    m_flags.set(flFixedRotation, true);
}

CPHActivationShape::~CPHActivationShape() 
{ 
    VERIFY(m_body == INVALID_BODY_HANDLE); 
}

void CPHActivationShape::Create(const Fvector start_pos, const Fvector start_size, IPhysicsShellHolder* ref_obj, EType _type /*=etBox*/, u16 flags)
{
    VERIFY(ref_obj);
    R_ASSERT(_valid(start_pos));
    R_ASSERT(_valid(start_size));

    // В ODE использовалась колоссальная масса для продавливания объектов
    float mass = 100000.f;

    switch (_type)
    {
    case etBox: 
        // В X-Ray start_size это полный размер, а IPhysicsCore::CreateBox ожидает half_extents
        m_body = GetPhysicsCore()->CreateBox(Fvector().set(start_size).mul(0.5f), start_pos, mass); 
        break;
    case etSphere: 
        // Создание сферы (метод необходимо добавить в IPhysicsCore)
        m_body = GetPhysicsCore()->CreateSphere(start_size.x, start_pos, mass); 
        break;
    case etCylinder:
        // Создание цилиндра (метод необходимо добавить в IPhysicsCore)
        m_body = GetPhysicsCore()->CreateCylinder(start_size.x, start_size.y, start_pos, mass);
        break;
    };

    // Отключаем гравитацию для выталкивающего шейпа
    GetPhysicsCore()->SetBodyGravityFactor(m_body, 0.0f);

    m_safe_state.create(m_body);
    spatial_register();
    m_flags.set(flags, true);
}

void CPHActivationShape::Destroy()
{
    VERIFY(m_body != INVALID_BODY_HANDLE);
    spatial_unregister();
    CPHObject::deactivate();
    
    GetPhysicsCore()->DestroyBody(m_body);
    m_body = INVALID_BODY_HANDLE;
    m_geom = nullptr;
}

bool CPHActivationShape::Activate(const Fvector need_size, u16 steps, float max_displacement, float max_rotation, bool un_freeze_later /* =false*/)
{
    using namespace ::detail::activation_shape;

#ifdef DEBUG
    if (debug_output().ph_dbg_draw_mask().test(phDbgDrawDeathActivationBox))
    {
        debug_output().DBG_OpenCashedDraw();
        Fmatrix M;
        GetPhysicsCore()->GetBodyTransform(m_body, M);
        Fvector v;
        GetPhysicsCore()->GetBoxExtents(m_body, v); // v это уже half_extents
        debug_output().DBG_DrawOBB(M, v, color_xrgb(0, 255, 0));
    }
#endif

    VERIFY(m_body != INVALID_BODY_HANDLE);
    CPHObject::activate();
    ph_world->Freeze();
    UnFreeze();
    max_depth = 0.f;

    ph_world->StepTouch();
    
    u16 num_it = 15;
    float fnum_it = float(num_it);
    float fnum_steps = float(steps);
    float fnum_steps_r = 1.f / fnum_steps;
    float resolve_depth = 0.01f;
    float max_vel = max_depth / fnum_it * fnum_steps_r / fixed_step;
    float limit_l_vel = _max(_max(need_size.x, need_size.y), need_size.z) / fnum_it * fnum_steps_r / fixed_step;

    if (limit_l_vel > default_l_limit) limit_l_vel = default_l_limit;
    if (max_vel > limit_l_vel) max_vel = limit_l_vel;

    float max_a_vel = max_rotation / fnum_it * fnum_steps_r / fixed_step;
    if (max_a_vel > default_w_limit) max_a_vel = default_w_limit;

    max_depth = 0.f;

    Fvector from_size;
    Fvector step_size, size;
    
    // Получаем текущие half_extents и переводим в полный размер
    GetPhysicsCore()->GetBoxExtents(m_body, from_size); 
    from_size.mul(2.0f); 
    
    step_size.sub(need_size, from_size);
    step_size.mul(fnum_steps_r);
    size.set(from_size);
    
    bool ret = false;
    V_PH_WORLD_STATE temp_state;
    ph_world->GetState(temp_state);
    
    for (int m = 0; steps > m; ++m)
    {
        size.add(step_size);
        
        // Устанавливаем новый размер (метод необходимо добавить в IPhysicsCore)
        GetPhysicsCore()->SetBoxExtents(m_body, Fvector().set(size).mul(0.5f)); 
        
        u16 attempts = 10;
        do
        {
            ret = false;
            for (int i = 0; num_it > i; ++i)
            {
                max_depth = 0.f;
                ph_world->Step();
                CHECK_POS(Position(), "pos after ph_world->Step()", false);
                
                CutVelocity(max_vel, max_a_vel);
                CHECK_POS(Position(), "pos after CutVelocity", true);
                
                if (max_depth < resolve_depth)
                {
                    ret = true;
                    break;
                }
            }
            attempts--;
        } while (!ret && attempts > 0);
    }
    
    RestoreVelocityState(temp_state);
    CHECK_POS(Position(), "pos after RestoreVelocityState(temp_state);", true);
    
    if (!un_freeze_later)
        ph_world->UnFreeze();

#ifdef DEBUG
    if (debug_output().ph_dbg_draw_mask().test(phDbgDrawDeathActivationBox))
    {
        debug_output().DBG_OpenCashedDraw();
        Fmatrix M;
        GetPhysicsCore()->GetBodyTransform(m_body, M);
        Fvector v;
        v.set(need_size).mul(0.5f);
        debug_output().DBG_DrawOBB(M, v, color_xrgb(0, 255, 255));
        debug_output().DBG_ClosedCashedDraw(30000);
    }
#endif
    return ret;
}

const Fvector& CPHActivationShape::Position() 
{ 
    static Fvector pos;
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    pos = transform.c;
    return pos; 
}

void CPHActivationShape::Size(Fvector& size) 
{ 
    GetPhysicsCore()->GetBoxExtents(m_body, size);
    size.mul(2.0f); // Возвращаем полный размер, а не half_extents
}

void CPHActivationShape::PhDataUpdate(float step) 
{ 
    m_safe_state.new_state(m_body); 
}

void CPHActivationShape::PhTune(float step) {}

PhysicsShapeHandle CPHActivationShape::dSpacedGeom() 
{ 
    return m_geom; 
}

void CPHActivationShape::get_spatial_params()
{
    Fvector center, half_extents;
    GetPhysicsCore()->GetBodyAABB(m_body, center, half_extents);
    spatial.sphere.P = center;
    spatial.sphere.R = _max(half_extents.x, _max(half_extents.y, half_extents.z));
    AABB = half_extents;
}

void CPHActivationShape::InitContact(bool& do_collide, bool bo1, float depth, CPhysicsGeom* my_geom, CPhysicsGeom* oposite_geom, u16 material_idx_1, u16 material_idx_2)
{
    if (!do_collide) return;
    detail::activation_shape::max_depth += depth;
}

void CPHActivationShape::CutVelocity(float l_limit, float /*a_limit*/)
{
    Fvector lin_vel;
    GetPhysicsCore()->GetBodyLinearVelocity(m_body, lin_vel);
    
    float mag = lin_vel.magnitude();
    if (mag > l_limit)
    {
        lin_vel.mul(l_limit / mag);
        GetPhysicsCore()->SetBodyLinearVelocity(m_body, lin_vel);
        GetPhysicsCore()->SetBodyAngularVelocity(m_body, Fvector().set(0.f, 0.f, 0.f));
    }
}

void CPHActivationShape::set_rotation(const Fmatrix& sof)
{
    Fmatrix current_transform;
    GetPhysicsCore()->GetBodyTransform(m_body, current_transform);
    
    Fmatrix new_transform = sof;
    new_transform.c = current_transform.c;
    
    GetPhysicsCore()->SetBodyTransform(m_body, new_transform);
    m_safe_state.set_rotation(new_transform);
}

#ifdef DEBUG
IPhysicsShellHolder* CPHActivationShape::ref_object()
{
    // В новой архитектуре мы можем вернуть nullptr для формы активации или 
    // передать IPhysicsShellHolder через user_data тела, если это потребуется в дебаге.
    return nullptr;
}
#endif

#undef CHECK_POS
