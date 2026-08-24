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
    m_char_handle = INVALID_CHARACTER_VIRTUAL_HANDLE;
    m_flags.zero();
    m_flags.set(flFixedRotation, true);
}

CPHActivationShape::~CPHActivationShape() 
{ 
    VERIFY(m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE); 
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
        m_char_handle = GetPhysicsCore()->CreateBox(Fvector().set(start_size).mul(0.5f), start_pos, mass); 
        break;
    case etSphere: 
        // Создание сферы (метод необходимо добавить в IPhysicsCore)
        m_char_handle = GetPhysicsCore()->CreateSphere(start_size.x, start_pos, mass); 
        break;
    case etCylinder:
        // Создание цилиндра (метод необходимо добавить в IPhysicsCore)
        m_char_handle = GetPhysicsCore()->CreateCylinder(start_size.x, start_size.y, start_pos, mass);
        break;
    };

    // Отключаем гравитацию для выталкивающего шейпа
    GetPhysicsCore()->SetBodyGravityFactor(m_char_handle, 0.0f);

    m_safe_state.create(m_char_handle);
    spatial_register();
    m_flags.set(flags, true);
}

void CPHActivationShape::Destroy()
{
    VERIFY(m_char_handle != INVALID_CHARACTER_VIRTUAL_HANDLE);
    spatial_unregister();
    CPHObject::deactivate();
    
    GetPhysicsCore()->DestroyBody(m_char_handle);
    m_char_handle = INVALID_CHARACTER_VIRTUAL_HANDLE;
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
        GetPhysicsCore()->GetBodyTransform(m_char_handle, M);
        Fvector v;
        GetPhysicsCore()->GetBoxExtents(m_char_handle, v); // v это уже half_extents
        debug_output().DBG_DrawOBB(M, v, color_xrgb(0, 255, 0));
    }
#endif

    VERIFY(m_char_handle != INVALID_CHARACTER_VIRTUAL_HANDLE);
    CPHObject::activate();

    GetPhysicsCore()->SetBoxExtents(m_char_handle, Fvector().set(need_size).mul(0.5f));

#ifdef DEBUG
    if (debug_output().ph_dbg_draw_mask().test(phDbgDrawDeathActivationBox))
    {
        debug_output().DBG_OpenCashedDraw();
        Fmatrix M;
        GetPhysicsCore()->GetBodyTransform(m_char_handle, M);
        Fvector v;
        v.set(need_size).mul(0.5f);
        debug_output().DBG_DrawOBB(M, v, color_xrgb(0, 255, 255));
        debug_output().DBG_ClosedCashedDraw(30000);
    }
#endif
    return true;
}

const Fvector& CPHActivationShape::Position() 
{ 
    static Fvector pos;
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_char_handle, transform);
    pos = transform.c;
    return pos; 
}

void CPHActivationShape::Size(Fvector& size) 
{ 
    GetPhysicsCore()->GetBoxExtents(m_char_handle, size);
    size.mul(2.0f); // Возвращаем полный размер, а не half_extents
}

void CPHActivationShape::PhDataUpdate(float step) 
{ 
    m_safe_state.new_state(m_char_handle); 
}

void CPHActivationShape::PhTune(float step) {}

PhysicsShapeHandle CPHActivationShape::dSpacedGeom() 
{ 
    return m_geom; 
}

void CPHActivationShape::get_spatial_params()
{
    Fvector center, half_extents;
    GetPhysicsCore()->GetBodyAABB(m_char_handle, center, half_extents);
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
    if (m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE) return;

    Fvector lin_vel;
    GetPhysicsCore()->GetBodyLinearVelocity(m_char_handle, lin_vel);
    
    float mag = lin_vel.magnitude();
    if (mag > l_limit && mag > EPS)
    {
        lin_vel.mul(l_limit / mag);
        GetPhysicsCore()->SetBodyLinearVelocity(m_char_handle, lin_vel);
        GetPhysicsCore()->SetBodyAngularVelocity(m_char_handle, Fvector().set(0.f, 0.f, 0.f));
    }
}

void CPHActivationShape::set_rotation(const Fmatrix& sof)
{
    Fmatrix current_transform;
    GetPhysicsCore()->GetBodyTransform(m_char_handle, current_transform);
    
    Fmatrix new_transform = sof;
    new_transform.c = current_transform.c;
    
    GetPhysicsCore()->SetBodyTransform(m_char_handle, new_transform);
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
