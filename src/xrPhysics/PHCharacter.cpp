#include "StdAfx.h"

#include "PHCharacter.h"
#include "PHDynamicData.h"
#include "Physics.h"
#include "ExtendedGeom.h"
#include "IPhysicsShellHolder.h"

#include "xrCDB/Intersect.hpp"

#include "PHAICharacter.h"
#include "PHActorCharacter.h"
#include "xrPhysicsCore/IPhysicsCore.h"

CPHCharacter::CPHCharacter(void) : CPHDisablingTranslational()
{
    m_params.acceleration = 0.001f;
    m_params.velocity = 0.0001f;
    m_body = INVALID_BODY_HANDLE;
    m_safe_velocity.set(0.f, 0.f, 0.f);
    m_safe_position.set(0.f, 0.f, 0.f);
    m_mean_y = 0.f;
    m_new_restriction_type = m_restriction_type = rtNone;
    b_actor_movable = true;
    p_lastMaterialIDX = &lastMaterialIDX;
    lastMaterialIDX = GAMEMTL_NONE_IDX;
    injuriousMaterialIDX = GAMEMTL_NONE_IDX;
    m_creation_step = u64(-1);
    b_in_touch_resrtrictor = false;
    m_current_object_radius = -1.f;
}

CPHCharacter::~CPHCharacter(void) {}

void CPHCharacter::FreezeContent()
{
    if (m_body != INVALID_BODY_HANDLE)
        GetPhysicsCore()->DeactivateBody(m_body);
    CPHObject::FreezeContent();
}

void CPHCharacter::UnFreezeContent()
{
    if (m_body != INVALID_BODY_HANDLE)
        GetPhysicsCore()->ActivateBody(m_body);
    CPHObject::UnFreezeContent();
}

void CPHCharacter::getForce(Fvector& force)
{
    if (!b_exist || m_body == INVALID_BODY_HANDLE)
    {
        force.set(0, 0, 0);
        return;
    }
    GetPhysicsCore()->GetBodyForce(m_body, force);
}

void CPHCharacter::setForce(const Fvector& force)
{
    if (!b_exist || m_body == INVALID_BODY_HANDLE)
        return;
    GetPhysicsCore()->SetBodyForce(m_body, force);
}

void CPHCharacter::get_State(SPHNetState& state)
{
    GetPosition(state.position);
    m_body_interpolation.GetPosition(state.previous_position, 0);
    GetVelocity(state.linear_vel);
    getForce(state.force);

    state.angular_vel.set(0.f, 0.f, 0.f);
    state.quaternion.identity();
    state.previous_quaternion.identity();
    state.torque.set(0.f, 0.f, 0.f);

    if (!b_exist)
    {
        state.enabled = false;
        return;
    }
    state.enabled = CPHObject::is_active(); 
}

void CPHCharacter::set_State(const SPHNetState& state)
{
    m_body_interpolation.SetPosition(state.previous_position, 0);
    m_body_interpolation.SetPosition(state.position, 1);
    SetPosition(state.position);
    SetVelocity(state.linear_vel);
    setForce(state.force);

    if (!b_exist)
        return;
    if (state.enabled)
        Enable();
    else
        Disable();
}

void CPHCharacter::Disable()
{
    CPHObject::deactivate();
    if (m_body != INVALID_BODY_HANDLE)
        GetPhysicsCore()->DeactivateBody(m_body);
    m_body_interpolation.ResetPositions();
}

void CPHCharacter::Enable()
{
    if (!b_exist)
        return;
    CPHObject::activate();
    if (m_body != INVALID_BODY_HANDLE)
        GetPhysicsCore()->ActivateBody(m_body);
}

void CPHCharacter::GetSavedVelocity(Fvector& vvel)
{
    if (IsEnabled())
        vvel.set(m_safe_velocity);
    else
        GetVelocity(vvel);
}

void CPHCharacter::CutVelocity(float l_limit, float a_limit)
{
    if (m_body == INVALID_BODY_HANDLE) return;

    Fvector linear_velocity;
    GetPhysicsCore()->GetBodyLinearVelocity(m_body, linear_velocity);
    
    float mag = linear_velocity.magnitude();
    if (mag > l_limit && !fis_zero(mag))
    {
        linear_velocity.mul(l_limit / mag);
        GetPhysicsCore()->SetBodyLinearVelocity(m_body, linear_velocity);
    }
    
    GetPhysicsCore()->SetBodyAngularVelocity(m_body, Fvector().set(0.f, 0.f, 0.f));
}

const Fmatrix& CPHCharacter::XFORM() const
{
    return m_phys_ref_object->ObjectXFORM(); 
}

void CPHCharacter::get_LinearVel(Fvector& velocity) const { GetVelocity(velocity); }
void CPHCharacter::get_AngularVel(Fvector& velocity) const { velocity.set(0, 0, 0); }

const Fvector& CPHCharacter::mass_Center() const 
{ 
    // В оригинале X-Ray здесь возвращался cast_fv(dBodyGetLinearVel(m_body)),
    // что логически является скоростью, а не позицией. Сохраняем это поведение.
    if (m_body != INVALID_BODY_HANDLE)
        GetPhysicsCore()->GetBodyLinearVelocity(m_body, m_last_velocity_cache);
    else
        m_last_velocity_cache.set(0,0,0);
        
    return m_last_velocity_cache; 
}

void CPHCharacter::get_body_position(Fvector& p)
{
    VERIFY(b_exist);
    VERIFY(m_body != INVALID_BODY_HANDLE);
    GetPhysicsCore()->GetBodyPosition(m_body, p);
}

void virtual_move_collide_callback(
    bool& do_colide, bool bo1, 
    CPhysicsGeom* my_geom, CPhysicsGeom* oposite_geom, 
    const Fvector& contact_normal, const Fvector& contact_pos, 
    SGameMtl* material_1, SGameMtl* material_2)
{
    if (!do_colide)
        return;
    do_colide = false;
}

void CPHCharacter::fix_body_rotation()
{
    if (m_body != INVALID_BODY_HANDLE)
    {
        GetPhysicsCore()->SetBodyAngularVelocity(m_body, Fvector().set(0.f, 0.f, 0.f));
        // Для Jolt можно также принудительно обнулить кватернион вращения, если требуется,
        // но обычно для Character используется LockRotations.
    }
}

CPHCharacter* create_ai_character() { return xr_new<CPHAICharacter>(); }
CPHCharacter* create_actor_character(bool single_game) { return xr_new<CPHActorCharacter>(single_game); }
