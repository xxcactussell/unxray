#include "StdAfx.h"
#include "TeleWhirlwind.h"
#include "xrPhysics/PhysicsShell.h"
#include "PhysicsShellHolder.h"
#include "Level.h"
#include "Hit.h"
#include "PHDestroyable.h"
#include "xrMessages.h"
#include "CharacterPhysicsSupport.h"
#include "xrPhysics/ActiveRagdoll.h"
#include "Include/xrRender/Kinematics.h"
#include "Include/xrRender/KinematicsAnimated.h"

CTeleWhirlwind::CTeleWhirlwind()
{
    m_owner_object = NULL;
    m_center.set(0.f, 0.f, 0.f);
    m_keep_radius = 1.f;
    m_throw_power = 100.f;
}

CTelekineticObject* CTeleWhirlwind::activate(
    CPhysicsShellHolder* obj, float strength, float height, u32 max_time_keep, bool rot)
{
    if (inherited::activate(obj, strength, height, max_time_keep, rot))
    {
        CTeleWhirlwindObject* o = smart_cast<CTeleWhirlwindObject*>(objects.back());
        VERIFY(o);
        o->set_throw_power(m_throw_power);
        return o;
    }
    else
        return 0;
}
void CTeleWhirlwind::clear_impacts() { m_saved_impacts.clear(); }
void CTeleWhirlwind::clear() { inherited::clear(); }
void CTeleWhirlwind::add_impact(const Fvector& dir, float val)
{
    Fvector force, point;
    force.set(dir);
    force.mul(val);
    point.set(0.f, 0.f, 0.f);
    m_saved_impacts.push_back(SPHImpact(force, point, 0));
}
void CTeleWhirlwind::reserve_impact(const size_t count) { m_saved_impacts.reserve(count); }
void CTeleWhirlwind::set_throw_power(float throw_pow) { m_throw_power = throw_pow; }
void CTeleWhirlwind::draw_out_impact(Fvector& dir, float& val)
{
    VERIFY2(m_saved_impacts.size(), "NO IMPACTS ADDED!");

    if (m_saved_impacts.empty())
        return;

    dir.set(m_saved_impacts[0].force);
    val = dir.magnitude();

    // Swartz
    //if (!fis_zero(val))
    //    dir.mul(1.f / val);
    m_saved_impacts.erase(m_saved_impacts.begin());
}

void CTeleWhirlwind::clear_notrelevant()
{
    //убрать все объеты со старыми параметрами
    const auto it = std::remove_if(objects.begin(), objects.end(), [](CTelekineticObject* tele_object)
    {
        return (!tele_object->get_object() || tele_object->get_object()->getDestroy());
    });
    objects.erase(it, objects.end());
}

void CTeleWhirlwind::play_destroy(CTeleWhirlwindObject* obj) {}
CTeleWhirlwindObject::CTeleWhirlwindObject()
{
    m_telekinesis = 0;
    throw_power = 0.f;
}

bool CTeleWhirlwindObject::init(CTelekinesis* tele, CPhysicsShellHolder* obj, float s, float h, u32 ttk, bool rot)
{
    if (!obj)
        return false;

    CEntityAlive* ea = smart_cast<CEntityAlive*>(obj);
    if (ea && ea->g_Alive())
        return false;

    bool result = inherited::init(tele, obj, s, h, ttk, rot);
    if (!result || !object)
        return false;

    m_telekinesis = static_cast<CTeleWhirlwind*>(tele);

    throw_power = strength;
    if (m_telekinesis->is_active_object(obj))
    {
        return false;
    }
    if (obj->PPhysicsShell())
    {
        obj->PPhysicsShell()->SetAirResistance(0.8f, 1.2f);
        obj->m_pPhysicsShell->set_ApplyByGravity(TRUE);
    }

    if (object->ph_destroyable() && object->ph_destroyable()->CanDestroy())
        b_destroyable = true;
    else
        b_destroyable = false;

    return true;
}
void CTeleWhirlwindObject::raise_update()
{
    // u32 time=Device.dwTimeGlobal;
    //	if (time_raise_started + 100000 < time) release();
}

void CTeleWhirlwindObject::release()
{
    if (!object || object->getDestroy())
        return;

    bool has_shell = object->m_pPhysicsShell && object->m_pPhysicsShell->isActive();
    bool has_ragdoll = object->character_physics_support() && object->character_physics_support()->active_ragdoll();

    if (!has_shell && !has_ragdoll)
        return;

    Fvector dir_inv;
    dir_inv.sub(object->Position(), m_telekinesis->Center());
    float magnitude = dir_inv.magnitude();

    if (has_shell)
    {
        object->m_pPhysicsShell->set_ApplyByGravity(TRUE);
        object->m_pPhysicsShell->SetAirResistance(0.01f, 0.01f);
    }

    float impulse = 0.f;
    if (magnitude > 0.2f)
    {
        dir_inv.mul(1.f / magnitude);
        impulse = std::clamp(throw_power * 0.005f, 5.0f, 25.0f);
    }
    else
    {
        dir_inv.random_dir();
        impulse = std::clamp(throw_power * 0.005f, 5.0f, 25.0f);
    }

    bool b_destroyed = false;
    if (b_destroyable && magnitude < 2.5f * object->Radius() + 1.5f)
    {
        b_destroyed = destroy_object(dir_inv, throw_power * 0.05f);
    }

    if (!b_destroyed)
    {
        if (has_shell)
            object->m_pPhysicsShell->applyImpulse(dir_inv, impulse * object->m_pPhysicsShell->getMass() / 100.f);
        else if (has_ragdoll)
            object->character_physics_support()->active_ragdoll()->ApplyLinearImpulse(dir_inv, impulse);
    }
    switch_state(TS_None);
}

bool CTeleWhirlwindObject::destroy_object(const Fvector dir, float val)
{
    CPHDestroyable* D = object->ph_destroyable();
    if (D)
    {
        D->PhysicallyRemoveSelf();
        D->Destroy(m_telekinesis->OwnerObject()->ID());

        if (IsGameTypeSingle())
        {
            m_telekinesis->reserve_impact(D->m_destroyed_obj_visual_names.size());
            for ([[maybe_unused]] const auto& i : D->m_destroyed_obj_visual_names)
                m_telekinesis->add_impact(dir, val * 10.f);
        };

        CParticlesPlayer* PP = smart_cast<CParticlesPlayer*>(object);
        if (PP)
        {
            IKinematics* K = smart_cast<IKinematics*>(object->Visual());
            if (K)
            {
                u16 root = K->LL_GetBoneRoot();
                PP->StartParticles(
                    m_telekinesis->destroing_particles(), root, Fvector().set(0, 1, 0), m_telekinesis->OwnerObject()->ID());
            }
        }
        return true;
    }
    return false;
}

void CTeleWhirlwindObject::raise(float step)
{
    CPhysicsShellHolder* holder = get_object();
    if (!holder)
        return;

    Fvector center = m_telekinesis->Center();
    CPhysicsShell* p = holder->PPhysicsShell();
    if (p && p->isActive())
    {
        p->SetAirResistance(0.8f, 1.2f);
        p->set_ApplyByGravity(TRUE);

        u16 element_number = p->get_ElementsNumber();
        CPhysicsElement* maxE = p->get_ElementByStoreOrder(0);

        for (u16 element = 0; element < element_number; ++element)
        {
            CPhysicsElement* E = p->get_ElementByStoreOrder(element);
            if (maxE->getMass() < E->getMass())
                maxE = E;
            if (!E->isActive())
                continue;
            Fvector pos = E->mass_Center();

            Fvector to_center_h;
            to_center_h.set(center.x - pos.x, 0.0f, center.z - pos.z);
            float dist_h = to_center_h.magnitude();
            if (dist_h > 0.01f)
                to_center_h.mul(1.f / dist_h);
            else
                to_center_h.random_dir();
            to_center_h.y = 0.0f;

            Fvector tangent;
            tangent.set(to_center_h.z, 0.0f, -to_center_h.x);

            float elem_mass = E->getMass();
            float dy = center.y - pos.y;

            // Smooth lift against gravity + centripetal pull + gentle tangential orbit
            float lift_accel = holder->EffectiveGravity() + std::clamp(dy * 4.0f, -2.0f, 6.0f);
            float pull_accel = std::clamp(dist_h * 0.8f, 0.2f, 1.5f);
            float swirl_accel = 0.2f;

            Fvector force;
            force.x = (to_center_h.x * pull_accel + tangent.x * swirl_accel) * elem_mass;
            force.y = lift_accel * elem_mass;
            force.z = (to_center_h.z * pull_accel + tangent.z * swirl_accel) * elem_mass;

            // Strong physical velocity drag to keep props calm and stable
            Fvector cur_v;
            E->get_LinearVel(cur_v);
            force.x -= cur_v.x * elem_mass * 3.0f;
            force.y -= cur_v.y * elem_mass * 1.5f;
            force.z -= cur_v.z * elem_mass * 3.0f;

            E->applyForce(force.x, force.y, force.z);
        }

        // Angular drag torque to stop fast spinning
        Fvector cur_w;
        p->get_AngularVel(cur_w);
        p->setTorque(Fvector().mul(cur_w, -p->getMass() * 0.5f));

        Fvector max_pos = maxE->mass_Center();
        float dist_h = Fvector().set(center.x - max_pos.x, 0.0f, center.z - max_pos.z).magnitude();
        float dy = center.y - max_pos.y;
        if (dist_h < m_telekinesis->keep_radius() && _abs(dy) < 0.5f)
        {
            p->setTorque(Fvector().set(0, 0, 0));
            p->setForce(Fvector().set(0, 0, 0));
            switch_state(TS_Keep);
        }
    }
    else if (holder->character_physics_support() && holder->character_physics_support()->active_ragdoll())
    {
        auto ragdoll = holder->character_physics_support()->active_ragdoll();
        Fvector pos = ragdoll->GetSimulatedPosition(0);

        Fvector cur_v, cur_w;
        ragdoll->GetVelocity(cur_v, cur_w);

        // Horizontal vector towards center
        Fvector to_center_h;
        to_center_h.set(center.x - pos.x, 0.0f, center.z - pos.z);
        float dist_h = to_center_h.magnitude();
        if (dist_h > 0.01f)
            to_center_h.mul(1.f / dist_h);
        else
            to_center_h.random_dir();
        to_center_h.y = 0.0f;

        // Tangential swirl vector (orthogonal to to_center_h)
        Fvector tangent;
        tangent.set(to_center_h.z, 0.0f, -to_center_h.x);

        float dy = center.y - pos.y;
        float mass = 75.0f;

        // Target smooth upward velocity (up to 2.0 m/s when low, 0 m/s at center.y)
        float target_v_y = std::clamp(dy * 2.0f, -1.0f, 2.0f);
        float delta_v_y = target_v_y - cur_v.y;
        float lift_accel = 9.81f + std::clamp(delta_v_y * 6.0f, -6.0f, 15.0f);

        // Centripetal inward pull with velocity damping
        float target_v_in = std::clamp(dist_h * 1.5f, 0.2f, 2.0f);
        float cur_v_in = to_center_h.dotproduct(cur_v);
        float pull_accel = std::clamp((target_v_in - cur_v_in) * 4.0f, -4.0f, 10.0f);

        // Calm, majestic tangential orbital swirl (target tangential speed ~1.0 m/s)
        float target_v_swirl = 1.0f;
        float cur_v_swirl = tangent.dotproduct(cur_v);
        float swirl_accel = std::clamp((target_v_swirl - cur_v_swirl) * 3.0f, -4.0f, 4.0f);

        // Combine into per-step impulse (F * dt)
        Fvector total_imp;
        total_imp.x = (to_center_h.x * pull_accel + tangent.x * swirl_accel) * mass * step;
        total_imp.y = lift_accel * mass * step;
        total_imp.z = (to_center_h.z * pull_accel + tangent.z * swirl_accel) * mass * step;

        float imp_mag = total_imp.magnitude();
        Fvector imp_dir = total_imp;
        imp_dir.normalize_safe();

        if (OnServer())
            ragdoll->ApplyLinearImpulse(imp_dir, imp_mag);

        if (dist_h < m_telekinesis->keep_radius() && _abs(dy) < 0.5f)
        {
            switch_state(TS_Keep);
        }
    }
}

void CTeleWhirlwindObject::keep()
{
    CPhysicsShellHolder* holder = get_object();
    if (!holder)
        return;

    Fvector center = m_telekinesis->Center();
    CPhysicsShell* p = holder->PPhysicsShell();
    if (p && p->isActive())
    {
        p->SetAirResistance(1.5f, 2.5f);
        p->set_ApplyByGravity(FALSE);

        u16 element_number = p->get_ElementsNumber();
        CPhysicsElement* maxE = p->get_ElementByStoreOrder(0);
        for (u16 element = 0; element < element_number; ++element)
        {
            CPhysicsElement* E = p->get_ElementByStoreOrder(element);
            if (maxE->getMass() < E->getMass())
                maxE = E;
            if (!E->isActive())
                continue;
            Fvector pos = E->mass_Center();

            Fvector to_center_h;
            to_center_h.set(center.x - pos.x, 0.0f, center.z - pos.z);
            float dist_h = to_center_h.magnitude();
            if (dist_h > 0.01f)
                to_center_h.mul(1.f / dist_h);
            else
                to_center_h.random_dir();
            to_center_h.y = 0.0f;

            Fvector tangent;
            tangent.set(to_center_h.z, 0.0f, -to_center_h.x);

            float elem_mass = E->getMass();
            float dy = center.y - pos.y;

            // In keep phase: slow calm orbit + height hold + strong drag
            Fvector force;
            force.x = (to_center_h.x * 0.6f + tangent.x * 0.08f) * elem_mass;
            force.y = std::clamp(dy * 4.0f, -2.0f, 2.0f) * elem_mass;
            force.z = (to_center_h.z * 0.6f + tangent.z * 0.08f) * elem_mass;

            Fvector cur_v;
            E->get_LinearVel(cur_v);
            force.x -= cur_v.x * elem_mass * 3.5f;
            force.y -= cur_v.y * elem_mass * 1.5f;
            force.z -= cur_v.z * elem_mass * 3.5f;

            E->applyForce(force.x, force.y, force.z);
        }

        // Angular drag torque
        Fvector cur_w;
        p->get_AngularVel(cur_w);
        p->setTorque(Fvector().mul(cur_w, -p->getMass() * 0.8f));

        Fvector max_pos = maxE->mass_Center();
        float dist_h = Fvector().set(center.x - max_pos.x, 0.0f, center.z - max_pos.z).magnitude();
        float dy = center.y - max_pos.y;
        if (dist_h > m_telekinesis->keep_radius() * 1.5f || _abs(dy) > 1.0f)
        {
            p->set_ApplyByGravity(TRUE);
            switch_state(TS_Raise);
        }
    }
    else if (holder->character_physics_support() && holder->character_physics_support()->active_ragdoll())
    {
        auto ragdoll = holder->character_physics_support()->active_ragdoll();
        Fvector pos = ragdoll->GetSimulatedPosition(0);

        Fvector cur_v, cur_w;
        ragdoll->GetVelocity(cur_v, cur_w);

        Fvector to_center_h;
        to_center_h.set(center.x - pos.x, 0.0f, center.z - pos.z);
        float dist_h = to_center_h.magnitude();
        if (dist_h > 0.01f)
            to_center_h.mul(1.f / dist_h);
        else
            to_center_h.random_dir();
        to_center_h.y = 0.0f;

        Fvector tangent;
        tangent.set(to_center_h.z, 0.0f, -to_center_h.x);

        float dy = center.y - pos.y;
        float mass = 75.0f;

        // In keep state: maintain exact height hover around center.y with strong damping
        float target_v_y = std::clamp(dy * 3.0f, -1.0f, 1.0f);
        float delta_v_y = target_v_y - cur_v.y;
        float lift_accel = 9.81f + std::clamp(delta_v_y * 8.0f, -8.0f, 8.0f);

        // Keep inward pull
        float target_v_in = std::clamp(dist_h * 1.0f, 0.1f, 1.2f);
        float cur_v_in = to_center_h.dotproduct(cur_v);
        float pull_accel = std::clamp((target_v_in - cur_v_in) * 4.0f, -4.0f, 6.0f);

        // Gentle, slow orbit (target 0.8 m/s)
        float target_v_swirl = 0.8f;
        float cur_v_swirl = tangent.dotproduct(cur_v);
        float swirl_accel = std::clamp((target_v_swirl - cur_v_swirl) * 3.0f, -3.0f, 3.0f);

        Fvector total_imp;
        total_imp.x = (to_center_h.x * pull_accel + tangent.x * swirl_accel) * mass * 0.0166f;
        total_imp.y = lift_accel * mass * 0.0166f;
        total_imp.z = (to_center_h.z * pull_accel + tangent.z * swirl_accel) * mass * 0.0166f;

        float imp_mag = total_imp.magnitude();
        Fvector imp_dir = total_imp;
        imp_dir.normalize_safe();

        if (OnServer())
            ragdoll->ApplyLinearImpulse(imp_dir, imp_mag);

        if (dist_h > m_telekinesis->keep_radius() * 1.5f || _abs(dy) > 1.0f)
        {
            switch_state(TS_Raise);
        }
    }
}
void CTeleWhirlwindObject::fire(const Fvector& target)
{
    // inherited::fire(target);
}
void CTeleWhirlwindObject::fire(const Fvector& target, float power)
{
    // inherited:: fire(target,power);
}

void CTeleWhirlwindObject::set_throw_power(float throw_pow) { throw_power = throw_pow; }
void CTeleWhirlwindObject::switch_state(ETelekineticState new_state)
{
    if (new_state == TS_None && object)
    {
        if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
        {
            object->m_pPhysicsShell->set_ApplyByGravity(TRUE);
            object->m_pPhysicsShell->SetAirResistance(0.01f, 0.01f);
        }
    }
    inherited::switch_state(new_state);
}
bool CTeleWhirlwindObject::can_activate(CPhysicsShellHolder* obj)
{
    if (!obj) return false;
    if (smart_cast<CPHDestroyableNotificate*>(obj)) return false;
    CEntityAlive* ea = smart_cast<CEntityAlive*>(obj);
    if (ea && ea->g_Alive()) return false;
    return (obj->m_pPhysicsShell || (obj->character_physics_support() && obj->character_physics_support()->active_ragdoll()));
}
