#include "StdAfx.h"
#include "PhysicsShellHolder.h"
#include "telekinetic_object.h"
#include "xrPhysics/PhysicsShell.h"
#include "xrPhysics/MathUtils.h"
#include "CharacterPhysicsSupport.h"
#include "xrPhysics/ActiveRagdoll.h"
#include "PHDestroyableNotificate.h"
#include "Level.h"
#include "GameObject.h"

#define KEEP_IMPULSE_UPDATE 200
#define FIRE_TIME 3000
#define RAISE_MAX_TIME 5000

CTelekineticObject::CTelekineticObject()
{
    state = TS_None;
    object = 0;
    telekinesis = 0;
    m_rotate = false;
}

CTelekineticObject::~CTelekineticObject() {}
bool CTelekineticObject::init(CTelekinesis* tele, CPhysicsShellHolder* obj, float s, float h, u32 ttk, bool rot)
{
    if (!can_activate(obj))
        return false;

    // state				= TS_Raise;
    switch_state(TS_Raise);
    object = obj;

    float base_y = obj->Position().y;
    if (obj->character_physics_support() && obj->character_physics_support()->active_ragdoll())
        base_y = obj->character_physics_support()->active_ragdoll()->GetSimulatedPosition(0).y;

    target_height = base_y + h;

    time_keep_started = 0;
    time_keep_updated = 0;
    time_to_keep = ttk;

    strength = s;

    time_fire_started = 0;
    // time_raise_started	= Device.dwTimeGlobal;

    m_rotate = rot;

    if (object->m_pPhysicsShell)
        object->m_pPhysicsShell->set_ApplyByGravity(FALSE);

    return true;
}

void CTelekineticObject::set_sound(const ref_sound& snd_hold, const ref_sound& snd_throw)
{
    sound_hold.clone(snd_hold, st_Effect, sg_SourceType);
    sound_throw.clone(snd_throw, st_Effect, sg_SourceType);
}

void CTelekineticObject::raise_update()
{
    if (check_height() || check_raise_time_out())
        prepare_keep(); // начать удержание предмета
    // else if (check_raise_time_out()) release();
    else
    {
        if (m_rotate)
            rotate();
    }
}
void CTelekineticObject::keep_update()
{
    if (time_keep_elapsed())
        release();
}
void CTelekineticObject::fire_update()
{
    if (time_fire_elapsed())
        release();
}
void CTelekineticObject::update_state()
{
    switch (get_state())
    {
    case TS_Raise: raise_update(); break;
    case TS_Keep: keep_update(); break;
    case TS_Fire: fire_update(); break;
    case TS_None: break;
    }
}

void CTelekineticObject::switch_state(ETelekineticState new_state)
{
    u32 time = Device.dwTimeGlobal;

    switch (new_state)
    {
    case TS_Raise: time_raise_started = time; break;
    case TS_Keep: time_keep_started = time; break;
    case TS_Fire: time_fire_started = time; break;
    case TS_None: break;
    }
    state = new_state;
}
void CTelekineticObject::raise(float step)
{
    if (!object)
        return;

    step *= strength;

    Fvector dir;
    dir.set(0.f, 1.0f, 0.f);

    if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
    {
        float elem_size = float(object->m_pPhysicsShell->Elements().size());
        dir.mul(elem_size * elem_size * strength);

        if (OnServer())
            (object->m_pPhysicsShell->get_ElementByStoreOrder(0))->applyGravityAccel(dir);
    }
    else if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
    {
        auto ragdoll = object->character_physics_support()->active_ragdoll();
        Fvector pos = ragdoll->GetSimulatedPosition(0);
        Fvector cur_v, cur_w;
        ragdoll->GetVelocity(cur_v, cur_w);

        float dy = target_height - pos.y;
        float mass = 75.0f;
        float target_v_y = std::clamp(dy * 2.0f, -1.0f, 2.0f);
        float delta_v_y = target_v_y - cur_v.y;
        float lift_accel = 9.81f + std::clamp(delta_v_y * 6.0f, -6.0f, 15.0f);

        Fvector total_imp = Fvector().set(0.0f, lift_accel * mass * 0.0166f, 0.0f);
        if (OnServer())
            ragdoll->ApplyLinearImpulse(Fvector().set(0, 1, 0), total_imp.y);
    }

    update_hold_sound();
}

void CTelekineticObject::prepare_keep()
{
    // time_keep_started	= Device.dwTimeGlobal;
    // state				= TS_Keep;
    switch_state(TS_Keep);
    time_keep_updated = 0;
}

bool CTelekineticObject::time_keep_elapsed()
{
    if (time_keep_started + time_to_keep < Device.dwTimeGlobal)
        return true;
    return false;
}

bool CTelekineticObject::time_fire_elapsed()
{
    if (time_fire_started + FIRE_TIME < Device.dwTimeGlobal)
        return true;
    return false;
}

void CTelekineticObject::keep()
{
    // проверить время последнего обновления
    // if (time_keep_updated + KEEP_IMPULSE_UPDATE > Device.dwTimeGlobal) return;

    if (!object)
        return;

    // проверить высоту
    float cur_h = object->Position().y;
    if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
        cur_h = object->character_physics_support()->active_ragdoll()->GetSimulatedPosition(0).y;

    // установить dir в соответствие с текущей высотой
    Fvector dir;
    if (cur_h > target_height + 0.4f)
        dir.set(0.f, -1.0f, 0.f);
    else if (cur_h < target_height - 0.4f)
        dir.set(0.f, 1.0f, 0.f);
    else
    {
        dir.set(Random.randF(-1.0f, 1.0f), Random.randF(-0.3f, 0.3f), Random.randF(-1.0f, 1.0f));
        dir.normalize_safe();
    }

    if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
    {
        Fvector accel_dir = dir;
        accel_dir.mul(5.0f);
        if (OnServer())
            (object->m_pPhysicsShell->get_ElementByStoreOrder(0))->applyGravityAccel(accel_dir);
    }
    else if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
    {
        auto ragdoll = object->character_physics_support()->active_ragdoll();
        Fvector pos = ragdoll->GetSimulatedPosition(0);
        Fvector cur_v, cur_w;
        ragdoll->GetVelocity(cur_v, cur_w);

        float dy = target_height - pos.y;
        float mass = 75.0f;
        float target_v_y = std::clamp(dy * 3.0f, -1.0f, 1.0f);
        float delta_v_y = target_v_y - cur_v.y;
        float lift_accel = 9.81f + std::clamp(delta_v_y * 8.0f, -8.0f, 8.0f);

        Fvector total_imp = Fvector().set(0.0f, lift_accel * mass * 0.0166f, 0.0f);
        if (OnServer())
            ragdoll->ApplyLinearImpulse(Fvector().set(0, 1, 0), total_imp.y);
    }

    // установить время последнего обновления
    time_keep_updated = Device.dwTimeGlobal;

    update_hold_sound();
}

void CTelekineticObject::release()
{
    if (!object)
        return;

    bool has_shell = object->m_pPhysicsShell && object->m_pPhysicsShell->isActive();
    bool has_ragdoll = object->character_physics_support() && object->character_physics_support()->active_ragdoll();

    if (!has_shell && !has_ragdoll)
        return;

    Fvector dir_inv;
    dir_inv.set(Random.randF(-1.0f, 1.0f), Random.randF(-0.3f, 0.3f), Random.randF(-1.0f, 1.0f));
    dir_inv.normalize_safe();

    if (has_shell)
    {
        object->m_pPhysicsShell->set_ApplyByGravity(TRUE);
        if (OnServer())
        {
            float impulse = std::clamp(strength * 5.0f, 5.0f, 25.0f);
            object->m_pPhysicsShell->applyImpulse(dir_inv, impulse * object->m_pPhysicsShell->getMass() / 100.f);
        }
    }
    else if (has_ragdoll)
    {
        if (OnServer())
        {
            object->character_physics_support()->active_ragdoll()->ApplyLinearImpulse(dir_inv, 15.0f);
        }
    }

    // state = TS_None;
    switch_state(TS_None);
}

void CTelekineticObject::fire_t(const Fvector& target, float time)
{
    switch_state(TS_Fire);
    // time_fire_started	= Device.dwTimeGlobal;

    if (!object)
        return;

    if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
    {
        // включить гравитацию
        object->m_pPhysicsShell->set_ApplyByGravity(TRUE);

        Fvector transference;
        transference.sub(target, object->Position());
        TransferenceToThrowVel(transference, time, object->EffectiveGravity());
        object->m_pPhysicsShell->set_LinearVel(transference);
    }
    else if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
    {
        Fvector dir;
        dir.sub(target, object->Position());
        dir.normalize_safe();

        if (OnServer())
        {
            object->character_physics_support()->active_ragdoll()->ApplyLinearImpulse(dir, 160.0f);
        }
    }

    if (sound_throw._handle())
        sound_throw.play_at_pos(object, object->Position());

    if (sound_hold._handle() && sound_hold._feedback())
        sound_hold.stop();

    update_hold_sound();
}

void CTelekineticObject::fire(const Fvector& target, float power)
{
    switch_state(TS_Fire);
    time_fire_started = Device.dwTimeGlobal;

    if (!object)
        return;

    Fvector dir;
    dir.sub(target, object->Position());
    dir.normalize_safe();

    if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
    {
        object->m_pPhysicsShell->set_ApplyByGravity(TRUE);

        if (OnServer())
        {
            float total_impulse = power * 15.0f * object->m_pPhysicsShell->getMass();
            float per_element_impulse = total_impulse / object->m_pPhysicsShell->Elements().size();

            for (u32 i = 0; i < object->m_pPhysicsShell->get_ElementsNumber(); i++)
                object->m_pPhysicsShell->get_ElementByStoreOrder(u16(i))->applyImpulse(dir, per_element_impulse);
        }
    }
    else if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
    {
        if (OnServer())
        {
            float total_impulse = power * 180.0f;
            object->character_physics_support()->active_ragdoll()->ApplyLinearImpulse(dir, total_impulse);
        }
    }
}

bool CTelekineticObject::check_height()
{
    if (!object)
        return true;

    float cur_y = object->Position().y;
    if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
        cur_y = object->character_physics_support()->active_ragdoll()->GetSimulatedPosition(0).y;

    return (cur_y > target_height);
}
bool CTelekineticObject::check_raise_time_out()
{
    if (time_raise_started + RAISE_MAX_TIME < Device.dwTimeGlobal)
        return true;

    return false;
}

void CTelekineticObject::enable()
{
    if (object && object->m_pPhysicsShell)
        object->m_pPhysicsShell->Enable();
}

void CTelekineticObject::rotate()
{
    if (!object)
        return;

    // вычислить направление
    Fvector dir;
    dir.random_dir();
    dir.normalize_safe();

    if (object->m_pPhysicsShell && object->m_pPhysicsShell->isActive())
    {
        if (OnServer())
            object->m_pPhysicsShell->applyImpulse(dir, 2.5f * object->m_pPhysicsShell->getMass());
    }
    else if (object->character_physics_support() && object->character_physics_support()->active_ragdoll())
    {
        if (OnServer())
            object->character_physics_support()->active_ragdoll()->ApplyLinearImpulse(dir, 2.5f);
    }
}

bool CTelekineticObject::can_activate(CPhysicsShellHolder* obj)
{
    if (!obj)
        return false;
    if (smart_cast<CPHDestroyableNotificate*>(obj))
        return false;
    return (obj->m_pPhysicsShell || (obj->character_physics_support() && obj->character_physics_support()->active_ragdoll()));
}

void CTelekineticObject::update_hold_sound()
{
    if (!sound_hold._handle())
        return;

    if (sound_hold._feedback())
        sound_hold.set_position(object->Position());
    else
        sound_hold.play_at_pos(object, object->Position());
}
