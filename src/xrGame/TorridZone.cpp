#include "StdAfx.h"
#include "TorridZone.h"
#include "xrEngine/ObjectAnimator.h"
#include "xrServer_Objects_ALife_Monsters.h"
#include "xrEngine/xr_collide_form.h"

CTorridZone::CTorridZone() { m_animator = xr_new<CObjectAnimator>(); }
CTorridZone::~CTorridZone() { xr_delete(m_animator); }
bool CTorridZone::net_Spawn(CSE_Abstract* DC)
{
    if (!inherited::net_Spawn(DC))
        return (FALSE);

    CSE_Abstract* abstract = (CSE_Abstract*)(DC);
    CSE_ALifeTorridZone* zone = smart_cast<CSE_ALifeTorridZone*>(abstract);
    VERIFY(zone);

    m_animator->Load(zone->get_motion());
    m_animator->Play(true);

    return (TRUE);
}

void CTorridZone::UpdateWorkload(u32 dt)
{
    inherited::UpdateWorkload(dt);
    m_animator->Update(float(dt) / 1000.f);
    XFORM().set(m_animator->XFORM());
    OnMove();
}

void CTorridZone::shedule_Update(u32 dt)
{
    inherited::shedule_Update(dt);

    if (m_idle_sound._feedback())
        m_idle_sound.set_position(XFORM().c);
    if (m_blowout_sound._feedback())
        m_blowout_sound.set_position(XFORM().c);
    if (m_hit_sound._feedback())
        m_hit_sound.set_position(XFORM().c);
    if (m_entrance_sound._feedback())
        m_entrance_sound.set_position(XFORM().c);
}

bool CTorridZone::Enable()
{
    bool res = inherited::Enable();
    if (res)
    {
        m_animator->Stop();
        m_animator->Play(true);
    }
    return res;
}

bool CTorridZone::Disable()
{
    bool res = inherited::Disable();
    if (res)
        m_animator->Stop();

    return res;
}

// Lain: added
bool CTorridZone::light_in_slow_mode() { return false; }
bool CTorridZone::AlwaysTheCrow() { return true; }

void CTorridZone::Affect(SZoneObjectInfo* O)
{
    CPhysicsShellHolder* pGameObject = smart_cast<CPhysicsShellHolder*>(O->object);
    if (!pGameObject || O->zone_ignore)
        return;

    Fvector P;
    XFORM().transform_tiny(P, GetCForm()->getSphere().P);

    float dist = pGameObject->Position().distance_to(P) - pGameObject->Radius();
    float power = Power(dist > 0.f ? dist : 0.f, Radius());

    if (power > 0.01f)
    {
        Fvector position_in_bone_space = Fvector().set(0.f, 0.f, 0.f);
        Fvector hit_dir = Fvector().set(0.f, 0.f, 0.f);
        CreateHit(pGameObject->ID(), ID(), hit_dir, power, 0, position_in_bone_space, 0.0f, m_eHitTypeBlowout);
        PlayHitParticles(pGameObject);
    }
}
