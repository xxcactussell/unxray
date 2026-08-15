#include "StdAfx.h"

#include "icollisiondamagereceiver.h"
#include "IPhysicsShellHolder.h"
#include "ExtendedGeom.h"
#include "Physics.h"

void DamageReceiverCollisionCallback(bool& do_colide, bool bo1, CPhysicsGeom* geom1, CPhysicsGeom* geom2, const Fvector& contact_normal, const Fvector& contact_pos, SGameMtl* material_1, SGameMtl* material_2)
{
    if (material_1->Flags.test(SGameMtl::flPassable) || material_2->Flags.test(SGameMtl::flPassable))
        return;

    CharacterVirtualHandle b1 = geom1 ? geom1->m_char_handle : INVALID_CHARACTER_VIRTUAL_HANDLE;
    CharacterVirtualHandle b2 = geom2 ? geom2->m_char_handle : INVALID_CHARACTER_VIRTUAL_HANDLE;

    CPhysicsGeom* geom_self = bo1 ? geom1 : geom2;
    CPhysicsGeom* geom_damager = bo1 ? geom2 : geom1;

    SGameMtl* material_self = bo1 ? material_1 : material_2;
    SGameMtl* material_damager = bo1 ? material_2 : material_1;
    
    VERIFY(geom_self);
    
    IPhysicsShellHolder* o_self = geom_self->ph_ref_object;
    IPhysicsShellHolder* o_damager = geom_damager ? geom_damager->ph_ref_object : nullptr;
    
    u16 source_id = o_damager ? o_damager->ObjectID() : u16(-1);

    ICollisionDamageReceiver* dr = o_self->ObjectPhCollisionDamageReceiver();
    VERIFY2(dr, "wrong callback");

    float damager_material_factor = material_damager->fBounceDamageFactor;

    if (geom_damager && geom_damager->ph_object && geom_damager->ph_object->CastType() == CPHObject::tpCharacter)
        o_damager->BonceDamagerCallback(damager_material_factor);

    float dfs = (material_self->fBounceDamageFactor + damager_material_factor);
    if (fis_zero(dfs))
        return;
        
    Fvector dir = contact_normal;
    Fvector pos;
    
    if (geom_self->m_char_handle != INVALID_CHARACTER_VIRTUAL_HANDLE) {
        Fmatrix self_transform;
        GetPhysicsCore()->GetBodyTransform(geom_self->m_char_handle, self_transform);
        pos.sub(contact_pos, self_transform.c);
    } else {
        pos.set(0, 0, 0);
    }

    dr->CollisionHit(
        source_id, geom_self->m_bone_id, E_NL(b1, b2, contact_normal) * damager_material_factor / dfs, dir, pos);
}

void BreakableObjectCollisionCallback(
    bool& do_colide, bool bo1, CPhysicsGeom* geom1, CPhysicsGeom* geom2, const Fvector& contact_normal, const Fvector& contact_pos, SGameMtl* material_1, SGameMtl* material_2)
{
    VERIFY(geom1);
    VERIFY(geom2);

    ICollisionDamageReceiver* damag_receiver = 0;
    CharacterVirtualHandle body = INVALID_CHARACTER_VIRTUAL_HANDLE;
    float norm_sign = 0;

    if (bo1)
    {
        VERIFY(geom1->ph_ref_object);
        damag_receiver = geom1->ph_ref_object->ObjectPhCollisionDamageReceiver();
        body = geom2->m_char_handle;
        norm_sign = -1.f;
    }
    else
    {
        VERIFY(geom2->ph_ref_object);
        damag_receiver = geom2->ph_ref_object->ObjectPhCollisionDamageReceiver();
        body = geom1->m_char_handle;
        norm_sign = 1.f;
    }
    VERIFY(damag_receiver);

    float c_damage = E_NlS(body, contact_normal, norm_sign);
    Fvector dir;
    dir.set(-contact_normal.x * norm_sign, -contact_normal.y * norm_sign, -contact_normal.z * norm_sign);
    
    Fvector pos = contact_pos;

    damag_receiver->CollisionHit(u16(-1), u16(-1), c_damage, dir, pos);
}
