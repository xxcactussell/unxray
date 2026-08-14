#pragma once

void CPHSimpleCharacter::UpdateStaticDamage(const Fvector& normal, const Fvector& pos, SGameMtl* tri_material, bool bo1)
{
    Fvector v;
    GetPhysicsCore()->GetBodyLinearVelocity(m_body, v);
    
    float norm_prg = _abs(v.dotproduct(normal));
    float smag = v.square_magnitude();
    float plane_pgr = _sqrt(smag - norm_prg * norm_prg);
    float mag = 0.f;
    
    if (tri_material->Flags.test(SGameMtl::flPassable))
    {
        mag = _sqrt(smag) * tri_material->fBounceDamageFactor;
    }
    else
    {
        float vel_prg = _max(plane_pgr * tri_material->fPHFriction, norm_prg);
        mag = vel_prg * tri_material->fBounceDamageFactor;
    }
    
    if (mag > m_collision_damage_info.m_contact_velocity)
    {
        m_collision_damage_info.m_contact_velocity = mag;
        m_collision_damage_info.m_dmc_signum = bo1 ? 1.f : -1.f;
        m_collision_damage_info.m_dmc_type = SCollisionDamageInfo::ctStatic;
        
        m_collision_damage_info.m_damage_normal = normal;
        m_collision_damage_info.m_damage_pos = pos;
        
        m_collision_damage_info.m_obj_id = u16(-1);
    }
}

void CPHSimpleCharacter::UpdateDynamicDamage(const Fvector& normal, const Fvector& pos, BodyHandle b2, u16 obj_material_idx, bool bo1)
{
    Fvector vel, obj_vel;
    GetPhysicsCore()->GetBodyLinearVelocity(m_body, vel);
    GetPhysicsCore()->GetBodyLinearVelocity(b2, obj_vel);
    
    float m_mass_other = GetPhysicsCore()->GetBodyMass(b2);

    float norm_vel = vel.dotproduct(normal);
    float norm_obj_vel = obj_vel.dotproduct(normal);

    if ((bo1 && norm_vel > norm_obj_vel) || (!bo1 && norm_obj_vel > norm_vel))
        return;

    Fvector Pc;
    Pc.x = vel.x * m_mass + obj_vel.x * m_mass_other;
    Pc.y = vel.y * m_mass + obj_vel.y * m_mass_other;
    Pc.z = vel.z * m_mass + obj_vel.z * m_mass_other;

    float Kself = norm_vel * norm_vel * m_mass / 2.f;
    float Kobj = norm_obj_vel * norm_obj_vel * m_mass_other / 2.f;

    float Pcnorm = Pc.dotproduct(normal);
    float KK = Pcnorm * Pcnorm / (m_mass + m_mass_other) / 2.f;
    float accepted_energy = Kself * m_collision_damage_factor + Kobj * object_damage_factor - KK;

    float c_vel = 0.f;
    if (accepted_energy > 0.f)
    {
        SGameMtl* obj_material = GMLib.GetMaterialByIdx(obj_material_idx);
        c_vel = _sqrt(accepted_energy / m_mass * 2.f) * obj_material->fBounceDamageFactor;
    }
    
    if (c_vel > m_collision_damage_info.m_contact_velocity)
    {
        // Для Jolt можно извлечь IPhysicsShellHolder через userData тела 
        // или передавать его снаружи (через CPhysicsGeom)
        // Если userData в ядре настроена на возврат IPhysicsShellHolder:
        IPhysicsShellHolder* obj = bo1 ? (IPhysicsShellHolder*)GetPhysicsCore()->GetBodyUserData(b2) 
                                       : (IPhysicsShellHolder*)GetPhysicsCore()->GetBodyUserData(m_body);
        
        if (obj && !obj->ObjectGetDestroy())
        {
            m_collision_damage_info.m_contact_velocity = c_vel;
            m_collision_damage_info.m_dmc_signum = bo1 ? 1.f : -1.f;
            m_collision_damage_info.m_dmc_type = SCollisionDamageInfo::ctObject;
            
            m_collision_damage_info.m_damage_normal = normal;
            m_collision_damage_info.m_damage_pos = pos;
            
            m_collision_damage_info.m_hit_callback = obj->ObjectGetCollisionHitCallback();
            m_collision_damage_info.m_obj_id = obj->ObjectID();
        }
    }
}

IC void CPHSimpleCharacter::foot_material_update(u16 contact_material_idx, u16 foot_material_idx)
{
    if (m_elevator_state.UpdateMaterial(*p_lastMaterialIDX))
        return;
    if (*p_lastMaterialIDX != u16(-1) &&
        GMLib.GetMaterialByIdx(*p_lastMaterialIDX)->Flags.test(SGameMtl::flPassable) && !b_foot_mtl_check)
        return;
    b_foot_mtl_check = false;

    const SGameMtl* contact_material = GMLib.GetMaterialByIdx(contact_material_idx);

    if (contact_material->Flags.test(SGameMtl::flPassable))
    {
        if (contact_material->Flags.test(SGameMtl::flInjurious))
            injuriousMaterialIDX = contact_material_idx;
        else
            *p_lastMaterialIDX = contact_material_idx;
    }
    else
        *p_lastMaterialIDX = foot_material_idx;
}
