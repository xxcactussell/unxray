#include "StdAfx.h"

#include "PHDynamicData.h"
#include "Physics.h"
#include "ExtendedGeom.h"
#include "xrCDB/Intersect.hpp"

#include "PHAICharacter.h"
#include "xrEngine/device.h"
#include "xrPhysicsCore/IPhysicsCore.h"

#ifdef DEBUG
#include "debug_output.h"
#endif

CPHAICharacter::CPHAICharacter() { m_forced_physics_control = false; }

void CPHAICharacter::Create(Fvector sizes)
{
    inherited::Create(sizes);
    m_forced_physics_control = false;
}

bool CPHAICharacter::TryPosition(Fvector pos, bool exact_state)
{
    if (!b_exist || m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE)
        return false;
        
    if (m_forced_physics_control || JumpState())
        return false; 
        
    if (DoCollideObj())
        return false;
        
    Fvector current_pos;
    GetPosition(current_pos);
    Fvector cur_vel;
    GetVelocity(cur_vel);

    Fvector displace;
    displace.sub(pos, current_pos);
    float disp_mag = displace.magnitude();

    if (fis_zero(disp_mag) || fis_zero(Device.fTimeDelta))
        return true;
        
    const u32 max_steps = 15;
    const float fmax_steps = float(max_steps);
    float fsteps_num = 1.f;
    u32 steps_num = 1;
    float disp_pstep = FootRadius();
    float rest = 0.f;

    float parts = disp_mag / disp_pstep;
    fsteps_num = floor(parts);
    steps_num = iFloor(parts);
    
    if (steps_num > max_steps)
    {
        steps_num = max_steps;
        fsteps_num = fmax_steps;
        disp_pstep = disp_mag / fsteps_num;
    }
    rest = disp_mag - fsteps_num * disp_pstep;

    Fvector vel;
    vel.mul(displace, disp_pstep / fixed_step / disp_mag);
    bool ret = true;
    
    // Сохраняем и временно отключаем гравитацию для попытки перемещения
    float save_gm = GetPhysicsCore()->GetBodyGravityFactor(m_char_handle);
    GetPhysicsCore()->SetBodyGravityFactor(m_char_handle, 0.0f);
    
    for (u32 i = 0; steps_num > i; ++i)
    {
        SetVelocity(vel);
        Enable();
        if (!step_single(fixed_step))
        {
            SetVelocity(cur_vel);
            ret = false;
            break;
        }
    }

    vel.mul(displace, rest / fixed_step / disp_mag);
    SetVelocity(vel);
    Enable();
    ret = step_single(fixed_step);

    // Восстанавливаем гравитацию
    GetPhysicsCore()->SetBodyGravityFactor(m_char_handle, save_gm);
    SetVelocity(cur_vel);
    
    Fvector pos_new;
    GetPosition(pos_new);

    SetPosition(pos_new);
    m_last_move.sub(pos_new, current_pos).mul(1.f / Device.fTimeDelta);
    m_char_handle_interpolation.UpdatePositions();
    m_char_handle_interpolation.UpdatePositions();
    
    if (ret)
        Disable();
        
    m_collision_damage_info.m_contact_velocity = 0.f;
    return ret;
}

void CPHAICharacter::Jump(const Fvector& jump_velocity)
{
    b_jump = true;
    m_jump_accel.set(jump_velocity);
}

void CPHAICharacter::ValidateWalkOn()
{
    inherited::ValidateWalkOn();
}

void CPHAICharacter::InitContact(bool& do_collide, bool bo1, float depth, CPhysicsGeom* my_geom, CPhysicsGeom* oposite_geom, u16 material_idx_1, u16 material_idx_2)
{
    SGameMtl* material_1 = GMLib.GetMaterialByIdx(material_idx_1);
    SGameMtl* material_2 = GMLib.GetMaterialByIdx(material_idx_2);
    
    if ((material_1 && material_1->Flags.test(SGameMtl::flActorObstacle)) ||
        (material_2 && material_2->Flags.test(SGameMtl::flActorObstacle)))
        do_collide = true;
        
    inherited::InitContact(do_collide, bo1, depth, my_geom, oposite_geom, material_idx_1, material_idx_2);
    
    // Примечание: В оригинальном ODE здесь сбрасывалось трение (c->surface.mu = 0.00f). 
    // В Jolt Physics трение настраивается через Contact Listener.

    if (my_geom && oposite_geom && my_geom->ph_object && oposite_geom->ph_object && 
        my_geom->ph_object->CastType() == tpCharacter &&
        oposite_geom->ph_object->CastType() == tpCharacter)
    {
        b_on_object = true;
        b_valide_wall_contact = false;
    }
    
#ifdef DEBUG
    if (debug_output().ph_dbg_draw_mask().test(phDbgNeverUseAiPhMove))
        do_collide = false;
#endif
}

#ifdef DEBUG
void CPHAICharacter::OnRender()
{
    inherited::OnRender();
}
#endif
