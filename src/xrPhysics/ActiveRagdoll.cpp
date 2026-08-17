#include "StdAfx.h"
#include "ActiveRagdoll.h"
#include "IPhysicsShellHolder.h"
#include "MathUtils.h"

// Helper function
static bool shape_is_physic(const SBoneShape& shape) {
    return shape.type != SBoneShape::stNone && !shape.flags.test(SBoneShape::sfNoPhysics);
}

// ===========================================================================
// CActiveRagdollSkeletonMapper
// ===========================================================================

void CActiveRagdollSkeletonMapper::Build(IKinematics* kinematics) {
    m_bone_to_part.clear();
    m_part_to_bone.clear();
    m_bind_pose_inv.clear();
    
    if (!kinematics) return;
    
    u16 bone_count = kinematics->LL_BoneCount();
    m_bone_to_part.resize(bone_count);
    m_part_to_bone.resize(bone_count);
    m_bind_pose_inv.resize(bone_count);
    
    for (u16 bone_id = 0; bone_id < bone_count; ++bone_id) {
        m_bone_to_part[bone_id] = bone_id;
        m_part_to_bone[bone_id] = bone_id;
        
        const IBoneData& bone_data = kinematics->GetBoneData(bone_id);
        Fmatrix bind_transform = bone_data.get_bind_transform();
        Fquaternion bind_rot;
        bind_rot.set(bind_transform);
        bind_rot.inverse();
        m_bind_pose_inv[bone_id] = bind_rot;
    }
}

u16 CActiveRagdollSkeletonMapper::BoneToPart(u16 bone_id) const {
    if (bone_id < m_bone_to_part.size()) return m_bone_to_part[bone_id];
    return u16(-1);
}

u16 CActiveRagdollSkeletonMapper::PartToBone(u16 part_index) const {
    if (part_index < m_part_to_bone.size()) return m_part_to_bone[part_index];
    return u16(-1);
}

// ===========================================================================
// CActiveRagdollSettingsBuilder
// ===========================================================================

SRagdollSettings CActiveRagdollSettingsBuilder::BuildSettings(IKinematics* kinematics, const CActiveRagdollSkeletonMapper& mapper) {
    SRagdollSettings settings;
    settings.default_motor.stiffness = 1000.f;
    settings.default_motor.damping = 100.f;
    
    if (!kinematics) return settings;
    
    u32 num_parts = kinematics->LL_BoneCount();
    settings.parts.resize(num_parts);
    
    xr_vector<Fmatrix> bind_matrices;
    kinematics->LL_GetBindTransform(bind_matrices);
    
    for (u32 part_idx = 0; part_idx < num_parts; ++part_idx) {
        u16 bone_id = (u16)part_idx;
        const IBoneData& bone_data = kinematics->GetBoneData(bone_id);
        
        SRagdollPartDesc& part_desc = settings.parts[part_idx];
        part_desc.bone_id = bone_id;
        
        // Parent index in skeleton hierarchy
        u16 parent_bone_id = bone_data.GetParentID();
        if (parent_bone_id == u16(-1) || parent_bone_id == BI_NONE) {
            part_desc.parent_index = -1;
        } else {
            part_desc.parent_index = (int)parent_bone_id;
        }
        
        // Build Shape with local offset and rotation (RotatedTranslatedShape)
        part_desc.shape = nullptr;
        const SBoneShape& shape = bone_data.get_shape();
        if (shape_is_physic(shape)) {
            if (shape.type == SBoneShape::stBox) {
                PhysicsShapeHandle base_box = GetPhysicsCore()->CreateBoxShape(Fvector().set(shape.box.m_halfsize));
                Fmatrix box_mat;
                box_mat.i = shape.box.m_rotate.i;
                box_mat.j = shape.box.m_rotate.j;
                box_mat.k = shape.box.m_rotate.k;
                box_mat.c = shape.box.m_translate;
                Fquaternion box_rot;
                box_rot.set(box_mat);
                part_desc.shape = GetPhysicsCore()->CreateRotatedTranslatedShape(base_box, shape.box.m_translate, box_rot);
            } else if (shape.type == SBoneShape::stSphere) {
                PhysicsShapeHandle base_sphere = GetPhysicsCore()->CreateSphereShape(shape.sphere.R);
                part_desc.shape = GetPhysicsCore()->CreateRotatedTranslatedShape(base_sphere, shape.sphere.P, Fquaternion().identity());
            } else if (shape.type == SBoneShape::stCylinder) {
                PhysicsShapeHandle base_capsule = GetPhysicsCore()->CreateCapsuleShape(shape.cylinder.m_radius, shape.cylinder.m_height / 2.0f);
                
                // Align capsule (which Jolt creates along Y axis) with cylinder.m_direction
                Fmatrix cyl_mat;
                Fvector norm = shape.cylinder.m_direction;
                if (norm.square_magnitude() > 0.0001f) {
                    norm.normalize();
                } else {
                    norm.set(0.f, 1.f, 0.f);
                }
                Fvector y = norm;
                Fvector x, z;
                if (_abs(y.x) > 0.9f) z.set(0, 0, 1); else z.set(1, 0, 0);
                x.crossproduct(y, z); x.normalize();
                z.crossproduct(x, y); z.normalize();
                cyl_mat.i = x; cyl_mat.j = y; cyl_mat.k = z; cyl_mat.c = shape.cylinder.m_center;
                
                Fquaternion cyl_rot;
                cyl_rot.set(cyl_mat);
                part_desc.shape = GetPhysicsCore()->CreateRotatedTranslatedShape(base_capsule, shape.cylinder.m_center, cyl_rot);
            }
        }
        
        if (!part_desc.shape) {
            // Micro dummy shape for bones without direct collision to maintain 1:1 skeleton hierarchy
            part_desc.shape = GetPhysicsCore()->CreateSphereShape(0.01f);
            part_desc.mass = 0.05f;
        } else {
            part_desc.mass = std::max(bone_data.get_mass(), 0.05f);
        }
        
        // Cumulative Model-Space Rest Pose
        if (part_idx < bind_matrices.size()) {
            part_desc.position = bind_matrices[part_idx].c;
            part_desc.rotation.set(bind_matrices[part_idx]);
        } else {
            part_desc.position.set(0, 0, 0);
            part_desc.rotation.identity();
        }
        
        // Constraint with parent
        if (part_desc.parent_index != -1) {
            const SJointIKData& ik_data = bone_data.get_IK_data();
            
            SRagdollConstraintDesc c_desc;
            c_desc.child_index = (int)part_idx;
            
            // Longitudinal twist axis along bone length (Y axis in Model Space)
            c_desc.twist_axis = Fvector().set(0.f, 1.f, 0.f);
            c_desc.plane_axis = Fvector().set(0.f, 0.f, 1.f);
            
            if (ik_data.type == jtRigid || ik_data.type == jtNone) {
                c_desc.swing_limit_y = 0.f;
                c_desc.swing_limit_z = 0.f;
                c_desc.twist_limit_min = 0.f;
                c_desc.twist_limit_max = 0.f;
            } else {
                float lim_x = std::max(0.01f, ik_data.limits[0].limit.y - ik_data.limits[0].limit.x) * 0.5f;
                float lim_y = std::max(0.01f, ik_data.limits[1].limit.y - ik_data.limits[1].limit.x) * 0.5f;
                c_desc.swing_limit_y = std::min(lim_x, float(M_PI * 0.5f));
                c_desc.swing_limit_z = std::min(lim_y, float(M_PI * 0.5f));
                c_desc.twist_limit_min = ik_data.limits[2].limit.x;
                c_desc.twist_limit_max = ik_data.limits[2].limit.y;
            }
            c_desc.max_friction_torque = ik_data.friction;
            settings.constraints.push_back(c_desc);
        }
    }
    
    return settings;
}

// ===========================================================================
// CActiveRagdollController
// ===========================================================================

CActiveRagdollController::~CActiveRagdollController() {
    Deactivate();
}

void CActiveRagdollController::Initialize(IKinematics* kinematics, IPhysicsShellHolder* holder) {
    if (!kinematics || !holder) return;
    
    m_kinematics = kinematics;
    m_holder = holder;
    m_mapper.Build(kinematics);
    
    SRagdollSettings settings = CActiveRagdollSettingsBuilder::BuildSettings(kinematics, m_mapper);
    m_ragdoll_handle = GetPhysicsCore()->CreateRagdoll(settings);
    
    if (m_ragdoll_handle != INVALID_RAGDOLL_HANDLE) {
        u32 part_count = (u32)settings.parts.size();
        m_target_matrices.resize(part_count);
        m_simulated_positions.resize(part_count);
        m_simulated_rotations.resize(part_count);
        m_part_reactions.resize(part_count);
        m_cb_data.resize(part_count);
        
        xr_vector<Fmatrix> bind_matrices;
        m_kinematics->LL_GetBindTransform(bind_matrices);
        
        // 1. Set kinematic for all parts initially while alive
        GetPhysicsCore()->SetRagdollAllPartsKinematic(m_ragdoll_handle, true);
        
        // 2. Set user data on ragdoll bodies so CharacterVirtual ignores self collisions
        GetPhysicsCore()->SetRagdollUserData(m_ragdoll_handle, m_holder);
        
        // 3. Add ragdoll to world
        GetPhysicsCore()->AddRagdollToWorld(m_ragdoll_handle, true);
        
        // 4. Instantly position all bodies at the character's world spawn pose and reset constraint strain
        GetPhysicsCore()->SetRagdollWorldPose(m_ragdoll_handle, m_holder->ObjectXFORM(), bind_matrices.data(), (u32)bind_matrices.size());
        
        // 5. Read back synchronized world positions immediately
        GetPhysicsCore()->GetRagdollAllTransforms(m_ragdoll_handle, m_simulated_positions.data(), m_simulated_rotations.data(), part_count);
        
        // 6. Setup bone callbacks
        for (u32 i = 0; i < part_count; ++i) {
            u16 bone_id = m_mapper.PartToBone((u16)i);
            if (bone_id == u16(-1) || bone_id >= m_kinematics->LL_BoneCount()) continue;
            
            m_cb_data[i].controller = this;
            m_cb_data[i].part_index = (u16)i;
            m_cb_data[i].bone_id = bone_id;
            
            CBoneInstance& B = m_kinematics->LL_GetBoneInstance(bone_id);
            B.set_callback(bctCustom, BonesCallback, &m_cb_data[i]);
        }
        
        m_state = ERagdollState::Active;
    }
}

void CActiveRagdollController::Deactivate() {
    if (m_ragdoll_handle != INVALID_RAGDOLL_HANDLE) {
        GetPhysicsCore()->DestroyRagdoll(m_ragdoll_handle);
        m_ragdoll_handle = INVALID_RAGDOLL_HANDLE;
    }
    m_state = ERagdollState::Inactive;
    
    // Clear callbacks
    if (m_kinematics && !m_cb_data.empty()) {
        for (u32 i = 0; i < m_cb_data.size(); ++i) {
            u16 bone_id = m_cb_data[i].bone_id;
            CBoneInstance& B = m_kinematics->LL_GetBoneInstance(bone_id);
            if (B.callback_param() == &m_cb_data[i]) {
                B.reset_callback();
            }
        }
        m_cb_data.clear();
    }
    m_part_reactions.clear();
}

void CActiveRagdollController::OnDeath() {
    if (m_state == ERagdollState::Active) {
        m_state = ERagdollState::Dying;
        m_death_decay_timer = 0.f;
    }
}

void CActiveRagdollController::SetMotorDefaults(float stiffness, float damping) {
    m_motor_stiffness = stiffness;
    m_motor_damping = damping;
    if (m_state == ERagdollState::Active && m_ragdoll_handle != INVALID_RAGDOLL_HANDLE) {
        GetPhysicsCore()->SetRagdollMotorStiffness(m_ragdoll_handle, stiffness);
        GetPhysicsCore()->SetRagdollMotorDamping(m_ragdoll_handle, damping);
    }
}

void CActiveRagdollController::KnockDown(u16 bone_id, const Fvector& dir, float impulse, const Fvector& hit_pos) {
    if (m_state == ERagdollState::Inactive || m_state == ERagdollState::Dead || m_state == ERagdollState::Dying || m_ragdoll_handle == INVALID_RAGDOLL_HANDLE)
        return;
    
    m_state = ERagdollState::KnockedDown;
    m_knockdown_timer = 0.0f;
    
    // Switch all parts to dynamic
    GetPhysicsCore()->SetRagdollAllPartsKinematic(m_ragdoll_handle, false);
    // Turn off motors for free-fall ragdoll
    GetPhysicsCore()->SetRagdollMotorState(m_ragdoll_handle, false);
    
    // Apply physical impulse
    u16 part_idx = m_mapper.BoneToPart(bone_id);
    if (part_idx == u16(-1) && m_kinematics) {
        u16 parent_id = m_kinematics->GetBoneData(bone_id).GetParentID();
        while (parent_id != u16(-1)) {
            part_idx = m_mapper.BoneToPart(parent_id);
            if (part_idx != u16(-1)) break;
            parent_id = m_kinematics->GetBoneData(parent_id).GetParentID();
        }
    }
    if (part_idx == u16(-1) || part_idx >= m_mapper.m_part_to_bone.size()) {
        part_idx = 0;
    }
    
    Fvector impulse_vec = dir;
    impulse_vec.mul(impulse);
    GetPhysicsCore()->ApplyRagdollImpulse(m_ragdoll_handle, (u32)part_idx, impulse_vec, hit_pos);
}

void CActiveRagdollController::ApplyHit(u16 bone_id, const Fvector& dir, float impulse, const Fvector& hit_pos) {
    if (m_state == ERagdollState::Inactive || m_ragdoll_handle == INVALID_RAGDOLL_HANDLE) return;
    
    // Find matching part index or traverse up parent chain
    u16 part_idx = m_mapper.BoneToPart(bone_id);
    if (part_idx == u16(-1) && m_kinematics) {
        u16 parent_id = m_kinematics->GetBoneData(bone_id).GetParentID();
        while (parent_id != u16(-1)) {
            part_idx = m_mapper.BoneToPart(parent_id);
            if (part_idx != u16(-1)) break;
            parent_id = m_kinematics->GetBoneData(parent_id).GetParentID();
        }
    }
    
    if (part_idx == u16(-1) || part_idx >= m_mapper.m_part_to_bone.size()) {
        part_idx = 0; // Fallback to root (Pelvis)
    }
    
    // Calculate physical impulse vector
    Fvector impulse_vec = dir;
    impulse_vec.mul(impulse);
    
    // Apply impulse to Jolt physics body
    GetPhysicsCore()->ApplyRagdollImpulse(m_ragdoll_handle, (u32)part_idx, impulse_vec, hit_pos);
    
    // Apply hit flinch to motors for living character
    if (m_state == ERagdollState::Active && part_idx < m_part_reactions.size()) {
        float flinch = std::min(1.0f, impulse / 150.0f);
        if (flinch < 0.2f) flinch = 0.2f; // minimum perceptible flinch on hit
        
        m_part_reactions[part_idx].flinch_factor = flinch;
        m_part_reactions[part_idx].recovery_duration = 0.35f + flinch * 0.15f;
        m_part_reactions[part_idx].elapsed_time = 0.0f;
        m_part_reactions[part_idx].active = true;
        
        // Weaken immediate parent joint if exists
        if (m_kinematics) {
            u16 b_id = m_mapper.PartToBone(part_idx);
            u16 p_bone = m_kinematics->GetBoneData(b_id).GetParentID();
            if (p_bone != u16(-1)) {
                u16 p_part = m_mapper.BoneToPart(p_bone);
                if (p_part != u16(-1) && p_part < m_part_reactions.size()) {
                    m_part_reactions[p_part].flinch_factor = std::max(m_part_reactions[p_part].flinch_factor, flinch * 0.6f);
                    m_part_reactions[p_part].recovery_duration = 0.3f;
                    m_part_reactions[p_part].elapsed_time = 0.0f;
                    m_part_reactions[p_part].active = true;
                }
            }
        }
    }
}

void CActiveRagdollController::ApplyRadialImpulse(const Fvector& center, float radius, float max_impulse) {
    if (m_state == ERagdollState::Inactive || m_ragdoll_handle == INVALID_RAGDOLL_HANDLE) return;
    
    for (u32 i = 0; i < m_simulated_positions.size(); ++i) {
        Fvector to_part;
        to_part.sub(m_simulated_positions[i], center);
        float dist = to_part.magnitude();
        if (dist < radius && dist > 0.001f) {
            float factor = 1.0f - (dist / radius);
            to_part.normalize();
            Fvector impulse = to_part;
            impulse.mul(max_impulse * factor);
            GetPhysicsCore()->ApplyRagdollLinearImpulse(m_ragdoll_handle, i, impulse);
            
            // Flinch reaction
            if (i < m_part_reactions.size()) {
                auto& rx = m_part_reactions[i];
                rx.active = true;
                rx.flinch_factor = factor;
                rx.elapsed_time = 0.0f;
                rx.recovery_duration = 0.4f;
            }
        }
    }
}

void CActiveRagdollController::Update(float dt) {
    if (m_state == ERagdollState::Inactive || m_ragdoll_handle == INVALID_RAGDOLL_HANDLE) return;
    
    // 1. KnockedDown: free falling, check when body rests on the ground
    if (m_state == ERagdollState::KnockedDown) {
        m_knockdown_timer += dt;
        if (m_knockdown_timer >= m_min_knockdown_duration) {
            float total_energy = GetPhysicsCore()->GetRagdollTotalEnergy(m_ragdoll_handle);
            if (total_energy < 1.0f || m_knockdown_timer >= m_max_knockdown_duration) {
                // Determine orientation (Face Up vs Face Down)
                if (!m_simulated_rotations.empty()) {
                    u32 spine_part = (m_simulated_rotations.size() > 1) ? 1 : 0;
                    Fmatrix torso_mat;
                    torso_mat.rotation(m_simulated_rotations[spine_part]);
                    m_facing_up = (torso_mat.k.y > 0.0f || torso_mat.j.y > 0.0f);
                }
                
                m_state = ERagdollState::KnockdownResting;
                m_ramp_up_timer = 0.0f;
            }
        }
    }
    
    // 2. KnockdownResting: brief rest before initiating get-up
    else if (m_state == ERagdollState::KnockdownResting) {
        m_ramp_up_timer += dt;
        if (m_ramp_up_timer >= 0.25f) {
            m_state = ERagdollState::MotorRampingUp;
            m_ramp_up_timer = 0.0f;
            
            // Turn on motors with low initial stiffness
            GetPhysicsCore()->SetRagdollMotorState(m_ragdoll_handle, true);
            GetPhysicsCore()->SetRagdollMotorStiffness(m_ragdoll_handle, 10.0f);
            GetPhysicsCore()->SetRagdollMotorDamping(m_ragdoll_handle, 5.0f);
            
            // Invoke callback to reposition capsule and launch animation
            if (m_get_up_callback) {
                m_get_up_callback();
            }
        }
    }
    
    // 3. MotorRampingUp: smoothly ramp up stiffness to pull skeleton to animation pose
    else if (m_state == ERagdollState::MotorRampingUp) {
        m_ramp_up_timer += dt;
        float t = std::min(m_ramp_up_timer / m_ramp_up_duration, 1.0f);
        
        float ease = t * t * (3.0f - 2.0f * t);
        float cur_stiffness = m_motor_stiffness * ease;
        float cur_damping = m_motor_damping * ease;
        
        GetPhysicsCore()->SetRagdollMotorStiffness(m_ragdoll_handle, cur_stiffness);
        GetPhysicsCore()->SetRagdollMotorDamping(m_ragdoll_handle, cur_damping);
        
        if (t >= 1.0f) {
            // Restore kinematic bodies
            GetPhysicsCore()->SetRagdollAllPartsKinematic(m_ragdoll_handle, true);
            m_state = ERagdollState::GettingUp;
            m_get_up_timer = 0.0f;
        }
    }
    
    // 4. GettingUp: waiting for animation to finish
    else if (m_state == ERagdollState::GettingUp) {
        m_get_up_timer += dt;
        if (m_get_up_timer >= m_get_up_duration) {
            m_state = ERagdollState::Active;
        }
    }
    
    // 5. Active: Hit flinch recovery
    else if (m_state == ERagdollState::Active) {
        for (u32 i = 0; i < m_part_reactions.size(); ++i) {
            auto& rx = m_part_reactions[i];
            if (!rx.active) continue;
            
            rx.elapsed_time += dt;
            float progress = rx.elapsed_time / rx.recovery_duration;
            if (progress >= 1.0f) {
                rx.active = false;
                rx.flinch_factor = 0.0f;
                GetPhysicsCore()->SetRagdollPartMotor(m_ragdoll_handle, i, m_motor_stiffness, m_motor_damping);
            } else {
                // Ease out recovery
                float factor = rx.flinch_factor * (1.0f - progress * progress);
                float cur_stiffness = m_motor_stiffness * (1.0f - 0.85f * factor);
                float cur_damping = m_motor_damping * (1.0f - 0.4f * factor);
                GetPhysicsCore()->SetRagdollPartMotor(m_ragdoll_handle, i, cur_stiffness, cur_damping);
            }
        }
    }
    
    // 6. Dying: smoothly decay motors to dead ragdoll
    else if (m_state == ERagdollState::Dying) {
        m_death_decay_timer += dt;
        float t = std::min(m_death_decay_timer / m_death_decay_duration, 1.0f);
        
        // Smooth step decay
        float ease = t * t * (3.0f - 2.0f * t);
        
        float current_stiffness = m_motor_stiffness * (1.0f - ease);
        float current_damping = m_motor_damping * (1.0f - ease);
        
        GetPhysicsCore()->SetRagdollMotorStiffness(m_ragdoll_handle, current_stiffness);
        GetPhysicsCore()->SetRagdollMotorDamping(m_ragdoll_handle, current_damping);
        
        if (t >= 1.0f) {
            m_state = ERagdollState::Dead;
            GetPhysicsCore()->SetRagdollAllPartsKinematic(m_ragdoll_handle, false);
            GetPhysicsCore()->SetRagdollMotorState(m_ragdoll_handle, false);
        }
    }
}

void CActiveRagdollController::SyncToPhysics() {
    if (m_state == ERagdollState::Inactive || m_state == ERagdollState::Dead || m_state == ERagdollState::KnockedDown || m_state == ERagdollState::KnockdownResting) return;
    
    if (!m_holder || !m_kinematics || m_mapper.m_part_to_bone.empty()) return;

    // Sync root transform (part 0 in world space) when in active kinematic drive or get-up
    if (m_state == ERagdollState::Active || m_state == ERagdollState::GettingUp) {
        u16 root_bone = m_mapper.PartToBone(0);
        const CBoneInstance& root_bi = m_kinematics->LL_GetBoneInstance(root_bone);
        Fmatrix root_world;
        root_world.mul_43(m_holder->ObjectXFORM(), root_bi.mTransform);
        Fquaternion rot;
        rot.set(root_world);
        GetPhysicsCore()->SetRagdollRootTransform(m_ragdoll_handle, root_world.c, rot);
    }
    
    // Sync all target bone matrices in WORLD SPACE
    const Fmatrix& obj_xform = m_holder->ObjectXFORM();
    for (u32 i = 0; i < m_mapper.m_part_to_bone.size(); ++i) {
        u16 bone_id = m_mapper.PartToBone(i);
        const CBoneInstance& B = m_kinematics->LL_GetBoneInstance(bone_id);
        m_target_matrices[i].mul_43(obj_xform, B.mTransform);
    }
    
    GetPhysicsCore()->SetRagdollTargetPose(m_ragdoll_handle, m_target_matrices.data(), (u32)m_target_matrices.size());
}

void CActiveRagdollController::SyncFromPhysics() {
    if (m_state == ERagdollState::Inactive) return;
    
    GetPhysicsCore()->GetRagdollAllTransforms(m_ragdoll_handle, m_simulated_positions.data(), m_simulated_rotations.data(), (u32)m_simulated_positions.size());
}

void CActiveRagdollController::BonesCallback(CBoneInstance* B) {
    if (!B) return;
    ActiveRagdollCallbackData* cb_data = static_cast<ActiveRagdollCallbackData*>(B->callback_param());
    if (!cb_data || !cb_data->controller) return;
    
    CActiveRagdollController* controller = cb_data->controller;
    if (controller->m_state == ERagdollState::Inactive) return;
    
    u32 part_idx = cb_data->part_index;
    if (part_idx < controller->m_simulated_positions.size()) {
        // In Active state (alive & walking): only override if there is an active flinch reaction!
        if (controller->m_state == ERagdollState::Active) {
            if (part_idx < controller->m_part_reactions.size() && controller->m_part_reactions[part_idx].active) {
                Fvector pos = controller->m_simulated_positions[part_idx];
                Fquaternion rot = controller->m_simulated_rotations[part_idx];
                
                Fmatrix part_world;
                part_world.rotation(rot);
                part_world.c = pos;
                
                if (controller->m_holder) {
                    Fmatrix obj_xform_inv;
                    obj_xform_inv.invert(controller->m_holder->ObjectXFORM());
                    B->mTransform.mul_43(obj_xform_inv, part_world);
                } else {
                    B->mTransform = part_world;
                }
                B->set_callback_overwrite(TRUE);
            }
            return;
        }
        
        // In KnockedDown, Resting, Ramping, GettingUp, Dying, Dead states:
        // Ragdoll physics fully positions the visual skeleton.
        Fvector pos = controller->m_simulated_positions[part_idx];
        Fquaternion rot = controller->m_simulated_rotations[part_idx];
        
        Fmatrix part_world;
        part_world.rotation(rot);
        part_world.c = pos;
        
        if (controller->m_holder) {
            Fmatrix obj_xform_inv;
            obj_xform_inv.invert(controller->m_holder->ObjectXFORM());
            B->mTransform.mul_43(obj_xform_inv, part_world);
        } else {
            B->mTransform = part_world;
        }
        
        B->set_callback_overwrite(TRUE);
    }
}

// ===========================================================================
// CActiveRagdollManager
// ===========================================================================

static CActiveRagdollManager g_active_ragdoll_manager;

CActiveRagdollManager& CActiveRagdollManager::GetInstance() {
    return g_active_ragdoll_manager;
}

CActiveRagdollManager::~CActiveRagdollManager() {
    for (auto c : m_controllers) {
        xr_delete(c);
    }
    m_controllers.clear();
}

CActiveRagdollController* CActiveRagdollManager::RegisterRagdoll(IKinematics* kinematics, IPhysicsShellHolder* holder) {
    CActiveRagdollController* controller = xr_new<CActiveRagdollController>();
    controller->Initialize(kinematics, holder);
    m_controllers.push_back(controller);
    return controller;
}

void CActiveRagdollManager::UnregisterRagdoll(CActiveRagdollController* controller) {
    auto it = std::find(m_controllers.begin(), m_controllers.end(), controller);
    if (it != m_controllers.end()) {
        xr_delete(*it);
        m_controllers.erase(it);
    }
}

void CActiveRagdollManager::UpdateAll(float dt) {
    // Basic LOD check could be added here based on m_holder->ObjectXFORM().c distance to camera
    for (auto c : m_controllers) {
        c->Update(dt);
        c->SyncToPhysics();
        c->SyncFromPhysics();
    }
}
