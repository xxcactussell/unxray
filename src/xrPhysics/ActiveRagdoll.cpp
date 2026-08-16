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
    
    m_bone_to_part.resize(kinematics->LL_BoneCount(), u16(-1));
    m_bind_pose_inv.resize(kinematics->LL_BoneCount());
    
    for (u16 bone_id = 0; bone_id < kinematics->LL_BoneCount(); ++bone_id) {
        const IBoneData& bone_data = kinematics->GetBoneData(bone_id);
        
        // Calculate bind pose inverse for rotation
        Fmatrix bind_transform = bone_data.get_bind_transform();
        Fquaternion bind_rot;
        bind_rot.set(bind_transform);
        bind_rot.inverse();
        m_bind_pose_inv[bone_id] = bind_rot;
        
        if (shape_is_physic(bone_data.get_shape())) {
            u16 part_index = u16(m_part_to_bone.size());
            m_bone_to_part[bone_id] = part_index;
            m_part_to_bone.push_back(bone_id);
        }
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
    settings.default_motor.stiffness = 1000.f; // Hardcoded default
    settings.default_motor.damping = 100.f;    // Hardcoded default
    
    u32 num_parts = mapper.m_part_to_bone.size();
    settings.parts.resize(num_parts);
    
    for (u32 part_idx = 0; part_idx < num_parts; ++part_idx) {
        u16 bone_id = mapper.PartToBone(part_idx);
        const IBoneData& bone_data = kinematics->GetBoneData(bone_id);
        
        SRagdollPartDesc& part_desc = settings.parts[part_idx];
        part_desc.bone_id = bone_id;
        
        // Find parent part index
        part_desc.parent_index = -1;
        u16 parent_bone_id = bone_data.GetParentID();
        while (parent_bone_id != u16(-1)) {
            u16 parent_part = mapper.BoneToPart(parent_bone_id);
            if (parent_part != u16(-1)) {
                part_desc.parent_index = parent_part;
                break;
            }
            parent_bone_id = kinematics->GetBoneData(parent_bone_id).GetParentID();
        }
        
        // Get shape and mass
        part_desc.shape = nullptr; 
        if (shape_is_physic(bone_data.get_shape())) {
            const SBoneShape& shape = bone_data.get_shape();
            if (shape.type == SBoneShape::stBox) {
                part_desc.shape = GetPhysicsCore()->CreateBoxShape(Fvector().set(shape.box.m_halfsize));
            } else if (shape.type == SBoneShape::stSphere) {
                part_desc.shape = GetPhysicsCore()->CreateSphereShape(shape.sphere.R);
            } else if (shape.type == SBoneShape::stCylinder) {
                part_desc.shape = GetPhysicsCore()->CreateCapsuleShape(shape.cylinder.m_height / 2.0f, shape.cylinder.m_radius);
            }
        }
        
        part_desc.mass = bone_data.get_mass();
        
        // Initial transform
        Fmatrix bind_transform = bone_data.get_bind_transform();
        part_desc.position = bind_transform.c;
        part_desc.rotation.set(bind_transform);
        
        // If not root, create constraint with parent
        if (part_desc.parent_index != -1) {
            const SJointIKData& ik_data = bone_data.get_IK_data();
            
            SRagdollConstraintDesc c_desc;
            c_desc.child_index = part_idx;
            
            // Simplified joint mapping (assumes SwingTwist capabilities)
            c_desc.twist_axis = Fvector().set(1.f, 0.f, 0.f);
            c_desc.plane_axis = Fvector().set(0.f, 1.f, 0.f);
            
            if (ik_data.type == jtRigid || ik_data.type == jtNone) {
                // Fixed joint
                c_desc.swing_limit_y = 0.f;
                c_desc.swing_limit_z = 0.f;
                c_desc.twist_limit_min = 0.f;
                c_desc.twist_limit_max = 0.f;
            } else {
                // Generic joint
                c_desc.swing_limit_y = M_PI / 4.f; // Hardcoded default
                c_desc.swing_limit_z = M_PI / 4.f; // Hardcoded default
                c_desc.twist_limit_min = -M_PI / 4.f;
                c_desc.twist_limit_max = M_PI / 4.f;
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
        GetPhysicsCore()->SetRagdollRootKinematic(m_ragdoll_handle, true);
        GetPhysicsCore()->AddRagdollToWorld(m_ragdoll_handle, true);
        m_state = ERagdollState::Active;
        
        m_target_rotations.resize(m_mapper.m_part_to_bone.size());
        m_simulated_positions.resize(m_mapper.m_part_to_bone.size());
        m_simulated_rotations.resize(m_mapper.m_part_to_bone.size());
        
        m_cb_data.resize(m_mapper.m_part_to_bone.size());
        for (u32 i = 0; i < m_mapper.m_part_to_bone.size(); ++i) {
            u16 bone_id = m_mapper.PartToBone(i);
            m_cb_data[i].controller = this;
            m_cb_data[i].part_index = i;
            m_cb_data[i].bone_id = bone_id;
            
            CBoneInstance& B = m_kinematics->LL_GetBoneInstance(bone_id);
            B.set_callback(bctPhysics, BonesCallback, &m_cb_data[i]);
        }
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
}

void CActiveRagdollController::OnDeath() {
    if (m_state == ERagdollState::Active) {
        m_state = ERagdollState::Dying;
        m_death_decay_timer = 0.f;
    }
}

void CActiveRagdollController::Update(float dt) {
    if (m_state == ERagdollState::Inactive || m_ragdoll_handle == INVALID_RAGDOLL_HANDLE) return;
    
    // Handle Dying state
    if (m_state == ERagdollState::Dying) {
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
            GetPhysicsCore()->SetRagdollRootKinematic(m_ragdoll_handle, false);
        }
    }
}

void CActiveRagdollController::SyncToPhysics() {
    if (m_state == ERagdollState::Inactive || m_state == ERagdollState::Dead) return;
    
    // Sync root transform
    if (m_holder) {
        Fmatrix xform = m_holder->ObjectXFORM();
        Fquaternion rot;
        rot.set(xform);
        GetPhysicsCore()->SetRagdollRootTransform(m_ragdoll_handle, xform.c, rot);
    }
    
    // Sync target pose
    for (u32 i = 0; i < m_mapper.m_part_to_bone.size(); ++i) {
        u16 bone_id = m_mapper.PartToBone(i);
        CBoneInstance& B = m_kinematics->LL_GetBoneInstance(bone_id);
        m_target_rotations[i].set(B.mTransform);
    }
    
    GetPhysicsCore()->SetRagdollTargetPose(m_ragdoll_handle, m_target_rotations.data(), m_target_rotations.size());
}

void CActiveRagdollController::SyncFromPhysics() {
    if (m_state == ERagdollState::Inactive) return;
    
    GetPhysicsCore()->GetRagdollAllTransforms(m_ragdoll_handle, m_simulated_positions.data(), m_simulated_rotations.data(), m_simulated_positions.size());
}

void CActiveRagdollController::BonesCallback(CBoneInstance* B) {
    if (!B) return;
    ActiveRagdollCallbackData* cb_data = static_cast<ActiveRagdollCallbackData*>(B->callback_param());
    if (!cb_data || !cb_data->controller) return;
    
    CActiveRagdollController* controller = cb_data->controller;
    if (controller->m_state == ERagdollState::Inactive) return;
    
    u32 part_idx = cb_data->part_index;
    if (part_idx < controller->m_simulated_positions.size()) {
        // Read simulated transform
        Fvector pos = controller->m_simulated_positions[part_idx];
        Fquaternion rot = controller->m_simulated_rotations[part_idx];
        
        // Jolt transform is in world space. We need to convert it to parent-local space.
        // Or if Jolt transform is already relative, we wouldn't need this, but Jolt returns world space (or physics-system space).
        Fmatrix part_world;
        part_world.rotation(rot);
        part_world.c = pos;
        
        u16 parent_id = controller->m_kinematics->GetBoneData(cb_data->bone_id).GetParentID();
        if (parent_id != u16(-1)) {
            // Get parent world transform
            // Assuming parent's callback has already run (X-Ray processes callbacks hierarchically root-to-leaf)
            const CBoneInstance& parent_B = controller->m_kinematics->LL_GetBoneInstance(parent_id);
            Fmatrix parent_world = parent_B.mTransform;
            
            // For root bone, we might need the object's transform
            if (controller->m_holder && parent_id == controller->m_kinematics->LL_GetBoneRoot()) {
                parent_world.mulA_43(controller->m_holder->ObjectXFORM());
            } else if (controller->m_holder) {
                // If it's a child bone, mTransform is usually relative to root, but let's check
                // X-Ray mTransform is actually relative to the Object's root (ObjectXFORM) during calculate, 
                // so we need to convert part_world to Object Space.
            }
        }
        
        // Actually, X-Ray's B->mTransform is in Object Space, not Parent Space!
        // Let's verify: In CPHElement::BonesCallBack, B->mTransform is set to the body's Object Space transform.
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
