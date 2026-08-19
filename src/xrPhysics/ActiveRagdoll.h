#pragma once

#include "xrPhysicsCore/IPhysicsCore.h"
#include "xrCore/Animation/Bone.hpp"
#include "Include/xrRender/Kinematics.h"
#include "xrPhysics.h"

class IPhysicsShellHolder;

class XRPHYSICS_API CActiveRagdollSkeletonMapper {
public:
    xr_vector<u16> m_bone_to_part;
    xr_vector<u16> m_part_to_bone;
    xr_vector<Fquaternion> m_bind_pose_inv;
    
    void Build(IKinematics* kinematics);
    u16 BoneToPart(u16 bone_id) const;
    u16 PartToBone(u16 part_index) const;
};

class XRPHYSICS_API CActiveRagdollSettingsBuilder {
public:
    static SRagdollSettings BuildSettings(IKinematics* kinematics, const CActiveRagdollSkeletonMapper& mapper);
};

#include "xrCore/fastdelegate.h"
#include "xrCore/Animation/Bone.hpp"

enum class ERagdollState {
    Inactive,
    Active,
    KnockedDown,        // Full dynamic ragdoll simulation during fall
    KnockdownResting,   // Settled on ground, preparing to stand up
    MotorRampingUp,     // Motor stiffness ramping up to blend to get-up pose
    GettingUp,          // Playing get-up animation, root kinematic restored
    Dying,
    Dead
};

class CActiveRagdollController;

struct ActiveRagdollCallbackData {
    CActiveRagdollController* controller = nullptr;
    u16 part_index = 0;
    u16 bone_id = 0;
    BoneCallback previous_callback = nullptr;
    void* previous_param = nullptr;
};

struct SPartHitReaction {
    Fvector impulse_dir = {0.0f, 0.0f, 0.0f};
    float flinch_factor = 0.0f;
    float recovery_duration = 0.35f;
    float elapsed_time = 0.0f;
    bool active = false;
};

class XRPHYSICS_API CActiveRagdollController {
public:
    using GetUpCallback = fastdelegate::FastDelegate0<>;

private:
    RagdollHandle m_ragdoll_handle = INVALID_RAGDOLL_HANDLE;
    CActiveRagdollSkeletonMapper m_mapper;
    IKinematics* m_kinematics = nullptr;
    IPhysicsShellHolder* m_holder = nullptr;
    
    ERagdollState m_state = ERagdollState::Inactive;
    
    float m_motor_stiffness = 1000.f;
    float m_motor_damping = 100.f;
    
    float m_death_decay_timer = 0.f;
    float m_death_decay_duration = 2.0f; // 2 seconds to die
    
    float m_visual_blend_factor = 0.f;
    float m_visual_blend_duration = 0.8f; // Shorter than decay so visual is fully physical when motors die
    
    float m_knockdown_timer = 0.0f;
    float m_min_knockdown_duration = 0.8f;
    float m_max_knockdown_duration = 3.5f;
    
    float m_ramp_up_timer = 0.0f;
    float m_ramp_up_duration = 0.5f;
    
    float m_get_up_timer = 0.0f;
    float m_get_up_duration = 1.5f;
    
    bool m_facing_up = true;
    bool m_is_wounded = false;
    
    GetUpCallback m_get_up_callback;
    GetUpCallback m_start_get_up_callback;
    
    xr_vector<Fmatrix> m_target_matrices;
    xr_vector<Fmatrix> m_simulated_matrices;
    xr_vector<Fmatrix> m_anim_matrices;
    xr_vector<SPartHitReaction> m_part_reactions;
    
    xr_vector<ActiveRagdollCallbackData> m_cb_data;
    
public:
    CActiveRagdollController() = default;
    ~CActiveRagdollController();
    
    void Initialize(IKinematics* kinematics, IPhysicsShellHolder* holder);
    void Deactivate();
    
    void OnDeath();
    void KnockDown(u16 bone_id, const Fvector& dir, float impulse, const Fvector& hit_pos);
    void ApplyHit(u16 bone_id, const Fvector& dir, float impulse, const Fvector& hit_pos);
    void ApplyRadialImpulse(const Fvector& center, float radius, float max_impulse);
    void SetMotorDefaults(float stiffness, float damping);
    
    void SetGetUpCallback(GetUpCallback cb) { m_get_up_callback = cb; }
    void SetStartGetUpCallback(GetUpCallback cb) { m_start_get_up_callback = cb; }
    void SetGetUpDuration(float duration) { m_get_up_duration = duration; }
    void SetTargetWounded(bool wounded) { m_is_wounded = wounded; }
    bool IsTargetWounded() const { return m_is_wounded; }
    void OnGetUpFinished() { m_state = ERagdollState::Active; }
    
    bool IsFacingUp() const { return m_facing_up; }
    const Fvector& GetSimulatedPosition(u32 part_idx) const {
        VERIFY(part_idx < m_simulated_matrices.size());
        return m_simulated_matrices[part_idx].c;
    }
    
    void Update(float dt);
    void SyncToPhysics();
    void SyncFromPhysics();
    
    static void BonesCallback(CBoneInstance* B);
    
    RagdollHandle GetHandle() const { return m_ragdoll_handle; }
    ERagdollState GetState() const { return m_state; }
    const CActiveRagdollSkeletonMapper& GetMapper() const { return m_mapper; }
    u32 GetPartCount() const { return (u32)m_mapper.m_part_to_bone.size(); }
    IKinematics* GetKinematics() const { return m_kinematics; }
    IPhysicsShellHolder* GetHolder() const { return m_holder; }
};

class XRPHYSICS_API CActiveRagdollManager {
private:
    xr_vector<CActiveRagdollController*> m_controllers;
    
public:
    ~CActiveRagdollManager();
    
    CActiveRagdollController* RegisterRagdoll(IKinematics* kinematics, IPhysicsShellHolder* holder);
    void UnregisterRagdoll(CActiveRagdollController* controller);
    
    void UpdateAll(float dt);
    
    static CActiveRagdollManager& GetInstance();
};
