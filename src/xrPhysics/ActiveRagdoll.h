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

enum class ERagdollState {
    Inactive,
    Active,
    Dying,
    Dead
};

class CActiveRagdollController;

struct ActiveRagdollCallbackData {
    CActiveRagdollController* controller;
    u16 part_index;
    u16 bone_id;
};

class XRPHYSICS_API CActiveRagdollController {
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
    
    xr_vector<Fquaternion> m_target_rotations;
    xr_vector<Fvector> m_simulated_positions;
    xr_vector<Fquaternion> m_simulated_rotations;
    
    xr_vector<ActiveRagdollCallbackData> m_cb_data;
    
public:
    CActiveRagdollController() = default;
    ~CActiveRagdollController();
    
    void Initialize(IKinematics* kinematics, IPhysicsShellHolder* holder);
    void Deactivate();
    
    void OnDeath();
    
    void Update(float dt);
    void SyncToPhysics();
    void SyncFromPhysics();
    
    static void BonesCallback(CBoneInstance* B);
    
    RagdollHandle GetHandle() const { return m_ragdoll_handle; }
    ERagdollState GetState() const { return m_state; }
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
