#pragma once
#include "xrPhysicsCore/IPhysicsCore.h"
#include "JoltLayers.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

#include <unordered_map>

class JoltPhysicsCore : public IPhysicsCore {
private:
    JPH::PhysicsSystem* m_physics_system = nullptr;
    JPH::TempAllocatorImpl* m_temp_allocator = nullptr;
    JPH::JobSystemThreadPool* m_job_system = nullptr;

    BPLayerInterfaceImpl m_broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl m_object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl m_object_vs_object_layer_filter;

    std::unordered_map<JointHandle, JPH::Ref<JPH::Constraint>> m_constraints;
    JointHandle m_next_joint_handle = 1;

    std::unordered_map<CharacterVirtualHandle, JPH::Ref<JPH::CharacterVirtual>> m_characters;
    CharacterVirtualHandle m_next_character_handle = 1;

public:
    JoltPhysicsCore() = default;
    virtual ~JoltPhysicsCore() override;

    void Initialize() override;
    void Step(float delta_time) override;
    void Destroy() override;

    PhysicsShapeHandle BuildCDBModel(const Fvector* verts, u32 v_cnt, const void* tris, u32 t_cnt) override;
    void DestroyCDBModel(PhysicsShapeHandle handle) override;
    
    void RaycastCDBModel(PhysicsShapeHandle handle, const Fvector& start, const Fvector& dir, float range, std::vector<CDBRaycastHit>& out_hits) override;
    void RaycastCDBModel(PhysicsShapeHandle handle, 
                        const Fvector& start, const Fvector& dir, float range, 
                        CDBRayMode mode, bool cull_backfaces, 
                        std::vector<CDBRaycastHit>& out_hits) override;

    bool BoxQueryCDB(PhysicsShapeHandle handle, 
                        const Fvector& box_center, 
                        const Fvector& box_z_axis, 
                        const Fvector& box_y_axis, 
                        const Fvector& box_sizes, 
                        std::vector<u32>& out_tri_indices) override;
    void BoxQueryCDB(PhysicsShapeHandle handle, 
                        const Fvector& center, const Fvector& extents, 
                        CDBRayMode mode, bool cull_backfaces, 
                        std::vector<u32>& out_tri_indices) override;

    long GetShapeMemoryUsage(PhysicsShapeHandle handle) override;
    void GetCDBModelBounds(PhysicsShapeHandle handle, Fvector& out_center, Fvector& out_extents) const override;

    BodyHandle CreateBox(const Fvector& half_extents, const Fvector& position, float mass) override;
    BodyHandle CreateSphere(float radius, const Fvector& position, float mass) override;
    BodyHandle CreateCylinder(float radius, float half_height, const Fvector& position, float mass) override;

    void GetBoxExtents(BodyHandle body, Fvector& out_extents) const override;
    void SetBoxExtents(BodyHandle body, const Fvector& extents) override;

    void DestroyBody(BodyHandle body) override;
    void GetBodyTransform(BodyHandle body, Fmatrix& out_matrix) const override;
    void SetBodyTransform(BodyHandle body, const Fmatrix& matrix) override;
    void GetBodyAABB(BodyHandle body, Fvector& center, Fvector& half_extents) const override;
    
    // --- Сочленения (Joints / Constraints) ---
    // Замени старый CreateJoint на этот:
    JointHandle CreateJoint(int type, BodyHandle b1, BodyHandle b2, 
                                    const Fvector& anchor, 
                                    const Fvector& axis0, const Fvector& axis1, const Fvector& axis2, 
                                    const Fvector& limits_lo, const Fvector& limits_hi) override;
    void DestroyJoint(JointHandle joint) override;

    void SetJointLimits(JointHandle joint, int axis_num, float lo, float hi) override;
    void SetJointMotor(JointHandle joint, int axis_num, float force, float velocity) override;
    void SetJointSpringDamping(JointHandle joint, int axis_num, float erp, float cfm) override;
    void SetJointAxisDir(JointHandle joint, int axis_num, const Fvector& axis) override;
    void SetJointFudgeFactor(JointHandle joint, float factor) override;
    void SetJointFeedback(JointHandle joint, SPhysicsJointFeedback* feedback) override;

    void GetJointAxisDir(JointHandle joint, int axis_num, Fvector& axis) const override;
    void GetJointAnchor(JointHandle joint, Fvector& anchor) const override;
    float GetJointAxisAngle(JointHandle joint, int axis_num) const override;
    float GetJointAxisAngleRate(JointHandle joint, int axis_num) const override;

    // -----------------------------------------

    void GetBodyLinearVelocity(BodyHandle body, Fvector& out_vel) const override;
    void SetBodyLinearVelocity(BodyHandle body, const Fvector& vel) override;
    
    void GetBodyAngularVelocity(BodyHandle body, Fvector& out_vel) const override;
    void SetBodyAngularVelocity(BodyHandle body, const Fvector& vel) override;

    void SetBodyGravityFactor(BodyHandle body, float factor) override;

    void ApplyLinearImpulse(BodyHandle body, const Fvector& impulse) override;
    void ApplyPointImpulse(BodyHandle body, const Fvector& impulse, const Fvector& point) override;
    void ApplyForce(BodyHandle body, const Fvector& force) override;
    void ApplyTorque(BodyHandle body, const Fvector& torque) override;

    bool IsBodyActive(BodyHandle body) const override;
    void ActivateBody(BodyHandle body) override;
    void DeactivateBody(BodyHandle body) override;

    float GetBodyMass(BodyHandle body) const override;

    void GetBodyPointVelocity(BodyHandle body, const Fvector& point, Fvector& velocity) const override;

    void SetBodyIgnoreStatic(BodyHandle body) override;

    void GetBodyPosition(BodyHandle body, Fvector& position) const override;
    void SetBodyPosition(BodyHandle body, const Fvector& position) override;

    void GetBodyForce(BodyHandle body, Fvector& force) const override;
    void SetBodyForce(BodyHandle body, const Fvector& force) override;

    float GetBodyGravityFactor(BodyHandle body) const override;

    void* GetBodyUserData(BodyHandle body) const override;
    void SetBodyUserData(BodyHandle body, void* data) override;

    void SetBodyFixedRotation(BodyHandle body_handle) override;
    BodyHandle CreateStaticBody(PhysicsShapeHandle shape_handle, const Fvector& position) override;
    virtual PhysicsShapeHandle CreateCompoundShape(PhysicsShapeHandle* shapes, const Fmatrix* transforms, size_t count) override;
    virtual BodyHandle CreateBodyFromShape(PhysicsShapeHandle shape, const Fvector& initial_pos, float mass) override;

    PhysicsShapeHandle CreateCapsuleShape(float radius, float half_height) override;
    CharacterVirtualHandle CreateCharacterVirtual(PhysicsShapeHandle shape, const Fvector& initial_pos) override;
    void DestroyCharacterVirtual(CharacterVirtualHandle handle) override;
    virtual void SetCharacterVirtualVelocity(CharacterVirtualHandle handle, const Fvector& velocity) override;
    virtual void GetCharacterVirtualVelocity(CharacterVirtualHandle handle, Fvector& velocity) const override;
    virtual void SetCharacterVirtualUserData(CharacterVirtualHandle handle, void* data) override;
    virtual void GetCharacterVirtualGroundState(CharacterVirtualHandle handle, SJoltCharacterGroundState& out_state) const override;
    virtual void GetCharacterVirtualPosition(CharacterVirtualHandle handle, Fvector& position) const override;
    void SetCharacterVirtualPosition(CharacterVirtualHandle handle, const Fvector& position) override;
    void SetCharacterVirtualShape(CharacterVirtualHandle handle, PhysicsShapeHandle shape) override;
    void ActivateCharacterVirtual(CharacterVirtualHandle handle) override;
    void DeactivateCharacterVirtual(CharacterVirtualHandle handle) override;
    bool IsCharacterVirtualOnGround(CharacterVirtualHandle handle) const override;
    void UpdateCharacterVirtual(CharacterVirtualHandle handle, float delta_time, const Fvector& gravity) override;
};
