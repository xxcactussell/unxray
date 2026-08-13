#pragma once
#include "IPhysicsCore.h"
#include "JoltLayers.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>

class JoltPhysicsCore : public IPhysicsCore {
private:
    JPH::PhysicsSystem* m_physics_system = nullptr;
    JPH::TempAllocatorImpl* m_temp_allocator = nullptr;
    JPH::JobSystemThreadPool* m_job_system = nullptr;

    BPLayerInterfaceImpl m_broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl m_object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl m_object_vs_object_layer_filter;

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

    BodyHandle CreateBox(const Fvector& half_extents, const Fvector& position, float mass) override;
    void DestroyBody(BodyHandle body) override;
    void GetBodyTransform(BodyHandle body, Fmatrix& out_matrix) const override;
    void SetBodyTransform(BodyHandle body, const Fmatrix& matrix) override;
    void GetBodyAABB(BodyHandle body, Fvector& center, Fvector& half_extents) const override;
    
    void GetCDBModelBounds(PhysicsShapeHandle handle, Fvector& out_center, Fvector& out_extents) const override;
};
