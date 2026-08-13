#pragma once

#include "Common/Common.hpp"
#include "xrCore/xrCore.h"
#include <vector>

#ifdef _MSC_VER
#   define PHYSICS_CORE_API __declspec(dllexport)
#else
#   define PHYSICS_CORE_API __attribute__((visibility("default")))
#endif

typedef uint32_t BodyHandle;
constexpr BodyHandle INVALID_BODY_HANDLE = 0xFFFFFFFF;

typedef void* PhysicsShapeHandle;

struct CDBRaycastHit {
    float range;
    u32 tri_index;
};

enum class CDBRayMode {
    All,
    Nearest,
    First
};

class IPhysicsCore {
public:
    virtual ~IPhysicsCore() = default;

    virtual void Initialize() = 0;
    virtual void Step(float delta_time) = 0;
    virtual void Destroy() = 0;

    virtual PhysicsShapeHandle BuildCDBModel(const Fvector* verts, u32 v_cnt, const void* tris, u32 t_cnt) = 0;
    virtual void DestroyCDBModel(PhysicsShapeHandle handle) = 0;
    virtual void RaycastCDBModel(PhysicsShapeHandle handle, const Fvector& start, const Fvector& dir, float range, std::vector<CDBRaycastHit>& out_hits) = 0;
    virtual void RaycastCDBModel(PhysicsShapeHandle handle, 
                                 const Fvector& start, const Fvector& dir, float range, 
                                 CDBRayMode mode, bool cull_backfaces, 
                                 std::vector<CDBRaycastHit>& out_hits) = 0;

    virtual bool BoxQueryCDB(PhysicsShapeHandle handle, 
                         const Fvector& box_center, 
                         const Fvector& box_z_axis, 
                         const Fvector& box_y_axis, 
                         const Fvector& box_sizes, 
                         std::vector<u32>& out_tri_indices) = 0;

    virtual void BoxQueryCDB(PhysicsShapeHandle handle, 
                             const Fvector& center, const Fvector& extents, 
                             CDBRayMode mode, bool cull_backfaces, 
                             std::vector<u32>& out_tri_indices) = 0;

    virtual long GetShapeMemoryUsage(PhysicsShapeHandle handle) = 0;

    virtual BodyHandle CreateBox(const Fvector& half_extents, const Fvector& position, float mass) = 0;
    virtual void DestroyBody(BodyHandle body) = 0;
    
    virtual void GetBodyTransform(BodyHandle body, Fmatrix& out_matrix) const = 0;
    virtual void SetBodyTransform(BodyHandle body, const Fmatrix& matrix) = 0;
    virtual void GetBodyAABB(BodyHandle body, Fvector& center, Fvector& half_extents) const = 0;

    virtual void GetCDBModelBounds(PhysicsShapeHandle handle, Fvector& out_center, Fvector& out_extents) const = 0;
};

extern "C" PHYSICS_CORE_API IPhysicsCore* GetPhysicsCore();
