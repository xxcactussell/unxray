#include "stdafx.h"
#include "JoltPhysicsCore.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#ifdef _MSC_VER
#   define PHYSICS_CORE_API __declspec(dllexport)
#else
#   define PHYSICS_CORE_API __attribute__((visibility("default")))
#endif


JoltPhysicsCore::~JoltPhysicsCore() 
{
    Destroy();
}

void JoltPhysicsCore::Initialize() 
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
    m_job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

    m_physics_system = new JPH::PhysicsSystem();
    const uint32_t cMaxBodies = 10240;
    const uint32_t cNumBodyMutexes = 0; 
    const uint32_t cMaxBodyPairs = 10240;
    const uint32_t cMaxContactConstraints = 10240;

    // Убрали разыменование (*), так как фильтры теперь являются значениями, а не указателями
    m_physics_system->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        m_broad_phase_layer_interface,
        m_object_vs_broadphase_layer_filter,
        m_object_vs_object_layer_filter);
}

void JoltPhysicsCore::Step(float delta_time) 
{
    int collision_steps = 1; 
    m_physics_system->Update(delta_time, collision_steps, m_temp_allocator, m_job_system);
}

void JoltPhysicsCore::Destroy() 
{
    if (m_physics_system) {
        delete m_physics_system;
        m_physics_system = nullptr;
    }
    if (m_job_system) {
        delete m_job_system;
        m_job_system = nullptr;
    }
    if (m_temp_allocator) {
        delete m_temp_allocator;
        m_temp_allocator = nullptr;
    }
    if (JPH::Factory::sInstance) {
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}


struct CDB_TRI_Mock {
    u32 verts[3];
    u32 dummy;
};

PhysicsShapeHandle JoltPhysicsCore::BuildCDBModel(const Fvector* verts, u32 v_cnt, const void* tris_raw, u32 t_cnt) 
{
    const CDB_TRI_Mock* tris = static_cast<const CDB_TRI_Mock*>(tris_raw);

    JPH::VertexList jolt_vertices;
    jolt_vertices.reserve(v_cnt);
    for (u32 i = 0; i < v_cnt; ++i) {
        jolt_vertices.push_back(JPH::Float3(verts[i].x, verts[i].y, verts[i].z));
    }

    JPH::IndexedTriangleList jolt_triangles;
    jolt_triangles.reserve(t_cnt);
    for (u32 i = 0; i < t_cnt; ++i) {
        jolt_triangles.push_back(JPH::IndexedTriangle(tris[i].verts[0], tris[i].verts[1], tris[i].verts[2]));
    }

    JPH::MeshShapeSettings settings(jolt_vertices, jolt_triangles);
    JPH::ShapeSettings::ShapeResult result = settings.Create();

    if (result.HasError()) {
        return nullptr;
    }

    JPH::Shape* shape = result.Get().GetPtr();
    shape->AddRef(); 
    return static_cast<PhysicsShapeHandle>(shape);
}

void JoltPhysicsCore::DestroyCDBModel(PhysicsShapeHandle handle) 
{
    if (handle) {
        JPH::Shape* shape = static_cast<JPH::Shape*>(handle);
        shape->Release(); 
    }
}

long JoltPhysicsCore::GetShapeMemoryUsage(PhysicsShapeHandle handle)
{
    if(handle) {
        JPH::Shape* shape = static_cast<JPH::Shape*>(handle);
        return sizeof(shape);
    } else {
        return 0;
    }
}

void JoltPhysicsCore::RaycastCDBModel(PhysicsShapeHandle handle, const Fvector& start, const Fvector& dir, float range, std::vector<CDBRaycastHit>& out_hits) 
{
    if (!handle) return;
    JPH::Shape* shape = static_cast<JPH::Shape*>(handle);

    JPH::Vec3 j_start(start.x, start.y, start.z);
    JPH::Vec3 j_dir(dir.x * range, dir.y * range, dir.z * range);
    JPH::RayCast ray(j_start, j_dir);

    JPH::RayCastSettings settings;
    JPH::SubShapeIDCreator id_creator;
    JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;

    shape->CastRay(ray, settings, id_creator, collector);

    for (const JPH::RayCastResult& hit : collector.mHits) {
        CDBRaycastHit cdb_hit;
        cdb_hit.range = hit.mFraction * range;
        cdb_hit.tri_index = hit.mSubShapeID2.GetValue(); 
        out_hits.push_back(cdb_hit);
    }
}

void JoltPhysicsCore::RaycastCDBModel(PhysicsShapeHandle handle, 
                                      const Fvector& start, const Fvector& dir, float range, 
                                      CDBRayMode mode, bool cull_backfaces, 
                                      std::vector<CDBRaycastHit>& out_hits) 
{
    if (!handle) return;
    JPH::Shape* shape = static_cast<JPH::Shape*>(handle);

    JPH::Vec3 j_start(start.x, start.y, start.z);
    JPH::Vec3 j_dir(dir.x * range, dir.y * range, dir.z * range);
    JPH::RayCast ray(j_start, j_dir);

    JPH::RayCastSettings settings;
    settings.mBackFaceModeTriangles = cull_backfaces ? JPH::EBackFaceMode::IgnoreBackFaces : JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mTreatConvexAsSolid = false;

    JPH::SubShapeIDCreator id_creator;

    if (mode == CDBRayMode::Nearest) 
    {
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        shape->CastRay(ray, settings, id_creator, collector);
        if (collector.HadHit()) {
            out_hits.push_back({ collector.mHit.mFraction * range, collector.mHit.mSubShapeID2.GetValue() });
        }
    } 
    else if (mode == CDBRayMode::First) 
    {
        JPH::AnyHitCollisionCollector<JPH::CastRayCollector> collector;
        shape->CastRay(ray, settings, id_creator, collector);
        if (collector.HadHit()) {
            out_hits.push_back({ collector.mHit.mFraction * range, collector.mHit.mSubShapeID2.GetValue() });
        }
    } 
    else 
    {
        JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
        shape->CastRay(ray, settings, id_creator, collector);
        for (const JPH::RayCastResult& hit : collector.mHits) {
            out_hits.push_back({ hit.mFraction * range, hit.mSubShapeID2.GetValue() });
        }
    }
}

BodyHandle JoltPhysicsCore::CreateBox(const Fvector& half_extents, const Fvector& position, float mass) {
    if (!m_physics_system) return INVALID_BODY_HANDLE;

    JPH::BoxShapeSettings shape_settings(JPH::Vec3(half_extents.x, half_extents.y, half_extents.z));
    JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
    if (shape_result.HasError()) return INVALID_BODY_HANDLE;

    JPH::BodyCreationSettings body_settings(
        shape_result.Get(), 
        JPH::Vec3(position.x, position.y, position.z), 
        JPH::Quat::sIdentity(), 
        JPH::EMotionType::Dynamic, 
        Layers::MOVING
    );

    body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    body_settings.mMassPropertiesOverride.mMass = mass;

    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    JPH::Body* body = body_interface.CreateBody(body_settings);
    
    if (!body) return INVALID_BODY_HANDLE;

    body_interface.AddBody(body->GetID(), JPH::EActivation::Activate);
    
    return body->GetID().GetIndexAndSequenceNumber();
}

void JoltPhysicsCore::DestroyBody(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    body_interface.RemoveBody(id);
    body_interface.DestroyBody(id);
}

void JoltPhysicsCore::GetBodyTransform(BodyHandle body_handle, Fmatrix& out_matrix) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    JPH::Mat44 transform = body_interface.GetWorldTransform(id);
    
    out_matrix.i.set(transform(0, 0), transform(0, 1), transform(0, 2));
    out_matrix.j.set(transform(1, 0), transform(1, 1), transform(1, 2));
    out_matrix.k.set(transform(2, 0), transform(2, 1), transform(2, 2));
    out_matrix.c.set(transform(3, 0), transform(3, 1), transform(3, 2));
}

void JoltPhysicsCore::SetBodyTransform(BodyHandle body_handle, const Fmatrix& matrix) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    JPH::Vec3 position(matrix.c.x, matrix.c.y, matrix.c.z);
    
    JPH::Mat44 transform(
        JPH::Vec4(matrix.i.x, matrix.i.y, matrix.i.z, 0),
        JPH::Vec4(matrix.j.x, matrix.j.y, matrix.j.z, 0),
        JPH::Vec4(matrix.k.x, matrix.k.y, matrix.k.z, 0),
        JPH::Vec4(0, 0, 0, 1)
    );
    JPH::Quat rotation = transform.GetQuaternion();

    body_interface.SetPositionAndRotation(id, position, rotation, JPH::EActivation::Activate);
}

void JoltPhysicsCore::GetBodyAABB(BodyHandle body_handle, Fvector& center, Fvector& half_extents) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    JPH::AABox bounds = body_interface.GetTransformedShape(id).GetWorldSpaceBounds();
    
    center.set(bounds.GetCenter().GetX(), bounds.GetCenter().GetY(), bounds.GetCenter().GetZ());
    half_extents.set(bounds.GetExtent().GetX(), bounds.GetExtent().GetY(), bounds.GetExtent().GetZ());
}

bool JoltPhysicsCore::BoxQueryCDB(PhysicsShapeHandle handle, 
                                  const Fvector& box_center, 
                                  const Fvector& box_z_axis, 
                                  const Fvector& box_y_axis, 
                                  const Fvector& box_sizes, 
                                  std::vector<u32>& out_tri_indices)
{
    if (!handle) return false;
    JPH::Shape* mesh_shape = static_cast<JPH::Shape*>(handle);

    JPH::Vec3 j_z(box_z_axis.x, box_z_axis.y, box_z_axis.z);
    JPH::Vec3 j_y(box_y_axis.x, box_y_axis.y, box_y_axis.z);

    j_z = j_z.Normalized();
    j_y = j_y.Normalized();
    JPH::Vec3 j_x = j_y.Cross(j_z).Normalized(); 

    // Создаем Jolt Box
    JPH::BoxShape box(JPH::Vec3(box_sizes.x * 0.5f, box_sizes.y * 0.5f, box_sizes.z * 0.5f));
    
    JPH::Mat44 rot(
        JPH::Vec4(j_x.GetX(), j_x.GetY(), j_x.GetZ(), 0.0f),
        JPH::Vec4(j_y.GetX(), j_y.GetY(), j_y.GetZ(), 0.0f),
        JPH::Vec4(j_z.GetX(), j_z.GetY(), j_z.GetZ(), 0.0f),
        JPH::Vec4(box_center.x, box_center.y, box_center.z, 1.0f)
    );

    class JoltOBBCollector : public JPH::CollideShapeCollector {
    public:
        std::vector<u32>& indices;
        bool has_hit = false;

        JoltOBBCollector(std::vector<u32>& out_ind) : indices(out_ind) {}

        virtual void AddHit(const JPH::CollideShapeResult &inResult) override {
            has_hit = true;
            indices.push_back(inResult.mSubShapeID2.GetValue());
        }
    };

    JoltOBBCollector collector(out_tri_indices);
    JPH::CollideShapeSettings settings;
    settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

    JPH::CollisionDispatch::sCollideShapeVsShape(
        &box, mesh_shape, 
        JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f), 
        rot, JPH::Mat44::sIdentity(), 
        JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), 
        settings, collector, JPH::ShapeFilter()
    );

    return collector.has_hit;
}

void JoltPhysicsCore::BoxQueryCDB(PhysicsShapeHandle handle, 
                                  const Fvector& center, const Fvector& extents, 
                                  CDBRayMode mode, bool cull_backfaces, 
                                  std::vector<u32>& out_tri_indices) 
{
    if (!handle) return;
    JPH::Shape* mesh_shape = static_cast<JPH::Shape*>(handle);

    JPH::BoxShape box(JPH::Vec3(extents.x, extents.y, extents.z));
    
    JPH::CollideShapeSettings settings;
    settings.mBackFaceMode = cull_backfaces ? JPH::EBackFaceMode::IgnoreBackFaces : JPH::EBackFaceMode::CollideWithBackFaces;

    JPH::Mat44 boxTransform = JPH::Mat44::sTranslation(JPH::Vec3(center.x, center.y, center.z));
    JPH::Mat44 meshTransform = JPH::Mat44::sIdentity();

    JPH::SubShapeIDCreator id_creator1, id_creator2;

    if (mode == CDBRayMode::First) 
    {
        JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
        JPH::CollisionDispatch::sCollideShapeVsShape(
            &box, mesh_shape, 
            JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f), 
            boxTransform, meshTransform, 
            id_creator1, id_creator2, 
            settings, collector, JPH::ShapeFilter()
        );
        if (collector.HadHit()) {
            out_tri_indices.push_back(collector.mHit.mSubShapeID2.GetValue());
        }
    } 
    else 
    {
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
        JPH::CollisionDispatch::sCollideShapeVsShape(
            &box, mesh_shape, 
            JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f), 
            boxTransform, meshTransform, 
            id_creator1, id_creator2, 
            settings, collector, JPH::ShapeFilter()
        );
        for (const JPH::CollideShapeResult& hit : collector.mHits) {
            out_tri_indices.push_back(hit.mSubShapeID2.GetValue());
        }
    }
}

void JoltPhysicsCore::GetCDBModelBounds(PhysicsShapeHandle handle, Fvector& out_center, Fvector& out_extents) const 
{
    if (!handle) return;
    
    const JPH::Shape* shape = static_cast<const JPH::Shape*>(handle);
    JPH::AABox bounds = shape->GetLocalBounds();
    
    out_center.set(bounds.GetCenter().GetX(), bounds.GetCenter().GetY(), bounds.GetCenter().GetZ());
    out_extents.set(bounds.GetExtent().GetX(), bounds.GetExtent().GetY(), bounds.GetExtent().GetZ());
}

static JoltPhysicsCore g_physics_core;

extern "C" PHYSICS_CORE_API IPhysicsCore* GetPhysicsCore() {
    return &g_physics_core;
}
