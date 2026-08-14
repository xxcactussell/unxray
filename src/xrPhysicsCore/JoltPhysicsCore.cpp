#include "stdafx.h"
#include "JoltPhysicsCore.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>

#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>

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
    for (auto& pair : m_constraints) {
        if (m_physics_system && pair.second) {
            m_physics_system->RemoveConstraint(pair.second);
        }
    }
    m_constraints.clear();

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

BodyHandle JoltPhysicsCore::CreateSphere(float radius, const Fvector& position, float mass) {
    if (!m_physics_system) return INVALID_BODY_HANDLE;

    JPH::SphereShapeSettings shape_settings(radius);
    JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
    
    JPH::BodyCreationSettings body_settings(
        shape_result.Get(), 
        JPH::Vec3(position.x, position.y, position.z), 
        JPH::Quat::sIdentity(), 
        JPH::EMotionType::Dynamic, 
        Layers::MOVING
    );

    body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    body_settings.mMassPropertiesOverride.mMass = mass;

    JPH::Body* body = m_physics_system->GetBodyInterface().CreateBody(body_settings);
    m_physics_system->GetBodyInterface().AddBody(body->GetID(), JPH::EActivation::Activate);
    
    return body->GetID().GetIndexAndSequenceNumber();
}

BodyHandle JoltPhysicsCore::CreateCylinder(float radius, float half_height, const Fvector& position, float mass) {
    if (!m_physics_system) return INVALID_BODY_HANDLE;

    JPH::CylinderShapeSettings shape_settings(half_height, radius);
    JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
    
    JPH::BodyCreationSettings body_settings(
        shape_result.Get(), 
        JPH::Vec3(position.x, position.y, position.z), 
        JPH::Quat::sIdentity(), 
        JPH::EMotionType::Dynamic, 
        Layers::MOVING
    );

    body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    body_settings.mMassPropertiesOverride.mMass = mass;

    JPH::Body* body = m_physics_system->GetBodyInterface().CreateBody(body_settings);
    m_physics_system->GetBodyInterface().AddBody(body->GetID(), JPH::EActivation::Activate);
    
    return body->GetID().GetIndexAndSequenceNumber();
}

void JoltPhysicsCore::GetBoxExtents(BodyHandle body_handle, Fvector& out_extents) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    const JPH::Shape* shape = m_physics_system->GetBodyInterface().GetShape(id).GetPtr();
    if (shape->GetSubType() == JPH::EShapeSubType::Box) {
        const JPH::BoxShape* box = static_cast<const JPH::BoxShape*>(shape);
        JPH::Vec3 extents = box->GetHalfExtent();
        out_extents.set(extents.GetX(), extents.GetY(), extents.GetZ());
    }
}

void JoltPhysicsCore::SetBoxExtents(BodyHandle body_handle, const Fvector& extents) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BoxShapeSettings shape_settings(JPH::Vec3(extents.x, extents.y, extents.z));
    JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
    
    if (shape_result.IsValid()) {
        m_physics_system->GetBodyInterface().SetShape(id, shape_result.Get(), false, JPH::EActivation::Activate);
    }
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
    
    JPH::Vec3 axis_x = transform.GetAxisX();
    JPH::Vec3 axis_y = transform.GetAxisY();
    JPH::Vec3 axis_z = transform.GetAxisZ();
    JPH::Vec3 pos = transform.GetTranslation();

    out_matrix.i.set(axis_x.GetX(), axis_x.GetY(), axis_x.GetZ());
    out_matrix.j.set(axis_y.GetX(), axis_y.GetY(), axis_y.GetZ());
    out_matrix.k.set(axis_z.GetX(), axis_z.GetY(), axis_z.GetZ());
    out_matrix.c.set(pos.GetX(), pos.GetY(), pos.GetZ());
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

void JoltPhysicsCore::SetBodyFixedRotation(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyLockWrite lock(m_physics_system->GetBodyLockInterface(), id);
    if (lock.Succeeded()) {
        JPH::Body& body = lock.GetBody();
        if (body.IsDynamic()) {
            body.GetMotionProperties()->SetInverseInertia(JPH::Vec3::sZero(), JPH::Quat::sIdentity());
        }
    }
}

BodyHandle JoltPhysicsCore::CreateStaticBody(PhysicsShapeHandle shape_handle, const Fvector& position) 
{
    if (!m_physics_system || !shape_handle) return INVALID_BODY_HANDLE;

    JPH::Shape* shape = static_cast<JPH::Shape*>(shape_handle);

    JPH::BodyCreationSettings body_settings(
        shape, 
        JPH::Vec3(position.x, position.y, position.z), 
        JPH::Quat::sIdentity(), 
        JPH::EMotionType::Static, 
        Layers::NON_MOVING
    );

    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    JPH::Body* body = body_interface.CreateBody(body_settings);
    
    if (!body) return INVALID_BODY_HANDLE;

    body_interface.AddBody(body->GetID(), JPH::EActivation::DontActivate);
    
    return body->GetID().GetIndexAndSequenceNumber();
}

// ============================================================================
// JOINT IMPLEMENTATIONS (NEW UNIVERSAL)
// ============================================================================

JointHandle JoltPhysicsCore::CreateJoint(int type, BodyHandle body1, BodyHandle body2, const Fvector& anchor, const Fvector& axis0, const Fvector& axis1, const Fvector& axis2, const Fvector& limits_lo, const Fvector& limits_hi) 
{
    if (!m_physics_system) return INVALID_JOINT_HANDLE;

    JPH::BodyLockRead lock1(m_physics_system->GetBodyLockInterface(), JPH::BodyID(body1));
    JPH::BodyLockRead lock2(m_physics_system->GetBodyLockInterface(), JPH::BodyID(body2));
    
    const JPH::Body* b1 = (body1 == INVALID_BODY_HANDLE) ? &JPH::Body::sFixedToWorld : (lock1.Succeeded() ? &lock1.GetBody() : nullptr);
    const JPH::Body* b2 = (body2 == INVALID_BODY_HANDLE) ? &JPH::Body::sFixedToWorld : (lock2.Succeeded() ? &lock2.GetBody() : nullptr);

    if (!b1 || !b2) return INVALID_JOINT_HANDLE;

    JPH::Vec3 j_anchor(anchor.x, anchor.y, anchor.z);
    JPH::Constraint* constraint = nullptr;

    switch (type) {
        case 0: { // ball
            JPH::PointConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = settings.mPoint2 = j_anchor;
            constraint = settings.Create(*const_cast<JPH::Body*>(b1), *const_cast<JPH::Body*>(b2));
            break;
        }
        case 1: { // hinge
            JPH::HingeConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = settings.mPoint2 = j_anchor;
            
            JPH::Vec3 j_axis(axis0.x, axis0.y, axis0.z);
            settings.mHingeAxis1 = settings.mHingeAxis2 = j_axis.Normalized();
            settings.mNormalAxis1 = settings.mNormalAxis2 = settings.mHingeAxis1.GetNormalizedPerpendicular();
            
            settings.mLimitsMin = limits_lo.x;
            settings.mLimitsMax = limits_hi.x;
            
            constraint = settings.Create(*const_cast<JPH::Body*>(b1), *const_cast<JPH::Body*>(b2));
            break;
        }
        case 4: { // slider
            JPH::SliderConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = settings.mPoint2 = j_anchor;
            
            JPH::Vec3 j_axis(axis0.x, axis0.y, axis0.z);
            settings.mSliderAxis1 = settings.mSliderAxis2 = j_axis.Normalized();
            settings.mNormalAxis1 = settings.mNormalAxis2 = settings.mSliderAxis1.GetNormalizedPerpendicular();
            
            settings.mLimitsMin = limits_lo.x;
            settings.mLimitsMax = limits_hi.x;
            
            constraint = settings.Create(*const_cast<JPH::Body*>(b1), *const_cast<JPH::Body*>(b2));
            break;
        }
        case 2: // hinge2
        case 3: // full_control
        { 
            JPH::SixDOFConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPosition1 = settings.mPosition2 = j_anchor;
            
            JPH::Vec3 j_axis0 = JPH::Vec3(axis0.x, axis0.y, axis0.z).Normalized();
            JPH::Vec3 j_axis1(axis1.x, axis1.y, axis1.z);
            j_axis1 = (j_axis1.LengthSq() > 0.001f) ? j_axis1.Normalized() : j_axis0.GetNormalizedPerpendicular();

            settings.mAxisX1 = settings.mAxisX2 = j_axis0;
            settings.mAxisY1 = settings.mAxisY2 = j_axis1;
            
            // Лямбда для безопасного назначения лимитов SixDOF осей
            auto apply_limit = [&](JPH::SixDOFConstraintSettings::EAxis axis, float lo, float hi) {
                if (lo <= -M_PI && hi >= M_PI) {
                    settings.MakeFreeAxis(axis);
                } else if (lo == hi || lo > hi) {
                    settings.MakeFixedAxis(axis);
                } else {
                    settings.mLimitMin[(int)axis] = lo;
                    settings.mLimitMax[(int)axis] = hi;
                }
            };

            apply_limit(JPH::SixDOFConstraintSettings::EAxis::RotationX, limits_lo.x, limits_hi.x);
            apply_limit(JPH::SixDOFConstraintSettings::EAxis::RotationY, limits_lo.y, limits_hi.y);
            
            if (type == 3) {
                apply_limit(JPH::SixDOFConstraintSettings::EAxis::RotationZ, limits_lo.z, limits_hi.z);
            } else {
                settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::RotationZ);
            }

            constraint = settings.Create(*const_cast<JPH::Body*>(b1), *const_cast<JPH::Body*>(b2));
            break;
        }
    }

    if (constraint) {
        m_physics_system->AddConstraint(constraint);
        JointHandle handle = m_next_joint_handle++;
        m_constraints[handle] = constraint;
        return handle;
    }

    return INVALID_JOINT_HANDLE;
}

void JoltPhysicsCore::DestroyJoint(JointHandle joint) 
{
    if (!m_physics_system) return;

    auto it = m_constraints.find(joint);
    if (it != m_constraints.end()) {
        m_physics_system->RemoveConstraint(it->second);
        m_constraints.erase(it);
    }
}

void JoltPhysicsCore::SetJointLimits(JointHandle joint, int axis_num, float lo, float hi) 
{
    auto it = m_constraints.find(joint);
    if (it == m_constraints.end()) return;

    JPH::Constraint* c = it->second.GetPtr();
    if (c->GetSubType() == JPH::EConstraintSubType::Hinge) {
        static_cast<JPH::HingeConstraint*>(c)->SetLimits(lo, hi);
    } else if (c->GetSubType() == JPH::EConstraintSubType::Slider) {
        static_cast<JPH::SliderConstraint*>(c)->SetLimits(lo, hi);
    } else if (c->GetSubType() == JPH::EConstraintSubType::SixDOF) {
        // 
    }
}

void JoltPhysicsCore::SetJointMotor(JointHandle joint, int axis_num, float force, float velocity) 
{
    auto it = m_constraints.find(joint);
    if (it == m_constraints.end()) return;

    JPH::Constraint* c = it->second.GetPtr();
    bool active = (force > 0.0f || velocity > 0.0f);

    if (c->GetSubType() == JPH::EConstraintSubType::Hinge) {
        auto* hinge = static_cast<JPH::HingeConstraint*>(c);
        hinge->SetMotorState(active ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        if (active) hinge->SetTargetAngularVelocity(velocity);
    } else if (c->GetSubType() == JPH::EConstraintSubType::Slider) {
        auto* slider = static_cast<JPH::SliderConstraint*>(c);
        slider->SetMotorState(active ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        if (active) slider->SetTargetVelocity(velocity);
    } else if (c->GetSubType() == JPH::EConstraintSubType::SixDOF) {
        auto* six = static_cast<JPH::SixDOFConstraint*>(c);
        auto axis = (axis_num == 0) ? JPH::SixDOFConstraintSettings::EAxis::RotationX :
                    (axis_num == 1) ? JPH::SixDOFConstraintSettings::EAxis::RotationY :
                                      JPH::SixDOFConstraintSettings::EAxis::RotationZ;
        six->SetMotorState(axis, active ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
    }
}

void JoltPhysicsCore::SetJointSpringDamping(JointHandle joint, int axis_num, float erp, float cfm) {
    // В Jolt упругость задается через SpringSettings (Frequency/Damping) 
    // Заглушка, если потребуется тонкая настройка для специфических суставов машин.
}

void JoltPhysicsCore::SetJointAxisDir(JointHandle joint, int axis_num, const Fvector& axis) {}
void JoltPhysicsCore::SetJointFudgeFactor(JointHandle joint, float factor) {}
void JoltPhysicsCore::SetJointFeedback(JointHandle joint, SPhysicsJointFeedback* feedback) {}

void JoltPhysicsCore::GetJointAxisDir(JointHandle joint, int axis_num, Fvector& axis) const { axis.set(0, 1, 0); }
void JoltPhysicsCore::GetJointAnchor(JointHandle joint, Fvector& anchor) const { anchor.set(0, 0, 0); }

float JoltPhysicsCore::GetJointAxisAngle(JointHandle joint, int axis_num) const {
    auto it = m_constraints.find(joint);
    if (it == m_constraints.end()) return 0.0f;
    
    JPH::Constraint* c = it->second.GetPtr();
    if (c->GetSubType() == JPH::EConstraintSubType::Hinge) {
        return static_cast<JPH::HingeConstraint*>(c)->GetCurrentAngle();
    } else if (c->GetSubType() == JPH::EConstraintSubType::Slider) {
        return static_cast<JPH::SliderConstraint*>(c)->GetCurrentPosition();
    }
    return 0.0f;
}

float JoltPhysicsCore::GetJointAxisAngleRate(JointHandle joint, int axis_num) const {
    return 0.0f;
}
void JoltPhysicsCore::GetBodyLinearVelocity(BodyHandle body_handle, Fvector& out_vel) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    JPH::Vec3 vel = body_interface.GetLinearVelocity(id);
    out_vel.set(vel.GetX(), vel.GetY(), vel.GetZ());
}

void JoltPhysicsCore::SetBodyLinearVelocity(BodyHandle body_handle, const Fvector& vel) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    body_interface.SetLinearVelocity(id, JPH::Vec3(vel.x, vel.y, vel.z));
}

void JoltPhysicsCore::GetBodyAngularVelocity(BodyHandle body_handle, Fvector& out_vel) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    JPH::Vec3 vel = body_interface.GetAngularVelocity(id);
    out_vel.set(vel.GetX(), vel.GetY(), vel.GetZ());
}

void JoltPhysicsCore::SetBodyAngularVelocity(BodyHandle body_handle, const Fvector& vel) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    
    body_interface.SetAngularVelocity(id, JPH::Vec3(vel.x, vel.y, vel.z));
}

void JoltPhysicsCore::SetBodyGravityFactor(BodyHandle body_handle, float factor) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().SetGravityFactor(id, factor);
}

void JoltPhysicsCore::ApplyLinearImpulse(BodyHandle body_handle, const Fvector& impulse) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().AddImpulse(id, JPH::Vec3(impulse.x, impulse.y, impulse.z));
}

void JoltPhysicsCore::ApplyPointImpulse(BodyHandle body_handle, const Fvector& impulse, const Fvector& point) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().AddImpulse(id, 
        JPH::Vec3(impulse.x, impulse.y, impulse.z), 
        JPH::Vec3(point.x, point.y, point.z));
}

void JoltPhysicsCore::ApplyForce(BodyHandle body_handle, const Fvector& force) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().AddForce(id, JPH::Vec3(force.x, force.y, force.z));
}

void JoltPhysicsCore::ApplyTorque(BodyHandle body_handle, const Fvector& torque) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().AddTorque(id, JPH::Vec3(torque.x, torque.y, torque.z));
}

bool JoltPhysicsCore::IsBodyActive(BodyHandle body_handle) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return false;
    JPH::BodyID id(body_handle);
    return m_physics_system->GetBodyInterface().IsActive(id);
}

void JoltPhysicsCore::ActivateBody(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().ActivateBody(id);
}

void JoltPhysicsCore::DeactivateBody(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().DeactivateBody(id);
}

float JoltPhysicsCore::GetBodyMass(BodyHandle body_handle) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return 1.0f;
    
    JPH::BodyID id(body_handle);
    return m_physics_system->GetBodyInterface().GetShape(id)->GetMassProperties().mMass;
}

void JoltPhysicsCore::GetBodyPointVelocity(BodyHandle body_handle, const Fvector& point, Fvector& velocity) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        velocity.set(0.f, 0.f, 0.f);
        return;
    }
    
    JPH::BodyID id(body_handle);
    JPH::Vec3 jolt_pos(point.x, point.y, point.z);
    
    JPH::Vec3 jolt_vel = m_physics_system->GetBodyInterface().GetPointVelocity(id, jolt_pos);
    
    velocity.set(jolt_vel.GetX(), jolt_vel.GetY(), jolt_vel.GetZ());
}

void JoltPhysicsCore::SetBodyIgnoreStatic(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    
    m_physics_system->GetBodyInterface().SetObjectLayer(id, Layers::NON_MOVING);
}

void JoltPhysicsCore::GetBodyPosition(BodyHandle body_handle, Fvector& position) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        position.set(0.f, 0.f, 0.f);
        return;
    }
    
    JPH::BodyID id(body_handle);
    JPH::Vec3 jolt_pos = m_physics_system->GetBodyInterface().GetPosition(id);
    
    position.set(jolt_pos.GetX(), jolt_pos.GetY(), jolt_pos.GetZ());
}

void JoltPhysicsCore::SetBodyPosition(BodyHandle body_handle, const Fvector& position) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    m_physics_system->GetBodyInterface().SetPosition(id, JPH::Vec3(position.x, position.y, position.z), JPH::EActivation::Activate);
}

void JoltPhysicsCore::GetBodyForce(BodyHandle body_handle, Fvector& force) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        force.set(0.f, 0.f, 0.f);
        return;
    }
    force.set(0.f, 0.f, 0.f);
}

void JoltPhysicsCore::SetBodyForce(BodyHandle body_handle, const Fvector& force) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    
    JPH::Vec3 jolt_force(force.x, force.y, force.z);
    m_physics_system->GetBodyInterface().AddForce(id, jolt_force);
}

float JoltPhysicsCore::GetBodyGravityFactor(BodyHandle body_handle) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return 1.0f;
    JPH::BodyID id(body_handle);
    return m_physics_system->GetBodyInterface().GetGravityFactor(id);
}

void* JoltPhysicsCore::GetBodyUserData(BodyHandle body_handle) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return nullptr;
    
    JPH::BodyID id(body_handle);
    JPH::uint64 user_data = m_physics_system->GetBodyInterface().GetUserData(id);
    
    return reinterpret_cast<void*>(user_data);
}

void JoltPhysicsCore::SetBodyUserData(BodyHandle body_handle, void* data) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    JPH::BodyLockWrite lock(m_physics_system->GetBodyLockInterface(), id);
    if (lock.Succeeded()) {
        lock.GetBody().SetUserData(reinterpret_cast<JPH::uint64>(data));
    }
}

static JoltPhysicsCore g_physics_core;

extern "C" PHYSICS_CORE_API IPhysicsCore* GetPhysicsCore() {
    static JoltPhysicsCore* g_physics_core = nullptr;
    
    if (!g_physics_core) {
        g_physics_core = new JoltPhysicsCore();
        g_physics_core->Initialize(); 
    }
    
    return g_physics_core;
}
