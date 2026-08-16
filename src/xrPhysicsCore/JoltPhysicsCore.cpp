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
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
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

#ifdef JPH_DEBUG_RENDERER
    m_debug_renderer = new CXRayJoltDebugRenderer();
#endif

    m_physics_system = new JPH::PhysicsSystem();
    const uint32_t cMaxBodies = 10240;
    const uint32_t cNumBodyMutexes = 1024; 
    const uint32_t cMaxBodyPairs = 10240;
    const uint32_t cMaxContactConstraints = 10240;

    m_physics_system->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        m_broad_phase_layer_interface,
        m_object_vs_broadphase_layer_filter,
        m_object_vs_object_layer_filter);

    m_physics_system->SetContactListener(&m_contact_listener);
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
    m_connected_bodies.clear();

    if (m_physics_system) {
        delete m_physics_system;
        m_physics_system = nullptr;
    }

#ifdef JPH_DEBUG_RENDERER
    delete m_debug_renderer;
    m_debug_renderer = nullptr;
#endif

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

void JoltPhysicsCore::DebugDraw(const Fvector& camera_pos)
{
#ifdef JPH_DEBUG_RENDERER
    if (m_physics_system && m_debug_renderer && m_debug_draw_flags > 0) {
        m_debug_renderer->SetCameraPos(JPH::Vec3(camera_pos.x, camera_pos.y, camera_pos.z));

        // Reset per-frame counters
        m_debug_renderer->m_draw_line_count = 0;
        m_debug_renderer->m_draw_tri_count = 0;

        // Extended diagnostic: count bodies actually in broadphase
        u32 total_bodies = m_physics_system->GetNumBodies();
        u32 active_bodies = m_physics_system->GetNumActiveBodies(JPH::EBodyType::RigidBody);

        static u32 log_counter = 0;
        if (log_counter % 300 == 0) { // Log every ~5 seconds at 60fps
            Msg("! [JOLT DEBUG] Flags: %u, Total bodies: %u, Active bodies: %u, Camera: (%.1f, %.1f, %.1f)",
                m_debug_draw_flags, total_bodies, active_bodies,
                camera_pos.x, camera_pos.y, camera_pos.z);
        }

        JPH::BodyManager::DrawSettings draw_settings;
        draw_settings.mDrawShape = (m_debug_draw_flags & 1) != 0;
        draw_settings.mDrawShapeWireframe = true; // Force wireframe for reliable rendering
        draw_settings.mDrawBoundingBox = (m_debug_draw_flags & 2) != 0;
        draw_settings.mDrawGetSupportFunction = false;
        draw_settings.mDrawMassAndInertia = (m_debug_draw_flags & 8) != 0;
        draw_settings.mDrawCenterOfMassTransform = (m_debug_draw_flags & 8) != 0;
        
        m_physics_system->DrawBodies(draw_settings, m_debug_renderer);
        
        if ((m_debug_draw_flags & 4) != 0) {
            m_physics_system->DrawConstraints(m_debug_renderer);
        }

        if (log_counter % 300 == 0) {
            Msg("! [JOLT DEBUG] After DrawBodies: lines=%u, tris=%u",
                m_debug_renderer->m_draw_line_count, m_debug_renderer->m_draw_tri_count);
        }
        
        // Call NextFrame to release unused cached geometry
        m_debug_renderer->NextFrame();
        
        log_counter++;
    }
#endif
}


void JoltPhysicsCore::SetDebugDrawFlags(u32 flags)
{
    m_debug_draw_flags = flags;
}

struct CDB_TRI_Mock {
    u32 verts[3];
    u32 dummy;
};

PhysicsShapeHandle JoltPhysicsCore::BuildCDBModel(const Fvector* verts, u32 v_cnt, const void* tris_raw, u32 t_cnt, const u32* tri_indices, u32 tri_indices_cnt) 
{
    const CDB_TRI_Mock* tris = static_cast<const CDB_TRI_Mock*>(tris_raw);

    JPH::VertexList jolt_vertices;
    jolt_vertices.reserve(v_cnt);
    for (u32 i = 0; i < v_cnt; ++i) {
        jolt_vertices.push_back(JPH::Float3(verts[i].x, verts[i].y, verts[i].z));
    }

    u32 actual_t_cnt = tri_indices ? tri_indices_cnt : t_cnt;
    JPH::IndexedTriangleList jolt_triangles;
    jolt_triangles.reserve(actual_t_cnt);
    
    for (u32 i = 0; i < actual_t_cnt; ++i) {
        u32 original_index = tri_indices ? tri_indices[i] : i;
        jolt_triangles.push_back(JPH::IndexedTriangle(
            tris[original_index].verts[0], 
            tris[original_index].verts[1],
            tris[original_index].verts[2],
            0,
            original_index
        ));
    }

    JPH::MeshShapeSettings settings(jolt_vertices, jolt_triangles);
    settings.mPerTriangleUserData = true;

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
    
    JPH::MeshShape* mesh_shape = static_cast<JPH::MeshShape*>(handle);

    JPH::Vec3 j_start(start.x, start.y, start.z);
    JPH::Vec3 j_dir(dir.x * range, dir.y * range, dir.z * range);
    JPH::RayCast ray(j_start, j_dir);

    JPH::RayCastSettings settings;
    settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mTreatConvexAsSolid = false;

    JPH::SubShapeIDCreator id_creator;
    JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;

    mesh_shape->CastRay(ray, settings, id_creator, collector);

    for (const JPH::RayCastResult& hit : collector.mHits) {
        CDBRaycastHit cdb_hit;
        cdb_hit.range = hit.mFraction * range;
        cdb_hit.tri_index = mesh_shape->GetTriangleUserData(hit.mSubShapeID2); 
        out_hits.push_back(cdb_hit);
    }
}

void JoltPhysicsCore::RaycastCDBModel(PhysicsShapeHandle handle, 
                                      const Fvector& start, const Fvector& dir, float range, 
                                      CDBRayMode mode, bool cull_backfaces, 
                                      std::vector<CDBRaycastHit>& out_hits) 
{
    if (!handle) return;
    
    JPH::MeshShape* mesh_shape = static_cast<JPH::MeshShape*>(handle);

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
        mesh_shape->CastRay(ray, settings, id_creator, collector);
        if (collector.HadHit()) {
            out_hits.push_back({ 
                collector.mHit.mFraction * range, 
                mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2) 
            });
        }
    } 
    else if (mode == CDBRayMode::First) 
    {
        JPH::AnyHitCollisionCollector<JPH::CastRayCollector> collector;
        mesh_shape->CastRay(ray, settings, id_creator, collector);
        if (collector.HadHit()) {
            out_hits.push_back({ 
                collector.mHit.mFraction * range, 
                mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2) 
            });
        }
    } 
    else 
    {
        JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
        mesh_shape->CastRay(ray, settings, id_creator, collector);
        for (const JPH::RayCastResult& hit : collector.mHits) {
            out_hits.push_back({ 
                hit.mFraction * range, 
                mesh_shape->GetTriangleUserData(hit.mSubShapeID2) 
            });
        }
        std::sort(out_hits.begin(), out_hits.end(), [](const CDBRaycastHit& a, const CDBRaycastHit& b) {
            return a.tri_index < b.tri_index;
        });
        out_hits.erase(std::unique(out_hits.begin(), out_hits.end(), [](const CDBRaycastHit& a, const CDBRaycastHit& b) {
            return a.tri_index == b.tri_index;
        }), out_hits.end());
    }
}

PhysicsShapeHandle JoltPhysicsCore::CreateCompoundShape(PhysicsShapeHandle* shapes, const Fmatrix* transforms, size_t count)
{
    if (count == 0 || !shapes || !transforms) return nullptr;

    JPH::StaticCompoundShapeSettings compound_settings;

    for (size_t i = 0; i < count; ++i)
    {
        JPH::Shape* jph_shape = static_cast<JPH::Shape*>(shapes[i]);
        if (!jph_shape) continue;

        const Fvector& pos = transforms[i].c;
        JPH::Vec3 position(pos.x, pos.y, pos.z);
        
        const Fmatrix& m = transforms[i];
        float trace = m._11 + m._22 + m._33;
        float x, y, z, w;

        if (trace > 0.0f) {
            float s = std::sqrt(trace + 1.0f) * 2.0f;
            w = 0.25f * s;
            x = (m._32 - m._23) / s;
            y = (m._13 - m._31) / s;
            z = (m._21 - m._12) / s;
        } else if ((m._11 > m._22) && (m._11 > m._33)) {
            float s = std::sqrt(1.0f + m._11 - m._22 - m._33) * 2.0f;
            w = (m._32 - m._23) / s;
            x = 0.25f * s;
            y = (m._21 + m._12) / s;
            z = (m._13 + m._31) / s;
        } else if (m._22 > m._33) {
            float s = std::sqrt(1.0f + m._22 - m._11 - m._33) * 2.0f;
            w = (m._13 - m._31) / s;
            x = (m._21 + m._12) / s;
            y = 0.25f * s;
            z = (m._32 + m._23) / s;
        } else {
            float s = std::sqrt(1.0f + m._33 - m._11 - m._22) * 2.0f;
            w = (m._21 - m._12) / s;
            x = (m._13 + m._31) / s;
            y = (m._32 + m._23) / s;
            z = 0.25f * s;
        }

        JPH::Quat rotation(x, y, z, w);
        rotation = rotation.Normalized();
        // Добавляем форму с ее локальным смещением
        compound_settings.AddShape(position, rotation, jph_shape);
    }

    JPH::ShapeSettings::ShapeResult result = compound_settings.Create();
    if (result.HasError())
    {
        Msg("! [JOLT] CreateCompoundShape ERROR: %s, falling back to BoxShape", result.GetError().c_str());
        return CreateBoxShape({0.5f, 0.5f, 0.5f});
    }

    JPH::Shape* final_shape = result.Get().GetPtr();
    final_shape->AddRef(); 
    
    return static_cast<PhysicsShapeHandle>(final_shape);
}

BodyHandle JoltPhysicsCore::CreateBodyFromShape(PhysicsShapeHandle shape_handle, const Fvector& pos, float mass)
{
    if (!m_physics_system || !shape_handle) return INVALID_BODY_HANDLE;

    JPH::Shape* shape = static_cast<JPH::Shape*>(shape_handle);
    
    JPH::BodyCreationSettings body_settings(
        shape, 
        JPH::RVec3(pos.x, pos.y, pos.z), 
        JPH::Quat::sIdentity(), 
        JPH::EMotionType::Dynamic, 
        Layers::MOVING
    );
    
    body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    body_settings.mMassPropertiesOverride.mMass = mass;

    JPH::Body* body = m_physics_system->GetBodyInterface().CreateBody(body_settings);
    if (!body) {
        return INVALID_BODY_HANDLE;
    }

    m_physics_system->GetBodyInterface().AddBody(body->GetID(), JPH::EActivation::Activate);
    
    return static_cast<BodyHandle>(body->GetID().GetIndexAndSequenceNumber());
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

    // For safety, initialize the projection part to standard values
    out_matrix._14_ = 0.0f; out_matrix._24_ = 0.0f; out_matrix._34_ = 0.0f; out_matrix._44_ = 1.0f;
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
    
    JPH::MeshShape* mesh_shape = static_cast<JPH::MeshShape*>(handle);

    JPH::Vec3 j_z(box_z_axis.x, box_z_axis.y, box_z_axis.z);
    JPH::Vec3 j_y(box_y_axis.x, box_y_axis.y, box_y_axis.z);

    j_z = j_z.Normalized();
    j_y = j_y.Normalized();
    JPH::Vec3 j_x = j_y.Cross(j_z).Normalized(); 

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
        JPH::MeshShape* m_mesh_shape; 
        bool has_hit = false;

        JoltOBBCollector(std::vector<u32>& out_ind, JPH::MeshShape* shape) 
            : indices(out_ind), m_mesh_shape(shape) {}

        virtual void AddHit(const JPH::CollideShapeResult &inResult) override {
            has_hit = true;
            indices.push_back(m_mesh_shape->GetTriangleUserData(inResult.mSubShapeID2));
        }
    };
    
    JoltOBBCollector collector(out_tri_indices, mesh_shape);
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
    
    // Сразу приводим к MeshShape
    JPH::MeshShape* mesh_shape = static_cast<JPH::MeshShape*>(handle);

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
            out_tri_indices.push_back(mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2));
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
            // Читаем UserData
            out_tri_indices.push_back(mesh_shape->GetTriangleUserData(hit.mSubShapeID2));
        }
        std::sort(out_tri_indices.begin(), out_tri_indices.end());
        out_tri_indices.erase(std::unique(out_tri_indices.begin(), out_tri_indices.end()), out_tri_indices.end());
    }
}

void JoltPhysicsCore::GetCDBModelBounds(PhysicsShapeHandle handle, Fvector& out_center, Fvector& out_extents) const 
{
    if (!handle) 
    {
        out_center.set(0.f, 0.f, 0.f);
        out_extents.set(0.f, 0.f, 0.f);
        return; 
    }
    
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
            if (j_axis1.LengthSq() > 0.001f) {
                j_axis1 = j_axis1.Normalized();
                JPH::Vec3 right = j_axis0.Cross(j_axis1);
                if (right.LengthSq() < 0.001f) {
                    j_axis1 = j_axis0.GetNormalizedPerpendicular();
                } else {
                    j_axis1 = right.Cross(j_axis0).Normalized();
                }
            } else {
                j_axis1 = j_axis0.GetNormalizedPerpendicular();
            }

            settings.mAxisX1 = settings.mAxisX2 = j_axis0;
            settings.mAxisY1 = settings.mAxisY2 = j_axis1;

            settings.mSwingType = JPH::ESwingType::Pyramid;

            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationX);
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationY);
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationZ);
            
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

        if (b1 != &JPH::Body::sFixedToWorld && b2 != &JPH::Body::sFixedToWorld) {
            m_connected_bodies[b1->GetID()].push_back(b2->GetID());
            m_connected_bodies[b2->GetID()].push_back(b1->GetID());
        }

        return handle;
    }

    return INVALID_JOINT_HANDLE;
}

void JoltPhysicsCore::DestroyJoint(JointHandle joint) 
{
    if (!m_physics_system) return;

    auto it = m_constraints.find(joint);
    if (it != m_constraints.end()) {
        JPH::Constraint* c = it->second.GetPtr();
        JPH::TwoBodyConstraint* two_body_c = static_cast<JPH::TwoBodyConstraint*>(c);
        const JPH::Body* b1 = two_body_c->GetBody1();
        const JPH::Body* b2 = two_body_c->GetBody2();

        if (b1 != &JPH::Body::sFixedToWorld && b2 != &JPH::Body::sFixedToWorld) {
            auto& vec1 = m_connected_bodies[b1->GetID()];
            vec1.erase(std::remove(vec1.begin(), vec1.end(), b2->GetID()), vec1.end());
            
            auto& vec2 = m_connected_bodies[b2->GetID()];
            vec2.erase(std::remove(vec2.begin(), vec2.end(), b1->GetID()), vec2.end());
        }

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
        if (active) {
            hinge->SetTargetAngularVelocity(velocity);
            hinge->GetMotorSettings().SetTorqueLimit(force);
        }
    } else if (c->GetSubType() == JPH::EConstraintSubType::Slider) {
        auto* slider = static_cast<JPH::SliderConstraint*>(c);
        slider->SetMotorState(active ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        if (active) {
            slider->SetTargetVelocity(velocity);
            slider->GetMotorSettings().SetForceLimit(force);
        }
    } else if (c->GetSubType() == JPH::EConstraintSubType::SixDOF) {
        auto* six = static_cast<JPH::SixDOFConstraint*>(c);
        auto axis = (axis_num == 0) ? JPH::SixDOFConstraintSettings::EAxis::RotationX :
                    (axis_num == 1) ? JPH::SixDOFConstraintSettings::EAxis::RotationY :
                                      JPH::SixDOFConstraintSettings::EAxis::RotationZ;
        six->SetMotorState(axis, active ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        if (active) {
            JPH::Vec3 target_vel = six->GetTargetAngularVelocityCS();
            if (axis_num == 0) target_vel.SetX(velocity);
            else if (axis_num == 1) target_vel.SetY(velocity);
            else if (axis_num == 2) target_vel.SetZ(velocity);
            six->SetTargetAngularVelocityCS(target_vel);
            six->GetMotorSettings(axis).SetTorqueLimit(force);
        }
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

PhysicsShapeHandle JoltPhysicsCore::CreateBoxShape(const Fvector& half_extents) {
    float hx = std::max(half_extents.x, 0.001f);
    float hy = std::max(half_extents.y, 0.001f);
    float hz = std::max(half_extents.z, 0.001f);
    JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(JPH::Vec3(hx, hy, hz));
    shape->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(const_cast<JPH::Shape*>(shape.GetPtr()));
}

PhysicsShapeHandle JoltPhysicsCore::CreateSphereShape(float radius) {
    float r = std::max(radius, 0.001f);
    JPH::RefConst<JPH::Shape> shape = new JPH::SphereShape(r);
    shape->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(const_cast<JPH::Shape*>(shape.GetPtr()));
}

PhysicsShapeHandle JoltPhysicsCore::CreateCylinderShape(float radius, float half_height) {
    float hh = std::max(half_height, 0.001f);
    float r = std::max(radius, 0.001f);
    JPH::RefConst<JPH::Shape> shape = new JPH::CylinderShape(hh, r);
    shape->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(const_cast<JPH::Shape*>(shape.GetPtr()));
}

PhysicsShapeHandle JoltPhysicsCore::CreateCapsuleShape(float radius, float half_height) {
    float hh = std::max(half_height, 0.001f);
    float r = std::max(radius, 0.001f);
    JPH::RefConst<JPH::Shape> capsule = new JPH::CapsuleShape(hh, r);
    
    JPH::RefConst<JPH::Shape> translated_capsule = JPH::RotatedTranslatedShapeSettings(
        JPH::Vec3(0, half_height, 0),
        JPH::Quat::sIdentity(),
        capsule
    ).Create().Get();

    translated_capsule->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(const_cast<JPH::Shape*>(translated_capsule.GetPtr()));
}

CharacterVirtualHandle JoltPhysicsCore::CreateCharacterVirtual(PhysicsShapeHandle shape, const Fvector& initial_pos, float mass) {
    JPH::Shape* jolt_shape = reinterpret_cast<JPH::Shape*>(shape);

    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape = jolt_shape;
    settings->mMass = mass;
    settings->mMaxSlopeAngle = JPH::DegreesToRadians(45.0f);
    settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -0.1f);
    
    JPH::RVec3 pos(initial_pos.x, initial_pos.y, initial_pos.z);
    
    JPH::CharacterVirtual* character = new JPH::CharacterVirtual(settings, pos, JPH::Quat::sIdentity(), 0, m_physics_system);
    
    CharacterVirtualHandle handle = m_next_character_handle++;
    m_characters[handle] = character;
    m_stick_to_floor[handle] = true;
    return handle;
}

void JoltPhysicsCore::DestroyCharacterVirtual(CharacterVirtualHandle handle) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        m_characters.erase(it);
        m_stick_to_floor.erase(handle);
    }
}

void JoltPhysicsCore::SetCharacterVirtualVelocity(CharacterVirtualHandle handle, const Fvector& velocity) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        it->second->SetLinearVelocity(JPH::Vec3(velocity.x, velocity.y, velocity.z));
    }
}

void JoltPhysicsCore::GetCharacterVirtualVelocity(CharacterVirtualHandle handle, Fvector& velocity) const {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::Vec3 vel = it->second->GetLinearVelocity();
        velocity.set(vel.GetX(), vel.GetY(), vel.GetZ());
    }
}

void JoltPhysicsCore::GetCharacterVirtualPosition(CharacterVirtualHandle handle, Fvector& position) const {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        JPH::RVec3 pos = character->GetPosition();
        pos -= character->GetUp() * character->GetCharacterPadding();
        position.set(pos.GetX(), pos.GetY(), pos.GetZ());
    }
}

void JoltPhysicsCore::SetCharacterVirtualPosition(CharacterVirtualHandle handle, const Fvector& position) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        JPH::RVec3 pos(position.x, position.y, position.z);
        pos += character->GetUp() * character->GetCharacterPadding();
        character->SetPosition(pos);
    }
}

void JoltPhysicsCore::SetCharacterVirtualShape(CharacterVirtualHandle handle, PhysicsShapeHandle shape) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::Shape* jolt_shape = reinterpret_cast<JPH::Shape*>(shape);
        it->second->SetShape(jolt_shape, 1.5f * m_physics_system->GetPhysicsSettings().mPenetrationSlop,
                             m_physics_system->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                             m_physics_system->GetDefaultLayerFilter(Layers::MOVING),
                             {}, {}, *m_temp_allocator);
    }
}

void JoltPhysicsCore::ActivateCharacterVirtual(CharacterVirtualHandle handle) {
    // Virtual characters don't need activation in Jolt like rigid bodies do, 
    // since they are updated manually. We can leave this as a no-op or manage a list.
}

void JoltPhysicsCore::DeactivateCharacterVirtual(CharacterVirtualHandle handle) {
    // Same as above.
}

bool JoltPhysicsCore::IsCharacterVirtualOnGround(CharacterVirtualHandle handle) const {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        return it->second->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround && !it->second->IsSlopeTooSteep(it->second->GetGroundNormal());
    }
    return false;
}

void JoltPhysicsCore::GetCharacterVirtualGroundState(CharacterVirtualHandle handle, SJoltCharacterGroundState& out_state) const {
    out_state.on_ground = false;
    out_state.ground_normal.set(0.f, 1.f, 0.f);
    out_state.ground_velocity.set(0.f, 0.f, 0.f);

    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        
        out_state.on_ground = (character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround || 
                               character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnSteepGround);
        
        JPH::Vec3 normal = character->GetGroundNormal();
        out_state.ground_normal.set(normal.GetX(), normal.GetY(), normal.GetZ());
        
        JPH::Vec3 ground_vel = character->GetGroundVelocity();
        out_state.ground_velocity.set(ground_vel.GetX(), ground_vel.GetY(), ground_vel.GetZ());
    }
}

void JoltPhysicsCore::SetCharacterVirtualUserData(CharacterVirtualHandle handle, void* data) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        it->second->SetUserData(reinterpret_cast<JPH::uint64>(data));
    }
}

class JoltIgnoreActorBodyFilter : public JPH::BodyFilter {
public:
    JPH::PhysicsSystem* m_system;
    JPH::uint64 m_actor_user_data;
    
    JoltIgnoreActorBodyFilter(JPH::PhysicsSystem* system, JPH::uint64 user_data) 
        : m_system(system), m_actor_user_data(user_data) {}
        
    virtual bool ShouldCollide(const JPH::BodyID &inBodyID) const override {
        if (m_actor_user_data == 0) return true;
        
        JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), inBodyID);
        if (lock.Succeeded()) {
            if (lock.GetBody().GetUserData() == m_actor_user_data) {
                return false;
            }
        }
        return true;
    }
};

void JoltPhysicsCore::UpdateCharacterVirtual(CharacterVirtualHandle handle, float delta_time, const Fvector& gravity) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        
        // X-Ray doesn't automatically apply gravity to velocity when not on ground
        // CharacterVirtual requires us to explicitly add gravity to mLinearVelocity
        JPH::Vec3 current_vel = character->GetLinearVelocity();
        current_vel += JPH::Vec3(gravity.x, gravity.y, gravity.z) * delta_time;
        character->SetLinearVelocity(current_vel);

        JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
        bool stick_to_floor = true;
        if (m_stick_to_floor.find(handle) != m_stick_to_floor.end()) {
            stick_to_floor = m_stick_to_floor[handle];
        }
        
        if (!stick_to_floor) {
            update_settings.mStickToFloorStepDown = JPH::Vec3::sZero();
        } else {
            update_settings.mStickToFloorStepDown = -character->GetUp() * 0.6f;
        }
        update_settings.mWalkStairsStepUp = character->GetUp() * 0.4f;

        JoltIgnoreActorBodyFilter body_filter(m_physics_system, character->GetUserData());

        character->ExtendedUpdate(
            delta_time,
            JPH::Vec3(gravity.x, gravity.y, gravity.z),
            update_settings,
            m_physics_system->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
            m_physics_system->GetDefaultLayerFilter(Layers::MOVING),
            body_filter, {}, *m_temp_allocator
        );
    }
}

void JoltPhysicsCore::SetCharacterVirtualStickToFloor(CharacterVirtualHandle handle, bool stick_to_floor) {
    if (m_stick_to_floor.find(handle) != m_stick_to_floor.end()) {
        m_stick_to_floor[handle] = stick_to_floor;
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
