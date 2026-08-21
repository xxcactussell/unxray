#include "stdafx.h"
#include "JoltPhysicsCore.h"
#include "xrMaterialSystem/GameMtlLib.h"

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
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

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
    if (m_physics_system) return;

    if (!JPH::Factory::sInstance) {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }

    if (!m_temp_allocator) {
        m_temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
    }
    if (!m_job_system) {
        m_job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);
    }

#ifdef JPH_DEBUG_RENDERER
    if (!m_debug_renderer) {
        m_debug_renderer = new CXRayJoltDebugRenderer();
    }
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

void JoltPhysicsCore::Clear()
{
    if (!m_physics_system) return;

    // 1. Remove and clear all constraints
    for (auto& pair : m_constraints) {
        if (pair.second) {
            m_physics_system->RemoveConstraint(pair.second);
        }
    }
    m_constraints.clear();
    m_connected_bodies.clear();

    // 2. Remove and clear all ragdolls
    for (auto& pair : m_ragdolls) {
        if (pair.second) {
            pair.second->RemoveFromPhysicsSystem();
        }
    }
    m_ragdolls.clear();
    m_ragdoll_settings.clear();

    // 3. Clear virtual characters
    m_characters.clear();
    m_stick_to_floor.clear();
    m_character_gravity_factors.clear();

    // 4. Remove and destroy all rigid bodies
    JPH::BodyInterface& bi = m_physics_system->GetBodyInterface();
    JPH::BodyIDVector all_bodies;
    m_physics_system->GetBodies(all_bodies);
    if (!all_bodies.empty()) {
        bi.RemoveBodies(all_bodies.data(), (int)all_bodies.size());
        bi.DestroyBodies(all_bodies.data(), (int)all_bodies.size());
    }

    // Reset handle counters
    m_next_joint_handle = 1;
    m_next_character_handle = 1;
    m_next_ragdoll_handle = 1;

    // Optimize broadphase for clean state
    m_physics_system->OptimizeBroadPhase();
}

void JoltPhysicsCore::Step(float delta_time) 
{
    if (!m_physics_system || !m_temp_allocator || !m_job_system) return;

    int collision_steps = 1;
    if (delta_time > 1.0f / 60.0f) {
        collision_steps = (int)(delta_time * 60.0f) + 1; 
    }
    
    if (collision_steps > 4) collision_steps = 4;

    m_physics_system->Update(delta_time, collision_steps, m_temp_allocator, m_job_system);
}

void JoltPhysicsCore::Destroy() 
{
    Clear();

    if (m_physics_system) {
        delete m_physics_system;
        m_physics_system = nullptr;
    }

#ifdef JPH_DEBUG_RENDERER
    if (m_debug_renderer) {
        delete m_debug_renderer;
        m_debug_renderer = nullptr;
    }
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

#ifdef JPH_DEBUG_RENDERER
class DistanceBodyFilter : public JPH::BodyDrawFilter {
    JPH::Vec3 m_camera_pos;
    float m_max_dist_sq;
public:
    DistanceBodyFilter(JPH::Vec3 camera_pos, float max_dist)
        : m_camera_pos(camera_pos), m_max_dist_sq(max_dist * max_dist) {}

    virtual bool ShouldDraw(const JPH::Body& inBody) const override {
        return (inBody.GetCenterOfMassPosition() - m_camera_pos).LengthSq() <= m_max_dist_sq;
    }
};
#endif

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

        DistanceBodyFilter filter(JPH::Vec3(camera_pos.x, camera_pos.y, camera_pos.z), m_debug_draw_distance);

        JPH::BodyManager::DrawSettings draw_settings;
        draw_settings.mDrawShape = (m_debug_draw_flags & 1) != 0;
        draw_settings.mDrawShapeWireframe = true; // Force wireframe for reliable rendering
        draw_settings.mDrawBoundingBox = (m_debug_draw_flags & 2) != 0;
        draw_settings.mDrawGetSupportFunction = false;
        draw_settings.mDrawMassAndInertia = (m_debug_draw_flags & 8) != 0;
        draw_settings.mDrawCenterOfMassTransform = (m_debug_draw_flags & 8) != 0;
        
        m_physics_system->DrawBodies(draw_settings, m_debug_renderer, &filter);
        
        if ((m_debug_draw_flags & 4) != 0) {
            m_physics_system->DrawConstraints(m_debug_renderer);
            m_physics_system->DrawConstraintLimits(m_debug_renderer);
            m_physics_system->DrawConstraintReferenceFrame(m_debug_renderer);
        }
        
        // Draw virtual characters
        for (const auto& [handle, character] : m_characters) {
            if ((character->GetPosition() - JPH::Vec3(camera_pos.x, camera_pos.y, camera_pos.z)).LengthSq() <= m_debug_draw_distance * m_debug_draw_distance) {
                if ((m_debug_draw_flags & 1) != 0) {
                    character->GetShape()->Draw(m_debug_renderer, character->GetCenterOfMassTransform(), JPH::Vec3::sOne(), JPH::Color::sYellow, false, true);
                }
            }
        }
        
        // Call NextFrame to release unused cached geometry
        m_debug_renderer->NextFrame();
    }
#endif
}


void JoltPhysicsCore::SetDebugDrawFlags(u32 flags)
{
    m_debug_draw_flags = flags;
}

void JoltPhysicsCore::SetDebugDrawDistance(float distance)
{
    m_debug_draw_distance = distance;
}

struct CDB_TRI_Mock {
    u32 verts[3];
    u16 material;
    u16 sector;
};

static inline u32 PackTriangleUserData(u16 mtl_idx, u32 tri_idx) {
    return (u32(mtl_idx) << 22) | (tri_idx & 0x003FFFFF);
}

static inline u16 UnpackMaterialIndex(u32 user_data) {
    return (user_data == u32(-1)) ? GAMEMTL_NONE_IDX : static_cast<u16>(user_data >> 22);
}

static inline u32 UnpackTriangleIndex(u32 user_data) {
    return (user_data == u32(-1)) ? u32(-1) : (user_data & 0x003FFFFF);
}

static inline bool IsPassableMaterial(const SGameMtl* mtl) {
    if (!mtl) return false;
    return (mtl->Flags.get() & (SGameMtl::flPassable | SGameMtl::flLiquid)) != 0;
}

static inline u32 GetTriangleUserDataForSubShape(const JPH::Shape* shape, const JPH::SubShapeID& sub_shape_id)
{
    if (!shape) return u32(-1);

    JPH::SubShapeID remainder;
    const JPH::Shape* leaf = shape->GetLeafShape(sub_shape_id, remainder);
    if (leaf && leaf->GetSubType() == JPH::EShapeSubType::Mesh)
    {
        return static_cast<const JPH::MeshShape*>(leaf)->GetTriangleUserData(remainder);
    }
    return u32(-1);
}

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
        u16 mtl = tris[original_index].material & 0x3FFF; // Очищаем от флагов компилятора
        u32 packed_data = PackTriangleUserData(mtl, original_index);

        jolt_triangles.push_back(JPH::IndexedTriangle(
            tris[original_index].verts[0], 
            tris[original_index].verts[1],
            tris[original_index].verts[2],
            0,
            packed_data
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
                UnpackTriangleIndex(mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2)) 
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
                UnpackTriangleIndex(mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2)) 
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
                UnpackTriangleIndex(mesh_shape->GetTriangleUserData(hit.mSubShapeID2)) 
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
        cdb_hit.tri_index = UnpackTriangleIndex(mesh_shape->GetTriangleUserData(hit.mSubShapeID2));
        out_hits.push_back(cdb_hit);
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
    body_settings.mFriction = 0.7f;
    body_settings.mRestitution = 0.1f;

    if (mass > 0.0f && mass <= 3.0f) {
        body_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }

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
    body_settings.mFriction = 0.7f;
    body_settings.mRestitution = 0.1f;

    if (mass > 0.0f && mass <= 3.0f) {
        body_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }

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
    body_settings.mFriction = 0.7f;
    body_settings.mRestitution = 0.1f;

    if (mass > 0.0f && mass <= 3.0f) {
        body_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }

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
    body_settings.mFriction = 0.7f;
    body_settings.mRestitution = 0.1f;

    if (mass > 0.0f && mass <= 3.0f) {
        body_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }

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
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        out_matrix.i.set(1.0f, 0.0f, 0.0f);
        out_matrix.j.set(0.0f, 1.0f, 0.0f);
        out_matrix.k.set(0.0f, 0.0f, 1.0f);
        out_matrix.c.set(0.0f, 0.0f, 0.0f);
        return;
    }

    JPH::BodyID id(body_handle);
    JPH::Mat44 transform = m_physics_system->GetBodyInterface().GetWorldTransform(id);
    JPH::Vec3 pos = transform.GetTranslation();

    if (_isnan(pos.GetX()) || _isnan(pos.GetY()) || _isnan(pos.GetZ())) {
        out_matrix.i.set(1.0f, 0.0f, 0.0f);
        out_matrix.j.set(0.0f, 1.0f, 0.0f);
        out_matrix.k.set(0.0f, 0.0f, 1.0f);
        out_matrix.c.set(0.0f, 0.0f, 0.0f);
        return;
    }

    JPH::Vec3 axis_x = transform.GetAxisX();
    JPH::Vec3 axis_y = transform.GetAxisY();
    JPH::Vec3 axis_z = transform.GetAxisZ();

    out_matrix.i.set(axis_x.GetX(), axis_x.GetY(), axis_x.GetZ());
    out_matrix.j.set(axis_y.GetX(), axis_y.GetY(), axis_y.GetZ());
    out_matrix.k.set(axis_z.GetX(), axis_z.GetY(), axis_z.GetZ());
    out_matrix.c.set(pos.GetX(), pos.GetY(), pos.GetZ());
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
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        center.set(0.f, 0.f, 0.f);
        half_extents.set(0.1f, 0.1f, 0.1f);
        return;
    }

    JPH::BodyID id(body_handle);
    JPH::AABox bounds = m_physics_system->GetBodyInterface().GetTransformedShape(id).GetWorldSpaceBounds();
    
    JPH::Vec3 j_center = bounds.GetCenter();
    JPH::Vec3 j_extents = bounds.GetExtent();

    if (_isnan(j_center.GetX()) || _isnan(j_extents.GetX())) {
        center.set(0.f, 0.f, 0.f);
        half_extents.set(0.1f, 0.1f, 0.1f);
        return;
    }

    center.set(j_center.GetX(), j_center.GetY(), j_center.GetZ());
    half_extents.set(j_extents.GetX(), j_extents.GetY(), j_extents.GetZ());
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
            indices.push_back(UnpackTriangleIndex(m_mesh_shape->GetTriangleUserData(inResult.mSubShapeID2)));
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
            out_tri_indices.push_back(UnpackTriangleIndex(mesh_shape->GetTriangleUserData(collector.mHit.mSubShapeID2)));
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
            out_tri_indices.push_back(UnpackTriangleIndex(mesh_shape->GetTriangleUserData(hit.mSubShapeID2)));
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

void JoltPhysicsCore::SetBodyMotionType(BodyHandle body_handle, int motion_type) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;

    JPH::BodyID id(body_handle);
    JPH::EMotionType j_motion = JPH::EMotionType::Dynamic;
    JPH::ObjectLayer j_layer = Layers::MOVING;

    bool ignore_static = m_ignore_static_bodies.count(id) != 0;

    if (motion_type == 0) {
        j_motion = JPH::EMotionType::Static;
        j_layer = Layers::NON_MOVING;
    } else if (motion_type == 1) {
        j_motion = JPH::EMotionType::Kinematic;
        j_layer = ignore_static ? Layers::MOVING_NO_STATIC : Layers::MOVING;
    } else {
        j_motion = JPH::EMotionType::Dynamic;
        j_layer = ignore_static ? Layers::MOVING_NO_STATIC : Layers::MOVING;
    }

    JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
    body_interface.SetMotionType(id, j_motion, JPH::EActivation::Activate);
    body_interface.SetObjectLayer(id, j_layer);
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
            
            JPH::Vec3 j_axis = JPH::Vec3(axis0.x, axis0.y, axis0.z).Normalized();
            settings.mHingeAxis1 = settings.mHingeAxis2 = j_axis;
            
            JPH::Vec3 ref_normal(axis1.x, axis1.y, axis1.z);
            if (ref_normal.LengthSq() < 0.001f) {
                ref_normal = j_axis.GetNormalizedPerpendicular();
            } else {
                ref_normal = ref_normal.Normalized();
            }
            
            settings.mNormalAxis1 = ref_normal;
            settings.mNormalAxis2 = ref_normal;
            
            float min_limit = std::min(limits_lo.x, limits_hi.x);
            float max_limit = std::max(limits_lo.x, limits_hi.x);

            if (min_limit <= -float(M_PI) && max_limit >= float(M_PI)) {
                settings.mLimitsMin = -JPH::JPH_PI;
                settings.mLimitsMax = JPH::JPH_PI;
            } else {
                settings.mLimitsMin = std::clamp(min_limit, -JPH::JPH_PI, JPH::JPH_PI);
                settings.mLimitsMax = std::clamp(max_limit, -JPH::JPH_PI, JPH::JPH_PI);
            }

            settings.mLimitsSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
            settings.mLimitsSpringSettings.mFrequency = 20.0f;
            settings.mLimitsSpringSettings.mDamping = 1.0f;
            
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

        if (b1 && b2) {
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

        if (b1 && b2) {
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

void JoltPhysicsCore::SetJointSpringDamping(JointHandle joint, int axis_num, float erp, float cfm)
{
    auto it = m_constraints.find(joint);
    if (it == m_constraints.end()) return;
    JPH::Constraint* c = it->second.GetPtr();

    if (cfm <= 1e-8f) return;

    const float h = 1.0f / 60.0f;
    JPH::SpringSettings spring;
    spring.mMode      = JPH::ESpringMode::StiffnessAndDamping;
    spring.mStiffness = erp / (cfm * h);
    spring.mDamping   = (1.0f - erp) / cfm;

    switch (c->GetSubType())
    {
    case JPH::EConstraintSubType::Hinge:
        if (axis_num == 0 || axis_num == -1)
            static_cast<JPH::HingeConstraint*>(c)->SetLimitsSpringSettings(spring);
        break;
    case JPH::EConstraintSubType::Slider:
        if (axis_num == 0 || axis_num == -1)
            static_cast<JPH::SliderConstraint*>(c)->SetLimitsSpringSettings(spring);
        break;
    case JPH::EConstraintSubType::SixDOF:
        if (axis_num >= 0) {
            auto ax = axis_num == 0 ? JPH::SixDOFConstraintSettings::EAxis::RotationX
                    : axis_num == 1 ? JPH::SixDOFConstraintSettings::EAxis::RotationY
                                    : JPH::SixDOFConstraintSettings::EAxis::RotationZ;
            static_cast<JPH::SixDOFConstraint*>(c)->SetLimitsSpringSettings(ax, spring);
        }
        break;
    default: break;
    }
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
    
    body_interface.ActivateBody(id); // Гарантируем пробуждение
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
    
    body_interface.ActivateBody(id); // Гарантируем пробуждение
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
    JPH::BodyInterface& bi = m_physics_system->GetBodyInterface();
    bi.ActivateBody(id);
    bi.AddImpulse(id, JPH::Vec3(impulse.x, impulse.y, impulse.z));
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
    if (!m_physics_system->GetBodyInterface().IsAdded(id)) return 1.0f;

    JPH::BodyLockRead lock(m_physics_system->GetBodyLockInterface(), id);
    if (!lock.Succeeded()) return 1.0f;

    const JPH::Body& body = lock.GetBody();
    const JPH::Shape* shape = body.GetShape();
    if (!shape) return 1.0f;

    return shape->GetMassProperties().mMass;
}

void JoltPhysicsCore::GetBodyPointVelocity(BodyHandle body_handle, const Fvector& point, Fvector& velocity) const {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) {
        velocity.set(0.f, 0.f, 0.f);
        return;
    }
    
    JPH::BodyID id(body_handle);
    if (!m_physics_system->GetBodyInterface().IsAdded(id)) {
        velocity.set(0.f, 0.f, 0.f);
        return;
    }
    
    JPH::Vec3 jolt_pos(point.x, point.y, point.z);
    JPH::Vec3 jolt_vel = m_physics_system->GetBodyInterface().GetPointVelocity(id, jolt_pos);
    
    velocity.set(jolt_vel.GetX(), jolt_vel.GetY(), jolt_vel.GetZ());
}

void JoltPhysicsCore::SetBodyIgnoreStatic(BodyHandle body_handle) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    
    JPH::BodyID id(body_handle);
    
    m_physics_system->GetBodyInterface().SetObjectLayer(id, Layers::NON_MOVING);
}

void JoltPhysicsCore::SetBodyCollideWithStatics(BodyHandle body_handle, bool collide) {
    if (!m_physics_system || body_handle == INVALID_BODY_HANDLE) return;
    JPH::BodyID id(body_handle);

    if (collide)
        m_ignore_static_bodies.erase(id);
    else
        m_ignore_static_bodies[id] = true;

    JPH::EMotionType motion = m_physics_system->GetBodyLockInterface()
        .TryGetBody(id)->GetMotionType();
    JPH::ObjectLayer layer = collide ? Layers::MOVING : Layers::MOVING_NO_STATIC;

    if (motion != JPH::EMotionType::Static)
        m_physics_system->GetBodyInterface().SetObjectLayer(id, layer);
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
    JPH::RefConst<JPH::Shape> cylinder_shape = new JPH::CylinderShape(hh, r);

    JPH::Shape* shape = const_cast<JPH::Shape*>(cylinder_shape.GetPtr());
    shape->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(shape);
}

PhysicsShapeHandle JoltPhysicsCore::CreateCapsuleShape(float radius, float half_height) {
    float hh = std::max(half_height, 0.001f);
    float r = std::max(radius, 0.001f);
    JPH::RefConst<JPH::Shape> capsule = new JPH::CapsuleShape(hh, r);
    capsule->AddRef();
    return reinterpret_cast<PhysicsShapeHandle>(const_cast<JPH::Shape*>(capsule.GetPtr()));
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

class JoltPassableShapeFilter : public JPH::ShapeFilter {
public:
    virtual bool ShouldCollide(const JPH::Shape* inShape2, const JPH::SubShapeID& inSubShapeIDOfShape2) const override {
        if (inShape2) {
            u32 user_data = inShape2->GetSubShapeUserData(inSubShapeIDOfShape2);
            u16 mtl_idx = UnpackMaterialIndex(user_data);
            SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
            if (user_data != u32(-1)) {
                u16 mtl_idx = UnpackMaterialIndex(user_data);
                SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
                if (IsPassableMaterial(mtl)) {
                    return false;
                }
            }
        }
        return true;
    }

    virtual bool ShouldCollide(const JPH::Shape* inShape1, const JPH::SubShapeID& inSubShapeIDOfShape1,
                               const JPH::Shape* inShape2, const JPH::SubShapeID& inSubShapeIDOfShape2) const override {
        return ShouldCollide(inShape2, inSubShapeIDOfShape2);
    }
};

void JoltPhysicsCore::MyContactListener::OnContactAdded(
    const JPH::Body& inBody1, 
    const JPH::Body& inBody2, 
    const JPH::ContactManifold& inManifold, 
    JPH::ContactSettings& ioSettings)
{
    const JPH::Body* static_body = inBody1.IsStatic() ? &inBody1 : (inBody2.IsStatic() ? &inBody2 : nullptr);
    const JPH::Body* dynamic_body = inBody1.IsDynamic() ? &inBody1 : (inBody2.IsDynamic() ? &inBody2 : nullptr);

    if (static_body && dynamic_body)
    {
        JPH::SubShapeID sub_shape = inBody1.IsStatic() ? inManifold.mSubShapeID1 : inManifold.mSubShapeID2;
        const JPH::Shape* shape = static_body->GetShape();
        
        if (shape)
        {
            u32 user_data = GetTriangleUserDataForSubShape(shape, sub_shape);
            if (user_data != u32(-1))
            {
                u16 mtl_idx = UnpackMaterialIndex(user_data); 
                SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
                if (mtl)
                {
                    ioSettings.mCombinedFriction = std::sqrt(dynamic_body->GetFriction() * mtl->fPHFriction);
                    
                    if (mtl->Flags.test(SGameMtl::flBounceable))
                        ioSettings.mCombinedRestitution = std::max(dynamic_body->GetRestitution(), mtl->fPHBouncing);
                    else
                        ioSettings.mCombinedRestitution = 0.0f;
                    
                    return;
                }
            }
        }
    }

    ioSettings.mCombinedFriction = std::sqrt(inBody1.GetFriction() * inBody2.GetFriction());
    ioSettings.mCombinedRestitution = std::max(inBody1.GetRestitution(), inBody2.GetRestitution());
}

void JoltPhysicsCore::MyContactListener::OnContactPersisted(
    const JPH::Body& inBody1, 
    const JPH::Body& inBody2, 
    const JPH::ContactManifold& inManifold, 
    JPH::ContactSettings& ioSettings)
{
    OnContactAdded(inBody1, inBody2, inManifold, ioSettings);
}

void JoltPhysicsCore::MyCharacterContactListener::OnContactAdded(
    const JPH::CharacterVirtual* inCharacter, 
    const JPH::CharacterContact& inContact, 
    JPH::CharacterContactSettings& ioSettings) 
{
    if (!inContact.mBodyB.IsInvalid() && m_core->m_physics_system) 
    {
        JPH::BodyLockWrite lock(m_core->m_physics_system->GetBodyLockInterface(), inContact.mBodyB);
        if (lock.Succeeded()) 
        {
            JPH::Body& body = lock.GetBody();
            
            if (body.IsDynamic())
            {
                JPH::Vec3 char_vel = inCharacter->GetLinearVelocity();
                float vel_sq = char_vel.LengthSq();
                if (vel_sq > 0.01f)
                {
                    JPH::Vec3 push_dir = -inContact.mContactNormal;
                    float push_force = inCharacter->GetMass() * 0.4f;
                    body.AddImpulse(push_dir * push_force, inContact.mPosition);
                }
            }

            const JPH::Shape* shape = body.GetShape();
            if (shape) 
            {
                u32 user_data = GetTriangleUserDataForSubShape(shape, inContact.mSubShapeIDB);
                if (user_data != u32(-1)) 
                {
                    u16 mtl_idx = UnpackMaterialIndex(user_data);
                    SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
                    if (IsPassableMaterial(mtl)) 
                    {
                        ioSettings.mCanPushCharacter = false;
                    }
                }
            }
        }
    }

    ProcessContact(inCharacter, inContact);
}

void JoltPhysicsCore::MyCharacterContactListener::OnContactPersisted(
    const JPH::CharacterVirtual* inCharacter, 
    const JPH::CharacterContact& inContact, 
    JPH::CharacterContactSettings& ioSettings) 
{
    OnContactAdded(inCharacter, inContact, ioSettings);
}

bool JoltPhysicsCore::MyCharacterContactListener::OnContactValidate(
    const JPH::CharacterVirtual* inCharacter,
    const JPH::CharacterContact& inContact)
{
    if (inContact.mBodyB.IsInvalid() || !m_core->m_physics_system)
        return true;

    JPH::BodyLockRead lock(m_core->m_physics_system->GetBodyLockInterface(), inContact.mBodyB);
    if (lock.Succeeded())
    {
        const JPH::Shape* shape = lock.GetBody().GetShape();
        if (shape)
        {
            u32 user_data = GetTriangleUserDataForSubShape(shape, inContact.mSubShapeIDB);
            if (user_data != u32(-1))
            {
                u16 mtl_idx = UnpackMaterialIndex(user_data);
                SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
                if (IsPassableMaterial(mtl))
                    return false;
            }
        }
    }
    return true;
}

void JoltPhysicsCore::MyCharacterContactListener::ProcessContact(const JPH::CharacterVirtual* inCharacter, const JPH::CharacterContact& inContact) {
    if (!inCharacter) return;

    u32 tri_user_data = u32(-1);
    void* other_body_user_data = nullptr;
    BodyHandle other_body_handle = INVALID_BODY_HANDLE;

    if (!inContact.mBodyB.IsInvalid() && m_core->m_physics_system) {
        JPH::BodyLockRead lock(m_core->m_physics_system->GetBodyLockInterface(), inContact.mBodyB);
        if (lock.Succeeded()) {
            const JPH::Body& body = lock.GetBody();
            if (body.IsDynamic()) {
                other_body_handle = inContact.mBodyB.GetIndexAndSequenceNumber();
                other_body_user_data = reinterpret_cast<void*>(body.GetUserData());
            }
            const JPH::Shape* shape = body.GetShape();
            if (shape) {
                u32 packed_data = GetTriangleUserDataForSubShape(shape, inContact.mSubShapeIDB);
                tri_user_data = UnpackTriangleIndex(packed_data);
            }
        }
    }

    Fvector contact_pos = { (float)inContact.mPosition.GetX(), (float)inContact.mPosition.GetY(), (float)inContact.mPosition.GetZ() };
    Fvector contact_norm = { inContact.mContactNormal.GetX(), inContact.mContactNormal.GetY(), inContact.mContactNormal.GetZ() };
    Fvector contact_vel = { inContact.mLinearVelocity.GetX(), inContact.mLinearVelocity.GetY(), inContact.mLinearVelocity.GetZ() };

    for (const auto& [handle, char_ptr] : m_core->m_characters) {
        if (char_ptr.GetPtr() == inCharacter) {
            auto cb_it = m_core->m_character_callbacks.find(handle);
            if (cb_it != m_core->m_character_callbacks.end() && cb_it->second.callback) {
                cb_it->second.callback(cb_it->second.user_data, contact_pos, contact_norm, contact_vel, tri_user_data, other_body_handle, other_body_user_data, inContact.mIsSensorB);
            }
            break;
        }
    }
}

JPH::ValidateResult JoltPhysicsCore::MyContactListener::OnContactValidate(
    const JPH::Body& inBody1, 
    const JPH::Body& inBody2, 
    JPH::RVec3Arg inBaseOffset, 
    const JPH::CollideShapeResult& inCollisionResult)
{
    auto it = m_core->m_connected_bodies.find(inBody1.GetID());
    if (it != m_core->m_connected_bodies.end()) {
        const auto& connected = it->second;
        if (std::find(connected.begin(), connected.end(), inBody2.GetID()) != connected.end()) {
            return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
        }
    }

    const JPH::Body* static_body = inBody1.IsStatic() ? &inBody1 : (inBody2.IsStatic() ? &inBody2 : nullptr);
    if (static_body)
    {
        JPH::SubShapeID sub_shape = inBody1.IsStatic() ? inCollisionResult.mSubShapeID1 : inCollisionResult.mSubShapeID2;
        const JPH::Shape* shape = static_body->GetShape();
        if (shape)
        {
            u32 tri_user_data = GetTriangleUserDataForSubShape(shape, sub_shape);
            if (tri_user_data != u32(-1))
            {
                u16 mtl_idx = UnpackMaterialIndex(tri_user_data);
                SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
                if (IsPassableMaterial(mtl))
                {
                    return JPH::ValidateResult::RejectContact;
                }
            }
        }
    }

    return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
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
    character->SetListener(&m_character_contact_listener);
    
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
        m_character_callbacks.erase(handle);
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
    if (it != m_characters.end() && m_physics_system && m_temp_allocator && shape) {
        JPH::Shape* jolt_shape = reinterpret_cast<JPH::Shape*>(shape);
        JPH::CharacterVirtual* character = it->second.GetPtr();
        if (!character) return;
        
        JoltIgnoreActorBodyFilter body_filter(m_physics_system, character->GetUserData());
        JoltPassableShapeFilter shape_filter;
        if (character->SetShape(jolt_shape, 1.5f,
                             m_physics_system->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                             m_physics_system->GetDefaultLayerFilter(Layers::MOVING),
                             body_filter, shape_filter, *m_temp_allocator))
        {
            character->SetInnerBodyShape(jolt_shape);
        }
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
    out_state.ground_triangle_user_data = u32(-1);

    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        
        out_state.on_ground = (character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround || 
                               character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnSteepGround);
        
        JPH::Vec3 normal = character->GetGroundNormal();
        out_state.ground_normal.set(normal.GetX(), normal.GetY(), normal.GetZ());
        
        JPH::Vec3 ground_vel = character->GetGroundVelocity();
        out_state.ground_velocity.set(ground_vel.GetX(), ground_vel.GetY(), ground_vel.GetZ());

        JPH::BodyID ground_body = character->GetGroundBodyID();
        JPH::SubShapeID ground_sub_shape = character->GetGroundSubShapeID();
        if (!ground_body.IsInvalid() && m_physics_system) {
            JPH::BodyLockRead lock(m_physics_system->GetBodyLockInterface(), ground_body);
            if (lock.Succeeded()) {
                const JPH::Body& body = lock.GetBody();
                const JPH::Shape* shape = body.GetShape();
                if (shape) {
                    out_state.ground_triangle_user_data = UnpackTriangleIndex(GetTriangleUserDataForSubShape(shape, ground_sub_shape));
                }
            }
        }
    }
}

void JoltPhysicsCore::SetCharacterVirtualUserData(CharacterVirtualHandle handle, void* data) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        it->second->SetUserData(reinterpret_cast<JPH::uint64>(data));
    }
}

void JoltPhysicsCore::SetCharacterVirtualContactCallback(CharacterVirtualHandle handle, CharacterContactCallbackFun callback, void* char_user_data) {
    if (callback) {
        m_character_callbacks[handle] = { callback, char_user_data };
    } else {
        m_character_callbacks.erase(handle);
    }
}


class JoltPassableContactCollector : public JPH::CollideShapeCollector {
public:
    JPH::PhysicsSystem* m_system;
    IPhysicsCore::CharacterContactCallbackFun m_callback;
    void* m_user_data;
    Fvector m_char_vel;

    JoltPassableContactCollector(JPH::PhysicsSystem* sys, IPhysicsCore::CharacterContactCallbackFun cb, void* ud, const Fvector& vel)
        : m_system(sys), m_callback(cb), m_user_data(ud), m_char_vel(vel) {}

    virtual void AddHit(const JPH::CollideShapeResult& inResult) override {
        if (inResult.mBodyID2.IsInvalid() || !m_callback || !m_system) return;

        JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), inResult.mBodyID2);
        if (!lock.Succeeded()) return;

        const JPH::Body& body = lock.GetBody();
        const JPH::Shape* shape = body.GetShape();
        if (!shape) return;

        u32 user_data = GetTriangleUserDataForSubShape(shape, inResult.mSubShapeID2);
        if (user_data == u32(-1)) return;

        u16 mtl_idx = UnpackMaterialIndex(user_data);
        SGameMtl* mtl = GMLib.GetMaterialByIdx(mtl_idx);
        
        if (IsPassableMaterial(mtl)) {
            JPH::RVec3 hit_pos = inResult.mContactPointOn2;
            JPH::Vec3 hit_norm = -inResult.mPenetrationAxis.Normalized();

            Fvector contact_pos = { (float)hit_pos.GetX(), (float)hit_pos.GetY(), (float)hit_pos.GetZ() };
            Fvector contact_norm = { hit_norm.GetX(), hit_norm.GetY(), hit_norm.GetZ() };
            u32 tri_idx = UnpackTriangleIndex(user_data);

            m_callback(m_user_data, contact_pos, contact_norm, m_char_vel, tri_idx, INVALID_BODY_HANDLE, nullptr, true);
        }
    }
};


void JoltPhysicsCore::UpdateCharacterVirtual(CharacterVirtualHandle handle, float delta_time, const Fvector& gravity) {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        JPH::CharacterVirtual* character = it->second.GetPtr();
        
        float gravity_factor = 1.0f;
        if (m_character_gravity_factors.find(handle) != m_character_gravity_factors.end()) {
            gravity_factor = m_character_gravity_factors[handle];
        }

        JPH::Vec3 current_vel = character->GetLinearVelocity();
        JPH::Vec3 jolt_gravity(gravity.x, gravity.y, gravity.z);
        JPH::Vec3 applied_gravity = jolt_gravity * gravity_factor;

        auto ground_state = character->GetGroundState();

        if (ground_state == JPH::CharacterVirtual::EGroundState::OnGround || 
            ground_state == JPH::CharacterVirtual::EGroundState::OnSteepGround) 
        {
            if (current_vel.GetY() < 0.0f) {
                current_vel.SetY(0.0f);
            }
        } else {
            current_vel += applied_gravity * delta_time;
            if (current_vel.GetY() < -100.0f) {
                current_vel.SetY(-100.0f);
            }
        }

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
        JoltPassableShapeFilter shape_filter;

        character->ExtendedUpdate(
            delta_time,
            jolt_gravity * gravity_factor,
            update_settings,
            m_physics_system->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
            m_physics_system->GetDefaultLayerFilter(Layers::MOVING),
            body_filter, shape_filter, *m_temp_allocator
        );

        auto cb_it = m_character_callbacks.find(handle);
        if (cb_it != m_character_callbacks.end() && cb_it->second.callback) {
            JPH::CollideShapeSettings collide_settings;
            collide_settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

            Fvector vel;
            GetCharacterVirtualVelocity(handle, vel);

            JoltPassableContactCollector passable_collector(
                m_physics_system,
                cb_it->second.callback,
                cb_it->second.user_data,
                vel
            );

            m_physics_system->GetNarrowPhaseQuery().CollideShape(
                character->GetShape(),
                JPH::Vec3::sReplicate(1.0f),
                character->GetCenterOfMassTransform(),
                collide_settings,
                character->GetPosition(),
                passable_collector,
                m_physics_system->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                m_physics_system->GetDefaultLayerFilter(Layers::MOVING),
                body_filter,
                JPH::ShapeFilter()
            );
        }
    }
}

void JoltPhysicsCore::SetCharacterVirtualStickToFloor(CharacterVirtualHandle handle, bool stick_to_floor) {
    if (m_stick_to_floor.find(handle) != m_stick_to_floor.end()) {
        m_stick_to_floor[handle] = stick_to_floor;
    }
}

PhysicsShapeHandle JoltPhysicsCore::CreateRotatedTranslatedShape(PhysicsShapeHandle base_shape, const Fvector& position, const Fquaternion& rotation) {
    if (!base_shape) return nullptr;
    JPH::Shape* inner = reinterpret_cast<JPH::Shape*>(base_shape);
    JPH::RotatedTranslatedShapeSettings settings(
        JPH::Vec3(position.x, position.y, position.z),
        JPH::Quat(-rotation.x, -rotation.y, -rotation.z, rotation.w).Normalized(),
        inner
    );
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.IsValid()) {
        JPH::Shape* shape = result.Get().GetPtr();
        shape->AddRef();
        return reinterpret_cast<PhysicsShapeHandle>(shape);
    }
    return base_shape;
}

RagdollHandle JoltPhysicsCore::CreateRagdoll(const SRagdollSettings& settings) {
    if (!m_physics_system || settings.parts.empty()) return INVALID_RAGDOLL_HANDLE;

    JPH::Ref<JPH::RagdollSettings> jph_settings = new JPH::RagdollSettings();
    jph_settings->mSkeleton = new JPH::Skeleton();

    // 1. Build Skeleton & Parts
    jph_settings->mParts.resize(settings.parts.size());
    for (size_t i = 0; i < settings.parts.size(); ++i) {
        const auto& part = settings.parts[i];
        
        std::string joint_name = "joint_" + std::to_string(part.bone_id);
        jph_settings->mSkeleton->AddJoint(
            joint_name.c_str(), 
            part.parent_index
        );
        
        auto& jph_part = jph_settings->mParts[i];
        jph_part.SetShape(static_cast<JPH::Shape*>(part.shape));
        jph_part.mPosition = JPH::Vec3(part.position.x, part.position.y, part.position.z);
        JPH::Quat part_rot(-part.rotation.x, -part.rotation.y, -part.rotation.z, part.rotation.w);
        jph_part.mRotation = part_rot.Normalized();
        jph_part.mFriction = 0.8f;
        jph_part.mRestitution = 0.02f;
        
        float mass = std::max(part.mass, 0.05f);
        jph_part.mMassPropertiesOverride.mMass = mass;
        jph_part.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        
        // Root is kinematic, others are dynamic
        if (part.parent_index == -1) {
            jph_part.mMotionType = JPH::EMotionType::Kinematic;
        } else {
            jph_part.mMotionType = JPH::EMotionType::Dynamic;
        }
        
        jph_part.mMotionQuality = JPH::EMotionQuality::LinearCast;
        
        jph_part.mObjectLayer = Layers::RAGDOLL;
    }
    
    jph_settings->mSkeleton->CalculateParentJointIndices();
    
    // 2. Build Constraints
    for (const auto& c_desc : settings.constraints) {
        if (c_desc.child_index < 0 || c_desc.child_index >= settings.parts.size()) continue;
        
        const auto& part_child = settings.parts[c_desc.child_index];
        const auto& part_parent = settings.parts[part_child.parent_index];
        
        JPH::Ref<JPH::SwingTwistConstraintSettings> constraint = new JPH::SwingTwistConstraintSettings();
        constraint->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
        
        JPH::Quat rot_parent = JPH::Quat(-part_parent.rotation.x, -part_parent.rotation.y, -part_parent.rotation.z, part_parent.rotation.w).Normalized();
        JPH::Quat rot_child = JPH::Quat(-part_child.rotation.x, -part_child.rotation.y, -part_child.rotation.z, part_child.rotation.w).Normalized();
        
        JPH::Vec3 pos_parent(part_parent.position.x, part_parent.position.y, part_parent.position.z);
        JPH::Vec3 pos_child(part_child.position.x, part_child.position.y, part_child.position.z);
        
        JPH::Vec3 parent_com_local = jph_settings->mParts[part_child.parent_index].GetShape()->GetCenterOfMass();
        JPH::Vec3 child_com_local = jph_settings->mParts[c_desc.child_index].GetShape()->GetCenterOfMass();
        
        // The constraint is located at the child's origin, but must be specified relative to the real COM.
        constraint->mPosition1 = (rot_parent.Conjugated() * (pos_child - pos_parent)) - parent_com_local;
        constraint->mPosition2 = -child_com_local;
        
        JPH::Vec3 twist_world = rot_child * JPH::Vec3(c_desc.twist_axis.x, c_desc.twist_axis.y, c_desc.twist_axis.z);
        JPH::Vec3 plane_world = rot_child * JPH::Vec3(c_desc.plane_axis.x, c_desc.plane_axis.y, c_desc.plane_axis.z);
        
        constraint->mTwistAxis1 = (rot_parent.Conjugated() * twist_world).Normalized();
        constraint->mPlaneAxis1 = (rot_parent.Conjugated() * plane_world).Normalized();
        constraint->mTwistAxis2 = JPH::Vec3(c_desc.twist_axis.x, c_desc.twist_axis.y, c_desc.twist_axis.z).Normalized();
        constraint->mPlaneAxis2 = JPH::Vec3(c_desc.plane_axis.x, c_desc.plane_axis.y, c_desc.plane_axis.z).Normalized();
        
        constraint->mNormalHalfConeAngle = std::clamp(c_desc.swing_limit_y, 0.0f, float(M_PI * 0.5f));
        constraint->mPlaneHalfConeAngle = std::clamp(c_desc.swing_limit_z, 0.0f, float(M_PI * 0.5f));
        constraint->mTwistMinAngle = std::clamp(c_desc.twist_limit_min, -float(M_PI), float(M_PI));
        constraint->mTwistMaxAngle = std::clamp(c_desc.twist_limit_max, -float(M_PI), float(M_PI));
        if (constraint->mTwistMaxAngle < constraint->mTwistMinAngle) {
            std::swap(constraint->mTwistMinAngle, constraint->mTwistMaxAngle);
        }
        constraint->mMaxFrictionTorque = c_desc.max_friction_torque;
        
        // Motor settings with bounded torque limits to prevent explosive constraint forces
        constraint->mSwingMotorSettings = JPH::MotorSettings(JPH::ESpringMode::StiffnessAndDamping, settings.default_motor.stiffness, settings.default_motor.damping, 500.0f, 250.0f);
        constraint->mTwistMotorSettings = JPH::MotorSettings(JPH::ESpringMode::StiffnessAndDamping, settings.default_motor.stiffness, settings.default_motor.damping, 500.0f, 250.0f);
        
        // Attach to part
        jph_settings->mParts[c_desc.child_index].mToParent = constraint;
    }
    
    // X-Ray shapes overlap massively. Disable ALL internal collisions within the same ragdoll,
    // not just parent-child (DisableParentChildCollisions() alone is not enough).
    JPH::Ref<JPH::GroupFilterTable> group_filter = new JPH::GroupFilterTable((uint32_t)settings.parts.size());
    for (int i = 0; i < (int)settings.parts.size(); ++i)
        for (int j = 0; j < (int)settings.parts.size(); ++j)
            if (i != j)
                group_filter->DisableCollision(i, j);

    RagdollHandle handle = m_next_ragdoll_handle++;
    for (int i = 0; i < (int)settings.parts.size(); ++i) {
        jph_settings->mParts[i].mCollisionGroup.SetGroupFilter(group_filter);
        jph_settings->mParts[i].mCollisionGroup.SetSubGroupID(i);
        jph_settings->mParts[i].mCollisionGroup.SetGroupID(handle);
    }

    jph_settings->CalculateConstraintPriorities();
    jph_settings->Stabilize();
    jph_settings->CalculateBodyIndexToConstraintIndex();
    jph_settings->CalculateConstraintIndexToBodyIdxPair();
    
    JPH::Ragdoll* ragdoll = jph_settings->CreateRagdoll(handle, 0, m_physics_system);
    if (!ragdoll) return INVALID_RAGDOLL_HANDLE;
    m_ragdolls[handle] = ragdoll;
    m_ragdoll_settings[handle] = jph_settings;
    
    return handle;
}

void JoltPhysicsCore::DestroyRagdoll(RagdollHandle handle) {
    if (m_ragdolls.find(handle) != m_ragdolls.end()) {
        RemoveRagdollFromWorld(handle);
        m_ragdolls.erase(handle);
        m_ragdoll_settings.erase(handle);
    }
}

void JoltPhysicsCore::AddRagdollToWorld(RagdollHandle handle, bool activate) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        it->second->AddToPhysicsSystem(JPH::EActivation::Activate);
    }
}

void JoltPhysicsCore::RemoveRagdollFromWorld(RagdollHandle handle) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        it->second->RemoveFromPhysicsSystem();
    }
}

void JoltPhysicsCore::SetRagdollTargetPose(RagdollHandle handle, const Fquaternion* target_rotations, u32 count) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && target_rotations) {
        JPH::SkeletonPose target_pose;
        target_pose.SetSkeleton(m_ragdoll_settings[handle]->mSkeleton);
        
        u32 j_count = (u32)target_pose.GetJointMatrices().size();
        u32 it_count = (count < j_count) ? count : j_count;
        for (u32 i = 0; i < it_count; ++i) {
            target_pose.GetJointMatrices()[i] = JPH::Mat44::sRotation(JPH::Quat(target_rotations[i].x, target_rotations[i].y, target_rotations[i].z, target_rotations[i].w));
        }
        
        target_pose.CalculateJointStates();
        it->second->DriveToPoseUsingMotors(target_pose);
    }
}

void JoltPhysicsCore::SetRagdollTargetPose(RagdollHandle handle, const Fmatrix* target_matrices, u32 count) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && target_matrices && m_physics_system) {
        JPH::Ragdoll* ragdoll = it->second;
        
        std::vector<JPH::Mat44> jph_matrices(count);
        for (u32 i = 0; i < count; ++i) {
            const Fmatrix& m = target_matrices[i];
            JPH::Mat44 mat(
                JPH::Vec4(m._11, m._12, m._13, 0.0f),
                JPH::Vec4(m._21, m._22, m._23, 0.0f),
                JPH::Vec4(m._31, m._32, m._33, 0.0f),
                JPH::Vec3(0, 0, 0)
            );
            JPH::Quat q = mat.GetQuaternion();
            JPH::Quat jph_q(-q.GetX(), -q.GetY(), -q.GetZ(), q.GetW());
            JPH::Vec3 jph_pos(m.c.x, m.c.y, m.c.z);
            jph_matrices[i] = JPH::Mat44::sRotationTranslation(jph_q.Normalized(), jph_pos);
        }
        
        // Check if there are kinematic vs dynamic bodies
        bool has_dynamic = false;
        bool has_kinematic = false;
        for (u32 i = 0; i < ragdoll->GetBodyCount(); ++i) {
            JPH::BodyID id = ragdoll->GetBodyID(i);
            if (!id.IsInvalid()) {
                JPH::EMotionType mt = m_physics_system->GetBodyInterface().GetMotionType(id);
                if (mt == JPH::EMotionType::Kinematic) {
                    has_kinematic = true;
                } else if (mt == JPH::EMotionType::Dynamic) {
                    has_dynamic = true;
                }
            }
        }
        
        // 1. Smoothly drive kinematic bodies to the world target pose
        if (has_kinematic) {
            ragdoll->DriveToPoseUsingKinematics(JPH::RVec3::sZero(), jph_matrices.data(), 1.0f / 60.0f);
        }
        
        // 2. Drive dynamic bodies using motors only if dynamic bodies actually exist (prevents 0/0 divide in constraints)
        if (has_dynamic) {
            JPH::SkeletonPose target_pose;
            target_pose.SetSkeleton(m_ragdoll_settings[handle]->mSkeleton);
            u32 j_count = (u32)target_pose.GetJointMatrices().size();
            u32 it_count = (count < j_count) ? count : j_count;
            
            const auto& ragdoll_settings = m_ragdoll_settings[handle];
            for (u32 i = 0; i < it_count; ++i) {
                int parent_idx = ragdoll_settings->mSkeleton->GetJoint(i).mParentJointIndex;
                if (parent_idx == -1 || parent_idx >= (int)count) {
                    target_pose.GetJointMatrices()[i] = jph_matrices[i];
                } else {
                    JPH::Mat44 parent_inv = jph_matrices[parent_idx].InversedRotationTranslation();
                    target_pose.GetJointMatrices()[i] = parent_inv * jph_matrices[i];
                }
            }
            
            target_pose.CalculateJointStates();
            ragdoll->DriveToPoseUsingMotors(target_pose);
        }
    }
}

static inline void MatrixMul43(Fmatrix& dest, const Fmatrix& A, const Fmatrix& B) {
    dest.m[0][0] = A.m[0][0] * B.m[0][0] + A.m[1][0] * B.m[0][1] + A.m[2][0] * B.m[0][2];
    dest.m[0][1] = A.m[0][1] * B.m[0][0] + A.m[1][1] * B.m[0][1] + A.m[2][1] * B.m[0][2];
    dest.m[0][2] = A.m[0][2] * B.m[0][0] + A.m[1][2] * B.m[0][1] + A.m[2][2] * B.m[0][2];
    dest.m[0][3] = 0.0f;

    dest.m[1][0] = A.m[0][0] * B.m[1][0] + A.m[1][0] * B.m[1][1] + A.m[2][0] * B.m[1][2];
    dest.m[1][1] = A.m[0][1] * B.m[1][0] + A.m[1][1] * B.m[1][1] + A.m[2][1] * B.m[1][2];
    dest.m[1][2] = A.m[0][2] * B.m[1][0] + A.m[1][2] * B.m[1][1] + A.m[2][2] * B.m[1][2];
    dest.m[1][3] = 0.0f;

    dest.m[2][0] = A.m[0][0] * B.m[2][0] + A.m[1][0] * B.m[2][1] + A.m[2][0] * B.m[2][2];
    dest.m[2][1] = A.m[0][1] * B.m[2][0] + A.m[1][1] * B.m[2][1] + A.m[2][1] * B.m[2][2];
    dest.m[2][2] = A.m[0][2] * B.m[2][0] + A.m[1][2] * B.m[2][1] + A.m[2][2] * B.m[2][2];
    dest.m[2][3] = 0.0f;

    dest.m[3][0] = A.m[0][0] * B.m[3][0] + A.m[1][0] * B.m[3][1] + A.m[2][0] * B.m[3][2] + A.m[3][0];
    dest.m[3][1] = A.m[0][1] * B.m[3][0] + A.m[1][1] * B.m[3][1] + A.m[2][1] * B.m[3][2] + A.m[3][1];
    dest.m[3][2] = A.m[0][2] * B.m[3][0] + A.m[1][2] * B.m[3][1] + A.m[2][2] * B.m[3][2] + A.m[3][2];
    dest.m[3][3] = 1.0f;
}

void JoltPhysicsCore::SetRagdollWorldPose(RagdollHandle handle, const Fmatrix& world_transform, const Fmatrix* bone_model_matrices, u32 count) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system && bone_model_matrices) {
        JPH::Ragdoll* ragdoll = it->second;
        
        JPH::RVec3 root_offset(world_transform.c.x, world_transform.c.y, world_transform.c.z);
        
        std::vector<JPH::Mat44> jph_matrices(count);
        for (u32 i = 0; i < count; ++i) {
            Fmatrix bone_world;
            MatrixMul43(bone_world, world_transform, bone_model_matrices[i]);
            
            jph_matrices[i] = JPH::Mat44(
                JPH::Vec4(bone_world._11, bone_world._12, bone_world._13, 0.0f),
                JPH::Vec4(bone_world._21, bone_world._22, bone_world._23, 0.0f),
                JPH::Vec4(bone_world._31, bone_world._32, bone_world._33, 0.0f),
                JPH::Vec3(bone_world._41 - root_offset.GetX(), bone_world._42 - root_offset.GetY(), bone_world._43 - root_offset.GetZ())
            );
        }
        
        ragdoll->SetPose(root_offset, jph_matrices.data());
        ragdoll->ResetWarmStart();
        ragdoll->SetLinearAndAngularVelocity(JPH::Vec3::sZero(), JPH::Vec3::sZero());
    }
}

void JoltPhysicsCore::SetRagdollRootKinematic(RagdollHandle handle, bool kinematic) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID root_id = it->second->GetBodyID(0);
        if (!root_id.IsInvalid()) {
            m_physics_system->GetBodyInterface().SetMotionType(root_id, kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
        }
    }
}

void JoltPhysicsCore::SetRagdollRootTransform(RagdollHandle handle, const Fvector& position, const Fquaternion& rotation) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID root_id = it->second->GetBodyID(0);
        if (!root_id.IsInvalid()) {
            m_physics_system->GetBodyInterface().SetPositionAndRotation(root_id, JPH::Vec3(position.x, position.y, position.z), JPH::Quat(-rotation.x, -rotation.y, -rotation.z, rotation.w).Normalized(), JPH::EActivation::DontActivate);
        }
    }
}

void JoltPhysicsCore::SetRagdollMotorState(RagdollHandle handle, bool enabled) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        JPH::EMotorState state = enabled ? JPH::EMotorState::Position : JPH::EMotorState::Off;
        for (u32 i = 0; i < it->second->GetConstraintCount(); ++i) {
            JPH::TwoBodyConstraint* c = it->second->GetConstraint(i);
            if (c && c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
                st->SetSwingMotorState(state);
                st->SetTwistMotorState(state);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollConstraintMotorState(RagdollHandle handle, u32 constraint_index, bool enabled) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        if (constraint_index < it->second->GetConstraintCount()) {
            JPH::TwoBodyConstraint* c = it->second->GetConstraint(constraint_index);
            if (c && c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
                JPH::EMotorState state = enabled ? JPH::EMotorState::Position : JPH::EMotorState::Off;
                st->SetSwingMotorState(state);
                st->SetTwistMotorState(state);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollMotorStiffness(RagdollHandle handle, float stiffness) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        for (u32 i = 0; i < it->second->GetConstraintCount(); ++i) {
            JPH::TwoBodyConstraint* c = it->second->GetConstraint(i);
            if (c && c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
                st->GetSwingMotorSettings().mSpringSettings.mStiffness = stiffness;
                st->GetTwistMotorSettings().mSpringSettings.mStiffness = stiffness;
                st->GetSwingMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
                st->GetTwistMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollMotorDamping(RagdollHandle handle, float damping) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        for (u32 i = 0; i < it->second->GetConstraintCount(); ++i) {
            JPH::TwoBodyConstraint* c = it->second->GetConstraint(i);
            if (c && c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
                st->GetSwingMotorSettings().mSpringSettings.mDamping = damping;
                st->GetTwistMotorSettings().mSpringSettings.mDamping = damping;
                st->GetSwingMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
                st->GetTwistMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollConstraintMotor(RagdollHandle handle, u32 constraint_index, float stiffness, float damping) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        if (constraint_index < it->second->GetConstraintCount()) {
            JPH::TwoBodyConstraint* c = it->second->GetConstraint(constraint_index);
            if (c && c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
                st->GetSwingMotorSettings().mSpringSettings.mStiffness = stiffness;
                st->GetSwingMotorSettings().mSpringSettings.mDamping = damping;
                st->GetTwistMotorSettings().mSpringSettings.mStiffness = stiffness;
                st->GetTwistMotorSettings().mSpringSettings.mDamping = damping;
                st->GetSwingMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
                st->GetTwistMotorSettings().SetTorqueLimits(-250.0f, 250.0f);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollPartMotor(RagdollHandle handle, u32 part_index, float stiffness, float damping) {
    auto it_settings = m_ragdoll_settings.find(handle);
    auto it_ragdoll = m_ragdolls.find(handle);
    if (it_settings != m_ragdoll_settings.end() && it_ragdoll != m_ragdolls.end()) {
        const auto& b2c = it_settings->second->GetBodyIndexToConstraintIndex();
        if (part_index < b2c.size()) {
            int constraint_idx = it_settings->second->GetConstraintIndexForBodyIndex((int)part_index);
            if (constraint_idx >= 0 && (size_t)constraint_idx < it_ragdoll->second->GetConstraintCount()) {
                SetRagdollConstraintMotor(handle, (u32)constraint_idx, stiffness, damping);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollPartMotionType(RagdollHandle handle, u32 part_index, bool kinematic) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid()) {
            m_physics_system->GetBodyInterface().SetMotionType(
                body_id, 
                kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic, 
                JPH::EActivation::Activate
            );
        }
    }
}

void JoltPhysicsCore::SetRagdollAllPartsKinematic(RagdollHandle handle, bool kinematic) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::EMotionType motion_type = kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
        for (u32 i = 0; i < it->second->GetBodyCount(); ++i) {
            JPH::BodyID body_id = it->second->GetBodyID(i);
            if (!body_id.IsInvalid()) {
                m_physics_system->GetBodyInterface().SetMotionType(body_id, motion_type, JPH::EActivation::Activate);
                if (!kinematic) {
                    m_physics_system->GetBodyInterface().SetLinearAndAngularVelocity(body_id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
                }
            }
        }
        if (!kinematic) {
            it->second->ResetWarmStart();
        }
    }
}

void JoltPhysicsCore::SetRagdollUserData(RagdollHandle handle, void* user_data) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::uint64 ud = reinterpret_cast<JPH::uint64>(user_data);
        for (u32 i = 0; i < it->second->GetBodyCount(); ++i) {
            JPH::BodyID id = it->second->GetBodyID(i);
            if (!id.IsInvalid()) {
                m_physics_system->GetBodyInterface().SetUserData(id, ud);
            }
        }
    }
}

void JoltPhysicsCore::SetRagdollGroupID(RagdollHandle handle, u32 group_id) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        it->second->SetGroupID(static_cast<JPH::CollisionGroup::GroupID>(group_id));
    }
}

void JoltPhysicsCore::SetRagdollPartTransform(RagdollHandle handle, u32 part_index, const Fvector& position, const Fquaternion& rotation) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid()) {
            m_physics_system->GetBodyInterface().SetPositionAndRotation(
                body_id, 
                JPH::Vec3(position.x, position.y, position.z), 
                JPH::Quat(-rotation.x, -rotation.y, -rotation.z, rotation.w).Normalized(), 
                JPH::EActivation::DontActivate
            );
        }
    }
}

void JoltPhysicsCore::ApplyRagdollImpulse(RagdollHandle handle, u32 part_index, const Fvector& impulse, const Fvector& point) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid() &&
            m_physics_system->GetBodyInterface().GetMotionType(body_id) == JPH::EMotionType::Dynamic) {
            m_physics_system->GetBodyInterface().AddImpulse(
                body_id, 
                JPH::Vec3(impulse.x, impulse.y, impulse.z), 
                JPH::Vec3(point.x, point.y, point.z)
            );
        }
    }
}

void JoltPhysicsCore::ApplyRagdollLinearImpulse(RagdollHandle handle, u32 part_index, const Fvector& impulse) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid() &&
            m_physics_system->GetBodyInterface().GetMotionType(body_id) == JPH::EMotionType::Dynamic) {
            m_physics_system->GetBodyInterface().AddImpulse(
                body_id, 
                JPH::Vec3(impulse.x, impulse.y, impulse.z)
            );
        }
    }
}

void JoltPhysicsCore::GetRagdollPartTransform(RagdollHandle handle, u32 part_index, Fvector& out_position, Fquaternion& out_rotation) const {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid()) {
            JPH::Vec3 pos = m_physics_system->GetBodyInterface().GetPosition(body_id);
            JPH::Quat rot = m_physics_system->GetBodyInterface().GetRotation(body_id);
            out_position.set(pos.GetX(), pos.GetY(), pos.GetZ());
            out_rotation.set(rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW());
        }
    }
}

void JoltPhysicsCore::GetRagdollAllTransforms(RagdollHandle handle, Fmatrix* out_matrices, u32 count) const {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        u32 max_parts = std::min(count, (u32)it->second->GetBodyCount());
        for (u32 i = 0; i < max_parts; ++i) {
            JPH::BodyID body_id = it->second->GetBodyID(i);
            if (!body_id.IsInvalid()) {
                JPH::Mat44 jph_mat = m_physics_system->GetBodyInterface().GetWorldTransform(body_id);
                Fmatrix& mat = out_matrices[i];
                
                JPH::Vec3 axis_x = jph_mat.GetAxisX();
                JPH::Vec3 axis_y = jph_mat.GetAxisY();
                JPH::Vec3 axis_z = jph_mat.GetAxisZ();
                JPH::Vec3 pos = jph_mat.GetTranslation();
                
                mat.i.set(axis_x.GetX(), axis_x.GetY(), axis_x.GetZ());
                mat.j.set(axis_y.GetX(), axis_y.GetY(), axis_y.GetZ());
                mat.k.set(axis_z.GetX(), axis_z.GetY(), axis_z.GetZ());
                mat.c.set(pos.GetX(), pos.GetY(), pos.GetZ());
                
                mat._14_ = 0.0f; mat._24_ = 0.0f; mat._34_ = 0.0f; mat._44_ = 1.0f;
            }
        }
    }
}

void JoltPhysicsCore::GetRagdollPartVelocity(RagdollHandle handle, u32 part_index, Fvector& out_linear_vel, Fvector& out_angular_vel) const {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        JPH::BodyID body_id = it->second->GetBodyID(part_index);
        if (!body_id.IsInvalid()) {
            JPH::Vec3 l_vel = m_physics_system->GetBodyInterface().GetLinearVelocity(body_id);
            JPH::Vec3 a_vel = m_physics_system->GetBodyInterface().GetAngularVelocity(body_id);
            out_linear_vel.set(l_vel.GetX(), l_vel.GetY(), l_vel.GetZ());
            out_angular_vel.set(a_vel.GetX(), a_vel.GetY(), a_vel.GetZ());
            return;
        }
    }
    out_linear_vel.set(0.f, 0.f, 0.f);
    out_angular_vel.set(0.f, 0.f, 0.f);
}

float JoltPhysicsCore::GetRagdollTotalEnergy(RagdollHandle handle) const {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end() && m_physics_system) {
        float total_energy = 0.0f;
        for (u32 i = 0; i < it->second->GetBodyCount(); ++i) {
            JPH::BodyID body_id = it->second->GetBodyID(i);
            if (!body_id.IsInvalid() && m_physics_system->GetBodyInterface().IsActive(body_id)) {
                JPH::Vec3 v = m_physics_system->GetBodyInterface().GetLinearVelocity(body_id);
                JPH::Vec3 w = m_physics_system->GetBodyInterface().GetAngularVelocity(body_id);
                total_energy += v.LengthSq() + w.LengthSq();
            }
        }
        return total_energy;
    }
    return 0.0f;
}

u32 JoltPhysicsCore::GetRagdollPartCount(RagdollHandle handle) const {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        return (u32)it->second->GetBodyCount();
    }
    return 0;
}

void JoltPhysicsCore::SetRagdollCollisionGroup(RagdollHandle handle, u32 group_id) {
    auto it = m_ragdolls.find(handle);
    if (it != m_ragdolls.end()) {
        it->second->SetGroupID(group_id);
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

void JoltPhysicsCore::GetCharacterVirtualAABB(CharacterVirtualHandle handle, Fvector& center, Fvector& half_extents) const {
    auto it = m_characters.find(handle);
    if (it != m_characters.end()) {
        const JPH::Shape* shape = it->second->GetShape();
        if (shape) {
            JPH::AABox bounds = shape->GetLocalBounds();
            JPH::Mat44 transform = JPH::Mat44::sTranslation(it->second->GetPosition());
            JPH::AABox world_bounds = bounds.Transformed(transform);
            JPH::Vec3 jph_center = world_bounds.GetCenter();
            JPH::Vec3 jph_extents = world_bounds.GetExtent();
            center.set(jph_center.GetX(), jph_center.GetY(), jph_center.GetZ());
            half_extents.set(jph_extents.GetX(), jph_extents.GetY(), jph_extents.GetZ());
            return;
        }
    }
    center.set(0.f, 0.f, 0.f);
    half_extents.set(0.f, 0.f, 0.f);
}

void JoltPhysicsCore::SetCharacterVirtualGravityFactor(CharacterVirtualHandle handle, float factor) {
    if (m_character_gravity_factors.find(handle) != m_character_gravity_factors.end() || factor != 1.0f) {
        m_character_gravity_factors[handle] = factor;
    }
}
