#include "stdafx.h"
#include "xrPhysicsCore.h"
#include "xrCDB.h"

namespace xrPhysicsCore
{
    static std::once_flag s_JoltInitFlag;

    static void InitializeJolt()
    {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }

    XRPHYSICScore_API void EnsureInitialized()
    {
        std::call_once(s_JoltInitFlag, InitializeJolt);
    }

    XRPHYSICScore_API const JPH::Shape* CreateMeshShape(const Fvector* verts, u32 verts_count, const CDB::TRI* tris, u32 tris_count)
    {
        EnsureInitialized();

        JPH::VertexList vertices;
        vertices.reserve(verts_count);
        for (u32 i = 0; i < verts_count; ++i)
        {
            vertices.push_back(JPH::Float3(verts[i].x, verts[i].y, verts[i].z));
        }

        JPH::IndexedTriangleList indices;
        indices.reserve(tris_count);
        for (u32 i = 0; i < tris_count; ++i)
        {
            indices.push_back(JPH::IndexedTriangle(tris[i].verts[0], tris[i].verts[1], tris[i].verts[2], 0, i));
        }

        JPH::MeshShapeSettings settings(vertices, indices);
        settings.mPerTriangleUserData = true;

        JPH::ShapeSettings::ShapeResult result = settings.Create();
        if (!result.IsValid())
        {
            Msg("! xrPhysicsCore: Jolt MeshShape build failed: %s", result.GetError().c_str());
            return nullptr;
        }

        const JPH::Shape* shape = result.Get().GetPtr();
        shape->AddRef();
        return shape;
    }

    void ReleaseShape(const JPH::Shape* shape)
    {
        if (shape)
        {
            shape->Release();
        }
    }
}
