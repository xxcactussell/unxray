#pragma once

#include "Common/Common.hpp"
#include "xrCore/xrCore.h"

namespace CDB
{
    class TRI;
}

#ifdef XRPHYSICSCORE_EXPORTS
#   define XRPHYSICScore_API XR_EXPORT
#else
#   define XRPHYSICScore_API XR_IMPORT
#endif

namespace xrPhysicsCore
{
    XRPHYSICScore_API void EnsureInitialized();
    XRPHYSICScore_API const JPH::Shape* CreateMeshShape(const Fvector* verts, u32 verts_count, const CDB::TRI* tris, u32 tris_count);
    XRPHYSICScore_API void ReleaseShape(const JPH::Shape* shape);
}
