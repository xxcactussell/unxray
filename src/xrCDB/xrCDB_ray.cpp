#include "stdafx.h"
#pragma hdrstop

#include "xrCore/_fbox.h"
#include "xrCDB.h"
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

namespace CDB
{

class XRCDB_CastRayCollector : public JPH::CastRayCollector
{
public:
    COLLIDER* dest;
    const MODEL* m_def;
    u32 ray_mode;
    float rRange;

    XRCDB_CastRayCollector(COLLIDER* CL, const MODEL* M, u32 mode, float R)
        : dest(CL), m_def(M), ray_mode(mode), rRange(R)
    {
    }

    virtual void AddHit(const JPH::RayCastResult &inResult) override
    {
        float r = inResult.mFraction * rRange;
        
        const JPH::MeshShape* meshShape = static_cast<const JPH::MeshShape*>(m_def->get_shape());
        u32 prim = meshShape->GetTriangleUserData(inResult.mSubShapeID2);

        if (ray_mode & OPT_ONLYNEAREST)
        {
            if (dest->r_count())
            {
                RESULT& R = *dest->r_begin();
                if (r < R.range)
                {
                    R.id = prim;
                    R.range = r;
                    // Jolt doesn't return u,v directly in CastRayResult. 
                    // We can compute them if needed, or leave 0 since most engine code doesn't strictly use u,v from physics, or we calculate it.
                    // For now, calculating u,v requires getting the vertices and ray.
                    R.u = 0.f;
                    R.v = 0.f;
                    R.verts[0] = m_def->get_verts()[m_def->get_tris()[prim].verts[0]];
                    R.verts[1] = m_def->get_verts()[m_def->get_tris()[prim].verts[1]];
                    R.verts[2] = m_def->get_verts()[m_def->get_tris()[prim].verts[2]];
                    R.dummy = m_def->get_tris()[prim].dummy;
                    UpdateEarlyOutFraction(inResult.mFraction);
                }
            }
            else
            {
                RESULT& R = dest->r_add();
                R.id = prim;
                R.range = r;
                R.u = 0.f;
                R.v = 0.f;
                R.verts[0] = m_def->get_verts()[m_def->get_tris()[prim].verts[0]];
                R.verts[1] = m_def->get_verts()[m_def->get_tris()[prim].verts[1]];
                R.verts[2] = m_def->get_verts()[m_def->get_tris()[prim].verts[2]];
                R.dummy = m_def->get_tris()[prim].dummy;
                UpdateEarlyOutFraction(inResult.mFraction);
            }
        }
        else
        {
            RESULT& R = dest->r_add();
            R.id = prim;
            R.range = r;
            R.u = 0.f;
            R.v = 0.f;
            R.verts[0] = m_def->get_verts()[m_def->get_tris()[prim].verts[0]];
            R.verts[1] = m_def->get_verts()[m_def->get_tris()[prim].verts[1]];
            R.verts[2] = m_def->get_verts()[m_def->get_tris()[prim].verts[2]];
            R.dummy = m_def->get_tris()[prim].dummy;
            
            if (ray_mode & OPT_ONLYFIRST)
            {
                ForceEarlyOut();
            }
        }
    }
};

void COLLIDER::ray_query(u32 ray_mode, const MODEL* m_def, const Fvector& r_start, const Fvector& r_dir, float r_range)
{
    ZoneScoped;
    m_def->syncronize();

    r_clear();
    
    if (!m_def->get_shape())
        return;

    JPH::RayCast ray(JPH::Vec3(r_start.x, r_start.y, r_start.z), JPH::Vec3(r_dir.x * r_range, r_dir.y * r_range, r_dir.z * r_range));
    JPH::RayCastSettings settings;

    // Use backface culling if OPT_CULL is set
    if (ray_mode & OPT_CULL)
        settings.mBackFaceModeTriangles = JPH::EBackFaceMode::IgnoreBackFaces;
    else
        settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;

    settings.mTreatConvexAsSolid = false;

    XRCDB_CastRayCollector collector(this, m_def, ray_mode, r_range);
    
    // We cast against the shape directly.
    m_def->get_shape()->CastRay(ray, settings, JPH::SubShapeIDCreator(), collector);
    
    // Calculate u, v manually for the results if needed by X-Ray
    for (size_t i = 0; i < rd.size(); ++i)
    {
        RESULT& R = rd[i];
        Fvector edge1, edge2, tvec, pvec, qvec;
        edge1.sub(R.verts[1], R.verts[0]);
        edge2.sub(R.verts[2], R.verts[0]);
        pvec.crossproduct(r_dir, edge2);
        float det = edge1.dotproduct(pvec);
        float inv_det = 1.0f / det;
        tvec.sub(r_start, R.verts[0]);
        R.u = tvec.dotproduct(pvec) * inv_det;
        qvec.crossproduct(tvec, edge1);
        R.v = r_dir.dotproduct(qvec) * inv_det;
    }
}
} // namespace CDB
