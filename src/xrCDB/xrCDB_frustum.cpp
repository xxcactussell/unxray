#include "stdafx.h"
#pragma hdrstop

#include "xrCDB.h"
#include "Frustum.h"
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace CDB
{

void COLLIDER::frustum_query(u32 frustum_mode, const MODEL* m_def, const CFrustum& F)
{
    ZoneScoped;
    m_def->syncronize();
    r_clear();
    
    if (!m_def->shape)
        return;

    // Fast-path rejection using model bounds
    JPH::AABox bounds = m_def->shape->GetLocalBounds();
    Fvector C, E;
    C.set(bounds.GetCenter().GetX(), bounds.GetCenter().GetY(), bounds.GetCenter().GetZ());
    E.set(bounds.GetExtent().GetX(), bounds.GetExtent().GetY(), bounds.GetExtent().GetZ());
    
    u32 test_mask = F.getMask();
    EFC_Visible res = F.testAABB(&C.x, test_mask); // Note: CFrustum expects center ptr or mM? Wait, testAABB takes mM[6] which is [minX, minY, minZ, maxX, maxY, maxZ]!
    
    // Actually testAABB takes mM which is [minx, miny, minz, maxx, maxy, maxz] in Frustum.h:
    // "Fvector mM[2]; mM[0].sub(C, E); mM[1].add(C, E);"
    Fvector mM[2];
    mM[0].sub(C, E);
    mM[1].add(C, E);
    if (F.testAABB(&mM[0].x, test_mask) == fcvNone)
        return;

    // Linear fallback since we do not build an AABB tree for frustum queries specifically
    // HOM meshes are usually small enough that this doesn't bottleneck.
    // for (u32 i = 0; i < m_def->verts_count / 3; /*wait, tris_count*/ ) {}
    
    for (u32 i = 0; i < m_def->tris_count; ++i)
    {
        const TRI& T = m_def->tris[i];
        
        if (frustum_mode & OPT_FULL_TEST)
        {
            sPoly src, dst;
            src.resize(3);
            src[0] = m_def->verts[T.verts[0]];
            src[1] = m_def->verts[T.verts[1]];
            src[2] = m_def->verts[T.verts[2]];
            
            if (F.ClipPoly(src, dst))
            {
                RESULT& R = r_add();
                R.id = i;
                R.verts[0] = src[0]; // wait, originally it just adds the original verts!
                R.verts[1] = src[1];
                R.verts[2] = src[2];
                R.dummy = T.dummy;
                if (frustum_mode & OPT_ONLYFIRST)
                    break;
            }
        }
        else
        {
            Fvector pts[3];
            pts[0] = m_def->verts[T.verts[0]];
            pts[1] = m_def->verts[T.verts[1]];
            pts[2] = m_def->verts[T.verts[2]];
            
            if (F.testPolyInside(pts, 3))
            {
                RESULT& R = r_add();
                R.id = i;
                R.verts[0] = pts[0];
                R.verts[1] = pts[1];
                R.verts[2] = pts[2];
                R.dummy = T.dummy;
                if (frustum_mode & OPT_ONLYFIRST)
                    break;
            }
        }
    }
}

} // namespace CDB
