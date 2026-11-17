#include "stdafx.h"
#pragma hdrstop

#include "xrCDB.h"
#include "Frustum.h"
#include "xrPhysicsCore/IPhysicsCore.h"

namespace CDB
{

void COLLIDER::frustum_query(u32 frustum_mode, const MODEL* m_def, const CFrustum& F)
{
    ZoneScoped;
    m_def->syncronize();
    r_clear();
    
    if (!m_def->get_shape_handle())
        return;

    Fvector C, E;
    GetPhysicsCore()->GetCDBModelBounds(m_def->get_shape_handle(), C, E);
    float extents_sq_len = (E.x * E.x) + (E.y * E.y) + (E.z * E.z);
    
    if (extents_sq_len > 0.0001f)
    {
        u32 test_mask = F.getMask();
        Fvector mM[2];
        mM[0].sub(C, E);
        mM[1].add(C, E);
        
        if (F.testAABB(&mM[0].x, test_mask) == fcvNone)
            return;
    }

    const u32 tris_count = m_def->get_tris_count();
    const TRI* tris = m_def->get_tris();
    const Fvector* verts = m_def->get_verts();
    const u32 frustum_mask = F.getMask();
    
    for (u32 i = 0; i < tris_count; ++i)
    {
        const TRI& T = tris[i];
        const Fvector& v0 = verts[T.verts[0]];
        const Fvector& v1 = verts[T.verts[1]];
        const Fvector& v2 = verts[T.verts[2]];

        // Fast bounding box rejection against frustum planes
        float min_x = _min(_min(v0.x, v1.x), v2.x);
        float min_y = _min(_min(v0.y, v1.y), v2.y);
        float min_z = _min(_min(v0.z, v1.z), v2.z);

        float max_x = _max(_max(v0.x, v1.x), v2.x);
        float max_y = _max(_max(v0.y, v1.y), v2.y);
        float max_z = _max(_max(v0.z, v1.z), v2.z);

        float mM[6] = { min_x, min_y, min_z, max_x, max_y, max_z };
        u32 mask = frustum_mask;
        EFC_Visible vis = F.testAABB(mM, mask);
        if (vis == fcvNone)
            continue;

        if (vis == fcvFully)
        {
            RESULT& R = r_add();
            R.id = i;
            R.verts[0] = v0;
            R.verts[1] = v1;
            R.verts[2] = v2;
            R.dummy = T.dummy;

            if (frustum_mode & OPT_ONLYFIRST)
                break;
            continue;
        }

        // Boundary triangle (fcvPartial): detailed polygon test
        if (frustum_mode & OPT_FULL_TEST)
        {
            sPoly src, dst;
            src.resize(3);
            src[0] = v0;
            src[1] = v1;
            src[2] = v2;
            
            if (F.ClipPoly(src, dst))
            {
                RESULT& R = r_add();
                R.id = i;
                R.verts[0] = src[0]; 
                R.verts[1] = src[1];
                R.verts[2] = src[2];
                R.dummy = T.dummy;
                
                if (frustum_mode & OPT_ONLYFIRST)
                    break;
            }
        }
        else
        {
            Fvector pts[3] = { v0, v1, v2 };
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
