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
    
    u32 test_mask = F.getMask();
    Fvector mM[2];
    mM[0].sub(C, E);
    mM[1].add(C, E);
    
    if (F.testAABB(&mM[0].x, test_mask) == fcvNone)
        return;

    const u32 tris_count = m_def->get_tris_count();
    const TRI* tris = m_def->get_tris();
    const Fvector* verts = m_def->get_verts();
    
    for (u32 i = 0; i < tris_count; ++i)
    {
        const TRI& T = tris[i];
        
        if (frustum_mode & OPT_FULL_TEST)
        {
            sPoly src, dst;
            src.resize(3);
            src[0] = verts[T.verts[0]];
            src[1] = verts[T.verts[1]];
            src[2] = verts[T.verts[2]];
            
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
            Fvector pts[3];
            pts[0] = verts[T.verts[0]];
            pts[1] = verts[T.verts[1]];
            pts[2] = verts[T.verts[2]];
            
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
