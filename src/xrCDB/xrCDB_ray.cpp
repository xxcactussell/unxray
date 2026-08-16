#include "stdafx.h"
#pragma hdrstop

#include "xrCore/_fbox.h"
#include "xrCDB.h"
#include "xrPhysicsCore/IPhysicsCore.h"

namespace CDB
{
void COLLIDER::ray_query(u32 ray_mode, const MODEL* m_def, const Fvector& r_start, const Fvector& r_dir, float r_range)
{
    ZoneScoped;
    m_def->syncronize();
    r_clear();

    if (!_valid(r_start) || !_valid(r_dir) || !_valid(r_range))
        return;

    if (r_range > 100000.f)
        r_range = 100000.f;

    if (!m_def->get_shape_handle()) 
        return;

    CDBRayMode mode = CDBRayMode::All;
    if (ray_mode & OPT_ONLYNEAREST)
        mode = CDBRayMode::Nearest;
    else if (ray_mode & OPT_ONLYFIRST)
        mode = CDBRayMode::First;

    bool cull_backfaces = (ray_mode & OPT_CULL) != 0;

    std::vector<CDBRaycastHit> hits;
    GetPhysicsCore()->RaycastCDBModel(m_def->get_shape_handle(), r_start, r_dir, r_range, mode, cull_backfaces, hits);

    const CDB::TRI* tris = m_def->get_tris();
    const Fvector* verts = m_def->get_verts();

    for (const auto& hit : hits)
    {
        
        RESULT& R = r_add();
        R.id = hit.tri_index;
        R.range = hit.range;

        const CDB::TRI& T = tris[hit.tri_index];
        R.verts[0] = verts[T.verts[0]];
        R.verts[1] = verts[T.verts[1]];
        R.verts[2] = verts[T.verts[2]];
        
        R.dummy = T.dummy;

        Fvector edge1, edge2, tvec, pvec, qvec;
        edge1.sub(R.verts[1], R.verts[0]);
        edge2.sub(R.verts[2], R.verts[0]);
        
        pvec.crossproduct(r_dir, edge2);
        float det = edge1.dotproduct(pvec);
        
        // Строгая защита от деления на ноль
        if (!fis_zero(det))
        {
            float inv_det = 1.0f / det;
            
            tvec.sub(r_start, R.verts[0]);
            R.u = tvec.dotproduct(pvec) * inv_det;
            
            qvec.crossproduct(tvec, edge1);
            R.v = r_dir.dotproduct(qvec) * inv_det;
        }
        else
        {
            R.u = 0.f;
            R.v = 0.f;
        }
    }
}
} // namespace CDB
