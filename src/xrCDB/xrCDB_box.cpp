#include "stdafx.h"
#pragma hdrstop

#include "xrCDB.h"

namespace CDB
{
void COLLIDER::box_query(u32 box_mode, const MODEL* m_def, const Fvector& b_center, const Fvector& b_dim)
{
    ZoneScoped;
    m_def->syncronize();
    r_clear();
    
    if (!m_def->get_shape_handle())
        return;

    CDBRayMode mode = (box_mode & OPT_ONLYFIRST) ? CDBRayMode::First : CDBRayMode::All;
    bool cull_backfaces = (box_mode & OPT_CULL) != 0;

    std::vector<u32> hit_indices;
    GetPhysicsCore()->BoxQueryCDB(m_def->get_shape_handle(), b_center, b_dim, mode, cull_backfaces, hit_indices);

    const CDB::TRI* tris = m_def->get_tris();
    const Fvector* verts = m_def->get_verts();

    for (u32 tri_index : hit_indices)
    {
        RESULT& R = r_add();
        R.id = tri_index;
        
        const CDB::TRI& T = tris[tri_index];
        R.verts[0] = verts[T.verts[0]];
        R.verts[1] = verts[T.verts[1]];
        R.verts[2] = verts[T.verts[2]];
        
        R.dummy = T.dummy;
    }
}
} // namespace CDB
