#include "stdafx.h"
#include "xr_area.h"
#include "Frustum.h"
#include "xrCore/_vector3d_ext.h"
#include "xrCDB.h"

#include "xrPhysicsCore/IPhysicsCore.h" 

using namespace collide;

bool CObjectSpace::BoxQuery(Fvector const& box_center, Fvector const& box_z_axis, Fvector const& box_y_axis,
    Fvector const& box_sizes, xr_vector<Fvector>* out_tris)
{
    ZoneScoped;

    if (!Static.get_shape_handle())
        return false;

    std::vector<u32> hit_indices;
    
    bool has_hit = GetPhysicsCore()->BoxQueryCDB(
        Static.get_shape_handle(), 
        box_center, box_z_axis, box_y_axis, box_sizes, 
        hit_indices
    );

    if (has_hit && out_tris)
    {
        const CDB::TRI* tris = Static.get_tris();
        const Fvector* verts = Static.get_verts();

        for (u32 prim_index : hit_indices) 
        {
            const CDB::TRI& tri = tris[prim_index];
            out_tris->push_back(verts[tri.verts[0]]);
            out_tris->push_back(verts[tri.verts[1]]);
            out_tris->push_back(verts[tri.verts[2]]);
        }
    }

    return has_hit;
}
