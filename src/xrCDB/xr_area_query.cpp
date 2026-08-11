#include "stdafx.h"
#include "xr_area.h"
#include "Frustum.h"

#include "xrCore/_vector3d_ext.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include "xrCDB.h"

using namespace collide;

class XRCDB_OBBCollector : public JPH::CollideShapeCollector
{
public:
    xr_vector<Fvector>* out_tris;
    const CDB::MODEL* m_def;
    bool has_hit;

    XRCDB_OBBCollector(xr_vector<Fvector>* tris, const CDB::MODEL* M)
        : out_tris(tris), m_def(M), has_hit(false)
    {
    }

    virtual void AddHit(const JPH::CollideShapeResult &inResult) override
    {
        has_hit = true;
        if (out_tris)
        {
            const JPH::MeshShape* meshShape = static_cast<const JPH::MeshShape*>(m_def->get_shape());
            u32 prim = meshShape->GetTriangleUserData(inResult.mSubShapeID2);
            out_tris->push_back(m_def->get_verts()[m_def->get_tris()[prim].verts[0]]);
            out_tris->push_back(m_def->get_verts()[m_def->get_tris()[prim].verts[1]]);
            out_tris->push_back(m_def->get_verts()[m_def->get_tris()[prim].verts[2]]);
        }
    }
};

bool CObjectSpace::BoxQuery(Fvector const& box_center, Fvector const& box_z_axis, Fvector const& box_y_axis,
    Fvector const& box_sizes, xr_vector<Fvector>* out_tris)
{
    ZoneScoped;

    Fvector z_axis = box_z_axis;
    z_axis.normalize();
    Fvector y_axis = box_y_axis;
    y_axis.normalize();
    Fvector x_axis;
    x_axis.crossproduct(box_y_axis, box_z_axis).normalize();

    if (!Static.get_shape())
        return false;

    // Construct Jolt BoxShape
    JPH::BoxShape box(JPH::Vec3(box_sizes.x * 0.5f, box_sizes.y * 0.5f, box_sizes.z * 0.5f));
    
    // Rotation matrix from axis
    JPH::Mat44 rot(
        JPH::Vec4(x_axis.x, x_axis.y, x_axis.z, 0.0f),
        JPH::Vec4(y_axis.x, y_axis.y, y_axis.z, 0.0f),
        JPH::Vec4(z_axis.x, z_axis.y, z_axis.z, 0.0f),
        JPH::Vec4(box_center.x, box_center.y, box_center.z, 1.0f)
    );

    JPH::CollideShapeSettings settings;
    settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

    XRCDB_OBBCollector collector(out_tris, &Static);

    JPH::CollisionDispatch::sCollideShapeVsShape(
        &box, Static.get_shape(), 
        JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f), 
        rot, JPH::Mat44::sIdentity(), 
        JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), 
        settings, collector, JPH::ShapeFilter()
    );

    return collector.has_hit;
}
