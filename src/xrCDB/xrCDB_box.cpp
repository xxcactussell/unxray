#include "stdafx.h"
#pragma hdrstop

#include "xrCDB.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>

namespace CDB
{

class XRCDB_CollideShapeCollector : public JPH::CollideShapeCollector
{
public:
    COLLIDER* dest;
    const MODEL* m_def;
    u32 box_mode;

    XRCDB_CollideShapeCollector(COLLIDER* CL, const MODEL* M, u32 mode)
        : dest(CL), m_def(M), box_mode(mode)
    {
    }

    virtual void AddHit(const JPH::CollideShapeResult &inResult) override
    {
        const JPH::MeshShape* meshShape = static_cast<const JPH::MeshShape*>(m_def->get_shape());
        // Box is shape 1 (so SubShapeID1 is the box), mesh is shape 2
        u32 prim = meshShape->GetTriangleUserData(inResult.mSubShapeID2);

        RESULT& R = dest->r_add();
        R.id = prim;
        R.verts[0] = m_def->get_verts()[m_def->get_tris()[prim].verts[0]];
        R.verts[1] = m_def->get_verts()[m_def->get_tris()[prim].verts[1]];
        R.verts[2] = m_def->get_verts()[m_def->get_tris()[prim].verts[2]];
        R.dummy = m_def->get_tris()[prim].dummy;

        if (box_mode & OPT_ONLYFIRST)
        {
            ForceEarlyOut();
        }
    }
};

void COLLIDER::box_query(u32 box_mode, const MODEL* m_def, const Fvector& b_center, const Fvector& b_dim)
{
    ZoneScoped;
    m_def->syncronize();

    r_clear();
    
    if (!m_def->get_shape())
        return;

    // Create a Jolt box shape
    JPH::BoxShape box(JPH::Vec3(b_dim.x, b_dim.y, b_dim.z));
    
    JPH::CollideShapeSettings settings;
    // X-Ray OPT_CULL historically meant backface culling, but for boxes against meshes, Jolt handles it via settings
    if (box_mode & OPT_CULL)
        settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
    else
        settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

    // Center of mass transform for the box
    JPH::Mat44 boxTransform = JPH::Mat44::sTranslation(JPH::Vec3(b_center.x, b_center.y, b_center.z));
    // Center of mass transform for the mesh (identity)
    JPH::Mat44 meshTransform = JPH::Mat44::sIdentity();

    XRCDB_CollideShapeCollector collector(this, m_def, box_mode);
    
    // Collide shape 1 (box) vs shape 2 (mesh)
    JPH::CollisionDispatch::sCollideShapeVsShape(
        &box, m_def->get_shape(), 
        JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f), 
        boxTransform, meshTransform, 
        JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), 
        settings, collector, JPH::ShapeFilter()
    );
}

} // namespace CDB
