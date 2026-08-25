#include "StdAfx.h"
#include "IActivationShape.h"
#include "PHActivationShape.h"
#include "Physics.h"
#include "IPhysicsShellHolder.h"
#include "PHCollideValidator.h"
#include "xrPhysicsCore/IPhysicsCore.h"
#include <cmath>

void ActivateShapeExplosive(IPhysicsShellHolder* self_obj, const Fvector& size, Fvector& out_size, Fvector& in_out_pos)
{
    out_size.set(size);
}

void ActivateShapePhysShellHolder(
    IPhysicsShellHolder* obj, const Fmatrix& in_xform, const Fvector& in_size, Fvector& in_pos, Fvector& out_pos)
{
    out_pos = in_pos;
}

bool ActivateShapeCharacterPhysicsSupport(Fvector& out_pos, const Fvector& vbox, const Fvector& activation_pos,
    const Fmatrix& mXFORM, bool not_collide_characters, bool set_rotation, IPhysicsShellHolder* m_EntityAlife)
{
    out_pos.set(activation_pos);

    if (!GetPhysicsCore())
        return true;

    // Create a temporary capsule shape matching character size
    float radius = std::min(vbox.x, vbox.z) / 2.f;
    float cyl_height = vbox.y - 2.f * radius;
    if (cyl_height < 0.f)
        cyl_height = 0.01f;

    PhysicsShapeHandle shape = GetPhysicsCore()->CreateCapsuleShape(radius, cyl_height / 2.f);
    shape = GetPhysicsCore()->CreateRotatedTranslatedShape(shape, Fvector().set(0.f, cyl_height / 2.f, 0.f), Fquaternion().identity());

    Fquaternion rot;
    rot.identity();

    void* ignore_user_data = m_EntityAlife;
    bool check_characters = !not_collide_characters;

    Fvector free_pos;
    bool found = GetPhysicsCore()->FindFreeShapePlacement(shape, activation_pos, rot, free_pos, 3.0f, 16, check_characters, ignore_user_data);
    if (found)
    {
        out_pos.set(free_pos);
    }

    return found;
}
