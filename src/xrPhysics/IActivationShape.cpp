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
    return true;
}
