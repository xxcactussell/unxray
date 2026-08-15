#include "StdAfx.h"
#include "PhysicsExternalCommon.h"
#include "ExtendedGeom.h"
#include "Geometry.h"
#include "xrPhysicsCore/IPhysicsCore.h"

bool ContactShotMarkGetEffectPars(const Fvector& pos, const Fvector& normal, CPhysicsGeom* g1, CPhysicsGeom* g2, CPhysicsGeom*& data, float& vel_cret, bool& b_invert_normal)
{
    if (!g1 || !g2) return false;

    CharacterVirtualHandle b = g1->get_body();
    b_invert_normal = false;
    
    if (b == INVALID_CHARACTER_VIRTUAL_HANDLE)
    {
        b = g2->get_body();
        data = g2;
        b_invert_normal = true;
    }
    else
    {
        data = g1;
    }
    
    if (b == INVALID_CHARACTER_VIRTUAL_HANDLE)
        return false;

    Fvector vel;
    float mass = GetPhysicsCore()->GetBodyMass(b);
    
    // Получаем скорость тела в конкретной точке контакта
    GetPhysicsCore()->GetBodyPointVelocity(b, pos, vel);
    
    vel_cret = _abs(vel.dotproduct(normal)) * _sqrt(mass);
    return true;
}
