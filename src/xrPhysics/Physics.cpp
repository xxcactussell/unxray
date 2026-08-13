#include "StdAfx.h"
#include <limits>
#include "PHDynamicData.h"
#include "Physics.h"
#include "PHContactBodyEffector.h"
#include "PHCollideValidator.h"
#include "ExtendedGeom.h"
#include "xrPhysicsCore/IPhysicsCore.h" // Наше ядро Jolt

#ifdef DEBUG
#include "debug_output.h"
#endif

extern CPHWorld* ph_world;

const float default_w_limit = 9.8174770f;
const float default_l_limit = 150.f;
const float default_l_scale = 1.01f;
const float default_w_scale = 1.01f;
const float default_k_l = 0.0002f;
const float default_k_w = 0.05f;

extern const u16 max_joint_allowed_for_exeact_integration = 30;

// base params
const float base_fixed_step = 0.02f;
const float base_erp = 0.54545456f;
const float base_cfm = 1.1363636e-006f;
// base params
float fixed_step = 0.01f;
float world_cfm = CFM(SPRING_S(base_cfm, base_erp, base_fixed_step), DAMPING(base_cfm, base_erp));
float world_erp = ERP(SPRING_S(base_cfm, base_erp, base_fixed_step), DAMPING(base_cfm, base_erp));
float world_spring = 1.0f * SPRING(world_cfm, world_erp);
float world_damping = 1.0f * DAMPING(world_cfm, world_erp);

const float default_world_gravity = 2 * 9.81f;

int phIterations = 18;
float phTimefactor = 1.f;
Fbox phBoundaries = {1000.f, 1000.f, -1000.f, -1000.f};

CBlockAllocator<CPHContactBodyEffector, 128> ContactEffectors;

void FixBody(BodyHandle body, float ext_param, float mass_param)
{

}

void FixBody(BodyHandle body) 
{ 
    FixBody(body, fix_ext_param, fix_mass_param); 
}

void BodyCutForce(BodyHandle body, float l_limit, float w_limit)
{

}

float E_NlS(BodyHandle body, const Fvector& norm, float norm_sign) 
{
    // Заглушка. Позже реализуем через GetPhysicsCore()->GetBodyLinearVelocity(body)
    return 0.f; 
}

float E_NLD(BodyHandle b1, BodyHandle b2, const Fvector& norm) 
{
    return 0.f;
}

float E_NL(BodyHandle b1, BodyHandle b2, const Fvector& norm)
{
    VERIFY(b1 != INVALID_BODY_HANDLE || b2 != INVALID_BODY_HANDLE);
    if (b1 != INVALID_BODY_HANDLE)
    {
        if (b2 != INVALID_BODY_HANDLE)
            return E_NLD(b1, b2, norm);
        else
            return E_NlS(b1, norm, 1);
    }
    else
        return E_NlS(b2, norm, -1);
}

void ApplyGravityAccel(BodyHandle body, const Fvector& accel)
{

}
