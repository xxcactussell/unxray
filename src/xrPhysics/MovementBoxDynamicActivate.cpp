#include "StdAfx.h"

#include "MovementBoxDynamicActivate.h"

#include "ExtendedGeom.h"
#include "MathUtils.h"
#include "Physics.h"
#include "IPhysicsShellHolder.h"
#include "PHCharacter.h"
#include "PHWorld.h"
#include "xrPhysicsCore/IPhysicsCore.h"

namespace detail::movement_box
{
    ObjectContactCallbackFun* saved_callback = nullptr;
    static float max_depth = 0.f;
}

struct STestCallbackPars
{
    static float calback_friction_factor;
    static float depth_to_use_force;
    static float callback_force_factor;
    static float depth_to_change_softness_pars;
    static float callback_cfm_factor;
    static float callback_erp_factor;
    static float decrement_depth;
    static float max_real_depth;
};

float STestCallbackPars::calback_friction_factor = 0.0f;
float STestCallbackPars::depth_to_use_force = 0.3f;
float STestCallbackPars::callback_force_factor = 10.f;
float STestCallbackPars::depth_to_change_softness_pars = 0.00f;
float STestCallbackPars::callback_cfm_factor = world_cfm * 0.00001f;
float STestCallbackPars::callback_erp_factor = 1.f;
float STestCallbackPars::decrement_depth = 0.f;
float STestCallbackPars::max_real_depth = 0.2f;

struct STestFootCallbackPars
{
    static float calback_friction_factor;
    static float depth_to_use_force;
    static float callback_force_factor;
    static float depth_to_change_softness_pars;
    static float callback_cfm_factor;
    static float callback_erp_factor;
    static float decrement_depth;
    static float max_real_depth;
};

float STestFootCallbackPars::calback_friction_factor = 0.3f;
float STestFootCallbackPars::depth_to_use_force = 0.3f;
float STestFootCallbackPars::callback_force_factor = 10.f;
float STestFootCallbackPars::depth_to_change_softness_pars = 0.00f;
float STestFootCallbackPars::callback_cfm_factor = world_cfm * 0.00001f;
float STestFootCallbackPars::callback_erp_factor = 1.f;
float STestFootCallbackPars::decrement_depth = 0.05f;
float STestFootCallbackPars::max_real_depth = 0.2f;

template <class Pars>

void TTestDepthCallback(bool& do_colide, bool bo1, CPhysicsGeom* geom1, CPhysicsGeom* geom2, const Fvector& contact_normal, const Fvector& contact_pos, SGameMtl* material_1, SGameMtl* material_2)
{
    using namespace ::detail::movement_box;

    if (saved_callback)
        saved_callback(do_colide, bo1, geom1, geom2, contact_normal, contact_pos, material_1, material_2);

    if (do_colide)
    {
        // Без доступа к реальной глубине, форсируем применение выталкивающей силы
        float test_depth = Pars::depth_to_use_force + 0.1f;
        save_max(max_depth, test_depth);
        
        if (test_depth > Pars::depth_to_use_force)
        {
            float force = Pars::callback_force_factor * ph_world->Gravity();
            BodyHandle b1 = geom1 ? geom1->get_body() : INVALID_BODY_HANDLE;
            BodyHandle b2 = geom2 ? geom2->get_body() : INVALID_BODY_HANDLE;

            if (b1 != INVALID_BODY_HANDLE)
                GetPhysicsCore()->ApplyForce(b1, Fvector().set(contact_normal).mul(force));
            if (b2 != INVALID_BODY_HANDLE)
                GetPhysicsCore()->ApplyForce(b2, Fvector().set(contact_normal).mul(-force));

            if (geom1 && geom1->ph_ref_object)
            {
                CPhysicsShell* phsl = geom1->ph_ref_object->ObjectPPhysicsShell();
                if (phsl) phsl->Enable();
            }

            if (geom2 && geom2->ph_ref_object)
            {
                CPhysicsShell* phsl = geom2->ph_ref_object->ObjectPPhysicsShell();
                if (phsl) phsl->Enable();
            }

            do_colide = false;
        }
    }
}

ObjectContactCallbackFun* TestDepthCallback = &TTestDepthCallback<STestCallbackPars>;
ObjectContactCallbackFun* TestFootDepthCallback = &TTestDepthCallback<STestFootCallbackPars>;

///////////////////////////////////////////////////////////////////////////////////////
class CVelocityLimiter : public CPHUpdateObject
{
    BodyHandle m_body;

public:
    float l_limit;
    float y_limit;

private:
    Fvector m_safe_velocity;
    Fvector m_safe_position;

public:
    CVelocityLimiter(BodyHandle b, float l, float yl)
    {
        R_ASSERT(b != INVALID_BODY_HANDLE);
        m_body = b;
        GetPhysicsCore()->GetBodyLinearVelocity(m_body, m_safe_velocity);
        GetPhysicsCore()->GetBodyPosition(m_body, m_safe_position);
        l_limit = l;
        y_limit = yl;
    }
    
    virtual ~CVelocityLimiter()
    {
        Deactivate();
        m_body = INVALID_BODY_HANDLE;
    }

    bool VelocityLimit()
    {
        Fvector linear_velocity;
        GetPhysicsCore()->GetBodyLinearVelocity(m_body, linear_velocity);
        
        bool ret = false;
        float mag = _sqrt(linear_velocity.x * linear_velocity.x + linear_velocity.z * linear_velocity.z);
        if (mag > l_limit)
        {
            float f = mag / l_limit;
            linear_velocity.x /= f;
            linear_velocity.z /= f;
            ret = true;
        }
        mag = _abs(linear_velocity.y);
        if (mag > y_limit)
        {
            linear_velocity.y = (linear_velocity.y / mag) * y_limit;
            ret = true;
        }
        
        if (ret)
            GetPhysicsCore()->SetBodyLinearVelocity(m_body, linear_velocity);
            
        return ret;
    }
    
    virtual void PhDataUpdate(float step)
    {
        Fvector linear_velocity;
        GetPhysicsCore()->GetBodyLinearVelocity(m_body, linear_velocity);

        if (VelocityLimit())
        {
            // В Jolt позиция и так вычисляется корректно, мы лишь обрезаем скорость.
        }

        GetPhysicsCore()->GetBodyPosition(m_body, m_safe_position);
        GetPhysicsCore()->GetBodyLinearVelocity(m_body, m_safe_velocity);
    }

    virtual void PhTune(float step) { VelocityLimit(); }
};

/////////////////////////////////////////////////////////////////////////////////////
bool ActivateBoxDynamic(IPHMovementControl* mov_control, bool character_exist, u32 id, int num_it /*=8*/,
    int num_steps /*5*/, float resolve_depth /*=0.01f*/)
{
    using namespace ::detail::movement_box;

    VERIFY(mov_control);
    VERIFY(mov_control->character());

    mov_control->character()->CPHObject::activate();
    ph_world->Freeze();
    mov_control->character()->UnFreeze();

    saved_callback = mov_control->character()->ObjectContactCallBack();
    mov_control->character()->SetObjectContactCallback(TestDepthCallback);
    mov_control->character()->SetWheelContactCallback(TestFootDepthCallback);

    max_depth = 0.f;

    if (!character_exist)
    {
        num_it = 20;
        num_steps = 1;
        resolve_depth = 0.1f;
    }
    
    float fnum_it = float(num_it);
    float fnum_steps = float(num_steps);
    float fnum_steps_r = 1.f / fnum_steps;

    float pass = character_exist ? _abs(mov_control->Box().getradius() - mov_control->Boxes()[id].getradius()) :
                                   mov_control->Boxes()[id].getradius();
    float max_vel = pass / 2.f / fnum_it / fnum_steps / fixed_step;
    float max_a_vel = M_PI / 8.f / fnum_it / fnum_steps / fixed_step;
    
    VERIFY(mov_control->character());
    
    BodyHandle char_body = mov_control->character()->get_body();
    if (char_body != INVALID_BODY_HANDLE)
    {
        GetPhysicsCore()->SetBodyLinearVelocity(char_body, Fvector().set(0, 0, 0));
        GetPhysicsCore()->SetBodyAngularVelocity(char_body, Fvector().set(0, 0, 0));
    }

    mov_control->actor_calculate(Fvector().set(0, 0, 0), Fvector().set(1, 0, 0), 0, 0, 0, 0);

    CVelocityLimiter vl(char_body, max_vel, max_vel);
    max_vel = 1.f / fnum_it / fnum_steps / fixed_step;

    bool ret = false;
    mov_control->character()->SwitchOFFInitContact();
    mov_control->character()->SetStaticContactCallBack(0);
    vl.Activate();
    vl.l_limit *= (fnum_it * fnum_steps / 5.f);
    vl.y_limit = vl.l_limit;
    
    for (int m = 0; 30 > m; ++m)
    {
        mov_control->actor_calculate(Fvector().set(0, 0, 0), Fvector().set(1, 0, 0), 0, 0, 0, 0);
        VERIFY(mov_control->character()->b_exist);
        mov_control->character()->Enable();

        mov_control->character()->ApplyForce(0, ph_world->Gravity() * mov_control->character()->Mass(), 0);
        max_depth = 0.f;
        ph_world->Step();
        if (max_depth < resolve_depth)
        {
            break;
        }
        ph_world->CutVelocity(max_vel, max_a_vel);
    }
    vl.l_limit /= (fnum_it * fnum_steps / 5.f);
    vl.y_limit = vl.l_limit;
    
    for (int m = 0; num_steps > m; ++m)
    {
        float param = fnum_steps_r * (1 + m);
        mov_control->InterpolateBox(id, param);
        ret = false;
        for (int i = 0; num_it > i; ++i)
        {
            max_depth = 0.f;
            mov_control->actor_calculate(Fvector().set(0, 0, 0), Fvector().set(1, 0, 0), 0, 0, 0, 0);
            mov_control->character()->Enable();
            mov_control->character()->ApplyForce(0, ph_world->Gravity() * mov_control->character()->Mass(), 0);
            ph_world->Step();
            ph_world->CutVelocity(max_vel, max_a_vel);
            if (max_depth < resolve_depth)
            {
                ret = true;
                break;
            }
        }
        if (!ret)
            break;
    }

    mov_control->character()->SwitchInInitContact();
    mov_control->character()->SetStaticContactCallBack(ph_world->default_character_contact_shotmark());
    vl.Deactivate();

    ph_world->UnFreeze();

    mov_control->character()->SetObjectContactCallback(saved_callback);
    saved_callback = nullptr;

    return ret;
}
