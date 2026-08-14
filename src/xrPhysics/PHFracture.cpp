#include "StdAfx.h"
#include "PHFracture.h"
#include "Physics.h"
#include "PHElement.h"
#include "PHShell.h"
#include "console_vars.h"
#include "PHJointDestroyInfo.h"
#include "Include/xrRender/Kinematics.h"
#include "xrCore/Animation/Bone.hpp"
#include "xrPhysicsCore/IPhysicsCore.h"

extern class CPHWorld* ph_world;
static const float torque_factor = 10000000.f;

CPHFracturesHolder::CPHFracturesHolder() { m_has_breaks = false; }

CPHFracturesHolder::~CPHFracturesHolder()
{
    m_has_breaks = false;
    m_fractures.clear();
    m_impacts.clear();
    m_feedbacks.clear();
}

void CPHFracturesHolder::ApplyImpactsToElement(CPHElement* E)
{
    auto i = m_impacts.begin(), e = m_impacts.end();
    BOOL ac_state = E->isActive();
    E->m_flags.set(CPHElement::flActive, TRUE);
    for (; e != i; ++i)
    {
        E->applyImpact(*i);
    }
    E->m_flags.set(CPHElement::flActive, ac_state);
}

element_fracture CPHFracturesHolder::SplitFromEnd(CPHElement* element, u16 fracture)
{
    FRACTURE_I fract_i = m_fractures.begin() + fracture;
    u16 geom_num = fract_i->m_start_geom_num;
    u16 end_geom_num = fract_i->m_end_geom_num;
    SubFractureMass(fracture);

    CPHElement* new_element = cast_PHElement(P_create_Element());
    new_element->m_SelfID = fract_i->m_bone_id;
    new_element->mXFORM.set(element->mXFORM);
    element->PassEndGeoms(geom_num, end_geom_num, new_element);
    
    IKinematics* pKinematics = element->m_shell->PKinematics();
    const CBoneInstance& new_bi = pKinematics->LL_GetBoneInstance(new_element->m_SelfID);
    const CBoneInstance& old_bi = pKinematics->LL_GetBoneInstance(element->m_SelfID);

    Fmatrix shift_pivot;
    shift_pivot.set(new_bi.mTransform);
    shift_pivot.invert();
    shift_pivot.mulB_43(old_bi.mTransform);
    
    float density = element->getDensity();
    new_element->SetShell(element->PHShell());
    Fmatrix current_transtform;
    element->GetGlobalTransformDynamic(&current_transtform);
    InitNewElement(new_element, shift_pivot, density);
    
    Fmatrix shell_form;
    element->PHShell()->GetGlobalTransformDynamic(&shell_form);
    current_transtform.mulA_43(shell_form);
    new_element->SetTransform(current_transtform, mh_unspecified);

    ApplyImpactsToElement(new_element);

    element_fracture ret = std::make_pair(new_element, (CShellSplitInfo)(*fract_i));

    if (m_fractures.size() - fracture > 0)
    {
        if (new_element->m_fratures_holder == NULL) 
        {
            new_element->m_fratures_holder = xr_new<CPHFracturesHolder>();
        }
        PassEndFractures(fracture, new_element);
    }

    return ret;
}

void CPHFracturesHolder::PassEndFractures(u16 from, CPHElement* dest)
{
    FRACTURE_I i = m_fractures.begin(), i_from = m_fractures.begin() + from, e = m_fractures.end();
    u16 end_geom = i_from->m_end_geom_num;
    u16 begin_geom_num = i_from->m_start_geom_num;
    u16 leaved_geoms = begin_geom_num;
    u16 passed_geoms = end_geom - begin_geom_num;
    if (i_from == e)
        return;

    for (; i != i_from; ++i) 
    {
        u16& cur_end_geom = i->m_end_geom_num;
        if (cur_end_geom > begin_geom_num)
            cur_end_geom = cur_end_geom - passed_geoms;
    }

    ++i; 
    for (; i != e; ++i) 
    {
        u16& cur_end_geom = i->m_end_geom_num;
        u16& cur_geom = i->m_start_geom_num;
        if (cur_geom >= end_geom)
            break;
        cur_end_geom = cur_end_geom - leaved_geoms;
        cur_geom = cur_geom - leaved_geoms;
    }
    FRACTURE_I i_to = i;
    for (; i != e; ++i) 
    {
        u16& cur_end_geom = i->m_end_geom_num;
        u16& cur_geom = i->m_start_geom_num;
        cur_end_geom = cur_end_geom - passed_geoms;
        cur_geom = cur_geom - passed_geoms;
    }

    if (i_from + 1 != i_to) 
    {
        CPHFracturesHolder*& dest_fract_holder = dest->m_fratures_holder;
        if (!dest_fract_holder)
            dest_fract_holder = xr_new<CPHFracturesHolder>();
        dest_fract_holder->m_fractures.insert(dest_fract_holder->m_fractures.end(), i_from + 1, i_to);
    }
    m_fractures.erase(i_from, i_to); 
}

void CPHFracturesHolder::SplitProcess(CPHElement* element, ELEMENT_PAIR_VECTOR& new_elements)
{
    u16 i = u16(m_fractures.size() - 1);

    for (; i != u16(-1); i--)
    {
        if (m_fractures[i].Breaked())
        {
            new_elements.push_back(SplitFromEnd(element, i));
        }
    }
}

void CPHFracturesHolder::InitNewElement(CPHElement* element, const Fmatrix& shift_pivot, float density)
{
    element->CreateSimulBase();
    element->ReInitDynamics(shift_pivot, density);
    VERIFY(element->get_body() != INVALID_BODY_HANDLE);
}

void CPHFracturesHolder::PhTune(BodyHandle body)
{
    // Обратная связь суставов в Jolt обрабатывается автоматически на уровне Constraints.
}

bool CPHFracturesHolder::PhDataUpdate(CPHElement* element)
{
    FRACTURE_I i = m_fractures.begin(), e = m_fractures.end();
    for (; i != e; ++i)
    {
        m_has_breaks = i->Update(element) || m_has_breaks;
    }
    if (!m_has_breaks)
        m_impacts.clear();
    return m_has_breaks;
}

void CPHFracturesHolder::AddImpact(const Fvector& force, const Fvector& point, u16 id)
{
    m_impacts.push_back(SPHImpact(force, point, id));
}

u16 CPHFracturesHolder::AddFracture(const CPHFracture& fracture)
{
    m_fractures.push_back(fracture);
    return u16(m_fractures.size() - 1);
}

CPHFracture& CPHFracturesHolder::Fracture(u16 num)
{
    R_ASSERT2(num < m_fractures.size(), "out of range!");
    return m_fractures[num];
}

void CPHFracturesHolder::DistributeAdditionalMass(u16 geom_num, float m)
{
    FRACTURE_I f_i = m_fractures.begin(), f_e = m_fractures.end();
    for (; f_i != f_e; ++f_i)
    {
        R_ASSERT2(u16(-1) != f_i->m_start_geom_num, "fracture does not initialized!");

        if (f_i->m_end_geom_num == u16(-1))
            f_i->MassAddToSecond(m);
        else
            f_i->MassAddToFirst(m);
    }
}

void CPHFracturesHolder::SubFractureMass(u16 fracture_num)
{
    FRACTURE_I f_i = m_fractures.begin(), f_e = m_fractures.end();
    FRACTURE_I fracture = f_i + fracture_num;
    u16 start_geom = fracture->m_start_geom_num;
    u16 end_geom = fracture->m_end_geom_num;
    float second_mass = fracture->m_secondM;
    float first_mass = fracture->m_firstM;
    
    for (; f_i != f_e; ++f_i)
    {
        if (f_i == fracture)
            continue;
        R_ASSERT2(start_geom != f_i->m_start_geom_num, "Double fracture!!!");

        if (start_geom > f_i->m_start_geom_num)
        {
            if (end_geom <= f_i->m_end_geom_num)
                f_i->MassSubFromSecond(second_mass); 
            else
            {
                R_ASSERT2(start_geom >= f_i->m_end_geom_num, "Odd fracture!!!");
                f_i->MassSubFromFirst(second_mass); 
            }
        }
        else
        {
            if (end_geom >= f_i->m_end_geom_num)
                f_i->MassSubFromFirst(first_mass); 
            else
            {
                R_ASSERT2(end_geom <= f_i->m_start_geom_num, "Odd fracture!!!");
                f_i->MassSubFromFirst(second_mass); 
            }
        }
    }
}

CPHFracture::CPHFracture()
{
    m_start_geom_num = u16(-1);
    m_end_geom_num = u16(-1);
    m_breaked = false;
    m_firstM = 0.f;
    m_secondM = 0.f;
    m_break_force = 0.f;
    m_break_torque = 0.f;
    m_add_torque_z = 0.f;
}

bool CPHFracture::Update(CPHElement* element)
{
    BodyHandle body = element->get_body();
    if (body == INVALID_BODY_HANDLE) return false;

    CPHFracturesHolder* holder = element->FracturesHolder();
    PH_IMPACT_STORAGE& impacts = holder->Impacts();

    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(body, transform);
    Fvector body_global_pos = transform.c;

    Fvector second_part_force, first_part_force, second_part_torque, first_part_torque;
    second_part_force.set(0.f, 0.f, 0.f);
    first_part_force.set(0.f, 0.f, 0.f);
    second_part_torque.set(0.f, 0.f, 0.f);
    first_part_torque.set(0.f, 0.f, 0.f);

    // Обработка внешних импульсов/ударов (основной триггер разрушений от взрывов/оружия)
    auto i_i = impacts.begin(), i_e = impacts.end();
    for (; i_i != i_e; ++i_i)
    {
        u16 geom = i_i->geom;

        if ((geom >= m_start_geom_num && geom < m_end_geom_num))
        {
            Fvector force;
            force.set(i_i->force);
            force.mul(ph_console::phRigidBreakWeaponFactor);
            Fvector second_to_point;
            second_to_point.sub(body_global_pos, i_i->point);
            second_part_force.add(force);
            Fvector torque;
            torque.crossproduct(second_to_point, force);
            second_part_torque.add(torque);
        }
        else
        {
            Fvector force;
            force.set(i_i->force);
            Fvector first_to_point;
            first_to_point.sub(body_global_pos, i_i->point);
            first_part_force.add(force);
            Fvector torque;
            torque.crossproduct(first_to_point, force);
            first_part_torque.add(torque);
        }
    }

    Fvector gravity_force;
    gravity_force.set(0.f, -ph_world->Gravity() * m_firstM, 0.f);
    first_part_force.add(gravity_force);
    second_part_force.add(gravity_force);

    // Проверка порога разрушения по силе/моменту от импактов
    if (second_part_torque.magnitude() * ph_console::phBreakCommonFactor > m_break_torque * torque_factor)
    {
        m_pos_in_element.set(second_part_force);
        m_break_force = second_part_torque.x;
        m_break_torque = second_part_torque.y;
        m_add_torque_z = second_part_torque.z;
        m_breaked = true;
        return m_breaked;
    }

    Fvector break_force;
    break_force.set(first_part_force);
    break_force.mul(m_secondM);
    Fvector vtemp;
    vtemp.set(second_part_force);
    vtemp.mul(m_firstM);
    break_force.sub(vtemp);
    
    float element_mass = element->getMass();
    if (element_mass > 0.001f)
        break_force.mul(1.f / element_mass);

    float bfm = break_force.magnitude() * ph_console::phBreakCommonFactor;

    if (m_break_force < bfm && m_break_force > 0.f)
    {
        second_part_force.mul(bfm / m_break_force);
        m_pos_in_element.set(second_part_force);
        m_break_force = second_part_torque.x;
        m_break_torque = second_part_torque.y;
        m_add_torque_z = second_part_torque.z;
        m_breaked = true;
        return m_breaked;
    }

    return m_breaked;
}

void CPHFracture::SetMassParts(float first, float second)
{
    m_firstM = first;
    m_secondM = second;
}

void CPHFracture::MassAddToFirst(float m) { m_firstM += m; }
void CPHFracture::MassAddToSecond(float m) { m_secondM += m; }
void CPHFracture::MassSubFromFirst(float m) { m_firstM -= m; }
void CPHFracture::MassSubFromSecond(float m) { m_secondM -= m; }
void CPHFracture::MassSetFirst(float m) { m_firstM = m; }
void CPHFracture::MassSetSecond(float m) { m_secondM = m; }
void CPHFracture::MassUnsplitFromFirstToSecond(float m)
{
    m_firstM -= m;
    m_secondM += m;
}
void CPHFracture::MassSetZerro()
{
    m_firstM = 0.f;
    m_secondM = 0.f;
}
