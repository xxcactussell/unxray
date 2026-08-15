/////////////////////////////////////////////////////////////////////////////////////////////////
#include "StdAfx.h"

#include "PHCapture.h"
#include "PHCharacter.h"
#include "Physics.h"
#include "ExtendedGeom.h"

#include "Include/xrRender/Kinematics.h"
#include "IPhysicsShellHolder.h"
#include "xrCore/Animation/Bone.hpp"
#include "xrEngine/device.h"
#include "PHElement.h"

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

IPHCapture* phcapture_create(CPHCharacter* ch, IPhysicsShellHolder* object, NearestToPointCallback* cb /*=0*/)
{
    VERIFY(ch);
    return xr_new<CPHCapture>(ch, object, cb);
}

IPHCapture* phcapture_create(CPHCharacter* ch, IPhysicsShellHolder* object, u16 element)
{
    VERIFY(ch);
    return xr_new<CPHCapture>(ch, object, element);
}

void phcapture_destroy(IPHCapture*& c)
{
    CPHCapture* capture = smart_cast<CPHCapture*>(c);
    xr_delete(capture);
    c = 0;
}

void CPHCapture::CreateBody()
{
    // В Jolt Physics мы больше не создаем фейковое твердое тело для захвата.
    // Вместо этого мы будем использовать привязку к статичному пространству (INVALID_CHARACTER_VIRTUAL_HANDLE)
    // в нашем FullControlJoint и просто будем обновлять его параметры мотора.
    m_char_handle = INVALID_CHARACTER_VIRTUAL_HANDLE;
}

CPHCapture::~CPHCapture() 
{ 
    Deactivate(); 
}

bool CPHCapture::Invalid()
{
    return !m_taget_object->ObjectPPhysicsShell() || !m_taget_object->ObjectPPhysicsShell()->isActive() ||
        !m_character->b_exist;
}

void CPHCapture::PhDataUpdate(float step)
{
    switch (e_state)
    {
    case cstFree: break;
    case cstPulling: PullingUpdate(); break;
    case cstCaptured: CapturedUpdate(); break;
    case cstReleased: ReleasedUpdate(); break;
    default: NODEFAULT;
    }
}

void CPHCapture::PhTune(float step)
{
    if (e_state == cstFree)
        return;

    VERIFY(m_character && m_character->b_exist);
    VERIFY(m_taget_object);
    VERIFY(m_taget_object->ObjectPPhysicsShell());
    VERIFY(m_taget_object->ObjectPPhysicsShell()->isFullActive());
    VERIFY(m_taget_element);
    VERIFY(m_taget_element->isFullActive());

    bool act_capturer = m_character->CPHObject::is_active();
    bool act_taget = m_taget_object->ObjectPPhysicsShell()->isEnabled();

    b_disabled = !act_capturer && !act_taget;
    if (act_capturer)
    {
        m_taget_element->Enable();
    }
    if (act_taget)
    {
        m_character->Enable();
    }
    switch (e_state)
    {
    case cstPulling: break;
    case cstCaptured:
    {
        if (b_disabled)
        {
            if (m_taget_element->get_body() != INVALID_CHARACTER_VIRTUAL_HANDLE)
                GetPhysicsCore()->DeactivateBody(m_taget_element->get_body());
        }
    }
    break;
    case cstReleased: break;
    default: NODEFAULT;
    }
}

void CPHCapture::PullingUpdate()
{
    if (!m_taget_element->isActive() || Device.dwTimeGlobal - m_time_start > m_capture_time)
    {
        Release();
        return;
    }

    Fvector dir;
    Fvector capture_bone_position;
    
    capture_bone_position.set(m_capture_bone->mTransform.c);
    m_character->PhysicsRefObject()->ObjectXFORM().transform_tiny(capture_bone_position);
    m_taget_element->GetGlobalPositionDynamic(&dir);
    
    dir.sub(capture_bone_position, dir);
    float dist = dir.magnitude();
    
    if (dist > m_pull_distance)
    {
        Release();
        return;
    }
    
    if (dist > EPS)
        dir.mul(1.f / dist);
        
    if (dist < m_capture_distance)
    {
        m_back_force = 0.f;

        CreateBody();
        
        CPHElement* e = static_cast<CPHElement*>(m_taget_element);
        CharacterVirtualHandle target_body = e->get_body();
        
        if (target_body == INVALID_CHARACTER_VIRTUAL_HANDLE) 
        {
            Release();
            return;
        }

        m_joint = INVALID_JOINT_HANDLE;
        m_ajoint = INVALID_JOINT_HANDLE;

        m_taget_element->set_LinearVel(Fvector().set(0, 0, 0));
        m_taget_element->set_AngularVel(Fvector().set(0, 0, 0));
        m_taget_element->set_DynamicLimits();
        
        e_state = cstCaptured;
        return;
    }
    
    m_taget_element->applyForce(dir, m_pull_force);
}

void CPHCapture::CapturedUpdate()
{
    if (m_character->CPHObject::is_active())
    {
        m_taget_element->Enable();
    }

    if (!m_taget_element->isActive())
    {
        Release();
        return;
    }
    
    CPHElement* e = static_cast<CPHElement*>(m_taget_element);
    CharacterVirtualHandle target_body = e->get_body();

    // Простая проверка разрыва: если цель отдалилась от точки захвата слишком сильно
    Fvector target_pos;
    m_taget_element->GetGlobalPositionDynamic(&target_pos);
    
    Fvector capture_bone_position;
    capture_bone_position.set(m_capture_bone->mTransform.c);
    m_character->PhysicsRefObject()->ObjectXFORM().transform_tiny(capture_bone_position);

    float current_dist = capture_bone_position.distance_to(target_pos);

    if (current_dist > m_capture_distance * 1.5f) // Толерантность на отрыв
    {
        Release();
        return;
    }

    if (b_character_feedback && current_dist > m_capture_distance * 0.5f)
    {
        // Передача обратной связи (тяжести) на актора
        Fvector force_dir;
        force_dir.sub(target_pos, capture_bone_position);
        if (force_dir.magnitude() > EPS)
        {
            force_dir.normalize();
            float f = current_dist * m_capture_force * 0.1f;
            m_character->ApplyForce(force_dir.x * f, force_dir.y * f, force_dir.z * f);
        }
    }

    // Обновляем позицию якоря (перемещаем сустав)
    Fmatrix new_transform;
    new_transform.identity();
    new_transform.c = capture_bone_position;
    
    // В Jolt для FullControlJoint можно менять TargetPosition мотора (реализация в IPhysicsCore)
    // Чтобы код компилировался сейчас без добавления новых методов, мы можем
    // разрушать и пересоздавать joint, либо просто прикладывать силы.
    // Если в IPhysicsCore появится SetJointTargetTransform, нужно будет вызвать его:
    // GetPhysicsCore()->SetJointTargetTransform(m_joint, new_transform);
    
    // Пока что эмулируем удержание через пружинную силу
    Fvector hold_dir;
    hold_dir.sub(capture_bone_position, target_pos);
    float dist = hold_dir.magnitude();
    if (dist > EPS)
    {
        hold_dir.normalize();
        float p_force = dist * m_capture_force * 0.5f;
        GetPhysicsCore()->ApplyForce(target_body, Fvector().set(hold_dir.x * p_force, hold_dir.y * p_force, hold_dir.z * p_force));
        
        // Гашение скорости (демпфирование)
        Fvector vel;
        GetPhysicsCore()->GetBodyLinearVelocity(target_body, vel);
        vel.mul(0.9f); // Искусственное гашение
        GetPhysicsCore()->SetBodyLinearVelocity(target_body, vel);
    }
}

void CPHCapture::ReleasedUpdate()
{
    if (b_disabled)
        return;
    if (!b_collide)
    {
        e_state = cstFree;
        m_taget_element->Enable();
    }
    b_collide = false;
}

void CPHCapture::ReleaseInCallBack()
{
    b_collide = true;
}

void CPHCapture::object_contactCallbackFun(
    bool& do_colide, bool bo1, 
    CPhysicsGeom* my_geom, CPhysicsGeom* oposite_geom, 
    const Fvector& contact_normal, const Fvector& contact_pos, 
    SGameMtl* material_1, SGameMtl* material_2)
{
    if (!my_geom || !oposite_geom)
        return;

    IPhysicsShellHolder* capturer1 = (IPhysicsShellHolder*)my_geom->get_callback_data();
    IPhysicsShellHolder* capturer2 = (IPhysicsShellHolder*)oposite_geom->get_callback_data();

    if (capturer1)
    {
        IPHCapture* icapture = capturer1->PHCapture();
        CPHCapture* capture = static_cast<CPHCapture*>(icapture);
        if (capture && capture->m_taget_element)
        {
            if (capture->m_taget_element->PhysicsRefObject() == capturer2)
            {
                do_colide = false;
                capture->m_taget_element->Enable();
                if (capture->e_state == CPHCapture::cstReleased)
                    capture->ReleaseInCallBack();
            }
        }
    }

    if (capturer2)
    {
        CPHCapture* capture = static_cast<CPHCapture*>(capturer2->PHCapture());
        if (capture && capture->m_taget_element)
        {
            if (capture->m_taget_element->PhysicsRefObject() == capturer1)
            {
                do_colide = false;
                capture->m_taget_element->Enable();
                if (capture->e_state == CPHCapture::cstReleased)
                    capture->ReleaseInCallBack();
            }
        }
    }
}

void CPHCapture::RemoveConnection(IPhysicsShellHolder* O)
{
    if (m_taget_object == O)
    {
        Deactivate();
    }
}

void CPHCapture::NetRelcase(CPhysicsShell* s)
{
    VERIFY(s);
    VERIFY(s->get_ElementByStoreOrder(0));
    VERIFY(s->get_ElementByStoreOrder(0)->PhysicsRefObject());
    RemoveConnection(s->get_ElementByStoreOrder(0)->PhysicsRefObject());
}
