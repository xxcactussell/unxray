#include "StdAfx.h"
#include "PHInterpolation.h"
#include "PHDynamicData.h"
#include "Physics.h"
#include "MathUtils.h"

extern CPHWorld* ph_world;
extern float fixed_step;

CPHInterpolation::CPHInterpolation()
{
    m_body = INVALID_BODY_HANDLE;
}

void CPHInterpolation::SetBody(BodyHandle body)
{
    if (body == INVALID_BODY_HANDLE)
        return;
        
    m_body = body;
    
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    
    qPositions.fill_in(transform.c);
    
    Fquaternion fQ;
    fQ.set(transform);
    qRotations.fill_in(fQ);
}

void CPHInterpolation::UpdatePositions()
{
    if (m_body == INVALID_BODY_HANDLE) return;
    
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    qPositions.push_back(transform.c);
}

void CPHInterpolation::UpdateRotations()
{
    if (m_body == INVALID_BODY_HANDLE) return;
    
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    
    Fquaternion fQ;
    fQ.set(transform);
    qRotations.push_back(fQ);
}

void CPHInterpolation::InterpolatePosition(Fvector& pos)
{
    float t = ph_world->m_frame_time / fixed_step;
    clamp(t, 0.f, 1.f); // Защита от выхода за пределы, чтобы не было дерганий
    pos.lerp(qPositions[0], qPositions[1], t);
}

void CPHInterpolation::InterpolateRotation(Fmatrix& rot)
{
    Fquaternion q;
    float t = ph_world->m_frame_time / fixed_step;
    clamp(t, 0.f, 1.f);
    q.slerp(qRotations[0], qRotations[1], t);
    rot.rotation(q);
}

void CPHInterpolation::ResetPositions()
{
    if (m_body == INVALID_BODY_HANDLE) return;
    
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    qPositions.fill_in(transform.c);
}

void CPHInterpolation::ResetRotations()
{
    if (m_body == INVALID_BODY_HANDLE) return;
    
    Fmatrix transform;
    GetPhysicsCore()->GetBodyTransform(m_body, transform);
    
    Fquaternion fQ;
    fQ.set(transform);
    qRotations.fill_in(fQ);
}

void CPHInterpolation::GetRotation(Fquaternion& q, u16 num)
{
    q = qRotations[num];
}

void CPHInterpolation::GetPosition(Fvector& p, u16 num)
{
    p = qPositions[num];
}

void CPHInterpolation::SetRotation(const Fquaternion& q, u16 num)
{
    qRotations[num] = q;
}

void CPHInterpolation::SetPosition(const Fvector& p, u16 num)
{
    qPositions[num] = p;
}
