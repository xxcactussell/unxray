#include "StdAfx.h"
#include "PHInterpolation.h"
#include "PHDynamicData.h"
#include "Physics.h"
#include "MathUtils.h"

extern CPHWorld* ph_world;
extern float fixed_step;

CPHInterpolation::CPHInterpolation()
{
    m_char_handle = INVALID_CHARACTER_VIRTUAL_HANDLE;
}

void CPHInterpolation::SetBody(CharacterVirtualHandle body)
{
    if (body == INVALID_CHARACTER_VIRTUAL_HANDLE)
        return;
        
    m_char_handle = body;
    
    Fvector pos;
    GetPhysicsCore()->GetCharacterVirtualPosition(m_char_handle, pos);
    
    qPositions.fill_in(pos);
    
    Fquaternion fQ;
    fQ.identity();
    qRotations.fill_in(fQ);
}

void CPHInterpolation::UpdatePositions()
{
    if (m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE) return;
    
    Fvector pos;
    GetPhysicsCore()->GetCharacterVirtualPosition(m_char_handle, pos);
    qPositions.push_back(pos);
}

void CPHInterpolation::UpdateRotations()
{
    if (m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE) return;
    
    Fquaternion fQ;
    fQ.identity();
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
    if (m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE) return;
    
    Fvector pos;
    GetPhysicsCore()->GetCharacterVirtualPosition(m_char_handle, pos);
    qPositions.fill_in(pos);
}

void CPHInterpolation::ResetRotations()
{
    if (m_char_handle == INVALID_CHARACTER_VIRTUAL_HANDLE) return;
    
    Fquaternion fQ;
    fQ.identity();
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
