#pragma once

#include "xrPhysicsCore/IPhysicsCore.h"

class CPHBaseBodyEffector
{
protected:
    BodyHandle m_body = INVALID_BODY_HANDLE;

public:
    void Init(BodyHandle body) { m_body = body; }
};
