#pragma once

#include "PHShell.h"
#include "xrPhysicsCore/IPhysicsCore.h"

class CPHElement;

class CPhysicsShellAnimatorBoneData
{
    friend class CPhysicsShellAnimator;
    JointHandle m_anim_fixed_joint = INVALID_JOINT_HANDLE;
    CPHElement* m_element = nullptr;
};
