#include "StdAfx.h"

#include "PHSoundPlayer.h"
#include "PhysicsShellHolder.h"
CPHSoundPlayer::CPHSoundPlayer(CPhysicsShellHolder* obj) { m_object = obj; }
CPHSoundPlayer::~CPHSoundPlayer()
{
    m_sound.stop();
    m_object = NULL;
}

void CPHSoundPlayer::Play(SGameMtlPair* mtl_pair, const Fvector& pos)
{
    Fvector vel;
    m_object->PHGetLinearVell(vel);
    bool feedback = (m_sound._feedback() != nullptr);
    Msg("[Bush-Debug] CPHSoundPlayer::Play: feedback=%d, vel_sq=%.4f, sounds_cnt=%u, pos=(%.2f, %.2f, %.2f)",
        feedback, vel.square_magnitude(), mtl_pair ? (u32)mtl_pair->CollideSounds.size() : 0, pos.x, pos.y, pos.z);

    if (!feedback)
    {
        if (vel.square_magnitude() > 0.01f)
        {
            VERIFY2(!mtl_pair->CollideSounds.empty(), mtl_pair->dbg_Name());
            ref_sound& randSound = mtl_pair->CollideSounds[Random.randI(mtl_pair->CollideSounds.size())];
            m_sound.clone(randSound, st_Effect, sg_SourceType);
            m_sound.play_at_pos(smart_cast<CPhysicsShellHolder*>(m_object), pos);
            Msg("[Bush-Debug] CPHSoundPlayer::Play: Sound PLAYED successfully!");
        }
    }
}
