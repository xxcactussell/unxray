#include "stdafx.h"
#include "dxUISequenceVideoItem.h"

namespace xray::render::RENDER_NAMESPACE
{
dxUISequenceVideoItem::dxUISequenceVideoItem() { m_texture = nullptr; }
void dxUISequenceVideoItem::Copy(IUISequenceVideoItem& _in) { *this = *((dxUISequenceVideoItem*)&_in); }

void dxUISequenceVideoItem::CaptureTexture()
{
    R_constant* C = RCache.get_c(c_sbase)._get(); // get sampler
    if (C)
        m_texture = RCache.get_ActiveTexture(C->samp.index);
    if (!m_texture)
    {
        for (u32 i = 0; i < CTexture::mtMaxPixelShaderTextures; ++i)
        {
            m_texture = RCache.get_ActiveTexture(i);
            if (m_texture)
                break;
        }
    }
    if (!m_texture)
    {
        Msg("! [dxUISequenceVideoItem] Warning: Could not capture active texture from RCache");
        return;
    }
}
} // namespace xray::render::RENDER_NAMESPACE
