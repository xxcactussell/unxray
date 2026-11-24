#include "stdafx.h"
#include "vkState.h"
#include "vkStateUtils.h"
#include "vkHW.h"

namespace xray::render::RENDER_NAMESPACE
{
vkState::vkState()
{
    memset(m_samplerArray, 0, sizeof(m_samplerArray));

    rasterizerCullMode = D3DCULL_CCW;

    m_pDepthStencilState.DepthEnable = TRUE;
    m_pDepthStencilState.DepthFunc = D3DCMP_LESSEQUAL;
    m_pDepthStencilState.DepthWriteMask = TRUE;
    m_pDepthStencilState.StencilEnable = FALSE;
    m_pDepthStencilState.StencilFailOp = D3DSTENCILOP_KEEP;
    m_pDepthStencilState.StencilDepthFailOp = D3DSTENCILOP_KEEP;
    m_pDepthStencilState.StencilPassOp = D3DSTENCILOP_KEEP;
    m_pDepthStencilState.StencilFunc = D3DCMP_ALWAYS;
    m_pDepthStencilState.StencilMask = 0xFFFFFFFF;
    m_pDepthStencilState.StencilWriteMask = 0xFFFFFFFF;
    m_pDepthStencilState.StencilRef = 0;

    m_pBlendState.BlendEnable = TRUE;
    m_pBlendState.SrcBlend = D3DBLEND_ONE;
    m_pBlendState.DestBlend = D3DBLEND_ZERO;
    m_pBlendState.SrcBlendAlpha = D3DBLEND_ONE;
    m_pBlendState.DestBlendAlpha = D3DBLEND_ZERO;
    m_pBlendState.BlendOp = D3DBLENDOP_ADD;
    m_pBlendState.BlendOpAlpha = D3DBLENDOP_ADD;
    m_pBlendState.ColorMask = 0xF;

    m_uiMipLODBias = FLT_MAX;
}

vkState::~vkState()
{
    Release();
}

vkState* vkState::Create()
{
    return xr_new<vkState>();
}

void vkState::Apply()
{
    RCache.set_CullMode(rasterizerCullMode);
    RCache.set_Z(m_pDepthStencilState.DepthEnable);
    RCache.set_ZFunc(m_pDepthStencilState.DepthFunc);
    RCache.set_Stencil(
        m_pDepthStencilState.StencilEnable,
        m_pDepthStencilState.StencilFunc,
        m_pDepthStencilState.StencilRef,
        m_pDepthStencilState.StencilMask,
        m_pDepthStencilState.StencilWriteMask,
        m_pDepthStencilState.StencilFailOp,
        m_pDepthStencilState.StencilPassOp,
        m_pDepthStencilState.StencilDepthFailOp
    );

    RCache.set_ColorWriteEnable(m_pBlendState.ColorMask);
}

void vkState::Release()
{
    if (HW.pDevice != VK_NULL_HANDLE)
    {
        for (size_t i = 0; i < std::size(m_samplerArray); ++i)
        {
            if (m_samplerArray[i] != VK_NULL_HANDLE)
            {
                vkDestroySampler(HW.pDevice, m_samplerArray[i], nullptr);
                m_samplerArray[i] = VK_NULL_HANDLE;
            }
        }
    }
}

void vkState::UpdateRenderState(u32 name, u32 value)
{
    switch (name)
    {
    case D3DRS_CULLMODE:
        rasterizerCullMode = (D3DCULL)value;
        break;

    case D3DRS_ZENABLE:
        m_pDepthStencilState.DepthEnable = value ? TRUE : FALSE;
        break;

    case D3DRS_ZWRITEENABLE:
        m_pDepthStencilState.DepthWriteMask = value ? TRUE : FALSE;
        break;

    case D3DRS_ZFUNC:
        m_pDepthStencilState.DepthFunc = (D3DCMPFUNC)value;
        break;

    case D3DRS_STENCILENABLE:
        m_pDepthStencilState.StencilEnable = value ? TRUE : FALSE;
        break;

    case D3DRS_STENCILMASK:
        m_pDepthStencilState.StencilMask = (u32)value;
        break;

    case D3DRS_STENCILWRITEMASK:
        m_pDepthStencilState.StencilWriteMask = (u32)value;
        break;

    case D3DRS_STENCILFAIL:
        m_pDepthStencilState.StencilFailOp = (D3DSTENCILOP)value;
        break;

    case D3DRS_STENCILZFAIL:
        m_pDepthStencilState.StencilDepthFailOp = (D3DSTENCILOP)value;
        break;

    case D3DRS_STENCILPASS:
        m_pDepthStencilState.StencilPassOp = (D3DSTENCILOP)value;
        break;

    case D3DRS_STENCILFUNC:
        m_pDepthStencilState.StencilFunc = (D3DCMPFUNC)value;
        break;

    case D3DRS_STENCILREF:
        m_pDepthStencilState.StencilRef = value;
        break;

    case D3DRS_SRCBLEND:
        m_pBlendState.SrcBlend = (D3DBLEND)value;
        break;

    case D3DRS_DESTBLEND:
        m_pBlendState.DestBlend = (D3DBLEND)value;
        break;

    case D3DRS_BLENDOP:
        m_pBlendState.BlendOp = (D3DBLENDOP)value;
        break;

    case D3DRS_SRCBLENDALPHA:
        m_pBlendState.SrcBlendAlpha = (D3DBLEND)value;
        break;

    case D3DRS_DESTBLENDALPHA:
        m_pBlendState.DestBlendAlpha = (D3DBLEND)value;
        break;

    case D3DRS_BLENDOPALPHA:
        m_pBlendState.BlendOpAlpha = (D3DBLENDOP)value;
        break;

    case D3DRS_ALPHABLENDENABLE:
        m_pBlendState.BlendEnable = value ? TRUE : FALSE;
        break;

    case D3DRS_COLORWRITEENABLE:
    case D3DRS_COLORWRITEENABLE1:
    case D3DRS_COLORWRITEENABLE2:
    case D3DRS_COLORWRITEENABLE3:
        m_pBlendState.ColorMask = (u32)value;
        break;

    default:
        break;
    }
}

void vkState::UpdateSamplerState(u32 stage, u32 name, u32 value)
{
    if (stage >= std::size(m_samplerArray) || !HW.pDevice)
        return;

    if (m_samplerArray[stage] == VK_NULL_HANDLE)
    {
        VkSamplerCreateInfo samplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.maxLod = 16.0f;
        vkCreateSampler(HW.pDevice, &samplerInfo, nullptr, &m_samplerArray[stage]);
    }
}
} // namespace xray::render::RENDER_NAMESPACE
