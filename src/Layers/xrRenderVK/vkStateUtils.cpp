#include "stdafx.h"
#include "vkStateUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
namespace vkStateUtils
{
VkPolygonMode ConvertFillMode(u32 Mode)
{
    switch (Mode)
    {
    case D3DFILL_WIREFRAME:
        return VK_POLYGON_MODE_LINE;
    case D3DFILL_POINT:
        return VK_POLYGON_MODE_POINT;
    case D3DFILL_SOLID:
    default:
        return VK_POLYGON_MODE_FILL;
    }
}

VkCullModeFlags ConvertCullMode(u32 Mode)
{
    switch (Mode)
    {
    case D3DCULL_NONE:
        return VK_CULL_MODE_NONE;
    case D3DCULL_CW:
        return VK_CULL_MODE_FRONT_BIT;
    case D3DCULL_CCW:
    default:
        return VK_CULL_MODE_BACK_BIT;
    }
}

VkCompareOp ConvertCmpFunction(u32 Func)
{
    switch (Func)
    {
    case D3DCMP_NEVER:
        return VK_COMPARE_OP_NEVER;
    case D3DCMP_LESS:
        return VK_COMPARE_OP_LESS;
    case D3DCMP_EQUAL:
        return VK_COMPARE_OP_EQUAL;
    case D3DCMP_LESSEQUAL:
        return VK_COMPARE_OP_LESS_OR_EQUAL;
    case D3DCMP_GREATER:
        return VK_COMPARE_OP_GREATER;
    case D3DCMP_NOTEQUAL:
        return VK_COMPARE_OP_NOT_EQUAL;
    case D3DCMP_GREATEREQUAL:
        return VK_COMPARE_OP_GREATER_OR_EQUAL;
    case D3DCMP_ALWAYS:
    default:
        return VK_COMPARE_OP_ALWAYS;
    }
}

VkStencilOp ConvertStencilOp(u32 Op)
{
    switch (Op)
    {
    case D3DSTENCILOP_KEEP:
        return VK_STENCIL_OP_KEEP;
    case D3DSTENCILOP_ZERO:
        return VK_STENCIL_OP_ZERO;
    case D3DSTENCILOP_REPLACE:
        return VK_STENCIL_OP_REPLACE;
    case D3DSTENCILOP_INCRSAT:
        return VK_STENCIL_OP_INCREMENT_AND_CLAMP;
    case D3DSTENCILOP_DECRSAT:
        return VK_STENCIL_OP_DECREMENT_AND_CLAMP;
    case D3DSTENCILOP_INVERT:
        return VK_STENCIL_OP_INVERT;
    case D3DSTENCILOP_INCR:
        return VK_STENCIL_OP_INCREMENT_AND_WRAP;
    case D3DSTENCILOP_DECR:
        return VK_STENCIL_OP_DECREMENT_AND_WRAP;
    default:
        return VK_STENCIL_OP_KEEP;
    }
}

VkBlendFactor ConvertBlendArg(u32 Arg)
{
    switch (Arg)
    {
    case D3DBLEND_ZERO:
        return VK_BLEND_FACTOR_ZERO;
    case D3DBLEND_ONE:
        return VK_BLEND_FACTOR_ONE;
    case D3DBLEND_SRCCOLOR:
        return VK_BLEND_FACTOR_SRC_COLOR;
    case D3DBLEND_INVSRCCOLOR:
        return VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
    case D3DBLEND_SRCALPHA:
        return VK_BLEND_FACTOR_SRC_ALPHA;
    case D3DBLEND_INVSRCALPHA:
        return VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    case D3DBLEND_DESTALPHA:
        return VK_BLEND_FACTOR_DST_ALPHA;
    case D3DBLEND_INVDESTALPHA:
        return VK_BLEND_FACTOR_ONE_MINUS_DST_ALPHA;
    case D3DBLEND_DESTCOLOR:
        return VK_BLEND_FACTOR_DST_COLOR;
    case D3DBLEND_INVDESTCOLOR:
        return VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR;
    case D3DBLEND_SRCALPHASAT:
        return VK_BLEND_FACTOR_SRC_ALPHA_SATURATE;
    default:
        return VK_BLEND_FACTOR_ONE;
    }
}

VkBlendOp ConvertBlendOp(u32 Op)
{
    switch (Op)
    {
    case D3DBLENDOP_ADD:
        return VK_BLEND_OP_ADD;
    case D3DBLENDOP_SUBTRACT:
        return VK_BLEND_OP_SUBTRACT;
    case D3DBLENDOP_REVSUBTRACT:
        return VK_BLEND_OP_REVERSE_SUBTRACT;
    case D3DBLENDOP_MIN:
        return VK_BLEND_OP_MIN;
    case D3DBLENDOP_MAX:
        return VK_BLEND_OP_MAX;
    default:
        return VK_BLEND_OP_ADD;
    }
}

VkSamplerAddressMode ConvertTextureAddressMode(u32 Mode)
{
    switch (Mode)
    {
    case D3DTADDRESS_WRAP:
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    case D3DTADDRESS_MIRROR:
        return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
    case D3DTADDRESS_CLAMP:
    default:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    case D3DTADDRESS_BORDER:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    case D3DTADDRESS_MIRRORONCE:
        return VK_SAMPLER_ADDRESS_MODE_MIRROR_CLAMP_TO_EDGE;
    }
}

VkFilter ConvertTextureFilter(u32 dxFilter)
{
    switch (dxFilter)
    {
    case D3DTEXF_NONE:
    case D3DTEXF_POINT:
        return VK_FILTER_NEAREST;
    case D3DTEXF_LINEAR:
    case D3DTEXF_ANISOTROPIC:
    default:
        return VK_FILTER_LINEAR;
    }
}

VkSamplerMipmapMode ConvertTextureMipFilter(u32 dxFilter)
{
    switch (dxFilter)
    {
    case D3DTEXF_NONE:
    case D3DTEXF_POINT:
        return VK_SAMPLER_MIPMAP_MODE_NEAREST;
    case D3DTEXF_LINEAR:
    case D3DTEXF_ANISOTROPIC:
    default:
        return VK_SAMPLER_MIPMAP_MODE_LINEAR;
    }
}
} // namespace vkStateUtils
} // namespace xray::render::RENDER_NAMESPACE
