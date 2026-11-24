#pragma once

#ifdef USE_VK

namespace xray::render::RENDER_NAMESPACE
{
namespace vkStateUtils
{
VkPolygonMode ConvertFillMode(u32 Mode);
VkCullModeFlags ConvertCullMode(u32 Mode);
VkCompareOp ConvertCmpFunction(u32 Func);
VkStencilOp ConvertStencilOp(u32 Op);
VkBlendFactor ConvertBlendArg(u32 Arg);
VkBlendOp ConvertBlendOp(u32 Op);
VkSamplerAddressMode ConvertTextureAddressMode(u32 Mode);
VkFilter ConvertTextureFilter(u32 dxFilter);
VkSamplerMipmapMode ConvertTextureMipFilter(u32 dxFilter);
} // namespace vkStateUtils
} // namespace xray::render::RENDER_NAMESPACE

#endif // USE_VK
