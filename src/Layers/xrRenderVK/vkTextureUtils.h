#pragma once

#ifdef USE_VK

namespace xray::render::RENDER_NAMESPACE
{
namespace vkTextureUtils
{
VkFormat ConvertTextureFormat(D3DFORMAT dx9FMT);

void RegisterImage(VkImage image, VkImageView view, VkFormat format, VkImageAspectFlags aspect, VkImageLayout initialLayout = VK_IMAGE_LAYOUT_UNDEFINED, u32 width = 0, u32 height = 0);
void UnregisterImage(VkImageView view);

void RegisterViewFormat(VkImageView view, VkFormat format);
void UnregisterViewFormat(VkImageView view);
VkFormat GetViewFormat(VkImageView view, VkFormat defaultFormat = VK_FORMAT_UNDEFINED);

bool GetViewExtent(VkImageView view, u32& width, u32& height);

VkImage GetViewImage(VkImageView view);
VkImageLayout GetViewLayout(VkImageView view);
void SetViewLayout(VkImageView view, VkImageLayout layout);

void TransitionViewLayout(VkCommandBuffer cmd, VkImageView view, VkImageLayout newLayout);
} // namespace vkTextureUtils
} // namespace xray::render::RENDER_NAMESPACE

#endif // USE_VK
