#include "stdafx.h"
#include "vkTextureUtils.h"
#include "vkImageTracker.h"

namespace xray::render::RENDER_NAMESPACE
{
namespace vkTextureUtils
{
struct TextureFormatPairs
{
    D3DFORMAT m_dx9FMT;
    VkFormat m_vkFMT;
};

static const TextureFormatPairs TextureFormatList[] =
{
    { D3DFMT_UNKNOWN, VK_FORMAT_UNDEFINED },
    { D3DFMT_A8R8G8B8, VK_FORMAT_B8G8R8A8_UNORM },
    { D3DFMT_X8R8G8B8, VK_FORMAT_B8G8R8A8_UNORM },
    { D3DFMT_A8B8G8R8, VK_FORMAT_R8G8B8A8_UNORM },
    { D3DFMT_R5G6B5, VK_FORMAT_R8G8B8A8_UNORM },
    { D3DFMT_G16R16, VK_FORMAT_R16G16_UNORM },
    { D3DFMT_A16B16G16R16, VK_FORMAT_R16G16B16A16_UNORM },
    { D3DFMT_A16B16G16R16F, VK_FORMAT_R16G16B16A16_SFLOAT },
    { D3DFMT_R32F, VK_FORMAT_R32_SFLOAT },
    { D3DFMT_G32R32F, VK_FORMAT_R32G32_SFLOAT },
    { D3DFMT_A32B32G32R32F, VK_FORMAT_R32G32B32A32_SFLOAT },
    { D3DFMT_R16F, VK_FORMAT_R16_SFLOAT },
    { D3DFMT_G16R16F, VK_FORMAT_R16G16_SFLOAT },
    { D3DFMT_L8, VK_FORMAT_R8_UNORM },
    { D3DFMT_V8U8, VK_FORMAT_R8G8_UNORM },
    { D3DFMT_Q8W8V8U8, VK_FORMAT_R8G8B8A8_UNORM },
    { D3DFMT_V16U16, VK_FORMAT_R16G16_UNORM },
    { D3DFMT_D16, VK_FORMAT_D16_UNORM },
    { D3DFMT_D24S8, VK_FORMAT_D24_UNORM_S8_UINT },
    { D3DFMT_D24X8, VK_FORMAT_D24_UNORM_S8_UINT },
    { D3DFMT_DXT1, VK_FORMAT_BC1_RGBA_UNORM_BLOCK },
    { D3DFMT_DXT2, VK_FORMAT_BC1_RGBA_UNORM_BLOCK },
    { D3DFMT_DXT3, VK_FORMAT_BC2_UNORM_BLOCK },
    { D3DFMT_DXT4, VK_FORMAT_BC2_UNORM_BLOCK },
    { D3DFMT_DXT5, VK_FORMAT_BC3_UNORM_BLOCK },
};

VkFormat ConvertTextureFormat(D3DFORMAT dx9FMT)
{
    for (const auto& pair : TextureFormatList)
    {
        if (pair.m_dx9FMT == dx9FMT)
            return pair.m_vkFMT;
    }
    return VK_FORMAT_R8G8B8A8_UNORM;
}

struct ImageTrackingInfo
{
    VkImage image{ VK_NULL_HANDLE };
    VkFormat format{ VK_FORMAT_UNDEFINED };
    VkImageAspectFlags aspect{ VK_IMAGE_ASPECT_COLOR_BIT };
    VkImageLayout currentLayout{ VK_IMAGE_LAYOUT_UNDEFINED };
};

static xr_map<VkImageView, ImageTrackingInfo> s_trackedImages;
static Lock s_trackedImagesLock;

void RegisterImage(VkImage image, VkImageView view, VkFormat format, VkImageAspectFlags aspect, VkImageLayout initialLayout)
{
    if (view == VK_NULL_HANDLE) return;
    ScopeLock lock(&s_trackedImagesLock);
    ImageTrackingInfo info{};
    info.image = image;
    info.format = format;
    info.aspect = aspect;
    info.currentLayout = initialLayout;
    s_trackedImages[view] = info;
}

void UnregisterImage(VkImageView view)
{
    if (view == VK_NULL_HANDLE) return;
    ScopeLock lock(&s_trackedImagesLock);
    s_trackedImages.erase(view);
}

void RegisterViewFormat(VkImageView view, VkFormat format)
{
    if (view == VK_NULL_HANDLE) return;
    ScopeLock lock(&s_trackedImagesLock);
    s_trackedImages[view].format = format;
}

void UnregisterViewFormat(VkImageView view)
{
    UnregisterImage(view);
}

VkFormat GetViewFormat(VkImageView view, VkFormat defaultFormat)
{
    if (view == VK_NULL_HANDLE) return defaultFormat;
    ScopeLock lock(&s_trackedImagesLock);
    auto it = s_trackedImages.find(view);
    if (it != s_trackedImages.end() && it->second.format != VK_FORMAT_UNDEFINED)
        return it->second.format;
    return defaultFormat;
}

VkImage GetViewImage(VkImageView view)
{
    if (view == VK_NULL_HANDLE) return VK_NULL_HANDLE;
    ScopeLock lock(&s_trackedImagesLock);
    auto it = s_trackedImages.find(view);
    if (it != s_trackedImages.end())
        return it->second.image;
    return VK_NULL_HANDLE;
}

VkImageLayout GetViewLayout(VkImageView view)
{
    if (view == VK_NULL_HANDLE) return VK_IMAGE_LAYOUT_UNDEFINED;
    ScopeLock lock(&s_trackedImagesLock);
    auto it = s_trackedImages.find(view);
    if (it != s_trackedImages.end())
        return it->second.currentLayout;
    return VK_IMAGE_LAYOUT_UNDEFINED;
}

void SetViewLayout(VkImageView view, VkImageLayout layout)
{
    if (view == VK_NULL_HANDLE) return;
    ScopeLock lock(&s_trackedImagesLock);
    auto it = s_trackedImages.find(view);
    if (it != s_trackedImages.end())
        it->second.currentLayout = layout;
}

void TransitionViewLayout(VkCommandBuffer cmd, VkImageView view, VkImageLayout newLayout)
{
    if (view == VK_NULL_HANDLE || cmd == VK_NULL_HANDLE) return;
    ScopeLock lock(&s_trackedImagesLock);
    auto it = s_trackedImages.find(view);
    if (it == s_trackedImages.end()) return;

    if (it->second.currentLayout == newLayout)
        return;

    VkPipelineStageFlags2 dstStage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    VkAccessFlags2 dstAccess = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;

    if (newLayout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL)
    {
        dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT;
        dstStage = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    }
    else if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
    {
        dstAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
        dstStage = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    }
    else if (newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
    {
        dstAccess = VK_ACCESS_2_SHADER_READ_BIT;
        dstStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT;
    }
    else if (newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
    {
        dstAccess = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        dstStage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    }
    else if (newLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)
    {
        dstAccess = VK_ACCESS_2_TRANSFER_READ_BIT;
        dstStage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    }
    else if (newLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR)
    {
        dstAccess = VK_ACCESS_2_NONE;
        dstStage = VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT;
    }

    G_ImageTracker.TransitionImage(cmd, it->second.image, it->second.aspect, newLayout, dstStage, dstAccess);
    it->second.currentLayout = newLayout;
}
} // namespace vkTextureUtils
} // namespace xray::render::RENDER_NAMESPACE
