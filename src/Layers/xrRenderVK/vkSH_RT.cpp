#include "stdafx.h"
#pragma hdrstop

#include "../xrRender/ResourceManager.h"
#include "vkTextureUtils.h"
#include "vkHW.h"
#include "vkDeletionQueue.h"

namespace xray::render::RENDER_NAMESPACE
{
CRT::~CRT()
{
    destroy();
    RImplementation.Resources->_DeleteRT(this);
}

void CRT::set_slice_read(int slice) {}
void CRT::set_slice_write(u32 context_id, int slice) {}

void CRT::create(LPCSTR Name, u32 w, u32 h, D3DFORMAT f, u32 SampleCount, u32 slices_num, Flags32 flags)
{
    if (pRT || pZRT) return;

    R_ASSERT(Name && Name[0] && w && h);
    _order = CPU::QPC();

    dwWidth = w;
    dwHeight = h;
    fmt = f;
    sampleCount = SampleCount;
    n_slices = slices_num;

    if (!HW.pDevice || !HW.m_allocator) return;

    RImplementation.Resources->Evict();

    bool isDepth = (f == D3DFMT_D16 || f == D3DFMT_D24S8 || f == D3DFMT_D24X8);
    VkFormat vkFmt = vkTextureUtils::ConvertTextureFormat(fmt);

    if (flags.test(CRT::CreateBase))
    {
        dwFlags |= CRT::CreateBase;
        if (!isDepth)
        {
            u32 idx = 0;
            char const* str = strrchr(Name, '_');
            if (str)
                sscanf(++str, "%u", &idx);
            if (idx < HW.m_swapchainImageViews.size())
            {
                pSurface = HW.m_swapchainImages[idx];
                pRT = HW.m_swapchainImageViews[idx];
                allocation = VK_NULL_HANDLE;
                pTexture = RImplementation.Resources->_CreateTexture(Name);
                pTexture->flags.bUser = true;
                pTexture->flags.bLoaded = true;
                pTexture->surface_set(pRT, pSurface);
                // Note: swapchain image and view are already registered and tracked in CHW::CreateSwapChain
                return;
            }
        }
    }

    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = w;
    imageInfo.extent.height = h;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = slices_num > 1 ? slices_num : 1;
    imageInfo.format = vkFmt;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.samples = (SampleCount > 1) ? static_cast<VkSampleCountFlagBits>(SampleCount) : VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (isDepth)
        imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    else
        imageInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

    VkResult res = vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &pSurface, &allocation, nullptr);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to allocate render target image '%s' (%ux%u)! Result = %d", Name, w, h, res);
        return;
    }

    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = pSurface;
    viewInfo.viewType = (slices_num > 1) ? VK_IMAGE_VIEW_TYPE_2D_ARRAY : VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = vkFmt;
    viewInfo.subresourceRange.aspectMask = isDepth ? (VK_IMAGE_ASPECT_DEPTH_BIT | (f == D3DFMT_D24S8 ? VK_IMAGE_ASPECT_STENCIL_BIT : 0)) : VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = imageInfo.arrayLayers;

    if (isDepth)
    {
        res = vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &pZRT);
        if (res == VK_SUCCESS)
            vkTextureUtils::RegisterImage(pSurface, pZRT, vkFmt, viewInfo.subresourceRange.aspectMask, VK_IMAGE_LAYOUT_UNDEFINED);
    }
    else
    {
        res = vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &pRT);
        if (res == VK_SUCCESS)
            vkTextureUtils::RegisterImage(pSurface, pRT, vkFmt, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
    }

    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create view for RT '%s'! Result = %d", Name, res);
        return;
    }

    pTexture = RImplementation.Resources->_CreateTexture(Name);
    pTexture->flags.bUser = true;
    pTexture->flags.bLoaded = true;
    pTexture->surface_set(pRT ? pRT : pZRT, pSurface);
}

void CRT::destroy()
{
    if (pTexture._get())
    {
        pTexture->surface_set(VK_NULL_HANDLE, VK_NULL_HANDLE);
        pTexture = nullptr;
    }

    if (dwFlags & CRT::CreateBase)
    {
        pRT = VK_NULL_HANDLE;
        pZRT = VK_NULL_HANDLE;
        pSurface = VK_NULL_HANDLE;
        allocation = VK_NULL_HANDLE;
        return;
    }

    VkImageView viewToDestroy = VK_NULL_HANDLE;
    if (pRT != VK_NULL_HANDLE)
    {
        vkTextureUtils::UnregisterImage(pRT);
        if (HW.pDevice != VK_NULL_HANDLE && allocation != VK_NULL_HANDLE)
            viewToDestroy = pRT;
        pRT = VK_NULL_HANDLE;
    }
    if (pZRT != VK_NULL_HANDLE)
    {
        vkTextureUtils::UnregisterImage(pZRT);
        if (HW.pDevice != VK_NULL_HANDLE && allocation != VK_NULL_HANDLE)
            viewToDestroy = pZRT;
        pZRT = VK_NULL_HANDLE;
    }

    if (pSurface != VK_NULL_HANDLE && allocation != VK_NULL_HANDLE)
    {
        if (HW.Valid)
        {
            G_DeletionQueue.PushImage(HW.m_currentFrame, pSurface, allocation, viewToDestroy);
        }
        else
        {
            if (viewToDestroy != VK_NULL_HANDLE && HW.pDevice != VK_NULL_HANDLE)
                vkDestroyImageView(HW.pDevice, viewToDestroy, nullptr);
            if (HW.m_allocator != VK_NULL_HANDLE)
                vmaDestroyImage(HW.m_allocator, pSurface, allocation);
        }
        pSurface = VK_NULL_HANDLE;
        allocation = VK_NULL_HANDLE;
    }
    else if (viewToDestroy != VK_NULL_HANDLE)
    {
        if (HW.Valid)
            G_DeletionQueue.PushImage(HW.m_currentFrame, VK_NULL_HANDLE, VK_NULL_HANDLE, viewToDestroy);
        else if (HW.pDevice != VK_NULL_HANDLE)
            vkDestroyImageView(HW.pDevice, viewToDestroy, nullptr);
    }
}

void CRT::reset_begin()
{
    destroy();
}

void CRT::reset_end()
{
    create(cName.c_str(), dwWidth, dwHeight, fmt, sampleCount, n_slices);
}

void CRT::resolve_into(CRT& destination) const
{
    // In Vulkan, multisample resolve is typically handled via subpass resolve attachments or vkCmdResolveImage.
    // Stub for now when resolving intermediate RTs
    UNUSED(destination);
}

void resptrcode_crt::create(LPCSTR Name, u32 w, u32 h, D3DFORMAT f, u32 SampleCount, u32 slices_num, Flags32 flags)
{
    _set(RImplementation.Resources->_CreateRT(Name, w, h, f, SampleCount, slices_num, flags));
}
} // namespace xray::render::RENDER_NAMESPACE
