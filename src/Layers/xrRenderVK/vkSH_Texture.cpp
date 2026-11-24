#include "stdafx.h"
#pragma hdrstop

#include "../xrRender/ResourceManager.h"
#include "xrEngine/xrTheora_Surface.h"
#include "vkHW.h"
#include "vkPipelineManager.h"
#include "vkTextureUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
void resptrcode_texture::create(LPCSTR _name)
{
    _set(RImplementation.Resources->_CreateTexture(_name));
}

CTexture::CTexture()
{
    pSurface = VK_NULL_HANDLE;
    m_image = VK_NULL_HANDLE;
    m_allocation = VK_NULL_HANDLE;
    m_stagingBuffer = VK_NULL_HANDLE;
    m_stagingAlloc = VK_NULL_HANDLE;
    m_stagingData = nullptr;
    pAVI = nullptr;
    pTheora = nullptr;
    desc = VK_FORMAT_UNDEFINED;
    desc_cache = VK_NULL_HANDLE;
    seqMSPF = 0;
    flags.MemoryUsage = 0;
    flags.bLoaded = false;
    flags.bUser = false;
    flags.seqCycles = FALSE;
    m_material = 1.0f;
    bind = fastdelegate::FastDelegate2<CBackend&, u32>(this, &CTexture::apply_load);
}

CTexture::~CTexture()
{
    Unload();
    RImplementation.Resources->_DeleteTexture(this);
}

void CTexture::surface_set(VkImageView surf, VkImage img)
{
    pSurface = surf;
    m_image = img;
}

VkImageView CTexture::surface_get() const
{
    return pSurface;
}

void CTexture::PostLoad()
{
    if (pTheora)
        bind = fastdelegate::FastDelegate2<CBackend&, u32>(this, &CTexture::apply_theora);
    else if (!seqDATA.empty())
        bind = fastdelegate::FastDelegate2<CBackend&, u32>(this, &CTexture::apply_seq);
    else
        bind = fastdelegate::FastDelegate2<CBackend&, u32>(this, &CTexture::apply_normal);
}

void CTexture::apply_load(CBackend& cmd_list, u32 dwStage)
{
    if (!flags.bLoaded) Load();
    else PostLoad();
    bind(cmd_list, dwStage);
}

void CTexture::apply_normal(CBackend& cmd_list, u32 dwStage) const
{
    // Backend records image view descriptor binding for stage
}

void CTexture::apply_seq(CBackend& cmd_list, u32 dwStage)
{
    if (seqDATA.empty()) return;
    u32 frame = Device.dwTimeContinual / seqMSPF;
    u32 frames = static_cast<u32>(seqDATA.size());
    u32 idx = flags.seqCycles ? (frame % frames) : std::min(frame, frames - 1);
    pSurface = seqDATA[idx];
}

void CTexture::apply_theora(CBackend& cmd_list, u32 dwStage)
{
    if (pTheora && m_stagingData && pTheora->Update(m_play_time != 0xFFFFFFFF ? m_play_time : Device.dwTimeContinual))
    {
        int _pos = 0;
        pTheora->DecompressFrame(static_cast<u32*>(m_stagingData), 0, _pos);

        if (m_stagingAlloc && HW.m_allocator)
        {
            vmaFlushAllocation(HW.m_allocator, m_stagingAlloc, 0, m_width * m_height * 4);
        }

        if (HW.m_frameStarted && m_stagingBuffer != VK_NULL_HANDLE && m_image != VK_NULL_HANDLE)
        {
            VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

            extern vkPipelineManager G_PipelineManager;
            const bool wasInsideDynamicRendering = G_PipelineManager.IsInsideDynamicRendering();
            if (wasInsideDynamicRendering)
                G_PipelineManager.EndDynamicRendering(cmd);

            vkTextureUtils::TransitionViewLayout(cmd, pSurface, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

            VkBufferImageCopy region{};
            region.bufferOffset = 0;
            region.bufferRowLength = m_width;
            region.bufferImageHeight = m_height;
            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = 0;
            region.imageSubresource.baseArrayLayer = 0;
            region.imageSubresource.layerCount = 1;
            region.imageOffset = { 0, 0, 0 };
            region.imageExtent = { m_width, m_height, 1 };

            vkCmdCopyBufferToImage(cmd, m_stagingBuffer, m_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

            vkTextureUtils::TransitionViewLayout(cmd, pSurface, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    }
}

void CTexture::apply_avi(CBackend& cmd_list, u32 dwStage) const {}
void CTexture::set_slice(int slice) {}

void CTexture::Preload()
{
    m_bumpmap = RImplementation.Resources->m_textures_description.GetBumpName(cName);
    m_material = RImplementation.Resources->m_textures_description.GetMaterial(cName);
}

void CTexture::Load()
{
    flags.bLoaded = true;
    desc_cache = VK_NULL_HANDLE;

    if (strncmp(cName.c_str(), "$user$", 6) == 0)
    {
        flags.bUser = true;
        return;
    }

    if (flags.bUser)
        return;

    Preload();

    // Check for Theora video stream (.ogm)
    string_path fn;
    if (FS.exist(fn, "$game_textures$", cName.c_str(), ".ogm"))
    {
        pTheora = xr_new<CTheoraSurface>();
        if (!pTheora->Load(fn))
        {
            Msg("! [Vulkan] Can't open Theora video stream: %s", fn);
            xr_delete(pTheora);
        }
        else
        {
            m_width = pTheora->Width(false);
            m_height = pTheora->Height(false);
            desc = VK_FORMAT_R8G8B8A8_UNORM;

            VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.extent = { m_width, m_height, 1 };
            imageInfo.mipLevels = 1;
            imageInfo.arrayLayers = 1;
            imageInfo.format = desc;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

            VmaAllocationCreateInfo allocInfo{};
            allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

            VkResult res = vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &m_image, &m_allocation, nullptr);
            if (res == VK_SUCCESS)
            {
                VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
                viewInfo.image = m_image;
                viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
                viewInfo.format = desc;
                viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                viewInfo.subresourceRange.baseMipLevel = 0;
                viewInfo.subresourceRange.levelCount = 1;
                viewInfo.subresourceRange.baseArrayLayer = 0;
                viewInfo.subresourceRange.layerCount = 1;

                vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &pSurface);
                vkTextureUtils::RegisterImage(m_image, pSurface, desc, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED);

                // Create staging buffer for Theora frames
                VkBufferCreateInfo stagingBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
                stagingBufferInfo.size = m_width * m_height * 4;
                stagingBufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                stagingBufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

                VmaAllocationCreateInfo stagingAllocInfo{};
                stagingAllocInfo.usage = VMA_MEMORY_USAGE_AUTO;
                stagingAllocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo vmaInfo{};
                res = vmaCreateBuffer(HW.m_allocator, &stagingBufferInfo, &stagingAllocInfo, &m_stagingBuffer, &m_stagingAlloc, &vmaInfo);
                if (res == VK_SUCCESS)
                {
                    m_stagingData = vmaInfo.pMappedData;
                }
            }

            flags.MemoryUsage = m_width * m_height * 4;
            PostLoad();
            return;
        }
    }

    u32 msize = 0;
    pSurface = RImplementation.texture_load(cName.c_str(), msize, desc, &m_image, &m_allocation);
    flags.MemoryUsage = msize;

    PostLoad();
}

void CTexture::Unload()
{
    flags.bLoaded = false;

    if (m_stagingBuffer != VK_NULL_HANDLE && HW.m_allocator)
    {
        vmaDestroyBuffer(HW.m_allocator, m_stagingBuffer, m_stagingAlloc);
        m_stagingBuffer = VK_NULL_HANDLE;
        m_stagingAlloc = VK_NULL_HANDLE;
        m_stagingData = nullptr;
    }

    xr_delete(pTheora);

    if (m_allocation != VK_NULL_HANDLE && m_image != VK_NULL_HANDLE && HW.m_allocator)
    {
        if (pSurface != VK_NULL_HANDLE && HW.pDevice)
            vkDestroyImageView(HW.pDevice, pSurface, nullptr);
        vmaDestroyImage(HW.m_allocator, m_image, m_allocation);
    }
    else if (pSurface != VK_NULL_HANDLE && HW.pDevice && !flags.bUser)
    {
        vkDestroyImageView(HW.pDevice, pSurface, nullptr);
    }

    pSurface = VK_NULL_HANDLE;
    m_image = VK_NULL_HANDLE;
    m_allocation = VK_NULL_HANDLE;
}

void CTexture::desc_update()
{
    desc_cache = pSurface;
}

void CTexture::video_Play(BOOL looped, u32 _time)
{
    if (pTheora) pTheora->Play(looped, _time != 0xFFFFFFFF ? (m_play_time = _time) : Device.dwTimeContinual);
}

void CTexture::video_Pause(BOOL state) const
{
    if (pTheora) pTheora->Pause(state);
}

void CTexture::video_Stop() const
{
    if (pTheora) pTheora->Stop();
}

BOOL CTexture::video_IsPlaying() const
{
    return pTheora ? pTheora->IsPlaying() : FALSE;
}
} // namespace xray::render::RENDER_NAMESPACE
