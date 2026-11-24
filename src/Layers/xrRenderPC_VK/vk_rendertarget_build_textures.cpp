#include "stdafx.h"
#include "vk_rendertarget.h"
#include "Layers/xrRenderVK/vkHW.h"

namespace xray::render::RENDER_NAMESPACE
{
static void generate_jitter(u32* dest, u32 elem_count)
{
    const int cmax = 8;
    svector<Ivector2, cmax> samples;
    while (samples.size() < elem_count * 2)
    {
        Ivector2 test;
        test.set(Random.randI(0, 256), Random.randI(0, 256));
        BOOL valid = TRUE;
        for (auto& sample : samples)
        {
            int dist = _abs(test.x - sample.x) + _abs(test.y - sample.y);
            if (dist < 32)
            {
                valid = FALSE;
                break;
            }
        }
        if (valid)
            samples.push_back(test);
    }
    for (u32 it = 0; it < elem_count; it++, dest++)
        *dest = color_rgba(samples[2 * it].x, samples[2 * it].y, samples[2 * it + 1].y, samples[2 * it + 1].x);
}

static VkImageView create_vk_texture(u32 width, u32 height, u32 depth, VkFormat format, const void* data, size_t dataSize, VkImage& out_image, VmaAllocation& out_alloc)
{
    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = (depth > 1) ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D;
    imageInfo.format = format;
    imageInfo.extent = { width, height, depth };
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

    if (vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &out_image, &out_alloc, nullptr) != VK_SUCCESS)
        return VK_NULL_HANDLE;

    VkBuffer stagingBuffer = VK_NULL_HANDLE;
    VmaAllocation stagingAlloc = VK_NULL_HANDLE;
    VmaAllocationInfo stagingAllocInfo{};
    VkBufferCreateInfo stagingInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    stagingInfo.size = dataSize;
    stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    stagingInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo stagingAllocCi{};
    stagingAllocCi.usage = VMA_MEMORY_USAGE_AUTO;
    stagingAllocCi.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

    vmaCreateBuffer(HW.m_allocator, &stagingInfo, &stagingAllocCi, &stagingBuffer, &stagingAlloc, &stagingAllocInfo);
    CopyMemory(stagingAllocInfo.pMappedData, data, dataSize);

    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = HW.m_cmdPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(HW.pDevice, &cmdAlloc, &cmd);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &beginInfo);

    VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = out_image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

    VkBufferImageCopy copyRegion{};
    copyRegion.bufferOffset = 0;
    copyRegion.bufferRowLength = 0;
    copyRegion.bufferImageHeight = 0;
    copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copyRegion.imageSubresource.mipLevel = 0;
    copyRegion.imageSubresource.baseArrayLayer = 0;
    copyRegion.imageSubresource.layerCount = 1;
    copyRegion.imageOffset = { 0, 0, 0 };
    copyRegion.imageExtent = { width, height, depth };
    vkCmdCopyBufferToImage(cmd, stagingBuffer, out_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

    vkEndCommandBuffer(cmd);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;
    vkQueueSubmit(HW.m_graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(HW.m_graphicsQueue);

    vkFreeCommandBuffers(HW.pDevice, HW.m_cmdPool, 1, &cmd);
    vmaDestroyBuffer(HW.m_allocator, stagingBuffer, stagingAlloc);

    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = out_image;
    viewInfo.viewType = (depth > 1) ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = format;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    VkImageView view = VK_NULL_HANDLE;
    vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &view);
    return view;
}

void CRenderTarget::build_textures()
{
    // Build material(s)
    {
        static constexpr u32 RowPitch = TEX_material_LdotN * 2;
        static constexpr u32 SlicePitch = TEX_material_LdotH * RowPitch;
        u16 pBits[TEX_material_LdotN * TEX_material_LdotH * TEX_material_Count];
        for (u32 slice = 0; slice < TEX_material_Count; slice++)
        {
            for (u32 y = 0; y < TEX_material_LdotH; y++)
            {
                for (u32 x = 0; x < TEX_material_LdotN; x++)
                {
                    u16* p = (u16*)((u8*)(pBits) + slice * SlicePitch + y * RowPitch + x * 2);
                    float ld = float(x) / float(TEX_material_LdotN - 1);
                    float ls = float(y) / float(TEX_material_LdotH - 1) + EPS_S;
                    ls *= powf(ld, 1 / 32.f);
                    float fd, fs;

                    switch (slice)
                    {
                    case 0:
                    {
                        fd = powf(ld, 0.75f);
                        fs = powf(ls, 16.f) * .5f;
                    }
                    break;
                    case 1:
                    {
                        fd = powf(ld, 0.90f);
                        fs = powf(ls, 24.f);
                    }
                    break;
                    case 2:
                    {
                        fd = ld;
                        fs = powf(ls * 1.01f, 128.f);
                    }
                    break;
                    case 3:
                    {
                        float s0 = _abs(1 - _abs(0.05f * _sin(33.f * ld) + ld - ls));
                        float s1 = _abs(1 - _abs(0.05f * _cos(33.f * ld * ls) + ld - ls));
                        float s2 = _abs(1 - _abs(ld - ls));
                        fd = ld;
                        fs = powf(_max(_max(s0, s1), s2), 24.f);
                        fs *= powf(ld, 1 / 7.f);
                    }
                    break;
                    default: fd = fs = 0;
                    }
                    s32 _d = clampr(iFloor(fd * 255.5f), 0, 255);
                    s32 _s = clampr(iFloor(fs * 255.5f), 0, 255);
                    if (y == (TEX_material_LdotH - 1) && x == (TEX_material_LdotN - 1))
                    {
                        _d = 255;
                        _s = 255;
                    }
                    *p = u16(_s * 256 + _d);
                }
            }
        }

        VkImage matImage = VK_NULL_HANDLE;
        VmaAllocation matAlloc = VK_NULL_HANDLE;
        t_material_surf = create_vk_texture(TEX_material_LdotN, TEX_material_LdotH, TEX_material_Count, VK_FORMAT_R8G8_UNORM, pBits, sizeof(pBits), matImage, matAlloc);
        t_material = RImplementation.Resources->_CreateTexture(r2_material);
        t_material->surface_set(t_material_surf, matImage);
    }

    // Build noise table
    {
        static const int sampleSize = 4;
        u32 tempData[TEX_jitter_count][TEX_jitter * TEX_jitter];

        static const u32 Pitch = TEX_jitter * sampleSize;
        for (u32 y = 0; y < TEX_jitter; y++)
        {
            for (u32 x = 0; x < TEX_jitter; x++)
            {
                u32 data[TEX_jitter_count - 1];
                generate_jitter(data, TEX_jitter_count - 1);
                for (u32 it2 = 0; it2 < TEX_jitter_count - 1; it2++)
                {
                    u32* p = (u32*)((u8*)(tempData[it2]) + y * Pitch + x * 4);
                    *p = data[it2];
                }
            }
        }

        for (u32 it1 = 0; it1 < TEX_jitter_count - 1; it1++)
        {
            string_path name;
            xr_sprintf(name, "%s%d", r2_jitter, it1);

            VkImage noiseImg = VK_NULL_HANDLE;
            VmaAllocation noiseAlloc = VK_NULL_HANDLE;
            t_noise_surf[it1] = create_vk_texture(TEX_jitter, TEX_jitter, 1, VK_FORMAT_R8G8B8A8_UNORM, tempData[it1], sizeof(tempData[it1]), noiseImg, noiseAlloc);
            t_noise[it1] = RImplementation.Resources->_CreateTexture(name);
            t_noise[it1]->surface_set(t_noise_surf[it1], noiseImg);
        }

        // generate HBAO jitter texture
        float tempDataHBAO[TEX_jitter * TEX_jitter * 4];
        static const int HBAOPitch = TEX_jitter * sampleSize * sizeof(float);
        for (u32 y = 0; y < TEX_jitter; y++)
        {
            for (u32 x = 0; x < TEX_jitter; x++)
            {
                float numDir = 1.0f;
                switch (ps_r_ssao)
                {
                case 1: numDir = 4.0f; break;
                case 2: numDir = 6.0f; break;
                case 3: numDir = 8.0f; break;
                }
                float angle = 2 * PI * Random.randF(0.0f, 1.0f) / numDir;
                float dist = Random.randF(0.0f, 1.0f);

                float* p = (float*)((u8*)(tempDataHBAO) + y * HBAOPitch + x * 4 * sizeof(float));
                *p = (float)_cos(angle);
                *(p + 1) = (float)_sin(angle);
                *(p + 2) = (float)dist;
                *(p + 3) = 0;
            }
        }

        int it = TEX_jitter_count - 1;
        string_path name;
        xr_sprintf(name, "%s%d", r2_jitter, it);
        VkImage hbaoImg = VK_NULL_HANDLE;
        VmaAllocation hbaoAlloc = VK_NULL_HANDLE;
        t_noise_surf[it] = create_vk_texture(TEX_jitter, TEX_jitter, 1, VK_FORMAT_R32G32B32A32_SFLOAT, tempDataHBAO, sizeof(tempDataHBAO), hbaoImg, hbaoAlloc);
        t_noise[it] = RImplementation.Resources->_CreateTexture(name);
        t_noise[it]->surface_set(t_noise_surf[it], hbaoImg);

        // Mipped noise
        VkImage mippedImg = VK_NULL_HANDLE;
        VmaAllocation mippedAlloc = VK_NULL_HANDLE;
        t_noise_surf_mipped = create_vk_texture(TEX_jitter, TEX_jitter, 1, VK_FORMAT_R8G8B8A8_UNORM, tempData[0], sizeof(tempData[0]), mippedImg, mippedAlloc);
        t_noise_mipped = RImplementation.Resources->_CreateTexture(r2_jitter_mipped);
        t_noise_mipped->surface_set(t_noise_surf_mipped, mippedImg);
    }
}
} // namespace xray::render::RENDER_NAMESPACE
