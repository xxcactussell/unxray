#include "stdafx.h"
#include "vkTextureUtils.h"
#include "vkHW.h"

#include <gli/gli.hpp>
#include <gli/dx.hpp>

namespace xray::render::RENDER_NAMESPACE
{
void fix_texture_name(pstr fn)
{
    pstr _ext = strext(fn);
    if (_ext &&
        (0 == xr_stricmp(_ext, ".tga") ||
            0 == xr_stricmp(_ext, ".dds") ||
            0 == xr_stricmp(_ext, ".bmp") ||
            0 == xr_stricmp(_ext, ".ogm")))
        *_ext = 0;
}

static VkFormat GliToVkFormat(gli::format format)
{
    switch (format)
    {
    case gli::FORMAT_RGBA_DXT1_UNORM_BLOCK8:
    case gli::FORMAT_RGB_DXT1_UNORM_BLOCK8:
        return VK_FORMAT_BC1_RGBA_UNORM_BLOCK;
    case gli::FORMAT_RGBA_DXT1_SRGB_BLOCK8:
    case gli::FORMAT_RGB_DXT1_SRGB_BLOCK8:
        return VK_FORMAT_BC1_RGBA_SRGB_BLOCK;
    case gli::FORMAT_RGBA_DXT3_UNORM_BLOCK16:
        return VK_FORMAT_BC2_UNORM_BLOCK;
    case gli::FORMAT_RGBA_DXT3_SRGB_BLOCK16:
        return VK_FORMAT_BC2_SRGB_BLOCK;
    case gli::FORMAT_RGBA_DXT5_UNORM_BLOCK16:
        return VK_FORMAT_BC3_UNORM_BLOCK;
    case gli::FORMAT_RGBA_DXT5_SRGB_BLOCK16:
        return VK_FORMAT_BC3_SRGB_BLOCK;
    case gli::FORMAT_R_ATI1N_UNORM_BLOCK8:
    case gli::FORMAT_R_ATI1N_SNORM_BLOCK8:
        return VK_FORMAT_BC4_UNORM_BLOCK;
    case gli::FORMAT_RG_ATI2N_UNORM_BLOCK16:
    case gli::FORMAT_RG_ATI2N_SNORM_BLOCK16:
        return VK_FORMAT_BC5_UNORM_BLOCK;
    case gli::FORMAT_RGBA_BP_UNORM_BLOCK16:
        return VK_FORMAT_BC7_UNORM_BLOCK;
    case gli::FORMAT_RGBA_BP_SRGB_BLOCK16:
        return VK_FORMAT_BC7_SRGB_BLOCK;
    case gli::FORMAT_RGBA8_UNORM_PACK8:
        return VK_FORMAT_R8G8B8A8_UNORM;
    case gli::FORMAT_RGBA8_SRGB_PACK8:
        return VK_FORMAT_R8G8B8A8_SRGB;
    case gli::FORMAT_BGRA8_UNORM_PACK8:
        return VK_FORMAT_B8G8R8A8_UNORM;
    case gli::FORMAT_BGRA8_SRGB_PACK8:
        return VK_FORMAT_B8G8R8A8_SRGB;
    case gli::FORMAT_R8_UNORM_PACK8:
        return VK_FORMAT_R8_UNORM;
    case gli::FORMAT_L8_UNORM_PACK8:
        return VK_FORMAT_R8_UNORM;
    case gli::FORMAT_A8_UNORM_PACK8:
        return VK_FORMAT_R8_UNORM;
    case gli::FORMAT_RG8_UNORM_PACK8:
        return VK_FORMAT_R8G8_UNORM;
    case gli::FORMAT_LA8_UNORM_PACK8:
        return VK_FORMAT_R8G8_UNORM;
    case gli::FORMAT_BGR8_UNORM_PACK32:
        return VK_FORMAT_B8G8R8A8_UNORM;
    case gli::FORMAT_BGR8_SRGB_PACK32:
        return VK_FORMAT_B8G8R8A8_SRGB;
    case gli::FORMAT_R5G6B5_UNORM_PACK16:
        return VK_FORMAT_R5G6B5_UNORM_PACK16;
    case gli::FORMAT_B5G6R5_UNORM_PACK16:
        return VK_FORMAT_B5G6R5_UNORM_PACK16;
    case gli::FORMAT_BGR5A1_UNORM_PACK16:
        return VK_FORMAT_B5G5R5A1_UNORM_PACK16;
    case gli::FORMAT_RGB5A1_UNORM_PACK16:
        return VK_FORMAT_A1R5G5B5_UNORM_PACK16;
    case gli::FORMAT_RGBA16_SFLOAT_PACK16:
        return VK_FORMAT_R16G16B16A16_SFLOAT;
    case gli::FORMAT_RGBA32_SFLOAT_PACK32:
        return VK_FORMAT_R32G32B32A32_SFLOAT;
    default:
        return VK_FORMAT_R8G8B8A8_UNORM;
    }
}

VkImageView CRender::texture_load(LPCSTR fRName, u32& ret_msize, VkFormat& ret_desc, VkImage* out_image, VmaAllocation* out_alloc)
{
    ret_msize = 0;
    ret_desc = VK_FORMAT_UNDEFINED;
    R_ASSERT1_CURE(fRName && fRName[0], { return VK_NULL_HANDLE; });

    if (strncmp(fRName, "$user$", 6) == 0)
        return VK_NULL_HANDLE;

    string_path fn;
    {
        string_path fname;
        xr_strcpy(fname, fRName);
        fix_texture_name(fname);

        if (!FS.exist(fn, "$game_textures$", fname, ".dds") && strstr(fname, "_bump"))
        {
            if (strstr(fname, "_bump#"))
                R_ASSERT1_CURE(FS.exist(fn, "$game_textures$", "ed\\ed_dummy_bump#", ".dds"), return VK_NULL_HANDLE);
            else
                R_ASSERT1_CURE(FS.exist(fn, "$game_textures$", "ed\\ed_dummy_bump", ".dds"), return VK_NULL_HANDLE);
        }
        else
        {
            bool exist = false;
            for (cpcstr folder : { "$level$", "$game_saves$", "$game_textures$" })
            {
                exist = FS.exist(fn, folder, fname, ".dds");
                if (exist)
                    break;
            }

            if (!exist)
            {
                if (!FS.exist(fn, "$game_textures$", "ed\\ed_not_existing_texture", ".dds"))
                    return VK_NULL_HANDLE;
            }
        }
    }

    IReader* S = FS.r_open(fn);
    R_ASSERT2_CURE(S, fn, { return VK_NULL_HANDLE; });
    size_t img_size = S->length();

    gli::texture texture = gli::load((char*)S->pointer(), img_size);
    FS.r_close(S);

    if (texture.empty())
    {
        Msg("! [Vulkan] Failed to parse texture: '%s'", fn);
        return VK_NULL_HANDLE;
    }

    VkFormat vkFmt = GliToVkFormat(texture.format());
    ret_desc = vkFmt;
    ret_msize = static_cast<u32>(texture.size());

    glm::tvec3<size_t> const tex_extent(texture.extent());
    bool isCube = gli::is_target_cube(texture.target());
    u32 arrayLayers = static_cast<u32>(texture.layers() * (isCube ? 6 : 1));

    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = static_cast<u32>(tex_extent.x);
    imageInfo.extent.height = static_cast<u32>(tex_extent.y);
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = static_cast<u32>(texture.levels());
    imageInfo.arrayLayers = arrayLayers;
    imageInfo.format = vkFmt;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (isCube)
        imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

    VkImage image = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VkResult res = vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &image, &allocation, nullptr);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create image for texture '%s'! Result = %d", fn, res);
        return VK_NULL_HANDLE;
    }

    // Staging buffer
    VkBuffer stagingBuffer = VK_NULL_HANDLE;
    VmaAllocation stagingAlloc = VK_NULL_HANDLE;
    VmaAllocationInfo stagingAllocInfo{};

    VkBufferCreateInfo stagingInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    stagingInfo.size = texture.size();
    stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    stagingInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo stagingAllocCi{};
    stagingAllocCi.usage = VMA_MEMORY_USAGE_AUTO;
    stagingAllocCi.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

    vmaCreateBuffer(HW.m_allocator, &stagingInfo, &stagingAllocCi, &stagingBuffer, &stagingAlloc, &stagingAllocInfo);

    xr_vector<VkBufferImageCopy> copyRegions;
    size_t currentOffset = 0;

    for (size_t layer = 0; layer < texture.layers(); ++layer)
    {
        for (size_t face = 0; face < texture.faces(); ++face)
        {
            for (size_t level = 0; level < texture.levels(); ++level)
            {
                size_t faceSize = texture.size(level);
                CopyMemory((u8*)stagingAllocInfo.pMappedData + currentOffset, texture.data(layer, face, level), faceSize);

                VkBufferImageCopy region{};
                region.bufferOffset = currentOffset;
                region.bufferRowLength = 0;
                region.bufferImageHeight = 0;
                region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.mipLevel = static_cast<u32>(level);
                region.imageSubresource.baseArrayLayer = static_cast<u32>(layer * (isCube ? 6 : 1) + face);
                region.imageSubresource.layerCount = 1;
                region.imageOffset = { 0, 0, 0 };
                glm::tvec3<size_t> const levelExtent(texture.extent(level));
                region.imageExtent = { static_cast<u32>(levelExtent.x), static_cast<u32>(levelExtent.y), 1 };

                copyRegions.push_back(region);
                currentOffset += faceSize;
            }
        }
    }

    // Execute transfer commands
    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = HW.m_cmdPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;

    VkCommandBuffer transferCmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(HW.pDevice, &cmdAlloc, &transferCmd);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(transferCmd, &beginInfo);

    // Transition to TRANSFER_DST_OPTIMAL
    VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = static_cast<u32>(texture.levels());
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = arrayLayers;
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    vkCmdPipelineBarrier(transferCmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
        0, 0, nullptr, 0, nullptr, 1, &barrier);

    vkCmdCopyBufferToImage(transferCmd, stagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        static_cast<u32>(copyRegions.size()), copyRegions.data());

    // Transition to SHADER_READ_ONLY_OPTIMAL
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(transferCmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0, 0, nullptr, 0, nullptr, 1, &barrier);

    vkEndCommandBuffer(transferCmd);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &transferCmd;
    vkQueueSubmit(HW.m_graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(HW.m_graphicsQueue);

    vkFreeCommandBuffers(HW.pDevice, HW.m_cmdPool, 1, &transferCmd);
    vmaDestroyBuffer(HW.m_allocator, stagingBuffer, stagingAlloc);

    // Create Image View
    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = image;
    viewInfo.viewType = isCube ? VK_IMAGE_VIEW_TYPE_CUBE : VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = vkFmt;
    if (texture.format() == gli::FORMAT_L8_UNORM_PACK8)
    {
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_ONE;
    }
    else if (texture.format() == gli::FORMAT_A8_UNORM_PACK8)
    {
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_ZERO;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_ZERO;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_ZERO;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_R;
    }
    else if (texture.format() == gli::FORMAT_LA8_UNORM_PACK8)
    {
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_R;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_G;
    }
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = static_cast<u32>(texture.levels());
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = arrayLayers;

    VkImageView imageView = VK_NULL_HANDLE;
    res = vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &imageView);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create image view for texture '%s'! Result = %d", fn, res);
        vmaDestroyImage(HW.m_allocator, image, allocation);
        return VK_NULL_HANDLE;
    }

    if (out_image) *out_image = image;
    if (out_alloc) *out_alloc = allocation;

    vkTextureUtils::RegisterImage(image, imageView, vkFmt, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        static_cast<u32>(tex_extent.x), static_cast<u32>(tex_extent.y));

    return imageView;
}
} // namespace xray::render::RENDER_NAMESPACE
