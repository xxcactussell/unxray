#include "stdafx.h"
#include "xrCore/Media/Image.hpp"
#include "xrEngine/xrImage_Resampler.h"
#include "vkHW.h"
#include "vkTextureUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
using namespace XRay::Media;

void CRender::Screenshot(ScreenshotMode mode, pcstr name)
{
    if (!HW.Valid || HW.m_swapchain == VK_NULL_HANDLE)
        return;

    u32 w = HW.m_swapchainExtent.width;
    u32 h = HW.m_swapchainExtent.height;
    if (w == 0 || h == 0)
        return;

    VkImage srcImage = HW.GetCurrentBackBufferImage();
    if (srcImage == VK_NULL_HANDLE)
        return;

    vkDeviceWaitIdle(HW.pDevice);

    VkDeviceSize bufferSize = (VkDeviceSize)w * h * 4;
    VkBuffer stagingBuffer = VK_NULL_HANDLE;
    VmaAllocation stagingAllocation = VK_NULL_HANDLE;

    VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufInfo.size = bufferSize;
    bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_CPU_ONLY;
    allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VmaAllocationInfo vmaInfo{};
    VkResult res = vmaCreateBuffer(HW.m_allocator, &bufInfo, &allocInfo, &stagingBuffer, &stagingAllocation, &vmaInfo);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to allocate staging buffer for screenshot, res=%d", res);
        return;
    }

    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = HW.m_cmdPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;

    VkCommandBuffer copyCmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(HW.pDevice, &cmdAlloc, &copyCmd);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(copyCmd, &beginInfo);

    VkImageLayout currentLayout = vkTextureUtils::GetViewLayout(HW.GetCurrentBackBufferView());
    if (currentLayout == VK_IMAGE_LAYOUT_UNDEFINED)
        currentLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    barrier.oldLayout = currentLayout;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = srcImage;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.srcAccessMask = (currentLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR) ? 0 : (VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT);
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    vkCmdPipelineBarrier(copyCmd,
        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0, 0, nullptr, 0, nullptr, 1, &barrier);

    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = w;
    region.bufferImageHeight = h;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = { 0, 0, 0 };
    region.imageExtent = { w, h, 1 };

    vkCmdCopyImageToBuffer(copyCmd, srcImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, stagingBuffer, 1, &region);

    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.newLayout = currentLayout;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.dstAccessMask = (currentLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR) ? 0 : VK_ACCESS_MEMORY_READ_BIT;

    vkCmdPipelineBarrier(copyCmd,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        0, 0, nullptr, 0, nullptr, 1, &barrier);

    vkEndCommandBuffer(copyCmd);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &copyCmd;

    {
        ScopeLock lock(&HW.m_graphicsQueueLock);
        vkQueueSubmit(HW.m_graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(HW.m_graphicsQueue);
    }
    vkFreeCommandBuffers(HW.pDevice, HW.m_cmdPool, 1, &copyCmd);

    vmaInvalidateAllocation(HW.m_allocator, stagingAllocation, 0, VK_WHOLE_SIZE);

    const u8* srcData = (const u8*)vmaInfo.pMappedData;
    if (srcData)
    {
        xr_vector<u8> rgbData(w * h * 3);
        u8 maxR = 0, maxG = 0, maxB = 0;
        for (u32 y = 0; y < h; ++y)
        {
            for (u32 x = 0; x < w; ++x)
            {
                u32 srcIdx = (y * w + x) * 4;
                u32 dstIdx = (y * w + x) * 3;
                if (HW.m_swapchainFormat == VK_FORMAT_B8G8R8A8_UNORM)
                {
                    rgbData[dstIdx + 0] = srcData[srcIdx + 2]; // R
                    rgbData[dstIdx + 1] = srcData[srcIdx + 1]; // G
                    rgbData[dstIdx + 2] = srcData[srcIdx + 0]; // B
                }
                else
                {
                    rgbData[dstIdx + 0] = srcData[srcIdx + 0]; // R
                    rgbData[dstIdx + 1] = srcData[srcIdx + 1]; // G
                    rgbData[dstIdx + 2] = srcData[srcIdx + 2]; // B
                }
                if (rgbData[dstIdx + 0] > maxR) maxR = rgbData[dstIdx + 0];
                if (rgbData[dstIdx + 1] > maxG) maxG = rgbData[dstIdx + 1];
                if (rgbData[dstIdx + 2] > maxB) maxB = rgbData[dstIdx + 2];
            }
        }
        Msg("* [Vulkan] Screenshot pixel range: max R=%u, G=%u, B=%u", maxR, maxG, maxB);

        string_path buf;
        if (name && name[0])
        {
            xr_strcpy(buf, name);
        }
        else
        {
            string64 time;
            xr_sprintf(buf, sizeof(buf), "ss_%s_%s_(%s).jpg",
                Core.UserName, timestamp(time),
                g_pGameLevel ? g_pGameLevel->name().c_str() : "mainmenu");
        }

        IWriter* fs = nullptr;
        if (name && name[0])
            fs = FS.w_open(name);
        if (!fs)
            fs = FS.w_open("$screenshots$", buf);

        if (fs)
        {
            Image img{ w, h, rgbData.data(), ImageDataFormat::RGB8 };
            if (!img.SaveJPEG(*fs, 100, false))
                Log("! [Vulkan] Failed to save screenshot JPEG");
            else
                Msg("* [Vulkan] Screenshot saved: %s", buf);
            FS.w_close(fs);
        }
    }

    vmaDestroyBuffer(HW.m_allocator, stagingBuffer, stagingAllocation);
}
} // namespace xray::render::RENDER_NAMESPACE
