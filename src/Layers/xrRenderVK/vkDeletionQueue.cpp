#include "stdafx.h"
#pragma hdrstop

#include "vkDeletionQueue.h"

namespace xray::render::RENDER_NAMESPACE {

vkDeletionQueue G_DeletionQueue;

void vkDeletionQueue::PushBuffer(u32 frame, VkBuffer buf, VmaAllocation alloc)
{
    VERIFY(frame < FRAMES);
    m_buffers[frame].push_back({ buf, alloc });
}

void vkDeletionQueue::PushImage(u32 frame, VkImage img, VmaAllocation alloc, VkImageView view)
{
    VERIFY(frame < FRAMES);
    m_images[frame].push_back({ img, alloc, view });
}

void vkDeletionQueue::Flush(u32 frame)
{
    VERIFY(frame < FRAMES);

    for (auto& entry : m_buffers[frame])
    {
        if (entry.buffer != VK_NULL_HANDLE)
            vmaDestroyBuffer(HW.m_allocator, entry.buffer, entry.allocation);
    }
    m_buffers[frame].clear();

    for (auto& entry : m_images[frame])
    {
        if (entry.view != VK_NULL_HANDLE)
            vkDestroyImageView(HW.pDevice, entry.view, nullptr);
        if (entry.image != VK_NULL_HANDLE)
            vmaDestroyImage(HW.m_allocator, entry.image, entry.allocation);
    }
    m_images[frame].clear();
}

void vkDeletionQueue::FlushAll()
{
    for (u32 i = 0; i < FRAMES; ++i)
        Flush(i);
}

} // namespace xray::render::RENDER_NAMESPACE
