#pragma once
#include "CommonTypes.h"
#include "vkHW.h"

namespace xray::render::RENDER_NAMESPACE {

class vkDeletionQueue {
public:
    struct BufferEntry { VkBuffer buffer; VmaAllocation allocation; };
    struct ImageEntry  { VkImage image;  VmaAllocation allocation; VkImageView view; };

    static constexpr u32 FRAMES = CHW::MAX_FRAMES_IN_FLIGHT;

    void PushBuffer(u32 frame, VkBuffer buf, VmaAllocation alloc);
    void PushImage(u32 frame, VkImage img, VmaAllocation alloc, VkImageView view = VK_NULL_HANDLE);
    void Flush(u32 frame);   // call after vkWaitForFences
    void FlushAll();

private:
    xr_vector<BufferEntry> m_buffers[FRAMES];
    xr_vector<ImageEntry>  m_images[FRAMES];
};

extern vkDeletionQueue G_DeletionQueue;
} // namespace xray::render::RENDER_NAMESPACE
