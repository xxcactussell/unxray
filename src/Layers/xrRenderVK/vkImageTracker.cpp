#include "stdafx.h"
#pragma hdrstop

#include "vkImageTracker.h"

namespace xray::render::RENDER_NAMESPACE {

vkImageTracker G_ImageTracker;

void vkImageTracker::RegisterImage(VkImage image, VkImageLayout initialLayout)
{
    ImageState state;
    state.layout = initialLayout;
    state.stage  = VK_PIPELINE_STAGE_2_NONE;
    state.access = VK_ACCESS_2_NONE;
    m_states[image] = state;
}

void vkImageTracker::UnregisterImage(VkImage image)
{
    m_states.erase(image);
}

void vkImageTracker::TransitionImage(VkCommandBuffer cmd, VkImage image,
                                     VkImageAspectFlags aspect,
                                     VkImageLayout newLayout,
                                     VkPipelineStageFlags2 newStage,
                                     VkAccessFlags2 newAccess)
{
    auto it = m_states.find(image);
    if (it == m_states.end())
    {
        // Auto-register as UNDEFINED if not tracked yet
        RegisterImage(image, VK_IMAGE_LAYOUT_UNDEFINED);
        it = m_states.find(image);
    }

    ImageState& cur = it->second;

    // Skip if already in target state
    if (cur.layout == newLayout && cur.stage == newStage && cur.access == newAccess)
        return;

    VkImageMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask        = cur.stage  != VK_PIPELINE_STAGE_2_NONE ? cur.stage  : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
    barrier.srcAccessMask       = cur.access;
    barrier.dstStageMask        = newStage;
    barrier.dstAccessMask       = newAccess;
    barrier.oldLayout           = cur.layout;
    barrier.newLayout           = newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image               = image;
    barrier.subresourceRange.aspectMask     = aspect;
    barrier.subresourceRange.baseMipLevel   = 0;
    barrier.subresourceRange.levelCount     = VK_REMAINING_MIP_LEVELS;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount     = VK_REMAINING_ARRAY_LAYERS;

    VkDependencyInfo depInfo{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    depInfo.imageMemoryBarrierCount = 1;
    depInfo.pImageMemoryBarriers    = &barrier;

    vkCmdPipelineBarrier2(cmd, &depInfo);

    cur.layout = newLayout;
    cur.stage  = newStage;
    cur.access = newAccess;
}

VkImageLayout vkImageTracker::GetLayout(VkImage image) const
{
    auto it = m_states.find(image);
    if (it != m_states.end())
        return it->second.layout;
    return VK_IMAGE_LAYOUT_UNDEFINED;
}

void vkImageTracker::Clear()
{
    m_states.clear();
}

} // namespace xray::render::RENDER_NAMESPACE
