#pragma once
#include "CommonTypes.h"

namespace xray::render::RENDER_NAMESPACE {

struct ImageState {
    VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
    VkAccessFlags2 access = VK_ACCESS_2_NONE;
};

class vkImageTracker {
public:
    void RegisterImage(VkImage image, VkImageLayout initialLayout = VK_IMAGE_LAYOUT_UNDEFINED);
    void UnregisterImage(VkImage image);
    // Inserts VkImageMemoryBarrier2 if state differs.
    void TransitionImage(VkCommandBuffer cmd, VkImage image,
                         VkImageAspectFlags aspect,
                         VkImageLayout newLayout,
                         VkPipelineStageFlags2 newStage,
                         VkAccessFlags2 newAccess);
    VkImageLayout GetLayout(VkImage image) const;
    void Clear();
private:
    xr_map<VkImage, ImageState> m_states;
};

extern vkImageTracker G_ImageTracker;
} // namespace xray::render::RENDER_NAMESPACE
