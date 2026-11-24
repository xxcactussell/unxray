#pragma once

#include "vkHW.h"
#include "vkState.h"
#include "Layers/xrRender/SH_Atomic.h"

namespace xray::render::RENDER_NAMESPACE
{
struct ShaderInputParam
{
    xr_string semantic;
    u32 index;
    u32 location;
};

void RegisterVSInputs(VkShaderModule vs, const xr_vector<ShaderInputParam>& inputs);
void UnregisterVSInputs(VkShaderModule vs);

class vkPipelineManager
{
public:
    vkPipelineManager();
    ~vkPipelineManager();

    bool Init();
    void Destroy();

    VkPipelineLayout GetPipelineLayout() const { return m_pipelineLayout; }
    VkDescriptorSetLayout GetDescriptorSetLayout() const { return m_descriptorSetLayout; }

    VkDescriptorSet AllocateDescriptorSet(u32 frameIndex);
    void ResetDescriptorPool(u32 frameIndex);
    bool AllocateConstantBufferSlice(u32 frameIndex, const void* data, size_t size, VkDescriptorBufferInfo& outBufferInfo);
    bool GetOrAllocateConstantBufferSlice(u32 frameIndex, u32 version, const void* data, size_t size, VkDescriptorBufferInfo& outBufferInfo);
    VkSampler GetDefaultSampler() const { return m_defaultSampler; }
    VkImageView GetDummyImageView() const { return m_dummyImageView; }

    VkPipeline GetOrCreatePipeline(
        VkShaderModule vs,
        VkShaderModule ps,
        VkShaderModule gs,
        SDeclaration* decl,
        u32 vbStride,
        D3DPRIMITIVETYPE topology,
        const vkState* state,
        u32 colorAttachmentCount,
        const VkFormat* colorFormats,
        VkFormat depthFormat);

    void BeginDynamicRendering(
        VkCommandBuffer cmd,
        u32 width, u32 height,
        VkImageView rt0, VkImageView rt1, VkImageView rt2, VkImageView rt3,
        VkImageView zb,
        bool clearColor = false,
        bool clearDepth = false,
        const VkClearColorValue* clearColorValue = nullptr,
        float clearDepthValue = 1.0f);

    void EndDynamicRendering(VkCommandBuffer cmd);
    bool IsInsideDynamicRendering() const { return m_insideDynamicRendering; }

private:
    struct PipelineKey
    {
        VkShaderModule vs{ VK_NULL_HANDLE };
        VkShaderModule ps{ VK_NULL_HANDLE };
        VkShaderModule gs{ VK_NULL_HANDLE };
        SDeclaration* decl{ nullptr };
        u32 vbStride{ 0 };
        D3DPRIMITIVETYPE topology{ D3DPT_TRIANGLELIST };
        BOOL blendEnable{ FALSE };
        D3DBLEND srcBlend{ D3DBLEND_ONE };
        D3DBLEND destBlend{ D3DBLEND_ZERO };
        u32 colorMask{ 0xF };
        u32 colorAttachmentCount{ 0 };
        VkFormat colorFormats[4]{ VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED };
        VkFormat depthFormat{ VK_FORMAT_UNDEFINED };

        bool operator<(const PipelineKey& o) const
        {
            if (vs != o.vs) return vs < o.vs;
            if (ps != o.ps) return ps < o.ps;
            if (gs != o.gs) return gs < o.gs;
            if (decl != o.decl) return decl < o.decl;
            if (vbStride != o.vbStride) return vbStride < o.vbStride;
            if (topology != o.topology) return topology < o.topology;
            if (blendEnable != o.blendEnable) return blendEnable < o.blendEnable;
            if (srcBlend != o.srcBlend) return srcBlend < o.srcBlend;
            if (destBlend != o.destBlend) return destBlend < o.destBlend;
            if (colorMask != o.colorMask) return colorMask < o.colorMask;
            if (colorAttachmentCount != o.colorAttachmentCount) return colorAttachmentCount < o.colorAttachmentCount;
            for (u32 i = 0; i < 4; ++i)
                if (colorFormats[i] != o.colorFormats[i]) return colorFormats[i] < o.colorFormats[i];
            return depthFormat < o.depthFormat;
        }
    };

    VkDescriptorSetLayout m_descriptorSetLayout{ VK_NULL_HANDLE };
    VkPipelineLayout m_pipelineLayout{ VK_NULL_HANDLE };
    static constexpr u32 MAX_POOLS = 4;
    VkDescriptorPool m_descriptorPools[MAX_POOLS]{ VK_NULL_HANDLE };

    static constexpr size_t UBO_POOL_SIZE = 128 * 1024 * 1024;
    VkBuffer m_uboBuffers[MAX_POOLS]{ VK_NULL_HANDLE };
    VmaAllocation m_uboAllocations[MAX_POOLS]{ VK_NULL_HANDLE };
    u8* m_uboMappedData[MAX_POOLS]{ nullptr };
    size_t m_uboOffset[MAX_POOLS]{ 0 };

    struct FrameConstantCache
    {
        u32 version{ 0 };
        VkDescriptorBufferInfo bufferInfo{};
    };
    FrameConstantCache m_constantCache[MAX_POOLS]{};

    VkSampler m_defaultSampler{ VK_NULL_HANDLE };
    VkImage m_dummyImage{ VK_NULL_HANDLE };
    VmaAllocation m_dummyAllocation{ VK_NULL_HANDLE };
    VkImageView m_dummyImageView{ VK_NULL_HANDLE };
    xr_map<PipelineKey, VkPipeline> m_pipelines;
    VkPipelineCache m_pipelineCache{ VK_NULL_HANDLE };

    bool m_insideDynamicRendering{ false };
};

extern vkPipelineManager G_PipelineManager;
} // namespace xray::render::RENDER_NAMESPACE
