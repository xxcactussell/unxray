#include "stdafx.h"
#pragma hdrstop

#include "vkPipelineManager.h"
#include "vkStateUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
static xr_map<VkShaderModule, xr_vector<ShaderInputParam>> s_vsInputs;

void RegisterVSInputs(VkShaderModule vs, const xr_vector<ShaderInputParam>& inputs)
{
    s_vsInputs[vs] = inputs;
}

void UnregisterVSInputs(VkShaderModule vs)
{
    s_vsInputs.erase(vs);
}

static const char* GetSemanticName(BYTE usage)
{
    switch (usage)
    {
    case D3DDECLUSAGE_POSITION:     return "POSITION";
    case D3DDECLUSAGE_BLENDWEIGHT:  return "BLENDWEIGHT";
    case D3DDECLUSAGE_BLENDINDICES: return "BLENDINDICES";
    case D3DDECLUSAGE_NORMAL:       return "NORMAL";
    case D3DDECLUSAGE_PSIZE:        return "PSIZE";
    case D3DDECLUSAGE_TEXCOORD:     return "TEXCOORD";
    case D3DDECLUSAGE_TANGENT:      return "TANGENT";
    case D3DDECLUSAGE_BINORMAL:     return "BINORMAL";
    case D3DDECLUSAGE_TESSFACTOR:   return "TESSFACTOR";
    case D3DDECLUSAGE_POSITIONT:    return "POSITIONT";
    case D3DDECLUSAGE_COLOR:        return "COLOR";
    case D3DDECLUSAGE_FOG:          return "FOG";
    case D3DDECLUSAGE_DEPTH:        return "DEPTH";
    case D3DDECLUSAGE_SAMPLE:       return "SAMPLE";
    default:                        return "";
    }
}

vkPipelineManager G_PipelineManager;

static VkPrimitiveTopology TranslateTopologyVK(D3DPRIMITIVETYPE T)
{
    switch (T)
    {
    case D3DPT_POINTLIST: return VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    case D3DPT_LINELIST: return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    case D3DPT_LINESTRIP: return VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
    case D3DPT_TRIANGLELIST: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    case D3DPT_TRIANGLESTRIP: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    case D3DPT_TRIANGLEFAN: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN;
    default: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    }
}

static VkCullModeFlags TranslateCullMode(D3DCULL mode)
{
    switch (mode)
    {
    case D3DCULL_NONE: return VK_CULL_MODE_NONE;
    case D3DCULL_CW: return VK_CULL_MODE_FRONT_BIT;
    case D3DCULL_CCW: return VK_CULL_MODE_BACK_BIT;
    default: return VK_CULL_MODE_BACK_BIT;
    }
}

static VkCompareOp TranslateCompareOp(D3DCMPFUNC func)
{
    switch (func)
    {
    case D3DCMP_NEVER: return VK_COMPARE_OP_NEVER;
    case D3DCMP_LESS: return VK_COMPARE_OP_LESS;
    case D3DCMP_EQUAL: return VK_COMPARE_OP_EQUAL;
    case D3DCMP_LESSEQUAL: return VK_COMPARE_OP_LESS_OR_EQUAL;
    case D3DCMP_GREATER: return VK_COMPARE_OP_GREATER;
    case D3DCMP_NOTEQUAL: return VK_COMPARE_OP_NOT_EQUAL;
    case D3DCMP_GREATEREQUAL: return VK_COMPARE_OP_GREATER_OR_EQUAL;
    case D3DCMP_ALWAYS: return VK_COMPARE_OP_ALWAYS;
    default: return VK_COMPARE_OP_LESS_OR_EQUAL;
    }
}

static VkBlendFactor TranslateBlendFactor(D3DBLEND factor)
{
    switch (factor)
    {
    case D3DBLEND_ZERO: return VK_BLEND_FACTOR_ZERO;
    case D3DBLEND_ONE: return VK_BLEND_FACTOR_ONE;
    case D3DBLEND_SRCCOLOR: return VK_BLEND_FACTOR_SRC_COLOR;
    case D3DBLEND_INVSRCCOLOR: return VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
    case D3DBLEND_SRCALPHA: return VK_BLEND_FACTOR_SRC_ALPHA;
    case D3DBLEND_INVSRCALPHA: return VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    case D3DBLEND_DESTALPHA: return VK_BLEND_FACTOR_DST_ALPHA;
    case D3DBLEND_INVDESTALPHA: return VK_BLEND_FACTOR_ONE_MINUS_DST_ALPHA;
    case D3DBLEND_DESTCOLOR: return VK_BLEND_FACTOR_DST_COLOR;
    case D3DBLEND_INVDESTCOLOR: return VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR;
    default: return VK_BLEND_FACTOR_ONE;
    }
}

static VkFormat TranslateVertexFormat(D3DDECLTYPE type)
{
    switch (type)
    {
    case D3DDECLTYPE_FLOAT1: return VK_FORMAT_R32_SFLOAT;
    case D3DDECLTYPE_FLOAT2: return VK_FORMAT_R32G32_SFLOAT;
    case D3DDECLTYPE_FLOAT3: return VK_FORMAT_R32G32B32_SFLOAT;
    case D3DDECLTYPE_FLOAT4: return VK_FORMAT_R32G32B32A32_SFLOAT;
    case D3DDECLTYPE_D3DCOLOR: return VK_FORMAT_R8G8B8A8_UNORM;
    case D3DDECLTYPE_UBYTE4: return VK_FORMAT_R8G8B8A8_UINT;
    case D3DDECLTYPE_UBYTE4N: return VK_FORMAT_R8G8B8A8_UNORM;
    case D3DDECLTYPE_SHORT2: return VK_FORMAT_R16G16_SINT;
    case D3DDECLTYPE_SHORT2N: return VK_FORMAT_R16G16_SNORM;
    case D3DDECLTYPE_SHORT4: return VK_FORMAT_R16G16B16A16_SINT;
    case D3DDECLTYPE_SHORT4N: return VK_FORMAT_R16G16B16A16_SNORM;
    case D3DDECLTYPE_USHORT2N: return VK_FORMAT_R16G16_UNORM;
    case D3DDECLTYPE_USHORT4N: return VK_FORMAT_R16G16B16A16_UNORM;
    case D3DDECLTYPE_FLOAT16_2: return VK_FORMAT_R16G16_SFLOAT;
    case D3DDECLTYPE_FLOAT16_4: return VK_FORMAT_R16G16B16A16_SFLOAT;
    default: return VK_FORMAT_R32G32B32A32_SFLOAT;
    }
}

vkPipelineManager::vkPipelineManager() = default;

vkPipelineManager::~vkPipelineManager()
{
    Destroy();
}

bool vkPipelineManager::Init()
{
    if (!HW.pDevice)
        return false;

    if (m_pipelineLayout != VK_NULL_HANDLE)
        return true;

    VkDescriptorSetLayoutBinding bindings[80]{};
    VkDescriptorBindingFlags bindingFlags[80]{};
    for (u32 i = 0; i < 16; ++i) // 0..15: uniform buffers (b0..b15 -> binding 0..15)
    {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS;
        bindingFlags[i] = VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT;
    }
    for (u32 i = 16; i < 48; ++i) // 16..47: sampled images (t0..t31 -> binding 16..47)
    {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS;
        bindingFlags[i] = VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT;
    }
    for (u32 i = 48; i < 64; ++i) // 48..63: samplers (s0..s15 -> binding 48..63)
    {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS;
        bindingFlags[i] = VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT;
    }
    for (u32 i = 64; i < 80; ++i) // 64..79: UAV/Storage (u0..u15 -> binding 64..79)
    {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS;
        bindingFlags[i] = VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT;
    }

    VkDescriptorSetLayoutBindingFlagsCreateInfo flagsInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO };
    flagsInfo.bindingCount = 80;
    flagsInfo.pBindingFlags = bindingFlags;

    VkDescriptorSetLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    layoutInfo.pNext = &flagsInfo;
    layoutInfo.bindingCount = 80;
    layoutInfo.pBindings = bindings;

    VkResult res = vkCreateDescriptorSetLayout(HW.pDevice, &layoutInfo, nullptr, &m_descriptorSetLayout);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create descriptor set layout! Result = %d", res);
        return false;
    }

    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS;
    pushRange.offset = 0;
    pushRange.size = 128;

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;

    res = vkCreatePipelineLayout(HW.pDevice, &pipelineLayoutInfo, nullptr, &m_pipelineLayout);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create pipeline layout! Result = %d", res);
        return false;
    }

    // Default bilinear repeat sampler
    VkSamplerCreateInfo samplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.maxLod = 16.0f;
    vkCreateSampler(HW.pDevice, &samplerInfo, nullptr, &m_defaultSampler);

    // Dummy 1x1 2D texture (white) for unbound texture descriptor slots
    {
        VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = 1;
        imageInfo.extent.height = 1;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;

        res = vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &m_dummyImage, &m_dummyAllocation, nullptr);
        if (res == VK_SUCCESS)
        {
            // Create staging buffer to upload 1x1 white pixel (0xFFFFFFFF)
            VkBufferCreateInfo stagingBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            stagingBufferInfo.size = 4;
            stagingBufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            stagingBufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

            VmaAllocationCreateInfo stagingAllocInfo{};
            stagingAllocInfo.usage = VMA_MEMORY_USAGE_AUTO;
            stagingAllocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

            VkBuffer stagingBuffer = VK_NULL_HANDLE;
            VmaAllocation stagingAlloc = VK_NULL_HANDLE;
            VmaAllocationInfo stagingVmaInfo{};
            if (vmaCreateBuffer(HW.m_allocator, &stagingBufferInfo, &stagingAllocInfo, &stagingBuffer, &stagingAlloc, &stagingVmaInfo) == VK_SUCCESS && stagingVmaInfo.pMappedData)
            {
                u32 whitePixel = 0xFFFFFFFF;
                CopyMemory(stagingVmaInfo.pMappedData, &whitePixel, sizeof(whitePixel));
                vmaFlushAllocation(HW.m_allocator, stagingAlloc, 0, 4);

                VkCommandBufferAllocateInfo cmdAllocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
                cmdAllocInfo.commandPool = HW.m_cmdPool;
                cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmdAllocInfo.commandBufferCount = 1;

                VkCommandBuffer transferCmd = VK_NULL_HANDLE;
                vkAllocateCommandBuffers(HW.pDevice, &cmdAllocInfo, &transferCmd);

                VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
                beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(transferCmd, &beginInfo);

                VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
                barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.image = m_dummyImage;
                barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                barrier.subresourceRange.baseMipLevel = 0;
                barrier.subresourceRange.levelCount = 1;
                barrier.subresourceRange.baseArrayLayer = 0;
                barrier.subresourceRange.layerCount = 1;
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

                vkCmdPipelineBarrier(transferCmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                    0, 0, nullptr, 0, nullptr, 1, &barrier);

                VkBufferImageCopy region{};
                region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.layerCount = 1;
                region.imageExtent = { 1, 1, 1 };

                vkCmdCopyBufferToImage(transferCmd, stagingBuffer, m_dummyImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

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
            }

            VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = m_dummyImage;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.baseMipLevel = 0;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.baseArrayLayer = 0;
            viewInfo.subresourceRange.layerCount = 1;

            vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &m_dummyImageView);
        }
    }

    // Dummy 1x1 Cube texture (white) for unbound cube texture descriptor slots
    {
        VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = 1;
        imageInfo.extent.height = 1;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 6;
        imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;

        res = vmaCreateImage(HW.m_allocator, &imageInfo, &allocInfo, &m_dummyCubeImage, &m_dummyCubeAllocation, nullptr);
        if (res == VK_SUCCESS)
        {
            VkBufferCreateInfo stagingBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            stagingBufferInfo.size = 4 * 6;
            stagingBufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            stagingBufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

            VmaAllocationCreateInfo stagingAllocInfo{};
            stagingAllocInfo.usage = VMA_MEMORY_USAGE_AUTO;
            stagingAllocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

            VkBuffer stagingBuffer = VK_NULL_HANDLE;
            VmaAllocation stagingAlloc = VK_NULL_HANDLE;
            VmaAllocationInfo stagingVmaInfo{};
            if (vmaCreateBuffer(HW.m_allocator, &stagingBufferInfo, &stagingAllocInfo, &stagingBuffer, &stagingAlloc, &stagingVmaInfo) == VK_SUCCESS && stagingVmaInfo.pMappedData)
            {
                u32 whitePixels[6] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
                CopyMemory(stagingVmaInfo.pMappedData, whitePixels, sizeof(whitePixels));
                vmaFlushAllocation(HW.m_allocator, stagingAlloc, 0, sizeof(whitePixels));

                VkCommandBufferAllocateInfo cmdAllocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
                cmdAllocInfo.commandPool = HW.m_cmdPool;
                cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmdAllocInfo.commandBufferCount = 1;

                VkCommandBuffer transferCmd = VK_NULL_HANDLE;
                vkAllocateCommandBuffers(HW.pDevice, &cmdAllocInfo, &transferCmd);

                VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
                beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(transferCmd, &beginInfo);

                VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
                barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.image = m_dummyCubeImage;
                barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                barrier.subresourceRange.baseMipLevel = 0;
                barrier.subresourceRange.levelCount = 1;
                barrier.subresourceRange.baseArrayLayer = 0;
                barrier.subresourceRange.layerCount = 6;
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

                vkCmdPipelineBarrier(transferCmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                    0, 0, nullptr, 0, nullptr, 1, &barrier);

                VkBufferImageCopy regions[6]{};
                for (u32 face = 0; face < 6; ++face)
                {
                    regions[face].bufferOffset = face * 4;
                    regions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    regions[face].imageSubresource.baseArrayLayer = face;
                    regions[face].imageSubresource.layerCount = 1;
                    regions[face].imageExtent = { 1, 1, 1 };
                }

                vkCmdCopyBufferToImage(transferCmd, stagingBuffer, m_dummyCubeImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 6, regions);

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
            }

            VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = m_dummyCubeImage;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
            viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.baseMipLevel = 0;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.baseArrayLayer = 0;
            viewInfo.subresourceRange.layerCount = 6;

            vkCreateImageView(HW.pDevice, &viewInfo, nullptr, &m_dummyCubeImageView);
        }
    }

    // Dynamic descriptor pools per frame in flight (32k sets to handle all frame draw calls)
    VkDescriptorPoolSize poolSizes[4]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = 16 * 32768;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    poolSizes[1].descriptorCount = 32 * 32768;
    poolSizes[2].type = VK_DESCRIPTOR_TYPE_SAMPLER;
    poolSizes[2].descriptorCount = 16 * 32768;
    poolSizes[3].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[3].descriptorCount = 16 * 32768;

    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.flags = 0;
    poolInfo.maxSets = 32768;
    poolInfo.poolSizeCount = 4;
    poolInfo.pPoolSizes = poolSizes;

    for (u32 i = 0; i < MAX_POOLS; ++i)
    {
        res = vkCreateDescriptorPool(HW.pDevice, &poolInfo, nullptr, &m_descriptorPools[i]);
        if (res != VK_SUCCESS)
        {
            Msg("! [Vulkan] Failed to create descriptor pool [%u]! Result = %d", i, res);
        }
    }

    // Dynamic uniform buffer for constant buffers (64MB per frame)
    VkBufferCreateInfo uboInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    uboInfo.size = UBO_POOL_SIZE;
    uboInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    uboInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo uboAllocCi{};
    uboAllocCi.usage = VMA_MEMORY_USAGE_AUTO;
    uboAllocCi.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

    for (u32 i = 0; i < MAX_POOLS; ++i)
    {
        VmaAllocationInfo vmaInfo{};
        res = vmaCreateBuffer(HW.m_allocator, &uboInfo, &uboAllocCi, &m_uboBuffers[i], &m_uboAllocations[i], &vmaInfo);
        if (res == VK_SUCCESS && vmaInfo.pMappedData)
        {
            m_uboMappedData[i] = static_cast<u8*>(vmaInfo.pMappedData);
            m_uboOffset[i] = 0;
        }
        else
        {
            Msg("! [Vulkan] Failed to create dynamic UBO [%u]! Result = %d", i, res);
        }
    }

    // Pipeline cache - load from disk if available
    {
        string_path cachePath;
        FS.update_path(cachePath, "$app_data_root$", "pipeline_cache_vk.bin");

        VkPipelineCacheCreateInfo cacheCI{ VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO };
        xr_vector<u8> cacheData;

        IReader* cacheFile = FS.exist(cachePath) ? FS.r_open(cachePath) : nullptr;
        if (cacheFile)
        {
            cacheData.resize(cacheFile->length());
            cacheFile->r(cacheData.data(), (u32)cacheData.size());
            cacheFile->close();
            cacheCI.initialDataSize = cacheData.size();
            cacheCI.pInitialData = cacheData.data();
        }

        res = vkCreatePipelineCache(HW.pDevice, &cacheCI, nullptr, &m_pipelineCache);
        if (res != VK_SUCCESS)
        {
            Msg("! [Vulkan] Failed to create pipeline cache! Result = %d", res);
            m_pipelineCache = VK_NULL_HANDLE;
        }
    }

    return true;
}

void vkPipelineManager::Destroy()
{
    if (HW.pDevice != VK_NULL_HANDLE)
    {
        for (auto& [key, pipe] : m_pipelines)
        {
            if (pipe != VK_NULL_HANDLE)
                vkDestroyPipeline(HW.pDevice, pipe, nullptr);
        }
        m_pipelines.clear();

        // Save pipeline cache to disk
        if (m_pipelineCache != VK_NULL_HANDLE)
        {
            size_t cacheSize = 0;
            if (vkGetPipelineCacheData(HW.pDevice, m_pipelineCache, &cacheSize, nullptr) == VK_SUCCESS && cacheSize > 0)
            {
                xr_vector<u8> cacheData(cacheSize);
                if (vkGetPipelineCacheData(HW.pDevice, m_pipelineCache, &cacheSize, cacheData.data()) == VK_SUCCESS)
                {
                    string_path cachePath;
                    FS.update_path(cachePath, "$app_data_root$", "pipeline_cache_vk.bin");
                    IWriter* cacheFile = FS.w_open(cachePath);
                    if (cacheFile)
                    {
                        cacheFile->w(cacheData.data(), (u32)cacheSize);
                        FS.w_close(cacheFile);
                    }
                }
            }
            vkDestroyPipelineCache(HW.pDevice, m_pipelineCache, nullptr);
            m_pipelineCache = VK_NULL_HANDLE;
        }

        for (u32 i = 0; i < MAX_POOLS; ++i)
        {
            if (m_uboBuffers[i] != VK_NULL_HANDLE && HW.m_allocator != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(HW.m_allocator, m_uboBuffers[i], m_uboAllocations[i]);
                m_uboBuffers[i] = VK_NULL_HANDLE;
                m_uboAllocations[i] = VK_NULL_HANDLE;
                m_uboMappedData[i] = nullptr;
                m_uboOffset[i] = 0;
            }

            if (m_descriptorPools[i] != VK_NULL_HANDLE)
            {
                vkDestroyDescriptorPool(HW.pDevice, m_descriptorPools[i], nullptr);
                m_descriptorPools[i] = VK_NULL_HANDLE;
            }
        }

        if (m_defaultSampler != VK_NULL_HANDLE)
        {
            vkDestroySampler(HW.pDevice, m_defaultSampler, nullptr);
            m_defaultSampler = VK_NULL_HANDLE;
        }

        if (m_dummyImageView != VK_NULL_HANDLE)
        {
            vkDestroyImageView(HW.pDevice, m_dummyImageView, nullptr);
            m_dummyImageView = VK_NULL_HANDLE;
        }

        if (m_dummyImage != VK_NULL_HANDLE && HW.m_allocator != VK_NULL_HANDLE)
        {
            vmaDestroyImage(HW.m_allocator, m_dummyImage, m_dummyAllocation);
            m_dummyImage = VK_NULL_HANDLE;
            m_dummyAllocation = VK_NULL_HANDLE;
        }

        if (m_dummyCubeImageView != VK_NULL_HANDLE)
        {
            vkDestroyImageView(HW.pDevice, m_dummyCubeImageView, nullptr);
            m_dummyCubeImageView = VK_NULL_HANDLE;
        }

        if (m_dummyCubeImage != VK_NULL_HANDLE && HW.m_allocator != VK_NULL_HANDLE)
        {
            vmaDestroyImage(HW.m_allocator, m_dummyCubeImage, m_dummyCubeAllocation);
            m_dummyCubeImage = VK_NULL_HANDLE;
            m_dummyCubeAllocation = VK_NULL_HANDLE;
        }

        if (m_pipelineLayout != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(HW.pDevice, m_pipelineLayout, nullptr);
            m_pipelineLayout = VK_NULL_HANDLE;
        }

        if (m_descriptorSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(HW.pDevice, m_descriptorSetLayout, nullptr);
            m_descriptorSetLayout = VK_NULL_HANDLE;
        }
    }
}

VkDescriptorSet vkPipelineManager::AllocateDescriptorSet(u32 frameIndex)
{
    u32 idx = frameIndex % MAX_POOLS;
    if (m_descriptorPools[idx] == VK_NULL_HANDLE || !HW.pDevice || m_descriptorSetLayout == VK_NULL_HANDLE)
        return VK_NULL_HANDLE;

    VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    allocInfo.descriptorPool = m_descriptorPools[idx];
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &m_descriptorSetLayout;

    VkDescriptorSet set = VK_NULL_HANDLE;
    VkResult res = vkAllocateDescriptorSets(HW.pDevice, &allocInfo, &set);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Descriptor pool [%u] exhausted! Result = %d", idx, res);
        return VK_NULL_HANDLE;
    }
    return set;
}

void vkPipelineManager::ResetDescriptorPool(u32 frameIndex)
{
    u32 idx = frameIndex % MAX_POOLS;
    if (m_descriptorPools[idx] != VK_NULL_HANDLE && HW.pDevice != VK_NULL_HANDLE)
    {
        vkResetDescriptorPool(HW.pDevice, m_descriptorPools[idx], 0);
    }
    m_uboOffset[idx] = 0;
    m_constantCache[idx] = {};
}

bool vkPipelineManager::GetOrAllocateConstantBufferSlice(u32 frameIndex, u32 version, const void* data, size_t size, VkDescriptorBufferInfo& outBufferInfo)
{
    u32 idx = frameIndex % MAX_POOLS;
    if (version != 0 && m_constantCache[idx].version == version && m_constantCache[idx].bufferInfo.buffer != VK_NULL_HANDLE)
    {
        outBufferInfo = m_constantCache[idx].bufferInfo;
        return true;
    }

    if (AllocateConstantBufferSlice(frameIndex, data, size, outBufferInfo))
    {
        m_constantCache[idx].version = version;
        m_constantCache[idx].bufferInfo = outBufferInfo;
        return true;
    }
    return false;
}

bool vkPipelineManager::AllocateConstantBufferRaw(u32 frameIndex, size_t size, VkBuffer& outBuffer, VkDeviceSize& outOffset, u8*& outMappedPtr)
{
    u32 idx = frameIndex % MAX_POOLS;
    if (!m_uboMappedData[idx] || size == 0)
        return false;

    // Align to 256 bytes for Vulkan uniform buffer offset requirement
    size_t alignedOffset = (m_uboOffset[idx] + 255) & ~size_t(255);
    if (alignedOffset + size > UBO_POOL_SIZE)
    {
        Msg("! [Vulkan] UBO ring buffer exhausted for frame [%u]! Offset=%zu, requested=%zu", idx, alignedOffset, size);
        return false;
    }

    m_uboOffset[idx] = alignedOffset + size;
    outBuffer = m_uboBuffers[idx];
    outOffset = alignedOffset;
    outMappedPtr = m_uboMappedData[idx] + alignedOffset;
    return true;
}

void vkPipelineManager::FlushConstantBufferRange(u32 frameIndex, VkDeviceSize offset, size_t size)
{
    u32 idx = frameIndex % MAX_POOLS;
    if (HW.m_allocator && m_uboAllocations[idx] && size > 0)
    {
        vmaFlushAllocation(HW.m_allocator, m_uboAllocations[idx], offset, size);
    }
}

bool vkPipelineManager::AllocateConstantBufferSlice(u32 frameIndex, const void* data, size_t size, VkDescriptorBufferInfo& outBufferInfo)
{
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceSize offset = 0;
    u8* mapped = nullptr;
    if (!AllocateConstantBufferRaw(frameIndex, size, buf, offset, mapped))
        return false;

    CopyMemory(mapped, data, size);
    FlushConstantBufferRange(frameIndex, offset, size);

    outBufferInfo.buffer = buf;
    outBufferInfo.offset = offset;
    outBufferInfo.range = size;
    return true;
}

VkPipeline vkPipelineManager::GetOrCreatePipeline(
    VkShaderModule vs,
    VkShaderModule ps,
    VkShaderModule gs,
    SDeclaration* decl,
    u32 vbStride,
    D3DPRIMITIVETYPE topology,
    const vkState* state,
    u32 colorAttachmentCount,
    const VkFormat* colorFormats,
    VkFormat depthFormat)
{
    if (!Init() || vs == VK_NULL_HANDLE || ps == VK_NULL_HANDLE)
        return VK_NULL_HANDLE;

    PipelineKey key{};
    memset(&key, 0, sizeof(key));
    key.vs = vs;
    key.ps = ps;
    key.gs = gs;
    key.decl = decl;
    key.vbStride = vbStride;
    key.topology = topology;
    if (state)
    {
        key.blendEnable = state->m_pBlendState.BlendEnable;
        key.srcBlend = state->m_pBlendState.SrcBlend;
        key.destBlend = state->m_pBlendState.DestBlend;
        key.colorMask = state->m_pBlendState.ColorMask;
    }
    key.colorAttachmentCount = std::min(colorAttachmentCount, 4u);
    for (u32 i = 0; i < key.colorAttachmentCount; ++i)
        key.colorFormats[i] = colorFormats ? colorFormats[i] : VK_FORMAT_UNDEFINED;
    key.depthFormat = depthFormat;

    auto it = m_pipelines.find(key);
    if (it != m_pipelines.end())
        return it->second;

    xr_vector<VkPipelineShaderStageCreateInfo> stages;
    VkPipelineShaderStageCreateInfo vsStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    vsStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vsStage.module = vs;
    vsStage.pName = "main";
    stages.push_back(vsStage);

    VkPipelineShaderStageCreateInfo psStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    psStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    psStage.module = ps;
    psStage.pName = "main";
    stages.push_back(psStage);

    if (gs != VK_NULL_HANDLE)
    {
        VkPipelineShaderStageCreateInfo gsStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        gsStage.stage = VK_SHADER_STAGE_GEOMETRY_BIT;
        gsStage.module = gs;
        gsStage.pName = "main";
        stages.push_back(gsStage);
    }

    // Vertex input
    xr_vector<VkVertexInputBindingDescription> bindings;
    xr_vector<VkVertexInputAttributeDescription> attributes;

    if (decl && !decl->dcl_code.empty())
    {
        auto inputIt = s_vsInputs.find(vs);
        const bool hasReflection = (inputIt != s_vsInputs.end());

        u32 fallbackLoc = 0;
        for (const auto& elem : decl->dcl_code)
        {
            if (elem.Type == D3DDECLTYPE_UNUSED)
                break;

            u32 targetLoc = (u32)-1;
            if (hasReflection)
            {
                const char* semName = GetSemanticName(elem.Usage);
                for (const auto& inParam : inputIt->second)
                {
                    if (inParam.index == elem.UsageIndex)
                    {
                        if (inParam.semantic.compare(semName) == 0)
                        {
                            targetLoc = inParam.location;
                            break;
                        }
                        // POSITION and POSITIONT are interchangeable in DX9 -> DX12/Vulkan vertex shaders
                        if ((elem.Usage == D3DDECLUSAGE_POSITION || elem.Usage == D3DDECLUSAGE_POSITIONT) &&
                            (inParam.semantic.compare("POSITION") == 0 || inParam.semantic.compare("POSITIONT") == 0))
                        {
                            targetLoc = inParam.location;
                            break;
                        }
                    }
                }
            }
            else
            {
                if (elem.Usage == D3DDECLUSAGE_POSITION || elem.Usage == D3DDECLUSAGE_POSITIONT)
                    targetLoc = 0;
                else if (elem.Usage == D3DDECLUSAGE_TEXCOORD)
                    targetLoc = 1 + elem.UsageIndex;
                else if (elem.Usage == D3DDECLUSAGE_COLOR)
                    targetLoc = (vbStride == 28) ? 2 : (fallbackLoc++);
                else
                    targetLoc = fallbackLoc++;
            }

            if (targetLoc != (u32)-1)
            {
                VkVertexInputAttributeDescription attr{};
                attr.location = targetLoc;
                attr.binding = elem.Stream;
                attr.offset = elem.Offset;
                attr.format = TranslateVertexFormat((D3DDECLTYPE)elem.Type);
                attributes.push_back(attr);
            }
        }

        VkVertexInputBindingDescription bindDesc{};
        bindDesc.binding = 0;
        bindDesc.stride = vbStride ? vbStride : 32;
        bindDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        bindings.push_back(bindDesc);
    }
    else if (vbStride == 28) // FVF::TL fallback
    {
        VkVertexInputBindingDescription bindDesc{};
        bindDesc.binding = 0;
        bindDesc.stride = 28;
        bindDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        bindings.push_back(bindDesc);

        // Position: location 0, offset 0
        attributes.push_back({ 0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0 });
        // Tex0: location 1, offset 20
        attributes.push_back({ 1, 0, VK_FORMAT_R32G32_SFLOAT, 20 });
        // Color: location 2, offset 16
        attributes.push_back({ 2, 0, VK_FORMAT_R8G8B8A8_UNORM, 16 });
    }
    else if (vbStride == 36) // FVF::TL2uv fallback
    {
        VkVertexInputBindingDescription bindDesc{};
        bindDesc.binding = 0;
        bindDesc.stride = 36;
        bindDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        bindings.push_back(bindDesc);

        // Position: location 0, offset 0
        attributes.push_back({ 0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0 });
        // Tex0: location 1, offset 20
        attributes.push_back({ 1, 0, VK_FORMAT_R32G32_SFLOAT, 20 });
        // Tex1: location 2, offset 28
        attributes.push_back({ 2, 0, VK_FORMAT_R32G32_SFLOAT, 28 });
        // Color: location 3, offset 16
        attributes.push_back({ 3, 0, VK_FORMAT_R8G8B8A8_UNORM, 16 });
    }
    else if (vbStride == 40) // v_skybox fallback (xyz:12, diffuse:4, tc0:12, tc1:12)
    {
        VkVertexInputBindingDescription bindDesc{};
        bindDesc.binding = 0;
        bindDesc.stride = 40;
        bindDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        bindings.push_back(bindDesc);

        // Position: location 0, offset 0 (float3)
        attributes.push_back({ 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 });
        // Color: location 1, offset 12 (D3DCOLOR -> rgba8)
        attributes.push_back({ 1, 0, VK_FORMAT_R8G8B8A8_UNORM, 12 });
        // Tex0: location 2, offset 16 (float3)
        attributes.push_back({ 2, 0, VK_FORMAT_R32G32B32_SFLOAT, 16 });
        // Tex1: location 3, offset 28 (float3)
        attributes.push_back({ 3, 0, VK_FORMAT_R32G32B32_SFLOAT, 28 });
    }

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<u32>(bindings.size());
    vertexInputInfo.pVertexBindingDescriptions = bindings.data();
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<u32>(attributes.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    inputAssembly.topology = TranslateTopologyVK(topology);
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    VkPipelineViewportStateCreateInfo viewportState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rasterizer.depthClampEnable = VK_TRUE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = state ? TranslateCullMode(state->rasterizerCullMode) : VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;

    VkPipelineMultisampleStateCreateInfo multisampling{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
    if (state && depthFormat != VK_FORMAT_UNDEFINED)
    {
        depthStencil.depthTestEnable = state->m_pDepthStencilState.DepthEnable ? VK_TRUE : VK_FALSE;
        depthStencil.depthWriteEnable = state->m_pDepthStencilState.DepthWriteMask ? VK_TRUE : VK_FALSE;
        depthStencil.depthCompareOp = TranslateCompareOp(state->m_pDepthStencilState.DepthFunc);
        depthStencil.stencilTestEnable = state->m_pDepthStencilState.StencilEnable ? VK_TRUE : VK_FALSE;
        if (state->m_pDepthStencilState.StencilEnable)
        {
            VkStencilOpState stencilOp{};
            stencilOp.failOp = VK_STENCIL_OP_KEEP;
            stencilOp.passOp = VK_STENCIL_OP_REPLACE;
            stencilOp.depthFailOp = VK_STENCIL_OP_KEEP;
            stencilOp.compareOp = TranslateCompareOp(state->m_pDepthStencilState.StencilFunc);
            stencilOp.compareMask = state->m_pDepthStencilState.StencilMask;
            stencilOp.writeMask = state->m_pDepthStencilState.StencilWriteMask;
            stencilOp.reference = 0; // set dynamically via vkCmdSetStencilReference
            depthStencil.front = stencilOp;
            depthStencil.back = stencilOp;
        }
    }

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    u32 mask = (state != nullptr) ? state->m_pBlendState.ColorMask : 0xF;
    VkColorComponentFlags writeMask = 0;
    if (mask & D3DCOLORWRITEENABLE_RED)   writeMask |= VK_COLOR_COMPONENT_R_BIT;
    if (mask & D3DCOLORWRITEENABLE_GREEN) writeMask |= VK_COLOR_COMPONENT_G_BIT;
    if (mask & D3DCOLORWRITEENABLE_BLUE)  writeMask |= VK_COLOR_COMPONENT_B_BIT;
    if (mask & D3DCOLORWRITEENABLE_ALPHA) writeMask |= VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.colorWriteMask = writeMask;

    if (state && state->m_pBlendState.BlendEnable)
    {
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.srcColorBlendFactor = TranslateBlendFactor(state->m_pBlendState.SrcBlend);
        colorBlendAttachment.dstColorBlendFactor = TranslateBlendFactor(state->m_pBlendState.DestBlend);
        colorBlendAttachment.colorBlendOp = vkStateUtils::ConvertBlendOp(state->m_pBlendState.BlendOp);
        colorBlendAttachment.srcAlphaBlendFactor = TranslateBlendFactor(state->m_pBlendState.SrcBlendAlpha);
        colorBlendAttachment.dstAlphaBlendFactor = TranslateBlendFactor(state->m_pBlendState.DestBlendAlpha);
        colorBlendAttachment.alphaBlendOp = vkStateUtils::ConvertBlendOp(state->m_pBlendState.BlendOpAlpha);
    }
    else
    {
        colorBlendAttachment.blendEnable = VK_FALSE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
    }

    VkPipelineColorBlendAttachmentState blendAttachments[4]{};
    for (u32 i = 0; i < key.colorAttachmentCount; ++i)
        blendAttachments[i] = colorBlendAttachment;

    VkPipelineColorBlendStateCreateInfo colorBlending{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.attachmentCount = key.colorAttachmentCount;
    colorBlending.pAttachments = key.colorAttachmentCount > 0 ? blendAttachments : nullptr;

    VkDynamicState dynamicStates[] = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR,
        VK_DYNAMIC_STATE_CULL_MODE,
        VK_DYNAMIC_STATE_FRONT_FACE,
        VK_DYNAMIC_STATE_DEPTH_TEST_ENABLE,
        VK_DYNAMIC_STATE_DEPTH_WRITE_ENABLE,
        VK_DYNAMIC_STATE_DEPTH_COMPARE_OP,
        VK_DYNAMIC_STATE_STENCIL_TEST_ENABLE,
    };
    VkPipelineDynamicStateCreateInfo dynamicState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dynamicState.dynamicStateCount = _countof(dynamicStates);
    dynamicState.pDynamicStates = dynamicStates;

    // Dynamic rendering create info
    VkPipelineRenderingCreateInfo renderingCreateInfo{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
    renderingCreateInfo.colorAttachmentCount = key.colorAttachmentCount;
    renderingCreateInfo.pColorAttachmentFormats = (key.colorAttachmentCount > 0) ? key.colorFormats : nullptr;
    if (depthFormat != VK_FORMAT_UNDEFINED)
    {
        renderingCreateInfo.depthAttachmentFormat = depthFormat;
        renderingCreateInfo.stencilAttachmentFormat = depthFormat;
    }

    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.pNext = &renderingCreateInfo;
    pipelineInfo.stageCount = static_cast<u32>(stages.size());
    pipelineInfo.pStages = stages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = m_pipelineLayout;

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkResult res = vkCreateGraphicsPipelines(HW.pDevice, m_pipelineCache, 1, &pipelineInfo, nullptr, &pipeline);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create graphics pipeline! Result = %d", res);
        return VK_NULL_HANDLE;
    }

    m_pipelines[key] = pipeline;
    return pipeline;
}

void vkPipelineManager::BeginDynamicRendering(
    VkCommandBuffer cmd,
    u32 width, u32 height,
    VkImageView rt0, VkImageView rt1, VkImageView rt2, VkImageView rt3,
    VkImageView zb,
    bool clearColor,
    bool clearDepth,
    const VkClearColorValue* clearColorValue,
    float clearDepthValue,
    const VkViewport* customViewport)
{
    if (m_insideDynamicRendering)
        EndDynamicRendering(cmd);

    VkRenderingAttachmentInfo colorAttachments[4]{};
    u32 colorCount = 0;
    VkImageView rts[4] = { rt0, rt1, rt2, rt3 };

    static const VkClearColorValue defaultClearColor = { {0.0f, 0.0f, 0.0f, 0.0f} };
    const VkClearColorValue& clearCV = clearColorValue ? *clearColorValue : defaultClearColor;

    for (u32 i = 0; i < 4; ++i)
    {
        if (rts[i] != VK_NULL_HANDLE)
        {
            colorAttachments[colorCount].sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
            colorAttachments[colorCount].imageView = rts[i];
            colorAttachments[colorCount].imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            colorAttachments[colorCount].loadOp = clearColor ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
            colorAttachments[colorCount].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            if (clearColor)
                colorAttachments[colorCount].clearValue.color = clearCV;
            colorCount++;
        }
    }

    VkRenderingAttachmentInfo depthAttachment{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    if (zb != VK_NULL_HANDLE)
    {
        depthAttachment.imageView = zb;
        depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachment.loadOp = clearDepth ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        if (clearDepth)
            depthAttachment.clearValue.depthStencil = { clearDepthValue, 0 };
    }

    VkRenderingInfo renderingInfo{ VK_STRUCTURE_TYPE_RENDERING_INFO };
    renderingInfo.renderArea.offset = { 0, 0 };
    renderingInfo.renderArea.extent = { width, height };
    renderingInfo.layerCount = 1;
    renderingInfo.colorAttachmentCount = colorCount;
    renderingInfo.pColorAttachments = colorCount > 0 ? colorAttachments : nullptr;
    renderingInfo.pDepthAttachment = zb != VK_NULL_HANDLE ? &depthAttachment : nullptr;
    renderingInfo.pStencilAttachment = zb != VK_NULL_HANDLE ? &depthAttachment : nullptr;

    vkCmdBeginRendering(cmd, &renderingInfo);
    m_insideDynamicRendering = true;

    VkViewport viewport{};
    if (customViewport)
    {
        viewport = *customViewport;
    }
    else
    {
        viewport.x = 0.0f;
        viewport.y = static_cast<float>(height);        // bottom of screen (flipped Y)
        viewport.width = static_cast<float>(width);
        viewport.height = -static_cast<float>(height);  // invert Y axis
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
    }
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = { 0, 0 };
    scissor.extent = { width, height };
    vkCmdSetScissor(cmd, 0, 1, &scissor);
}

void vkPipelineManager::EndDynamicRendering(VkCommandBuffer cmd)
{
    if (m_insideDynamicRendering)
    {
        vkCmdEndRendering(cmd);
        m_insideDynamicRendering = false;
    }
}
} // namespace xray::render::RENDER_NAMESPACE
