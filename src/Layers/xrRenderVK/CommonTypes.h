#pragma once

#if defined(XR_PLATFORM_WINDOWS)
#include <d3d9types.h>
#else
#include "Common/d3d9compat.hpp"
#endif

#ifndef VK_NO_PROTOTYPES
#define VK_NO_PROTOTYPES
#endif
#include <vulkan/vulkan.h>
#include "vk_mem_alloc.h"

namespace xray::render::RENDER_NAMESPACE
{
class vkState;

typedef enum D3D_CLEAR_FLAG {
    D3D_CLEAR_DEPTH = 0x1L,
    D3D_CLEAR_STENCIL = 0x2L
} D3D_CLEAR_FLAG;

typedef enum D3D_COMPARISON_FUNC {
    D3D_COMPARISON_NEVER = VK_COMPARE_OP_NEVER,
    D3D_COMPARISON_LESS = VK_COMPARE_OP_LESS,
    D3D_COMPARISON_EQUAL = VK_COMPARE_OP_EQUAL,
    D3D_COMPARISON_LESS_EQUAL = VK_COMPARE_OP_LESS_OR_EQUAL,
    D3D_COMPARISON_GREATER = VK_COMPARE_OP_GREATER,
    D3D_COMPARISON_NOT_EQUAL = VK_COMPARE_OP_NOT_EQUAL,
    D3D_COMPARISON_GREATER_EQUAL = VK_COMPARE_OP_GREATER_OR_EQUAL,
    D3D_COMPARISON_ALWAYS = VK_COMPARE_OP_ALWAYS
} D3D_COMPARISON_FUNC;

struct XR_VK_VIEWPORT
{
    float TopLeftX, TopLeftY;
    float Width, Height;
    float MinDepth, MaxDepth;
};

struct D3D_VIEWPORT : XR_VK_VIEWPORT
{
    using XR_VK_VIEWPORT::XR_VK_VIEWPORT;

    template <typename TopLeftCoords, typename Dimensions>
    D3D_VIEWPORT(TopLeftCoords x, TopLeftCoords y, Dimensions w, Dimensions h, float minZ, float maxZ)
        : XR_VK_VIEWPORT{
            static_cast<float>(x), static_cast<float>(y),
            static_cast<float>(w), static_cast<float>(h),
            minZ, maxZ,
        }
    {}
};

using D3D_QUERY = enum XR_VK_QUERY
{
    D3D_QUERY_EVENT,
    D3D_QUERY_OCCLUSION
};

using ID3DState = vkState;

#define DX11_ONLY(expr) do {} while (0)

using unused_t = int[0];

struct SBufferResource
{
    VkBuffer buffer{ VK_NULL_HANDLE };
    VmaAllocation allocation{ VK_NULL_HANDLE };
    VmaAllocationInfo allocInfo{};
    size_t size{ 0 };
    void* pMappedData{ nullptr };
};

using IndexBufferHandle = SBufferResource*;
using VertexBufferHandle = SBufferResource*;
using ConstantBufferHandle = SBufferResource*;
using HostBufferHandle = void*;

using VertexElement = D3DVERTEXELEMENT9;
using InputElementDesc = unused_t;
} // namespace xray::render::RENDER_NAMESPACE
