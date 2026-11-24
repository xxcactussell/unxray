#include "stdafx.h"
#include "vkRingBuffer.h"
#include "vkHW.h"

namespace xray::render::RENDER_NAMESPACE {

FrameRingBuffers G_FrameRingBuffers;

// ---------------------------------------------------------------------------
// vkRingBuffer
// ---------------------------------------------------------------------------

bool vkRingBuffer::Init(size_t size, VkBufferUsageFlags usage)
{
    if (!HW.m_allocator || size == 0)
        return false;

    m_size = size;

    VkBufferCreateInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.size        = size;
    bufferInfo.usage       = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                    | VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VmaAllocationInfo allocResult{};
    VkResult res = vmaCreateBuffer(HW.m_allocator, &bufferInfo, &allocInfo,
                                   &m_buffer, &m_allocation, &allocResult);
    if (res != VK_SUCCESS)
    {
        Msg("! [RingBuffer] vmaCreateBuffer failed: size=%zu usage=0x%X res=%d",
            size, usage, (int)res);
        m_buffer     = VK_NULL_HANDLE;
        m_allocation = VK_NULL_HANDLE;
        return false;
    }

    m_mappedData = static_cast<u8*>(allocResult.pMappedData);
    if (!m_mappedData)
    {
        Msg("! [RingBuffer] Buffer was not mapped after creation");
        vmaDestroyBuffer(HW.m_allocator, m_buffer, m_allocation);
        m_buffer     = VK_NULL_HANDLE;
        m_allocation = VK_NULL_HANDLE;
        return false;
    }

    // Check HOST_COHERENT to skip explicit flushes when possible
    VkMemoryPropertyFlags memProps = 0;
    vmaGetAllocationMemoryProperties(HW.m_allocator, m_allocation, &memProps);
    m_hostCoherent = !!(memProps & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    m_offset          = 0;
    m_lastLockOffset  = 0;
    m_lastLockSize    = 0;

    return true;
}

void vkRingBuffer::Destroy()
{
    if (m_buffer != VK_NULL_HANDLE && HW.m_allocator)
    {
        vmaDestroyBuffer(HW.m_allocator, m_buffer, m_allocation);
        m_buffer      = VK_NULL_HANDLE;
        m_allocation  = VK_NULL_HANDLE;
        m_mappedData  = nullptr;
        m_size        = 0;
        m_offset      = 0;
    }
}

void* vkRingBuffer::Lock(size_t sizeBytes, size_t& outByteOffset, size_t alignment)
{
    if (!m_mappedData || sizeBytes == 0)
        return nullptr;

    // Align current offset
    const size_t alignedOffset = (m_offset + (alignment - 1)) & ~(alignment - 1);

    if (alignedOffset + sizeBytes > m_size)
    {
        Msg("! [RingBuffer] Overflow: requested %zu, available %zu (size=%zu offset=%zu)",
            sizeBytes, m_size - alignedOffset, m_size, alignedOffset);
        return nullptr;
    }

    outByteOffset    = alignedOffset;
    m_lastLockOffset = alignedOffset;
    m_lastLockSize   = sizeBytes;
    m_offset         = alignedOffset + sizeBytes;

    return m_mappedData + alignedOffset;
}

void vkRingBuffer::Unlock()
{
    if (m_hostCoherent || !m_allocation || m_lastLockSize == 0)
        return;

    vmaFlushAllocation(HW.m_allocator, m_allocation,
                       m_lastLockOffset, m_lastLockSize);
}

void vkRingBuffer::Reset()
{
    m_offset         = 0;
    m_lastLockOffset = 0;
    m_lastLockSize   = 0;
}

// ---------------------------------------------------------------------------
// FrameRingBuffers
// ---------------------------------------------------------------------------

bool FrameRingBuffers::Init()
{
    for (u32 i = 0; i < CHW::MAX_FRAMES_IN_FLIGHT; ++i)
    {
        if (!vertex[i].Init(vkRingBuffer::DEFAULT_VERTEX_SIZE,
                            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT))
        {
            Msg("! [RingBuffer] Failed to init vertex ring buffer [%u]", i);
            return false;
        }
        if (!index[i].Init(vkRingBuffer::DEFAULT_INDEX_SIZE,
                           VK_BUFFER_USAGE_INDEX_BUFFER_BIT))
        {
            Msg("! [RingBuffer] Failed to init index ring buffer [%u]", i);
            return false;
        }
    }
    Msg("* [RingBuffer] FrameRingBuffers initialized (%u frames)",
        CHW::MAX_FRAMES_IN_FLIGHT);
    return true;
}

void FrameRingBuffers::Destroy()
{
    for (u32 i = 0; i < CHW::MAX_FRAMES_IN_FLIGHT; ++i)
    {
        vertex[i].Destroy();
        index[i].Destroy();
    }
}

void FrameRingBuffers::Reset(u32 frame)
{
    const u32 idx = frame % CHW::MAX_FRAMES_IN_FLIGHT;
    vertex[idx].Reset();
    index[idx].Reset();
}

} // namespace xray::render::RENDER_NAMESPACE
