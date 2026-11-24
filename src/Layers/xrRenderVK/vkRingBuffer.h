#pragma once
#include "CommonTypes.h"
#include "vkHW.h"

namespace xray::render::RENDER_NAMESPACE {

// Permanently mapped ring buffer for dynamic CPU-to-GPU data.
// One instance per frame-in-flight (not reused while GPU is reading).
class vkRingBuffer
{
public:
    static constexpr size_t DEFAULT_VERTEX_SIZE = 16 * 1024 * 1024; // 16 MB per frame
    static constexpr size_t DEFAULT_INDEX_SIZE  =  4 * 1024 * 1024; //  4 MB per frame

    bool Init(size_t size, VkBufferUsageFlags usage);
    void Destroy();

    // Allocate a slice from the current frame.
    // Returns write pointer and byte offset into buffer.
    // Aligned to `alignment` bytes. Returns nullptr if overflow.
    void* Lock(size_t sizeBytes, size_t& outByteOffset, size_t alignment = 4);
    void  Unlock(); // flush if not HOST_COHERENT

    // Reset offset to start (call at beginning of frame after GPU fence)
    void Reset();

    VkBuffer GetBuffer() const { return m_buffer; }
    bool     IsValid()   const { return m_buffer != VK_NULL_HANDLE; }

private:
    VkBuffer      m_buffer       { VK_NULL_HANDLE };
    VmaAllocation m_allocation   { VK_NULL_HANDLE };
    u8*           m_mappedData   { nullptr };
    size_t        m_size         { 0 };
    size_t        m_offset       { 0 };
    size_t        m_lastLockOffset{ 0 };
    size_t        m_lastLockSize  { 0 };
    bool          m_hostCoherent  { false };
};

// Two per-frame-in-flight sets for vertices and indices
struct FrameRingBuffers
{
    vkRingBuffer vertex[CHW::MAX_FRAMES_IN_FLIGHT];
    vkRingBuffer index [CHW::MAX_FRAMES_IN_FLIGHT];

    bool Init();
    void Destroy();
    void Reset(u32 frame); // call in BeginScene after fence

    vkRingBuffer& GetVertexBuffer(u32 frame) { return vertex[frame % CHW::MAX_FRAMES_IN_FLIGHT]; }
    vkRingBuffer& GetIndexBuffer (u32 frame) { return index [frame % CHW::MAX_FRAMES_IN_FLIGHT]; }
};

extern FrameRingBuffers G_FrameRingBuffers;

} // namespace xray::render::RENDER_NAMESPACE
