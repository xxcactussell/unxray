#include "stdafx.h"
#include "Layers/xrRender/BufferUtils.h"
#include "vkHW.h"
#include <FlexibleVertexFormat.h>

namespace xray::render::RENDER_NAMESPACE
{
u32 GetFVFVertexSize(u32 FVF)
{
    return static_cast<u32>(::FVF::ComputeVertexSize(FVF));
}

u32 GetDeclVertexSize(const VertexElement* decl, u32 Stream)
{
    return static_cast<u32>(::FVF::ComputeVertexSize(decl, Stream));
}

u32 GetDeclLength(const VertexElement* decl)
{
    return static_cast<u32>(::FVF::GetDeclLength(decl));
}

static SBufferResource* CreateBuffer(const void* pData, u32 dataSize, VkBufferUsageFlags usage, bool bDynamic)
{
    if (dataSize == 0 || !HW.m_allocator)
        return nullptr;

    auto* res = xr_new<SBufferResource>();
    res->size = dataSize;

    VkBufferCreateInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.size = dataSize;
    bufferInfo.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocCreateInfo{};
    allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_ALLOW_TRANSFER_INSTEAD_BIT |
                            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                            VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VkResult vkRes = vmaCreateBuffer(HW.m_allocator, &bufferInfo, &allocCreateInfo,
        &res->buffer, &res->allocation, &res->allocInfo);

    if (vkRes != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to allocate buffer of size %u! Result = %d", dataSize, vkRes);
        xr_delete(res);
        return nullptr;
    }

    if (res->allocInfo.pMappedData)
    {
        res->pMappedData = res->allocInfo.pMappedData;
    }
    else
    {
        void* mapped = nullptr;
        if (vmaMapMemory(HW.m_allocator, res->allocation, &mapped) == VK_SUCCESS)
        {
            res->pMappedData = mapped;
        }
    }

    if (res->pMappedData && pData)
    {
        CopyMemory(res->pMappedData, pData, dataSize);
        vmaFlushAllocation(HW.m_allocator, res->allocation, 0, dataSize);
    }

    return res;
}

static void DestroyBufferResource(SBufferResource*& res)
{
    if (res)
    {
        if (HW.m_allocator && res->buffer && res->allocation)
            vmaDestroyBuffer(HW.m_allocator, res->buffer, res->allocation);
        xr_delete(res);
    }
}

namespace BufferUtils
{
HRESULT CreateConstantBuffer(ConstantBufferHandle* ppBuffer, u32 DataSize)
{
    *ppBuffer = CreateBuffer(nullptr, DataSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, true);
    return *ppBuffer ? S_OK : E_FAIL;
}
}

void ConvertVertexDeclaration(const VertexElement* dxdecl, SDeclaration* decl)
{
    decl->dcl = 1;
}

void ConvertVertexDeclaration(const xr_vector<VertexElement>& declIn, xr_vector<InputElementDesc>& declOut)
{
}

//-----------------------------------------------------------------------------
// VertexStagingBuffer
//-----------------------------------------------------------------------------
VertexStagingBuffer::~VertexStagingBuffer()
{
    Destroy();
}

void VertexStagingBuffer::Create(size_t size, bool allowReadBack)
{
    m_Size = size;
    m_AllowReadBack = allowReadBack;
    m_HostBuffer = xr_alloc<u8>(size);
    AddRef();
}

bool VertexStagingBuffer::IsValid() const
{
    return !!m_DeviceBuffer;
}

void* VertexStagingBuffer::Map(size_t offset, size_t size, bool read)
{
    VERIFY2(m_HostBuffer, "Buffer wasn't created or already discarded");
    return static_cast<u8*>(m_HostBuffer) + offset;
}

void VertexStagingBuffer::Unmap(bool doFlush)
{
    if (!doFlush) return;

    VERIFY(m_HostBuffer && m_Size);
    m_DeviceBuffer = CreateBuffer(m_HostBuffer, static_cast<u32>(m_Size), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, false);

    if (!m_AllowReadBack)
        DiscardHostBuffer();
}

VertexBufferHandle VertexStagingBuffer::GetBufferHandle() const
{
    return m_DeviceBuffer;
}

void VertexStagingBuffer::Destroy()
{
    DiscardHostBuffer();
    m_Size = 0;
    if (m_DeviceBuffer)
    {
        DestroyBufferResource(m_DeviceBuffer);
        m_DeviceBuffer = nullptr;
    }
}

void VertexStagingBuffer::DiscardHostBuffer()
{
    if (m_HostBuffer)
    {
        xr_free(m_HostBuffer);
        m_HostBuffer = nullptr;
    }
}

size_t VertexStagingBuffer::GetSystemMemoryUsage() const
{
    return m_HostBuffer ? m_Size : 0;
}

size_t VertexStagingBuffer::GetVideoMemoryUsage() const
{
    return m_DeviceBuffer ? m_DeviceBuffer->size : 0;
}

//-----------------------------------------------------------------------------
// IndexStagingBuffer
//-----------------------------------------------------------------------------
IndexStagingBuffer::~IndexStagingBuffer()
{
    Destroy();
}

void IndexStagingBuffer::Create(size_t size, bool allowReadBack, bool managed)
{
    m_Size = size;
    m_AllowReadBack = allowReadBack;
    m_HostBuffer = xr_alloc<u8>(size);
    AddRef();
}

bool IndexStagingBuffer::IsValid() const
{
    return !!m_DeviceBuffer;
}

void* IndexStagingBuffer::Map(size_t offset, size_t size, bool read)
{
    VERIFY2(m_HostBuffer, "Buffer wasn't created or already discarded");
    return static_cast<u8*>(m_HostBuffer) + offset;
}

void IndexStagingBuffer::Unmap(bool doFlush)
{
    if (!doFlush) return;

    VERIFY(m_HostBuffer && m_Size);
    m_DeviceBuffer = CreateBuffer(m_HostBuffer, static_cast<u32>(m_Size), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, false);

    if (!m_AllowReadBack)
        DiscardHostBuffer();
}

IndexBufferHandle IndexStagingBuffer::GetBufferHandle() const
{
    return m_DeviceBuffer;
}

void IndexStagingBuffer::Destroy()
{
    DiscardHostBuffer();
    m_Size = 0;
    if (m_DeviceBuffer)
    {
        DestroyBufferResource(m_DeviceBuffer);
        m_DeviceBuffer = nullptr;
    }
}

void IndexStagingBuffer::DiscardHostBuffer()
{
    if (m_HostBuffer)
    {
        xr_free(m_HostBuffer);
        m_HostBuffer = nullptr;
    }
}

size_t IndexStagingBuffer::GetSystemMemoryUsage() const
{
    return m_HostBuffer ? m_Size : 0;
}

size_t IndexStagingBuffer::GetVideoMemoryUsage() const
{
    return m_DeviceBuffer ? m_DeviceBuffer->size : 0;
}

//-----------------------------------------------------------------------------
// VertexStreamBuffer
//-----------------------------------------------------------------------------
VertexStreamBuffer::~VertexStreamBuffer()
{
    Destroy();
}

void VertexStreamBuffer::Create(size_t size)
{
    m_DeviceBuffer = CreateBuffer(nullptr, static_cast<u32>(size), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, true);
    AddRef();
}

void VertexStreamBuffer::Destroy()
{
    if (m_DeviceBuffer)
    {
        DestroyBufferResource(m_DeviceBuffer);
        m_DeviceBuffer = nullptr;
    }
}

void* VertexStreamBuffer::Map(size_t offset, size_t size, bool flush)
{
    if (m_DeviceBuffer && m_DeviceBuffer->pMappedData)
        return static_cast<u8*>(m_DeviceBuffer->pMappedData) + offset;
    return nullptr;
}

void VertexStreamBuffer::Unmap()
{
    if (m_DeviceBuffer && m_DeviceBuffer->allocation)
        vmaFlushAllocation(HW.m_allocator, m_DeviceBuffer->allocation, 0, VK_WHOLE_SIZE);
}

bool VertexStreamBuffer::IsValid() const
{
    return !!m_DeviceBuffer;
}

//-----------------------------------------------------------------------------
// IndexStreamBuffer
//-----------------------------------------------------------------------------
IndexStreamBuffer::~IndexStreamBuffer()
{
    Destroy();
}

void IndexStreamBuffer::Create(size_t size)
{
    m_DeviceBuffer = CreateBuffer(nullptr, static_cast<u32>(size), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, true);
    AddRef();
}

void IndexStreamBuffer::Destroy()
{
    if (m_DeviceBuffer)
    {
        DestroyBufferResource(m_DeviceBuffer);
        m_DeviceBuffer = nullptr;
    }
}

void* IndexStreamBuffer::Map(size_t offset, size_t size, bool flush)
{
    if (m_DeviceBuffer && m_DeviceBuffer->pMappedData)
        return static_cast<u8*>(m_DeviceBuffer->pMappedData) + offset;
    return nullptr;
}

void IndexStreamBuffer::Unmap()
{
    if (m_DeviceBuffer && m_DeviceBuffer->allocation)
        vmaFlushAllocation(HW.m_allocator, m_DeviceBuffer->allocation, 0, VK_WHOLE_SIZE);
}

bool IndexStreamBuffer::IsValid() const
{
    return !!m_DeviceBuffer;
}
} // namespace xray::render::RENDER_NAMESPACE
