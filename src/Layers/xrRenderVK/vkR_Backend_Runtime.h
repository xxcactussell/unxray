#pragma once

#include "vkStateUtils.h"
#include "vkHW.h"
#include "vkPipelineManager.h"
#include "vkTextureUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
inline VkViewport s_currentVkViewport{};
inline bool s_hasCustomVkViewport = false;
IC void CBackend::set_xform(u32 ID, const Fmatrix& M)
{
    stat.xforms++;
}

IC void CBackend::set_RT(VkImageView RT, u32 ID)
{
    if (RT != pRT[ID])
    {
        stat.target_rt++;
        pRT[ID] = RT;
        if (HW.m_frameStarted && G_PipelineManager.IsInsideDynamicRendering())
        {
            G_PipelineManager.EndDynamicRendering(HW.GetCurrentCommandBuffer());
        }
    }
}

IC void CBackend::set_ZB(VkImageView ZB)
{
    if (ZB != pZB)
    {
        stat.target_zb++;
        pZB = ZB;
        if (HW.m_frameStarted && G_PipelineManager.IsInsideDynamicRendering())
        {
            G_PipelineManager.EndDynamicRendering(HW.GetCurrentCommandBuffer());
        }
    }
}

IC void CBackend::ClearRT(VkImageView rt, const Fcolor& color)
{
    if (!HW.m_frameStarted) return;
    VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

    if (G_PipelineManager.IsInsideDynamicRendering())
    {
        VkClearAttachment clearAtt{};
        clearAtt.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        clearAtt.colorAttachment = 0;
        clearAtt.clearValue.color.float32[0] = color.r;
        clearAtt.clearValue.color.float32[1] = color.g;
        clearAtt.clearValue.color.float32[2] = color.b;
        clearAtt.clearValue.color.float32[3] = color.a;
        VkClearRect clearRect{};
        clearRect.rect.offset = { 0, 0 };
        clearRect.rect.extent = { curr_rt_width ? curr_rt_width : 1, curr_rt_height ? curr_rt_height : 1 };
        clearRect.baseArrayLayer = 0;
        clearRect.layerCount = 1;
        vkCmdClearAttachments(cmd, 1, &clearAtt, 1, &clearRect);
    }
    else
    {
        VkImage img = vkTextureUtils::GetViewImage(rt);
        if (img != VK_NULL_HANDLE)
        {
            vkTextureUtils::TransitionViewLayout(cmd, rt, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            VkClearColorValue clearVal = { { color.r, color.g, color.b, color.a } };
            VkImageSubresourceRange range{ VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };
            vkCmdClearColorImage(cmd, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearVal, 1, &range);
            vkTextureUtils::TransitionViewLayout(cmd, rt, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        }
    }
}

IC void CBackend::ClearZB(VkImageView zb, float depth)
{
    ClearZB(zb, depth, 0);
}

IC void CBackend::ClearZB(VkImageView zb, float depth, u8 stencil)
{
    if (!HW.m_frameStarted) return;
    VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

    if (G_PipelineManager.IsInsideDynamicRendering())
    {
        VkClearAttachment clearAtt{};
        clearAtt.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
        clearAtt.clearValue.depthStencil.depth = depth;
        clearAtt.clearValue.depthStencil.stencil = stencil;
        VkClearRect clearRect{};
        clearRect.rect.offset = { 0, 0 };
        clearRect.rect.extent = { curr_rt_width ? curr_rt_width : 1, curr_rt_height ? curr_rt_height : 1 };
        clearRect.baseArrayLayer = 0;
        clearRect.layerCount = 1;
        vkCmdClearAttachments(cmd, 1, &clearAtt, 1, &clearRect);
    }
    else
    {
        VkImage img = vkTextureUtils::GetViewImage(zb);
        if (img != VK_NULL_HANDLE)
        {
            vkTextureUtils::TransitionViewLayout(cmd, zb, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            VkClearDepthStencilValue clearVal = { depth, stencil };
            VkImageSubresourceRange range{ VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };
            vkCmdClearDepthStencilImage(cmd, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearVal, 1, &range);
            vkTextureUtils::TransitionViewLayout(cmd, zb, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
        }
    }
}

IC bool CBackend::ClearRTRect(VkImageView rt, const Fcolor& color, size_t numRects, const Irect* rects)
{
    if (!HW.m_frameStarted || numRects == 0 || !rects) return true;
    VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

    if (G_PipelineManager.IsInsideDynamicRendering())
    {
        VkClearAttachment clearAtt{};
        clearAtt.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        clearAtt.colorAttachment = 0;
        clearAtt.clearValue.color.float32[0] = color.r;
        clearAtt.clearValue.color.float32[1] = color.g;
        clearAtt.clearValue.color.float32[2] = color.b;
        clearAtt.clearValue.color.float32[3] = color.a;

        xr_vector<VkClearRect> vkRects(numRects);
        for (size_t i = 0; i < numRects; ++i)
        {
            vkRects[i].rect.offset = { std::max(0, rects[i].left), std::max(0, rects[i].top) };
            vkRects[i].rect.extent = { static_cast<u32>(std::max(0, rects[i].width())), static_cast<u32>(std::max(0, rects[i].height())) };
            vkRects[i].baseArrayLayer = 0;
            vkRects[i].layerCount = 1;
        }
        vkCmdClearAttachments(cmd, 1, &clearAtt, static_cast<u32>(numRects), vkRects.data());
    }
    return true;
}

IC bool CBackend::ClearZBRect(VkImageView zb, float depth, size_t numRects, const Irect* rects)
{
    if (!HW.m_frameStarted || numRects == 0 || !rects) return true;
    VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

    if (G_PipelineManager.IsInsideDynamicRendering())
    {
        VkClearAttachment clearAtt{};
        clearAtt.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
        clearAtt.clearValue.depthStencil.depth = depth;
        clearAtt.clearValue.depthStencil.stencil = 0;

        xr_vector<VkClearRect> vkRects(numRects);
        for (size_t i = 0; i < numRects; ++i)
        {
            vkRects[i].rect.offset = { std::max(0, rects[i].left), std::max(0, rects[i].top) };
            vkRects[i].rect.extent = { static_cast<u32>(std::max(0, rects[i].width())), static_cast<u32>(std::max(0, rects[i].height())) };
            vkRects[i].baseArrayLayer = 0;
            vkRects[i].layerCount = 1;
        }
        vkCmdClearAttachments(cmd, 1, &clearAtt, static_cast<u32>(numRects), vkRects.data());
    }
    return true;
}

ICF void CBackend::set_Format(SDeclaration* _decl)
{
    decl = _decl;
}

ICF void CBackend::set_PS(VkShaderModule _ps, LPCSTR _n)
{
    if (ps != _ps)
    {
        stat.ps++;
        ps = _ps;
#ifdef DEBUG
        ps_name = _n;
#endif
    }
}

ICF void CBackend::set_GS(VkShaderModule _gs, LPCSTR _n)
{
    if (gs != _gs)
    {
        stat.gs++;
        gs = _gs;
#ifdef DEBUG
        gs_name = _n;
#endif
    }
}

ICF void CBackend::set_VS(VkShaderModule _vs, LPCSTR _n)
{
    if (vs != _vs)
    {
        stat.vs++;
        vs = _vs;
#ifdef DEBUG
        vs_name = _n;
#endif
    }
}

ICF void CBackend::set_Vertices(VertexBufferHandle _vb, u32 _vb_stride)
{
    if (vb != _vb || vb_stride != _vb_stride)
    {
        vb = _vb;
        vb_stride = _vb_stride;
        if (HW.m_frameStarted && vb && vb->buffer != VK_NULL_HANDLE)
        {
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(HW.GetCurrentCommandBuffer(), 0, 1, &vb->buffer, &offset);
        }
    }
}

ICF void CBackend::set_Indices(IndexBufferHandle _ib)
{
    if (ib != _ib)
    {
        ib = _ib;
        if (HW.m_frameStarted && ib && ib->buffer != VK_NULL_HANDLE)
        {
            vkCmdBindIndexBuffer(HW.GetCurrentCommandBuffer(), ib->buffer, 0, VK_INDEX_TYPE_UINT16);
        }
    }
}

IC VkPrimitiveTopology TranslateTopology(D3DPRIMITIVETYPE T)
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

IC u32 GetIndexCount(D3DPRIMITIVETYPE T, u32 iPrimitiveCount)
{
    switch (T)
    {
    case D3DPT_POINTLIST: return iPrimitiveCount;
    case D3DPT_LINELIST: return iPrimitiveCount * 2;
    case D3DPT_LINESTRIP: return iPrimitiveCount + 1;
    case D3DPT_TRIANGLELIST: return iPrimitiveCount * 3;
    case D3DPT_TRIANGLESTRIP: return iPrimitiveCount + 2;
    default: return iPrimitiveCount * 3;
    }
}

IC bool CBackend::ApplyPipelineAndDynamicRendering(D3DPRIMITIVETYPE T)
{
    if (!HW.m_frameStarted)
        return false;

    VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();

    // Check if any bound texture needs layout transition to SHADER_READ_ONLY_OPTIMAL
    bool needTextureTransition = false;
    for (u32 i = 0; i < CTexture::mtMaxPixelShaderTextures; ++i)
    {
        CTexture* tex = textures_ps[i];
        VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
        if (view != VK_NULL_HANDLE && vkTextureUtils::GetViewLayout(view) != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        {
            needTextureTransition = true;
            break;
        }
    }
    if (!needTextureTransition)
    {
        for (u32 i = 0; i < CTexture::mtMaxVertexShaderTextures; ++i)
        {
            CTexture* tex = textures_vs[i];
            VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
            if (view != VK_NULL_HANDLE && vkTextureUtils::GetViewLayout(view) != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
            {
                needTextureTransition = true;
                break;
            }
        }
    }

    if (needTextureTransition && G_PipelineManager.IsInsideDynamicRendering())
    {
        G_PipelineManager.EndDynamicRendering(cmd);
    }

    if (needTextureTransition)
    {
        for (u32 i = 0; i < CTexture::mtMaxPixelShaderTextures; ++i)
        {
            CTexture* tex = textures_ps[i];
            VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
            if (view != VK_NULL_HANDLE && vkTextureUtils::GetViewLayout(view) != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
                vkTextureUtils::TransitionViewLayout(cmd, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        for (u32 i = 0; i < CTexture::mtMaxVertexShaderTextures; ++i)
        {
            CTexture* tex = textures_vs[i];
            VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
            if (view != VK_NULL_HANDLE && vkTextureUtils::GetViewLayout(view) != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
                vkTextureUtils::TransitionViewLayout(cmd, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    }

    // Ensure dynamic rendering is active
    if (!G_PipelineManager.IsInsideDynamicRendering())
    {
        VkImageView targetRT0 = pRT[0];
        const bool hasColorRT = (pRT[0] != VK_NULL_HANDLE || pRT[1] != VK_NULL_HANDLE || pRT[2] != VK_NULL_HANDLE || pRT[3] != VK_NULL_HANDLE);
        if (!hasColorRT && pZB == VK_NULL_HANDLE)
            targetRT0 = HW.GetCurrentBackBufferView();

        u32 width = curr_rt_width;
        u32 height = curr_rt_height;
        if (width == 0 || height == 0)
        {
            auto [sw, sh] = HW.GetSurfaceSize();
            width = sw;
            height = sh;
        }

        if (width > 0 && height > 0 && (targetRT0 != VK_NULL_HANDLE || pZB != VK_NULL_HANDLE))
        {
            if (targetRT0 != VK_NULL_HANDLE)
                vkTextureUtils::TransitionViewLayout(cmd, targetRT0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            for (u32 i = 1; i < 4; ++i)
                if (pRT[i] != VK_NULL_HANDLE)
                    vkTextureUtils::TransitionViewLayout(cmd, pRT[i], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            if (pZB != VK_NULL_HANDLE)
                vkTextureUtils::TransitionViewLayout(cmd, pZB, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

            G_PipelineManager.BeginDynamicRendering(cmd, width, height,
                targetRT0, pRT[1], pRT[2], pRT[3], pZB,
                false, false, nullptr, 1.0f,
                s_hasCustomVkViewport ? &s_currentVkViewport : nullptr);
        }
        else
        {
            return false;
        }
    }

    if (!vs || !ps)
        return false;

    VkFormat colorFormats[4] = { VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED };
    u32 colorAttachmentCount = 0;
    for (u32 i = 0; i < 4; ++i)
    {
        if (pRT[i] != VK_NULL_HANDLE)
        {
            colorFormats[colorAttachmentCount++] = vkTextureUtils::GetViewFormat(pRT[i], VK_FORMAT_R8G8B8A8_UNORM);
        }
    }
    if (colorAttachmentCount == 0 && pZB == VK_NULL_HANDLE && pRT[0] == VK_NULL_HANDLE)
    {
        colorFormats[0] = HW.m_swapchainFormat;
        colorAttachmentCount = 1;
    }

    VkFormat depthFormat = (pZB != VK_NULL_HANDLE) ? vkTextureUtils::GetViewFormat(pZB, VK_FORMAT_D24_UNORM_S8_UINT) : VK_FORMAT_UNDEFINED;

    VkPipeline pipeline = G_PipelineManager.GetOrCreatePipeline(
        vs, ps, gs, decl, vb_stride, T, state, colorAttachmentCount, colorFormats, depthFormat);

    if (pipeline == VK_NULL_HANDLE)
        return false;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    const bool hasDepth = (pZB != VK_NULL_HANDLE);
    vkCmdSetCullMode(cmd, vkStateUtils::ConvertCullMode(cull_mode));
    vkCmdSetFrontFace(cmd, VK_FRONT_FACE_CLOCKWISE);
    if (hasDepth)
    {
        vkCmdSetDepthTestEnable(cmd, z_enable ? VK_TRUE : VK_FALSE);
        vkCmdSetDepthWriteEnable(cmd, (state && !state->m_pDepthStencilState.DepthWriteMask) ? VK_FALSE : (z_enable ? VK_TRUE : VK_FALSE));
        vkCmdSetDepthCompareOp(cmd, vkStateUtils::ConvertCmpFunction(z_func));
        vkCmdSetStencilTestEnable(cmd, stencil_enable ? VK_TRUE : VK_FALSE);
        if (stencil_enable)
        {
            vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, stencil_ref);
            vkCmdSetStencilCompareMask(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, stencil_mask);
            vkCmdSetStencilWriteMask(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, stencil_writemask);
            vkCmdSetStencilOp(cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
                vkStateUtils::ConvertStencilOp(stencil_fail),
                vkStateUtils::ConvertStencilOp(stencil_pass),
                vkStateUtils::ConvertStencilOp(stencil_zfail),
                vkStateUtils::ConvertCmpFunction(stencil_func));
        }
    }
    else
    {
        vkCmdSetDepthTestEnable(cmd, VK_FALSE);
        vkCmdSetDepthWriteEnable(cmd, VK_FALSE);
        vkCmdSetDepthCompareOp(cmd, VK_COMPARE_OP_ALWAYS);
        vkCmdSetStencilTestEnable(cmd, VK_FALSE);
    }

    // Push constant buffer data (up to 128 bytes)
    if (constants.GetData())
    {
        vkCmdPushConstants(cmd, G_PipelineManager.GetPipelineLayout(),
            VK_SHADER_STAGE_ALL_GRAPHICS, 0, 128, constants.GetData());
    }

    // Dynamic descriptor set for textures and samplers
    VkDescriptorSet descSet = G_PipelineManager.AllocateDescriptorSet(HW.m_currentFrame);
    if (descSet != VK_NULL_HANDLE)
    {
        VkWriteDescriptorSet writes[128];
        VkDescriptorImageInfo imageInfos[64];
        VkDescriptorBufferInfo slotInfos[64];
        u32 writeCount = 0;
        u32 uboWriteCount = 0;

        // Textures
        if (ctable && !ctable->m_tex_records.empty())
        {
            for (const auto& rec : ctable->m_tex_records)
            {
                if (writeCount >= 128) break;
                CTexture* tex = nullptr;
                if (rec.slot < CTexture::mtMaxPixelShaderTextures)
                {
                    tex = textures_ps[rec.slot];
                }
                else if (rec.slot >= CTexture::rstVertex && rec.slot < CTexture::rstVertex + CTexture::mtMaxVertexShaderTextures)
                {
                    tex = textures_vs[rec.slot - CTexture::rstVertex];
                }
                else if (rec.slot >= 16 && rec.slot < 16 + CTexture::mtMaxVertexShaderTextures)
                {
                    tex = textures_vs[rec.slot - 16];
                }

                VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
                if (view == VK_NULL_HANDLE)
                {
                    const bool isCube = (rec.name == "s_sky0" || rec.name == "s_sky1" || strstr(rec.name.c_str(), "sky") != nullptr);
                    view = isCube ? G_PipelineManager.GetDummyCubeImageView() : G_PipelineManager.GetDummyImageView();
                }

                if (view != VK_NULL_HANDLE)
                {
                    imageInfos[writeCount].sampler = VK_NULL_HANDLE;
                    imageInfos[writeCount].imageView = view;
                    imageInfos[writeCount].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

                    writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                    writes[writeCount].dstSet = descSet;
                    writes[writeCount].dstBinding = rec.binding;
                    writes[writeCount].dstArrayElement = 0;
                    writes[writeCount].descriptorCount = 1;
                    writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
                    writes[writeCount].pImageInfo = &imageInfos[writeCount];
                    writeCount++;
                }
            }
        }
        else
        {
            // Pixel shader textures (fallback: bindings 16..31)
            for (u32 i = 0; i < CTexture::mtMaxPixelShaderTextures; ++i)
            {
                CTexture* tex = textures_ps[i];
                VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
                if (view != VK_NULL_HANDLE)
                {
                    imageInfos[writeCount].sampler = VK_NULL_HANDLE;
                    imageInfos[writeCount].imageView = view;
                    imageInfos[writeCount].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

                    writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                    writes[writeCount].dstSet = descSet;
                    writes[writeCount].dstBinding = 16 + i;
                    writes[writeCount].dstArrayElement = 0;
                    writes[writeCount].descriptorCount = 1;
                    writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
                    writes[writeCount].pImageInfo = &imageInfos[writeCount];
                    writeCount++;
                }
            }

            // Vertex shader textures (fallback: bindings 32..35)
            for (u32 i = 0; i < CTexture::mtMaxVertexShaderTextures; ++i)
            {
                CTexture* tex = textures_vs[i];
                VkImageView view = tex ? tex->surface_get() : VK_NULL_HANDLE;
                if (view != VK_NULL_HANDLE)
                {
                    imageInfos[writeCount].sampler = VK_NULL_HANDLE;
                    imageInfos[writeCount].imageView = view;
                    imageInfos[writeCount].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

                    writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                    writes[writeCount].dstSet = descSet;
                    writes[writeCount].dstBinding = 16 + CTexture::mtMaxPixelShaderTextures + i;
                    writes[writeCount].dstArrayElement = 0;
                    writes[writeCount].descriptorCount = 1;
                    writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
                    writes[writeCount].pImageInfo = &imageInfos[writeCount];
                    writeCount++;
                }
            }
        }

        // Samplers
        if (ctable && !ctable->m_smp_records.empty())
        {
            for (const auto& rec : ctable->m_smp_records)
            {
                if (writeCount >= 128) break;
                VkSampler smp = VK_NULL_HANDLE;
                if (state)
                {
                    if (rec.slot < 64 && state->m_samplerArray[rec.slot] != VK_NULL_HANDLE)
                        smp = state->m_samplerArray[rec.slot];
                    else if (rec.slot < 16 && state->m_samplerArray[32 + rec.slot] != VK_NULL_HANDLE)
                        smp = state->m_samplerArray[32 + rec.slot];
                }
                if (smp == VK_NULL_HANDLE)
                    smp = G_PipelineManager.GetDefaultSampler();

                if (smp != VK_NULL_HANDLE)
                {
                    imageInfos[writeCount].sampler = smp;
                    imageInfos[writeCount].imageView = VK_NULL_HANDLE;
                    imageInfos[writeCount].imageLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                    writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                    writes[writeCount].dstSet = descSet;
                    writes[writeCount].dstBinding = rec.binding;
                    writes[writeCount].dstArrayElement = 0;
                    writes[writeCount].descriptorCount = 1;
                    writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
                    writes[writeCount].pImageInfo = &imageInfos[writeCount];
                    writeCount++;
                }
            }
        }
        else
        {
            // Samplers (fallback: bindings 48..63)
            for (u32 j = 0; j < 16; ++j)
            {
                VkSampler smp = state ? state->m_samplerArray[32 + j] : VK_NULL_HANDLE;
                if (smp == VK_NULL_HANDLE && state)
                    smp = state->m_samplerArray[j];
                if (smp == VK_NULL_HANDLE)
                    smp = G_PipelineManager.GetDefaultSampler();

                if (smp != VK_NULL_HANDLE)
                {
                    imageInfos[writeCount].sampler = smp;
                    imageInfos[writeCount].imageView = VK_NULL_HANDLE;
                    imageInfos[writeCount].imageLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                    writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                    writes[writeCount].dstSet = descSet;
                    writes[writeCount].dstBinding = 48 + j;
                    writes[writeCount].dstArrayElement = 0;
                    writes[writeCount].descriptorCount = 1;
                    writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
                    writes[writeCount].pImageInfo = &imageInfos[writeCount];
                    writeCount++;
                }
            }
        }

        // Constant buffers (using real SPIR-V bindings from reflection)
        if (constants.GetData() && constants.GetSize() > 0)
        {
            if (ctable && !ctable->m_cb_records.empty())
            {
                u32 slotToLocal[16];
                for (u32 i = 0; i < 16; ++i) slotToLocal[i] = u32(-1);
                u32 uniqueSlots[16]{};
                u32 uniqueCount = 0;

                for (const auto& rec : ctable->m_cb_records)
                {
                    u32 slot = rec.slot < 16 ? rec.slot : 0;
                    if (slotToLocal[slot] == u32(-1))
                    {
                        slotToLocal[slot] = uniqueCount;
                        uniqueSlots[uniqueCount++] = slot;
                    }
                }

                VkBuffer uboBuf = VK_NULL_HANDLE;
                VkDeviceSize uboBaseOffset = 0;
                u8* uboMapped = nullptr;
                const size_t allocSize = uniqueCount * 4096;
                if (uniqueCount > 0 && G_PipelineManager.AllocateConstantBufferRaw(HW.m_currentFrame, allocSize, uboBuf, uboBaseOffset, uboMapped))
                {
                    const u8* srcData = static_cast<const u8*>(constants.GetData());
                    for (u32 i = 0; i < uniqueCount; ++i)
                    {
                        CopyMemory(uboMapped + i * 4096, srcData + uniqueSlots[i] * 4096, 4096);
                    }
                    G_PipelineManager.FlushConstantBufferRange(HW.m_currentFrame, uboBaseOffset, allocSize);

                    for (const auto& rec : ctable->m_cb_records)
                    {
                        if (writeCount >= 128 || uboWriteCount >= 64) break;
                        u32 slot = rec.slot < 16 ? rec.slot : 0;
                        u32 localIdx = slotToLocal[slot];
                        slotInfos[uboWriteCount].buffer = uboBuf;
                        slotInfos[uboWriteCount].offset = uboBaseOffset + localIdx * 4096;
                        slotInfos[uboWriteCount].range = 4096;

                        writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                        writes[writeCount].dstSet = descSet;
                        writes[writeCount].dstBinding = rec.binding;
                        writes[writeCount].dstArrayElement = 0;
                        writes[writeCount].descriptorCount = 1;
                        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                        writes[writeCount].pBufferInfo = &slotInfos[uboWriteCount];
                        writeCount++;
                        uboWriteCount++;
                    }
                }
            }
            else
            {
                VkDescriptorBufferInfo uboBufferInfo{};
                if (G_PipelineManager.GetOrAllocateConstantBufferSlice(HW.m_currentFrame, constants.GetVersion(), constants.GetData(), constants.GetSize(), uboBufferInfo))
                {
                    for (u32 b = 0; b < 16; ++b)
                    {
                        if (writeCount >= 128 || uboWriteCount >= 64) break;
                        slotInfos[uboWriteCount].buffer = uboBufferInfo.buffer;
                        slotInfos[uboWriteCount].offset = uboBufferInfo.offset + b * 4096;
                        slotInfos[uboWriteCount].range = 4096;

                        writes[writeCount] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
                        writes[writeCount].dstSet = descSet;
                        writes[writeCount].dstBinding = 0 + b;
                        writes[writeCount].dstArrayElement = 0;
                        writes[writeCount].descriptorCount = 1;
                        writes[writeCount].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                        writes[writeCount].pBufferInfo = &slotInfos[uboWriteCount];
                        writeCount++;
                        uboWriteCount++;
                    }
                }
            }
        }

        if (writeCount > 0)
        {
            vkUpdateDescriptorSets(HW.pDevice, writeCount, writes, 0, nullptr);
        }

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            G_PipelineManager.GetPipelineLayout(), 0, 1, &descSet, 0, nullptr);
    }

    return true;
}

ICF void CBackend::Render(D3DPRIMITIVETYPE T, u32 baseV, u32 startV, u32 countV, u32 startI, u32 PC)
{
    u32 iIndexCount = GetIndexCount(T, PC);
    stat.render.calls++;
    stat.render.verts += countV;
    stat.render.polys += PC;

    if (HW.m_frameStarted)
    {
        if (ApplyPipelineAndDynamicRendering(T))
        {
            VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();
            if (vb && vb->buffer != VK_NULL_HANDLE)
            {
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &vb->buffer, &offset);
            }
            if (ib && ib->buffer != VK_NULL_HANDLE)
            {
                vkCmdBindIndexBuffer(cmd, ib->buffer, 0, VK_INDEX_TYPE_UINT16);
            }
            vkCmdDrawIndexed(cmd, iIndexCount, 1, startI, baseV, 0);
        }
    }
}

ICF void CBackend::Render(D3DPRIMITIVETYPE T, u32 startV, u32 PC)
{
    u32 iIndexCount = GetIndexCount(T, PC);
    stat.render.calls++;
    stat.render.verts += iIndexCount;
    stat.render.polys += PC;

    if (HW.m_frameStarted)
    {
        if (ApplyPipelineAndDynamicRendering(T))
        {
            VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();
            if (vb && vb->buffer != VK_NULL_HANDLE)
            {
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &vb->buffer, &offset);
            }
            vkCmdDraw(cmd, iIndexCount, 1, startV, 0);
        }
    }
}

IC void CBackend::set_Geometry(SGeometry* _geom)
{
    set_Format(&*_geom->dcl);
    set_Vertices(_geom->vb, _geom->vb_stride);
    set_Indices(_geom->ib);
}

IC void CBackend::set_Scissor(const Irect* R)
{
    if (HW.m_frameStarted)
    {
        VkRect2D scissor{};
        if (R)
        {
            int left = std::max(0, R->left);
            int top = std::max(0, R->top);
            int right = std::max(left, R->right);
            int bottom = std::max(top, R->bottom);
            scissor.offset.x = left;
            scissor.offset.y = top;
            scissor.extent.width = static_cast<u32>(std::max(1, right - left));
            scissor.extent.height = static_cast<u32>(std::max(1, bottom - top));
        }
        else
        {
            u32 w = curr_rt_width;
            u32 h = curr_rt_height;
            if (w == 0 || h == 0)
            {
                auto [sw, sh] = HW.GetSurfaceSize();
                w = sw;
                h = sh;
            }
            scissor.offset = { 0, 0 };
            scissor.extent = { std::max(1u, w), std::max(1u, h) };
        }
        vkCmdSetScissor(HW.GetCurrentCommandBuffer(), 0, 1, &scissor);
    }
}

IC void CBackend::SetViewport(const D3D_VIEWPORT& viewport) const
{
    VkViewport vp{};
    vp.x = viewport.TopLeftX;
    vp.y = viewport.TopLeftY + viewport.Height;   // bottom of screen (flipped Y)
    vp.width = viewport.Width;
    vp.height = -viewport.Height;                  // invert Y axis
    vp.minDepth = viewport.MinDepth;
    vp.maxDepth = viewport.MaxDepth;

    s_currentVkViewport = vp;
    s_hasCustomVkViewport = true;

    if (HW.m_frameStarted)
    {
        vkCmdSetViewport(HW.GetCurrentCommandBuffer(), 0, 1, &vp);

        if (viewport.Width > 0 && viewport.Height > 0)
        {
            VkRect2D sc{};
            sc.offset.x = static_cast<int32_t>(viewport.TopLeftX);
            sc.offset.y = static_cast<int32_t>(viewport.TopLeftY);
            sc.extent.width = static_cast<uint32_t>(viewport.Width);
            sc.extent.height = static_cast<uint32_t>(viewport.Height);
            vkCmdSetScissor(HW.GetCurrentCommandBuffer(), 0, 1, &sc);
        }
    }
}

IC void CBackend::set_Stencil(u32 _enable, u32 _func, u32 _ref, u32 _mask, u32 _writemask, u32 _fail, u32 _pass, u32 _zfail)
{
    stencil_enable = _enable;
    stencil_func = _func;
    stencil_ref = _ref;
    stencil_mask = _mask;
    stencil_writemask = _writemask;
    stencil_fail = _fail;
    stencil_pass = _pass;
    stencil_zfail = _zfail;

    if (HW.m_frameStarted)
    {
        VkCommandBuffer cmd = HW.GetCurrentCommandBuffer();
        vkCmdSetStencilTestEnable(cmd, _enable ? VK_TRUE : VK_FALSE);
        vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, _ref);
        vkCmdSetStencilCompareMask(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, _mask);
        vkCmdSetStencilWriteMask(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, _writemask);
        vkCmdSetStencilOp(cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
            vkStateUtils::ConvertStencilOp(_fail),
            vkStateUtils::ConvertStencilOp(_pass),
            vkStateUtils::ConvertStencilOp(_zfail),
            vkStateUtils::ConvertCmpFunction(_func));
    }
}

IC void CBackend::set_Z(u32 _enable)
{
    z_enable = _enable;
    if (HW.m_frameStarted)
        vkCmdSetDepthTestEnable(HW.GetCurrentCommandBuffer(), _enable ? VK_TRUE : VK_FALSE);
}

IC void CBackend::set_ZFunc(u32 _func)
{
    z_func = _func;
    if (HW.m_frameStarted)
        vkCmdSetDepthCompareOp(HW.GetCurrentCommandBuffer(), vkStateUtils::ConvertCmpFunction(_func));
}

IC void CBackend::set_AlphaRef(u32 _value) {}

IC void CBackend::set_ColorWriteEnable(u32 _mask)
{
    colorwrite_mask = _mask;
}

ICF void CBackend::set_CullMode(u32 _mode)
{
    if (cull_mode != _mode)
    {
        cull_mode = _mode;
        if (HW.m_frameStarted)
        {
            vkCmdSetCullMode(HW.GetCurrentCommandBuffer(), vkStateUtils::ConvertCullMode(_mode));
        }
    }
}

ICF void CBackend::set_FillMode(u32 _mode)
{
    fill_mode = _mode;
}

ICF void CBackend::SetTextureFactor(u32) const {}
ICF void CBackend::SetAmbient(u32) const {}

ICF void CBackend::set_VS(ref_vs& _vs)
{
    set_VS(_vs->sh, _vs->cName.c_str());
}

IC void CBackend::set_Constants(R_constant_table* C)
{
    if (ctable != C)
    {
        ctable = C;
        xforms.unmap();
        hemi.unmap();
        tree.unmap();
    }
    if (nullptr == C) return;

    for (auto& Cs : C->table)
        if (Cs->handler) Cs->handler->setup(*this, &*Cs);
}

void CBackend::set_pass_targets(const ref_rt& _1, const ref_rt& _2, const ref_rt& _3, const ref_rt& zb)
{
    if (_1)
    {
        curr_rt_width = _1->dwWidth;
        curr_rt_height = _1->dwHeight;
    }
    else if (zb)
    {
        curr_rt_width = zb->dwWidth;
        curr_rt_height = zb->dwHeight;
    }

    set_RT(_1 ? _1->pRT : VK_NULL_HANDLE, 0);
    set_RT(_2 ? _2->pRT : VK_NULL_HANDLE, 1);
    set_RT(_3 ? _3->pRT : VK_NULL_HANDLE, 2);
    set_ZB(zb ? zb->pZRT : VK_NULL_HANDLE);

    const D3D_VIEWPORT viewport = { 0, 0, curr_rt_width, curr_rt_height, 0.f, 1.f };
    SetViewport(viewport);
}
} // namespace xray::render::RENDER_NAMESPACE
