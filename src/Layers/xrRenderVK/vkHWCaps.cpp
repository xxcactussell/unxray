#include "stdafx.h"
#pragma hdrstop

#include "Layers/xrRender/HWCaps.h"
#include "vkHW.h"

namespace xray::render::RENDER_NAMESPACE
{
void CHWCaps::Update()
{
    geometry_major = 5;
    geometry_minor = 0;
    geometry_profile = "vs_5_0";
    geometry.bSoftware = FALSE;
    geometry.bPointSprites = FALSE;
    geometry.bNPatches = FALSE;
    geometry.dwRegisters = 256;
    geometry.dwInstructions = 256;
    geometry.dwClipPlanes = 6;
    geometry.bVTF = TRUE;
    geometry.dwVertexCache = 24;

    raster_major = 5;
    raster_minor = 0;
    raster_profile = "ps_5_0";
    raster.dwStages = 16;
    raster.bNonPow2 = TRUE;
    raster.bCubemap = TRUE;
    raster.dwMRT_count = 4;
    raster.b_MRT_mixdepth = TRUE;
    raster.dwInstructions = 256;

    bTableFog = FALSE;
    bStencil = TRUE;
    bScissor = TRUE;

    hasFixedPipeline = false;
    useCombinedSamplers = false;
    iGPUNum = 1;

    soDec = D3DSTENCILOP_DECR;
    soInc = D3DSTENCILOP_INCR;
    dwMaxStencilValue = (1 << 8) - 1;

    fTarget = D3DFMT_A8R8G8B8;
    fDepth = D3DFMT_D24S8;

    id_vendor = HW.Caps.id_vendor;
    id_device = HW.Caps.id_device;
}
} // namespace xray::render::RENDER_NAMESPACE
