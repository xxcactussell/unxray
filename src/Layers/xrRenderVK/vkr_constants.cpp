#include "stdafx.h"
#pragma hdrstop

#include "Layers/xrRender/ResourceManager.h"
#include "xrCore/xrPool.h"
#include "Layers/xrRender/r_constants.h"
#include <d3d12shader.h>

namespace xray::render::RENDER_NAMESPACE
{
BOOL R_constant_table::parseConstants(ID3D12ShaderReflectionConstantBuffer* pTable, u32 destination)
{
    VERIFY(pTable);
    D3D12_SHADER_BUFFER_DESC TableDesc;
    if (FAILED(pTable->GetDesc(&TableDesc)))
        return FALSE;

    for (u32 i = 0; i < TableDesc.Variables; ++i)
    {
        ID3D12ShaderReflectionVariable* pVar = pTable->GetVariableByIndex(i);
        VERIFY(pVar);
        D3D12_SHADER_VARIABLE_DESC VarDesc;
        pVar->GetDesc(&VarDesc);
        ID3D12ShaderReflectionType* pType = pVar->GetType();
        VERIFY(pType);
        D3D12_SHADER_TYPE_DESC TypeDesc;
        pType->GetDesc(&TypeDesc);

        LPCSTR name = VarDesc.Name;

        u16 type = u16(-1);
        switch (TypeDesc.Type)
        {
        case D3D_SVT_FLOAT: type = RC_float; break;
        case D3D_SVT_BOOL: type = RC_bool; break;
        case D3D_SVT_INT: type = RC_int; break;
        default: continue;
        }

        u32 cbSlot = 0;
        u32 storageSlot = 0;
        if (destination & RC_dest_vertex)
        {
            cbSlot = (destination & RC_dest_vertex_cb_index_mask) >> RC_dest_vertex_cb_index_shift;
            if (cbSlot >= 6) cbSlot = 0;
            storageSlot = cbSlot; // 0..5
        }
        else if (destination & RC_dest_pixel)
        {
            cbSlot = (destination & RC_dest_pixel_cb_index_mask) >> RC_dest_pixel_cb_index_shift;
            if (cbSlot >= 6) cbSlot = 0;
            storageSlot = 6 + cbSlot; // 6..11
        }
        else if (destination & RC_dest_geometry)
        {
            cbSlot = (destination & RC_dest_geometry_cb_index_mask) >> RC_dest_geometry_cb_index_shift;
            if (cbSlot >= 4) cbSlot = 0;
            storageSlot = 12 + cbSlot; // 12..15
        }

        if (storageSlot >= 16)
            storageSlot = 0;

        u16 r_index = u16(storageSlot * 4096 + VarDesc.StartOffset);
        u16 r_type = u16(-1);

        switch (TypeDesc.Class)
        {
        case D3D_SVC_SCALAR: r_type = RC_1x1; break;
        case D3D_SVC_VECTOR:
        {
            switch (TypeDesc.Columns)
            {
            case 4: r_type = RC_1x4; break;
            case 3: r_type = RC_1x3; break;
            case 2: r_type = RC_1x2; break;
            default: continue;
            }
        }
        break;
        case D3D_SVC_MATRIX_ROWS:
        {
            switch (TypeDesc.Columns)
            {
            case 4:
                switch (TypeDesc.Rows)
                {
                case 2: r_type = RC_2x4; break;
                case 3: r_type = RC_3x4; break;
                case 4: r_type = RC_4x4; break;
                default: continue;
                }
                break;
            default: continue;
            }
        }
        break;
        case D3D_SVC_MATRIX_COLUMNS:
        {
            switch (TypeDesc.Rows)
            {
            case 4:
                switch (TypeDesc.Columns)
                {
                case 2: r_type = RC_2x4; break;
                case 3: r_type = RC_3x4; break;
                case 4: r_type = RC_4x4; break;
                default: continue;
                }
                break;
            default: continue;
            }
        }
        break;
        default: continue;
        }

        ref_constant C = get(name);
        if (!C)
        {
            C = table.emplace_back(xr_new<R_constant>());
            C->name = name;
            C->destination = destination;
            C->type = type;
            R_constant_load& L = C->get_load(destination);
            L.index = r_index;
            L.cls = r_type;
        }
        else
        {
            C->destination |= destination;
            VERIFY(C->type == type);
            R_constant_load& L = C->get_load(destination);
            L.index = r_index;
            L.cls = r_type;
        }
    }
    return TRUE;
}

BOOL R_constant_table::parseResources(ID3D12ShaderReflection* pReflection, int ResNum, u32 destination)
{
    for (int i = 0; i < ResNum; ++i)
    {
        D3D12_SHADER_INPUT_BIND_DESC ResDesc;
        pReflection->GetResourceBindingDesc(i, &ResDesc);

        u16 type = 0;

        switch (ResDesc.Type)
        {
        case D3D_SIT_TEXTURE: type = RC_dx11texture; break;
        case D3D_SIT_SAMPLER: type = RC_sampler; break;
        case D3D_SIT_UAV_RWTYPED: type = RC_dx11UAV; break;
        default: continue;
        }

        VERIFY(ResDesc.BindCount == 1);

        u16 r_index = u16(-1);

        if (destination & RC_dest_pixel)
        {
            r_index = u16(ResDesc.BindPoint + CTexture::rstPixel);
        }
        else if (destination & RC_dest_vertex)
        {
            r_index = u16(ResDesc.BindPoint + CTexture::rstVertex);
        }
        else if (destination & RC_dest_geometry)
        {
            r_index = u16(ResDesc.BindPoint + CTexture::rstGeometry);
        }
        else
        {
            r_index = u16(ResDesc.BindPoint);
        }

        ref_constant C = get(ResDesc.Name, dx9compatibility ? type : u16(-1));
        if (!C)
        {
            C = table.emplace_back(xr_new<R_constant>());
            C->name = ResDesc.Name;
            C->destination = RC_dest_sampler;
            C->type = type;
            R_constant_load& L = C->samp;
            L.index = r_index;
            L.cls = type;
        }
        else
        {
            R_ASSERT(C->destination == RC_dest_sampler);
            R_ASSERT(C->type == type);
            R_constant_load& L = C->samp;
            R_ASSERT(L.index == r_index);
            R_ASSERT(L.cls == type);
        }
    }
    return TRUE;
}

BOOL R_constant_table::parse(void* _desc, u32 destination)
{
    if (!_desc)
        return FALSE;

    ID3D12ShaderReflection* pReflection = (ID3D12ShaderReflection*)_desc;

    D3D12_SHADER_DESC ShaderDesc;
    if (FAILED(pReflection->GetDesc(&ShaderDesc)))
        return FALSE;

    if (ShaderDesc.ConstantBuffers)
    {
        for (u16 iBuf = 0; iBuf < ShaderDesc.ConstantBuffers; ++iBuf)
        {
            ID3D12ShaderReflectionConstantBuffer* pTable = pReflection->GetConstantBufferByIndex(iBuf);
            if (pTable)
            {
                D3D12_SHADER_BUFFER_DESC TableDesc{};
                pTable->GetDesc(&TableDesc);

                u32 cbSlot = iBuf;
                D3D12_SHADER_INPUT_BIND_DESC bindDesc{};
                if (SUCCEEDED(pReflection->GetResourceBindingDescByName(TableDesc.Name, &bindDesc)))
                {
                    cbSlot = bindDesc.BindPoint;
                }
                if (cbSlot >= 16)
                    cbSlot = 0;

                u32 updatedDest = destination;
                if (destination & RC_dest_pixel)
                    updatedDest |= (cbSlot << RC_dest_pixel_cb_index_shift);
                else if (destination & RC_dest_vertex)
                    updatedDest |= (cbSlot << RC_dest_vertex_cb_index_shift);
                else if (destination & RC_dest_geometry)
                    updatedDest |= (cbSlot << RC_dest_geometry_cb_index_shift);

                parseConstants(pTable, updatedDest);
            }
        }
    }

    if (ShaderDesc.BoundResources)
    {
        parseResources(pReflection, ShaderDesc.BoundResources, destination);
    }

    std::sort(table.begin(), table.end(), [](const ref_constant& C1, const ref_constant& C2)
    {
        return xr_strcmp(C1->name, C2->name) < 0;
    });

    return TRUE;
}
} // namespace xray::render::RENDER_NAMESPACE
