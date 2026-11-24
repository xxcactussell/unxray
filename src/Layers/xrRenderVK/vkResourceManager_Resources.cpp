#include "stdafx.h"
#pragma hdrstop

#include "Layers/xrRender/ResourceManager.h"
#include "Layers/xrRender/tss.h"
#include "Layers/xrRender/blender.h"
#include "Layers/xrRender/blender_recorder.h"
#include "Layers/xrRender/BufferUtils.h"
#include "Layers/xrRender/ShaderResourceTraits.h"
#include "vkHW.h"
#include "vkPipelineManager.h"

namespace xray::render::RENDER_NAMESPACE
{
SPass* CResourceManager::_CreatePass(const SPass& proto)
{
    for (SPass* pass : v_passes)
        if (pass->equal(proto))
            return pass;

    SPass* P = v_passes.emplace_back(xr_new<SPass>());
    P->dwFlags |= xr_resource_flagged::RF_REGISTERED;
    P->state = proto.state;
    P->ps = proto.ps;
    P->vs = proto.vs;
    P->gs = proto.gs;
    P->constants = proto.constants;
    P->T = proto.T;
#ifdef _EDITOR
    P->M = proto.M;
#endif
    P->C = proto.C;

    return P;
}

SVS* CResourceManager::_CreateVS(cpcstr shader, u32 flags)
{
    string_path name;
    xr_strcpy(name, shader);
    switch (RImplementation.m_skinning)
    {
    case 0: xr_strcat(name, "_0"); break;
    case 1: xr_strcat(name, "_1"); break;
    case 2: xr_strcat(name, "_2"); break;
    case 3: xr_strcat(name, "_3"); break;
    case 4: xr_strcat(name, "_4"); break;
    }

    return CreateShader<SVS>(name, shader, flags);
}

void CResourceManager::_DeleteVS(const SVS* vs)
{
    if (vs)
        UnregisterVSInputs(vs->sh);
    DestroyShader(vs);
}

SPS* CResourceManager::_CreatePS(LPCSTR _name)
{
    string_path name;
    xr_strcpy(name, _name);
    switch (RImplementation.m_MSAASample)
    {
    case 0: xr_strcat(name, "_0"); break;
    case 1: xr_strcat(name, "_1"); break;
    case 2: xr_strcat(name, "_2"); break;
    case 3: xr_strcat(name, "_3"); break;
    case 4: xr_strcat(name, "_4"); break;
    case 5: xr_strcat(name, "_5"); break;
    case 6: xr_strcat(name, "_6"); break;
    case 7: xr_strcat(name, "_7"); break;
    }

    return CreateShader<SPS>(name, _name);
}

void CResourceManager::_DeletePS(const SPS* ps)
{
    DestroyShader(ps);
}

SGS* CResourceManager::_CreateGS(LPCSTR Name)
{
    return CreateShader<SGS>(Name);
}

void CResourceManager::_DeleteGS(const SGS* GS)
{
    DestroyShader(GS);
}

SHS* CResourceManager::_CreateHS(LPCSTR Name)
{
    return CreateShader<SHS>(Name);
}

void CResourceManager::_DeleteHS(const SHS* HS)
{
    DestroyShader(HS);
}

SDS* CResourceManager::_CreateDS(LPCSTR Name)
{
    return CreateShader<SDS>(Name);
}

void CResourceManager::_DeleteDS(const SDS* DS)
{
    DestroyShader(DS);
}

SCS* CResourceManager::_CreateCS(LPCSTR Name)
{
    return CreateShader<SCS>(Name);
}

void CResourceManager::_DeleteCS(const SCS* CS)
{
    DestroyShader(CS);
}

SDeclaration* CResourceManager::_CreateDecl(const D3DVERTEXELEMENT9* dcl)
{
    for (SDeclaration* D : v_declarations)
    {
        if (dcl_equal(dcl, &D->dcl_code.front()))
            return D;
    }

    SDeclaration* D = v_declarations.emplace_back(xr_new<SDeclaration>());
    u32 dcl_size = GetDeclLength(dcl) + 1;
    D->dcl_code.assign(dcl, dcl + dcl_size);
    D->dcl = static_cast<u32>(v_declarations.size());
    D->dwFlags |= xr_resource_flagged::RF_REGISTERED;

    return D;
}
} // namespace xray::render::RENDER_NAMESPACE
