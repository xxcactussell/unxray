#include "stdafx.h"

#include "Layers/xrRender/dxRenderFactory.h"
#include "Layers/xrRender/dxUIRender.h"
#include "Layers/xrRender/dxDebugRender.h"
#include "Layers/xrRender/D3DUtils.h"

#include "Include/xrRender/xrRender.h"

namespace xray::render::RENDER_NAMESPACE
{
constexpr pcstr RENDERER_RVK_MODE = "renderer_vk"; // id 7

class RVKRendererModule final : public RendererModule
{
    xr_vector<std::pair<pcstr, int>> modes;

public:
    const xr_vector<std::pair<pcstr, int>>& ObtainSupportedModes() override
    {
        ZoneScoped;

        if (modes.empty())
        {
            modes.emplace_back(RENDERER_RVK_MODE, 7);
        }
        return modes;
    }

    bool CheckGameRequirements() override
    {
        return true;
    }

    void SetupEnv(pcstr mode) override
    {
        ZoneScoped;

        ps_r2_sun_static = false;
        ps_r2_advanced_pp = true;

        GEnv.Render = &RImplementation;
        GEnv.RenderFactory = &RenderFactoryImpl;
        GEnv.DU = &DUImpl;
        GEnv.UIRender = &UIRenderImpl;
        GEnv.DRender = rdebug_render;
        rdebug_render->Register();
        xrRender_initconsole();
    }

    void ClearEnv() override
    {
        modes.clear();

        if (GEnv.Render == &RImplementation)
        {
            GEnv.Render = nullptr;
            GEnv.RenderFactory = nullptr;
            GEnv.DU = nullptr;
            GEnv.UIRender = nullptr;
            GEnv.DRender = nullptr;
            rdebug_render->Unregister();
        }
    }
} static s_rvk_module;

RendererModule* GetRendererModule()
{
    return &s_rvk_module;
}
} // namespace xray::render::RENDER_NAMESPACE
