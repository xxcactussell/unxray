#include "stdafx.h"

#include "dx11HW.h"

#include "StateManager/dx11SamplerStateCache.h"
#include "dx11TextureUtils.h"

#include <SDL3/SDL_system.h>

namespace xray::render::RENDER_NAMESPACE
{
CHW HW;

CHW::CHW()
{
    if (!ThisInstanceIsGlobal())
        return;

    Device.seqAppActivate.Add(this);
    Device.seqAppDeactivate.Add(this);
}

CHW::~CHW()
{
    if (!ThisInstanceIsGlobal())
        return;

    Device.seqAppActivate.Remove(this);
    Device.seqAppDeactivate.Remove(this);
}

void CHW::OnAppActivate()
{
    if (m_pSwapChain && !m_ChainDesc.Windowed)
    {
        ShowWindow(m_ChainDesc.OutputWindow, SW_RESTORE);
    }
}

void CHW::OnAppDeactivate()
{
    if (m_pSwapChain && !m_ChainDesc.Windowed)
    {
        if (psDeviceMode.WindowStyle == rsFullscreen || psDeviceMode.WindowStyle == rsFullscreenBorderless)
            ShowWindow(m_ChainDesc.OutputWindow, SW_MINIMIZE);
    }
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
void CHW::CreateD3D()
{
    ZoneScoped;

    hDXGI = XRay::LoadModule("dxgi");
    hD3D = XRay::LoadModule("d3d11");
    if (!hD3D->IsLoaded() || !hDXGI->IsLoaded())
    {
        Valid = false;
        return;
    }

    // Минимально поддерживаемая версия Windows => Windows Vista SP2 или Windows 7.
    const auto createDXGIFactory = static_cast<decltype(&CreateDXGIFactory1)>(hDXGI->GetProcAddress("CreateDXGIFactory1"));
    if (createDXGIFactory)
        createDXGIFactory(__uuidof(IDXGIFactory1), (void**)(&m_pFactory));

    if (m_pFactory)
        m_pFactory->EnumAdapters1(0, &m_pAdapter);

    Valid = m_pAdapter;
}

void CHW::DestroyD3D()
{
    _SHOW_REF("refCount:m_pAdapter", m_pAdapter);
    _RELEASE(m_pAdapter);

    _SHOW_REF("refCount:m_pFactory", m_pFactory);
    _RELEASE(m_pFactory);

    // Manually close and unload additional DLLs
    // To make it work with DXVK, etc.
    hD3D->Close();
    hDXGI->Close();
    if (auto hModule = GetModuleHandleA("d3d11.dll"))
        FreeLibrary(hModule);
    if (auto hModule = GetModuleHandleA("dxgi.dll"))
        FreeLibrary(hModule);
}

void CHW::CreateDevice(SDL_Window* sdlWnd)
{
    ZoneScoped;

    CreateD3D();
    if (!Valid)
        return;

    m_DriverType = Caps.bForceGPU_REF ? D3D_DRIVER_TYPE_REFERENCE : D3D_DRIVER_TYPE_HARDWARE;

    // Display the name of video board
    DXGI_ADAPTER_DESC1 Desc{};
    if (FAILED(m_pAdapter->GetDesc1(&Desc)))
        Msg("! [%s] failed to retrieve adapter description", __FUNCTION__);
    //  Warning: Desc.Description is wide string
    Msg("* GPU [vendor:%X]-[device:%X]: %S", Desc.VendorId, Desc.DeviceId, Desc.Description);

    Caps.id_vendor = Desc.VendorId;
    Caps.id_device = Desc.DeviceId;

    u32 createDeviceFlags = 0;

#ifdef DEBUG
    if (xrDebug::DebuggerIsPresent())
        createDeviceFlags |= D3D_CREATE_DEVICE_DEBUG;
#endif

    HRESULT R;

    D3D_FEATURE_LEVEL featureLevels[] =
    {
#ifdef HAS_DX11_3
        D3D_FEATURE_LEVEL_12_1,
        D3D_FEATURE_LEVEL_12_0,
#endif
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0
    };

    D3D_FEATURE_LEVEL featureLevels2[] =
    {
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0
    };

    D3D_FEATURE_LEVEL featureLevels3[] =
    {
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0
    };

    auto& pContext = d3d_contexts_pool[CHW::IMM_CTX_ID];

    const auto createDevice = [&](const D3D_FEATURE_LEVEL* level, const u32 levels)
    {
        ZoneScopedN("CreateDevice");

        static const auto d3d11CreateDevice = static_cast<PFN_D3D11_CREATE_DEVICE>(hD3D->GetProcAddress("D3D11CreateDevice"));
        return d3d11CreateDevice(m_pAdapter, D3D_DRIVER_TYPE_UNKNOWN,
            nullptr, createDeviceFlags, level, levels,
            D3D11_SDK_VERSION, &pDevice, &FeatureLevel, &pContext);
    };

    if (DX10Only)
        R = createDevice(featureLevels3, std::size(featureLevels3));
    else
    {
        R = createDevice(featureLevels, std::size(featureLevels));
        if (FAILED(R))
            R = createDevice(featureLevels2, std::size(featureLevels2));
    }

    if (SUCCEEDED(R))
    {
        pContext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&pContext1));
#ifdef HAS_DX11_3
        pDevice->QueryInterface(__uuidof(ID3D11Device3), reinterpret_cast<void**>(&pDevice3));
#endif
        if (FeatureLevel >= D3D_FEATURE_LEVEL_11_0)
        {
            D3DCompile = &::D3DCompile;
            ComputeShadersSupported = true;
        }
        else
        {
            if (ClearSkyMode)
            {
                hD3DCompiler = XRay::LoadModule("d3dcompiler_37");
                D3DCompile = static_cast<D3DCompileFunc>(hD3DCompiler->GetProcAddress("D3DCompileFromMemory"));
            }
            else
            {
                D3DCompile = &::D3DCompile;
            }

            D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS data;
            pDevice->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS,
                &data, sizeof(data));
            ComputeShadersSupported = data.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x;
        }
        D3D11_FEATURE_DATA_D3D11_OPTIONS options;
        pDevice->CheckFeatureSupport(D3D11_FEATURE_D3D11_OPTIONS, &options, sizeof(options));

        D3D11_FEATURE_DATA_DOUBLES doubles;
        pDevice->CheckFeatureSupport(D3D11_FEATURE_DOUBLES, &doubles, sizeof(doubles));

        DoublePrecisionFloatShaderOps = doubles.DoublePrecisionFloatShaderOps;
        SAD4ShaderInstructions = options.SAD4ShaderInstructions;
        ExtendedDoublesShaderInstructions = options.ExtendedDoublesShaderInstructions;
    }

    if (FAILED(R))
    {
        Valid = false;
        Msg("Failed to initialize graphics hardware.\n"
            "CreateDevice returned 0x%08x", R);
        return;
    }

    _SHOW_REF("* CREATE: DeviceREF:", pDevice);

    // Register immediate context in profiler
    if (ThisInstanceIsGlobal())
    {
        TaskScheduler->AddTask([this]
        {
            ZoneScopedN("TracyD3D11Context");
            profiler_ctx = TracyD3D11Context(pDevice, get_context(CHW::IMM_CTX_ID));
        });
    }

    // Create deferred contexts
    if (ThisInstanceIsGlobal())
    {
        ZoneScopedN("Create deferred contexts");
        for (int id = 0; id < R__NUM_PARALLEL_CONTEXTS; ++id)
        {
            R = pDevice->CreateDeferredContext(0, &d3d_contexts_pool[id]);
            VERIFY(SUCCEEDED(R));
        }
    }

    const HWND hwnd = (HWND)SDL_GetPointerProperty(SDL_GetWindowProperties(sdlWnd), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr);
    if (!hwnd)
    {
        Msg("! Failed to retrieve SDL window handle: %s", SDL_GetError());
        Valid = false;
        return;
    }

    if (!CreateSwapChain2(hwnd))
    {
        if (!CreateSwapChain(hwnd))
        {
            Log("! CreateSwapChain failed");
            Valid = false;
        }
    }

    // Select depth-stencil format
    constexpr DXGI_FORMAT formats[] =
    {
        //DXGI_FORMAT_D32_FLOAT_S8X24_UINT,
        DXGI_FORMAT_D24_UNORM_S8_UINT,
    };
    const DXGI_FORMAT selectedFormat = SelectFormat(D3D_FORMAT_SUPPORT_DEPTH_STENCIL, formats);
    if (selectedFormat == DXGI_FORMAT_UNKNOWN)
    {
        Log("! Failed to select depth-stencil format");
        Valid = false;
        return;
    }
    Caps.fDepth = dx11TextureUtils::ConvertTextureFormat(selectedFormat);

    const auto memory = Desc.DedicatedVideoMemory;
    Msg("*   Texture memory: %d M", memory / (1024 * 1024));
}

bool CHW::CreateSwapChain(HWND hwnd)
{
    ZoneScoped;

    // Set up the presentation parameters
    DXGI_SWAP_CHAIN_DESC& sd = m_ChainDesc;
    ZeroMemory(&sd, sizeof(sd));

    RECT rect;
    GetClientRect(hwnd, &rect);
    int w = rect.right - rect.left;
    int h = rect.bottom - rect.top;
    if (w <= 0) w = 640;
    if (h <= 0) h = 480;

    // Back buffer
    sd.BufferDesc.Width = Device.dwWidth ? Device.dwWidth : w;
    sd.BufferDesc.Height = Device.dwHeight ? Device.dwHeight : h;

    //  TODO: DX11: implement dynamic format selection
    constexpr DXGI_FORMAT formats[] =
    {
        //DXGI_FORMAT_R16G16B16A16_FLOAT, // Do we even need this?
        //DXGI_FORMAT_R10G10B10A2_UNORM, // D3DX11SaveTextureToMemory fails on this format
        DXGI_FORMAT_R8G8B8A8_UNORM,
    };

    // Select back-buffer format
    sd.BufferDesc.Format = SelectFormat(D3D_FORMAT_SUPPORT_DISPLAY, formats);
    if (sd.BufferDesc.Format == DXGI_FORMAT_UNKNOWN)
    {
        Log("! SelectFormat failed to find a display format, falling back to DXGI_FORMAT_R8G8B8A8_UNORM");
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    }
    Caps.fTarget = dx11TextureUtils::ConvertTextureFormat(sd.BufferDesc.Format);

    // Buffering
    BackBufferCount = 1;
    sd.BufferCount = BackBufferCount;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;

    // Multisample
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;

    // Windoze
    /* XXX:
       Probably the reason of weird tearing
       glitches reported by Shoker in windowed
       mode with VSync enabled.
       XXX: Fix this windoze stuff!!!
    */
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    sd.OutputWindow = hwnd;

    sd.Windowed = true; // Let SDL handle fullscreen

    //  Additional set up
    // Note: DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH is intentionally omitted.
    // That flag lets DXGI intercept Alt+Enter and manage display mode switching,
    // which conflicts with SDL3's fullscreen management on DXVK/Wayland.
    sd.Flags = 0;

    const auto hr = m_pFactory->CreateSwapChain(pDevice, &sd, &m_pSwapChain);
    if (FAILED(hr))
    {
        Msg("! CreateSwapChain HRESULT: 0x%08X", hr);
        Msg("! sd.BufferDesc.Width = %d, Height = %d, Format = %d", sd.BufferDesc.Width, sd.BufferDesc.Height, sd.BufferDesc.Format);
        Msg("! sd.SampleDesc.Count = %d, Quality = %d", sd.SampleDesc.Count, sd.SampleDesc.Quality);
        Msg("! sd.BufferUsage = %d, BufferCount = %d", sd.BufferUsage, sd.BufferCount);
        Msg("! sd.OutputWindow = %p, Windowed = %d, SwapEffect = %d, Flags = %d", sd.OutputWindow, sd.Windowed, sd.SwapEffect, sd.Flags);
    }
    return SUCCEEDED(hr);
}

bool CHW::CreateSwapChain2(HWND hwnd)
{
    if (strstr(Core.Params, "-no_dx11_2"))
        return false;

    ZoneScoped;

#ifdef HAS_DX11_2
    IDXGIFactory2* pFactory2{};
    m_pAdapter->GetParent(__uuidof(IDXGIFactory2), (void**)&pFactory2);
    if (!pFactory2)
        return false;

    // Set up the presentation parameters
    DXGI_SWAP_CHAIN_DESC1 desc{};

    RECT rect;
    GetClientRect(hwnd, &rect);
    int w = rect.right - rect.left;
    int h = rect.bottom - rect.top;
    if (w <= 0) w = 640;
    if (h <= 0) h = 480;

    // Back buffer
    desc.Width = Device.dwWidth ? Device.dwWidth : w;
    desc.Height = Device.dwHeight ? Device.dwHeight : h;

    constexpr DXGI_FORMAT formats[] =
    {
        //DXGI_FORMAT_R16G16B16A16_FLOAT,
        //DXGI_FORMAT_R10G10B10A2_UNORM,
        DXGI_FORMAT_R8G8B8A8_UNORM,
    };

    // Select back-buffer format
    desc.Format = SelectFormat(D3D11_FORMAT_SUPPORT_DISPLAY, formats);
    if (desc.Format == DXGI_FORMAT_UNKNOWN)
    {
        Log("! SelectFormat failed to find a display format, falling back to DXGI_FORMAT_R8G8B8A8_UNORM");
        desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    }
    Caps.fTarget = dx11TextureUtils::ConvertTextureFormat(desc.Format);

    // Buffering
    BackBufferCount = 1; // For DXGI_SWAP_EFFECT_FLIP_DISCARD we need at least two
    desc.BufferCount = BackBufferCount;
    desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;

    // Multisample
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;

    // Windoze
    //desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD; // XXX: tearing glitches with flip presentation model
    desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    desc.Scaling = DXGI_SCALING_STRETCH;

    DXGI_SWAP_CHAIN_FULLSCREEN_DESC fulldesc{};
    fulldesc.Windowed = true; // Let SDL handle fullscreen

    // Additional setup
    // Note: DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH is intentionally omitted.
    // Fullscreen is managed by SDL3, not DXGI.
    desc.Flags = 0;

    IDXGISwapChain1* swapchain{};
    const HRESULT result = pFactory2->CreateSwapChainForHwnd(pDevice, hwnd, &desc,
        fulldesc.Windowed ? nullptr : &fulldesc, nullptr, &swapchain);
    _RELEASE(pFactory2);

    if (FAILED(result))
        return false;

    if (FAILED(swapchain->GetDesc(&m_ChainDesc)))
    {
        _RELEASE(swapchain);
        return false;
    }
    m_pSwapChain = swapchain;

    m_pSwapChain->QueryInterface(__uuidof(IDXGISwapChain2), reinterpret_cast<void**>(&m_pSwapChain2));

    if (m_pSwapChain2 && ThisInstanceIsGlobal())
        Device.PresentationFinished = m_pSwapChain2->GetFrameLatencyWaitableObject();

    return true;
#else // #ifdef HAS_DX11_2
    UNUSED(hwnd);
#endif

    return false;
}

bool CHW::ThisInstanceIsGlobal() const
{
    return this == &HW;
}

void CHW::DestroyDevice()
{
    if (ThisInstanceIsGlobal()) // only if we are global HW
    {
        RSManager.ClearStateArray();
        DSSManager.ClearStateArray();
        BSManager.ClearStateArray();
        SSManager.ClearStateArray();
    }
    //  Must switch to windowed mode to release swap chain
    if (!m_ChainDesc.Windowed && m_pSwapChain)
    {
        // Removed SetFullscreenState(FALSE, NULL)
    }
#ifdef HAS_DX11_2
    _RELEASE(m_pSwapChain2);
#endif
    _SHOW_REF("refCount:m_pSwapChain", m_pSwapChain);
    _RELEASE(m_pSwapChain);

    if (profiler_ctx)
        TracyD3D11Destroy(profiler_ctx);

    _RELEASE(pContext1);
    for (int id = 0; id < R__NUM_CONTEXTS; ++id)
    {
        _SHOW_REF("refCount:pContext", d3d_contexts_pool[id]);
        _RELEASE(d3d_contexts_pool[id]);
    }

#ifdef HAS_DX11_3
    _RELEASE(pDevice3);
#endif
    _SHOW_REF("refCount:pDevice:", pDevice);
    _RELEASE(pDevice);
    DestroyD3D();
}

//////////////////////////////////////////////////////////////////////
// Resetting device
//////////////////////////////////////////////////////////////////////
void CHW::Reset()
{
    ZoneScoped;
    DXGI_SWAP_CHAIN_DESC& cd = m_ChainDesc;
    const bool bWindowed = true; // Let SDL handle fullscreen
    cd.Windowed = bWindowed;
    // m_pSwapChain->SetFullscreenState(!bWindowed, NULL); // Removed to prevent DXVK Wayland deadlocks
    DXGI_MODE_DESC& desc = m_ChainDesc.BufferDesc;
    desc.Width = Device.dwWidth;
    desc.Height = Device.dwHeight;

    for (int i = 0; i < R__NUM_CONTEXTS; ++i)
    {
        if (d3d_contexts_pool[i])
        {
            d3d_contexts_pool[i]->ClearState();
            d3d_contexts_pool[i]->Flush();
        }
    }

    // NOTE: ResizeTarget is intentionally omitted here.
    // On DXVK/Wayland it attempts to change the DXGI output's display mode,
    // which deadlocks with the compositor's surface resize protocol.
    // SDL already manages the window geometry, so we only need ResizeBuffers
    // to re-create the swap chain backbuffers at the new size.
    HRESULT hrResize = m_pSwapChain->ResizeBuffers(
        cd.BufferCount, desc.Width, desc.Height, desc.Format, cd.Flags);
    if (FAILED(hrResize))
    {
        string128 buf;
        sprintf(buf, "! ResizeBuffers failed with HRESULT: 0x%08X", hrResize);
        Log(buf);
        sprintf(buf, "! desc.Width = %d, desc.Height = %d, desc.Format = %d, cd.Flags = %d", desc.Width, desc.Height, desc.Format, cd.Flags);
        Log(buf);
        FlushLog();
        // Skip CHK_DX to prevent crash, let it try to survive
    }

    // Update cached chain description after resize
    m_pSwapChain->GetDesc(&m_ChainDesc);
}

void CHW::SetPrimaryAttributes(u32& /*windowFlags*/)
{

}

bool CHW::CheckFormatSupport(const DXGI_FORMAT format, const u32 feature) const
{
    u32 supports;

    if (SUCCEEDED(pDevice->CheckFormatSupport(format, &supports)))
    {
        if (supports & feature)
            return true;
    }

    return false;
}

DXGI_FORMAT CHW::SelectFormat(D3D_FORMAT_SUPPORT feature, const DXGI_FORMAT formats[], size_t count) const
{
    for (size_t i = 0; i < count; ++i)
        if (CheckFormatSupport(formats[i], feature))
            return formats[i];

    return DXGI_FORMAT_UNKNOWN;
}

bool CHW::UsingFlipPresentationModel() const
{
    return m_ChainDesc.SwapEffect == DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL
#ifdef HAS_DXGI1_4
        || m_ChainDesc.SwapEffect == DXGI_SWAP_EFFECT_FLIP_DISCARD
#endif
    ;
}

std::pair<u32, u32> CHW::GetSurfaceSize() const
{
    return
    {
        m_ChainDesc.BufferDesc.Width,
        m_ChainDesc.BufferDesc.Height
    };
}

void CHW::BeginScene() { }
void CHW::EndScene() { }

void CHW::Present()
{
    const bool bUseVSync = psDeviceMode.WindowStyle == rsFullscreen &&
        psDeviceFlags.test(rsVSync); // xxx: weird tearing glitches when VSync turned on for windowed mode in DX11

    switch (m_pSwapChain->Present(bUseVSync ? 1 : 0, 0))
    {
    case DXGI_STATUS_OCCLUDED:
    case DXGI_ERROR_DEVICE_REMOVED:
        doPresentTest = true;
        break;
    }

    CurrentBackBuffer = (CurrentBackBuffer + 1) % BackBufferCount;

    TracyD3D11Collect(profiler_ctx);
}

DeviceState CHW::GetDeviceState()
{
    if (doPresentTest)
    {
        switch (m_pSwapChain->Present(0, DXGI_PRESENT_TEST))
        {
        case S_OK:
            doPresentTest = false;
            break;

        case DXGI_STATUS_OCCLUDED:
            // Do not render until we become visible again
            return DeviceState::Lost;

        case DXGI_ERROR_DEVICE_RESET:
            return DeviceState::NeedReset;

        case DXGI_ERROR_DEVICE_REMOVED:
            FATAL("Graphics driver was updated or GPU was physically removed from computer.\n"
                  "Please, restart the game.");
            break;
        }
    }

    return DeviceState::Normal;
}
} // namespace xray::render::RENDER_NAMESPACE
