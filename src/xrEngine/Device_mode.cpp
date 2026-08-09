#include "stdafx.h"

#include "xrCore/xr_token.h"
#include "xr_input.h"

xr_vector<xr_token> vid_monitor_token;
xr_map<u32, xr_vector<xr_token>> vid_mode_token;

void FillResolutionsForMonitor(const int monitorID)
{
    int modeCount = 0;
    if (SDL_DisplayMode** modes = SDL_GetFullscreenDisplayModes((SDL_DisplayID)monitorID, &modeCount))
    {
        for (int i = 0; i < modeCount; ++i)
        {
            SDL_DisplayMode* mode = modes[i];
            string256 buf;
            xr_sprintf(buf, sizeof(buf), "%ux%u (%dHz)", mode->w, mode->h, (int)mode->refresh_rate);
            vid_mode_token[monitorID].emplace_back(xr_strdup(buf), i);
        }
        SDL_free(modes);
    }
    vid_mode_token[monitorID].emplace_back(nullptr, -1);
}

void FillImGuiMonitorData(const int monitorID)
{
    ImGuiPlatformIO& platform_io = ImGui::GetPlatformIO();

    // Warning: the validity of monitor DPI information on Windows
    // depends on the application DPI awareness settings,
    // which generally needs to be set in the manifest or at runtime.
    ImGuiPlatformMonitor monitor;
    SDL_Rect r;
    SDL_GetDisplayBounds(monitorID, &r);
    monitor.MainPos = monitor.WorkPos = ImVec2((float)r.x, (float)r.y);
    monitor.MainSize = monitor.WorkSize = ImVec2((float)r.w, (float)r.h);

    SDL_GetDisplayUsableBounds(monitorID, &r);
    monitor.WorkPos = ImVec2((float)r.x, (float)r.y);
    monitor.WorkSize = ImVec2((float)r.w, (float)r.h);

    // FIXME-VIEWPORT: On MacOS SDL reports actual monitor DPI scale, ignoring OS configuration. We may want to set
    //  DpiScale to cocoa_window.backingScaleFactor here.
    monitor.DpiScale = SDL_GetDisplayContentScale((SDL_DisplayID)monitorID);

    monitor.PlatformHandle = (void*)(intptr_t)monitorID;
    platform_io.Monitors.push_back(monitor);
}

void CRenderDevice::FillVideoModes()
{
    ZoneScoped;

    int displayCount = 0;
    if (SDL_DisplayID* displays = SDL_GetDisplays(&displayCount))
    {
        for (int i = 0; i < displayCount; ++i)
        {
            SDL_DisplayID displayID = displays[i];
            string256 buf;
            xr_sprintf(buf, "%d. %s", (int)displayID, SDL_GetDisplayName(displayID));
            vid_monitor_token.emplace_back(xr_strdup(buf), (int)displayID);

            FillResolutionsForMonitor((int)displayID);
            FillImGuiMonitorData((int)displayID);
        }
        SDL_free(displays);
    }
    vid_monitor_token.emplace_back(nullptr, -1);
}

void CRenderDevice::CleanupVideoModes()
{
    ZoneScoped;

    for (auto& [monitor_id, tokens] : vid_mode_token)
    {
        for (auto& token : tokens)
        {
            auto tokenName = const_cast<pstr>(token.name);
            xr_free(tokenName);
        }
        tokens.clear();
    }
    vid_mode_token.clear();

    for (auto& token : vid_monitor_token)
    {
        pstr tokenName = const_cast<pstr>(token.name);
        xr_free(tokenName);
    }
    vid_monitor_token.clear();

    ImGui::GetPlatformIO().Monitors.resize(0);
}

void CRenderDevice::SetWindowDraggable(bool draggable)
{
    // Only draggable if resizable too
    const bool windowed = psDeviceMode.WindowStyle == rsWindowed || psDeviceMode.WindowStyle == rsWindowedBorderless;
    const bool resizable = SDL_GetWindowFlags(Device.m_sdlWnd) & SDL_WINDOW_RESIZABLE;
    m_allowWindowDrag = draggable && windowed && resizable;

    SDL_SetWindowOpacity(Device.m_sdlWnd, m_allowWindowDrag ? 0.95f : 1.0f);
}

void CRenderDevice::UpdateWindowProps()
{
    ZoneScoped;

    const bool windowed = psDeviceMode.WindowStyle != rsFullscreen;
    SelectResolution(windowed);

    // Changing monitor, unset fullscreen for the previous monitor
    // and move the window to the new monitor
    if (SDL_GetDisplayForWindow(m_sdlWnd) != static_cast<SDL_DisplayID>(psDeviceMode.Monitor))
    {
        SDL_SetWindowFullscreen(m_sdlWnd, false);

        SDL_Rect rect;
        SDL_GetDisplayBounds(psDeviceMode.Monitor, &rect);
        SDL_SetWindowPosition(m_sdlWnd, rect.x, rect.y);
    }

    if (psDeviceMode.WindowStyle != rsFullscreenBorderless)
        SDL_SetWindowSize(m_sdlWnd, psDeviceMode.Width, psDeviceMode.Height);
    else
    {
        const SDL_DisplayMode *current = SDL_GetCurrentDisplayMode(psDeviceMode.Monitor);
        if (current) {
            psDeviceMode.Width = current->w;
            psDeviceMode.Height = current->h;
            psDeviceMode.RefreshRate = (u32)current->refresh_rate;
        }

        if (current) SDL_SetWindowSize(m_sdlWnd, current->w, current->h);
    }

    if (windowed)
    {
        const bool drawBorders = psDeviceMode.WindowStyle == rsWindowed;
        const bool useDesktopFullscreen = b_is_Ready && psDeviceMode.WindowStyle == rsFullscreenBorderless;

        SDL_SetWindowBordered(m_sdlWnd, drawBorders);
        SDL_SetWindowResizable(m_sdlWnd, !useDesktopFullscreen);
        SDL_SetWindowFullscreen(m_sdlWnd, useDesktopFullscreen);
    }
    else if (b_is_Ready)
    {
        SDL_SetWindowResizable(m_sdlWnd, false);
        SDL_SetWindowFullscreen(m_sdlWnd, true);

        const SDL_DisplayMode* mode_ptr = SDL_GetWindowFullscreenMode(m_sdlWnd);
        SDL_DisplayMode mode;
        if (mode_ptr) mode = *mode_ptr;
        else SDL_zerop(&mode);
        mode.w = psDeviceMode.Width;
        mode.h = psDeviceMode.Height;
        mode.refresh_rate = (float)psDeviceMode.RefreshRate;
        SDL_SetWindowFullscreenMode(m_sdlWnd, &mode);
    }

    SDL_PumpEvents();
    UpdateWindowRects();

    ImGuiIO& io = ImGui::GetIO();

    io.DisplaySize = { static_cast<float>(psDeviceMode.Width), static_cast<float>(psDeviceMode.Height) };
    io.DisplayFramebufferScale = ImVec2{ float(dwWidth / m_rcWindowClient.w), float(dwHeight / m_rcWindowClient.h) };
}

void CRenderDevice::UpdateWindowRects()
{
    m_rcWindowClient.x = 0;
    m_rcWindowClient.y = 0;
    SDL_GetWindowSize(m_sdlWnd, &m_rcWindowClient.w, &m_rcWindowClient.h);

    SDL_GetWindowPosition(m_sdlWnd, &m_rcWindowBounds.x, &m_rcWindowBounds.y);
    SDL_GetWindowSize(m_sdlWnd, &m_rcWindowBounds.w, &m_rcWindowBounds.h);

    int top, left, bottom, right;
    SDL_GetWindowBordersSize(m_sdlWnd, &top, &left, &bottom, &right);
    m_rcWindowBounds.x -= left;
    m_rcWindowBounds.y -= top;
    m_rcWindowBounds.w += right;
    m_rcWindowBounds.h += bottom;
}

void CRenderDevice::SelectResolution(const bool windowed)
{
    // Dedicated server hardcoded resolution
    // XXX: to be removed
    if (GEnv.isDedicatedServer)
    {
        psDeviceMode.Width = 640;
        psDeviceMode.Height = 480;
    }
    else if (psDeviceMode.Width == 0 && psDeviceMode.Height == 0 && psDeviceMode.RefreshRate == 0)
    {
        const SDL_DisplayMode *current = SDL_GetCurrentDisplayMode(psDeviceMode.Monitor);
        if (current) {
            psDeviceMode.Width = current->w;
            psDeviceMode.Height = current->h;
            psDeviceMode.RefreshRate = (u32)current->refresh_rate;
        }
        
    }
    else if (!windowed) // check if safe for fullscreen
    {
        string256 buf;
        xr_sprintf(buf, "%ux%u (%dHz)", psDeviceMode.Width, psDeviceMode.Height, psDeviceMode.RefreshRate);

        auto modes = vid_mode_token[psDeviceMode.Monitor];
        const auto it = std::find_if(modes.begin(), modes.end(), [&buf](const xr_token& token)
        {
            return token.name && xr_strcmp(token.name, buf) == 0;
        });

        if (it == modes.end()) // not found
        {
            SDL_DisplayMode closest;
            if (!SDL_GetClosestFullscreenDisplayMode(psDeviceMode.Monitor, (int)psDeviceMode.Width, (int)psDeviceMode.Height, (float)psDeviceMode.RefreshRate, false, &closest))
            {
                const SDL_DisplayMode *curr = SDL_GetCurrentDisplayMode(psDeviceMode.Monitor);
                if (curr) closest = *curr;
            }
            psDeviceMode.Width = closest.w;
            psDeviceMode.Height = closest.h;
            psDeviceMode.RefreshRate = (u32)closest.refresh_rate;
        }
    }

    dwWidth = psDeviceMode.Width;
    dwHeight = psDeviceMode.Height;
}

SDL_Window* CRenderDevice::GetApplicationWindow()
{
    return m_sdlWnd;
}

void CRenderDevice::OnErrorDialog(bool beforeDialog)
{
    const bool restore = !beforeDialog;
    const bool needUpdateInput = pInput && pInput->IsExclusiveMode();

    if (restore)
        UpdateWindowProps();
    else
        SDL_SetWindowFullscreen(m_sdlWnd, false);

    if (needUpdateInput)
        pInput->GrabInput(restore);
}

void CRenderDevice::OnFatalError()
{
    // make it sure window will hide in any way
    SDL_SetWindowFullscreen(m_sdlWnd, false);
    SDL_SetWindowAlwaysOnTop(m_sdlWnd, false);
    SDL_ShowWindow(m_sdlWnd);
    SDL_MinimizeWindow(m_sdlWnd);
    SDL_HideWindow(m_sdlWnd);
}
