#pragma once

#include "CommonTypes.h"
#include "Layers/xrRender/HWCaps.h"
#include "xrCore/ModuleLookup.hpp"
#include "xrCore/Threading/Lock.hpp"
#include "xrCore/Threading/ScopeLock.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

namespace xray::render::RENDER_NAMESPACE
{
class CHW
    : public pureAppActivate,
      public pureAppDeactivate
{
public:
    CHW();
    ~CHW();

    void CreateDevice(SDL_Window* sdlWnd);
    void DestroyDevice();

    void Reset();

    void SetPrimaryAttributes(u32& windowFlags);

    std::pair<u32, u32> GetSurfaceSize() const;
    DeviceState GetDeviceState() const;

public:
    void BeginScene();
    void EndScene();
    void Present();

public:
    void OnAppActivate() override;
    void OnAppDeactivate() override;

    void BeginPixEvent(pcstr name) const;
    void EndPixEvent() const;

public:
    static constexpr auto IMM_CTX_ID = 0;
    static constexpr u32 MAX_FRAMES_IN_FLIGHT = 2;

    CHWCaps Caps;
    bool Valid{ false };

    u32 BackBufferCount{ 2 };
    u32 CurrentBackBuffer{ 0 };

    SDL_Window* m_window{ nullptr };

    VkInstance m_instance{ VK_NULL_HANDLE };
    VkSurfaceKHR m_surface{ VK_NULL_HANDLE };
    VkPhysicalDevice m_physicalDevice{ VK_NULL_HANDLE };
    VkDevice pDevice{ VK_NULL_HANDLE };

    VkQueue m_graphicsQueue{ VK_NULL_HANDLE };
    VkQueue m_presentQueue{ VK_NULL_HANDLE };
    Lock m_graphicsQueueLock;
    u32 m_graphicsQueueFamily{ 0 };
    u32 m_presentQueueFamily{ 0 };

    VmaAllocator m_allocator{ VK_NULL_HANDLE };

    VkSwapchainKHR m_swapchain{ VK_NULL_HANDLE };
    VkFormat m_swapchainFormat{ VK_FORMAT_B8G8R8A8_UNORM };
    VkExtent2D m_swapchainExtent{ 0, 0 };
    xr_vector<VkImage> m_swapchainImages;
    xr_vector<VkImageView> m_swapchainImageViews;

    VkDebugUtilsMessengerEXT m_debugMessenger{ VK_NULL_HANDLE };

    VkCommandPool m_cmdPool{ VK_NULL_HANDLE };
    VkCommandBuffer m_cmdBuffers[MAX_FRAMES_IN_FLIGHT]{};
    VkSemaphore m_imageAvailableSemaphores[MAX_FRAMES_IN_FLIGHT]{};
    VkSemaphore m_renderFinishedSemaphores[MAX_FRAMES_IN_FLIGHT]{};
    VkFence m_inFlightFences[MAX_FRAMES_IN_FLIGHT]{};

    u32 m_currentFrame{ 0 };
    u32 m_imageIndex{ 0 };
    bool m_frameStarted{ false };

    pcstr AdapterName{ "Vulkan GPU" };
    pcstr VulkanVersionString{ "Vulkan 1.3" };

    VkCommandBuffer GetCurrentCommandBuffer() const { return m_cmdBuffers[m_currentFrame]; }
    VkImageView GetCurrentBackBufferView() const
    {
        if (m_imageIndex < m_swapchainImageViews.size())
            return m_swapchainImageViews[m_imageIndex];
        return VK_NULL_HANDLE;
    }
    VkImage GetCurrentBackBufferImage() const
    {
        if (m_imageIndex < m_swapchainImages.size())
            return m_swapchainImages[m_imageIndex];
        return VK_NULL_HANDLE;
    }

private:
    bool CreateInstance();
    bool SelectPhysicalDevice();
    bool CreateLogicalDevice();
    bool CreateSwapChain();
    void CleanupSwapChain();
    bool CreateSyncObjects();
};

extern ECORE_API CHW HW;
} // namespace xray::render::RENDER_NAMESPACE
