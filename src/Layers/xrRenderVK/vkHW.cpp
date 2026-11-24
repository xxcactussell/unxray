#include "stdafx.h"
#pragma hdrstop

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"

#include "vkHW.h"
#include "vkTextureUtils.h"
#include "vkImageTracker.h"
#include "vkDeletionQueue.h"
#include "vkRingBuffer.h"
#include "r2.h"

namespace xray::render::RENDER_NAMESPACE
{
ECORE_API CHW HW;

CHW::CHW()
{
    Caps.iGPUNum = 1;
}

CHW::~CHW()
{
    DestroyDevice();
}

void CHW::SetPrimaryAttributes(u32& windowFlags)
{
    windowFlags |= SDL_WINDOW_VULKAN;
}

std::pair<u32, u32> CHW::GetSurfaceSize() const
{
    if (m_window)
    {
        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(m_window, &w, &h);
        return { static_cast<u32>(w), static_cast<u32>(h) };
    }
    return { 0, 0 };
}

DeviceState CHW::GetDeviceState() const
{
    return DeviceState::Normal;
}

void CHW::OnAppActivate() {}
void CHW::OnAppDeactivate() {}

void CHW::BeginPixEvent(pcstr name) const
{
#ifdef DEBUG
    VkCommandBuffer cmd = m_cmdBuffers[m_currentFrame];
    VkDebugUtilsLabelEXT label{ VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT };
    label.pLabelName = name;
    label.color[0] = 1.0f; label.color[1] = 1.0f;
    label.color[2] = 0.0f; label.color[3] = 1.0f;
    vkCmdBeginDebugUtilsLabelEXT(cmd, &label);
#endif
}

void CHW::EndPixEvent() const
{
#ifdef DEBUG
    VkCommandBuffer cmd = m_cmdBuffers[m_currentFrame];
    vkCmdEndDebugUtilsLabelEXT(cmd);
#endif
}

bool CHW::CreateInstance()
{
    if (volkInitialize() != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to initialize volk loader!");
        return false;
    }

    u32 sdlExtCount = 0;
    const char* const* sdlExtensions = SDL_Vulkan_GetInstanceExtensions(&sdlExtCount);
    if (!sdlExtensions || sdlExtCount == 0)
    {
        Msg("! [Vulkan] SDL_Vulkan_GetInstanceExtensions failed: %s", SDL_GetError());
        return false;
    }

    xr_vector<const char*> extensions;
    for (u32 i = 0; i < sdlExtCount; ++i)
        extensions.push_back(sdlExtensions[i]);

    xr_vector<const char*> enabledLayerNames;
#ifdef DEBUG
    u32 layerCount = 0;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    if (layerCount > 0)
    {
        xr_vector<VkLayerProperties> availableLayers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());
        for (const auto& layer : availableLayers)
        {
            if (xr_strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0)
            {
                enabledLayerNames.push_back("VK_LAYER_KHRONOS_validation");
                Msg("* [Vulkan] Enabled validation layer: VK_LAYER_KHRONOS_validation");
                break;
            }
        }
    }

    u32 instanceExtCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &instanceExtCount, nullptr);
    bool hasDebugUtils = false;
    if (instanceExtCount > 0)
    {
        xr_vector<VkExtensionProperties> availableExts(instanceExtCount);
        vkEnumerateInstanceExtensionProperties(nullptr, &instanceExtCount, availableExts.data());
        for (const auto& ext : availableExts)
        {
            if (xr_strcmp(ext.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0)
            {
                hasDebugUtils = true;
                break;
            }
        }
    }
    if (hasDebugUtils)
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
#endif

    VkApplicationInfo appInfo{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    appInfo.pApplicationName = "OpenXRay";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "X-Ray Engine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_3;

    VkInstanceCreateInfo createInfo{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<u32>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();
    createInfo.enabledLayerCount = static_cast<u32>(enabledLayerNames.size());
    createInfo.ppEnabledLayerNames = enabledLayerNames.empty() ? nullptr : enabledLayerNames.data();

    VkResult res = vkCreateInstance(&createInfo, nullptr, &m_instance);
    if (res != VK_SUCCESS && !enabledLayerNames.empty())
    {
        Msg("! [Vulkan] vkCreateInstance with layers returned %d, retrying without layers...", res);
        createInfo.enabledLayerCount = 0;
        createInfo.ppEnabledLayerNames = nullptr;
        res = vkCreateInstance(&createInfo, nullptr, &m_instance);
    }
    if (res != VK_SUCCESS)
    {
        // Try fallback to Vulkan 1.2
        appInfo.apiVersion = VK_API_VERSION_1_2;
        res = vkCreateInstance(&createInfo, nullptr, &m_instance);
        if (res != VK_SUCCESS)
        {
            Msg("! [Vulkan] Failed to create VkInstance! Result = %d", res);
            return false;
        }
    }

    volkLoadInstance(m_instance);

#ifdef DEBUG
    if (hasDebugUtils)
    {
        static auto debugCallback = [](
            VkDebugUtilsMessageSeverityFlagBitsEXT severity,
            VkDebugUtilsMessageTypeFlagsEXT /*type*/,
            const VkDebugUtilsMessengerCallbackDataEXT* data,
            void* /*user*/) -> VkBool32
        {
            if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
                Msg("! [VK Validation] %s", data->pMessage);
            else
                Msg("~ [VK Validation] %s", data->pMessage);
            return VK_FALSE;
        };

        VkDebugUtilsMessengerCreateInfoEXT messengerInfo{ VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        messengerInfo.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        messengerInfo.messageType =
            VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        messengerInfo.pfnUserCallback = debugCallback;

        auto vkCreateDebugUtilsMessengerEXT_fn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT"));
        if (vkCreateDebugUtilsMessengerEXT_fn)
            vkCreateDebugUtilsMessengerEXT_fn(m_instance, &messengerInfo, nullptr, &m_debugMessenger);
    }
#endif

    Msg("* [Vulkan] VkInstance created successfully");
    return true;
}

bool CHW::SelectPhysicalDevice()
{
    u32 deviceCount = 0;
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);
    if (deviceCount == 0)
    {
        Msg("! [Vulkan] No physical devices with Vulkan support found!");
        return false;
    }

    xr_vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

    VkPhysicalDevice selected = VK_NULL_HANDLE;
    for (const auto& dev : devices)
    {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);

        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
        {
            selected = dev;
            break;
        }
    }

    if (selected == VK_NULL_HANDLE)
        selected = devices[0];

    m_physicalDevice = selected;

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(m_physicalDevice, &props);
    static string256 gpuName;
    xr_strcpy(gpuName, props.deviceName);
    AdapterName = gpuName;
    Caps.id_vendor = props.vendorID;
    Caps.id_device = props.deviceID;

    Msg("* [Vulkan] Selected GPU: %s (Vendor: 0x%04X, Device: 0x%04X, Driver: %u.%u.%u)",
        AdapterName, props.vendorID, props.deviceID,
        VK_VERSION_MAJOR(props.driverVersion),
        VK_VERSION_MINOR(props.driverVersion),
        VK_VERSION_PATCH(props.driverVersion));

    u32 queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, nullptr);
    xr_vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, queueFamilies.data());

    bool foundGraphics = false;
    bool foundPresent = false;

    for (u32 i = 0; i < queueFamilyCount; ++i)
    {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
        {
            m_graphicsQueueFamily = i;
            foundGraphics = true;
        }

        VkBool32 presentSupport = false;
        if (m_surface)
            vkGetPhysicalDeviceSurfaceSupportKHR(m_physicalDevice, i, m_surface, &presentSupport);
        else
            presentSupport = true;

        if (presentSupport)
        {
            m_presentQueueFamily = i;
            foundPresent = true;
        }

        if (foundGraphics && foundPresent)
            break;
    }

    return foundGraphics && foundPresent;
}

bool CHW::CreateLogicalDevice()
{
    xr_vector<VkDeviceQueueCreateInfo> queueCreateInfos;
    xr_vector<u32> uniqueQueueFamilies = { m_graphicsQueueFamily };
    if (m_presentQueueFamily != m_graphicsQueueFamily)
        uniqueQueueFamilies.push_back(m_presentQueueFamily);

    float queuePriority = 1.0f;
    for (u32 queueFamily : uniqueQueueFamilies)
    {
        VkDeviceQueueCreateInfo queueCreateInfo{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
        queueCreateInfo.queueFamilyIndex = queueFamily;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        queueCreateInfos.push_back(queueCreateInfo);
    }

    xr_vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME
    };

    VkPhysicalDeviceExtendedDynamicStateFeaturesEXT extDynamicStateFeature{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTENDED_DYNAMIC_STATE_FEATURES_EXT
    };
    extDynamicStateFeature.extendedDynamicState = VK_TRUE;

    VkPhysicalDeviceVulkan13Features vk13Features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES
    };
    vk13Features.pNext = &extDynamicStateFeature;
    vk13Features.dynamicRendering = VK_TRUE;
    vk13Features.synchronization2 = VK_TRUE;

    VkPhysicalDeviceDescriptorIndexingFeatures indexingFeatures{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES
    };
    indexingFeatures.pNext = &vk13Features;
    indexingFeatures.descriptorBindingPartiallyBound = VK_TRUE;
    indexingFeatures.descriptorBindingVariableDescriptorCount = VK_TRUE;
    indexingFeatures.runtimeDescriptorArray = VK_TRUE;

    VkPhysicalDeviceFeatures2 deviceFeatures2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    deviceFeatures2.pNext = &indexingFeatures;
    deviceFeatures2.features.shaderClipDistance = VK_TRUE;
    deviceFeatures2.features.fillModeNonSolid = VK_TRUE;
    deviceFeatures2.features.samplerAnisotropy = VK_TRUE;

    VkDeviceCreateInfo createInfo{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    createInfo.pNext = &deviceFeatures2;
    createInfo.queueCreateInfoCount = static_cast<u32>(queueCreateInfos.size());
    createInfo.pQueueCreateInfos = queueCreateInfos.data();
    createInfo.enabledExtensionCount = static_cast<u32>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    VkResult res = vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &pDevice);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create logical VkDevice! Result = %d", res);
        return false;
    }

    volkLoadDevice(pDevice);

    vkGetDeviceQueue(pDevice, m_graphicsQueueFamily, 0, &m_graphicsQueue);
    vkGetDeviceQueue(pDevice, m_presentQueueFamily, 0, &m_presentQueue);

    // Initialize VMA
    VmaVulkanFunctions vmaFunctions{};
    vmaFunctions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
    vmaFunctions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;

    VmaAllocatorCreateInfo allocInfo{};
    allocInfo.vulkanApiVersion = VK_API_VERSION_1_3;
    allocInfo.physicalDevice = m_physicalDevice;
    allocInfo.device = pDevice;
    allocInfo.instance = m_instance;
    allocInfo.pVulkanFunctions = &vmaFunctions;

    res = vmaCreateAllocator(&allocInfo, &m_allocator);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create VmaAllocator! Result = %d", res);
        return false;
    }

    Msg("* [Vulkan] Logical device and memory allocator created successfully");
    return true;
}

bool CHW::CreateSwapChain()
{
    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_physicalDevice, m_surface, &capabilities);

    u32 formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, nullptr);
    xr_vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, formats.data());

    VkSurfaceFormatKHR surfaceFormat = formats[0];
    for (const auto& f : formats)
    {
        if (f.format == VK_FORMAT_B8G8R8A8_UNORM && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
        {
            surfaceFormat = f;
            break;
        }
    }
    m_swapchainFormat = surfaceFormat.format;

    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(m_window, &width, &height);

    if (width <= 0 || height <= 0)
        return false;

    m_swapchainExtent.width = std::clamp(static_cast<u32>(width), capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
    m_swapchainExtent.height = std::clamp(static_cast<u32>(height), capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

    u32 imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
        imageCount = capabilities.maxImageCount;

    VkSwapchainCreateInfoKHR createInfo{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
    createInfo.surface = m_surface;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = m_swapchainExtent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    u32 queueFamilyIndices[] = { m_graphicsQueueFamily, m_presentQueueFamily };
    if (m_graphicsQueueFamily != m_presentQueueFamily)
    {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    }
    else
    {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR; // V-Sync enabled by default
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    VkResult res = vkCreateSwapchainKHR(pDevice, &createInfo, nullptr, &m_swapchain);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create swapchain! Result = %d", res);
        return false;
    }

    vkGetSwapchainImagesKHR(pDevice, m_swapchain, &imageCount, nullptr);
    m_swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(pDevice, m_swapchain, &imageCount, m_swapchainImages.data());

    m_swapchainImageViews.resize(imageCount);
    for (size_t i = 0; i < imageCount; ++i)
    {
        VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        res = vkCreateImageView(pDevice, &viewInfo, nullptr, &m_swapchainImageViews[i]);
        if (res != VK_SUCCESS)
        {
            Msg("! [Vulkan] Failed to create swapchain image view %zu! Result = %d", i, res);
            return false;
        }
        vkTextureUtils::RegisterImage(m_swapchainImages[i], m_swapchainImageViews[i], m_swapchainFormat, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
    }

    BackBufferCount = imageCount;
    Msg("* [Vulkan] Swapchain created: %ux%u (%u images, format %d)",
        m_swapchainExtent.width, m_swapchainExtent.height, imageCount, m_swapchainFormat);
    return true;
}

void CHW::CleanupSwapChain()
{
    if (pDevice != VK_NULL_HANDLE)
    {
        vkDeviceWaitIdle(pDevice);

        for (auto view : m_swapchainImageViews)
        {
            if (view != VK_NULL_HANDLE)
            {
                vkTextureUtils::UnregisterImage(view);
                vkDestroyImageView(pDevice, view, nullptr);
            }
        }
        m_swapchainImageViews.clear();
        m_swapchainImages.clear();

        if (m_swapchain != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(pDevice, m_swapchain, nullptr);
            m_swapchain = VK_NULL_HANDLE;
        }
    }
}

bool CHW::CreateSyncObjects()
{
    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.queueFamilyIndex = m_graphicsQueueFamily;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    VkResult res = vkCreateCommandPool(pDevice, &poolInfo, nullptr, &m_cmdPool);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to create command pool! Result = %d", res);
        return false;
    }

    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool = m_cmdPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = MAX_FRAMES_IN_FLIGHT;

    res = vkAllocateCommandBuffers(pDevice, &allocInfo, m_cmdBuffers);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] Failed to allocate command buffers! Result = %d", res);
        return false;
    }

    VkSemaphoreCreateInfo semaphoreInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo fenceInfo{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (u32 i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i)
    {
        vkCreateSemaphore(pDevice, &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]);
        vkCreateSemaphore(pDevice, &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]);
        vkCreateFence(pDevice, &fenceInfo, nullptr, &m_inFlightFences[i]);
    }

    return true;
}

void CHW::CreateDevice(SDL_Window* sdlWnd)
{
    m_window = sdlWnd;

    if (!CreateInstance()) return;

    if (!SDL_Vulkan_CreateSurface(m_window, m_instance, nullptr, &m_surface))
    {
        Msg("! [Vulkan] SDL_Vulkan_CreateSurface failed: %s", SDL_GetError());
        return;
    }

    if (!SelectPhysicalDevice()) return;
    if (!CreateLogicalDevice()) return;
    if (!CreateSwapChain()) return;
    if (!CreateSyncObjects()) return;

    G_FrameRingBuffers.Init();

    Caps.Update();
    Valid = true;
    Msg("* [Vulkan] Hardware device initialized successfully!");
}

void CHW::DestroyDevice()
{
    if (pDevice != VK_NULL_HANDLE)
    {
        vkDeviceWaitIdle(pDevice);

        G_FrameRingBuffers.Destroy();
        G_DeletionQueue.FlushAll();

        for (u32 i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i)
        {
            if (m_imageAvailableSemaphores[i] != VK_NULL_HANDLE)
                vkDestroySemaphore(pDevice, m_imageAvailableSemaphores[i], nullptr);
            if (m_renderFinishedSemaphores[i] != VK_NULL_HANDLE)
                vkDestroySemaphore(pDevice, m_renderFinishedSemaphores[i], nullptr);
            if (m_inFlightFences[i] != VK_NULL_HANDLE)
                vkDestroyFence(pDevice, m_inFlightFences[i], nullptr);
        }

        if (m_cmdPool != VK_NULL_HANDLE)
        {
            vkDestroyCommandPool(pDevice, m_cmdPool, nullptr);
            m_cmdPool = VK_NULL_HANDLE;
        }

        CleanupSwapChain();

        if (m_allocator != VK_NULL_HANDLE)
        {
            vmaDestroyAllocator(m_allocator);
            m_allocator = VK_NULL_HANDLE;
        }

        vkDestroyDevice(pDevice, nullptr);
        pDevice = VK_NULL_HANDLE;
    }

    if (m_surface != VK_NULL_HANDLE)
    {
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
        m_surface = VK_NULL_HANDLE;
    }

    if (m_instance != VK_NULL_HANDLE)
    {
#ifdef DEBUG
        if (m_debugMessenger != VK_NULL_HANDLE)
        {
            auto vkDestroyDebugUtilsMessengerEXT_fn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
                vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT"));
            if (vkDestroyDebugUtilsMessengerEXT_fn)
                vkDestroyDebugUtilsMessengerEXT_fn(m_instance, m_debugMessenger, nullptr);
            m_debugMessenger = VK_NULL_HANDLE;
        }
#endif
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }

    Valid = false;
    m_window = nullptr;
}

void CHW::Reset()
{
    if (!m_window || !pDevice) return;
    vkDeviceWaitIdle(pDevice);
    CleanupSwapChain();
    CreateSwapChain();
}

void CHW::BeginScene()
{
    if (!Valid) return;

    vkWaitForFences(pDevice, 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);
    G_DeletionQueue.Flush(m_currentFrame);
    G_FrameRingBuffers.Reset(m_currentFrame);

    VkResult res = vkAcquireNextImageKHR(pDevice, m_swapchain, UINT64_MAX,
        m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &m_imageIndex);

    if (res != VK_SUCCESS && res != VK_SUBOPTIMAL_KHR)
    {
        if (res == VK_ERROR_OUT_OF_DATE_KHR || res == VK_ERROR_SURFACE_LOST_KHR)
            Reset();
        return;
    }

    CurrentBackBuffer = m_imageIndex;

    vkResetFences(pDevice, 1, &m_inFlightFences[m_currentFrame]);

    extern vkPipelineManager G_PipelineManager;
    G_PipelineManager.ResetDescriptorPool(m_currentFrame);

    VkCommandBuffer cmd = m_cmdBuffers[m_currentFrame];
    vkResetCommandBuffer(cmd, 0);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &beginInfo);

    // Transition backbuffer from UNDEFINED to TRANSFER_DST_OPTIMAL, clear to solid black, then to COLOR_ATTACHMENT_OPTIMAL
    VkImageSubresourceRange subresourceRange{};
    subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    subresourceRange.baseMipLevel = 0;
    subresourceRange.levelCount = 1;
    subresourceRange.baseArrayLayer = 0;
    subresourceRange.layerCount = 1;

    VkImageMemoryBarrier2 clearBarrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    clearBarrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    clearBarrier.srcAccessMask = 0;
    clearBarrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    clearBarrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    clearBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    clearBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    clearBarrier.image = m_swapchainImages[m_imageIndex];
    clearBarrier.subresourceRange = subresourceRange;

    VkDependencyInfo clearDepInfo{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    clearDepInfo.imageMemoryBarrierCount = 1;
    clearDepInfo.pImageMemoryBarriers = &clearBarrier;
    vkCmdPipelineBarrier2(cmd, &clearDepInfo);

    VkClearColorValue blackValue = { { 0.0f, 0.0f, 0.0f, 1.0f } };
    vkCmdClearColorImage(cmd, m_swapchainImages[m_imageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &blackValue, 1, &subresourceRange);

    VkImageMemoryBarrier2 attachBarrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    attachBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    attachBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    attachBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    attachBarrier.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    attachBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    attachBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    attachBarrier.image = m_swapchainImages[m_imageIndex];
    attachBarrier.subresourceRange = subresourceRange;

    VkDependencyInfo attachDepInfo{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    attachDepInfo.imageMemoryBarrierCount = 1;
    attachDepInfo.pImageMemoryBarriers = &attachBarrier;
    vkCmdPipelineBarrier2(cmd, &attachDepInfo);

    G_ImageTracker.RegisterImage(m_swapchainImages[m_imageIndex], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    vkTextureUtils::SetViewLayout(m_swapchainImageViews[m_imageIndex], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    m_frameStarted = true;
}

void CHW::EndScene()
{
    if (!Valid || !m_frameStarted) return;

    VkCommandBuffer cmd = m_cmdBuffers[m_currentFrame];

    extern vkPipelineManager G_PipelineManager;
    G_PipelineManager.EndDynamicRendering(cmd);

    // Transition backbuffer image layout from COLOR_ATTACHMENT_OPTIMAL to PRESENT_SRC_KHR
    VkImageMemoryBarrier2 presentBarrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    presentBarrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    presentBarrier.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    presentBarrier.dstStageMask = VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT;
    presentBarrier.dstAccessMask = 0;
    presentBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    presentBarrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    presentBarrier.image = m_swapchainImages[m_imageIndex];
    presentBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    presentBarrier.subresourceRange.baseMipLevel = 0;
    presentBarrier.subresourceRange.levelCount = 1;
    presentBarrier.subresourceRange.baseArrayLayer = 0;
    presentBarrier.subresourceRange.layerCount = 1;

    VkDependencyInfo presentDepInfo{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    presentDepInfo.imageMemoryBarrierCount = 1;
    presentDepInfo.pImageMemoryBarriers = &presentBarrier;
    vkCmdPipelineBarrier2(cmd, &presentDepInfo);

    G_ImageTracker.RegisterImage(m_swapchainImages[m_imageIndex], VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    vkTextureUtils::SetViewLayout(m_swapchainImageViews[m_imageIndex], VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    vkEndCommandBuffer(cmd);
    m_frameStarted = false;
}

void CHW::Present()
{
    if (!Valid) return;

    if (m_frameStarted)
    {
        EndScene();
    }

    ScopeLock lock(&m_graphicsQueueLock);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    VkSemaphore waitSemaphores[] = { m_imageAvailableSemaphores[m_currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_cmdBuffers[m_currentFrame];

    VkSemaphore signalSemaphores[] = { m_renderFinishedSemaphores[m_currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    VkResult res = vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_inFlightFences[m_currentFrame]);
    if (res != VK_SUCCESS)
    {
        Msg("! [Vulkan] vkQueueSubmit failed! Result = %d", res);
        m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
        return;
    }

    VkPresentInfoKHR presentInfo{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &m_swapchain;
    presentInfo.pImageIndices = &m_imageIndex;

    res = vkQueuePresentKHR(m_presentQueue, &presentInfo);
    if (res == VK_ERROR_OUT_OF_DATE_KHR || res == VK_SUBOPTIMAL_KHR)
    {
        Reset();
    }

    static u32 s_vkPresentCount = 0;
    s_vkPresentCount++;
    if (s_vkPresentCount == 60 || s_vkPresentCount == 150)
    {
        RImplementation.Screenshot(IRender::ScreenshotMode::SM_NORMAL);
    }

    m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}
} // namespace xray::render::RENDER_NAMESPACE
