#include "stdafx.h"
#include <windows.h>
#include <unknwn.h>
#include <combaseapi.h>
#include <wtypes.h>
#include <oleauto.h>
#include <dxcapi.h>
#include <d3d12shader.h>

#include "r2.h"
#include "Layers/xrRender/ShaderResourceTraits.h"
#include "Layers/xrRenderVK/vkPipelineManager.h"
#include "xrCore/FileCRC32.h"
#include "spirv_reflect.h"

namespace xray::render::RENDER_NAMESPACE
{
void CRender::addShaderOption(const char* name, const char* value)
{
    // Store as define
    m_ShaderOptions += "#define ";
    m_ShaderOptions += name;
    m_ShaderOptions += " ";
    m_ShaderOptions += value;
    m_ShaderOptions += "\n";
}

class shader_name_holder
{
    size_t pos{};
    string_path name;

public:
    void append(cpcstr string)
    {
        const size_t size = xr_strlen(string);
        for (size_t i = 0; i < size; ++i)
        {
            name[pos] = string[i];
            ++pos;
        }
    }

    void append(u32 value)
    {
        name[pos] = '0' + char(value);
        ++pos;
    }

    void finish()
    {
        name[pos] = '\0';
    }

    pcstr c_str() const { return name; }
};

struct ShaderOption
{
    xr_string name;
    xr_string value;
};

class CEngineDxcIncludeHandler : public IDxcIncludeHandler
{
    IDxcUtils* m_pUtils{ nullptr };
    std::atomic<ULONG> m_refCount{ 1 };

public:
    CEngineDxcIncludeHandler(IDxcUtils* pUtils) : m_pUtils(pUtils) {}

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject) override
    {
        if (!ppvObject) return E_POINTER;
        if (riid == __uuidof(IUnknown) || riid == __uuidof(IDxcIncludeHandler))
        {
            *ppvObject = static_cast<IDxcIncludeHandler*>(this);
            AddRef();
            return S_OK;
        }
        *ppvObject = nullptr;
        return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return ++m_refCount; }
    ULONG STDMETHODCALLTYPE Release() override
    {
        ULONG count = --m_refCount;
        return count;
    }

    HRESULT STDMETHODCALLTYPE LoadSource(LPCWSTR pFilename, IDxcBlob** ppIncludeSource) override
    {
        if (!pFilename || !ppIncludeSource || !m_pUtils)
            return E_FAIL;

        *ppIncludeSource = nullptr;

        int len = WideCharToMultiByte(CP_UTF8, 0, pFilename, -1, nullptr, 0, nullptr, nullptr);
        if (len <= 0)
            return E_FAIL;

        xr_string narrowPath(len, 0);
        WideCharToMultiByte(CP_UTF8, 0, pFilename, -1, &narrowPath[0], len, nullptr, nullptr);
        if (!narrowPath.empty() && narrowPath.back() == '\0')
            narrowPath.pop_back();

        string_path pname;
        strconcat(pname, RImplementation.getShaderPath(), narrowPath.c_str());
        IReader* R = FS.r_open("$game_shaders$", pname);
        if (!R)
        {
            R = FS.r_open("$game_shaders$", narrowPath.c_str());
        }
        if (!R)
        {
            string_path r3path;
            strconcat(r3path, "r3\\", narrowPath.c_str());
            R = FS.r_open("$game_shaders$", r3path);
        }

        if (!R)
        {
            Msg("! [Vulkan DXC] Include not found: '%s'", narrowPath.c_str());
            return HRESULT_FROM_WIN32(ERROR_FILE_NOT_FOUND);
        }

        IDxcBlobEncoding* pBlob = nullptr;
        HRESULT hr = m_pUtils->CreateBlob(R->pointer(), (UINT32)R->length(), DXC_CP_ACP, &pBlob);
        FS.r_close(R);

        if (FAILED(hr) || !pBlob)
            return hr;

        *ppIncludeSource = pBlob;
        return S_OK;
    }
};

class CDxcCompilerHelper
{
    HMODULE m_hDxc{ nullptr };
    IDxcUtils* m_pUtils{ nullptr };
    IDxcCompiler3* m_pCompiler{ nullptr };
    bool m_initialized{ false };

public:
    CDxcCompilerHelper() = default;

    ~CDxcCompilerHelper()
    {
        if (m_pCompiler) { m_pCompiler->Release(); m_pCompiler = nullptr; }
        if (m_pUtils) { m_pUtils->Release(); m_pUtils = nullptr; }
        if (m_hDxc) { FreeLibrary(m_hDxc); m_hDxc = nullptr; }
    }

    IDxcUtils* GetUtils() const { return m_pUtils; }

    bool Init()
    {
        if (m_initialized)
            return (m_pCompiler != nullptr);

        m_initialized = true;

        m_hDxc = LoadLibraryA("dxcompiler.dll");
        if (!m_hDxc)
        {
            // Try Windows SDK fallback path
            m_hDxc = LoadLibraryA("C:\\Program Files (x86)\\Windows Kits\\10\\bin\\10.0.26100.0\\x64\\dxcompiler.dll");
        }

        if (!m_hDxc)
        {
            Msg("! [Vulkan] Failed to load dxcompiler.dll");
            return false;
        }

        DxcCreateInstanceProc pfnDxcCreateInstance = (DxcCreateInstanceProc)GetProcAddress(m_hDxc, "DxcCreateInstance");
        if (!pfnDxcCreateInstance)
        {
            Msg("! [Vulkan] GetProcAddress(DxcCreateInstance) failed");
            return false;
        }

        HRESULT hr = pfnDxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&m_pUtils));
        if (FAILED(hr))
        {
            Msg("! [Vulkan] Failed to create DxcUtils, hr=0x%08x", hr);
            return false;
        }

        hr = pfnDxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&m_pCompiler));
        if (FAILED(hr))
        {
            Msg("! [Vulkan] Failed to create DxcCompiler, hr=0x%08x", hr);
            return false;
        }

        return true;
    }

    HRESULT Compile(const void* srcData, size_t srcSize, pcstr entryPoint, pcstr targetProfile,
        const xr_vector<ShaderOption>& options, pcstr includeDir,
        xr_vector<u8>& outBytecode, xr_vector<u8>& outReflection, xr_string& outErrors)
    {
        if (!Init())
            return E_FAIL;

        DxcBuffer sourceBuffer{};
        sourceBuffer.Ptr = srcData;
        sourceBuffer.Size = srcSize;
        sourceBuffer.Encoding = DXC_CP_ACP;

        xr_vector<std::wstring> argStorage;
        auto addArg = [&](const std::wstring& str)
        {
            argStorage.push_back(str);
        };

        addArg(L"-spirv");
        addArg(L"-fspv-target-env=vulkan1.2");
        addArg(L"-O3");
        addArg(L"-Zpc");
        addArg(L"-Wno-conversion");
        addArg(L"-Wno-switch");
        addArg(L"-fvk-auto-shift-bindings");
        addArg(L"-fvk-b-shift");
        if (targetProfile[0] == 'v')
            addArg(L"0");
        else if (targetProfile[0] == 'p')
            addArg(L"6");
        else if (targetProfile[0] == 'g')
            addArg(L"12");
        else
            addArg(L"0");
        addArg(L"0");
        addArg(L"-fvk-t-shift");
        addArg(L"16");
        addArg(L"0");
        addArg(L"-fvk-s-shift");
        addArg(L"48");
        addArg(L"0");
        addArg(L"-fvk-u-shift");
        addArg(L"64");
        addArg(L"0");

        // Target profile (vs_6_0, ps_6_0, etc.)
        std::wstring wProfile;
        if (targetProfile[0] == 'v') wProfile = L"vs_6_0";
        else if (targetProfile[0] == 'p') wProfile = L"ps_6_0";
        else if (targetProfile[0] == 'g') wProfile = L"gs_6_0";
        else if (targetProfile[0] == 'c') wProfile = L"cs_6_0";
        else if (targetProfile[0] == 'h') wProfile = L"hs_6_0";
        else if (targetProfile[0] == 'd') wProfile = L"ds_6_0";
        else wProfile = L"ps_6_0";

        addArg(L"-T");
        addArg(wProfile);

        // Entry point
        std::wstring wEntry;
        if (entryPoint && entryPoint[0])
        {
            int wlen = MultiByteToWideChar(CP_UTF8, 0, entryPoint, -1, nullptr, 0);
            wEntry.resize(wlen);
            MultiByteToWideChar(CP_UTF8, 0, entryPoint, -1, &wEntry[0], wlen);
            if (!wEntry.empty() && wEntry.back() == L'\0') wEntry.pop_back();
        }
        else
        {
            wEntry = L"main";
        }

        addArg(L"-E");
        addArg(wEntry);

        // Include dir
        std::wstring wInclude;
        if (includeDir && includeDir[0])
        {
            int wlen = MultiByteToWideChar(CP_UTF8, 0, includeDir, -1, nullptr, 0);
            wInclude.resize(wlen);
            MultiByteToWideChar(CP_UTF8, 0, includeDir, -1, &wInclude[0], wlen);
            if (!wInclude.empty() && wInclude.back() == L'\0') wInclude.pop_back();

            addArg(L"-I");
            addArg(wInclude);
        }

        // Defines
        xr_vector<std::wstring> wDefines;
        for (const auto& opt : options)
        {
            xr_string defStr = opt.name;
            if (!opt.value.empty())
            {
                defStr += "=";
                defStr += opt.value;
            }

            int wlen = MultiByteToWideChar(CP_UTF8, 0, defStr.c_str(), -1, nullptr, 0);
            std::wstring wDef(wlen, 0);
            MultiByteToWideChar(CP_UTF8, 0, defStr.c_str(), -1, &wDef[0], wlen);
            if (!wDef.empty() && wDef.back() == L'\0') wDef.pop_back();

            wDefines.push_back(wDef);
            addArg(L"-D");
            addArg(wDefines.back());
        }

        xr_vector<LPCWSTR> args;
        args.reserve(argStorage.size());
        for (const auto& a : argStorage)
            args.push_back(a.c_str());

        CEngineDxcIncludeHandler includeHandler(m_pUtils);
        IDxcResult* pResult = nullptr;
        HRESULT hr = m_pCompiler->Compile(&sourceBuffer, args.data(), (UINT32)args.size(), &includeHandler, IID_PPV_ARGS(&pResult));
        if (FAILED(hr) || !pResult)
        {
            outErrors = "DXC Compile call failed";
            if (pResult) pResult->Release();
            return hr;
        }

        HRESULT hrStatus = S_OK;
        pResult->GetStatus(&hrStatus);

        IDxcBlobUtf8* pErrorBlob = nullptr;
        pResult->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(&pErrorBlob), nullptr);
        if (pErrorBlob && pErrorBlob->GetStringLength() > 0)
        {
            outErrors = pErrorBlob->GetStringPointer();
            pErrorBlob->Release();
        }

        if (SUCCEEDED(hrStatus))
        {
            IDxcBlob* pBlob = nullptr;
            pResult->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(&pBlob), nullptr);
            if (pBlob)
            {
                outBytecode.resize(pBlob->GetBufferSize());
                CopyMemory(outBytecode.data(), pBlob->GetBufferPointer(), pBlob->GetBufferSize());
                pBlob->Release();
            }

            // Try to extract reflection from first pass
            IDxcBlob* pReflBlob = nullptr;
            if (SUCCEEDED(pResult->GetOutput(DXC_OUT_REFLECTION, IID_PPV_ARGS(&pReflBlob), nullptr)) && pReflBlob && pReflBlob->GetBufferSize() > 0)
            {
                outReflection.resize(pReflBlob->GetBufferSize());
                CopyMemory(outReflection.data(), pReflBlob->GetBufferPointer(), pReflBlob->GetBufferSize());
                pReflBlob->Release();
            }
            else
            {
                // Second pass without -spirv specifically to get DXC_OUT_REFLECTION
                xr_vector<std::wstring> reflArgStorage;
                auto addRefl = [&](const std::wstring& str)
                {
                    reflArgStorage.push_back(str);
                };

                addRefl(L"-O0");
                addRefl(L"-Zpc");
                addRefl(L"-Wno-conversion");
                addRefl(L"-Wno-switch");
                addRefl(L"-T");
                addRefl(wProfile);
                addRefl(L"-E");
                addRefl(wEntry);
                if (!wInclude.empty())
                {
                    addRefl(L"-I");
                    addRefl(wInclude);
                }
                for (const auto& wd : wDefines)
                {
                    addRefl(L"-D");
                    addRefl(wd);
                }

                xr_vector<LPCWSTR> reflArgs;
                reflArgs.reserve(reflArgStorage.size());
                for (const auto& a : reflArgStorage)
                    reflArgs.push_back(a.c_str());

                IDxcResult* pReflResult = nullptr;
                HRESULT hrRefl = m_pCompiler->Compile(&sourceBuffer, reflArgs.data(), (UINT32)reflArgs.size(), &includeHandler, IID_PPV_ARGS(&pReflResult));
                if (SUCCEEDED(hrRefl) && pReflResult)
                {
                    IDxcBlob* pBlobRefl = nullptr;
                    if (SUCCEEDED(pReflResult->GetOutput(DXC_OUT_REFLECTION, IID_PPV_ARGS(&pBlobRefl), nullptr)) && pBlobRefl && pBlobRefl->GetBufferSize() > 0)
                    {
                        outReflection.resize(pBlobRefl->GetBufferSize());
                        CopyMemory(outReflection.data(), pBlobRefl->GetBufferPointer(), pBlobRefl->GetBufferSize());
                        pBlobRefl->Release();
                    }
                    pReflResult->Release();
                }
            }
        }

        pResult->Release();
        return hrStatus;
    }
};

static CDxcCompilerHelper s_dxcHelper;

template <typename T>
static HRESULT create_shader(DWORD const* spv_buffer, size_t const spv_size,
    const void* refl_buffer, size_t const refl_size,
    LPCSTR const file_name, T*& result)
{
    HRESULT _hr = ShaderTypeTraits<T>::CreateHWShader(spv_buffer, spv_size, result->sh);
    if (!SUCCEEDED(_hr))
    {
        Log("! Shader: ", file_name);
        Msg("! CreateHWShader hr == 0x%08x", _hr);
        return E_FAIL;
    }

    if (!refl_buffer || refl_size == 0)
    {
        Msg("! [Vulkan] Missing reflection data for shader '%s'", file_name);
        return E_FAIL;
    }

    if (!s_dxcHelper.GetUtils())
        s_dxcHelper.Init();

    if (!s_dxcHelper.GetUtils())
    {
        Msg("! [Vulkan] DXC Utils not initialized for shader '%s'", file_name);
        return E_FAIL;
    }

    {
        DxcBuffer dxcBuffer{};
        dxcBuffer.Ptr = refl_buffer;
        dxcBuffer.Size = refl_size;
        dxcBuffer.Encoding = DXC_CP_ACP;

        ID3D12ShaderReflection* pReflection = nullptr;
        _hr = s_dxcHelper.GetUtils()->CreateReflection(&dxcBuffer, IID_PPV_ARGS(&pReflection));
        if (FAILED(_hr) || !pReflection)
        {
            Msg("! [Vulkan] CreateReflection failed hr=0x%08x for %s", _hr, file_name);
            return E_FAIL;
        }
        {
            result->constants.parse(pReflection, ShaderTypeTraits<T>::GetShaderDest());

            // Use SPIRV-Reflect to find the real Vulkan descriptor bindings for UBOs (including $Globals)
            if (spv_buffer && spv_size > 0)
            {
                SpvReflectShaderModule spvModule{};
                SpvReflectResult spvRes = spvReflectCreateShaderModule(spv_size, spv_buffer, &spvModule);
                if (spvRes == SPV_REFLECT_RESULT_SUCCESS)
                {
                    uint32_t bindingCount = 0;
                    spvReflectEnumerateDescriptorBindings(&spvModule, &bindingCount, nullptr);
                    if (bindingCount > 0)
                    {
                        xr_vector<SpvReflectDescriptorBinding*> spvBindings(bindingCount);
                        spvReflectEnumerateDescriptorBindings(&spvModule, &bindingCount, spvBindings.data());
                        for (auto* b : spvBindings)
                        {
                            if (!b) continue;

                            LPCSTR resName = b->name ? b->name : (b->type_description ? b->type_description->type_name : "");

                            if (b->descriptor_type == SPV_REFLECT_DESCRIPTOR_TYPE_UNIFORM_BUFFER)
                            {
                                // Find matching cbSlot in DXIL reflection
                                u32 cbSlot = u32(-1);

                                D3D12_SHADER_INPUT_BIND_DESC bindDesc{};
                                if (SUCCEEDED(pReflection->GetResourceBindingDescByName(resName, &bindDesc)))
                                {
                                    cbSlot = bindDesc.BindPoint;
                                }
                                else if (resName[0] == '$' || strstr(resName, "Globals") != nullptr)
                                {
                                    if (SUCCEEDED(pReflection->GetResourceBindingDescByName("$Globals", &bindDesc)))
                                    {
                                        cbSlot = bindDesc.BindPoint;
                                    }
                                    else
                                    {
                                        cbSlot = 0;
                                    }
                                }

                                if (cbSlot == u32(-1))
                                {
                                    // Fallback: check constant buffer count or assume slot based on binding
                                    cbSlot = (b->binding < 16) ? b->binding : 0;
                                }

                                constexpr u32 stageOffset = std::is_same_v<T, SVS> ? 0 :
                                                            (std::is_same_v<T, SPS> ? 6 :
                                                            (std::is_same_v<T, SGS> ? 12 : 0));
                                u32 storageSlot = cbSlot + stageOffset;
                                if (storageSlot < 16)
                                {
                                    result->constants.m_cb_records.push_back({ storageSlot, b->binding });
                                }
                            }
                            else if (b->descriptor_type == SPV_REFLECT_DESCRIPTOR_TYPE_SAMPLED_IMAGE)
                            {
                                u32 slot = u32(-1);
                                D3D12_SHADER_INPUT_BIND_DESC bindDesc{};
                                if (SUCCEEDED(pReflection->GetResourceBindingDescByName(resName, &bindDesc)))
                                {
                                    slot = bindDesc.BindPoint;
                                }
                                else if (b->binding >= 16 && b->binding < 48)
                                {
                                    slot = b->binding - 16;
                                }

                                if constexpr (std::is_same_v<T, SVS>)
                                {
                                    if (slot < CTexture::mtMaxVertexShaderTextures)
                                        slot += CTexture::rstVertex;
                                }

                                if (b->binding >= 16 && b->binding < 48)
                                {
                                    result->constants.m_tex_records.push_back({ slot, b->binding, RC_dx11texture, resName });
                                }
                            }
                            else if (b->descriptor_type == SPV_REFLECT_DESCRIPTOR_TYPE_SAMPLER)
                            {
                                u32 slot = u32(-1);
                                D3D12_SHADER_INPUT_BIND_DESC bindDesc{};
                                if (SUCCEEDED(pReflection->GetResourceBindingDescByName(resName, &bindDesc)))
                                {
                                    slot = bindDesc.BindPoint;
                                }
                                else if (b->binding >= 48 && b->binding < 64)
                                {
                                    slot = b->binding - 48;
                                }

                                if (b->binding >= 48 && b->binding < 64)
                                {
                                    result->constants.m_smp_records.push_back({ slot, b->binding, RC_sampler, resName });
                                }
                            }
                        }
                    }
                    spvReflectDestroyShaderModule(&spvModule);
                }
            }

            if constexpr (std::is_same_v<T, SVS>)
            {
                D3D12_SHADER_DESC shaderDesc{};
                pReflection->GetDesc(&shaderDesc);
                xr_vector<ShaderInputParam> inputs;
                for (u32 i = 0; i < shaderDesc.InputParameters; ++i)
                {
                    D3D12_SIGNATURE_PARAMETER_DESC paramDesc{};
                    pReflection->GetInputParameterDesc(i, &paramDesc);
                    ShaderInputParam p{};
                    p.semantic = paramDesc.SemanticName;
                    p.index = paramDesc.SemanticIndex;
                    p.location = (paramDesc.Register != (UINT)-1 && paramDesc.Register < 32) ? paramDesc.Register : i;
                    inputs.push_back(p);
                }
                RegisterVSInputs(result->sh, inputs);
            }

            pReflection->Release();
        }
    }

    return S_OK;
}

static HRESULT create_shader(LPCSTR const pTarget, DWORD const* spv_buffer, size_t const spv_size,
    const void* refl_buffer, size_t const refl_size,
    LPCSTR const file_name, void*& result)
{
    if (pTarget[0] == 'p')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SPS*&)result);
    if (pTarget[0] == 'v')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SVS*&)result);
    if (pTarget[0] == 'g')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SGS*&)result);
    if (pTarget[0] == 'c')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SCS*&)result);
    if (pTarget[0] == 'h')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SHS*&)result);
    if (pTarget[0] == 'd')
        return create_shader(spv_buffer, spv_size, refl_buffer, refl_size, file_name, (SDS*&)result);

    NODEFAULT;
    return E_FAIL;
}

HRESULT CRender::shader_compile(pcstr name, IReader* fs, pcstr pFunctionName,
    pcstr pTarget, u32 Flags, void*& result)
{
    xr_vector<ShaderOption> options;
    shader_name_holder sh_name;

    string32 c_smap;
    string32 c_gloss;
    string32 c_sun_shafts;
    string32 c_ssao;
    string32 c_sun_quality;
    char c_msaa_samples[2];

    const auto appendShaderOption = [&](u32 option, cpcstr macro, cpcstr value)
    {
        if (option)
            options.push_back({ macro, value });

        sh_name.append(option);
    };

    // Shadow map size
    xr_itoa(m_SMAPSize, c_smap, 10);
    appendShaderOption(m_SMAPSize, "SMAP_size", c_smap);

    appendShaderOption(o.fp16_filter, "FP16_FILTER", "1");
    appendShaderOption(o.fp16_blend, "FP16_BLEND", "1");
    appendShaderOption(o.HW_smap, "USE_HWSMAP", "1");
    appendShaderOption(o.HW_smap_PCF, "USE_HWSMAP_PCF", "1");
    appendShaderOption(o.HW_smap_FETCH4, "USE_FETCH4", "1");
    appendShaderOption(o.sjitter, "USE_SJITTER", "1");
    appendShaderOption(HW.Caps.raster_major >= 3, "USE_BRANCHING", "1");
    appendShaderOption(HW.Caps.geometry.bVTF, "USE_VTF", "1");
    appendShaderOption(o.Tshadows, "USE_TSHADOWS", "1");
    appendShaderOption(o.mblur, "USE_MBLUR", "1");
    appendShaderOption(o.sunfilter, "USE_SUNFILTER", "1");
    appendShaderOption(o.sunstatic, "USE_R2_STATIC_SUN", "1");

    xr_sprintf(c_gloss, "%f", o.forcegloss_v);
    appendShaderOption(o.forcegloss, "FORCE_GLOSS", c_gloss);

    appendShaderOption(o.forceskinw, "SKIN_COLOR", "1");
    appendShaderOption(o.ssao_blur_on, "USE_SSAO_BLUR", "1");

    // SSAO HDAO / HBAO
    if (o.ssao_hdao)
    {
        options.push_back({ "HDAO", "1" });
        sh_name.append(static_cast<u32>(1)); // HDAO on
        sh_name.append(static_cast<u32>(0)); // HBAO off
        sh_name.append(static_cast<u32>(0)); // Half data off
    }
    else // SSAO HBAO
    {
        sh_name.append(static_cast<u32>(0)); // HDAO off
        sh_name.append(o.ssao_hbao);         // HBAO on/off
        sh_name.append(o.ssao_half_data);    // Half data on/off

        if (o.ssao_hbao)
        {
            if (o.ssao_half_data)
                options.push_back({ "SSAO_OPT_DATA", "2" });
            else
                options.push_back({ "SSAO_OPT_DATA", "1" });

            if (o.hbao_vectorized)
                options.push_back({ "VECTORIZED_CODE", "1" });

            options.push_back({ "USE_HBAO", "1" });
        }
    }

    // skinning
    appendShaderOption(m_skinning < 0, "SKIN_NONE", "1");
    appendShaderOption(0 == m_skinning, "SKIN_0", "1");
    appendShaderOption(1 == m_skinning, "SKIN_1", "1");
    appendShaderOption(2 == m_skinning, "SKIN_2", "1");
    appendShaderOption(3 == m_skinning, "SKIN_3", "1");
    appendShaderOption(4 == m_skinning, "SKIN_4", "1");

    // Soft water
    {
        const bool softWater = RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_SOFT_WATER);
        appendShaderOption(softWater, "USE_SOFT_WATER", "1");
    }

    // Soft particles
    {
        const bool useSoftParticles = RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_SOFT_PARTICLES);
        appendShaderOption(useSoftParticles, "USE_SOFT_PARTICLES", "1");
    }

    // Depth of field
    {
        const bool dof = RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_DOF);
        appendShaderOption(dof, "USE_DOF", "1");
    }

    // Sun shafts
    if (RImplementation.o.advancedpp && ps_r_sun_shafts)
    {
        xr_sprintf(c_sun_shafts, "%d", ps_r_sun_shafts);
        options.push_back({ "SUN_SHAFTS_QUALITY", c_sun_shafts });
        sh_name.append(ps_r_sun_shafts);
    }
    else
        sh_name.append(static_cast<u32>(0));

    // SSAO
    if (RImplementation.o.advancedpp && ps_r_ssao)
    {
        xr_sprintf(c_ssao, "%d", ps_r_ssao);
        options.push_back({ "SSAO_QUALITY", c_ssao });
        sh_name.append(ps_r_ssao);
    }
    else
        sh_name.append(static_cast<u32>(0));

    // Sun quality
    if (RImplementation.o.advancedpp && ps_r_sun_quality)
    {
        xr_sprintf(c_sun_quality, "%d", ps_r_sun_quality);
        options.push_back({ "SUN_QUALITY", c_sun_quality });
        sh_name.append(ps_r_sun_quality);
    }
    else
        sh_name.append(static_cast<u32>(0));

    // Steep parallax
    {
        const bool steepParallax = RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_STEEP_PARALLAX);
        appendShaderOption(steepParallax, "ALLOW_STEEPPARALLAX", "1");
    }

    // Geometry buffer optimization
    appendShaderOption(o.gbuffer_opt, "GBUFFER_OPTIMIZATION", "1");

    // SM5
    appendShaderOption(1, "SM_5", "1");

    // Minmax SM
    appendShaderOption(o.minmax_sm, "USE_MINMAX_SM", "1");

    // MSAA
    if (o.msaa)
    {
        appendShaderOption(o.msaa, "USE_MSAA", "1");

        // Number of samples
        {
            c_msaa_samples[0] = char(o.msaa_samples) + '0';
            c_msaa_samples[1] = 0;
            appendShaderOption(o.msaa_samples, "MSAA_SAMPLES", c_msaa_samples);
        }
        // Current sample
        char c_msaa_current_sample[2];
        {
            if (m_MSAASample < 0 || o.msaa_opt)
                c_msaa_current_sample[0] = '0';
            else
                c_msaa_current_sample[0] = '0' + char(m_MSAASample);
            c_msaa_current_sample[1] = 0;

            appendShaderOption(m_MSAASample >= 0 ? m_MSAASample : 0,
                "ISAMPLE", c_msaa_current_sample);
        }

        appendShaderOption(o.msaa_opt, "MSAA_OPTIMIZATION", "1");

        switch (o.msaa_alphatest)
        {
        case MSAA_ATEST_DX10_0_ATOC:
            options.push_back({ "MSAA_ALPHATEST_DX10_0_ATOC", "1" });

            sh_name.append(static_cast<u32>(1)); // DX10_0_ATOC   on
            sh_name.append(static_cast<u32>(0)); // DX10_1_ATOC   off
            sh_name.append(static_cast<u32>(0)); // DX10_1_NATIVE off
            break;
        case MSAA_ATEST_DX10_1_ATOC:
            options.push_back({ "MSAA_ALPHATEST_DX10_1_ATOC", "1" });

            sh_name.append(static_cast<u32>(0)); // DX10_0_ATOC   off
            sh_name.append(static_cast<u32>(1)); // DX10_1_ATOC   on
            sh_name.append(static_cast<u32>(0)); // DX10_1_NATIVE off
            break;
        case MSAA_ATEST_DX10_1_NATIVE:
            options.push_back({ "MSAA_ALPHATEST_DX10_1", "1" });

            sh_name.append(static_cast<u32>(0)); // DX10_0_ATOC   off
            sh_name.append(static_cast<u32>(0)); // DX10_1_ATOC   off
            sh_name.append(static_cast<u32>(1)); // DX10_1_NATIVE on
            break;
        default:
            sh_name.append(static_cast<u32>(0)); // DX10_0_ATOC   off
            sh_name.append(static_cast<u32>(0)); // DX10_1_ATOC   off
            sh_name.append(static_cast<u32>(0)); // DX10_1_NATIVE off
        }
    }
    else
    {
        sh_name.append(static_cast<u32>(0)); // MSAA off
        sh_name.append(static_cast<u32>(0)); // No MSAA samples
    }

    sh_name.finish();

    char extension[3];
    strncpy_s(extension, pTarget, 2);

    pcstr renderer = "r4_vk_v7" DELIMITER;

    string_path filename;
    strconcat(sizeof(filename), filename, renderer, name, ".", extension);

    string_path file_name;
    {
        string_path file;
        strconcat(sizeof(file), file, "shaders_cache_oxr" DELIMITER, filename, DELIMITER, sh_name.c_str());
        strconcat(sizeof(filename), filename, filename, DELIMITER, sh_name.c_str());
        FS.update_path(file_name, "$app_data_root$", file);
    }

    string_path shadersFolder;
    FS.update_path(shadersFolder, "$game_shaders$", RImplementation.getShaderPath());

    u32 fileCrc = 0;
    getFileCrc32(fs, shadersFolder, fileCrc);
    fs->seek(0);

    xr_string optionsKey;
    optionsKey += pTarget ? pTarget : "";
    optionsKey += "|";
    optionsKey += pFunctionName ? pFunctionName : "";
    for (const auto& opt : options)
    {
        optionsKey += "|";
        optionsKey += opt.name;
        optionsKey += "=";
        optionsKey += opt.value;
    }
    const u32 optionsCrc = crc32(optionsKey.data(), (u32)optionsKey.size());

    constexpr u32 CACHE_MAGIC = 0x43534B56; // 'VKSC'
    constexpr u32 CACHE_VERSION = 2;

    struct VkShaderCacheHeader
    {
        u32 magic{ CACHE_MAGIC };
        u32 version{ CACHE_VERSION };
        u32 fileCrc{ 0 };
        u32 optionsCrc{ 0 };
        u32 bytecodeCrc{ 0 };
        u32 bytecodeSize{ 0 };
        u32 reflectionCrc{ 0 };
        u32 reflectionSize{ 0 };
    };

    HRESULT _result = E_FAIL;

    if (FS.exist(file_name))
    {
        IReader* file = FS.r_open(file_name);
        if (file && file->length() >= (int)sizeof(VkShaderCacheHeader))
        {
            VkShaderCacheHeader hdr{};
            file->r(&hdr, sizeof(hdr));
            if (hdr.magic == CACHE_MAGIC &&
                hdr.version == CACHE_VERSION &&
                hdr.fileCrc == fileCrc &&
                hdr.optionsCrc == optionsCrc &&
                hdr.bytecodeSize > 0 &&
                hdr.reflectionSize > 0 &&
                (u32)file->elapsed() == hdr.bytecodeSize + hdr.reflectionSize)
            {
                xr_vector<u8> bytecode(hdr.bytecodeSize);
                file->r(bytecode.data(), hdr.bytecodeSize);
                xr_vector<u8> reflection(hdr.reflectionSize);
                file->r(reflection.data(), hdr.reflectionSize);

                if (crc32(bytecode.data(), hdr.bytecodeSize) == hdr.bytecodeCrc &&
                    crc32(reflection.data(), hdr.reflectionSize) == hdr.reflectionCrc)
                {
                    _result = create_shader(pTarget,
                        (DWORD*)bytecode.data(), bytecode.size(),
                        reflection.data(), reflection.size(),
                        filename, result);
                }
            }
        }
        if (file)
            file->close();
    }

    if (FAILED(_result))
    {
        xr_vector<u8> bytecode;
        xr_vector<u8> reflection;
        xr_string errors;
        _result = s_dxcHelper.Compile(fs->pointer(), fs->length(), pFunctionName, pTarget, options, shadersFolder, bytecode, reflection, errors);

        if (SUCCEEDED(_result) && !bytecode.empty() && !reflection.empty())
        {
            _result = create_shader(pTarget,
                (DWORD*)bytecode.data(), bytecode.size(),
                reflection.data(), reflection.size(),
                filename, result);

            if (SUCCEEDED(_result))
            {
                IWriter* file = FS.w_open(file_name);
                if (file)
                {
                    VkShaderCacheHeader hdr{};
                    hdr.fileCrc = fileCrc;
                    hdr.optionsCrc = optionsCrc;
                    hdr.bytecodeCrc = crc32(bytecode.data(), (u32)bytecode.size());
                    hdr.bytecodeSize = (u32)bytecode.size();
                    hdr.reflectionCrc = crc32(reflection.data(), (u32)reflection.size());
                    hdr.reflectionSize = (u32)reflection.size();

                    file->w(&hdr, sizeof(hdr));
                    file->w(bytecode.data(), hdr.bytecodeSize);
                    file->w(reflection.data(), hdr.reflectionSize);
                    FS.w_close(file);
                }
            }
        }
        else
        {
            _result = E_FAIL;
            if (errors.length() > 500)
            {
                Msg("! [Vulkan] Failed to compile shader '%s' (%s): (tail) %s",
                    filename, pTarget, errors.c_str() + (errors.length() - 500));
            }
            else
            {
                Msg("! [Vulkan] Failed to compile shader '%s' (%s): %s", filename, pTarget, errors.c_str());
            }
        }
    }

    return _result;
}
} // namespace xray::render::RENDER_NAMESPACE
