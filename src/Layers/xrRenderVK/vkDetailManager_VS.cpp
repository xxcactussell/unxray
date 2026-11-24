#include "stdafx.h"
#include "Layers/xrRender/DetailManager.h"
#include "xrEngine/IGame_Persistent.h"
#include "xrEngine/Environment.h"
#include "Layers/xrRender/BufferUtils.h"

namespace xray::render::RENDER_NAMESPACE
{
namespace detail_manager
{
extern const int quant;
}

void CDetailManager::hw_Load_Shaders()
{
    ref_shader S;
    S.create("details" DELIMITER "set");
    if (S && S->E[0] && !S->E[0]->passes.empty() && S->E[0]->passes[0]->constants)
    {
        R_constant_table& T0 = *S->E[0]->passes[0]->constants;
        hwc_consts = T0.get("consts");
        hwc_wave = T0.get("wave");
        hwc_wind = T0.get("dir2D");
        hwc_array = T0.get("array");
    }
    if (S && S->E[1] && !S->E[1]->passes.empty() && S->E[1]->passes[0]->constants)
    {
        R_constant_table& T1 = *S->E[1]->passes[0]->constants;
        hwc_s_consts = T1.get("consts");
        hwc_s_xform = T1.get("xform");
        hwc_s_array = T1.get("array");
    }
}

void CDetailManager::hw_Render(CBackend& cmd_list)
{
    using namespace detail_manager;

    float fDelta = Device.fTimeGlobal - m_global_time_old;
    if (fDelta < 0 || fDelta > 1)
        fDelta = 0.03f;
    m_global_time_old = Device.fTimeGlobal;

    m_time_rot_1 += PI_MUL_2 * fDelta / swing_current.rot1;
    m_time_rot_2 += PI_MUL_2 * fDelta / swing_current.rot2;
    m_time_pos += fDelta * swing_current.speed;

    float tm_rot1 = m_time_rot_1;
    float tm_rot2 = m_time_rot_2;

    Fvector4 dir1, dir2;
    dir1.set(_sin(tm_rot1), 0, _cos(tm_rot1), 0).normalize().mul(swing_current.amp1);
    dir2.set(_sin(tm_rot2), 0, _cos(tm_rot2), 0).normalize().mul(swing_current.amp2);

    cmd_list.set_Geometry(hw_Geom);

    float scale = 1.f / float(quant);
    Fvector4 wave;
    Fvector4 consts;
    consts.set(scale, scale, ps_r__Detail_l_aniso, ps_r__Detail_l_ambient);
    wave.set(1.f / 5.f, 1.f / 7.f, 1.f / 3.f, m_time_pos);
    hw_Render_dump(cmd_list, consts, wave.div(PI_MUL_2), dir1, 1, 0);

    wave.set(1.f / 3.f, 1.f / 7.f, 1.f / 5.f, m_time_pos);
    hw_Render_dump(cmd_list, consts, wave.div(PI_MUL_2), dir2, 2, 0);

    consts.set(scale, scale, scale, 1.f);
    hw_Render_dump(cmd_list, consts, wave.div(PI_MUL_2), dir2, 0, 1);
}

void CDetailManager::hw_Render_dump(CBackend& cmd_list, const Fvector4& consts, const Fvector4& wave, const Fvector4& wind, u32 var_id, u32 lod_id)
{
    static shared_str strConsts("consts");
    static shared_str strWave("wave");
    static shared_str strDir2D("dir2D");
    static shared_str strArray("array");
    static shared_str strXForm("xform");

    RImplementation.BasicStats.DetailCount = 0;

    u32 vOffset = 0;
    u32 iOffset = 0;

    vis_list& list = m_visibles[var_id];

    for (u32 O = 0; O < objects.size(); O++)
    {
        CDetail& Object = *objects[O];
        xr_vector<SlotItemVec*>& vis = list[O];
        if (!vis.empty())
        {
            for (u32 iPass = 0; iPass < Object.shader->E[lod_id]->passes.size(); ++iPass)
            {
                cmd_list.set_Element(Object.shader->E[lod_id], iPass);
                cmd_list.apply_lmaterial();

                cmd_list.set_c(strConsts, consts);
                cmd_list.set_c(strWave, wave);
                cmd_list.set_c(strDir2D, wind);
                cmd_list.set_c(strXForm, Device.mFullTransform);

                ref_constant constArray = cmd_list.get_c(strArray);
                VERIFY(constArray);

                u32 dwBatch = 0;

                for (auto items : vis)
                {
                    for (auto& instance : *items)
                    {
                        u32 base = dwBatch * 4;

                        float scale = instance->scale_calculated;
                        float sn = _sin(instance->yaw_angle);
                        float cs = _cos(instance->yaw_angle);
                        cmd_list.set_ca(&*constArray, base + 0, cs * scale, 0, sn * scale, instance->pos.x);
                        cmd_list.set_ca(&*constArray, base + 1, 0, scale, 0, instance->pos.y);
                        cmd_list.set_ca(&*constArray, base + 2, -sn * scale, 0, cs * scale, instance->pos.z);

                        float h = instance->c_hemi;
                        float s = instance->c_sun;
                        cmd_list.set_ca(&*constArray, base + 3, s, s, s, h);
                        dwBatch++;
                        if (dwBatch == hw_BatchSize)
                        {
                            RImplementation.BasicStats.DetailCount += dwBatch;
                            u32 dwCNT_verts = dwBatch * Object.number_vertices;
                            u32 dwCNT_prims = dwBatch * Object.number_indices / 3;
                            cmd_list.Render(D3DPT_TRIANGLELIST, vOffset, 0, dwCNT_verts, iOffset, dwCNT_prims);
                            cmd_list.stat.r.s_details.add(dwCNT_verts);
                            dwBatch = 0;
                        }
                    }
                }
                if (dwBatch)
                {
                    RImplementation.BasicStats.DetailCount += dwBatch;
                    u32 dwCNT_verts = dwBatch * Object.number_vertices;
                    u32 dwCNT_prims = dwBatch * Object.number_indices / 3;
                    cmd_list.Render(D3DPT_TRIANGLELIST, vOffset, 0, dwCNT_verts, iOffset, dwCNT_prims);
                    cmd_list.stat.r.s_details.add(dwCNT_verts);
                }
            }
        }
        vOffset += hw_BatchSize * Object.number_vertices;
        iOffset += hw_BatchSize * Object.number_indices;
    }
}
} // namespace xray::render::RENDER_NAMESPACE
