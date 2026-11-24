#include "stdafx.h"

namespace xray::render::RENDER_NAMESPACE
{
void CRenderTarget::u_setrt(CBackend& cmd_list, const ref_rt& _1, const ref_rt& _2, const ref_rt& _3, const ref_rt& _zb)
{
    dwWidth[cmd_list.context_id] = 0;
    dwHeight[cmd_list.context_id] = 0;

    if (_1)
    {
        dwWidth[cmd_list.context_id] = _1->dwWidth;
        dwHeight[cmd_list.context_id] = _1->dwHeight;
        cmd_list.set_RT(_1->pRT, 0);
    }
    else
    {
        cmd_list.set_RT(VK_NULL_HANDLE, 0);
    }

    if (_2)
    {
        if (!dwWidth[cmd_list.context_id])
        {
            dwWidth[cmd_list.context_id] = _2->dwWidth;
            dwHeight[cmd_list.context_id] = _2->dwHeight;
        }
        cmd_list.set_RT(_2->pRT, 1);
    }
    else
    {
        cmd_list.set_RT(VK_NULL_HANDLE, 1);
    }

    if (_3)
    {
        if (!dwWidth[cmd_list.context_id])
        {
            dwWidth[cmd_list.context_id] = _3->dwWidth;
            dwHeight[cmd_list.context_id] = _3->dwHeight;
        }
        cmd_list.set_RT(_3->pRT, 2);
    }
    else
    {
        cmd_list.set_RT(VK_NULL_HANDLE, 2);
    }

    if (_zb)
    {
        if (!dwWidth[cmd_list.context_id])
        {
            dwWidth[cmd_list.context_id] = _zb->dwWidth;
            dwHeight[cmd_list.context_id] = _zb->dwHeight;
        }
        cmd_list.set_ZB(_zb->pZRT);
    }
    else
    {
        cmd_list.set_ZB(VK_NULL_HANDLE);
    }

    cmd_list.curr_rt_width = dwWidth[cmd_list.context_id];
    cmd_list.curr_rt_height = dwHeight[cmd_list.context_id];
}

void CRenderTarget::u_setrt(CBackend& cmd_list, const ref_rt& _1, const ref_rt& _2, const ref_rt& _zb)
{
    u_setrt(cmd_list, _1, _2, nullptr, _zb);
}

void CRenderTarget::u_setrt(CBackend& cmd_list, u32 W, u32 H, VkImageView _1, VkImageView _2, VkImageView _3, VkImageView zb)
{
    dwWidth[cmd_list.context_id] = W;
    dwHeight[cmd_list.context_id] = H;
    cmd_list.curr_rt_width = W;
    cmd_list.curr_rt_height = H;

    cmd_list.set_RT(_1, 0);
    cmd_list.set_RT(_2, 1);
    cmd_list.set_RT(_3, 2);
    cmd_list.set_ZB(zb);
}
} // namespace xray::render::RENDER_NAMESPACE
