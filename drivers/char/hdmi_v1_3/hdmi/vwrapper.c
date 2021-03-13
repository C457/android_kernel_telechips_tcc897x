/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hdmi/vwrapper.c
 * Description : hdmi driver
 *
 * Copyright (C) 2013 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 * ****************************************************************************/

#include <asm/io.h>

#include "regs-vwrapper.h"
#include "vwrapper.h"

void video_wrapper_enable(unsigned char enable)
{
    if (enable)
        writeb(0x01,VIDEO_WRAPPER_SYS_EN);
    else
        writeb(0x00,VIDEO_WRAPPER_SYS_EN);
}

void video_wrapper_set_mode(struct device_video_params param)
{
    writel(param.HBlank, VIDEO_WRAPPER_H_BLANK);

    writel((param.HVLine&0x00000FFF), VIDEO_WRAPPER_HV_LINE_0);
    writel((param.HVLine&0x00FFF000)>>12, VIDEO_WRAPPER_HV_LINE_1);

    writel((param.HSYNCGEN&0x000001FF), VIDEO_WRAPPER_HSYNC_GEN);
    writel((param.HSYNCGEN>>20), VIDEO_WRAPPER_SYNC_MODE);

    writel((param.VSYNCGEN&0x00FFF000)>>12, VIDEO_WRAPPER_VSYNC_GEN);

    writel(0x1,VIDEO_WRAPPER_V_PATTERN);
}
