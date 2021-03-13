/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hdmi/regs-vwrapper.h
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

#ifndef __REGS_VIDEO_WRAPPER_H
#define __REGS_VIDEO_WRAPPER_H

#define HDMIDP_VIDEO_WRAPPER_REG(x)    (0x00A50000 + x)

//@{
/**
 * @name Video Wrapper config registers
 */
#define VIDEO_WRAPPER_SYNC_MODE         HDMIDP_VIDEO_WRAPPER_REG(0x0000)
#define VIDEO_WRAPPER_HV_LINE_0         HDMIDP_VIDEO_WRAPPER_REG(0x0004)
#define VIDEO_WRAPPER_HV_LINE_1         HDMIDP_VIDEO_WRAPPER_REG(0x0008)
#define VIDEO_WRAPPER_VSYNC_GEN         HDMIDP_VIDEO_WRAPPER_REG(0x000C)
#define VIDEO_WRAPPER_HSYNC_GEN         HDMIDP_VIDEO_WRAPPER_REG(0x0010)
#define VIDEO_WRAPPER_H_BLANK           HDMIDP_VIDEO_WRAPPER_REG(0x0014)
#define VIDEO_WRAPPER_V_PATTERN         HDMIDP_VIDEO_WRAPPER_REG(0x0018)
#define VIDEO_WRAPPER_SYS_EN            HDMIDP_VIDEO_WRAPPER_REG(0x001C)
#define VIDEO_WRAPPER_VCLK_SEL		HDMIDP_VIDEO_WRAPPER_REG(0x0020)
#define VIDEO_WRAPPER_3D_SEL		HDMIDP_VIDEO_WRAPPER_REG(0x0030)
//@}

#define V_PATTERN_PASS_TO_CORE		0x1
#define VCLK_SEL_TO_EXTERNAL		0x0
#define VCLK_SEL_TO_INTERNAL		0x1
#define WRAPPER_3D_ENABLE		0x1
#define WRAPPER_2D_ENABLE		0x0


#endif /* __REGS_VIDEO_WRAPPER_H */
