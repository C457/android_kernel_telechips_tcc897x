/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hdmi/vwrapper.h
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

#ifndef _LINUX_VIDEO_WRAPPER_H_
#define _LINUX_VIDEO_WRAPPER_H_

#include <mach/hdmi_1_3_hdmi.h>

void video_wrapper_set_mode(struct device_video_params param);
void video_wrapper_enable(unsigned char enable);

#endif /* _LINUX_CEC_H_ */
