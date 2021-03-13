/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/
#ifndef TCC_HDIN_CTRL_H
#define TCC_HDIN_CTRL_H

#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR_EP9553T)
#define USING_HW_I2C
#define SENSOR_EP9553T
#define HDMIIN_FEATURE_YUV422_MODE
#include "module/ep9553t.h"
#endif

extern void hdin_clock_get(struct tcc_hdin_device *hdev);
extern void hdin_clock_put(struct tcc_hdin_device *hdev);
extern void hdin_clock_enable(struct tcc_hdin_device *hdev);
extern void hdin_clock_disable(struct tcc_hdin_device *hdev);
extern int hdin_ctrl_get_resolution(struct tcc_hdin_device *hdev);
extern int hdin_ctrl_get_fps(int *nFrameRate);
extern int hdin_ctrl_get_audio_samplerate(struct tcc_hdin_device *hdev, int value);
extern int hdin_ctrl_get_audio_type(struct tcc_hdin_device *hdev, int value);
extern int hdin_ctrl_init(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_get_gpio(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_pwr_enable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_pwr_disable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_key_enable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_key_disable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_rst_enable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_rst_disable(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_cleanup(struct tcc_hdin_device *hdev);
extern void hdin_ctrl_delay(int ms);
#ifdef HDIN_DRV_BYPASS_EN
extern void hdin_ctrl_set_bypass_mode(struct tcc_hdin_device *hdev, int *onoff);
#endif
#endif
