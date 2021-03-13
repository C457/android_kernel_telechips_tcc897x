/*
 * drivers/media/video/tcccam/atv/tw9912.h
 *
 * Author: khcho (khcho@telechips.com)
 *
 * Copyright (C) 2008 Telechips, Inc.
 * Copyright (C) 2013 Cleinsoft, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 */

#ifndef __TW9990_H__
#define __TW9990_H__

enum TW9990_MODE {
	MODE_TW9990_INIT = 0,
	MODE_TW9990_OPEN,
	MODE_TW9990_CLOSE,
};

#ifdef CONFIG_DAUDIO //mhjung
#define MODULE_NODE "tw9990_atv"
#define CAM_I2C_NAME "tcc_camera,0x88"
#endif
#define TW9990_REG_ID		0x00
#define TW9990_REG_CSTATUS	0x01

#define TW9990_I2C_BRIGHTNESS		0x10
#define TW9990_I2C_CONTRAST			0x11
#define TW9990_I2C_HUE				0x15
#define TW9990_I2C_SATURATION_U		0x13
#define TW9990_I2C_SATURATION_V		0x14
#define TW9990_I2C_SHARPNESS		0x12 //GT system SHARPNESS CONTROL REGISTER I 

#ifdef CONFIG_DAUDIO //mhjung

enum rear_camera_type {
	CAM_TYPE_DEFAULT	= 0,
	CAM_TYPE_CVBS		= 0,
	CAM_TYPE_SVIDEO,
	CAM_TYPE_COMPONENT,
	CAM_TYPE_AUX,
	CAM_TYPE_CMMB,
	CAM_TYPE_LVDS,
	CAM_TYPE_MAX
};

enum rear_camera_enc {
	CAM_ENC_DEFAULT		= 0,
	CAM_ENC_NTSC		= 0,
	CAM_ENC_PAL,
	CAM_ENC_MAX
};

extern void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info);
extern void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func);
#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
extern void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info);
extern void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func);
#endif

extern void sensor_info_init(TCC_SENSOR_INFO_TYPE *sensor_info);
#endif 

#if defined(CONFIG_VIDEO_ATV_SENSOR)
extern struct sensor_reg sensor_initialize[];
#endif

#endif
