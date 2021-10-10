/*
 * drivers/media/video/tcccam/atv/LVDS.h
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

#ifndef __LVDS_H__
#define __LVDS_H__

enum LVDS_MODE {
	MODE_LVDS_INIT = 0,
	MODE_LVDS_OPEN,
	MODE_LVDS_CLOSE,
};

#define LVDS_MODULE_NODE "max96706_lvds"
#define LVDS_CAM_I2C_NAME "tcc_camera,0x94"


#define LVDS_REG_ID			0x01
#define LVDS_REG_RESET		0x01
#define LVDS_REG_DESLOCK	0x04

#define LVDS_VAL_ID			0x94
#define LVDS_VAL_RESET		0x01
#define LVDS_VAL_DESLOCK	0x80

extern void lvds_sensor_info_init(TCC_SENSOR_INFO_TYPE * sensor_info);
#endif
