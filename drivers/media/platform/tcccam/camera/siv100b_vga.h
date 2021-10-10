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

#include "../sensor_if.h"

#ifndef SIV100B_H
#define SIV100B_H

#define SIV100B_REG_TERM 				0xff	/* terminating list entry for reg */
#define SIV100B_VAL_TERM 				0xff	/* terminating list entry for val */

#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)
#define FRONT_MODULE_NODE "siv100b_vga"
#define FRONT_CAM_I2C_NAME "tcc_camera,0x66"
#else
#define MODULE_NODE "siv100b_vga"
#define CAM_I2C_NAME "tcc_camera,0x66"
#endif

extern void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info);
extern void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func);
#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
extern void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info);
extern void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func);
#endif


#endif /* SIV100B_H */

