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

#ifndef _TCC_REAR_CAMERA_INTERFACE_H_
#define _TCC_REAR_CAMERA_INTERFACE_H_
#if defined(CONFIG_ARCH_TCC898X)
#include <video/tcc/tcc_cam_ioctrl.h>
#else
#include <mach/tcc_cam_ioctrl.h>
#endif

typedef enum {
	INIT_MODE	= 0,
	PREIVEW_MODE,
	CAPTURE_MODE,
	CLOSE_MODE,
} TCC_REAR_CAMERA_SENSOR_MODE;

typedef struct {
	int	camera_type;	// AVM, SVM, LVDS, ...
	int	camera_encode;	// NTSC, PAL, ...
	int	preview_x;
	int	preview_y;
	int	preview_width;
	int	preview_height;
	int	handover;
} DIRECT_DISPLAY_IF_PARAMETERS;

struct tcc_camera_device;

extern int direct_display_start_monitor(struct tcc_camera_device * vdev);
extern int direct_display_stop_monitor(struct tcc_camera_device * vdev);

extern int direct_display_if_initialize(struct tcc_camera_device * vdev);
extern int direct_display_if_start(DIRECT_DISPLAY_IF_PARAMETERS params, struct tcc_camera_device * vdev);
extern int direct_display_if_stop(struct tcc_camera_device * vdev);
extern int direct_display_if_terminate(void);

#if defined(CONFIG_TCC_REAR_CAMERA_DRV)
extern void rcam_get_line_buf_addr_by_index(struct tcc_camera_device * vdev, RCAM_LINE_BUFFER_INFO * rcam_line_buffer_info);
extern void rcam_line_buf_update(struct tcc_camera_device * vdev, RCAM_LINE_BUFFER_UPDATE_INFO * pInfo);
extern void rcam_parse_vioc_dt_data(struct tcc_camera_device * vdev);
extern int rcam_process_pmap(struct tcc_camera_device * vdev);
#endif//defined(CONFIG_TCC_REAR_CAMERA_DRV)

#endif // _TCC_REAR_CAMERA_INTERFACE_H_


