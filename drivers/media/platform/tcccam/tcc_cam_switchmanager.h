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

#ifndef __TCC_CAM_SWITCHMANAGER_H__
#define __TCC_CAM_SWITCHMANAGER_H__ 

#include "tcc_camera_device.h"

#define DEV_NAME "Telechips-Camera-SwitchManager"

extern int tcc_cam_switchmanager_show_info(struct tcc_camera_device * vdev);
extern int tcc_cam_switchmanager_start_preview(struct tcc_camera_device * vdev);
extern int tcc_cam_switchmanager_stop_preview(struct tcc_camera_device * vdev);
extern int tcc_cam_swtichmanager_start_monitor(struct tcc_camera_device * vdev);
extern int tcc_cam_switchmanager_stop_monitor(struct tcc_camera_device * vdev);
extern int tcc_cam_switchmanager_handover_handler(struct tcc_camera_device * vdev);
extern int tcc_cam_switchmanager_probe(struct tcc_camera_device * vdev);

#endif//__TCC_CAM_SWITCHMANAGER_H__

