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
 
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#if defined(CONFIG_ARCH_TCC898X)
#include <video/tcc/tcc_cam_ioctrl.h>
#else
#include <mach/tcc_cam_ioctrl.h>
#endif
#include "camera_core.h"
#include "cam_reg.h"
#include "cam_clock.h"
#include "sensor_if.h"
#include "tcc_cam.h"
#include "tcc_camera_device.h"
#include "tcc_cam_direct_display_if.h"
#include "tcc_cam_switchmanager.h"

#include "../../../char/tcc_early_view_control/tcc_cam_cm_control.h"
#include "../../../char/tcc_early_view_control/tcc_avn_proc.h"

#include <linux/interrupt.h>

#include <video/tcc/viocmg.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR  "Telechips.Co.Ltd"
#define DRIVER_DESC    "TCC Camera SwitchManager Driver"

static int debug	= 0;
#define TAG		"tcc_cam_switchmanager"
#define dprintk(msg...)	if(debug) { printk(TAG ": " msg); }
#define log(msg...)	if(debug) { printk(TAG ": %s - ", __func__); printk(msg); }
#define FUNCTION_IN	log("In\n");
#define FUNCTION_OUT	log("Out\n");

extern unsigned int do_hibernation;
extern unsigned int do_hibernate_boot;

int tcc_cam_switchmanager_show_info(struct tcc_camera_device * vdev) {
	log("In ----------------------------------------\n");
	log("do_hibernation: %d, do_hibernate_boot: %d\n\n", do_hibernation, do_hibernate_boot);
	
	log("vdev				: 0x%08x\n", (unsigned int)vdev);
	log("vdev->gParams.camera_encode	: %d\n", vdev->gParams.camera_encode);
	log("vdev->gParams.camera_type		: %d\n", vdev->gParams.camera_type);
	log("vdev->gParams.preview_x		: %d\n", vdev->gParams.preview_x);
	log("vdev->gParams.preview_y		: %d\n", vdev->gParams.preview_y);
	log("vdev->gParams.preview_width	: %d\n", vdev->gParams.preview_width);
	log("vdev->gParams.preview_height	: %d\n", vdev->gParams.preview_height);
	log("vdev->gParams.handover		: %d\n", vdev->gParams.handover);
	log("vdev->cam_streaming		: %d\n", vdev->cam_streaming);
	log("vdev->preview_method		: %d\n", vdev->preview_method);
	log("Out ----------------------------------------\n\n");
	return 0;
}

int tcc_cam_switchmanager_parse_device_tree(struct tcc_camera_device * vdev) {
	struct device_node * np = NULL;
	
	if(vdev->camera_np == NULL) {
		printk(KERN_ERR "camera node is NULL\n");
		return -ENODEV;
	}
	
	np = of_parse_phandle(vdev->camera_np, "rear_cam_misc", 0);
	if(np == NULL) {
		printk(KERN_ERR "count not find rear_cam_misc\r\n");
		return -ENODEV;
	}
	return 0;
}

int tcc_cam_switchmanager_start_preview(struct tcc_camera_device * vdev) {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	log("cam_streaming = %d, preview_method = %d\n", vdev->cam_streaming, vdev->preview_method);
	if(0 < vdev->cam_streaming) {
		if(vdev->preview_method == PREVIEW_V4L2) {
			vdev->need_to_backup = 1;
			
			// stop V4L2 camera
			tcc_videobuf_streamoff(&type, vdev);
		} else if(vdev->preview_method == PREVIEW_DD) {
			log("Camera is streaming by Direct display\n");
			return 0;
		}
	}
	
	// set preview method
	vdev->preview_method = PREVIEW_DD;
	
	viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_PREPARE);
	
	// start preview
	tcc_get_sensor_info(vdev, 0);
	if(!vdev->camera_np) {
		printk(KERN_ERR CAM_NAME ": vdev->camera_np is NULL\n");
		return -1;
	}
	if(sensor_if_parse_gpio_dt_data(vdev) < 0) {
		printk(KERN_ERR CAM_NAME ": cannot initialize sensor\n");
		return -EINVAL;
	}
	direct_display_if_initialize(vdev);
	direct_display_if_start(*(DIRECT_DISPLAY_IF_PARAMETERS *)&vdev->gParams, vdev);
	
	viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
	
	log("cam_streaming = %d, preview_method = %d\n", vdev->cam_streaming, vdev->preview_method);
	return 0;
}

int tcc_cam_switchmanager_stop_preview(struct tcc_camera_device * vdev) {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int	idxBuf = 0, nBuf = TCC_CAMERA_MIN_BUFNBRS;
	
	log("cam_streaming = %d, preview_method = %d\n", vdev->cam_streaming, vdev->preview_method);
	if((0 < vdev->cam_streaming) && (vdev->preview_method == PREVIEW_DD)) {
		// stop preview
		direct_display_if_stop(vdev);
		direct_display_if_terminate();
		
		viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_STOP);
	}
	
	if(vdev->need_to_backup == 1) {
		vdev->need_to_backup = 0;
		
		// init and open decoder
		if(sensor_if_init(vdev) < 0) {
			printk(KERN_ERR CAM_NAME ": cannot initialize sensor\n");
			return -1;
		}
		
		// set format
		tcc_videobuf_s_fmt(&(vdev->backup_format), vdev);
		
		// reset all buffer lists
		vdev->data.done_list.prev	= vdev->data.done_list.next	= &(vdev->data.done_list);
		vdev->data.list.prev		= vdev->data.list.next		= &(vdev->data.list);
		
		// queue all buffers
		for(idxBuf=0; idxBuf<nBuf; idxBuf++)
			tcc_videobuf_qbuf(&(vdev->data.static_buf[idxBuf].v4lbuf), vdev);
		
		// start V4L2 camera
		tcc_videobuf_streamon(&type, vdev);
	}
	log("cam_streaming = %d, preview_method = %d\n", vdev->cam_streaming, vdev->preview_method);
	return 0;
}

int tcc_cam_switchmanager_monitor_thread(void * data) {
	struct tcc_camera_device * vdev = (struct tcc_camera_device *)data;
	int gear_status = 0;
	
	FUNCTION_IN
	
	// switching
	while(1) {
		msleep(200);
		
		if(kthread_should_stop())
			break;
		
		mutex_lock(&vdev->tcc_cam_switchmanager_lock);
		
		if(((do_hibernation == 0) && (do_hibernate_boot == 0))	\
			|| ((do_hibernation == 1) && (do_hibernate_boot == 1))) {
			gear_status = tcc_cm_ctrl_get_gear_status();
			if(gear_status) {
				vdev->is_running_by_gear = 1;
				tcc_cam_switchmanager_start_preview(vdev);
			} else {
				if(vdev->is_running_by_gear)
					tcc_cam_switchmanager_stop_preview(vdev);
			}
		}
		
		mutex_unlock(&vdev->tcc_cam_switchmanager_lock);
	}
	
	FUNCTION_OUT
	return 0;
}

int tcc_cam_swtichmanager_start_monitor(struct tcc_camera_device * vdev) {
	if(vdev->threadSwitching != NULL) {
		printk(KERN_ERR "%s - FAILED: thread(0x%x) is not null\n", __func__, (unsigned)vdev->threadSwitching);
		return -1;
	} else {
		vdev->threadSwitching = kthread_run(tcc_cam_switchmanager_monitor_thread, (void *)vdev, "threadSwitching");
		if(IS_ERR_OR_NULL(vdev->threadSwitching)) {
			printk("%s - FAILED: kthread_run\n", __func__);
			vdev->threadSwitching = NULL;
			return -1;
		}
	}
	return 0;
}

int tcc_cam_switchmanager_stop_monitor(struct tcc_camera_device * vdev) {
	if(vdev->threadSwitching == NULL) {
		printk(KERN_ERR "%s - FAILED: thread(0x%x) is null\n", __func__, (unsigned)vdev->threadSwitching);
		return -1;
	} else {
		if(kthread_stop(vdev->threadSwitching) != 0) {
			printk("%s - FAILED: kthread_stop\n", __func__);
			return -1;
		}
		vdev->threadSwitching = NULL;
	}
	return 0;
}

int tcc_cam_switchmanager_handover_handler(struct tcc_camera_device * vdev) {
	int gear_status = 0;

	FUNCTION_IN
	
	// Stop monitor thread.
	if(direct_display_stop_monitor(vdev) < 0) {
		printk("FAILED direct_display_start_monitor\n");
		return -1;
	}
	
	// reset params for handover
	vdev->cam_streaming	= 0;
	vdev->preview_method	= 0;
	vdev->gParams.handover	= 0;
	
	// show info
	if(debug)
		tcc_cam_switchmanager_show_info(vdev);
	
	// cm clock on
	tcc_cm_ctrl_on();
	
	// All data in the context before making snapshot image is not restored yet during snapshot resume.
	// So, it will fail if you access the gpio by calling tcc_cam_switchmanager_get_gear_status();
	gear_status = tcc_cm_ctrl_get_gear_status();
	if(gear_status) {
		tcc_cm_ctrl_exit_earlycamera();
		
		vdev->preview_method = PREVIEW_DD;
		vdev->deint_plugin_status = OFF;
		vdev->sc_plugin_status = ON;
		
		vdev->gParams.handover	= 1;

		tcc_cam_switchmanager_start_preview(vdev);
		
		// Need to parse dt data again because EarlyCamera update new dt data although doing this in probe function.
#if 0
		if(vdev->camera_np)
			tccxxx_parse_vioc_dt_data(vdev);
#endif
	} else {
		tcc_cm_ctrl_stop_earlycamera();
	}
	
	// cm clock off
	tcc_cm_ctrl_off();
	tcc_cm_ctrl_knock();
	
	// show info
	tcc_cam_switchmanager_show_info(vdev);
	
	FUNCTION_OUT
	return 0;
}

int tcc_cam_switchmanager_probe(struct tcc_camera_device * vdev) {
	FUNCTION_IN
		
	// parse device tree
//	tcc_cam_switchmanager_parse_device_tree(vdev);
	
	mutex_init(&vdev->tcc_cam_switchmanager_lock);

	// init params
	vdev->gParams.camera_encode	= 0;
	vdev->gParams.camera_type	= 0;
	vdev->gParams.preview_x		= 0;
	vdev->gParams.preview_y		= 0;
	vdev->gParams.preview_width	= 1024;
	vdev->gParams.preview_height	= 600;
	
	// handover
	tcc_cam_switchmanager_handover_handler(vdev);
	
	// start to monitor
	tcc_cam_swtichmanager_start_monitor(vdev);
	
	FUNCTION_OUT
	return 0;
}

