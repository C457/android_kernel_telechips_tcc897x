/*
 * thcv220.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is THCV220 video data receiver driver.
 *
 * Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>		// standard ioctl
#include <mach/tcc_cam_ioctrl.h>	// user spec ioctl
#include <mach/vioc_vin.h>

#include "tcc_vin_hw.h"


#ifdef CONFIG_TCC_SENSOR_DEBUG
#define dprintk(msg...)	printk(msg)
#else
#define dprintk(msg...)
#endif


/* Array of image sizes supported by THCV220.
 * These must be ordered from smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{1280, 720}
};

extern int vincore_set_zoom(struct tcc_video_device *vdev, int arg);
extern int vincore_set_frameskip(struct tcc_video_device *vdev, int arg);


static struct v4l2_ctrl_t _v4l2_ctrl[] = {
/* Private control */
	{
		{
			.id = V4L2_CID_FRAMESKIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Frame Skip",
			.minimum = 0,
			.maximum = 15,
			.step = 1,
			.default_value = 0,	// not skip
			.flags = 0,
		},
		.current_value = 0,
		.need_set = 0,
	},
};

static unsigned int _v4l2_re_ctrl[] = {
	V4L2_CID_FRAMESKIP,
};


static int sensor_open(struct tcc_video_device *vdev)
{
	int ret = 0;

	if (is_cam_init_by_lk(CAM_INIT_THCV220)) {
		clear_cam_init_by_lk(CAM_INIT_THCV220);

		cif_open(vdev);
		//msleep(40);
	} else {
		cif_power_disable(vdev);
		//msleep(10);

		cif_power_enable(vdev);
		//msleep(10);

		cif_powerdown_enable(vdev);		// high is operating mode
		//msleep(10);

		cif_open(vdev);
		//msleep(40);
	}

	return ret;
}

static int sensor_close(struct tcc_video_device *vdev)
{
	cif_power_disable(vdev);
	cif_powerdown_disable(vdev);	// low is power-down mode

	cif_close(vdev);
	msleep(5);

	return 0;
}

static int sensor_v4l2_set_ctrl(struct tcc_video_device *vdev, __u32 id, int val)
{
	int ret = -EINVAL;

	switch (id) {
	case V4L2_CID_FRAMESKIP:
		ret = vincore_set_frameskip(vdev, val);
		break;
	default:
		break;
	}
	return ret;
}

void sensor_init_thcv220(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "thcv220";
	//sinfo->i2c_addr		= (0xB8 >> 1);
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 1280;
	sinfo->prv_h			= 720;
	sinfo->framerate		= 30;
	//sinfo->mclk			= 245000;
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= POSITIVE_EDGE;
	sinfo->polarity_vsync	= ACT_HIGH;
	sinfo->polarity_hsync	= ACT_HIGH;
	sinfo->de2hsync			= DE_FROM_HS_CONNECT;
	sinfo->interface_type	= BT601;
	sinfo->scan_type		= SCAN_PROGRESSIVE;
	sinfo->frame_skip_vin	= FRAME_NOT_SKIP;
	sinfo->frame_skip_irq	= FRAME_NOT_SKIP;

	if (sinfo->frame_skip_vin == FRAME_SKIP || sinfo->frame_skip_irq == FRAME_SKIP)
		sinfo->framerate	= sinfo->framerate / 2;

	//sinfo->init_mode		= SINIT_ZOMBI;

	/* zoom */
	sinfo->zoom_type		= ZOOM_NOT_USE;
	sinfo->prv_zoffx		= 0;
	sinfo->prv_zoffy		= 0;

	/* callback func */
	sinfo->open				= sensor_open;
	sinfo->close			= sensor_close;
	sinfo->powerdown		= NULL;
	sinfo->set_preview		= NULL;

	/* sensor tunning func */
	sinfo->v4l2_set_ctrl	= sensor_v4l2_set_ctrl;
	sinfo->need_new_set		= 0;
	sinfo->v4l2_re_ctrl		= _v4l2_re_ctrl;
	sinfo->n_v4l2_re_ctrl	= ARRAY_SIZE(_v4l2_re_ctrl);
	sinfo->n_v4l2_ctrl		= ARRAY_SIZE(_v4l2_ctrl);
	v4l2_ctrl_size = (sizeof(struct v4l2_ctrl_t) * sinfo->n_v4l2_ctrl);
	sinfo->v4l2_ctrl = kzalloc(v4l2_ctrl_size, GFP_KERNEL);
	memcpy(sinfo->v4l2_ctrl, _v4l2_ctrl, v4l2_ctrl_size);

	/* initialize current_value */
	for (i = 0; i < sinfo->n_v4l2_ctrl; i++) {
		sinfo->v4l2_ctrl[i].current_value = sinfo->v4l2_ctrl[i].qc.default_value;
		if (sinfo->v4l2_ctrl[i].qc.id == V4L2_CID_ZOOM_ABSOLUTE
			&& sinfo->zoom_type == ZOOM_USE_SW) {
				sinfo->v4l2_ctrl[i].qc.maximum = ZOOM_USE_SW_MAX_VAL;
				sinfo->v4l2_ctrl[i].qc.flags = 0;
		}
	}
}
EXPORT_SYMBOL(sensor_init_thcv220);
