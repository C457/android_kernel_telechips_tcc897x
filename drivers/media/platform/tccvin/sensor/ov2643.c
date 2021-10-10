/*
 * ov2643.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is OV2643 image sensor driver.
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
#define DBG_I2C_READ
#define dprintk(msg...)	printk(msg)
#else
#define dprintk(msg...)
#endif

#define OV2643_INIT_720P

#define I2C_FLAG	0xFF

struct reg_fmt {
	unsigned char reg;		/* 8-bit register */
	unsigned char val;		/* 8-bit value */
};

/* Array of image sizes supported by MT9D111.  These must be ordered from 
 * smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{ 1600, 1200 },	/* UXGA */
	{ 1280,  960 },	/* SXGA */
	{ 1024,  768 },	/* XGA */
	{  800,  600 },	/* SVGA */
	{  640,  480 },	/* VGA */
	{  320,  240 },	/* QVGA */
	{  176,  144 },	/* QCIF */
};

extern int cif_i2c_send(struct i2c_client *client, unsigned char* data, unsigned short reg_bytes, unsigned short data_bytes);
extern int cif_i2c_recv(struct i2c_client *client, unsigned short reg, unsigned char reg_bytes, unsigned char *val, unsigned short val_bytes);	
extern int vincore_set_zoom(struct tcc_video_device *vdev, int arg);
extern int vincore_set_frameskip(struct tcc_video_device *vdev, int arg);

static struct v4l2_ctrl_t _v4l2_ctrl[] = {
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


/* register initialization tables for sensor */
/* common sensor register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
#ifdef OV2643_INIT_720P
static struct reg_fmt sensor_initialize[] = {
	/* @@ HD720P(YUV) 30fps
	 * 100 99 1280 720
	 * 100 98 0 0
	 * 
	 * OV2643 setting Version History
	 * date 12/15/2009
	 * --6th release of OV2643 Rev1A(AA) setting based AA_V07.
	 */
	{0x12, 0x80},
	{0xc3, 0x1f},
	{0xc4, 0xff},
	{0x3d, 0x48},
	{0xdd, 0xa5},
	{0x0e, 0xb4},
	{0x10, 0x0a},
	{0x11, 0x00},
	{0x0f, 0x14},

	{0x20, 0x01},
	{0x21, 0x25},
	{0x22, 0x00},
	{0x23, 0x0c},
	{0x24, 0x50},
	{0x25, 0x04},
	{0x26, 0x2d},
	{0x27, 0x04},
	{0x28, 0x40},
	{0x29, 0x06},
	{0x2a, 0x40},
	{0x2b, 0x02},
	{0x2c, 0xee},
	{0x1d, 0x04},

	{0x12, 0x48},
	{0x39, 0x10},
	{0xcd, 0x12},
	{0x13, 0xff},
	{0x14, 0xa7},
	{0x15, 0x42},
	{0x3c, 0xa4},
	{0x18, 0x60},
	{0x19, 0x50},
	{0x1a, 0xe2},
	{0x37, 0xe8},
	{0x16, 0x90},
	{0x43, 0x00},
	{0x40, 0xfb},
	{0xa9, 0x44},
	{0x2f, 0xec},
	{0x35, 0x10},
	{0x36, 0x10},
	{0x0c, 0x00},
	{0x0d, 0x00},
	{0xd0, 0x93},
	{0xdc, 0x2b},
	{0xd9, 0x41},
	{0xd3, 0x02},
	{0x3d, 0x08},
	{0x0c, 0x00},
	{0x18, 0x2c},
	{0x19, 0x24},
	{0x1a, 0x71},
	{0x9b, 0x69},
	{0x9c, 0x7d},
	{0x9d, 0x7d},
	{0x9e, 0x69},
	{0x35, 0x04},
	{0x36, 0x04},
	{0x65, 0x12},
	{0x66, 0x20},
	{0x67, 0x39},
	{0x68, 0x4e},
	{0x69, 0x62},
	{0x6a, 0x74},
	{0x6b, 0x85},
	{0x6c, 0x92},
	{0x6d, 0x9e},
	{0x6e, 0xb2},
	{0x6f, 0xc0},
	{0x70, 0xcc},
	{0x71, 0xe0},
	{0x72, 0xee},
	{0x73, 0xf6},
	{0x74, 0x11},
	{0xab, 0x20},
	{0xac, 0x5b},
	{0xad, 0x05},
	{0xae, 0x1b},
	{0xaf, 0x76},
	{0xb0, 0x90},
	{0xb1, 0x90},
	{0xb2, 0x8c},
	{0xb3, 0x04},
	{0xb4, 0x98},
	{0x4c, 0x03},
	{0x4d, 0x30},
	{0x4e, 0x02},
	{0x4f, 0x5c},
	{0x50, 0x56},
	{0x51, 0x00},
	{0x52, 0x66},
	{0x53, 0x03},
	{0x54, 0x30},
	{0x55, 0x02},
	{0x56, 0x5c},
	{0x57, 0x40},
	{0x58, 0x00},
	{0x59, 0x66},
	{0x5a, 0x03},
	{0x5b, 0x20},
	{0x5c, 0x02},
	{0x5d, 0x5c},
	{0x5e, 0x3a},
	{0x5f, 0x00},
	{0x60, 0x66},
	{0x41, 0x1f},
	{0xb5, 0x01},
	{0xb6, 0x02},
	{0xb9, 0x40},
	{0xba, 0x28},
	{0xbf, 0x0c},
	{0xc0, 0x3e},
	{0xa3, 0x0a},
	{0xa4, 0x0f},
	{0xa5, 0x09},
	{0xa6, 0x16},
	{0x9f, 0x0a},
	{0xa0, 0x0f},
	{0xa7, 0x0a},
	{0xa8, 0x0f},
	{0xa1, 0x10},
	{0xa2, 0x04},
	{0xa9, 0x04},
	{0xaa, 0xa6},
	{0x75, 0x6a},
	{0x76, 0x11},
	{0x77, 0x92},
	{0x78, 0x21},
	{0x79, 0xe1},
	{0x7a, 0x02},
	{0x7c, 0x05},
	{0x7d, 0x08},
	{0x7e, 0x08},
	{0x7f, 0x7c},
	{0x80, 0x58},
	{0x81, 0x2a},
	{0x82, 0xc5},
	{0x83, 0x46},
	{0x84, 0x3a},
	{0x85, 0x54},
	{0x86, 0x44},
	{0x87, 0xf8},
	{0x88, 0x08},
	{0x89, 0x70},
	{0x8a, 0xf0},
	{0x8b, 0xf0},
	{0x90, 0xe3},
	{0x93, 0x10},
	{0x94, 0x20},
	{0x95, 0x10},
	{0x96, 0x18},
	{I2C_FLAG, I2C_FLAG}
};
#else //OV2643_INIT_720P
static struct reg_fmt sensor_initialize[] = {
	/* @@ crop 640*480 (YUV) 30fps
	 * 100 99 640 480
	 * 100 98 0 0
	 * 
	 * OV2643 setting Version History
	 * date 03/31/2009
	 * --R1B 3rd release of OV2643 Rev1B(AA) setting based AB_V01
	 * --Lower the PCLK for SVGA 30fps and CIF 30fps settings
	 */
	{0x12, 0x80},
	{0xc3, 0x1f},
	{0xc4, 0xff},
	{0x3d, 0x48},
	{0xdd, 0xa5},
	{0x0e, 0xb8},
	{0x10, 0x0a},
	{0x11, 0x00},
	{0x0f, 0x14},

	{0x20, 0x01}, // HStart
	{0x21, 0xe8}, // HStart
	{0x22, 0x00}, // VStart
	{0x23, 0x42}, // Vstart
	{0x24, 0x28}, // Hsize
	{0x25, 0x04}, // Hsize
	{0x26, 0x1e}, // Vsize
	{0x27, 0x04}, // Vsize
	{0x28, 0x40},
	{0x29, 0x04},
	{0x2a, 0xce},
	{0x2b, 0x02},
	{0x2c, 0x8a},
	{0x1d, 0x04},

	//{0x27, 0x84},

	{0x12, 0x09},
	{0x39, 0xd0},
	{0xcd, 0x13},
	{0x13, 0xff},
	{0x14, 0xa7},
	{0x15, 0x42},
	{0x3c, 0xa4},
	{0x18, 0x60},
	{0x19, 0x50},
	{0x1a, 0xe2},
	{0x37, 0xe8},
	{0x16, 0x90},
	{0x43, 0x00},
	{0x40, 0xfb},
	{0xa9, 0x44},
	{0x2f, 0xec},
	{0x35, 0x10},
	{0x36, 0x10},
	{0x0c, 0x00},
	{0x0d, 0x00},
	{0xd0, 0x93},
	{0xdc, 0x2b},
	{0xd9, 0x41},
	{0xd3, 0x02},
	{0xde, 0x7c},
	{0x3d, 0x08},
	{0x0c, 0x00},
	{0x18, 0x2c},
	{0x19, 0x24},
	{0x1a, 0x71},
	{0x9b, 0x69},
	{0x9c, 0x7d},
	{0x9d, 0x7d},
	{0x9e, 0x69},
	{0x35, 0x04},
	{0x36, 0x04},
	{0x65, 0x12},
	{0x66, 0x20},
	{0x67, 0x39},
	{0x68, 0x4e},
	{0x69, 0x62},
	{0x6a, 0x74},
	{0x6b, 0x85},
	{0x6c, 0x92},
	{0x6d, 0x9e},
	{0x6e, 0xb2},
	{0x6f, 0xc0},
	{0x70, 0xcc},
	{0x71, 0xe0},
	{0x72, 0xee},
	{0x73, 0xf6},
	{0x74, 0x11},
	{0xab, 0x20},
	{0xac, 0x5b},
	{0xad, 0x05},
	{0xae, 0x1b},
	{0xaf, 0x76},
	{0xb0, 0x90},
	{0xb1, 0x90},
	{0xb2, 0x8c},
	{0xb3, 0x04},
	{0xb4, 0x98},
	{0x4c, 0x03},
	{0x4d, 0x30},
	{0x4e, 0x02},
	{0x4f, 0x5c},
	{0x50, 0x56},
	{0x51, 0x00},
	{0x52, 0x66},
	{0x53, 0x03},
	{0x54, 0x30},
	{0x55, 0x02},
	{0x56, 0x5c},
	{0x57, 0x40},
	{0x58, 0x00},
	{0x59, 0x66},
	{0x5a, 0x03},
	{0x5b, 0x20},
	{0x5c, 0x02},
	{0x5d, 0x5c},
	{0x5e, 0x3a},
	{0x5f, 0x00},
	{0x60, 0x66},
	{0x41, 0x1f},
	{0xb5, 0x01},
	{0xb6, 0x02},
	{0xb9, 0x40},
	{0xba, 0x28},
	{0xbf, 0x0c},
	{0xc0, 0x3e},
	{0xa3, 0x0a},
	{0xa4, 0x0f},
	{0xa5, 0x09},
	{0xa6, 0x16},
	{0x9f, 0x0a},
	{0xa0, 0x0f},
	{0xa7, 0x0a},
	{0xa8, 0x0f},
	{0xa1, 0x10},
	{0xa2, 0x04},
	{0xa9, 0x04},
	{0xaa, 0xa6},
	{0x75, 0x6a},
	{0x76, 0x11},
	{0x77, 0x92},
	{0x78, 0x21},
	{0x79, 0xe1},
	{0x7a, 0x02},
	{0x7c, 0x05},
	{0x7d, 0x08},
	{0x7e, 0x08},
	{0x7f, 0x7c},
	{0x80, 0x58},
	{0x81, 0x2a},
	{0x82, 0xc5},
	{0x83, 0x46},
	{0x84, 0x3a},
	{0x85, 0x54},
	{0x86, 0x44},
	{0x87, 0xf8},
	{0x88, 0x08},
	{0x89, 0x70},
	{0x8a, 0xf0},
	{0x8b, 0xf0},
	{0x90, 0xe3},
	{0x93, 0x10},
	{0x94, 0x20},
	{0x95, 0x10},
	{0x96, 0x18},
	{I2C_FLAG, I2C_FLAG}
};
#endif

static struct reg_fmt sensor_set_preview[] = {
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt *sensor_reg_common[2] = {
	sensor_initialize,
	sensor_set_preview
};

static int write_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	int err, err_cnt = 0;
	unsigned char data[2];
	unsigned char bytes = 0;
	const struct reg_fmt *next = reglist;

	while (!(next->reg == I2C_FLAG && next->val == I2C_FLAG)) {
		if (next->reg == I2C_FLAG && next->val != I2C_FLAG) {
			mdelay(next->val);
			dprintk("sensor write mdelay(%d)\n", next->val);
			next++;
		} else {
again_reg:
			bytes = 0;
			data[bytes++] = next->reg;
			data[bytes++] = next->val;
			err = cif_i2c_send(client, data, 1, 1);
			if (err) {
				dprintk("error: i2c_send{0x%02X, 0x%02X} try(%d)\n", data[0], data[1], err_cnt);
				if (err_cnt++ >= 3) return err;
				else goto again_reg;
			}
#ifdef DBG_I2C_READ
			{
				unsigned char val;
				err = cif_i2c_recv(client, next->reg, 1, &val, 1);
				if (err) dprintk("error: i2c_recv{0x%02X, 0x%02X}\n", next->reg, val);
				dprintk("0x%02X: 0x%02X = 0x%02X", next->reg, next->val, val);

				if (data[1] != val)
					dprintk("\e[31m mismatch!!! \e[0m \n");
				else
					dprintk("\n");
			}
#endif
			err_cnt = 0;
			next++;
		}
	}

	return 0;
}

static int sensor_open(struct tcc_video_device *vdev)
{
	cif_power_disable(vdev);
	msleep(10);

	cif_power_enable(vdev);
	msleep(10);

	cif_powerdown_disable(vdev);
	msleep(10);

	cif_reset_low(vdev);
	msleep(10);

	cif_open(vdev);
	msleep(50);

	cif_reset_high(vdev);
	msleep(10);

	return write_regs(vdev->cif_i2c_client, sensor_reg_common[0]);
}

static int sensor_close(struct tcc_video_device *vdev)
{
	cif_reset_low(vdev);
	cif_power_disable(vdev);
	cif_powerdown_enable(vdev);

	cif_close(vdev);
	msleep(5);

	return 0;
}

static int sensor_powerdown(struct tcc_video_device *vdev)
{
	cif_reset_low(vdev);
	cif_power_disable(vdev);
	cif_powerdown_enable(vdev);
	return 0;
}

static int sensor_preview(struct tcc_video_device *vdev)
{
	return write_regs(vdev->cif_i2c_client, sensor_reg_common[1]);
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

void sensor_init_ov2643(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "ov2643";
	sinfo->i2c_addr			= (0x60 >> 1);
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 1280;
	sinfo->prv_h			= 720;
	sinfo->framerate		= 30;
	sinfo->mclk				= 240000;
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= POSITIVE_EDGE;
	sinfo->polarity_vsync	= ACT_LOW;
	sinfo->polarity_hsync	= ACT_HIGH;
	sinfo->de2hsync			= DE_FROM_HS_CONNECT;
	sinfo->interface_type	= BT601;
	sinfo->scan_type		= SCAN_PROGRESSIVE;
	sinfo->frame_skip_vin	= FRAME_NOT_SKIP;
	sinfo->frame_skip_irq	= FRAME_NOT_SKIP;

	if (sinfo->frame_skip_vin == FRAME_SKIP || sinfo->frame_skip_irq == FRAME_SKIP)
		sinfo->framerate	= sinfo->framerate / 2;

	/* zoom */
	sinfo->zoom_type		= ZOOM_NOT_USE;
	sinfo->prv_zoffx		= 0;
	sinfo->prv_zoffy		= 0;

	/* callback func */
	sinfo->open				= sensor_open;
	sinfo->close			= sensor_close;
	sinfo->powerdown		= sensor_powerdown;
	sinfo->set_preview		= sensor_preview;

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
EXPORT_SYMBOL(sensor_init_ov2643);
