/*
 * nt99141.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2013 
 * Description: This is NT99141 image sensor driver.
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

#define I2C_FLAG	0xFF

struct reg_fmt {
	unsigned short reg;		/* 16-bit register */
	unsigned char val;		/* 8-bit value */
};

/* Array of image sizes supported by nt99141.  These must be ordered from 
 * smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{ 1280,  720 },	/* 720p */
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
static struct reg_fmt sensor_initialize[] = {
	/* [Date] 20130329
	 * [Sensor] Novatek NT99141 1/4"
	 *       MCLK: 24Mhz
	 *       PCLK: 68Mhz
	 * Frame Rate: 30fps for YUV preview mode
	 *    Flicker: 60hz/50hz
	 */
	{0x3109, 0x04},
	{0x3040, 0x04},
	{0x3041, 0x02},
	{0x3042, 0xFF},
	{0x3043, 0x08},
	{0x3052, 0xE0},
	{0x305F, 0x33},
	{0x3100, 0x07},
	{0x3106, 0x03},
	{0x3108, 0x00},
	{0x3110, 0x22},
	{0x3111, 0x57},
	{0x3112, 0x22},
	{0x3113, 0x55},
	{0x3114, 0x05},
	{0x3135, 0x00},
	{0x32F0, 0x00},	// Output_Format
	{0x3290, 0x01},
	{0x3291, 0xA0},
	{0x3296, 0x01},
	{0x3297, 0x73},
	{0x3250, 0xC0},
	{0x3251, 0x00},
	{0x3252, 0xDF},
	{0x3253, 0x85},
	{0x3254, 0x00},
	{0x3255, 0xEB},
	{0x3256, 0x81},
	{0x3257, 0x40},
	{0x329B, 0x01},
	{0x32A1, 0x00},
	{0x32A2, 0xA0},
	{0x32A3, 0x01},
	{0x32A4, 0xA0},
	{0x32A5, 0x01},
	{0x32A6, 0x18},
	{0x32A7, 0x01},
	{0x32A8, 0xE0},
	{0x3210, 0x16},
	{0x3211, 0x19},
	{0x3212, 0x16},
	{0x3213, 0x14},
	{0x3214, 0x15},
	{0x3215, 0x18},
	{0x3216, 0x15},
	{0x3217, 0x14},
	{0x3218, 0x15},
	{0x3219, 0x18},
	{0x321A, 0x15},
	{0x321B, 0x14},
	{0x321C, 0x14},
	{0x321D, 0x17},
	{0x321E, 0x14},
	{0x321F, 0x12},
	{0x3231, 0x74},
	{0x3232, 0xC4},
	{0x3270, 0x00},
	{0x3271, 0x0C},
	{0x3272, 0x18},
	{0x3273, 0x32},
	{0x3274, 0x44},
	{0x3275, 0x54},
	{0x3276, 0x70},
	{0x3277, 0x88},
	{0x3278, 0x9D},
	{0x3279, 0xB0},
	{0x327A, 0xCF},
	{0x327B, 0xE2},
	{0x327C, 0xEF},
	{0x327D, 0xF7},
	{0x327E, 0xFF},
	{0x3302, 0x00},
	{0x3303, 0x40},
	{0x3304, 0x00},
	{0x3305, 0x96},
	{0x3306, 0x00},
	{0x3307, 0x29},
	{0x3308, 0x07},
	{0x3309, 0xBA},
	{0x330A, 0x06},
	{0x330B, 0xF5},
	{0x330C, 0x01},
	{0x330D, 0x51},
	{0x330E, 0x01},
	{0x330F, 0x30},
	{0x3310, 0x07},
	{0x3311, 0x16},
	{0x3312, 0x07},
	{0x3313, 0xBA},
	{0x3326, 0x02},
	{0x3327, 0x0A},
	{0x3328, 0x0A},
	{0x3329, 0x06},
	{0x332A, 0x06},
	{0x332B, 0x1C},
	{0x332C, 0x1C},
	{0x332D, 0x00},
	{0x332E, 0x1D},
	{0x332F, 0x1F},
	{0x3360, 0x10},
	{0x3361, 0x18},
	{0x3362, 0x1f},
	{0x3363, 0x37},
	{0x3364, 0x80},
	{0x3365, 0x80},
	{0x3366, 0x68},
	{0x3367, 0x60},
	{0x3368, 0x30},
	{0x3369, 0x28},
	{0x336A, 0x20},
	{0x336B, 0x10},
	{0x336C, 0x00},
	{0x336D, 0x20},
	{0x336E, 0x1C},
	{0x336F, 0x18},
	{0x3370, 0x10},
	{0x3371, 0x38},
	{0x3372, 0x3C},
	{0x3373, 0x3F},
	{0x3374, 0x3F},
	{0x338A, 0x34},
	{0x338B, 0x7F},
	{0x338C, 0x10},
	{0x338D, 0x23},
	{0x338E, 0x7F},
	{0x338F, 0x14},
	{0x32F6, 0x0F},
	{0x32F9, 0x42},
	{0x32FA, 0x24},
	{0x3325, 0x4A},
	{0x3330, 0x00},
	{0x3331, 0x0A},
	{0x3332, 0xFF},
	{0x3338, 0x30},
	{0x3339, 0x84},
	{0x333A, 0x48},
	{0x333F, 0x07},
	{0x3375, 0x0A},
	{0x3376, 0x0C},
	{0x3377, 0x10},
	{0x3378, 0x14},
	{0x3060, 0x01},

	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt sensor_50hz[] = {
	/* [Date] 20130329 (fixed 30fps)
	 * Flicker: 50Hz
	 */
	{0x32BF, 0x60},
	{0x32C0, 0x5A},
	{0x32C1, 0x5A},
	{0x32C2, 0x5A},
	{0x32C3, 0x00},
	{0x32C4, 0x20},
	{0x32C5, 0x20},
	{0x32C6, 0x20},
	{0x32C7, 0x00},
	{0x32C8, 0xF1},
	{0x32C9, 0x5A},
	{0x32CA, 0x7A},
	{0x32CB, 0x7A},
	{0x32CC, 0x7A},
	{0x32CD, 0x7A},
	{0x32DB, 0x7E},
	{0x3200, 0x3E},
	{0x3201, 0x0F},
	{0x3028, 0x13},
	{0x3029, 0x20},
	{0x302A, 0x00},
	{0x3022, 0x27},	// rotation reg
	{0x3023, 0x24},
	{0x3002, 0x00},
	{0x3003, 0x04},
	{0x3004, 0x00},
	{0x3005, 0x04},
	{0x3006, 0x05},
	{0x3007, 0x03},
	{0x3008, 0x02},
	{0x3009, 0xD3},
	{0x300A, 0x06},
	{0x300B, 0x7C},
	{0x300C, 0x03},
	{0x300D, 0x23},
	{0x300E, 0x05},
	{0x300F, 0x00},
	{0x3010, 0x02},
	{0x3011, 0xD0},
	{0x32B8, 0x3F},
	{0x32B9, 0x31},
	{0x32BB, 0x87},
	{0x32BC, 0x38},
	{0x32BD, 0x3C},
	{0x32BE, 0x34},
	{0x3201, 0x3F},
	{0x3021, 0x06},
	{0x3060, 0x01},

	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt sensor_60hz[] = {
	/* [Date] 20130329 (fixed 30fps)
	 * Flicker: 60Hz
	 */
	{0x32BF, 0x60},
	{0x32C0, 0x60},
	{0x32C1, 0x5F},
	{0x32C2, 0x5F},
	{0x32C3, 0x00},
	{0x32C4, 0x20},
	{0x32C5, 0x20},
	{0x32C6, 0x20},
	{0x32C7, 0x00},
	{0x32C8, 0xC9},
	{0x32C9, 0x5F},
	{0x32CA, 0x7F},
	{0x32CB, 0x7F},
	{0x32CC, 0x7F},
	{0x32CD, 0x80},
	{0x32DB, 0x79},
	{0x3200, 0x3E},
	{0x3201, 0x0F},
	{0x3028, 0x13},
	{0x3029, 0x20},
	{0x302A, 0x00},
	{0x3022, 0x27},	// rotation reg
	{0x3023, 0x24},
	{0x3002, 0x00},
	{0x3003, 0x04},
	{0x3004, 0x00},
	{0x3005, 0x04},
	{0x3006, 0x05},
	{0x3007, 0x03},
	{0x3008, 0x02},
	{0x3009, 0xD3},
	{0x300A, 0x06},
	{0x300B, 0x7C},
	{0x300C, 0x03},
	{0x300D, 0x23},
	{0x300E, 0x05},
	{0x300F, 0x00},
	{0x3010, 0x02},
	{0x3011, 0xD0},
	{0x32B8, 0x3F},
	{0x32B9, 0x31},
	{0x32BB, 0x87},
	{0x32BC, 0x38},
	{0x32BD, 0x3C},
	{0x32BE, 0x34},
	{0x3201, 0x3F},
	{0x3021, 0x06},
//	{0x3069, 0x02},	// d, v/hsycn drivring strength
//	{0x306A, 0x02},	// pclk drivring strength
	{0x3060, 0x01},

	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt sensor_set_preview[] = {
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt *sensor_reg_common[4] = {
	sensor_initialize,
	sensor_set_preview,
	sensor_50hz,
	sensor_60hz,
};

#ifdef DBG_I2C_READ
static void read_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	unsigned char val;
	const struct reg_fmt *next = reglist;

	printk("---read sensor regs---\n");
	while (!(next->reg == I2C_FLAG && next->val == I2C_FLAG)) {
		if (next->reg == I2C_FLAG && next->val != I2C_FLAG) {
			next++;
		} else {
			if (cif_i2c_recv(client, next->reg, 2, &val, 1))
				printk("error: i2c_recv{0x%04x}\n", next->reg);
			printk("0x%04x: 0x%02x = 0x%02x", next->reg, next->val, val);
			if (next->val != val) printk("\e[31m mismatch! \e[0m\n");
			else  printk("\n");
			next++;
		}
	}
	printk("----------------------\n");
}
#endif

static int write_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	int err, err_cnt = 0;
	unsigned char data[3];
	const struct reg_fmt *next = reglist;

	while (!(next->reg == I2C_FLAG && next->val == I2C_FLAG)) {
		if (next->reg == I2C_FLAG && next->val != I2C_FLAG) {
			mdelay(next->val);
			dprintk("sensor write mdelay(%d)\n", next->val);
			next++;
		} else {
again_reg:
			data[0] = next->reg >> 8;
			data[1] = (u8)next->reg & 0xff;
			data[2] = next->val;
			err = cif_i2c_send(client, data, 2, 1);
			if (err) {
				printk("error: i2c_send{0x%02x%02x, 0x%02x} try(%d)\n", data[0], data[1], data[2], err_cnt);
				if (err_cnt++ >= 3) return err;
				else goto again_reg;
			}
#ifdef DBG_I2C_READ
			{
				unsigned char val;
				err = cif_i2c_recv(client, next->reg, 2, &val, 1);
				if (err) dprintk("error: i2c_recv{0x%04x, 0x%02x}\n", next->reg, val);
				dprintk("0x%04x: 0x%02x = 0x%02x", next->reg, data[2], val);

				if (data[2] != val) {
					if (next->reg == 0x3060) {
						dprintk(" activate setting (auto clear)\n");
					} else {
						printk("\e[31m mismatch! \e[0m\n");
						cif_i2c_send(client, data, 2, 1);
						cif_i2c_recv(client, next->reg, 2, &val, 1);
						dprintk("       retry = 0x%02x\n", val);
					}
				} else {
					dprintk("\n");
				}
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
	int ret;

	//cif_power_disable(vdev);

	//cif_power_enable(vdev);

	cif_powerdown_disable(vdev);

	cif_reset_low(vdev);

	cif_open(vdev);
	msleep(10);		// 300 mclk cycle

	cif_reset_high(vdev);
	msleep(10);		// 300 mclk cycle

	ret = write_regs(vdev->cif_i2c_client, sensor_reg_common[0]);
	ret |= write_regs(vdev->cif_i2c_client, sensor_reg_common[3]);

#ifdef DBG_I2C_READ
	if (0) {
		read_regs(vdev->cif_i2c_client, sensor_reg_common[0]);
		read_regs(vdev->cif_i2c_client, sensor_reg_common[3]);
	}
#endif

	return ret;
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

void sensor_init_nt99141(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "nt99141";
	sinfo->i2c_addr			= (0x54 >> 1);	/* 7bit addr: 0x2a */
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 1280;
	sinfo->prv_h			= 720;
	sinfo->framerate		= 30;
	sinfo->mclk				= 240000;		/* 24Mhz */
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= NEGATIVE_EDGE;
	sinfo->polarity_vsync	= ACT_HIGH;
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
EXPORT_SYMBOL(sensor_init_nt99141);
