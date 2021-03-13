/*
 * tw9900.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is TVP5150 decoder driver.
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

#define CONFIG_TCC_SENSOR_DEBUG

#ifdef CONFIG_TCC_SENSOR_DEBUG
#define DBG_I2C_READ
#define dprintk(msg...)	printk(msg)
#else
#define dprintk(msg...)
#endif

#define I2C_FLAG 0xFF

struct reg_fmt {
	unsigned char reg;	/* 8-bit register */
	unsigned char val;	/* 8-bit value */
};

/* Array of image sizes supported by TVP5150.
 * These must be ordered from smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{720, 480}
};

extern int cif_i2c_send(struct i2c_client *client, unsigned char* data, unsigned short reg_bytes, unsigned short data_bytes);
extern int cif_i2c_recv(struct i2c_client *client, unsigned short reg, unsigned char reg_bytes, unsigned char *val, unsigned short val_bytes);	
extern int vincore_set_zoom(struct tcc_video_device *vdev, int arg);


static struct v4l2_ctrl_t _v4l2_ctrl[] = {
};

static unsigned int _v4l2_re_ctrl[] = {
};


/* register initialization tables for sensor */
/* common sensor register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
static struct reg_fmt tw9900_bt656_initialize[] = {
	{0x03, 0xa2}, // MODE:ITU-R-656 compatible data sequence, LEN:8Bit YCbCr4:2:2 Output format
	{0x05, 0x09}, // VSync:L , HSync:L
	{0x08, 0x12}, // VDELAY:	0x12
	{0x09, 0xf4}, // VACTIVE:	0xF4
	{0x0A, 0x10}, // HDELAY:	0x10
	{0x0B, 0xD0}, // HACTIVE:	0xD0 (Low) -> 0x2D0
	{0x19, 0x5f}, // VBI Control
	{0x1a, 0x0f}, // Analog Control
	{0x2d, 0x07}, // MISC1 
	{0x2F, 0xE6}, // MISC2
	{0x6b, 0x09}, // HSync Start Position
	{0x6c, 0x19}, // HSync End Position
	{0x6d, 0x0a}, // VSync Start Position
	{I2C_FLAG, I2C_FLAG}
};


static struct reg_fmt tw9900_set_preview[] = {
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt *sensor_reg_common[2] = {
	tw9900_bt656_initialize,
	tw9900_set_preview
};

static void read_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	unsigned char reg, val;

	for (reg = 0x00; reg <= 0xFC; reg++) {
		if (cif_i2c_recv(client, reg, 1, &val, 1))
			printk("error: i2c_recv{0x%02X}\n", reg);
		dprintk("0x%02X, 0x%02X\n", reg, val);
	}
}

static int write_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	int err;
	int err_cnt = 0;	
	unsigned char data[2];
	const struct reg_fmt *next = reglist;

	dprintk("tw9900 sensor init\n");

	while (!(next->reg == I2C_FLAG && next->val == I2C_FLAG)) {
		if (next->reg == I2C_FLAG && next->val != I2C_FLAG) {
			mdelay(next->val);
			dprintk("sensor write mdelay(%d)\n", next->val);
			next++;
		} else {
again_reg:
			data[0] = next->reg;
			data[1] = next->val;
			err = cif_i2c_send(client, data, 1, 1);
			if (err) {
				printk("error: i2c_send{0x%02X, 0x%02X} try(%d)\n", data[0], data[1], err_cnt);
				if (err_cnt++ >= 3) return err;
				else goto again_reg;
			}
#ifdef DBG_I2C_READ
			{
				unsigned char val;
				err = cif_i2c_recv(client, next->reg, 1, &val, 1);
				if (err) dprintk("error: i2c_recv{0x%02X, 0x%02X}\n", next->reg, val);
				dprintk("0x%02X: 0x%02X = 0x%02X", next->reg, data[1], val);

				if (data[1] != val) {
					printk("\e[31m mismatch! \e[0m\n");
					cif_i2c_send(client, data, 1, 1);
					cif_i2c_recv(client, next->reg, 1, &val, 1);
					dprintk("       retry = 0x%02X\n", val);
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

	cif_power_disable(vdev);
	msleep(10);
	
	cif_power_enable(vdev);
	msleep(10);

	//cif_powerdown_disable(vdev);
	//msleep(10);

	//cif_reset_low(vdev);
	//msleep(10);

	cif_open(vdev);
	msleep(40);

	cif_reset_high(vdev);
	msleep(15);


	ret = write_regs(vdev->cif_i2c_client, sensor_reg_common[0]);

	if (0) read_regs(vdev->cif_i2c_client, sensor_reg_common[0]);

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

static int sensor_preview(struct tcc_video_device *vdev)
{
	return write_regs(vdev->cif_i2c_client, sensor_reg_common[1]);
}

static int sensor_v4l2_set_ctrl(struct tcc_video_device *vdev, __u32 id, int val)
{
	int err = -EINVAL;

	switch (id) {
	default:
		break;
	}
	return err;
}

void sensor_init_tw9900(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "tw9900";
	sinfo->i2c_addr			= (0x8A >> 1);
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 720;
	sinfo->prv_h			= 480;
	sinfo->framerate		= 60;
	sinfo->mclk				= 143810;	// 14.318Mhz
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= POSITIVE_EDGE;
	sinfo->polarity_vsync	= ACT_HIGH;
	sinfo->polarity_hsync	= ACT_HIGH;
	sinfo->de2hsync			= DE_HS_NORMAL;
	sinfo->interface_type	= BT656;
	sinfo->scan_type		= SCAN_INTERLACE;
	sinfo->frame_skip_vin	= FRAME_NOT_SKIP;

#if defined(CONFIG_TCC_DEINTL_ONE_FIELD)
	sinfo->de_interlace		= DEINTL_ONE_FIELD;
	sinfo->scan_type		= SCAN_PROGRESSIVE;
	sinfo->frame_skip_vin	= FRAME_SKIP;
#elif defined(CONFIG_TCC_DEINTL_SIMPLE)
	sinfo->de_interlace		= DEINTL_SIMPLE;
#elif defined(CONFIG_TCC_DEINTL_VIQE_2D)
	sinfo->de_interlace		= DEINTL_VIQE_2D;
#elif defined(CONFIG_TCC_DEINTL_VIQE_3D)
	sinfo->de_interlace		= DEINTL_VIQE_3D;
#elif defined(CONFIG_TCC_DEINTL_WDMA_AUTO)
	sinfo->de_interlace		= DEINTL_WDMA_AUTO;
#elif defined(CONFIG_TCC_DEINTL_WDMA_MANUAL)
	sinfo->de_interlace		= DEINTL_WDMA_MANUAL;
	sinfo->scan_type		= SCAN_PROGRESSIVE;
#else
	sinfo->de_interlace		= DEINTL_NOT_USE;
#endif

#if defined(CONFIG_TCC_DEINTL_FRAMESKIP)
	sinfo->frame_skip_irq	= FRAME_SKIP;
#endif

	if (sinfo->frame_skip_vin == FRAME_SKIP || sinfo->frame_skip_irq == FRAME_SKIP)
		sinfo->framerate	/= 2;

	/* zoom */
	sinfo->zoom_type		= ZOOM_NOT_USE;
	sinfo->prv_zoffx		= 0;
	sinfo->prv_zoffy		= 0;

	/* callback func */
	sinfo->open				= sensor_open;
	sinfo->close			= sensor_close;
	sinfo->powerdown		= NULL;
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
EXPORT_SYMBOL(sensor_init_tw9900);
