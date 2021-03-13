/*
 * mc1080a.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2013
 * Description: This is MC1080A image sensor module (OV2710 Sensor + THP7212 ISP) driver.
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
#include "mc1080a.h"


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

/* Array of image sizes supported by MC1080A.
 * These must be ordered from smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{1920, 1080}
};

extern int cif_i2c_send(struct i2c_client *client, unsigned char* data, unsigned short reg_bytes, unsigned short data_bytes);
extern int cif_i2c_recv(struct i2c_client *client, unsigned short reg, unsigned char reg_bytes, unsigned char *val, unsigned short val_bytes);	
extern int vincore_set_zoom(struct tcc_video_device *vdev, int arg);


static struct v4l2_ctrl_t _v4l2_ctrl[] = {
};

static unsigned int _v4l2_re_ctrl[] = {
};

//#define I2C_DBG
int thp7212_fw_download(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msgs;
#ifndef I2C_DBG
	unsigned char cmd_1[] = {0xff, 0x08, 0x10, 0x00, 0x80};
	unsigned char cmd_2[] = {0xff, 0x08, 0x10, 0x01, 0x00};
	unsigned char cmd_3[] = {0xff, 0x00, 0x01};
	unsigned char cmd_4[] = {0xf0, 0x0b, 0x01};
	unsigned char cmd_func[] = {0xf0, 0x38, 0x01};	// {0xf038, 0x01} - rotate 180 degree

	msgs.addr = client->addr;
	msgs.flags = 0;
#endif

#ifdef I2C_DBG
	unsigned char cmd[] = {0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x01, 0x02, 0x03};
	msgs.addr = client->addr;
	msgs.flags = 0;
	msgs.buf = (unsigned char *)cmd;
	msgs.len = sizeof(cmd);
	msleep(100);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	dprintk("addr(0x%x) len(%d) ret(%d) client->flags(0x%x)\n", msgs.addr, msgs.len, ret, client->flags);

#else

	printk("\e[31m[MC1080A] wait until the firmware download is finished (about 13secs) \e[0m\n");

	/* 4. Start C0 00 00 thine_fw_78_02_00_00.1 Stop */
	msgs.buf = (unsigned char *)thine_fw_78_02_00_00_1;;
	msgs.len = thine_fw_78_02_00_00_1_len;
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step4 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 5. Start C0 FF 08 10 00 80 Stop */
	msgs.buf = (unsigned char *)cmd_1;
	msgs.len = sizeof(cmd_1);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step5 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 6. Start C0 00 00 thine_fw_78_02_00_00.2 Stop */
	msgs.buf = (unsigned char *)thine_fw_78_02_00_00_2;
	msgs.len = thine_fw_78_02_00_00_2_len;
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step6 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 7. Start C0 FF 08 10 01 00 Stop */
	msgs.buf = (unsigned char *)cmd_2;;
	msgs.len = sizeof(cmd_2);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step7 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 8. Start C0 00 00 thine_fw_78_02_00_00.3 Stop */
	msgs.buf = (unsigned char *)thine_fw_78_02_00_00_3;
	msgs.len = thine_fw_78_02_00_00_3_len;
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step8 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 9. Start C0 FF 00 01 Stop */
	msgs.buf = (unsigned char *)cmd_3;;
	msgs.len = sizeof(cmd_3);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step9 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* 10. Wait 100ms (if error, 200ms) */
	msleep(100);

	/* 11. Start C0 F0 0B 01 Stop */
	msgs.buf = (unsigned char *)cmd_4;;
	msgs.len = sizeof(cmd_4);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: step11 error(%d)\n", __func__, ret);
		goto exit;
	}

	/* function: {0xf038, 0x01} - rotate 180 degree */
	//msleep(100);
	msgs.buf = (unsigned char *)cmd_func;;
	msgs.len = sizeof(cmd_func);
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) {
		dprintk("%s: function error(%d)\n", __func__, ret);
		goto exit;
	}
#endif

exit:
	dprintk("mc1080a: thp7212 fw download(%d)\n", ret);
	return ret;
}

static int sensor_open(struct tcc_video_device *vdev)
{
	/* THP7212 boot sequence reference (on the basis of Clock-Stretching)
	 *  1. power on
	 *  2. RESET_N low to high
	 *  3. wait 2ms
	 *  4. Start C0 00 00 thine_fw_78_02_00_00.1 Stop
	 *  5. Start C0 FF 08 10 00 80 Stop
	 *  6. Start C0 00 00 thine_fw_78_02_00_00.2 Stop
	 *  7. Start C0 FF 08 10 01 00 Stop
	 *  8. Start C0 00 00 thine_fw_78_02_00_00.3 Stop
	 *  9. Start C0 FF 00 01 Stop
	 * 10. Wait 100ms (if error, 200ms)
	 * 11. Start C0 F0 0B 01 Stop
	 */
	//cif_reset_low(vdev);
	cif_open(vdev);
	cif_reset_high(vdev);
	msleep(2);
	return thp7212_fw_download(vdev->cif_i2c_client);
}

static int sensor_close(struct tcc_video_device *vdev)
{
	cif_reset_low(vdev);
	cif_close(vdev);
	msleep(5);
	return 0;
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

void sensor_init_mc1080a(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "mc1080a";
	sinfo->i2c_addr			= (0xC0 >> 1);		// 7bit addr is 0x60
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 1920;
	sinfo->prv_h			= 1080;
	sinfo->framerate		= 30;
	sinfo->mclk				= 240000;			// 24Mhz
	sinfo->input_data_fmt	= FMT_YUV422_16BIT;	// YUV422 16bits
	sinfo->input_data_order	= ORDER_RBG;		// RBG - VUY
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= NEGATIVE_EDGE;
	sinfo->polarity_vsync	= ACT_HIGH;
	sinfo->polarity_hsync	= ACT_HIGH;
	sinfo->de2hsync			= DE_FROM_HS_CONNECT;
	sinfo->interface_type	= BT601;
	sinfo->scan_type		= FRAME_NOT_SKIP;
	sinfo->frame_skip_vin	= FRAME_NOT_SKIP;

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
EXPORT_SYMBOL(sensor_init_mc1080a);
