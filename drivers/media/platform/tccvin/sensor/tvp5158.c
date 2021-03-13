/*
 * tvp5158.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is TVP5158 decoder driver.
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
#include "tvp5158.h"


#ifdef CONFIG_TCC_SENSOR_DEBUG
#define DBG_I2C_READ
#define dprintk(msg...)	printk(msg)
#else
#define dprintk(msg...)
#endif


/* Array of image sizes supported.
 * These must be ordered from smallest image size to largest.
 */
static struct capture_size sensor_sizes[] = {
	{720, 480},
	{720, 480},
};

extern int cif_i2c_send(struct i2c_client *client, unsigned char* data, unsigned short reg_bytes, unsigned short data_bytes);
extern int cif_i2c_recv(struct i2c_client *client, unsigned short reg, unsigned char reg_bytes, unsigned char *val, unsigned short val_bytes);	
extern int vincore_set_zoom(struct tcc_video_device *vdev, int arg);


static struct v4l2_ctrl_t _v4l2_ctrl[] = {
};

static unsigned int _v4l2_re_ctrl[] = {
};


int tvp5158_read_enable(struct i2c_client *client, unsigned char value)
{
	unsigned char data[2];

	data[0] = 0xFF;
	data[1] = value;

	return cif_i2c_send(client, data, 1, 1);
}

int tvp5158_write_enable(struct i2c_client *client, unsigned char value)
{
	unsigned char data[2];

	data[0] = 0xFE;
	data[1] = value;

	return cif_i2c_send(client, data, 1, 1);
}

#ifdef DBG_I2C_READ
void tvp5158_reg_dump(struct i2c_client *client)
{
	int ret;
	unsigned char reg, val;

	printk("-----tvp5150 reg dump------------\n");
	ret = tvp5158_read_enable(client, 0x1);
	if (ret < 0)
		printk("%s: error tvp5158_read_enable\n", __func__);

	for (reg = 0x00; reg < 0xff; reg++) {
		ret = cif_i2c_recv(client, reg, 1, &val, 1);
		if (ret < 0)
			printk("0x%02x: read error\n", reg);
		else
			printk("0x%02x: 0x%02x\n", reg, val);
	}
	printk("--------------------------------\n");
}
#endif

int tvp5158_vbus_write(struct i2c_client *client, unsigned int vbus_addr, unsigned char val, int len)
{
	int ret;
	unsigned char data[2];

	/* set vbus address */
	data[0] = 0xE8;
	data[1] = (unsigned char)((vbus_addr >> 0) & 0xff);
	ret = cif_i2c_send(client, data, 1, 1);
	data[0] = 0xE9;
	data[1] = (unsigned char)((vbus_addr >> 8) & 0xff);
	ret |= cif_i2c_send(client, data, 1, 1);
	data[0] = 0xEA;
	data[1] = (unsigned char)((vbus_addr >> 16) & 0xff);
	ret |= cif_i2c_send(client, data, 1, 1);

	if (len) {
		data[0] = 0xE0;
		data[1] = val;
		ret |= cif_i2c_send(client, data, 1, 1);
	}

	return ret;
}

int tvp5158_check_chipid(struct i2c_client *client)
{
	unsigned char reg[2];
	unsigned char val[2];

	reg[0] = 0x08;	// 0x51
	reg[1] = 0x09;	// 0x58

	cif_i2c_recv(client, reg[0], 1, &val[0], 1);
	cif_i2c_recv(client, reg[1], 1, &val[1], 1);
	if (val[0] != 0x51 || val[1] != 0x58) {
		dprintk("%s: tvp%x%x\n", __func__, val[0], val[1]);
		return -1;
	}
	return 0;
}

int tvp5158_ofm_reset(struct i2c_client *client)
{
	int ret;
	unsigned char data[2];

	ret = tvp5158_write_enable(client, 0x1);
	if (ret < 0) goto exit;

	data[0] = 0xB2;
	data[1] = 0;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;

	msleep(10);
exit:
	return ret;
}

int tvp5158_patch_download(struct i2c_client *client)
{
	int ret;
	unsigned char data[2];
	unsigned char vbus_status;
	unsigned char *patch_addr;
	unsigned int patch_size;
	struct i2c_msg msgs;

	ret = tvp5158_write_enable(client, 0x1);
	if (ret < 0) goto exit;

	/* set vbus address */
	data[0] = 0xE8;
	data[1] = 0x60;
	ret |= cif_i2c_send(client, data, 1, 1);
	data[0] = 0xE9;
	data[1] = 0x00;
	ret |= cif_i2c_send(client, data, 1, 1);
	data[0] = 0xEA;
	data[1] = 0xB0;
	ret |= cif_i2c_send(client, data, 1, 1);
	/* read vbus status */
	data[0] = 0xE0;
	ret |= cif_i2c_recv(client, data[0], 1, &vbus_status, 1);
	if (ret < 0) goto exit;

	if (vbus_status & 0x2) {
		dprintk("tvp5158: patch is already running (0x%x)\n", vbus_status);
		goto exit;
	} else {
		dprintk("tvp5158: patch is downloading... (0x%x)\n", vbus_status);
	}

	ret = tvp5158_write_enable(client, 0xF);
	if (ret < 0) goto exit;

	vbus_status |= 0x1;
	ret = tvp5158_vbus_write(client, 0xB00060, vbus_status, 1);
	if (ret < 0) goto exit;

	/* write patch */
	ret = tvp5158_vbus_write(client, 0x400000, 0, 0);
	if (ret < 0) goto exit;

	patch_addr = (unsigned char *)tvp5158_patch;
	patch_size = sizeof(tvp5158_patch);
	msgs.addr = client->addr;
	msgs.flags = 0;
	msgs.buf = patch_addr;
	msgs.len = patch_size;
	ret = i2c_transfer(client->adapter, &msgs, 1);
	if (ret < 0) goto exit;

	dprintk("tvp5158: patch is downloaded(%d). soft reset\n", ret);

	vbus_status |= 0x3;
	ret = tvp5158_vbus_write(client, 0xB00060, vbus_status, 1);
	if (ret < 0) goto exit;

	vbus_status &= ~(0x1);
	ret = tvp5158_vbus_write(client, 0xB00060, vbus_status, 1);
	if (ret < 0) goto exit;

	msleep(300);
exit:
	return ret;
}

int tvp5158_set_regs(struct i2c_client *client, unsigned int channel)
{
	int ret = 0, i, reg;
	unsigned char B[10], data[2];

//#define TEST
#ifdef TEST
/*--- set all decoder ---*/
	ret = tvp5158_write_enable(client, 0xF);
	if (ret < 0) goto exit;

	/* Blue screen (Red color setting)
	 */
	data[0] = 0x90;	// Y
	data[1] = 0x51;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;
	data[0] = 0x91; // Cb
	data[1] = 0x5A;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;
	data[0] = 0x92; // Cr
	data[1] = 0xF0;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;
	data[0] = 0x93; // LSB
	data[1] = 0x00;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;
	data[0] = 0xA9;
	data[1] = 0x44; // Blue screen output when detects lost lock
	//data[1] = 0x48; // Force Blue screen output
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;
	
	/* video mode
	 */
	data[0] = 0x0D;
	data[1] = 0x01;	// (M, J) NTSC
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;

	/* Output formatter control 1
	 */
	//data[0] = 0xA8;
	//data[1] = 0x04;	// ITU-R BT.601 coding range
	//ret = cif_i2c_send(client, data, 1, 1);
	//if (ret < 0) goto exit;

/*--- set only decoder 0 ---*/
	//ret = tvp5158_write_enable(client, 0x1);
	//if (ret < 0) goto exit;

	/* OFM super-frame size
	 */
	//data[0] = 0xB6;
	//data[1] = 0x1B;
	//ret = cif_i2c_send(client, data, 1, 1);
	//if (ret < 0) goto exit;
	//data[0] = 0xB7;
	//data[1] = 0x04;	// 525-line
	//ret = cif_i2c_send(client, data, 1, 1);
	//if (ret < 0) goto exit;
#endif

	/* output/channel/ofm
	 */
	B[1] = 0x14;	//@alanK: Chan_ID_SAVEAV_En(bit[2]) of AVD Output Control 2 register set 1.
	B[2] = 0x25;
	B[3] = 0xE4;
	B[4] = 0xE4;
	B[5] = 0x00;
	B[6] = 0x1B;
	//B[7] = 0x14;	// super-frame size 625			// TODO: org
	B[7] = 0x04;	// super-frame size 525

	switch (channel) {
	case VINDEMUX_MODE_4CH_D1_108MHZ:
		//B[0] = 0xA0;	// line-interleaved			// TODO: org
		B[0] = 0x60;	// pixel-interleaved
		B[8] = 0xF8;
		B[9] = 0x10;
		break;
	case VINDEMUX_MODE_1CH_D1_27MHZ:
		B[0] = 0x00;
		B[8] = 0xF8;
		B[9] = 0x10;
		break;
	case VINDEMUX_MODE_2CH_D1_54MHZ:
		B[0] = 0x50;
		B[2] = 0x25;	// 54Mhz
		B[8] = 0xF8;
		B[9] = 0x10;
		break;
	case VINDEMUX_MODE_2CH_D1_27MHZ:
		B[0] = 0x50;
		B[2] = 0x2D;	// 27Mhz
		B[8] = 0xF8;
		B[9] = 0x10;
		break;
	default:
		printk("tvp5158: Not support channel mode\n");
		ret = -1;
		break;
	}

	ret = tvp5158_write_enable(client, 0xF);
	if (ret < 0) goto exit;

	for (i = 0, reg = 0xB0; i < 10; i++, reg++) {
		if (i == 2) {
			ret = tvp5158_write_enable(client, 0x1);
			if (ret < 0) goto exit;
		}
		data[0] = reg;
		data[1] = B[i];
		ret = cif_i2c_send(client, data, 1, 1);
		if (ret < 0) goto exit;
	}

	data[0] = 0xC3;
	data[1] = 0xE0;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;

	msleep(20);

	data[0] = 0xBA;
	data[1] = 0x01;
	ret = cif_i2c_send(client, data, 1, 1);
	if (ret < 0) goto exit;

#ifdef DBG_I2C_READ
	dprintk("--------------------------------\n");
	ret = tvp5158_read_enable(client, 0x1);
	if (ret < 0) goto exit;
	for (i = 0, reg = 0xB0; i < 10; i++, reg++) {
		data[0] = reg;
		ret = cif_i2c_recv(client, data[0], 1, &data[1], 1);
		if (ret < 0) goto exit;
		dprintk("0x%x: set(0x%x) get(0x%x)\n", reg, B[i], data[1]);
	}
	dprintk("--------------------------------\n");
#endif

exit:
	return ret;
}

static int sensor_open(struct tcc_video_device *vdev)
{
	int ret;

	cif_power_disable(vdev);
	//msleep(10);

	cif_power_enable(vdev);
	//msleep(10);

	cif_powerdown_disable(vdev);
	//msleep(10);

	cif_reset_low(vdev);
	//msleep(10);

	cif_open(vdev);
	//msleep(50);

	cif_reset_high(vdev);
	//msleep(10);

	ret = tvp5158_check_chipid(vdev->cif_i2c_client);
	if (ret < 0) {
		printk("error: tvp5158_check_chipid\n");
		goto exit;
	}

	ret = tvp5158_ofm_reset(vdev->cif_i2c_client);
	if (ret < 0) {
		printk("error: tvp5158_ofm_reset\n");
		goto exit;
	}

	ret = tvp5158_patch_download(vdev->cif_i2c_client);
	if (ret < 0) {
		printk("error: tvp5158_patch_download\n");
		goto exit;
	}

	ret = tvp5158_set_regs(vdev->cif_i2c_client, vdev->sinfo->private);
	if (ret < 0) {
		printk("error: tvp5158_select_channel\n");
		goto exit;
	}

#ifdef DBG_I2C_READ
	tvp5158_reg_dump(vdev->cif_i2c_client);
#endif

exit:
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

static int sensor_v4l2_set_ctrl(struct tcc_video_device *vdev, __u32 id, int val)
{
	int err = -EINVAL;

	switch (id) {
	default:
		break;
	}
	return err;
}
void sensor_init_tvp5158(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "tvp5158";
	sinfo->i2c_addr			= (0xB0 >> 1);
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 720;
	sinfo->prv_h			= 480;
	sinfo->framerate		= 60;
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	sinfo->polarity_pclk 	= NEGATIVE_EDGE;
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

	/* select tvp5158 video output mode */
#if defined(BB_4CH_TYPE5)
	sinfo->private			= VINDEMUX_MODE_4CH_D1_108MHZ;
#elif defined(BB_4CH_TYPE6)
	sinfo->private			= VINDEMUX_MODE_1CH_D1_27MHZ;
#elif defined(BB_4CH_TYPE7)
	sinfo->private			= VINDEMUX_MODE_2CH_D1_54MHZ;
#elif defined(BB_4CH_TYPE8)
	sinfo->private			= VINDEMUX_MODE_2CH_D1_27MHZ;
#else
	#error "tvp5158: choose channel type"
#endif
	switch (sinfo->private & VINDEMUX_CLK_MASK) {
	case VINDEMUX_CLK_4TIMES_FREQ:
		sinfo->mclk			= 1080000;
		break;
	case VINDEMUX_CLK_DOUBLE_EDGE:
		sinfo->mclk			= 270000;
		break;
	case VINDEMUX_CLK_DOUBLE_FREQ:
	case VINDEMUX_CLK_DOUBLE_EDGE_FREQ:
		sinfo->mclk			= 540000;
		break;	
	}

	/* callback func */
	sinfo->open				= sensor_open;
	sinfo->close			= sensor_close;
	sinfo->powerdown		= sensor_powerdown;
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
EXPORT_SYMBOL(sensor_init_tvp5158);
