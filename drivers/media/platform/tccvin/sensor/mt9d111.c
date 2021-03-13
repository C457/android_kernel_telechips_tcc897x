/*
 * mt9d111.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is MT9D111 image sensor driver.
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

#define I2C_DELAY	0xFE	/* delay (msec) */
#define I2C_TERM	0xFF	/* end of list entry */

struct reg_fmt {
	u8 page;		/* page number */
	u8 reg;			/* 8-bit register */
	u16 val;		/* 16-bit value */
};


/* Array of image sizes supported by MT9D111.
 * These must be ordered from smallest image size to largest.
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

enum v4l2_scene {
	V4L2_SCENE_NONE = 0,
	V4L2_SCENE_MONO = 1,
	V4L2_SCENE_SEPIA = 2,
	V4L2_SCENE_NEGATIVE = 3,
	V4L2_SCENE_SOLARIZE1 = 4,
	V4L2_SCENE_SOLARIZE2 = 5,
	V4L2_SCENE_DEFAULT = V4L2_SCENE_NONE,
};

enum v4l2_framerate {
	V4L2_FRAMERATE_15 = 15,
	V4L2_FRAMERATE_20 = 20,
	V4L2_FRAMERATE_24 = 24,
	V4L2_FRAMERATE_25 = 25, 
	V4L2_FRAMERATE_30 = 30,
	V4L2_FRAMERATE_DEFAULT = V4L2_FRAMERATE_30,
};

enum v4l2_rotate {
	V4L2_ORIENTATION_NOMAL = 0,
	V4L2_ORIENTATION_VERTICAL_FLIP = 1,
	V4L2_ORIENTATION_HORIZONTAL_MIRROR = 2,
	V4L2_ORIENTATION_180_DEGREE = 3,
	V4L2_ORIENTATION_DEFAULT = V4L2_ORIENTATION_NOMAL,
};

enum v4l2_saturation {
	V4L2_SATURATION_MINUS_3 = 0,
	V4L2_SATURATION_MINUS_2 = 1,
	V4L2_SATURATION_MINUS_1 = 2,
	V4L2_SATURATION_DEFAULT = 3,
	V4L2_SATURATION_PLUS_1 = 4,
	V4L2_SATURATION_PLUS_2 = 5,
	V4L2_SATURATION_PLUS_3 = 6,
};

enum v4l2_contrast {
	V4L2_CONTRAST_NORMAL = 0,
	V4L2_CONTRAST_SOME = 1,
	V4L2_CONTRAST_MORE = 2,
	V4L2_CONTRAST_MOST = 3,
	V4L2_CONTRAST_NOISEREDUCTION = 4,
	V4L2_CONTRAST_DEFAULT = V4L2_CONTRAST_NORMAL,
};

enum v4l2_exposure {
	V4L2_EXPOSURE_AUTO_ON = 0,
	V4L2_EXPOSURE_12MS = 1,
	V4L2_EXPOSURE_22MS = 2,
	V4L2_EXPOSURE_32MS = 3,
	V4L2_EXPOSURE_42MS = 4,
	V4L2_EXPOSURE_52MS = 5,
	V4L2_EXPOSURE_DEFAULT = V4L2_EXPOSURE_AUTO_ON,
};

enum v4l2_awb {
	V4L2_AWB_ON = 0,
	V4L2_AWB_OFF_2800K = 1,
	V4L2_AWB_OFF_5000K = 2,
	V4L2_AWB_OFF_6500K = 3,
	V4L2_AWB_DEFAULT = V4L2_AWB_ON,
};

static struct v4l2_ctrl_t _v4l2_ctrl[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = 5,
			.step = 1,
			.default_value = V4L2_EXPOSURE_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_EXPOSURE_DEFAULT,
		.need_set = 0,
	},
	{
		{
			.id = V4L2_CID_AUTO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Auto White Balance",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = V4L2_AWB_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_AWB_DEFAULT,
		.need_set = 0,
	},
	{
		{
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = V4L2_CONTRAST_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_CONTRAST_DEFAULT,
		.need_set = 0,
	},
	{
		{
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = V4L2_SATURATION_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_SATURATION_DEFAULT,
		.need_set = 0,
	},
	{
		{
			.id = V4L2_CID_ROTATE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Rotate",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = V4L2_ORIENTATION_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_ORIENTATION_DEFAULT,
		.need_set = 0,
	},
/* Private control */
	{
		{
			.id = V4L2_CID_FRAMERATE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Frame Rate",
			.minimum = 15,
			.maximum = 30,
			.step = 1,
			.default_value = V4L2_FRAMERATE_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_FRAMERATE_DEFAULT,
		.need_set = 0,
	},
	{
		{
			.id = V4L2_CID_SCENE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Scene Mode",
			.minimum = 0,
			.maximum = 5,
			.step = 1,
			.default_value = V4L2_SCENE_DEFAULT,
			.flags = 0,
		},
		.current_value = V4L2_SCENE_DEFAULT,
		.need_set = 0,
	},
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
	V4L2_CID_EXPOSURE,
	V4L2_CID_AUTO_WHITE_BALANCE,
	V4L2_CID_CONTRAST,
	V4L2_CID_SATURATION,
	V4L2_CID_ROTATE,
	V4L2_CID_FRAMERATE,
	V4L2_CID_SCENE,
	V4L2_CID_FRAMESKIP,
};


/* register initialization tables for sensor */
/* common sensor register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
static struct reg_fmt sensor_initialize[] = {
	/* 
	 * @@ Patron 24MHz 80MHz 720P 30FPS 
	 */
	{0, 0xF0, 0x0000},		//Page 0 setting REG = 0
	{0, 0x65, 0xA000},		// bypassed PLL (prepare for soft reset)
	{0, 0xF0, 0x0001},		//Page 0 setting REG = 0
	{0, 0xC3, 0x0501},		// MCU_BOOT_MODE (MCU reset)
	{0, 0xF0, 0x0000},		//Page 0 setting REG = 0
	{0, 0x0D, 0x0021},		// RESET_REG (enable soft reset)
	{I2C_DELAY, 0, 1},	//DELAY=1
	{0, 0x0D, 0x0000},		// RESET_REG (disable soft reset)
	{I2C_DELAY, 0, 100},//DELAY=100
	
	/* 720p30 Timing Settings 24mhz */
	{0, 0xF0, 0x0000},		//Page 0 setting REG = 0
	{0, 0x05, 0x011E},		//HBLANK (B) = 286
	{0, 0x06, 0x00C7},		//VBLANK (B) = 11
	{0, 0x07, 0x011E},		//HBLANK (A) = 286
	{0, 0x08, 0x0077},		//VBLANK (A) = 11
	{0, 0x20, 0x0300},		//Read Mode (B) = 768
	{0, 0x21, 0x0000},		//Read Mode (A) = 0
	{0, 0x66, 0x1402},		//PLL Control 1 = 16394
	{0, 0x67, 0x0500},		//PLL Control 2 = 1280
	{0, 0x65, 0xA000},		//Clock CNTRL: PLL ON = 40960
	{I2C_DELAY, 0, 1},	//DELAY=1
	{0, 0x65, 0x2000},		//Clock CNTRL: USE PLL = 8192
	{I2C_DELAY, 0, 100},//DELAY=100
	{1, 0xF0, 0x0001},		//Page 0 setting REG = 0
	{1, 0xC6, 0x2717},		//Extra Delay (A)
	{1, 0xC8, 0x01C2},		//      = 450
	{1, 0xC6, 0x2719},		//Row Speed (A)
	{1, 0xC8, 0x0011},		//      = 17
	{1, 0xC6, 0x2723},		//Extra Delay (B)
	{1, 0xC8, 0x01C2},		//      = 450
	{1, 0xC6, 0x2725},		//Row Speed (B)
	{1, 0xC8, 0x0011},		//      = 17
	{1, 0xC6, 0x276D},		//FIFO_Conf1 (A)
	{1, 0xC8, 0xE0E1},		//      = 57569
	{1, 0xC6, 0xA76F},		//FIFO_Conf2 (A)
	{1, 0xC8, 0x00E1},		//      = 225
	{1, 0xC6, 0x2774},		//FIFO_Conf1 (B)
	{1, 0xC8, 0xE0E1},		//      = 57569
	{1, 0xC6, 0xA776},		//FIFO_Conf2 (B)
	{1, 0xC8, 0x00E1},		//      = 225
	{1, 0xC6, 0x220B},		//Max R12 (B)(Shutter Delay)
	{1, 0xC8, 0x0459},		//      = 1113
	{1, 0xC6, 0xA217},		//IndexTH23
	{1, 0xC8, 0x0008},		//      = 8
	{1, 0xC6, 0x2228},		//RowTime (msclk per)/4
	{1, 0xC8, 0x0187},		//      = 391
	{1, 0xC6, 0x222F},		//R9 Step
	{1, 0xC8, 0x00B8},		//      = 184
	{I2C_DELAY, 0, 100},//DELAY=100				//bugfix: @alank
	{1, 0xC6, 0xA408},		//search_f1_50
	{1, 0xC8, 0x002B},		//      = 43
	{1, 0xC6, 0xA409},		//search_f2_50
	{1, 0xC8, 0x002D},		//      = 45
	{1, 0xC6, 0xA40A},		//search_f1_60
	{1, 0xC8, 0x0023},		//      = 35
	{1, 0xC6, 0xA40B},		//search_f2_60
	{1, 0xC8, 0x0025},		//      = 37
	{1, 0xC6, 0x2411},		//R9_Step_60
	{1, 0xC8, 0x00B8},		//      = 184
	{1, 0xC6, 0x2413},		//R9_Step_50
	{1, 0xC8, 0x00DD},		//      = 221
	                 				
	/* 720p30 Settings */
	{1, 0xC6, 0x2703},		//Output Width (A)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x2705},		//Output Height (A)
	{1, 0xC8, 0x02D0},		//      = 720
	{1, 0xC6, 0x2707},		//Output Width (B)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x2709},		//Output Height (B)
	{1, 0xC8, 0x02D0},		//      = 720
	{1, 0xC6, 0x270B},		//mode_config
	{1, 0xC8, 0x0030},		//      = 48
	{1, 0xC6, 0x270F},		//Row Start (A)
	{1, 0xC8, 0x010C},		//      = 28+240
	{1, 0xC6, 0x2711},		//Column Start (A)
	{1, 0xC8, 0x00DC},		//      = 60+160
	{1, 0xC6, 0x2713},		//Row Height (A)
	{1, 0xC8, 0x02D0},		//      = 720
	{1, 0xC6, 0x2715},		//Column Width (A)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x271B},		//Row Start (B)
	{1, 0xC8, 0x010C},		//      = 28+240
	{1, 0xC6, 0x271D},		//Column Start (B)
	{1, 0xC8, 0x00DC},		//      = 60+160
	{1, 0xC6, 0x271F},		//Row Height (B)
	{1, 0xC8, 0x02D0},		//      = 720
	{1, 0xC6, 0x2721},		//Column Width (B)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x2727},		//Crop_X0 (A)
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0x2729},		//Crop_X1 (A)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x272B},		//Crop_Y0 (A)
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0x272D},		//Crop_Y1 (A)
	{1, 0xC8, 0x02D0},		//      = 720
	{1, 0xC6, 0x2735},		//Crop_X0 (B)
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0x2737},		//Crop_X1 (B)
	{1, 0xC8, 0x0500},		//      = 1280
	{1, 0xC6, 0x2739},		//Crop_Y0 (B)
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0x273B},		//Crop_Y1 (B)
	{1, 0xC8, 0x02D0},		//      = 720
	
	/* Max 30fps */
	{1, 0xC6, 0xA20E},		// MCU_ADDRESS [AE_MAX_INDEX]
	{1, 0xC8, 0x0004},		// MCU_DATA_0 Zone 4
	{1, 0xC6, 0xA217},		//IndexTH23
	{1, 0xC8, 0x0004},		// 	 = 4
	
	/* Slow down AE & Increase Gate */
	{1, 0xC6, 0xA208},		// MCU_ADDRESS [AE_SKIP_FRAMES]
	{1, 0xC8, 0x0002},		// = 2
	{1, 0xC6, 0xA107},		// MCU_ADDRESS [SEQ_AE_CONTBUFF]
	{1, 0xC8, 0x0004},		// MCU_DATA_0
	{1, 0xC6, 0xA207},		// MCU_ADDRESS [AE_GATE]
	{1, 0xC8, 0x0016},		// MCU_DATA_0
	
	/* slow down AWB */
	{1, 0xC6, 0xA10B},		// MCU_ADDRESS [SEQ_AWB_CONTBUFF]
	{1, 0xC8, 0x0004},		// MCU_DATA_0
	
	/* Tuning */
	/* Sequencer States... */
	{1, 0xC6, 0xA122},		//Enter Preview: Auto Exposure
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA123},		//Enter Preview: Flicker Detection
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA124},		//Enter Preview: Auto White Balance
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA125},		//Enter Preview: Auto Focus
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA126},		//Enter Preview: Histogram
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA127},		//Enter Preview: Strobe Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA128},		//Enter Preview: Skip Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA129},		//In Preview: Auto Exposure
	{1, 0xC8, 0x0003},		//      = 3
	{1, 0xC6, 0xA12A},		//In Preview: Flicker Detection
	{1, 0xC8, 0x0002},		//      = 2
	{1, 0xC6, 0xA12B},		//In Preview: Auto White Balance
	{1, 0xC8, 0x0003},		//      = 3
	{1, 0xC6, 0xA12C},		//In Preview: Auto Focus
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA12D},		//In Preview: Histogram
	{1, 0xC8, 0x0003},		//      = 3
	{1, 0xC6, 0xA12E},		//In Preview: Strobe Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA12F},		//In Preview: Skip Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA130},		//Exit Preview: Auto Exposure
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA131},		//Exit Preview: Flicker Detection
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA132},		//Exit Preview: Auto White Balance
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA133},		//Exit Preview: Auto Focus
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA134},		//Exit Preview: Histogram
	{1, 0xC8, 0x0001},		//      = 1
	{1, 0xC6, 0xA135},		//Exit Preview: Strobe Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA136},		//Exit Preview: Skip Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA137},		//Capture: Auto Exposure
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA138},		//Capture: Flicker Detection
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA139},		//Capture: Auto White Balance
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA13A},		//Capture: Auto Focus
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA13B},		//Capture: Histogram
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA13C},		//Capture: Strobe Control
	{1, 0xC8, 0x0000},		//      = 0
	{1, 0xC6, 0xA13D},		//Capture: Skip Control
	{1, 0xC8, 0x0000},		//      = 0
	
	/* Gamma settings */
	{1, 0xC6, 0xA743},		// MCU_ADDRESS [MODE_GAM_CONT_A]
	{1, 0xC8, 0x0003},		// MCU_DATA_0
	{1, 0xC6, 0xA743},		// MCU_ADDRESS [MODE_GAM_CONT_A]
	{1, 0xC8, 0x0003},		// MCU_DATA_0
	{1, 0xC6, 0xA745},		// MCU_ADDRESS [MODE_GAM_TABLE_A_0]
	{1, 0xC8, 0x0000},		// MCU_DATA_0
	{1, 0xC6, 0xA746},		// MCU_ADDRESS [MODE_GAM_TABLE_A_1]
	{1, 0xC8, 0x0006},		// MCU_DATA_0
	{1, 0xC6, 0xA747},		// MCU_ADDRESS [MODE_GAM_TABLE_A_2]
	{1, 0xC8, 0x0012},		// MCU_DATA_0
	{1, 0xC6, 0xA748},		// MCU_ADDRESS [MODE_GAM_TABLE_A_3]
	{1, 0xC8, 0x002F},		// MCU_DATA_0
	{1, 0xC6, 0xA749},		// MCU_ADDRESS [MODE_GAM_TABLE_A_4]
	{1, 0xC8, 0x0053},		// MCU_DATA_0
	{1, 0xC6, 0xA74A},		// MCU_ADDRESS [MODE_GAM_TABLE_A_5]
	{1, 0xC8, 0x006D},		// MCU_DATA_0
	{1, 0xC6, 0xA74B},		// MCU_ADDRESS [MODE_GAM_TABLE_A_6]
	{1, 0xC8, 0x0083},		// MCU_DATA_0
	{1, 0xC6, 0xA74C},		// MCU_ADDRESS [MODE_GAM_TABLE_A_7]
	{1, 0xC8, 0x0096},		// MCU_DATA_0
	{1, 0xC6, 0xA74D},		// MCU_ADDRESS [MODE_GAM_TABLE_A_8]
	{1, 0xC8, 0x00A6},		// MCU_DATA_0
	{1, 0xC6, 0xA74E},		// MCU_ADDRESS [MODE_GAM_TABLE_A_9]
	{1, 0xC8, 0x00B3},		// MCU_DATA_0
	{1, 0xC6, 0xA74F},		// MCU_ADDRESS [MODE_GAM_TABLE_A_10]
	{1, 0xC8, 0x00BF},		// MCU_DATA_0
	{1, 0xC6, 0xA750},		// MCU_ADDRESS [MODE_GAM_TABLE_A_11]
	{1, 0xC8, 0x00CA},		// MCU_DATA_0
	{1, 0xC6, 0xA751},		// MCU_ADDRESS [MODE_GAM_TABLE_A_12]
	{1, 0xC8, 0x00D3},		// MCU_DATA_0
	{1, 0xC6, 0xA752},		// MCU_ADDRESS [MODE_GAM_TABLE_A_13]
	{1, 0xC8, 0x00DC},		// MCU_DATA_0
	{1, 0xC6, 0xA753},		// MCU_ADDRESS [MODE_GAM_TABLE_A_14]
	{1, 0xC8, 0x00E4},		// MCU_DATA_0
	{1, 0xC6, 0xA754},		// MCU_ADDRESS [MODE_GAM_TABLE_A_15]
	{1, 0xC8, 0x00EB},		// MCU_DATA_0
	{1, 0xC6, 0xA755},		// MCU_ADDRESS [MODE_GAM_TABLE_A_16]
	{1, 0xC8, 0x00F2},		// MCU_DATA_0
	{1, 0xC6, 0xA756},		// MCU_ADDRESS [MODE_GAM_TABLE_A_17]
	{1, 0xC8, 0x00F9},		// MCU_DATA_0
	{1, 0xC6, 0xA757},		// MCU_ADDRESS [MODE_GAM_TABLE_A_18]
	{1, 0xC8, 0x00FF},		// MCU_DATA_0
	{1, 0xC6, 0xA103},		// MCU_ADDRESS [SEQ_CMD]
	{1, 0xC8, 0x0005},		// MCU_DATA_0
	
	{1, 0xC6, 0xA744},		// MCU_ADDRESS [MODE_GAM_CONT_B]
	{1, 0xC8, 0x0003},		// MCU_DATA_0
	{1, 0xC6, 0xA744},		// MCU_ADDRESS [MODE_GAM_CONT_B]
	{1, 0xC8, 0x0003},		// MCU_DATA_0
	{1, 0xC6, 0xA758},		// MCU_ADDRESS [MODE_GAM_TABLE_B_0]
	{1, 0xC8, 0x0000},		// MCU_DATA_0
	{1, 0xC6, 0xA759},		// MCU_ADDRESS [MODE_GAM_TABLE_B_1]
	{1, 0xC8, 0x0006},		// MCU_DATA_0
	{1, 0xC6, 0xA75A},		// MCU_ADDRESS [MODE_GAM_TABLE_B_2]
	{1, 0xC8, 0x0012},		// MCU_DATA_0
	{1, 0xC6, 0xA75B},		// MCU_ADDRESS [MODE_GAM_TABLE_B_3]
	{1, 0xC8, 0x002F},		// MCU_DATA_0
	{1, 0xC6, 0xA75C},		// MCU_ADDRESS [MODE_GAM_TABLE_B_4]
	{1, 0xC8, 0x0053},		// MCU_DATA_0
	{1, 0xC6, 0xA75D},		// MCU_ADDRESS [MODE_GAM_TABLE_B_5]
	{1, 0xC8, 0x006D},		// MCU_DATA_0
	{1, 0xC6, 0xA75E},		// MCU_ADDRESS [MODE_GAM_TABLE_B_6]
	{1, 0xC8, 0x0083},		// MCU_DATA_0
	{1, 0xC6, 0xA75F},		// MCU_ADDRESS [MODE_GAM_TABLE_B_7]
	{1, 0xC8, 0x0096},		// MCU_DATA_0
	{1, 0xC6, 0xA760},		// MCU_ADDRESS [MODE_GAM_TABLE_B_8]
	{1, 0xC8, 0x00A6},		// MCU_DATA_0
	{1, 0xC6, 0xA761},		// MCU_ADDRESS [MODE_GAM_TABLE_B_9]
	{1, 0xC8, 0x00B3},		// MCU_DATA_0
	{1, 0xC6, 0xA762},		// MCU_ADDRESS [MODE_GAM_TABLE_B_10]
	{1, 0xC8, 0x00BF},		// MCU_DATA_0
	{1, 0xC6, 0xA763},		// MCU_ADDRESS [MODE_GAM_TABLE_B_11]
	{1, 0xC8, 0x00CA},		// MCU_DATA_0
	{1, 0xC6, 0xA764},		// MCU_ADDRESS [MODE_GAM_TABLE_B_12]
	{1, 0xC8, 0x00D3},		// MCU_DATA_0
	{1, 0xC6, 0xA765},		// MCU_ADDRESS [MODE_GAM_TABLE_B_13]
	{1, 0xC8, 0x00DC},		// MCU_DATA_0
	{1, 0xC6, 0xA766},		// MCU_ADDRESS [MODE_GAM_TABLE_B_14]
	{1, 0xC8, 0x00E4},		// MCU_DATA_0
	{1, 0xC6, 0xA767},		// MCU_ADDRESS [MODE_GAM_TABLE_B_15]
	{1, 0xC8, 0x00EB},		// MCU_DATA_0
	{1, 0xC6, 0xA768},		// MCU_ADDRESS [MODE_GAM_TABLE_B_16]
	{1, 0xC8, 0x00F2},		// MCU_DATA_0
	{1, 0xC6, 0xA769},		// MCU_ADDRESS [MODE_GAM_TABLE_B_17]
	{1, 0xC8, 0x00F9},		// MCU_DATA_0
	{1, 0xC6, 0xA76A},		// MCU_ADDRESS [MODE_GAM_TABLE_B_18]
	{1, 0xC8, 0x00FF},		// MCU_DATA_0
	{1, 0xC6, 0xA103},		// MCU_ADDRESS [SEQ_CMD]
	{1, 0xC8, 0x0005},		// MCU_DATA_0

	/* LENS SHADING (20120614) */
	{0, 0xF0, 0x0002},		//Page 2 setting REG = 2
	{2, 0x80, 0x01F0},		// LENS_CORRECTION_CONTROL
	{2, 0x81, 0x6432},		// ZONE_BOUNDS_X1_X2
	{2, 0x82, 0x3296},		// ZONE_BOUNDS_X0_X3
	{2, 0x83, 0x9664},		// ZONE_BOUNDS_X4_X5
	{2, 0x84, 0x5028},		// ZONE_BOUNDS_Y1_Y2
	{2, 0x85, 0x2878},		// ZONE_BOUNDS_Y0_Y3
	{2, 0x86, 0x7850},		// ZONE_BOUNDS_Y4_Y5
	{2, 0x87, 0x0000},		// CENTER_OFFSET
	{2, 0x88, 0x0088},		// FX_RED
	{2, 0x8B, 0x004F},		// FY_RED
	{2, 0x8E, 0x0ADD},		// DF_DX_RED
	{2, 0x91, 0x0CAD},		// DF_DY_RED
	{2, 0x94, 0xF835},		// SECOND_DERIV_ZONE_0_RED
	{2, 0x97, 0x2831},		// SECOND_DERIV_ZONE_1_RED
	{2, 0x9A, 0x0F28},		// SECOND_DERIV_ZONE_2_RED
	{2, 0x9D, 0x2B48},		// SECOND_DERIV_ZONE_3_RED
	{2, 0xA0, 0x1C3A},		// SECOND_DERIV_ZONE_4_RED
	{2, 0xA3, 0x1C2D},		// SECOND_DERIV_ZONE_5_RED
	{2, 0xA6, 0x0A23},		// SECOND_DERIV_ZONE_6_RED
	{2, 0xA9, 0x4552},		// SECOND_DERIV_ZONE_7_RED
	{2, 0x89, 0x0071},		// FX_GREEN
	{2, 0x8C, 0x0048},		// FY_GREEN
	{2, 0x8F, 0x0BB1},		// DF_DX_GREEN
	{2, 0x92, 0x0CA9},		// DF_DY_GREEN
	{2, 0x95, 0x152D},		// SECOND_DERIV_ZONE_0_GREEN
	{2, 0x98, 0x1A2E},		// SECOND_DERIV_ZONE_1_GREEN
	{2, 0x9B, 0xFF17},		// SECOND_DERIV_ZONE_2_GREEN
	{2, 0x9E, 0x3248},		// SECOND_DERIV_ZONE_3_GREEN
	{2, 0xA1, 0x1A2B},		// SECOND_DERIV_ZONE_4_GREEN
	{2, 0xA4, 0x1424},		// SECOND_DERIV_ZONE_5_GREEN
	{2, 0xA7, 0xF113},		// SECOND_DERIV_ZONE_6_GREEN
	{2, 0xAA, 0x3130},		// SECOND_DERIV_ZONE_7_GREEN
	{2, 0x8A, 0x0064},		// FX_BLUE
	{2, 0x8D, 0x0049},		// FY_BLUE
	{2, 0x90, 0x0BB5},		// DF_DX_BLUE
	{2, 0x93, 0x0D4D},		// DF_DY_BLUE
	{2, 0x96, 0x0342},		// SECOND_DERIV_ZONE_0_BLUE
	{2, 0x99, 0x192B},		// SECOND_DERIV_ZONE_1_BLUE
	{2, 0x9C, 0xF708},		// SECOND_DERIV_ZONE_2_BLUE
	{2, 0x9F, 0x2C43},		// SECOND_DERIV_ZONE_3_BLUE
	{2, 0xA2, 0x151B},		// SECOND_DERIV_ZONE_4_BLUE
	{2, 0xA5, 0x1225},		// SECOND_DERIV_ZONE_5_BLUE
	{2, 0xA8, 0x0522},		// SECOND_DERIV_ZONE_6_BLUE
	{2, 0xAB, 0xF927},		// SECOND_DERIV_ZONE_7_BLUE
	{2, 0xAC, 0x0080},		// X2_FACTORS
	{2, 0xAD, 0x0000},		// GLOBAL_OFFSET_FXY_FUNCTION
	{2, 0xAE, 0x018E},		// K_FACTOR_IN_K_FX_FY
	{0, 0xF0, 0x0001},		//Page 1 setting REG = 1
	{1, 0x08, 0x01FC},		// COLOR_PIPELINE_CONTROL

	/* AWB, CCM (20120614) */
	{1, 0xC6, 0x2306},		// MCU_ADDRESS [AWB_CCM_L_0]
	{1, 0xC8, 0x0219},		// MCU_DATA_0
	{1, 0xC6, 0x2308},		// MCU_ADDRESS [AWB_CCM_L_1]
	{1, 0xC8, 0xFEEC},		// MCU_DATA_0
	{1, 0xC6, 0x230A},		// MCU_ADDRESS [AWB_CCM_L_2]
	{1, 0xC8, 0xFFFF},		// MCU_DATA_0
	{1, 0xC6, 0x230C},		// MCU_ADDRESS [AWB_CCM_L_3]
	{1, 0xC8, 0xFF6A},		// MCU_DATA_0
	{1, 0xC6, 0x230E},		// MCU_ADDRESS [AWB_CCM_L_4]
	{1, 0xC8, 0x01D4},		// MCU_DATA_0
	{1, 0xC6, 0x2310},		// MCU_ADDRESS [AWB_CCM_L_5]
	{1, 0xC8, 0xFFC9},		// MCU_DATA_0
	{1, 0xC6, 0x2312},		// MCU_ADDRESS [AWB_CCM_L_6]
	{1, 0xC8, 0xFF71},		// MCU_DATA_0
	{1, 0xC6, 0x2314},		// MCU_ADDRESS [AWB_CCM_L_7]
	{1, 0xC8, 0xFDD3},		// MCU_DATA_0
	{1, 0xC6, 0x2316},		// MCU_ADDRESS [AWB_CCM_L_8]
	{1, 0xC8, 0x03ED},		// MCU_DATA_0
	{1, 0xC6, 0x2318},		// MCU_ADDRESS [AWB_CCM_L_9]
	{1, 0xC8, 0x0020},		// MCU_DATA_0
	{1, 0xC6, 0x231A},		// MCU_ADDRESS [AWB_CCM_L_10]
	{1, 0xC8, 0x0035},		// MCU_DATA_0
	{1, 0xC6, 0x231C},		// MCU_ADDRESS [AWB_CCM_RL_0]
	{1, 0xC8, 0xFF92},		// MCU_DATA_0
	{1, 0xC6, 0x231E},		// MCU_ADDRESS [AWB_CCM_RL_1]
	{1, 0xC8, 0x0098},		// MCU_DATA_0
	{1, 0xC6, 0x2320},		// MCU_ADDRESS [AWB_CCM_RL_2]
	{1, 0xC8, 0xFFE1},		// MCU_DATA_0
	{1, 0xC6, 0x2322},		// MCU_ADDRESS [AWB_CCM_RL_3]
	{1, 0xC8, 0x0014},		// MCU_DATA_0
	{1, 0xC6, 0x2324},		// MCU_ADDRESS [AWB_CCM_RL_4]
	{1, 0xC8, 0xFFFC},		// MCU_DATA_0
	{1, 0xC6, 0x2326},		// MCU_ADDRESS [AWB_CCM_RL_5]
	{1, 0xC8, 0xFFF2},		// MCU_DATA_0
	{1, 0xC6, 0x2328},		// MCU_ADDRESS [AWB_CCM_RL_6]
	{1, 0xC8, 0x004E},		// MCU_DATA_0
	{1, 0xC6, 0x232A},		// MCU_ADDRESS [AWB_CCM_RL_7]
	{1, 0xC8, 0x0148},		// MCU_DATA_0
	{1, 0xC6, 0x232C},		// MCU_ADDRESS [AWB_CCM_RL_8]
	{1, 0xC8, 0xFE3F},		// MCU_DATA_0
	{1, 0xC6, 0x232E},		// MCU_ADDRESS [AWB_CCM_RL_9]
	{1, 0xC8, 0x000A},		// MCU_DATA_0
	{1, 0xC6, 0x2330},		// MCU_ADDRESS [AWB_CCM_RL_10]
	{1, 0xC8, 0xFFF1},		// MCU_DATA_0
	{1, 0xC6, 0xA364},		// MCU_ADDRESS [AWB_KR_L]
	{1, 0xC8, 0x009D},		// MCU_DATA_0
	{1, 0xC6, 0xA365},		// MCU_ADDRESS [AWB_KG_L]
	{1, 0xC8, 0x0096},		// MCU_DATA_0
	{1, 0xC6, 0xA366},		// MCU_ADDRESS [AWB_KB_L]
	{1, 0xC8, 0x0080},		// MCU_DATA_0
	{1, 0xC6, 0xA367},		// MCU_ADDRESS [AWB_KR_R]
	{1, 0xC8, 0x0089},		// MCU_DATA_0
	{1, 0xC6, 0xA368},		// MCU_ADDRESS [AWB_KG_R]
	{1, 0xC8, 0x0080},		// MCU_DATA_0
	{1, 0xC6, 0xA369},		// MCU_ADDRESS [AWB_KB_R]
	{1, 0xC8, 0x0071},		// MCU_DATA_0

	{I2C_DELAY, 0, 250},//DELAY = 250
	{1, 0xC6, 0xA103},		//Refresh Sequencer Mode
	{1, 0xC8, 0x0006},		//      = 6
	{I2C_DELAY, 0, 250},//DELAY = 250
	{1, 0xC6, 0xA103},		//Refresh Sequencer
	{1, 0xC8, 0x0005},		//      = 5

	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_short_init[] = {
	/* 
	 * This code is last part of sensor_initialize[]
	 * for booting-time optimize.
	 */
	//{0, 0xF0, 0x0001},		//Page 1 setting REG = 1
	{1, 0xC6, 0xA103},		//Refresh Sequencer
	{1, 0xC8, 0x0005},		//      = 5

	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_set_preview[] = {
	{I2C_TERM, 0, 0}
};

static struct reg_fmt *sensor_reg_common[3] = {
	sensor_initialize,
	sensor_set_preview,
	sensor_short_init
};

static int write_regs(struct i2c_client *client, const struct reg_fmt reglist[])
{
	int err, err_cnt = 0;
	u8 data[4];
	u8 bytes = 0;
	//u8 page = 0xff;
	const struct reg_fmt *next = reglist;

	while (next->page != I2C_TERM) {
		if (next->page == I2C_DELAY) {
			mdelay(next->val * 2);
			dprintk("               mdelay(%d)\n", next->val);
			next++;
		} else {
			/* 8-bit reg, 16-bit data write sequence 
			 * Start-0xBA-A-Reg-A-Val(MSByte)-A-Val(LSByte)-A-Stop
			 */
again_reg:
			bytes = 0;
			data[bytes++] = next->reg;
			data[bytes++] = (u8)(next->val >> 8);
			data[bytes++] = (u8)(next->val & 0xff);
			err = cif_i2c_send(client, data, 1, bytes - 1);
			if (err) {
				printk("error: i2c_send{0x%02X, 0x%02X%02X} try(%d)\n", data[0], data[1], data[2], err_cnt);
				if (err_cnt++ >= 3) return err;
				else goto again_reg;
			}
#ifdef DBG_I2C_READ
			{/* READ TEST
			  * 16-bit read sequence
			  * Start-0xBA-A-Reg-A-Start-0xBB-A-Read(MSByte)-A-Read(LSByte)-NAck-Stop
			  */
				u8 val[2];
				if (data[0] != 0xF0) {
					err = cif_i2c_recv(client, (u16)next->reg, 1, &val[0], 2);
					if (err) dprintk("error: i2c_recv{0x%02X, 0x%02X%02X}\n", next->reg, val[0], val[1]);
					dprintk("0x%02X: 0x%04X = 0x%02X%02X", next->reg, next->val, val[0], val[1]);

					if ((data[1] != val[0]) || (data[2] != val[1])) {
						if (data[0] == 0x0D)
							dprintk("\e[31m reset \e[0m \n");
						else
							dprintk("\e[31m mismatch! \e[0m \n");
					} else {
						dprintk("\n");
					}
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
	int ret = -1;

	if (is_cam_init_by_lk(CAM_INIT_MT9D111)) {
		clear_cam_init_by_lk(CAM_INIT_MT9D111);

		cif_open(vdev);
		//msleep(50);

		ret =  write_regs(vdev->cif_i2c_client, sensor_reg_common[2]);
	} else {
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

		ret =  write_regs(vdev->cif_i2c_client, sensor_reg_common[0]);
	}

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


static struct reg_fmt sensor_reg_orientation[] = {
	/* orientation: normal */
	{0, 0xF0, 0x0000},			//Page 0 setting REG = 0
	{0, 0x20, 0x0300},			// READ_MODE_B
	{I2C_DELAY, 0, 250},	// TODO: reduce delay
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_saturation[] = {
	/* saturation: default */
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA352},			// MCU_ADDRESS [AWB_SATURATION]
	{1, 0xC8, 0x0080},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_contrast[] = {
	/* contrast: none */
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA743},			// MCU_ADDRESS [MODE_GAM_CONT_A]
	{1, 0xC8, 0x0003},			// MCU_DATA_0
	{1, 0xC6, 0xA103},			// MCU_ADDRESS [SEQ_CMD]
	{1, 0xC8, 0x0005},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_scene[] = {
	/* scene mode: off */
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0x277F},			// MCU_ADDRESS [MODE_SPEC_EFFECTS_A]
	{1, 0xC8, 0x6440},			// MCU_DATA_0
	{1, 0xC6, 0x2781},			// MCU_ADDRESS [MODE_SPEC_EFFECTS_B]
	{1, 0xC8, 0x6440},			// MCU_DATA_0
	{1, 0xC6, 0xA103},			// MCU_ADDRESS [SEQ_CMD]
	{1, 0xC8, 0x0005},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_framerate_30fps[] = {
	/* framerate: fix 30fps */
	{0, 0xF0, 0x0000},			//Page 0 setting REG = 0
	{0, 0x05, 0x011E},			// HBLANK (B) = 286
	{0, 0x06, 0x00C7},			// VBLANK (B) = 11
	{0, 0x07, 0x011E},			// HBLANK (A) = 286
	{0, 0x08, 0x0077},			// VBLANK (A) = 11  **
	{0, 0x20, 0x0300},			// Read Mode (B) = 768
	{0, 0x21, 0x0000},			// Read Mode (A) = 0
	{0, 0x66, 0x1402},			// PLL Control 1 = 16394
	{0, 0x67, 0x0500},			// PLL Control 2 = 1280
	{0, 0x65, 0xA000},			// Clock CNTRL: PLL ON = 40960
	{I2C_DELAY, 0, 1},
	{0, 0x65, 0x2000},			// Clock CNTRL: USE PLL = 8192
	{I2C_DELAY, 0, 100},
 	{0, 0x07, 0x011E},			// HBLANK (A) = 286
 	{0, 0x08, 0x0077},			//  VERT_BLANK_A
	{0, 0xF0, 0x0001},			//Page 0 setting REG = 0
 	{1, 0xC6, 0x2717},			//  MCU_ADDRESS [MODE_SENSOR_X_DELAY_A]
 	{1, 0xC8, 0x01C2},			//  MCU_DATA_0
 	{1, 0xC6, 0x220B},			// Max R12 (B)(Shutter Delay)
 	{1, 0xC8, 0x0459},			//       = 1113
 	{1, 0xC6, 0xA217},			// IndexTH23
 	{1, 0xC8, 0x0008},			//       = 8
 	{1, 0xC6, 0x2228},			// RowTime (msclk per)/4
 	{1, 0xC8, 0x0187},			//       = 391
 	{1, 0xC6, 0x222F},			// R9 Step
 	{1, 0xC8, 0x00B8},			//       = 184
 	{1, 0xC6, 0xA408},			// search_f1_50
 	{1, 0xC8, 0x002B},			//       = 43
 	{1, 0xC6, 0xA409},			// search_f2_50
 	{1, 0xC8, 0x002D},			//       = 45
 	{1, 0xC6, 0xA40A},			// search_f1_60
 	{1, 0xC8, 0x0023},			//       = 35
 	{1, 0xC6, 0xA40B},			// search_f2_60
 	{1, 0xC8, 0x0025},			//       = 37
 	{1, 0xC6, 0x2411},			// R9_Step_60
 	{1, 0xC8, 0x00B8},			//       = 184
 	{1, 0xC6, 0x2413},			// R9_Step_50
 	{1, 0xC8, 0x00DD},			//       = 221
	{I2C_DELAY, 0, 250},
	{1, 0xC6, 0xA103},			// Refresh Sequencer
	{1, 0xC8, 0x0005},			//       = 5
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_framerate_25fps[] = {
	/* framerate: fix 25fps */
	{0, 0xF0, 0x0000},			//Page 0 setting REG = 0
	{0, 0x05, 0x011E},			// HBLANK (B) = 286
	{0, 0x06, 0x00C7},			// VBLANK (B) = 11
	{0, 0x07, 0x01B4},			// HBLANK (A) = 286
	{0, 0x08, 0x00C8},			// VBLANK (A) = 11  **
	{0, 0x20, 0x0300},			// Read Mode (B) = 768
	{0, 0x21, 0x0000},			// Read Mode (A) = 0
	{0, 0x66, 0x1402},			// PLL Control 1 = 16394
	{0, 0x67, 0x0500},			// PLL Control 2 = 1280
	{0, 0x65, 0xA000},			// Clock CNTRL: PLL ON = 40960
	{I2C_DELAY, 0, 1},
	{0, 0x65, 0x2000},			// Clock CNTRL: USE PLL = 8192
	{I2C_DELAY, 0, 100},
	{0, 0x07, 0x01B4},			// HBLANK (A) = 436
	{0, 0x08, 0x00C8},			// VBLANK (A) = 200
	{0, 0xF0, 0x0001},			//Page 0 setting REG = 0
	{1, 0xC6, 0x2717},			// Extra Delay (A)
	{1, 0xC8, 0x0080},			//       = 128
	{1, 0xC6, 0x220B},			// Max R12 (B)(Shutter Delay)
	{1, 0xC8, 0x04EF},			//       = 1263
	{1, 0xC6, 0xA217},			// IndexTH23
	{1, 0xC8, 0x0008},			//       = 8
	{1, 0xC6, 0x2228},			// RowTime (msclk per)/4
	{1, 0xC8, 0x01AD},			//       = 429
	{1, 0xC6, 0x222F},			// R9 Step
	{1, 0xC8, 0x00C1},			//       = 193
	{1, 0xC6, 0xA408},			// search_f1_50
	{1, 0xC8, 0x002D},			//       = 45
	{1, 0xC6, 0xA409},			// search_f2_50
	{1, 0xC8, 0x002F},			//       = 47
	{1, 0xC6, 0xA40A},			// search_f1_60
	{1, 0xC8, 0x0025},			//       = 37
	{1, 0xC6, 0xA40B},			// search_f2_60
	{1, 0xC8, 0x0027},			//       = 39
	{1, 0xC6, 0x2411},			// R9_Step_60
	{1, 0xC8, 0x00C1},			//       = 193
	{1, 0xC6, 0x2413},			// R9_Step_50
	{1, 0xC8, 0x00E8},			//       = 232
	{I2C_DELAY, 0, 250},
	{1, 0xC6, 0xA103},			// Refresh Sequencer
	{1, 0xC8, 0x0005},			//       = 5
	{I2C_TERM, 0, 0}
};

static struct reg_fmt *sensor_reg_framerate[2] = {
	sensor_reg_framerate_30fps,
	sensor_reg_framerate_25fps
};

static struct reg_fmt sensor_reg_exposure_auto[] = {
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000F},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_exposure_ms[] = {
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000E},			// MCU_DATA_0
	{1, 0xC6, 0x2225},			// MCU_ADDRESS [AE_R9]
	{1, 0xC8, 0x0132},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt *sensor_reg_exposure[2] = {
	sensor_reg_exposure_auto,
	sensor_reg_exposure_ms
};

static struct reg_fmt sensor_reg_awb_on[] = {
	/* auto white balance: on */
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000F},			// MCU_DATA_0
	// AWB, CCM
	{1, 0xC6, 0x2306},			// MCU_ADDRESS [AWB_CCM_L_0]
	{1, 0xC8, 0x0219},			// MCU_DATA_0
	{1, 0xC6, 0x2308},			// MCU_ADDRESS [AWB_CCM_L_1]
	{1, 0xC8, 0xFEEC},			// MCU_DATA_0
	{1, 0xC6, 0x230A},			// MCU_ADDRESS [AWB_CCM_L_2]
	{1, 0xC8, 0xFFFF},			// MCU_DATA_0
	{1, 0xC6, 0x230C},			// MCU_ADDRESS [AWB_CCM_L_3]
	{1, 0xC8, 0xFF6A},			// MCU_DATA_0
	{1, 0xC6, 0x230E},			// MCU_ADDRESS [AWB_CCM_L_4]
	{1, 0xC8, 0x01D4},			// MCU_DATA_0
	{1, 0xC6, 0x2310},			// MCU_ADDRESS [AWB_CCM_L_5]
	{1, 0xC8, 0xFFC9},			// MCU_DATA_0
	{1, 0xC6, 0x2312},			// MCU_ADDRESS [AWB_CCM_L_6]
	{1, 0xC8, 0xFF71},			// MCU_DATA_0
	{1, 0xC6, 0x2314},			// MCU_ADDRESS [AWB_CCM_L_7]
	{1, 0xC8, 0xFDD3},			// MCU_DATA_0
	{1, 0xC6, 0x2316},			// MCU_ADDRESS [AWB_CCM_L_8]
	{1, 0xC8, 0x03ED},			// MCU_DATA_0
	{1, 0xC6, 0x2318},			// MCU_ADDRESS [AWB_CCM_L_9]
	{1, 0xC8, 0x0020},			// MCU_DATA_0
	{1, 0xC6, 0x231A},			// MCU_ADDRESS [AWB_CCM_L_10]
	{1, 0xC8, 0x0035},			// MCU_DATA_0
	{1, 0xC6, 0x231C},			// MCU_ADDRESS [AWB_CCM_RL_0]
	{1, 0xC8, 0xFF92},			// MCU_DATA_0
	{1, 0xC6, 0x231E},			// MCU_ADDRESS [AWB_CCM_RL_1]
	{1, 0xC8, 0x0098},			// MCU_DATA_0
	{1, 0xC6, 0x2320},			// MCU_ADDRESS [AWB_CCM_RL_2]
	{1, 0xC8, 0xFFE1},			// MCU_DATA_0
	{1, 0xC6, 0x2322},			// MCU_ADDRESS [AWB_CCM_RL_3]
	{1, 0xC8, 0x0014},			// MCU_DATA_0
	{1, 0xC6, 0x2324},			// MCU_ADDRESS [AWB_CCM_RL_4]
	{1, 0xC8, 0xFFFC},			// MCU_DATA_0
	{1, 0xC6, 0x2326},			// MCU_ADDRESS [AWB_CCM_RL_5]
	{1, 0xC8, 0xFFF2},			// MCU_DATA_0
	{1, 0xC6, 0x2328},			// MCU_ADDRESS [AWB_CCM_RL_6]
	{1, 0xC8, 0x004E},			// MCU_DATA_0
	{1, 0xC6, 0x232A},			// MCU_ADDRESS [AWB_CCM_RL_7]
	{1, 0xC8, 0x0148},			// MCU_DATA_0
	{1, 0xC6, 0x232C},			// MCU_ADDRESS [AWB_CCM_RL_8]
	{1, 0xC8, 0xFE3F},			// MCU_DATA_0
	{1, 0xC6, 0x232E},			// MCU_ADDRESS [AWB_CCM_RL_9]
	{1, 0xC8, 0x000A},			// MCU_DATA_0
	{1, 0xC6, 0x2330},			// MCU_ADDRESS [AWB_CCM_RL_10]
	{1, 0xC8, 0xFFF1},			// MCU_DATA_0
	{1, 0xC6, 0xA364},			// MCU_ADDRESS [AWB_KR_L]
	{1, 0xC8, 0x009D},			// MCU_DATA_0
	{1, 0xC6, 0xA365},			// MCU_ADDRESS [AWB_KG_L]
	{1, 0xC8, 0x0096},			// MCU_DATA_0
	{1, 0xC6, 0xA366},			// MCU_ADDRESS [AWB_KB_L]
	{1, 0xC8, 0x0080},			// MCU_DATA_0
	{1, 0xC6, 0xA367},			// MCU_ADDRESS [AWB_KR_R]
	{1, 0xC8, 0x0089},			// MCU_DATA_0
	{1, 0xC6, 0xA368},			// MCU_ADDRESS [AWB_KG_R]
	{1, 0xC8, 0x0080},			// MCU_DATA_0
	{1, 0xC6, 0xA369},			// MCU_ADDRESS [AWB_KB_R]
	{1, 0xC8, 0x0071},			// MCU_DATA_0

	{1, 0xC6, 0xA103},			// Refresh Sequencer
	{1, 0xC8, 0x0005},			//      = 5
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_awb_2800k[] = {
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000B},			// MCU_DATA_0
	{1, 0xC6, 0xA351},			// MCU_ADDRESS [AWB_CCM_POSITION]
	{1, 0xC8, 0x0000},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_awb_5000k[] = {
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA351},			// MCU_ADDRESS [AWB_CCM_POSITION]
	{1, 0xC8, 0x0057},			// MCU_DATA_0
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000B},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt sensor_reg_awb_6500k[] = {
	{1, 0xF0, 0x0001},			//Page 1 setting REG = 1
	{1, 0xC6, 0xA102},			// MCU_ADDRESS [SEQ_MODE]
	{1, 0xC8, 0x000B},			// MCU_DATA_0
	{1, 0xC6, 0xA351},			// MCU_ADDRESS [AWB_CCM_POSITION]
	{1, 0xC8, 0x007F},			// MCU_DATA_0
	{I2C_TERM, 0, 0}
};

static struct reg_fmt *sensor_reg_awb[4] = {
	sensor_reg_awb_on,
	sensor_reg_awb_2800k,
	sensor_reg_awb_5000k,
	sensor_reg_awb_6500k
};

static int v4l2_exposure(struct tcc_video_device *vdev, int val)
{
	int ret = -1;
	struct reg_fmt *reg;
	u16 ae_r9;

	if (val == V4L2_EXPOSURE_AUTO_ON) {
		reg = sensor_reg_exposure[0];
	} else {
		switch (val) {
			case V4L2_EXPOSURE_12MS:
				ae_r9 = 0x0132;
				break;
			case V4L2_EXPOSURE_22MS:
				ae_r9 = 0x0230;
				break;
			case V4L2_EXPOSURE_32MS:
				ae_r9 = 0x032E;
				break;
			case V4L2_EXPOSURE_42MS:
				ae_r9 = 0x042C;
				break;
			case V4L2_EXPOSURE_52MS:
				ae_r9 = 0x052B;
				break;
			default:
				reg = sensor_reg_exposure[0];
				ae_r9 = 0;
				goto exposure_auto;
				break;
		}
		reg = sensor_reg_exposure[1];
		reg[4].val = ae_r9;
	}

exposure_auto:
	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int v4l2_auto_white_balance(struct tcc_video_device *vdev, int val)
{
	int ret = -1;

	ret = write_regs(vdev->cif_i2c_client, sensor_reg_awb[val]);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int v4l2_contrast(struct tcc_video_device *vdev, int val)
{
	int ret = -1;
	struct reg_fmt *reg = sensor_reg_contrast;
	u16 mode_gam_cont_a;

	mode_gam_cont_a = 0x03 + (val * 0x10);
	reg[2].val = mode_gam_cont_a;
	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int v4l2_saturation(struct tcc_video_device *vdev, int val)
{
	int ret = -1;
	struct reg_fmt *reg = sensor_reg_saturation;
	u16 awb_saturation;

	switch (val) {
	case V4L2_SATURATION_MINUS_3:
		awb_saturation = 0x0000;
		break;
	case V4L2_SATURATION_MINUS_2:
		awb_saturation = 0x002A;
		break;
	case V4L2_SATURATION_MINUS_1:
		awb_saturation = 0x0054;
		break;
	case V4L2_SATURATION_DEFAULT:
		awb_saturation = 0x0080;
		break;
	case V4L2_SATURATION_PLUS_1:
		awb_saturation = 0x00AA;
		break;
	case V4L2_SATURATION_PLUS_2:
		awb_saturation = 0x00D4;
		break;
	case V4L2_SATURATION_PLUS_3:
		awb_saturation = 0x00FF;
		break;
	default:
		awb_saturation = 0x0080;
		break;
	}

	reg[2].val = awb_saturation;
	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int v4l2_rotate(struct tcc_video_device *vdev, int val)
{
	int ret = -1;
	struct reg_fmt *reg = sensor_reg_orientation;
	u16 read_mode_b;

	read_mode_b = 0x0300 + val;
	reg[1].val = read_mode_b;
	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int v4l2_framerate(struct tcc_video_device *vdev, int val)
{
	int ret = 0;
	struct reg_fmt *reg;
	u16 vert_blank_a0, vert_blank_a1, extra_delay_a;
	struct sensor_info *sinfo = vdev->sinfo;

	if (val == V4L2_FRAMERATE_25) {
		reg = sensor_reg_framerate[1];
	} else {
		switch (val) {
		case V4L2_FRAMERATE_15:
			vert_blank_a0 = 0x03C6;		// VBLANK (A) = 966
			vert_blank_a1 = 0x03C6;		// VBLANK (A) = 966
			extra_delay_a = 0x03FC;		// Extra Delay (A) = 1020
			break;
		case V4L2_FRAMERATE_20:
			vert_blank_a0 = 0x021E;		// VBLANK (A) = 542
			vert_blank_a1 = 0x021E;		// VBLANK (A) = 542
			extra_delay_a = 0x03FC;		// Extra Delay (A) = 1020
			break;
		case V4L2_FRAMERATE_24:
			vert_blank_a0 = 0x021E;		// VBLANK (A) = 542
			vert_blank_a1 = 0x014A;		// VBLANK (A) = 330
			extra_delay_a = 0x03FC;		// Extra Delay (A) = 1020
			break;
		case V4L2_FRAMERATE_30:
			vert_blank_a0 = 0x0077;		// VBLANK (A) = default 30 fps
			vert_blank_a1 = 0x0077;		// VBLANK (A) = default 30 fps
			extra_delay_a = 0x01C2;		// Extra Delay (A) = default 30 fps
			break;
		default:
			vert_blank_a0 = 0x0077;		// VBLANK (A) = default 30 fps
			vert_blank_a1 = 0x0077;		// VBLANK (A) = default 30 fps
			extra_delay_a = 0x01C2;		// Extra Delay (A) = default 30 fps
			ret = -EINVAL;
			break;
		}
		reg = sensor_reg_framerate[0];
		reg[4].val = vert_blank_a0;
		reg[14].val = vert_blank_a1;
		reg[17].val = extra_delay_a;
	}

	if (ret == 0)
		sinfo->framerate = val;

	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s %s(%d)\n", vdev->name, vdev->name,  __func__, val);
	return ret;
}

static int v4l2_scene_mode(struct tcc_video_device *vdev, int val)
{
	int ret = -1;
	struct reg_fmt *reg = sensor_reg_scene;
	u16 mode_spec_effects;

	switch (val) {
	case V4L2_SCENE_NONE:
		mode_spec_effects = 0x6440;
		break;
	case V4L2_SCENE_MONO:
		mode_spec_effects = 0x6441;
		break;
	case V4L2_SCENE_SEPIA:
		mode_spec_effects = 0x6442;
		break;
	case V4L2_SCENE_NEGATIVE:
		mode_spec_effects = 0x6443;
		break;
	case V4L2_SCENE_SOLARIZE1:
		mode_spec_effects = 0x6444;
		break;
	case V4L2_SCENE_SOLARIZE2:
		mode_spec_effects = 0x5245;
		break;
	default:
		mode_spec_effects = 0x6440;
		break;
	}

	reg[2].val = mode_spec_effects;
	reg[4].val = mode_spec_effects;
	ret = write_regs(vdev->cif_i2c_client, reg);

	dprintk("%s: %s(%d)\n", vdev->name, __func__, val);
	return ret;
}

static int sensor_v4l2_set_ctrl(struct tcc_video_device *vdev, __u32 id, int val)
{
	int ret = -EINVAL;

	switch (id) {
	case V4L2_CID_EXPOSURE:
		ret = v4l2_exposure(vdev, val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = v4l2_auto_white_balance(vdev, val);
		break;
	case V4L2_CID_CONTRAST:
		ret = v4l2_contrast(vdev, val);
		break;
	case V4L2_CID_SATURATION:
		ret = v4l2_saturation(vdev, val);
		break;
	case V4L2_CID_ROTATE:
		ret = v4l2_rotate(vdev, val);
		break;
	/* private control */
	case V4L2_CID_FRAMERATE:
		ret = v4l2_framerate(vdev, val);
		break;
	case V4L2_CID_SCENE:
		ret = v4l2_scene_mode(vdev, val);
		break;
	case V4L2_CID_FRAMESKIP:
		ret = vincore_set_frameskip(vdev, val);
		break;
	default:
		break;
	}
	return ret;
}

void sensor_init_mt9d111(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "mt9d111";
	sinfo->i2c_addr			= (0xBA >> 1);
	sinfo->sensor_sizes		= sensor_sizes;
	sinfo->prv_w			= 1280;
	sinfo->prv_h			= 720;
	sinfo->framerate		= 30;
	sinfo->mclk				= 240000;				/* 24Mhz 80Mhz 720p 30FPS */
	sinfo->input_data_fmt	= FMT_YUV422_8BIT;
	sinfo->input_data_order	= ORDER_RGB;
	sinfo->vin_y2r_mode		= VIN_Y2R_NOT_USE;
	sinfo->wdma_r2y_mode	= WDMA_R2Y_NOT_USE;
	if (system_rev == 0xA004)
		sinfo->polarity_pclk 	= POSITIVE_EDGE;
	else
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

	//sinfo->init_mode		= SINIT_ZOMBI;

	/* zoom info */
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
EXPORT_SYMBOL(sensor_init_mt9d111);
