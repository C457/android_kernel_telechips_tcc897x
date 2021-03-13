/*
 * tw2867.c 
 *
 * Author:  <linux@telechips.com>
 * Created: 2012 
 * Description: This is TW2867 decoder driver.
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

#define I2C_FLAG	0xFF	/* NOTE: 0xFF is DEV_ID register */
#define _SKIP_		0xFE	/* skip data. check it */

struct reg_fmt {
	unsigned char reg;	/* 8-bit register */
	unsigned char val;	/* 8-bit value */
};

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


/* 1. system clock setting
 */
static struct reg_fmt clk_pll_27mhz[] = {
/* CLOCK PLL mode - 27mhz */
	{0x60, 0x15},	// system clock select - 27mhz input
	{0x61, 0x07},
	{0x9E, 0x72},	// ch_id mode
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_pll_54mhz[] = {
/* CLOCK PLL mode - 54mhz */
	{0x60, 0x05},	// system clock select - 54mhz input
	{0x61, 0x03},
	{0x9E, 0x72},	// ch_id mode
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_pll_108mhz[] = {
/* CLOCK PLL mode - 108mhz */
	{0x60, 0x15},	// system clock select - 108mhz input
	{0x61, 0x02},
	{0x9E, 0x72},	// ch_id mode
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_xin_108mhz[] = {
/* CLOCK XTI mode - 108mhz */
	{0x60, 0x15},	// system clock select - 108mhz input
	{0x61, 0x06},	// 108mhz XTI input
	{0x9E, 0x72},	// ch_id mode
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_xin_27mhz[] = {
/* CLOCK XTI mode - 27mhz */
	{0x60, 0x15},	// system clock select - 108mhz input
	{0x61, 0x04},	// 27mhz XTI input
	{0x9E, 0x72},	// ch_id mode
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_pll_powerdown_27mhz[] = {
/* CLOCK PLL Power down mode - 27mhz */
	{0x60, 0x45},	// system clock select - 27mhz input
	{0x61, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt clk_pll_powerdown_54mhz[] = {
/* CLOCK PLL Power down mode - 54mhz */
	{0x60, 0x45},	// system clock select - 54mhz input
	{0x61, 0x01},
	{I2C_FLAG, I2C_FLAG}
};

/* 2. common register setting - ntsc/pal select
 */
static unsigned char tv_mode_ntsc[] = {
/* NTSC common register table */
// 00     01     02     03     04     05     06     07     08     09     0A     0B     0C     0D     0E     0F
  0x68,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x02,  0x12,  0xF0,  0x0F,  0xD0,  0x00,  0x10,  0x00,  0x7F,  //00
  0x68,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x02,  0x12,  0xF0,  0x0F,  0xD0,  0x00,  0x10,  0x00,  0x7F,  //01
  0x68,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x02,  0x12,  0xF0,  0x0F,  0xD0,  0x00,  0x10,  0x00,  0x7F,  //02
  0x68,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x02,  0x12,  0xF0,  0x0F,  0xD0,  0x00,  0x10,  0x00,  0x7F,  //03
  0x00,  0x40,  0x90,  0x08,  0x00,  0x38,  0x80,  0x80,  0x80,  0x80,  0xCC,  0x00,  0x00,  0x00,  0x00,  0x00,  //04
  0x22,  0x00,  0x08,  0x00,  0x07,  0x00,  0x00,  0x00,  0x00,  0x00,  0x07,  0xFF,  0x01,  0xE0,  0xE0,  0xE0,  //05
_SKIP_,_SKIP_,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x0F,  0x0F,  0x00,  0x21,  0x43,  //06
  0x08,  0x00,  0x00,  0x01,  0x01,  0x00,  0x00,  0x02,  0x00,  0x02,  0x00,  0x15,  0x15,  0xE4,  0xA3,  0x50,  //07
  0x00,  0x02,  0x80,  0xCC,  0x00,  0x80,  0x44,  0x50,  0x22,  0x01,  0xD8,  0xBC,  0xB8,  0x44,  0x38,  0x00,  //08
  0x00,  0x78,  0x44,  0x30,  0x14,  0xA5,  0xE0,  0x05,  0x00,  0x28,  0x44,  0x44,  0xA0,  0x8A,  0x52,  0x11,  //09
  0x08,  0x08,  0x08,  0x08,  0x1A,  0x1A,  0x1A,  0x1A,  0x00,  0x00,  0x00,  0xF0,  0xF0,  0xF0,  0xF0,  0x44,  //0A
  0x44,  0x4A,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0xFF,  0xF0,  0xC8,  0xE8,  0xF8,  0xFF,  0xF1,  0xC9,  //0B
  0xE6,  0xF8,  0x00,  0x00,  0xFF,  0xFF,  0xFF,  0xFF,  0x88,  0x00,  0x00,  0x00,  0x39,  0xE4,  0x00,  0x80,  //0C
  0x55,  0x55,  0x01,  0x10,  0x32,  0x10,  0x32,  0x10,  0x32,  0x10,  0x32,  0xC1,  0x00,  0x00,  0x00,  0x80,  //0D
  0xFF,  0xC0,  0xAA,  0xAA,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  //0E
  0x83,  0xB5,  0x09,  0x78,  0x85,  0x00,  0x00,  0x20,  0xC4,  0x51,  0x40,  0xAF,  0xFF,  0x2F,  0x00,  0xC8   //0F
};

static unsigned char tv_mode_pal[] = {
/* PAL comon reister table */
// 00     01     02     03     04     05     06     07     08     09     0A     0B     0C     0D     0E     0F
  0x69,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x12,  0x12,  0x20,  0x0C,  0xD0,  0x00,  0x10,  0x01,  0x7F,  //00
  0x69,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x12,  0x12,  0x20,  0x0C,  0xD0,  0x00,  0x10,  0x01,  0x7F,  //01
  0x69,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x12,  0x12,  0x20,  0x0C,  0xD0,  0x00,  0x10,  0x01,  0x7F,  //02
  0x79,  0x00,  0x64,  0x13,  0x80,  0x80,  0x00,  0x12,  0x12,  0x20,  0x0C,  0xD0,  0x00,  0x10,  0x01,  0x7F,  //03
  0x00,  0xD4,  0x90,  0x08,  0x00,  0x38,  0x80,  0x80,  0x80,  0x80,  0xCC,  0x00,  0x00,  0x00,  0x00,  0x00,  //04
  0x22,  0x00,  0x08,  0x00,  0x07,  0x00,  0x00,  0x00,  0x00,  0x00,  0x07,  0x00,  0x01,  0xE0,  0xE0,  0xE0,  //05
_SKIP_,_SKIP_,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x0F,  0x0F,  0x00,  0x21,  0x43,  //06
  0x08,  0x00,  0x00,  0x01,  0x01,  0x00,  0x00,  0x02,  0x00,  0x02,  0x00,  0x15,  0x15,  0xE4,  0xA3,  0x50,  //07
  0x00,  0x02,  0x80,  0xCC,  0x00,  0x80,  0x44,  0x50,  0x22,  0x01,  0xD8,  0xBC,  0xB8,  0x44,  0x38,  0x00,  //08
  0x00,  0x78,  0x44,  0x30,  0x14,  0xA5,  0xE0,  0x05,  0x00,  0x28,  0x44,  0x44,  0xA0,  0x90,  0x52,  0x00,  //09
  0x18,  0x18,  0x18,  0x18,  0x1A,  0x1A,  0x1A,  0x1A,  0x00,  0x00,  0x00,  0xF0,  0xF0,  0xF0,  0xF0,  0x44,  //0A
  0x44,  0x4A,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0xFF,  0xF4,  0xC6,  0xE9,  0xF9,  0xFF,  0xF2,  0xC9,  //0B
  0xE7,  0xF8,  0x00,  0x00,  0xFF,  0xFF,  0xFF,  0xFF,  0x88,  0x88,  0x00,  0x00,  0x39,  0xE4,  0x00,  0x80,  //0C
  0x55,  0x55,  0x01,  0x10,  0x32,  0x10,  0x32,  0x10,  0x32,  0x10,  0x32,  0xC1,  0x00,  0x00,  0x00,  0x80,  //0D
  0xFF,  0xC0,  0xAA,  0xAA,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  0x00,  0x11,  0x00,  //0E
  0x83,  0xB5,  0x09,  0x00,  0xA0,  0x00,  0x00,  0x20,  0xC4,  0x51,  0x40,  0x2F,  0xFF,  0x2F,  0x00,  0xC8   //0F
};

/* 3. ITU.BT656 ouput frequency setting 27mhz / 54mhz / 108mhz
 */
static struct reg_fmt output_single_27mhz[] = {
/* ITU.BT656 27MHz */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x00},	// single channel select

	{0xfa, 0x40},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x00},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x00},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x00},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_two_27mhz[] = {
/* ITU.BT656 27MHz */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x55},	// two channel select

	{0xfa, 0x40},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x00},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x00},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x00},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_three_27mhz[] = {
/* 2ch VD1 + 1ch single direct */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x01},	// CHMD2: single, CHMD1: two
	{0xcd, 0xd8},
	{0xcc, 0x39},

	{0xfa, 0x40},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x00},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x00},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x00},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_two_54mhz[] = {
/* ITU.BT656 DATA 54MHz / CLOCK 54MHZ */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x55},	// two channel select
	{0xcc, 0x39},	// 2nd channel selection

	{0xfa, 0x45},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x05},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x05},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x05},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_four_108mhz[] = {
/* ITU.BT656 DATA 108MHz / CLOCK 108MHZ */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0xaa},	// four channel select
	{0xcc, 0x39},	// 2nd channel selection

	{0xfa, 0x4a},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x0a},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x0a},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x0a},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_four_54mhz[] = {
/* ITU.BT656 DATA 54MHz / CLOCK 54MHZ */
	{0x82, 0x80},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0xaa},	// four channel select
	{0xcc, 0x39},	// 2nd channel selection

	{0xfa, 0x45},	// control the clock frequency of CLK 1 PIN (VSCL_ENA: CIF)
	//{0xfa, 0xC5},	// control the clock frequency of CLK 1 PIN (VSCL_ENA: Quad)
	{0x6a, 0x05},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x05},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x05},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_set_temp0[] = {
/* ITU.BT656 data 54MHZ / CLOCK 27MHZ positive CLKPO1~4 / negetive CLKNO1~4 */
	{0x80, 0x10},	// clock reset
	{0x82, 0x00},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x55},	// single / two channel / four channel select : VD1/2/3/4 port data 54 MHz
	{0xcc, 0x39},	// 2nd channel selection
	
	{0xfa, 0x40},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x00},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x00},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x00},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0xaf},	// clock polarity control - CLKNO1: inverse - CLKPO1 : normal
	{0x6d, 0xa8},	// clock polarity control - CLKNO2~4 : inverse  - CLKPO2~4 : normal
	{I2C_FLAG, I2C_FLAG}
};

static struct reg_fmt output_set_temp1[] = {
/* Using TW2864/65/66 and Solo6010 - ITU.BT656 data 54MHZ / CLOCK 27MHZ positive only */
	{0x82, 0x00},	// 27Mhz clock ouput signal rise/fall timing

	{0x9f, 0x00},	// clock ouput delay control register
	{0x67, 0x00},
	{0x68, 0x00},
	{0x69, 0x00},

	{0xca, 0x55},	// single / two channel / four channel select
	{0xcc, 0x39},	// 2nd channel selection

	{0xfa, 0x40},	// control the clock frequency of CLK 1 PIN
	{0x6a, 0x00},	// control the clock frequency of CLK 2 PIN
	{0x6b, 0x00},	// control the clock frequency of CLK 3 PIN
	{0x6c, 0x00},	// control the clock frequency of CLK 4 PIN

	{0xfb, 0x2f},	// clock polarity control
	{0x6d, 0x00},

	{0x43, 0x68},	// encoder setting for solo6xxxx
	{I2C_FLAG, I2C_FLAG}
};			

static struct reg_fmt *sensor_clock_pll_mode[7] = {
	clk_pll_27mhz,
	clk_pll_54mhz,
	clk_pll_108mhz,
	clk_xin_108mhz,
	clk_xin_27mhz,
	clk_pll_powerdown_27mhz,
	clk_pll_powerdown_54mhz
};

static unsigned char *sensor_tv_mode[2] = {
	tv_mode_ntsc,
	tv_mode_pal
};

static struct reg_fmt *sensor_output_mode[8] = {
	output_single_27mhz,
	output_two_27mhz,
	output_two_54mhz,
	output_four_54mhz,
	output_four_108mhz,
	output_three_27mhz,
	output_set_temp0,
	output_set_temp1
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
			dprintk("==========> write: {0x%02X, 0x%02X} try(%d)\n", data[0], data[1], err_cnt);
			if (err) {
				if (err_cnt++ >= 3) return err;
				else goto again_reg;
			} else {
				err_cnt = 0;
				next++;
			}

#ifdef DBG_I2C_READ
			{
				unsigned char reg, val;
				reg = data[0];
				cif_i2c_recv(client, reg, 1, &val, 1);
				dprintk("            read : {0x%02X, 0x%02X}\n", reg, val);
				if (data[1] != val)
					dprintk("\e[31m mismatch!!! \e[0m \n");
			}
#endif
		}
	}

	return 0;
}

static int write_table(struct i2c_client *client, const unsigned char table[])
{
	int err, err_cnt = 0;
	unsigned char i;
	unsigned char data[2];

	for (i = 0; i < 0xff; i++) {
again_reg:
		data[0] = i;
		data[1] = table[i];

		if (unlikely(data[1] == _SKIP_))
			continue;

		err = cif_i2c_send(client, data, 1, 1);
		dprintk("==========> write: {0x%02X, 0x%02X} try(%d)\n", data[0], data[1], err_cnt);
		if (err) {
			if (err_cnt++ >= 3) return err;
			else goto again_reg;
		} else {
			err_cnt = 0;
		}
#ifdef DBG_I2C_READ
		{
			unsigned char reg, val;
			reg = data[0];
			cif_i2c_recv(client, reg, 1, &val, 1);
			dprintk("            read : {0x%02X, 0x%02X}\n", reg, val);
			if (data[1] != val)
				dprintk("\e[31m mismatch!!! \e[0m \n");
		}
#endif
	}

	return 0;
}

static int sensor_open(struct tcc_video_device *vdev)
{
	int ret = 0;

	cif_power_disable(vdev);
	//msleep(10);

	cif_power_enable(vdev);
	//msleep(10);

	cif_powerdown_disable(vdev);
	//msleep(10);

	cif_reset_low(vdev);
	//msleep(10);

	cif_open(vdev);
	msleep(50);

	cif_reset_high(vdev);
	//msleep(10);

	switch (vdev->sinfo->private) {
	case VINDEMUX_MODE_1CH_D1_27MHZ:
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[0]);
		msleep(50);
		ret |= write_table(vdev->cif_i2c_client, sensor_tv_mode[0]);
		msleep(50);
		ret |= write_regs(vdev->cif_i2c_client, sensor_output_mode[0]);
		msleep(50);
		break;
	case VINDEMUX_MODE_2CH_D1_27MHZ:
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[0]);
		msleep(50);
		ret |= write_table(vdev->cif_i2c_client, sensor_tv_mode[0]);
		msleep(50);
		ret |= write_regs(vdev->cif_i2c_client, sensor_output_mode[1]);
		msleep(50);
		break;
	case VINDEMUX_MODE_4CH_D1_108MHZ:
		#ifndef USE_MCLK_XTAL
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[3]);
		#else
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[0]);
		#endif
		msleep(50);
		ret |= write_table(vdev->cif_i2c_client, sensor_tv_mode[0]);
		msleep(50);
		ret |= write_regs(vdev->cif_i2c_client, sensor_output_mode[4]);
		msleep(50);
		break;
	case VINDEMUX_MODE_2CH_1CH_D1_27MHZ:
		#ifndef USE_MCLK_XTAL
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[4]);
		#else
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[0]);
		#endif
		msleep(50);
		ret |= write_table(vdev->cif_i2c_client, sensor_tv_mode[0]);
		msleep(50);
		ret |= write_regs(vdev->cif_i2c_client, sensor_output_mode[5]);
		msleep(50);
		break;
	case VINDEMUX_MODE_4CH_D1_54MHZ:
	case VINDEMUX_MODE_4CH_CIF_54MHZ:
		ret = write_regs(vdev->cif_i2c_client, sensor_clock_pll_mode[0]);
		msleep(50);
		ret |= write_table(vdev->cif_i2c_client, sensor_tv_mode[0]);
		msleep(50);
		ret |= write_regs(vdev->cif_i2c_client, sensor_output_mode[3]);
		msleep(50);
		break;
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

static int sensor_v4l2_set_ctrl(struct tcc_video_device *vdev, __u32 id, int val)
{
	int err = -EINVAL;

	switch (id) {
	default:
		break;
	}
	return err;
}

void sensor_init_tw2867(struct sensor_info *sinfo)
{
	int i, v4l2_ctrl_size;

	/* basic info */
	sinfo->name				= "tw2867";
	sinfo->i2c_addr			= (0x50 >> 1);
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

	/* select tw2857 video output mode */
#if defined(BB_4CH_TYPE1)
	sinfo->private			= VINDEMUX_MODE_4CH_D1_108MHZ;
#elif defined(BB_4CH_TYPE2)
	sinfo->private			= VINDEMUX_MODE_2CH_1CH_D1_27MHZ;
#elif defined(BB_4CH_TYPE3)
	sinfo->private			= VINDEMUX_MODE_4CH_D1_54MHZ;
#elif defined(BB_4CH_TYPE4)
	sinfo->private			= VINDEMUX_MODE_4CH_CIF_54MHZ;
	sinfo->prv_w			/= 2;
#else
	#error "tw2867: choose channel type" 
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

	/* zoom */
	sinfo->zoom_type		= ZOOM_NOT_USE;
	sinfo->prv_zoffx		= 0;
	sinfo->prv_zoffy		= 0;

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
EXPORT_SYMBOL(sensor_init_tw2867);
