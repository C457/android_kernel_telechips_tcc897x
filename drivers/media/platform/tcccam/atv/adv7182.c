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

#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/bsp.h>

#include "../sensor_if.h"
#include "../cam_reg.h"
#include "../tcc_cam_i2c.h"
#include "../tcc_cam.h"
#include "../tcc_camera_device.h"

#ifdef CONFIG_VIDEO_ATV_SENSOR_ADV7182

#define FEATURE_TV_STANDARD_NTSC

#if defined(FEATURE_TV_STANDARD_NTSC)	// support to NTSC
#define ADV7182_WIDTH			720
#define ADV7182_HEIGHT			480
#else									// support to PAL
#define ADV7182_WIDTH			720
#define ADV7182_HEIGHT			576
#endif

struct capture_size sensor_sizes[] =
{
	{ADV7182_WIDTH, ADV7182_HEIGHT},
	{ADV7182_WIDTH, ADV7182_HEIGHT}
};

/*
COMMON()
*/ // for UltraEdit-32
static struct sensor_reg sensor_initialize[] =
{
#if 1	/*20160811 CVBS FAST Switch */
	{0x0f, 0x00},	/* 42 0F 00 ; Exit Power Down Mode */
	{0x00, 0x00},	/* 42 00 00 ; INSEL = CVBS in on Ain 1 */
	{0x03, 0x0c},	/* 42 03 0C ; Enable Pixel & Sync output drivers */
	{0x04, 0x17},	/* 42 04 17 ; Power-up INTRQ pad & Enable SFL */
	{0x13, 0x00},	/* 42 13 00 ; Enable INTRQ output driver */
	{0x17, 0x41},	/* 42 17 41 ; select SH1 */
	{0x1d, 0x40},	/* 42 1D 40 ; Enable LLC output driver */
	{0x52, 0xcb},	/* 42 52 CB ; ADI Recommended Writes */
	{0x0e, 0x80},	/* 42 0E 80 ; ADI Recommended Writes */
	{0xd9, 0x44},	/* 42 D9 44 ; ADI Recommended Writes */
	{0x0e, 0x00},	/* 42 0E 00 ; ADI Recommended Writes */
	{0x0e, 0x40},	/* 42 0E 40 ; Select User Sub Map 2 */
	{0xe0, 0x01},	/* 42 E0 01 ; Select fast Switching Mode */
	{0x0e, 0x00},	/* 42 0E 00 ; Select User Map */
#else	/*20160811 Autodetect CVBS Single Ended In Ain 1, YPrPb Out: */
	{0x0f, 0x00},	// 42 0F 00 ; Exit Power Down Mode
	{0x00, 0x00},	// 42 00 00 ; INSEL = CVBS in on Ain 1
	{0x03, 0x0c},	// 42 03 0C ; Enable Pixel & Sync output drivers
	{0x04, 0x17},	// 42 04 17 ; Power-up INTRQ pad & Enable SFL
	{0x13, 0x00},	// 42 13 00 ; Enable INTRQ output driver
	{0x17, 0x41},	// 42 17 41 ; select SH1
	{0x1d, 0x40},	// 42 1D 40 ; Enable LLC output driver
	{0x52, 0xcb},	// 42 52 CB ; ADI Recommended Writes
#endif
	{REG_TERM, VAL_TERM}

};

static struct sensor_reg sensor_set_preview[] =
{
//	{0x00, 0x00},
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_set_capture[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_common[5] =
{
	sensor_initialize,
	sensor_set_preview,
	sensor_set_capture,
};

/*
BRIGHTNESS()
*/ // for UltraEdit-32
static struct sensor_reg sensor_brightness_0[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_1[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_2[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_3[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_4[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_brightness[5] =
{
	sensor_brightness_0,
	sensor_brightness_1,
	sensor_brightness_2,
	sensor_brightness_3,
	sensor_brightness_4
};

/*
AWB()
*/ // for UltraEdit-32
static struct sensor_reg sensor_awb_auto[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_daylight[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_incandescent[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_fluorescent[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_cloudy[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_sunset[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_awb[6] =
{
	sensor_awb_auto,
	sensor_awb_daylight,
	sensor_awb_incandescent,
	sensor_awb_fluorescent,
	sensor_awb_cloudy,
	sensor_awb_sunset
	
};

/*
ISO()
*/ // for UltraEdit-32
static struct sensor_reg sensor_iso_auto[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_100[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_200[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_400[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_iso[4] =
{
	sensor_iso_auto,
	sensor_iso_100,
	sensor_iso_200,
	sensor_iso_400
};

/*
EFFECT()
*/ // for UltraEdit-32
static struct sensor_reg sensor_effect_normal[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_gray[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_negative[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sepia[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sharpness[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sketch[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_effect[6] =
{
	sensor_effect_normal,
	sensor_effect_gray,
	sensor_effect_negative,
	sensor_effect_sepia,
	sensor_effect_sharpness,
	sensor_effect_sketch,
};

/*
FLIP()
*/ // for UltraEdit-32
static struct sensor_reg sensor_reg_flipnone[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_hflip[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_vflip[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_hvflip[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_flip[4] =
{
	sensor_reg_flipnone,
	sensor_reg_hflip,
	sensor_reg_vflip,
	sensor_reg_hvflip,
};

/*
SCENE()
*/ // for UltraEdit-32
static struct sensor_reg sensor_secne_auto[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_night[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_landscape[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_portrait[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_sport[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_scene[5] =
{
	sensor_secne_auto,
	sensor_secne_night,
	sensor_secne_landscape,
	sensor_secne_portrait,
	sensor_secne_sport
};

/*
METERING_EXPOSURE()
*/ // for UltraEdit-32
static struct sensor_reg sensor_me_mtrix[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_me_center_weighted[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_me_spot[] =
{
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_metering_exposure[3] =
{
	sensor_me_mtrix,
	sensor_me_center_weighted,
	sensor_me_spot,
};

/*
AF()
*/ // for UltraEdit-32
static struct sensor_reg sensor_af_single[] =
{
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_af_manual[] =
{
	 {REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_af[2] =
{
	sensor_af_single,
	sensor_af_manual,
};


static int write_regs_adv7182(const struct sensor_reg reglist[], struct tcc_camera_device * vdev)
{
	int err;
	int err_cnt = 0;	
	unsigned char data[132];
	unsigned char bytes;
	const struct sensor_reg *next = reglist;

	while (!((next->reg == REG_TERM) && (next->val == VAL_TERM)))
	{
		bytes = 0;
		data[bytes]= (unsigned char)next->reg&0xff;
		bytes++;
		data[bytes]= (unsigned char)next->val&0xff;
		if(next->reg == 0x00) {
			data[bytes] = (unsigned char)(vdev->cvbs_input & 0xff);
//			printk("cvbs_input : 0x%x data : 0x%x \n", vdev->cvbs_input & 0xff, data[bytes]);
		}
		bytes++;
			
		err = DDI_I2C_Write(data, 1, 1, vdev);

		if (err)
		{
			err_cnt++;
			if(err_cnt >= 3)
			{
				printk("ERROR: Sensor I2C !!!! \n"); 
				return err;
			}
		}
		else
		{
			err_cnt = 0;
			next++;
		}
	}

	return 0;
}

static int sensor_open(struct tcc_camera_device * vdev, bool bChangeCamera)
{
	int ret;

	sensor_port_disable(vdev->gpio_data.rst_port);
//	sensor_delay(15);
//	sensor_delay(10);

	sensor_port_enable(vdev->gpio_data.rst_port);
//	sensor_delay(15);
	msleep(5);
//	sensor_delay(10);

	ret = write_regs_adv7182(sensor_reg_common[0], vdev);
	msleep(50);
//	mdelay(100);
	return ret;
}

static int sensor_close(struct tcc_camera_device * vdev)
{

	sensor_port_disable(vdev->gpio_data.rst_port);
	sensor_port_disable(vdev->gpio_data.pwr_port);
	sensor_port_disable(vdev->gpio_data.pwd_port);

	sensor_delay(5);

	return 0;
}

static int sensor_preview(struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_common[1], vdev);
}

static int sensor_capture(struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_common[2], vdev);
}

// CAMERA - NTSC
static struct sensor_reg sensor_camera_ntsc[] = {
	{REG_TERM, VAL_TERM}
};

// CAMERA - PAL
static struct sensor_reg sensor_camera_pal[] = {
	{REG_TERM, VAL_TERM}
};

// AUX - NTSC
static struct sensor_reg sensor_aux_ntsc[] = {
	{REG_TERM, VAL_TERM}
};

// AUX - PAL
static struct sensor_reg sensor_aux_pal[] = {
	{REG_TERM, VAL_TERM}
};

struct sensor_reg *sensor_reg_type_and_mode[CAMERA_TYPE_MAX][CAMERA_ENCODE_MAX] =
{
	// CAMERA
	{
		sensor_camera_ntsc,
		sensor_camera_pal,
	},
	// AUX
	{
		sensor_aux_ntsc,
		sensor_aux_pal,
	},
};

static int sensor_change_mode(int type, int encode, struct tcc_camera_device * vdev) {
	if(CAMERA_TYPE_MAX <= type || CAMERA_ENCODE_MAX <= encode) {
		printk("%s() - type or encode is wrong.\n", __func__);
		return -1;
	}
	return write_regs_adv7182(sensor_reg_type_and_mode[type][encode], vdev);
}
static int sensor_capturecfg(int width, int height, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_zoom(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_autofocus(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_af[val], vdev);
}

static int sensor_effect(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_effect[val], vdev);
}

static int sensor_flip(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_flip[val], vdev);
}

static int sensor_iso(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_iso[val], vdev);
}

static int sensor_me(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_metering_exposure[val], vdev);
}

static int sensor_wb(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_awb[val], vdev);
}

static int sensor_bright(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_brightness[val], vdev);
}

static int sensor_scene(int val, struct tcc_camera_device * vdev)
{
	return write_regs_adv7182(sensor_reg_scene[val], vdev);
}

static int sensor_readRegister(int reg, struct tcc_camera_device * vdev)
{
	unsigned char val = 0;
	DDI_I2C_Read((unsigned short)reg, 1, &val, 1, vdev);
	return val;
}

static int sensor_check_esd(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_check_luma(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

/**********************************************************
*	 Function  
**********************************************************/
void sensor_info_init(TCC_SENSOR_INFO_TYPE *sensor_info)
{
#if defined(FEATURE_TV_STANDARD_NTSC)	// support to NTSC	
	sensor_info->preview_w				= 720;
	sensor_info->preview_h				= 240;
	sensor_info->capture_w				= 720;
	sensor_info->capture_h				= 240;
#else									// support to PAL
	sensor_info->preview_w				= 720;
	sensor_info->preview_h				= 288;
	sensor_info->capture_w				= 720;
	sensor_info->capture_h				= 288;
#endif
	sensor_info->preview_zoom_offset_x	= 0;
	sensor_info->preview_zoom_offset_y	= 0;
	sensor_info->capture_zoom_offset_x	= 0;
	sensor_info->capture_zoom_offset_y	= 0;
	sensor_info->max_zoom_step			= 25;
	sensor_info->cam_capchg_width			= 720;
	sensor_info->framerate				= 15;
	sensor_info->p_clock_pol				= NEGATIVE_EDGE;
	sensor_info->v_sync_pol 				= ACT_HIGH;
	sensor_info->h_sync_pol 				= ACT_HIGH;
	sensor_info->de_pol 					= ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en				= OFF;
	sensor_info->conv_en					= ON;
	sensor_info->hsde_connect_en			= OFF;
	sensor_info->vs_mask					= OFF;
	sensor_info->input_fmt				= FMT_YUV422_8BIT;
	sensor_info->data_order 				= ORDER_RGB;
	sensor_info->intl_en					= ON;
	sensor_info->intpl_en					= OFF;	
	sensor_info->format 					= M420_EVEN;
	sensor_info->capture_skip_frame 		= 0;
	sensor_info->sensor_sizes				= sensor_sizes;
}

void sensor_init_fnc(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_func->Open			= sensor_open;
	sensor_func->Close			= sensor_close;

	sensor_func->Set_Preview	= sensor_preview;
	sensor_func->Set_Capture	= sensor_capture;
	sensor_func->Set_CameraMode	= sensor_change_mode;
	sensor_func->Set_CaptureCfg = sensor_capturecfg;

	sensor_func->Set_Zoom		= sensor_zoom;
	sensor_func->Set_AF 		= sensor_autofocus;
	sensor_func->Set_Effect 	= sensor_effect;
	sensor_func->Set_Flip		= sensor_flip;
	sensor_func->Set_ISO		= sensor_iso;
	sensor_func->Set_ME 		= sensor_me;
	sensor_func->Set_WB 		= sensor_wb;
	sensor_func->Set_Bright 	= sensor_bright;
	sensor_func->Set_Scene		= sensor_scene;

	sensor_func->Check_ESD		= sensor_check_esd;
	sensor_func->Check_Luma 	= sensor_check_luma;

	sensor_func->ReadSensorRegister	= sensor_readRegister;
}

void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init(sensor_info);
}

void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc(sensor_func);
}

#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init(sensor_info);
}
void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc(sensor_func);
}
#endif


#endif /*CONFIG_VIDEO_CAMERA_SENSOR_ADV7182*/

