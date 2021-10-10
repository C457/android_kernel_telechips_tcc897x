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

//#include <asm/system.h>
#include <asm/io.h>

#include "../sensor_if.h"
#include "../cam_reg.h"
#include "../tcc_cam_i2c.h"
#include "../tcc_cam.h"
#include "../tcc_camera_device.h"

#ifdef CONFIG_VIDEO_CAMERA_SENSOR_OV7690

/* Array of image sizes supported by OV7690.  These must be ordered from 
 * smallest image size to largest.
 */

struct capture_size sensor_sizes_ov7690[] = {
	{  640,  480 },	/* VGA */
	{  352,  288 },	/* CIF */
	{  320,  240 },	/* QVGA */
	{  176,  144 },	/* QCIF */
};


/* register initialization tables for sensor */
/* common sensor register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
static struct sensor_reg sensor_initialize_ov7690[] = {
	{0x12, 0x80},
	{0x0C, 0x16},
	{0x27, 0x80},

	{0x64, 0x10},
	{0x68, 0xB4},
	{0x69, 0x12},
	{0x2F, 0x60},

	{0x41, 0x41},
	{0x44, 0x24},		// Updated per v5 
	{0x4B, 0x0E},		// Updated per V13
	{0x4C, 0x7B},		// Updated per V16;  set black sun reference voltage.

	{0x4D, 0x0A},		// Updated per V6
	{0x29, 0x50},

	{0x1B, 0x19},		// updated per V8
	{0x39, 0x00},		// updated per V16

	{0x80, 0x7E},
	{0x81, 0xFF},

	{0x91, 0x20},		// YAVG after Gamma
	{0x21, 0x33},		// updated per V16

	/* Format */

	{0x11, 0x01},
	{0x12, 0x00},
	{0x82, 0x03},
	{0xD0, 0x26},
	{0x2B, 0x38},
	{0x15, 0xB4},

	/* Resolution */

	{0x16, 0x00},
	{0x17, 0x69},
	{0x18, 0xA4},
	{0x19, 0x0C},
	{0x1A, 0xF6},
	{0x3E, 0x30},		

	{0xC8, 0x02},
	{0xC9, 0x80},		// ISP input hsize (640) 
	{0xCA, 0x01},
	{0xCB, 0xE0},		// ISP input vsize (480)
	
	{0xCC, 0x02},
	{0xCD, 0x80},		// ISP output hsize (640)
	{0xCE, 0x01},
	{0xCF, 0xE0},		// ISP output vsize (480)

	/* Lens Correction */

	{0x80, 0x7F},
	{0x85, 0x10},
	{0x86, 0x20},
	{0x87, 0x09},
	{0x88, 0xAF},
	{0x89, 0x25},
	{0x8A, 0x20},
	{0x8B, 0x20},

	/* Color Matrix */

	{0xBB, 0xAC},		// D7
	{0xBC, 0xAE},		// DA
	{0xBD, 0x02},		// 03
	{0xBE, 0x1F},		// 27
	{0xBF, 0x93},		// B8
	{0xC0, 0xB1},		// DE
	{0xC1, 0x1A},

	/* Edge + Denoise */

	{0xB4, 0x06},		
	{0xB5, 0x05},		// auto, no meaning
	{0xB6, 0x00},		// auto, no meaning
	{0xB7, 0x00},		
	{0xB8, 0x06},		
	{0xB9, 0x02},		
	{0xBA, 0x78},

	/* AEC/AGC target */

	{0x24, 0x94},		
	{0x25, 0x80},		
	{0x26, 0xB4},

	/* UV adjust */

	{0x81, 0xFF},
	{0x5A, 0x10},
	{0x5B, 0xA1},
	{0x5C, 0x3A},
	{0x5D, 0x20},

	/* Gamma */

	{0xA3, 0x05},
	{0xA4, 0x10},
	{0xA5, 0x25},
	{0xA6, 0x46},
	{0xA7, 0x57},
	{0xA8, 0x64},
	{0xA9, 0x70},
	{0xAA, 0x7C},
	{0xAB, 0x87},
	{0xAC, 0x90},
	{0xAD, 0x9F},
	{0xAE, 0xAC},
	{0xAF, 0xC1},
	{0xB0, 0xD5},
	{0xB1, 0xE7},
	{0xB2, 0x21},	

	/* AWB, Advance */

	{0x8C, 0x52},
	{0x8D, 0x11},
	{0x8E, 0x12},
	{0x8F, 0x19},
	{0x90, 0x50},
	{0x91, 0x20},
	{0x92, 0xB1},
	{0x93, 0x9A},
	{0x94, 0xC},
	{0x95, 0xC},
	{0x96, 0xF0},
	{0x97, 0x10},
	{0x98, 0x61},
	{0x99, 0x63},
	{0x9A, 0x71},
	{0x9B, 0x78},
	{0x9C, 0xF0},
	{0x9D, 0xF0},
	{0x9E, 0xF0},
	{0x9F, 0xFF},
	{0xA0, 0xA7},
	{0xA1, 0xB0},
	{0xA2, 0xF},

	{0x14, 0x20},		// 16x gain ceiling, PPChrg off

	{0x13, 0xF7},
	{0x50, 0x49},		// banding	

	{REG_TERM, VAL_TERM}
};


static struct sensor_reg sensor_set_preview_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_set_capture_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

 struct sensor_reg* sensor_reg_common[3] =
{
	sensor_initialize_ov7690,
	sensor_set_preview_ov7690,
	sensor_set_capture_ov7690
};

static struct sensor_reg sensor_brightness_0_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_1_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_2_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_3_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_brightness_4_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_brightness[5] =
{
	sensor_brightness_0_ov7690,
	sensor_brightness_1_ov7690,
	sensor_brightness_2_ov7690,
	sensor_brightness_3_ov7690,
	sensor_brightness_4_ov7690
};


static struct sensor_reg sensor_awb_auto_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_daylight_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_incandescent_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_fluorescent_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_cloudy_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_awb_sunset_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_awb[6] =
{
	sensor_awb_auto_ov7690,
	sensor_awb_daylight_ov7690,
	sensor_awb_incandescent_ov7690,
	sensor_awb_fluorescent_ov7690,
	sensor_awb_cloudy_ov7690,
	sensor_awb_sunset_ov7690
	
};


static struct sensor_reg sensor_iso_auto_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_100_ov7690[] = {
	
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_200_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_iso_400_ov7690[] = {

	{REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_iso[4] =
{
	sensor_iso_auto_ov7690,
	sensor_iso_100_ov7690,
	sensor_iso_200_ov7690,
	sensor_iso_400_ov7690
};


static struct sensor_reg sensor_effect_normal_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_gray_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_negative_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sepia_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sharpness_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_effect_sketch_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_effect[6] =
{
	sensor_effect_normal_ov7690,
	sensor_effect_gray_ov7690,
	sensor_effect_negative_ov7690,
	sensor_effect_sepia_ov7690,
	sensor_effect_sharpness_ov7690,
	sensor_effect_sketch_ov7690,
};


static struct sensor_reg sensor_reg_flipnone_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_hflip_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_vflip_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_reg_hvflip_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_flip[4] =
{
	sensor_reg_flipnone_ov7690,
	sensor_reg_hflip_ov7690,
	sensor_reg_vflip_ov7690,
	sensor_reg_hvflip_ov7690,
};


static struct sensor_reg sensor_secne_auto_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_night_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_landscape_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_portrait_ov7690[] = {
	{REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_secne_sport_ov7690[] = {
	{REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_scene[5] =
{
	sensor_secne_auto_ov7690,
	sensor_secne_night_ov7690,
	sensor_secne_landscape_ov7690,
	sensor_secne_portrait_ov7690,
	sensor_secne_sport_ov7690
};

static struct sensor_reg sensor_me_mtrix_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_me_center_weighted_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_me_spot_ov7690[] = {
    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_metering_exposure[3] =
{
	sensor_me_mtrix_ov7690,
	sensor_me_center_weighted_ov7690,
	sensor_me_spot_ov7690,
};

static struct sensor_reg sensor_af_single_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

static struct sensor_reg sensor_af_manual_ov7690[] = {

    {REG_TERM, VAL_TERM}
};

struct sensor_reg* sensor_reg_af[2] =
{
	sensor_af_single_ov7690,
	sensor_af_manual_ov7690,
};

static int write_regs_ov7690(const struct sensor_reg reglist[], struct tcc_camera_device * vdev)
{
	int err;
	int err_cnt = 0;	
	unsigned char data[132];
	unsigned char bytes;
	const struct sensor_reg *next = reglist;
	
	while (!((next->reg == REG_TERM) && (next->val == VAL_TERM)))
	{
		bytes = 0;
		data[bytes++] = (unsigned char)(next->reg) & 0xff; 	
		data[bytes]   = (unsigned char)(next->val) & 0xff;
		
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

static int sensor_open_ov7690(struct tcc_camera_device * vdev, bool bChangeCamera)
{
	sensor_port_disable(vdev->gpio_data.pwr_port);
	sensor_delay(10);
	
	sensor_port_enable(vdev->gpio_data.pwr_port);
	sensor_delay(10);

	sensor_port_disable(vdev->gpio_data.pwd_port);
	sensor_delay(10);
	
	sensor_cifmc_enable(vdev);
	sensor_delay(40);
	
	//sensor_reset_high();
	//sensor_delay(15); // sensor_delay(1);

	return write_regs_ov7690(sensor_reg_common[0], vdev);
}

static int sensor_close_ov7690(struct tcc_camera_device * vdev)
{
	sensor_port_disable(vdev->gpio_data.rst_port);
	sensor_port_disable(vdev->gpio_data.pwr_port);
	sensor_port_enable(vdev->gpio_data.pwd_port);

	sensor_cifmc_disable(vdev);   
	sensor_delay(5);    

	return 0;
}

static int sensor_powerdown_ov7690(struct tcc_camera_device * vdev, bool bChangeCamera)
{
	sensor_port_disable(vdev->gpio_data.rst_port);
	sensor_port_disable(vdev->gpio_data.pwr_port);
	sensor_port_enable(vdev->gpio_data.pwd_port);

	return 0;
}

static int sensor_preview_ov7690(struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_common[1], vdev);
}

static int sensor_capture_ov7690(struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_common[2], vdev);
}

static int sensor_capturecfg_ov7690(int width, int height, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_zoom_ov7690(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_autofocus_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_af[val], vdev);
}

static int sensor_effect_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_effect[val], vdev);
}

static int sensor_flip_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_flip[val], vdev);
}

static int sensor_iso_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_iso[val], vdev);
}

static int sensor_me_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_metering_exposure[val], vdev);
}

static int sensor_wb_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_awb[val], vdev);
}

static int sensor_bright_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_brightness[val], vdev);
}

static int sensor_scene_ov7690(int val, struct tcc_camera_device * vdev)
{
	return write_regs_ov7690(sensor_reg_scene[val], vdev);
}

static int sensor_check_esd_ov7690(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_check_luma_ov7690(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

/**********************************************************
*    Function  
**********************************************************/
void sensor_info_init_ov7690(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info->preview_w 				= 640;
	sensor_info->preview_h 				= 480;
	sensor_info->preview_zoom_offset_x 	= 8;
	sensor_info->preview_zoom_offset_y 	= 6;
	sensor_info->capture_w 				= 640;
	sensor_info->capture_h 				= 480;
	sensor_info->capture_zoom_offset_x 	= 8;
	sensor_info->capture_zoom_offset_y 	= 6;
	sensor_info->max_zoom_step 			= 15;
	sensor_info->cam_capchg_width 		= 640;
	sensor_info->framerate				= 15;
	sensor_info->p_clock_pol 				= NEGATIVE_EDGE;
	sensor_info->v_sync_pol 				= ACT_LOW;
	sensor_info->h_sync_pol                 = ACT_HIGH;
	sensor_info->de_pol                 = ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en				= OFF;
	sensor_info->conv_en 					= OFF;
	sensor_info->hsde_connect_en 			= OFF;
	sensor_info->vs_mask 					= OFF;
	sensor_info->input_fmt 				= FMT_YUV422_8BIT;
	sensor_info->data_order 				= ORDER_RGB;
	sensor_info->intl_en					= OFF;
	sensor_info->intpl_en	 				= OFF;	
	sensor_info->format 					= M420_ZERO;
	sensor_info->capture_skip_frame 		= 1;
	sensor_info->sensor_sizes 			= sensor_sizes_ov7690;
}

void sensor_init_fnc_ov7690(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_func->Open 					= sensor_open_ov7690;
	sensor_func->Close 					= sensor_close_ov7690;
	sensor_func->PowerDown				= sensor_powerdown_ov7690;

	sensor_func->Set_Preview 			= sensor_preview_ov7690;
	sensor_func->Set_Capture 			= sensor_capture_ov7690;
	sensor_func->Set_CaptureCfg 			= sensor_capturecfg_ov7690;

	sensor_func->Set_Zoom 				= sensor_zoom_ov7690;
	sensor_func->Set_AF 				= sensor_autofocus_ov7690;
	sensor_func->Set_Effect 				= sensor_effect_ov7690;
	sensor_func->Set_Flip 				= sensor_flip_ov7690;
	sensor_func->Set_ISO 				= sensor_iso_ov7690;
	sensor_func->Set_ME 				= sensor_me_ov7690;
	sensor_func->Set_WB 				= sensor_wb_ov7690;
	sensor_func->Set_Bright 				= sensor_bright_ov7690;
	sensor_func->Set_Scene 				= sensor_scene_ov7690;

	sensor_func->Check_ESD 				= sensor_check_esd_ov7690;
	sensor_func->Check_Luma 			= sensor_check_luma_ov7690;
}

void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init_ov7690(sensor_info);
}

void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc_ov7690(sensor_func);
}

#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init_ov7690(sensor_info);
}

void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc_ov7690(sensor_func);
}

#endif

#endif

