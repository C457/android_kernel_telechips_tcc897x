/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

               CAMERA    API    M O D U L E

                        EDIT HISTORY FOR MODULE

when        who       what, where, why
--------    ---       -------------------------------------------------------
10/xx/08  Telechips   Created file.
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*===========================================================================

                           INCLUDE FILES FOR MODULE

===========================================================================*/

#include <linux/delay.h>
#include <mach/daudio.h>
#include <mach/daudio_debug.h>
#include <linux/gpio.h>

#include "../tcc_cam_i2c.h"

#include <mach/gpio.h>
#include <mach/vioc_vin.h>

#include "tcc_camera_device.h"

#include "../cam_reg.h"

#include "daudio_atv.h"
#include "daudio_lvds.h"

#define PRV_W 1920 
#define PRV_H 720
#define CAP_W 1920
#define CAP_H 720

#define MULTI_NUM 1000
#define SATURATION_RANGE   	4 // -X ~ +X
#define CONTRAST_RANGE   	4 // -X ~ +X

int rgb[257] = {0, };
static int _video_brightness = 127;
static int _video_contrast = 127;
static int _video_gamma = -1;
static int _video_saturation = 127;

static int debug_lvds	= DEBUG_DAUDIO_LVDS;

#define VPRINTK(fmt, args...) if (1) printk(KERN_INFO TAG_DAUDIO_LVDS fmt, ##args)

static int sensor_writeRegister(int reg, int val, struct tcc_camera_device * vdev);
static int sensor_readRegister(int reg, struct tcc_camera_device * vdev);
int lvds_write_ie(int cmd, unsigned char value, struct tcc_camera_device * vdev);

static struct capture_size sensor_size[] = {
	{1920, 720}
};

int get_gpioG14(void)
{
	int gpioG14 = -1;
	gpioG14 = gpio_get_value(TCC_GPG(14));
	return 1;// gpioD17;
}

static void lvds_sensor_get_preview_size(int *width, int *height)
{
	*width = PRV_W;
	*height = PRV_H;
}

static void lvds_sensor_get_capture_size(int *width, int *height)
{
	*width = CAP_W;
	*height = CAP_H;
}

static int write_regs(const struct sensor_reg reglist[])
{
	VPRINTK("%s-\n", __func__);
		return 0;
}

static int sensor_writeRegister(int reg, int val, struct tcc_camera_device * vdev)
{
	int ret;
	unsigned char data[2];
	VPRINTK("%s-\n", __func__);
	
	data[0]= (u8)reg & 0xff;
	data[1]= (u8)val & 0xff;
	ret = DDI_I2C_Write(data, 1, 1,vdev);

	return ret;
}

static int sensor_readRegister(int reg, struct tcc_camera_device * vdev)
{
	char ret;
	DDI_I2C_Read((unsigned short)reg, 1, &ret, 1,vdev);
	return ret;
}

static unsigned int lvds_read(unsigned short reg, struct tcc_camera_device * vdev)
{
	int i = I2C_RETRY_COUNT;
	unsigned char ret;
	int loop  ;
	int retval; 

	//getChipAddress();

	do {
		retval = DDI_I2C_Read(reg , 1, &ret, 1,vdev );
		if (retval == 0 )
		{
			//success to I2C.
			return ret;
		}
		i--;
	} while(i > 0);

	return FAIL_I2C;
}

static int lvds_write(unsigned char* data, unsigned char reg_bytes, unsigned char data_bytes, struct tcc_camera_device * vdev)
{
	int i = I2C_RETRY_COUNT;
	do {
		int ret = DDI_I2C_Write(data, reg_bytes, data_bytes,vdev);
		if (ret == 0)
		{
			//success to I2C.
			return SUCCESS_I2C;
		}
		i--;
	} while(i > 0);

	return FAIL_I2C;
}

static int lvds_sensor_open(eTW_YSEL yin, struct tcc_camera_device * vdev)
{

	unsigned int id =0 ; 
	unsigned int status =0;
	unsigned char val[2];

	id	= lvds_read(LVDS_REG_ID, vdev);
	VPRINTK("%s, LVDS_REG_ID = 0x%x\n", __func__, id);

	if( id == LVDS_VAL_ID )
	{
		val[0]	= 0x04; //main_config
		val[1]	= lvds_read(val[0], vdev);
		VPRINTK("%s, OLD LVDS_REG_DESLOCK = 0x%x \n", __func__, val[1]);

		val[1]	|= (unsigned char)0x03;
		lvds_write(val, 1, 1, vdev);
				
		val[1]	= lvds_read(val[0], vdev);
		VPRINTK("%s, NEW LVDS_REG_DESLOCK = 0x%x \n", __func__, val[1]);

		lvds_write_ie(0,0,0);
	}
	return 0;

}

static int lvds_sensor_close(void)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

int lvds_read_id(struct tcc_camera_device * vdev)
{
 	unsigned int id =0 ; 
	unsigned int status =0;

	//VPRINTK("%s-----------------------------------------------\n", __func__);
	id	= lvds_read(LVDS_REG_ID, vdev);
	VPRINTK("%s, LVDS_REG_ID = %d\n", __func__, id	);

	return id;
}

static int lvds_sensor_preview(void)
{
	VPRINTK("%s-\n", __func__);
 	return 0;
}

static int lvds_sensor_resetVideoDecoder(struct tcc_camera_device * vdev)
{
	return 0;
}

void lvds_close_cam(struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);

	unsigned int id =0 ; 
	unsigned int status =0;
	unsigned char val[2];

	id	= lvds_read(LVDS_REG_ID, vdev);
	VPRINTK("%s, LVDS_REG_ID = 0x%x\n", __func__, id);

	if( id == LVDS_VAL_ID )
	{
		val[0]	= 0x04; //main_config
		val[1]	= lvds_read(val[0], vdev);
		VPRINTK("%s, OLD LVDS_REG_DESLOCK = 0x%x \n", __func__, val[1]);

		val[1]	&= (unsigned char)(~0x03);
		lvds_write(val, 1, 1, vdev);
				
		val[1]	= lvds_read(val[0], vdev);
		VPRINTK("%s, NEW LVDS_REG_DESLOCK = 0x%x \n", __func__, val[1]);
	}
	return 0;
}

int lvds_init(void)
{
	VPRINTK("%s\n", __func__);
	return 0;
}

int lvds_check_video_signal(struct tcc_camera_device * vdev)
{
	int status = 0;

	status  = gpio_get_value(TCC_GPB(17));

	if(!status)
		VPRINTK("%s, LVDS_DESLOCK(gpiob17) = %d \n", __func__,  status);
	
	return status;
}

int lvds_display_mute(int mute)
{
	printk(KERN_INFO "%s lvds not support display mute.\n", __func__);

	return FAIL;
}

int lvds_write_ie(int cmd, unsigned char value, struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);

	int ret = FAIL;
	int vR, vG, vB, i;
	int fsaturation, fcontrast;
	VIOC_VIN *pVIN =(VIOC_VIN*)0xf3004000;

	switch(cmd)
	{
		case SET_CAM_BRIGHTNESS:
			_video_brightness = value;
			break;
		case SET_CAM_CONTRAST:
			_video_contrast = value;
			break;
		case SET_CAM_SATURATION:
			_video_saturation = value;
			break;
		default:
			break;
	}

	if (_video_brightness < 0 || _video_brightness > 255)
		_video_brightness = 127;
	if (_video_contrast < 0 || _video_contrast > 255)
		_video_contrast = 127;
	if (_video_saturation < 0 || _video_saturation > 255)
		_video_saturation = 127;

	for (i = 0; i < 256; i++)
	{
		//contrast brightness
		if(_video_contrast > 127)
			fcontrast = (1 * MULTI_NUM) + ((_video_contrast -127) * MULTI_NUM * (CONTRAST_RANGE - 1)) / 127;
		else
			fcontrast = (_video_contrast * MULTI_NUM) / 127;

		vR = ((i - 16)*fcontrast)/MULTI_NUM + 16;

		//brightness
		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(_video_brightness >= 127)
			vR = vR + ((_video_brightness - 127) * 2);
		else
			vR = vR + ((_video_brightness - 127) * 2) - 1;

		//saturation
		if(_video_saturation > 127)
			fsaturation = (1 * MULTI_NUM) + ((_video_saturation - 127) * MULTI_NUM * (SATURATION_RANGE - 1)) / 127;
		else
			fsaturation = (_video_saturation * MULTI_NUM) / 127;

		//fsaturation = (1* MULTI_NUM)+ ((((_video_saturation - 127) * MULTI_NUM) / 127) * SATURATION_RANGE);

		vG = vB = (((i - 128)*fsaturation * fcontrast)/MULTI_NUM)/MULTI_NUM + 128;

		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(vG > 255)    vG = 255;
		if(vG < 0)      vG = 0;
		if(vB > 255)    vB = 255;
		if(vB < 0)      vB = 0;

		rgb[i] =((vR & 0xff) << 16) | ((vG & 0xff) << 8) | ((vB & 0xff));

		//VPRINTK("%s count: %d rgb:0x%2x\n", __func__, i, rgb[i]);
	}
	VPRINTK("%s _video_brightness:%d  _video_contrast:%d  _video_saturation:%d \n",
						__func__, _video_brightness, _video_contrast, _video_saturation);

	VIOC_VIN_SetLUT_by_table(pVIN, rgb);

	ret = SUCCESS;
	printk(KERN_INFO "%s,ret = %d \n", __func__ ,ret  );
	return ret;
}

int lvds_read_ie(int cmd, unsigned char *level, struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);

	switch(cmd)
	{
		case GET_CAM_BRIGHTNESS:
			*level = _video_brightness;
			break;
		case GET_CAM_CONTRAST:
			*level = _video_contrast;
			break;
		case GET_CAM_SATURATION:
			*level = _video_saturation;
			break;
		default:
			break;
	}

	return 0;
}

static int lvds_sensor_capture(void)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int lvds_sensor_capturecfg(int width, int height)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_zoom(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_autofocus(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_effect(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_flip(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_iso(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_me(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_wb(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_bright(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_scene(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_contrast(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_saturation(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

//GT system start
static int sensor_u_gain(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}
static int sensor_v_gain(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}
static int sensor_sharpness(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}
//GT system end

static int sensor_hue(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int sensor_getVideo( struct tcc_camera_device * vdev)
{
		char ret;

	DDI_I2C_Read(LVDS_REG_RESET , 1, &ret, 1,vdev);
	if ((ret & 0x80) == 0x00)  {
		ret = 0; // detected video signal
	} else {
		ret = 1; // not detected video signal
	}
	
	return ret;
}

static int sensor_checkNoise( struct tcc_camera_device * vdev)
{
		char ret;

	DDI_I2C_Read(LVDS_REG_RESET , 1, &ret, 1,vdev);
	if ((ret & 0xE8) == 0x68)  {
		ret = 0; // not detected video noise
	} else {
		ret = 1; // detected video noise
	}
	
	return ret;
}
static int lvds_sensor_setPath(int val,struct tcc_camera_device * vdev)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int lvds_sensor_getVideoFormat(void)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int lvds_sensor_check_esd(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

static int lvds_sensor_check_luma(int val)
{
	VPRINTK("%s-\n", __func__);
	return 0;
}

void lvds_sensor_info_init(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	printk("%s - MAX96706 sensor info\n", __func__);

	sensor_info->preview_w				= 1920;
	sensor_info->preview_h				= 720;

	sensor_info->cam_capchg_width			= 1920;
	sensor_info->framerate				= 15;
	//2017.12.22 - LVDS SVM Display Timing Fixed. PCLK(Positive Edge) HS(Active High) VS(Active High)
	//Because of setup time issue, AVN detects PCLK at negative edge
	sensor_info->p_clock_pol			= NEGATIVE_EDGE; //POSITIVE_EDGE;
	sensor_info->v_sync_pol 			= ACT_HIGH;
	sensor_info->h_sync_pol 			= ACT_HIGH;
	sensor_info->de_pol 				= ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en			= OFF;
	sensor_info->conv_en				= OFF;
	sensor_info->hsde_connect_en			= ON;
	sensor_info->vs_mask				= ON;
	sensor_info->input_fmt				= FMT_YUV422_8BIT;
	sensor_info->data_order 			= ORDER_RGB;
	sensor_info->intl_en				= OFF;
	sensor_info->intpl_en				= OFF;
	sensor_info->format 				= M420_ZERO;
	sensor_info->capture_skip_frame 		= 1;
	sensor_info->sensor_sizes			= sensor_size;
}

void lvds_sensor_init_fnc(SENSOR_FUNC_TYPE_DAUDIO *sensor_func)
{
	printk("%s - MAX96706 sensor function\n", __func__);

	sensor_func->sensor_open				= lvds_sensor_open;
	sensor_func->sensor_close				= lvds_sensor_close;

	sensor_func->sensor_preview				= lvds_sensor_preview;
	sensor_func->sensor_capture				= lvds_sensor_capture;
	sensor_func->sensor_capturecfg				= lvds_sensor_capturecfg;
	sensor_func->sensor_reset_video_decoder			= lvds_sensor_resetVideoDecoder;
	sensor_func->sensor_init				= lvds_init;
	sensor_func->sensor_read_id				= lvds_read_id;
	sensor_func->sensor_check_video_signal			= lvds_check_video_signal;
	sensor_func->sensor_display_mute			= lvds_display_mute;
	sensor_func->sensor_close_cam				= lvds_close_cam;
	sensor_func->sensor_write_ie				= lvds_write_ie;
	sensor_func->sensor_read_ie				= lvds_read_ie;
	sensor_func->sensor_get_preview_size			= lvds_sensor_get_preview_size;
	sensor_func->sensor_get_capture_size			= lvds_sensor_get_capture_size;

	sensor_func->Set_Zoom					= sensor_zoom;
	sensor_func->Set_AF					= sensor_autofocus;
	sensor_func->Set_Effect					= sensor_effect;
	sensor_func->Set_Flip					= sensor_flip;
	sensor_func->Set_ISO					= sensor_iso;
	sensor_func->Set_ME					= sensor_me;
	sensor_func->Set_WB					= sensor_wb;
	sensor_func->Set_Bright					= sensor_bright;
	sensor_func->Set_Scene					= sensor_scene;
	sensor_func->Set_Contrast				= sensor_contrast;
	sensor_func->Set_Saturation				= sensor_saturation;
	sensor_func->Set_Hue					= sensor_hue;
	sensor_func->Get_Video					= sensor_getVideo;
	sensor_func->Check_Noise				= sensor_checkNoise;

	sensor_func->Set_Path					= lvds_sensor_setPath;
	sensor_func->Get_videoFormat				= lvds_sensor_getVideoFormat;
	sensor_func->Set_writeRegister				= sensor_writeRegister;
	sensor_func->Get_readRegister				= sensor_readRegister;

	sensor_func->Check_ESD					= NULL;
	sensor_func->Check_Luma					= NULL;
}
