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
#include <mach/daudio_info.h>
#include <mach/gpio.h>
//#include <mach/daudio_settings.h>

#include "../tcc_cam_i2c.h"

#include "daudio_atv.h"
#include "tw9990.h"

#include "../cam_reg.h"

#include "tcc_camera_device.h"

#define PRV_W 720
#define PRV_H 240
#define CAP_W 720
#define CAP_H 240


//#define FEATURE_PROGRESSIVE_SUPPORT



static int sensor_writeRegister(int reg, int val, struct tcc_camera_device * vdev);

static int sensor_readRegister(int reg, struct tcc_camera_device * vdev);

static int aux_encode = 0;	// NTSC

static int debug_tw9990 =0;

#define VPRINTK(fmt, args...) if(debug_tw9990){ printk(KERN_CRIT TAG_DAUDIO_TW9990 fmt, ##args); }

struct sensor_reg sensor_initialize[] = {
	//james.kang@zentech.co.kr 140923
	//cmd, value
	{0x01, 0x78},
    {0x02, 0x40},
    {0x03, 0xA0},
    {0x04, 0x00},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x02},
    {0x08, 0x12},
    {0x09, 0xF0},
    {0x0A, 0x10},
    {0x0B, 0xD0},
    {0x0C, 0xCC},
    {0x0D, 0x00},
    {0x0F, 0x00},
    {0x10, 0x00},
    {0x11, 0x64},
    {0x12, 0x21},
    {0x13, 0x80},
    {0x14, 0x80},
    {0x15, 0x00},
    {0x16, 0x00},
    {0x17, 0x30},
    {0x18, 0x44},
    {0x19, 0x58},
    {0x1A, 0x0A},
    {0x1B, 0x00},
    {0x1C, 0x08},
    {0x1D, 0x7F},
    {0x1E, 0x08},
    {0x1F, 0x00},
    {0x20, 0xA0},
    {0x21, 0x42},
    {0x22, 0xF0},
    {0x23, 0xD8},
    {0x24, 0xBC},
    {0x25, 0xB8},
    {0x26, 0x44},
    {0x27, 0x2A},
    {0x28, 0x00},
    {0x29, 0x00},
    {0x2A, 0x78},
    {0x2B, 0x44},
    {0x2C, 0x40},
    {0x2D, 0x14},
    {0x2E, 0xA5},
    {0x2F, 0xE0},
    {0x30, 0x00},
    {0x31, 0x10},
    {0x32, 0xFF},
    {0x33, 0x05},
    {0x34, 0x1A},
    {0x35, 0xA0},
    {0x4C, 0x05},
    {0x4D, 0x40},
    {0x4E, 0x00},
    {0x4F, 0x00},
    {0x50, 0xA0},
    {0x51, 0x22},
    {0x52, 0x31},
    {0x53, 0x80},
    {0x54, 0x00},
    {0x55, 0x00},
    {0x6B, 0x06},
    {0x6C, 0x24},
    {0x6D, 0x00},
    {0x6E, 0x20},
    {0x6F, 0x13},
    {0xAE, 0x1A},
    {0xAF, 0x80},
	{ REG_TERM, VAL_TERM }
};


static struct sensor_reg TW9990_OPEN[] = {
	{ 0x03, 0xA0 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9990_OPEN_YIN0[] = {		//CVBS Input, MUX0
	{ 0x02, 0x40 },
	{ 0x03, 0xA0 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9990_OPEN_YIN1[] = {		//CVBS INPUT, MUX0
	{ 0x02, 0x40 },
	{ 0x03, 0xA0 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9990_OPEN_YIN2[] = {		//S-Video Input, MUX0
	{ 0x02, 0x50 },
	{ 0x03, 0xA0 },
	{ 0x06, 0x00 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *TW9990_OPEN_YIN[] = {
	TW9990_OPEN_YIN0,
	TW9990_OPEN_YIN1,
	TW9990_OPEN_YIN2
};


static struct sensor_reg TW9990_CLOSE[] = {
	{ 0x02, 0x46 },			// set YIN3 for closing input
	{ 0x03, 0xA7 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9990_sensor_reg_common[] = {
	sensor_initialize,
	TW9990_OPEN,
	TW9990_CLOSE,
};

static struct sensor_reg sensor_brightness_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9990_sensor_reg_brightness[] = {
	sensor_brightness_p0,
};

static struct sensor_reg sensor_contrast_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9990_sensor_reg_contrast[] = {
	sensor_contrast_p0,
};

static struct sensor_reg sensor_saturation_p0[] = {
	{ REG_TERM, VAL_TERM }
};

//GT system start
static struct sensor_reg sensor_u_gain_p0[] = {
	{ 0x13, 0x80 },
	{ REG_TERM, VAL_TERM }
};
static struct sensor_reg sensor_v_gain_p0[] = {
	{ 0x14, 0x80 },
	{ REG_TERM, VAL_TERM }
};
static struct sensor_reg sensor_sharpness_p0[] = {
	{ 0x12, 0x11 },
	{ REG_TERM, VAL_TERM }
};
//GT system end

struct sensor_reg *tw9990_sensor_reg_saturation[] = {
	sensor_saturation_p0,
};

//GT system start
struct sensor_reg *tw9990_sensor_reg_u_gain[] = {
	sensor_u_gain_p0,
};
struct sensor_reg *tw9990_sensor_reg_v_gain[] = {
	sensor_v_gain_p0,
};
struct sensor_reg *tw9990_sensor_reg_sharpness[] = {
	sensor_sharpness_p0,
};
//GT system end

static struct sensor_reg sensor_hue_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9990_sensor_reg_hue[] = {
	sensor_hue_p0,
};

static struct sensor_reg tw9990_sensor_reg_backup[] = {
	//james.kang@zentech.co.kr 140923
	//cmd, value
	{0x01, 0x78},
    {0x02, 0x40},
    {0x03, 0xA0},
    {0x04, 0x00},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x02},
    {0x08, 0x12},
    {0x09, 0xF0},
    {0x0A, 0x10},
    {0x0B, 0xD0},
    {0x0C, 0xCC},
    {0x0D, 0x00},
    {0x0F, 0x00},
    {0x10, 0x00},
    {0x11, 0x64},
    {0x12, 0x21},
    {0x13, 0x80},
    {0x14, 0x80},
    {0x15, 0x00},
    {0x16, 0x00},
    {0x17, 0x30},
    {0x18, 0x44},
    {0x19, 0x58},
    {0x1A, 0x0A},
    {0x1B, 0x00},
    {0x1C, 0x08},
    {0x1D, 0x7F},
    {0x1E, 0x08},
    {0x1F, 0x00},
    {0x20, 0xA0},
    {0x21, 0x42},
    {0x22, 0xF0},
    {0x23, 0xD8},
    {0x24, 0xBC},
    {0x25, 0xB8},
    {0x26, 0x44},
    {0x27, 0x2A},
    {0x28, 0x00},
    {0x29, 0x00},
    {0x2A, 0x78},
    {0x2B, 0x44},
    {0x2C, 0x40},
    {0x2D, 0x14},
    {0x2E, 0xA5},
    {0x2F, 0xE0},
    {0x30, 0x00},
    {0x31, 0x10},
    {0x32, 0xFF},
    {0x33, 0x05},
    {0x34, 0x1A},
    {0x35, 0xA0},
    {0x4C, 0x05},
    {0x4D, 0x40},
    {0x4E, 0x00},
    {0x4F, 0x00},
    {0x50, 0xA0},
    {0x51, 0x22},
    {0x52, 0x31},
    {0x53, 0x80},
    {0x54, 0x00},
    {0x55, 0x00},
    {0x6B, 0x06},
    {0x6C, 0x24},
    {0x6D, 0x00},
    {0x6E, 0x20},
    {0x6F, 0x13},
    {0xAE, 0x1A},
    {0xAF, 0x80},
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9990_sensor_path_rear_camera[] = {
	{ 0x07, 0x02 },
	{ 0x09, 0xf0 }, // set on init
	{ 0x1c, 0x08 },
	{ 0x1d, 0x7f },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9990_sensor_path_aux_ntsc[] = {
	{ 0x07, 0x02 },
	{ 0x09, 0xf0 },
	{ 0x1c, 0x08 },
	{ 0x1d, 0x7f },
	{ 0x02, 0x40 }, // set yin as YIN0
    { 0x03, 0xA0 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9990_sensor_path_aux_pal[] = {
	{ 0x07, 0x12 },
	{ 0x09, 0x20 },
	{ 0x1c, 0x09 },
	{ 0x1d, 0x7f },
	{ 0x02, 0x40 }, // set yin as YIN0
    { 0x03, 0xA0 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9990_sensor_reg_path[] = {
	tw9990_sensor_path_rear_camera,
	tw9990_sensor_path_aux_ntsc,
	tw9990_sensor_path_aux_pal
};

#ifdef CONFIG_DAUDIO
static struct sensor_reg cvbs_composite[] = {
	{0x02, 0x40},
	{0x03, 0xA0},
	{0x06, 0x00},
	{0xFF, 0xFF}
};

static struct sensor_reg s_video[] = {
	{0x02, 0x50},
	{0x03, 0xA0},
	{0x06, 0x00},
	{0xFF, 0xFF}
};

static struct sensor_reg cvbs_component[] = {
	{0xFF, 0xFF}
};

static struct sensor_reg sensor_regs_stop[] = {
	{0x03, 0xA7},
	{0xFF, 0xFF}
};

struct sensor_reg * sensor_regs_type_and_encode[CAM_TYPE_MAX][CAM_ENC_MAX] = {
	// CAM_TYPE_DEFAULT
	{
		cvbs_composite, 	//cvbs_ntsc,
		NULL
	},
	{
		s_video,			//svideo_ntsc,
		NULL
	},
	{
		cvbs_component,		//component_ntsc,
		NULL
	},
};
#endif

int tw9990_read_ie(int cmd, unsigned char *level, struct tcc_camera_device * vdev);

static int getVideoType(void)
{
	int videoType;

	/* CVBS : 0, S_VIDEO : 1, COMPONENT : 2*/
	//videoType = ((gpio_get_value(TCC_GPF(26))<<1)|gpio_get_value(TCC_GPF(27)));
       videoType = ((gpio_get_value(TCC_GPB(19))<<1)|gpio_get_value(TCC_GPB(23)));

	if(videoType > 2)
		videoType = CVBS;

	return videoType;
}

static int getCamType(void)
{
	int camType = DIRECT_CAM;
	
	switch(getVideoType())
	{
		case S_VIDEO:
		case COMPONENT:
			camType = SVM_CAM;
			break;
		default:
			camType = DIRECT_CAM;
			break;
	}

	return camType;
}

static void tw9990_sensor_get_preview_size(int *width, int *height)
{
	*width = PRV_W;
	*height = PRV_H;
}

static void tw9990_sensor_get_capture_size(int *width, int *height)
{
	*width = CAP_W;
	*height = CAP_H;
}

static int write_regs(const struct sensor_reg reglist[], struct tcc_camera_device * vdev)
{
	int err, err_cnt = 0, backup_cnt;
	unsigned char data[132];
	unsigned char bytes;
	const struct sensor_reg *next = reglist;

	VPRINTK("TW9990 FARMER Sensor I2C !!!!START \n"); 
	
	while (!((next->reg == REG_TERM) && (next->val == VAL_TERM))) {
		bytes = 0;
		data[bytes]= (u8)next->reg & 0xff; 	bytes++;
		data[bytes]= (u8)next->val & 0xff; 	bytes++;

		VPRINTK("TW9990 REG : 0x%x, VAL : 0x%x \n", data[0], data[1]); 
		err = DDI_I2C_Write(data, 1, 1,vdev);
		if (err) {
			err_cnt++;
			if(err_cnt >= 3) {
				printk(KERN_ERR "ERROR: Sensor I2C !!!! \n"); 
				return err;
			}
		} else {
			for (backup_cnt = 0; backup_cnt < ARRAY_SIZE(tw9990_sensor_reg_backup); backup_cnt++) {
				if (tw9990_sensor_reg_backup[backup_cnt].reg == next->reg) {
					tw9990_sensor_reg_backup[backup_cnt].val = next->val;
				}
			}

			err_cnt = 0;
			next++;
		}
	}
	
	VPRINTK("TW9990 FARMER Sensor I2C !!!! END \n"); 

	return 0;
}

static int sensor_writeRegister(int reg, int val, struct tcc_camera_device * vdev)
{
	int ret;
	unsigned char data[2];

	data[0]= (u8)reg & 0xff;
	data[1]= (u8)val & 0xff;
	ret = DDI_I2C_Write(data, 1, 1,vdev);

	return ret;
}

static int sensor_readRegister(int reg, struct tcc_camera_device * vdev)
{
	char ret;
	DDI_I2C_Read((unsigned short)reg, 1, &ret, 1, vdev);
	return ret;
}

static int tw9990_read(int reg, struct tcc_camera_device * vdev)
{
	int i = I2C_RETRY_COUNT;
	unsigned char ret;
	do {
		int retval = DDI_I2C_Read((unsigned short)reg, 1, &ret, 1, vdev);
		if (retval == 0)
		{
			//success to I2C.
			return ret;
		}
		i--;
	} while(i > 0);

	return FAIL_I2C;
}

static int tw9990_write(unsigned char* data, unsigned char reg_bytes, unsigned char data_bytes, struct tcc_camera_device * vdev)
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
#ifdef CONFIG_DAUDIO
int tw9990_sensor_type(unsigned int camera_type, unsigned int camera_encode, struct tcc_camera_device * vdev) {
	VPRINTK("============%s\n", __func__);

	if(((CAM_TYPE_MAX <= camera_type) || (sensor_regs_type_and_encode[camera_type][camera_encode] == NULL)) || 
	   ((CAM_ENC_MAX <= camera_encode) || (sensor_regs_type_and_encode[camera_type][camera_encode] == NULL))) {
		VPRINTK( "!@#---- %s() - WRONG arguments\n", __func__);
		return -1;
	}
	return write_regs(sensor_regs_type_and_encode[camera_type][camera_encode],vdev);
}
#endif
static int tw9990_sensor_open(eTW_YSEL yin, struct tcc_camera_device * vdev)
{
	VPRINTK("%s yin=%d\n", __func__, yin);
	
#if 0//def CONFIG_DAUDIO
	gpio_set_value_cansleep(vdev->gpio_data.rst_port,0);
	sensor_delay(15);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,1);
	sensor_delay(15);

	write_regs(tw9990_sensor_reg_common[MODE_TW9990_INIT],vdev);
#endif
	
	if (yin == TW_YIN0)	// AUX
	{
		return write_regs(tw9990_sensor_reg_common[MODE_TW9990_CLOSE],vdev);
	}

	return write_regs(TW9990_OPEN_YIN[yin],vdev);
}

static int tw9990_sensor_close( struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);
	return write_regs(tw9990_sensor_reg_common[MODE_TW9990_CLOSE],vdev);
}

int tw9990_read_id( struct tcc_camera_device * vdev)
{
	return tw9990_read(TW9990_REG_ID,vdev);
}

static int tw9990_sensor_preview( struct tcc_camera_device * vdev)
{
	int id = tw9990_read_id(vdev);
	VPRINTK("%s read id: 0x%x\n", __func__, id);

	return 0;
}

static int tw9990_sensor_resetVideoDecoder(struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,0);
	sensor_delay(10);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,1);
	sensor_delay(10);

	return write_regs(tw9990_sensor_reg_backup,vdev);
}

void tw9990_close_cam( struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);

	tw9990_sensor_close(vdev);
}

int tw9990_init(void)
{
	VPRINTK("%s\n", __func__);

	return 0;
}

int tw9990_check_video_signal(struct tcc_camera_device * vdev)
{
	int ret = 0, cstatus;
	unsigned char vdloss, hlock, slock, vlock, mono;

	cstatus = tw9990_read(TW9990_REG_CSTATUS,vdev);
	if (cstatus >= FAIL)
		//error
		return cstatus;

	/*if (aux_encode != (cstatus & 0x1))
	{
	    printk("[%s] invalid format (%d:0x%x)\n", __func__, aux_encode, cstatus);
	    cstatus = 0;
	}*/

	vdloss = (unsigned char)cstatus & VDLOSS ? 1 : 0;
	hlock = (unsigned char)cstatus & HLOCK ? 1 : 0;
	slock = (unsigned char)cstatus & SLOCK ? 1 : 0;
	vlock = (unsigned char)cstatus & VLOCK ? 1 : 0;
	mono = (unsigned char)cstatus & MONO ? 1 : 0;

	ret = (!vdloss) && hlock && slock && vlock && (!mono);

	if(!ret)
		printk("[%s](%x) vdloss : %d, hlock : %d, slock : %d, vlock : %d, mono : %d\n", __func__, cstatus, vdloss, hlock, slock, vlock, mono);

	return ret;
}

int tw9990_display_mute(int mute)
{
	printk(KERN_INFO "%s TW9990 not support display mute.\n", __func__);

	return FAIL;
}

int tw9990_write_ie(int cmd, unsigned char value, struct tcc_camera_device * vdev)
{
	int ret = FAIL;
	unsigned char data[2] = {0x0, };
	/*ie_setting_info setting_info,setting_info_read;*/

	//printk(KERN_CRIT "%s cmd: %d value: 0x%x\n", __func__, cmd, value);

/*
	read_ie_setting(&setting_info);
	msleep(200);*/

	if (cmd < SET_CAM_BRIGHTNESS || cmd > SET_AUX_SATURATION)
	{
		printk(KERN_CRIT "%s cmd: cmd < SET_CAM_BRIGHTNESS || cmd > SET_AUX_SATURATION return ~~\n", __func__);
		return ret;
	}	
	switch (cmd)
	{
		case SET_AUX_BRIGHTNESS:
		case SET_CAM_BRIGHTNESS:
			data[0] = TW9990_I2C_BRIGHTNESS;
			/*setting_info.twxxxx_cam_brightness = value - 128;*/
			break;
		case SET_AUX_CONTRAST:
		case SET_CAM_CONTRAST:
			data[0] = TW9990_I2C_CONTRAST;
			/*setting_info.twxxxx_cam_contrast=value;*/
			break;
		case SET_AUX_GAMMA:
		case SET_CAM_HUE:
			data[0] = TW9990_I2C_HUE;
			/*setting_info.twxxxx_cam_hue= value - 128;*/
			break;

	/*	case SET_CAM_SHARPNESS:
			data[0] = TW9990_I2C_SHARPNESS;
			
			break;

		case SET_CAM_U_GAIN:
			data[0] = TW9990_I2C_SATURATION_U;
			break;
		case SET_CAM_V_GAIN:
			data[0] = TW9990_I2C_SATURATION_V;
			break;
	*/
		case SET_AUX_SATURATION:
		case SET_CAM_SATURATION:
			data[0] = TW9990_I2C_SATURATION_U;
			/*setting_info.twxxxx_cam_saturation= value;*/
			break;
	}
	
	/*if(cmd == SET_CAM_SHARPNESS)
	{	
		int val=0;
		char lev;
		val = tw9990_read_ie( GET_CAM_SHARPNESS , &lev);
		printk(KERN_CRIT "[GT system] %s, set cam sharpness resd =%d \n", __func__, lev);
		if(val != SUCCESS)
			return ret ;
		data[1] = lev ;       // 4~7 bit setting
		data[1] = 0x0f&value; // SHARPNESS 0~3 bit
	}
	else*/
	{
		data[1] = value;
		//printk(KERN_CRIT "[GT system] %s value = %d\n", __func__, value);
	}

	if ((cmd == SET_CAM_BRIGHTNESS || cmd == SET_CAM_HUE) ||
		(cmd == SET_AUX_BRIGHTNESS || cmd == SET_AUX_GAMMA)   )
		data[1] -= 128;


	ret = tw9990_write(data, 1, 1,vdev);

	printk(KERN_CRIT "[GT system] %s , data[1]= %d,  ret = %d\n", __func__,data[1], ret );
	if( (ret == SUCCESS_I2C && cmd == SET_CAM_SATURATION) ||
		(ret == SUCCESS_I2C && cmd == SET_AUX_SATURATION) )
	{
		data[0] = TW9990_I2C_SATURATION_V;
		ret = tw9990_write(data, 1, 1,vdev);
	}

/*
	VPRINTK( "1 WRITE [GT system] %s , cam_brightness= %d,  cam_contrast = %d,cam_hue =%d, cam_saturation= %d\n", __func__,setting_info.twxxxx_cam_brightness, setting_info.twxxxx_cam_contrast,setting_info.twxxxx_cam_hue,setting_info.twxxxx_cam_saturation);

	ret = write_ie_setting(&setting_info);

	  if (!ret) {
		ret = -EINVAL;
		printk(KERN_ERR "[GT system] %s , write_ie_setting FAIL~~!!\n", __func__ );
	}
*/
	if (ret >= FAIL)
		ret = FAIL;
	else
		ret = SUCCESS;

	return ret;
}

int tw9990_read_ie(int cmd, unsigned char *level, struct tcc_camera_device * vdev)
{
	int ret = FAIL;
	int value_u = 0, value_v = 0;
	VPRINTK("%s cmd: %d\n", __func__, cmd);

	switch (cmd)
	{
		case  GET_BRIGHTNESS:
		case  GET_CONTRAST:
		case  GET_SATURATION:
		case  GET_HUE:
			ret = SUCCESS;
			break;

		case GET_CAM_BRIGHTNESS:
		case GET_AUX_BRIGHTNESS:
			ret = tw9990_read(TW9990_I2C_BRIGHTNESS,vdev);
			break;

		case GET_CAM_CONTRAST:
		case GET_AUX_CONTRAST:
			ret = tw9990_read(TW9990_I2C_CONTRAST,vdev);
			break;

		case GET_CAM_HUE:
		case GET_AUX_GAMMA:
			ret = tw9990_read(TW9990_I2C_HUE,vdev);
			break;

		case GET_CAM_SATURATION:
		case GET_AUX_SATURATION:
			value_u = tw9990_read(TW9990_I2C_SATURATION_U,vdev);
			//printk(KERN_CRIT "[GT system] %s , value_u= %d \n", __func__ , value_u);
			value_v = tw9990_read(TW9990_I2C_SATURATION_V,vdev);
			//printk(KERN_CRIT "[GT system] %s , value_v= %d \n", __func__ , value_v);

			if (value_u == FAIL_I2C || value_v == FAIL_I2C)
				ret = FAIL_I2C;
			else
				ret = value_u;
			break;
		/*
		case GET_CAM_U_GAIN:
			ret = tw9990_read(TW9990_I2C_SATURATION_U);
			printk(KERN_CRIT "[GT system] %s , value_u= %d \n", __func__ , ret);
			break;			
		case GET_CAM_V_GAIN:
			ret = tw9990_read(TW9990_I2C_SATURATION_V);
			printk(KERN_CRIT "[GT system] %s , value_v= %d \n", __func__ , ret);
			break;
		case GET_CAM_SHARPNESS:
			ret = tw9990_read(TW9990_I2C_SHARPNESS);
			break;*/
		default:
			break;
			
	}

	if (ret >= FAIL)
	{
		ret = FAIL; 
		*level = 0;
	}
	else
	{
		*level = ret;
		if ((cmd == GET_CAM_BRIGHTNESS || cmd == GET_CAM_HUE) ||
			(cmd == GET_AUX_BRIGHTNESS || cmd == GET_AUX_GAMMA))
			*level += 128;

		ret = SUCCESS;
	}
	//printk(KERN_CRIT "[GT system] %s , ret=%d : level= %d \n", __func__ , ret ,*level);

	return ret;
}

static int tw9990_sensor_capture(void)
{
	return 0;
}

static int tw9990_sensor_capturecfg(int width, int height)
{
	return 0;
}

static int sensor_zoom(int val)
{
	return 0;
}

static int sensor_autofocus(int val)
{
	return 0;
}

static int sensor_effect(int val)
{
	return 0;
}

static int sensor_flip(int val)
{
	return 0;
}

static int sensor_iso(int val)
{
	return 0;
}

static int sensor_me(int val)
{
	return 0;
}

static int sensor_wb(int val)
{
	return 0;
}

static int sensor_bright(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_brightness[val],vdev);
}

static int sensor_scene(int val,struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_contrast(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_contrast[val],vdev);
}

static int sensor_saturation(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_saturation[val],vdev);
}

//GT system start
static int sensor_u_gain(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_u_gain[val],vdev);
}
static int sensor_v_gain(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_v_gain[val],vdev);
}
static int sensor_sharpness(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_sharpness[val],vdev);
}
//GT system end

static int sensor_hue(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9990_sensor_reg_hue[val],vdev);
}

static int sensor_getVideo(struct tcc_camera_device * vdev)
{
	char ret;

	DDI_I2C_Read(0x01, 1, &ret, 1,vdev);
	if ((ret & 0x80) == 0x00)  {
		ret = 0; // detected video signal
	} else {
		ret = 1; // not detected video signal
	}
	
	return ret;
}

static int sensor_checkNoise(struct tcc_camera_device * vdev)
{
	char ret;

	DDI_I2C_Read(0x01, 1, &ret, 1,vdev);
	if ((ret & 0xE8) == 0x68)  {
		ret = 0; // not detected video noise
	} else {
		ret = 1; // detected video noise
	}
	
	return ret;
}

static int tw9990_sensor_setPath(int val, struct tcc_camera_device * vdev)
{
	int ret;

	aux_encode = (val == 2) ? 1 : 0;

	printk("%s val = %d\n", __func__, val);	
	
	ret = write_regs(tw9990_sensor_reg_path[val],vdev);

	if (val != 0) {
		int cstatus;
		msleep(200);
		cstatus = tw9990_read(TW9990_REG_CSTATUS,vdev);
		printk("%s signal status 0x%x\n", __func__, cstatus);
	}
	
	return ret;
}

static int tw9990_sensor_getVideoFormat(void)
{
	char ret = 0;
	//TODO

	return ret;
}

static int tw9990_sensor_check_esd(int val)
{
	return 0;
}

static int tw9990_sensor_check_luma(int val)
{
	return 0;
}

void sensor_info_init(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	VPRINTK("[TW9990] %s()\n", __func__);

#ifdef CONFIG_DAUDIO
	sensor_info->preview_w				= 720;
	sensor_info->preview_h				= 240;
	
	sensor_info->cam_capchg_width		= 720;
	sensor_info->framerate				= 15;
	sensor_info->p_clock_pol				= NEGATIVE_EDGE;
	sensor_info->v_sync_pol 				= ACT_HIGH;
	sensor_info->h_sync_pol 				= ACT_HIGH;
	sensor_info->de_pol 					= ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en				= OFF;
#if defined(CONFIG_LVDS_CAMERA_BT601) || defined(INCLUDE_LVDS_CAMERA_BT601)
	sensor_info->conv_en				= OFF;
	sensor_info->hsde_connect_en		= ON;
#else
	sensor_info->conv_en				= ON;
	sensor_info->hsde_connect_en		= OFF;
#endif
	sensor_info->vs_mask				= OFF;
	sensor_info->input_fmt				= FMT_YUV422_8BIT;
	sensor_info->data_order 				= ORDER_RGB;
	sensor_info->intl_en					= ON;
	sensor_info->intpl_en					= OFF;	
	sensor_info->format 					= M420_ZERO;
	sensor_info->capture_skip_frame 		= 1;
	sensor_info->sensor_sizes				= sensor_sizes;

#else
	sensor_info->preview_w				= 720;
	sensor_info->preview_h				= 480;
	
	sensor_info->p_clock_pol 				= NEGATIVE_EDGE;//POSITIVE_EDGE;//
	sensor_info->v_sync_pol 				= ACT_HIGH;
	sensor_info->h_sync_pol                 	= ACT_HIGH;
	sensor_info->de_pol                 		= ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en				= OFF;
	sensor_info->conv_en 				= ON;
	sensor_info->hsde_connect_en 		= OFF;
	sensor_info->vs_mask 				= OFF;
	sensor_info->input_fmt 				= FMT_YUV422_8BIT;
	sensor_info->data_order 				= ORDER_RGB;
	sensor_info->intl_en					= OFF;
	sensor_info->intpl_en	 				= OFF;	
	sensor_info->format 					= M420_ZERO;
#endif	
}
void tw9990_sensor_init_fnc(SENSOR_FUNC_TYPE_DAUDIO *sensor_func)
{
	sensor_func->sensor_open				= tw9990_sensor_open;
	sensor_func->sensor_close				= tw9990_sensor_close;

	sensor_func->sensor_preview				= tw9990_sensor_preview;
	sensor_func->sensor_capture				= tw9990_sensor_capture;
	sensor_func->sensor_capturecfg			= tw9990_sensor_capturecfg;
#ifdef CONFIG_DAUDIO
	sensor_func->Set_CameraMode			= tw9990_sensor_type;
#endif	
	sensor_func->sensor_reset_video_decoder	= tw9990_sensor_resetVideoDecoder;
	sensor_func->sensor_init				= tw9990_init;
	sensor_func->sensor_read_id				= tw9990_read_id;
	sensor_func->sensor_check_video_signal	= tw9990_check_video_signal;
	sensor_func->sensor_display_mute		= tw9990_display_mute;
	sensor_func->sensor_close_cam			= tw9990_close_cam;
	sensor_func->sensor_write_ie			= tw9990_write_ie;
	sensor_func->sensor_read_ie				= tw9990_read_ie;
	sensor_func->sensor_get_preview_size	= tw9990_sensor_get_preview_size;
	sensor_func->sensor_get_capture_size	= tw9990_sensor_get_capture_size;

	sensor_func->Set_Zoom					= sensor_zoom;
	sensor_func->Set_AF						= sensor_autofocus;
	sensor_func->Set_Effect					= sensor_effect;
	sensor_func->Set_Flip					= sensor_flip;
	sensor_func->Set_ISO					= sensor_iso;
	sensor_func->Set_ME						= sensor_me;
	sensor_func->Set_WB						= sensor_wb;
	sensor_func->Set_Bright					= sensor_bright;
	sensor_func->Set_Scene					= sensor_scene;
	sensor_func->Set_Contrast				= sensor_contrast;
	sensor_func->Set_Saturation				= sensor_saturation;

	sensor_func->Set_UGain					= sensor_u_gain; //GT system start
	sensor_func->Set_VGain					= sensor_v_gain;
	sensor_func->Set_Sharpness				= sensor_sharpness; //GT system end

	sensor_func->Set_Hue					= sensor_hue;
	sensor_func->Get_Video					= sensor_getVideo;
	sensor_func->Check_Noise				= sensor_checkNoise;

	sensor_func->Set_Path					= tw9990_sensor_setPath;
	sensor_func->Get_videoFormat			= tw9990_sensor_getVideoFormat;
	sensor_func->Set_writeRegister			= sensor_writeRegister;
	sensor_func->Get_readRegister			= sensor_readRegister;

	sensor_func->Check_ESD					= NULL;
	sensor_func->Check_Luma					= NULL;
}

void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init(sensor_info);
}

void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func)
{
	tw9990_sensor_init_fnc(sensor_func);
}

#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init(sensor_info);
}
void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func)
{
	tw9990_sensor_init_fnc(sensor_func);
}
#endif
