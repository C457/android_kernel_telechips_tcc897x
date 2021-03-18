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

#include "../tcc_cam_i2c.h"

#include "daudio_atv.h"
#include "tw9921.h"

#include "../cam_reg.h"

#include "tcc_camera_device.h"

#define PRV_W 720
#define PRV_H 240
#define CAP_W 720
#define CAP_H 240


//#define FEATURE_PROGRESSIVE_SUPPORT



static int sensor_writeRegister(int reg, int val, struct tcc_camera_device * vdev);

static int sensor_readRegister(int reg,struct tcc_camera_device * vdev);

static int aux_encode = 0;	// NTSC
static int camera_type = DIRECT_CAM;

static int debug_tw9921 =1;

#define VPRINTK(fmt, args...)  printk(KERN_CRIT TAG_DAUDIO_TW9921 fmt, ##args)

static struct sensor_reg TW9921_INIT[] = {
	//james.kang@zentech.co.kr 140923
	//cmd, value
	{ 0x01, 0x68 },
	{ 0x02, 0x44 },
	{ 0x03, 0x27 },
	{ 0x04, 0x00 },
	{ 0x05, 0x0D },
	{ 0x06, 0x03 },
	{ 0x07, 0x02 },
	{ 0x08, 0x16 },
	{ 0x09, 0xF0 },
	{ 0x0A, 0x10 },
	{ 0x0B, 0xD0 },
	{ 0x0C, 0xCC },
	{ 0x10, 0x00 },
	{ 0x11, 0x64 },
	{ 0x12, 0x11 },//GT 0X12 . SHARPNESS CONTROL REGISTER I (SHARPNESS)
	{ 0x13, 0x80 },//GT 0X13 . CHROMA (U) GAIN REGISTER (SAT_U)
	{ 0x14, 0x80 },//GT 0X14 . CHROMA (V) GAIN REGISTER (SAT_V)
	{ 0x15, 0x00 },
	{ 0x17, 0x30 },
	{ 0x18, 0x44 },
	{ 0x1A, 0x10 },
	{ 0x1B, 0x00 },
	{ 0x1C, 0x0F },
	{ 0x1D, 0x7F },
	{ 0x1E, 0x08 },
	{ 0x1F, 0x00 },
	{ 0x20, 0x50 },
	{ 0x21, 0x42 },
	{ 0x22, 0xF0 },
	{ 0x23, 0xD8 },
	{ 0x24, 0xBC },
	{ 0x25, 0xB8 },
	{ 0x26, 0x44 },
	{ 0x27, 0x38 },
	{ 0x28, 0x00 },
	{ 0x29, 0x00 },
	{ 0x2A, 0x78 },
	{ 0x2B, 0x44 },
	{ 0x2C, 0x30 },
	{ 0x2D, 0x14 },
	{ 0x2E, 0xA5 },
	{ 0x2F, 0x26 },
	{ 0x30, 0x00 },
	{ 0x31, 0x10 },
	{ 0x32, 0x00 },
	{ 0x33, 0x05 },
	{ 0x34, 0x1A },
	{ 0x35, 0x00 },
	{ 0x36, 0xE2 },
	{ 0x37, 0x12 },
	{ 0x38, 0x01 },
	{ 0xC0, 0x01 },
	{ 0xC1, 0x07 },
	{ 0xC2, 0x11 },
	{ 0xC3, 0x03 },
	{ 0xC4, 0x5A },
	{ 0xC5, 0x00 },
	{ 0xC6, 0x20 },
	{ 0xC7, 0x04 },
	{ 0xC8, 0x00 },
	{ 0xC9, 0x06 },
	{ 0xCA, 0x06 },
	{ 0xCB, 0x30 },
	{ 0xCC, 0x00 },
	{ 0xCD, 0x54 },
	{ 0xD0, 0x00 },
	{ 0xD1, 0xF0 },
	{ 0xD2, 0xF0 },
	{ 0xD3, 0xF0 },
	{ 0xD4, 0x00 },
	{ 0xD5, 0x00 },
	{ 0xD6, 0x10 },
	{ 0xD7, 0x70 },
	{ 0xD8, 0x00 },
	{ 0xD9, 0x04 },
	{ 0xDA, 0x80 },
	{ 0xDB, 0x80 },
	{ 0xDC, 0x20 },
	{ 0xE0, 0x00 },
	{ 0xE1, 0x05 },
	{ 0xE2, 0xD9 },
	{ 0xE3, 0x00 },
	{ 0xE4, 0x00 },
	{ 0xE5, 0x00 },
	{ 0xE6, 0x00 },
	{ 0xE7, 0x2A },
	{ 0xE8, 0x0F },
	//{ 0xE9, 0x14 }, pclk falling
	{ 0xE9, 0x34 },	//pclk rising
	{ REG_TERM, VAL_TERM }
};

#if defined(FEATURE_PROGRESSIVE_SUPPORT)
static struct sensor_reg TW9921_INIT_REGS_I[] = {
	{ 0x05, 0x0D },
	{ 0x08, 0x13 },
	{ 0x09, 0xF0 },
	{ 0x0A, 0x10 },
	{ 0x36, 0xE2 },
	{ 0xE1, 0x05 },
	{ 0xE9, 0x34 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_INIT_REGS_P[] = {
    { 0x05, 0x10 },
    { 0x08, 0x14 },
	{ 0x09, 0xF3 },
	{ 0x0A, 0x0B },
	{ 0x36, 0xE8 },
	{ 0xE1, 0x45 },
	{ 0xE9, 0x75 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_INIT_REGS_SVM_P[] = {
	{ 0x05, 0x10 },
	{ 0x08, 0x14 },
	{ 0x09, 0xF4 },
	{ 0x0A, 0x0B },
	{ 0x36, 0xE8 },
	{ 0xE1, 0x45 },
	{ 0xE9, 0x75 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *TW9921_INIT_MODE[] = {
	TW9921_INIT_REGS_I,
	TW9921_INIT_REGS_P,
	TW9921_INIT_REGS_SVM_P
};
#endif

static struct sensor_reg TW9921_OPEN[] = {
	{ 0x03, 0x21 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_OPEN_YIN0[] = {
	{ 0x02, 0x40 },
	{ 0x03, 0x21 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_OPEN_YIN1[] = {
	{ 0x02, 0x44 },
	{ 0x03, 0x21 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_OPEN_YIN2[] = {
	{ 0x02, 0x54 },
	{ 0x03, 0x21 },
	{ 0x06, 0x00 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *TW9921_OPEN_YIN[] = {
	TW9921_OPEN_YIN0,
	TW9921_OPEN_YIN1,
	TW9921_OPEN_YIN2
};

#if defined(FEATURE_PROGRESSIVE_SUPPORT)
static struct sensor_reg TW9921_OPEN_YIN0P[] = {
	{ 0x02, 0x40 },
	{ 0x03, 0x20 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_OPEN_YIN1P[] = {
	{ 0x02, 0x44 },
	{ 0x03, 0x20 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg TW9921_OPEN_YIN2P[] = {
    { 0x02, 0x54 },
    { 0x03, 0x20 },
	{ 0x06, 0x00 },
    { REG_TERM, VAL_TERM }
};

struct sensor_reg *TW9921_OPEN_YINP[] = {
    TW9921_OPEN_YIN0P,
	TW9921_OPEN_YIN1P,
	TW9921_OPEN_YIN2P
};
#endif

static struct sensor_reg TW9921_CLOSE[] = {
	{ 0x02, 0x46 },			// set YIN3 for closing input
	{ 0x03, 0x27 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9921_sensor_reg_common[] = {
	TW9921_INIT,
	TW9921_OPEN,
	TW9921_CLOSE,
};

static struct sensor_reg sensor_brightness_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9921_sensor_reg_brightness[] = {
	sensor_brightness_p0,
};

static struct sensor_reg sensor_contrast_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9921_sensor_reg_contrast[] = {
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

struct sensor_reg *tw9921_sensor_reg_saturation[] = {
	sensor_saturation_p0,
};

//GT system start
struct sensor_reg *tw9921_sensor_reg_u_gain[] = {
	sensor_u_gain_p0,
};
struct sensor_reg *tw9921_sensor_reg_v_gain[] = {
	sensor_v_gain_p0,
};
struct sensor_reg *tw9921_sensor_reg_sharpness[] = {
	sensor_sharpness_p0,
};
//GT system end

static struct sensor_reg sensor_hue_p0[] = {
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9921_sensor_reg_hue[] = {
	sensor_hue_p0,
};

static struct sensor_reg tw9921_sensor_reg_backup[] = {
	//james.kang@zentech.co.kr 140923
	//cmd, value
	{ 0x01, 0x68 },
	{ 0x02, 0x44 },
	{ 0x03, 0x27 },
	{ 0x04, 0x00 },
	{ 0x05, 0x0D },
	{ 0x06, 0x03 },
	{ 0x07, 0x02 },
	{ 0x08, 0x16 },
	{ 0x09, 0xF0 },
	{ 0x0A, 0x10 },
	{ 0x0B, 0xD0 },
	{ 0x0C, 0xCC },
	{ 0x10, 0x00 },
	{ 0x11, 0x64 },
	{ 0x12, 0x11 },//GT 0X12 . SHARPNESS CONTROL REGISTER I (SHARPNESS)
	{ 0x13, 0x80 },//GT 0X13 . CHROMA (U) GAIN REGISTER (SAT_U)
	{ 0x14, 0x80 },//GT 0X14 . CHROMA (V) GAIN REGISTER (SAT_V)
	{ 0x15, 0x00 },
	{ 0x17, 0x30 },
	{ 0x18, 0x44 },
	{ 0x1A, 0x10 },
	{ 0x1B, 0x00 },
	{ 0x1C, 0x0F },
	{ 0x1D, 0x7F },
	{ 0x1E, 0x08 },
	{ 0x1F, 0x00 },
	{ 0x20, 0x50 },
	{ 0x21, 0x42 },
	{ 0x22, 0xF0 },
	{ 0x23, 0xD8 },
	{ 0x24, 0xBC },
	{ 0x25, 0xB8 },
	{ 0x26, 0x44 },
	{ 0x27, 0x38 },
	{ 0x28, 0x00 },
	{ 0x29, 0x00 },
	{ 0x2A, 0x78 },
	{ 0x2B, 0x44 },
	{ 0x2C, 0x30 },
	{ 0x2D, 0x14 },
	{ 0x2E, 0xA5 },
	{ 0x2F, 0x26 },
	{ 0x30, 0x00 },
	{ 0x31, 0x10 },
	{ 0x32, 0x00 },
	{ 0x33, 0x05 },
	{ 0x34, 0x1A },
	{ 0x35, 0x00 },
	{ 0x36, 0xE2 },
	{ 0x37, 0x12 },
	{ 0x38, 0x01 },
	{ 0xC0, 0x01 },
	{ 0xC1, 0x07 },
	{ 0xC2, 0x11 },
	{ 0xC3, 0x03 },
	{ 0xC4, 0x5A },
	{ 0xC5, 0x00 },
	{ 0xC6, 0x20 },
	{ 0xC7, 0x04 },
	{ 0xC8, 0x00 },
	{ 0xC9, 0x06 },
	{ 0xCA, 0x06 },
	{ 0xCB, 0x30 },
	{ 0xCC, 0x00 },
	{ 0xCD, 0x54 },
	{ 0xD0, 0x00 },
	{ 0xD1, 0xF0 },
	{ 0xD2, 0xF0 },
	{ 0xD3, 0xF0 },
	{ 0xD4, 0x00 },
	{ 0xD5, 0x00 },
	{ 0xD6, 0x10 },
	{ 0xD7, 0x70 },
	{ 0xD8, 0x00 },
	{ 0xD9, 0x04 },
	{ 0xDA, 0x80 },
	{ 0xDB, 0x80 },
	{ 0xDC, 0x20 },
	{ 0xE0, 0x00 },
	{ 0xE1, 0x05 },
	{ 0xE2, 0xD9 },
	{ 0xE3, 0x00 },
	{ 0xE4, 0x00 },
	{ 0xE5, 0x00 },
	{ 0xE6, 0x00 },
	{ 0xE7, 0x2A },
	{ 0xE8, 0x0F },
	//{ 0xE9, 0x14 }, pclk falling
	{ 0xE9, 0x14 },	//pclk rising
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9921_sensor_path_rear_camera[] = {
	{ 0x07, 0x02 },
#if !defined(FEATURE_PROGRESSIVE_SUPPORT)
	{ 0x09, 0xf0 }, // set on init
#endif
	{ 0x1c, 0x08 },
	{ 0x1d, 0x80 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9921_sensor_path_aux_ntsc[] = {
	{ 0x07, 0x02 },
	{ 0x09, 0xf0 },
	{ 0x1c, 0x08 },
	{ 0x1d, 0x80 },
	{ 0x02, 0x40 }, // set yin as YIN0
    { 0x03, 0x21 },
	{ REG_TERM, VAL_TERM }
};

static struct sensor_reg tw9921_sensor_path_aux_pal[] = {
	{ 0x07, 0x12 },
	{ 0x09, 0x20 },
	{ 0x1c, 0x09 },
	{ 0x1d, 0x80 },
	{ 0x02, 0x40 }, // set yin as YIN0
    { 0x03, 0x21 },
	{ REG_TERM, VAL_TERM }
};

struct sensor_reg *tw9921_sensor_reg_path[] = {
	tw9921_sensor_path_rear_camera,
	tw9921_sensor_path_aux_ntsc,
	tw9921_sensor_path_aux_pal
};

#ifdef CONFIG_DAUDIO
static struct sensor_reg cvbs_component[] = {
	{0x02, 0x44},
	{0x03, 0x21},
	{0x06, 0x03},
	{0xFF, 0xFF}
};

static struct sensor_reg s_video[] = {
	{0x02, 0x54},
	{0x03, 0x21},
	{0x06, 0x00},
	{0xFF, 0xFF}
};

static struct sensor_reg sensor_regs_stop[] = {
	{0x03, 0x27},
	{0xFF, 0xFF}
};

struct sensor_reg * sensor_regs_type_and_encode[CAM_TYPE_MAX][CAM_ENC_MAX] = {
	// CAM_TYPE_DEFAULT
	{
		cvbs_component, 	//cvbs_ntsc,
	},
	{
		s_video,			//svideo_ntsc,
	},
	{
		cvbs_component,		//component_ntsc,
	},
};
#endif

int tw9921_read_ie(int cmd, char *level, struct tcc_camera_device * vdev);

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

static void tw9921_sensor_get_preview_size(int *width, int *height)
{
	*width = PRV_W;
	*height = PRV_H;
}

static void tw9921_sensor_get_capture_size(int *width, int *height)
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

	printk(KERN_ERR "TW9921 FARMER Sensor I2C !!!!START \n"); 
	
	while (!((next->reg == REG_TERM) && (next->val == VAL_TERM))) {
		bytes = 0;
		data[bytes]= (u8)next->reg & 0xff; 	bytes++;
		data[bytes]= (u8)next->val & 0xff; 	bytes++;

		err = DDI_I2C_Write(data, 1, 1,vdev);
		if (err) {
			err_cnt++;
			if(err_cnt >= 3) {
				printk(KERN_ERR "ERROR: Sensor I2C !!!! \n"); 
				return err;
			}
		} else {
			for (backup_cnt = 0; backup_cnt < ARRAY_SIZE(tw9921_sensor_reg_backup); backup_cnt++) {
				if (tw9921_sensor_reg_backup[backup_cnt].reg == next->reg) {
					tw9921_sensor_reg_backup[backup_cnt].val = next->val;
				}
			}

			err_cnt = 0;
			next++;
		}
	}
	
	printk(KERN_ERR "TW9921 FARMER Sensor I2C !!!! END \n"); 

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
	DDI_I2C_Read((unsigned short)reg, 1, &ret, 1,vdev);
	return ret;
}

static int tw9921_read(int reg, struct tcc_camera_device * vdev)
{
	int i = I2C_RETRY_COUNT;
	unsigned char ret;
	do {
		int retval = DDI_I2C_Read((unsigned short)reg, 1, &ret, 1,vdev);
		if (retval == 0)
		{
			//success to I2C.
			return ret;
		}
		i--;
	} while(i > 0);

	return FAIL_I2C;
}

static int tw9921_write(unsigned char* data, unsigned char reg_bytes, unsigned char data_bytes, struct tcc_camera_device * vdev)
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
int tw9921_sensor_type(unsigned int camera_type, unsigned int camera_encode,struct tcc_camera_device * vdev) {
	VPRINTK("============%s\n", __func__);

	if(((CAM_TYPE_MAX <= camera_type) || (sensor_regs_type_and_encode[camera_type][camera_encode] == NULL)) || 
	   ((CAM_ENC_MAX <= camera_encode) || (sensor_regs_type_and_encode[camera_type][camera_encode] == NULL))) {
		VPRINTK( "!@#---- %s() - WRONG arguments\n", __func__);
		return -1;
	}
	return write_regs(sensor_regs_type_and_encode[camera_type][camera_encode],vdev);
}
#endif
static int tw9921_sensor_open(eTW_YSEL yin,struct tcc_camera_device * vdev)
{
	VPRINTK("%s yin=%d\n", __func__, yin);
	
#ifdef CONFIG_DAUDIO
	gpio_set_value_cansleep(vdev->gpio_data.rst_port,0);
	sensor_delay(15);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,1);
	sensor_delay(15);

	write_regs(tw9921_sensor_reg_common[MODE_TW9921_INIT],vdev);
#endif
	
	if (yin == TW_YIN0)	// AUX
	{
#ifdef FEATURE_PROGRESSIVE_SUPPORT
		printk(KERN_INFO "%s AUX Mode\n", __func__);
		write_regs(TW9921_INIT_MODE[0],vdev);    // set interlaced mode
#endif		
		return write_regs(tw9921_sensor_reg_common[MODE_TW9921_CLOSE],vdev);
	}

#ifdef FEATURE_PROGRESSIVE_SUPPORT
	camera_type = getCamType();

	if(camera_type == SVM_CAM)
	{
		printk(KERN_INFO "%s SVM Mode\n", __func__);
		write_regs(TW9921_INIT_MODE[2],vdev);// set svm progressive mode
	}
	else
	{
		printk(KERN_INFO "%s Default Cam Mode\n", __func__);
		write_regs(TW9921_INIT_MODE[1],vdev);// set progressive mode
	}
		
	return write_regs(TW9921_OPEN_YINP[yin],vdev);
#else	
	return write_regs(TW9921_OPEN_YIN[yin],vdev);
#endif
}

static int tw9921_sensor_close(struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);
	return write_regs(tw9921_sensor_reg_common[MODE_TW9921_CLOSE],vdev);
}

int tw9921_read_id(struct tcc_camera_device * vdev)
{
#ifdef CONFIG_DAUDIO
	int cstatus;
	
	cstatus = tw9921_read(TW9921_REG_CSTATUS,vdev);
	printk("%s signal status 0x%x\n", __func__, cstatus);
#endif

	return tw9921_read(TW9921_REG_ID,vdev);
}

static int tw9921_sensor_preview(struct tcc_camera_device * vdev)
{
	if (debug_tw9921)	//for debug
	{
		int id = tw9921_read_id(vdev);
		VPRINTK("%s read id: 0x%x\n", __func__, id);
	}

	return 0;
}

static int tw9921_sensor_resetVideoDecoder(struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,0);
	sensor_delay(10);

	gpio_set_value_cansleep(vdev->gpio_data.rst_port,1);
	sensor_delay(10);

	return write_regs(tw9921_sensor_reg_backup,vdev);
}

void tw9921_close_cam(struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);

	tw9921_sensor_close(vdev);
}

int tw9921_init(void)
{
	VPRINTK("%s\n", __func__);

	return 0;
}

int tw9921_check_video_signal(unsigned long arg,struct tcc_camera_device * vdev)
{
	int ret = 0, cstatus = 0;
	tw_cstatus *tw9921_cstatus;

	tw9921_cstatus = (tw_cstatus *)arg;
	cstatus = tw9921_read(TW9921_REG_CSTATUS,vdev);
	if (cstatus >= FAIL)
		//error
		return cstatus;

	if (aux_encode != (cstatus & 0x1))
	{
	    printk("[%s] invalid format (%d:0x%x)\n", __func__, aux_encode, cstatus);
	    cstatus = 0;
	}
	tw9921_cstatus->vdloss = (char)cstatus & VDLOSS ? 1 : 0;
	tw9921_cstatus->hlock = (char)cstatus & HLOCK ? 1 : 0;
	tw9921_cstatus->slock = (char)cstatus & VLOCK ? 1 : 0;
	tw9921_cstatus->vlock = (char)cstatus & SLOCK ? 1 : 0;
	tw9921_cstatus->mono = (char)cstatus & MONO ? 1 : 0;
	ret = 1;

	printk("[%s](%x) vdloss : 0x%x, hlock : 0x%x, slock : 0x%x, vlock : 0x%x, mono : 0x%x\n", __func__, cstatus, tw9921_cstatus->vdloss, tw9921_cstatus->hlock, tw9921_cstatus->slock, tw9921_cstatus->vlock, tw9921_cstatus->mono);

	return ret;
}

int tw9921_display_mute(int mute)
{
	printk(KERN_INFO "%s TW9921 not support display mute.\n", __func__);

	return FAIL;
}

int tw9921_write_ie(int cmd, unsigned char value,struct tcc_camera_device * vdev)
{
	int ret = FAIL;
	unsigned char data[2] = {0x0, };
	//printk(KERN_CRIT "%s cmd: %d value: 0x%x\n", __func__, cmd, value);

	if (cmd < SET_CAM_BRIGHTNESS || cmd > SET_AUX_SATURATION)
	{
		printk(KERN_CRIT "%s cmd: cmd < SET_CAM_BRIGHTNESS || cmd > SET_AUX_SATURATION return ~~\n", __func__);
		return ret;
	}	
	switch (cmd)
	{
		case SET_AUX_BRIGHTNESS:
		case SET_CAM_BRIGHTNESS:
			data[0] = TW9921_I2C_BRIGHTNESS;
			break;
		case SET_AUX_CONTRAST:
		case SET_CAM_CONTRAST:
			data[0] = TW9921_I2C_CONTRAST;
			break;
		case SET_AUX_GAMMA:
		case SET_CAM_HUE:
			data[0] = TW9921_I2C_HUE;
			break;

	/*	case SET_CAM_SHARPNESS:
			data[0] = TW9921_I2C_SHARPNESS;
			
			break;

		case SET_CAM_U_GAIN:
			data[0] = TW9921_I2C_SATURATION_U;
			break;
		case SET_CAM_V_GAIN:
			data[0] = TW9921_I2C_SATURATION_V;
			break;
	*/
		case SET_AUX_SATURATION:
		case SET_CAM_SATURATION:
			data[0] = TW9921_I2C_SATURATION_U;
			break;
	}
	
	/*if(cmd == SET_CAM_SHARPNESS)
	{	
		int val=0;
		char lev;
		val = tw9921_read_ie( GET_CAM_SHARPNESS , &lev);
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

	ret = tw9921_write(data, 1, 1,vdev);

	printk(KERN_CRIT "[GT system] %s , data[1]= %d,  ret = %d\n", __func__,data[1], ret );
	if( (ret == SUCCESS_I2C && cmd == SET_CAM_SATURATION) ||
		(ret == SUCCESS_I2C && cmd == SET_AUX_SATURATION) )
	{
		data[0] = TW9921_I2C_SATURATION_V;
		ret = tw9921_write(data, 1, 1,vdev);
	}

	if (ret >= FAIL)
		ret = FAIL;
	else
		ret = SUCCESS;

	return ret;
}

int tw9921_read_ie(int cmd, char *level,struct tcc_camera_device * vdev)
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
			ret = tw9921_read(TW9921_I2C_BRIGHTNESS,vdev);
			break;

		case GET_CAM_CONTRAST:
		case GET_AUX_CONTRAST:
			ret = tw9921_read(TW9921_I2C_CONTRAST,vdev);
			break;

		case GET_CAM_HUE:
		case GET_AUX_GAMMA:
			ret = tw9921_read(TW9921_I2C_HUE,vdev);
			break;

		case GET_CAM_SATURATION:
		case GET_AUX_SATURATION:
			value_u = tw9921_read(TW9921_I2C_SATURATION_U,vdev);
			//printk(KERN_CRIT "[GT system] %s , value_u= %d \n", __func__ , value_u);
			value_v = tw9921_read(TW9921_I2C_SATURATION_V,vdev);
			//printk(KERN_CRIT "[GT system] %s , value_v= %d \n", __func__ , value_v);

			if (value_u == FAIL_I2C || value_v == FAIL_I2C)
				ret = FAIL_I2C;
			else
				ret = value_u;
			break;
		/*
		case GET_CAM_U_GAIN:
			ret = tw9921_read(TW9921_I2C_SATURATION_U);
			printk(KERN_CRIT "[GT system] %s , value_u= %d \n", __func__ , ret);
			break;			
		case GET_CAM_V_GAIN:
			ret = tw9921_read(TW9921_I2C_SATURATION_V);
			printk(KERN_CRIT "[GT system] %s , value_v= %d \n", __func__ , ret);
			break;
		case GET_CAM_SHARPNESS:
			ret = tw9921_read(TW9921_I2C_SHARPNESS);
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
			*level += 127;

		ret = SUCCESS;
	}
	//printk(KERN_CRIT "[GT system] %s , ret=%d : level= %d \n", __func__ , ret ,*level);

	return ret;
}

static int tw9921_sensor_capture(void)
{
	return 0;
}

static int tw9921_sensor_capturecfg(int width, int height)
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
	return write_regs(tw9921_sensor_reg_brightness[val],vdev);
}

static int sensor_scene(int val,struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_contrast(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_contrast[val],vdev);
}

static int sensor_saturation(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_saturation[val],vdev);
}

//GT system start
static int sensor_u_gain(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_u_gain[val],vdev);
}
static int sensor_v_gain(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_v_gain[val],vdev);
}
static int sensor_sharpness(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_sharpness[val],vdev);
}
//GT system end

static int sensor_hue(int val,struct tcc_camera_device * vdev)
{
	return write_regs(tw9921_sensor_reg_hue[val],vdev);
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

	DDI_I2C_Read(0x01, 1, &ret, 1, vdev);
	if ((ret & 0xE8) == 0x68)  {
		ret = 0; // not detected video noise
	} else {
		ret = 1; // detected video noise
	}
	
	return ret;
}

static int tw9921_sensor_setPath(int val,struct tcc_camera_device * vdev)
{
	int ret;

	aux_encode = (val == 2) ? 1 : 0;

	printk("%s val = %d\n", __func__, val);	
	
	ret = write_regs(tw9921_sensor_reg_path[val],vdev);

	if (val != 0) {
		int cstatus;
		msleep(200);
		cstatus = tw9921_read(TW9921_REG_CSTATUS,vdev);
		printk("%s signal status 0x%x\n", __func__, cstatus);
	}
	
	return ret;
}

static int tw9921_sensor_getVideoFormat(void)
{
	char ret = 0;
	//TODO

	return ret;
}

static int tw9921_sensor_check_esd(int val)
{
	return 0;
}

static int tw9921_sensor_check_luma(int val)
{
	return 0;
}

void sensor_info_init(TCC_SENSOR_INFO_TYPE *sensor_info)
{
		printk("\n\n\n\n%s TW9921 Init \n\n\n\n\n", __func__);

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
#ifdef CONFIG_LVDS_CAMERA_BT601
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
void tw9921_sensor_init_fnc(SENSOR_FUNC_TYPE_DAUDIO *sensor_func)
{
	sensor_func->sensor_open				= tw9921_sensor_open;
	sensor_func->sensor_close				= tw9921_sensor_close;

	sensor_func->sensor_preview				= tw9921_sensor_preview;
	sensor_func->sensor_capture				= tw9921_sensor_capture;
	sensor_func->sensor_capturecfg			= tw9921_sensor_capturecfg;
#ifdef CONFIG_DAUDIO
	sensor_func->Set_CameraMode			= tw9921_sensor_type;
#endif	
	sensor_func->sensor_reset_video_decoder	= tw9921_sensor_resetVideoDecoder;
	sensor_func->sensor_init				= tw9921_init;
	sensor_func->sensor_read_id				= tw9921_read_id;
	sensor_func->sensor_check_video_signal	= tw9921_check_video_signal;
	sensor_func->sensor_display_mute		= tw9921_display_mute;
	sensor_func->sensor_close_cam			= tw9921_close_cam;
	sensor_func->sensor_write_ie			= tw9921_write_ie;
	sensor_func->sensor_read_ie				= tw9921_read_ie;
	sensor_func->sensor_get_preview_size	= tw9921_sensor_get_preview_size;
	sensor_func->sensor_get_capture_size	= tw9921_sensor_get_capture_size;

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

	sensor_func->Set_Path					= tw9921_sensor_setPath;
	sensor_func->Get_videoFormat			= tw9921_sensor_getVideoFormat;
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
	tw9921_sensor_init_fnc(sensor_func);
}

#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init(sensor_info);
}
void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func)
{
	tw9921_sensor_init_fnc(sensor_func);
}
#endif

