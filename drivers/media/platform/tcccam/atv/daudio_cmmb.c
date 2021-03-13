#include <mach/daudio.h>
#include <mach/daudio_debug.h>

#include "../tcc_cam_i2c.h"

#include "daudio_atv.h"
#include "daudio_cmmb.h"

#define PRV_W 800
#define PRV_H 240
#define CAP_W 800
#define CAP_H 240

static int debug_daudio_cmmb	= DEBUG_DAUDIO_ATV;

#define VPRINTK(fmt, args...) if (debug_daudio_cmmb) printk(KERN_INFO TAG_DAUDIO_CMMB fmt, ##args)

static void daudio_cmmb_sensor_get_preview_size(int *width, int *heigth)
{
	*width = PRV_W;
	*heigth = PRV_H;
}

static void daudio_cmmb_sensor_get_capture_size(int *width, int *heigth)
{
	*width = CAP_W;
	*heigth = CAP_H;
}

static int daudio_cmmb_sensor_open(eTW_YSEL yin)
{
	VPRINTK("%s\n", __func__);

	return 0;
}

static int daudio_cmmb_sensor_close(void)
{
	VPRINTK("%s\n", __func__);

	return 0;
}

static int daudio_cmmb_sensor_preview(void)
{
	return 0;
}

static int daudio_cmmb_sensor_resetVideoDecoder(struct tcc_camera_device * vdev)
{
	VPRINTK("%s\n", __func__);

	sensor_reset_low();
	sensor_delay(10);

	sensor_reset_high();
	sensor_delay(10);

	return 0;
}

void daudio_cmmb_close_cam(void)
{
}

int daudio_cmmb_init(void)
{
	return 0;
}

void daudio_cmmb_sensor_init_fnc(SENSOR_FUNC_TYPE_DAUDIO *sensor_func)
{
	sensor_func->sensor_open				= daudio_cmmb_sensor_open;
	sensor_func->sensor_close				= daudio_cmmb_sensor_close;

	sensor_func->sensor_preview				= daudio_cmmb_sensor_preview;
	sensor_func->sensor_capture				= NULL;
	sensor_func->sensor_capturecfg			= NULL;
	sensor_func->sensor_reset_video_decoder	= daudio_cmmb_sensor_resetVideoDecoder;
	sensor_func->sensor_init				= daudio_cmmb_init;
	sensor_func->sensor_read_id				= NULL;
	sensor_func->sensor_check_video_signal	= NULL;
	sensor_func->sensor_display_mute		= NULL;
	sensor_func->sensor_close_cam			= daudio_cmmb_close_cam;
	sensor_func->sensor_write_ie			= NULL;
	sensor_func->sensor_read_ie				= NULL;
	sensor_func->sensor_get_preview_size	= daudio_cmmb_sensor_get_preview_size;
	sensor_func->sensor_get_capture_size	= daudio_cmmb_sensor_get_capture_size;

	sensor_func->Set_Zoom					= NULL;
	sensor_func->Set_AF						= NULL;
	sensor_func->Set_Effect					= NULL;
	sensor_func->Set_Flip					= NULL;
	sensor_func->Set_ISO					= NULL;
	sensor_func->Set_ME						= NULL;
	sensor_func->Set_WB						= NULL;
	sensor_func->Set_Bright					= NULL;
	sensor_func->Set_Scene					= NULL;
	sensor_func->Set_Contrast				= NULL;
	sensor_func->Set_Saturation				= NULL;
	sensor_func->Set_Hue					= NULL;
	sensor_func->Get_Video					= NULL;
	sensor_func->Check_Noise				= NULL;

	sensor_func->Set_Path					= NULL;
	sensor_func->Get_videoFormat			= NULL;
	sensor_func->Set_writeRegister			= NULL;
	sensor_func->Get_readRegister			= NULL;

	sensor_func->Check_ESD					= NULL;
	sensor_func->Check_Luma					= NULL;
}
