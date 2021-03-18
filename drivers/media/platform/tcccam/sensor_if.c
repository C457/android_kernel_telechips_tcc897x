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
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/gpio.h>

#if defined(CONFIG_ARCH_TCC898X)
#include <video/tcc/tcc_cam_ioctrl.h>
#else
#include <mach/tcc_cam_ioctrl.h>
#endif

#include "tcc_cam_i2c.h"  
#include "sensor_if.h"
#include "tcc_cam.h"
#include "cam_clock.h"
#include "tcc_camera_device.h"

static int debug	= 0;
#define TAG		"sensor_if"
#define dprintk(msg...)	if(debug) { printk(TAG ": " msg); }
#define log(msg...)	if(debug) { printk(TAG ": %s - ", __func__); printk(msg); }
#define FUNCTION_IN	log("In\n");
#define FUNCTION_OUT	log("Out\n");

/* list of image formats supported by sensor sensor */
const static struct v4l2_fmtdesc sensor_formats[] = 
{
	{
		.description	= "RGB565, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},
	{
		.description	= "RGB565, be",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
	},
	{
		.description	=   "RGB888, packed", 
		.pixelformat	= 	V4L2_PIX_FMT_RGB24,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		.description	= "RGB555, le",
		.pixelformat	= V4L2_PIX_FMT_RGB555,
	},
	{
		.description	= "RGB555, be",
		.pixelformat	= V4L2_PIX_FMT_RGB555X,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(sensor_formats)

static struct vcontrol control[] = {
	{{V4L2_CID_BRIGHTNESS           ,V4L2_CTRL_TYPE_INTEGER,"Brightness"            ,0    ,4    ,1    ,DEF_BRIGHTNESS}           ,0},
	{{V4L2_CID_AUTO_WHITE_BALANCE   ,V4L2_CTRL_TYPE_INTEGER,"Auto White Balance"    ,0    ,5    ,1    ,DEF_AWB}                  ,0},
	{{V4L2_CID_ISO                  ,V4L2_CTRL_TYPE_INTEGER,"Iso"                   ,0    ,3    ,1    ,DEF_ISO}                  ,0},
	{{V4L2_CID_EFFECT               ,V4L2_CTRL_TYPE_INTEGER,"Special Effect"        ,0    ,6    ,1    ,DEF_EFFECT}               ,0},
	{{V4L2_CID_ZOOM                 ,V4L2_CTRL_TYPE_INTEGER,"Zoom"                  ,0    ,24   ,1    ,DEF_ZOOM}                 ,0},
	{{V4L2_CID_FLIP                 ,V4L2_CTRL_TYPE_INTEGER,"Flip"                  ,0    ,3    ,1    ,DEF_FLIP}                 ,0},
	{{V4L2_CID_SCENE                ,V4L2_CTRL_TYPE_INTEGER,"Scene"                 ,0    ,5    ,1    ,DEF_SCENE}                ,0},
	{{V4L2_CID_METERING_EXPOSURE    ,V4L2_CTRL_TYPE_INTEGER,"Metering Exposure"     ,0    ,3    ,1    ,DEF_METERING_EXPOSURE}    ,0},
	{{V4L2_CID_EXPOSURE             ,V4L2_CTRL_TYPE_INTEGER,"Exposure"              ,-20  ,20   ,5    ,DEF_EXPOSURE}             ,0},
	{{V4L2_CID_FOCUS_MODE           ,V4L2_CTRL_TYPE_INTEGER,"AutoFocus"             ,0    ,1    ,2    ,DEF_FOCUSMODE}            ,0},
	{{V4L2_CID_FLASH                ,V4L2_CTRL_TYPE_INTEGER,"Flash"                 ,0    ,4    ,1    ,DEF_FLASH}                ,0}
};

unsigned int sensor_control[] = {
	V4L2_CID_BRIGHTNESS,
	V4L2_CID_AUTO_WHITE_BALANCE,
	V4L2_CID_ISO,
	V4L2_CID_EFFECT,
	V4L2_CID_ZOOM,
	V4L2_CID_FLIP,
	V4L2_CID_SCENE,
	V4L2_CID_METERING_EXPOSURE,
	V4L2_CID_EXPOSURE,
	V4L2_CID_FOCUS_MODE,
	V4L2_CID_FLASH
};

int sensor_if_change_mode(struct tcc_camera_device * vdev, unsigned char capture_mode)
{
    int ret=0;

	if(capture_mode)
		ret = vdev->sensor_func.Set_Capture(vdev);
	else
		ret = vdev->sensor_func.Set_Preview(vdev);
	
    return ret;
}

int sensor_if_change_mode_ex(int camera_type, int camera_encode, struct tcc_camera_device * vdev) {
	printk("%s() - camera_type = 0x%x, camera_encode = 0x%x\n", __func__, camera_type, camera_encode);
#ifndef CONFIG_LVDS_CAMERA
	vdev->sensor_func.Set_CameraMode(camera_type, camera_encode, vdev);
#endif

	return 0;
}

int sensor_if_adjust_autofocus(struct tcc_camera_device * vdev)
{
	return vdev->sensor_func.Set_AF(0, vdev);
}

/* Returns the index of the requested ID from the control structure array */
static int find_vctrl(int id)
{
	int i;
	int ctrl_cnt = ARRAY_SIZE(control);

	if(id < V4L2_CID_BASE)
		return -EDOM;

	for(i = ctrl_cnt - 1; i >= 0; i--)
	{
		if(control[i].qc.id == id)
			break;
	}

	if(i < 0)
		i = -EINVAL;
	
	return i;
}

/* Find the best match for a requested image capture size.  The best match 
 * is chosen as the nearest match that has the same number or fewer pixels 
 * as the requested size, or the smallest image size if the requested size 
 * has fewer pixels than the smallest image.
 */
static enum image_size sensor_find_size(struct tcc_camera_device * vdev, unsigned int width, unsigned int height)
{
	enum image_size isize;
	unsigned long pixels = width*height;

	for(isize=0; isize < (NUM_IMAGE_SIZES-1); isize++)
	{
		if((vdev->tcc_sensor_info.sensor_sizes[isize + 1].height * vdev->tcc_sensor_info.sensor_sizes[isize + 1].width) > pixels)
		{
			return isize;
		}
	}
	//CONFIG_COBY_MID9125
	#if defined(SENSOR_VGA) || defined(CONFIG_VIDEO_TCCXX_ATV)
	{
		return VGA;
	}
	#else
	{
		return SXGA;
	}
	#endif
}

/* following are sensor interface functions implemented by 
 * sensor driver.
 */
int sensor_if_query_control(struct v4l2_queryctrl *qc)
{
	int i = find_vctrl (qc->id);
	
	if(i == -EINVAL) 
	{
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	
	if(i < 0) 		
		return -EINVAL;
	
	*qc = control[i].qc;
	
	return 0;
}

int sensor_if_get_control(struct v4l2_control *vc)
{
	int i;
	struct vcontrol * lvc;
	
	if((i = find_vctrl(vc->id)) < 0)
		return -EINVAL;

	lvc = &control[i];	
	vc->value = lvc->current_value;
	
	return 0;
}

int sensor_if_set_control(struct v4l2_control *vc, unsigned char init, struct tcc_camera_device * vdev)
{	
	struct vcontrol *lvc;
	int val = vc->value, err = 0;
	
	if((err = find_vctrl(vc->id)) < 0) 	
		return -EINVAL;
	
	lvc = &control[err];
	
	if(lvc->qc.maximum < val) 			
		return val;
	
	if(lvc->current_value != val || init) 
	{
		lvc->current_value = val;
		
		switch(lvc->qc.id) {
			case V4L2_CID_BRIGHTNESS:
				err = vdev->sensor_func.Set_Bright(lvc->current_value, vdev);
				break;

			case V4L2_CID_AUTO_WHITE_BALANCE:
				err = vdev->sensor_func.Set_WB(lvc->current_value, vdev);
				break;

			case V4L2_CID_ISO:
				err = vdev->sensor_func.Set_ISO(lvc->current_value, vdev);
				break;

			case V4L2_CID_EFFECT:
				err = vdev->sensor_func.Set_Effect(lvc->current_value, vdev);
				break;

			case V4L2_CID_ZOOM:
				
				#ifdef USE_SENSOR_ZOOM_IF
				{
					err = vdev->sensor_func.Set_Zoom(lvc->current_value, vdev);
				}
				#else
				{
					err = tccxxx_cif_set_zoom(lvc->current_value, vdev);
				}
				#endif
				break;

			case V4L2_CID_FLIP:
				err = vdev->sensor_func.Set_Flip(lvc->current_value, vdev);
				break;

			case V4L2_CID_SCENE:
				err = vdev->sensor_func.Set_Scene(lvc->current_value, vdev);
				break;

			case V4L2_CID_METERING_EXPOSURE:
				err = vdev->sensor_func.Set_ME(lvc->current_value, vdev);
				break;

			case V4L2_CID_FLASH:
				// todo: 
				break;

			case V4L2_CID_EXPOSURE:
				err = vdev->sensor_func.Set_Exposure(lvc->current_value, vdev);
				break;

			case V4L2_CID_FOCUS_MODE:
				err = vdev->sensor_func.Set_FocusMode(lvc->current_value, vdev);
				break;

			default:
				break;
		}
			
	}

	return err;
}

/* In case of ESD-detection, to set current value in sensor after module was resetted. */
void sensor_if_set_current_control(struct tcc_camera_device * vdev)
{
	struct v4l2_control vc;
	int i;
	int ctrl_cnt = ARRAY_SIZE(sensor_control);

	printk("Setting Sensor-ctrl!! %d /n", ctrl_cnt);
	
	for (i = ctrl_cnt - 1; i >= 0; i--)
	{
		vc.id = sensor_control[i];
		sensor_if_get_control(&vc);
		sensor_if_set_control(&vc, 1, vdev);
	}

}

/* Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
int sensor_if_enum_pixformat(struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type)
	{
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
		break;

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		if (index >= NUM_OVERLAY_FORMATS)
			return -EINVAL;
		break;

		default:
			return -EINVAL;
	}

	fmt->flags = sensor_formats[index].flags;
	strlcpy(fmt->description, sensor_formats[index].description, sizeof(fmt->description));
	fmt->pixelformat = sensor_formats[index].pixelformat;

	return 0;
}

/* Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This 
 * ioctl is used to negotiate the image capture size and pixel format 
 * without actually making it take effect.
 */
static int sensor_try_format(struct tcc_camera_device * vdev)
{
	enum image_size isize;
	int ifmt;

	isize = sensor_find_size(vdev, vdev->pix_format.width, vdev->pix_format.height);
	
	vdev->pix_format.width  = vdev->tcc_sensor_info.sensor_sizes[isize].width;
	vdev->pix_format.height = vdev->tcc_sensor_info.sensor_sizes[isize].height;

	for(ifmt=0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) 
	{
		if(vdev->pix_format.pixelformat == sensor_formats[ifmt].pixelformat)
			break;
	}
	
	if(ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	
	vdev->pix_format.pixelformat = sensor_formats[ifmt].pixelformat;
	vdev->pix_format.field = V4L2_FIELD_NONE;
	vdev->pix_format.bytesperline = vdev->pix_format.width*2;
	vdev->pix_format.sizeimage = vdev->pix_format.bytesperline*vdev->pix_format.height;
	vdev->pix_format.priv = 0;
	
	switch(vdev->pix_format.pixelformat)
	{
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		default:
			vdev->pix_format.colorspace = V4L2_COLORSPACE_JPEG;
			break;
			
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB555X:
		case V4L2_PIX_FMT_RGB24: 
		case V4L2_PIX_FMT_RGB32: 
			vdev->pix_format.colorspace = V4L2_COLORSPACE_SRGB;
			break;
	}
	
	return 0;
}

int sensor_if_parse_gpio_dt_data(struct tcc_camera_device * vdev)
{
	// set to initialize normal gpio that power, power_down(or stand-by) and reset of sensor.

	struct device_node	*cam_np = vdev->camera_np;
	struct device_node	*module_np;


#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)

	if(vdev->CameraID)
	{
		module_np = of_find_node_by_name(cam_np,FRONT_MODULE_NODE);
	}
	else
	{
		module_np = of_find_node_by_name(cam_np,BACK_MODULE_NODE);
	}
	
#else

	if(vdev->data.cam_info == DAUDIO_CAMERA_LVDS)
		module_np = of_find_node_by_name(cam_np,LVDS_MODULE_NODE);
	else
		module_np = of_find_node_by_name(cam_np,MODULE_NODE);

#endif

	if(module_np)
	{
		vdev->gpio_data.pwr_port = of_get_named_gpio_flags(module_np, "pwr-gpios", 0, &vdev->gpio_data.pwr_value);
		vdev->gpio_data.pwd_port = of_get_named_gpio_flags(module_np, "pwd-gpios", 0, &vdev->gpio_data.pwd_value);
		vdev->gpio_data.rst_port = of_get_named_gpio_flags(module_np, "rst-gpios", 0, &vdev->gpio_data.rst_value);

		if(of_property_read_u32(module_np, "cvbs-input", &vdev->cvbs_input))
		{
			vdev->cvbs_input = 0;
			dprintk("cvbs_input default 0 \n");
		}
		else
		{
			dprintk("cvbs_input : 0x%x \n", vdev->cvbs_input);
		}

		if(vdev->gpio_data.pwr_port < 0) vdev->gpio_data.pwr_port = 0;
		if(vdev->gpio_data.pwd_port < 0) vdev->gpio_data.pwd_port = 0;
		if(vdev->gpio_data.rst_port < 0) vdev->gpio_data.rst_port = 0;

		dprintk("pwr: port = %3d, curr val = %d, set val = %d\n",	\
			vdev->gpio_data.pwr_port, gpio_get_value(vdev->gpio_data.pwr_port), vdev->gpio_data.pwr_value);
		dprintk("pwd: port = %3d, curr val = %d, set val = %d\n",	\
			vdev->gpio_data.pwd_port, gpio_get_value(vdev->gpio_data.pwd_port), vdev->gpio_data.pwd_value);
		dprintk("rst: port = %3d, curr val = %d, set val = %d\n",	\
			vdev->gpio_data.rst_port, gpio_get_value(vdev->gpio_data.rst_port), vdev->gpio_data.rst_value);
	}
	else
	{
		printk("could not find sensor module node!! \n");
		return -ENODEV;	
	}

	gpio_request(vdev->gpio_data.pwr_port, "camera power");
	gpio_direction_output(vdev->gpio_data.pwr_port, vdev->gpio_data.pwr_value);

	gpio_request(vdev->gpio_data.pwd_port, "camera power down");
	gpio_direction_output(vdev->gpio_data.pwd_port, vdev->gpio_data.pwd_value);

	gpio_request(vdev->gpio_data.rst_port, "camera reset");
	gpio_direction_output(vdev->gpio_data.rst_port, vdev->gpio_data.rst_value);

	cam_clock_clkrate(vdev);

	return 0;
}

/* Initialize the sensor.
 * This routine allocates and initializes the data structure for the sensor, 
 * powers up the sensor, registers the I2C driver, and sets a default image 
 * capture format in pix.  The capture format is not actually programmed 
 * into the sensor by this routine.
 * This function must return a non-NULL value to indicate that 
 * initialization is successful.
 */
int sensor_if_init(struct tcc_camera_device * vdev)
{

#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)

	if(vdev->CameraID)
	{
		spin_lock(&vdev->sensor_lock);
		vdev->CameraID = CIF_FACING_BACK;

		if(!sensor_get_port(vdev->gpio_data.pwd_port))
		{
			dprintk("First back camera powerdown! \n");
			sensor_if_api_connect(vdev,vdev->CameraID);
	
			/* Go To PowerDown Mode */
			if(vdev->sensor_func.PowerDown(vdev,false) < 0)
			{
				printk("back camera close error! \n");
				return -1;
			}
		}	

		vdev->CameraID = CIF_FACING_FRONT;
		spin_unlock(&vdev->sensor_lock);
		sensor_if_api_connect(vdev,vdev->CameraID);
	}
	else
	{
		spin_lock(&vdev->sensor_lock);
		vdev->CameraID = CIF_FACING_FRONT;
		
		if(!sensor_get_port(vdev->gpio_data.pwd_port))
		{
			dprintk("First front camera powerdown! \n");
			sensor_if_api_connect(vdev,vdev->CameraID);

			/* Go To PowerDown Mode */
			if(vdev->sensor_func.PowerDown(vdev,false) < 0)
			{
				printk("front camera close error! \n");
				return -1;
			}				
		}
			
		vdev->CameraID = CIF_FACING_BACK;
		spin_unlock(&vdev->sensor_lock);

		sensor_if_api_connect(vdev,vdev->CameraID);
	}
#else // CONFIG_VIDEO_DUAL_CAMERA_SUPPORT

	sensor_if_api_connect(vdev,vdev->CameraID);

#endif // CONFIG_VIDEO_DUAL_CAMERA_SUPPORT

	sensor_if_open(vdev);

	/* Make the default capture format QVGA YUV422 */
	vdev->pix_format.width = vdev->tcc_sensor_info.sensor_sizes[0].width;
	vdev->pix_format.height = vdev->tcc_sensor_info.sensor_sizes[0].height;

	vdev->pix_format.pixelformat = V4L2_PIX_FMT_YUYV;
	
	sensor_try_format(vdev);

	return 0;
}

int sensor_if_get_max_resolution(struct tcc_camera_device * vdev)
{
	switch(vdev->tcc_sensor_info.sensor_sizes[0].width) 
	{
		// 5M size
		case 2560: return QQXGA;

		// 3M size
		case 2048: return QXGA; 
		
		// 2M size
		case 1600: return UXGA;

		// 1.3M size		
		case 1280: return SXGA;

		// analog tv
		case 720: return XGA;

		// VGA
		case 640: return VGA;
	}
	return 0;
}

int sensor_if_get_sensor_framerate(struct tcc_camera_device * vdev, int *nFrameRate)
{	
	*nFrameRate = vdev->tcc_sensor_info.framerate;

	if(*nFrameRate)
	{
		return 0;
	}
	else
	{
		printk("Sensor Driver dosen't have frame rate information!!\n");
		return -1;
	}
}

void sensor_if_set(struct tcc_camera_device * vdev, int index)
{
#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)
	if(index)	sensor_info_init_front(&vdev->tcc_sensor_info);
	else		sensor_info_init_back(&vdev->tcc_sensor_info);
#else
	if(vdev->data.cam_info == DAUDIO_CAMERA_LVDS)
		lvds_sensor_info_init(&vdev->tcc_sensor_info);
	else
		sensor_info_init_back(&vdev->tcc_sensor_info);
#endif
}

void sensor_if_api_connect(struct tcc_camera_device * vdev, int index)
{
#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)
	if(index)	sensor_init_fnc_front(&vdev->sensor_func);
	else		sensor_init_fnc_back(&vdev->sensor_func);
#else
	sensor_init_fnc(&vdev->sensor_func);
#endif
}

int sensor_if_open(struct tcc_camera_device * vdev)
{
	if(!vdev->enabled)	vdev->enabled = 1;
	
	vdev->sensor_func.Open(vdev,false);
	
	return 0;
}

int sensor_if_capture_config(struct tcc_camera_device * vdev, int width, int height)
{
	return vdev->sensor_func.Set_CaptureCfg(width, height, vdev);
}

int sensor_if_isESD(struct tcc_camera_device * vdev)
{
	return vdev->sensor_func.Check_ESD(0, vdev);
}
/* Prepare for the driver to exit.
 * Balances sensor_if_init().
 * This function must de-initialize the sensor and its associated data
 * structures.
 */
int sensor_if_cleanup(struct tcc_camera_device * vdev)
{
	dprintk("enabled = [%d]\n", vdev->enabled);
	
	if(vdev->enabled)
	{
		vdev->sensor_func.Close(vdev);
		vdev->enabled = 0;
		
		gpio_free(vdev->gpio_data.pwr_port);
		gpio_free(vdev->gpio_data.pwd_port);
		gpio_free(vdev->gpio_data.rst_port);
	}
	
    return 0;
}

void sensor_delay(int ms)
{
	unsigned int msec;

	msec = ms / 10; //10msec unit

	if(!msec)	msleep(1);
	else		msleep(msec);
}

void sensor_cifmc_enable(struct tcc_camera_device * vdev)
{
	cam_clock_enable(vdev, CIF_MCLOCK);
}

void sensor_cifmc_disable(struct tcc_camera_device * vdev)
{
	cam_clock_disable(vdev, CIF_MCLOCK);
}

void sensor_port_enable(int port)
{
//	gpio_set_value(vdev->gpio_data.pwr_port, 1);
	gpio_set_value_cansleep(port, ON);

}

void sensor_port_disable(int port)
{
//	gpio_set_value(vdev->gpio_data.pwr_port, 0);
	gpio_set_value_cansleep(port, OFF);
}

int sensor_get_port(int port)
{
	return gpio_get_value(port);
}

int sensor_if_read_i2c(int reg, struct tcc_camera_device * vdev) {
	if(vdev->sensor_func.ReadSensorRegister == NULL) {
		printk("%s() - sensor_func.readSensorRegister == NULL\n", __func__);
		return -1;
	}
	return vdev->sensor_func.ReadSensorRegister(reg,vdev);
}

int sensor_if_check_camera_module(struct tcc_camera_device * vdev) {
	if(vdev->sensor_func.CheckCameraModule == NULL) {
		printk("%s() - Not support CheckCameraModule \n", __func__);
		return 0;
	}
	return vdev->sensor_func.CheckCameraModule(vdev);
}

MODULE_DESCRIPTION("Camera SENSOR  driver");
MODULE_AUTHOR("Telechips");
MODULE_LICENSE("GPL");

