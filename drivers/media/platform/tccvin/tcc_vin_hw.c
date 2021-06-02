
/*
 * drivers/media/video/tccvin/tcc_vin_hw.c
 *
 * Copyright (C) 2008 Telechips, Inc. 
 *
 * Video-for-Linux (Version 2) camera capture driver for Telechisp SoC.
 *
 * leverage some code from CEE distribution 
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 *
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

#include <mach/irqs.h>
#include <mach/gpio.h>
//#include <mach/clock.h>
#include <mach/tcc_cam_ioctrl.h>
#include "tcc_vin_hw.h"
#include "sensor/support_sensor.h"

#if defined(CONFIG_TCC_VIOCMG)
#include <video/tcc/viocmg.h>
#endif

#ifdef CONFIG_TCC_VIN_DEBUG
#define dprintk(fmt, args...) printk("\e[33m[  vin_hw]%s(%d) \e[0m" fmt, __func__, __LINE__, ## args);
#else
#define dprintk(fmt, args...)
#endif

/* list of image formats supported by sensor sensor */
const static struct v4l2_fmtdesc sensor_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description	= "RGB565, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	b4 b3 b2 b1 b0 g5 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	r4 r3 r2 r1 r0 g5 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB565, be",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		/* Note:  V4L2 defines RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  x  b4 b3 b2 b1 b0 g4 g3
		 *
		 * We interpret RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  x  r4 r3 r2 r1 r0 g4 g3
		 */
		.description	= "RGB555, le",
		.pixelformat	= V4L2_PIX_FMT_RGB555,
	},{
		/* Note:  V4L2 defines RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  b4 b3 b2 b1 b0 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  r4 r3 r2 r1 r0 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB555, be",
		.pixelformat	= V4L2_PIX_FMT_RGB555X,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(sensor_formats)
enum pixel_format { YUV, RGB565, RGB555 };
#define NUM_PIXEL_FORMATS 3
#define NUM_OVERLAY_FORMATS 2

extern void vioc_vin_path_reset(struct tcc_video_device *vdev);

struct sensor_gpio gpio_data;
//struct VIOC_Index vdev->vioc.vioc_num;

static VIOC_WMIX*			pWMIXBase;
static VIOC_WDMA*			pWDMABase;
static VIOC_VIN*			pVINBase;
static VIOC_SC* 			pSCBase;
static VIQE*				pVIQEBase;
static VIOC_IREQ_CONFIG*	pVIOCConfig;
static unsigned int*		pLUTBase;

int sensor_if_change_mode(struct tcc_video_device *vdev, unsigned char mode)
{
	struct sensor_info *sinfo = vdev->sinfo;
	int ret = 0;

	dprintk("___ in \n");
	
	if (mode == 0 && sinfo->set_preview)
		ret = sinfo->set_preview(vdev);

	dprintk("___ out \n");
	return ret;
}

/* Returns the index of the requested ID from the control structure array */
static int find_vctrl(struct sensor_info *sinfo, int id)
{
	int i;
	int ctrl_cnt = sinfo->n_v4l2_ctrl;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = ctrl_cnt - 1; i >= 0; i--) {
		if (sinfo->v4l2_ctrl[i].qc.id == id)
			break;
	}

	if (i < 0)
		i = -EINVAL;

	return i;
}

/* following are sensor interface functions implemented by 
 * sensor driver.
 */
int sensor_if_query_control(struct tcc_video_device *vdev, struct v4l2_queryctrl *qc)
{
	int i;
	struct sensor_info *sinfo = vdev->sinfo;

	if ((i = find_vctrl(sinfo, qc->id)) < 0)
		return i;
	*qc = sinfo->v4l2_ctrl[i].qc;
	return 0;
}

int sensor_if_get_control(struct tcc_video_device *vdev, struct v4l2_control *vc)
{
	int i;
	struct v4l2_ctrl_t *lvc;
	struct sensor_info *sinfo = vdev->sinfo;
	
	if((i = find_vctrl(sinfo, vc->id)) < 0)
		return i;

	lvc = &sinfo->v4l2_ctrl[i];
	vc->value = lvc->current_value;
	
	return 0;
}

int sensor_if_set_control(struct tcc_video_device *vdev, struct v4l2_control *vc, unsigned char init)
{
	int i;
	int val = vc->value;
	struct v4l2_ctrl_t *lvc;
	struct sensor_info *sinfo = vdev->sinfo;

	if((i = find_vctrl(sinfo, vc->id)) < 0)
		return i;

	lvc = &sinfo->v4l2_ctrl[i];
	if(lvc->qc.maximum < val)
		return val;
	
	//if(lvc->current_value != val || init) {
		lvc->current_value = val;
		lvc->need_set = 1;
		sinfo->need_new_set = 1;
	//}

	return 0;
}

int sensor_if_check_control(struct tcc_video_device *vdev)
{
	int i, err = 0;
	int ctrl_cnt;
	struct v4l2_ctrl_t *lvc;
	struct sensor_info *sinfo = vdev->sinfo;

	if (!sinfo->need_new_set)
		return 0;

	ctrl_cnt = sinfo->n_v4l2_ctrl;
	for (i = (ctrl_cnt - 1); i >= 0; i--) {
		lvc = &sinfo->v4l2_ctrl[i];
		if (lvc->need_set) {
			err = sinfo->v4l2_set_ctrl(vdev, lvc->qc.id, lvc->current_value);			
			lvc->need_set = 0;
		}
	}
	sinfo->need_new_set = 0;
	return err;
}

/* In case of ESD-detection, to set current value in sensor after module was resetted. */
int sensor_if_set_current_control(struct tcc_video_device *vdev)
{
	int i;
	struct v4l2_control vc;
	struct sensor_info *sinfo = vdev->sinfo;
	int ctrl_cnt = sinfo->n_v4l2_re_ctrl;

	printk("Setting Sensor-ctrl!! %d /n", ctrl_cnt);
	
	for (i = ctrl_cnt - 1; i >= 0; i--) {
		vc.id = sinfo->v4l2_re_ctrl[i];
		sensor_if_get_control(vdev, &vc);
		sensor_if_set_control(vdev, &vc, 1);
	}

	return 0;
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

	switch (fmt->type) {
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
static int sensor_try_format(struct sensor_info *sinfo, struct v4l2_pix_format *pix)
{
	int ifmt;

	/* sensor module setting size */
	pix->width  = sinfo->prv_w;
	pix->height = sinfo->prv_h;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if(pix->pixelformat == sensor_formats[ifmt].pixelformat)
			break;
	}

	if(ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;

	pix->pixelformat = sensor_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	
	switch(pix->pixelformat) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		default:
			pix->colorspace = V4L2_COLORSPACE_JPEG;
			break;			
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB555X:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			break;
	}
	return 0;
}

/* 
 * Mapping each sensor module
 */
int sensor_if_mapping(struct sensor_info *sinfo, unsigned int sensor_type)
{
	switch (sensor_type) {
	case SENSOR_MT9D112:
		sensor_init_mt9d112(sinfo);
		break;
	case SENSOR_MT9D111:
		sensor_init_mt9d111(sinfo);
		break;
	case SENSOR_OV2643:
		sensor_init_ov2643(sinfo);
		break;
	case SENSOR_OV5640:
		sensor_init_ov5640(sinfo);
		break;
	case SENSOR_NT99141:
		sensor_init_nt99141(sinfo);
		break;
	case SENSOR_TVP5150:
		sensor_init_tvp5150(sinfo);
		break;
	case SENSOR_TW2867:
		sensor_init_tw2867(sinfo);
		break;
	case SENSOR_TVP5158:
		sensor_init_tvp5158(sinfo);
		break;
	case SENSOR_THCV220:
		sensor_init_thcv220(sinfo);
		break;
	case SENSOR_TW9900:
		sensor_init_tw9900(sinfo);
		break;
	case SENSOR_ADV7182:
		sensor_init_adv7182(sinfo);
		break;
	default:
		printk("%s: invalid sensor type(%d)\n", __func__, sensor_type);
		return -EINVAL;
		break;
	}
	return 0;
}

/* 
 * Initialize the sensor
 */
int sensor_if_init(struct tcc_video_device *vdev)
{
	int ret = 0;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	struct sensor_info *sinfo = vdev->sinfo;

	struct device_node	*cam_np;//dev_node;
	struct device_node	*module_np;

	dprintk("__ in\n");

	cam_np = of_parse_phandle(vdev->dev->of_node, "telechips,cif_camera", 0);

	// TODO: remove enabled var. or other using
	vdev->enabled = 1;

	module_np = of_find_node_by_name(cam_np, MODULE_NODE);
	
	if(module_np)
	{
		gpio_data.pwr_port	= of_get_named_gpio(module_np,"pwr-gpios",0);
				if(IS_ERR(gpio_data.pwr_port)) gpio_data.pwr_port = NULL;
		gpio_data.pwd_port	= of_get_named_gpio(module_np,"pwd-gpios",0);
				if(IS_ERR(gpio_data.pwd_port)) gpio_data.pwd_port = NULL;
		gpio_data.rst_port	= of_get_named_gpio(module_np,"rst-gpios",0);
				if(IS_ERR(gpio_data.rst_port)) gpio_data.rst_port = NULL;
	}
	else
	{
		printk("could not find sensor module node!! \n");
		return -ENODEV; 
	}
	
	gpio_request(gpio_data.pwr_port, "camera power");
	gpio_direction_output(gpio_data.pwr_port, 1);

	gpio_request(gpio_data.pwd_port, "camera power down");
	gpio_direction_output(gpio_data.pwd_port, 1);
	
	gpio_request(gpio_data.rst_port, "camera reset");
	gpio_direction_output(gpio_data.rst_port, 0);

	dprintk("camera power, reset gpio port have been set\n");
	
#if 0
	/* for the management of devices that share a one sensor */
	if (pdata->cif_shared_sensor_ctrl == NULL) {
		/* initialize cif h/w port
		 * - normal case
	 	 */
		pdata->cif_port_init(dev, 0, 0);
	} else {
		if ((pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_PORT, 1, NULL) == 0)
				|| (pdata->vioc->vin_demux == VIOC_NULL)) {
			/* initialize cif h/w port
			 * - shared sensor case
		 	 */
			pdata->cif_port_init(dev, 1, vdev->video_nr);
		}
	}
	if (!(pdata->cif_shared_sensor_ctrl != NULL 
			&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_OPEN, 1, NULL))) {
		/* initialize sensor module 
	 	 */
	 	if (sinfo->open != NULL) {
			ret = sinfo->open(vdev);
	 	} else {
	 		printk("%s: sensor's open function does not exist\n", vdev->name);
	 		return -ENODEV;
	 	}
	}
#endif
	if (sinfo->open != NULL) {
		ret = sinfo->open(vdev);
	} else {
		printk("%s: sensor's open function does not exist\n", vdev->name);
		return -ENODEV;
	}

	/* Make the default capture format QVGA YUV422 */
	//vdev->pix.width = sinfo->sensor_sizes[VGA].width;
	//vdev->pix.height = sinfo->sensor_sizes[VGA].height;
	//vdev->pix.pixelformat = V4L2_PIX_FMT_YUYV;
	sensor_try_format(sinfo, &vdev->pix);
	
	dprintk("__ out\n");
	
	return ret;
}

int sensor_if_deinit(struct tcc_video_device *vdev)
{
	int ret = 0;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	struct sensor_info *sinfo = vdev->sinfo;

	dprintk("__ in \n");

	dprintk("enabled = [%d]\n", vdev->enabled);
	if (!vdev->enabled)
		return 0;

#if 0
	/* for the management of devices that share a one sensor */ 
	if (!(pdata->cif_shared_sensor_ctrl != NULL 
			&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_OPEN, 0, NULL))) {
		/* close sensor module 
		 */
		if (sinfo->close != NULL) {
			ret = sinfo->close(vdev);
		}
	}
#endif

	if (sinfo->close != NULL) {
		ret = sinfo->close(vdev);
	}

#if 0
	if (pdata->cif_shared_sensor_ctrl == NULL) {
		/* cif port deinit 
		 * - normal case
		 */
		pdata->cif_port_deinit(dev, 0, 0);
	} else {
		if ((pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_PORT, 0, NULL) == 0)
				|| (pdata->vioc->vin_demux == VIOC_NULL)) {
			/* cif port deinit 
			 * - shared sensor case
			 */
			pdata->cif_port_deinit(dev, 1, vdev->video_nr);
		}
	}
#endif
	gpio_free(gpio_data.pwr_port);
	gpio_free(gpio_data.pwd_port);
	gpio_free(gpio_data.rst_port);
	
	vdev->enabled = 0;

	dprintk("__ out \n");
    return ret;
}

int sensor_if_get_sensor_framerate(struct tcc_video_device *vdev, int *nFrameRate)
{
	*nFrameRate = vdev->sinfo->framerate;
	if (*nFrameRate)
		return 0;
	else {
		printk("Sensor Driver dosen't have frame rate information!!\n");
		return -1;
	}
}

int sensor_if_restart(struct tcc_video_device *vdev)
{
	int ret = 0;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	struct sensor_info *sinfo = vdev->sinfo;

	/* for the management of devices that share a one sensor */ 
	if (!(pdata->cif_shared_sensor_ctrl != NULL 
			&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_OPEN, 1, NULL))) {
		/* initialize sensor module 
		 */
		ret = sinfo->open(vdev);
	}
	return ret;
}

/*---------------------------------------------------------------------------------
 * wrapper functions for cif h/w contrl.
 *---------------------------------------------------------------------------------
 */
void cif_open(struct tcc_video_device *vdev)
{
	int ret;

	dprintk("__ %s in \n", __func__);
	
	if (vdev->cifmc_clk) {
		ret = clk_set_rate(vdev->cifmc_clk, vdev->sinfo->mclk * 100);
		ret = clk_enable(vdev->cifmc_clk);
		if (ret)
			printk("%s mclk error\n", vdev->sinfo->name);
		else
			printk("%s mclk %luhz\n", vdev->sinfo->name, clk_get_rate(vdev->cifmc_clk));
//		dprintk("mclk src is '%s':%dMhz\n", vdev->cifmc_clk->name, vdev->sinfo->mclk / 10000);
	}

	/* reset vion-vin components */
	vioc_vin_path_reset(vdev);

	dprintk("__ %s out \n", __func__);
}

void cif_close(struct tcc_video_device *vdev)
{
	if (vdev->cifmc_clk)
		clk_disable(vdev->cifmc_clk);
	dprintk("\n");
}
#if 0
void cif_power_enable(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_pwr_ctrl(dev, CIF_CTRL_ENABLE);
	dprintk("\n");
}
void cif_power_disable(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_pwr_ctrl(dev, CIF_CTRL_DISABLE);
	dprintk("\n");
}

void cif_powerdown_enable(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_pwdn_ctrl(dev, CIF_CTRL_ENABLE);
	dprintk("\n");
}
void cif_powerdown_disable(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_pwdn_ctrl(dev, CIF_CTRL_DISABLE);
	dprintk("\n");
}

void cif_reset_high(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_rst_ctrl(dev, CIF_RESET_HIGH);
	dprintk("\n");
}
void cif_reset_low(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	pdata->cif_rst_ctrl(dev, CIF_RESET_LOW);
	dprintk("\n");
}
#endif
void cif_set_port(struct tcc_video_device *vdev)
{
	struct device_node	*cam_np;// = dev_node;
	struct device_node *port_np;
	unsigned int nUsingPort;
	unsigned int *cifport_addr;
	unsigned int port_index, old_port_val, cifport_val;

	dprintk("__ in \n");
	
	cam_np = of_parse_phandle(vdev->dev->of_node, "telechips,cif_camera", 0);
	if(cam_np < 1) {
		pr_err("can't find telechips,cif_camera\r\n");
		return NULL;
	}

	port_np = of_parse_phandle(cam_np, "camera_port", 0);
	of_property_read_u32_index(cam_np,"camera_port",1,&nUsingPort);
	cifport_addr = of_iomap(port_np, 0);

	dprintk("cifport register = 0x%x \n",*cifport_addr);

	cifport_val = *cifport_addr;
		
	for(port_index=0;port_index<5;port_index++) {
		if(((cifport_val >> (port_index << 2)) & 0x7) == nUsingPort) {
			old_port_val = ((cifport_val >> (vdev->vioc.vioc_num.vin.index/*vioc_num.vin.index*/ << 2)) & 0x7);
			BITCSET(cifport_val, (0x7 << (port_index << 2)) | (0x7 << (vdev->vioc.vioc_num.vin.index << 2)), 
					(old_port_val << (port_index << 2)) | (nUsingPort << (vdev->vioc.vioc_num.vin.index << 2)));
			*cifport_addr = cifport_val;
			printk("find oldval=%d pos=%d, using_port=%d, index=%d\r\n", 
					old_port_val, port_index, nUsingPort, vdev->vioc.vioc_num.vin.index);
			break;
		}
	}
	
	
	if(port_index > 4) {
		BITCSET(cifport_val, 0x7 << (vdev->vioc.vioc_num.vin.index << 2), nUsingPort << (vdev->vioc.vioc_num.vin.index << 2));
		*cifport_addr = cifport_val;
	}			
	dprintk("cifport_val = 0x%x \n", *cifport_addr);
	
	dprintk("___ out \n");
}

void sensor_power_enable(void)
{
//	gpio_set_value(gpio_data.pwr_port, 1);
	gpio_set_value_cansleep(gpio_data.pwr_port, 1);

}

void sensor_power_disable(void)
{
//	gpio_set_value(gpio_data.pwr_port, 0);
	gpio_set_value_cansleep(gpio_data.pwr_port, 0);
}

int sensor_get_powerdown(void)
{
	return gpio_get_value(gpio_data.pwd_port);
}

void sensor_powerdown_enable(void)
{
//	gpio_set_value(gpio_data.pwd_port, 1);
	gpio_set_value_cansleep(gpio_data.pwd_port, 1); 
}

void sensor_powerdown_disable(void)
{
//	gpio_set_value(gpio_data.pwd_port, 0);
	gpio_set_value_cansleep(gpio_data.pwd_port, 0); 

}

void sensor_reset_high(void)
{
//	gpio_set_value(gpio_data.rst_port, 1);
	gpio_set_value_cansleep(gpio_data.rst_port, 1);
}

void sensor_reset_low(void)
{
//	gpio_set_value(gpio_data.rst_port, 0);
	gpio_set_value_cansleep(gpio_data.rst_port, 0);
}


#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
/*---------------------------------------------------------------------------------
 * detect sensor plug-in/out
 *    low: plug-in
 *   high: plug-out
 *---------------------------------------------------------------------------------
 */
static void detect_sensor_wq(struct work_struct *work)
{
    struct tcc_video_device *vdev = container_of(work, struct tcc_video_device, det_wq);
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

    vdev->det_status = gpio_get_value(pdata->port->det_gpio);
    if (vdev->det_status) {
        printk("Image sensor plug-out\n");
        kobject_uevent(&dev->kobj, KOBJ_OFFLINE);
    } else {
        printk("Image sensor plug-in\n");
        kobject_uevent(&dev->kobj, KOBJ_ONLINE);
    }
}

static irqreturn_t detect_sensor_handler(int irq, void *priv)
{
    struct tcc_video_device *vdev = priv;
    dprintk("change rear-cam plug status\n");
    schedule_work(&vdev->det_wq);
    return IRQ_HANDLED;
}

static ssize_t show_attach(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct tcc_video_device *vdev = dev_get_drvdata(dev);
	struct tcc_cif_platform_data *pdata = dev->platform_data;

    vdev->det_status = gpio_get_value(pdata->port->det_gpio);
    return sprintf(buf, "%d\n", vdev->det_status ? 0 : 1);
}
static DEVICE_ATTR(sensor_attach, S_IRUSR, show_attach, NULL);

int detect_sensor_init(struct tcc_video_device *vdev)
{
	int ret;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	tcc_gpio_config(pdata->port->det_gpio, GPIO_FN(0)|GPIO_INPUT);
	gpio_request(pdata->port->det_gpio, NULL);

	vdev->det_status = gpio_get_value(pdata->port->det_gpio);
	kobject_uevent(&dev->kobj, vdev->det_status ? KOBJ_OFFLINE : KOBJ_ONLINE);

	INIT_WORK(&vdev->det_wq, detect_sensor_wq);

	tcc_gpio_config_ext_intr(pdata->port->det_irq, pdata->port->det_eint);
	ret = request_irq(pdata->port->det_irq, detect_sensor_handler, IRQ_TYPE_EDGE_BOTH|IRQF_DISABLED, "det_sensor", vdev);
	if (ret) {
		dprintk("request_irq(%d) error(%d)\n", pdata->port->det_irq, ret);
		goto out;
	}

	ret = device_create_file(dev, &dev_attr_sensor_attach);
	if (ret) {
		dprintk("device_create_file error(%d)\n", ret);
		goto err;
	}

	return 0;
err:
	free_irq(pdata->port->det_irq, vdev);
out:
	return ret;
}

void detect_sensor_deinit(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	free_irq(pdata->port->det_irq, vdev);
	device_remove_file(dev, &dev_attr_sensor_attach);
}
#endif

void tccxxx_vioc_dt_parse_data(struct device_node *dev_node, struct tcc_video_device *vdev)
{
	struct device_node	*cam_np;// = dev_node;
	struct device_node *wmixer_np, *wdma_np, *vin_np, *scaler_np, *config_np, *viqe_np;
	int lut_offset;
	unsigned int *vioc_addr;

	dprintk("__ in \n");

	of_property_read_u32_index(dev_node,"telechips,deintls", 0, &vdev->vioc.deintls);
	of_property_read_u32_index(dev_node,"telechips,deintls_plugin", 0, &vdev->vioc.deintls_plugin);
	of_property_read_u32_index(dev_node,"telechips,wdma_mode", 0, &vdev->vioc.wdma_continuous_mode);
	of_property_read_u32_index(dev_node,"telechips,vsync_ecc", 0, &vdev->vioc.vsync_ecc);
	
	cam_np = of_parse_phandle(dev_node, "telechips,cif_camera", 0);
	if(cam_np < 1) {
		pr_err("can't find telechips,cif_camera\r\n");
		return NULL;
	}
	
	if(cam_np)
	{
		wmixer_np = of_parse_phandle(cam_np, "camera_wmixer", 0);
		if(wmixer_np) 
		{
			of_property_read_u32_index(cam_np,"camera_wmixer", 1, &vdev->vioc.vioc_num.wmixer.index);
			//vdev->vioc.vioc_num.wmixer.index);
			vioc_addr = of_iomap(wmixer_np, vdev->vioc.vioc_num.wmixer.index);
			vdev->vioc.wmix_addr = (VIOC_WMIX*)vioc_addr;//pWMIXBase	= (VIOC_WMIX*)vioc_addr;
		}
		else
		{
			printk("could not find camera wmixer node!! \n");
		}
		
		wdma_np = of_parse_phandle(cam_np, "camera_wdma", 0);
		if(wdma_np) 
		{
			of_property_read_u32_index(cam_np,"camera_wdma", 1, &vdev->vioc.vioc_num.wdma.index);
			vioc_addr = of_iomap(wdma_np, vdev->vioc.vioc_num.wdma.index);
			vdev->vioc.wdma_addr = (VIOC_WDMA*)vioc_addr;
		}
		else
		{
			printk("could not find camera wdma node!! \n");
		}
		
		vin_np = of_parse_phandle(cam_np, "camera_videoin", 0);
		if(vin_np) 
		{
			of_property_read_u32_index(cam_np,"camera_videoin", 2, &vdev->vioc.vioc_num.vin.index);
			//vdev->vioc.vioc_num.vin.index);
			vioc_addr = of_iomap(vin_np, vdev->vioc.vioc_num.vin.index);
			vdev->vioc.vin_addr = (VIOC_VIN*)vioc_addr;
#if 1			
			of_property_read_u32_index(cam_np,"camera_videoin", 1, &lut_offset);			
			vioc_addr  = of_iomap(vin_np, vdev->vioc.vioc_num.vin.index + lut_offset);
			vdev->vioc.pLUTBase = (unsigned int*)vioc_addr;
#endif 

		}
		else
		{
			printk("could not find video input node!! \n");
		}
		
		scaler_np = of_parse_phandle(cam_np, "camera_scaler", 0);
		if(scaler_np) 
		{
			of_property_read_u32_index(cam_np,"camera_scaler", 1, &vdev->vioc.vioc_num.scaler.index);
			vioc_addr = of_iomap(scaler_np, vdev->vioc.vioc_num.scaler.index);
			vdev->vioc.sc_addr= (VIOC_SC*)vioc_addr;
			of_property_read_u32_index(dev_node,"telechips,sc_plugin", 0, &vdev->vioc.sc_plugin);
		}
		else
		{
			printk("could not find camera scaler node!! \n");
		}

		config_np = of_parse_phandle(cam_np, "camera_config", 0);
		if(config_np) 
		{
			of_property_read_u32_index(cam_np,"camera_config", 1, &vdev->vioc.vioc_num.config.index);
			vioc_addr = of_iomap(config_np, vdev->vioc.vioc_num.config.index);
			vdev->vioc.config_addr= (VIOC_IREQ_CONFIG*)vioc_addr;
		}
		else
		{
			printk("could not find camera irqe node!! \n");
		}

		viqe_np = of_parse_phandle(cam_np, "camera_viqe", 0);
		if(viqe_np) 
		{
			of_property_read_u32_index(cam_np,"camera_viqe", 1, &vdev->vioc.vioc_num.viqe.index);
			vioc_addr = of_iomap(viqe_np, vdev->vioc.vioc_num.viqe.index);
			vdev->vioc.viqe_addr= (VIQE*)vioc_addr;
			of_property_read_u32_index(dev_node,"telechips,viqe_plugin", 0, &vdev->vioc.viqe_plugin);
		}
		else
		{
			printk("could not find camera viqe node!! \n");
		}

		vdev->vioc.vioc_num.vin_demux.index = VIOC_NULL;

	}
	else
	{
		printk("could not find camera platform node!! \n");
	}

	dprintk("__ out \n");
}

#if 0
void tccxxx_vioc_component_reset(void)
{
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_WDMA,vdev->vioc.vioc_num.wdma.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_WMIXER,vdev->vioc.vioc_num.wmixer.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_SCALER,vdev->vioc.vioc_num.scaler.index,VIOC_CONFIG_RESET);
#if defined(CONFIG_TCC_VIOCMG)
	if(viocmg_is_feature_rear_cam_enable() && viocmg_is_feature_rear_cam_use_viqe())
		VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_DEINTS,0,VIOC_CONFIG_RESET);  
	else 
#endif
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_VIQE,vdev->vioc.vioc_num.viqe.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_VIN,vdev->vioc.vioc_num.vin.index,VIOC_CONFIG_RESET);
	
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_VIN,vdev->vioc.vioc_num.vin.index,VIOC_CONFIG_CLEAR);	
#if defined(CONFIG_TCC_VIOCMG)
	if(viocmg_is_feature_rear_cam_enable() && viocmg_is_feature_rear_cam_use_viqe())
		VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_DEINTS,0, VIOC_CONFIG_CLEAR);  
	else 
#endif
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_VIQE,vdev->vioc.vioc_num.viqe.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_SCALER,vdev->vioc.vioc_num.scaler.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_WMIXER,vdev->vioc.vioc_num.wmixer.index,VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset(pVIOCConfig,VIOC_CONFIG_WDMA,vdev->vioc.vioc_num.wdma.index,VIOC_CONFIG_CLEAR);	

}
#endif
