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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/poll.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_disp.h>
#else
#include <video/tcc/tcc_types.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_disp.h>
#endif

//static int debug	   		= 0;
//#define dprintk(msg...)	if(debug) { 	printk( "dispman:  " msg); 	}

#define DEVICE_NAME			"tcc_dispman"
#define MANAGER_MAJOR_ID	250
#define MANAGER_MINOR_ID 	1

static int tcc_dispman_major = MANAGER_MAJOR_ID;

enum {
	MGEM_TCC_HDMI_720P_FIXED = 0,
	MGEM_TCC_AUDIO_HDMI_LINK,
	MGEM_TCC_AUDIO_SAMPLING_RATE,
	MGEM_TCC_AUDIO_CHANNELS,
	MGEM_TCC_CEC_IMAGEVIEW_ON,
	MGEM_TCC_CEC_TEXTVIEW_ON,
	MGEM_TCC_CEC_ACTIVE_SOURCE,
	MGEM_TCC_CEC_IN_ACTIVE_SOURCE,
	MGEM_TCC_CEC_STANDBY,
	MGEM_TCC_HDMI_AUDIO_TYPE,
	MGEM_TCC_OUTPUT_DISPLAY_TYPE,
	MGEM_TCC_OUTPUT_HDMI_3D_FORMAT,
	MGEM_TCC_OUTPUT_HDMI_AUDIO_ONOFF,
	MGEM_TCC_OUTPUT_HDMI_AUDIO_DISABLE,
	MGEM_TCC_OUTPUT_HDMI_VIDEO_FORMAT,
	MGEM_TCC_OUTPUT_HDMI_STRUCTURE_3D, 
	MGEM_TCC_VIDEO_HDMI_RESOLUTION, 
	MGEM_TCC_OUTPUT_MODE_DETECTED, 
	MGEM_TCC_OUTPUT_MODE_STB, 
	MGEM_TCC_OUTPUT_MODE_PLUGOUT,
	MGEM_TCC_2D_COMPRESSION,

	MGEM_PERSIST_DISPLAY_MODE,
	MGEM_PERSIST_EXTENDDISPLAY_RESET,
	MGEM_PERSIST_OUTPUT_ATTACH_MAIN,
	MGEM_PERSIST_OUTPUT_ATTACH_SUB,
	MGEM_PERSIST_OUTPUT_MODE,
	MGEM_PERSIST_AUTO_RESOLUTION,
	MGEM_PERSIST_SPDIF_SETTING,
	MGEM_PERSIST_HDMI_MODE,
	MGEM_PERSIST_HDMI_RESIZE_UP,
	MGEM_PERSIST_HDMI_RESIZE_DN, 
	MGEM_PERSIST_HDMI_RESIZE_LT, 
	MGEM_PERSIST_HDMI_RESIZE_RT, 
	MGEM_PERSIST_HDMI_CEC,
	MGEM_PERSIST_HDMI_RESOLUTION,
	MGEM_PERSIST_HDMI_DETECTED_RES,
	MGEM_PERSIST_HDMI_AUTO_SELECT,
	MGEM_PERSIST_HDMI_COLOR_DEPTH,
	MGEM_PERSIST_HDMI_COLOR_SPACE,
	MGEM_PERSIST_HDMI_ASPECT_RATIO,
	MGEM_PERSIST_HDMI_DETECTED,
	MGEM_PERSIST_HDMI_DETECTED_MODE,
	MGEM_PERSIST_COMPOSITE_MODE,
	MGEM_PERSIST_COMPOSITE_RESIZE_UP,
	MGEM_PERSIST_COMPOSITE_RESIZE_DN, 
	MGEM_PERSIST_COMPOSITE_RESIZE_LT, 
	MGEM_PERSIST_COMPOSITE_RESIZE_RT, 
	MGEM_PERSIST_COMPOSITE_DETECTED,
	MGEM_PERSIST_COMPONENT_MODE,
	MGEM_PERSIST_COMPONENT_RESIZE_UP,
	MGEM_PERSIST_COMPONENT_RESIZE_DN, 
	MGEM_PERSIST_COMPONENT_RESIZE_LT, 
	MGEM_PERSIST_COMPONENT_RESIZE_RT,
	MGEM_PERSIST_COMPONENT_DETECTED,
	MGEM_TCCDISPMAN_SUPPORT_RESOLUTION_COUNT,

	//+ COLOR ENHANCEMENT
	MGEM_COLOR_ENHANCE_LCD_HUE,
	MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS,
	MGEM_COLOR_ENHANCE_LCD_CONTRAST,
	MGEM_COLOR_ENHANCE_HDMI_HUE,
	MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS,
	MGEM_COLOR_ENHANCE_HDMI_CONTRAST,
	MGEM_COLOR_ENHANCE_COMPOSITE_HUE,
	MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS,
	MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST,
	MGEM_COLOR_ENHANCE_COMPONENT_HUE,
	MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS,
	MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST,	
	//- COLOR ENHANCEMENT	

	MGEM_PERSIST_HDMI_NATIVE_FIRST,
	MGEM_PERSIST_HDMI_HW_CTS,
	MGEM_ATTR_MAX
};

#define SR_ATTR_MAX	500

static DEFINE_MUTEX(tcc_dispman_mutex);

static  atomic_t tcc_dispman_attribute_data[MGEM_ATTR_MAX];

static  int  tcc_dispman_support_resolution_length = 0;
static  char tcc_dispman_support_resolution[SR_ATTR_MAX];

extern struct lcd_panel *tccfb_get_panel(void);
extern unsigned int tca_get_lcd_lcdc_num(void);
extern unsigned int tca_get_output_lcdc_num(void);

// Support Layer Ctrl
extern TCC_LAYER_MODE Output_LayerMode;

// Support Lock_F
static  atomic_t tcc_dispman_lock_file;


#define DBGENHANCE		0


void tcc_init_display_enhancement(void)
{
	int dispdev_id, dispdev_ext_id; 
	signed char hue, brightness, contrast;
	VIOC_DISP *pDISP;
	
	dispdev_id = tca_get_lcd_lcdc_num();
	dispdev_ext_id = tca_get_output_lcdc_num();
	
	if (dispdev_id)
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
	else
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);

	VIOC_DISP_GetColorEnhancement(pDISP, &contrast, &brightness, &hue);

	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_HUE], hue);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS], brightness);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_CONTRAST], contrast);

	if (dispdev_ext_id)
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
	else
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);

	VIOC_DISP_GetColorEnhancement(pDISP, &contrast, &brightness, &hue);

	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_HUE], hue);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS], brightness);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_CONTRAST], contrast);

	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_HUE], hue);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS], brightness);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST], contrast);

	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_HUE], hue);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS], brightness);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST], contrast);
}

void tcc_set_display_enhancement(unsigned int OutputMode)
{
	int dispdev_id;
	signed char hue, brightness, contrast;
	VIOC_DISP *pDISP;
	
	switch(OutputMode) {
		case OUTPUT_NONE:
			dispdev_id = tca_get_lcd_lcdc_num();

			hue = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_HUE]);
			brightness = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS]);
			contrast = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_CONTRAST]);

			#if DBGENHANCE
			printk("ENHANCE LCD HUE(%d), BRIGHTNESS(%d), CONTRAST(%d)\r\n", hue, brightness, contrast);
			#endif
			break;
			
		case OUTPUT_HDMI:
		case OUTPUT_COMPOSITE:
		case OUTPUT_COMPONENT:
			dispdev_id = tca_get_output_lcdc_num();
			switch(OutputMode) {
				case OUTPUT_HDMI:
					hue = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_HUE]);
					brightness = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS]);
					contrast = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_CONTRAST]);
					#if DBGENHANCE
					printk("ENHANCE HDMI HUE(%d), BRIGHTNESS(%d), CONTRAST(%d)\r\n", hue, brightness, contrast);
					#endif
					break;
				case OUTPUT_COMPOSITE:
					hue = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_HUE]);
					brightness = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS]);
					contrast = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST]);
					#if DBGENHANCE
					printk("ENHANCE COMPOSITE HUE(%d), BRIGHTNESS(%d), CONTRAST(%d)\r\n", hue, brightness, contrast);
					#endif
					break;
				case OUTPUT_COMPONENT:				
					hue = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_HUE]);
					brightness = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS]);
					contrast = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST]);
					#if DBGENHANCE
					printk("ENHANCE COMPONENT HUE(%d), BRIGHTNESS(%d), CONTRAST(%d)\r\n", hue, brightness, contrast);
					#endif

					break;
			}
			break;
		default:
			return;
	}

	if (dispdev_id)
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
	else
		pDISP = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);

	VIOC_DISP_SetColorEnhancement(pDISP, contrast, brightness, hue);
}


static ssize_t tcc_hdmi_720p_fixed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_HDMI_720P_FIXED]));
}
static ssize_t tcc_hdmi_720p_fixed_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_HDMI_720P_FIXED], data);
	return count;
}

static ssize_t tcc_audio_hdmi_link_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_HDMI_LINK]));
}
static ssize_t tcc_audio_hdmi_link_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_HDMI_LINK], data);
	return count;
}

static ssize_t tcc_audio_sampling_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_SAMPLING_RATE]));
}
static ssize_t tcc_audio_sampling_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_SAMPLING_RATE], data);
	return count;
}

static ssize_t tcc_audio_channels_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_CHANNELS]));
}
static ssize_t tcc_audio_channels_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_AUDIO_CHANNELS], data);
	return count;
}

static ssize_t tcc_cec_imageview_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_CEC_IMAGEVIEW_ON]));
}

static ssize_t tcc_cec_imageview_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_CEC_IMAGEVIEW_ON], data);
	return count;
}


static ssize_t tcc_cec_textview_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_CEC_TEXTVIEW_ON]));
}

static ssize_t tcc_cec_textview_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_CEC_TEXTVIEW_ON], data);
	return count;
}


static ssize_t tcc_cec_active_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_CEC_ACTIVE_SOURCE]));
}

static ssize_t tcc_cec_active_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_CEC_ACTIVE_SOURCE], data);
	return count;
}


static ssize_t tcc_cec_in_active_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_CEC_IN_ACTIVE_SOURCE]));
}

static ssize_t tcc_cec_in_active_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_CEC_IN_ACTIVE_SOURCE], data);
	return count;
}


static ssize_t tcc_cec_standby_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_CEC_STANDBY]));
}

static ssize_t tcc_cec_standby_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_CEC_STANDBY], data);
	return count;
}


static ssize_t tcc_hdmi_audio_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_HDMI_AUDIO_TYPE]));
}

static ssize_t tcc_hdmi_audio_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_HDMI_AUDIO_TYPE], data);
	return count;
}

static ssize_t tcc_output_display_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_DISPLAY_TYPE]));
}

static ssize_t tcc_output_display_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_DISPLAY_TYPE], data);
	return count;
}

static ssize_t tcc_output_hdmi_3d_format_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_3D_FORMAT]));
}

static ssize_t tcc_output_hdmi_3d_format_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_3D_FORMAT], data);
	return count;
}


static ssize_t tcc_output_hdmi_audio_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_AUDIO_ONOFF]));
}

static ssize_t tcc_output_hdmi_audio_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_AUDIO_ONOFF], data);
	return count;
}

static ssize_t tcc_output_hdmi_audio_disable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_AUDIO_DISABLE]));
}

static ssize_t tcc_output_hdmi_audio_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_AUDIO_DISABLE], data);
	return count;
}

static ssize_t tcc_output_hdmi_video_format_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_VIDEO_FORMAT]));
}

static ssize_t tcc_output_hdmi_video_format_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_VIDEO_FORMAT], data);
	return count;
}


static ssize_t tcc_output_hdmi_structure_3d_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_STRUCTURE_3D]));
}

static ssize_t tcc_output_hdmi_structure_3d_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_HDMI_STRUCTURE_3D], data);
	return count;
}


static ssize_t tcc_video_hdmi_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_VIDEO_HDMI_RESOLUTION]));
}

static ssize_t tcc_video_hdmi_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_VIDEO_HDMI_RESOLUTION], data);
	return count;
}


static ssize_t tcc_output_mode_detected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_DETECTED]));
}

static ssize_t tcc_output_mode_detected_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_DETECTED], data);
	return count;
}

static ssize_t tcc_output_panel_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t result_size;
	
	struct lcd_panel *panel = tccfb_get_panel();

	if(panel) {
		result_size = sprintf(buf, "%d\n", panel->xres);
	}
	else {
		result_size = sprintf(buf, "%d\n", 0);
	}
		
	return result_size;
}

static ssize_t tcc_output_panel_height_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t result_size;
	struct lcd_panel *panel = tccfb_get_panel();
	
	if(panel) {
		result_size = sprintf(buf, "%d\n", panel->yres);
	}
	else {
		result_size = sprintf(buf, "%d\n", 0);
	}
		
	return result_size;
}

static ssize_t tcc_output_dispdev_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	PVIOC_DISP pDISPBase = NULL;
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *ptccfb_info = NULL;
	struct tcc_dp_device *pdp_data = NULL;
	unsigned int lcd_width, lcd_height;
	ssize_t result_size;

	ptccfb_info = info->par;

	if((ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_HDMI)
	    || (ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_COMPOSITE)  
		|| (ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_COMPONENT) ) {
		pdp_data = &ptccfb_info->pdata.Mdp_data;
	} else if((ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_HDMI)
		|| (ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_COMPOSITE)  
		|| (ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_COMPONENT) ) {
		pdp_data = &ptccfb_info->pdata.Sdp_data;
	} else {
		printk("%s Can't find  output , Main:%d, Sub :%d \n",
				__func__, 
				ptccfb_info->pdata.Mdp_data.DispDeviceType, 
				ptccfb_info->pdata.Sdp_data.DispDeviceType);

		return 0;
	}
	
	pDISPBase = pdp_data->ddc_info.virt_addr;
	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);

	if(pdp_data) {
		result_size = sprintf(buf, "%d\n", lcd_width);
	}
	else {
		result_size = sprintf(buf, "%d\n", 0);
	}
		
	return result_size;
}

static ssize_t tcc_output_dispdev_height_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	PVIOC_DISP pDISPBase = NULL;
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *ptccfb_info = NULL;
	struct tcc_dp_device *pdp_data = NULL;
	unsigned int lcd_width, lcd_height;
	ssize_t result_size;

	ptccfb_info = info->par;

	if((ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_HDMI)
	    || (ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_COMPOSITE)  
		|| (ptccfb_info->pdata.Mdp_data.DispDeviceType == TCC_OUTPUT_COMPONENT) ) {
		pdp_data = &ptccfb_info->pdata.Mdp_data;
	} else if((ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_HDMI)
		|| (ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_COMPOSITE)  
		|| (ptccfb_info->pdata.Sdp_data.DispDeviceType == TCC_OUTPUT_COMPONENT) ) {
		pdp_data = &ptccfb_info->pdata.Sdp_data;
	} else {
		printk("%s Can't find  output , Main:%d, Sub :%d \n",
				__func__, 
				ptccfb_info->pdata.Mdp_data.DispDeviceType, 
				ptccfb_info->pdata.Sdp_data.DispDeviceType);

		return 0;
	}
	
	pDISPBase = pdp_data->ddc_info.virt_addr;
	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);

	if(pdp_data) {
		result_size = sprintf(buf, "%d\n", lcd_height);
	}
	else {
		result_size = sprintf(buf, "%d\n", 0);
	}
		
	return result_size;
}

static ssize_t tcc_output_mode_stb_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_STB]));
}

static ssize_t tcc_output_mode_plugout_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_PLUGOUT]));
}

static ssize_t tcc_output_mode_plugout_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_PLUGOUT], data);
	return count;
}

int tcc_2d_compression_read(void)
{
	return atomic_read(&tcc_dispman_attribute_data[MGEM_TCC_2D_COMPRESSION]);
}
EXPORT_SYMBOL(tcc_2d_compression_read);

static ssize_t tcc_2d_compression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	    return sprintf(buf, "%d\n", tcc_2d_compression_read());
}

static ssize_t tcc_2d_compression_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long data;
    int error = kstrtoul(buf, 10, &data);
    if (error)
        return error;
    //if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_2D_COMPRESSION], data);
    return count;
}

static DEVICE_ATTR(tcc_hdmi_720p_fixed, S_IRUGO|S_IWUSR|S_IWGRP, tcc_hdmi_720p_fixed_show, tcc_hdmi_720p_fixed_store);
static DEVICE_ATTR(tcc_audio_hdmi_link, S_IRUGO|S_IWUSR|S_IWGRP, tcc_audio_hdmi_link_show, tcc_audio_hdmi_link_store);
static DEVICE_ATTR(tcc_audio_sampling_rate, S_IRUGO|S_IWUSR|S_IWGRP, tcc_audio_sampling_rate_show, tcc_audio_sampling_rate_store);
static DEVICE_ATTR(tcc_audio_channels, S_IRUGO|S_IWUSR|S_IWGRP, tcc_audio_channels_show, tcc_audio_channels_store);
static DEVICE_ATTR(tcc_cec_imageview_on, S_IRUGO|S_IWUSR|S_IWGRP, tcc_cec_imageview_on_show, tcc_cec_imageview_on_store);
static DEVICE_ATTR(tcc_cec_textview_on , S_IRUGO|S_IWUSR|S_IWGRP, tcc_cec_textview_on_show, tcc_cec_textview_on_store);
static DEVICE_ATTR(tcc_cec_active_source, S_IRUGO|S_IWUSR|S_IWGRP, tcc_cec_active_source_show, tcc_cec_active_source_store);
static DEVICE_ATTR(tcc_cec_in_active_source, S_IRUGO|S_IWUSR|S_IWGRP, tcc_cec_in_active_source_show, tcc_cec_in_active_source_store);
static DEVICE_ATTR(tcc_cec_standby, S_IRUGO|S_IWUSR|S_IWGRP, tcc_cec_standby_show, tcc_cec_standby_store);
static DEVICE_ATTR(tcc_hdmi_audio_type, S_IRUGO|S_IWUSR|S_IWGRP, tcc_hdmi_audio_type_show, tcc_hdmi_audio_type_store);
static DEVICE_ATTR(tcc_output_display_type, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_display_type_show, tcc_output_display_type_store);
static DEVICE_ATTR(tcc_output_hdmi_3d_format, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_hdmi_3d_format_show, tcc_output_hdmi_3d_format_store);
static DEVICE_ATTR(tcc_output_hdmi_audio_onoff, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_hdmi_audio_onoff_show, tcc_output_hdmi_audio_onoff_store);
static DEVICE_ATTR(tcc_output_hdmi_audio_disable, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_hdmi_audio_disable_show, tcc_output_hdmi_audio_disable_store);
static DEVICE_ATTR(tcc_output_hdmi_video_format, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_hdmi_video_format_show, tcc_output_hdmi_video_format_store);
static DEVICE_ATTR(tcc_output_hdmi_structure_3d, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_hdmi_structure_3d_show, tcc_output_hdmi_structure_3d_store);
static DEVICE_ATTR(tcc_video_hdmi_resolution, S_IRUGO|S_IWUSR|S_IWGRP, tcc_video_hdmi_resolution_show, tcc_video_hdmi_resolution_store);
static DEVICE_ATTR(tcc_output_mode_detected, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_mode_detected_show, tcc_output_mode_detected_store);

static DEVICE_ATTR(tcc_output_panel_width, S_IRUGO, tcc_output_panel_width_show, NULL);
static DEVICE_ATTR(tcc_output_panel_height, S_IRUGO, tcc_output_panel_height_show, NULL);
static DEVICE_ATTR(tcc_output_dispdev_width, S_IRUGO, tcc_output_dispdev_width_show, NULL);
static DEVICE_ATTR(tcc_output_dispdev_height, S_IRUGO, tcc_output_dispdev_height_show, NULL);

static DEVICE_ATTR(tcc_output_mode_stb, S_IRUGO, tcc_output_mode_stb_show, NULL);
static DEVICE_ATTR(tcc_output_mode_plugout, S_IRUGO|S_IWUSR|S_IWGRP, tcc_output_mode_plugout_show, tcc_output_mode_plugout_store);
static DEVICE_ATTR(tcc_2d_compression, S_IRUGO|S_IWUSR|S_IWGRP, tcc_2d_compression_show, tcc_2d_compression_store);

// == COMMON ==

static ssize_t persist_hdcp1x_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_TCC_HDMI_HDCP_USED
	return sprintf(buf, "%d\n", 1);
#else
	return sprintf(buf, "%d\n", 0);
#endif
}

static ssize_t persist_display_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_DISPLAY_MODE]));
}

static ssize_t persist_extenddisplay_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_EXTENDDISPLAY_RESET]));
}

static ssize_t persist_extenddisplay_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_EXTENDDISPLAY_RESET], data);
	return count;
}

static ssize_t persist_output_attach_main_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_ATTACH_MAIN]));
}

static ssize_t persist_output_attach_main_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_ATTACH_MAIN], data);
	return count;
}

static ssize_t persist_output_attach_sub_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_ATTACH_SUB]));
}

static ssize_t persist_output_attach_sub_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_ATTACH_SUB], data);
	return count;
}


static ssize_t persist_output_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_MODE]));
}

static ssize_t persist_output_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_MODE], data);
	return count;
}


static ssize_t persist_auto_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_AUTO_RESOLUTION]));
}

static ssize_t persist_auto_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_AUTO_RESOLUTION], data);
	return count;
}


static ssize_t persist_spdif_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_SPDIF_SETTING]));
}

static ssize_t persist_spdif_setting_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_SPDIF_SETTING], data);
	return count;
}

static DEVICE_ATTR(persist_hdcp1x_enable, S_IRUGO, persist_hdcp1x_enable_show, NULL);
static DEVICE_ATTR(persist_display_mode, S_IRUGO, persist_display_mode_show, NULL);
static DEVICE_ATTR(persist_extenddisplay_reset, S_IRUGO|S_IWUSR|S_IWGRP, persist_extenddisplay_reset_show, persist_extenddisplay_reset_store);
static DEVICE_ATTR(persist_output_attach_main, S_IRUGO|S_IWUSR|S_IWGRP, persist_output_attach_main_show, persist_output_attach_main_store);
static DEVICE_ATTR(persist_output_attach_sub, S_IRUGO|S_IWUSR|S_IWGRP, persist_output_attach_sub_show, persist_output_attach_sub_store);
static DEVICE_ATTR(persist_output_mode, S_IRUGO|S_IWUSR|S_IWGRP, persist_output_mode_show, persist_output_mode_store);
static DEVICE_ATTR(persist_auto_resolution, S_IRUGO|S_IWUSR|S_IWGRP, persist_auto_resolution_show, persist_auto_resolution_store);
static DEVICE_ATTR(persist_spdif_setting, S_IRUGO|S_IWUSR|S_IWGRP, persist_spdif_setting_show, persist_spdif_setting_store);


//== HDMI ==

static ssize_t persist_hdmi_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_MODE]));
}

static ssize_t persist_hdmi_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_MODE], data);
	return count;
}

static ssize_t persist_hdmi_resize_up_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_UP]));
}

static ssize_t persist_hdmi_resize_up_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_UP], data);
	return count;
}

static ssize_t persist_hdmi_resize_dn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_DN]));
}

static ssize_t persist_hdmi_resize_dn_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_DN], data);
	return count;
}


static ssize_t persist_hdmi_resize_lt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_LT]));
}

static ssize_t persist_hdmi_resize_lt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_LT], data);
	return count;
}

static ssize_t persist_hdmi_resize_rt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_RT]));
}

static ssize_t persist_hdmi_resize_rt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESIZE_RT], data);
	return count;
}


static ssize_t persist_hdmi_cec_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_CEC]));
}

static ssize_t persist_hdmi_cec_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_CEC], data);
	return count;
}



static ssize_t persist_hdmi_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESOLUTION]));
}

static ssize_t persist_hdmi_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESOLUTION], data);
	return count;
}



static ssize_t persist_hdmi_detected_res_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED_RES]));
}

static ssize_t persist_hdmi_detected_res_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED_RES], data);
	return count;
}


static ssize_t persist_hdmi_auto_select_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_AUTO_SELECT]));
}

static ssize_t persist_hdmi_auto_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_AUTO_SELECT], data);
	return count;
}


static ssize_t persist_hdmi_color_depth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_COLOR_DEPTH]));
}

static ssize_t persist_hdmi_color_depth_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_COLOR_DEPTH], data);
	return count;
}



static ssize_t persist_hdmi_color_space_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_COLOR_SPACE]));
}

static ssize_t persist_hdmi_color_space_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_COLOR_SPACE], data);
	return count;
}

static ssize_t persist_hdmi_aspect_ratio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_ASPECT_RATIO]));
}

static ssize_t persist_hdmi_aspect_ratio_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_ASPECT_RATIO], data);
	return count;
}



static ssize_t persist_hdmi_detected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED]));
}

static ssize_t persist_hdmi_detected_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED], data);
	if(data && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_HDMI);
	}
	return count;
}

static ssize_t persist_hdmi_detected_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED_MODE]));
}

static ssize_t persist_hdmi_detected_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED_MODE], data);
	return count;
}

static DEVICE_ATTR(persist_hdmi_mode, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_mode_show, persist_hdmi_mode_store);
static DEVICE_ATTR(persist_hdmi_resize_up, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_resize_up_show, persist_hdmi_resize_up_store);
static DEVICE_ATTR(persist_hdmi_resize_dn, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_resize_dn_show, persist_hdmi_resize_dn_store);
static DEVICE_ATTR(persist_hdmi_resize_lt, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_resize_lt_show, persist_hdmi_resize_lt_store);
static DEVICE_ATTR(persist_hdmi_resize_rt, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_resize_rt_show, persist_hdmi_resize_rt_store);
static DEVICE_ATTR(persist_hdmi_cec, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_cec_show, persist_hdmi_cec_store);
static DEVICE_ATTR(persist_hdmi_resolution, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_resolution_show, persist_hdmi_resolution_store);
static DEVICE_ATTR(persist_hdmi_detected_res, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_detected_res_show, persist_hdmi_detected_res_store);
static DEVICE_ATTR(persist_hdmi_auto_select, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_auto_select_show, persist_hdmi_auto_select_store);
static DEVICE_ATTR(persist_hdmi_color_depth, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_color_depth_show, persist_hdmi_color_depth_store);
static DEVICE_ATTR(persist_hdmi_color_space, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_color_space_show, persist_hdmi_color_space_store);
static DEVICE_ATTR(persist_hdmi_aspect_ratio, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_aspect_ratio_show, persist_hdmi_aspect_ratio_store);
static DEVICE_ATTR(persist_hdmi_detected, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_detected_show, persist_hdmi_detected_store);
static DEVICE_ATTR(persist_hdmi_detected_mode, S_IRUGO|S_IWUSR|S_IWGRP, persist_hdmi_detected_mode_show, persist_hdmi_detected_mode_store);


//==COMPOSITE==

static ssize_t persist_composite_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_MODE]));
}

static ssize_t persist_composite_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_MODE], data);
	return count;
}


static ssize_t persist_composite_resize_up_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_UP]));
}

static ssize_t persist_composite_resize_up_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_UP], data);
	return count;
}


static ssize_t persist_composite_resize_dn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_DN]));
}

static ssize_t persist_composite_resize_dn_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_DN], data);
	return count;
}

static ssize_t persist_composite_resize_lt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_LT]));
}

static ssize_t persist_composite_resize_lt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_LT], data);
	return count;
}

static ssize_t persist_composite_resize_rt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_RT]));
}

static ssize_t persist_composite_resize_rt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_RESIZE_RT], data);
	return count;
}

static ssize_t persist_composite_detected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED]));
}

static ssize_t persist_composite_detected_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED], data);
	if(data && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPOSITE);
	}
	return count;
}

static DEVICE_ATTR(persist_composite_mode, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_mode_show, persist_composite_mode_store);
static DEVICE_ATTR(persist_composite_resize_up, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_resize_up_show, persist_composite_resize_up_store);
static DEVICE_ATTR(persist_composite_resize_dn, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_resize_dn_show, persist_composite_resize_dn_store);
static DEVICE_ATTR(persist_composite_resize_lt, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_resize_lt_show, persist_composite_resize_lt_store);
static DEVICE_ATTR(persist_composite_resize_rt, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_resize_rt_show, persist_composite_resize_rt_store);
static DEVICE_ATTR(persist_composite_detected, S_IRUGO|S_IWUSR|S_IWGRP, persist_composite_detected_show, persist_composite_detected_store);


//==COMPONENT==
static ssize_t persist_component_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_MODE]));
}

static ssize_t persist_component_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_MODE], data);
	return count;
}


static ssize_t persist_component_resize_up_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_UP]));
}

static ssize_t persist_component_resize_up_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_UP], data);
	return count;
}


static ssize_t persist_component_resize_dn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_DN]));
}

static ssize_t persist_component_resize_dn_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_DN], data);
	return count;
}


static ssize_t persist_component_resize_lt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_LT]));
}

static ssize_t persist_component_resize_lt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_LT], data);
	return count;
}

static ssize_t persist_component_resize_rt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_RT]));
}

static ssize_t persist_component_resize_rt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_RESIZE_RT], data);
	return count;
}

static ssize_t persist_component_detected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED]));
}

static ssize_t persist_component_detected_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED]);	
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED], data);

	if(data && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPONENT);
	}	
	return count;
}
static DEVICE_ATTR(persist_component_mode, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_mode_show, persist_component_mode_store);
static DEVICE_ATTR(persist_component_resize_up, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_resize_up_show, persist_component_resize_up_store);
static DEVICE_ATTR(persist_component_resize_dn, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_resize_dn_show, persist_component_resize_dn_store);
static DEVICE_ATTR(persist_component_resize_lt, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_resize_lt_show, persist_component_resize_lt_store);
static DEVICE_ATTR(persist_component_resize_rt, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_resize_rt_show, persist_component_resize_rt_store);
static DEVICE_ATTR(persist_component_detected, S_IRUGO|S_IWUSR|S_IWGRP, persist_component_detected_show, persist_component_detected_store);


static ssize_t persist_supported_resolution_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_TCCDISPMAN_SUPPORT_RESOLUTION_COUNT]));
}

static ssize_t persist_supported_resolution_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	//if (data > 1) data = 1;
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCCDISPMAN_SUPPORT_RESOLUTION_COUNT], data);
	return count;
}

static ssize_t persist_supported_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret_size = 0;
	mutex_lock(&tcc_dispman_mutex);
	if(tcc_dispman_support_resolution_length > 0) {
		memcpy(buf, tcc_dispman_support_resolution, sizeof(char) * tcc_dispman_support_resolution_length);	
		ret_size = tcc_dispman_support_resolution_length;
	}
	mutex_unlock(&tcc_dispman_mutex);
	return ret_size;
}

static ssize_t persist_supported_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	mutex_lock(&tcc_dispman_mutex);
	memset(tcc_dispman_support_resolution, 0, sizeof(tcc_dispman_support_resolution));
	memcpy(tcc_dispman_support_resolution, buf, sizeof(char) * count);	
	tcc_dispman_support_resolution_length = count;
	mutex_unlock(&tcc_dispman_mutex);
	return count;
}

static DEVICE_ATTR(persist_supported_resolution_count, S_IRUGO|S_IWUSR|S_IWGRP, persist_supported_resolution_count_show, persist_supported_resolution_count_store);
static DEVICE_ATTR(persist_supported_resolution, S_IRUGO|S_IWUSR|S_IWGRP, persist_supported_resolution_show, persist_supported_resolution_store);  


// ENHANCE LCD
static ssize_t color_enhance_lcd_hue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_HUE]));
}

static ssize_t color_enhance_lcd_hue_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_HUE]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_HUE], data);

	if(prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_NONE);
	}
	return count;
}

static ssize_t color_enhance_lcd_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS]));
}

static ssize_t color_enhance_lcd_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_BRIGHTNESS], data);

	if(prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_NONE);
	}
	return count;
}

static ssize_t color_enhance_lcd_contrast_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_CONTRAST]));
}

static ssize_t color_enhance_lcd_contrast_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_CONTRAST]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_LCD_CONTRAST], data);

	if(prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_NONE);
	}
	return count;
}


static DEVICE_ATTR(color_enhance_lcd_hue, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_lcd_hue_show, color_enhance_lcd_hue_store);  
static DEVICE_ATTR(color_enhance_lcd_brightness, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_lcd_brightness_show, color_enhance_lcd_brightness_store);  
static DEVICE_ATTR(color_enhance_lcd_contrast, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_lcd_contrast_show, color_enhance_lcd_contrast_store);


// ENHANCE HDMI
static ssize_t color_enhance_hdmi_hue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_HUE]));
}

static ssize_t color_enhance_hdmi_hue_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_HUE]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_HUE], data);

	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_HDMI);		
	}
	return count;
}

static ssize_t color_enhance_hdmi_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS]));
}

static ssize_t color_enhance_hdmi_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_BRIGHTNESS], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_HDMI);		
	}
	return count;
}

static ssize_t color_enhance_hdmi_contrast_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_CONTRAST]));
}

static ssize_t color_enhance_hdmi_contrast_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_CONTRAST]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_HDMI_CONTRAST], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_HDMI);		
	}
	return count;
}


static DEVICE_ATTR(color_enhance_hdmi_hue, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_hdmi_hue_show, color_enhance_hdmi_hue_store);  
static DEVICE_ATTR(color_enhance_hdmi_brightness, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_hdmi_brightness_show, color_enhance_hdmi_brightness_store);  
static DEVICE_ATTR(color_enhance_hdmi_contrast, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_hdmi_contrast_show, color_enhance_hdmi_contrast_store);



// ENHANCE COMPOSITE
static ssize_t color_enhance_composite_hue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_HUE]));
}

static ssize_t color_enhance_composite_hue_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_HUE]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_HUE], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPOSITE);		
	}
	return count;
}

static ssize_t color_enhance_composite_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS]));
}

static ssize_t color_enhance_composite_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_BRIGHTNESS], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPOSITE);		
	}
	return count;
}

static ssize_t color_enhance_composite_contrast_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST]));
}

static ssize_t color_enhance_composite_contrast_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPOSITE_CONTRAST], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPOSITE_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPOSITE);		
	}
	return count;
}


static DEVICE_ATTR(color_enhance_composite_hue, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_composite_hue_show, color_enhance_composite_hue_store);  
static DEVICE_ATTR(color_enhance_composite_brightness, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_composite_brightness_show, color_enhance_composite_brightness_store);  
static DEVICE_ATTR(color_enhance_composite_contrast, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_composite_contrast_show, color_enhance_composite_contrast_store);




// ENHANCE COMPONENT
static ssize_t color_enhance_component_hue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_HUE]));
}

static ssize_t color_enhance_component_hue_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_HUE]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_HUE], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPONENT);		
	}
	return count;
}

static ssize_t color_enhance_component_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS]));
}

static ssize_t color_enhance_component_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_BRIGHTNESS], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPONENT);		
	}
	return count;
}

static ssize_t color_enhance_component_contrast_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST]));
}

static ssize_t color_enhance_component_contrast_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, prev_data;
	int error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if(data > 255)
		data = 255;
	prev_data = atomic_read(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST]);
	atomic_set(&tcc_dispman_attribute_data[MGEM_COLOR_ENHANCE_COMPONENT_CONTRAST], data);
	if(atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_COMPONENT_DETECTED]) && prev_data != data) {
		tcc_set_display_enhancement(OUTPUT_COMPONENT);		
	}
	return count;
}


static DEVICE_ATTR(color_enhance_component_hue, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_component_hue_show, color_enhance_component_hue_store);  
static DEVICE_ATTR(color_enhance_component_brightness, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_component_brightness_show, color_enhance_component_brightness_store);  
static DEVICE_ATTR(color_enhance_component_contrast, S_IRUGO|S_IWUSR|S_IWGRP, color_enhance_component_contrast_show, color_enhance_component_contrast_store);


static ssize_t hdmi_native_first_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_NATIVE_FIRST]));
}

static ssize_t hdmi_native_first_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned long data;
        int error = kstrtoul(buf, 10, &data);
        if (error)
                return error;
        //if (data > 1) data = 1;
        atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_NATIVE_FIRST], data);
        return count;

}
static DEVICE_ATTR(persist_hdmi_native_first, S_IRUGO|S_IWUSR|S_IWGRP, hdmi_native_first_show, hdmi_native_first_store);



static ssize_t hdmi_hw_cts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_HW_CTS]));
}

static ssize_t hdmi_hw_cts_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned long data;
        int error = kstrtoul(buf, 10, &data);
        if (error)
                return error;
        //if (data > 1) data = 1;
        atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_HW_CTS], data);
        return count;

}
static DEVICE_ATTR(persist_hdmi_hw_cts, S_IRUGO|S_IWUSR|S_IWGRP, hdmi_hw_cts_show, hdmi_hw_cts_store);

        


static ssize_t tcc_dispman_lock_file_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", atomic_read(&tcc_dispman_lock_file));
}

static ssize_t tcc_dispman_lock_file_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned long data;
        int error = kstrtoul(buf, 10, &data);
        if (error)
                return error;
        //if (data > 1) data = 1;
        atomic_set(&tcc_dispman_lock_file, data);
        return count;

}
static DEVICE_ATTR(tcc_dispman_lock_file, S_IRUGO|S_IWUSR|S_IWGRP, tcc_dispman_lock_file_show, tcc_dispman_lock_file_store);

static ssize_t tcc_dispman_video_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", Output_LayerMode);
}

static ssize_t tcc_dispman_video_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error = kstrtoul(buf, 10, &data);
	if(error)
		return error;
	Output_LayerMode = data;
	return count;
}
static DEVICE_ATTR(tcc_dispman_video_ctrl, S_IRUGO|S_IWUSR|S_IWGRP, tcc_dispman_video_ctrl_show, tcc_dispman_video_ctrl_store);

static struct attribute *tcc_dispman_attributes[] = {
	&dev_attr_tcc_hdmi_720p_fixed.attr,
	&dev_attr_tcc_audio_hdmi_link.attr,
	&dev_attr_tcc_audio_sampling_rate.attr,
	&dev_attr_tcc_audio_channels.attr,
	&dev_attr_tcc_cec_imageview_on.attr,
	&dev_attr_tcc_cec_textview_on.attr,
	&dev_attr_tcc_cec_active_source.attr,
	&dev_attr_tcc_cec_in_active_source.attr,
	&dev_attr_tcc_cec_standby.attr,
	&dev_attr_tcc_hdmi_audio_type.attr,
	&dev_attr_tcc_output_display_type.attr,
	&dev_attr_tcc_output_hdmi_3d_format.attr,
	&dev_attr_tcc_output_hdmi_audio_onoff.attr,
	&dev_attr_tcc_output_hdmi_audio_disable.attr,
	&dev_attr_tcc_output_hdmi_video_format.attr, 
	&dev_attr_tcc_output_hdmi_structure_3d.attr,
	&dev_attr_tcc_video_hdmi_resolution.attr,	
	&dev_attr_tcc_output_mode_detected.attr,
	&dev_attr_tcc_output_panel_width.attr,
	&dev_attr_tcc_output_panel_height.attr,
	&dev_attr_tcc_output_dispdev_width.attr,
	&dev_attr_tcc_output_dispdev_height.attr,
	&dev_attr_tcc_output_mode_stb.attr,
	&dev_attr_tcc_output_mode_plugout.attr,
	&dev_attr_tcc_2d_compression.attr,
	&dev_attr_persist_hdcp1x_enable.attr,
	&dev_attr_persist_display_mode.attr,
	&dev_attr_persist_extenddisplay_reset.attr,
	&dev_attr_persist_output_attach_main.attr,
	&dev_attr_persist_output_attach_sub.attr,
	&dev_attr_persist_output_mode.attr,
	&dev_attr_persist_auto_resolution.attr,
	&dev_attr_persist_spdif_setting.attr,
	&dev_attr_persist_hdmi_mode.attr,
	&dev_attr_persist_hdmi_resize_up.attr, 
	&dev_attr_persist_hdmi_resize_dn.attr, 
	&dev_attr_persist_hdmi_resize_lt.attr, 
	&dev_attr_persist_hdmi_resize_rt.attr,
	&dev_attr_persist_hdmi_cec.attr,
	&dev_attr_persist_hdmi_resolution.attr,
	&dev_attr_persist_hdmi_detected_res.attr,
	&dev_attr_persist_hdmi_auto_select.attr,
	&dev_attr_persist_hdmi_color_depth.attr,
	&dev_attr_persist_hdmi_color_space.attr,
	&dev_attr_persist_hdmi_aspect_ratio.attr,
	&dev_attr_persist_hdmi_detected.attr,
	&dev_attr_persist_hdmi_detected_mode.attr,
	&dev_attr_persist_composite_mode.attr,
	&dev_attr_persist_composite_resize_up.attr,
	&dev_attr_persist_composite_resize_dn.attr,
	&dev_attr_persist_composite_resize_lt.attr,
	&dev_attr_persist_composite_resize_rt.attr,
	&dev_attr_persist_composite_detected.attr,
	&dev_attr_persist_component_mode.attr,
	&dev_attr_persist_component_resize_up.attr,  
	&dev_attr_persist_component_resize_dn.attr,  
	&dev_attr_persist_component_resize_lt.attr,  
	&dev_attr_persist_component_resize_rt.attr,
	&dev_attr_persist_supported_resolution.attr,
	&dev_attr_persist_component_detected.attr,
	&dev_attr_persist_supported_resolution_count.attr,
	&dev_attr_color_enhance_lcd_hue.attr,
	&dev_attr_color_enhance_lcd_brightness.attr,
	&dev_attr_color_enhance_lcd_contrast.attr,
	&dev_attr_color_enhance_hdmi_hue.attr,
	&dev_attr_color_enhance_hdmi_brightness.attr,
	&dev_attr_color_enhance_hdmi_contrast.attr,
	&dev_attr_color_enhance_composite_hue.attr,
	&dev_attr_color_enhance_composite_brightness.attr,
	&dev_attr_color_enhance_composite_contrast.attr,
	&dev_attr_color_enhance_component_hue.attr,
	&dev_attr_color_enhance_component_brightness.attr,
	&dev_attr_color_enhance_component_contrast.attr,
	&dev_attr_persist_hdmi_native_first.attr,
	&dev_attr_persist_hdmi_hw_cts.attr,
	&dev_attr_tcc_dispman_lock_file.attr,
	&dev_attr_tcc_dispman_video_ctrl.attr,
	NULL
};


static struct attribute_group tcc_dispman_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = tcc_dispman_attributes,
};

extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int tcc_dispman_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "%s():  This address is not allowed. \n", __func__);
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk(KERN_ERR  "%s():  Virtual address page port error. \n", __func__);
		return -EAGAIN;
	}

	vma->vm_ops	= NULL;
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

	return 0;
}

static unsigned int tcc_dispman_poll(struct file *filp, poll_table *wait)
{
#if 1
	return 0;
#else
	int ret = 0;
	intr_data_t *msc_data = (intr_data_t *)filp->private_data;

	if (msc_data == NULL) {
		return -EFAULT;
	}
	
	poll_wait(filp, &(msc_data->poll_wq), wait);

	spin_lock_irq(&(msc_data->poll_lock));

	if (msc_data->block_operating == 0) 	{
		ret =  (POLLIN|POLLRDNORM);
	}

	spin_unlock_irq(&(msc_data->poll_lock));
	
	return ret;
#endif
}

long tcc_dispman_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	
	return 0;
}
EXPORT_SYMBOL(tcc_dispman_ioctl);

int tcc_dispman_release(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(tcc_dispman_release);

int tcc_dispman_open(struct inode *inode, struct file *filp)
{	
	return 0;
}
EXPORT_SYMBOL(tcc_dispman_open);


static struct file_operations tcc_dispman_fops = 
{
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= tcc_dispman_ioctl,
	.mmap			= tcc_dispman_mmap,
	.open			= tcc_dispman_open,
	.release		= tcc_dispman_release,
	.poll			= tcc_dispman_poll,
};




static char banner[] __initdata = KERN_INFO "dispman driver initializing....... \n";
static struct class *tcc_dispman_class;

void __exit tcc_dispman_cleanup(void)
{
	unregister_chrdev(tcc_dispman_major, DEVICE_NAME);
	device_destroy(tcc_dispman_class, MKDEV(tcc_dispman_major, MANAGER_MINOR_ID));
	class_destroy(tcc_dispman_class);
	
	return;
}
static struct device *tcc_dispman_dev;

int __init tcc_dispman_init(void)
{
	int ret;
	printk(banner);

	memset(tcc_dispman_attribute_data, 0, sizeof(tcc_dispman_attribute_data));
		
	tcc_dispman_major = MANAGER_MAJOR_ID;

	if (register_chrdev(tcc_dispman_major, DEVICE_NAME, &tcc_dispman_fops)) {
		tcc_dispman_major = register_chrdev(0, DEVICE_NAME, &tcc_dispman_fops);
		if (tcc_dispman_major <= 0) {
			printk (KERN_ERR "tcc_dispman: unable to register tcc_dispman device\n");
			return -EIO;
		}
		printk(KERN_ERR "tcc_dispman: unable to register major %d. Registered %d instead\n", MANAGER_MAJOR_ID, tcc_dispman_major);
	}
	
	tcc_dispman_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(tcc_dispman_class)) {
		unregister_chrdev(tcc_dispman_major, DEVICE_NAME);
		return PTR_ERR(tcc_dispman_class);
	}

#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
	#if defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT)
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_DISPLAY_MODE], (unsigned long)1);
	#elif defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS)
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_DISPLAY_MODE], (unsigned long)2);
	#elif defined(CONFIG_TCC_DISPLAY_MODE_DUAL_AUTO)
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_DISPLAY_MODE], (unsigned long)3);
	#endif
#endif
        // Support Lock_F
        atomic_set(&tcc_dispman_lock_file, (unsigned long)0);
        
	// HDMI AUTO RESOLUTION
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_RESOLUTION], (unsigned long)125);
	atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_VIDEO_HDMI_RESOLUTION], (unsigned long)999);

	#ifdef CONFIG_TCC_DISPLAY_MODE_USE
		printk("%s - STB Mode\n", __func__);
		// STB MODE
		atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_STB], (unsigned long)1);

		// PLUGOUT 
		atomic_set(&tcc_dispman_attribute_data[MGEM_TCC_OUTPUT_MODE_PLUGOUT], (unsigned long)0);

		// HDMI OUTPUT
		atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_OUTPUT_MODE], (unsigned long)1);
	#endif

	// auto color space
	atomic_set(&tcc_dispman_attribute_data[MGEM_PERSIST_HDMI_COLOR_SPACE], (unsigned long)125);
        
	tcc_init_display_enhancement();
	
	tcc_dispman_dev = device_create(tcc_dispman_class, NULL,MKDEV(tcc_dispman_major, MANAGER_MINOR_ID), NULL, DEVICE_NAME);

	ret = sysfs_create_group(&tcc_dispman_dev->kobj, &tcc_dispman_attribute_group);
	if(ret)
		printk("failed create sysfs\r\n");
	
	return 0;
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("dispman driver");
MODULE_LICENSE("GPL");


late_initcall(tcc_dispman_init);
module_exit(tcc_dispman_cleanup);


