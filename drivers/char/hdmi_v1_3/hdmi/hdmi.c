/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hdmi/hdmi.c
 * Description : hdmi driver
 *
 * Copyright (C) 2013 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 * ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "regs-hdmi.h"
#include <mach/hdmi_1_3_audio.h>
#include <mach/hdmi_1_3_hdmi.h>
#include <asm/mach-types.h>

#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/tca_display_config.h>

#ifdef HDMI_TX13_REV_05
#include "vwrapper.h"
#include "regs-vwrapper.h"
#endif
#include <mach/tcc_board_hdmi.h>
#include <mach/tca_fb_output.h>

#include <mach/vioc_outcfg.h>
#include <mach/vioc_disp.h>

#define VERSION 		"1.2.1" /* Driver version number */
#define HDMI_MINOR 	240 /* Major 10, Minor 240, /dev/hdmi */


/**
 * If 'SIMPLAYHD' is 1, check Ri of 127th and 128th frame -@n
 * on 3rd authentication. And also check if Ri of 127th frame is -@n
 * different from that of 128th frame. if 'SIMPLAYHD' is 0, check only Ri -@n
 * of 128th frame.
 */
#define HDMI_DEBUG 	0
#define HDMI_DEBUG_TIME 0

#if HDMI_DEBUG
#define dprintk(args...)   { printk( "hdmi1.3: " args); }
#else
#define dprintk(args...)
#endif


#if HDMI_DEBUG_TIME
unsigned long jstart, jend;
unsigned long ji2cstart, ji2cend;
#endif

static inline u8 hdmi_readb (unsigned offset){
    return readb (IOMEM(offset));
}
static inline void hdmi_writeb(u8 b, unsigned offset){
    writeb(b, IOMEM(offset));
}
static inline u32 hdmi_readl(unsigned offset){
    return readl(IOMEM(offset));
}
static inline void hdmi_writel(u32 b, unsigned offset){
    writel(b, IOMEM(offset));
}



static struct clk *hdmi_pclk = NULL;
static struct clk *hdmi_hclk = NULL;
/**
 * N value of ACR packet.@n
 * 4096  is the N value for 32 KHz sampling frequency @n
 * 6272  is the N value for 44.1 KHz sampling frequency @n
 * 12544 is the N value for 88.2 KHz sampling frequency @n
 * 25088 is the N value for 176.4 KHz sampling frequency @n
 * 6144  is the N value for 48 KHz sampling frequency @n
 * 12288 is the N value for 96 KHz sampling frequency @n
 * 24576 is the N value for 192 KHz sampling frequency @n
 */
static const unsigned int ACR_N_params[] =
{
    4096,
    6272,
    12544,
    25088,
    6144,
    12288,
    24576
};

static int /*__init*/ hdmi_init(void);
static void /*__init*/ hdmi_exit(void);
static int hdmi_open(struct inode *inode, struct file *file);
static int hdmi_release(struct inode *inode, struct file *file);
//static irqreturn_t hdmi_handler(int irq, void *dev_id);
static ssize_t hdmi_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static ssize_t hdmi_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static long hdmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static void hdmi_spd_update_checksum(void);
static int hdmi_set_spd_infoframe(struct HDMIVideoFormatCtrl VideoFormatCtrl);

static void hdmi_avi_update_checksum(void);
static void hdmi_aui_update_checksum(void);
static int hdmi_set_hdmimode(int mode);
static int hdmi_set_colorimetry(enum HDMIColorimetry);
static int hdmi_set_audio_sample_freq(enum SamplingFreq);
static int hdmi_set_audio_packet_type(enum HDMIASPType);
static int hdmi_set_audio_number_of_channels(enum ChannelNum);
static void hdmi_enable_bluescreen(unsigned char enable);

#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
int hdmi_set_yuv420_color_space(void);
#endif
int hdmi_set_color_space(enum ColorSpace);
int hdmi_set_color_depth(enum ColorDepth);
void hdmi_set_video_mode(struct device_video_params mode);
int hdmi_set_pixel_limit(enum PixelLimit);
int hdmi_set_pixel_aspect_ratio(enum PixelAspectRatio);

void hdmi_start(void);
void hdmi_stop(void);
void hdmi_audio_start(void);
void hdmi_audio_stop(void);
void hdmi_phy_reset(void);

#if defined(TELECHIPS)
// not use
//static struct device_lcdc_timing_params lcdc_timing_params;
void tcc_hdmi_power_on(void);
void tcc_hdmi_power_off(void);

void tcc_hdmi_v5_power_on(void);
void tcc_hdmi_v5_power_off(void);

static unsigned int gHdmiSettingFlag = 0;


/*static */char tcc_hdmi_open_num;
#endif

static unsigned int gHDMISuspendStatus = 0;

struct HDMIVideoParameter gHdmiVideoParms;
static unsigned int gHdmiStartFlag=0;
static unsigned int gPixelLimit=0;
static unsigned int gAudioChNum=0;
static unsigned int gSampleFreq=0;
static unsigned int gOutPacket=0;

#ifdef CONFIG_REGULATOR
static struct regulator         *vdd_v5p0 = NULL;
static struct regulator         *vdd_hdmi = NULL;
#endif


#if defined(CONFIG_SWITCH_GPIO_HDMI)
static struct platform_device tcc_hdmi_detect_device = {
	.name   = "switch-gpio-hdmi-detect",
	.id             = -1,
	.dev = {
		.platform_data = NULL,
	}, 
};
#endif


#define	PWR_STATUS_OFF		0
#define	PWR_STATUS_ON		1
typedef struct {
	int status;
}stpwrinfo;
static stpwrinfo gHdmiPwrInfo = {PWR_STATUS_OFF};


static const struct file_operations hdmi_fops =
{
    .owner          = THIS_MODULE,
    .open           = hdmi_open,
    .release        = hdmi_release,
    .read           = hdmi_read,
    .write          = hdmi_write,
    .unlocked_ioctl = hdmi_ioctl,
};

static struct miscdevice hdmi_misc_device =
{
    HDMI_MINOR,
    "hdmi",  //"HDMI",
    &hdmi_fops,
};

static struct device *pdev_hdmi;

static int hdmi_open(struct inode *inode, struct file *file)
{
    	dprintk(KERN_INFO "%s open_num:%d\n", __FUNCTION__, tcc_hdmi_open_num);
		tcc_hdmi_open_num++;

    	return 0;
}

static int hdmi_release(struct inode *inode, struct file *file)
{
    dprintk(KERN_INFO "%s\n", __FUNCTION__);

	tcc_hdmi_open_num--;

	return 0;
}

ssize_t hdmi_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    return 0;
}

ssize_t hdmi_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    return 0;
}

/**
 *      tccfb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
#ifdef CONFIG_PM_RUNTIME

static int hdmi_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_hdmi);

	return 0;
}

static int hdmi_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_hdmi);

	return 0;
}

#endif

static int hdmi_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n",__func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_hdmi->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
	      //printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_hdmi->power.usage_count, pdev_hdmi->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			hdmi_disable();
			break;
		case FB_BLANK_UNBLANK:
			hdmi_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}


long hdmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

    switch (cmd)
    {
		case HDMI_IOC_GET_PWR_STATUS:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PWR_STATUS : %d )\n", gHdmiPwrInfo.status);

			put_user(gHdmiPwrInfo.status,(unsigned int __user*)arg);

			break;
		}
		case HDMI_IOC_SET_PWR_CONTROL:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PWR_CONTROL :  %d )\n", cmd);
			if (cmd == 0)
				tcc_hdmi_power_off();
			else
				tcc_hdmi_power_on();
			break;
		}
		case HDMI_IOC_SET_PWR_V5_CONTROL:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PWR_V5_CONTROL :  %d )\n", cmd);
			if (cmd == 0)
				tcc_hdmi_v5_power_off();
			else
				tcc_hdmi_v5_power_on();
			break;
		}
		case HDMI_IOC_GET_SUSPEND_STATUS:
		{
			//dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_SUSPEND_STATUS : %d )\n", gHDMISuspendStatus);

			put_user(gHDMISuspendStatus,(unsigned int __user*)arg);

			break;
		}
        case HDMI_IOC_SET_COLORSPACE:
        {
            int space;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORSPACE)\n");

            // get arg
            if (get_user(space, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_color_space(space) )
            {
                dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORSPACE) : Not Correct Arg = %d\n", space);
                return -EFAULT;
            }

			gHdmiVideoParms.colorSpace = space;

            break;
        }
        case HDMI_IOC_SET_COLORDEPTH:
        {
            int depth;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORDEPTH)\n");

            // get arg
            if (get_user(depth, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_color_depth(depth) )
            {
                dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORDEPTH) : Not Correct Arg = %d\n", depth);
                return -EFAULT;
            }

			gHdmiVideoParms.colorDepth = depth;

            break;
        }
        case HDMI_IOC_SET_HDMIMODE:
        {
            int mode;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_HDMIMODE)\n");

            // get arg
            if (get_user(mode, (int __user *) arg))
	            return -EFAULT;

			hdmi_set_hdmimode(mode);
			
			gHdmiVideoParms.mode = mode;

            break;
        }
        case HDMI_IOC_SET_VIDEOMODE:
        {
            unsigned int ret;
		struct device_video_params video_mode;
		
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_VIDEOMODE)\n");

            // get arg
            if ( (ret = copy_from_user((void*)&video_mode,(const void*)arg,sizeof(struct device_video_params))) < 0)
            {
                return -EFAULT;
            }
#ifdef HDMI_TX13_REV_05
            video_wrapper_set_mode(video_mode);
#endif

            hdmi_set_video_mode(video_mode);

            break;
        }
		
        case HDMI_IOC_SET_VIDEOFORMAT_INFO:
        {
            enum VideoFormat video_format;
            unsigned int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_VIDEOFORMAT_INFO)\n");

            // get arg
            if ( (ret = copy_from_user((void*)&video_format,(const void*)arg,sizeof(enum VideoFormat))) < 0)
            {
                return -EFAULT;
            }

			gHdmiVideoParms.resolution = video_format;

            break;
        }

        case HDMI_IOC_GET_VIDEOCONFIG:
        {
            int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_VIDEOCONFIG)\n");

            // copy to user
            if ( (ret = copy_to_user((void*)arg,(const void*)&gHdmiVideoParms,sizeof(struct HDMIVideoParameter))) < 0)
                return -EFAULT;

            break;
        }

        case HDMI_IOC_GET_HDMISTART_STATUS:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_HDMISTART_STATUS)\n");

            put_user(gHdmiStartFlag,(unsigned int __user*)arg);

            break;
        }

#if defined(TELECHIPS)
		case HDMI_IOC_SET_LCDC_TIMING:
		{

			struct device_lcdc_timing_params lcdc_mode;
            unsigned int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_LCDC_TIMING)\n");

            // get arg
			if ( (ret = copy_from_user((void*)&lcdc_mode,(const void*)arg,sizeof(struct device_lcdc_timing_params))) < 0)	{
			    return -EFAULT;
			}

			#if 0	// Not use
			#if 1
            hdmi_set_lcdc_timing(lcdc_mode);
			#else
			memcpy(&lcdc_timing_params, &lcdc_mode, sizeof(struct device_lcdc_timing_params));
			#endif
			#endif//
            break;
		}
#endif /*TELECHIPS*/

        case HDMI_IOC_SET_BLUESCREEN:
        {
            unsigned char val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_BLUESCREEN)\n");

            // get arg
            if (get_user(val, (unsigned char __user *) arg))
                return -EFAULT;

            if (val) // if on
            {
				dprintk(" ON\n");
				hdmi_enable_bluescreen(1);
            }
            else // if off
            {
				dprintk(" OFF\n");
				hdmi_enable_bluescreen(0);
            }

            break;
        }
        case HDMI_IOC_SET_PIXEL_LIMIT:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PIXEL_LIMIT)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_pixel_limit(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gPixelLimit = val;

            break;
        }
        case HDMI_IOC_GET_PIXEL_LIMIT:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PIXEL_LIMIT)\n");

            // put to user
            if (put_user(gPixelLimit, (int __user *) arg))
                return -EFAULT;

            break;
        }

        case HDMI_IOC_SET_PIXEL_ASPECT_RATIO:
        {
            int val;
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PIXEL_ASPECT_RATIO)\n");
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_pixel_aspect_ratio(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gHdmiVideoParms.pixelAspectRatio = val;
			
            break;
        }
	case HDMI_IOC_SET_COLORIMETRY:
	{
		int val;

		dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORIMETRY)\n");

		if (get_user(val, (int __user *) arg))
			return -EFAULT;

		if (!hdmi_set_colorimetry(val))
		{
			dprintk(KERN_INFO "invalid argument!\n");
			return -EFAULT;
		}
		break;
	}
        case HDMI_IOC_SET_AVMUTE:
        {
            unsigned char val,reg;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AVMUTE)\n");

            // get arg
            if (get_user(val, (unsigned int __user *) arg))
                return -EFAULT;

            reg = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;
            if (reg)
            {
                if (val)
                {
                    // set AV Mute
                    hdmi_writeb(GCP_AVMUTE_ON,HDMI_GCP_BYTE1);
                    hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
                }
                else
                {
                    // clear AV Mute
                    hdmi_writeb(GCP_AVMUTE_OFF, HDMI_GCP_BYTE1);
                    hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
                }
            }

            break;
        }
        case HDMI_IOC_SET_AUDIOPACKETTYPE:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOPACKETTYPE)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_audio_packet_type(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gOutPacket = val;

            break;
        }
        case HDMI_IOC_GET_AUDIOPACKETTYPE:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOPACKETTYPE)\n");

			put_user(gOutPacket,(unsigned int __user*)arg);

            break;
        }

        case HDMI_IOC_SET_AUDIOSAMPLEFREQ:
        {
            int val;
//            unsigned char reg = hdmi_readb(HDMI_CON_0);
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOSAMPLEFREQ)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_audio_sample_freq(val) )
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }
#if 0
            // set audio enable
            hdmi_writeb(reg|HDMI_ASP_ENABLE ,HDMI_CON_0);
#endif /* 0 */

			gSampleFreq = val;
			
            break;
        }
        case HDMI_IOC_GET_AUDIOSAMPLEFREQ:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOSAMPLEFREQ)\n");

			put_user(gSampleFreq,(unsigned int __user*)arg);
			
            break;
        }

        case HDMI_IOC_SET_AUDIOCHANNEL:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOCHANNEL)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_audio_number_of_channels(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gAudioChNum = val;

            break;
        }
        case HDMI_IOC_GET_AUDIOCHANNEL:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOCHANNEL)\n");

			put_user(gAudioChNum,(unsigned int __user*)arg);			

            break;
        }

        case HDMI_IOC_SET_SPEAKER_ALLOCATION:
        {
            unsigned int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_SPEAKER_ALLOCATION)\n");

            // get arg
            if (get_user(val, (unsigned int __user *) arg))
                return -EFAULT;

            hdmi_writeb(val,HDMI_AUI_BYTE4);

            break;
        }
        case HDMI_IOC_GET_SPEAKER_ALLOCATION:
        {
            unsigned int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_SPEAKER_ALLOCATION)\n");

            val = hdmi_readb(HDMI_AUI_BYTE4);

            put_user(val,(unsigned int __user*)arg);

            break;
        }
		
        case HDMI_IOC_GET_PHYREADY:
        {
            unsigned char phy_status;

            //dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PHYREADY)\n");

            phy_status = hdmi_readbHDMI_PHY_STATUS);

            put_user(phy_status,(unsigned char __user*)arg);

            break;
        }

		case HDMI_IOC_SET_PHYRESET:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PHYRESET)\n");
			hdmi_phy_reset();
			break;
		}
		
        case HDMI_IOC_START_HDMI:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_START_HDMI)\n");

#ifdef HDMI_TX13_REV_05
            video_wrapper_enable(1);
#endif

			#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
				if(gHdmiVideoParms.mode == HDMI)
					hdmi_set_yuv420_color_space();
			#endif

            hdmi_start();

			#if defined(CONFIG_SWITCH_GPIO_HDMI)
			if(tcc_hdmi_detect_device.dev.platform_data != NULL)
			{
				struct hdmi_gpio_switch_data	*switch_data = tcc_hdmi_detect_device.dev.platform_data;
				switch_data->send_hdp_event(switch_data, TCC_HDMI_ON);
			}
			#endif//
            break;
        }
        case HDMI_IOC_STOP_HDMI:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_STOP_HDMI)\n");

		#if defined(CONFIG_SWITCH_GPIO_HDMI)
		if(tcc_hdmi_detect_device.dev.platform_data != NULL)
		{
			struct hdmi_gpio_switch_data	*switch_data = tcc_hdmi_detect_device.dev.platform_data;
			switch_data->send_hdp_event(switch_data, TCC_HDMI_OFF);
		}
		#endif//
			hdmi_stop();
		break;

        }
        case HDMI_IOC_SET_AUDIO_ENABLE:
        {
            unsigned char enable;
            unsigned char reg, mode;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE)\n");

            // get arg
            if (get_user(enable, (int __user *) arg))
                return -EFAULT;

			// check HDMI mode
			mode = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;

            reg = hdmi_readb(HDMI_CON_0);
            // enable audio output
            if ( enable && mode )
            {
                hdmi_aui_update_checksum();
                hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AUI_CON);
	        //  hdmi_writeb(TRANSMIT_ONCE,HDMI_AUI_CON);
				hdmi_writeb(ACR_MEASURED_CTS_MODE,HDMI_ACR_CON);
                hdmi_writeb(reg|HDMI_ASP_ENABLE,HDMI_CON_0);

				dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE) : enable\n");
            }
            else // disable encryption
            {
	            hdmi_writeb(reg& ~HDMI_ASP_ENABLE,HDMI_CON_0);
                hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AUI_CON);
                hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACR_CON);
				dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE) : disable\n");
            }

            break;
        }
        case HDMI_IOC_RESET_AUISAMPLEFREQ:
        {
            unsigned char reg = hdmi_readb(HDMI_AUI_BYTE2) & ~HDMI_AUI_SF_MASK;
            hdmi_writeb(reg, HDMI_AUI_BYTE2);
            break;
        }
		case HDMI_IOC_VIDEO_FORMAT_CONTROL:
		{
            struct HDMIVideoFormatCtrl video_format_ctrl;
            int ret;

            //dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_VIDEO_FORMAT_CONTROL)\n");

            // get size arg;
            if ( (ret = copy_from_user((void*)&video_format_ctrl,(const void*)arg,sizeof(video_format_ctrl))) < 0)
                return -EFAULT;

			hdmi_set_spd_infoframe( video_format_ctrl );

			hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_SPD_CON);

			break;
		}

		case HDMI_IOC_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			printk(KERN_INFO "HDMI: ioctl(HDMI_IOC_BLANK :  %d )\n", cmd);

			hdmi_blank(cmd);

			break;

		}

        default:
            return -EINVAL;
    }

    return 0;
}

int hdmi_set_setting_flag(unsigned int hdmi_setting)
{
	gHdmiSettingFlag = hdmi_setting;

	return 1;
}

int hdmi_get_setting_flag(unsigned int *hdmi_setting)
{
	*hdmi_setting = gHdmiSettingFlag;

	return 1;
}



void tcc_usleep(unsigned int delay)
{
	if(delay < 1000)
		udelay(delay);
	else
		msleep(delay/1000);
}

// onoff : 1 - power down
void tcc_ddi_pwdn_hdmi(char onoff)
{
	unsigned int  regl;

	// HDMI Power-on
	regl = hdmi_readl(DDICFG_PWDN);

	if(onoff)
		hdmi_writel(regl & ~PWDN_HDMI, DDICFG_PWDN);	
	else
		hdmi_writel(regl | PWDN_HDMI, DDICFG_PWDN);	

}

// onoff : 1 -reset
void tcc_ddi_swreset_hdmi(char onoff)
{
	unsigned int  regl;

	regl = hdmi_readl(DDICFG_SWRESET);

	if(onoff)
		hdmi_writel(regl & ~SWRESET_HDMI, DDICFG_SWRESET);
	else
		hdmi_writel(regl | SWRESET_HDMI, DDICFG_SWRESET);	

}

// onoff : 1 -power on
void tcc_ddi_hdmi_ctrl(unsigned int index, char onoff)
{
	unsigned int regl;

	regl = hdmi_readl(DDICFG_HDMICTRL);
	
	switch(index)
	{
		case HDMICTRL_RESET_HDMI:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_HDMI, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_HDMI, DDICFG_HDMICTRL);

		}
		break;

		case HDMICTRL_RESET_SPDIF:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_SPDIF, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_SPDIF, DDICFG_HDMICTRL);
		}
		break;
		
		case HDMICTRL_RESET_TMDS:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_TMDS, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_TMDS, DDICFG_HDMICTRL);
		}
		break;
		
		case HDMICTRL_HDMI_ENABLE:
		{
			if(onoff)
			{
				hdmi_writel(regl | HDMICTRL_HDMI_ENABLE, DDICFG_HDMICTRL);
				
			}		
			else
			{
				hdmi_writel(regl & ~HDMICTRL_HDMI_ENABLE, DDICFG_HDMICTRL);
			}


		}
		break;
	}
}
void tcc_hdmi_power_on(void)
{
	unsigned int  regl;

	dprintk(KERN_INFO "%s Start\n", __FUNCTION__);

	if (hdmi_pclk && hdmi_hclk && (gHdmiPwrInfo.status == PWR_STATUS_OFF)) {
		clk_prepare_enable(hdmi_pclk);
		clk_prepare_enable(hdmi_hclk);
	}

	tcc_usleep(100);

	#if 1 //юс╫ц
	{
		PCKC				pCKC ;
		pCKC = (CKC *)tcc_p2v(HwCKC_BASE);

		if(tca_get_output_lcdc_num())
			pCKC->PCLKCTRL05.nREG = 0x2C000000;
		else
			pCKC->PCLKCTRL03.nREG = 0x2C000000;
	}
	#endif

	if (hdmi_pclk)
		clk_set_rate(hdmi_pclk, 27*1000*1000);
	
	tca_ckc_setippwdn(PMU_ISOL_HDMI, 1); // power down
	tcc_usleep(100);
	tca_ckc_setippwdn(PMU_ISOL_HDMI, 0); // power up

	// HDMI Power-on
	tcc_ddi_pwdn_hdmi(0);
	
	// swreset DDI_BUS HDMI
	tcc_ddi_swreset_hdmi(1);
	{volatile int ttt;for(ttt=0;ttt<0x100;ttt++);}
	tcc_ddi_swreset_hdmi(0);


	// enable DDI_BUS HDMI CLK
	tcc_ddi_hdmi_ctrl(HDMICTRL_HDMI_ENABLE, 1);


	// HDMI PHY Reset
	hdmi_phy_reset();

	// HDMI SPDIF Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 0);
	tcc_usleep(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 1);

	// HDMI TMDS Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 0);
	tcc_usleep(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 1);


	// swreset DDI_BUS HDMI
	tcc_ddi_swreset_hdmi(1);
	tcc_usleep(1);
	tcc_ddi_swreset_hdmi(0);


	if(tca_get_output_lcdc_num())
		VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, 1);
	else
		VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, 0);

	tcc_ddi_hdmi_ctrl(HDMICTRL_HDMI_ENABLE, 1);

	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));


	// default video mode setting.
	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));

	gHdmiVideoParms.mode = DVI;
	gHdmiVideoParms.resolution = max_video_formats;
	gHdmiVideoParms.colorSpace = HDMI_CS_RGB;
	gHdmiVideoParms.colorDepth = HDMI_CD_24;
	gHdmiVideoParms.colorimetry = HDMI_COLORIMETRY_NO_DATA;
	gHdmiVideoParms.pixelAspectRatio = HDMI_PIXEL_RATIO_4_3;

	hdmi_set_hdmimode(gHdmiVideoParms.mode);
	hdmi_set_color_space(gHdmiVideoParms.colorSpace);
	hdmi_set_color_depth(gHdmiVideoParms.colorDepth);
	hdmi_set_pixel_aspect_ratio(gHdmiVideoParms.pixelAspectRatio);

	gHdmiPwrInfo.status = PWR_STATUS_ON;

	// disable HDCP INT
	regl = hdmi_readb(HDMI_SS_INTC_CON);
	hdmi_writeb(regl & ~(1<<HDMI_IRQ_HDCP), HDMI_SS_INTC_CON);

	// disable SPDIF INT
	regl = hdmi_readb(HDMI_SS_INTC_CON);
	hdmi_writeb(regl & ~(1<<HDMI_IRQ_SPDIF), HDMI_SS_INTC_CON);

	dprintk(KERN_INFO "%s End\n", __FUNCTION__);

}



void tcc_hdmi_power_off(void)
{

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

	// HDMI PHY Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 0);
	tcc_usleep(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 1);

	// HDMI SPDIF Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 0);
	tcc_usleep(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 1);

	// HDMI TMDS Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 0);
	tcc_usleep(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 1);

	// swreset DDI_BUS HDMI
	tcc_ddi_swreset_hdmi(1);
	tcc_usleep(1);
	tcc_ddi_swreset_hdmi(0);

	// disable DDI_BUS HDMI CLK
	tcc_ddi_hdmi_ctrl(HDMICTRL_HDMI_ENABLE, 0);

	// ddi hdmi power down
	tcc_ddi_pwdn_hdmi(1);

	// enable HDMI PHY Power-off
	tca_ckc_setippwdn(PMU_ISOL_HDMI, 1); // power up
	
	// gpio power on
	tcc_usleep(100);

	// enable HDMI Power-down
	if (hdmi_pclk && hdmi_hclk && (gHdmiPwrInfo.status == PWR_STATUS_ON)) {
		clk_disable_unprepare(hdmi_pclk);
		clk_disable_unprepare(hdmi_hclk);
	}
	
	gHdmiPwrInfo.status = PWR_STATUS_OFF;
	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));
 }


void tcc_hdmi_v5_power_on(void)
{

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

#if defined(CONFIG_REGULATOR)
	if(vdd_v5p0) {
		regulator_enable(vdd_v5p0);
	}

	if(vdd_hdmi) {
		regulator_enable(vdd_hdmi);
	}
#endif

}

void tcc_hdmi_v5_power_off(void)
{

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

#if defined(CONFIG_REGULATOR)
	if(vdd_v5p0) {
		regulator_disable(vdd_v5p0);
	}

	if(vdd_hdmi) {
		regulator_disable(vdd_hdmi);
	}
#endif

}

static int hdmi_remove(struct platform_device *pdev)
{
	unsigned char reg;

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

    // disable HDCP INT
    reg = hdmi_readb(HDMI_SS_INTC_CON);
    hdmi_writeb(reg & ~(1<<HDMI_IRQ_HDCP), HDMI_SS_INTC_CON);

    // disable hdmi
    reg = hdmi_readb(HDMI_CON_0);
    hdmi_writeb(reg & ~HDMI_SYS_ENABLE,HDMI_CON_0);


	gHdmiPwrInfo.status = PWR_STATUS_OFF;
	
    misc_deregister(&hdmi_misc_device);

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(pdev_hdmi);
#endif

	return 0;
	
}


/**
 * Set checksum in SPD InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_spd_update_checksum(void)
{
    unsigned char index, checksum;

    checksum = SPD_HEADER;

    for (index = 0; index < SPD_PACKET_BYTE_LENGTH; index++)
    {
        checksum += hdmi_readb(HDMI_SPD_DATA1 + 4*index);
    }

    hdmi_writeb(~checksum+1,HDMI_SPD_CHECK_SUM);
}

/**
 * Set checksum in Audio InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_aui_update_checksum(void)
{
    unsigned char index, checksum;

    checksum = AUI_HEADER;
    for (index = 0; index < AUI_PACKET_BYTE_LENGTH; index++)
    {
#if 1
        // when write this byte(PB5), HW shift 3 bit to right direction.
        // to compensate it, when read it, SW should shift 3 bit to left.
        if (index == 4)
            checksum += (hdmi_readb(HDMI_AUI_BYTE1 + 4*index)<<3);
        else
            checksum += hdmi_readb(HDMI_AUI_BYTE1 + 4*index);
#else
        checksum += hdmi_readb(HDMI_AUI_BYTE1 + 4*index);
#endif
    }
	
    hdmi_writeb(~checksum+1,HDMI_AUI_CHECK_SUM);
}

/**
 * Set checksum in AVI InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_avi_update_checksum(void)
{
    unsigned char index, checksum;

    checksum = AVI_HEADER;
    for (index = 0; index < AVI_PACKET_BYTE_LENGTH; index++)
    {
        checksum += hdmi_readb(HDMI_AVI_BYTE1 + 4*index);
    }
    hdmi_writeb(~checksum+1,HDMI_AVI_CHECK_SUM);
}

/**
 * Set color space in HDMI H/W. @n
 * @param   space   [in] Color space
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_color_space(enum ColorSpace space)
{
	VIOC_DISP *pDispBase;
    unsigned char reg,aviYY;
    int ret = 1;
	unsigned int iPXWD=12, iR2YMD=0, iR2Y=0, iSWAP=0;

	if(tca_get_output_lcdc_num())
		pDispBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
	else
		pDispBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);

    reg = hdmi_readb(HDMI_CON_0);
    aviYY = hdmi_readb(HDMI_AVI_BYTE1);
    // clear fields
    hdmi_writeb(aviYY & ~(AVI_CS_Y422|AVI_CS_Y444),HDMI_AVI_BYTE1);

    if (space == HDMI_CS_YCBCR422)
    {
        // set video input interface
        hdmi_writeb( reg | HDMI_YCBCR422_ENABLE, HDMI_CON_0);
        // set avi
        hdmi_writeb( aviYY | AVI_CS_Y422, HDMI_AVI_BYTE1);

		iPXWD = 8;
		iR2YMD = 3;
		iR2Y = 1;
		iSWAP =0;
    }
    else
    {
        // set video input interface
        hdmi_writeb( reg & ~HDMI_YCBCR422_ENABLE, HDMI_CON_0);
        if (space == HDMI_CS_YCBCR444)
        {
            // set AVI packet
            hdmi_writeb( aviYY | AVI_CS_Y444, HDMI_AVI_BYTE1);

			iPXWD = 12;
			iR2YMD = 3;
			iR2Y = 1;
			iSWAP = 4;
			
        }
        // aviYY for RGB = 0, nothing to set
        else if (space == HDMI_CS_RGB)
        {
			iPXWD = 12;
			iR2YMD = 0;
			iR2Y = 0;
			iSWAP = 0;
        }
    }

	VIOC_DISP_SetPXDW(pDispBase,  iPXWD);
	VIOC_DISP_SetR2YMD(pDispBase, iR2YMD);
	VIOC_DISP_SetR2Y(pDispBase, iR2Y);
	VIOC_DISP_SetSWAP(pDispBase, iSWAP);

    return ret;
}


#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
int hdmi_set_yuv420_color_space(void)
{
	unsigned char reg,aviYY;

    reg = hdmi_readb(HDMI_CON_0);
    aviYY = hdmi_readb(HDMI_AVI_BYTE1);
    // clear fields
    hdmi_writeb(aviYY & ~(AVI_CS_Y422|AVI_CS_Y444),HDMI_AVI_BYTE1);

    // set video input interface
    hdmi_writeb( reg | HDMI_YCBCR422_ENABLE, HDMI_CON_0);
    // set avi
    hdmi_writeb( aviYY | AVI_CS_Y422, HDMI_AVI_BYTE1);

	return 1;
}
#endif

/**
 * Set color depth.@n
 * @param   depth   [in] Color depth of input vieo stream
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_color_depth(enum ColorDepth depth)
{
    int ret = 1;

    switch (depth)
    {
        case HDMI_CD_36:
            // set GCP CD
            hdmi_writeb(GCP_CD_36BPP,HDMI_GCP_BYTE2);
            // set DC_CTRL
            hdmi_writeb(HDMI_DC_CTL_12,HDMI_DC_CONTROL);
            break;
        case HDMI_CD_30:
            // set GCP CD
            hdmi_writeb(GCP_CD_30BPP,HDMI_GCP_BYTE2);
            // set DC_CTRL
            hdmi_writeb(HDMI_DC_CTL_10,HDMI_DC_CONTROL);
            break;
        case HDMI_CD_24:
            // set GCP CD
            hdmi_writeb(GCP_CD_24BPP,HDMI_GCP_BYTE2);
            // set DC_CTRL
            hdmi_writeb(HDMI_DC_CTL_8,HDMI_DC_CONTROL);
            break;
        default:
            ret = 0;
		break;
    }
    return ret;
}

/**
 * Set video timing parameters.@n
 * @param   mode   [in] Video timing parameters
 */
void hdmi_set_video_mode(struct device_video_params mode)
{
    unsigned char reg;
    unsigned int  val;

    // set HBLANK;
    val = mode.HBlank;
    reg = val & 0xff;
    hdmi_writeb(reg,HDMI_H_BLANK_0);
    reg = (val>>8) & 0xff;
    hdmi_writeb(reg,HDMI_H_BLANK_1);

    // set VBlank
    val = mode.VBlank;
    reg = val & 0xff;
    hdmi_writeb(reg, HDMI_V_BLANK_0);
    reg = (val>>8) & 0xff;
    hdmi_writeb(reg, HDMI_V_BLANK_1);
    reg = (val>>16) & 0xff;
    hdmi_writeb(reg, HDMI_V_BLANK_2);

    // set HVLine
    val = mode.HVLine;
    reg = val & 0xff;
    hdmi_writeb(reg, HDMI_H_V_LINE_0);
    reg = (val>>8) & 0xff;
    hdmi_writeb(reg, HDMI_H_V_LINE_1);
    reg = (val>>16) & 0xff;
    hdmi_writeb(reg, HDMI_H_V_LINE_2);

    // set VSync Polarity
    hdmi_writeb(mode.polarity, HDMI_VSYNC_POL);

    // set HSyncGen
    val = mode.HSYNCGEN;
    reg = val & 0xff;
    hdmi_writeb(reg, HDMI_H_SYNC_GEN_0);
    reg = (val>>8) & 0xff;
    hdmi_writeb(reg, HDMI_H_SYNC_GEN_1);
    reg = (val>>16) & 0xff;
    hdmi_writeb(reg, HDMI_H_SYNC_GEN_2);

    // set VSyncGen1
    val = mode.VSYNCGEN;
    reg = val & 0xff;
    hdmi_writeb(reg, HDMI_V_SYNC_GEN1_0);
    reg = (val>>8) & 0xff;
    hdmi_writeb(reg, HDMI_V_SYNC_GEN1_1);
    reg = (val>>16) & 0xff;
    hdmi_writeb(reg, HDMI_V_SYNC_GEN1_2);

    // set interlace or progresive mode
    hdmi_writeb(mode.interlaced,HDMI_INT_PRO_MODE);

    if ( mode.interlaced ) // interlaced mode
    {
        // set VBlank_F
        val = mode.VBLANK_F;
        reg = val & 0xff;
        hdmi_writeb(reg, HDMI_V_BLANK_F_0);
        reg = (val>>8) & 0xff;
        hdmi_writeb(reg, HDMI_V_BLANK_F_1);
        reg = (val>>16) & 0xff;
        hdmi_writeb(reg, HDMI_V_BLANK_F_2);

        // set VSyncGen2
        val = mode.VSYNCGEN2;
        reg = val & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN2_0);
        reg = (val>>8) & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN2_1);
        reg = (val>>16) & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN2_2);

        // set VSyncGen3
        val = mode.VSYNCGEN3;
        reg = val & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN3_0);
        reg = (val>>8) & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN3_1);
        reg = (val>>16) & 0xff;
        hdmi_writeb(reg, HDMI_V_SYNC_GEN3_2);
    }
    else
    {
        // set VBlank_F with default value
        hdmi_writeb(0x00, HDMI_V_BLANK_F_0);
        hdmi_writeb(0x00, HDMI_V_BLANK_F_1);
        hdmi_writeb(0x00, HDMI_V_BLANK_F_2);

        // set VSyncGen2 with default value
        hdmi_writeb(0x01, HDMI_V_SYNC_GEN2_0);
        hdmi_writeb(0x10, HDMI_V_SYNC_GEN2_1);
        hdmi_writeb(0x00, HDMI_V_SYNC_GEN2_2);

        // set VSyncGen3 with default value
        hdmi_writeb(0x01, HDMI_V_SYNC_GEN3_0);
        hdmi_writeb(0x10, HDMI_V_SYNC_GEN3_1);
        hdmi_writeb(0x00, HDMI_V_SYNC_GEN3_2);
    }

    // set pixel repetition
    reg = readb(HDMI_CON_1);
    if ( mode.repetition )
    {
        // set pixel repetition
        hdmi_writeb(reg|HDMICON1_DOUBLE_PIXEL_REPETITION,HDMI_CON_1);
        // set avi packet
        hdmi_writeb(AVI_PIXEL_REPETITION_DOUBLE,HDMI_AVI_BYTE5);
    }
    else
    {
        // clear pixel repetition
        hdmi_writeb(reg & ~(1<<1|1<<0),HDMI_CON_1);
        // set avi packet
        hdmi_writeb(0x00,HDMI_AVI_BYTE5);
    }

    // set AVI packet with VIC
	reg = hdmi_readb(HDMI_AVI_BYTE2);

	if (reg & (unsigned char)AVI_PICTURE_ASPECT_RATIO_4_3)
		hdmi_writeb(mode.AVI_VIC,HDMI_AVI_BYTE4);
	else
		hdmi_writeb(mode.AVI_VIC_16_9,HDMI_AVI_BYTE4);
    return;
}

/**
 * Set pixel limitation.
 * @param   limit   [in] Pixel limitation.
* @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_pixel_limit(enum PixelLimit limit)
{
    int ret = 1;
    unsigned char reg,aviQQ;

    // clear field
	reg = hdmi_readb(HDMI_CON_1);
    reg &= ~HDMICON1_LIMIT_MASK;

	aviQQ = hdmi_readb(HDMI_AVI_BYTE3);
    aviQQ &= ~AVI_QUANTIZATION_MASK;

    switch (limit) // full
    {
        case HDMI_FULL_RANGE:
            aviQQ |= AVI_QUANTIZATION_FULL;
            break;
        case HDMI_RGB_LIMIT_RANGE:
            reg |= HDMICON1_RGB_LIMIT;
            aviQQ |= AVI_QUANTIZATION_LIMITED;
            break;
        case HDMI_YCBCR_LIMIT_RANGE:
            reg |= HDMICON1_YCBCR_LIMIT;
            aviQQ |= AVI_QUANTIZATION_LIMITED;
            break;
        default:
            ret = 0;
			break;
        }

    // set pixel repetition
	hdmi_writeb(reg,HDMI_CON_1);
    // set avi packet body
	hdmi_writeb(aviQQ,HDMI_AVI_BYTE3);

    return ret;
}

/**
 * Set pixel aspect ratio information in AVI InfoFrame
 * @param   ratio   [in] Pixel Aspect Ratio
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_pixel_aspect_ratio(enum PixelAspectRatio ratio)
{
    int ret = 1;
    unsigned char reg = AVI_FORMAT_ASPECT_AS_PICTURE;

    switch (ratio)
    {
		case HDMI_PIXEL_RATIO_AS_PICTURE:
			break;
        case HDMI_PIXEL_RATIO_16_9:
            reg |= AVI_PICTURE_ASPECT_RATIO_16_9;
            break;
        case HDMI_PIXEL_RATIO_4_3:
            reg |= AVI_PICTURE_ASPECT_RATIO_4_3;
            break;
        default:
            ret = 0;
			break;
     }

	hdmi_writeb(reg,HDMI_AVI_BYTE2);
    return ret;
}

/**
 * Set colorimetry information in AVI InfoFrame
 * @param   colorimetry   [in] colorimetry
 * @return  If argument is invalid, return 0; Otherwise return 1.
 */
int hdmi_set_colorimetry(enum HDMIColorimetry colorimetry)
{
	int ret = 1;
	unsigned char avi2,avi3;
	avi2 = hdmi_readb(HDMI_AVI_BYTE2);
	avi3 = hdmi_readb(HDMI_AVI_BYTE3);

	avi2 &= ~AVI_COLORIMETRY_MASK;
	avi3 &= ~AVI_COLORIMETRY_EXT_MASK;

	switch (colorimetry)
	{
		case HDMI_COLORIMETRY_NO_DATA:
			break;

		case HDMI_COLORIMETRY_ITU601:
			avi2 |= AVI_COLORIMETRY_ITU601;
			break;

		case HDMI_COLORIMETRY_ITU709:
			avi2 |= AVI_COLORIMETRY_ITU709;
			break;

		case HDMI_COLORIMETRY_EXTENDED_xvYCC601:
			avi2 |= AVI_COLORIMETRY_EXTENDED;
			avi3 |= AVI_COLORIMETRY_EXT_xvYCC601;
			break;

		case HDMI_COLORIMETRY_EXTENDED_xvYCC709:
			avi2 |= AVI_COLORIMETRY_EXTENDED;
			avi3 |= AVI_COLORIMETRY_EXT_xvYCC709;
			break;

		default:
			ret = 0;
			break;
	}

	hdmi_writeb(avi2,HDMI_AVI_BYTE2);
	hdmi_writeb(avi3,HDMI_AVI_BYTE3);
	return ret;
}


/**
 * Set HDMI/DVI mode
 * @param   mode   [in] HDMI/DVI mode
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_hdmimode(int mode)
{
	int ret = 1;

	switch(mode)
	{
		case HDMI:
	        hdmi_writeb(HDMI_MODE_SEL_HDMI,HDMI_MODE_SEL);
	        hdmi_writeb(HDMICON2_HDMI,HDMI_CON_2);
			break;
		case DVI:
	        hdmi_writeb(HDMI_MODE_SEL_DVI,HDMI_MODE_SEL);
	        hdmi_writeb(HDMICON2_DVI,HDMI_CON_2);
			break;
		default:
			ret = 0;
			break;
	}

	return ret;
}

/**
 * Set Audio Clock Recovery and Audio Infoframe packet -@n
 * based on sampling frequency.
 * @param   freq   [in] Sampling frequency
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_sample_freq(enum SamplingFreq freq)
{
    unsigned char reg;
    unsigned int n;
    int ret = 1;

    // check param
    if ( freq > sizeof(ACR_N_params)/sizeof(unsigned int) || freq < 0 )
        return 0;

    // set ACR packet
    // set N value
    n = ACR_N_params[freq];
    reg = n & 0xff;
	hdmi_writeb(0,HDMI_ACR_N0);
    hdmi_writeb(reg,HDMI_ACR_N0);
    reg = (n>>8) & 0xff;
    hdmi_writeb(reg,HDMI_ACR_N1);
    reg = (n>>16) & 0xff;
    hdmi_writeb(reg,HDMI_ACR_N2);

#if 0
    // set as measure cts mode
	hdmi_writeb(ACR_MEASURED_CTS_MODE,HDMI_ACR_CON);
#endif
    // set AUI packet
    reg = hdmi_readb(HDMI_AUI_BYTE2) & ~HDMI_AUI_SF_MASK;

    switch (freq)
    {
        case SF_32KHZ:
            reg |= HDMI_AUI_SF_SF_32KHZ;
            break;

        case SF_44KHZ:
            reg |= HDMI_AUI_SF_SF_44KHZ;
            break;

        case SF_88KHZ:
            reg |= HDMI_AUI_SF_SF_88KHZ;
            break;

        case SF_176KHZ:
            reg |= HDMI_AUI_SF_SF_176KHZ;
            break;

        case SF_48KHZ:
            reg |= HDMI_AUI_SF_SF_48KHZ;
            break;

        case SF_96KHZ:
            reg |= HDMI_AUI_SF_SF_96KHZ;
            break;

        case SF_192KHZ:
            reg |= HDMI_AUI_SF_SF_192KHZ;
            break;

        default:
            ret = 0;
			break;
    }

	hdmi_writeb(reg, HDMI_AUI_BYTE2);

    return ret;
}

/**
 * Set HDMI audio output packet type.
 * @param   packet   [in] Audio packet type
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_packet_type(enum HDMIASPType packet)
{
    int ret = 1;
    unsigned char reg;

    reg = hdmi_readb(HDMI_ASP_CON);
    reg &= ~ASP_TYPE_MASK;

    switch (packet)
    {
        case HDMI_ASP:
        {
            reg |= ASP_LPCM_TYPE;
            break;
        }
        case HDMI_DSD:
        {
            reg |= ASP_DSD_TYPE;
            break;
        }
        case HDMI_HBR:
        {
            unsigned char regb = hdmi_readb(HDMI_SS_I2S_CH_ST_3) & ~I2S_CH_ST_3_SF_MASK;
            regb |= I2S_CH_ST_3_SF_768KHZ;
            hdmi_writeb(regb, HDMI_SS_I2S_CH_ST_3);
            reg |= ASP_HBR_TYPE;
            break;
        }
        case HDMI_DST:
        {
            reg |= ASP_DST_TYPE;
            break;
        }
        default:
        {
            ret = 0;
            break;
        }
    }

    hdmi_writeb(reg,HDMI_ASP_CON);
    return ret;
}

/**
 * Set layout and sample present fields in Audio Sample Packet -@n
 * and channel number field in Audio InfoFrame packet.
 * @param   channel   [in]  Number of channels
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_number_of_channels(enum ChannelNum channel)
{
    int ret = 1;
    unsigned char reg, reg_byte4;

	reg = hdmi_readb(HDMI_ASP_CON);
    // clear field
    reg &= ~(ASP_MODE_MASK|ASP_SP_MASK);

	// clear field
	hdmi_writeb(0x00,HDMI_ASP_SP_FLAT);

    // celar field
    reg_byte4 = 0;

    // set layout & SP_PRESENT on ASP_CON
    // set AUI Packet
    switch (channel)
    {
        case CH_2:
            reg |= (ASP_LAYOUT_0|ASP_SP_0);
            hdmi_writeb(AUI_CC_2CH,HDMI_AUI_BYTE1);
            break;
        case CH_3:
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1);
            hdmi_writeb(AUI_CC_3CH,HDMI_AUI_BYTE1);
            break;
        case CH_4:
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1);
            hdmi_writeb(AUI_CC_4CH,HDMI_AUI_BYTE1);
            break;
        case CH_5:
            reg_byte4 = 0x0A;
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2);
            hdmi_writeb(AUI_CC_5CH,HDMI_AUI_BYTE1);
            break;
        case CH_6:
            reg_byte4 = 0x0A;
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2);
            hdmi_writeb(AUI_CC_6CH,HDMI_AUI_BYTE1);
            break;
        case CH_7:
            reg_byte4 = 0x12;
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2|ASP_SP_3);
            hdmi_writeb(AUI_CC_7CH,HDMI_AUI_BYTE1);
            break;
        case CH_8:
            reg_byte4 = 0x12;
            reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2|ASP_SP_3);
            hdmi_writeb(AUI_CC_8CH,HDMI_AUI_BYTE1);
            break;
        default:
            ret = 0;
    }

	hdmi_writeb(reg_byte4, HDMI_AUI_BYTE4);
	hdmi_writeb(reg,HDMI_ASP_CON);
    return ret;
}

int hdmi_set_spd_infoframe(struct HDMIVideoFormatCtrl VideoFormatCtrl)
{
	//SPD INFOFRAME PACKET HEADER
	hdmi_writeb(SPD_PACKET_TYPE,HDMI_SPD_HEADER0);
	hdmi_writeb(SPD_PACKET_VERSION,HDMI_SPD_HEADER1);
	hdmi_writeb(SPD_PACKET_BYTE_LENGTH,HDMI_SPD_HEADER2);

	//SPD INFOFRAME PACKET CONTENTS
	hdmi_writeb(SPD_PACKET_ID0,HDMI_SPD_DATA1);
	hdmi_writeb(SPD_PACKET_ID1,HDMI_SPD_DATA2);
	hdmi_writeb(SPD_PACKET_ID2,HDMI_SPD_DATA3);

	switch(VideoFormatCtrl.video_format)
	{
		case HDMI_2D:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_NONE << 5),HDMI_SPD_DATA4);
			break;
		case HDMI_VIC:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_VIC << 5),HDMI_SPD_DATA4);
			break;
		case HDMI_3D:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_3D << 5),HDMI_SPD_DATA4);
			break;
		default:
			break;
	}

	if(VideoFormatCtrl.video_format == HDMI_3D)
	{
		switch(VideoFormatCtrl.structure_3D)
		{
			case FRAME_PACKING:
				hdmi_writeb((SPD_3D_STRUCT_FRAME_PACKING << 4),HDMI_SPD_DATA5);
				break;
			case TOP_AND_BOTTOM:
				hdmi_writeb((SPD_3D_STRUCT_TOP_AND_BOTTOM << 4),HDMI_SPD_DATA5);
				break;
			case SIDE_BY_SIDE:
				hdmi_writeb((SPD_3D_STRUCT_SIDE_BY_SIDE << 4),HDMI_SPD_DATA5);
				break;
		}

		if(VideoFormatCtrl.ext_data_3D)
			hdmi_writeb(VideoFormatCtrl.ext_data_3D << 4,HDMI_SPD_DATA5);
	}
	else
	{
		hdmi_writeb(0,HDMI_SPD_DATA5);
		hdmi_writeb(0,HDMI_SPD_DATA6);
		hdmi_writeb(0,HDMI_SPD_DATA7);
	}
	
	hdmi_spd_update_checksum();

	return 1;
}

unsigned char hdmi_get_power_status(void)
{
	return gHdmiPwrInfo.status;
}


unsigned char hdmi_get_system_en(void)
{
	unsigned char hdmi_system_en = 0, reg;
	reg = hdmi_readb(HDMI_CON_0);
	hdmi_system_en = reg & 0x01;

	if(hdmi_system_en) 	{
		//dprintk(KERN_INFO "%s hdmi system is enabled : %d\n", __FUNCTION__);
	}
	else
	{
		printk(KERN_INFO "%s hdmi system is not enabled : %d\n", __FUNCTION__, hdmi_system_en);
	}

	return hdmi_system_en;
}

unsigned char hdmi_get_phy_status(void)
{
	unsigned char phy_status = 0;

	phy_status = hdmi_readb(HDMI_PHY_STATUS);

	if(phy_status) 	{
		//printk(KERN_INFO "%s phy is ready \n", __FUNCTION__);
	}
	else
	{
		printk(KERN_INFO "%s phy is not ready \n", __FUNCTION__);
	}

	return phy_status;
}

unsigned char hdmi_get_hdmimode(void)
{
	return gHdmiVideoParms.mode;
}

/**
 * hdmi_phy_reset.
 */
void hdmi_phy_reset(void)
{
	unsigned char phy_status;
	unsigned int phy_chk_cnt = 0;

	// HDMI PHY Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 0);
	tcc_usleep(100); // TCC93xx 25us, tcc8900 5us
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 1);

}

/**
 * Enable HDMI output.
 */
void hdmi_start(void)
{
    unsigned char reg,mode;

	dprintk(KERN_INFO "%s \n", __func__);

    // check HDMI mode
    mode = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;
    reg = hdmi_readb(HDMI_CON_0);

    // enable external vido gen.
    hdmi_writeb(HDMI_EXTERNAL_VIDEO,HDMI_VIDEO_PATTERN_GEN);

    if (mode) // HDMI
    {
        // enable AVI packet: mandatory
        // update avi packet checksum
        hdmi_avi_update_checksum();
        // enable avi packet
        hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AVI_CON);

		// check if audio is enable
		if (hdmi_readb(HDMI_ACR_CON))
		{
			// enable aui packet
			hdmi_aui_update_checksum();
			hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AUI_CON);
			reg |= HDMI_ASP_ENABLE;
		}

        // check if it is deep color mode or not
        if (hdmi_readb(HDMI_DC_CONTROL))
        {
            hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
        }
		else
		{
			// disable GCP
			hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GCP_CON);
		}

        	// for checking
		udelay(200);

        // enable hdmi
		#if defined(TELECHIPS)
		hdmi_writeb(reg|HDMI_SYS_ENABLE,HDMI_CON_0);
		#else
        hdmi_writeb(reg|HDMI_SYS_ENABLE|HDMI_ENCODING_OPTION_ENABLE, HDMI_CON_0);
		#endif
    }
    else // DVI
    {
        // disable all packet
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AVI_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AUI_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GCP_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GAMUT_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACP_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ISRC_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_MPG_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_SPD_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACR_CON);

        // enable hdmi without audio
        reg &= ~HDMI_ASP_ENABLE;
		#if defined(TELECHIPS)
		hdmi_writeb(reg|HDMI_SYS_ENABLE,HDMI_CON_0);
		#else
        	hdmi_writeb(reg|HDMI_SYS_ENABLE|HDMI_ENCODING_OPTION_ENABLE,HDMI_CON_0);
		#endif
    }

	hdmi_enable_bluescreen(0);

	gHdmiStartFlag = 1;

    return;
}

void hdmi_stop(void)
{
    unsigned char reg;

	dprintk(KERN_INFO "%s \n", __func__);

    reg = hdmi_readb(HDMI_CON_0);
    hdmi_writeb(reg & ~HDMI_SYS_ENABLE,HDMI_CON_0);
#ifdef HDMI_TX13_REV_05
    video_wrapper_enable(0);
#endif

	gHdmiStartFlag = 0;
}

/**
 * Enable/disable Blue-Screen.
 *
 * @param  enable	[in] 0 to stop sending, 1 to start sending.
 */
void hdmi_enable_bluescreen(unsigned char enable)
{
	unsigned char reg = hdmi_readb(HDMI_CON_0);

	dprintk(KERN_INFO "%s enable = %d\n", __func__, enable);
	
	if (enable)
	{
		reg |= HDMI_BLUE_SCR_ENABLE;
	}
	else
	{
		reg &= ~HDMI_BLUE_SCR_ENABLE;
	}
	hdmi_writeb(reg,HDMI_CON_0);
}

#ifdef CONFIG_PM_RUNTIME
int hdmi_runtime_suspend(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	if(gHdmiPwrInfo.status == PWR_STATUS_ON) {
		hdmi_stop();

		TCC_OUTPUT_LCDC_OnOff(TCC_OUTPUT_HDMI, tca_get_output_lcdc_num(), false);
	}
	
	gHDMISuspendStatus = 1;

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int hdmi_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	gHDMISuspendStatus = 0;

	return 0;
}

#endif

#ifdef CONFIG_PM
int hdmi_suspend(struct platform_device *dev, pm_message_t state)
{
	printk(KERN_INFO "%s  state:%d \n", __FUNCTION__, gHdmiPwrInfo.status);
	
	if(gHdmiPwrInfo.status == PWR_STATUS_ON) {
		tcc_hdmi_power_off();
		tcc_hdmi_v5_power_off();
	}

	return 0;
}

int hdmi_resume(struct platform_device *dev)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);

/*
	if(gHdmiPwrInfo.status == PWR_STATUS_OFF) {

		tcc_hdmi_v5_power_on();
		tcc_hdmi_power_on();

		gHdmiPwrInfo.status = PWR_STATUS_OFF;

	}
*/

	return 0;
}
#else
#define hdmi_suspend NULL
#define hdmi_resume NULL
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops hdmi_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend      = hdmi_runtime_suspend,
	.runtime_resume       = hdmi_runtime_resume,
#else
	.suspend	= hdmi_suspend,
	.resume = hdmi_resume,
#endif
};
#endif

#if 0
#ifdef CONFIG_OF
static void hdmi_parse_dt(struct device_node *np, struct tcc_hdmi *hdmi)
{
}
#else
static void hdmi_parse_dt(struct device_node *np, struct tcc_hdmi *hdmi)
{
}
#endif
#endif

#ifdef CONFIG_OF
static struct of_device_id hdmi_of_match[] = {
	{ .compatible = "telechips,tcc893x-hdmi" },
	{}
};
MODULE_DEVICE_TABLE(of, hdmi_of_match);
#endif


static int hdmi_probe(struct platform_device *pdev)
{
 	unsigned char reg;

	if (!pdev->dev.of_node) {
		pdev_hdmi = pdev->dev.platform_data;;
		if (!pdev_hdmi) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	pdev_hdmi = &pdev->dev;
	
	printk("%s : hdmi_lcdc_num = %d\n", __func__,tca_get_output_lcdc_num());

	if (hdmi_pclk == NULL && hdmi_hclk == NULL ) {

		hdmi_hclk = of_clk_get(pdev->dev.of_node, 1);
		hdmi_pclk = of_clk_get(pdev->dev.of_node, 0);

		if (IS_ERR(hdmi_hclk)) {
			printk(KERN_WARNING "HDMI: failed to get hdmi hclock\n");
			hdmi_hclk = NULL;
			return -ENODEV;
		}

		if (IS_ERR(hdmi_pclk)) {
			printk(KERN_WARNING "HDMI: failed to get hdmi pclock\n");
			hdmi_pclk = NULL;
			return -ENODEV;
		}
	}

	if (hdmi_pclk)
		clk_prepare_enable(hdmi_pclk);

	if (hdmi_hclk)
		clk_prepare_enable(hdmi_hclk);

    dprintk(KERN_INFO "%s\n", __FUNCTION__);

    if (!machine_is_hdmidp())
        return -ENODEV;

    printk(KERN_INFO "HDMI Driver ver. %s\n", VERSION);

    if (misc_register(&hdmi_misc_device))
    {
        dprintk(KERN_WARNING "HDMI: Couldn't register device 10, %d.\n", HDMI_MINOR);
        return -EBUSY;
    }

    // disable HDCP INT
    reg = hdmi_readb(HDMI_SS_INTC_CON);
    hdmi_writeb(reg & ~(1<<HDMI_IRQ_HDCP), HDMI_SS_INTC_CON);

	if (hdmi_pclk)
		clk_disable_unprepare(hdmi_pclk);

	if (hdmi_hclk)
		clk_disable_unprepare(hdmi_hclk);

#if defined(CONFIG_REGULATOR)
	vdd_v5p0 = regulator_get(pdev_hdmi, "vdd_v5p0");
    if (IS_ERR(vdd_v5p0)) {
        pr_warning("clock_table: failed to obtain vdd_v5p0\n");
        vdd_v5p0 = NULL;
	}

	vdd_hdmi = regulator_get(pdev_hdmi, "vdd_hdmi");
    if (IS_ERR(vdd_hdmi)) {
        pr_warning("clock_table: failed to obtain vdd_hdmi\n");
        vdd_hdmi = NULL;
	}
#endif

	#if defined(CONFIG_SWITCH_GPIO_HDMI)
	platform_device_register(&tcc_hdmi_detect_device);
	#endif//
	
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_hdmi);	
	pm_runtime_enable(pdev_hdmi);  
	pm_runtime_get_noresume(pdev_hdmi);  //increase usage_count 
#endif

	
	return 0;
}


static struct platform_driver tcc_hdmi = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver	= {
		.name	= "tcc_hdmi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &hdmi_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(hdmi_of_match),
#endif
	},
};

static __init int hdmi_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	return platform_driver_register(&tcc_hdmi);
}

static __exit void hdmi_exit(void)
{
	printk("%s \n", __FUNCTION__);

	platform_driver_unregister(&tcc_hdmi);
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("TCCxxx hdmi driver");
MODULE_LICENSE("GPL");
