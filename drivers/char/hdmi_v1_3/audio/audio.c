/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/audio/audio.c
 * Description : hdmi audio driver
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <asm/mach-types.h>


#include <asm/io.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <asm/uaccess.h>

#include "../hdmi/regs-hdmi.h"

#include <mach/hdmi_1_3_audio.h>


#define VERSION 		"1.1" /* Driver version number */
#define AUDIO_MINOR 	241 /* Major 10, Minor 241, /dev/audio */

#define AUDIO_DEBUG 0
#if AUDIO_DEBUG
#define DPRINTK(args...)    printk(args)
#else
#define DPRINTK(args...)
#endif

static struct clk *hdmi_audio_pclk;
static struct clk *hdmi_audio_hclk;

static int audio_irq;

static int audio_open(struct inode *inode, struct file *file);
static int audio_release(struct inode *inode, struct file *file);
static irqreturn_t audio_spdif_handler(int irq, void *dev_id);
static ssize_t audio_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static ssize_t audio_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static long audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

int setAudioInputPort(enum HDMIAudioPort port);
int setCUVSampleFreq(enum SamplingFreq freq);
int setCUVCodingType(enum CUVAudioCoding coding);
int setCUVChannelNum(enum CUVChannelNumber num);
int setCUVWordLength(enum CUVWordLength length);
int setI2SParameter(struct I2SParameter i2s);
int setSPDIFSampleFreq(enum SamplingFreq freq);

static const struct file_operations audio_fops =
{
    .owner          = THIS_MODULE,
    .open           = audio_open,
    .release        = audio_release,
    .read           = audio_read,
    .write          = audio_write,
    .unlocked_ioctl = audio_ioctl,
};

static struct miscdevice audio_misc_device =
{
    AUDIO_MINOR,
    "audio", //"AUDIO",
    &audio_fops,
};
//[audio]<
struct HDMIAudioParameter	gHDMIAudioParams;
static unsigned int gCodingType=0;
static unsigned int gLPCMWordLen=0;

#define SPDIF_DONE          0
#define SPDIF_ERROR         1

/**
 * @struct spdif_struct
 *
 * This struct is used in SPDIF interrupt handler.
 */
struct spdif_struct {
    /** State of SPDIF receiver */
    int state;
    /** Coding type of input audio stream */
    int codingtype;
};

static struct spdif_struct spdif_struct;

static inline u8 hdmi_readb(unsigned offset){
    return readb(IOMEM(offset));
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

static int audio_open(struct inode *inode, struct file *file)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	clk_prepare_enable(hdmi_audio_pclk);
	clk_prepare_enable(hdmi_audio_hclk);

    return 0;
}


static int audio_release(struct inode *inode, struct file *file)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	clk_disable_unprepare(hdmi_audio_hclk);
	clk_disable_unprepare(hdmi_audio_pclk);

    return 0;
}

ssize_t audio_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}

ssize_t audio_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}


long  audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);
    switch (cmd)
    {
        case AUDIO_IOC_SET_AUDIOINPUT:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_AUDIOINPUT)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setAudioInputPort(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }

			gHDMIAudioParams.inputPort = val;
			
            break;
        }

        case AUDIO_IOC_GET_AUDIOINPUT:
        {
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_AUDIOINPUT)\n");

            // put to user
            if (put_user(gHDMIAudioParams.inputPort, (int __user *) arg))
                return -EFAULT;

            break;
        }
		
        case AUDIO_IOC_SET_I2S_CUV_SET_SAMPLEFREQ:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_CUV_SET_SAMPLEFREQ)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setCUVSampleFreq(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }

			gHDMIAudioParams.sampleFreq = val;

            break;
        }
        case AUDIO_IOC_GET_I2S_CUV_SET_SAMPLEFREQ:
        {
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_I2S_CUV_SET_SAMPLEFREQ)\n");

            // put to user
            if (put_user(gHDMIAudioParams.sampleFreq, (int __user *) arg))
                return -EFAULT;

            break;
        }

        case AUDIO_IOC_SET_I2S_CUV_SET_CODINGTYPE:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_CUV_SET_CODINGTYPE)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setCUVCodingType(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }

			gCodingType = val;
			
            break;
        }

        case AUDIO_IOC_GET_I2S_CUV_SET_CODINGTYPE:
        {
//          int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_I2S_CUV_SET_CODINGTYPE)\n");

            // put to user
            if (put_user(gCodingType, (int __user *) arg))
                return -EFAULT;

            break;
        }


        case AUDIO_IOC_SET_AUDIOFORMATCODE_INFO:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_AUDIOFORMATCODE_INFO)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

			gHDMIAudioParams.formatCode = val;

            break;
        }

        case AUDIO_IOC_GET_AUDIOFORMATCODE_INFO:
        {
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_AUDIOFORMATCODE_INFO)\n");

            // put to user
            if (put_user(gHDMIAudioParams.formatCode, (int __user *) arg))
                return -EFAULT;

            break;
        }

        case AUDIO_IOC_SET_I2S_CUV_SET_CHANNELNUMBER:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_CUV_SET_CHANNELNUMBER)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setCUVChannelNum(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }

			gHDMIAudioParams.channelNum = val;
			
            break;
        }
        case AUDIO_IOC_GET_I2S_CUV_SET_CHANNELNUMBER:
        {
//          int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_I2S_CUV_SET_CHANNELNUMBER)\n");
            // put to user
            if (put_user(gHDMIAudioParams.channelNum, (int __user *) arg))
                return -EFAULT;
            break;
        }

        case AUDIO_IOC_SET_I2S_CUV_SET_WORDLENGTH:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_CUV_SET_WORDLENGTH)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setCUVWordLength(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }

            break;
        }
        case AUDIO_IOC_SET_I2S_CUV_SET_WORDLENGTH_INFO:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_CUV_SET_WORDLENGTH)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

			gLPCMWordLen = val;
			
            break;
        }
        case AUDIO_IOC_GET_I2S_CUV_SET_WORDLENGTH_INFO:
        {
//          int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_I2S_CUV_SET_WORDLENGTH)\n");

            // put to user
            if (put_user(gLPCMWordLen, (int __user *) arg))
                return -EFAULT;
			
            break;
        }

        case AUDIO_IOC_SET_SPDIF_SET_CODINGTYPE:
        {
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_SPDIF_SET_CODINGTYPE)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (val == SPDIF_NLPCM)
            {
                spdif_struct.codingtype = SPDIF_NLPCM;
            }
            else if (val == SPDIF_LPCM)
            {
                spdif_struct.codingtype = SPDIF_LPCM;
            }
            else
            {
                spdif_struct.codingtype = -1;
                return -EFAULT;
            }
            break;
        }
		case AUDIO_IOC_SET_SPDIF_SET_SAMPLEFREQ:
		{
            int val;
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_SPDIF_SET_SAMPLEFREQ)\n");
            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!setSPDIFSampleFreq(val))
            {
                DPRINTK(KERN_INFO "Not Available Arg\n");
                return -EFAULT;
            }
			break;
		}
		
        case AUDIO_IOC_SET_I2S_PARAMETER:
        {
            struct I2SParameter i2s;
            unsigned int ret;

            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_SET_I2S_PARAMETER)\n");

            // get arg
            if ( (ret = copy_from_user((void*)&i2s,(const void*)arg,sizeof(struct I2SParameter))) < 0)
            {
                return -EFAULT;
            }

            ret = setI2SParameter(i2s);

			memcpy(&gHDMIAudioParams.i2sParam, &i2s, sizeof(struct I2SParameter));

            break;
        }

        case AUDIO_IOC_GET_I2S_PARAMETER:
        {
//          struct I2SParameter i2s;
            unsigned int ret;

            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_GET_I2S_PARAMETER)\n");

            // copy to user
            if ( (ret = copy_to_user((void*)arg,(const void*)&gHDMIAudioParams.i2sParam,sizeof(gHDMIAudioParams.i2sParam))) < 0)
                return -EFAULT;

            break;
        }
		

        case AUDIO_IOC_UPDATE_I2S_CUV:
        {
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_UPDATE_I2S_CUV)\n");
            hdmi_writeb(0x01, HDMI_SS_I2S_CH_ST_CON);
            break;
        }
        case AUDIO_IOC_RESET_I2S_CUV:
        {
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_RESET_I2S_CUV)\n");

            hdmi_writeb(0x00, HDMI_SS_I2S_CH_ST_0);
            hdmi_writeb(0x00, HDMI_SS_I2S_CH_ST_1);
            hdmi_writeb(0x00, HDMI_SS_I2S_CH_ST_2);
            hdmi_writeb(0x00, HDMI_SS_I2S_CH_ST_3);
            hdmi_writeb(0x00, HDMI_SS_I2S_CH_ST_4);
            // update CUV
            hdmi_writeb(0x01, HDMI_SS_I2S_CH_ST_CON);
            break;
        }
		case AUDIO_IOC_ENABLE_I2S_CLK_CON:
		{
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_ENABLE_I2S_CLK_CON)\n");

            // disable audio
            hdmi_writeb(I2S_CLK_CON_ENABLE,HDMI_SS_I2S_CLK_CON);

            break;
		}			
		case AUDIO_IOC_DISABLE_I2S_CLK_CON:
		{
            DPRINTK(KERN_INFO "AUDIO: ioctl(AUDIO_IOC_DISABLE_I2S_CLK_CON)\n");

            // disable audio
            hdmi_writeb(I2S_CLK_CON_DISABLE,HDMI_SS_I2S_CLK_CON);
            break;
		}
        default:
            return -EINVAL;
    }
    return 0;
}

/**
 * @brief SPDIF interrupt handler.
 *
 * Handle interrupt requests from SPDIF receiver. @n
 * When it succeeds in synchronizing audio stream, handler -@n
 * starts to send audio stream to HDMI H/W;Otherwise, when fail to @n
 * synchronize audio stream, handler restarts to synchronize audio stream.
 */

irqreturn_t audio_spdif_handler(int irq, void *dev_id)
{
    unsigned char flag,status;
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    flag = hdmi_readb(HDMI_SS_INTC_FLAG);

    if (flag & (1<<HDMI_IRQ_SPDIF))
    {
        // check spdif status
        status = hdmi_readb(HDMI_SS_SPDIF_IRQ_STATUS);

#if AUDIO_DEBUG
        if (status & SPDIF_CLK_RECOVERY_FAIL_MASK)
        {
            DPRINTK(KERN_INFO "SPDIFKERN_INFO_CLK_RECOVERY_FAIL_MASK\n");
        }

        if (status & SPDIF_STATUS_RECOVERED_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_STATUS_RECOVERED_MASK\n");
        }

        if (status & SPDIF_PREAMBLE_DETECTED_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_PREAMBLE_DETECTED_MASK\n");
        }

        if (status & SPDIF_HEADER_NOT_DETECTED_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_HEADER_NOT_DETECTED_MASK\n");
        }

        if (status & SPDIF_HEADER_DETECTED_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_HEADER_DETECTED_MASK\n");
        }

        if (status & SPDIF_PAPD_NOT_DETECTED_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_PAPD_NOT_DETECTED_MASK\n");
        }

        if (status & SPDIF_ABNORMAL_PD_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_ABNORMAL_PD_MASK\n");
        }

        if (status & SPDIF_BUFFER_OVERFLOW_MASK)
        {
            DPRINTK(KERN_INFO "SPDIF_BUFFER_OVERFLOW_MASK\n");
        }
#endif
        // clear pending bit
        hdmi_writeb(status,HDMI_SS_SPDIF_IRQ_STATUS);

        if ( spdif_struct.codingtype == SPDIF_NLPCM )
        {
            // for NLPCM
            switch(status)
            {
                // normal
                case SPDIF_STATUS_RECOVERED_MASK:
                case SPDIF_HEADER_DETECTED_MASK:
                {
                    DPRINTK(KERN_INFO "Normal Procedure\n");
                    break;
                }
                case (SPDIF_HEADER_DETECTED_MASK | SPDIF_STATUS_RECOVERED_MASK):
                {
                    // mask this interrupt. because this will happen continuously even if in normal state
                    hdmi_writeb(hdmi_readb(HDMI_SS_SPDIF_IRQ_MASK) & ~(SPDIF_HEADER_DETECTED_MASK | SPDIF_STATUS_RECOVERED_MASK), HDMI_SS_SPDIF_IRQ_MASK);
                    DPRINTK(KERN_INFO "Succeed! Audio Start\n");
                    hdmi_writeb(SPDIF_RUNNING, HDMI_SS_SPDIF_OP_CTRL); // 0b11 // run
                    break;
                }
                // error state
                default :
                {
                    DPRINTK(KERN_INFO "Fail! Audio Restart\n");
                    hdmi_writeb(SPDIF_SIGNAL_RESET, HDMI_SS_SPDIF_OP_CTRL); // software reset
                    hdmi_writeb(0xFF, HDMI_SS_SPDIF_IRQ_MASK); // enable all interrupts
                    hdmi_writeb(SPDIF_SIGNAL_DETECT, HDMI_SS_SPDIF_OP_CTRL); // detect signal
                }
            }
        }
        else if ( spdif_struct.codingtype == SPDIF_LPCM )
        {
            // for LPCM
            switch(status)
            {
                // normal
                case SPDIF_STATUS_RECOVERED_MASK:
// TODO: check if it is bug or not even if SPDIF_HEADER_NOT_DETECTED_MASK occurs, it works well.
                case (SPDIF_STATUS_RECOVERED_MASK | SPDIF_HEADER_NOT_DETECTED_MASK):
                {
                    hdmi_writeb(hdmi_readb(HDMI_SS_SPDIF_IRQ_MASK) & ~(SPDIF_STATUS_RECOVERED_MASK | SPDIF_HEADER_NOT_DETECTED_MASK), HDMI_SS_SPDIF_IRQ_MASK);
                    DPRINTK(KERN_INFO "Succeed! Audio Start\n");
                    hdmi_writeb(SPDIF_RUNNING, HDMI_SS_SPDIF_OP_CTRL); // 0b11 // run
                    break;
                }
                // error state
                default :
                {
                    // initialize history
                    DPRINTK(KERN_INFO "Fail! Audio Restart\n");
                    hdmi_writeb(SPDIF_SIGNAL_RESET, HDMI_SS_SPDIF_OP_CTRL); // software reset
                    hdmi_writeb(SPDIF_BUFFER_OVERFLOW_MASK | SPDIF_PREAMBLE_DETECTED_MASK | SPDIF_STATUS_RECOVERED_MASK | SPDIF_CLK_RECOVERY_FAIL_MASK, HDMI_SS_SPDIF_IRQ_MASK); // enable all interrupts
                    hdmi_writeb(SPDIF_SIGNAL_DETECT, HDMI_SS_SPDIF_OP_CTRL); // detect signal
                }
            }
        }
        else
        {
            DPRINTK(KERN_INFO "Not Correct SPDIF CODING TYPE\n");
        }
    }
    else
    {
        return IRQ_NONE;
    }


    return IRQ_HANDLED;
}

static int audio_remove(struct platform_device *pdev)
{
    unsigned char reg;
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    // disable SPDIF INT
    reg = hdmi_readb(HDMI_SS_INTC_CON);
    hdmi_writeb(reg & ~(1<<HDMI_IRQ_SPDIF), HDMI_SS_INTC_CON);

    free_irq(audio_irq, audio_spdif_handler);
    misc_deregister(&audio_misc_device);

	return 0;
}

#ifdef CONFIG_PM
static int audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}

static int audio_resume(struct platform_device *pdev)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}
#else
#define audio_suspend NULL
#define audio_resume NULL
#endif



/**
 * Set audio input port.
 *
 * @param   port    [in]    Audio input port.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */

int setAudioInputPort(enum HDMIAudioPort port)
{

    switch (port)
    {
        case I2S_PORT:
        {

			//@STORM::090928
		    // disable SPDIF INT
			{
			    unsigned char reg;
			    reg = hdmi_readb(HDMI_SS_INTC_CON);
			    hdmi_writeb(reg & ~(1<<HDMI_IRQ_SPDIF), HDMI_SS_INTC_CON);
			}
			
            // disable DSD
            hdmi_writeb(I2S_DSD_CON_DISABLE, HDMI_SS_I2S_DSD_CON);

            // I2S control
            hdmi_writeb(I2S_CON_SC_POL_FALLING | I2S_CON_CH_POL_LOW, HDMI_SS_I2S_CON_1);

            // I2S MUX Control
            hdmi_writeb(I2S_IN_MUX_ENABLE | I2S_IN_MUX_CUV_ENABLE | I2S_IN_MUX_SELECT_I2S | I2S_IN_MUX_IN_ENABLE, HDMI_SS_I2S_IN_MUX_CON);

            // enable all channels
            hdmi_writeb(I2S_MUX_CH_ALL_ENABLE , HDMI_SS_I2S_MUX_CH);

            // enable CUV from right and left channel
            hdmi_writeb(I2S_MUX_CUV_LEFT_ENABLE| I2S_MUX_CUV_RIGHT_ENABLE , HDMI_SS_I2S_MUX_CUV);

            break;
        }
        case SPDIF_PORT:
        {
            unsigned char reg;

            // set mux control
            hdmi_writeb(I2S_IN_MUX_SELECT_SPDIF | I2S_IN_MUX_ENABLE, HDMI_SS_I2S_IN_MUX_CON);

            // enable all channels
            hdmi_writeb(I2S_MUX_CH_ALL_ENABLE , HDMI_SS_I2S_MUX_CH);

            // enable CUV from right and left channel
            hdmi_writeb(I2S_MUX_CUV_LEFT_ENABLE| I2S_MUX_CUV_RIGHT_ENABLE , HDMI_SS_I2S_MUX_CUV);

            hdmi_writeb(0, HDMI_SS_SPDIF_CLK_CTRL); // enable clock???
            hdmi_writeb(SPDIF_CLK_CTRL_ENABLE, HDMI_SS_SPDIF_CLK_CTRL);

            hdmi_writeb(SPDIF_CONFIG_1_NOISE_FILTER_3_SAMPLES | SPDIF_CONFIG_1_UVCP_ENABLE | SPDIF_CONFIG_1_WORD_LENGTH_MANUAL | SPDIF_CONFIG_1_ALIGN_32BIT | SPDIF_CONFIG_1_HDMI_2_BURST, HDMI_SS_SPDIF_CONFIG_1);

            // max 24bits
            hdmi_writeb(0x0b, HDMI_SS_SPDIF_USER_VALUE_1);

            if (spdif_struct.codingtype == SPDIF_NLPCM)
            {
                hdmi_writeb(0xFF,HDMI_SS_SPDIF_IRQ_MASK); // enable all
            }
            else
            {
                hdmi_writeb(SPDIF_BUFFER_OVERFLOW_MASK | SPDIF_PREAMBLE_DETECTED_MASK | SPDIF_STATUS_RECOVERED_MASK | SPDIF_CLK_RECOVERY_FAIL_MASK, HDMI_SS_SPDIF_IRQ_MASK);
            }
            // enable SPDIF INT
            reg = hdmi_readb(HDMI_SS_INTC_CON);
            hdmi_writeb(reg | (1<<HDMI_IRQ_SPDIF) | (1<<HDMI_IRQ_GLOBAL), HDMI_SS_INTC_CON);

            // start to detect signal
            hdmi_writeb(SPDIF_SIGNAL_DETECT, HDMI_SS_SPDIF_OP_CTRL);

            break;
        }
//TODO: need to check!!!
        case DSD_PORT:
        {
            // enable all channels
            hdmi_writeb(I2S_MUX_CH_0_LEFT_ENABLE | I2S_MUX_CH_0_RIGHT_ENABLE |
                    I2S_MUX_CH_1_LEFT_ENABLE | I2S_MUX_CH_1_RIGHT_ENABLE |
                    I2S_MUX_CH_2_LEFT_ENABLE | I2S_MUX_CH_2_RIGHT_ENABLE , HDMI_SS_I2S_MUX_CH);

            // enable CUV from right and left channel
            hdmi_writeb(~(I2S_MUX_CUV_LEFT_ENABLE| I2S_MUX_CUV_RIGHT_ENABLE) , HDMI_SS_I2S_MUX_CUV);


            // set mux control
            hdmi_writeb(I2S_IN_MUX_ENABLE | I2S_IN_MUX_SELECT_DSD | I2S_IN_MUX_IN_ENABLE, HDMI_SS_I2S_IN_MUX_CON);

            break;
        }
        default:
            return 0;
    }
    return 1;
}


#if defined(CONFIG_SOC_SND_TCC)
//extern void tcc_i2s_set_clock(unsigned int clock_rate);
//extern void tcc_spdif_set_clock(unsigned int clock_rate);

void tca_i2s_setclock(unsigned int clock_rate)
{
	BUG();
	//tcc_i2s_set_clock(clock_rate);
}

void tca_spdif_setclock(unsigned int clock_rate)
{
	BUG();
	//tcc_spdif_set_clock(clock_rate);
}
#endif//


static unsigned int io_ckc_get_dai_clock(unsigned int freq)
{
    switch (freq) {
        case 44100: 
            return (44100 * 256);
        case 22000: 
            return (22050 * 256);
        case 11000: 
            return (11025 * 256);
        default:
            break;
    }
    return (freq * 256);
}

void tcc_hdmi_audio_set_clock(unsigned int output, unsigned int clock_rate)
{
    unsigned int clk_rate;
	DPRINTK(KERN_WARNING "[%s], clock_rate[%u]\n", __func__, clock_rate);


    if( output == I2S_PORT )
    {
		clk_disable_unprepare(hdmi_audio_pclk);
		clk_set_rate(hdmi_audio_pclk, XTIN_CLK_RATE);
	    clk_rate = io_ckc_get_dai_clock(clock_rate) * 2;   /* set 512xfs for HDMI */
		tcc_ckc_set_hdmi_audio_src(PERI_ADAI0); 
	    clk_set_rate(hdmi_audio_pclk, clk_rate);
		clk_prepare_enable(hdmi_audio_pclk);
    }
    else if( output == SPDIF_PORT )
    {
		//To Do : SPDIF Clock Setting...
		clk_disable_unprepare(hdmi_audio_pclk);
		clk_set_rate(hdmi_audio_pclk, XTIN_CLK_RATE);
	    clk_rate = io_ckc_get_dai_clock(clock_rate) * 2;   /* set 512xfs for HDMI */
		tcc_ckc_set_hdmi_audio_src(PERI_SPDIF1); 
	    clk_set_rate(hdmi_audio_pclk, clk_rate);
		clk_prepare_enable(hdmi_audio_pclk);
    }
}

void setHDMIAudioClock(unsigned int output, unsigned int freq)
{
		tcc_hdmi_audio_set_clock(output, freq);
	}



/**
 * Set sampling frequency in I2S receiver.
 *
 * @param   freq    [in]   Sampling frequency.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setCUVSampleFreq(enum SamplingFreq freq)
{
    int ret = 1;
    unsigned char reg;
	unsigned int  audio_freq;
	
    DPRINTK(KERN_INFO "%s freq:%d \n", __FUNCTION__, freq);

	reg = hdmi_readb(HDMI_SS_I2S_CH_ST_3) & ~I2S_CH_ST_3_SF_MASK;

    switch (freq)
    {
        case SF_32KHZ:
            reg |= I2S_CH_ST_3_SF_32KHZ;
			audio_freq = 32000;
            break;
        case SF_44KHZ:
            reg |= I2S_CH_ST_3_SF_44KHZ;
			audio_freq = 44100;
            break;
        case SF_88KHZ:
            reg |= I2S_CH_ST_3_SF_88KHZ;
			audio_freq = 88200;
            break;
        case SF_176KHZ:
            reg |= I2S_CH_ST_3_SF_176KHZ;
			audio_freq = 176000;
            break;
        case SF_48KHZ:
            reg |= I2S_CH_ST_3_SF_48KHZ;
			audio_freq = 48000;
            break;
        case SF_96KHZ:
            reg |= I2S_CH_ST_3_SF_96KHZ;
			audio_freq = 96000;
            break;
        case SF_192KHZ:
            reg |= I2S_CH_ST_3_SF_192KHZ;
			audio_freq = 192000;
            break;

        default:
            ret = 0;
    }

	if( ret )
	{
		hdmi_writeb(reg, HDMI_SS_I2S_CH_ST_3);
		setHDMIAudioClock(I2S_PORT, audio_freq);
	}

    return ret;
}

/**
 * Set sampling frequency in SPDIF receiver.
 *
 * @param   freq    [in]   Sampling frequency.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setSPDIFSampleFreq(enum SamplingFreq freq)
{
    int ret = 1;
	unsigned int audio_freq;
	
    DPRINTK(KERN_INFO "%s freq:%d \n", __FUNCTION__, freq);

    switch (freq)
    {
        case SF_32KHZ:
			audio_freq = 32000;
            break;
        case SF_44KHZ:
			audio_freq = 44100;
            break;
        case SF_88KHZ:
			audio_freq = 88200;
            break;
        case SF_176KHZ:
			audio_freq = 176000;
            break;
        case SF_48KHZ:
			audio_freq = 48000;
            break;
        case SF_96KHZ:
			audio_freq = 96000;
            break;
        case SF_192KHZ:
			audio_freq = 192000;
            break;

        default:
            ret = 0;
    }

	if( ret )
		setHDMIAudioClock(SPDIF_PORT, audio_freq);

    return ret;
}

/**
 * Set coding type in I2S receiver.
 *
 * @param   coding    [in]   Coding type.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setCUVCodingType(enum CUVAudioCoding coding)
{
    int ret = 1;
    unsigned char reg = hdmi_readb(HDMI_SS_I2S_CH_ST_0) & ~I2S_CH_ST_0_TYPE_MASK;

    switch (coding)
    {
        case CUV_LPCM:
            reg |= I2S_CH_ST_0_TYPE_LPCM;
            break;

        case CUV_NLPCM:
            reg |= I2S_CH_ST_0_TYPE_NLPCM;
            break;

        default:
            ret = 0;
    };

    hdmi_writeb(reg, HDMI_SS_I2S_CH_ST_0);

    return ret;
}

/**
 * Set the number of channels in I2S receiver.
 *
 * @param   num     [in]   Number of channels.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setCUVChannelNum(enum CUVChannelNumber num)
{
    int ret = 1;
    unsigned char reg = hdmi_readb(HDMI_SS_I2S_CH_ST_2) & ~I2S_CH_ST_2_CHANNEL_MASK;

    switch (num)
    {
        case CUV_CH_UNDEFINED:
            reg |= I2S_CH_ST_2_CH_UNDEFINED;
            break;

        case CUV_CH_01:
            reg |= I2S_CH_ST_2_CH_01;
            break;

        case CUV_CH_02:
            reg |= I2S_CH_ST_2_CH_02;
            break;

        case CUV_CH_03:
            reg |= I2S_CH_ST_2_CH_03;
            break;

        case CUV_CH_04:
            reg |= I2S_CH_ST_2_CH_04;
            break;

        case CUV_CH_05:
            reg |= I2S_CH_ST_2_CH_05;
            break;

        case CUV_CH_06:
            reg |= I2S_CH_ST_2_CH_06;
            break;

        case CUV_CH_07:
            reg |= I2S_CH_ST_2_CH_07;
            break;

        case CUV_CH_08:
            reg |= I2S_CH_ST_2_CH_08;
            break;

        case CUV_CH_09:
            reg |= I2S_CH_ST_2_CH_09;
            break;

        case CUV_CH_10:
            reg |= I2S_CH_ST_2_CH_10;
            break;

        case CUV_CH_11:
            reg |= I2S_CH_ST_2_CH_11;
            break;

        case CUV_CH_12:
            reg |= I2S_CH_ST_2_CH_12;
            break;

        case CUV_CH_13:
            reg |= I2S_CH_ST_2_CH_13;
            break;

        case CUV_CH_14:
            reg |= I2S_CH_ST_2_CH_14;
            break;

        case CUV_CH_15:
            reg |= I2S_CH_ST_2_CH_15;
            break;

        default:
            ret = 0;
    }

    hdmi_writeb(reg, HDMI_SS_I2S_CH_ST_2);

    return ret;
}

/**
 * Set word length in I2S receiver.
 *
 * @param   length    [in]   Word length.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setCUVWordLength(enum CUVWordLength length)
{
    int ret = 1;
    unsigned char reg = hdmi_readb(HDMI_SS_I2S_CH_ST_4) & ~I2S_CH_ST_4_WL_MASK;

    switch (length)
    {
        case CUV_WL_20_NOT_DEFINED:
            reg |= I2S_CH_ST_4_WL_20_NOT_DEFINED;
            break;

        case CUV_WL_20_16:
            reg |= I2S_CH_ST_4_WL_20_16;
            break;

        case CUV_WL_20_18:
            reg |= I2S_CH_ST_4_WL_20_18;
            break;

        case CUV_WL_20_19:
            reg |= I2S_CH_ST_4_WL_20_19;
            break;

        case CUV_WL_20_20:
            reg |= I2S_CH_ST_4_WL_20_20;
            break;

        case CUV_WL_20_17:
            reg |= I2S_CH_ST_4_WL_20_17;
            break;

        case CUV_WL_24_NOT_DEFINED:
            reg |= I2S_CH_ST_4_WL_24_NOT_DEFINED;
            break;

        case CUV_WL_24_20:
            reg |= I2S_CH_ST_4_WL_24_20;
            break;

        case CUV_WL_24_22:
            reg |= I2S_CH_ST_4_WL_24_22;
            break;

        case CUV_WL_24_23:
            reg |= I2S_CH_ST_4_WL_24_23;
            break;

        case CUV_WL_24_24:
            reg |= I2S_CH_ST_4_WL_24_24;
            break;

        case CUV_WL_24_21:
            reg |= I2S_CH_ST_4_WL_24_21;
            break;

        default:
            ret = 0;
    }
    hdmi_writeb(reg, HDMI_SS_I2S_CH_ST_4);

    return ret;
}

/**
 * Set I2S audio paramters in I2S receiver.
 *
 * @param   i2s     [in]   I2S audio paramters.
 * @return  If argument is invalid, return 0;Otherwise, return 1.
 */
int setI2SParameter(struct I2SParameter i2s)
{
    unsigned char reg;
    // bit per channel
    switch(i2s.bpc)
    {
        case I2S_BPC_16:
            reg = I2S_CON_DATA_NUM_16;
            break;

        case I2S_BPC_20:
            reg = I2S_CON_DATA_NUM_20;
            break;

        case I2S_BPC_24:
            reg = I2S_CON_DATA_NUM_24;
            break;
        default:
            return 0;
    }

    // LR clock
    switch(i2s.clk)
    {
        case I2S_32FS:
            reg = (I2S_CON_BIT_CH_32 | I2S_CON_DATA_NUM_16);
            break;

        case I2S_48FS:
            reg |= I2S_CON_BIT_CH_48;
            break;

        case I2S_64FS:
            reg |= I2S_CON_BIT_CH_64;
            break;
        default:
            return 0;
    }

    // format
    switch(i2s.format)
    {
        case I2S_BASIC:
            reg |= I2S_CON_I2S_MODE_BASIC;
            break;

        case I2S_LEFT_JUSTIFIED:
            reg |= I2S_CON_I2S_MODE_LEFT_JUSTIFIED;
            break;

        case I2S_RIGHT_JUSTIFIED:
            reg |= I2S_CON_I2S_MODE_RIGHT_JUSTIFIED;
            break;
        default:
            return -1;
    }
    hdmi_writeb(reg , HDMI_SS_I2S_CON_2);

    return 1;

}

static int audio_probe(struct platform_device *pdev)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);


	if (hdmi_audio_pclk == NULL && hdmi_audio_hclk == NULL ) {

		hdmi_audio_hclk = of_clk_get(pdev->dev.of_node, 1);
		hdmi_audio_pclk = of_clk_get(pdev->dev.of_node, 0);

		if (IS_ERR(hdmi_audio_hclk)) {
			printk(KERN_WARNING "HDMI: failed to get hpd hclock\n");
			hdmi_audio_hclk = NULL;
			return -ENODEV;
		}

		if (IS_ERR(hdmi_audio_pclk)) {
			printk(KERN_WARNING "HDMI: failed to get hpd pclock\n");
			hdmi_audio_pclk = NULL;
			return -ENODEV;
		}
	}

	if(hdmi_audio_pclk)
		clk_prepare_enable(hdmi_audio_pclk);

	if(hdmi_audio_hclk)
		clk_prepare_enable(hdmi_audio_hclk);

    if (!machine_is_hdmidp())
        return -ENODEV;

    printk(KERN_INFO "HDMI Audio Driver ver. %s\n", VERSION);

	audio_irq = platform_get_irq(pdev, 0);
	if (audio_irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
        return -EIO;
	}


    if (misc_register(&audio_misc_device))
    {
        DPRINTK(KERN_WARNING "HDMI: Couldn't register device 10, %d.\n", AUDIO_MINOR);
        return -EBUSY;
    }

    spdif_struct.state = -1;
    spdif_struct.codingtype = -1;

    if (request_irq(audio_irq, audio_spdif_handler, IRQF_SHARED, "spdif", audio_spdif_handler))
    {
        DPRINTK(KERN_WARNING "HDMI: IRQ %d is not free.\n", audio_irq);
        misc_deregister(&audio_misc_device);
        return -EIO;
    }
	
	if(hdmi_audio_pclk)
		clk_disable_unprepare(hdmi_audio_pclk);

	if(hdmi_audio_hclk)
		clk_disable_unprepare(hdmi_audio_hclk);

    return 0;
}

#if 0
#ifdef CONFIG_OF
static void audio_parse_dt(struct device_node *np, struct tcc_hdmi_audio *hpd)
{
}
#else
static void audio_parse_dt(struct device_node *np, struct tcc_hdmi_audio *hpd)
{
}
#endif
#endif

#ifdef CONFIG_OF
static struct of_device_id audio_of_match[] = {
	{ .compatible = "telechips,tcc893x-hdmi-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, audio_of_match);
#endif

static struct platform_driver tcc_hdmi_audio = {
	.probe	= audio_probe,
	.remove	= audio_remove,
	.suspend = audio_suspend,
	.resume = audio_resume,
	.driver	= {
		.name	= "tcc_hdmi_audio",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(audio_of_match),
#endif
	},
};

static __init int audio_init(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return platform_driver_register(&tcc_hdmi_audio);
}

static __exit void audio_exit(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	platform_driver_unregister(&tcc_hdmi_audio);
}

module_init(audio_init);
module_exit(audio_exit);
MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("TCCxxx hdmi audio driver");
MODULE_LICENSE("GPL");

