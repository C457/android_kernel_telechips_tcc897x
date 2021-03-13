/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_misc.c
*  \brief       HDMI TX controller driver
*  \details   
*  \version     1.0
*  \date        2014-2015
*  \copyright
This source code contains confidential information of Telechips.
Any unauthorized use without a written  permission  of Telechips including not 
limited to re-distribution in source  or binary  form  is strictly prohibited.
This source  code is  provided "AS IS"and nothing contained in this source 
code  shall  constitute any express  or implied warranty of any kind, including
without limitation, any warranty of merchantability, fitness for a   particular 
purpose or non-infringement  of  any  patent,  copyright  or  other third party 
intellectual property right. No warranty is made, express or implied, regarding 
the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability 
arising from, out of or in connection with this source  code or the  use in the 
source code. 
This source code is provided subject  to the  terms of a Mutual  Non-Disclosure 
Agreement between Telechips and Company.
*******************************************************************************/

#include "include/hdmi_includes.h"
#include <linux/clk.h>  // clk (example clk_set_rate)
#include <asm/bitops.h> // bit macros
#include <linux/regulator/consumer.h> // regulator_enable
#include <video/tcc/vioc_outcfg.h> // VIOC_OUTCFG_SetOutConfig

#include "include/irq_handlers.h"
#include "include/hdmi_ioctls.h"
#include "include/hdmi_access.h"
#include "include/video_params.h"


#include "hdmi_api_lib/include/bsp/i2cm.h"
#include "hdmi_api_lib/include/core/main_controller.h"
#include "hdmi_api_lib/include/phy/phy.h"
#include "hdmi_api_lib/include/core/video.h"
#include "hdmi_api_lib/include/core/audio.h"
#include "hdmi_api_lib/include/bsp/i2cm.h"
#include "hdmi_api_lib/include/core/irq.h"
#include "hdmi_api_lib/include/api/api.h"
#include "hdmi_api_lib/include/scdc/scrambling.h"
#include "hdmi_api_lib/include/scdc/scdc.h"

//#define HDMI_USE_PERICLK `

extern int dwc_hdmi_phy_config(struct hdmi_tx_dev *dev, unsigned int freq_hz, unsigned int pixel);
extern void hdmi_phy_gen2_tx_power_on(struct hdmi_tx_dev *dev, u8 bit);
        


void dwc_hdmi_hw_reset(struct hdmi_tx_dev *dev, int reset_on) {
        volatile void __iomem   *ddibus_io = NULL;
        
        volatile unsigned long bits;

        if(reset_on) {

                hdmi_dev_write(dev, 0x4005 << 2, 0);
                
                // RESET1-HDMICTRL SWRESET PHY
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x10);             
                bits = ioread32(ddibus_io);
                BITSET(bits, Hw1);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("HDMICTRL (set Hw1) = 0x%x\r\n",  ioread32(ddibus_io));

                hdmi_phy_gen2_tx_power_on(dev, 0); 

                // RESET4-HDMI LINK RESET
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x10);  
                bits = ioread32(ddibus_io);
                BITSET(bits, Hw0);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io+0x10 = 0x%x\r\n",  ioread32(ddibus_io));

                // RESET5-DDIBUS SWRESET HDMI
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x04);
                bits = ioread32(ddibus_io);
                BITCLR(bits, Hw2);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io+0x04 = 0x%x\r\n",  ioread32(ddibus_io));
                
        }
        else {
                // RESET2-DDIBUS SWRESET HDMI
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x04);
                bits = ioread32(ddibus_io);
                BITSET(bits, Hw2);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io+0x04 = 0x%x\r\n",  ioread32(ddibus_io));
                
                 // RESET3-HDMICTRL SWRESET LINK
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x10);
                bits = ioread32(ddibus_io);
                BITCLR(bits, Hw0);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("HDMICTRL (clear Hw0) = 0x%x\r\n",  ioread32(ddibus_io));

                hdmi_phy_gen2_tx_power_on(dev, 1); 
                
                // HDMICTRL SWRESET PHY
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x10);
                bits = ioread32(ddibus_io);
                BITCLR(bits, Hw1);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io+0x10 = 0x%x\r\n",  ioread32(ddibus_io));

                
                hdmi_dev_write(dev, 0x4005 << 2, 1);
        }
}

void dwc_hdmi_core_power_on(struct hdmi_tx_dev *dev)
{
        
        volatile void __iomem   *ddibus_io = NULL;
        volatile unsigned long bits;
        #if defined(CONFIG_REGULATOR)
        int ret;
        #endif

        #if defined(CONFIG_REGULATOR)
        if(dev->regulator) {
                ret = regulator_enable(dev->regulator);
                if (ret)
                        pr_err("failed to enable hdmi regulator: %d\n", ret);
        }
        #endif

        if(++dev->clock_enable_count == 1) {
                if(dev->verbose >= VERBOSE_IO)
                        printk("dwc_hdmi_core_power_on\r\n");
                
                // Select REF - PHY 24MHz
                bits = ioread32(dev->ddibus_io+0x10); 
                #if defined(HDMI_USE_PERICLK)
                BITCSET(bits, Hw13|Hw12|Hw11, Hw13|Hw12);       // PHY 24M
                #else
                BITCSET(bits, Hw13|Hw12|Hw11, 0);       // PAD-XIN
                #endif
                iowrite32(bits, dev->ddibus_io+0x10);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io(0x%x) + 0x10 = 0x%x\r\n", (u32)dev->ddibus_io+0x10, (u32)bits);
                
                #if defined(HDMI_USE_PERICLK)
                // HDMI_CLK_INDEX_PHY27M - 24MHz
                clk_set_rate(dev->clk[HDMI_CLK_INDEX_PHY27M], HDMI_PHY_REF_CLK_RATE);
                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_PHY27M]);
                if(dev->verbose >= VERBOSE_IO)
                                pr_info("%s: HDMI PHY24M is %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_PHY27M]));
                #endif
                
                // PCLK is set to 50MHz
                clk_set_rate(dev->clk[HDMI_CLK_INDEX_PCLK], dev->clk_freq_pclk);
                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_PCLK]);
                if(dev->verbose >= VERBOSE_IO)
				pr_info("%s: HDMI PCLK is %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_PCLK]));

				clk_set_rate(dev->clk[HDMI_CLK_INDEX_SPDIF], HDMI_SPDIF_REF_CLK_RATE);
				clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_SPDIF]);
                if(dev->verbose >= VERBOSE_IO)
                                pr_info("%s: HDMI SPDIF is %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_SPDIF]));	
				
                // HPD14 is set to 50MHz
                clk_set_rate(dev->clk[HDMI_CLK_INDEX_HDCP14], HDMI_HDCP14_CLK_RATE);
                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_HDCP14]);
                if(dev->verbose >= VERBOSE_IO)
                        pr_info("%s: HDMI HDCP14 is %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_HDCP14]));
                
                // HPCP22 is set to 50MHz
                clk_set_rate(dev->clk[HDMI_CLK_INDEX_HDCP22], HDMI_HDCP22_CLK_RATE);
                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_HDCP22]);
                if(dev->verbose >= VERBOSE_IO)
                        pr_info("%s: HDMI HDCP22 is %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_HDCP22]));

                // Enable PRNG
                bits = ioread32(dev->ddibus_io+0x10); 
                BITSET(bits, Hw14);       // PRNG
                iowrite32(bits, dev->ddibus_io+0x10);

                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_PHY]);
                clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_DDIBUS]);

                // ENABLE HDMI LINK
                ddibus_io = (volatile void __iomem   *)(dev->ddibus_io + 0x10);
                bits = ioread32(ddibus_io);
                BITSET(bits, Hw15);
                iowrite32(bits, ddibus_io);
                if(dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io+0x10 = 0x%x\r\n",  ioread32(ddibus_io));
                
                dwc_hdmi_hw_reset(dev, 0);
        }
        if(dev->verbose >= VERBOSE_IO)
                printk("dwc_hdmi_core_power_on : dev->clock_enable_count=%d\r\n", dev->clock_enable_count);
}

void dwc_hdmi_core_power_off(struct hdmi_tx_dev *dev)
{        
        if(--dev->clock_enable_count == 0) {
                if(dev->verbose >= VERBOSE_IO)
                        printk("dwc_hdmi_core_power_off\r\n");
                // RESET HDMI COMPONENTS
                dwc_hdmi_hw_reset(dev, 1);

                #if defined(HDMI_USE_PERICLK)
                clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_PHY27M]);
                #endif
                
                clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_PCLK]);

                clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_HDCP14]);

                clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_DDIBUS]);

                clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_PHY]);

				clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_SPDIF]);

        }
        if(dev->verbose >= VERBOSE_IO)
                printk("dwc_hdmi_core_power_off : dev->clock_enable_count=%d\r\n", dev->clock_enable_count);
}


static long
dwc_hdmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
        long ret = 0;
        struct hdmi_tx_dev *dev = (struct hdmi_tx_dev *)file->private_data;        
        
	if(dev == NULL){
		pr_err("%s:hdmi_tx_dev device is NULL\n", FUNC_NAME);
		return -EINVAL;
	}

	switch(cmd){
	#if defined(DEBUG_HDMI_LINK)
	case FB_HDMI_CORE_READ:
                {
                        dwc_hdmi_ioctl_data hdmi_data;
        		ret = copy_from_user(&hdmi_data, (void __user *)arg,
        				sizeof(dwc_hdmi_ioctl_data));
        		hdmi_data.value = hdmi_dev_read(dev, hdmi_data.address);

        		ret = copy_to_user((void __user *)arg, &hdmi_data,
        				sizeof(dwc_hdmi_ioctl_data));
                        if (ret) {
                                if(dev->verbose >= VERBOSE_IO)
                                        pr_info("%s:READ:  reg 0x%08x [-EIO]\n",
                                                        FUNC_NAME, hdmi_data.address);
                                return ret;
                        }
        		//if (dev->verbose >= VERBOSE_IO)
        			//pr_info("%s:READ:  reg 0x%08x - value 0x%08x\n",
        				//FUNC_NAME, hdmi_data.address, hdmi_data.value);
	        }
		break;

	case FB_HDMI_CORE_WRITE:
                {
                        dwc_hdmi_ioctl_data hdmi_data;
        		ret = copy_from_user(&hdmi_data, (void __user *)arg,
        						sizeof(dwc_hdmi_ioctl_data));
        		ret = hdmi_dev_write(dev, hdmi_data.address, hdmi_data.value);
        		if (ret) {
        			if(dev->verbose >= VERBOSE_IO)
        				pr_info("%s:WRITE: reg 0x%08x [-EIO]\n",
        						FUNC_NAME, hdmi_data.address);
        			return ret;
        		}
        		ret = copy_to_user((void __user *)arg, &hdmi_data,
        						sizeof(dwc_hdmi_ioctl_data));

        		//if (dev->verbose >= VERBOSE_IO)
        			//pr_info("%s:WRITE: reg 0x%08x - value 0x%08x\n",
        				//FUNC_NAME, hdmi_data.address, hdmi_data.value);
	        }
		break;
	#endif

        
        case FB_HDMI_GET_HDCP22:
                ret = copy_to_user((void __user *)arg, &dev->hdcp22, sizeof(dev->hdcp22));
                break;

                
	case HDMI_GET_DTD_INFO:
                {
                        dwc_hdmi_dtd_data dtd_param;
                        ret = copy_from_user(&dtd_param, (void __user *)arg, sizeof(dtd_param));
                        if(ret) {
                                printk("HDMI_GET_DTD_INFO copy_from_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }
                        ret = hdmi_dtd_fill(&dtd_param.dtd, dtd_param.code, dtd_param.refreshRate);
                        if(ret) {
                                printk("invalid code (%d) and (%d)hz (ret=%d)\r\n", dtd_param.code, dtd_param.refreshRate, (int)ret);
                                return ret;
                        }
                        ret = copy_to_user((void __user *)arg, &dtd_param, sizeof(dtd_param));

                }
                break;

        
        case HDMI_PHY_SET_CONFIG:
                {
                        dwc_hdmi_phy_config_data config_data;
                        ret = copy_from_user(&config_data, (void __user *)arg, sizeof(config_data));
                        ret = dwc_hdmi_phy_config(dev, config_data.freq_hz, config_data.pixel);
                }
                break;

        // DDC
        case HDMI_DDC_SET_CLK_CONFIG:
                {
                        dwc_hdmi_ddc_config_data config_data;
                        ret = copy_from_user(&config_data, (void __user *)arg, sizeof(config_data));
                        hdmi_i2cddc_fast_mode(dev, 0);
                        hdmi_i2cddc_clk_config(dev, config_data.sfrClock, config_data.ss_low_ckl, config_data.ss_high_ckl, config_data.fs_low_ckl, config_data.fs_high_ckl);
                        ret = 0;
                }
                break;
                
        case HDMI_DDC_WRITE_DATA:
                {
                        u8 __user *data_ptrs;
                        dwc_hdmi_ddc_transfer_data transfer_data;
                        ret = copy_from_user(&transfer_data, (void __user *)arg, sizeof(transfer_data));
                        data_ptrs = (u8 __user *)transfer_data.data;
                        transfer_data.data = memdup_user(data_ptrs, transfer_data.len);
                        ret = hdmi_ddc_write(dev, transfer_data.i2cAddr, transfer_data.addr, transfer_data.len, data_ptrs);
                }
                break;
                
        case HDMI_DDC_READ_DATA:
                {
                        u8 __user *data_ptrs;                    
                        dwc_hdmi_ddc_transfer_data transfer_data;
                        ret = copy_from_user(&transfer_data, (void __user *)arg, sizeof(transfer_data));
                        data_ptrs = (u8 __user *)transfer_data.data;
                        transfer_data.data = memdup_user(data_ptrs, transfer_data.len);
                        if (IS_ERR(transfer_data.data)) {
                                ret = PTR_ERR(transfer_data.data);
                                printk("HDMI_DDC_READ_DATA memdup_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }
                        ret = hdmi_ddc_read(dev, transfer_data.i2cAddr, transfer_data.segment, transfer_data.pointer, transfer_data.addr, transfer_data.len, transfer_data.data);
                        ret = copy_to_user(data_ptrs, transfer_data.data, transfer_data.len);
                }
                break;

        case HDMI_VIDEO_CONFIG:
                {
                        videoParams_t videoParam;
                        ret = copy_from_user(&videoParam, (void __user *)arg, sizeof(videoParams_t));
                        if(ret) {
                                printk("HDMI_VIDEO_CONFIG copy_from_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }
                        // Need Validate Check.!!
                        memcpy(dev->videoParam, &videoParam, sizeof(videoParam));
                                
                        video_Configure(dev, (videoParams_t*)dev->videoParam);
                }
                break;
                
        case HDMI_VIDEO_SET_SCRAMBLING:
                {
                        int enable;
                        ret = copy_from_user(&enable, (void __user *)arg, sizeof(int));
                        scrambling(dev, (uint8_t)enable);
                }
                break;
                
        case HDMI_VIDEO_SET_TMDS_CLOCK_RATIO:
                {
                        int tmds_clock_ratio_enable;
                        ret = copy_from_user(&tmds_clock_ratio_enable, (void __user *)arg, sizeof(int));
                        ret = scdc_tmds_bit_clock_ratio_enable_flag(dev, tmds_clock_ratio_enable);
                }
                break;

        case HDMI_AUDIO_INIT:
		ret = audio_Initialize(dev);
		printk("HDMI Audio Init!!!\n");
		break;

	case HDMI_AUDIO_CONFIG:
		{
                        audioParams_t	audioParam;
                        ret = copy_from_user(&audioParam, (void __user *)arg, sizeof(audioParams_t));
                        if(ret) {
                                printk("HDMI_AUDIO_CONFIG copy_from_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }
                        memcpy(dev->audioParam, &audioParam, sizeof(audioParam));
                        ret = audio_Configure(dev, &audioParam);

		}
		break;

        case HDMI_HPD_SET_ENABLE:
                {
                        int hdp_enable;
                        ret = copy_from_user(&hdp_enable, (void __user *)arg, sizeof(hdp_enable));
                        if(ret) {
                                printk("HDMI_HPD_SET_ENABLE copy_from_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }

                        if(dev->hpd_enable != hdp_enable) {
                                dev->hpd_enable = hdp_enable;
                                hdmi_hpd_enable(dev);
                        }
                }
                break;
                
        case HDMI_HPD_GET_STATUS:
                ret = copy_to_user((void __user *)arg, &dev->hpd, sizeof(int));
                break;
                
        case HDMI_PRODUCT_SET_VPAYLOAD_DATA:
                {
                        productParams_t productParam;
                        ret = copy_from_user(&productParam, (void __user *)arg, sizeof(productParam));
                        memcpy(dev->productParam, &productParam, sizeof(productParam));
                }
                break;
                
        case HDMI_PRODUCT_SET_VPAYLOAD_LENGTH:
                {
                        ret = copy_to_user((void __user *)arg, dev->productParam, sizeof(productParams_t));
                }
                break;

        case HDMI_SET_AV_MUTE:
                {       
                        
                        void fc_force_output(struct hdmi_tx_dev *dev, int enable);
                        unsigned int magic;
                        ret = copy_from_user(&magic, (void __user *)arg, sizeof(unsigned int));

                        if((magic & 0xFFFFFFF0) == 0xF0F0A0A0) {
                                hdmi_api_avmute(dev, (magic & 1));

                        }
                }
                break;
                
        case HDMI_API_CONFIG:
                {
                        dwc_hdmi_api_data api_data;
                        ret = copy_from_user(&api_data, (void __user *)arg, sizeof(dwc_hdmi_api_data));
                        if(ret) {
                                printk("HDMI_API_CONFIG copy_from_user failed (ret=%d)\r\n", (int)ret);
                                return ret;
                        }
                        // Need Validate Check.!!
                        memcpy(dev->videoParam, &api_data.videoParam, sizeof(videoParams_t));
                        memcpy(dev->audioParam, &api_data.audioParam, sizeof(audioParams_t));
                        memcpy(dev->productParam, &api_data.productParam, sizeof(productParams_t));

                        ret = hdmi_api_Configure(dev, (videoParams_t*)dev->videoParam, (audioParams_t*)dev->audioParam, (productParams_t*)dev->productParam, &api_data.hdcpParam);
                }
                break;

	default:
//		if (dev->verbose >= VERBOSE_IO)
                        pr_err("%s:IOCTL (0x%x) is unknown!\n", FUNC_NAME, cmd);
		break;
	}
	return 0;
}

static int
dwc_hdmi_open(struct inode *inode, struct file *file) {

        struct miscdevice *misc = (struct miscdevice *)file->private_data;
        struct hdmi_tx_dev *dev = dev_get_drvdata(misc->parent);
        

        file->private_data = dev;        


        dev->open_cs++;
        
        dwc_hdmi_core_power_on(dev);
        
	return 0;
}

static int
dwc_hdmi_release(struct inode *inode, struct file *file){
        struct hdmi_tx_dev *dev = (struct hdmi_tx_dev *)file->private_data;   
        if(dev->verbose)
                pr_info("%s: read function\n", FUNC_NAME);

        dev->open_cs--;

        dwc_hdmi_core_power_off(dev);
        
	return 0;
}

static ssize_t
dwc_hdmi_read( struct file *file, char *buf, size_t count, loff_t *f_pos ){
        struct hdmi_tx_dev *dev = (struct hdmi_tx_dev *)file->private_data;   
	if(dev->verbose)
		pr_info("%s: read function\n", FUNC_NAME);
	return 0;
}

static ssize_t
dwc_hdmi_write( struct file *file, const char *buf, size_t count, loff_t *f_pos ){
        struct hdmi_tx_dev *dev = (struct hdmi_tx_dev *)file->private_data;   
	if(dev->verbose)
		pr_info("%s: write function\n", FUNC_NAME);
	return count;
}

static unsigned int 
dwc_hdmi_poll(struct file *file, poll_table *wait){
        unsigned int mask = 0;
        struct hdmi_tx_dev *dev = (struct hdmi_tx_dev *)file->private_data;   
        if(dev->verbose)
                pr_info("%s: poll function\n", FUNC_NAME);
        return mask;
}


static const struct file_operations dwc_hdmi_fops =
{
        .owner          = THIS_MODULE,
        .open           = dwc_hdmi_open,
        .release        = dwc_hdmi_release,
        .read           = dwc_hdmi_read,
        .write          = dwc_hdmi_write,
        .unlocked_ioctl = dwc_hdmi_ioctl,
        .poll           = dwc_hdmi_poll,
};


/**
 * @short misc register routine
 * @param[in] dev pointer to the hdmi_tx_devstructure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
int 
dwc_hdmi_misc_register(struct hdmi_tx_dev *dev) {
        int ret = 0;
        
        dev->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
        dev->misc->minor = MISC_DYNAMIC_MINOR;
        dev->misc->name = "dw-hdmi-tx";
        dev->misc->fops = &dwc_hdmi_fops;
        dev->misc->parent = dev->parent_dev;
        ret = misc_register(dev->misc);
        if(ret) {
                goto end_process;
        }
        dev_set_drvdata(dev->parent_dev, dev);

end_process:
        return ret;
}

/**
 * @short misc deregister routine
 * @param[in] dev pointer to the hdmi_tx_devstructure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
int 
dwc_hdmi_misc_deregister(struct hdmi_tx_dev *dev) {
        if(dev->misc) {
                misc_deregister(dev->misc);
                kfree(dev->misc);
                dev->misc = 0;
        }

        return 0;
}

