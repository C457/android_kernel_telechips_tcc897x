/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_phy.c
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

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h> // of_iomap
#include <linux/poll.h> // poll_table
#include <linux/device.h> // dev_xet_drv_data
#include <asm/io.h>
#include <linux/clk.h>  // clk (example clk_set_rate)
#include <asm/bitops.h> // bit macros
#include <linux/regulator/consumer.h> // regulator_enable
#include <video/tcc/vioc_outcfg.h> // VIOC_OUTCFG_SetOutConfig

#include <asm/bitops.h> // bit macros


#include <video/tcc/gpio.h>
#include <video/tcc/tcc_types.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_composite_ioctl.h>
#include <video/tcc/tcc_component_ioctl.h>
#include <video/tcc/tcc_scaler_ctrl.h>


// HDMI Header
#include "hdmi_v2_0/include/hdmi_includes.h"
#include "hdmi_v2_0/include/hdmi_ioctls.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/main_controller.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/irq.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/fc_video.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/product.h"
#include "hdmi_v2_0/include/hdmi_access.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/fc_debug.h"
#include "hdmi_v2_0/hdmi_api_lib/include/core/audio.h"
#include "hdmi_v2_0/hdmi_api_lib/include/api/api.h"
#include "hdmi_v2_0/include/video_params.h"
#include "hdmi_v2_0/hdmi_api_lib/include/scdc/scdc.h"
#include "hdmi_v2_0/hdmi_api_lib/include/scdc/scrambling.h"


// HDMI_V2_0 API
extern struct hdmi_tx_dev *dwc_hdmi_api_get_dev(void);
extern int dwc_hdmi_get_video_dtd(dtd_t *hdmi_dtd, uint32_t code, uint32_t hz);

// HDMI_V2 MISC
extern void dwc_hdmi_core_power_on(struct hdmi_tx_dev *dev);
extern void dwc_hdmi_hw_reset(struct hdmi_tx_dev *dev, int reset_on);

// HDMI PHY
extern int dwc_hdmi_phy_config(struct hdmi_tx_dev *dev, unsigned int freq_hz, unsigned int pixel);
extern void hdmi_phy_gen2_tx_power_on(struct hdmi_tx_dev *dev, u8 bit);


// HDMI_LIB API
extern int video_Configure(struct hdmi_tx_dev *dev, videoParams_t * video);
extern void video_params_reset(struct hdmi_tx_dev *dev, videoParams_t * params);

extern int packets_Configure(struct hdmi_tx_dev *dev, videoParams_t * video, productParams_t * prod);

// Output Start API
extern void tcc_output_starter_memclr(int img_width, int img_height);
extern void tccfb_output_starter(char output_type, char lcdc_num, stLTIMING *pstTiming, stLCDCTR *pstCtrl, unsigned int format);

extern int hdmi_api_Configure(struct hdmi_tx_dev *dev, videoParams_t * video, audioParams_t * audio,
		  productParams_t * product, hdcpParams_t * hdcp);
// STATIC API----------------------------------------------------------------------------------

static unsigned int hdmi_get_actual_tmds_bit_ratio(videoParams_t *videoParam)
{
        unsigned int actual_pixelclock = videoParam->mDtd.mPixelClock;

        switch(videoParam->mEncodingOut) 
        {
                case RGB:
                case YCC444:
                        switch(videoParam->mColorResolution) 
                        {
                                case COLOR_DEPTH_10:
                                        // 1.25x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock * 125)/100;
                                        break;
                                case COLOR_DEPTH_12:
                                        // 1.5x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock * 15)/10;
                                        break;
                                case COLOR_DEPTH_16:
                                        // 2x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock << 1);
                                        break;
                        }
                        break;
                case YCC420:
                        switch(videoParam->mColorResolution) 
                        {
                                case COLOR_DEPTH_8:
                                        // 0.5x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock >> 1);
                                        break;
                                case COLOR_DEPTH_10:
                                        // 0.625x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock * 625)/1000;
                                        break;
                                case COLOR_DEPTH_12:
                                        // 0.75x
                                        actual_pixelclock = (videoParam->mDtd.mPixelClock * 75)/100;
                                        break;
                        }
                        break;
                default:
                        // nothing..
                        break;
        }
        printk("\r\n >>actual pixel clock is %dKHz\r\n", actual_pixelclock);
        return actual_pixelclock;
}


static void tcc_output_starter_hdmi_start(unsigned int display_device){     
        struct hdmi_tx_dev *hdmi_dev = dwc_hdmi_api_get_dev();
        
        videoParams_t *videoParam = hdmi_dev->videoParam;
        audioParams_t *audioParam = hdmi_dev->audioParam;
        productParams_t *productParam = hdmi_dev->productParam;

        // Reset Unused Parameters
        product_reset(hdmi_dev, productParam);
        audio_reset(hdmi_dev, audioParam);
        
        hdmi_api_Configure(hdmi_dev, videoParam, audioParam, productParam, NULL);
		

        if(hdmi_get_actual_tmds_bit_ratio(videoParam) >= 340000) {
                scrambling(hdmi_dev, 1);
                scdc_tmds_bit_clock_ratio_enable_flag(hdmi_dev, 1);
        }
        else {
                scrambling(hdmi_dev, 0);
                scdc_tmds_bit_clock_ratio_enable_flag(hdmi_dev, 0);
        }
}


int tcc_output_starter_hdmi_v2_0(unsigned int vic, unsigned int hz, unsigned int isdvi,
                                        unsigned int display_device,
                                        VIOC_RDMA *pRDMA, VIOC_SC *pSC, VIOC_WMIX *pWMIX, VIOC_DISP *pDISP, 
                                        unsigned int scaler_num, unsigned int image_width, unsigned int image_height, unsigned int image_fmt)
{                                       
        int ret = 0;
        unsigned int displaydevice_width, displaydevice_height;
        struct hdmi_tx_dev *hdmi_dev;
        struct lcdc_timimg_parms_t lcdc_timimg_parms = {0};
        static videoParams_t video_param = {0};

        // Timing Param. 
        stLTIMING stHDMI_Timing;
        stLCDCTR stHDMI_Ctrl;

        // for YCC420
        uint16_t mHActive, mHBlanking, mHSyncOffset, mHSyncPulseWidth;
        
        hdmi_dev = dwc_hdmi_api_get_dev();

        video_params_reset(hdmi_dev, &video_param);
        
        // Initialize Video Param
        video_param.mEncodingOut = RGB;
        video_param.mColorResolution = COLOR_DEPTH_8;
        video_param.mEncodingIn = video_param.mEncodingOut;
        video_param.mHdmi20 = 1;

        if(isdvi)
                video_param.mHdmi = DVI;
        else
                video_param.mHdmi = HDMI;                

        if(dwc_hdmi_get_video_dtd(&video_param.mDtd, vic, hz) < 0) {
                printk("Force 1280x720@60p\r\n");
                dwc_hdmi_get_video_dtd(&video_param.mDtd, 4, 60000); // FORCE 720P
        }
        
        displaydevice_width = video_param.mDtd.mPixelRepetitionInput?(video_param.mDtd.mHActive>>1):video_param.mDtd.mHActive;
        displaydevice_height = video_param.mDtd.mInterlaced?(video_param.mDtd.mVActive << 1):(video_param.mDtd.mVActive);

        printk("\r\ntcc_output_starter_hdmi_v2_0 %dx%d\r\n", displaydevice_width, displaydevice_height);

        mHActive = video_param.mDtd.mHActive;
        mHBlanking = video_param.mDtd.mHBlanking;
        mHSyncOffset = video_param.mDtd.mHSyncOffset;
        mHSyncPulseWidth = video_param.mDtd.mHSyncPulseWidth;
        
        // Timing 
        memset(&lcdc_timimg_parms, 0, sizeof(lcdc_timimg_parms));
        lcdc_timimg_parms.iv = video_param.mDtd.mVSyncPolarity?0:1;  /** 0 for Active low, 1 active high */
        lcdc_timimg_parms.ih = video_param.mDtd.mHSyncPolarity?0:1;  /** 0 for Active low, 1 active high */
        lcdc_timimg_parms.dp = video_param.mDtd.mPixelRepetitionInput;
        
        if(video_param.mDtd.mInterlaced)
                lcdc_timimg_parms.tv = 1;
        else    
                lcdc_timimg_parms.ni = 1;      
        
        lcdc_timimg_parms.tft = 0;
        lcdc_timimg_parms.lpw = mHSyncPulseWidth-1;
        lcdc_timimg_parms.lpc = mHActive-1;
        lcdc_timimg_parms.lswc = (mHBlanking - (mHSyncOffset+mHSyncPulseWidth))-1;
        lcdc_timimg_parms.lewc = mHSyncOffset-1;
        
        if(video_param.mDtd.mInterlaced){
                if(video_param.mDtd.mPixelRepetitionInput) {
                        lcdc_timimg_parms.fpw = (video_param.mDtd.mVSyncPulseWidth << 1)-1;     
                        lcdc_timimg_parms.flc = (video_param.mDtd.mVActive << 1)-1;
                        lcdc_timimg_parms.fswc = (((video_param.mDtd.mVBlanking - (video_param.mDtd.mVSyncOffset + video_param.mDtd.mVSyncPulseWidth))) << 1)-1;
                        lcdc_timimg_parms.fewc = (video_param.mDtd.mVSyncOffset <<1 )-1;
                        lcdc_timimg_parms.fpw2 = lcdc_timimg_parms.fpw;
                        lcdc_timimg_parms.flc2 = lcdc_timimg_parms.flc;
                        lcdc_timimg_parms.fswc2 = lcdc_timimg_parms.fswc+1;
                        lcdc_timimg_parms.fewc2 = lcdc_timimg_parms.fewc+1;
                }else {
                        lcdc_timimg_parms.fpw = (video_param.mDtd.mVSyncPulseWidth << 1)-1;    
                        lcdc_timimg_parms.flc = (video_param.mDtd.mVActive<<1)-1;
                        lcdc_timimg_parms.fswc = ((video_param.mDtd.mVBlanking - (video_param.mDtd.mVSyncOffset + video_param.mDtd.mVSyncPulseWidth)) << 1)-1;
                        lcdc_timimg_parms.fewc = (video_param.mDtd.mVSyncOffset << 1);
                        lcdc_timimg_parms.fpw2 = lcdc_timimg_parms.fpw;
                        lcdc_timimg_parms.flc2 = lcdc_timimg_parms.flc;
                        lcdc_timimg_parms.fswc2 = lcdc_timimg_parms.fswc+1;
                        lcdc_timimg_parms.fewc2 = lcdc_timimg_parms.fewc-1;
                }
        }
        else {
                lcdc_timimg_parms.fpw = video_param.mDtd.mVSyncPulseWidth-1;     
                lcdc_timimg_parms.flc = video_param.mDtd.mVActive-1;
                lcdc_timimg_parms.fswc = (video_param.mDtd.mVBlanking - (video_param.mDtd.mVSyncOffset + video_param.mDtd.mVSyncPulseWidth))-1;
                lcdc_timimg_parms.fewc = video_param.mDtd.mVSyncOffset-1;
                lcdc_timimg_parms.fpw2 = lcdc_timimg_parms.fpw;
                lcdc_timimg_parms.flc2 = lcdc_timimg_parms.flc;
                lcdc_timimg_parms.fswc2 = lcdc_timimg_parms.fswc;
                lcdc_timimg_parms.fewc2 = lcdc_timimg_parms.fewc;
        } 

        if(display_device) {
                volatile unsigned long bits;
                bits = ioread32(hdmi_dev->ddibus_io); 
                set_bit(17, &bits);
                clear_bit(16, &bits);
                iowrite32(bits, hdmi_dev->ddibus_io);
                if(hdmi_dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io(0x%x) = 0x%x\r\n", (u32)hdmi_dev->ddibus_io, ioread32(hdmi_dev->ddibus_io));
                VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, VIOC_OUTCFG_DISP1);
        }
        else {
                volatile unsigned long bits;
                bits = ioread32(hdmi_dev->ddibus_io); 
                set_bit(16, &bits);
                clear_bit(17, &bits);
                iowrite32(bits, hdmi_dev->ddibus_io);
                if(hdmi_dev->verbose >= VERBOSE_IO)
                        printk("ddibus_io(0x%x) = 0x%x\r\n", (u32)hdmi_dev->ddibus_io, ioread32(hdmi_dev->ddibus_io));
                VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, VIOC_OUTCFG_DISP0);
        }

        // VIOC Display Device 
        VIOC_DISP_TurnOff(pDISP);
        VIOC_RDMA_SetImageDisable(pRDMA);

        // SWRESET WMIX, RDMA
        VIOC_WMIX_SetSWReset(VIOC_WMIX0+display_device, VIOC_WMIX_RDMA_00+(4*display_device), display_device);

        if(VIOC_CONFIG_PlugOut(scaler_num) != VIOC_PATH_DISCONNECTED)
                printk("%s, error: scaler%d couldn't be plugged-out!!\n", __func__, scaler_num);
        else
                printk("%s, success: scaler%d was plugged-out!!\n", __func__, scaler_num);

        tcc_output_starter_memclr(image_width, image_height);

        
        // Enable HDMI
        dwc_hdmi_core_power_on(hdmi_dev);


        // Mapping Display Device for HDMI output. 
        // Must Initialize display_device when HDMI Driver is open.
        VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, display_device);
                
        //tcc_hdmi_set_video_mode(display_device, &video);
        //tcc_hdmi_set_hdmi_mode(video_param.mode);

        VIOC_SC_SetBypass(pSC, 0);
        VIOC_SC_SetSrcSize(pSC, image_width, image_height);                     // set source size in scaler
        VIOC_SC_SetDstSize(pSC, displaydevice_width, displaydevice_height);           // set destination size in scaler
        VIOC_SC_SetOutSize(pSC, displaydevice_width, displaydevice_height);           // set output size in scaer
        VIOC_SC_SetUpdate(pSC);                                                                         // update scaler

        VIOC_RDMA_SetImageSize(pRDMA, image_width, image_height);       // set image size
        VIOC_RDMA_SetImageFormat(pRDMA, image_fmt);                                     // set image format
        VIOC_RDMA_SetImageIntl(pRDMA, 0);                                           // set image interlace mode
        VIOC_RDMA_SetImageOffset(pRDMA, image_fmt, image_width);        // set image offset

        VIOC_RDMA_SetImageAlphaSelect(pRDMA, 1);                                        // set alpha setting
        VIOC_RDMA_SetImageAlphaEnable(pRDMA, 1);                                        // set chroma key color setting
        VIOC_RDMA_SetImageEnable(pRDMA);

        VIOC_WMIX_SetPosition(pWMIX, 0, 0, 0);
        VIOC_WMIX_SetChromaKey(pWMIX, 0, 0, 0, 0, 0, 0xF8, 0xFC, 0xF8);
        VIOC_WMIX_SetUpdate(pWMIX);

        memset(&stHDMI_Timing, 0x00, sizeof(stHDMI_Timing));
        stHDMI_Timing.lpw = lcdc_timimg_parms.lpw;
        stHDMI_Timing.lpc = lcdc_timimg_parms.lpc + 1;
        stHDMI_Timing.lswc = lcdc_timimg_parms.lswc + 1;
        stHDMI_Timing.lewc = lcdc_timimg_parms.lewc + 1;
        stHDMI_Timing.vdb = lcdc_timimg_parms.vdb;
        stHDMI_Timing.vdf = lcdc_timimg_parms.vdf;
        stHDMI_Timing.fpw = lcdc_timimg_parms.fpw;
        stHDMI_Timing.flc = lcdc_timimg_parms.flc;
        stHDMI_Timing.fswc = lcdc_timimg_parms.fswc;
        stHDMI_Timing.fewc = lcdc_timimg_parms.fewc;
        stHDMI_Timing.fpw2 = lcdc_timimg_parms.fpw2;
        stHDMI_Timing.flc2 = lcdc_timimg_parms.flc2;
        stHDMI_Timing.fswc2 = lcdc_timimg_parms.fswc2;
        stHDMI_Timing.fewc2 = lcdc_timimg_parms.fewc2;
        
        memset(&stHDMI_Ctrl, 0x00, sizeof(stHDMI_Ctrl));
        stHDMI_Ctrl.id = lcdc_timimg_parms.id;
        stHDMI_Ctrl.iv = lcdc_timimg_parms.iv;
        stHDMI_Ctrl.ih = lcdc_timimg_parms.ih;
        stHDMI_Ctrl.ip = lcdc_timimg_parms.ip;
        stHDMI_Ctrl.clen = 0;
        stHDMI_Ctrl.dp = lcdc_timimg_parms.dp;
        stHDMI_Ctrl.ni = lcdc_timimg_parms.ni;
        #if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
                if(lcdc_timimg_parms.ni == 0)
                        stHDMI_Ctrl.advi = 0;
        #else
        #if defined(CONFIG_ARCH_TCC898X)
        stHDMI_Ctrl.advi = lcdc_timimg_parms.ni;
        #else
        if(lcdc_timimg_parms.ni == 0)
                stHDMI_Ctrl.advi = 1;
        #endif
        #endif
        
        stHDMI_Ctrl.tv = lcdc_timimg_parms.tv;

        // update video params.
        memcpy(hdmi_dev->videoParam, &video_param, sizeof(video_param));

        tcc_output_starter_hdmi_start(display_device);
        
        tccfb_output_starter(TCC_OUTPUT_HDMI, display_device, &stHDMI_Timing, &stHDMI_Ctrl, video_param.mEncodingOut);

        // Must Update SC, WMIX, RDMA!!
        VIOC_SC_SetUpdate(pSC);  
        VIOC_WMIX_SetUpdate(pWMIX);
        VIOC_RDMA_SetImageEnable(pRDMA);


                
        return ret;
}

