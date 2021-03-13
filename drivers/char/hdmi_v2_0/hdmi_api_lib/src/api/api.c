/*
 * @file:api.c
 *
 *  Synopsys Inc.
 *  SG DWC PT02
 */


#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../../include/video_params.h"

        

#include "../../include/api/api.h"
#include "../../include/core/main_controller.h"
#include "../../include/core/video.h"
#include "../../include/core/audio.h"
#include "../../include/core/packets.h"
#include "../../include/core/irq.h"

#include "../../include/core/product.h"

#include "../../include/core/fc_debug.h"
#include "../../include/core/fc_video.h"

/* Init Value. 

video_mode_t hdmi_on = 0;
struct hdmi_tx_ctrl *tx_ctrl = &dev->hdmi_tx_ctrl;

hdmi_on = video->mHdmi;
tx_ctrl->hdmi_on = (hdmi_on == HDMI) ? 1 : 0;
tx_ctrl->pixel_clock = videoParams_GetPixelClock(dev,video);
tx_ctrl->color_resolution = video->mColorResolution;
tx_ctrl->pixel_repetition = PIXEL_REPETITION_OFF;
tx_ctrl->csc_on = 1;
tx_ctrl->audio_on = (hdmi_on == HDMI) ? 1 : 0;

*/
extern int dwc_hdmi_phy_config(struct hdmi_tx_dev *dev, unsigned int freq_hz, unsigned int pixel);
extern void dwc_hdmi_hw_reset(struct hdmi_tx_dev *dev, int reset_on) ;

void hdmi_tx_prepare(struct hdmi_tx_dev *dev)
{
        videoParams_t video;
        
        //Always due to HDCP and Scrambler
        fc_video_hdcp_keepout(dev, 1);

        hdmi_irq_mute(dev, MUTE_ID_API);

        video_params_reset(dev, &video);
        hdmi_dtd_fill(&video.mDtd, 1, 60000); /* VGA */
        video.mHdmi = DVI;   /* DVI */

        // board_Initialize - by default mask all interrupts
        hdmi_dev_write(dev, 0x0000201C/*VP_MASK*/,     0xFF ); /* VP */
        hdmi_dev_write(dev, 0x00004348/*FC_MASK0*/,    0xFF ); /* Packets */
        hdmi_dev_write(dev, 0x00004358/*FC_MASK1*/,    0xFF ); /* Packets */
        hdmi_dev_write(dev, 0x00004368/*FC_MASK2*/,    0xFF ); /* Packets */
        hdmi_dev_write(dev, 0x0000C018/*PHY_MASK0*/,   0xF1 ); /* PHY - leave HPD */
        hdmi_dev_write(dev, 0x0000C408/*AUD_INT*/,     0xFF ); /* AS - I2S */
        hdmi_dev_write(dev, 0x0000CC08/*AUD_SPDIFINT*/,0xFF ); /* AS - SPDIF */
        hdmi_dev_write(dev, 0x00014020/*A_APIINTMSK*/, 0xFF ); /* HDCP */
        hdmi_dev_write(dev, 0x0001F408/*CEC_MASK*/,    0xFF ); /* CEC */

        packets_Initialize(dev);

        hdmi_irq_clear_all(dev);

        mc_disable_all_clocks(dev);

        video_Configure(dev, &video);

        mc_enable_all_clocks(dev);                

        dwc_hdmi_phy_config(dev, videoParams_GetPixelClock(dev, &video), video.mColorResolution);

        // disable blue screen transmission after turning on all necessary blocks (e.g. HDCP)
        fc_force_output(dev, TRUE);
}

int hdmi_api_Configure(struct hdmi_tx_dev *dev, videoParams_t * video, audioParams_t * audio,
productParams_t * product, hdcpParams_t * hdcp)
{
        int success = 0;
        unsigned int pixelclock, ColorResolution;
        
        if (!dev) {
                LOGGER(SNPS_ERROR, "dev pointer is NULL");
                success =  -1;
                goto end_process;
        }

        if (!video || !product ) {
                LOGGER(SNPS_ERROR, "Arguments: video=%x; audio=%x; product=%x; hdcp=%x", 
                (unsigned int)video, (unsigned int)audio, (unsigned int)product, (unsigned int)hdcp);
                success =  -1;
                goto end_process;
        }
        hdmi_irq_clear_all(dev);
        // Reset HDMI Link & PHY
        dwc_hdmi_hw_reset(dev, 1);
        mdelay(1);
        dwc_hdmi_hw_reset(dev, 0);
        mdelay(10);
        hdmi_irq_mask_all(dev, MUTE_ID_API);

        hdmi_tx_prepare(dev);
        if(video->mEncodingOut == YCC420){
                //video->mHdmi20 = 1;
                hdmi_dev_write_mask(dev, (0x1032<<2), 0xff, 0);
                hdmi_dev_write_mask(dev, (0x1033<<2), 0xff, 0);
        }

        hdmi_api_avmute(dev, TRUE);

        //phy_standby(dev);

        // Disable interrupts
        hdmi_irq_mute(dev, MUTE_ID_API);

        success = video_Configure(dev, video);
        if (success < 0) {
                LOGGER(SNPS_ERROR, "%s:Could not configure video", __func__);
                goto end_process;

        }

        // Audio - Workaround
        if(audio != NULL) {
                audio_Initialize(dev);
                success = audio_Configure(dev, audio);
                if(success == FALSE){
                        LOGGER(SNPS_ERROR, "Audio not configured");
                        success = -1;
                        goto end_process;
                }
        }

        // Packets
        success = packets_Configure(dev, video, product);
        if (success == FALSE) {
                LOGGER(SNPS_ERROR, "Could not configure packets");  
                success = -1;
                goto end_process;
        }

        mc_enable_all_clocks(dev);
        mdelay(10);


        
                
        // PHY CONFIG
        pixelclock = videoParams_GetPixelClock(dev, video);
        ColorResolution = video->mColorResolution;
        
        switch(video->mEncodingOut) {
                case YCC420:
                        pixelclock = videoParams_GetPixelClock(dev, video)>>1;
                        break;
                case YCC422:
                        // YCC422 must ust 8bit of phy setting. 
                        ColorResolution = COLOR_DEPTH_8;
                        break;
                default:
                        // Nothing - fix warring.
                        break;
        }
                
        if(dwc_hdmi_phy_config(dev, pixelclock, ColorResolution) < 0) {
                printk("Cann't settig HDMI PHY\r\n");
                success = -1;
                goto end_process;
        }

        // Disable blue screen transmission after turning on all necessary blocks (e.g. HDCP)
        fc_force_output(dev, 0);

        #if defined(HDMI_SUPPORT_HDCP)
        if(hdcp) {
                //TODO:This should be called at HPD event
                // HDCP is PHY independent
                if (hdcp_initialize(dev) != TRUE) {
                        LOGGER(SNPS_ERROR, "Could not initialize HDCP");
                }
                mdelay(20);
                success = hdcp_configure(dev, hdcp, video);
                if (success == FALSE) {
                        LOGGER(SNPS_ERROR, "Could not configure HDCP");
                }
        }
        #endif

        // enable interrupts
        hdmi_irq_unmute(dev, MUTE_ID_API);
        
        //hdmi_api_avmute(dev, 0); // disable av mute

        hdmi_hpd_enable(dev);
end_process:
        return success;
}

void hdmi_api_avmute(struct hdmi_tx_dev *dev, int enable)
{
        packets_AvMute(dev, enable);
#if defined(HDMI_SUPPORT_HDCP)
        hdcp_av_mute(dev, enable);
#endif
}
