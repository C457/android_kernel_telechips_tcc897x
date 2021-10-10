/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_api.c
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
#include "include/irq_handlers.h"
#include "include/hdmi_ioctls.h"
#include "include/hdmi_access.h"
#include "include/video_params.h"
#include "include/audio_params.h"


static struct hdmi_tx_dev *api_dev = NULL;

void dwc_hdmi_api_register(struct hdmi_tx_dev *dev){
        api_dev = dev;
}

struct hdmi_tx_dev *dwc_hdmi_api_get_dev(void){
        return api_dev;
}
EXPORT_SYMBOL(dwc_hdmi_api_get_dev);


int dwc_hdmi_get_video_dtd(dtd_t *hdmi_dtd, uint32_t code, uint32_t hz){
        return hdmi_dtd_fill(hdmi_dtd, code, hz); 
}
EXPORT_SYMBOL(dwc_hdmi_get_video_dtd);

void hdmi_start(void){
        // Must Implement It..!!
}
EXPORT_SYMBOL(hdmi_start);

void hdmi_stop(void){
        // Must Implement It..!!
}
EXPORT_SYMBOL(hdmi_stop);

int hdmi_get_VBlank(void){
        uint32_t vblank = 0;

        if(!api_dev) {
                pr_err("hdmi driver is not ready..!!\r\n");
                goto end_process;
        }
        
        vblank = hdmi_dev_read(api_dev, 0x0000401C);   // FC_INVBLANK
        if(vblank < 0) {
                pr_err("failed to read hdmi register: %d\n", vblank);
        }

end_process:
        return (vblank & 0xFF);
}
EXPORT_SYMBOL(hdmi_get_VBlank);

unsigned int hdmi_get_refreshrate(void) {
        
        videoParams_t *videoParams;
        unsigned int refreshrate = 0;
        
        if(!api_dev) {
                pr_err("hdmi driver is not ready..!!\r\n");
                goto end_process;
        }

        videoParams = (videoParams_t*)api_dev->videoParam;

        refreshrate = hdmi_dtd_get_refresh_rate(&videoParams->mDtd);

        if(refreshrate > 1000)
                refreshrate /= 1000;
end_process:
        return refreshrate;
        
}
EXPORT_SYMBOL(hdmi_get_refreshrate);

