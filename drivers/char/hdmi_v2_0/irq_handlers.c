/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        irq_handler.c
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
#include "include/hdmi_access.h"
#include "include/hdmi_log.h"
#include "include/irq_handlers.h"
#include "hdmi_api_lib/include/core/irq.h"
#include "hdmi_api_lib/include/core/interrupt/interrupt_reg.h"
#include "hdmi_api_lib/include/phy/phy.h"

static void dwc_hdmi_tx_handler_thread(struct work_struct *work)
{
        uint8_t phy_decode;
        uint32_t decode, hpd_signal;
        struct hdmi_tx_dev *dev;

        dev = container_of(work, struct hdmi_tx_dev, tx_handler);
        
        if(dev->verbose >= VERBOSE_IRQ)
                pr_info("dwc_hdmi_tx_handler_thread\r\n");
        

        // Read HDMI TX IRQ
        decode = hdmi_read_interrupt_decode(dev);

        // precess irq
        if((decode & IH_DECODE_IH_FC_STAT0_MASK) ? 1 : 0){
                hdmi_irq_clear_source(dev, AUDIO_PACKETS);
        }

        if((decode & IH_DECODE_IH_FC_STAT1_MASK) ? 1 : 0){
                hdmi_irq_clear_source(dev, OTHER_PACKETS);
        }

        if((decode & IH_DECODE_IH_FC_STAT2_VP_MASK) ? 1 : 0){
                hdmi_irq_mute_source(dev, PACKETS_OVERFLOW);
                hdmi_irq_mute_source(dev, VIDEO_PACKETIZER);
        }
        if((decode & IH_DECODE_IH_AS_STAT0_MASK) ? 1 : 0){
                hdmi_irq_clear_source(dev, AUDIO_SAMPLER);
        }
        if((decode & IH_DECODE_IH_PHY_MASK) ? 1 : 0){
                LOGGER(SNPS_TRACE, "%s:PHY interrupt 0x%08x", __func__, decode);

                hdmi_irq_read_stat(dev, PHY, &phy_decode);

                if((phy_decode & IH_PHY_STAT0_TX_PHY_LOCK_MASK) ? 1 : 0){
                        hdmi_irq_clear_bit(dev, PHY, IH_PHY_STAT0_TX_PHY_LOCK_MASK);
                }

                if((phy_decode & IH_PHY_STAT0_HPD_MASK) ? 1 : 0) {
                        LOGGER(SNPS_DEBUG, "HPD received");
                        dev->hpd = hdmi_phy_hot_plug_detected(dev);
                        hdmi_irq_clear_bit(dev, PHY, IH_PHY_STAT0_HPD_MASK);
                        if(dev->verbose >= VERBOSE_IRQ)
                                pr_info(" >>>HPD :%d\r\n",dev->hpd );
                }
        }
        if((decode & IH_DECODE_IH_I2CM_STAT0_MASK) ? 1 : 0){
                uint8_t state = 0;

                hdmi_irq_read_stat(dev, I2C_DDC, &state);

                // I2Cmastererror - I2Cmasterdone
                if(state & 0x3){
                        hdmi_irq_clear_bit(dev, I2C_DDC, IH_I2CM_STAT0_I2CMASTERDONE_MASK);
                        //The I2C communication interrupts must be masked - they will be handled by polling in the eDDC block
                        LOGGER(SNPS_NOTICE, "%s:I2C DDC interrupt received 0x%x - mask interrupt", __func__, state);
                }
                // SCDC_READREQ
                else if(state & 0x4){
                        hdmi_irq_clear_bit(dev, I2C_DDC, IH_I2CM_STAT0_SCDC_READREQ_MASK);
                }
        }
        if((decode & IH_DECODE_IH_CEC_STAT0_MASK) ? 1 : 0){
                hdmi_irq_clear_source(dev, CEC);
        }

        hdmi_irq_unmute(dev, MUTE_ID_IRQ);
        
}

static irqreturn_t
dwc_hdmi_tx_handler(int irq, void *dev_id){
	struct hdmi_tx_dev *dev = dev_id;
        uint32_t decode, hdcp_irq;

	if(dev == NULL)
		return IRQ_NONE;

	if(dev->verbose >= VERBOSE_BASIC)
		pr_info("%s\n", FUNC_NAME);
        decode = hdmi_read_interrupt_decode(dev);
        hdcp_irq = hdmi_dev_read(dev, 0x0001401C/*A_APIINTSTAT*/);

        if(decode) {
                hdmi_irq_mute(dev, MUTE_ID_IRQ);
        }
        else if(hdcp_irq){
                hdmi_dev_write(dev, 0x00014020/*A_APIINTMSK*/, 0xff); // a_apiintmsk 0x5008
        }
        else {
                return IRQ_HANDLED;
        }

        schedule_work(&dev->tx_handler);
        return IRQ_HANDLED;
}


static irqreturn_t
hdcp_handler(int irq, void *dev_id){
	struct hdmi_tx_dev *dev = NULL;

	if(dev_id == NULL)
		return IRQ_NONE;

	dev = dev_id;

	if(dev->verbose >= VERBOSE_BASIC)
		pr_info("%s\n", FUNC_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t
dwc_hdmi_tx_cec_handler(int irq, void *dev_id){
	struct hdmi_tx_dev *dev = NULL;

	if(dev_id == NULL)
		return IRQ_NONE;

	dev = dev_id;

	if(dev->verbose >= VERBOSE_BASIC)
		pr_info("%s\n", FUNC_NAME);

	return IRQ_HANDLED;
}


int
dwc_init_interrupts(struct hdmi_tx_dev *dev){
        int ret = 0;

        // Disable all interrupts
        if(dev->clock_enable_count > 0)
                hdmi_irq_mask_all(dev, MUTE_ID_IRQ);

        INIT_WORK(&dev->tx_handler, dwc_hdmi_tx_handler_thread);
        
        ret = devm_request_irq(dev->parent_dev, dev->irq[HDMI_IRQ_TX_CORE],
                               dwc_hdmi_tx_handler, IRQF_SHARED,
                               "dwc_hdmi_tx_handler", dev);
        if (ret){
                pr_err("%s:Could not register dwc_hdmi_tx interrupt\n",
                        FUNC_NAME);
        }
        
        ret = devm_request_irq(dev->parent_dev, dev->irq[HDMI_IRQ_TX_HDCP],
                               hdcp_handler, IRQF_SHARED,
                               "hdcp_handler", dev);
        if (ret){
                pr_err("%s:Could not register hdcp interrupt\n",
                        FUNC_NAME);
        }
        
        ret = devm_request_irq(dev->parent_dev, dev->irq[HDMI_IRQ_TX_CEC],
                               dwc_hdmi_tx_cec_handler, IRQF_SHARED,
                               "dwc_hdmi_tx_cec_handler", dev);
        if (ret){
                pr_err("%s:Could not register dwc_hdmi_tx_cec interrupt\n",
                        FUNC_NAME);
        }
        if(dev->clock_enable_count > 0)
                hdmi_irq_unmute(dev, MUTE_ID_IRQ);
        return ret;
}


int
dwc_deinit_interrupts(struct hdmi_tx_dev *dev){
        int i = 0;

        if(dev->clock_enable_count > 0)
                hdmi_irq_mask_all(dev, MUTE_ID_IRQ);
        
        flush_work(&dev->tx_handler);

        // Unregister interrupts
        for(i = 0; i < (sizeof(dev->irq) / sizeof(dev->irq[0])); i++){
                if(dev->irq[i]){
                        devm_free_irq(dev->parent_dev, dev->irq[i], dev);
                }
        }

        return 0;
}

