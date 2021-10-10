/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        phy.c
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


#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"

#include "../../include/core/interrupt/interrupt_reg.h"
#include "../../include/core/irq.h"
#include "../../include/core/main_controller.h"

#include "../../include/bsp/i2cm.h"
#include "../../include/phy/phy.h"



/*************************************************************
 * Internal functions
 */
void hdmi_phy_power_down(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_SPARES_2_MASK, (bit ? 1 : 0));
}

void hdmi_phy_enable_tmds(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_SPARES_1_MASK, (bit ? 1 : 0));
}

void hdmi_phy_gen2_pddq(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_PDDQ_MASK, (bit ? 1 : 0));
}

void hdmi_phy_gen2_tx_power_on(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_TXPWRON_MASK, (bit ? 1 : 0));
}

static void phy_gen2_en_hpd_rx_sense(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_ENHPDRXSENSE_MASK, (bit ? 1 : 0));
}

void hdmi_phy_data_enable_polarity(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_SELDATAENPOL_MASK, (bit ? 1 : 0));
}

void hdmi_phy_interface_control(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_SELDIPIF_MASK, (bit ? 1 : 0));
}

u8 hdmi_phy_interrupt_state(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	return hdmi_dev_read(dev,PHY_INT0);
}

u8 hdmi_phy_interrupt_mask_status(struct hdmi_tx_dev *dev, u8 mask)
{
	LOG_TRACE1(mask);
	return hdmi_dev_read(dev, PHY_MASK0) & mask;
}

void hdmi_phy_interrupt_polarity(struct hdmi_tx_dev *dev, u8 bitShift, u8 value)
{
	LOG_TRACE2(bitShift, value);
	hdmi_dev_write_mask(dev, PHY_POL0, (1 << bitShift), value);
}

u8 hdmi_phy_interrupt_polarity_status(struct hdmi_tx_dev *dev, u8 mask)
{
	LOG_TRACE1(mask);
	return hdmi_dev_read(dev, PHY_POL0) & mask;
}


/*************************************************************
 * External functions
 *************************************************************/
int hdmi_phy_initialize(struct hdmi_tx_dev *dev, u16 phy_model)
{
	LOG_TRACE1(dev->snps_hdmi_ctrl.data_enable_polarity);

	// phy_rstn to 0 (active low - phy reset)
	mc_phy_reset(dev, 0);
        
	// i_phy_en signal to 1
	hdmi_dev_write_mask(dev, PHY_CONF0, PHY_CONF0_TXPWRON_MASK, 1);

	// need refclk * 10 delay. 

	return TRUE;
}

int hdmi_phy_enable_hpd_sense(struct hdmi_tx_dev *dev)
{
	phy_gen2_en_hpd_rx_sense(dev, 0);
	return TRUE;
}

int hdmi_phy_disable_hpd_sense(struct hdmi_tx_dev *dev)
{
	phy_gen2_en_hpd_rx_sense(dev, 1);
	return TRUE;
}

int hdmi_phy_hot_plug_detected(struct hdmi_tx_dev *dev)
{
	/* MASK         STATUS          POLARITY        INTERRUPT        HPD
	 *   0             0                 0               1             0
	 *   0             1                 0               0             1
	 *   0             0                 1               0             0
	 *   0             1                 1               1             1
	 *   1             x                 x               0             x
	 */

	int hpd_polarity = hdmi_dev_read_mask(dev, PHY_POL0, PHY_POL0_HPD_MASK);
	int hpd = hdmi_dev_read_mask(dev, PHY_STAT0, PHY_STAT0_HPD_MASK);

	// Mask interrupt
	hdmi_phy_interrupt_mask(dev, PHY_MASK0_HPD_MASK);

	if (hpd_polarity == hpd) {
		hdmi_dev_write_mask(dev, PHY_POL0, PHY_POL0_HPD_MASK, !hpd_polarity);

		// Un-mask interrupts
		hdmi_phy_interrupt_unmask(dev, PHY_MASK0_HPD_MASK);

		return hpd_polarity;
	}

	// Un-mask interrupts
	hdmi_phy_interrupt_unmask(dev, PHY_MASK0_HPD_MASK);

	return !hpd_polarity;
}

int hdmi_phy_interrupt_enable(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write(dev, PHY_MASK0, value);
	return TRUE;
}

int hdmi_phy_phase_lock_loop_state(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	return hdmi_dev_read_mask(dev, (PHY_STAT0), PHY_STAT0_TX_PHY_LOCK_MASK);
}

void hdmi_phy_interrupt_mask(struct hdmi_tx_dev *dev, u8 mask)
{
	LOG_TRACE1(mask);
	// Mask will determine which bits will be enabled
	hdmi_dev_write_mask(dev, PHY_MASK0, mask, 0xff);
}

void hdmi_phy_interrupt_unmask(struct hdmi_tx_dev *dev, u8 mask)
{
	LOG_TRACE1(mask);
	// Mask will determine which bits will be enabled
	hdmi_dev_write_mask(dev, PHY_MASK0, mask, 0x0);
}

u8 hdmi_phy_hot_plug_state(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	return hdmi_dev_read_mask(dev, (PHY_STAT0), PHY_STAT0_HPD_MASK);
}


