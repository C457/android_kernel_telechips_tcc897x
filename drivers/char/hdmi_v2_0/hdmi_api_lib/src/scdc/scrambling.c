/*
 * scrambling.c
 *
 *  Created on: Feb 21, 2014
 *      Author: 
 */
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../core/frame_composer/frame_composer_reg.h"
#include "../../include/scdc/scdc.h"
#include "../../include/scdc/scrambling.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../include/core/fc_video.h"
#include "../../include/core/main_controller.h"


void scrambling(struct hdmi_tx_dev *dev, u8 enable){
	u8 data = 0;
	if (enable == 1) {
		scdc_scrambling_enable_flag(dev, 1);
		udelay(100);

		/* Start/stop HDCP keep-out window generation not needed because it's always on */
		/* TMDS software reset request */
		mc_tmds_clock_reset(dev, 1);

		/* Enable/Disable Scrambling */
		scrambling_Enable(dev, 1);

		/* Write on RX the bit Scrambling_Enable, register 0x20 */
		data = 0x01;
		scdc_write(dev, SCDC_TMDS_CONFIG, 1, &data);
	} else {
		/* Enable/Disable Scrambling */
		scrambling_Enable(dev, 0);
		scdc_scrambling_enable_flag(dev, 0);

		/* TMDS software reset request */
		mc_tmds_clock_reset(dev, 0);

		/* Write on RX the bit Scrambling_Enable, register 0x20 */
		data = 0x00;
		scdc_write(dev, SCDC_TMDS_CONFIG, 1, &data);
	}
}

void scrambling_Enable(struct hdmi_tx_dev *dev, u8 bit)
{
	hdmi_dev_write_mask(dev, FC_SCRAMBLER_CTRL, FC_SCRAMBLER_CTRL_SCRAMBLER_ON_MASK, bit);
}
