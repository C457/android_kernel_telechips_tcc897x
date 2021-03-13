/*
 * @file:halFrameComposerDebug.c
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"

#include "frame_composer/frame_composer_reg.h"
#include "../../include/core/fc_debug.h"


static void fc_force_audio(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);
	hdmi_dev_write_mask(dev, FC_DBGFORCE, FC_DBGFORCE_FORCEAUDIO_MASK, bit);
}

static void fc_force_video(struct hdmi_tx_dev *dev, u8 bit)
{
	LOG_TRACE1(bit);

	/* avoid glitches */
	if (bit != 0) {
		hdmi_dev_write(dev, FC_DBGTMDS2, bit ? 0x00 : 0x00);	/* R */
		hdmi_dev_write(dev, FC_DBGTMDS1, bit ? 0x00 : 0x00);	/* G */
		hdmi_dev_write(dev, FC_DBGTMDS0, bit ? 0xFF : 0x00);	/* B */
		hdmi_dev_write_mask(dev, FC_DBGFORCE, FC_DBGFORCE_FORCEVIDEO_MASK, 1);
	} else {
		hdmi_dev_write_mask(dev, FC_DBGFORCE, FC_DBGFORCE_FORCEVIDEO_MASK, 0);
		hdmi_dev_write(dev, FC_DBGTMDS2, bit ? 0x00 : 0x00);	/* R */
		hdmi_dev_write(dev, FC_DBGTMDS1, bit ? 0x00 : 0x00);	/* G */
		hdmi_dev_write(dev, FC_DBGTMDS0, bit ? 0xFF : 0x00);	/* B */
	}
}

void fc_force_output(struct hdmi_tx_dev *dev, int enable)
{
	fc_force_audio(dev, 0);
	fc_force_video(dev, (u8)enable);
}
