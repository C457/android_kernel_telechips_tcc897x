/*
 * @file:halFrameComposerAcp.c
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "frame_composer/frame_composer_reg.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../include/core/fc_acp.h"

void fc_acp_type(struct hdmi_tx_dev *dev, u8 type)
{
	LOG_TRACE1(type);
	hdmi_dev_write(dev, FC_ACP0, type);
}

void fc_acp_type_dependent_fields(struct hdmi_tx_dev *dev, u8 * fields, u8 fieldsLength)
{
	u8 c = 0;
	LOG_TRACE1(fields[0]);
	if (fieldsLength > (FC_ACP1 - FC_ACP16 + 1)) {
		fieldsLength = (FC_ACP1 - FC_ACP16 + 1);
		LOGGER(SNPS_WARN,"ACP Fields Truncated");
	}

	for (c = 0; c < fieldsLength; c++)
		hdmi_dev_write(dev, FC_ACP1 - c, fields[c]);
}
