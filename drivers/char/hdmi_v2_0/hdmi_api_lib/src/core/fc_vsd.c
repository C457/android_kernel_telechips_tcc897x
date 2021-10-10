/*
 * @file:halFrameComposerVsd.c
 *
 *  Created on: Jul 1, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "frame_composer/frame_composer_reg.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../include/core/fc_vsd.h"

void fc_vsd_vendor_OUI(struct hdmi_tx_dev *dev, u32 id)
{
	LOG_TRACE1(id);
	hdmi_dev_write(dev, (FC_VSDIEEEID0), id);
	hdmi_dev_write(dev, (FC_VSDIEEEID1), id >> 8);
	hdmi_dev_write(dev, (FC_VSDIEEEID2), id >> 16);
}

u8 fc_vsd_vendor_payload(struct hdmi_tx_dev *dev, const u8 * data, unsigned short length)
{
	const unsigned short size = 24;
	unsigned i = 0;
	LOG_TRACE();
	if (data == 0) {
		LOGGER(SNPS_WARN,"invalid parameter");
		return 1;
	}
	if (length > size) {
		length = size;
		LOGGER(SNPS_WARN,"vendor payload truncated");
	}
	for (i = 0; i < length; i++) {
		hdmi_dev_write(dev, (FC_VSDPAYLOAD0 + (i*4)), data[i]);
	}
	return 0;
}
