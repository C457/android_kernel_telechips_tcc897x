/*
 * @file:fc_video.h
 *
 *  Created on: Jul 1, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef HALFRAMECOMPOSERVIDEO_H_
#define HALFRAMECOMPOSERVIDEO_H_

int fc_video_config(struct hdmi_tx_dev *dev, videoParams_t *video);

void fc_video_hdcp_keepout(struct hdmi_tx_dev *dev, u8 bit);

#endif	/* HALFRAMECOMPOSERVIDEO_H_ */
