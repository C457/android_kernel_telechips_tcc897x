/*
 * @file:fc_gamut.h
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef HALFRAMECOMPOSERGAMUT_H_
#define HALFRAMECOMPOSERGAMUT_H_

void fc_gamut_enable_tx(struct hdmi_tx_dev *dev, u8 enable);

void fc_gamut_config(struct hdmi_tx_dev *dev);

void fc_gamut_packet_config(struct hdmi_tx_dev *dev, const u8 * gbdContent, u8 length);

#endif	/* HALFRAMECOMPOSERGAMUT_H_ */
