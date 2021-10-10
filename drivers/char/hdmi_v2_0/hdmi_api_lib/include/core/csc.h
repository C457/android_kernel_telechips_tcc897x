/*
 * halColorSpaceConverter.h
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef HALCOLORSPACECONVERTER_H_
#define HALCOLORSPACECONVERTER_H_

void csc_config(struct hdmi_tx_dev *dev, videoParams_t * params,
		unsigned interpolation, unsigned decimation, unsigned color_depth);

#endif	/* HALCOLORSPACECONVERTER_H_ */
