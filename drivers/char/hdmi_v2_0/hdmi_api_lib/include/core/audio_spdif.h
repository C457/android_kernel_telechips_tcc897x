/*
 * audio_spdif.h
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef HALAUDIOSPDIF_H_
#define HALAUDIOSPDIF_H_

#include "../../../include/hdmi_includes.h"
#include "../../../include/audio_params.h"

void audio_spdif_config(struct hdmi_tx_dev  *dev, audioParams_t *audio);

void audio_spdif_interrupt_mask(struct hdmi_tx_dev  *dev, u8 value);

#endif	/* HALAUDIOSPDIF_H_ */
