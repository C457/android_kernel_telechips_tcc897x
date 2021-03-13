/*
 * @file:fc_audio.h
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef HALFRAMECOMPOSERAUDIO_H_
#define HALFRAMECOMPOSERAUDIO_H_

#include "../../../include/hdmi_includes.h"
//#include "util/types.h"
#include "../../../include/audio_params.h"

void fc_audio_config(struct hdmi_tx_dev *dev, audioParams_t * audio);
void fc_audio_mute(struct hdmi_tx_dev *dev);
void fc_audio_unmute(struct hdmi_tx_dev *dev);

#endif	/* HALFRAMECOMPOSERAUDIO_H_ */
