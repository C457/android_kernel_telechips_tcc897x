/*
 * @file:halFrameComposerPackets.c
 *
 *  Created on: Jul 1, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */ 
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "frame_composer/frame_composer_reg.h"
#include "../../include/core/fc_packets.h"



void fc_packets_QueuePriorityHigh(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write_mask(dev, FC_CTRLQHIGH, FC_CTRLQHIGH_ONHIGHATTENDED_MASK, value);
}

void fc_packets_QueuePriorityLow(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write_mask(dev, FC_CTRLQLOW, FC_CTRLQLOW_ONLOWATTENDED_MASK, value);
}

void fc_packets_MetadataFrameInterpolation(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write_mask(dev, FC_DATAUTO1, FC_DATAUTO1_AUTO_FRAME_INTERPOLATION_MASK, value);
}

void fc_packets_MetadataFramesPerPacket(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write_mask(dev, FC_DATAUTO2, FC_DATAUTO2_AUTO_FRAME_PACKETS_MASK, value);
}

void fc_packets_MetadataLineSpacing(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write_mask(dev, FC_DATAUTO2, FC_DATAUTO2_AUTO_LINE_SPACING_MASK, value);
}

void fc_packets_AutoSend(struct hdmi_tx_dev *dev, u8 enable, u8 mask)
{
	LOG_TRACE2(enable, mask);
	hdmi_dev_write_mask(dev, FC_DATAUTO0, (1 << mask), (enable ? 1 : 0));
}

void fc_packets_ManualSend(struct hdmi_tx_dev *dev, u8 mask)
{
	LOG_TRACE1(mask);
	hdmi_dev_write_mask(dev, FC_DATMAN, (1 << mask), 1);
}

void fc_packets_disable_all(struct hdmi_tx_dev *dev)
{
	uint32_t value = ~( BIT(ACP_TX) | BIT(ISRC1_TX) | BIT(ISRC2_TX) | BIT(SPD_TX) | BIT(VSD_TX) );
	LOG_TRACE();
	hdmi_dev_write(dev, FC_DATAUTO0, value & hdmi_dev_read(dev, FC_DATAUTO0));
}

void fc_packets_metadata_config(struct hdmi_tx_dev *dev)
{
	fc_packets_MetadataFrameInterpolation(dev, 1);
	fc_packets_MetadataFramesPerPacket(dev, 1);
	fc_packets_MetadataLineSpacing(dev, 1);
}
