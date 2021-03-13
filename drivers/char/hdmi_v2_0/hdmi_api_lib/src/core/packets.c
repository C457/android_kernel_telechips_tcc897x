/*
 * @file:packets.c
 *
 *  Created on: Jul 7, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */

 
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "frame_composer/frame_composer_reg.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../include/core/packets.h"
#include "../../include/core/fc_packets.h"
#include "../../include/core/fc_gamut.h"

#include "../../include/core/fc_acp.h"

#include "../../include/core/fc_audio_info.h"

#include "../../include/core/fc_avi.h"
#include "../../include/core/fc_isrc.h"
#include "../../include/core/fc_spd.h"
#include "../../include/core/fc_vsd.h"
#include "../../include/core/audio_multistream.h"


#define ACP_PACKET_SIZE 	16
#define ISRC_PACKET_SIZE 	16


int packets_Initialize(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	packets_DisableAllPackets(dev);
	return TRUE;
}

int packets_Configure(struct hdmi_tx_dev *dev, videoParams_t * video, productParams_t * prod)
{
	u8 send3d = FALSE;
	LOG_TRACE();

	//if(dev->hdmi_tx_ctrl.hdmi_on == 0){
	//	LOGGER(SNPS_WARN, "DVI mode selected: packets not configured");
	//	return TRUE;
	//}

	if (video->mHdmiVideoFormat == 2) {
		u8 struct_3d = video->m3dStructure;

		LOGGER(SNPS_DEBUG,"%s:packets configure", __func__);

		// frame packing || tab || sbs
		if ((struct_3d == 0) || (struct_3d == 6) || (struct_3d == 8)) {
			u8 data[3];
			data[0] = video->mHdmiVideoFormat << 5;
			data[1] = struct_3d << 4;
			data[2] = video->m3dExtData << 4;
			// HDMI Licensing, LLC
			packets_VendorSpecificInfoFrame(dev, 0x000C03, data, sizeof(data), 1);
			send3d = TRUE;
		}
		else {
			LOGGER(SNPS_ERROR,"%s:3D structure not supported %d", __func__, struct_3d);
			return FALSE;
		}
	}

	if (prod != 0) {
		fc_spd_info_t spd_data = {
                        .vName = prod->mVendorName,
		        .vLength = prod->mVendorNameLength,
		        .pName = prod->mProductName,
		        .pLength = prod->mProductNameLength,
		        .code = prod->mSourceType,
		        .autoSend = 1,
		};

		u32 oui = prod->mOUI;
		u8 *vendor_payload = prod->mVendorPayload;
		u8 payload_length = prod->mVendorPayloadLength;

		fc_spd_config(dev, &spd_data);

		if (send3d) {
			LOGGER(SNPS_WARN, "%s_forcing Vendor Specific InfoFrame, 3D configuration will be ignored",
					__func__);
		} else {
        		packets_VendorSpecificInfoFrame(dev, oui, vendor_payload, payload_length, 1);
		}
	}
	else {
		LOGGER(SNPS_WARN,"No product info provided: not configured");
	}

	fc_packets_metadata_config(dev);

	// default phase 1 = true
	hdmi_dev_write_mask(dev, FC_GCP, FC_GCP_DEFAULT_PHASE_MASK, ((video->mPixelPackingDefaultPhase == 1) ? 1 : 0));

	fc_gamut_config(dev);

	fc_avi_config(dev, video);

	/** Colorimetry */
	packets_colorimetry_config(dev, video);

	return TRUE;
}

void packets_AudioContentProtection(struct hdmi_tx_dev *dev, u8 type, const u8 * fields, u8 length, u8 autoSend)
{
	u8 newFields[ACP_PACKET_SIZE];
	u16 i = 0;
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, ACP_TX);

	fc_acp_type(dev, type);

	for (i = 0; i < length; i++) {
		newFields[i] = fields[i];
	}
	if (length < ACP_PACKET_SIZE) {
		for (i = length; i < ACP_PACKET_SIZE; i++) {
			newFields[i] = 0;	/* Padding */
		}
		length = ACP_PACKET_SIZE;
	}
	fc_acp_type_dependent_fields(dev, newFields, length);
	if (!autoSend) {
		fc_packets_ManualSend(dev, ACP_TX);
	} else {
		fc_packets_AutoSend(dev, autoSend, ACP_TX);
	}

}

void packets_IsrcPackets(struct hdmi_tx_dev *dev, u8 initStatus, const u8 * codes, u8 length, u8 autoSend)
{
	u16 i = 0;
	u8 newCodes[ISRC_PACKET_SIZE * 2];
	LOG_TRACE();

	fc_packets_AutoSend(dev, 0, ISRC1_TX);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);

	fc_isrc_status(dev, initStatus);

	for (i = 0; i < length; i++) {
		newCodes[i] = codes[i];
	}

	if (length > ISRC_PACKET_SIZE) {
		for (i = length; i < (ISRC_PACKET_SIZE * 2); i++) {
			newCodes[i] = 0;	/* Padding */
		}
		length = (ISRC_PACKET_SIZE * 2);

		fc_isrc_isrc2_codes(dev, newCodes + (ISRC_PACKET_SIZE * sizeof(u8)), length - ISRC_PACKET_SIZE);
		fc_isrc_cont(dev, 1);

		fc_packets_AutoSend(dev, autoSend, ISRC2_TX);

		if (!autoSend) {
			fc_packets_ManualSend(dev, ISRC2_TX);
		}
	}
	if (length < ISRC_PACKET_SIZE) {
		for (i = length; i < ISRC_PACKET_SIZE; i++) {
			newCodes[i] = 0;	/* Padding */
		}

		length = ISRC_PACKET_SIZE;

		fc_isrc_cont(dev, 0);
	}

	fc_isrc_isrc1_codes(dev, newCodes, length);	/* first part only */
	fc_isrc_valid(dev, 1);

	fc_packets_AutoSend(dev, autoSend, ISRC1_TX);

	if (!autoSend) {
		fc_packets_ManualSend(dev, ISRC1_TX);
	}
}

void packets_AvMute(struct hdmi_tx_dev *dev, u8 enable)
{
	LOG_TRACE1(enable);
	hdmi_dev_write_mask(dev, FC_GCP, FC_GCP_SET_AVMUTE_MASK, (enable ? 1 : 0));
	hdmi_dev_write_mask(dev, FC_GCP, FC_GCP_CLEAR_AVMUTE_MASK, (enable ? 0 : 1));
}

void packets_IsrcStatus(struct hdmi_tx_dev *dev, u8 status)
{
	LOG_TRACE();
	fc_isrc_status(dev, status);
}

void packets_StopSendAcp(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, ACP_TX);
}

void packets_StopSendIsrc1(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, ISRC1_TX);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);
}

void packets_StopSendIsrc2(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_isrc_cont(dev, 0);
	fc_packets_AutoSend(dev, 0, ISRC2_TX);
}

void packets_StopSendSpd(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, SPD_TX);
}

void packets_StopSendVsd(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev, 0, VSD_TX);
}

void packets_DisableAllPackets(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();
	fc_packets_disable_all(dev);
}

int packets_VendorSpecificInfoFrame(struct hdmi_tx_dev *dev, u32 oui, const u8 * payload, u8 length, u8 autoSend)
{
	LOG_TRACE();
	fc_packets_AutoSend(dev,  0, VSD_TX);	/* prevent sending half the info. */
	fc_vsd_vendor_OUI(dev, oui);
	if (fc_vsd_vendor_payload(dev, payload, length)) {
		return FALSE;	/* DEFINE ERROR */
	}
	if (autoSend) {
		fc_packets_AutoSend(dev, autoSend, VSD_TX);
	} else {
		fc_packets_ManualSend(dev, VSD_TX);
	}
	return TRUE;
}

u8 packets_AudioMetaDataPacket(struct hdmi_tx_dev *dev, audioMetaDataPacket_t * audioMetaDataPckt)
{
	halAudioMultistream_MetaDataPacket_Header(dev, audioMetaDataPckt);
	halAudioMultistream_MetaDataPacketBody(dev, audioMetaDataPckt);
	return TRUE;
}

void packets_colorimetry_config(struct hdmi_tx_dev *dev, videoParams_t * video)
{
	u8 gamut_metadata[28] = {0};
	int gdb_color_space = 0;

	fc_gamut_enable_tx(dev, 0);

	if(video->mColorimetry == EXTENDED_COLORIMETRY){
		if(video->mExtColorimetry == XV_YCC601){
			gdb_color_space = 1;
		}
		else if(video->mExtColorimetry == XV_YCC709){
			gdb_color_space = 2;
			LOGGER(SNPS_WARN, "xv ycc709");
		}
		else if(video->mExtColorimetry == S_YCC601){
			gdb_color_space = 3;
		}
		else if(video->mExtColorimetry == ADOBE_YCC601){
			gdb_color_space = 3;
		}
		else if(video->mExtColorimetry == ADOBE_RGB){
			gdb_color_space = 3;
		}

		if(video->mColorimetryDataBlock == TRUE){
			gamut_metadata[0] = (1 << 7) | gdb_color_space;
			fc_gamut_packet_config(dev, gamut_metadata, (sizeof(gamut_metadata) / sizeof(u8)));
		}
	}
}
