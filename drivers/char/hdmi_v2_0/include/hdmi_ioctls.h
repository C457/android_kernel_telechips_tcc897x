/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_ioctls.h
*  \brief       HDMI TX controller driver
*  \details   
*  \version     1.0
*  \date        2014-2015
*  \copyright
This source code contains confidential information of Telechips.
Any unauthorized use without a written  permission  of Telechips including not 
limited to re-distribution in source  or binary  form  is strictly prohibited.
This source  code is  provided "AS IS"and nothing contained in this source 
code  shall  constitute any express  or implied warranty of any kind, including
without limitation, any warranty of merchantability, fitness for a   particular 
purpose or non-infringement  of  any  patent,  copyright  or  other third party 
intellectual property right. No warranty is made, express or implied, regarding 
the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability 
arising from, out of or in connection with this source  code or the  use in the 
source code. 
This source code is provided subject  to the  terms of a Mutual  Non-Disclosure 
Agreement between Telechips and Company.
*******************************************************************************/

#ifndef __HDMI_V2_0_H__
#define __HDMI_V2_0_H__

#if !defined(APP_BUILD)
typedef enum {
        MODE_UNDEFINED = -1,
        DVI = 0,
        HDMI
} video_mode_t;

typedef struct {
        /** VIC code */
        uint32_t mCode;

        /** Identifies modes that ONLY can be displayed in YCC 4:2:0 */
        uint8_t mLimitedToYcc420;

        /** Identifies modes that can also be displayed in YCC 4:2:0 */
        uint8_t mYcc420;

        uint16_t mPixelRepetitionInput;

        /** In units of 1MHz */
        uint32_t mPixelClock;

        /** 1 for interlaced, 0 progressive */
        uint8_t mInterlaced;

        uint16_t mHActive;

        uint16_t mHBlanking;

        uint16_t mHBorder;

        uint16_t mHImageSize;

        uint16_t mHSyncOffset;

        uint16_t mHSyncPulseWidth;

        /** 0 for Active low, 1 active high */
        uint8_t mHSyncPolarity;

        uint16_t mVActive;

        uint16_t mVBlanking;

        uint16_t mVBorder;

        uint16_t mVImageSize;

        uint16_t mVSyncOffset;

        uint16_t mVSyncPulseWidth;

        /** 0 for Active low, 1 active high */
        uint8_t mVSyncPolarity;

} dtd_t;

typedef enum {
        COLOR_DEPTH_INVALID = 0,
        COLOR_DEPTH_8 = 8,
        COLOR_DEPTH_10 = 10,
        COLOR_DEPTH_12 = 12,
        COLOR_DEPTH_16 = 16
} color_depth_t;

typedef enum {
        PIXEL_REPETITION_OFF = 0,
        PIXEL_REPETITION_1 = 1,
        PIXEL_REPETITION_2 = 2,
        PIXEL_REPETITION_3 = 3,
        PIXEL_REPETITION_4 = 4,
        PIXEL_REPETITION_5 = 5,
        PIXEL_REPETITION_6 = 6,
        PIXEL_REPETITION_7 = 7,
        PIXEL_REPETITION_8 = 8,
        PIXEL_REPETITION_9 = 9,
        PIXEL_REPETITION_10 = 10
} pixel_repetition_t;

typedef enum {
        HDMI_14 = 1,
        HDMI_20,
        MHL_24 ,
        MHL_PACKEDPIXEL
} operation_mode_t;

typedef enum {
        ENC_UNDEFINED = -1,
        RGB = 0,
        YCC444,
        YCC422,
        YCC420,
        ENC_AUTO=125,
} encoding_t;

typedef enum {
        ITU601 = 1,
        ITU709,
        EXTENDED_COLORIMETRY
} colorimetry_t;

typedef enum {
        COLORMETRY_INVALID=-1,
        XV_YCC601 = 0,
        XV_YCC709,
        S_YCC601,
        ADOBE_YCC601,
        ADOBE_RGB
} ext_colorimetry_t;

typedef struct {
        video_mode_t mHdmi;
        encoding_t mEncodingOut;
        encoding_t mEncodingIn;
        u8 mColorResolution;
        u8 mPixelRepetitionFactor;
        dtd_t mDtd;
        u8 mRgbQuantizationRange;
        u8 mPixelPackingDefaultPhase;
        u8 mColorimetry;
        u8 mScanInfo;
        u8 mActiveFormatAspectRatio;
        u8 mNonUniformScaling;
        ext_colorimetry_t mExtColorimetry;
        u8 mColorimetryDataBlock;
        u8 mItContent;
        u16 mEndTopBar;
        u16 mStartBottomBar;
        u16 mEndLeftBar;
        u16 mStartRightBar;
        u16 mCscFilter;
        u16 mCscA[4];
        u16 mCscC[4];
        u16 mCscB[4];
        u16 mCscScale;
        u8 mHdmiVideoFormat;
        u8 m3dStructure;
        u8 m3dExtData;
        u8 mHdmiVic;
        u8 mHdmi20;
} videoParams_t;

typedef enum {
	INTERFACE_NOT_DEFINED = -1, I2S = 0, SPDIF, HBR, GPA, DMA
} interfaceType_t;

typedef enum {
	PACKET_NOT_DEFINED = -1, AUDIO_SAMPLE = 1, HBR_STREAM
} packet_t;

typedef enum {
	CODING_NOT_DEFINED = -1,
	PCM = 1,
	AC3,
	MPEG1,
	MP3,
	MPEG2,
	AAC,
	DTS,
	ATRAC,
	ONE_BIT_AUDIO,
	DOLBY_DIGITAL_PLUS,
	DTS_HD,
	MAT,
	DST,
	WMAPRO
} codingType_t;

typedef enum {
	DMA_NOT_DEFINED = -1,
	DMA_4_BEAT_INCREMENT = 0,
	DMA_8_BEAT_INCREMENT,
	DMA_16_BEAT_INCREMENT,
	DMA_UNUSED_BEAT_INCREMENT,
	DMA_UNSPECIFIED_INCREMENT
} dmaIncrement_t;

/* Supplementary Audio type, table 8-14 HDMI 2.0 Spec. pg 79 */
typedef enum {
	RESERVED = 0,
	AUDIO_FOR_VIS_IMP_NARR,
	AUDIO_FOR_VIS_IMP_SPOKEN,
	AUDIO_FOR_HEAR_IMPAIRED,
	ADDITIONAL_AUDIO
} suppl_A_Type_t;

/* Audio Metadata Packet Header, table 8-4 HDMI 2.0 Spec. pg 71 */
typedef struct {
	u8 m3dAudio;
	u8 mNumViews;
	u8 mNumAudioStreams;
} audioMetaDataHeader_t;

/* Audio Metadata Descriptor, table 8-13 HDMI 2.0 Spec. pg 78 */
typedef struct {
	u8 mMultiviewRightLeft;
	u8 mLC_Valid;
	u8 mSuppl_A_Valid;
	u8 mSuppl_A_Mixed;
	suppl_A_Type_t mSuppl_A_Type;
	u8 mLanguage_Code[3];	/*ISO 639.2 alpha-3 code, examples: eng,fre,spa,por,jpn,chi */

} audioMetaDataDescriptor_t;

typedef struct {
	audioMetaDataHeader_t mAudioMetaDataHeader;
	audioMetaDataDescriptor_t mAudioMetaDataDescriptor[4];
} audioMetaDataPacket_t;

/**
 * For detailed handling of this structure,
 * refer to documentation of the functions
 */
typedef struct {
	interfaceType_t mInterfaceType;

	codingType_t mCodingType; /** (audioParams_t *params, see InfoFrame) */

	u8 mChannelAllocation; /** channel allocation (audioParams_t *params, 
						   see InfoFrame) */

	u8 mSampleSize;	/**  sample size (audioParams_t *params, 16 to 24) */

	u32 mSamplingFrequency;	/** sampling frequency (audioParams_t *params, Hz) */

	u8 mLevelShiftValue; /** level shift value (audioParams_t *params, 
						 see InfoFrame) */

	u8 mDownMixInhibitFlag;	/** down-mix inhibit flag (audioParams_t *params, 
							see InfoFrame) */

	u8 mIecCopyright; /** IEC copyright */

	u8 mIecCgmsA; /** IEC CGMS-A */

	u8 mIecPcmMode;	/** IEC PCM mode */

	u8 mIecCategoryCode; /** IEC category code */

	u8 mIecSourceNumber; /** IEC source number */

	u8 mIecClockAccuracy; /** IEC clock accuracy */

	packet_t mPacketType; /** packet type. currently only Audio Sample (AUDS) 
						  and High Bit Rate (HBR) are supported */

	u16 mClockFsFactor; /** Input audio clock Fs factor used at the audio
						packetizer to calculate the CTS value and ACR packet
						insertion rate */

	dmaIncrement_t mDmaBeatIncrement; /** Incremental burst modes: unspecified
									lengths (upper limit is 1kB boundary) and
									INCR4, INCR8, and INCR16 fixed-beat burst */

	u8 mDmaThreshold; /** When the number of samples in the Audio FIFO is lower
						than the threshold, the DMA engine requests a new burst
						request to the AHB master interface */

	u8 mDmaHlock; /** Master burst lock mechanism */

	u8 mGpaInsertPucv;	/* discard incoming (Parity, Channel status, User bit,
				   Valid and B bit) data and insert data configured in 
				   controller instead */
	audioMetaDataPacket_t mAudioMetaDataPacket; /** Audio Multistream variables, to be written to the Audio Metadata Packet */
} audioParams_t;


typedef struct {
        /* Vendor Name of eight 7-bit ASCII characters */
        u8 mVendorName[8];

        u8 mVendorNameLength;

        /* Product name or description, consists of sixteen 7-bit ASCII characters */
        u8 mProductName[16];

        u8 mProductNameLength;

        /* Code that classifies the source device (CEA Table 15) */
        u8 mSourceType;

        /* oui 24 bit IEEE Registration Identifier */
        u32 mOUI;

        u8 mVendorPayload[24];

        u8 mVendorPayloadLength;

} productParams_t;


typedef struct {
        /** Bypass encryption */
        int bypass;

        /** Enable Feature 1.1 */
        int mEnable11Feature;

        /** Check Ri every 128th frame */
        int mRiCheck;

        /** I2C fast mode */
        int mI2cFastMode;

        /** Enhanced link verification */
        int mEnhancedLinkVerification;

        /** Number of supported devices
         * (depending on instantiated KSV MEM RAM – Revocation Memory to support
         * HDCP repeaters)
         */
        u8 maxDevices;

        /** KSV List buffer
         * Shall be dimensioned to accommodate 5[bytes] x No. of supported devices
         * (depending on instantiated KSV MEM RAM – Revocation Memory to support
         * HDCP repeaters)
         * plus 8 bytes (64-bit) M0 secret value
         * plus 2 bytes Bstatus
         * Plus 20 bytes to calculate the SHA-1 (VH0-VH4)
         * Total is (30[bytes] + 5[bytes] x Number of supported devices)
         */
        u8 *mKsvListBuffer;

        /** aksv total of 14 chars**/
        u8 *mAksv;

        /** Keys list
         * 40 Keys of 14 characters each
         * stored in segments of 8 bits (2 chars)
         * **/
        u8 *mKeys;

        u8 *mSwEncKey;
} hdcpParams_t;

#endif
/**
 * Structure that interfaces with the IOCTL of the hdmi_tx. To the IOCTL
 * that support read and write parameters use this structure to get and set
 * data from and to the driver
 */
typedef struct {
        unsigned int address;
        unsigned int value;
} dwc_hdmi_ioctl_data;


/**
 * Structure that interfaces with the IOCTL of the hdmi_tx. To the IOCTL
 * that configure to hdmi phy use this structure to search valid config data.
 */
typedef struct {
        unsigned int freq_hz;
        unsigned int pixel;
}dwc_hdmi_phy_config_data;


typedef struct {
        uint16_t sfrClock;
        uint16_t ss_low_ckl;
        uint16_t ss_high_ckl;
        uint16_t fs_low_ckl;
        uint16_t fs_high_ckl;
}dwc_hdmi_ddc_config_data;

typedef struct {
        uint8_t i2cAddr;
        uint8_t segment;
        uint8_t pointer;
        uint8_t addr;
        uint8_t len;
        uint8_t *data;
}dwc_hdmi_ddc_transfer_data;


typedef struct {
        uint8_t code;
        uint32_t refreshRate;                
        dtd_t dtd;
}dwc_hdmi_dtd_data;

typedef struct {
        videoParams_t videoParam;
        audioParams_t audioParam;
        productParams_t productParam;
        hdcpParams_t hdcpParam;
}dwc_hdmi_api_data;


#define             IOCTL_HDMI_MAGIC            'H'
/**
 * IOCTL defines
 */
 
/**
 * @short IOCTL to read a byte from HDMI TX CORE
 * - fb_data->address -> address to read
 * - fb_data->value -> return value
 */
#define             FB_HDMI_CORE_READ              _IO( IOCTL_HDMI_MAGIC, 0)

/**
 * @short IOCTL to write a byte to HDMI TX CORE
 * fb_data->address -> address to write to
 * fb_data->value -> value to write
 */
#define             FB_HDMI_CORE_WRITE             _IO( IOCTL_HDMI_MAGIC, 1)

/**
 * @short IOCTL to read a byte from HDMI TX HDCP
 * - fb_data->address -> address to read
 * - fb_data->value -> return value
 */
#define             FB_HDMI_HDCP_READ              _IO( IOCTL_HDMI_MAGIC, 2)

/**
 * @short IOCTL to write a byte to HDMI TX HDCP
 * fb_data->address -> address to write to
 * fb_data->value -> value to write
 */
#define             FB_HDMI_HDCP_WRITE             _IO( IOCTL_HDMI_MAGIC, 3)

/**
 * @short IOCTL to read a byte from HDMI TX PHY INTERFACE
 * - fb_data->address -> address to read
 * - fb_data->value -> return value
 */
#define             FB_HDMI_PHY_IF_READ            _IO( IOCTL_HDMI_MAGIC, 4)

/**
 * @short IOCTL to write a byte to HDMI TX PHY INTERFACE
 * fb_data->address -> address to write to
 * fb_data->value -> value to write
 */
#define             FB_HDMI_PHY_IF_WRITE           _IO( IOCTL_HDMI_MAGIC, 5)




/**
 * @short IOCTL to get the device base address
 * fb_data->value -> base address
 */
#define HDMI_GET_DTD_INFO               _IOR( IOCTL_HDMI_MAGIC, 0x201, dwc_hdmi_dtd_data)

#define FB_HDMI_SET_HPD                 _IO( IOCTL_HDMI_MAGIC, 0x202)

#define FB_HDMI_GET_HDCP22              _IO( IOCTL_HDMI_MAGIC, 0x203)

#define FB_HDMI_FLUSH_SIG               _IO( IOCTL_HDMI_MAGIC, 0x204)



/**
 * @short IOCTL to write a byte to HDMI TX PHY INTERFACE
 * fb_data->address -> address to write to
 * fb_data->value -> value to write
 */
#define HDMI_PHY_SET_CONFIG                     _IOW( IOCTL_HDMI_MAGIC, 0x211, dwc_hdmi_phy_config_data)

#define HDMI_DDC_SET_CLK_CONFIG                 _IOW( IOCTL_HDMI_MAGIC, 0x221, dwc_hdmi_ddc_config_data)
#define HDMI_DDC_READ_DATA                      _IOR( IOCTL_HDMI_MAGIC, 0x222, dwc_hdmi_ddc_transfer_data)
#define HDMI_DDC_WRITE_DATA                     _IOW( IOCTL_HDMI_MAGIC, 0x223, dwc_hdmi_ddc_transfer_data)

#define HDMI_VIDEO_CONFIG                       _IOW( IOCTL_HDMI_MAGIC, 0x231, videoParams_t)
#define HDMI_VIDEO_SET_SCRAMBLING               _IOW( IOCTL_HDMI_MAGIC, 0x232, int)
#define HDMI_VIDEO_SET_TMDS_CLOCK_RATIO         _IOW( IOCTL_HDMI_MAGIC, 0x233, int)


/**
 * @short IOCTL to control HDMI Audio Interface
 * audioParams_t -> set parameters for HDMI Audio.
 */

#define HDMI_AUDIO_INIT               _IO( IOCTL_HDMI_MAGIC, 0x240)
#define HDMI_AUDIO_CONFIG             _IOW( IOCTL_HDMI_MAGIC, 0x241, audioParams_t)


/**
 * @short IOCTL to control HDMI HPD Interface
 */

#define HDMI_HPD_SET_ENABLE           _IO( IOCTL_HDMI_MAGIC, 0x250)
#define HDMI_HPD_GET_STATUS          _IOR( IOCTL_HDMI_MAGIC, 0x251, int)



#define HDMI_PRODUCT_SET_VPAYLOAD_DATA          _IO( IOCTL_HDMI_MAGIC, 0x260)
#define HDMI_PRODUCT_GET_VPAYLOAD_DATA          _IO( IOCTL_HDMI_MAGIC, 0x261)

#define HDMI_PRODUCT_SET_VPAYLOAD_LENGTH        _IO( IOCTL_HDMI_MAGIC, 0x262)
#define HDMI_PRODUCT_GET_VPAYLOAD_LENGTH        _IO( IOCTL_HDMI_MAGIC, 0x263)


#define HDMI_PACKET_CONFIG                      _IO( IOCTL_HDMI_MAGIC, 270)

#define HDMI_SET_AV_MUTE                        _IOW( IOCTL_HDMI_MAGIC, 271, unsigned int)

#define HDMI_API_CONFIG                         _IOW( IOCTL_HDMI_MAGIC, 280, dwc_hdmi_api_data)



/**
 * Interrupt signals
 */
#define SIG_DWC_HDMI_TX SIGUSR1
#define SIG_VID_BRIDGE  34
#define SIG_AUD_BRIDGE  35
#define SIG_TXPHY_IF    36
#define SIG_HDCP        37
#define SIG_CEC_DETECT  38
#define SIG_EDID        39


#endif  // __HDMI_V2_0_H__



