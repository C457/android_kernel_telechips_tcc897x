/*
 * @file:videoParams.c
 *
 *  Created on: Jul 2, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"
#include "../../../include/video_params.h"


int videoParams_GetCeaVicCode(int hdmi_vic_code)
{
	switch(hdmi_vic_code)
	{
	case 1:
		return 95;
		break;
	case 2:
		return 94;
		break;
	case 3:
		return 93;
		break;
	case 4:
		return 98;
		break;
	default:
		return -1;
		break;
	}
	return -1;
}

int videoParams_GetHdmiVicCode(int cea_code)
{
	switch(cea_code)
	{
	case 95:
		return 1;
		break;
	case 94:
		return 2;
		break;
	case 93:
		return 3;
		break;
	case 98:
		return 4;
		break;
	default:
		return -1;
		break;
	}
	return -1;
}

void video_params_reset(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	dtd_t dtd;
	hdmi_dtd_fill(&dtd, 1, 60000);

	params->mHdmi = DVI;
	params->mEncodingOut = RGB;
	params->mEncodingIn = RGB;
	params->mColorResolution = COLOR_DEPTH_8;
	params->mPixelRepetitionFactor = 0;
	params->mRgbQuantizationRange = 0;
	params->mPixelPackingDefaultPhase = 0;
	params->mColorimetry = 0;
	params->mScanInfo = 0;
	params->mActiveFormatAspectRatio = 8;
	params->mNonUniformScaling = 0;
	params->mExtColorimetry = ~0;
	params->mItContent = 0;
	params->mEndTopBar = ~0;
	params->mStartBottomBar = ~0;
	params->mEndLeftBar = ~0;
	params->mStartRightBar = ~0;
	params->mCscFilter = 0;
	params->mHdmiVideoFormat = 0;
	params->m3dStructure = 0;
	params->m3dExtData = 0;
	params->mHdmiVic = 0;
	params->mHdmi20 = 0;

	memcpy(&params->mDtd, &dtd, sizeof(dtd_t));
}

u16 *videoParams_GetCscA(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	videoParams_UpdateCscCoefficients(dev, params);
	return params->mCscA;
}

void videoParams_SetCscA(struct hdmi_tx_dev *dev, videoParams_t * params, u16 value[4])
{
	u16 i = 0;
	for (i = 0; i < sizeof(params->mCscA) / sizeof(params->mCscA[0]); i++) {
		params->mCscA[i] = value[i];
	}
}

u16 *videoParams_GetCscB(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	videoParams_UpdateCscCoefficients(dev, params);
	return params->mCscB;
}

void videoParams_SetCscB(struct hdmi_tx_dev *dev, videoParams_t * params, u16 value[4])
{
	u16 i = 0;
	for (i = 0; i < sizeof(params->mCscB) / sizeof(params->mCscB[0]); i++) {
		params->mCscB[i] = value[i];
	}
}

u16 *videoParams_GetCscC(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	videoParams_UpdateCscCoefficients(dev, params);
	return params->mCscC;
}

void videoParams_SetCscC(struct hdmi_tx_dev *dev, videoParams_t * params, u16 value[4])
{
	u16 i = 0;
	for (i = 0; i < sizeof(params->mCscC) / sizeof(params->mCscC[0]); i++) {
		params->mCscC[i] = value[i];
	}
}

void videoParams_SetCscScale(struct hdmi_tx_dev *dev, videoParams_t * params, u16 value)
{
	params->mCscScale = value;
}

/* [0.01 MHz] */
u32 videoParams_GetPixelClock(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	if (params->mHdmiVideoFormat == 2) {
		if (params->m3dStructure == 0) {
			return (2 * params->mDtd.mPixelClock)*1000;
		}
	}
	return params->mDtd.mPixelClock*1000;
}

/* 0.01 */
unsigned videoParams_GetRatioClock(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	unsigned ratio = 100;

	if (params->mEncodingOut != YCC422) {
		if (params->mColorResolution == 8) {
			ratio = 100;
		} else if (params->mColorResolution == 10) {
			ratio = 125;
		} else if (params->mColorResolution == 12) {
			ratio = 150;
		} else if (params->mColorResolution == 16) {
			ratio = 200;
		}
	}
	return ratio * (params->mPixelRepetitionFactor + 1);
}

int videoParams_IsColorSpaceConversion(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	return params->mEncodingIn != params->mEncodingOut;
}

int videoParams_IsColorSpaceDecimation(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	return params->mEncodingOut == YCC422 && (params->mEncodingIn == RGB
			|| params->mEncodingIn ==
					YCC444);
}

int videoParams_IsColorSpaceInterpolation(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	return params->mEncodingIn == YCC422 && (params->mEncodingOut == RGB
			|| params->mEncodingOut ==
					YCC444);
}

int videoParams_IsPixelRepetition(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	return (params->mPixelRepetitionFactor > 0) || (params->mDtd.mPixelRepetitionInput > 0);
}

void videoParams_UpdateCscCoefficients(struct hdmi_tx_dev *dev, videoParams_t * params)
{
	u16 i = 0;
	if (!videoParams_IsColorSpaceConversion(dev, params)) {
		for (i = 0; i < 4; i++) {
			params->mCscA[i] = 0;
			params->mCscB[i] = 0;
			params->mCscC[i] = 0;
		}
		params->mCscA[0] = 0x2000;
		params->mCscB[1] = 0x2000;
		params->mCscC[2] = 0x2000;
		params->mCscScale = 1;
	} else if (videoParams_IsColorSpaceConversion(dev, params) && params->mEncodingOut == RGB) {
		if (params->mColorimetry == ITU601) {
			params->mCscA[0] = 0x2000;
			params->mCscA[1] = 0x6926;
			params->mCscA[2] = 0x74fd;
			params->mCscA[3] = 0x010e;

			params->mCscB[0] = 0x2000;
			params->mCscB[1] = 0x2cdd;
			params->mCscB[2] = 0x0000;
			params->mCscB[3] = 0x7e9a;

			params->mCscC[0] = 0x2000;
			params->mCscC[1] = 0x0000;
			params->mCscC[2] = 0x38b4;
			params->mCscC[3] = 0x7e3b;

			params->mCscScale = 1;
		} else if (params->mColorimetry == ITU709) {
			params->mCscA[0] = 0x2000;
			params->mCscA[1] = 0x7106;
			params->mCscA[2] = 0x7a02;
			params->mCscA[3] = 0x00a7;

			params->mCscB[0] = 0x2000;
			params->mCscB[1] = 0x3264;
			params->mCscB[2] = 0x0000;
			params->mCscB[3] = 0x7e6d;

			params->mCscC[0] = 0x2000;
			params->mCscC[1] = 0x0000;
			params->mCscC[2] = 0x3b61;
			params->mCscC[3] = 0x7e25;

			params->mCscScale = 1;
		}
	} else if (videoParams_IsColorSpaceConversion(dev, params) && params->mEncodingIn == RGB) {
		if (params->mColorimetry == ITU601) {
			params->mCscA[0] = 0x2591;
			params->mCscA[1] = 0x1322;
			params->mCscA[2] = 0x074b;
			params->mCscA[3] = 0x0000;

			params->mCscB[0] = 0x6535;
			params->mCscB[1] = 0x2000;
			params->mCscB[2] = 0x7acc;
			params->mCscB[3] = 0x0200;

			params->mCscC[0] = 0x6acd;
			params->mCscC[1] = 0x7534;
			params->mCscC[2] = 0x2000;
			params->mCscC[3] = 0x0200;

			params->mCscScale = 0;
		} else if (params->mColorimetry == ITU709) {
			params->mCscA[0] = 0x2dc5;
			params->mCscA[1] = 0x0d9b;
			params->mCscA[2] = 0x049e;
			params->mCscA[3] = 0x0000;

			params->mCscB[0] = 0x62f0;
			params->mCscB[1] = 0x2000;
			params->mCscB[2] = 0x7d11;
			params->mCscB[3] = 0x0200;

			params->mCscC[0] = 0x6756;
			params->mCscC[1] = 0x78ab;
			params->mCscC[2] = 0x2000;
			params->mCscC[3] = 0x0200;

			params->mCscScale = 0;
		}
	}
	/* else use user coefficients */
}

/*
void videoParams_SetYcc420Support(hdmi_tx_dev_t *dev, dtd_t * paramsDtd, shortVideoDesc_t * paramsSvd)
{
	paramsDtd->mLimitedToYcc420 = paramsSvd->mLimitedToYcc420;
	paramsDtd->mYcc420 = paramsSvd->mYcc420;
	LOGGER(SNPS_DEBUG,"set ParamsDtd->limite %d", paramsDtd->mLimitedToYcc420);
	LOGGER(SNPS_DEBUG,"set ParamsDtd->supports %d", paramsDtd->mYcc420);
	LOGGER(SNPS_DEBUG,"set Paramssvd->limite %d", paramsSvd->mLimitedToYcc420);
	LOGGER(SNPS_DEBUG,"set ParamsSvd->supports %d", paramsSvd->mYcc420);
}
*/


char * getEncodingString(encoding_t encoding)
{
	switch (encoding){
		case 	RGB: return "RGB";
		case	YCC444: return "YCbCr-444";
		case	YCC422: return "YCbCr-422";
		case	YCC420: return "YCbCr-420";
		default :break;
	}
	return "Undefined";
}
