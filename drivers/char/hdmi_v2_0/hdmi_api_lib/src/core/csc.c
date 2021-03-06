/*
 * halColorSpaceConverter.c
 *
 *  Created on: Jun 30, 2010
 *  Synopsys Inc.
 *  SG DWC PT02
 */
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"

#include "color_space/color_space_reg.h"
#include "../../include/core/csc.h"


void csc_Interpolation(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	/* 2-bit width */
	hdmi_dev_write_mask(dev, CSC_CFG, CSC_CFG_INTMODE_MASK, value);
}

void csc_Decimation(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	/* 2-bit width */
	hdmi_dev_write_mask(dev, CSC_CFG, CSC_CFG_DECMODE_MASK, value);
}

void csc_ColorDepth(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	/* 4-bit width */
	hdmi_dev_write_mask(dev, CSC_SCALE, CSC_SCALE_CSC_COLOR_DEPTH_MASK, value);
}

void csc_ScaleFactor(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	/* 2-bit width */
	hdmi_dev_write_mask(dev, CSC_SCALE, CSC_SCALE_CSCSCALE_MASK, value);
}

void csc_CoefficientA1(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_A1_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_A1_MSB, CSC_COEF_A1_MSB_CSC_COEF_A1_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientA2(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_A2_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_A2_MSB, CSC_COEF_A2_MSB_CSC_COEF_A2_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientA3(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_A3_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_A3_MSB, CSC_COEF_A3_MSB_CSC_COEF_A3_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientA4(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_A4_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_A4_MSB, CSC_COEF_A4_MSB_CSC_COEF_A4_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientB1(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_B1_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_B1_MSB, CSC_COEF_B1_MSB_CSC_COEF_B1_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientB2(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_B2_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_B2_MSB, CSC_COEF_B2_MSB_CSC_COEF_B2_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientB3(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_B3_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_B3_MSB, CSC_COEF_B3_MSB_CSC_COEF_B3_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientB4(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_B4_LSB, (u8)(value));
	hdmi_dev_write_mask(dev, CSC_COEF_B4_MSB, CSC_COEF_B4_MSB_CSC_COEF_B4_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientC1(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_C1_LSB, (u8) (value));
	hdmi_dev_write_mask(dev, CSC_COEF_C1_MSB, CSC_COEF_C1_MSB_CSC_COEF_C1_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientC2(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_C2_LSB, (u8) (value));
	hdmi_dev_write_mask(dev, CSC_COEF_C2_MSB, CSC_COEF_C2_MSB_CSC_COEF_C2_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientC3(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	/* 15-bit width */
	hdmi_dev_write(dev, CSC_COEF_C3_LSB, (u8) (value));
	hdmi_dev_write_mask(dev, CSC_COEF_C3_MSB, CSC_COEF_C3_MSB_CSC_COEF_C3_MSB_MASK, (u8)(value >> 8));
}

void csc_CoefficientC4(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE1(value);
	hdmi_dev_write(dev, CSC_COEF_C4_LSB, (u8) (value));
	hdmi_dev_write_mask(dev, CSC_COEF_C4_MSB, CSC_COEF_C4_MSB_CSC_COEF_C4_MSB_MASK, (u8)(value >> 8));
}

void csc_config(struct hdmi_tx_dev *dev, videoParams_t * video,
		unsigned interpolation, unsigned decimation, unsigned color_depth)
{
	csc_Interpolation(dev, interpolation);
	csc_Decimation(dev, decimation);
	csc_CoefficientA1(dev, video->mCscA[0]);
	csc_CoefficientA2(dev, video->mCscA[1]);
	csc_CoefficientA3(dev, video->mCscA[2]);
	csc_CoefficientA4(dev, video->mCscA[3]);
	csc_CoefficientB1(dev, video->mCscB[0]);
	csc_CoefficientB2(dev, video->mCscB[1]);
	csc_CoefficientB3(dev, video->mCscB[2]);
	csc_CoefficientB4(dev, video->mCscB[3]);
	csc_CoefficientC1(dev, video->mCscC[0]);
	csc_CoefficientC2(dev, video->mCscC[1]);
	csc_CoefficientC3(dev, video->mCscC[2]);
	csc_CoefficientC4(dev, video->mCscC[3]);
	csc_ScaleFactor(dev, video->mCscScale);
	csc_ColorDepth(dev, color_depth);
}
