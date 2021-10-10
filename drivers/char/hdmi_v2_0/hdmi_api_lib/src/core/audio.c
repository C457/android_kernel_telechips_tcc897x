/*
 * @file:audio.c
 *
 *  Created on: Jul 2, 2010
 *  Modified: Oct 2010: GPA interface added
 *  Modified: May 2011: DMA added
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"

#include "../../include/core/audio.h"
#include "../../include/core/fc_audio.h"
#include "../../include/core/fc_audio_info.h"
#include "../../include/core/audio_i2s.h"
#include "../../include/core/audio_spdif.h"
#include "../../include/core/main_controller.h"

#include "audio/audio_packetizer_reg.h"

#include <linux/clk.h> 
#include <dt-bindings/clock/tcc898x-clks.h>

typedef struct audio_n_computation {
	u32 pixel_clock;
	u32 n;
}audio_n_computation_t;

audio_n_computation_t n_values_32[] = {
	{0,     4096},
	{25175, 4576},
	{25200, 4096},
	{27000, 4096},
	{27027, 4096},
	{54000, 4096},
	{54054, 4096},
	{74176,11648},
	{74250, 4096},
	{148352,11648},
	{148500, 4096},
	{296703,5824},
	{297000, 3072},
	{0, 0}
};

audio_n_computation_t n_values_44p1[] = {
	{0,     6272},
	{25175, 7007},
	{25200, 6272},
	{27000, 6272},
	{27027, 6272},
	{54000, 6272},
	{54054, 6272},
	{74176,17836},
	{74250, 6272},
	{148352,8918},
	{148500, 6272},
	{296703,4459},
	{297000, 4704},
	{0, 0}
};

audio_n_computation_t n_values_48[] = {
	{0,     6144},
	{25175, 6864},
	{25200, 6144},
	{27000, 6144},
	{27027, 6144},
	{54000, 6144},
	{54054, 6144},
	{74176,11648},
	{74250, 6144},
	{148352,5824},
	{148500, 6144},
	{296703,5824},
	{297000, 5120},
	{0, 0}
};


/**********************************************
 * Internal functions
 */
void _audio_clock_n(struct hdmi_tx_dev *dev, u32 value)
{
	LOG_TRACE();
	/* 19-bit width */
	hdmi_dev_write(dev, AUD_N1, (u8)(value >> 0));
	hdmi_dev_write(dev, AUD_N2, (u8)(value >> 8));
	hdmi_dev_write_mask(dev, AUD_N3, AUD_N3_AUDN_MASK, (u8)(value >> 16));
	/* no shift */
	hdmi_dev_write_mask(dev, AUD_CTS3, AUD_CTS3_N_SHIFT_MASK, 0);
}

void _audio_clock_cts(struct hdmi_tx_dev *dev, u32 value)
{
	LOG_TRACE1(value);
	if(value > 0){
		/* 19-bit width */
		hdmi_dev_write(dev, AUD_CTS1, (u8)(value >> 0));
		hdmi_dev_write(dev, AUD_CTS2, (u8)(value >> 8));
		hdmi_dev_write_mask(dev, AUD_CTS3, AUD_CTS3_AUDCTS_MASK, (u8)(value >> 16));
		hdmi_dev_write_mask(dev, AUD_CTS3, AUD_CTS3_CTS_MANUAL_MASK, 1);
	}
	else{
		/* Set to automatic generation of CTS values */
		hdmi_dev_write_mask(dev, AUD_CTS3, AUD_CTS3_CTS_MANUAL_MASK, 0);
	}
}

void _audio_clock_f(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE();
	hdmi_dev_write_mask(dev, AUD_INPUTCLKFS, AUD_INPUTCLKFS_IFSFACTOR_MASK, value);
}

int configure_i2s(struct hdmi_tx_dev *dev, audioParams_t * audio)
{
	audio_i2s_configure(dev, audio);

	switch (audio->mClockFsFactor) {
	case 64:
		_audio_clock_f(dev, 4);
		break;
	case 128:
		_audio_clock_f(dev, 0);
		break;
	case 256:
		_audio_clock_f(dev, 1);
		break;
	case 512:
		_audio_clock_f(dev, 2);
		break;
	default:
		_audio_clock_f(dev, 0);
		LOGGER(SNPS_ERROR, "%s:Fs Factor [%d] not supported\n", __func__, audio->mClockFsFactor);
		return FALSE;
		break;
	}

	return TRUE;
}

int configure_spdif(struct hdmi_tx_dev *dev, audioParams_t * audio)
{
	audio_spdif_config(dev, audio);

	switch (audio->mClockFsFactor) {
	case 64:
		_audio_clock_f(dev, 0);
		break;
	case 128:
		_audio_clock_f(dev, 0);
		break;
	case 256:
		_audio_clock_f(dev, 0);
		break;
	case 512:
		_audio_clock_f(dev, 2);
		break;
	default:
		_audio_clock_f(dev, 0);
		LOGGER(SNPS_ERROR, "%s:Fs Factor [%d] not supported\n", __func__, audio->mClockFsFactor);
		return FALSE;
		break;
	}

	return TRUE;
}

void tcc_hdmi_audio_select_source(struct hdmi_tx_dev *dev, audioParams_t * audio)
{
	volatile void __iomem *tcc_io_bus = (volatile void __iomem *)dev->io_bus;

    iowrite32(audio->mIecSourceNumber, (void*)(tcc_io_bus + dev->audio_offset));	

}

extern int tcc_ckc_set_hdmi_audio_src(unsigned int src_id);
void tcc_hdmi_spdif_clock_config(struct hdmi_tx_dev *dev, audioParams_t * audio)
{
    unsigned int clk_rate;
	
	if(dev->clk[HDMI_CLK_INDEX_SPDIF])
	{
		clk_rate = audio->mSamplingFrequency*audio->mClockFsFactor;
	    clk_disable_unprepare(dev->clk[HDMI_CLK_INDEX_SPDIF]);
		
		if(audio->mIecSourceNumber)
		{
			tcc_ckc_set_hdmi_audio_src(PERI_MSPDIF1);
			printk("%s : clock config is spdif1 \n",__func__);
		}
		else
		{
			tcc_ckc_set_hdmi_audio_src(PERI_MSPDIF0);
			printk("%s : clock config is spdif0 \n",__func__);			
		}
		
	    clk_set_rate(dev->clk[HDMI_CLK_INDEX_SPDIF],clk_rate);
	    clk_prepare_enable(dev->clk[HDMI_CLK_INDEX_SPDIF]);
		
		printk("%s: HDMI SPDIF clk =  %dHz\r\n", FUNC_NAME, (int)clk_get_rate(dev->clk[HDMI_CLK_INDEX_SPDIF]));
	}
}


/**********************************************
 * External functions
 */
int audio_Initialize(struct hdmi_tx_dev *dev)
{
	LOG_TRACE();

	// Mask all interrupts
	audio_i2s_interrupt_mask(dev, 0x3);
	audio_spdif_interrupt_mask(dev, 0x3);

	return audio_mute(dev,  1);
}

int audio_Configure(struct hdmi_tx_dev *dev, audioParams_t * audio)
{
	u32 n = 0;

	LOG_TRACE();

	if((dev == NULL) || (audio == NULL)){
		LOGGER(SNPS_ERROR, "Improper function arguments\n");
		return FALSE;
	}


	if(dev->hdmi_tx_ctrl.audio_on == 0){
		LOGGER(SNPS_WARN, "DVI mode selected: audio not configured");
		return TRUE;
	}

	LOGGER(SNPS_DEBUG, "Audio interface type = %s\n", audio->mInterfaceType == I2S ? "I2S" :
													audio->mInterfaceType == SPDIF ? "SPDIF" :
													audio->mInterfaceType == HBR ? "HBR" :
													audio->mInterfaceType == GPA ? "GPA" :
													audio->mInterfaceType == DMA ? "DMA" : "---");

	LOGGER(SNPS_DEBUG, "Audio coding = %s\n", audio->mCodingType == PCM ? "PCM" :
								   audio->mCodingType == AC3 ? "AC3" :
								   audio->mCodingType == MPEG1 ? "MPEG1" :
								   audio->mCodingType == MP3 ? "MP3" :
								   audio->mCodingType == MPEG2 ? "MPEG2" :
								   audio->mCodingType == AAC ? "AAC" :
								   audio->mCodingType == DTS ? "DTS" :
								   audio->mCodingType == ATRAC ? "ATRAC" :
								   audio->mCodingType == ONE_BIT_AUDIO ? "ONE BIT AUDIO" :
								   audio->mCodingType == DOLBY_DIGITAL_PLUS ? "DOLBY DIGITAL +" :
								   audio->mCodingType == DTS_HD ? "DTS HD" : "----");
	LOGGER(SNPS_DEBUG, "Audio frequency = %.3d Hz\n", audio->mSamplingFrequency);
	LOGGER(SNPS_DEBUG, "Audio sample size = %d\n", audio->mSampleSize);
	LOGGER(SNPS_DEBUG, "Audio FS factor = %d\n", audio->mClockFsFactor);

	// Set PCUV info from external source
	audio->mGpaInsertPucv = 1;

	audio_mute(dev, 1);

	tcc_hdmi_audio_select_source(dev,audio);

	// Configure Frame Composer audio parameters
	fc_audio_config(dev, audio);

	if(audio->mInterfaceType == I2S){
		if(configure_i2s(dev, audio) == FALSE){
			LOGGER(SNPS_ERROR, "Configuring I2S audio\n");
			return FALSE;
		}
	}
	else if(audio->mInterfaceType == SPDIF){
		if(configure_spdif(dev, audio) == FALSE){
			LOGGER(SNPS_ERROR, "Configuring SPDIF audio\n");
			return FALSE;
		}
	}
	else if(audio->mInterfaceType == HBR){
		LOGGER(SNPS_ERROR, "HBR is not supported\n");
		return FALSE;
	}
	else if (audio->mInterfaceType == GPA) {
		LOGGER(SNPS_ERROR, "GPA is not supported\n");
		return FALSE;
	}

	else if (audio->mInterfaceType == DMA) {
		LOGGER(SNPS_ERROR, "DMA is not supported\n");
		return FALSE;
	}

	n = audio_ComputeN(dev, audio->mSamplingFrequency, dev->hdmi_tx_ctrl.pixel_clock);

	_audio_clock_n(dev, n);

	_audio_clock_cts(dev, 0);

	mc_audio_sampler_clock_enable(dev, 0);

	audio_mute(dev, 0);

	// Configure audio info frame packets
	fc_audio_info_config(dev, audio);

	if(audio->mInterfaceType == SPDIF)
		tcc_hdmi_spdif_clock_config(dev, audio);

	return TRUE;
}

int audio_mute(struct hdmi_tx_dev *dev, u8 state)
{
	/* audio mute priority: AVMUTE, sample flat, validity */
	/* AVMUTE also mutes video */
	// TODO: Check the audio mute process
	if(state){
		fc_audio_mute(dev);
	}
	else{
		fc_audio_unmute(dev);
	}
	return TRUE;
}

u32 audio_ComputeN(struct hdmi_tx_dev *dev, u32  freq, u32 pixelClk)
{
	int i = 0;
	u32 n = 0;
	audio_n_computation_t *n_values = NULL;
	int multiplier_factor = 1;

	if(freq == 64000 || freq == 88200 || freq == 96000){
		multiplier_factor = 2;
	}
	else if(freq == 128000 || freq == 176400 || freq == 192000){
		multiplier_factor = 4;
	}
	else if(freq == 256000 || freq == 352800 || freq== 384000){
		multiplier_factor = 8;
	}

	if(32000 == freq/multiplier_factor){
		n_values = n_values_32;
	}
	else if(44100 == freq/multiplier_factor){
		n_values = n_values_44p1;
	}
	else if(48000 == freq/multiplier_factor){
		n_values = n_values_48;
	}
	else {
		LOGGER(SNPS_ERROR, "%s:Could not compute N values\n", __func__);
		LOGGER(SNPS_ERROR, "%s:Audio Frequency %d\n", __func__, freq);
		LOGGER(SNPS_ERROR, "%s:Pixel Clock %d\n", __func__, pixelClk);
		LOGGER(SNPS_ERROR, "%s:Multiplier factor %d\n", __func__, multiplier_factor);
		return FALSE;
	}

	for(i = 0; n_values[i].n != 0; i++){
		if(pixelClk == n_values[i].pixel_clock){
			n = n_values[i].n * multiplier_factor;
			LOGGER(SNPS_DEBUG, "Audio N value = %d\n", n);
			return n;
		}
	}

	n = n_values[0].n * multiplier_factor;

	LOGGER(SNPS_DEBUG, "Audio N value default = %d\n", n);

	return n;
}

u32 audio_ComputeCts(struct hdmi_tx_dev *dev, u32 freq, u32 pixelClk, u16 ratioClk)
{
	u32 cts = 0;

	uint32_t temp_freq = (uint32_t)freq;
	uint32_t temp_pixel_clk = (uint32_t)pixelClk;

	cts = temp_pixel_clk * audio_ComputeN(dev, freq, pixelClk);
	cts = cts / 128;
	cts = (cts * 100) / (temp_freq / 100);
	cts = cts * ratioClk;
	cts = cts / 100;
	if ((temp_pixel_clk * (ratioClk / 100)) > 34000) {
		cts = cts * 4;
	}
	LOGGER(SNPS_DEBUG, "audio_ComputeCtsEquation CTS = %d\n", cts);
	return cts;
}
