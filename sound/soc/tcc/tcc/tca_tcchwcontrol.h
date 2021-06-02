/****************************************************************************
 *   FileName    : tca_tcchwcontrol.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
#ifndef __TCA_TCCHWCONTROL_H__
#define __TCA_TCCHWCONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
*
* Defines
*
******************************************************************************/

//#define	AUDIO_DMA_PAGE_SIZE		(256)					// Size in bytes
//#define	AUDIO_DMA_IN_PAGE_SIZE	(256)					// Size in bytes

//#define SPDIF_24BIT_MODE // SPDIF_24BIT_MODE Planet_20140408
//#define I2S_24BIT_MODE We do not use them

//i2s control api
extern unsigned int tca_i2s_dai_init(void *pADMADAIBaseAddr);
extern unsigned int tca_i2s_deinit(void *pADMADAIBaseAddr);
extern unsigned int tca_i2s_setregister(void *pADMADAIBaseAddr, unsigned int nRegval);
extern unsigned int tca_i2s_getregister(void *pADMADAIBaseAddr);

extern void tca_i2s_start(void *pADMADAIBaseAddr, unsigned int nMode);
extern void tca_i2s_stop(void *pADMADAIBaseAddr, unsigned int nMode);

extern void tca_i2s_initoutput(void *pADMABaseAddr, unsigned int,unsigned int,unsigned int);
extern void tca_i2s_initinput(void *pADMABaseAddr, unsigned int,unsigned int,unsigned int);

//Get IRQ GetNumber
extern unsigned int tca_irq_getnumber(void);

//dma control api
extern unsigned int tca_dma_clrstatus(void *pADMABaseAddr, unsigned int nDmanum);
extern unsigned int tca_dma_getstatus(void *pADMABaseAddr, unsigned int nDmanum);
extern unsigned int tca_dma_control(void *pADMABaseAddr, void *pADMADAIBaseAddr, unsigned int nMode, unsigned int nDmanum, unsigned int nInMode);
extern unsigned int tca_dma_setsrcaddr(void *pADMABaseAddr, unsigned int DADONum, unsigned int nDmanum, unsigned int nAddr);
extern unsigned int tca_dma_setdestaddr(void *pADMABaseAddr, unsigned int DADINum, unsigned int nDmanum, unsigned int nAddr);

//GPIO port INIT
extern unsigned int tca_tcc_initport(void);
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
extern unsigned int tca_tcc_DAO_mute(bool enable);
extern void tca_tcc_DAI_mute(int index, int chs, bool enable);
#else
extern void tca_tcc_M0DAI_mute(int index, int chs, bool enable);
extern void tca_tcc_M1DAI_mute(int index, int chs, bool enable);
#endif
#if defined(CONFIG_ARCH_TCC898X)
extern void tca_i2s_port_mux(int id, int port);
extern void tca_spdif_port_mux(int id, int port);
#elif defined(CONFIG_ARCH_TCC802X)
struct audio_i2s_port {
    char clk[3];
    char daout[4];
    char dain[4];
};
extern void tca_i2s_port_mux(int id, struct audio_i2s_port *port);
extern void tca_spdif_port_mux(int id, char *port);
#endif
extern void tca_audio_reset(int index);
extern void tca_iobus_dump(void);
extern void tca_adma_dump(void *pADMABaseAddr);
extern void tca_dai_dump(void *pDAIBaseAddr);
extern void tca_spdif_dump(void *pSPDIFBaseAddr);

extern int line_out_mute; // This is for H/W mute
#ifdef __cplusplus
}
#endif
#endif

