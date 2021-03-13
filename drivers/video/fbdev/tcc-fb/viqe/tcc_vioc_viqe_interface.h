/****************************************************************************
linux/drivers/video/tcc/viqe/tcc_vioc_viqe_interface.h
Description: TCC VIOC h/w block 

Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#ifndef TCC_VIOC_VIQE_INTERFACE_H_INCLUDED
#define TCC_VIOC_VIQE_INTERFACE_H_INCLUDED

#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/tcc_viqe_ioctl.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tcc_fb.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_viqe.h>
#else
#include <video/tcc/tcc_viqe_ioctl.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/vioc_ireq.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_viqe.h>
#endif

struct tcc_viqe_common_virt_addr_info_t{
	VIQE *pVIQE0;
	VIQE *pVIQE1;
	VIOC_IREQ_CONFIG *pIREQConfig;
	int gVIOC_Deintls;
	int gVIOC_VIQE0;
	int gVIOC_VIQE1;
	int gBoard_num;
};

struct tcc_viqe_m2m_virt_addr_info_t{
	VIOC_RDMA *pRDMABase_m2m;
	union {
		int gVIQE_RDMA_num_m2m;
		int gDEINTLS_RDMA_num_m2m;
	};
};

struct tcc_viqe_60hz_virt_addr_info_t{
	VIOC_RDMA *pRDMABase_60Hz;
	VIOC_SC *pSCALERBase_60Hz;
	VIOC_WMIX *pWMIXBase_60Hz;
	VIOC_DISP *pDISPBase_60Hz;
	int gVIQE_RDMA_num_60Hz;
	int gSCALER_num_60Hz;
	int gSC_RDMA_num_60Hz;
	int gWMIX_RDMA;
};

struct tcc_viqe_m2m_scaler_data {
        unsigned char           irq_reged;
        unsigned int            dev_opened;
};

struct tcc_viqe_m2m_scaler_type_t {
        struct vioc_intr_type   *vioc_intr;

        unsigned int            id;
        unsigned int            irq;

        struct vioc_rdma_device *rdma;
        struct vioc_wmix_device *wmix;
        struct vioc_sc_device   *sc;
        struct vioc_wdma_device *wdma;

        struct tcc_viqe_m2m_scaler_data      *data;
        struct tcc_lcdc_image_update             *info;

        unsigned int            settop_support;
};

void TCC_VIQE_DI_PlugInOut_forAlphablending(int plugIn);
void TCC_VIQE_DI_Init(VIQE_DI_TYPE *viqe_arg);
void TCC_VIQE_DI_Run(int DI_use, int Multi_hwr);
void TCC_VIQE_DI_DeInit(int Multi_hwr);
void TCC_VIQE_DI_Init60Hz_M2M(TCC_OUTPUT_TYPE outputMode, struct tcc_lcdc_image_update *input_image);
void TCC_VIQE_DI_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image, int DI_use);
void TCC_VIQE_DI_DeInit60Hz_M2M(int layer);
void TCC_VIQE_Scaler_Init60Hz_M2M(void);
void TCC_VIQE_Scaler_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image);
void TCC_VIQE_Scaler_Release60Hz_M2M(void);
void TCC_VIQE_Scaler_Sub_Init60Hz_M2M(void);
void TCC_VIQE_Scaler_Sub_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image);
void TCC_VIQE_Scaler_Sub_Release60Hz_M2M(void);
irqreturn_t TCC_VIQE_Scaler_Handler60Hz_M2M(int irq, void *client_data);
irqreturn_t TCC_VIQE_Scaler_Sub_Handler60Hz_M2M(int irq, void *client_data);
void TCC_VIQE_Display_Update60Hz_M2M(struct tcc_lcdc_image_update *input_image);
void TCC_VIQE_DI_Init60Hz(TCC_OUTPUT_TYPE outputMode, int lcdCtrlNum, struct tcc_lcdc_image_update *input_image);
void TCC_VIQE_DI_Swap60Hz(int mode);
void TCC_VIQE_DI_SetFMT60Hz(int enable);
void TCC_VIQE_DI_Run60Hz(struct tcc_lcdc_image_update *input_image ,int reset_frmCnt);
void TCC_VIQE_DI_DeInit60Hz(void);


#endif
