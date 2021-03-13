/****************************************************************************
linux/drivers/video/tcc/viqe/tcc_vioc_viqe_interface.c
Description: TCC VIOC h/w block 

One line to give the program's name and a brief idea of what it does.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/fb.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/iomap.h>

#include <mach/tccfb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_intr.h>
#include <mach/of_vioc_rdma.h>
#include <mach/of_vioc_wmix.h>
#include <mach/of_vioc_sc.h>
#include <mach/of_vioc_wdma.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_viqe.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_config.h>
#include <mach/vioc_api.h>
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <mach/vioc_global.h>
#endif
#include <mach/tccfb.h>
#include <mach/tccfb_ioctrl.h>
#else
#include <video/tcc/tcc_types.h>

#include <video/tcc/tccfb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_intr.h>
#include <video/tcc/of_vioc_rdma.h>
#include <video/tcc/of_vioc_wmix.h>
#include <video/tcc/of_vioc_sc.h>
#include <video/tcc/of_vioc_wdma.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_viqe.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_global.h>
#endif

#include "tcc_vioc_viqe.h"
#include "tcc_vioc_viqe_interface.h"

#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
extern void tcc_video_frame_backup(struct tcc_lcdc_image_update *Image);
#endif
extern void tcc_video_post_process(struct tcc_lcdc_image_update *ImageInfo);

/*******			define extern symbol	******/
#if defined(CONFIG_FB_TCC_COMPOSITE)
extern void tcc_composite_update(struct tcc_lcdc_image_update *update);
#endif

#if defined(CONFIG_FB_TCC_COMPONENT)
extern void tcc_component_update(struct tcc_lcdc_image_update *update);
#endif

extern void tca_lcdc_set_onthefly(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo);
extern void tca_scale_display_update(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo);

//#define USE_DEINTERLACE_S_IN30Hz
//#define USE_DEINTERLACE_S_IN60Hz

//#define DYNAMIC_USE_DEINTL_COMPONENT
//#define MAX_VIQE_FRAMEBUFFER

#if defined(CONFIG_ARCH_TCC898X)
#define USE_VIQE_FOR_SUB_M2M
#ifdef USE_VIQE_FOR_SUB_M2M
static unsigned int gPMEM_VIQE_SUB_BASE;
static unsigned int gPMEM_VIQE_SUB_SIZE;

static int gFrmCnt_Sub_60Hz = 0;
static int gVIQE1_Init_State = 0;

#endif // USE_VIQE_FOR_SUB_M2M
#endif // CONFIG_ARCH_TCC898X

static pmap_t pmap_viqe;
static unsigned int gPMEM_VIQE_BASE;
static unsigned int gPMEM_VIQE_SIZE;

static struct tcc_viqe_common_virt_addr_info_t viqe_common_info;
static struct tcc_viqe_m2m_virt_addr_info_t main_m2m_info;
static struct tcc_viqe_m2m_virt_addr_info_t sub_m2m_info;
static struct tcc_viqe_60hz_virt_addr_info_t viqe_60hz_lcd_info;
static struct tcc_viqe_60hz_virt_addr_info_t viqe_60hz_external_info;
static struct tcc_viqe_m2m_scaler_type_t *scaler;
static struct tcc_viqe_m2m_scaler_type_t *scaler_sub;

static struct tcc_lcdc_image_update  output_m2m_image;
static struct tcc_lcdc_image_update  output_m2m_image_sub;

static unsigned int overlay_buffer[2][VPU_BUFFER_MANAGE_COUNT];
static int current_buffer_idx[2];

static int gFrmCnt_30Hz = 0;
static int gUseWmixer = 0;

static int gOutputMode = 0;
static int gFrmCnt_60Hz = 0;
static int gusingScale_60Hz = 0;
static int gLcdc_layer_60Hz = -1;
static VIOC_VIQE_FMT_TYPE gViqe_fmt_60Hz;
static int gImg_fmt_60Hz = -1;
static int gpreCrop_left_60Hz = 0;
static int gpreCrop_right_60Hz = 0;
static int gpreCrop_top_60Hz = 0;
static int gpreCrop_bottom_60Hz = 0;

static int gDeintlS_Use_60Hz = 0;
static int gDeintlS_Use_Plugin = 0;

static int not_use_viqe1 = 0;

#ifndef USE_DEINTERLACE_S_IN30Hz
//#if defined(CONFIG_ARCH_TCC897X)
static int gusingDI_S = 0;
//#endif
static int gbfield_30Hz =0;
static VIOC_VIQE_DEINTL_MODE gDI_mode_30Hz = VIOC_VIQE_DEINTL_MODE_2D;
#else
static int gusingDI_S = 0;
#endif

#ifndef USE_DEINTERLACE_S_IN60Hz
static VIOC_VIQE_DEINTL_MODE gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
#else
static VIOC_VIQE_DEINTL_MODE gDI_mode_60Hz = VIOC_VIQE_DEINTL_S;
#endif

static int gVIQE_Init_State = 0;

#if 0
#define dprintk(msg...)	 { printk( "tcc_vioc_viqe_interface: " msg); }
#define iprintk(msg...)  { printk( "tcc_vioc_viqe_interface: " msg); }
#else
#define dprintk(msg...)
#define iprintk(msg...)
#endif

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
static VIOC_RDMA *pRDMATempBase_60Hz;
extern int enabled_LastFrame;
extern VIOC_RDMA * pLastFrame_RDMABase;
#endif

extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
					unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);

#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
extern unsigned char hdmi_get_hdmimode(void);
#endif

#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
#ifdef CONFIG_ARCH_TCC896X
#define S_DI_BIT		(0x12003800)
#endif
#ifdef CONFIG_ARCH_TCC897X
#define S_DI_BIT		(0x72003800)
#endif

static inline void S_DI_writeb(u8 b, unsigned offset){
    writeb(b, IOMEM(offset));
}
#endif

void TCC_VIQE_DI_PlugInOut_forAlphablending(int plugIn)
{
	if(gUseWmixer)
	{
		if(plugIn)
		{
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
		}
		else
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
		}			
	}
	else
	{
		if(plugIn)
		{
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE1, main_m2m_info.gVIQE_RDMA_num_m2m);
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
		}
		else
		{
			VIOC_RDMA_SetImageDisable(main_m2m_info.pRDMABase_m2m);
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE1);
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
			gFrmCnt_30Hz =gFrmCnt_60Hz = 0;		
		}
	}
}

/* VIQE Set */
//////////////////////////////////////////////////////////////////////////////////////////
void TCC_VIQE_DI_Init(VIQE_DI_TYPE *viqe_arg)
{
#ifndef USE_DEINTERLACE_S_IN30Hz
	unsigned int deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3;
	int imgSize;
	int top_size_dont_use = OFF;		//If this value is OFF, The size information is get from VIOC modules.
#endif
	unsigned int framebufWidth, framebufHeight;
	VIOC_VIQE_FMT_TYPE img_fmt = VIOC_VIQE_FMT_YUV420;

	pmap_get_info("viqe", &pmap_viqe);
	gPMEM_VIQE_BASE = pmap_viqe.base;
	gPMEM_VIQE_SIZE = pmap_viqe.size;
	
	viqe_arg->crop_top = (viqe_arg->crop_top >>1)<<1;
#ifdef MAX_VIQE_FRAMEBUFFER
	framebufWidth = 1920;
	framebufHeight = 1088;
#else
	framebufWidth = ((viqe_arg->srcWidth - viqe_arg->crop_left - viqe_arg->crop_right) >> 3) << 3;			// 8bit align
	framebufHeight = ((viqe_arg->srcHeight - viqe_arg->crop_top - viqe_arg->crop_bottom) >> 2) << 2;		// 4bit align
#endif

	printk("TCC_VIQE_DI_Init, W:%d, H:%d, FMT:%s, OddFirst:%d \n",
		framebufWidth, framebufHeight, (img_fmt?"YUV422":"YUV420"), viqe_arg->OddFirst);

	if(viqe_common_info.gBoard_num == 1) {
		if(main_m2m_info.pRDMABase_m2m->uCTRL.nREG & HwDMA_IEN)
			VIOC_RDMA_SetImageDisable(main_m2m_info.pRDMABase_m2m);

		//TCC_OUTPUT_FB_ClearVideoImg();
	}

	VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
	VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
	VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
	VIOC_RDMA_SetImageBfield(main_m2m_info.pRDMABase_m2m, viqe_arg->OddFirst);
	
	if(viqe_arg->useWMIXER)
	{
		gUseWmixer = 1;
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
		VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
	}
	else
	{
		gUseWmixer = 0;
	#ifndef USE_DEINTERLACE_S_IN30Hz
		#if defined(CONFIG_ARCH_TCC897X)
			if(viqe_arg->multi_hwr)
			{
				BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
				BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
				#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
					S_DI_writeb(3, tcc_p2v(S_DI_BIT));
				#endif
				VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
				gusingDI_S = 1;
				printk("DEINTL-S\n");
			}
			else
		#endif
			{
				if(not_use_viqe1)
				{
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset

					// If you use 3D(temporal) De-interlace mode, you have to set physical address for using DMA register.
					//If 2D(spatial) mode, these registers are ignored
					imgSize = (framebufWidth * framebufHeight * 2);
					deintl_dma_base0	= gPMEM_VIQE_BASE;
					deintl_dma_base1	= deintl_dma_base0 + imgSize;
					deintl_dma_base2	= deintl_dma_base1 + imgSize;
					deintl_dma_base3	= deintl_dma_base2 + imgSize;
				}
				else
				{
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x01<<18)); // VIQE reset
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x00<<18)); // VIQE reset

					// If you use 3D(temporal) De-interlace mode, you have to set physical address for using DMA register.
					//If 2D(spatial) mode, these registers are ignored
					imgSize = (framebufWidth * framebufHeight * 2);
					deintl_dma_base0	= gPMEM_VIQE_BASE + (gPMEM_VIQE_SIZE/2);
					deintl_dma_base1	= deintl_dma_base0 + imgSize;
					deintl_dma_base2	= deintl_dma_base1 + imgSize;
					deintl_dma_base3	= deintl_dma_base2 + imgSize;
				}

				if (top_size_dont_use == OFF)
				{
					framebufWidth  = 0;
					framebufHeight = 0;
				}

				VIOC_VIQE_SetControlRegister(viqe_common_info.pVIQE1, framebufWidth, framebufHeight, img_fmt);
				VIOC_VIQE_SetDeintlRegister(viqe_common_info.pVIQE1, img_fmt, top_size_dont_use, framebufWidth, framebufHeight, gDI_mode_30Hz, deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3);
				VIOC_VIQE_SetControlEnable(viqe_common_info.pVIQE1, OFF, OFF, OFF, OFF, ON);

				if(main_m2m_info.gVIQE_RDMA_num_m2m == 3)
				{
				#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
					if(gOutputMode != TCC_OUTPUT_HDMI || hdmi_get_hdmimode())
						VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE1, false);
					else
						VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE1, true);
				#elif defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
					if(gOutputMode == TCC_OUTPUT_COMPOSITE)
						VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE1, false);
					else
						VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE1, true);
				#else
					VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE1, true);
				#endif
					VIOC_VIQE_SetImageY2RMode(viqe_common_info.pVIQE1, 0x02);
				}
				VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE1, main_m2m_info.gVIQE_RDMA_num_m2m);
				if(viqe_arg->OddFirst)
					gbfield_30Hz =1;
				else
					gbfield_30Hz =0;

				printk("DEINTL-VIQE\n");
			}
	#else
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
			#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
				S_DI_writeb(3, tcc_p2v(S_DI_BIT));
			#endif
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
			gusingDI_S = 1;
			printk("DEINTL-S\n");
	#endif
	}

	gFrmCnt_30Hz = 0;
	
	gVIQE_Init_State = 1;
}

void TCC_VIQE_DI_Run(int DI_use, int Multi_hwr)
{
#ifndef USE_DEINTERLACE_S_IN30Hz
	int ret;
	VIOC_PlugInOutCheck VIOC_PlugIn;
#endif

	if(viqe_common_info.gBoard_num == 1) {
		if(gVIQE_Init_State == 0)
		{
			dprintk("%s VIQE block isn't initailized\n", __func__);
			return;
		}
	}

#ifndef USE_DEINTERLACE_S_IN30Hz
	ret = VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE1, &VIOC_PlugIn);
	if (ret == VIOC_DEVICE_CONNECTED && VIOC_PlugIn.enable) {

	}
	else {
		dprintk("%s VIQE block isn't pluged!!!\n", __func__);
		return;
	}
#endif

	if(gUseWmixer)
	{
		VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
		VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

		if(DI_use)
		{
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
		}
		else
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
		}
		
	}
	else
	{
	#ifndef USE_DEINTERLACE_S_IN30Hz
		#ifdef CONFIG_ARCH_TCC897X
			if(Multi_hwr)
			{
				VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
				VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
				if(DI_use)
				{
					VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
					if(!gusingDI_S)
					{
						VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
						gusingDI_S = 1;
					}
				}
				else
				{
					VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
					if(gusingDI_S)
					{
						VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
						gusingDI_S = 0;
					}
				}
			}
			else
		#endif
			{
				if(DI_use)
				{
					if (gbfield_30Hz) 					// end fied of bottom field
					{
						VIOC_RDMA_SetImageBfield(main_m2m_info.pRDMABase_m2m, 0);				// change the bottom to top field
						// if you want to change the base address, you call the RDMA SetImageBase function in this line.
						gbfield_30Hz= 0;
					}
					else
					{
						VIOC_RDMA_SetImageBfield(main_m2m_info.pRDMABase_m2m, 1);				// change the top to bottom field
						gbfield_30Hz = 1;
					}

					VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
					VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

					if(gFrmCnt_30Hz >= 3)
					{
						VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE1, VIOC_VIQE_DEINTL_MODE_3D);
						gDI_mode_30Hz = VIOC_VIQE_DEINTL_MODE_3D;
					}
					else
					{
						VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE1, VIOC_VIQE_DEINTL_MODE_2D);
						gDI_mode_30Hz = VIOC_VIQE_DEINTL_MODE_2D;
					}
					VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE1, OFF, OFF, OFF, OFF, ON);
					VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
				}
				else
				{
					VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, true);
					VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
					VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE1, OFF, OFF, OFF, OFF, OFF);
					VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
					gFrmCnt_30Hz = 0;
				}
			}
	#else
			VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
			VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
			if(DI_use)
			{
				VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
				if(!gusingDI_S)
				{
					VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, main_m2m_info.gVIQE_RDMA_num_m2m);
					gusingDI_S = 1;
				}
			}
			else
			{
				VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
				if(gusingDI_S)
				{
					VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
					gusingDI_S = 0;
				}
			}
	#endif
	}

	gFrmCnt_30Hz++;	
}

void TCC_VIQE_DI_DeInit(int Multi_hwr)
{
	gFrmCnt_30Hz = 0;

	printk("TCC_VIQE_DI_DeInit\n");

	if(gUseWmixer)
	{
		VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
	}
	else
	{
	#ifndef USE_DEINTERLACE_S_IN30Hz
		#ifdef CONFIG_ARCH_TCC897X
			if(Multi_hwr)
			{
				VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
				gusingDI_S = 0;
				BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
				BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
			}
			else
		#endif
			{
				VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE1);
				if(not_use_viqe1)
				{
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset
				}
				else
				{
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x01<<18)); // VIQE reset
					BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x00<<18)); // VIQE reset
				}
			}
	#else
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			gusingDI_S = 0;
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
	#endif
	}

	gVIQE_Init_State = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
void TCC_VIQE_DI_Init60Hz_M2M(TCC_OUTPUT_TYPE outputMode, struct tcc_lcdc_image_update *input_image)
{
	unsigned int deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3;
	int imgSize;
	int top_size_dont_use = OFF;		//If this value is OFF, The size information is get from VIOC modules.
	unsigned int framebufWidth, framebufHeight;
	VIOC_VIQE_FMT_TYPE img_fmt = VIOC_VIQE_FMT_YUV420;

	gOutputMode = outputMode;
	gUseWmixer = 0;

	if (!input_image->deinterlace_mode)
		goto m2m_init_scaler;

	pmap_get_info("viqe", &pmap_viqe);
	gPMEM_VIQE_BASE = pmap_viqe.base;
#if defined(USE_VIQE_FOR_SUB_M2M)
	gPMEM_VIQE_SIZE = pmap_viqe.size/2;
	gPMEM_VIQE_SUB_BASE = pmap_viqe.base + (pmap_viqe.size/2);
	gPMEM_VIQE_SUB_SIZE = pmap_viqe.size/2;
#else
	gPMEM_VIQE_SIZE = pmap_viqe.size;
#endif

#ifdef MAX_VIQE_FRAMEBUFFER
	framebufWidth = 1920;
	framebufHeight = 1088;
#else
	framebufWidth = ((input_image->Frame_width) >> 3) << 3;			// 8bit align
	framebufHeight = ((input_image->Frame_height) >> 2) << 2;		// 4bit align
#endif

	printk("%s, W:%d, H:%d, FMT:%s, OddFirst:%d \n", __func__, framebufWidth, framebufHeight, (img_fmt?"YUV422":"YUV420"), input_image->odd_first_flag);

	if(viqe_common_info.gBoard_num == 1) {
		if (input_image->Lcdc_layer == RDMA_VIDEO_SUB) {
			if(sub_m2m_info.pRDMABase_m2m->uCTRL.nREG & HwDMA_IEN)
				VIOC_RDMA_SetImageDisable(sub_m2m_info.pRDMABase_m2m);
		} else if(input_image->Lcdc_layer == RDMA_VIDEO) {
			if(main_m2m_info.pRDMABase_m2m->uCTRL.nREG & HwDMA_IEN)
				VIOC_RDMA_SetImageDisable(main_m2m_info.pRDMABase_m2m);
		}

	    //TCC_OUTPUT_FB_ClearVideoImg();
	}

	if (input_image->Lcdc_layer == RDMA_VIDEO_SUB) {
#if defined(USE_VIQE_FOR_SUB_M2M)
		VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, false);
		VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
		VIOC_RDMA_SetImageIntl(sub_m2m_info.pRDMABase_m2m, 1);
		VIOC_RDMA_SetImageBfield(sub_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x01<<18)); // VIQE reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x00<<18)); // VIQE reset

		// If you use 3D(temporal) De-interlace mode, you have to set physical address for using DMA register.
		//If 2D(spatial) mode, these registers are ignored
		imgSize = (framebufWidth * framebufHeight * 2);
		deintl_dma_base0	= gPMEM_VIQE_SUB_BASE;
		deintl_dma_base1	= deintl_dma_base0 + imgSize;
		deintl_dma_base2	= deintl_dma_base1 + imgSize;
		deintl_dma_base3	= deintl_dma_base2 + imgSize;

		if (top_size_dont_use == OFF)
		{
			framebufWidth  = 0;
			framebufHeight = 0;
		}

		VIOC_VIQE_SetControlRegister(viqe_common_info.pVIQE1, framebufWidth, framebufHeight, img_fmt);
		VIOC_VIQE_SetDeintlRegister(viqe_common_info.pVIQE1, img_fmt, top_size_dont_use, framebufWidth, framebufHeight, gDI_mode_60Hz, deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3);
		VIOC_VIQE_SetControlEnable(viqe_common_info.pVIQE1, OFF, OFF, OFF, OFF, ON);

		VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE1, sub_m2m_info.gVIQE_RDMA_num_m2m);

		gFrmCnt_Sub_60Hz = 0;
		gVIQE1_Init_State = 1;
		printk("%s - VIQE1\n", __func__);

#else
		VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, true);
		VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
		VIOC_RDMA_SetImageIntl(sub_m2m_info.pRDMABase_m2m, 1);
		VIOC_RDMA_SetImageBfield(sub_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
	#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
		S_DI_writeb(3, tcc_p2v(S_DI_BIT));
	#endif
		VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, sub_m2m_info.gDEINTLS_RDMA_num_m2m);
		gusingDI_S = 1;
		printk("%s - DEINTL-S\n", __func__);
#endif
	}else if (input_image->Lcdc_layer == RDMA_VIDEO) {
		VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
		VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
		VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);
		VIOC_RDMA_SetImageBfield(main_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset

		// If you use 3D(temporal) De-interlace mode, you have to set physical address for using DMA register.
		//If 2D(spatial) mode, these registers are ignored
		imgSize = (framebufWidth * framebufHeight * 2);
		deintl_dma_base0	= gPMEM_VIQE_BASE;
		deintl_dma_base1	= deintl_dma_base0 + imgSize;
		deintl_dma_base2	= deintl_dma_base1 + imgSize;
		deintl_dma_base3	= deintl_dma_base2 + imgSize;

		if (top_size_dont_use == OFF)
		{
			framebufWidth  = 0;
			framebufHeight = 0;
		}

		VIOC_VIQE_SetControlRegister(viqe_common_info.pVIQE0, framebufWidth, framebufHeight, img_fmt);
		VIOC_VIQE_SetDeintlRegister(viqe_common_info.pVIQE0, img_fmt, top_size_dont_use, framebufWidth, framebufHeight, gDI_mode_60Hz, deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3);
		VIOC_VIQE_SetControlEnable(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, ON);

		VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE0, main_m2m_info.gVIQE_RDMA_num_m2m);

		gFrmCnt_60Hz= 0;
		gVIQE_Init_State = 1;
		printk("%s - VIQE0\n", __func__);
	}

m2m_init_scaler:
	if (input_image->Lcdc_layer == RDMA_VIDEO_SUB) {
		TCC_VIQE_Scaler_Sub_Init60Hz_M2M();
	} else if (input_image->Lcdc_layer == RDMA_VIDEO) {
		TCC_VIQE_Scaler_Init60Hz_M2M();
	}
}

void TCC_VIQE_DI_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image, int reset_frmCnt)
{
	VIOC_PlugInOutCheck VIOC_PlugIn;
	unsigned int JudderCnt = 0;
	int ret;

	if (!input_image->deinterlace_mode) {
		if (input_image->Lcdc_layer == RDMA_VIDEO_SUB) {
			VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, true);
			VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
			VIOC_RDMA_SetImageIntl(sub_m2m_info.pRDMABase_m2m, 0);
			TCC_VIQE_Scaler_Sub_Run60Hz_M2M(input_image);
		} else if (input_image->Lcdc_layer == RDMA_VIDEO) {
			VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
			VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 0);
			TCC_VIQE_Scaler_Run60Hz_M2M(input_image);
		}
	} else {
		if (input_image->Lcdc_layer == RDMA_VIDEO_SUB) {
			VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, true);
			VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */
			VIOC_RDMA_SetImageIntl(sub_m2m_info.pRDMABase_m2m, 1);
			VIOC_RDMA_SetImageBfield(sub_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);
#if defined(USE_VIQE_FOR_SUB_M2M)
			gFrmCnt_Sub_60Hz++;
			if(gVIQE1_Init_State == 0)
			{
				dprintk("%s VIQE1 block isn't initailized\n", __func__);
				return;
			}

			VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE1, &VIOC_PlugIn);
			if(VIOC_PlugIn.enable) {

			}
			else {
				dprintk("%s VIQE1 block isn't pluged!!!\n", __func__);
				return;
			}

			if(reset_frmCnt){
				gFrmCnt_Sub_60Hz = 0;
				dprintk("VIQE1 3D -> 2D \n");
			}

			VIOC_RDMA_SetImageBfield(sub_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);
			VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, false);
			VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

			if(gFrmCnt_Sub_60Hz >= 3)
			{
				VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE1, VIOC_VIQE_DEINTL_MODE_3D);
				// Process JUDDER CNTS
				JudderCnt = (input_image->Frame_width + 64)/64 -1;
				VIOC_VIQE_SetDeintlJudderCnt(viqe_common_info.pVIQE1, JudderCnt);
				gFrmCnt_Sub_60Hz = VIOC_VIQE_DEINTL_MODE_3D;
			}
			else
			{
				VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE1, VIOC_VIQE_DEINTL_MODE_2D);
				gFrmCnt_Sub_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
			}
			VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE1, OFF, OFF, OFF, OFF, ON);
			VIOC_RDMA_SetImageIntl(sub_m2m_info.pRDMABase_m2m, 1);
#else
			if(!gusingDI_S) {
				VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, sub_m2m_info.gDEINTLS_RDMA_num_m2m);
				gusingDI_S = 1;
			}
#endif
			TCC_VIQE_Scaler_Sub_Run60Hz_M2M(input_image);
		}
		else if(input_image->Lcdc_layer == RDMA_VIDEO) {
			gFrmCnt_60Hz++;

			if(gVIQE_Init_State == 0)
			{
				dprintk("%s VIQE block isn't initailized\n", __func__);
				return;
			}

			ret = VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE0, &VIOC_PlugIn);
			if (ret == VIOC_DEVICE_CONNECTED && VIOC_PlugIn.enable) {

			}
			else {
				dprintk("%s VIQE block isn't pluged!!!\n", __func__);
				return;
			}

			if(reset_frmCnt){
				gFrmCnt_60Hz = 0;
				dprintk("VIQE 3D -> 2D \n");
			}

			VIOC_RDMA_SetImageBfield(main_m2m_info.pRDMABase_m2m, input_image->odd_first_flag);
			VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, false);
			VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

			if(gFrmCnt_60Hz >= 3)
			{
				VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_3D);
				// Process JUDDER CNTS
				JudderCnt = (input_image->Frame_width + 64)/64 -1;
				VIOC_VIQE_SetDeintlJudderCnt(viqe_common_info.pVIQE0, JudderCnt);
				gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_3D;
			}
			else
			{
				VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_2D);
				gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
			}
			VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, ON);
			VIOC_RDMA_SetImageIntl(main_m2m_info.pRDMABase_m2m, 1);

			TCC_VIQE_Scaler_Run60Hz_M2M(input_image);
		}
	}

#if 0//def CONFIG_HDMI_DISPLAY_LASTFRAME
	if(enabled_LastFrame)
	{
		tcc_video_clear_last_frame(pLastFrame_RDMABase, false);
	}
#endif

#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
	tcc_video_frame_backup(input_image);
#endif

	return;
}

void TCC_VIQE_DI_DeInit60Hz_M2M(int layer)
{
	printk("%s - Layer:%d +\n", __func__, layer);

	if (layer == RDMA_VIDEO_SUB) {
		VIOC_RDMA_SetImageDisable(sub_m2m_info.pRDMABase_m2m);
		VIOC_RDMA_SetImageY2REnable(sub_m2m_info.pRDMABase_m2m, true);
		VIOC_RDMA_SetImageY2RMode(sub_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

		if (gusingDI_S) {
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset

			gusingDI_S = 0;
		}

#if defined(USE_VIQE_FOR_SUB_M2M)
		gFrmCnt_Sub_60Hz = 0;

		if (gVIQE1_Init_State) {
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE1);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x01<<18)); // VIQE1 reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<18), (0x00<<18)); // VIQE1 reset

			gVIQE1_Init_State = 0;
		}
#endif
		TCC_VIQE_Scaler_Sub_Release60Hz_M2M();
	}

	if (layer == RDMA_VIDEO) {
		gFrmCnt_60Hz = 0;

		VIOC_RDMA_SetImageDisable(main_m2m_info.pRDMABase_m2m);
		VIOC_RDMA_SetImageY2REnable(main_m2m_info.pRDMABase_m2m, true);
		VIOC_RDMA_SetImageY2RMode(main_m2m_info.pRDMABase_m2m, 0x02); /* Y2RMode Default 0 (Studio Color) */

		if (gVIQE_Init_State) {
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE0);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset

			gVIQE_Init_State = 0;
		}

		TCC_VIQE_Scaler_Release60Hz_M2M();
	}
	printk("%s - Layer:%d -\n", __func__, layer);
}

void TCC_VIQE_Scaler_Init_Buffer_M2M(void)
{
	pmap_t pmap_overlay;
	pmap_t pmap_overlay1;
	unsigned int szBuffer = 0;
	int buffer_cnt = 0;

	pmap_get_info("overlay", &pmap_overlay);
	szBuffer = pmap_overlay.size / VPU_BUFFER_MANAGE_COUNT;

	for(buffer_cnt=0; buffer_cnt < VPU_BUFFER_MANAGE_COUNT; buffer_cnt++)
	{
		overlay_buffer[0][buffer_cnt] = (unsigned int)pmap_overlay.base + (szBuffer * buffer_cnt);
		dprintk("[0]Overlay :: %d/%d = 0x%x  \n", buffer_cnt, VPU_BUFFER_MANAGE_COUNT, overlay_buffer[0][buffer_cnt]);
	}

	pmap_get_info("overlay1", &pmap_overlay1);
	szBuffer = pmap_overlay1.size / VPU_BUFFER_MANAGE_COUNT;

	for(buffer_cnt=0; buffer_cnt < VPU_BUFFER_MANAGE_COUNT; buffer_cnt++)
	{
		overlay_buffer[1][buffer_cnt] = (unsigned int)pmap_overlay1.base + (szBuffer * buffer_cnt);
		dprintk("[1]Overlay :: %d/%d = 0x%x  \n", buffer_cnt, VPU_BUFFER_MANAGE_COUNT, overlay_buffer[1][buffer_cnt]);
	}

	current_buffer_idx[0] = 0;
	current_buffer_idx[1] = 0;
}

unsigned int TCC_VIQE_Scaler_Get_Buffer_M2M(struct tcc_lcdc_image_update* input_image)
{
	unsigned int buffer_addr = 0;

	if(input_image->Lcdc_layer != RDMA_VIDEO)
		return 0x00;

	buffer_addr = overlay_buffer[0][current_buffer_idx[0]];
	current_buffer_idx[0] += 1;
	if(current_buffer_idx[0] >= VPU_BUFFER_MANAGE_COUNT)
		current_buffer_idx[0] = 0;

	return buffer_addr;
}

unsigned int TCC_VIQE_Scaler_Sub_Get_Buffer_M2M(struct tcc_lcdc_image_update* input_image)
{
	unsigned int buffer_addr = 0;

	if(input_image->Lcdc_layer != RDMA_VIDEO_SUB)
		return 0x00;

	buffer_addr = overlay_buffer[1][current_buffer_idx[1]];
	current_buffer_idx[1] += 1;
	if(current_buffer_idx[1] >= VPU_BUFFER_MANAGE_COUNT)
		current_buffer_idx[1] = 0;

	return buffer_addr;
}

void TCC_VIQE_Scaler_Init60Hz_M2M(void)
{
    int ret = 0;

    if (!scaler->data->irq_reged) {
#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
		// Don't STOP_REQ clear because of dual output problem(DISP FIFO under-run), when video is out.
#else
		vioc_config_stop_req(1);
#endif

		synchronize_irq(scaler->irq);
		vioc_intr_clear(scaler->vioc_intr->id, scaler->vioc_intr->bits);
		ret = request_irq(scaler->irq, TCC_VIQE_Scaler_Handler60Hz_M2M, IRQF_SHARED, "scaler1", scaler);

		if (ret) printk("failed to acquire scaler1 request_irq. \n");

		vioc_intr_enable(scaler->vioc_intr->id, scaler->vioc_intr->bits);
		scaler->data->irq_reged = 1;
    }

    scaler->data->dev_opened++;

	current_buffer_idx[0] = 0;

    dprintk("%s() : Out - open(%d). \n", __func__, scaler->data->dev_opened);
}

void TCC_VIQE_Scaler_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image)
{
        VIOC_SCALER_INFO_Type pScalerInfo;
        unsigned int pSrcBase0 = 0, pSrcBase1 = 0, pSrcBase2 = 0;
        unsigned int crop_width;

	 unsigned int dest_fmt = TCC_LCDC_IMG_FMT_YUYV;
	 //unsigned int dest_fmt = TCC_LCDC_IMG_FMT_YUV420ITL0;
	 //unsigned int dest_fmt = TCC_LCDC_IMG_FMT_COMP;

#ifndef CONFIG_OF_VIOC
        volatile PVIOC_RDMA pSC_RDMABase = (PVIOC_RDMA)scaler->rdma->reg;
        volatile PVIOC_WMIX pSC_WMIXBase = (PVIOC_WMIX)scaler->wmix->reg;
        volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler->wdma->reg;
        //volatile PVIOC_IREQ_CONFIG pSC_IREQConfig = (PVIOC_IREQ_CONFIG)io_p2v(TCC_PA_VIOC_CFGINT);
#endif

	dprintk("%s() : IN.\n", __func__);

	memcpy(scaler->info, (struct tcc_lcdc_image_update*)input_image, sizeof(struct tcc_lcdc_image_update));
	memcpy(&output_m2m_image, (struct tcc_lcdc_image_update*)input_image, sizeof(struct tcc_lcdc_image_update));
	dprintk("%s : %d(%d), %d(%d)\n",__func__, output_m2m_image.buffer_unique_id, output_m2m_image.odd_first_flag, input_image->buffer_unique_id, input_image->odd_first_flag);
        crop_width = scaler->info->crop_right - scaler->info->crop_left;

        if (scaler->settop_support) {
                VIOC_RDMA_SetImageAlphaSelect(pSC_RDMABase, 1);
                VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
        } else {
                VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
        }
        VIOC_RDMA_SetImageFormat(pSC_RDMABase, scaler->info->fmt);

        //interlaced frame process ex) MPEG2
        if (0) {//(scaler->info->interlaced) {
                VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler->info->crop_right - scaler->info->crop_left), (scaler->info->crop_bottom - scaler->info->crop_top)/2);
                VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler->info->fmt, scaler->info->Frame_width*2);
        } else {
                VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler->info->crop_right - scaler->info->crop_left), (scaler->info->crop_bottom - scaler->info->crop_top));
                VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler->info->fmt, scaler->info->Frame_width);
        }

        pSrcBase0 = (unsigned int)scaler->info->addr0;
        pSrcBase1 = (unsigned int)scaler->info->addr1;
        pSrcBase2 = (unsigned int)scaler->info->addr2;
        if (scaler->info->fmt >= SC_IMG_FMT_YCbCr444_SEP_YUV) { // address limitation!!
                dprintk("%s():  src addr is not allocate. \n", __func__);
                scaler->info->crop_left       = (scaler->info->crop_left>>3)<<3;
                scaler->info->crop_right = scaler->info->crop_left + crop_width;
                scaler->info->crop_right = scaler->info->crop_left + (scaler->info->crop_right - scaler->info->crop_left);
                tccxxx_GetAddress(scaler->info->fmt, (unsigned int)scaler->info->addr0,                 \
                                                                        scaler->info->Frame_width, scaler->info->Frame_height,        \
                                                                        scaler->info->crop_left, scaler->info->crop_top,            \
                                                                        &pSrcBase0, &pSrcBase1, &pSrcBase2);
        }
        #if defined(CONFIG_ARCH_TCC896X)
        VIOC_RDMA_SetImageR2YEnable(pSC_RDMABase, 0);
        #endif//
        VIOC_RDMA_SetImageY2REnable(pSC_RDMABase, 0);
        VIOC_RDMA_SetImageBase(pSC_RDMABase, (unsigned int)pSrcBase0, (unsigned int)pSrcBase1, (unsigned int)pSrcBase2);

        pScalerInfo.BYPASS                      = 0;
        pScalerInfo.DST_WIDTH           = scaler->info->Image_width;
        pScalerInfo.DST_HEIGHT          = scaler->info->Image_height;
        pScalerInfo.OUTPUT_POS_X                = 0;
        pScalerInfo.OUTPUT_POS_Y                = 0;
        pScalerInfo.OUTPUT_WIDTH                = pScalerInfo.DST_WIDTH;
        pScalerInfo.OUTPUT_HEIGHT       = pScalerInfo.DST_HEIGHT;
        VIOC_API_SCALER_SetConfig(scaler->sc->id, &pScalerInfo);
        VIOC_API_SCALER_SetPlugIn(scaler->sc->id, scaler->sc->path);
        VIOC_API_SCALER_SetUpdate(scaler->sc->id);

        VIOC_RDMA_SetImageEnable(pSC_RDMABase); // SoC guide info.

        VIOC_CONFIG_WMIXPath(scaler->wmix->path, 1); // wmixer op is mixing mode.
        VIOC_WMIX_SetSize(pSC_WMIXBase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);
        VIOC_WMIX_SetUpdate(pSC_WMIXBase);

        VIOC_WDMA_SetImageFormat(pSC_WDMABase, dest_fmt);
        VIOC_WDMA_SetImageSize(pSC_WDMABase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);
        VIOC_WDMA_SetImageOffset(pSC_WDMABase, dest_fmt, pScalerInfo.DST_WIDTH);
		if(scaler->info->dst_addr0 == 0)
			scaler->info->dst_addr0 = TCC_VIQE_Scaler_Get_Buffer_M2M(input_image);
        VIOC_WDMA_SetImageBase(pSC_WDMABase, (unsigned int)scaler->info->dst_addr0, (unsigned int)scaler->info->dst_addr1, (unsigned int)scaler->info->dst_addr2);

        VIOC_WDMA_SetImageR2YEnable(pSC_WDMABase, 0);
        VIOC_WDMA_SetImageEnable(pSC_WDMABase, 0);
        pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF; // wdma status register all clear.

        output_m2m_image.Frame_width = pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image.Frame_height = pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image.Image_width = pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image.Image_height = pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image.crop_left= 0;
        output_m2m_image.crop_top= 0;
        output_m2m_image.crop_right= pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image.crop_bottom= pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image.fmt = dest_fmt;
        output_m2m_image.addr0 = scaler->info->dst_addr0;

	dprintk("%s() : OUT.\n", __func__);
}

void TCC_VIQE_Scaler_Release60Hz_M2M(void)
{
	unsigned int rdma_offset = scaler->rdma->id;
	unsigned int sc_offset = scaler->sc->id + 28;
	unsigned int wdma_offset = scaler->wdma->id;
	unsigned int wmix_offset = scaler->wmix->id + 9;

	dprintk("%s():  In -open(%d), block(%d), wait(%d), cmd(%d), irq(%d) \n", __func__, scaler->data->dev_opened, scaler->data->block_operating, \
			scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->irq_reged);

	if (scaler->data->dev_opened > 0) {
		scaler->data->dev_opened--;
	}

	if (scaler->data->dev_opened == 0) {
		if (scaler->data->irq_reged) {
			vioc_intr_disable(scaler->vioc_intr->id, scaler->vioc_intr->bits);
			free_irq(scaler->irq, scaler);
			scaler->data->irq_reged = 0;
		}

		VIOC_CONFIG_PlugOut(scaler->sc->id);

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wdma_offset), (1<<wdma_offset)); 		// WDMA reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wmix_offset), (1<<wmix_offset));		// WMIX reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<sc_offset), (1<<sc_offset));		// SCALER reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<rdma_offset), (1<<rdma_offset));		// RDMA reset

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<rdma_offset), (0<<rdma_offset));		// RDMA reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<sc_offset), (0<<sc_offset));		// SCALER reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wmix_offset), (0<<wmix_offset));		// WMIX reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wdma_offset), (0<<wdma_offset));		// WDMA reset
	}

	dprintk("%s() : Out - open(%d). \n", __func__, scaler->data->dev_opened);
}

void TCC_VIQE_Scaler_Sub_Init60Hz_M2M(void)
{
	int ret = 0;

	if (!scaler_sub->data->irq_reged) {
#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
		// Don't STOP_REQ clear because of dual output problem(DISP FIFO under-run), when video is out.
#else
		vioc_config_stop_req(1);
#endif

		synchronize_irq(scaler_sub->irq);
		vioc_intr_clear(scaler_sub->vioc_intr->id, scaler_sub->vioc_intr->bits);
		ret = request_irq(scaler_sub->irq, TCC_VIQE_Scaler_Sub_Handler60Hz_M2M, IRQF_SHARED, "scaler3", scaler_sub);

		if (ret) printk("failed to acquire scaler3 request_irq. \n");

		vioc_intr_enable(scaler_sub->vioc_intr->id, scaler_sub->vioc_intr->bits);
		scaler_sub->data->irq_reged = 1;
	}

	scaler_sub->data->dev_opened++;

	current_buffer_idx[1] = 0;

	dprintk("%s() : Out - open(%d). \n", __func__, scaler_sub->data->dev_opened);
}

void TCC_VIQE_Scaler_Sub_Run60Hz_M2M(struct tcc_lcdc_image_update* input_image)
{
        VIOC_SCALER_INFO_Type pScalerInfo;
        unsigned int pSrcBase0 = 0, pSrcBase1 = 0, pSrcBase2 = 0;
        unsigned int crop_width;

	 unsigned int dest_fmt = TCC_LCDC_IMG_FMT_YUYV;
	 //unsigned int dest_fmt = TCC_LCDC_IMG_FMT_YUV420ITL0;
	 //unsigned int dest_fmt = TCC_LCDC_IMG_FMT_COMP;

#ifndef CONFIG_OF_VIOC
        volatile PVIOC_RDMA pSC_RDMABase = (PVIOC_RDMA)scaler_sub->rdma->reg;
        //volatile PVIOC_WMIX pSC_WMIXBase = (PVIOC_WMIX)scaler_sub->wmix->reg;
        volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler_sub->wdma->reg;
        //volatile PVIOC_IREQ_CONFIG pSC_IREQConfig = (PVIOC_IREQ_CONFIG)io_p2v(TCC_PA_VIOC_CFGINT);
#endif

	dprintk("%s() : IN.\n", __func__);

	memcpy(scaler_sub->info, (struct tcc_lcdc_image_update*)input_image, sizeof(struct tcc_lcdc_image_update));
	memcpy(&output_m2m_image_sub, (struct tcc_lcdc_image_update*)input_image, sizeof(struct tcc_lcdc_image_update));
	dprintk("%s : %d(%d), %d(%d)\n",__func__, output_m2m_image_sub.buffer_unique_id, output_m2m_image_sub.odd_first_flag, input_image->buffer_unique_id, input_image->odd_first_flag);
        crop_width = scaler_sub->info->crop_right - scaler_sub->info->crop_left;

        if (scaler_sub->settop_support) {
                VIOC_RDMA_SetImageAlphaSelect(pSC_RDMABase, 1);
                VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
        } else {
                VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
        }
        VIOC_RDMA_SetImageFormat(pSC_RDMABase, scaler_sub->info->fmt);

        //interlaced frame process ex) MPEG2
        if (0) {//(scaler_sub->info->interlaced) {
                VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler_sub->info->crop_right - scaler_sub->info->crop_left), (scaler_sub->info->crop_bottom - scaler_sub->info->crop_top)/2);
                VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler_sub->info->fmt, scaler_sub->info->Frame_width*2);
        } else {
                VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler_sub->info->crop_right - scaler_sub->info->crop_left), (scaler_sub->info->crop_bottom - scaler_sub->info->crop_top));
                VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler_sub->info->fmt, scaler_sub->info->Frame_width);
        }

        pSrcBase0 = (unsigned int)scaler_sub->info->addr0;
        pSrcBase1 = (unsigned int)scaler_sub->info->addr1;
        pSrcBase2 = (unsigned int)scaler_sub->info->addr2;
        if (scaler_sub->info->fmt >= SC_IMG_FMT_YCbCr444_SEP_YUV) { // address limitation!!
                dprintk("%s():  src addr is not allocate. \n", __func__);
                scaler_sub->info->crop_left       = (scaler_sub->info->crop_left>>3)<<3;
                scaler_sub->info->crop_right = scaler_sub->info->crop_left + crop_width;
                scaler_sub->info->crop_right = scaler_sub->info->crop_left + (scaler_sub->info->crop_right - scaler_sub->info->crop_left);
                tccxxx_GetAddress(scaler_sub->info->fmt, (unsigned int)scaler_sub->info->addr0,                 \
                                                                        scaler_sub->info->Frame_width, scaler_sub->info->Frame_height,        \
                                                                        scaler_sub->info->crop_left, scaler_sub->info->crop_top,            \
                                                                        &pSrcBase0, &pSrcBase1, &pSrcBase2);
        }
        #if defined(CONFIG_ARCH_TCC896X)
        VIOC_RDMA_SetImageR2YEnable(pSC_RDMABase, 0);
        #endif//
        VIOC_RDMA_SetImageY2REnable(pSC_RDMABase, 0);
        VIOC_RDMA_SetImageBase(pSC_RDMABase, (unsigned int)pSrcBase0, (unsigned int)pSrcBase1, (unsigned int)pSrcBase2);

        pScalerInfo.BYPASS                      = 0;
        pScalerInfo.DST_WIDTH           = scaler_sub->info->Image_width;
        pScalerInfo.DST_HEIGHT          = scaler_sub->info->Image_height;
        pScalerInfo.OUTPUT_POS_X                = 0;
        pScalerInfo.OUTPUT_POS_Y                = 0;
        pScalerInfo.OUTPUT_WIDTH                = pScalerInfo.DST_WIDTH;
        pScalerInfo.OUTPUT_HEIGHT       = pScalerInfo.DST_HEIGHT;
        VIOC_API_SCALER_SetConfig(scaler_sub->sc->id, &pScalerInfo);
        VIOC_API_SCALER_SetPlugIn(scaler_sub->sc->id, scaler_sub->sc->path);
        VIOC_API_SCALER_SetUpdate(scaler_sub->sc->id);

        VIOC_RDMA_SetImageEnable(pSC_RDMABase); // SoC guide info.

#if 0
        VIOC_CONFIG_WMIXPath(scaler_sub->wmix->path, 1); // wmixer op is mixing mode.
        VIOC_WMIX_SetSize(pSC_WMIXBase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);
        VIOC_WMIX_SetUpdate(pSC_WMIXBase);
#else
        VIOC_CONFIG_WMIXPath(scaler_sub->wmix->path, 0); // wmixer op is bypass mode.
#endif

        VIOC_WDMA_SetImageFormat(pSC_WDMABase, dest_fmt);
        VIOC_WDMA_SetImageSize(pSC_WDMABase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);
        VIOC_WDMA_SetImageOffset(pSC_WDMABase, dest_fmt, pScalerInfo.DST_WIDTH);
		if(scaler_sub->info->dst_addr0 == 0)
			scaler_sub->info->dst_addr0 = TCC_VIQE_Scaler_Sub_Get_Buffer_M2M(input_image);
        VIOC_WDMA_SetImageBase(pSC_WDMABase, (unsigned int)scaler_sub->info->dst_addr0, (unsigned int)scaler_sub->info->dst_addr1, (unsigned int)scaler_sub->info->dst_addr2);

        VIOC_WDMA_SetImageR2YEnable(pSC_WDMABase, 0);
        VIOC_WDMA_SetImageEnable(pSC_WDMABase, 0);
        pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF; // wdma status register all clear.

        output_m2m_image_sub.Frame_width = pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image_sub.Frame_height = pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image_sub.Image_width = pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image_sub.Image_height = pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image_sub.crop_left= 0;
        output_m2m_image_sub.crop_top= 0;
        output_m2m_image_sub.crop_right= pScalerInfo.OUTPUT_WIDTH;
        output_m2m_image_sub.crop_bottom= pScalerInfo.OUTPUT_HEIGHT;
        output_m2m_image_sub.fmt = dest_fmt;
        output_m2m_image_sub.addr0 = scaler_sub->info->dst_addr0;

	dprintk("%s() : OUT.\n", __func__);
}

void TCC_VIQE_Scaler_Sub_Release60Hz_M2M(void)
{
	unsigned int rdma_offset = scaler_sub->rdma->id;
	unsigned int sc_offset = scaler_sub->sc->id + 28;
	unsigned int wdma_offset = scaler_sub->wdma->id;
	//unsigned int wmix_offset = scaler_sub->wmix->id + 9;

	dprintk("%s():  In -open(%d), block(%d), wait(%d), cmd(%d), irq(%d) \n", __func__, scaler_sub->data->dev_opened, scaler_sub->data->block_operating, \
			scaler_sub->data->block_waiting, scaler_sub->data->cmd_count, scaler_sub->data->irq_reged);

	if (scaler_sub->data->dev_opened > 0) {
		scaler_sub->data->dev_opened--;
	}

	if (scaler_sub->data->dev_opened == 0) {
		if (scaler_sub->data->irq_reged) {
			vioc_intr_disable(scaler_sub->vioc_intr->id, scaler_sub->vioc_intr->bits);
			free_irq(scaler_sub->irq, scaler_sub);
			scaler_sub->data->irq_reged = 0;
		}

		VIOC_CONFIG_PlugOut(scaler_sub->sc->id);

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wdma_offset), (1<<wdma_offset)); 		// WDMA reset
// To support Dual-vsyn on LCD with HDMI, Needless codes because Bypass mode is ON
//		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wmix_offset), (1<<wmix_offset));		// WMIX reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<sc_offset), (1<<sc_offset));		// SCALER reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<rdma_offset), (1<<rdma_offset));		// RDMA reset

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<rdma_offset), (0<<rdma_offset));		// RDMA reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (1<<sc_offset), (0<<sc_offset));		// SCALER reset
// To support Dual-vsyn on LCD with HDMI, Needless codes because Bypass mode is ON
//		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wmix_offset), (0<<wmix_offset));		// WMIX reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (1<<wdma_offset), (0<<wdma_offset));		// WDMA reset
	}

	dprintk("%s() : Out - open(%d). \n", __func__, scaler_sub->data->dev_opened);
}

extern int tcc_vsync_isVsyncRunning(int idx);
irqreturn_t TCC_VIQE_Scaler_Handler60Hz_M2M(int irq, void *client_data)
{
#ifndef CONFIG_OF_VIOC
        volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler->wdma->reg;
#endif

	if(tcc_vsync_isVsyncRunning(0) == 0){
		return IRQ_HANDLED;
	}

        if (is_vioc_intr_activatied(scaler->vioc_intr->id, scaler->vioc_intr->bits) == false)
                return IRQ_NONE;

        if(pSC_WDMABase->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFR_MASK) {
                dprintk("Main WDMA Interrupt is VIOC_WDMA_IREQ_EOFR_MASK. \n");
                pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF;   // wdma status register all clear.
			TCC_VIQE_Display_Update60Hz_M2M(&output_m2m_image);
        }

       return IRQ_HANDLED;
}

irqreturn_t TCC_VIQE_Scaler_Sub_Handler60Hz_M2M(int irq, void *client_data)
{
#ifndef CONFIG_OF_VIOC
	volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler_sub->wdma->reg;
#endif

	if(tcc_vsync_isVsyncRunning(1) == 0){
		return IRQ_HANDLED;
	}

	if (is_vioc_intr_activatied(scaler_sub->vioc_intr->id, scaler_sub->vioc_intr->bits) == false)
		return IRQ_NONE;

	if(pSC_WDMABase->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFR_MASK) {
		dprintk("Sub WDMA Interrupt is VIOC_WDMA_IREQ_EOFR_MASK. \n");
		pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF;   // wdma status register all clear.

		TCC_VIQE_Display_Update60Hz_M2M(&output_m2m_image_sub);
	}

	return IRQ_HANDLED;
}

void TCC_VIQE_Display_Update60Hz_M2M(struct tcc_lcdc_image_update *input_image)
{
//	printk("update IN\n");
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;

	tccfb_info = info->par;
	if(tccfb_info->pdata.Mdp_data.DispDeviceType == gOutputMode)
		dp_device = &tccfb_info->pdata.Mdp_data;
	else if(tccfb_info->pdata.Sdp_data.DispDeviceType == gOutputMode)
		dp_device = &tccfb_info->pdata.Sdp_data;
	else {
		printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
			tccfb_info->pdata.Mdp_data.DispDeviceType,
			tccfb_info->pdata.Sdp_data.DispDeviceType,
			gOutputMode);
		return;
	}

	if(!input_image->first_frame_after_seek)
	{
		switch(gOutputMode)
		{
			case TCC_OUTPUT_NONE:
				break;

			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			case TCC_OUTPUT_LCD:
				tca_scale_display_update(dp_device, input_image);
				break;
			#endif

			case TCC_OUTPUT_HDMI:
				dprintk("%s : update %d, %d, %d\n",__func__, input_image->buffer_unique_id, input_image->addr0, input_image->odd_first_flag);
				tca_scale_display_update(dp_device, input_image);
				break;
			case TCC_OUTPUT_COMPONENT:
				#if defined(CONFIG_FB_TCC_COMPONENT)
					tcc_component_update(input_image);
				#endif
				break;
			case TCC_OUTPUT_COMPOSITE:
				#if defined(CONFIG_FB_TCC_COMPOSITE)
					tcc_composite_update(input_image);
				#endif
				break;

			default:
				break;
		}
	}
//	printk("update OUT\n");
}
/////////////////////////////////////////////////////////////////////////////////////////

/* 
	//img_fmt
	TCC_LCDC_IMG_FMT_YUV420SP = 24,	
	TCC_LCDC_IMG_FMT_YUV422SP = 25, 
	TCC_LCDC_IMG_FMT_YUYV = 26, 
	TCC_LCDC_IMG_FMT_YVYU = 27,
	
	TCC_LCDC_IMG_FMT_YUV420ITL0 = 28, 
	TCC_LCDC_IMG_FMT_YUV420ITL1 = 29, 
	TCC_LCDC_IMG_FMT_YUV422ITL0 = 30, 
	TCC_LCDC_IMG_FMT_YUV422ITL1 = 31, 
*/

//////////////////////////////////////////////////////////////////////////////////////////
void TCC_VIQE_DI_Init60Hz(TCC_OUTPUT_TYPE outputMode, int lcdCtrlNum, struct tcc_lcdc_image_update *input_image)
{
	unsigned int deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3;
	int imgSize;
	int top_size_dont_use = OFF;		//If this value is OFF, The size information is get from VIOC modules.
	unsigned int framebufWidth, framebufHeight;
	unsigned int lcd_width = 0, lcd_height = 0, scale_x = 0, scale_y = 0;

	gLcdc_layer_60Hz = 3;

	if(input_image->fmt == 24 || input_image->fmt == 28 || input_image->fmt==29)
		gViqe_fmt_60Hz = VIOC_VIQE_FMT_YUV420;
	else
		gViqe_fmt_60Hz = VIOC_VIQE_FMT_YUV422;
	gImg_fmt_60Hz = input_image->fmt;
		
	pmap_get_info("viqe", &pmap_viqe);
	gPMEM_VIQE_BASE = pmap_viqe.base;
	gPMEM_VIQE_SIZE = pmap_viqe.size;

	gOutputMode = outputMode;

#ifdef MAX_VIQE_FRAMEBUFFER
	framebufWidth = 1920;
	framebufHeight = 1088;
#else
	framebufWidth = ((input_image->Frame_width) >> 3) << 3;			// 8bit align
	framebufHeight = ((input_image->Frame_height) >> 2) << 2;		// 4bit align
#endif

	printk("TCC_VIQE_DI_Init60Hz, W:%d, H:%d, DW:%d, DH:%d, FMT:%d, VFMT:%s, OddFirst:%d\n", framebufWidth, framebufHeight, input_image->Image_width, input_image->Image_height, input_image->fmt, (gViqe_fmt_60Hz?"YUV422":"YUV420"), input_image->odd_first_flag);

	if(gOutputMode == TCC_OUTPUT_LCD)
		VIOC_DISP_GetSize(viqe_60hz_lcd_info.pDISPBase_60Hz, &lcd_width, &lcd_height);
	else
		VIOC_DISP_GetSize(viqe_60hz_external_info.pDISPBase_60Hz, &lcd_width, &lcd_height);

	if((!lcd_width) || (!lcd_height))
	{
		printk("%s invalid lcd size\n", __func__);
		return;
	}

	#if defined(USE_DEINTERLACE_S_IN60Hz)
		gDeintlS_Use_60Hz = 1;
	#else
		#if defined(DYNAMIC_USE_DEINTL_COMPONENT)
			if((framebufWidth >= 1280) && (framebufHeight >= 720))
			{
				gDeintlS_Use_60Hz = 1;
				gDI_mode_60Hz = VIOC_VIQE_DEINTL_S;
			}
			else
			{
				gDeintlS_Use_60Hz = 0;
				gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
			}
		#endif
	#endif

	if(viqe_common_info.gBoard_num == 1) {
		if(viqe_60hz_external_info.pRDMABase_60Hz->uCTRL.nREG & HwDMA_IEN)
			VIOC_RDMA_SetImageDisable(viqe_60hz_external_info.pRDMABase_60Hz);

		//TCC_OUTPUT_FB_ClearVideoImg();
	}

	//RDMA SETTING
	if(gDeintlS_Use_60Hz)
	{
		if(gOutputMode == TCC_OUTPUT_LCD) {
			VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, true);
			VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
		} else {
			VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, true);
			VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
		}
	}
	else
	{
		#if 0
		if( gDI_mode == VIOC_VIQE_DEINTL_MODE_BYPASS)
		{
			VIOC_RDMA_SetImageY2REnable(viqe_common_info.pRDMABase_60Hz, true);
			VIOC_RDMA_SetImageY2RMode(viqe_common_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
		}
		else
		#endif
		{
			if(gOutputMode == TCC_OUTPUT_LCD) {
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, false);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
			} else {
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, false);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
			}
		}
	}

	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_RDMA_SetImageOffset(viqe_60hz_lcd_info.pRDMABase_60Hz, input_image->fmt, framebufWidth);
		VIOC_RDMA_SetImageFormat(viqe_60hz_lcd_info.pRDMABase_60Hz, input_image->fmt);
		VIOC_RDMA_SetImageScale(viqe_60hz_lcd_info.pRDMABase_60Hz, scale_x, scale_y);
		VIOC_RDMA_SetImageSize(viqe_60hz_lcd_info.pRDMABase_60Hz, framebufWidth, framebufHeight);
		VIOC_RDMA_SetImageIntl(viqe_60hz_lcd_info.pRDMABase_60Hz, 1);
		VIOC_RDMA_SetImageBfield(viqe_60hz_lcd_info.pRDMABase_60Hz, input_image->odd_first_flag);
	} else {
		VIOC_RDMA_SetImageOffset(viqe_60hz_external_info.pRDMABase_60Hz,input_image->fmt, framebufWidth);
		VIOC_RDMA_SetImageFormat(viqe_60hz_external_info.pRDMABase_60Hz, input_image->fmt);
		VIOC_RDMA_SetImageScale(viqe_60hz_external_info.pRDMABase_60Hz, scale_x, scale_y);
		VIOC_RDMA_SetImageSize(viqe_60hz_external_info.pRDMABase_60Hz, framebufWidth, framebufHeight);
		VIOC_RDMA_SetImageIntl(viqe_60hz_external_info.pRDMABase_60Hz, 1);
		VIOC_RDMA_SetImageBfield(viqe_60hz_external_info.pRDMABase_60Hz, input_image->odd_first_flag);
	}

	if(gDeintlS_Use_60Hz)
	{
		printk("%s, DeinterlacerS is used!!\n", __func__);
		
		deintl_dma_base0	= 0;
		deintl_dma_base1	= 0;
		deintl_dma_base2	= 0;
		deintl_dma_base3	= 0;
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
		#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
			S_DI_writeb(3, tcc_p2v(S_DI_BIT));
		#endif
		if(gOutputMode == TCC_OUTPUT_LCD) {
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, viqe_60hz_lcd_info.gVIQE_RDMA_num_60Hz);
		} else {
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, viqe_60hz_external_info.gVIQE_RDMA_num_60Hz);
		}
		gDeintlS_Use_Plugin = 1;
	}
	else
	{
		printk("%s, VIQE is used!!\n", __func__);
		
		// If you use 3D(temporal) De-interlace mode, you have to set physical address for using DMA register.
		//If 2D(spatial) mode, these registers are ignored
		imgSize = (framebufWidth * framebufHeight * 2);
		deintl_dma_base0	= gPMEM_VIQE_BASE;
		deintl_dma_base1	= deintl_dma_base0 + imgSize;
		deintl_dma_base2	= deintl_dma_base1 + imgSize;
		deintl_dma_base3	= deintl_dma_base2 + imgSize;	

		VIOC_VIQE_SetControlRegister(viqe_common_info.pVIQE0, framebufWidth, framebufHeight, gViqe_fmt_60Hz);
		VIOC_VIQE_SetDeintlRegister(viqe_common_info.pVIQE0, gViqe_fmt_60Hz, top_size_dont_use, framebufWidth, framebufHeight, gDI_mode_60Hz, deintl_dma_base0, deintl_dma_base1, deintl_dma_base2, deintl_dma_base3);
		//VIOC_VIQE_SetDenoise(viqe_common_info.pVIQE0, gViqe_fmt_60Hz, framebufWidth, framebufHeight, 1, 0, deintl_dma_base0, deintl_dma_base1); 	//for bypass path on progressive frame
		VIOC_VIQE_SetControlEnable(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, ON);
		//if(gDI_mode != VIOC_VIQE_DEINTL_MODE_BYPASS)
		{
		#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
			if(gOutputMode != TCC_OUTPUT_HDMI || hdmi_get_hdmimode())
				VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE0, false);
			else
				VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE0, true);
		#elif defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
			if(gOutputMode == TCC_OUTPUT_COMPOSITE)
				VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE0, false);
			else
				VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE0, true);
		#else
			VIOC_VIQE_SetImageY2REnable(viqe_common_info.pVIQE0, true);
		#endif
			VIOC_VIQE_SetImageY2RMode(viqe_common_info.pVIQE0, 0x02);
		}

		if(gOutputMode == TCC_OUTPUT_LCD) {
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE0, viqe_60hz_lcd_info.gVIQE_RDMA_num_60Hz);
		} else {
			VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_VIQE0, viqe_60hz_external_info.gVIQE_RDMA_num_60Hz);
		}
	}

	//SCALER SETTING
	if(input_image->on_the_fly)
	{
		if(gOutputMode == TCC_OUTPUT_LCD) {
			VIOC_CONFIG_PlugOut(viqe_60hz_lcd_info.gSCALER_num_60Hz);
			VIOC_CONFIG_PlugIn (viqe_60hz_lcd_info.gSCALER_num_60Hz, viqe_60hz_lcd_info.gSC_RDMA_num_60Hz);
			VIOC_SC_SetSWReset(viqe_60hz_lcd_info.gSCALER_num_60Hz, 0xFF, 0xFF);
			VIOC_SC_SetBypass (viqe_60hz_lcd_info.pSCALERBase_60Hz, OFF);

			VIOC_SC_SetDstSize (viqe_60hz_lcd_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (viqe_60hz_lcd_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set output size in scaer
			VIOC_SC_SetUpdate (viqe_60hz_lcd_info.pSCALERBase_60Hz);
		} else {
			VIOC_CONFIG_PlugOut(viqe_60hz_external_info.gSCALER_num_60Hz);
			VIOC_CONFIG_PlugIn (viqe_60hz_external_info.gSCALER_num_60Hz, viqe_60hz_external_info.gSC_RDMA_num_60Hz);
			VIOC_SC_SetSWReset(viqe_60hz_external_info.gSCALER_num_60Hz, 0xFF, 0xFF);
			VIOC_SC_SetBypass (viqe_60hz_external_info.pSCALERBase_60Hz, OFF);

			VIOC_SC_SetDstSize (viqe_60hz_external_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (viqe_60hz_external_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set output size in scaer
			VIOC_SC_SetUpdate (viqe_60hz_external_info.pSCALERBase_60Hz);
		}
		gusingScale_60Hz = 1;
	}
	
	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_WMIX_SetPosition(viqe_60hz_lcd_info.pWMIXBase_60Hz, gLcdc_layer_60Hz,  input_image->offset_x, input_image->offset_y);
		VIOC_WMIX_SetUpdate(viqe_60hz_lcd_info.pWMIXBase_60Hz);
	} else {
		VIOC_WMIX_SetPosition(viqe_60hz_external_info.pWMIXBase_60Hz, gLcdc_layer_60Hz,  input_image->offset_x, input_image->offset_y);
		VIOC_WMIX_SetUpdate(viqe_60hz_external_info.pWMIXBase_60Hz);
	}

	gFrmCnt_60Hz= 0;		

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	if(gOutputMode == TCC_OUTPUT_LCD)
	{
		if(viqe_60hz_lcd_info.gSC_RDMA_num_60Hz < VIOC_SC_RDMA_04)
			pLastFrame_RDMABase = pRDMATempBase_60Hz = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA01);
		else
			pLastFrame_RDMABase = pRDMATempBase_60Hz = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA05);
	}
	else
	{
		if(viqe_60hz_external_info.gSC_RDMA_num_60Hz < VIOC_SC_RDMA_04)
			pLastFrame_RDMABase = pRDMATempBase_60Hz = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA01);
		else
			pLastFrame_RDMABase = pRDMATempBase_60Hz = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA05);
	}
#endif
	
	gVIQE_Init_State = 1;
}


void TCC_VIQE_DI_Swap60Hz(int mode)
{
	int ret;
	VIOC_PlugInOutCheck VIOC_PlugIn;

	ret = VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE0, &VIOC_PlugIn);
	if (ret == VIOC_DEVICE_CONNECTED && VIOC_PlugIn.enable)
		VIOC_VIQE_SwapDeintlBase(viqe_common_info.pVIQE0, mode);
}
void TCC_VIQE_DI_SetFMT60Hz(int enable)
{
	int ret;
	VIOC_PlugInOutCheck VIOC_PlugIn;

	ret = VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE0, &VIOC_PlugIn);
	if (ret == VIOC_DEVICE_CONNECTED && VIOC_PlugIn.enable)
		VIOC_VIQE_SetDeintlFMT(viqe_common_info.pVIQE0, enable);
}

void TCC_VIQE_DI_Run60Hz(struct tcc_lcdc_image_update *input_image, int reset_frmCnt)
{
	unsigned int lcd_width = 0, lcd_height = 0;
	int cropWidth, cropHeight;
	int crop_top = input_image->crop_top;
	int crop_bottom = input_image->crop_bottom;
	int crop_left = input_image->crop_left;
	int crop_right = input_image->crop_right;

#ifndef USE_DEINTERLACE_S_IN60Hz
	int ret;
	VIOC_PlugInOutCheck VIOC_PlugIn;
	unsigned int JudderCnt = 0;
#endif

	if(viqe_common_info.gBoard_num == 1) {
		if(gVIQE_Init_State == 0)
		{
			dprintk("%s VIQE block isn't initailized\n", __func__);
			return;
		}
	}

#ifndef USE_DEINTERLACE_S_IN60Hz
	ret = VIOC_CONFIG_Device_PlugState(viqe_common_info.gVIOC_VIQE0, &VIOC_PlugIn);
	if (ret == VIOC_DEVICE_CONNECTED && VIOC_PlugIn.enable) {

	}
	else {
		dprintk("%s VIQE block isn't pluged!!!\n", __func__);
		return;		
	}
#endif

	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_DISP_GetSize(viqe_60hz_lcd_info.pDISPBase_60Hz, &lcd_width, &lcd_height);
	} else {
		VIOC_DISP_GetSize(viqe_60hz_external_info.pDISPBase_60Hz, &lcd_width, &lcd_height);
	}

	if((!lcd_width) || (!lcd_height))
	{
		printk("%s invalid lcd size\n", __func__);
		return;
	}

	if(reset_frmCnt){
		gFrmCnt_60Hz = 0;
		dprintk("VIQE 3D -> 2D \n");
	}
	
	crop_top = (crop_top >>1)<<1;
	cropWidth = ((crop_right - crop_left) >> 3) << 3;		//8bit align
	cropHeight = ((crop_bottom - crop_top) >> 2) << 2;		//4bit align
	{
		int addr_Y = (unsigned int)input_image->addr0;
		int addr_U = (unsigned int)input_image->addr1;
		int addr_V = (unsigned int)input_image->addr2;

		tccxxx_GetAddress(gImg_fmt_60Hz, (unsigned int)input_image->addr0, input_image->Frame_width, input_image->Frame_height, crop_left, crop_top, &addr_Y, &addr_U, &addr_V);

		if(gOutputMode == TCC_OUTPUT_LCD) {
			VIOC_RDMA_SetImageSize(viqe_60hz_lcd_info.pRDMABase_60Hz, cropWidth, cropHeight);
			VIOC_RDMA_SetImageBase(viqe_60hz_lcd_info.pRDMABase_60Hz, addr_Y, addr_U, addr_V);
			VIOC_RDMA_SetImageOffset(viqe_60hz_lcd_info.pRDMABase_60Hz, gImg_fmt_60Hz, input_image->Frame_width);
		} else {
			VIOC_RDMA_SetImageSize(viqe_60hz_external_info.pRDMABase_60Hz, cropWidth, cropHeight);
			VIOC_RDMA_SetImageBase(viqe_60hz_external_info.pRDMABase_60Hz, addr_Y, addr_U, addr_V);
			VIOC_RDMA_SetImageOffset(viqe_60hz_external_info.pRDMABase_60Hz, gImg_fmt_60Hz, input_image->Frame_width);
		}

		gpreCrop_left_60Hz = crop_left;
		gpreCrop_right_60Hz = crop_right;
		gpreCrop_top_60Hz = crop_top;
		gpreCrop_bottom_60Hz = crop_bottom;
	}

	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_RDMA_SetImageBfield(viqe_60hz_lcd_info.pRDMABase_60Hz, input_image->odd_first_flag);
	} else {
		VIOC_RDMA_SetImageBfield(viqe_60hz_external_info.pRDMABase_60Hz, input_image->odd_first_flag);
	}
	
	dprintk(" Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d], W:%d, H:%d odd:%d\n", crop_left, crop_right, crop_top, crop_bottom, cropWidth, cropHeight, input_image->odd_first_flag);

	if(gDeintlS_Use_60Hz)
	{
		if(gOutputMode == TCC_OUTPUT_LCD) {
			//VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, false);
			//VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */

			if(input_image->frameInfo_interlace)
			{
				VIOC_RDMA_SetImageIntl(viqe_60hz_lcd_info.pRDMABase_60Hz, 1);
				if(!gDeintlS_Use_Plugin)
				{
					VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, viqe_60hz_lcd_info.gVIQE_RDMA_num_60Hz);
					gDeintlS_Use_Plugin = 1;
				}
			}
			else
			{
				VIOC_RDMA_SetImageIntl(viqe_60hz_lcd_info.pRDMABase_60Hz, 0);
				if(gDeintlS_Use_Plugin)
				{
					VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
					gDeintlS_Use_Plugin = 0;
				}
			}
		} else {
			//VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, false);
			//VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */

			if(input_image->frameInfo_interlace)
			{
				VIOC_RDMA_SetImageIntl(viqe_60hz_external_info.pRDMABase_60Hz, 1);
				if(!gDeintlS_Use_Plugin)
				{
					VIOC_CONFIG_PlugIn(viqe_common_info.gVIOC_Deintls, viqe_60hz_external_info.gVIQE_RDMA_num_60Hz);
					gDeintlS_Use_Plugin = 1;
				}
			}
			else
			{
				VIOC_RDMA_SetImageIntl(viqe_60hz_external_info.pRDMABase_60Hz, 0);
				if(gDeintlS_Use_Plugin)
				{
					VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
					gDeintlS_Use_Plugin = 0;
				}
			}		
		}
	}
	else
	{
		if(gOutputMode == TCC_OUTPUT_LCD) {
			if(1)//(input_image->frameInfo_interlace)
			{
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, false);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
				if(gFrmCnt_60Hz >= 3)
				{
					VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_3D);
					#ifndef USE_DEINTERLACE_S_IN60Hz
						// Process JUDDER CNTS
						JudderCnt = (input_image->Frame_width + 64)/64 -1;
						VIOC_VIQE_SetDeintlJudderCnt(viqe_common_info.pVIQE0, JudderCnt);
					#endif
					gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_3D;
				}
				else
				{
					VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_2D);
					gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
				}
				VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, ON);
				VIOC_RDMA_SetImageIntl(viqe_60hz_lcd_info.pRDMABase_60Hz, 1);
			}
			else
			{
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, true);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
				VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, OFF);
				VIOC_RDMA_SetImageIntl(viqe_60hz_lcd_info.pRDMABase_60Hz, 0);
				gFrmCnt_60Hz = 0;
			}
		} else {
			if(1)//(input_image->frameInfo_interlace)
			{
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, false);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
				if(gFrmCnt_60Hz >= 3)
				{
					VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_3D);
					#ifndef USE_DEINTERLACE_S_IN60Hz
						// Process JUDDER CNTS
						JudderCnt = (input_image->Frame_width + 64)/64 -1;
						VIOC_VIQE_SetDeintlJudderCnt(viqe_common_info.pVIQE0, JudderCnt);
					#endif
					gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_3D;
				}
				else
				{
					VIOC_VIQE_SetDeintlMode(viqe_common_info.pVIQE0, VIOC_VIQE_DEINTL_MODE_2D);
					gDI_mode_60Hz = VIOC_VIQE_DEINTL_MODE_2D;
				}
				VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, ON);
				VIOC_RDMA_SetImageIntl(viqe_60hz_external_info.pRDMABase_60Hz, 1);
			}
			else
			{
				VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, true);
				VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
				VIOC_VIQE_SetControlMode(viqe_common_info.pVIQE0, OFF, OFF, OFF, OFF, OFF);
				VIOC_RDMA_SetImageIntl(viqe_60hz_external_info.pRDMABase_60Hz, 0);
				gFrmCnt_60Hz = 0;
			}
		}
	}

	if(input_image->on_the_fly)
	{
		if(gOutputMode == TCC_OUTPUT_LCD) {
			if(!gusingScale_60Hz) {
				gusingScale_60Hz = 1;
				VIOC_CONFIG_PlugIn (viqe_60hz_lcd_info.gSCALER_num_60Hz, viqe_60hz_lcd_info.gSC_RDMA_num_60Hz);
				VIOC_SC_SetBypass (viqe_60hz_lcd_info.pSCALERBase_60Hz, OFF);
			}

			VIOC_SC_SetDstSize (viqe_60hz_lcd_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (viqe_60hz_lcd_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set output size in scaer
			VIOC_SC_SetUpdate (viqe_60hz_lcd_info.pSCALERBase_60Hz);
		} else {
			if(!gusingScale_60Hz) {
				gusingScale_60Hz = 1;
				VIOC_CONFIG_PlugIn (viqe_60hz_external_info.gSCALER_num_60Hz, viqe_60hz_external_info.gSC_RDMA_num_60Hz);
				VIOC_SC_SetBypass (viqe_60hz_external_info.pSCALERBase_60Hz, OFF);
			}

			VIOC_SC_SetDstSize (viqe_60hz_external_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (viqe_60hz_external_info.pSCALERBase_60Hz, input_image->Image_width, input_image->Image_height);			// set output size in scaer
			VIOC_SC_SetUpdate (viqe_60hz_external_info.pSCALERBase_60Hz);
		}
	}
	else
	{
		if(gusingScale_60Hz == 1)	{
			if(gOutputMode == TCC_OUTPUT_LCD) {	
				VIOC_RDMA_SetImageDisable(viqe_60hz_lcd_info.pRDMABase_60Hz);
				VIOC_CONFIG_PlugOut(viqe_60hz_lcd_info.gSCALER_num_60Hz);
			} else {
				VIOC_RDMA_SetImageDisable(viqe_60hz_external_info.pRDMABase_60Hz);
				VIOC_CONFIG_PlugOut(viqe_60hz_external_info.gSCALER_num_60Hz);
			}
			gusingScale_60Hz = 0;
		}
	}

	// position
	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_WMIX_SetPosition(viqe_60hz_lcd_info.pWMIXBase_60Hz, gLcdc_layer_60Hz,  input_image->offset_x, input_image->offset_y);
		VIOC_WMIX_SetUpdate(viqe_60hz_lcd_info.pWMIXBase_60Hz);

		VIOC_RDMA_SetImageEnable(viqe_60hz_lcd_info.pRDMABase_60Hz);
	} else {
		VIOC_WMIX_SetPosition(viqe_60hz_external_info.pWMIXBase_60Hz, gLcdc_layer_60Hz,  input_image->offset_x, input_image->offset_y);
		VIOC_WMIX_SetUpdate(viqe_60hz_external_info.pWMIXBase_60Hz);

		VIOC_RDMA_SetImageEnable(viqe_60hz_external_info.pRDMABase_60Hz);
	}

	tcc_video_post_process(input_image);
	gFrmCnt_60Hz++;
}

void TCC_VIQE_DI_DeInit60Hz(void)
{
	printk("%s\n", __func__);

	gusingScale_60Hz = 0;
	gFrmCnt_60Hz = 0;

	if(gOutputMode == TCC_OUTPUT_LCD) {
		VIOC_RDMA_SetImageDisable(viqe_60hz_lcd_info.pRDMABase_60Hz);
		VIOC_RDMA_SetImageY2REnable(viqe_60hz_lcd_info.pRDMABase_60Hz, true);
		VIOC_RDMA_SetImageY2RMode(viqe_60hz_lcd_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
		if(gDeintlS_Use_60Hz)
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
			gDeintlS_Use_Plugin = 0;
		}
		else
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE0);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset
		}
		VIOC_CONFIG_PlugOut(viqe_60hz_lcd_info.gSCALER_num_60Hz);
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+viqe_60hz_lcd_info.gSCALER_num_60Hz)), (0x1<<(28+viqe_60hz_lcd_info.gSCALER_num_60Hz))); // scaler reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+viqe_60hz_lcd_info.gSCALER_num_60Hz)), (0x0<<(28+viqe_60hz_lcd_info.gSCALER_num_60Hz))); // scaler reset

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<viqe_60hz_lcd_info.gWMIX_RDMA), (0x1<<viqe_60hz_lcd_info.gWMIX_RDMA)); // rdma reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<viqe_60hz_lcd_info.gWMIX_RDMA), (0x0<<viqe_60hz_lcd_info.gWMIX_RDMA)); // rdma reset
	} else {
		VIOC_RDMA_SetImageDisable(viqe_60hz_external_info.pRDMABase_60Hz);
		VIOC_RDMA_SetImageY2REnable(viqe_60hz_external_info.pRDMABase_60Hz, true);
		VIOC_RDMA_SetImageY2RMode(viqe_60hz_external_info.pRDMABase_60Hz, 0x02); /* Y2RMode Default 0 (Studio Color) */
		if(gDeintlS_Use_60Hz)
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_Deintls);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x01<<17)); // DEINTLS reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<17), (0x00<<17)); // DEINTLS reset
			gDeintlS_Use_Plugin = 0;
		}
		else
		{
			VIOC_CONFIG_PlugOut(viqe_common_info.gVIOC_VIQE0);
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x01<<16)); // VIQE reset
			BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[1], (0x1<<16), (0x00<<16)); // VIQE reset
		}
		VIOC_CONFIG_PlugOut(viqe_60hz_external_info.gSCALER_num_60Hz);
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+viqe_60hz_external_info.gSCALER_num_60Hz)), (0x1<<(28+viqe_60hz_external_info.gSCALER_num_60Hz))); // scaler reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+viqe_60hz_external_info.gSCALER_num_60Hz)), (0x0<<(28+viqe_60hz_external_info.gSCALER_num_60Hz))); // scaler reset

		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<viqe_60hz_external_info.gWMIX_RDMA), (0x1<<viqe_60hz_external_info.gWMIX_RDMA)); // rdma reset
		BITCSET(viqe_common_info.pIREQConfig->uSOFTRESET.nREG[0], (0x1<<viqe_60hz_external_info.gWMIX_RDMA), (0x0<<viqe_60hz_external_info.gWMIX_RDMA)); // rdma reset
	}

	gVIQE_Init_State = 0;
}

unsigned int tcc_viqe_commom_dt_parse(struct device_node *np, struct tcc_viqe_common_virt_addr_info_t *viqe_common_info)
{
	unsigned int index;
	struct device_node *viqe_node0, *viqe_node1, *config_node;

#ifdef CONFIG_OF
		viqe_node0 = of_parse_phandle(np,"telechips,viqe,0", 0);
		of_property_read_u32_index(np,"telechips,viqe,0", 1, &index);
		if (!viqe_node0) {
			pr_err( "could not find viqe_video viqe0 node\n");
		}
		else {
			viqe_common_info->pVIQE0= of_iomap(viqe_node0, index);
			of_property_read_u32(np, "vioc_viqe0", &viqe_common_info->gVIOC_VIQE0);
			//pr_info("viqe_video viqe0 %d %x  \n", index, viqe_common_info->pVIQE0);
		}

		viqe_node1 = of_parse_phandle(np,"telechips,viqe,1", 0);
		of_property_read_u32_index(np,"telechips,viqe,1", 1, &index);
		if (!viqe_node1) {
			pr_err( "could not find viqe_video viqe1 node\n");
			viqe_node1 = of_parse_phandle(np,"telechips,viqe,0", 0);
			of_property_read_u32_index(np,"telechips,viqe,0", 1, &index);
			if (!viqe_node1) {
				pr_err( "could not find viqe_video viqe1(viqe0) node\n");
			}
			else {
				viqe_common_info->pVIQE1= of_iomap(viqe_node0, index);
				of_property_read_u32(np, "vioc_viqe0", &viqe_common_info->gVIOC_VIQE1);	
				not_use_viqe1 = 1;
				//pr_info("viqe_video viqe1(viqe0) %d %x  \n", index, viqe_common_info->pVIQE1);
			}
		}
		else {
			viqe_common_info->pVIQE1 = of_iomap(viqe_node1, index);
			of_property_read_u32(np, "vioc_viqe1", &viqe_common_info->gVIOC_VIQE1);
			//pr_info("viqe_video viqe1 %d %x  \n", index, viqe_common_info->pVIQE1);
		}

		config_node = of_parse_phandle(np, "telechips,config", 0);
		of_property_read_u32_index(np,"telechips,config", 1, &index);
		if (!config_node) {
			pr_err( "could not find viqe_video config node\n");
		}
		else {
			viqe_common_info->pIREQConfig= of_iomap(config_node, index);
			//pr_info("viqe_video config %d %x  \n", index, viqe_common_info->pIREQConfig);
		}

		of_property_read_u32(np, "vioc_deintls", &viqe_common_info->gVIOC_Deintls);
		of_property_read_u32(np, "board_num", &viqe_common_info->gBoard_num);

	iprintk("VIQE Common Information. \n");
	iprintk("Deintls(%d) / Viqe0(%d) / Viqe1(%d) / gBoard_num(%d) \n", viqe_common_info->gVIOC_Deintls, viqe_common_info->gVIOC_VIQE0, viqe_common_info->gVIOC_VIQE1, viqe_common_info->gBoard_num);

#endif

	return 0;
}

unsigned int tcc_viqe_60hz_dt_parse(struct device_node *np, struct tcc_viqe_60hz_virt_addr_info_t *viqe_60hz_info)
{
	unsigned int index;
	struct device_node *rdma_node, *wmixer_node, *disp_node, *scaler_node;

#ifdef CONFIG_OF
	rdma_node = of_parse_phandle(np,"telechips,rdma,60", 0);
	of_property_read_u32_index(np,"telechips,rdma,60", 1, &index);
	if (!rdma_node) {
		pr_err( "could not find viqe_video rdma_60hz node\n");
	}
	else {
		viqe_60hz_info->pRDMABase_60Hz = of_iomap(rdma_node, index);
		//pr_info("viqe_video rdma_60hz %d %x  \n", index, viqe_60hz_info->pRDMABase_60Hz);
	}

	wmixer_node = of_parse_phandle(np,"telechips,wmixer", 0);
	of_property_read_u32_index(np,"telechips,wmixer", 1, &index);
	if (!wmixer_node) {
		pr_err( "could not find viqe_video wmixer node\n");
	}
	else {
		viqe_60hz_info->pWMIXBase_60Hz = of_iomap(wmixer_node, index);
		//pr_info("viqe_video wmixe_60hz %d %x  \n", index, viqe_60hz_info->pWMIXBase_60Hz);
	}

	disp_node = of_parse_phandle(np, "telechips,disp", 0);
	of_property_read_u32_index(np,"telechips,disp", 1, &index);
	if (!disp_node) {
		pr_err( "could not find viqe_video display node\n");
	}
	else {
		viqe_60hz_info->pDISPBase_60Hz= of_iomap(disp_node, index);
		//pr_info("viqe_video display_60hz %d %x  \n", index, viqe_60hz_info->pDISPBase_60Hz);
	}

	scaler_node = of_parse_phandle(np, "telechips,sc", 0);
	of_property_read_u32_index(np,"telechips,sc", 1, &index);
	if (!scaler_node) {
		pr_err( "could not find viqe_video scaler node\n");
	}
	else {
		viqe_60hz_info->pSCALERBase_60Hz= of_iomap(scaler_node, index);
		//pr_info("viqe_video scaler_60hz %d %x  \n", index, viqe_60hz_info->pSCALERBase_60Hz);
	}

	of_property_read_u32(np, "viqe_rdma_num_60", &viqe_60hz_info->gVIQE_RDMA_num_60Hz);
	of_property_read_u32(np, "scaler_num_60", &viqe_60hz_info->gSCALER_num_60Hz);
	of_property_read_u32(np, "sc_rdma_num_60", &viqe_60hz_info->gSC_RDMA_num_60Hz);
	of_property_read_u32(np, "wmix_rdma", &viqe_60hz_info->gWMIX_RDMA);

	iprintk("VIQE_60Hz Information. \n");
	iprintk("Reg_Base :: RDMA / Scaler / WMIX / DISP = 0x%8x / 0x%8x / 0x%8x / 0x%8x \n",
			viqe_60hz_info->pRDMABase_60Hz, viqe_60hz_info->pSCALERBase_60Hz, viqe_60hz_info->pWMIXBase_60Hz, viqe_60hz_info->pDISPBase_60Hz);
	iprintk("NUM :: Viqe_rdma / SC / SC_rdma / Wmix_rdma = %d / %d / %d / %d \n",
			viqe_60hz_info->gVIQE_RDMA_num_60Hz, viqe_60hz_info->gSCALER_num_60Hz, viqe_60hz_info->gSC_RDMA_num_60Hz, viqe_60hz_info->gWMIX_RDMA);
#endif

	return 0;
}

unsigned int tcc_viqe_m2m_dt_parse(struct device_node *main_deIntl, struct device_node *sub_deIntl, struct device_node *sc_main, struct device_node *sc_sub,
	struct tcc_viqe_m2m_virt_addr_info_t *main_m2m_info, struct tcc_viqe_m2m_virt_addr_info_t *sub_m2m_info)
{
	unsigned int index;
	struct device_node *main_deintl_rdma_node, *sub_deintl_rdma_node;
	struct device_node *sc_rdma_node, *sc_wmixer_node, *sc_scaler_node, *sc_wdma_node;
	int ret = -ENODEV;

	//Main :: m2m_deIntl_info
	main_deintl_rdma_node = of_parse_phandle(main_deIntl,"telechips,main_rdma,m2m", 0);
	of_property_read_u32_index(main_deIntl,"telechips,main_rdma,m2m", 1, &index);
	if (!main_deintl_rdma_node) {
		pr_err( "could not find viqe_video main_rdma_m2m node\n");
	}
	else {
		main_m2m_info->pRDMABase_m2m = of_iomap(main_deintl_rdma_node, index);
		//pr_info("main_video rdma_m2m %d %x  \n", index, main_m2m_info->pRDMABase_m2m);
	}

	of_property_read_u32(main_deIntl, "main_rdma_num_m2m", &main_m2m_info->gVIQE_RDMA_num_m2m);

	//Sub :: m2m_deIntl_info
#ifdef USE_VIQE_FOR_SUB_M2M
	// VIQE1
	sub_deintl_rdma_node = of_parse_phandle(sub_deIntl,"telechips,sub_rdma,m2m", 0);
	of_property_read_u32_index(sub_deIntl,"telechips,sub_rdma,m2m", 1, &index);
	if (!sub_deintl_rdma_node) {
		pr_err( "could not find viqe_video sub_rdma_m2m node\n");
	}
	else {
		sub_m2m_info->pRDMABase_m2m = of_iomap(sub_deintl_rdma_node, index);
		//pr_info("sub_video rdma_m2m %d %x  \n", index, main_m2m_info->pRDMABase_m2m);
	}

	of_property_read_u32(sub_deIntl, "sub_rdma_num_m2m", &sub_m2m_info->gVIQE_RDMA_num_m2m);
#else
	// Simple DeInterlacer
	sub_deintl_rdma_node = of_parse_phandle(sub_deIntl,"telechips,sub_rdma,m2m", 0);
	of_property_read_u32_index(sub_deIntl,"telechips,sub_rdma,m2m", 1, &index);
	if (!sub_deintl_rdma_node) {
		pr_err( "could not find deintls_video sub_rdma_m2m node\n");
	}
	else {
		sub_m2m_info->pRDMABase_m2m= of_iomap(sub_deintl_rdma_node, index);
		//pr_info("sub_video rdma_m2m %d %x  \n", index, sub_m2m_info->pRDMABase_m2m);
	}

	of_property_read_u32(sub_deIntl, "sub_rdma_num_m2m", &sub_m2m_info->gDEINTLS_RDMA_num_m2m);
	//pr_info("sub_video rdma_num_m2m %d  \n", index, sub_m2m_info->gDEINTLS_RDMA_num_m2m);
#endif

	//Main :: m2m_deIntl_scaler
	scaler = kzalloc(sizeof(struct tcc_viqe_m2m_scaler_type_t), GFP_KERNEL);
	if (scaler == NULL) {
		pr_err("%s: could not allocate scaler\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_scaler;
	}

	scaler->info = kzalloc(sizeof(struct tcc_lcdc_image_update), GFP_KERNEL);
	if (scaler->info == NULL) {
		pr_err("%s: could not allocate scaler->info\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_info;
	}

	scaler->data = kzalloc(sizeof(struct tcc_viqe_m2m_scaler_data), GFP_KERNEL);
	if (scaler->data == NULL) {
		pr_err("%s: could not allocate scaler->data\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	scaler->vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (scaler->vioc_intr == NULL) {
		pr_err("%s: could not allocate scaler->vioc_intr\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_vioc_intr;
	}

	scaler->rdma = kzalloc(sizeof(struct vioc_rdma_device), GFP_KERNEL);
	if (scaler->rdma == NULL) {
		pr_err("%s: could not allocate scaler->rdma\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_rdma;
	}

	scaler->wmix = kzalloc(sizeof(struct vioc_wmix_device), GFP_KERNEL);
	if (scaler->wmix == NULL) {
		pr_err("%s: could not allocate scaler->wmix\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_wmix;
	}

	scaler->sc = kzalloc(sizeof(struct vioc_sc_device), GFP_KERNEL);
	if (scaler->sc == NULL) {
		pr_err("%s: could not allocate scaler->sc\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sc;
	}

	scaler->wdma = kzalloc(sizeof(struct vioc_wdma_device), GFP_KERNEL);
	if (scaler->wdma == NULL) {
		pr_err("%s: could not allocate scaler->wdma\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_wdma;
	}

	 sc_rdma_node = of_parse_phandle(sc_main,"m2m_sc_rdma", 0);
	 of_property_read_u32_index(sc_main,"m2m_sc_rdma", 1, &index);
	if (!sc_rdma_node) {
		pr_err( "could not find viqe_video rdma node of scaler\n");
	}
	else {
		scaler->rdma->reg = of_iomap(sc_rdma_node, index);
		scaler->rdma->id = index;
		//pr_info("main_video scaler->rdma[%d] %x  \n", scaler->rdma->id, scaler->rdma->reg);
	}

	 sc_wmixer_node = of_parse_phandle(sc_main,"m2m_sc_wmix", 0);
	 of_property_read_u32_index(sc_main,"m2m_sc_wmix", 1, &index);
	if (!sc_wmixer_node) {
		pr_err( "could not find viqe_video wmixer node of scaler\n");
	}
	else {
		scaler->wmix->reg = of_iomap(sc_wmixer_node, index);
		scaler->wmix->id = index;
		of_property_read_u32(sc_main, "m2m_sc_wmix_path", &scaler->wmix->path);
		//pr_info("main_video scaler->wmix[%d] %x  \n", scaler->wmix->id, scaler->wmix->reg);
	}

	 sc_scaler_node = of_parse_phandle(sc_main,"m2m_sc_scaler", 0);
	 of_property_read_u32_index(sc_main,"m2m_sc_scaler", 1, &index);
	if (!sc_scaler_node) {
		pr_err( "could not find viqe_video scaler node of scaler\n");
	}
	else {
		scaler->sc->reg = of_iomap(sc_scaler_node, index);
		scaler->sc->id = index;
		of_property_read_u32(sc_main, "m2m_sc_scaler_path", &scaler->sc->path);
		//pr_info("main_video scaler->scalers[%d] %x  \n", scaler->sc->id, scaler->sc->reg);
	}

	 sc_wdma_node = of_parse_phandle(sc_main,"m2m_sc_wdma", 0);
	 of_property_read_u32_index(sc_main,"m2m_sc_wdma", 1, &index);
	if (!sc_wdma_node) {
		pr_err( "could not find viqe_videowdma node of scaler\n");
	}
	else {
		scaler->wdma->reg  = of_iomap(sc_wdma_node, index);
		scaler->wdma->id = index;

		scaler->irq = irq_of_parse_and_map(sc_wdma_node, index);
              scaler->vioc_intr->id   = VIOC_INTR_WD0 + scaler->wdma->id;
              scaler->vioc_intr->bits = VIOC_WDMA_IREQ_EOFR_MASK;
		//pr_info("main_video scaler->wdmas[%d] %x  \n", scaler->wdma->id, scaler->wdma->reg);
	}

	of_property_read_u32(sc_main, "m2m_sc_settop_support", &scaler->settop_support);

	//Sub :: m2m_deIntl_scaler
	scaler_sub = kzalloc(sizeof(struct tcc_viqe_m2m_scaler_type_t), GFP_KERNEL);
	if (scaler_sub == NULL) {
		pr_err("%s: could not allocate scaler_sub\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_scaler;
	}

	scaler_sub->info = kzalloc(sizeof(struct tcc_lcdc_image_update), GFP_KERNEL);
	if (scaler_sub->info == NULL) {
		pr_err("%s: could not allocate scaler_sub->info\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_info;
	}

	scaler_sub->data = kzalloc(sizeof(struct tcc_viqe_m2m_scaler_data), GFP_KERNEL);
	if (scaler_sub->data == NULL) {
		pr_err("%s: could not allocate scaler_sub->data\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_data;
	}

	scaler_sub->vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (scaler_sub->vioc_intr == NULL) {
		pr_err("%s: could not allocate scaler_sub->vioc_intr\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_vioc_intr;
	}

	scaler_sub->rdma = kzalloc(sizeof(struct vioc_rdma_device), GFP_KERNEL);
	if (scaler_sub->rdma == NULL) {
		pr_err("%s: could not allocate scaler_sub->rdma\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_rdma;
	}

	scaler_sub->wmix = kzalloc(sizeof(struct vioc_wmix_device), GFP_KERNEL);
	if (scaler_sub->wmix == NULL) {
		pr_err("%s: could not allocate scaler_sub->wmix\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_wmix;
	}

	scaler_sub->sc = kzalloc(sizeof(struct vioc_sc_device), GFP_KERNEL);
	if (scaler_sub->sc == NULL) {
		pr_err("%s: could not allocate scaler_sub->sc\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_sc;
	}

	scaler_sub->wdma = kzalloc(sizeof(struct vioc_wdma_device), GFP_KERNEL);
	if (scaler_sub->wdma == NULL) {
		pr_err("%s: could not allocate scaler_sub->wdma\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sub_wdma;
	}

	sc_rdma_node = of_parse_phandle(sc_sub,"m2m_sc_rdma", 0);
	of_property_read_u32_index(sc_sub,"m2m_sc_rdma", 1, &index);
	if (!sc_rdma_node) {
		pr_err( "could not find viqe_video rdma node of scaler_sub\n");
	}
	else {
		scaler_sub->rdma->reg = of_iomap(sc_rdma_node, index);
		scaler_sub->rdma->id = index;
		//pr_info("sub_video scaler_sub->rdma[%d] %x  \n", scaler_sub->rdma->id, scaler_sub->rdma->reg);
	}

	sc_wmixer_node = of_parse_phandle(sc_sub,"m2m_sc_wmix", 0);
	of_property_read_u32_index(sc_sub,"m2m_sc_wmix", 1, &index);
	if (!sc_wmixer_node) {
		pr_err( "could not find viqe_video wmixer node of scaler_sub\n");
	}
	else {
		scaler_sub->wmix->reg = of_iomap(sc_wmixer_node, index);
		scaler_sub->wmix->id = index;
		of_property_read_u32(sc_sub, "m2m_sc_wmix_path", &scaler_sub->wmix->path);
		//pr_info("sub_video scaler_sub->wmix[%d] %x  \n", scaler_sub->wmix->id, scaler_sub->wmix->reg);
	}

	sc_scaler_node = of_parse_phandle(sc_sub,"m2m_sc_scaler", 0);
	of_property_read_u32_index(sc_sub,"m2m_sc_scaler", 1, &index);
	if (!sc_scaler_node) {
		pr_err( "could not find viqe_video scaler_sub node of scaler_sub\n");
	}
	else {
		scaler_sub->sc->reg = of_iomap(sc_scaler_node, index);
		scaler_sub->sc->id = index;
		of_property_read_u32(sc_sub, "m2m_sc_scaler_path", &scaler_sub->sc->path);
		//pr_info("sub_video scaler_sub->scalers[%d] %x  \n", scaler_sub->sc->id, scaler_sub->sc->reg);
	}

	sc_wdma_node = of_parse_phandle(sc_sub,"m2m_sc_wdma", 0);
	of_property_read_u32_index(sc_sub,"m2m_sc_wdma", 1, &index);
	if (!sc_wdma_node) {
		pr_err( "could not find viqe_videowdma node of scaler_sub\n");
	}
	else {
		scaler_sub->wdma->reg  = of_iomap(sc_wdma_node, index);
		scaler_sub->wdma->id = index;

		scaler_sub->irq = irq_of_parse_and_map(sc_wdma_node, index);
		scaler_sub->vioc_intr->id   = VIOC_INTR_WD0 + scaler_sub->wdma->id;
		scaler_sub->vioc_intr->bits = VIOC_WDMA_IREQ_EOFR_MASK;
		//pr_info("sub_video scaler_sub->wdmas[%d] %x  \n", scaler_sub->wdma->id, scaler_sub->wdma->reg);
	}

	of_property_read_u32(sc_sub, "m2m_sc_settop_support", &scaler_sub->settop_support);

	iprintk("Main/Sub M2M Information. \n");
	iprintk("RDMA  : (%d)0x%8x / (%d)0x%8x \n", scaler->rdma->id, scaler->rdma->reg, scaler_sub->rdma->id, scaler_sub->rdma->reg);
	iprintk("Scaler: (%d)0x%8x / (%d)0x%8x \n", scaler->sc->id, scaler->sc->reg, scaler_sub->sc->id, scaler_sub->sc->reg);
	iprintk("WMIX  : (%d)0x%8x / (%d)0x%8x \n", scaler->wmix->id, scaler->wmix->reg, scaler_sub->wmix->id, scaler_sub->wmix->reg);
	iprintk("WDMA  : (%d)0x%8x / (%d)0x%8x \n", scaler->wdma->id, scaler->wdma->reg, scaler_sub->wdma->id, scaler_sub->wdma->reg);

	iprintk("VIQE  : (%d)0x%8x \n", main_m2m_info->gVIQE_RDMA_num_m2m, main_m2m_info->pRDMABase_m2m);
#ifdef USE_VIQE_FOR_SUB_M2M
	iprintk("VIQE1 : (%d)0x%8x \n", sub_m2m_info->gVIQE_RDMA_num_m2m, sub_m2m_info->pRDMABase_m2m);
#else
	iprintk("sDeIntl: (%d)0x%8x \n", sub_m2m_info->gDEINTLS_RDMA_num_m2m, sub_m2m_info->pRDMABase_m2m);
#endif

	return 0;

//err:
//	kfree(scaler_sub->wdma);
err_alloc_sub_wdma:
	kfree(scaler_sub->sc);
err_alloc_sub_sc:
	kfree(scaler_sub->wmix);
err_alloc_sub_wmix:
	kfree(scaler_sub->rdma);
err_alloc_sub_rdma:
	kfree(scaler_sub->vioc_intr);
err_alloc_sub_vioc_intr:
	kfree(scaler_sub->data);
err_alloc_sub_data:
	kfree(scaler_sub->info);
err_alloc_sub_info:
	kfree(scaler_sub);

err_alloc_sub_scaler:
	kfree(scaler->wdma);
err_alloc_wdma:
	kfree(scaler->sc);
err_alloc_sc:
	kfree(scaler->wmix);
err_alloc_wmix:
	kfree(scaler->rdma);
err_alloc_rdma:
	kfree(scaler->vioc_intr);
err_alloc_vioc_intr:
	kfree(scaler->data);
err_alloc_data:
	kfree(scaler->info);
err_alloc_info:
	kfree(scaler);

err_alloc_scaler:
	return ret;
}

/*****************************************************************************
 * TCC_VIQE Probe
 ******************************************************************************/
static int tcc_viqe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *viqe_common, *lcd_60hz_node, *external_60Hz_node;
	struct device_node *m2m_deintl_main_node, *m2m_deintl_sub_node, *m2m_scaler_main_node, *m2m_scaler_sub_node;

#ifdef CONFIG_OF
	//viqe_common
	viqe_common = of_find_node_by_name(dev->of_node, "tcc_viqe_viqe_common");
	if(!viqe_common) {
		pr_err( "could not find viqe_common node\n");
		return -ENODEV;
	}
	tcc_viqe_commom_dt_parse(viqe_common, &viqe_common_info);

	//viqe_60hz_info
	lcd_60hz_node = of_find_node_by_name(dev->of_node, "tcc_video_viqe_lcd");
	external_60Hz_node = of_find_node_by_name(dev->of_node, "tcc_video_viqe_external");

	if(viqe_common_info.gBoard_num == 0) {
		if (!lcd_60hz_node) {
			pr_err( "could not find viqe_lcd node\n");
			return -ENODEV;
		}
	} else {
		if (!external_60Hz_node) {
			pr_err( "could not find viqe_external node\n");
			return -ENODEV;
		}
	}

	tcc_viqe_60hz_dt_parse(lcd_60hz_node, &viqe_60hz_lcd_info);
	tcc_viqe_60hz_dt_parse(external_60Hz_node, &viqe_60hz_external_info);

	//m2m_viqe
	m2m_deintl_main_node = of_find_node_by_name(dev->of_node, "tcc_video_main_m2m");
	m2m_deintl_sub_node = of_find_node_by_name(dev->of_node, "tcc_video_sub_m2m");

	m2m_scaler_main_node = of_find_node_by_name(dev->of_node, "tcc_video_scaler_main_m2m");
	m2m_scaler_sub_node = of_find_node_by_name(dev->of_node, "tcc_video_scaler_sub_m2m");

	if(!m2m_deintl_main_node || !m2m_scaler_main_node || !m2m_deintl_sub_node || !m2m_scaler_sub_node) {
		pr_err( "could not find m2m_viqe node\n");
		return -ENODEV;
	}

	tcc_viqe_m2m_dt_parse(m2m_deintl_main_node, m2m_deintl_sub_node, m2m_scaler_main_node, m2m_scaler_sub_node, &main_m2m_info, &sub_m2m_info);
#endif

	TCC_VIQE_Scaler_Init_Buffer_M2M();

	return 0;
}

/*****************************************************************************
 * TCC_VIQE Module Init/Exit
 ******************************************************************************/
#ifdef CONFIG_OF
static const struct of_device_id tcc_viqe_of_match[] = {
	{.compatible = "telechips,tcc_viqe", },
	{ },
};
MODULE_DEVICE_TABLE(of, tcc_viqe_of_match);
#endif

static struct platform_driver tcc_viqe = {
	.probe	= tcc_viqe_probe,
	.driver	= {
	.name	= "tcc_viqe",
	.owner	= THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = tcc_viqe_of_match,
#endif
	},
};

static int __init tcc_viqe_init(void)
{
	return platform_driver_register(&tcc_viqe);
}

static void __exit tcc_viqe_exit(void)
{
	platform_driver_unregister(&tcc_viqe);
}

module_init(tcc_viqe_init);
module_exit(tcc_viqe_exit);

MODULE_AUTHOR("Telechips Inc.");
MODULE_DESCRIPTION("TCC viqe driver");
MODULE_LICENSE("GPL");

