/****************************************************************************
 * linux/drivers/video/tca_map_converter.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 18, 2012
 * Description: TCC lcd Driver
 *
 * Copyright (C) 20010-2011 Telechips 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/tcc_ccfb_ioctl.h>
#include <mach/tcc_fb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_global.h>
#include <mach/vioc_mc.h>
#else
#include <video/tcc/tcc_types.h>
#include <video/tcc/tcc_ccfb_ioctl.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_mc.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#endif



void tca_map_convter_onoff(unsigned int mc_num, unsigned int onoff)
{
	VIOC_MC *HwVIOC_MC;
	struct device_node *ViocMapCV_np;
	
	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	HwVIOC_MC= (VIOC_MC *)of_iomap(ViocMapCV_np, mc_num);

	VIOC_MC_Start_OnOff((VIOC_MC *)HwVIOC_MC, onoff);
}

void tca_map_convter_wait_done(unsigned int mc_num)
{
	#define MAX_WAIT_TIEM 		0x10000000

	volatile unsigned int loop = 0;

	VIOC_MC *HwVIOC_MC;
	struct device_node *ViocMapCV_np;
	
	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	HwVIOC_MC= (VIOC_MC *)of_iomap(ViocMapCV_np, mc_num);

	for(loop = 0; loop < MAX_WAIT_TIEM; loop++)
	{
		if(!ISSET(HwVIOC_MC->uSTAT.nReg, 1<<4))
			break;
	}

}

void tca_map_convter_swreset(unsigned int mc_num)
{
	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();

	if(mc_num > 2)
		return;

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(26+mc_num)), (0x1<<(26+mc_num))); // MC reset
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(26+mc_num)), (0x0<<(26+mc_num))); // MC reset
}

//#define WDMA_TEST

void tca_map_convter_set(unsigned int mc_num, struct tcc_lcdc_image_update *ImageInfo)
{
	uint	width;
	uint	height;

	uint	frm_stride_y;
	uint	frm_stride_c;
	uint	pic_height;

	uint	bit_depth_y;
	uint	bit_depth_c;

	uint	mult_y, mult_c;
	
	VIOC_MC *HwVIOC_MC;
	struct device_node *ViocMapCV_np;

//	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)of_iomap(ViocConfig_np, 0);
		
	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(ViocMapCV_np == NULL)
		return;

	HwVIOC_MC= (VIOC_MC *)of_iomap(ViocMapCV_np, mc_num);

#if defined(WDMA_TEST)
	ImageInfo->Frame_width = 704;
	ImageInfo->Frame_height = 480;
#endif//

#if 0 //debug log
	printk(">> map converter info  width : %d height : %d \n", ImageInfo->Frame_width, ImageInfo->Frame_height);
	printk("Comp Y 0x%8x  Comp Y 0x%8x   Comp Cb 0x%8x  Comp Cb 0x%8x \n",
		ImageInfo->private_data.mapConv_info.m_CompressedY[0], ImageInfo->private_data.mapConv_info.m_CompressedY[1], 
		ImageInfo->private_data.mapConv_info.m_CompressedCb[0], ImageInfo->private_data.mapConv_info.m_CompressedCb[1]);

	printk("Offset Y 0x%8x  Offset Y 0x%8x   Offset Cb 0x%8x  Offset Cb 0x%8x \n",
		ImageInfo->private_data.mapConv_info.m_FbcYOffsetAddr[0], ImageInfo->private_data.mapConv_info.m_FbcYOffsetAddr[1], 
		ImageInfo->private_data.mapConv_info.m_FbcCOffsetAddr[0], ImageInfo->private_data.mapConv_info.m_FbcCOffsetAddr[1]);

	printk("Stride  (Luma : %d Chroma %d)  ~ bitDepth  (Luma:%d Croma:%d) Endian :%d \n", 	
		ImageInfo->private_data.mapConv_info.m_uiLumaStride, 
		ImageInfo->private_data.mapConv_info.m_uiChromaStride,
		ImageInfo->private_data.mapConv_info.m_uiLumaBitDepth, 
		ImageInfo->private_data.mapConv_info.m_uiChromaBitDepth,
		ImageInfo->private_data.mapConv_info.m_uiFrameEndian);
	
	printk("* map converter info end  *  \n");
#endif//
	
		if(ImageInfo->private_data.mapConv_info.m_uiLumaBitDepth == 8)
			bit_depth_y = 0;
		else if(ImageInfo->private_data.mapConv_info.m_uiLumaBitDepth == 16)
			bit_depth_y = 1;
		else
			bit_depth_y = 2;

		if(ImageInfo->private_data.mapConv_info.m_uiChromaBitDepth == 8)
			bit_depth_c = 0;
		else if(ImageInfo->private_data.mapConv_info.m_uiChromaBitDepth == 16)
			bit_depth_c = 1;
		else
			bit_depth_c = 2;

		width	= ImageInfo->Frame_width;
		height	=  ImageInfo->Frame_height;
		pic_height	=	(((height+15)>>4)<<4);

	    mult_y          =   (bit_depth_y    == 0) ? 4 : 5;
	    //frm_stride_y  =   ((((width+15)>>4)<<4)<<2);
	    frm_stride_y    =   ((((width+15)>>4)<<4) * mult_y);
	    frm_stride_y    =   (((frm_stride_y+31)>>5)<<5);

	    mult_c          =   (bit_depth_c    == 0) ? 4 : 5;
	    frm_stride_c    =   (((((width>>1)+15)>>4)<<4) * mult_c);
	    frm_stride_c    =   (((frm_stride_c+31)>>5)<<5);
		

	

#if defined(WDMA_TEST)
{
	volatile PVIOC_IREQ_CONFIG pIREQConfig;
	struct device_node *ViocConfig_np;
	volatile PVIOC_WMIX 			pWIXBase;
	volatile PVIOC_WDMA 			pWDMABase;
	char	*pDstBase;
	pDstBase  = (char *)0x35000000;

	pWIXBase = (volatile PVIOC_WMIX)tcc_p2v((unsigned int)HwVIOC_WMIX2);
	pWDMABase = (volatile PVIOC_WDMA)tcc_p2v((unsigned int)HwVIOC_WDMA02);
	
	ViocConfig_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_config");
	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)of_iomap(ViocConfig_np, 0);
					
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<2), 0x1<<2); // wdma 2 reset
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<2),0x0<<2); //  wdma 2 reset
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<11), 0x1<<11); //  wmixer 2 reset
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<11),0x0<<11); //  wmixer 2 reset


	width	= ImageInfo->Frame_width;
	height	=  ImageInfo->Frame_height;

	pic_height	=	(((height+15)>>4)<<4);

	mult_y          =   (bit_depth_y    == 0) ? 4 : 5;
	//frm_stride_y  =   ((((width+15)>>4)<<4)<<2);
	frm_stride_y    =   ((((width+15)>>4)<<4) * mult_y);
	frm_stride_y    =   (((frm_stride_y+31)>>5)<<5);

	mult_c          =   (bit_depth_c    == 0) ? 4 : 5;
	frm_stride_c    =   (((((width>>1)+15)>>4)<<4) * mult_c);
	frm_stride_c    =   (((frm_stride_c+31)>>5)<<5);

	printk("%4d %4d %4d %4d \n", pic_height, bit_depth_c,  frm_stride_y, frm_stride_c);
	printk("%4d %4d %4d %4d \n", pic_height, bit_depth_c, 
		ImageInfo->private_data.mapConv_info.m_uiLumaStride, 
		ImageInfo->private_data.mapConv_info.m_uiChromaStride);

	BITCSET(pIREQConfig->uMC.nREG, (0x7<<16) | (0x3E), (1<<16) | (1<<2)); // Map_Conv connects the RDMA11 path.

	VIOC_MC_ENDIAN((VIOC_MC *)HwVIOC_MC,
	ImageInfo->private_data.mapConv_info.m_uiFrameEndian, 
	ImageInfo->private_data.mapConv_info.m_uiFrameEndian);

	VIOC_MC_OFFSET_BASE((VIOC_MC *)HwVIOC_MC,
			ImageInfo->private_data.mapConv_info.m_FbcYOffsetAddr[0],
			ImageInfo->private_data.mapConv_info.m_FbcCOffsetAddr[0]);
	
	VIOC_MC_FRM_BASE((VIOC_MC *)HwVIOC_MC, 
			ImageInfo->private_data.mapConv_info.m_CompressedY[0],
			ImageInfo->private_data.mapConv_info.m_CompressedCb[0]);

	VIOC_MC_Start_BitDepth((VIOC_MC *)HwVIOC_MC, bit_depth_c, bit_depth_y);

	
	VIOC_MC_FRM_POS (HwVIOC_MC,  0, 0);

	//pjj test
	VIOC_MC_FRM_SIZE((VIOC_MC *)HwVIOC_MC, width,height);

	VIOC_MC_FRM_SIZE_MISC((VIOC_MC *)HwVIOC_MC, pic_height, 
							frm_stride_y, frm_stride_c);

	VIOC_MC_DITH_CONT((VIOC_MC *)HwVIOC_MC, 0, 0);
	VIOC_MC_Start_OnOff((VIOC_MC *)HwVIOC_MC, 1);

	VIOC_WMIX_SetSize(pWIXBase, width, height);

	VIOC_WMIX_SetUpdate(pWIXBase);

	VIOC_WDMA_SetImageFormat(pWDMABase , VIOC_IMG_FMT_ARGB8888);
	VIOC_WDMA_SetImageSize(pWDMABase, width, height);
	VIOC_WDMA_SetImageBase(pWDMABase, pDstBase, pDstBase, pDstBase);
	VIOC_WDMA_SetImageOffset(pWDMABase, width*4, width*4);
	VIOC_WDMA_SetImageY2REnable(pWDMABase, 1);
	VIOC_WDMA_SetImageEnable(pWDMABase, 1);
	
	while(1);
}
#else
	VIOC_MC_ENDIAN((VIOC_MC *)HwVIOC_MC,
	ImageInfo->private_data.mapConv_info.m_uiFrameEndian, ImageInfo->private_data.mapConv_info.m_uiFrameEndian);

	VIOC_MC_OFFSET_BASE((VIOC_MC *)HwVIOC_MC,
			ImageInfo->private_data.mapConv_info.m_FbcYOffsetAddr[0], ImageInfo->private_data.mapConv_info.m_FbcCOffsetAddr[0]);
	
	VIOC_MC_FRM_BASE((VIOC_MC *)HwVIOC_MC, 
			ImageInfo->private_data.mapConv_info.m_CompressedY[0], ImageInfo->private_data.mapConv_info.m_CompressedCb[0]);

	VIOC_MC_Start_BitDepth((VIOC_MC *)HwVIOC_MC, bit_depth_c, bit_depth_y);

	
	VIOC_MC_FRM_POS (HwVIOC_MC,  0, 0);

	//pjj test
	VIOC_MC_FRM_SIZE((VIOC_MC *)HwVIOC_MC, (ImageInfo->Frame_width),(ImageInfo->Frame_height));

	VIOC_MC_FRM_SIZE_MISC((VIOC_MC *)HwVIOC_MC, pic_height, 
							ImageInfo->private_data.mapConv_info.m_uiLumaStride, 
							ImageInfo->private_data.mapConv_info.m_uiChromaStride);

	VIOC_MC_DITH_CONT((VIOC_MC *)HwVIOC_MC, 0, 0);
	VIOC_MC_Y2R_OnOff((VIOC_MC *)HwVIOC_MC, 1);

	VIOC_MC_Start_OnOff((VIOC_MC *)HwVIOC_MC, 1);
	
#endif//
	
}
EXPORT_SYMBOL(tca_map_convter_set);

void tca_map_convter_driver_set(unsigned int mc_num, unsigned int Fwidth, unsigned int Fheight, unsigned int pos_x, unsigned int pos_y, unsigned int y2r, hevc_dec_MapConv_info_t *mapConv_info)
{
	uint	bit_depth_y, bit_depth_c;
	uint	pic_height;
	
	VIOC_MC *HwVIOC_MC;
	struct device_node *ViocMapCV_np;

	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(ViocMapCV_np == NULL)
		return;

	HwVIOC_MC= (VIOC_MC *)of_iomap(ViocMapCV_np, mc_num);

	pic_height	=	(((Fheight+15)>>4)<<4);


	if(mapConv_info->m_uiLumaBitDepth == 8)
		bit_depth_y = 0;
	else if(mapConv_info->m_uiLumaBitDepth == 16)
		bit_depth_y = 1;
	else
		bit_depth_y = 2;

	if(mapConv_info->m_uiChromaBitDepth == 8)
		bit_depth_c = 0;
	else if(mapConv_info->m_uiChromaBitDepth == 16)
		bit_depth_c = 1;
	else
		bit_depth_c = 2;

	VIOC_MC_ENDIAN((VIOC_MC *)HwVIOC_MC,
			mapConv_info->m_uiFrameEndian, mapConv_info->m_uiFrameEndian);

	VIOC_MC_OFFSET_BASE((VIOC_MC *)HwVIOC_MC,
			mapConv_info->m_FbcYOffsetAddr[0], mapConv_info->m_FbcCOffsetAddr[0]);
	
	VIOC_MC_FRM_BASE((VIOC_MC *)HwVIOC_MC, 
			mapConv_info->m_CompressedY[0], mapConv_info->m_CompressedCb[0]);

	VIOC_MC_Start_BitDepth((VIOC_MC *)HwVIOC_MC, bit_depth_c, bit_depth_y);
	
	VIOC_MC_FRM_POS (HwVIOC_MC,  pos_x, pos_y);

	VIOC_MC_FRM_SIZE((VIOC_MC *)HwVIOC_MC, Fwidth, Fheight);

	VIOC_MC_FRM_SIZE_MISC((VIOC_MC *)HwVIOC_MC, pic_height, 
							mapConv_info->m_uiLumaStride, 
							mapConv_info->m_uiChromaStride);

	VIOC_MC_Y2R_OnOff((VIOC_MC *)HwVIOC_MC, y2r);
	
	VIOC_MC_DITH_CONT((VIOC_MC *)HwVIOC_MC, 0, 0);

}
EXPORT_SYMBOL(tca_map_convter_driver_set);
