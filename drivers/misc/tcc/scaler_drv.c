/****************************************************************************
 * Copyright (C) 2015 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 ****************************************************************************/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/errno.h>

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/vioc_intr.h>
#include <mach/of_vioc_rdma.h>
#include <mach/of_vioc_wmix.h>
#include <mach/of_vioc_sc.h>
#include <mach/of_vioc_wdma.h>

#include <mach/tcc_scaler_ioctrl.h>
#include <mach/vioc_api.h>
#include <mach/vioc_config.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_global.h>
#include <mach/iomap.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_blk.h>
//#include <mach/bsp.h>
//#include <mach/vioc_config.h>
//#include <mach/vioc_scaler.h>
//#include <mach/vioc_api.h>
//#include <mach/vioc_global.h>
//#include <mach/tcc_wmixer_ioctrl.h>
#else
#include <video/tcc/tcc_types.h>

#include <video/tcc/vioc_intr.h>
#include <video/tcc/of_vioc_rdma.h>
#include <video/tcc/of_vioc_wmix.h>
#include <video/tcc/of_vioc_sc.h>
#include <video/tcc/of_vioc_wdma.h>

#include <video/tcc/tcc_scaler_ioctrl.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_lut.h>
#include <video/tcc/vioc_blk.h>
#include <video/tcc/vioc_mc.h>
#endif

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
#ifdef CONFIG_ARCH_TCC897X
#include <mach/TCC_HEVCDEC.h>
#else
#include <video/tcc/TCC_HEVCDEC.h>
#endif
extern void tca_map_convter_onoff(unsigned int mc_num, unsigned int onoff);
extern void tca_map_convter_driver_set(unsigned int mc_num, unsigned int Fwidth, unsigned int Fheight, unsigned int pos_x, unsigned int pos_y, unsigned int y2r, hevc_dec_MapConv_info_t *mapConv_info);
extern void tca_map_convter_swreset(unsigned int mc_num);
#endif

static int debug	   		= 0;
#define dprintk(msg...)	if(debug) { 	printk( "TCC_SCALER1:  " msg); 	}

#ifndef TCC_PA_VIOC_CFGINT
#define TCC_PA_VIOC_CFGINT	(HwVIOC_BASE + 0xA000)
#endif

struct scaler_data {
	// wait for poll
	wait_queue_head_t	poll_wq;
	spinlock_t		poll_lock;
	unsigned int		poll_count;

	// wait for ioctl command
	wait_queue_head_t	cmd_wq;
	spinlock_t		cmd_lock;
	unsigned int		cmd_count;

	struct mutex		io_mutex;
	unsigned char		block_operating;
	unsigned char		block_waiting;
	unsigned char		irq_reged;
	unsigned int		dev_opened;
};

struct scaler_drv_type {
	struct vioc_intr_type	*vioc_intr;

	unsigned int		id;
	unsigned int		irq;

	struct miscdevice	*misc;

	struct vioc_rdma_device	*rdma;
 	struct vioc_wmix_device	*wmix;
	struct vioc_sc_device	*sc;
	struct vioc_wdma_device	*wdma;

	struct clk		*clk;
	struct scaler_data	*data;
	SCALER_TYPE		*info;

	unsigned int		settop_support;
};

extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
								unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);
extern int range_is_allowed(unsigned long pfn, unsigned long size);

static int scaler_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	if (range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
		printk(KERN_ERR	 "%s():  This address is not allowed. \n", __func__);
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk(KERN_ERR	 "%s():  Virtual address page port error. \n", __func__);
		return -EAGAIN;
	}

	vma->vm_ops	= NULL;
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static char tcc_scaler_run(struct scaler_drv_type *scaler)
{
	int ret = 0;
	unsigned int pSrcBase0 = 0, pSrcBase1 = 0, pSrcBase2 = 0;
	unsigned int crop_width;

#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA pSC_RDMABase = (PVIOC_RDMA)scaler->rdma->reg;
	volatile PVIOC_WMIX pSC_WMIXBase = (PVIOC_WMIX)scaler->wmix->reg;
	volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler->wdma->reg;
	VIOC_SCALER_INFO_Type pScalerInfo;
	#if defined(CONFIG_ARCH_TCC896X)
	volatile PVIOC_IREQ_CONFIG pSC_IREQConfig = (PVIOC_IREQ_CONFIG)io_p2v(TCC_PA_VIOC_CFGINT);
	#endif//
#endif
	dprintk("%s():  IN. \n", __func__);

	spin_lock_irq(&(scaler->data->cmd_lock));

	dprintk("scaler 0 src   add : 0x%p 0x%p 0x%p, fmt :0x%x IMG:(%d %d )%d %d %d %d \n", 
		scaler->info->src_Yaddr, scaler->info->src_Uaddr, scaler->info->src_Vaddr, scaler->info->src_fmt, 
		scaler->info->src_ImgWidth, scaler->info->src_ImgHeight, 
		scaler->info->src_winLeft, scaler->info->src_winTop, scaler->info->src_winRight, scaler->info->src_winBottom);
	dprintk("scaler 0 dest  add : 0x%p 0x%p 0x%p, fmt :0x%x IMG:(%d %d )%d %d %d %d \n", 
		scaler->info->dest_Yaddr, scaler->info->dest_Uaddr, scaler->info->dest_Vaddr, scaler->info->dest_fmt, 
		scaler->info->dest_ImgWidth, scaler->info->dest_ImgHeight, 
		scaler->info->dest_winLeft, scaler->info->dest_winTop, scaler->info->dest_winRight, scaler->info->dest_winBottom);
	dprintk("scaler 0 interlace:%d \n",  scaler->info->interlaced);

	crop_width = scaler->info->src_winRight - scaler->info->src_winLeft;

#ifdef CONFIG_OF_VIOC
	if (scaler->sc) {
		scaler->sc->data.bypass = 0;
		scaler->sc->data.dst_width = scaler->info->dest_winRight - scaler->info->dest_winLeft;
		scaler->sc->data.dst_height = scaler->info->dest_winBottom - scaler->info->dest_winTop;
		scaler->sc->data.out_x = 0;
		scaler->sc->data.out_y = 0;
		scaler->sc->data.out_width = scaler->info->dest_winRight - scaler->info->dest_winLeft;
		scaler->sc->data.out_height = scaler->info->dest_winBottom - scaler->info->dest_winTop;
		vioc_sc_set_image(scaler->sc, 1);
	}

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	if(scaler->info->mapConv_info.m_CompressedY[0] != 0)
	{
		dprintk("scaler 0 src  map converter: size: %d %d pos:%d %d\n", scaler->info->src_winRight - scaler->info->src_winLeft, 
									scaler->info->src_winBottom - scaler->info->src_winTop, 
									scaler->info->src_winLeft, scaler->info->src_winTop);	
		tca_set_MC_Connect_to_RDMA(1, scaler->rdma->id, 1);
		// scaler limitation 
		tca_map_convter_driver_set(1, scaler->info->src_winRight - scaler->info->src_winLeft, scaler->info->src_winBottom - scaler->info->src_winTop, 
									scaler->info->src_winLeft, scaler->info->src_winTop, 0,
									&scaler->info->mapConv_info);
		tca_map_convter_onoff(1, 1);
	}
	else
	#endif//
	if (scaler->rdma) {
		if (scaler->settop_support)
			scaler->rdma->data.asel = 1;
		scaler->rdma->data.aen = 1;
		scaler->rdma->data.format = scaler->info->src_fmt;
		if (scaler->info->interlaced) {
			scaler->rdma->data.width = scaler->info->src_winRight - scaler->info->src_winLeft;
			scaler->rdma->data.height = (scaler->info->src_winBottom - scaler->info->src_winTop)/2;
			scaler->rdma->data.offset = scaler->info->src_ImgWidth*2;
		} else {
			scaler->rdma->data.width = scaler->info->src_winRight - scaler->info->src_winLeft;
			scaler->rdma->data.height = scaler->info->src_winBottom - scaler->info->src_winTop;
			scaler->rdma->data.offset = scaler->info->src_ImgWidth;
		}

		pSrcBase0 = (unsigned int)scaler->info->src_Yaddr;
		pSrcBase1 = (unsigned int)scaler->info->src_Uaddr;
		pSrcBase2 = (unsigned int)scaler->info->src_Vaddr;
		if (scaler->info->src_fmt >= SC_IMG_FMT_YCbCr444_SEP_YUV) { // address limitation!!
			dprintk("%s():  src addr is not allocate. \n", __func__);
			scaler->info->src_winLeft 	= (scaler->info->src_winLeft>>3)<<3;
			scaler->info->src_winRight = scaler->info->src_winLeft + crop_width;
			scaler->info->src_winRight = scaler->info->src_winLeft + (scaler->info->src_winRight - scaler->info->src_winLeft);
			tccxxx_GetAddress(scaler->info->src_fmt, (unsigned int)scaler->info->src_Yaddr, 		\
										scaler->info->src_ImgWidth, scaler->info->src_ImgHeight, 	\
										scaler->info->src_winLeft, scaler->info->src_winTop, 		\
										&pSrcBase0, &pSrcBase1, &pSrcBase2);
		}
		scaler->rdma->data.mode.mode = VIOC_RDMA_MODE_NONE;
		scaler->rdma->data.base0 = pSrcBase0;
		scaler->rdma->data.base1 = pSrcBase1;
		scaler->rdma->data.base2 = pSrcBase2;
		vioc_rdma_set_image(scaler->rdma, 1);
	}

	if (scaler->wmix) {
		scaler->wmix->mixmode = 1;	/* mixer op is mixing mode */
		scaler->wmix->data.width = scaler->info->dest_winRight - scaler->info->dest_winLeft;
		scaler->wmix->data.height = scaler->info->dest_winBottom - scaler->info->dest_winTop;
		vioc_wmix_set_image(scaler->wmix, 1);
	}
	
	// look up table use
	if(scaler->info->lut.use_lut)
	{
		tcc_set_lut_plugin(TVC_LUT(scaler->info->lut.use_lut_number), TVC_RDMA(scaler->rdma->id));
		tcc_set_lut_enable(TVC_LUT(scaler->info->lut.use_lut_number), true);
	}

	if (scaler->wdma) {
		scaler->wdma->data.format = scaler->info->dest_fmt;
		scaler->wdma->data.width = scaler->info->dest_winRight - scaler->info->dest_winLeft;
		scaler->wdma->data.height = scaler->info->dest_winBottom - scaler->info->dest_winTop;
		scaler->wdma->data.offset = scaler->info->dest_ImgWidth;
		scaler->wdma->data.base0 = (unsigned int)scaler->info->dest_Yaddr;
		scaler->wdma->data.base1 = (unsigned int)scaler->info->dest_Uaddr;
		scaler->wdma->data.base2 = (unsigned int)scaler->info->dest_Vaddr;

		#if defined(CONFIG_ARCH_TCC896X)
		if(scaler->info->dest_fmt == SC_IMG_FMT_FCDEC)
			vioc_wdma_set_path(scaler->wdma, VIOC_WDMA_PATH_FCENC0);
		else 
		#endif

		if ((scaler->info->src_fmt < SC_IMG_FMT_FCDEC) && (scaler->info->dest_fmt > SC_IMG_FMT_FCDEC))
			scaler->wdma->data.mode.mode = VIOC_WDMA_MODE_RGB2YUV;
		else {
			scaler->wdma->data.mode.mode = scaler->info->wdma_r2y;
			if (scaler->info->wdma_r2y)
				scaler->wdma->data.mode.convert = 0;
		}
		scaler->wdma->data.cont = 0;
		vioc_wdma_set_image(scaler->wdma, 1);
	}

#else

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	if(scaler->info->mapConv_info.m_CompressedY[0] != 0)
	{
		dprintk("scaler 0 src  map converter: size: %d %d pos:%d %d\n", scaler->info->src_winRight - scaler->info->src_winLeft, 
									scaler->info->src_winBottom - scaler->info->src_winTop, 
									scaler->info->src_winLeft, scaler->info->src_winTop);	
	
		tca_set_MC_Connect_to_RDMA(1, scaler->rdma->id, 1);
		// scaler limitation 
		tca_map_convter_driver_set(1, scaler->info->src_winRight - scaler->info->src_winLeft, scaler->info->src_winBottom - scaler->info->src_winTop, 
									scaler->info->src_winLeft, scaler->info->src_winTop, 0,
									&scaler->info->mapConv_info);
	}
	else
	#endif//		
	{
		if (scaler->settop_support) {
			VIOC_RDMA_SetImageAlphaSelect(pSC_RDMABase, 1);
			VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
		} else {
			VIOC_RDMA_SetImageAlphaEnable(pSC_RDMABase, 1);
		}
		VIOC_RDMA_SetImageFormat(pSC_RDMABase, scaler->info->src_fmt);

		//interlaced frame process ex) MPEG2
		if (scaler->info->interlaced) {
			VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler->info->src_winRight - scaler->info->src_winLeft), (scaler->info->src_winBottom - scaler->info->src_winTop)/2);
			VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler->info->src_fmt, scaler->info->src_ImgWidth*2);
		} else {
			VIOC_RDMA_SetImageSize(pSC_RDMABase, (scaler->info->src_winRight - scaler->info->src_winLeft), (scaler->info->src_winBottom - scaler->info->src_winTop));
			VIOC_RDMA_SetImageOffset(pSC_RDMABase, scaler->info->src_fmt, scaler->info->src_ImgWidth);
		}

		pSrcBase0 = (unsigned int)scaler->info->src_Yaddr;
		pSrcBase1 = (unsigned int)scaler->info->src_Uaddr;
		pSrcBase2 = (unsigned int)scaler->info->src_Vaddr;
		if (scaler->info->src_fmt >= SC_IMG_FMT_YCbCr444_SEP_YUV) { // address limitation!!
			dprintk("%s():  src addr is not allocate. \n", __func__);
			scaler->info->src_winLeft 	= (scaler->info->src_winLeft>>3)<<3;
			scaler->info->src_winRight = scaler->info->src_winLeft + crop_width;
			scaler->info->src_winRight = scaler->info->src_winLeft + (scaler->info->src_winRight - scaler->info->src_winLeft);
			tccxxx_GetAddress(scaler->info->src_fmt, (unsigned int)scaler->info->src_Yaddr, 		\
										scaler->info->src_ImgWidth, scaler->info->src_ImgHeight, 	\
										scaler->info->src_winLeft, scaler->info->src_winTop, 		\
										&pSrcBase0, &pSrcBase1, &pSrcBase2);
		}

		#if defined(CONFIG_ARCH_TCC896X)
		VIOC_RDMA_SetImageR2YEnable(pSC_RDMABase, 0);
		#endif//

		if ((scaler->info->src_fmt > SC_IMG_FMT_FCDEC) && (scaler->info->dest_fmt < SC_IMG_FMT_FCDEC)) {
			VIOC_RDMA_SetImageY2REnable(pSC_RDMABase, 1);
		}
		else{		
			VIOC_RDMA_SetImageY2REnable(pSC_RDMABase, 0);
		}
		VIOC_RDMA_SetImageBase(pSC_RDMABase, (unsigned int)pSrcBase0, (unsigned int)pSrcBase1, (unsigned int)pSrcBase2);
	}

	// look up table use
	if(scaler->info->lut.use_lut)		{
		tcc_set_lut_plugin(TVC_LUT(scaler->info->lut.use_lut_number), TVC_RDMA(scaler->rdma->id));
		tcc_set_lut_enable(TVC_LUT(scaler->info->lut.use_lut_number), true);
	}
	
	pScalerInfo.BYPASS 			= 0;
	pScalerInfo.DST_WIDTH 		= (scaler->info->dest_winRight - scaler->info->dest_winLeft);
	pScalerInfo.DST_HEIGHT 		= (scaler->info->dest_winBottom - scaler->info->dest_winTop);
	pScalerInfo.OUTPUT_POS_X 		= 0;
	pScalerInfo.OUTPUT_POS_Y 		= 0;
	pScalerInfo.OUTPUT_WIDTH 		= pScalerInfo.DST_WIDTH;
	pScalerInfo.OUTPUT_HEIGHT 	= pScalerInfo.DST_HEIGHT;
	VIOC_API_SCALER_SetConfig(scaler->sc->id, &pScalerInfo);
	VIOC_API_SCALER_SetPlugIn(scaler->sc->type, scaler->sc->path);
	VIOC_API_SCALER_SetUpdate(scaler->sc->id);

	
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER	
	if(scaler->info->mapConv_info.m_CompressedY[0] != 0)
		tca_map_convter_onoff(1, 1);
	else
#endif//		
	VIOC_RDMA_SetImageEnable(pSC_RDMABase); // SoC guide info.

	if(scaler->wmix->path == WMIX13)
		VIOC_CONFIG_WMIXPath(scaler->wmix->path, 0); // wmixer op is bypass mode.
	else
		VIOC_CONFIG_WMIXPath(scaler->wmix->path, 1); // wmixer op is mixing mode.	
		
	VIOC_WMIX_SetSize(pSC_WMIXBase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);	
	VIOC_WMIX_SetUpdate(pSC_WMIXBase);


	
	VIOC_WDMA_SetImageFormat(pSC_WDMABase, scaler->info->dest_fmt);
	VIOC_WDMA_SetImageSize(pSC_WDMABase, pScalerInfo.DST_WIDTH, pScalerInfo.DST_HEIGHT);
	

	#if !defined(CONFIG_PLATFORM_AVN) && !defined(CONFIG_PLATFORM_STB)
	if(scaler->info->wdma_aligned_offset)
		VIOC_WDMA_SetImageOffset_withYV12(pSC_WDMABase, scaler->info->dest_ImgWidth);
	else
		VIOC_WDMA_SetImageOffset(pSC_WDMABase, scaler->info->dest_fmt, scaler->info->dest_ImgWidth);
	#else

	VIOC_WDMA_SetImageOffset(pSC_WDMABase, scaler->info->dest_fmt, scaler->info->dest_ImgWidth);
	#endif//
	
	VIOC_WDMA_SetImageBase(pSC_WDMABase, (unsigned int)scaler->info->dest_Yaddr, (unsigned int)scaler->info->dest_Uaddr, (unsigned int)scaler->info->dest_Vaddr);
	#if defined(CONFIG_ARCH_TCC896X)
	if(scaler->info->dest_fmt == SC_IMG_FMT_FCDEC)	{
			VIOC_CONFIG_SWReset(pSC_IREQConfig, VIOC_CONFIG_FCENC, 0, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pSC_IREQConfig, VIOC_CONFIG_FCENC, 0, VIOC_CONFIG_CLEAR);
			VIOC_CONFIG_PlugIn(VIOC_FCENC0, scaler->wdma->id);
	}
	else 
	#endif//
	if ((scaler->info->src_fmt < SC_IMG_FMT_FCDEC) && (scaler->info->dest_fmt > SC_IMG_FMT_FCDEC)) {
		VIOC_WDMA_SetImageR2YEnable(pSC_WDMABase, 1);
	} else {
		VIOC_WDMA_SetImageR2YEnable(pSC_WDMABase, 0);
	}

	VIOC_WDMA_SetImageEnable(pSC_WDMABase, 0);
	pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF; // wdma status register all clear.

#endif
	spin_unlock_irq(&(scaler->data->cmd_lock));

	if (scaler->info->responsetype  == SCALER_POLLING) {
		ret = wait_event_interruptible_timeout(scaler->data->poll_wq,  scaler->data->block_operating == 0, msecs_to_jiffies(500));
		if (ret <= 0) {
			scaler->data->block_operating = 0;
			printk("%s():  time out(%d), line(%d). \n", __func__, ret, __LINE__);
		}
		// look up table use
		if(scaler->info->lut.use_lut)		{
			tcc_set_lut_enable(TVC_LUT(scaler->info->lut.use_lut_number), false);
			scaler->info->lut.use_lut = false;
		}
	} else if (scaler->info->responsetype  == SCALER_NOWAIT) {
		if(scaler->info->viqe_onthefly & 0x2)
			 scaler->data->block_operating = 0;
	}

	return ret;
}

static char tcc_scaler_data_copy_run(struct scaler_drv_type *scaler, SCALER_DATA_COPY_TYPE *copy_info)
{
	int ret = 0;
	VIOC_SCALER_INFO_Type pScalerInfo;

#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA pSC_RDMABase = (PVIOC_RDMA)scaler->rdma->reg;
	volatile PVIOC_WMIX pSC_WMIXBase = (PVIOC_WMIX)scaler->wmix->reg;
	volatile PVIOC_WDMA pSC_WDMABase = (PVIOC_WDMA)scaler->wdma->reg;
#endif

	dprintk("%s():  \n", __func__);
	dprintk("Src  : addr:0x%x 0x%x 0x%x  fmt:%d \n", copy_info->src_y_addr, copy_info->src_u_addr, copy_info->src_v_addr, copy_info->src_fmt);
	dprintk("Dest: addr:0x%x 0x%x 0x%x  fmt:%d \n", copy_info->dst_y_addr, copy_info->dst_u_addr, copy_info->dst_v_addr, copy_info->dst_fmt);
	dprintk("Size : W:%d  H:%d \n", copy_info->img_width, copy_info->img_height);


	spin_lock_irq(&(scaler->data->cmd_lock));

#ifdef CONFIG_OF_VIOC
	if (scaler->sc) {
		scaler->sc->data.bypass = 1;
		scaler->sc->data.dst_width = copy_info->img_width;
		scaler->sc->data.dst_height = copy_info->img_height;
		scaler->sc->data.out_x = 0;
		scaler->sc->data.out_y = 0;
		scaler->sc->data.out_width = pScalerInfo.DST_WIDTH;
		scaler->sc->data.out_height = pScalerInfo.DST_HEIGHT;
		vioc_sc_set_image(scaler->sc, 1);
	}

	if (scaler->rdma) {
		scaler->rdma->data.format = copy_info->src_fmt;
		scaler->rdma->data.width = copy_info->img_width;
		scaler->rdma->data.height = copy_info->img_height;
		scaler->rdma->data.offset = copy_info->img_width;
		scaler->rdma->data.base0 = (unsigned int)copy_info->src_y_addr;
		scaler->rdma->data.base1 = (unsigned int)copy_info->src_u_addr;
		scaler->rdma->data.base2 = (unsigned int)copy_info->src_v_addr;
		vioc_rdma_set_image(scaler->rdma, 1);
	}

	if (scaler->wmix) {
		scaler->wmix->mixmode = 1;	/* wmixer op is mixing mode. */
		scaler->wmix->data.width = copy_info->img_width;
		scaler->wmix->data.height = copy_info->img_height;
		vioc_wmix_set_image(scaler->wmix, 1);
	}

	if (scaler->wdma) {
		scaler->wdma->data.format = copy_info->dst_fmt;
		scaler->wdma->data.width = copy_info->img_width;
		scaler->wdma->data.height = copy_info->img_height;
		scaler->wdma->data.width = copy_info->img_width;
		scaler->wdma->data.base0 = (unsigned int)copy_info->dst_y_addr;
		scaler->wdma->data.base1 = (unsigned int)copy_info->dst_u_addr;
		scaler->wdma->data.base2 = (unsigned int)copy_info->dst_v_addr;
		scaler->wdma->data.cont = 0;
		vioc_wdma_set_image(scaler->wdma, 1);
	}
#else
	VIOC_RDMA_SetImageFormat(pSC_RDMABase, copy_info->src_fmt);
	VIOC_RDMA_SetImageSize(pSC_RDMABase, copy_info->img_width, copy_info->img_height);
	VIOC_RDMA_SetImageOffset(pSC_RDMABase, copy_info->src_fmt, copy_info->img_width);
	VIOC_RDMA_SetImageBase(pSC_RDMABase, (unsigned int)copy_info->src_y_addr, (unsigned int)copy_info->src_u_addr,  (unsigned int)copy_info->src_v_addr);

	pScalerInfo.BYPASS 			= 1;
	pScalerInfo.DST_WIDTH 		= copy_info->img_width;
	pScalerInfo.DST_HEIGHT 		= copy_info->img_height;
	pScalerInfo.OUTPUT_POS_X 		= 0;
	pScalerInfo.OUTPUT_POS_Y 		= 0;
	pScalerInfo.OUTPUT_WIDTH 		= pScalerInfo.DST_WIDTH;
	pScalerInfo.OUTPUT_HEIGHT 	= pScalerInfo.DST_HEIGHT;
	VIOC_API_SCALER_SetConfig(scaler->sc->id, &pScalerInfo);
	VIOC_API_SCALER_SetPlugIn(scaler->sc->type, scaler->sc->path);
	VIOC_API_SCALER_SetUpdate(scaler->sc->id);
	VIOC_RDMA_SetImageEnable(pSC_RDMABase); // SoC guide info.


	if(scaler->wmix->path == WMIX13)
		VIOC_CONFIG_WMIXPath(scaler->wmix->path, 0); // wmixer op is bypass mode.
	else
		VIOC_CONFIG_WMIXPath(scaler->wmix->path, 1); // wmixer op is mixing mode.	
		
	VIOC_WMIX_SetSize(pSC_WMIXBase, copy_info->img_width, copy_info->img_height);
	VIOC_WMIX_SetUpdate(pSC_WMIXBase);

	VIOC_WDMA_SetImageFormat(pSC_WDMABase, copy_info->dst_fmt);
	VIOC_WDMA_SetImageSize(pSC_WDMABase, copy_info->img_width, copy_info->img_height);
	VIOC_WDMA_SetImageOffset(pSC_WDMABase, copy_info->dst_fmt, copy_info->img_width);
	VIOC_WDMA_SetImageBase(pSC_WDMABase, (unsigned int)copy_info->dst_y_addr, (unsigned int)copy_info->dst_u_addr, (unsigned int)copy_info->dst_v_addr);
	VIOC_WDMA_SetImageEnable(pSC_WDMABase, 0/*OFF*/);
	pSC_WDMABase->uIRQSTS.nREG = 0xFFFFFFFF; // wdma status register all clear.
#endif

	spin_unlock_irq(&(scaler->data->cmd_lock));

	if (copy_info->rsp_type == SCALER_POLLING) {
		ret = wait_event_interruptible_timeout(scaler->data->poll_wq, scaler->data->block_operating == 0, msecs_to_jiffies(500));
		if (ret <= 0) {
			 scaler->data->block_operating = 0;
			printk("wmixer time out: %d, Line: %d. \n", ret, __LINE__);
		}
	}

	return ret;
}

static unsigned int scaler_drv_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct scaler_drv_type	*scaler = dev_get_drvdata(misc->parent);
	int ret = 0;

	if(scaler->data == NULL)
		return -EFAULT;

	poll_wait(filp, &(scaler->data->poll_wq), wait);
	spin_lock_irq(&(scaler->data->poll_lock));
	if (scaler->data->block_operating == 0)
		ret = (POLLIN|POLLRDNORM);
	spin_unlock_irq(&(scaler->data->poll_lock));

	return ret;
}

static void convert_image_format(struct scaler_drv_type *scaler)
{
	dprintk("before: src_fmt=%d, dst_fmt=%d. \n", scaler->info->src_fmt, scaler->info->dest_fmt);
	switch((unsigned char)scaler->info->src_fmt) {
		case SCALER_YUV422_sq0:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr422_SEQ_YUYV;
			break;
		case SCALER_YUV422_sq1:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr422_SEQ_UYVY;
			break;
		case SCALER_YUV422_sp:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr422_SEP;
			break;
		case SCALER_YUV420_sp:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr420_SEP;
			break;
		case SCALER_YUV422_inter:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr422_INT_TYPE0;
			break;
		case SCALER_YUV420_inter:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr420_INT_TYPE0;
			break;

		#if !defined(CONFIG_PLATFORM_AVN) && !defined(CONFIG_PLATFORM_STB)
		case SCALER_YUV420_inter_NV21:
			scaler->info->src_fmt = SC_IMG_FMT_YCbCr420_INT_TYPE1;
			break;
		#endif
		#if defined(CONFIG_ARCH_TCC896X)			
		case SCLAER_COMPRESS_DATA:
			scaler->info->src_fmt = SC_IMG_FMT_FCDEC;
			break;
		#endif//
		case SCALER_RGB565:
			scaler->info->src_fmt = SC_IMG_FMT_RGB565;
			break;
		case SCALER_RGB555:
			//scaler->info->src_fmt = SC_IMG_FMT_RGB555;
			//break;
		case SCALER_RGB454:
			scaler->info->src_fmt = SC_IMG_FMT_RGB555;
			break;
		case SCALER_RGB444:
			scaler->info->src_fmt = SC_IMG_FMT_RGB444;
			break;
		case SCALER_ARGB8888:
			scaler->info->src_fmt = SC_IMG_FMT_ARGB8888;
			break;
	}

	switch((unsigned char)scaler->info->dest_fmt) {
		case SCALER_YUV422_sq0:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr422_SEQ_YUYV;
			break;
		case SCALER_YUV422_sq1:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr422_SEQ_UYVY;
			break;
		case SCALER_YUV422_sp:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr422_SEP;
			break;
		case SCALER_YUV420_sp:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr420_SEP;
			break;
		case SCALER_YUV422_inter:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr422_INT_TYPE0;
			break;
		case SCALER_YUV420_inter:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr420_INT_TYPE0;
			break;

		#if !defined(CONFIG_PLATFORM_AVN) && !defined(CONFIG_PLATFORM_STB)
		case SCALER_YUV420_inter_NV21:
			scaler->info->dest_fmt = SC_IMG_FMT_YCbCr420_INT_TYPE1;
			break;
		#endif//
		#if defined(CONFIG_ARCH_TCC896X)						
		case SCLAER_COMPRESS_DATA:
			scaler->info->dest_fmt = SC_IMG_FMT_FCDEC;
			break;
		#endif//
		case SCALER_RGB565:
			scaler->info->dest_fmt = SC_IMG_FMT_RGB565;
			break;
		case SCALER_RGB555:
			//scaler->info->dest_fmt = SC_IMG_FMT_RGB555;
			//break;
		case SCALER_RGB454:
			scaler->info->dest_fmt = SC_IMG_FMT_RGB555;
			break;
		case SCALER_RGB444:
			scaler->info->dest_fmt = SC_IMG_FMT_RGB444;
			break;
		case SCALER_ARGB8888:
			scaler->info->dest_fmt = SC_IMG_FMT_ARGB8888;
			break;
	}
	dprintk("after: src_fmt=%d, dst_fmt=%d. \n", scaler->info->src_fmt, scaler->info->dest_fmt);
}

static irqreturn_t scaler_drv_handler(int irq, void *client_data)
{
	struct scaler_drv_type *scaler = (struct scaler_drv_type *)client_data;

	if (is_vioc_intr_activatied(scaler->vioc_intr->id, scaler->vioc_intr->bits) == false)
		return IRQ_NONE;
	vioc_intr_clear(scaler->vioc_intr->id, scaler->vioc_intr->bits);

	dprintk("%s():  block_operating(%d), block_waiting(%d), cmd_count(%d), poll_count(%d). \n", __func__, 	\
			scaler->data->block_operating, scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->poll_count);		

	if(scaler->data->block_operating >= 1)
		scaler->data->block_operating = 0;

	wake_up_interruptible(&(scaler->data->poll_wq));

	if(scaler->data->block_waiting)
		wake_up_interruptible(&scaler->data->cmd_wq);

	// look up table use
	if(scaler->info->lut.use_lut)		{
		tcc_set_lut_enable(TVC_LUT(scaler->info->lut.use_lut_number), false);
		scaler->info->lut.use_lut = false;
	}

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER	
	tca_set_MC_Connect_to_RDMA(1, scaler->rdma->id, 0);
#endif//
	return IRQ_HANDLED;
}

static long scaler_drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct scaler_drv_type	*scaler = dev_get_drvdata(misc->parent);

#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA pSC_RDMABase = (PVIOC_RDMA)scaler->rdma->reg;
#endif

	int ret = 0;
	SCALER_PLUGIN_Type scaler_plugin;
	VIOC_SCALER_INFO_Type pScalerInfo;
	SCALER_DATA_COPY_TYPE copy_info;

	dprintk("%s():  cmd(%d), block_operating(%d), block_waiting(%d), cmd_count(%d), poll_count(%d). \n", __func__, 	\
			cmd, scaler->data->block_operating, scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->poll_count);

	switch(cmd) {
		case TCC_SCALER_IOCTRL:			
		case TCC_SCALER_IOCTRL_KERENL:
			mutex_lock(&scaler->data->io_mutex);
			if(scaler->data->block_operating) {
				scaler->data->block_waiting = 1;
				ret = wait_event_interruptible_timeout(scaler->data->cmd_wq, scaler->data->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					scaler->data->block_operating = 0;
					printk("%s(%d):  timed_out block_operation(%d), cmd_count(%d). \n", __func__, ret, scaler->data->block_waiting, scaler->data->cmd_count);
				}
				ret = 0;
			}

			if(cmd == TCC_SCALER_IOCTRL_KERENL) {
				memcpy(scaler->info, (SCALER_TYPE*)arg, sizeof(SCALER_TYPE));
			} else {
				if(copy_from_user(scaler->info, (SCALER_TYPE*)arg, sizeof(SCALER_TYPE))) {
					printk(KERN_ALERT "%s():  Not Supported copy_from_user(%d). \n", __func__, cmd);
					ret = -EFAULT;
				}
			}

			if(ret >= 0) {
				if(scaler->data->block_operating >= 1) {
					printk("%s():  block_operating(%d), block_waiting(%d), cmd_count(%d), poll_count(%d). \n", __func__, 	\
							scaler->data->block_operating, scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->poll_count);
				}

				convert_image_format(scaler);

				scaler->data->block_waiting = 0;
				scaler->data->block_operating = 1;
				ret = tcc_scaler_run(scaler);
				if(ret < 0) scaler->data->block_operating = 0;
			}
			mutex_unlock(&scaler->data->io_mutex);
			return ret;

		case TCC_SCALER_VIOC_PLUGIN:
			mutex_lock(&scaler->data->io_mutex);
			if(copy_from_user(&scaler_plugin,(SCALER_PLUGIN_Type *)arg, sizeof(SCALER_PLUGIN_Type))) {
				printk(KERN_ALERT "%s():  Not Supported copy_from_user(%d)\n", __func__, cmd);
				ret = -EFAULT;
			}

			// set to scaler & plug in.
			pScalerInfo.BYPASS 			= scaler_plugin.bypass_mode;
			pScalerInfo.SRC_WIDTH 		= scaler_plugin.src_width;
			pScalerInfo.SRC_HEIGHT 		= scaler_plugin.src_height;
			pScalerInfo.DST_WIDTH 		= scaler_plugin.dst_width;
			pScalerInfo.DST_HEIGHT 		= scaler_plugin.dst_height;
			pScalerInfo.OUTPUT_POS_X 		= scaler_plugin.dst_win_left;
			pScalerInfo.OUTPUT_POS_Y 		= scaler_plugin.dst_win_top;
			pScalerInfo.OUTPUT_WIDTH 		= (scaler_plugin.dst_width  - scaler_plugin.dst_win_left);
			pScalerInfo.OUTPUT_HEIGHT 	= (scaler_plugin.dst_height - scaler_plugin.dst_win_top);

			VIOC_SC_SetSWReset(scaler_plugin.scaler_no, 0xFF/*RDMA*/, 0xFF/*WDMA*/);
			VIOC_API_SCALER_SetConfig(scaler_plugin.scaler_no, &pScalerInfo);
			ret = VIOC_API_SCALER_SetPlugIn(scaler_plugin.scaler_no, scaler_plugin.plugin_path);
			VIOC_API_SCALER_SetUpdate(scaler_plugin.scaler_no);			
			mutex_unlock(&scaler->data->io_mutex);
			return ret;

		case TCC_SCALER_VIOC_PLUGOUT:
#ifdef CONFIG_OF_VIOC
			vioc_rdma_interlaced_iamge(scaler->rdma, 0);
#else
			VIOC_RDMA_SetImageIntl(pSC_RDMABase, 0);
#endif
			ret = VIOC_API_SCALER_SetPlugOut((unsigned int)arg);
			return ret;

		case TCC_SCALER_VIOC_DATA_COPY:
			mutex_lock(&scaler->data->io_mutex);
			if(scaler->data->block_operating) {
				scaler->data->block_waiting = 1;
				ret = wait_event_interruptible_timeout(scaler->data->cmd_wq, scaler->data->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					scaler->data->block_operating = 0;
					printk("%s(%d):  wmixer 0 timed_out block_operation(%d), cmd_count(%d). \n", __func__, ret, scaler->data->block_waiting, scaler->data->cmd_count);
				}
				ret = 0;
			}

			if(copy_from_user(&copy_info, (SCALER_DATA_COPY_TYPE *)arg, sizeof(SCALER_DATA_COPY_TYPE))) {
				printk(KERN_ALERT "%s():  Not Supported copy_from_user(%d)\n", __func__, cmd);
				ret = -EFAULT;
			}

			if(ret >= 0) {
				if(scaler->data->block_operating >= 1) {
					printk("%s():  block_operating(%d), block_waiting(%d), cmd_count(%d), poll_count(%d). \n", __func__, 	\
						scaler->data->block_operating, scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->poll_count);
				}

				scaler->data->block_waiting = 0;
				scaler->data->block_operating = 1;
				ret = tcc_scaler_data_copy_run(scaler, &copy_info);
				if(ret < 0) 	scaler->data->block_operating = 0;
			}
			mutex_unlock(&scaler->data->io_mutex);
			return ret;

		default:
			printk(KERN_ALERT "%s():  Not Supported SCALER0_IOCTL(%d). \n", __func__, cmd);
			break;			
	}

	return 0;
}

static int scaler_drv_release(struct inode *inode, struct file *filp)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct scaler_drv_type	*scaler = dev_get_drvdata(misc->parent);

	int ret = 0;
	dprintk("%s():  In -release(%d), block(%d), wait(%d), cmd(%d), irq(%d) \n", __func__, scaler->data->dev_opened, scaler->data->block_operating, \
			scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->irq_reged);

	if (scaler->data->dev_opened > 0) {
		scaler->data->dev_opened--;
	}

	if (scaler->data->dev_opened == 0) {
		if (scaler->data->block_operating) {
			ret = wait_event_interruptible_timeout(scaler->data->cmd_wq, scaler->data->block_operating == 0, msecs_to_jiffies(200));
			if (ret <= 0) {
	 			printk("%s(%d):  timed_out block_operation:%d, cmd_count:%d. \n", __func__, ret, scaler->data->block_waiting, scaler->data->cmd_count);
		#if 0 // debug!!
				{
					if(scaler->info->mapConv_info.m_CompressedY[0] != 0)
					{
						VIOC_MC_DUMP(1);
						printk("Map-converter :: 0x%x/0x%x - 0x%x/0x%x, Stride(%d/%d), Depth(%d/%d)\n",
								scaler->info->mapConv_info.m_CompressedY[0], scaler->info->mapConv_info.m_CompressedCb[0],
								scaler->info->mapConv_info.m_FbcYOffsetAddr[0], scaler->info->mapConv_info.m_FbcCOffsetAddr[0],
								scaler->info->mapConv_info.m_uiLumaStride, scaler->info->mapConv_info.m_uiChromaStride,
								scaler->info->mapConv_info.m_uiLumaBitDepth, scaler->info->mapConv_info.m_uiChromaBitDepth);
					}
					else{
						VIOC_RDMA_DUMP(scaler->rdma->reg);
					}
					VIOC_SCALER_DUMP(scaler->sc->reg);
					VIOC_WMIX_DUMP(scaler->wmix->reg);
					VIOC_WDMA_DUMP(scaler->wdma->reg);
					printk("scaler src   add : 0x%p 0x%p 0x%p, fmt :0x%x IMG:(%d %d )%d %d %d %d \n",
								scaler->info->src_Yaddr, scaler->info->src_Uaddr, scaler->info->src_Vaddr, scaler->info->src_fmt,
								scaler->info->src_ImgWidth, scaler->info->src_ImgHeight,
								scaler->info->src_winLeft, scaler->info->src_winTop, scaler->info->src_winRight, scaler->info->src_winBottom);
					printk("scaler dest  add : 0x%p 0x%p 0x%p, fmt :0x%x IMG:(%d %d )%d %d %d %d \n",
								scaler->info->dest_Yaddr, scaler->info->dest_Uaddr, scaler->info->dest_Vaddr, scaler->info->dest_fmt,
								scaler->info->dest_ImgWidth, scaler->info->dest_ImgHeight,
								scaler->info->dest_winLeft, scaler->info->dest_winTop, scaler->info->dest_winRight, scaler->info->dest_winBottom);
				}
		#endif
			}
		}

		if (scaler->data->irq_reged) {
			free_irq(scaler->irq, scaler);
			scaler->data->irq_reged = 0;
		}

		VIOC_CONFIG_PlugOut(scaler->sc->type);
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
		tca_set_MC_Connect_to_RDMA(1, scaler->rdma->id, 0);
#endif//
		vioc_wdma_swreset(scaler->wdma, 1);
		vioc_wmix_swreset(scaler->wmix, 1);
		vioc_sc_swreset(scaler->sc, 1);
		vioc_rdma_swreset(scaler->rdma, 1);
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
		tca_map_convter_swreset(1);
#endif
		vioc_rdma_swreset(scaler->rdma, 0);
		vioc_sc_swreset(scaler->sc, 0);
		vioc_wmix_swreset(scaler->wmix, 0);
		vioc_wdma_swreset(scaler->wdma, 0);

		scaler->data->block_operating = scaler->data->block_waiting = 0;
		scaler->data->poll_count = scaler->data->cmd_count = 0;
	}

	if (scaler->clk)
		clk_disable_unprepare(scaler->clk);
	dprintk("%s():  Out - release(%d). \n", __func__, scaler->data->dev_opened);

	return 0;
}

static int scaler_drv_open(struct inode *inode, struct file *filp)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct scaler_drv_type	*scaler = dev_get_drvdata(misc->parent);

	int ret = 0;
	dprintk("%s():  In -open(%d), block(%d), wait(%d), cmd(%d), irq(%d) \n", __func__, scaler->data->dev_opened, scaler->data->block_operating, \
			scaler->data->block_waiting, scaler->data->cmd_count, scaler->data->irq_reged);

	if (scaler->clk)
		clk_prepare_enable(scaler->clk);

	if (!scaler->data->irq_reged) {
		vioc_wdma_swreset(scaler->wdma, 1);
		vioc_wmix_swreset(scaler->wmix, 1);
		vioc_sc_swreset(scaler->sc, 1);
		vioc_rdma_swreset(scaler->rdma, 1);
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
        tca_map_convter_swreset(1);
#endif
		vioc_rdma_swreset(scaler->rdma, 0);
		vioc_sc_swreset(scaler->sc, 0);
		vioc_wmix_swreset(scaler->wmix, 0);
		vioc_wdma_swreset(scaler->wdma, 0);

		#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X)
			// Don't STOP_REQ clear because of dual output problem(DISP FIFO under-run), when video is out.
			vioc_config_stop_req(0);
		#else
			vioc_config_stop_req(1);
		#endif

		synchronize_irq(scaler->irq);
		vioc_intr_clear(scaler->vioc_intr->id, scaler->vioc_intr->bits);
		ret = request_irq(scaler->irq, scaler_drv_handler, IRQF_SHARED, scaler->misc->name, scaler);
		if (ret) {
			if (scaler->clk)
				clk_disable_unprepare(scaler->clk);
			printk("failed to aquire %s request_irq. \n", scaler->misc->name);
			return -EFAULT;
		}
		vioc_intr_enable(scaler->vioc_intr->id, scaler->vioc_intr->bits);
		scaler->data->irq_reged = 1;
	}

	scaler->data->dev_opened++;
	dprintk("%s():  Out - open(%d). \n", __func__, scaler->data->dev_opened);
	return ret;
}

static struct file_operations scaler_drv_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= scaler_drv_ioctl,
	.mmap			= scaler_drv_mmap,
	.open			= scaler_drv_open,
	.release		= scaler_drv_release,
	.poll			= scaler_drv_poll,
};

static int scaler_drv_probe(struct platform_device *pdev)
{
	struct scaler_drv_type *scaler;
	int ret = -ENODEV;

	scaler = kzalloc(sizeof(struct scaler_drv_type), GFP_KERNEL);
	if (!scaler)
		return -ENOMEM;

	scaler->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(scaler->clk))
		scaler->clk = NULL;

	scaler->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (scaler->misc == 0)
		goto err_misc_alloc;

	scaler->info = kzalloc(sizeof(SCALER_TYPE), GFP_KERNEL);
	if (scaler->info == 0)
		goto err_info_alloc;

	scaler->data = kzalloc(sizeof(struct scaler_data), GFP_KERNEL);
	if (scaler->data == 0)
		goto err_data_alloc;

	scaler->vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (scaler->vioc_intr == 0)
		goto err_vioc_intr_alloc;

 	/* register scaler discdevice */
	scaler->misc->minor = MISC_DYNAMIC_MINOR;
	scaler->misc->fops = &scaler_drv_fops;
	scaler->misc->name = pdev->name;
	scaler->misc->parent = &pdev->dev;
	ret = misc_register(scaler->misc);
	if (ret)
		goto err_misc_register;

	scaler->id = of_alias_get_id(pdev->dev.of_node, "scaler_drv");

	scaler->rdma = devm_vioc_rdma_get(&pdev->dev, 0);
	if (IS_ERR(scaler->rdma)) {
		printk("could not find rdma0 node of %s driver. \n", scaler->misc->name);
		scaler->rdma = NULL;
	}

	scaler->wmix = devm_vioc_wmix_get(&pdev->dev, 0);
	if (IS_ERR(scaler->wmix)) {
		printk("could not find wmix node of %s driver. \n", scaler->misc->name);
		scaler->wmix = NULL;
	}

	scaler->sc = devm_vioc_sc_get(&pdev->dev, 0);
	if (IS_ERR(scaler->sc)) {
		printk("could not find sc node of %s driver. \n", scaler->misc->name);
		scaler->sc = NULL;
		
	}else {
	
		switch(scaler->sc->id) {
			
			case 0 :
				scaler->sc->type = VIOC_SC0;
				break;
			case 1 :
				scaler->sc->type = VIOC_SC1;
				break;
			case 2 :
				scaler->sc->type = VIOC_SC2;
				break;
			case 3 :
				scaler->sc->type = VIOC_SC3;
				break;
			case 4 :
				scaler->sc->type = VIOC_SC4;
				break;
			default:
				printk("Not support sc->id = %d\n",
						scaler->sc->id);
				goto err_misc_register;
		}
	}

	scaler->wdma = devm_vioc_wdma_get(&pdev->dev, 0);
	if (IS_ERR(scaler->wdma)) {
		printk("could not find wdma node of %s driver. \n", scaler->misc->name);
		scaler->wdma = NULL;
	}
	else {
		scaler->irq		= scaler->wdma->irq;
		scaler->vioc_intr->id	= scaler->wdma->intr->id;
		scaler->vioc_intr->bits	= scaler->wdma->intr->bits;
	}

	of_property_read_u32(pdev->dev.of_node, "settop_support", &scaler->settop_support);

	spin_lock_init(&(scaler->data->poll_lock));
	spin_lock_init(&(scaler->data->cmd_lock));

	mutex_init(&(scaler->data->io_mutex));
	
	init_waitqueue_head(&(scaler->data->poll_wq));
	init_waitqueue_head(&(scaler->data->cmd_wq));

	platform_set_drvdata(pdev, scaler);

	pr_info("%s: id:%d, Scaler Driver Initialized\n", pdev->name, scaler->id);
	return 0;

	misc_deregister(scaler->misc);
err_misc_register:
	kfree(scaler->vioc_intr);
err_vioc_intr_alloc:
	kfree(scaler->data);
err_data_alloc:
	kfree(scaler->info);
err_info_alloc:
	kfree(scaler->misc);
err_misc_alloc:
	kfree(scaler);

	printk("%s: %s: err ret:%d \n", __func__, pdev->name, ret);
	return ret;
}

static int scaler_drv_remove(struct platform_device *pdev)
{
	struct scaler_drv_type *scaler = (struct scaler_drv_type *)platform_get_drvdata(pdev);

	misc_deregister(scaler->misc);
	kfree(scaler->vioc_intr);
	kfree(scaler->data);
	kfree(scaler->info);
	kfree(scaler->misc);
	kfree(scaler);
	return 0;
}

static int scaler_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	// TODO:
	return 0;
}

static int scaler_drv_resume(struct platform_device *pdev)
{
	struct scaler_drv_type *scaler = (struct scaler_drv_type *)platform_get_drvdata(pdev);

	if(scaler->data->dev_opened > 0) {
		vioc_wdma_swreset(scaler->wdma, 1);
		vioc_wmix_swreset(scaler->wmix, 1);
		vioc_sc_swreset(scaler->sc, 1);
		vioc_rdma_swreset(scaler->rdma, 1);

		vioc_rdma_swreset(scaler->rdma, 0);
		vioc_sc_swreset(scaler->sc, 0);
		vioc_wmix_swreset(scaler->wmix, 0);
		vioc_wdma_swreset(scaler->wdma, 0);
	}

	return 0;
}

static struct of_device_id scaler_of_match[] = {
	{ .compatible = "telechips,scaler_drv" },
	{}
};
MODULE_DEVICE_TABLE(of, scaler_of_match);

static struct platform_driver scaler_driver = {
	.probe		= scaler_drv_probe,
	.remove		= scaler_drv_remove,
	.suspend	= scaler_drv_suspend,
	.resume		= scaler_drv_resume,
	.driver 	= {
		.name	= "scaler_pdev",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(scaler_of_match),
#endif
	},
};

static int __init scaler_drv_init(void)
{
	return platform_driver_register(&scaler_driver);
}

static void __exit scaler_drv_exit(void)
{
	platform_driver_unregister(&scaler_driver);
}

module_init(scaler_drv_init);
module_exit(scaler_drv_exit);


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("Telechips Scaler Driver");
MODULE_LICENSE("GPL");

