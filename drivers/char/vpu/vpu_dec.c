/****************************************************************************
 *   FileName    : vpu_dec.c
 *   Description : TCC VPU h/w block
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips Inc.
 *   All rights reserved 
 
This source code contains confidential information of Telechips.
Any unauthorized use without a written permission of Telechips including not limited to re-distribution in source or binary form is strictly prohibited.
This source code is provided "AS IS" and nothing contained in this source code shall constitute any express or implied warranty of any kind, including without limitation, any warranty of merchantability, fitness for a particular purpose or non-infringement of any patent, copyright or other third party intellectual property right. No warranty is made, express or implied, regarding the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability arising from, out of or in connection with this source code or the use in the source code. 
This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between Telechips and Company.
*
****************************************************************************/

#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/poll.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#include "vpu_buffer.h"
#include "vpu_mgr.h"

#ifdef CONFIG_SUPPORT_TCC_JPU
#include "jpu_mgr.h"
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
#include "hevc_mgr.h"
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
#include "vp9_mgr.h"
#endif

#if defined(CONFIG_VDEC_CNT_1) || defined(CONFIG_VDEC_CNT_2) || defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)

#define dprintk(msg...)	//printk( "TCC_VPU_DEC: " msg);
#define detailk(msg...) //printk( "TCC_VPU_DEC: " msg);
#define err(msg...) printk("TCC_VPU_DEC[Err]: "msg);

extern int range_is_allowed(unsigned long pfn, unsigned long size);
extern int vmem_get_pgprot(unsigned long ulOldProt, unsigned long ulPageOffset);

static void _vdec_inter_add_list(vpu_decoder_data *vdata, int cmd, void* args)
{
	vdata->vdec_list[vdata->list_idx].type 			= vdata->gsDecType;
	vdata->vdec_list[vdata->list_idx].cmd_type  	= cmd;
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
		vdata->vdec_list[vdata->list_idx].handle 	= vdata->gsJpuDecInit_Info.gsJpuDecHandle;
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
		vdata->vdec_list[vdata->list_idx].handle 	= vdata->gsHevcDecInit_Info.gsHevcDecHandle;
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
		vdata->vdec_list[vdata->list_idx].handle 	= vdata->gsVp9DecInit_Info.gsVp9DecHandle;
	else
#endif
		vdata->vdec_list[vdata->list_idx].handle 	= vdata->gsVpuDecInit_Info.gsVpuDecHandle;
	vdata->vdec_list[vdata->list_idx].args 			= args;	   
	vdata->vdec_list[vdata->list_idx].comm_data 	= &vdata->vComm_data;
	vdata->gsCommDecResult = RET0;
	vdata->vdec_list[vdata->list_idx].vpu_result	= &vdata->gsCommDecResult;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
		jmgr_list_manager(&vdata->vdec_list[vdata->list_idx], LIST_ADD);
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
		hmgr_list_manager(&vdata->vdec_list[vdata->list_idx], LIST_ADD);
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
		vp9mgr_list_manager(&vdata->vdec_list[vdata->list_idx], LIST_ADD);
	else
#endif
		vmgr_list_manager(&vdata->vdec_list[vdata->list_idx], LIST_ADD);

	vdata->list_idx = (vdata->list_idx+1)%LIST_MAX;
}

static void _vdec_init_list(vpu_decoder_data *vdata )
{
	int i = 0;

	for(i=0; i<LIST_MAX; i++)
	{
		vdata->vdec_list[i].comm_data = NULL;
	}
}

static int _vdec_proc_init(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	dprintk("%s :: _vdec_proc_init!! \n", vdata->misc->name);

	_vdec_init_list(vdata);

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecInit_Info, arg, sizeof(JDEC_INIT_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecInit_Info;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecInit_Info, arg, sizeof(HEVC_INIT_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecInit_Info;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecInit_Info, arg, sizeof(VP9_INIT_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecInit_Info;
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecInit_Info, arg, sizeof(VDEC_INIT_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecInit_Info;
	}

	_vdec_inter_add_list(vdata, VPU_DEC_INIT, pArgs);

	return 0;
}

static int _vdec_proc_exit(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	dprintk("%s :: _vdec_proc_exit!! \n", vdata->misc->name);

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecInOut_Info, arg, sizeof(JPU_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecInOut_Info;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecInOut_Info, arg, sizeof(HEVC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecInOut_Info;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecInOut_Info, arg, sizeof(VP9_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecInOut_Info;
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecInOut_Info, arg, sizeof(VDEC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecInOut_Info;
	}

	_vdec_inter_add_list(vdata, VPU_DEC_CLOSE, pArgs);

	return 0;
}

static int _vdec_proc_seq_header(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	dprintk("%s :: _vdec_proc_seq_header!! \n", vdata->misc->name);

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
#if defined(JPU_C6)
		if(copy_from_user(&vdata->gsJpuDecSeqHeader_Info, arg, sizeof(JDEC_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecSeqHeader_Info;
#else
		err("%s :: jpu not support this !! \n", vdata->misc->name);return -0x999;
#endif
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecSeqHeader_Info, arg, sizeof(HEVC_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecSeqHeader_Info;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecSeqHeader_Info, arg, sizeof(VP9_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecSeqHeader_Info;
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecSeqHeader_Info, arg, sizeof(VDEC_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecSeqHeader_Info;
	}

	_vdec_inter_add_list(vdata, VPU_DEC_SEQ_HEADER, pArgs);

	return 0;
}

static int _vdec_proc_reg_framebuffer(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecBuffer_Info, arg, sizeof(JPU_SET_BUFFER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecBuffer_Info;
		
		detailk("%s :: jpu_proc_reg_framebuffer :: phy = 0x%x, virt = 0x%x, cnt = 0x%x !! \n", vdata->misc->name, vdata->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_FrameBufferStartAddr[0],
						vdata->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_FrameBufferStartAddr[1], vdata->gsJpuDecBuffer_Info.gsJpuDecBuffer.m_iFrameBufferCount);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecBuffer_Info, arg, sizeof(HEVC_SET_BUFFER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecBuffer_Info;
		
		detailk("%s :: hevc_proc_reg_framebuffer :: phy = 0x%x, virt = 0x%x, cnt = 0x%x !! \n", vdata->misc->name, vdata->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[0],
						vdata->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_FrameBufferStartAddr[1], vdata->gsHevcDecBuffer_Info.gsHevcDecBuffer.m_iFrameBufferCount);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecBuffer_Info, arg, sizeof(VP9_SET_BUFFER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecBuffer_Info;
		
		detailk("%s :: vp9_proc_reg_framebuffer :: phy = 0x%x, virt = 0x%x, cnt = 0x%x !! \n", vdata->misc->name, vdata->gsVp9DecBuffer_Info.gsVp9DecBuffer.m_FrameBufferStartAddr[0],
						vdata->gsVp9DecBuffer_Info.gsVp9DecBuffer.m_FrameBufferStartAddr[1], vdata->gsVp9DecBuffer_Info.gsVp9DecBuffer.m_iFrameBufferCount);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecBuffer_Info, arg, sizeof(VDEC_SET_BUFFER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecBuffer_Info;
		
		detailk("%s :: _vdec_proc_reg_framebuffer :: phy = 0x%x, virt = 0x%x, cnt = 0x%x !! \n", vdata->misc->name, vdata->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[0],
						vdata->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_FrameBufferStartAddr[1], vdata->gsVpuDecBuffer_Info.gsVpuDecBuffer.m_iFrameBufferCount);
	}

	_vdec_inter_add_list(vdata, VPU_DEC_REG_FRAME_BUFFER, pArgs);

	return 0;
}

static int _vdec_proc_decode(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecInOut_Info, arg, sizeof(JPU_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecInOut_Info;
		
		detailk("%s ::_jpu_proc_decode In !! handle = 0x%x, in_stream_size = 0x%x \n", vdata->misc->name, vdata->gsJpuDecInit_Info.gsJpuDecHandle, vdata->gsJpuDecInOut_Info.gsJpuDecInput.m_iBitstreamDataSize);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecInOut_Info, arg, sizeof(HEVC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecInOut_Info;
		
		detailk("%s ::_hevc_proc_decode In !! handle = 0x%x, in_stream_size = 0x%x \n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle, vdata->gsHevcDecInOut_Info.gsHevcDecInput.m_iBitstreamDataSize);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecInOut_Info, arg, sizeof(VP9_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecInOut_Info;
		
		detailk("%s ::_vp9_proc_decode In !! handle = 0x%x, in_stream_size = 0x%x \n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle, vdata->gsVp9DecInOut_Info.gsVp9DecInput.m_iBitstreamDataSize);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecInOut_Info, arg, sizeof(VDEC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecInOut_Info;
		
		detailk("%s ::_vdec_proc_decode In !! handle = 0x%x, in_stream_size = 0x%x \n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle, vdata->gsVpuDecInOut_Info.gsVpuDecInput.m_iBitstreamDataSize);
	}

	_vdec_inter_add_list(vdata, VPU_DEC_DECODE, pArgs);
	
	return 0;
}

static int _vdec_proc_clear_bufferflag(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;

	if(copy_from_user(&vdata->gsDecClearBuffer_index, arg, sizeof(unsigned int)))
		return -EFAULT;
	pArgs = ( void *)&vdata->gsDecClearBuffer_index;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{		
		detailk("%s ::_jpu_proc_clear_bufferflag : %d !! \n", vdata->misc->name, vdata->gsDecClearBuffer_index);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{		
		detailk("%s ::_hevc_proc_clear_bufferflag : %d !! \n", vdata->misc->name, vdata->gsDecClearBuffer_index);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{		
		detailk("%s ::_vp9_proc_clear_bufferflag : %d !! \n", vdata->misc->name, vdata->gsDecClearBuffer_index);
	}
	else
#endif
	{
		detailk("%s ::_vdec_proc_clear_bufferflag : %d !! \n", vdata->misc->name, vdata->gsDecClearBuffer_index);
	}

	_vdec_inter_add_list(vdata, VPU_DEC_BUF_FLAG_CLEAR, pArgs);

	return 0;
}

static int _vdec_proc_flush(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecInOut_Info, arg, sizeof(JPU_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecInOut_Info;
		
		detailk("%s ::jpu_proc_flush In !! handle = 0x%x \n", vdata->misc->name, vdata->gsJpuDecInit_Info.gsJpuDecHandle);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecInOut_Info, arg, sizeof(HEVC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecInOut_Info;
		
		detailk("%s ::hevc_proc_flush In !! handle = 0x%x \n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecInOut_Info, arg, sizeof(VP9_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecInOut_Info;
		
		detailk("%s ::vp9_proc_flush In !! handle = 0x%x \n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecInOut_Info, arg, sizeof(VDEC_DECODE_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecInOut_Info;
		
		detailk("%s ::_vdec_proc_flush In !! handle = 0x%x \n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}

	_vdec_inter_add_list(vdata, VPU_DEC_FLUSH_OUTPUT, pArgs);
	
	return 0;
}

static int _vdec_result_general(vpu_decoder_data *vdata, void *arg)
{
	if (copy_to_user(arg, &vdata->gsCommDecResult, sizeof(int)))
		return -EFAULT;

	return 0;
}

static int _vdec_result_init(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		vdata->gsJpuDecInit_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsJpuDecInit_Info, sizeof(JDEC_INIT_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecInit_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecInit_Info, sizeof(HEVC_INIT_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecInit_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecInit_Info, sizeof(VP9_INIT_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecInit_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecInit_Info, sizeof(VDEC_INIT_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_seq_header(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
#if defined(JPU_C6)
		vdata->gsJpuDecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsJpuDecSeqHeader_Info, sizeof(JDEC_SEQ_HEADER_t)))
			return -EFAULT;
#else
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
#endif
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecSeqHeader_Info, sizeof(HEVC_SEQ_HEADER_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecSeqHeader_Info, sizeof(VP9_SEQ_HEADER_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecSeqHeader_Info, sizeof(VDEC_SEQ_HEADER_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_decode(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		vdata->gsJpuDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsJpuDecInOut_Info, sizeof(JPU_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecInOut_Info, sizeof(HEVC_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecInOut_Info, sizeof(VP9_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecInOut_Info, sizeof(VDEC_DECODE_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_flush(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		vdata->gsJpuDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsJpuDecInOut_Info, sizeof(JPU_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecInOut_Info, sizeof(HEVC_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecInOut_Info, sizeof(VP9_DECODE_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecInOut_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecInOut_Info, sizeof(VDEC_DECODE_t)))
			return -EFAULT;
	}

	return 0;
}


static int _vdec_proc_swreset(vpu_decoder_data *vdata)
{
	_vdec_inter_add_list(vdata, VPU_DEC_SWRESET, ( void *)NULL);
	
	return 0;
}

//JS Baek(111220)-S
static int _vdec_proc_buf_status(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecBufStatus, arg, sizeof(HEVC_RINGBUF_GETINFO_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecBufStatus;
		
		detailk("%s ::hevc_proc_buf_status In !! handle = 0x%x\n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecBufStatus, arg, sizeof(VP9_RINGBUF_GETINFO_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecBufStatus;
		
		detailk("%s ::vp9_proc_buf_status In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecBufStatus, arg, sizeof(VDEC_RINGBUF_GETINFO_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecBufStatus;
		
		detailk("%s ::vdec_proc_buf_status In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}

	_vdec_inter_add_list(vdata, GET_RING_BUFFER_STATUS, pArgs);
	
	return 0;
}

static int _vdec_proc_buf_fill(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecBufFill, arg, sizeof(HEVC_RINGBUF_SETBUF_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecBufFill;
		
		detailk("%s ::hevc_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecBufFill, arg, sizeof(VP9_RINGBUF_SETBUF_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecBufFill;
		
		detailk("%s ::vp9_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecBufFill, arg, sizeof(VDEC_RINGBUF_SETBUF_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecBufFill;
		
		detailk("%s ::_vdec_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}

	_vdec_inter_add_list(vdata, FILL_RING_BUFFER_AUTO, pArgs);
	
	return 0;
}

static int _vdec_proc_update_wp(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecUpdateWP, arg, sizeof(HEVC_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecUpdateWP;
		
		detailk("%s ::hevc_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecUpdateWP, arg, sizeof(VP9_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecUpdateWP;
		
		detailk("%s ::vp9_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecUpdateWP, arg, sizeof(VDEC_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecUpdateWP;
		
		detailk("%s ::_vdec_proc_buf_fill In !! handle = 0x%x\n", vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}

	_vdec_inter_add_list(vdata, VPU_UPDATE_WRITE_BUFFER_PTR, pArgs);
	
	return 0;
}

static int _vdec_proc_seq_header_ring(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;
	
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecSeqHeader_Info, arg, sizeof(HEVC_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecSeqHeader_Info;
		
		detailk("%s ::hevc_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecSeqHeader_Info, arg, sizeof(VP9_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecSeqHeader_Info;
		
		detailk("%s ::vp9_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecSeqHeader_Info, arg, sizeof(VDEC_SEQ_HEADER_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecSeqHeader_Info;
		
		detailk("%s ::_vdec_proc_buf_fill In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}

	_vdec_inter_add_list(vdata, GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY, pArgs);
	
	return 0;
}

static int _vdec_proc_get_version(vpu_decoder_data *vdata, void *arg)
{
	void *pArgs;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(copy_from_user(&vdata->gsJpuDecVersion, arg, sizeof(JPU_GET_VERSION_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsJpuDecVersion;
		
		detailk("%s ::jpu_proc_get_version In !! handle = 0x%x\n", vdata->misc->name, vdata->gsJpuDecInit_Info.gsJpuDecHandle);
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(copy_from_user(&vdata->gsHevcDecVersion, arg, sizeof(HEVC_GET_VERSION_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsHevcDecVersion;
		
		detailk("%s ::hevc_proc_get_version In !! handle = 0x%x\n", vdata->misc->name, vdata->gsHevcDecInit_Info.gsHevcDecHandle);
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(copy_from_user(&vdata->gsVp9DecVersion, arg, sizeof(VP9_GET_VERSION_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVp9DecVersion;
		
		detailk("%s ::vp9_proc_get_version In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVp9DecInit_Info.gsVp9DecHandle);
	}
	else
#endif
	{
		if(copy_from_user(&vdata->gsVpuDecVersion, arg, sizeof(VDEC_GET_VERSION_t)))
			return -EFAULT;
		pArgs = ( void *)&vdata->gsVpuDecVersion;
		
		detailk("%s ::_vdec_proc_get_version In !! handle = 0x%x\n", vdata->misc->name, vdata->gsVpuDecInit_Info.gsVpuDecHandle);
	}	

	_vdec_inter_add_list(vdata, VPU_CODEC_GET_VERSION, pArgs);
	
	return 0;
}

static int _vdec_result_buf_status(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecBufStatus.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecBufStatus, sizeof(HEVC_RINGBUF_GETINFO_t)))
			return -EFAULT;
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecBufStatus.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecBufStatus, sizeof(VP9_RINGBUF_GETINFO_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecBufStatus.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecBufStatus, sizeof(VDEC_RINGBUF_GETINFO_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_buf_fill(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecBufFill.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecBufFill, sizeof(HEVC_RINGBUF_SETBUF_t)))
			return -EFAULT;
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecBufFill.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecBufFill, sizeof(VP9_RINGBUF_SETBUF_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecBufFill.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecBufFill, sizeof(VDEC_RINGBUF_SETBUF_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_update_wp(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecUpdateWP.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecUpdateWP, sizeof(HEVC_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecUpdateWP.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecUpdateWP, sizeof(VP9_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecUpdateWP.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecUpdateWP, sizeof(VDEC_RINGBUF_SETBUF_PTRONLY_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_seq_header_ring(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		err("%s ::jpu not support this !! \n", vdata->misc->name);return -0x999;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecSeqHeader_Info, sizeof(HEVC_SEQ_HEADER_t)))
			return -EFAULT;
	}
	else
#endif
#if 0//def CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecSeqHeader_Info, sizeof(VP9_SEQ_HEADER_t)))
			return -EFAULT;
	}
	else
#endif
	{
		vdata->gsVpuDecSeqHeader_Info.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecSeqHeader_Info, sizeof(VDEC_SEQ_HEADER_t)))
			return -EFAULT;
	}

	return 0;
}

static int _vdec_result_get_version(vpu_decoder_data *vdata, void *arg)
{
#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		vdata->gsJpuDecVersion.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsJpuDecVersion, sizeof(JPU_GET_VERSION_t)))
			return -EFAULT;

		if((*(vdata->gsJpuDecVersion.pszVersion) == -1) || (*(vdata->gsJpuDecVersion.pszBuildData) == -1))
			vdata->gsJpuDecVersion.result = RETCODE_INVALID_COMMAND;
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		vdata->gsHevcDecVersion.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsHevcDecVersion, sizeof(HEVC_GET_VERSION_t)))
			return -EFAULT;

		if((*(vdata->gsHevcDecVersion.pszVersion) == -1) || (*(vdata->gsHevcDecVersion.pszBuildData) == -1))
			vdata->gsHevcDecVersion.result = RETCODE_INVALID_COMMAND;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vdata->gsVp9DecVersion.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVp9DecVersion, sizeof(VP9_GET_VERSION_t)))
			return -EFAULT;

		if((*(vdata->gsVp9DecVersion.pszVersion) == -1) || (*(vdata->gsVp9DecVersion.pszBuildData) == -1))
			vdata->gsVp9DecVersion.result = RETCODE_INVALID_COMMAND;
	}
	else
#endif
	{
		vdata->gsVpuDecVersion.result = vdata->gsCommDecResult;
		if (copy_to_user(arg, &vdata->gsVpuDecVersion, sizeof(VDEC_GET_VERSION_t)))
			return -EFAULT;

		if((*(vdata->gsVpuDecVersion.pszVersion) == -1) || (*(vdata->gsVpuDecVersion.pszBuildData) == -1))
			vdata->gsVpuDecVersion.result = RETCODE_INVALID_COMMAND;
	}
	return 0;
}
//JS Baek(111220)-E

static void _vdec_force_close(vpu_decoder_data *vdata)
{
	int ret = 0;
	VpuList_t *cmd_list = &vdata->vdec_list[vdata->list_idx];
	vdata->list_idx = (vdata->list_idx+1)%LIST_MAX;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		if(!jmgr_get_close(vdata->gsDecType) && jmgr_get_alive()){
			int max_count = 100;

			jmgr_process_ex(cmd_list, vdata->gsDecType, VPU_DEC_CLOSE, &ret);
			while(!jmgr_get_close(vdata->gsDecType))
			{
				max_count--;
				msleep(20);

				if(max_count <= 0)
					break;
			}
		}
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		if(!hmgr_get_close(vdata->gsDecType) && hmgr_get_alive()){
			int max_count = 100;

			hmgr_process_ex(cmd_list, vdata->gsDecType, VPU_DEC_CLOSE, &ret);
			while(!hmgr_get_close(vdata->gsDecType))
			{
				max_count--;
				msleep(20);

				if(max_count <= 0)
					break;
			}
		}
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		if(!vp9mgr_get_close(vdata->gsDecType) && vp9mgr_get_alive()){
			int max_count = 100;

			vp9mgr_process_ex(cmd_list, vdata->gsDecType, VPU_DEC_CLOSE, &ret);
			while(!vp9mgr_get_close(vdata->gsDecType))
			{
				max_count--;
				msleep(20);

				if(max_count <= 0)
					break;
			}
		}
	}
	else
#endif
	{
		if(!vmgr_get_close(vdata->gsDecType) && vmgr_get_alive()){
			int max_count = 100;		

			vmgr_process_ex(cmd_list, vdata->gsDecType, VPU_DEC_CLOSE, &ret);
			
			while(!vmgr_get_close(vdata->gsDecType))
			{
				max_count--;
				msleep(20);

				if(max_count <= 0)
					break;
			}
		}
	}
}

static int _vdev_init(vpu_decoder_data *vdata, void *arg)
{
	if(vdata->vComm_data.dev_opened > 1){

#ifdef CONFIG_SUPPORT_TCC_JPU
		if(vdata->gsCodecType == STD_MJPG){
			err("Jpu(%s) has been already opened. Maybe there is exceptional stop!! Mgr(%d)/Dec(%d) \n", vdata->misc->name, jmgr_get_alive(), jmgr_get_close(vdata->gsDecType));
		}
		else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
		if(vdata->gsCodecType == STD_HEVC){
			err("Hevc(%s) has been already opened. Maybe there is exceptional stop!! Mgr(%d)/Dec(%d) \n", vdata->misc->name, hmgr_get_alive(), hmgr_get_close(vdata->gsDecType));
		}
		else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
		if(vdata->gsCodecType == STD_VP9){
			err("Vp9(%s) has been already opened. Maybe there is exceptional stop!! Mgr(%d)/Dec(%d) \n", vdata->misc->name, vp9mgr_get_alive(), vp9mgr_get_close(vdata->gsDecType));
		}
		else
#endif
		{		
			err("Vpu(%s) has been already opened. Maybe there is exceptional stop!! Mgr(%d)/Dec(%d) \n", vdata->misc->name, vmgr_get_alive(), vmgr_get_close(vdata->gsDecType));
		}
		_vdec_force_close(vdata);

#ifdef CONFIG_SUPPORT_TCC_JPU
		if(vdata->gsCodecType == STD_MJPG)
			jmgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
		if(vdata->gsCodecType == STD_HEVC)
			hmgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
		if(vdata->gsCodecType == STD_VP9)
			vp9mgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
			vmgr_set_close(vdata->gsDecType, 1, 1);
		
		vdata->vComm_data.dev_opened--;
	}

	vdata->gsCodecType = -1;
	if(copy_from_user(&vdata->gsCodecType, arg, sizeof(int)))
		return -EFAULT;

#ifdef CONFIG_SUPPORT_TCC_JPU
	if(vdata->gsCodecType == STD_MJPG)
	{
		jmgr_set_clkstatus(vdata->gsDecType, vdata->vComm_data.dev_opened);
		
		memset(&vdata->gsJpuDecInit_Info, 0x00, sizeof(JDEC_INIT_t));
		//memset(&vdata->gsJpuDecSeqHeader_Info, 0x00, sizeof(JPU_SEQ_HEADER_t));
		memset(&vdata->gsJpuDecBuffer_Info, 0x00, sizeof(JPU_SET_BUFFER_t));
		memset(&vdata->gsJpuDecInOut_Info, 0x00, sizeof(JPU_DECODE_t));
	}
	else
#endif	
#ifdef CONFIG_SUPPORT_TCC_HEVC
	if(vdata->gsCodecType == STD_HEVC)
	{
		hmgr_set_clkstatus(vdata->gsDecType, vdata->vComm_data.dev_opened);
		
		memset(&vdata->gsHevcDecInit_Info, 0x00, sizeof(HEVC_INIT_t));
		memset(&vdata->gsHevcDecSeqHeader_Info, 0x00, sizeof(HEVC_SEQ_HEADER_t));
		memset(&vdata->gsHevcDecBuffer_Info, 0x00, sizeof(HEVC_SET_BUFFER_t));
		memset(&vdata->gsHevcDecInOut_Info, 0x00, sizeof(HEVC_DECODE_t));
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
	if(vdata->gsCodecType == STD_VP9)
	{
		vp9mgr_set_clkstatus(vdata->gsDecType, vdata->vComm_data.dev_opened);
		
		memset(&vdata->gsVp9DecInit_Info, 0x00, sizeof(VP9_INIT_t));
		memset(&vdata->gsVp9DecSeqHeader_Info, 0x00, sizeof(VP9_SEQ_HEADER_t));
		memset(&vdata->gsVp9DecBuffer_Info, 0x00, sizeof(VP9_SET_BUFFER_t));
		memset(&vdata->gsVp9DecInOut_Info, 0x00, sizeof(VP9_DECODE_t));
	}
	else
#endif
	{
		vmgr_set_clkstatus(vdata->gsDecType, vdata->vComm_data.dev_opened);

		memset(&vdata->gsVpuDecInit_Info, 0x00, sizeof(VDEC_INIT_t));
		memset(&vdata->gsVpuDecSeqHeader_Info, 0x00, sizeof(VDEC_SEQ_HEADER_t));
		memset(&vdata->gsVpuDecBuffer_Info, 0x00, sizeof(VDEC_SET_BUFFER_t));
		memset(&vdata->gsVpuDecInOut_Info, 0x00, sizeof(VDEC_DECODE_t));
	}


	return 0;
}

int vdec_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	vpu_decoder_data *vdata = dev_get_drvdata(misc->parent);
	
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "%s :: mmap: this address is not allowed \n", vdata->misc->name);
		return -EAGAIN;
	}

	vma->vm_page_prot = vmem_get_pgprot(vma->vm_page_prot, vma->vm_pgoff);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk("%s :: mmap :: remap_pfn_range failed\n", vdata->misc->name);
		return -EAGAIN;
	}

	vma->vm_ops 	= NULL;
	vma->vm_flags	|= VM_IO;
	vma->vm_flags   |= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}


unsigned int vdec_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	vpu_decoder_data *vdata = dev_get_drvdata(misc->parent);

	if (vdata == NULL){
		return -EFAULT;
	}

	if (vdata->vComm_data.count > 0)
	{
		vdata->vComm_data.count--;
		return POLLIN;
	}

	poll_wait(filp, &(vdata->vComm_data.wq), wait);

	if (vdata->vComm_data.count > 0)
	{
		vdata->vComm_data.count--;
		return POLLIN;
	}

	return 0;
}

long vdec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	vpu_decoder_data *vdata = dev_get_drvdata(misc->parent);
	
	if( cmd != DEVICE_INITIALIZE && cmd != V_DEC_GENERAL_RESULT
		&& vmgr_get_alive() == 0
#ifdef CONFIG_SUPPORT_TCC_JPU
		&& jmgr_get_alive() == 0
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
		&& hmgr_get_alive() == 0
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
		&& vp9mgr_get_alive() == 0
#endif
	)
	{
		err("This command(0x%x) for %s can't process because Mgr is not alive(v/h/j/v9 = %d/%d/%d/%d) !!!\n", cmd, vdata->misc->name, vmgr_get_alive(), 
	#if defined(CONFIG_SUPPORT_TCC_HEVC)
				hmgr_get_alive(),
	#else
				0,
	#endif
	#if defined(CONFIG_SUPPORT_TCC_JPU)
				jmgr_get_alive(),
	#else
				0,
	#endif
	#if defined(CONFIG_SUPPORT_TCC_VP9)
				vp9mgr_get_alive()
	#else
				0
	#endif
		);

		return -EPERM;;
	}
	
	switch(cmd)
	{
		case DEVICE_INITIALIZE:
			return _vdev_init(vdata, (void*)arg);
			
		case V_DEC_INIT:
			return _vdec_proc_init(vdata, (void*)arg);
			
		case V_DEC_SEQ_HEADER:
			return _vdec_proc_seq_header(vdata, (void*)arg);

		case V_DEC_REG_FRAME_BUFFER:
			return _vdec_proc_reg_framebuffer(vdata, (void*)arg);
			
		case V_DEC_DECODE:
			return _vdec_proc_decode(vdata, (void*)arg);
		
		case V_DEC_BUF_FLAG_CLEAR:
			return _vdec_proc_clear_bufferflag(vdata, (void*)arg);

		case V_DEC_CLOSE:
			return _vdec_proc_exit(vdata, (void*)arg);
		
		case V_DEC_FLUSH_OUTPUT:
			return _vdec_proc_flush(vdata, (void*)arg);

		case V_DEC_SWRESET:
			return _vdec_proc_swreset(vdata);

//JS Baek(111220)-S
		case V_GET_RING_BUFFER_STATUS:
			return _vdec_proc_buf_status(vdata, (void*)arg);

		case V_FILL_RING_BUFFER_AUTO:
			return _vdec_proc_buf_fill(vdata, (void*)arg);

		case V_DEC_UPDATE_RINGBUF_WP:
			return _vdec_proc_update_wp(vdata, (void*)arg);

		case V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY:
			return _vdec_proc_seq_header_ring(vdata, (void*)arg);

		case V_GET_VPU_VERSION:
			return _vdec_proc_get_version(vdata, (void*)arg);

		case V_GET_RING_BUFFER_STATUS_RESULT:
			return _vdec_result_buf_status(vdata, (void*)arg);

		case V_FILL_RING_BUFFER_AUTO_RESULT:
			return _vdec_result_buf_fill(vdata, (void*)arg);

		case V_DEC_UPDATE_RINGBUF_WP_RESULT:
			return _vdec_result_update_wp(vdata, (void*)arg);

		case V_GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY_RESULT:
			return _vdec_result_seq_header_ring(vdata, (void*)arg);

		case V_GET_VPU_VERSION_RESULT:
			return _vdec_result_get_version(vdata, (void*)arg);
//JS Baek(111220)-E

		case V_DEC_GET_INFO:		
		case V_DEC_REG_FRAME_BUFFER2:
			err("%s ::Not implemented ioctl - %d !!!\n", vdata->misc->name, cmd);
		break;

		case V_DEC_ALLOC_MEMORY:
		{
			int ret;
			MEM_ALLOC_INFO_t alloc_info;
			
			if(copy_from_user(&alloc_info, (MEM_ALLOC_INFO_t*)arg, sizeof(MEM_ALLOC_INFO_t)))
				return -EFAULT;
			
			ret = vmem_proc_alloc_memory(vdata->gsCodecType, &alloc_info, vdata->gsDecType);

			if (copy_to_user((MEM_ALLOC_INFO_t*)arg, &alloc_info, sizeof(MEM_ALLOC_INFO_t)))
				return -EFAULT;

			return ret;
		}
		break;

		case V_DEC_FREE_MEMORY:
		{
			return vmem_proc_free_memory(vdata->gsDecType);
		}
		break;

		case VPU_GET_FREEMEM_SIZE:
		{
			int szFreeMem = 0;

			szFreeMem = vmem_get_free_memory(vdata->gsDecType);
			if (copy_to_user((unsigned int*)arg, &szFreeMem, sizeof(szFreeMem)))
				return -EFAULT;
			return 0;
		}
		break;

		case V_DEC_GENERAL_RESULT:
			return _vdec_result_general(vdata, (int *)arg);
				
		case V_DEC_INIT_RESULT:
			return _vdec_result_init(vdata, (void*)arg);
			
		case V_DEC_SEQ_HEADER_RESULT:
			return _vdec_result_seq_header(vdata, (void*)arg);

		case V_DEC_DECODE_RESULT:
			return _vdec_result_decode(vdata, (void*)arg);

		case V_DEC_FLUSH_OUTPUT_RESULT:
			return _vdec_result_flush(vdata, (void*)arg);
			
		default:
			err("[%s] Unsupported ioctl[%d]!!!\n", vdata->misc->name, cmd);      
			break;			
	}

	return 0;
}

int vdec_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	vpu_decoder_data *vdata = dev_get_drvdata(misc->parent);

	dprintk("%s :: open(%d)!! \n", vdata->misc->name, vdata->vComm_data.dev_opened);

	if( vdata->vComm_data.dev_opened == 0 )
		vdata->vComm_data.count = 0;
	vdata->vComm_data.dev_opened++;

	return 0;
}

int vdec_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	vpu_decoder_data *vdata = dev_get_drvdata(misc->parent);	

	detailk("%s :: release In(%d)!! \n", vdata->misc->name, vdata->vComm_data.dev_opened);


	if(vdata->vComm_data.dev_opened > 0)
		vdata->vComm_data.dev_opened--;

	if(vdata->vComm_data.dev_opened == 0)
	{
		vdec_clear_instance(vdata->gsDecType-VPU_DEC);
		_vdec_force_close(vdata);

#ifdef CONFIG_SUPPORT_TCC_JPU
		if(vdata->gsCodecType == STD_MJPG)
			jmgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC
		if(vdata->gsCodecType == STD_HEVC)
			hmgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
		if(vdata->gsCodecType == STD_VP9)
			vp9mgr_set_close(vdata->gsDecType, 1, 1);
		else
#endif
			vmgr_set_close(vdata->gsDecType, 1, 1);
	}

	dprintk("%s :: release Out(%d)!! \n", vdata->misc->name, vdata->vComm_data.dev_opened);
	
	return 0;
}

static struct file_operations vdev_dec_fops = {
	.owner				= THIS_MODULE,
	.open				= vdec_open,
	.release			= vdec_release,
	.mmap				= vdec_mmap,
	.unlocked_ioctl		= vdec_ioctl,
	.poll				= vdec_poll,
};

int vdec_probe(struct platform_device *pdev)
{
	vpu_decoder_data *vdata;
	int ret = -ENODEV;

	if( vmem_get_free_memory(pdev->id) == 0 )
	{
        printk(KERN_WARNING "VPU %s: Couldn't register device because of no-reserved memory.\n", pdev->name);
		return -ENOMEM;
	}

	vdata = kzalloc(sizeof(vpu_decoder_data), GFP_KERNEL);
	if (!vdata)
		return -ENOMEM;

	vdata->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (!vdata->misc){
		ret = -ENOMEM;
		goto err_misc_alloc;
	}

	vdata->misc->minor = MISC_DYNAMIC_MINOR;
	vdata->misc->fops = &vdev_dec_fops;
	vdata->misc->name = pdev->name;
	vdata->misc->parent = &pdev->dev;
	
	vdata->gsDecType = pdev->id;
	memset(&vdata->vComm_data, 0, sizeof(vpu_comm_data_t));
	spin_lock_init(&(vdata->vComm_data.lock));
	init_waitqueue_head(&(vdata->vComm_data.wq));

    if (misc_register(vdata->misc))
    {
        printk(KERN_WARNING "VPU %s: Couldn't register device.\n", pdev->name);
        ret = -EBUSY;
		goto err_misc_register;
    }

	platform_set_drvdata(pdev, vdata);
	pr_info("VPU %s Driver(id:%d) Initialized.\n", pdev->name, pdev->id);

	return 0;

err_misc_register:
    kfree(vdata->misc);
err_misc_alloc:
    kfree(vdata);

	return ret;
}
EXPORT_SYMBOL(vdec_probe);

int vdec_remove(struct platform_device *pdev)
{
	vpu_decoder_data *vdata = (vpu_decoder_data *)platform_get_drvdata(pdev);
	
    misc_deregister(vdata->misc);

	kfree(vdata->misc);
	kfree(vdata);

	return 0;
}
EXPORT_SYMBOL(vdec_remove);
#endif

