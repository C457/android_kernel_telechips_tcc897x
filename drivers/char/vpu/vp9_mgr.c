/****************************************************************************
 *   FileName    : vp9_mgr.c
 *   Description : TCC VP9 h/w block
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

#ifdef CONFIG_SUPPORT_TCC_VP9

#include "vp9_mgr_sys.h"
#include <soc/tcc/pmap.h>
//#include <mach/tcc_fb.h>
#include <linux/of.h>
#include "vpu_buffer.h"
#include "vpu_devices.h"
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#define dprintk(msg...)	//printk( "TCC_VP9_MGR: " msg);
#define detailk(msg...) //printk( "TCC_VP9_MGR: " msg);
#define cmdk(msg...) //printk( "TCC_VP9_MGR [Cmd]: " msg);
#define err(msg...) printk("TCC_VP9_MGR [Err]: "msg);

#define VP9_REGISTER_DUMP
//#define VP9_REGISTER_INIT

//#define FORCED_ERROR //For test purpose!!
#ifdef FORCED_ERROR
#define FORCED_ERR_CNT 300
static int forced_error_count = FORCED_ERR_CNT;
#endif

/////////////////////////////////////////////////////////////////////////////
// Control only once!!
static mgr_data_t vp9mgr_data;
static struct task_struct *kidle_task = NULL;

#ifdef CONFIG_VPU_TIME_MEASUREMENT
extern void do_gettimeofday(struct timeval *tv);
#endif
extern int tcc_vp9_dec( int Op, codec_handle_t* pHandle, void* pParam1, void* pParam2 );
extern int range_is_allowed(unsigned long pfn, unsigned long size);
extern int vmem_get_pgprot(unsigned long ulOldProt, unsigned long ulPageOffset);

int vp9mgr_list_manager(void* args, unsigned int cmd);
/////////////////////////////////////////////////////////////////////////////

int vp9mgr_opened(void)
{
	if(vp9mgr_data.dev_opened == 0)
		return 0;
	return 1;
}
EXPORT_SYMBOL(vp9mgr_opened);

int vp9mgr_get_close(vputype type)
{
	return vp9mgr_data.closed[type];
}

int vp9mgr_get_alive(void)
{
	return vp9mgr_data.dev_opened;
}

int vp9mgr_set_close(vputype type, int value, int bfreemem)
{
	if( vp9mgr_get_close(type) == value ){
		dprintk(" %d was already set into %d. \n", type, value);
		return -1;
	}

	vp9mgr_data.closed[type] = value;
	if(value == 1){
		vp9mgr_data.handle[type] = 0x00;
		vp9mgr_set_clkstatus(type, 0);

		if(bfreemem)
			vmem_proc_free_memory(type);
	}

	return 0;
}

static void _vp9mgr_close_all(int bfreemem)
{
	vp9mgr_set_close(VPU_DEC, 1, bfreemem);
	vp9mgr_set_close(VPU_DEC_EXT, 1, bfreemem);
	vp9mgr_set_close(VPU_DEC_EXT2, 1, bfreemem);
	vp9mgr_set_close(VPU_DEC_EXT3, 1, bfreemem);
}

int	vp9mgr_process_ex(VpuList_t *cmd_list, vputype type, int Op, int *result)
{	
	if(vp9mgr_data.dev_opened == 0)
		return 0;
	
	printk(" \n process_ex %d - 0x%x \n\n", type, Op);

	if(!vp9mgr_get_close(type))
	{
		cmd_list->type 			= type;
		cmd_list->cmd_type		= Op;
		cmd_list->handle 		= vp9mgr_data.handle[type];
		cmd_list->args 			= NULL;
		cmd_list->comm_data 	= NULL;
		cmd_list->vpu_result	= result;
		vp9mgr_list_manager(( void *)cmd_list, LIST_ADD);

		return 1;
	}

	return 1;
}

static int _vp9mgr_internal_handler(void)
{
	int ret, ret_code = RETCODE_SUCCESS;
	int timeout = 200;

	if(vp9mgr_data.check_interrupt_detection)
	{
		if(vp9mgr_data.oper_intr > 0)
		{			
			detailk("Success 1: vp9 operation!! \n");
			ret_code = RETCODE_SUCCESS;
		}
		else
		{
			ret = wait_event_interruptible_timeout(vp9mgr_data.oper_wq, vp9mgr_data.oper_intr > 0, msecs_to_jiffies(timeout));

			if(vp9mgr_data.oper_intr > 0)
			{
				detailk("Success 2: vp9 operation!! \n");
#if defined(FORCED_ERROR)
				if(forced_error_count-- <= 0)
				{
					ret_code = RETCODE_CODEC_EXIT;
					forced_error_count = FORCED_ERR_CNT;
				}
				else
#endif
				ret_code = RETCODE_SUCCESS;
			}
			else
			{
				err("[CMD 0x%x][%d]: vp9 timed_out(ref %d msec) => oper_intr[%d]!! [%d]th frame len %d\n", vp9mgr_data.current_cmd, ret, timeout, vp9mgr_data.oper_intr, vp9mgr_data.nDecode_Cmd, vp9mgr_data.szFrame_Len);
				vetc_dump_reg_all(vp9mgr_data.base_addr, "_vp9mgr_internal_handler timed_out");
				ret_code = RETCODE_CODEC_EXIT;
			}
		}

		vp9mgr_data.oper_intr = 0;
		vp9mgr_status_clear(vp9mgr_data.base_addr);
	}

	return ret_code;
}

static int _vp9mgr_process(vputype type, int cmd, int pHandle, void* args)
{
	int ret = 0;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
	struct timeval t1 , t2;
	int time_gap_ms = 0;
#endif

	vp9mgr_data.check_interrupt_detection = 0;
	vp9mgr_data.current_cmd = cmd;

	if( type <= VPU_DEC_EXT3 )
	{
		if( cmd != VPU_DEC_INIT )
		{
			if(vp9mgr_get_close(type) || vp9mgr_data.handle[type] == 0x00)
				return RETCODE_MULTI_CODEC_EXIT_TIMEOUT;
		}

		if( cmd != VPU_DEC_BUF_FLAG_CLEAR && cmd != VPU_DEC_DECODE ) {
			cmdk("@@@@@@@@@@ Decoder(%d), command: 0x%x \n", type, cmd);
		}

		switch(cmd)
		{
			case VPU_DEC_INIT:
				{
					VP9_INIT_t* arg;

					arg = (VP9_INIT_t *)args;
					vp9mgr_data.handle[type] = 0x00;

					arg->gsVp9DecInit.m_RegBaseVirtualAddr = io_p2v(vp9mgr_data.base_addr);
					arg->gsVp9DecInit.m_Memcpy 			= (void* (*) ( void*, const void*, unsigned int ))memcpy;
					arg->gsVp9DecInit.m_Memset 			= (void* (*) ( void*, int, unsigned int ))memset;
					arg->gsVp9DecInit.m_Interrupt			= (int	(*) ( void ))_vp9mgr_internal_handler;

					vp9mgr_data.check_interrupt_detection = 1;
					dprintk("@@ Dec :: Init In => Reg: 0x%x/0x%x, format : %d, Stream(0x%x/0x%x, 0x%x)\n",
								vp9mgr_data.base_addr, arg->gsVp9DecInit.m_RegBaseVirtualAddr, arg->gsVp9DecInit.m_iBitstreamFormat,
								arg->gsVp9DecInit.m_BitstreamBufAddr[PA], arg->gsVp9DecInit.m_BitstreamBufAddr[VA], arg->gsVp9DecInit.m_iBitstreamBufSize);
					dprintk("@@ Dec :: Init In => optFlag 0x%x, Entropy(0x%x/0x%x, 0x%x), PlayEn: %d, MaxRes: %d\n",
								arg->gsVp9DecInit.m_uiDecOptFlags,
								arg->gsVp9DecInit.m_EntropySaveBuffer[PA], arg->gsVp9DecInit.m_EntropySaveBuffer[VA], arg->gsVp9DecInit.m_iEntropySaveBufferSize,
								arg->gsVp9DecInit.m_iFilePlayEnable, arg->gsVp9DecInit.m_iMaxResolution);

					ret = tcc_vp9_dec(cmd, (void*)(&arg->gsVp9DecHandle), (void*)(&arg->gsVp9DecInit), (void*)NULL);					
					if( ret != RETCODE_SUCCESS ){
						printk("@@ Dec :: Init Done with ret(0x%x)\n", ret);
						if( ret != RETCODE_CODEC_EXIT ){
							vetc_dump_reg_all(vp9mgr_data.base_addr, "init failure");
							//ret = tcc_vp9_dec(cmd, (void*)(&arg->gsVp9DecHandle), (void*)(&arg->gsVp9DecInit), (void*)NULL);
							//printk("@@ Dec :: Init Done with ret(0x%x)\n", ret);
							//vetc_dump_reg_all(vp9mgr_data.base_addr, "init success");
						}
					}
					if( ret != RETCODE_CODEC_EXIT && arg->gsVp9DecHandle != 0) {
						vp9mgr_data.handle[type] = arg->gsVp9DecHandle;
						vp9mgr_set_close(type, 0, 0);
						cmdk("@@ Dec :: vp9mgr_data.handle = 0x%x \n", arg->gsVp9DecHandle);
					}
					else{
						//To free memory!!
						vp9mgr_set_close(type, 0, 0);
						vp9mgr_set_close(type, 1, 1);	
					}
					dprintk("@@ Dec :: Init Done 0x%x, Handle(0x%x) \n", ret, arg->gsVp9DecHandle);

			#ifdef CONFIG_VPU_TIME_MEASUREMENT
					vp9mgr_data.iTime[type].print_out_index = vp9mgr_data.iTime[type].proc_base_cnt = 0;
					vp9mgr_data.iTime[type].accumulated_proc_time = vp9mgr_data.iTime[type].accumulated_frame_cnt = 0;
					vp9mgr_data.iTime[type].proc_time_30frames = 0;
			#endif
				}
				break;

			case VPU_DEC_SEQ_HEADER:
				{
					VP9_SEQ_HEADER_t *arg;
					int iSize;

					arg = (VP9_SEQ_HEADER_t *)args;
					vp9mgr_data.szFrame_Len = iSize = (int)arg->stream_size;
					vp9mgr_data.check_interrupt_detection = 1;
					vp9mgr_data.nDecode_Cmd = 0;
					dprintk("@@ Dec :: SEQ_HEADER in :: size(%d) \n", arg->stream_size);
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)iSize, (void*)(&arg->gsVp9DecInitialInfo));
					printk("@@ Dec :: SEQ_HEADER out 0x%x \n res info. %d - %d - %d, %d - %d - %d \n", ret, arg->gsVp9DecInitialInfo.m_iPicWidth,
									arg->gsVp9DecInitialInfo.m_iVp9PicCrop.m_iCropLeft, arg->gsVp9DecInitialInfo.m_iVp9PicCrop.m_iCropRight,
									arg->gsVp9DecInitialInfo.m_iPicHeight,
									arg->gsVp9DecInitialInfo.m_iVp9PicCrop.m_iCropTop, arg->gsVp9DecInitialInfo.m_iVp9PicCrop.m_iCropBottom);
					dprintk("@@ Dec :: SEQ_HEADER out MinFrameBuffer(%d/0x%x), TempBuffer: 0x%x, Version: %d, Profile: %d, BitDepth: %d, ErrorReason: %d \n", ret,
									arg->gsVp9DecInitialInfo.m_iMinFrameBufferCount, arg->gsVp9DecInitialInfo.m_iMinFrameBufferSize, arg->gsVp9DecInitialInfo.m_iTempBufferSize,
									arg->gsVp9DecInitialInfo.m_iVersion, arg->gsVp9DecInitialInfo.m_iProfile, arg->gsVp9DecInitialInfo.m_iBitDepth, 
									arg->gsVp9DecInitialInfo.m_iReportErrorReason);
				}
				break;
				
			case VPU_DEC_REG_FRAME_BUFFER:
				{
					VP9_SET_BUFFER_t *arg;
					
					arg = (VP9_SET_BUFFER_t *)args;
					dprintk("@@ Dec :: REG_FRAME_BUFFER in :: Frame(0x%x/0x%x, %d), Temp(0x%x/0x%x, 0x%x) \n",
									arg->gsVp9DecBuffer.m_FrameBufferStartAddr[0], arg->gsVp9DecBuffer.m_FrameBufferStartAddr[1], arg->gsVp9DecBuffer.m_iFrameBufferCount,
									arg->gsVp9DecBuffer.m_TempBufferAddr[0], arg->gsVp9DecBuffer.m_TempBufferAddr[1], arg->gsVp9DecBuffer.m_iTempBufferSize);
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(&arg->gsVp9DecBuffer), (void*)NULL);
					dprintk("@@ Dec :: REG_FRAME_BUFFER out 0x%x \n", ret);
				}
				break;
			
			case VPU_DEC_DECODE:
				{						
					VP9_DECODE_t *arg;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t1 );
#endif
					arg = (VP9_DECODE_t *)args;

					vp9mgr_data.szFrame_Len = arg->gsVp9DecInput.m_iBitstreamDataSize;
					dprintk("@@ Dec :: Dec In => Stream(0x%x/0x%x, 0x%x), FrameSearch: %d  \n",
							arg->gsVp9DecInput.m_BitstreamDataAddr[PA], arg->gsVp9DecInput.m_BitstreamDataAddr[VA], arg->gsVp9DecInput.m_iBitstreamDataSize,
							arg->gsVp9DecInput.m_iFrameSearchEnable);

					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(&arg->gsVp9DecInput), (void*)(&arg->gsVp9DecOutput));

					dprintk("@@ Dec :: Dec Out => %d - %d - %d, %d - %d - %d \n", arg->gsVp9DecOutput.m_DecOutInfo.m_iWidth, arg->gsVp9DecOutput.m_DecOutInfo.m_CropInfo.m_iCropLeft, 
									arg->gsVp9DecOutput.m_DecOutInfo.m_CropInfo.m_iCropRight, arg->gsVp9DecOutput.m_DecOutInfo.m_iHeight, 
									arg->gsVp9DecOutput.m_DecOutInfo.m_CropInfo.m_iCropTop, arg->gsVp9DecOutput.m_DecOutInfo.m_CropInfo.m_iCropBottom);

					dprintk("@@ Dec :: Dec Out => ret[%d] !! PicType[%d], OutIdx[%d/%d], OutStatus[%d/%d] \n", ret, arg->gsVp9DecOutput.m_DecOutInfo.m_iPicType,
										arg->gsVp9DecOutput.m_DecOutInfo.m_iDispOutIdx, arg->gsVp9DecOutput.m_DecOutInfo.m_iDecodedIdx,
										arg->gsVp9DecOutput.m_DecOutInfo.m_iOutputStatus, arg->gsVp9DecOutput.m_DecOutInfo.m_iDecodingStatus);
					dprintk("@@ Dec :: Dec Out => dec_Idx(%d), 0x%x 0x%x 0x%x / 0x%x 0x%x 0x%x \n", arg->gsVp9DecOutput.m_DecOutInfo.m_iDispOutIdx, 
								(unsigned int)arg->gsVp9DecOutput.m_pDispOut[PA][0], (unsigned int)arg->gsVp9DecOutput.m_pDispOut[PA][1], (unsigned int)arg->gsVp9DecOutput.m_pDispOut[PA][2], 
								(unsigned int)arg->gsVp9DecOutput.m_pDispOut[VA][0], (unsigned int)arg->gsVp9DecOutput.m_pDispOut[VA][1], (unsigned int)arg->gsVp9DecOutput.m_pDispOut[VA][2]);
					dprintk("@@ Dec :: Dec Out => disp_Idx(%d), 0x%x 0x%x 0x%x / 0x%x 0x%x 0x%x \n", arg->gsVp9DecOutput.m_DecOutInfo.m_iDecodedIdx, 
								(unsigned int)arg->gsVp9DecOutput.m_pCurrOut[PA][0], (unsigned int)arg->gsVp9DecOutput.m_pCurrOut[PA][1], (unsigned int)arg->gsVp9DecOutput.m_pCurrOut[PA][2], 
								(unsigned int)arg->gsVp9DecOutput.m_pCurrOut[VA][0], (unsigned int)arg->gsVp9DecOutput.m_pCurrOut[VA][1], (unsigned int)arg->gsVp9DecOutput.m_pCurrOut[VA][2]);

					dprintk("@@ Dec :: Dec Out => BitDepth (%dx%d), Stride(%d) \n", arg->gsVp9DecOutput.m_DecOutInfo.m_iBitDepthLuma,
								arg->gsVp9DecOutput.m_DecOutInfo.m_iBitDepthChroma, arg->gsVp9DecOutput.m_DecOutInfo.m_iPicStride);

					dprintk("@@ Dec :: Dec Out => Scale(%d/%d), Stride(%d), 0x%x 0x%x 0x%x / 0x%x 0x%x 0x%x \n", arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_iDscaleWidth,
								arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_iDscaleHeight, arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_iDscaleStride,
								(unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[PA][0], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[PA][1], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[PA][2], 
								(unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[VA][0], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[VA][1], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_DscaleInfo.m_pDscaleDispOut[VA][2]);

					dprintk("@@ Dec :: Dec Out => Compressed  Luma(0x%x/0x%x) / Chroma(0x%x/0x%x) \n",
								(unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_CompressionTableLuma[PA], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_CompressionTableLuma[PA], 
								(unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_CompressionTableChroma[VA], (unsigned int)arg->gsVp9DecOutput.m_DecOutInfo.m_CompressionTableChroma[VA]);

					if(arg->gsVp9DecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL){
						 err("Buffer full\n");
						//vetc_dump_reg_all(vp9mgr_data.base_addr, "Buffer full");
					}
					vp9mgr_data.nDecode_Cmd++;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t2 );
#endif
				}
				break;
				
			case VPU_DEC_BUF_FLAG_CLEAR:
				{
					int *arg;

					arg = (int *)args;
					dprintk("@@ Dec :: DispIdx Clear %d \n", *arg);
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(arg), (void*)NULL);
				}
				break;

			case VPU_DEC_FLUSH_OUTPUT:
				{						
					VP9_DECODE_t *arg;

					arg = (VP9_DECODE_t *)args;
					printk("@@ Dec :: VP9_DEC_FLUSH_OUTPUT !! \n");
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(&arg->gsVp9DecInput), (void*)(&arg->gsVp9DecOutput));
				}
				break;

			case VPU_DEC_CLOSE:
				{
					VP9_DECODE_t *arg;

					arg = (VP9_DECODE_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)NULL, (void*)NULL/*(&arg->gsVp9DecOutput)*/);					
					dprintk("@@ Dec :: VP9_DEC_CLOSED !! \n");
                    vp9mgr_set_close(type, 1, 1);
				}
				break;

/*
			case GET_RING_BUFFER_STATUS:
				{
					VP9_RINGBUF_GETINFO_t *arg;

					arg = (VP9_RINGBUF_GETINFO_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;

					ret = tcc_vp9_dec(cmd, &pHandle, (void*)NULL, (void*)(&arg->gsVp9DecRingStatus));
				}
				break;
			case FILL_RING_BUFFER_AUTO:
				{
					VP9_RINGBUF_SETBUF_t *arg;

					arg = (VP9_RINGBUF_SETBUF_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(&arg->gsVp9DecInit), (void*)(&arg->gsVp9DecRingFeed));
					dprintk("@@ Dec :: ReadPTR : 0x%08x, WritePTR : 0x%08x\n", *(volatile unsigned int *)(io_p2v(vp9mgr_data.base_addr) + 0x120), *(volatile unsigned int *)(io_p2v(vp9mgr_data.base_addr) + 0x124));
				}
				break;

			case VPU_UPDATE_WRITE_BUFFER_PTR:
				{
					VP9_RINGBUF_SETBUF_PTRONLY_t *arg;

					arg = (VP9_RINGBUF_SETBUF_PTRONLY_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(arg->iCopiedSize), (void*)(arg->iFlushBuf));
				}
				break;

			case GET_INITIAL_INFO_FOR_STREAMING_MODE_ONLY:
				{
					VP9_SEQ_HEADER_t *arg;

					arg = (VP9_SEQ_HEADER_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, (void*)(&arg->gsVp9DecInitialInfo), NULL);
				}
				break;
*/

			case VPU_CODEC_GET_VERSION:
				{
					VP9_GET_VERSION_t *arg;

					arg = (VP9_GET_VERSION_t *)args;
					vp9mgr_data.check_interrupt_detection = 1;
					ret = tcc_vp9_dec(cmd, &pHandle, arg->pszVersion, arg->pszBuildData);
					dprintk("@@ Dec :: version : %s, build : %s\n", arg->pszVersion, arg->pszBuildData);
				}
				break;

			case VPU_DEC_SWRESET:
				{
					ret = tcc_vp9_dec(cmd, &pHandle, NULL, NULL);
				}
				break;

			default:
				{
					err("@@ Dec :: not supported command(0x%x) \n", cmd);
					return 0x999;
				}
				break;
		}	
	}
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
	else
	{
		err("@@ Enc :: Encoder for VP9 do not support. command(0x%x) \n", cmd);
		return 0x999;
	}
#endif

#ifdef CONFIG_VPU_TIME_MEASUREMENT
	time_gap_ms = vetc_GetTimediff_ms(t1, t2);

	if( cmd == VPU_DEC_DECODE || cmd == VPU_ENC_ENCODE ) {
		vp9mgr_data.iTime[type].accumulated_frame_cnt++;
		vp9mgr_data.iTime[type].proc_time[vp9mgr_data.iTime[type].proc_base_cnt] = time_gap_ms;
		vp9mgr_data.iTime[type].proc_time_30frames += time_gap_ms;
		vp9mgr_data.iTime[type].accumulated_proc_time += time_gap_ms;
		if(vp9mgr_data.iTime[type].proc_base_cnt != 0 && vp9mgr_data.iTime[type].proc_base_cnt % 29 == 0)
		{
			printk("VP9[%d] :: Dec[%4d] time %2d.%2d / %2d.%2d ms: %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d \n",
						type, 
						vp9mgr_data.iTime[type].print_out_index,
						vp9mgr_data.iTime[type].proc_time_30frames/30,
						((vp9mgr_data.iTime[type].proc_time_30frames%30)*100)/30,
						vp9mgr_data.iTime[type].accumulated_proc_time/vp9mgr_data.iTime[type].accumulated_frame_cnt,
						((vp9mgr_data.iTime[type].accumulated_proc_time%vp9mgr_data.iTime[type].accumulated_frame_cnt)*100)/vp9mgr_data.iTime[type].accumulated_frame_cnt,
						vp9mgr_data.iTime[type].proc_time[0], vp9mgr_data.iTime[type].proc_time[1], vp9mgr_data.iTime[type].proc_time[2], vp9mgr_data.iTime[type].proc_time[3], vp9mgr_data.iTime[type].proc_time[4],
						vp9mgr_data.iTime[type].proc_time[5], vp9mgr_data.iTime[type].proc_time[6], vp9mgr_data.iTime[type].proc_time[7], vp9mgr_data.iTime[type].proc_time[8], vp9mgr_data.iTime[type].proc_time[9],
						vp9mgr_data.iTime[type].proc_time[10], vp9mgr_data.iTime[type].proc_time[11], vp9mgr_data.iTime[type].proc_time[12], vp9mgr_data.iTime[type].proc_time[13], vp9mgr_data.iTime[type].proc_time[14],
						vp9mgr_data.iTime[type].proc_time[15], vp9mgr_data.iTime[type].proc_time[16], vp9mgr_data.iTime[type].proc_time[17], vp9mgr_data.iTime[type].proc_time[18], vp9mgr_data.iTime[type].proc_time[19],
						vp9mgr_data.iTime[type].proc_time[20], vp9mgr_data.iTime[type].proc_time[21], vp9mgr_data.iTime[type].proc_time[22], vp9mgr_data.iTime[type].proc_time[23], vp9mgr_data.iTime[type].proc_time[24],
						vp9mgr_data.iTime[type].proc_time[25], vp9mgr_data.iTime[type].proc_time[26], vp9mgr_data.iTime[type].proc_time[27], vp9mgr_data.iTime[type].proc_time[28], vp9mgr_data.iTime[type].proc_time[29]);
			vp9mgr_data.iTime[type].proc_base_cnt = 0;
			vp9mgr_data.iTime[type].proc_time_30frames = 0;
			vp9mgr_data.iTime[type].print_out_index++;
		}
		else{
			vp9mgr_data.iTime[type].proc_base_cnt++;
		}
	}
#endif

	return ret;
}

static long _vp9mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	CONTENTS_INFO info;
	OPENED_sINFO open_info;

	mutex_lock(&vp9mgr_data.comm_data.io_mutex);
	
	switch(cmd)
	{
		case VPU_SET_CLK:
			if(copy_from_user(&info,(CONTENTS_INFO*)arg,sizeof(info)))
				ret = -EFAULT;
			else{
	#if defined(CONFIG_CLOCK_TABLE)
				{
					int clk_idx;

					clk_idx = vp9mgr_calcClock(info);								
					ret = vp9mgr_setClock(clk_idx, info, vp9mgr_data.dev_opened, vp9mgr_data.clk_limitation);
				}
	#endif
			}
			break;
			
		case VPU_GET_FREEMEM_SIZE:
			{
				unsigned int type;
				unsigned int freemem_sz;

				if(copy_from_user(&type, (unsigned int*)arg, sizeof(unsigned int)))
					ret = -EFAULT;
				else
				{
					if(type > VPU_MAX)
						type = VPU_DEC;
	 				freemem_sz = vmem_get_freemem_size(type);
					if (copy_to_user((unsigned int*)arg, &freemem_sz, sizeof(unsigned int)))
						ret = -EFAULT;
				}
			}
			break;

		case VPU_HW_RESET:
			break;

		case VPU_SET_MEM_ALLOC_MODE:
			if(copy_from_user(&open_info,(OPENED_sINFO*)arg, sizeof(OPENED_sINFO)))
				ret = -EFAULT;
			else
			{
				if(open_info.opened_cnt	!= 0)
					vmem_set_only_decode_mode(open_info.type);
				ret = 0;
			}
			break;

		case VPU_CHECK_CODEC_STATUS:
			{
				if (copy_to_user((int*)arg, vp9mgr_data.closed, sizeof(vp9mgr_data.closed)))
					ret = -EFAULT;
				else
					ret = 0;
			}
			break;

		case VPU_GET_INSTANCE_IDX:
			{
				INSTANCE_INFO iInst;

				if (copy_from_user(&iInst, (int*)arg, sizeof(INSTANCE_INFO)))
					ret = -EFAULT;
				else
				{
					if( iInst.type == VPU_ENC )
						venc_get_instance(&iInst.nInstance);
					else
						vdec_get_instance(&iInst.nInstance);

					if (copy_to_user((int*)arg, &iInst, sizeof(INSTANCE_INFO)))
						ret = -EFAULT;
				}
			}
			break;

		case VPU_CLEAR_INSTANCE_IDX:
			{
				INSTANCE_INFO iInst;

				if (copy_from_user(&iInst, (int*)arg, sizeof(INSTANCE_INFO)))
					ret = -EFAULT;
				else{
					if( iInst.type == VPU_ENC )
						venc_clear_instance(iInst.nInstance);
					else
						vdec_clear_instance(iInst.nInstance);
				}
			}
			break;

		default:
			err("Unsupported ioctl[%d]!!!\n", cmd);
			ret = -EINVAL;
			break;			
	}

	mutex_unlock(&vp9mgr_data.comm_data.io_mutex);

	return ret;
}

static irqreturn_t _vp9mgr_isr_handler(int irq, void *dev_id)
{
	unsigned long flags;
	
	detailk("_vp9mgr_isr_handler \n");
	
	spin_lock_irqsave(&(vp9mgr_data.oper_lock), flags);
	vp9mgr_data.oper_intr++;
	spin_unlock_irqrestore(&(vp9mgr_data.oper_lock), flags);
	
	wake_up_interruptible(&(vp9mgr_data.oper_wq));
	
	return IRQ_HANDLED;
}

static int _vp9mgr_proc_exit_by_external(struct VpuList *list, int *result, unsigned int type)
{
	if(!vp9mgr_get_close(type) && vp9mgr_data.handle[type] != 0x00)
	{
		list->type = type;
		if( type >= VPU_ENC )
			list->cmd_type = VPU_ENC_CLOSE;
		else
			list->cmd_type = VPU_DEC_CLOSE;
		list->handle 	= vp9mgr_data.handle[type];
		list->args 		= NULL;
		list->comm_data = NULL;
		list->vpu_result = result;

		printk("_vp9mgr_proc_exit_by_external for %d!! \n", type);
		vp9mgr_list_manager(( void *)list, LIST_ADD);

		return 1;
	}

	return 0;
}

static int _vp9mgr_open(struct inode *inode, struct file *filp)
{
	if (!vp9mgr_data.irq_reged) {
		err("not registered vp9-mgr-irq \n");
	}

	dprintk("_vp9mgr_open In!! %d'th \n", vp9mgr_data.dev_opened);

	vp9mgr_enable_clock();

	if(vp9mgr_data.dev_opened == 0)
	{
		vp9mgr_readyClock(vp9mgr_data.dev_opened, vp9mgr_data.base_addr);

#ifdef FORCED_ERROR
		forced_error_count = FORCED_ERR_CNT;
#endif
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
		vp9mgr_data.only_decmode = 0;
#else
		vp9mgr_data.only_decmode = 1;
#endif
		vp9mgr_data.clk_limitation = 1;
		//vp9mgr_hw_reset();
		vp9mgr_data.cmd_processing = 0;

		vp9mgr_enable_irq(vp9mgr_data.irq);
		vetc_reg_init(vp9mgr_data.base_addr);
		vmem_reinit();
	}
	vp9mgr_data.dev_opened++;	

	vp9mgr_log_clkstatus(0, vp9mgr_data.dev_opened, vp9mgr_data.nOpened_Count);

	filp->private_data = &vp9mgr_data;
	
	return 0;
}

static void _vp9mgr_wait_process(int wait_ms)
{
	int max_count = wait_ms/20;

	//wait!! in case exceptional processing. ex). sdcard out!!
	while(vp9mgr_data.cmd_processing)
	{
		max_count--;
		msleep(20);

		if(max_count <= 0)
		{
			err("cmd_processing(cmd %d) didn't finish!! \n", vp9mgr_data.current_cmd);
			break;
		}
	}
}

static int _vp9mgr_external_all_close(int wait_ms)
{
	int type = 0;
	int max_count = 0;
	int ret;

	for(type = 0; type < VP9_MAX; type++)
	{
		if(_vp9mgr_proc_exit_by_external(&vp9mgr_data.vList[type], &ret, type))
		{
			max_count = wait_ms/10;
			while(!vp9mgr_get_close(type))
			{
				max_count--;
				msleep(10);
			}
		}
	}

	return 0;
}

static int _vp9mgr_release(struct inode *inode, struct file *filp)
{
	dprintk("_vp9mgr_release In!! %d'th \n", vp9mgr_data.dev_opened);

	_vp9mgr_wait_process(2000);

	if(vp9mgr_data.dev_opened > 0)
		vp9mgr_data.dev_opened--;
	if(vp9mgr_data.dev_opened == 0)
	{
//////////////////////////////////////
		int type = 0, alive_cnt = 0;

#if 1 // To close whole vp9 instance when being killed process opened this.
		vp9mgr_data.external_proc = 1;
		_vp9mgr_external_all_close(200);
		_vp9mgr_wait_process(2000);
		vp9mgr_data.external_proc = 0;
#endif

		for(type=0; type<VP9_MAX; type++) {
			if( vp9mgr_data.closed[type] == 0 ){
				alive_cnt++;
			}
		}

		if( alive_cnt )
		{
			// clear instances of vp9 by force.
			//TCC_VPU_DEC( 0x40, (void*)NULL, (void*)NULL, (void*)NULL);
			printk("VP9 might be cleared by force. \n");
		}

//////////////////////////////////////
		vp9mgr_data.oper_intr = 0;
		vp9mgr_data.cmd_processing = 0;

		_vp9mgr_close_all(1);
//////////////////////////////////////

		vp9mgr_disable_irq(vp9mgr_data.irq);
		vp9mgr_BusPrioritySetting(BUS_FOR_NORMAL, 0);
	}

	vp9mgr_disable_clock();
	vp9mgr_clearClock(vp9mgr_data.dev_opened);

	vp9mgr_data.nOpened_Count++;
	vp9mgr_log_clkstatus(1, vp9mgr_data.dev_opened, vp9mgr_data.nOpened_Count);
	
	return 0;
}

int vp9mgr_list_manager(void* args, unsigned int cmd)
{
	int ret;

	mutex_lock(&vp9mgr_data.comm_data.list_mutex);

	{
		unsigned int *data_address;
		VpuList_t* data;

		ret = 0;
		data_address = (unsigned int *)args;

	/*
		data = (VpuList_t *)args;

		if(cmd == LIST_ADD)
			printk("cmd = %d - 0x%x \n", cmd, data->cmd_type);
		else
			printk("cmd = %d \n", cmd);
	*/
		switch(cmd)
		{
			case LIST_ADD:
				{
					if(!args)
					{
						err("data is null \n");
						goto Error;
					}

					data = (VpuList_t*)args;
					*(data->vpu_result) |= RET1;
					list_add_tail(&data->list, &vp9mgr_data.comm_data.main_list);vp9mgr_data.cmd_queued++;
					vp9mgr_data.comm_data.thread_intr++;
					wake_up_interruptible(&(vp9mgr_data.comm_data.thread_wq));
				}
				break;

			case LIST_DEL:
				data = (VpuList_t*)args;
				list_del(&data->list);vp9mgr_data.cmd_queued--;
				break;

			case LIST_IS_EMPTY:
				ret = list_empty(&vp9mgr_data.comm_data.main_list);
				break;

			case LIST_GET_ENTRY:
				if(!args)
				{
					err("data is null \n");
					goto Error;
				}
				*data_address = (unsigned int)list_first_entry(&vp9mgr_data.comm_data.main_list, VpuList_t, list);
				break;
		}
	}

Error:	
	mutex_unlock(&vp9mgr_data.comm_data.list_mutex);

	return ret;
}

//////////////////////////////////////////////////////////////////////////
// VP9 Thread!!
static int _vp9mgr_operation(void)
{
	int oper_finished;
	unsigned int data = 0;
	VpuList_t *oper_data = NULL;
    
	while(!vp9mgr_list_manager(NULL, LIST_IS_EMPTY))
	{
		vp9mgr_data.cmd_processing = 1;

		oper_finished = 1;
		dprintk("_vp9mgr_operation :: not empty cmd_queued(%d) \n", vp9mgr_data.cmd_queued);

		vp9mgr_list_manager(&data, LIST_GET_ENTRY);
		if(!data) {
			err("data is null \n");
			vp9mgr_data.cmd_processing = 0;
			return 0;
		}
		oper_data = (VpuList_t *)data;
		*(oper_data->vpu_result) |= RET2;        

		dprintk("_vp9mgr_operation [%d] :: cmd = 0x%x, vp9mgr_data.cmd_queued(%d) \n", oper_data->type, oper_data->cmd_type, vp9mgr_data.cmd_queued);

		if(oper_data->type < VP9_MAX && oper_data != NULL /*&& oper_data->comm_data != NULL*/)
		{
			*(oper_data->vpu_result) |= RET3;

			*(oper_data->vpu_result) = _vp9mgr_process(oper_data->type, oper_data->cmd_type, oper_data->handle, oper_data->args);
			oper_finished = 1;
			if(*(oper_data->vpu_result) != RETCODE_SUCCESS)
			{
				if( *(oper_data->vpu_result) != RETCODE_INSUFFICIENT_BITSTREAM && *(oper_data->vpu_result) != RETCODE_INSUFFICIENT_BITSTREAM_BUF)
					err("vp9mgr_out[0x%x] :: type = %d, vp9mgr_data.handle = 0x%x, cmd = 0x%x, frame_len %d \n", *(oper_data->vpu_result), oper_data->type, oper_data->handle, oper_data->cmd_type, vp9mgr_data.szFrame_Len);

				if(*(oper_data->vpu_result) == RETCODE_CODEC_EXIT)
				{
					int opened_count = vp9mgr_data.dev_opened;

					_vp9mgr_close_all(1);

			#if 1
					while(opened_count)
					{
						vp9mgr_disable_clock();
						if(opened_count > 0)
							opened_count--;
					}

					//msleep(1);
					opened_count = vp9mgr_data.dev_opened;
					while(opened_count)
					{
						vp9mgr_enable_clock();
						if(opened_count > 0)
							opened_count--;
					}
			#else
					vp9mgr_hw_reset();
			#endif
				}
			}
		}
		else
		{
			printk("_vp9mgr_operation :: missed info or unknown command => type = 0x%x, cmd = 0x%x,  \n", oper_data->type, oper_data->cmd_type);
			*(oper_data->vpu_result) = RETCODE_FAILURE;
			oper_finished = 0;
		}
		
		if(oper_finished)
		{
			if(oper_data->comm_data != NULL && vp9mgr_data.dev_opened != 0)
			{
				unsigned int old_count = oper_data->comm_data->count;
				//unsigned long flags;
				//spin_lock_irqsave(&(oper_data->comm_data->lock), flags);
				oper_data->comm_data->count += 1;
				if(oper_data->comm_data->count != 1){
					dprintk("poll wakeup count = %d (%d, 0x%x) :: type(0x%x) cmd(0x%x) \n",oper_data->comm_data->count, old_count, old_count, oper_data->type, oper_data->cmd_type);
				}
				//spin_unlock_irqrestore(&(oper_data->comm_data->lock), flags);
				wake_up_interruptible(&(oper_data->comm_data->wq));
			}
			else{
				err("Error: abnormal exception or external command was processed!! 0x%x - %d\n", (unsigned int)oper_data->comm_data, vp9mgr_data.dev_opened);
			}
		}
		else{
			err("Error: abnormal exception 2!! 0x%x - %d\n",(unsigned int)oper_data->comm_data, vp9mgr_data.dev_opened);
		}

        vp9mgr_list_manager((void*)oper_data, LIST_DEL);

        vp9mgr_data.cmd_processing = 0;
	}

	return 0;
}

static int _vp9mgr_thread(void *kthread)
{
	dprintk("_vp9mgr_thread for dec is running. \n");

	do 
	{
//		detailk("_vp9mgr_thread wait_sleep \n");

		if(vp9mgr_list_manager(NULL, LIST_IS_EMPTY))
		{
			vp9mgr_data.cmd_processing = 0;

			//wait_event_interruptible(vp9mgr_data.comm_data.thread_wq, vp9mgr_data.comm_data.thread_intr > 0);
			wait_event_interruptible_timeout(vp9mgr_data.comm_data.thread_wq, vp9mgr_data.comm_data.thread_intr > 0, msecs_to_jiffies(50));
			vp9mgr_data.comm_data.thread_intr = 0;
		}
		else
		{
			if(vp9mgr_data.dev_opened || vp9mgr_data.external_proc){
				_vp9mgr_operation();
			}
			else{
				unsigned int data = 0;
				VpuList_t *oper_data = NULL;

				printk("DEL for empty \n");

				vp9mgr_list_manager(&data, LIST_GET_ENTRY);
				oper_data = (VpuList_t *)data;
				if(oper_data)
					vp9mgr_list_manager((void*)oper_data, LIST_DEL);
			}
		}
		
	} while (!kthread_should_stop());

	return 0;
}

//////////////////////////////////////////////////////////////////////////
static int _vp9mgr_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "_vp9mgr_mmap: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = vmem_get_pgprot(vma->vm_page_prot, vma->vm_pgoff);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk("_vp9mgr_mmap :: remap_pfn_range failed\n");
		return -EAGAIN;
	}

	vma->vm_ops 	= NULL;
	vma->vm_flags	|= VM_IO;
	vma->vm_flags   |= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static struct file_operations _vp9mgr_fops = {
	.open				= _vp9mgr_open,
	.release			= _vp9mgr_release,
	.mmap				= _vp9mgr_mmap,
	.unlocked_ioctl		= _vp9mgr_ioctl,
};

static struct miscdevice _vp9mgr_misc_device =
{
    MISC_DYNAMIC_MINOR,
    VP9MGR_NAME,
    &_vp9mgr_fops,
};

int vp9mgr_probe(struct platform_device *pdev)
{
	int ret;
	int type = 0;
	unsigned long int_flags;
	struct resource *regs = NULL;

	if (pdev->dev.of_node == NULL) {
		return - -ENODEV;
	}

	dprintk("vp9mgr initializing!! \n");
	memset(&vp9mgr_data, 0, sizeof(mgr_data_t));
	for(type=0; type<VP9_MAX; type++) {
		vp9mgr_data.closed[type] = 1;
	}

	vp9mgr_init_variable();

	vp9mgr_data.irq = platform_get_irq(pdev, 0);
	vp9mgr_data.nOpened_Count = 0;
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vp9mgr_data.base_addr = regs->start;
	dprintk("============> VP9 base address [0x%x], irq num [%d] \n", vp9mgr_data.base_addr, vp9mgr_data.irq - 32);

	vp9mgr_get_clock(pdev->dev.of_node);

	spin_lock_init(&(vp9mgr_data.oper_lock));
//	spin_lock_init(&(vp9mgr_data.comm_data.lock));
	
	init_waitqueue_head(&vp9mgr_data.comm_data.thread_wq);
	init_waitqueue_head(&vp9mgr_data.oper_wq);

	mutex_init(&vp9mgr_data.comm_data.list_mutex);
	mutex_init(&(vp9mgr_data.comm_data.io_mutex));

    INIT_LIST_HEAD(&vp9mgr_data.comm_data.main_list);
	INIT_LIST_HEAD(&vp9mgr_data.comm_data.wait_list);

	vp9mgr_init_interrupt();
	int_flags = vp9mgr_get_int_flags();
    ret = vp9mgr_request_irq(vp9mgr_data.irq, _vp9mgr_isr_handler, int_flags, VP9MGR_NAME, &vp9mgr_data);
	if (ret) {
		err("to aquire vp9-dec-irq \n");
	}
	vp9mgr_data.irq_reged = 1;
	vp9mgr_disable_irq(vp9mgr_data.irq);

	kidle_task = kthread_run(_vp9mgr_thread, NULL, "kvp9mgr_operd");	
	if( IS_ERR(kidle_task) )
	{		
		err("unable to create thread!! \n");
		kidle_task = NULL;
		return -1;	
	}
	dprintk("success :: thread created!! \n");	

	_vp9mgr_close_all(1);

    if (misc_register(&_vp9mgr_misc_device))
    {
        printk(KERN_WARNING "VP9 Manager: Couldn't register device.\n");
        return -EBUSY;
    }

	return 0;
}
EXPORT_SYMBOL(vp9mgr_probe);

int vp9mgr_remove(struct platform_device *pdev)
{
    misc_deregister(&_vp9mgr_misc_device);

	if( kidle_task ){
		kthread_stop(kidle_task);
		kidle_task = NULL;
	}

	if( vp9mgr_data.irq_reged ){
		vp9mgr_free_irq(vp9mgr_data.irq, &vp9mgr_data);
		vp9mgr_data.irq_reged = 0;
	}

	vp9mgr_put_clock();

	printk("success :: vp9mgr thread stopped!! \n");

	return 0;
}
EXPORT_SYMBOL(vp9mgr_remove);

#if defined(CONFIG_PM)
int vp9mgr_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i, open_count = 0; 

	if(vp9mgr_data.dev_opened != 0)
	{
		printk(" \n vp9: suspend In DEC(%d/%d/%d/%d) \n", vp9mgr_get_close(VPU_DEC), vp9mgr_get_close(VPU_DEC_EXT), vp9mgr_get_close(VPU_DEC_EXT2), vp9mgr_get_close(VPU_DEC_EXT3));

		_vp9mgr_external_all_close(200);

		open_count = vp9mgr_data.dev_opened;
		for(i=0; i<open_count; i++) {
			vp9mgr_disable_clock();
		}
		printk("vp9: suspend Out DEC(%d/%d/%d/%d) \n\n", vp9mgr_get_close(VPU_DEC), vp9mgr_get_close(VPU_DEC_EXT), vp9mgr_get_close(VPU_DEC_EXT2), vp9mgr_get_close(VPU_DEC_EXT3));
	}

	return 0;
}
EXPORT_SYMBOL(vp9mgr_suspend);

int vp9mgr_resume(struct platform_device *pdev)
{
	int i, open_count = 0; 

	if(vp9mgr_data.dev_opened != 0){

		open_count = vp9mgr_data.dev_opened;

		for(i=0; i<open_count; i++) {
			vp9mgr_enable_clock();
		}
		printk("\n vp9: resume \n\n");
	}

	return 0;
}
EXPORT_SYMBOL(vp9mgr_resume);
#endif

MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC VP9 manager");
MODULE_LICENSE("GPL");

#endif
