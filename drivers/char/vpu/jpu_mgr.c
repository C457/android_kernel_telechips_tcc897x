/****************************************************************************
 *   FileName    : jpu_mgr.c
 *   Description : TCC JPU h/w block
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

#ifdef CONFIG_SUPPORT_TCC_JPU

#include "jpu_mgr_sys.h"
#include <linux/of.h>
#include "vpu_buffer.h"
#include "vpu_devices.h"
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
//#include <mach/tcc_fb.h>
#else
//#include <video/tcc/tcc_fb.h>
#endif

//#define JPU_DEBUG
#ifdef JPU_DEBUG
#define dprintk(msg...)	printk( "TCC_JPU_MGR: " msg);
#define detailk(msg...)	printk( "TCC_JPU_MGR: " msg);
#define cmdk(msg...)	printk( "TCC_JPU_MGR [Cmd]: " msg);
#else
#define dprintk(msg...)
#define detailk(msg...)
#define cmdk(msg...)
#endif
#define err(msg...) printk("TCC_JPU_MGR [Err]: "msg);

#define JPU_REGISTER_DUMP
//#define JPU_REGISTER_INIT

//#define FORCED_ERROR //For test purpose!!
#ifdef FORCED_ERROR
#define FORCED_ERR_CNT 300
static int forced_error_count = FORCED_ERR_CNT;
#endif

/////////////////////////////////////////////////////////////////////////////
// Control only once!!
static mgr_data_t jmgr_data;
static struct task_struct *kidle_task = NULL;

#ifdef CONFIG_VPU_TIME_MEASUREMENT
extern void do_gettimeofday(struct timeval *tv);
#endif
extern int tcc_jpu_dec( int Op, codec_handle_t* pHandle, void* pParam1, void* pParam2 );
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
extern int tcc_jpu_enc( int Op, codec_handle_t* pHandle, void* pParam1, void* pParam2 );
#endif
extern int range_is_allowed(unsigned long pfn, unsigned long size);
extern int vmem_get_pgprot(unsigned long ulOldProt, unsigned long ulPageOffset);

int jmgr_list_manager(void* args, unsigned int cmd);
/////////////////////////////////////////////////////////////////////////////

int jmgr_opened(void)
{
	if(jmgr_data.dev_opened == 0)
		return 0;
	return 1;
}
EXPORT_SYMBOL(jmgr_opened);

int jmgr_get_close(vputype type)
{
	return jmgr_data.closed[type];
}

int jmgr_get_alive(void)
{
	return jmgr_data.dev_opened;
}

int jmgr_set_close(vputype type, int value, int bfreemem)
{
	if( jmgr_get_close(type) == value ){
		dprintk(" %d was already set into %d. \n", type, value);
		return -1;
	}

	jmgr_data.closed[type] = value;
	if(value == 1){
		jmgr_data.handle[type] = 0x00;
		jmgr_set_clkstatus(type, 0);

		if(bfreemem)
			vmem_proc_free_memory(type);
	}

	return 0;
}

static void _jmgr_close_all(int bfreemem)
{
	jmgr_set_close(VPU_DEC, 1, bfreemem);
	jmgr_set_close(VPU_DEC_EXT, 1, bfreemem);
	jmgr_set_close(VPU_DEC_EXT2, 1, bfreemem);
	jmgr_set_close(VPU_DEC_EXT3, 1, bfreemem);
}

int	jmgr_process_ex(VpuList_t *cmd_list, vputype type, int Op, int *result)
{	
	if(jmgr_data.dev_opened == 0)
		return 0;
	
	printk(" \n process_ex %d - 0x%x \n\n", type, Op);

	if(!jmgr_get_close(type))
	{
		cmd_list->type 			= type;
		cmd_list->cmd_type		= Op;
		cmd_list->handle 		= jmgr_data.handle[type];
		cmd_list->args 			= NULL;
		cmd_list->comm_data 	= NULL;
		cmd_list->vpu_result	= result;
		jmgr_list_manager(( void *)cmd_list, LIST_ADD);

		return 1;
	}

	return 1;
}

static int _jmgr_internal_handler(void)
{
	int ret, ret_code = RETCODE_SUCCESS;
	int timeout = 200;

	if( jmgr_data.current_resolution > 1920*1080 )
		timeout = 5000;

	if(jmgr_data.check_interrupt_detection)
	{
		if(jmgr_data.oper_intr > 0)
		{			
			detailk("Success 1: jpu operation!! \n");
		}
		else
		{
			ret = wait_event_interruptible_timeout(jmgr_data.oper_wq, jmgr_data.oper_intr > 0, msecs_to_jiffies(timeout));

			if(jmgr_data.oper_intr > 0)
			{
				detailk("Success 2: jpu operation!! \n");
#if defined(FORCED_ERROR)
				if(forced_error_count-- <= 0)
				{
					ret_code = RETCODE_CODEC_EXIT;
					forced_error_count = FORCED_ERR_CNT;
				}
#endif
			}
			else
			{
				err("[CMD 0x%x][%d]: jpu timed_out(ref %d msec) => oper_intr[%d]!! [%d]th frame len %d\n", jmgr_data.current_cmd, ret, timeout, jmgr_data.oper_intr, jmgr_data.nDecode_Cmd, jmgr_data.szFrame_Len);
				vetc_dump_reg_all(jmgr_data.base_addr, "_jmgr_internal_handler timed_out");
				ret_code = RETCODE_CODEC_EXIT;
			}
		}

		jmgr_data.oper_intr = 0;
		jmgr_status_clear(jmgr_data.base_addr);
	}

	return ret_code;
}

static int _jmgr_convert_returnType(int err)
{
	if( err >= JPG_RET_INVALID_HANDLE && err <= JPG_RET_NOT_INITIALIZED )
	{
		return (err - 2);
	}

	switch(err)
	{
		case JPG_RET_BIT_EMPTY: 
			return 50;

		case JPG_RET_EOS: 
			return 51;

		case JPG_RET_INSUFFICIENT_BITSTREAM_BUF: 
			return RETCODE_INSUFFICIENT_BITSTREAM_BUF;

		case JPG_RET_CODEC_FINISH: 
			return RETCODE_CODEC_FINISH;

		default:
			return err;
	}
}

static int _jmgr_process(vputype type, int cmd, int pHandle, void* args)
{
	int ret = 0;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
	struct timeval t1 , t2;
	int time_gap_ms = 0;
#endif

	jmgr_data.check_interrupt_detection = 0;
	jmgr_data.current_cmd = cmd;

	if( type <= VPU_DEC_EXT3 )
	{
		if( cmd != VPU_DEC_INIT )
		{
			if(jmgr_get_close(type) || jmgr_data.handle[type] == 0x00)
				return RETCODE_MULTI_CODEC_EXIT_TIMEOUT;
		}

		if( cmd != VPU_DEC_BUF_FLAG_CLEAR && cmd != VPU_DEC_DECODE ) {
			cmdk("@@@@@@@@@@ Decoder(%d), command: 0x%x \n", type, cmd);
		}

		switch(cmd)
		{
			case VPU_DEC_INIT:
				{
					JDEC_INIT_t* arg;

					arg = (JDEC_INIT_t *)args;
					jmgr_data.handle[type] = 0x00;

					arg->gsJpuDecInit.m_RegBaseVirtualAddr 	= io_p2v(jmgr_data.base_addr);
					arg->gsJpuDecInit.m_Memcpy 				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
					arg->gsJpuDecInit.m_Memset 				= (void* (*) ( void*, int, unsigned int ))memset;
					arg->gsJpuDecInit.m_Interrupt			= (int	(*) ( void ))_jmgr_internal_handler;

					jmgr_data.check_interrupt_detection = 1;
#if defined(JPU_C5)
					dprintk("@@ Dec :: Init In => Reg(0x%x/0x%x), Stream(0x%x/0x%x, 0x%x)\n",
								jmgr_data.base_addr, arg->gsJpuDecInit.m_RegBaseVirtualAddr,
								arg->gsJpuDecInit.m_BitstreamBufAddr[PA], arg->gsJpuDecInit.m_BitstreamBufAddr[VA], arg->gsJpuDecInit.m_iBitstreamBufSize);

					dprintk("@@ Dec :: Init In => rotation(%d/%d), mirror(%d/%d), Interleave: %d \n",
								arg->gsJpuDecInit.m_iRot_angle, arg->gsJpuDecInit.m_iRot_enalbe, 
								arg->gsJpuDecInit.m_iMirrordir, arg->gsJpuDecInit.m_iMirror_enable, arg->gsJpuDecInit.m_bCbCrInterleaveMode);

					ret = tcc_jpu_dec(JPU_DEC_INIT, (void*)(&arg->gsJpuDecHandle), (void*)(&arg->gsJpuDecInit), (void*)(&arg->gsJpuDecInitialInfo));
					jmgr_data.current_resolution = arg->gsJpuDecInitialInfo.m_iPicWidth * arg->gsJpuDecInitialInfo.m_iPicHeight;
#else
					dprintk("@@ Dec :: Init In => Reg(0x%x/0x%x), Stream(0x%x/0x%x, 0x%x) Interleave: %d \n",
								jmgr_data.base_addr, arg->gsJpuDecInit.m_RegBaseVirtualAddr,
								arg->gsJpuDecInit.m_BitstreamBufAddr[PA], arg->gsJpuDecInit.m_BitstreamBufAddr[VA], 
								arg->gsJpuDecInit.m_iBitstreamBufSize, arg->gsJpuDecInit.m_iCbCrInterleaveMode);

					ret = tcc_jpu_dec(JPU_DEC_INIT, (void*)(&arg->gsJpuDecHandle), (void*)(&arg->gsJpuDecInit), (void*)NULL);
#endif
					if( ret != RETCODE_SUCCESS){
						printk("@@ Dec :: Init Done with ret(0x%x)\n", ret);
						if( ret != JPG_RET_CODEC_EXIT )
						{
							vetc_dump_reg_all(jmgr_data.base_addr, "init failure");
#if defined(JPU_C5)
							//ret = tcc_jpu_dec(JPU_DEC_INIT, (void*)(&arg->gsJpuDecHandle), (void*)(&arg->gsJpuDecInit), (void*)(&arg->gsJpuDecInitialInfo));
#else
							//ret = tcc_jpu_dec(JPU_DEC_INIT, (void*)(&arg->gsJpuDecHandle), (void*)(&arg->gsJpuDecInit), (void*)NULL);
#endif
							//printk("@@ Dec :: Init Done with ret(0x%x)\n", ret);
							//vetc_dump_reg_all(jmgr_data.base_addr, "init success");
						}
					}

					ret = _jmgr_convert_returnType(ret);
					if( ret != RETCODE_CODEC_EXIT && arg->gsJpuDecHandle != 0) {
						jmgr_data.handle[type] = arg->gsJpuDecHandle;
						jmgr_set_close(type, 0, 0);
						cmdk("@@ Dec :: jmgr_data.handle = 0x%x \n", arg->gsJpuDecHandle);
					}
					else{
						//To free memory!!
						jmgr_set_close(type, 0, 0);
						jmgr_set_close(type, 1, 1);
					}
					dprintk("@@ Dec :: Init Done Handle(0x%x) \n", arg->gsJpuDecHandle);

			#ifdef CONFIG_VPU_TIME_MEASUREMENT
					jmgr_data.iTime[type].print_out_index = jmgr_data.iTime[type].proc_base_cnt = 0;
					jmgr_data.iTime[type].accumulated_proc_time = jmgr_data.iTime[type].accumulated_frame_cnt = 0;
					jmgr_data.iTime[type].proc_time_30frames = 0;
			#endif
				}
				break;
				
			case VPU_DEC_SEQ_HEADER:
#if defined(JPU_C6)
				{
					JDEC_SEQ_HEADER_t *arg;
					int iSize;

					arg = (JDEC_SEQ_HEADER_t *)args;
					jmgr_data.szFrame_Len = iSize = (int)arg->stream_size;
					jmgr_data.check_interrupt_detection = 1;
					jmgr_data.nDecode_Cmd = 0;
					dprintk("@@ Dec :: JPU_DEC_SEQ_HEADER in :: Handle(0x%x) size(%d) \n", pHandle, arg->stream_size);
					ret = tcc_jpu_dec(JPU_DEC_SEQ_HEADER, &pHandle, (void*)iSize, (void*)(&arg->gsJpuDecInitialInfo));

					ret = _jmgr_convert_returnType(ret);
					printk("@@ Dec :: JPU_DEC_SEQ_HEADER out 0x%x :: res info(%d x %d), src_format(%d), Error_reason(%d), minFB(%d) \n", ret, arg->gsJpuDecInitialInfo.m_iPicWidth,
							arg->gsJpuDecInitialInfo.m_iPicHeight, arg->gsJpuDecInitialInfo.m_iSourceFormat,
							arg->gsJpuDecInitialInfo.m_iErrorReason, arg->gsJpuDecInitialInfo.m_iMinFrameBufferCount);
					jmgr_data.current_resolution = arg->gsJpuDecInitialInfo.m_iPicWidth * arg->gsJpuDecInitialInfo.m_iPicHeight;
				}
#else
				{
					err("@@ Dec :: not supported command(0x%x) \n", cmd);
					return 0x999;
				}
#endif
				break;
				
			case VPU_DEC_REG_FRAME_BUFFER:
				{
					JPU_SET_BUFFER_t *arg;
					
					arg = (JPU_SET_BUFFER_t *)args;
#if defined(JPU_C5)
					dprintk("@@ Dec :: JPU_DEC_REG_FRAME_BUFFER in :: scale[%d], addr[0x%x/0x%x] \n", arg->gsJpuDecBuffer.m_iJPGScaleRatio,
								arg->gsJpuDecBuffer.m_FrameBufferStartAddr[PA], arg->gsJpuDecBuffer.m_FrameBufferStartAddr[VA]);
#else
					dprintk("@@ Dec :: JPU_DEC_REG_FRAME_BUFFER in :: count[%d], scale[%d], addr[0x%x/0x%x], Reserved[8] = 0x%x \n", arg->gsJpuDecBuffer.m_iFrameBufferCount, arg->gsJpuDecBuffer.m_iJPGScaleRatio,
								arg->gsJpuDecBuffer.m_FrameBufferStartAddr[PA], arg->gsJpuDecBuffer.m_FrameBufferStartAddr[VA], arg->gsJpuDecBuffer.m_Reserved[8]);
#endif
					ret = tcc_jpu_dec(JPU_DEC_REG_FRAME_BUFFER, &pHandle, (void*)(&arg->gsJpuDecBuffer), (void*)NULL);
					ret = _jmgr_convert_returnType(ret);
					dprintk("@@ Dec :: JPU_DEC_REG_FRAME_BUFFER out \n");
				}
				break;
			
			case VPU_DEC_DECODE:
				{						
					JPU_DECODE_t *arg;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t1 );
#endif
					arg = (JPU_DECODE_t *)args;

					jmgr_data.szFrame_Len = arg->gsJpuDecInput.m_iBitstreamDataSize;
#if defined(JPU_C6)
					dprintk("@@ Dec :: Dec In => 0x%x - 0x%x, 0x%x \n", arg->gsJpuDecInput.m_BitstreamDataAddr[PA], arg->gsJpuDecInput.m_BitstreamDataAddr[VA], arg->gsJpuDecInput.m_iBitstreamDataSize);
#else
					dprintk("@@ Dec :: Dec In => 0x%x - 0x%x, 0x%x, 0x%x - 0x%x, toggle: %d \n", arg->gsJpuDecInput.m_BitstreamDataAddr[PA], arg->gsJpuDecInput.m_BitstreamDataAddr[VA], arg->gsJpuDecInput.m_iBitstreamDataSize,
							arg->gsJpuDecInput.m_FrameBufferStartAddr[PA], arg->gsJpuDecInput.m_FrameBufferStartAddr[VA],
							arg->gsJpuDecInput.m_iLooptogle);
#endif
					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_dec(JPU_DEC_DECODE, &pHandle, (void*)(&arg->gsJpuDecInput), (void*)(&arg->gsJpuDecOutput));
					ret = _jmgr_convert_returnType(ret);

					dprintk("@@ Dec :: Dec Out => %d x %d, status(%d), Consumed(%d), Err(%d) \n", arg->gsJpuDecOutput.m_DecOutInfo.m_iWidth, arg->gsJpuDecOutput.m_DecOutInfo.m_iHeight,
										arg->gsJpuDecOutput.m_DecOutInfo.m_iDecodingStatus, arg->gsJpuDecOutput.m_DecOutInfo.m_iConsumedBytes,
										arg->gsJpuDecOutput.m_DecOutInfo.m_iNumOfErrMBs);

					dprintk("@@ Dec :: Dec Out => 0x%x 0x%x 0x%x / 0x%x 0x%x 0x%x \n", 
								(unsigned int)arg->gsJpuDecOutput.m_pCurrOut[PA][0], (unsigned int)arg->gsJpuDecOutput.m_pCurrOut[PA][1], (unsigned int)arg->gsJpuDecOutput.m_pCurrOut[PA][2], 
								(unsigned int)arg->gsJpuDecOutput.m_pCurrOut[VA][0], (unsigned int)arg->gsJpuDecOutput.m_pCurrOut[VA][1], (unsigned int)arg->gsJpuDecOutput.m_pCurrOut[VA][2]);

					if(arg->gsJpuDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL){
						 err("Buffer full\n");
						//vetc_dump_reg_all(jmgr_data.base_addr, "Buffer full");
					}
					jmgr_data.nDecode_Cmd++;

#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t2 );
#endif
				}
				break;
				
			case VPU_DEC_CLOSE:
				{
					//JPU_DECODE_t *arg;

					//arg = (JPU_DECODE_t *)args;
					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_dec(JPU_DEC_CLOSE, &pHandle, (void*)NULL, (void*)NULL/*(&arg->gsJpuDecOutput)*/);	
					ret = _jmgr_convert_returnType(ret);
					dprintk("@@ Dec :: JPU_DEC_CLOSED !! \n");

                    jmgr_set_close(type, 1, 1);
				}
				break;

			case VPU_CODEC_GET_VERSION:
				{
					JPU_GET_VERSION_t *arg;

					arg = (JPU_GET_VERSION_t *)args;
					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_dec(JPU_CODEC_GET_VERSION, &pHandle, arg->pszVersion, arg->pszBuildData);
					ret = _jmgr_convert_returnType(ret);
					dprintk("@@ Dec :: version : %s, build : %s\n", arg->pszVersion, arg->pszBuildData);
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
		switch(cmd)
		{
			case VPU_ENC_INIT:
				{
					JENC_INIT_t* arg;

					arg = (JENC_INIT_t *)args;
					jmgr_data.handle[type] = 0x00;

					arg->gsJpuEncInit.m_RegBaseVirtualAddr 	= io_p2v(jmgr_data.base_addr);
					arg->gsJpuEncInit.m_Memcpy 				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
					arg->gsJpuEncInit.m_Memset 				= (void* (*) ( void*, int, unsigned int ))memset;
					arg->gsJpuEncInit.m_Interrupt			= (int	(*) ( void ))_jmgr_internal_handler;

					jmgr_data.check_interrupt_detection = 1;
#if defined(JPU_C6)
					dprintk("## Enc :: Init In => Reg(0x%x/0x%x), Src(%d x %d, %d), Q(%d), Stream(0x%x/0x%x, 0x%x), Inter(%d), Option(0x%x)\n",
								jmgr_data.base_addr, arg->gsJpuEncInit.m_RegBaseVirtualAddr,
								arg->gsJpuEncInit.m_iPicWidth, arg->gsJpuEncInit.m_iPicHeight, arg->gsJpuEncInit.m_iSourceFormat, arg->gsJpuEncInit.m_iEncQuality, 
								arg->gsJpuEncInit.m_BitstreamBufferAddr[PA], arg->gsJpuEncInit.m_BitstreamBufferAddr[VA], arg->gsJpuEncInit.m_iBitstreamBufferSize,
								arg->gsJpuEncInit.m_iCbCrInterleaveMode, arg->gsJpuEncInit.m_uiEncOptFlags);
#else
					dprintk("## Enc :: Init In => Reg(0x%x/0x%x), Src(%d x %d, %d), Stream(0x%x/0x%x, 0x%x), rotation(%d), Inter(%d), Option(0x%x)\n",
								jmgr_data.base_addr, arg->gsJpuEncInit.m_RegBaseVirtualAddr,
								arg->gsJpuEncInit.m_iPicWidth, arg->gsJpuEncInit.m_iPicHeight, arg->gsJpuEncInit.m_iMjpg_sourceFormat, arg->gsJpuEncInit.m_iEncQuality,
								arg->gsJpuEncInit.m_BitstreamBufferAddr[PA], arg->gsJpuEncInit.m_BitstreamBufferAddr[VA], arg->gsJpuEncInit.m_iBitstreamBufferSize,
								arg->gsJpuEncInit.m_iRotMode, arg->gsJpuEncInit.m_bCbCrInterleaveMode, arg->gsJpuEncInit.m_uiEncOptFlags);
#endif
					if(arg->gsJpuEncInit.m_iSourceFormat)
						jmgr_data.szFrame_Len = arg->gsJpuEncInit.m_iPicWidth * arg->gsJpuEncInit.m_iPicHeight * 2;
					else
						jmgr_data.szFrame_Len = arg->gsJpuEncInit.m_iPicWidth * arg->gsJpuEncInit.m_iPicHeight *3 / 2;
					jmgr_data.current_resolution = arg->gsJpuEncInit.m_iPicWidth * arg->gsJpuEncInit.m_iPicHeight;

					ret = tcc_jpu_enc(JPU_ENC_INIT, (void*)(&arg->gsJpuEncHandle), (void*)(&arg->gsJpuEncInit), (void*)NULL);
					if( ret != RETCODE_SUCCESS){
						printk("## Enc :: Init Done with ret(0x%x)\n", ret);
						if( ret != RETCODE_CODEC_EXIT ){
							vetc_dump_reg_all(jmgr_data.base_addr, "init failure");
							//ret = tcc_jpu_enc(cmd, (void*)(&arg->gsJpuEncHandle), (void*)(&arg->gsJpuEncInit), (void*)NULL);
							//printk("## Enc :: Init Done with ret(0x%x)\n", ret);
							//vetc_dump_reg_all(jmgr_data.base_addr, "init success");
						}
					}

					ret = _jmgr_convert_returnType(ret);
					if( ret != RETCODE_CODEC_EXIT && arg->gsJpuEncHandle != 0) {
						jmgr_data.handle[type] = arg->gsJpuEncHandle;
						jmgr_set_close(type, 0, 0);
						cmdk("## Enc :: jmgr_data.handle = 0x%x \n", arg->gsJpuEncHandle);
					}
					dprintk("## Enc :: Init Done Handle(0x%x) \n", arg->gsJpuEncHandle);
					jmgr_data.nDecode_Cmd = 0;
			#ifdef CONFIG_VPU_TIME_MEASUREMENT
					jmgr_data.iTime[type].print_out_index = jmgr_data.iTime[type].proc_base_cnt = 0;
					jmgr_data.iTime[type].accumulated_proc_time = jmgr_data.iTime[type].accumulated_frame_cnt = 0;
					jmgr_data.iTime[type].proc_time_30frames = 0;
			#endif
				}
				break;

			case VPU_ENC_ENCODE:
				{						
					JPU_ENCODE_t *arg;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t1 );
#endif
					arg = (JPU_ENCODE_t *)args;

					dprintk("## Enc :: Enc In => Handle(0x%x), YUV(0x%x - 0x%x - 0x%x) -> BitStream(0x%x - 0x%x / 0x%x) \n", pHandle,
							arg->gsJpuEncInput.m_PicYAddr, arg->gsJpuEncInput.m_PicCbAddr, arg->gsJpuEncInput.m_PicCrAddr,
							arg->gsJpuEncInput.m_BitstreamBufferAddr[PA], arg->gsJpuEncInput.m_BitstreamBufferAddr[VA], arg->gsJpuEncInput.m_iBitstreamBufferSize);

					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_enc(JPU_ENC_ENCODE, &pHandle, (void*)(&arg->gsJpuEncInput), (void*)(&arg->gsJpuEncOutput));
					ret = _jmgr_convert_returnType(ret);

#if defined(JPU_C5)
					dprintk("## Enc :: Enc Out => (0x%x/0x%x), Size(%d/%d) \n", arg->gsJpuEncOutput.m_BitstreamOut[0], arg->gsJpuEncOutput.m_BitstreamOut[1],
							arg->gsJpuEncOutput.m_iHeaderOutSize, arg->gsJpuEncOutput.m_iBitstreamOutSize);
#else
					dprintk("## Enc :: Enc Out => ret(%d) (0x%x/0x%x), Size(%d/%d) \n", ret, arg->gsJpuEncOutput.m_BitstreamOut[0], arg->gsJpuEncOutput.m_BitstreamOut[1],
							arg->gsJpuEncOutput.m_iBitstreamHeaderSize, arg->gsJpuEncOutput.m_iBitstreamOutSize);
#endif
					jmgr_data.nDecode_Cmd++;
#ifdef CONFIG_VPU_TIME_MEASUREMENT
					do_gettimeofday( &t2 );
#endif
				}
				break;

			case VPU_ENC_CLOSE:
				{
					//JPU_ENCODE_t *arg;

					//arg = (JPU_ENCODE_t *)args;
					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_enc(JPU_ENC_CLOSE, &pHandle, (void*)NULL, (void*)NULL/*(&arg->gsJpuEncOutput)*/);
					ret = _jmgr_convert_returnType(ret);
					dprintk("## Enc :: JPU_ENC_CLOSED !! \n");

                    jmgr_set_close(type, 1, 1);
				}
				break;

			case VPU_CODEC_GET_VERSION:
				{
					JPU_GET_VERSION_t *arg;

					arg = (JPU_GET_VERSION_t *)args;
					jmgr_data.check_interrupt_detection = 1;
					ret = tcc_jpu_enc(JPU_CODEC_GET_VERSION, &pHandle, arg->pszVersion, arg->pszBuildData);
					ret = _jmgr_convert_returnType(ret);
					dprintk("## Enc :: version : %s, build : %s\n", arg->pszVersion, arg->pszBuildData);
				}
				break;

			default:
				{
					err("## Enc :: not supported command(0x%x) \n", cmd);
					return 0x999;
				}
				break;
		}
	}
#endif

#ifdef CONFIG_VPU_TIME_MEASUREMENT
	time_gap_ms = vetc_GetTimediff_ms(t1, t2);

	if( cmd == VPU_DEC_DECODE || cmd == VPU_ENC_ENCODE ) {
		jmgr_data.iTime[type].accumulated_frame_cnt++;
		jmgr_data.iTime[type].proc_time[jmgr_data.iTime[type].proc_base_cnt] = time_gap_ms;
		jmgr_data.iTime[type].proc_time_30frames += time_gap_ms;
		jmgr_data.iTime[type].accumulated_proc_time += time_gap_ms;
		if(jmgr_data.iTime[type].proc_base_cnt != 0 && jmgr_data.iTime[type].proc_base_cnt % 29 == 0)
		{
			printk(" JPU[%d] :: Dec[%4d] time %2d.%2d / %2d.%2d ms: %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d \n",
						type, 
						jmgr_data.iTime[type].print_out_index,
						jmgr_data.iTime[type].proc_time_30frames/30,
						((jmgr_data.iTime[type].proc_time_30frames%30)*100)/30,
						jmgr_data.iTime[type].accumulated_proc_time/jmgr_data.iTime[type].accumulated_frame_cnt,
						((jmgr_data.iTime[type].accumulated_proc_time%jmgr_data.iTime[type].accumulated_frame_cnt)*100)/jmgr_data.iTime[type].accumulated_frame_cnt,
						jmgr_data.iTime[type].proc_time[0], jmgr_data.iTime[type].proc_time[1], jmgr_data.iTime[type].proc_time[2], jmgr_data.iTime[type].proc_time[3], jmgr_data.iTime[type].proc_time[4],
						jmgr_data.iTime[type].proc_time[5], jmgr_data.iTime[type].proc_time[6], jmgr_data.iTime[type].proc_time[7], jmgr_data.iTime[type].proc_time[8], jmgr_data.iTime[type].proc_time[9],
						jmgr_data.iTime[type].proc_time[10], jmgr_data.iTime[type].proc_time[11], jmgr_data.iTime[type].proc_time[12], jmgr_data.iTime[type].proc_time[13], jmgr_data.iTime[type].proc_time[14],
						jmgr_data.iTime[type].proc_time[15], jmgr_data.iTime[type].proc_time[16], jmgr_data.iTime[type].proc_time[17], jmgr_data.iTime[type].proc_time[18], jmgr_data.iTime[type].proc_time[19],
						jmgr_data.iTime[type].proc_time[20], jmgr_data.iTime[type].proc_time[21], jmgr_data.iTime[type].proc_time[22], jmgr_data.iTime[type].proc_time[23], jmgr_data.iTime[type].proc_time[24],
						jmgr_data.iTime[type].proc_time[25], jmgr_data.iTime[type].proc_time[26], jmgr_data.iTime[type].proc_time[27], jmgr_data.iTime[type].proc_time[28], jmgr_data.iTime[type].proc_time[29]);
			jmgr_data.iTime[type].proc_base_cnt = 0;
			jmgr_data.iTime[type].proc_time_30frames = 0;
			jmgr_data.iTime[type].print_out_index++;
		}
		else{
			jmgr_data.iTime[type].proc_base_cnt++;
		}
	}
#endif

	return ret;
}

static long _jmgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	CONTENTS_INFO info;
	OPENED_sINFO open_info;

	mutex_lock(&jmgr_data.comm_data.io_mutex);
	
	switch(cmd)
	{
		case VPU_SET_CLK:
			if(copy_from_user(&info,(CONTENTS_INFO*)arg,sizeof(info)))
				ret = -EFAULT;
			else{
	#if defined(CONFIG_CLOCK_TABLE)
				{
					int clk_idx;

					clk_idx = jmgr_calcClock(info);								
					ret = jmgr_setClock(clk_idx, info, jmgr_data.dev_opened, jmgr_data.clk_limitation);
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
			}
			break;

		case VPU_CHECK_CODEC_STATUS:
			{
				if (copy_to_user((int*)arg, jmgr_data.closed, sizeof(jmgr_data.closed)))
					ret = -EFAULT;
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

	mutex_unlock(&jmgr_data.comm_data.io_mutex);

	return ret;
}

static irqreturn_t _jmgr_isr_handler(int irq, void *dev_id)
{
	unsigned long flags;
	
	detailk("_jmgr_isr_handler \n");
	
	spin_lock_irqsave(&(jmgr_data.oper_lock), flags);
	jmgr_data.oper_intr++;
	spin_unlock_irqrestore(&(jmgr_data.oper_lock), flags);
	
	wake_up_interruptible(&(jmgr_data.oper_wq));
	
	return IRQ_HANDLED;
}

static int _jmgr_proc_exit_by_external(struct VpuList *list, int *result, unsigned int type)
{	
	if(!jmgr_get_close(type) && jmgr_data.handle[type] != 0x00)
	{
		list->type = type;
		if( type >= VPU_ENC )
			list->cmd_type = VPU_ENC_CLOSE;
		else
			list->cmd_type = VPU_DEC_CLOSE;
		list->handle 	= jmgr_data.handle[type];
		list->args 		= NULL;
		list->comm_data = NULL;
		list->vpu_result = result;

		printk("_jmgr_proc_exit_by_external for %d!! \n", type);
		jmgr_list_manager(( void *)list, LIST_ADD);

		return 1;
	}

	return 0;
}

static int _jmgr_open(struct inode *inode, struct file *filp)
{
	if (!jmgr_data.irq_reged) {
		err("not registered jpu-mgr-irq \n");
	}

	dprintk("_jmgr_open In!! %d'th \n", jmgr_data.dev_opened);

	jmgr_enable_clock();

	if(jmgr_data.dev_opened == 0)
	{
		jmgr_readyClock(jmgr_data.dev_opened, jmgr_data.base_addr);

#ifdef FORCED_ERROR
		forced_error_count = FORCED_ERR_CNT;
#endif
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)	
		jmgr_data.only_decmode = 0;
#else
		jmgr_data.only_decmode = 1;
#endif
		jmgr_data.clk_limitation = 1;
		//jmgr_hw_reset();
		jmgr_data.cmd_processing = 0;

		jmgr_enable_irq(jmgr_data.irq);
		vetc_reg_init(jmgr_data.base_addr);
		vmem_reinit();
	}
	jmgr_data.dev_opened++;	

	jmgr_log_clkstatus(0, jmgr_data.dev_opened, jmgr_data.nOpened_Count);

	filp->private_data = &jmgr_data;
	
	return 0;
}

static void _jmgr_wait_process(int wait_ms)
{
	int max_count = wait_ms/20;

	//wait!! in case exceptional processing. ex). sdcard out!!
	while(jmgr_data.cmd_processing)
	{
		max_count--;
		msleep(20);

		if(max_count <= 0)
		{
			err("cmd_processing(cmd %d) didn't finish!! \n", jmgr_data.current_cmd);
			break;
		}
	}
}

static int _jmgr_external_all_close(int wait_ms)
{
	int type;
	int max_count = 0;
	int ret;

	for(type = 0; type < JPU_MAX; type++)
	{
		if(_jmgr_proc_exit_by_external(&jmgr_data.vList[type], &ret, type))
		{
			max_count = wait_ms/10;
			while(!jmgr_get_close(type))
			{
				max_count--;
				msleep(10);
			}
		}
	}

	return 0;
}

static int _jmgr_release(struct inode *inode, struct file *filp)
{
	dprintk("_jmgr_release In!! %d'th \n", jmgr_data.dev_opened);

	_jmgr_wait_process(2000);

	if(jmgr_data.dev_opened > 0)
		jmgr_data.dev_opened--;
	if(jmgr_data.dev_opened == 0)
	{
//////////////////////////////////////
		int type, alive_cnt = 0;

#if 1 // To close whole jpu instance when being killed process opened this.
		jmgr_data.external_proc = 1;
		_jmgr_external_all_close(200);
		_jmgr_wait_process(2000);
		jmgr_data.external_proc = 0;
#endif

		for(type=0; type<JPU_MAX; type++) {
			if( jmgr_data.closed[type] == 0 ){
				alive_cnt++;
			}
		}

		if( alive_cnt )
		{
			// clear instances of jpu by force.
			//TCC_VPU_DEC( 0x40, (void*)NULL, (void*)NULL, (void*)NULL);
			printk("JPU might be cleared by force. \n");
		}

//////////////////////////////////////
		jmgr_data.oper_intr = 0;
		jmgr_data.cmd_processing = 0;

		_jmgr_close_all(1);
//////////////////////////////////////

		jmgr_disable_irq(jmgr_data.irq);
		jmgr_BusPrioritySetting(BUS_FOR_NORMAL, 0);
	}

	jmgr_disable_clock();
	jmgr_clearClock(jmgr_data.dev_opened);

	jmgr_data.nOpened_Count++;
	jmgr_log_clkstatus(1, jmgr_data.dev_opened, jmgr_data.nOpened_Count);
	
	return 0;
}

int jmgr_list_manager(void* args, unsigned int cmd)
{
	int ret;

	mutex_lock(&jmgr_data.comm_data.list_mutex);

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
					list_add_tail(&data->list, &jmgr_data.comm_data.main_list);jmgr_data.cmd_queued++;
					jmgr_data.comm_data.thread_intr++;
					wake_up_interruptible(&(jmgr_data.comm_data.thread_wq));
				}
				break;

			case LIST_DEL:
				data = (VpuList_t*)args;
				list_del(&data->list);jmgr_data.cmd_queued--;
				break;

			case LIST_IS_EMPTY:
				ret = list_empty(&jmgr_data.comm_data.main_list);
				break;

			case LIST_GET_ENTRY:
				if(!args)
				{
					err("data is null \n");
					goto Error;
				}
				*data_address = (unsigned int)list_first_entry(&jmgr_data.comm_data.main_list, VpuList_t, list);
				break;
		}
	}

Error:	
	mutex_unlock(&jmgr_data.comm_data.list_mutex);

	return ret;
}


//////////////////////////////////////////////////////////////////////////
// JPU Thread!!
static int _jmgr_operation(void)
{
	int oper_finished;
	unsigned int data = 0;
	VpuList_t *oper_data = NULL;
    
	while(!jmgr_list_manager(NULL, LIST_IS_EMPTY))
	{
		jmgr_data.cmd_processing = 1;

		oper_finished = 1;
		dprintk("_jmgr_operation :: not empty jmgr_data.cmd_queued(%d) \n", jmgr_data.cmd_queued);

        {
    		jmgr_list_manager(&data, LIST_GET_ENTRY);
    		if(!data) {
    			err("data is null \n");
    			jmgr_data.cmd_processing = 0;
    			return 0;
    		}
    		oper_data = (VpuList_t *)data;
    		*(oper_data->vpu_result) |= RET2;
        }
        

		dprintk("_jmgr_operation [%d] :: cmd = 0x%x, cmd_queued(%d) \n", oper_data->type, oper_data->cmd_type, jmgr_data.cmd_queued);

		if(oper_data->type < JPU_MAX /*&& oper_data != NULL*/ /*&& oper_data->comm_data != NULL*/)
		{
			*(oper_data->vpu_result) |= RET3;

			*(oper_data->vpu_result) = _jmgr_process(oper_data->type, oper_data->cmd_type, oper_data->handle, oper_data->args);
			oper_finished = 1;
			if(*(oper_data->vpu_result) != RETCODE_SUCCESS)
			{
				if( *(oper_data->vpu_result) != RETCODE_INSUFFICIENT_BITSTREAM && *(oper_data->vpu_result) != RETCODE_INSUFFICIENT_BITSTREAM_BUF)
					err("jmgr_out[0x%x] :: type = %d, handle = 0x%x, cmd = 0x%x, frame_len %d \n", *(oper_data->vpu_result), oper_data->type, oper_data->handle, oper_data->cmd_type, jmgr_data.szFrame_Len);

				if(*(oper_data->vpu_result) == RETCODE_CODEC_EXIT)
				{
					int opened_count = jmgr_data.dev_opened;

					_jmgr_close_all(1);

			#if 1
					while(opened_count)
					{
						jmgr_disable_clock();
						if(opened_count > 0)
							opened_count--;
					}

					//msleep(1);
					opened_count = jmgr_data.dev_opened;
					while(opened_count)
					{
						jmgr_enable_clock();
						if(opened_count > 0)
							opened_count--;
					}
			#else
					jmgr_hw_reset();
			#endif
				}
			}
		}
		else
		{
			printk("_jmgr_operation :: missed info or unknown command => type = 0x%x, cmd = 0x%x,  \n", oper_data->type, oper_data->cmd_type);
			*(oper_data->vpu_result) = RETCODE_FAILURE;
			oper_finished = 0;
		}
		
		if(oper_finished)
		{
			if(oper_data->comm_data != NULL && jmgr_data.dev_opened != 0)
			{
				#ifdef JPU_DEBUG
				unsigned int old_count = oper_data->comm_data->count;
				#endif
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
				err("Error: abnormal exception or external command was processed!! 0x%x - %d\n", (unsigned int)oper_data->comm_data, jmgr_data.dev_opened);
			}
		}
		else{
			err("Error: abnormal exception 2!! 0x%x - %d\n",(unsigned int)oper_data->comm_data, jmgr_data.dev_opened);
		}

        {
            jmgr_list_manager((void*)oper_data, LIST_DEL);
        }

        jmgr_data.cmd_processing = 0;
	}

	return 0;
}

static int _jmgr_thread(void *kthread)
{
	dprintk("_jmgr_thread for dec is running. \n");

	do 
	{
//		detailk("_jmgr_thread wait_sleep \n");

		if(jmgr_list_manager(NULL, LIST_IS_EMPTY))
		{
			jmgr_data.cmd_processing = 0;

			//wait_event_interruptible(jmgr_data.comm_data.thread_wq, jmgr_data.comm_data.thread_intr > 0);
			wait_event_interruptible_timeout(jmgr_data.comm_data.thread_wq, jmgr_data.comm_data.thread_intr > 0, msecs_to_jiffies(50));
			jmgr_data.comm_data.thread_intr = 0;
		}
		else
		{
			if(jmgr_data.dev_opened || jmgr_data.external_proc){
				_jmgr_operation();
			}
			else{
				unsigned int data = 0;
				VpuList_t *oper_data = NULL;

				printk("DEL for empty \n");

				jmgr_list_manager(&data, LIST_GET_ENTRY);
				oper_data = (VpuList_t *)data;
				if(oper_data)
					jmgr_list_manager((void*)oper_data, LIST_DEL);
			}
		}
		
	} while (!kthread_should_stop());

	return 0;
}

//////////////////////////////////////////////////////////////////////////
static int _jmgr_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "_jmgr_mmap: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = vmem_get_pgprot(vma->vm_page_prot, vma->vm_pgoff);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk("_jmgr_mmap :: remap_pfn_range failed\n");
		return -EAGAIN;
	}

	vma->vm_ops 	= NULL;
	vma->vm_flags	|= VM_IO;
	vma->vm_flags   |= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static struct file_operations _jmgr_fops = {
	.open				= _jmgr_open,
	.release			= _jmgr_release,
	.mmap				= _jmgr_mmap,
	.unlocked_ioctl		= _jmgr_ioctl,
};

static struct miscdevice _jmgr_misc_device =
{
    MISC_DYNAMIC_MINOR,
    JMGR_NAME,
    &_jmgr_fops,
};

int jmgr_probe(struct platform_device *pdev)
{
	int ret;
	int type;
	unsigned long int_flags;
	struct resource *regs = NULL;

	if (pdev->dev.of_node == NULL) {
		return - -ENODEV;
	}

	dprintk("jmgr initializing!! \n");
	memset(&jmgr_data, 0, sizeof(mgr_data_t));
	for(type=0; type<JPU_MAX; type++) {
		jmgr_data.closed[type] = 1;
	}

	jmgr_init_variable();

	jmgr_data.irq = platform_get_irq(pdev, 0);
	jmgr_data.nOpened_Count = 0;
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jmgr_data.base_addr = regs->start;
	dprintk("============> JPU base address [0x%x], irq num [%d] \n", jmgr_data.base_addr, jmgr_data.irq - 32);

	jmgr_get_clock(pdev->dev.of_node);

	spin_lock_init(&(jmgr_data.oper_lock));
//	spin_lock_init(&(jmgr_data.comm_data.lock));

	init_waitqueue_head(&jmgr_data.comm_data.thread_wq);
	init_waitqueue_head(&jmgr_data.oper_wq);

	mutex_init(&jmgr_data.comm_data.list_mutex);
	mutex_init(&(jmgr_data.comm_data.io_mutex));

    INIT_LIST_HEAD(&jmgr_data.comm_data.main_list);
	INIT_LIST_HEAD(&jmgr_data.comm_data.wait_list);


	jmgr_init_interrupt();
	int_flags = jmgr_get_int_flags();
    ret = jmgr_request_irq(jmgr_data.irq, _jmgr_isr_handler, int_flags, JMGR_NAME, &jmgr_data);
	if (ret) {
		err("to aquire jpu-dec-irq \n");
	}
	jmgr_data.irq_reged = 1;
	jmgr_disable_irq(jmgr_data.irq);

	kidle_task = kthread_run(_jmgr_thread, NULL, "kjmgr_operd");	
	if( IS_ERR(kidle_task) )
	{		
		err("unable to create thread!! \n");
		kidle_task = NULL;
		return -1;	
	}
	dprintk("success :: thread created!! \n");	

	_jmgr_close_all(1);

    if (misc_register(&_jmgr_misc_device))
    {
        printk(KERN_WARNING "JPU Manager: Couldn't register device.\n");
        return -EBUSY;
    }

	return 0;
}
EXPORT_SYMBOL(jmgr_probe);

int jmgr_remove(struct platform_device *pdev)
{
    misc_deregister(&_jmgr_misc_device);

	if( kidle_task ){
		kthread_stop(kidle_task);
		kidle_task = NULL;
	}

	if( jmgr_data.irq_reged ){
		jmgr_free_irq(jmgr_data.irq, &jmgr_data);
		jmgr_data.irq_reged = 0;
	}

	jmgr_put_clock();

	printk("success :: jmgr thread stopped!! \n");

	return 0;
}
EXPORT_SYMBOL(jmgr_remove);

#if defined(CONFIG_PM)
int jmgr_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i, open_count = 0; 

	if(jmgr_data.dev_opened != 0)
	{
		printk("\n jpu: suspend In DEC(%d/%d/%d/%d), ENC(%d/%d/%d/%d) \n", jmgr_get_close(VPU_DEC), jmgr_get_close(VPU_DEC_EXT), jmgr_get_close(VPU_DEC_EXT2), jmgr_get_close(VPU_DEC_EXT3),
								jmgr_get_close(VPU_ENC), jmgr_get_close(VPU_ENC_EXT), jmgr_get_close(VPU_ENC_EXT2), jmgr_get_close(VPU_ENC_EXT3));

		_jmgr_external_all_close(200);

		open_count = jmgr_data.dev_opened;
		for(i=0; i<open_count; i++) {
			jmgr_disable_clock();
		}
		printk("jpu: suspend Out DEC(%d/%d/%d/%d), ENC(%d/%d/%d/%d) \n\n", jmgr_get_close(VPU_DEC), jmgr_get_close(VPU_DEC_EXT), jmgr_get_close(VPU_DEC_EXT2), jmgr_get_close(VPU_DEC_EXT3),
								jmgr_get_close(VPU_ENC), jmgr_get_close(VPU_ENC_EXT), jmgr_get_close(VPU_ENC_EXT2), jmgr_get_close(VPU_ENC_EXT3));
	}

	return 0;
}
EXPORT_SYMBOL(jmgr_suspend);

int jmgr_resume(struct platform_device *pdev)
{
	int i, open_count = 0; 

	if(jmgr_data.dev_opened != 0){

		open_count = jmgr_data.dev_opened;

		for(i=0; i<open_count; i++) {
			jmgr_enable_clock();
		}
		printk("\n jpu: resume \n\n");
	}

	return 0;
}
EXPORT_SYMBOL(jmgr_resume);
#endif

MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC jpu manager");
MODULE_LICENSE("GPL");

#endif
