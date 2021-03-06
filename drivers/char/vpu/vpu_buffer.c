/****************************************************************************
 *   FileName    : vpu_buffer.c
 *   Description : 
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

#include <linux/module.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include "vpu_comm.h"
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
#include <soc/tcc/pmap.h>

//#define VPU_BUF_DEBUG
#ifdef VPU_BUF_DEBUG
#define dprintk(msg...)	printk( "TCC_VPU_MEM : " msg);
#define detailk(msg...)	printk( "TCC_VPU_MEM : " msg);
#else
#define dprintk(msg...)
#define detailk(msg...)
#endif
#define err(msg...) printk("TCC_VPU_MEM [Err]: "msg);

#define dprintk_mem(msg...)	//printk( "TCC_VPU_MEM : " msg);
#define detailk_mem_dec(msg...)	//printk( "TCC_VPU_MEM_dec : " msg);
#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
#define dtailk_mem_dec_ext23(msg...)	//printk( "TCC_VPU_MEM_dec_ext23 : " msg);
#endif
#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
#define dtailk_mem_enc_ext234(msg...)	//printk( "TCC_VPU_MEM_enc_ext234 : " msg);
#endif

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
// case(1-2-3 : on/on - off/on - on/off)
//#define FIXED_STREAM_BUFFER
#define FIXED_PS_SLICE_BUFFER

static MEM_ALLOC_INFO_t gsPs_memInfo[VPU_INST_MAX];
static MEM_ALLOC_INFO_t gsSlice_memInfo[VPU_INST_MAX];
static MEM_ALLOC_INFO_t gsStream_memInfo[VPU_INST_MAX];
#endif

//////////////////////////////////////////////////////////////////////////////
// Memory Management!!
#define ARRAY_MBYTE(x)            ((((x) + (SZ_1M-1))>> 20) << 20)

#define VPU_WORK_BUF_SIZE	(PAGE_ALIGN(WORK_CODE_PARA_BUF_SIZE))
static MEM_ALLOC_INFO_t gsVpuWork_memInfo;
extern int vmgr_opened(void);

#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
static MEM_ALLOC_INFO_t gsHevcWork_memInfo;
extern int hmgr_opened(void);
#define HEVC_WORK_BUF_SIZE	(PAGE_ALIGN(WAVE4_WORK_CODE_BUF_SIZE))
#else
#define HEVC_WORK_BUF_SIZE	0
#endif

#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
static MEM_ALLOC_INFO_t gsVp9Work_memInfo;
extern int vp9mgr_opened(void);
#define VP9_WORK_BUF_SIZE	0//(PAGE_ALIGN(VP9_WORK_CODE_BUF_SIZE))
#else
#define VP9_WORK_BUF_SIZE	0
#endif

#ifdef CONFIG_SUPPORT_TCC_JPU
static MEM_ALLOC_INFO_t gsJpuWork_memInfo;
extern int jmgr_opened(void);
#define JPU_WORK_BUF_SIZE	0//(PAGE_ALIGN(JPU_WORK_CODE_BUF_SIZE))
#else
#define JPU_WORK_BUF_SIZE	0
#endif

#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
#define ENC_HEADER_BUF_SIZE	(PAGE_ALIGN(VPU_ENC_PUT_HEADER_SIZE))
static MEM_ALLOC_INFO_t gsVpuEncSeqheader_memInfo;
#else
#define ENC_HEADER_BUF_SIZE	0
#endif

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
#define VPU_SW_ACCESS_REGION_SIZE ARRAY_MBYTE(VPU_WORK_BUF_SIZE + HEVC_WORK_BUF_SIZE + JPU_WORK_BUF_SIZE + ENC_HEADER_BUF_SIZE +(PAGE_ALIGN(PS_SAVE_SIZE) + PAGE_ALIGN(SLICE_SAVE_SIZE) + PAGE_ALIGN(LARGE_STREAM_BUF_SIZE)))
#else
#define VPU_SW_ACCESS_REGION_SIZE ARRAY_MBYTE(VPU_WORK_BUF_SIZE + HEVC_WORK_BUF_SIZE + JPU_WORK_BUF_SIZE + ENC_HEADER_BUF_SIZE)
#endif

// Regard only the operation of 2 component!!
static pmap_t pmap_video;
static unsigned int ptr_front_addr_mem, ptr_rear_addr_mem, ptr_ext_addr_mem;
static unsigned int sz_front_used_mem, sz_rear_used_mem, sz_ext_used_mem;
static unsigned int sz_remained_mem, sz_enc_mem;

#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
static pmap_t pmap_video_ext;
static unsigned int ptr_ext_front_addr_mem, ptr_ext_rear_addr_mem;
static unsigned int sz_ext_front_used_mem, sz_ext_rear_used_mem;
static unsigned int sz_ext_remained_mem;
#endif

#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
static pmap_t pmap_enc_ext[3];
static unsigned int ptr_enc_ext_addr_mem[3];
static unsigned int sz_enc_ext_used_mem[3];
static unsigned int sz_enc_ext_remained_mem[3];
#endif

static int vmem_allocated_count[VPU_MAX] = {0,};
static MEM_ALLOC_INFO_t vmem_alloc_info[VPU_MAX][20];
struct mutex mem_mutex;
static int only_decmode = 0;

static int vdec_used[VPU_INST_MAX] = {0,};
static int venc_used[VPU_INST_MAX] = {0,};

/////////////////////////////////////////
// page type configuration
#define PAGE_TYPE_MAX	16
static int           g_iMmapPropCnt = 0;
static int           g_aiProperty[PAGE_TYPE_MAX];
static unsigned long g_aulPageOffset[PAGE_TYPE_MAX];

static void _vmem_set_page_type(unsigned int uiAddress, int iProperty)
{
	if( iProperty != 0 )
	{
		int idx = 0;

		while( idx < PAGE_TYPE_MAX ) {
			if( g_aiProperty[idx] == 0 )
				break;
			idx++;
		}

		if( idx < PAGE_TYPE_MAX ) {
			g_aiProperty[idx] = iProperty;
			g_aulPageOffset[idx] = uiAddress / PAGE_SIZE;
			g_iMmapPropCnt++;

			dprintk("_vmem_set_page_type(0x%08X, %d) \r\n"
				   , uiAddress
				   , iProperty
				   );
		}
		else {
			dprintk("_vmem_set_page_type(0x%08X, %d) - FAILED \r\n"
				   , uiAddress
				   , iProperty
				   );
		}
	}
}

static int _vmem_get_page_type(unsigned long ulPageOffset)
{
	int idx = 0;
	int prop = 0;

	while( idx < PAGE_TYPE_MAX ) {
		if( g_aulPageOffset[idx] == ulPageOffset )
			break;
		idx++;
	}

	prop = g_aiProperty[idx];

	g_aiProperty[idx] = 0;
	g_aulPageOffset[idx] = 0;

	g_iMmapPropCnt--;

	return prop;
}

int vmem_get_pgprot(unsigned long ulOldProt, unsigned long ulPageOffset)
{
	mutex_lock(&mem_mutex);
	{
		int ret = 0;
		
		if( g_iMmapPropCnt > 0 )
		{
			int prop = _vmem_get_page_type(ulPageOffset);

			switch( prop )
			{
				case 0:
					dprintk("_vmem_get_pgprot (non-cached) \r\n");
					ret = pgprot_noncached(ulOldProt);
					break;

				case 1:
					dprintk("_vmem_get_pgprot (write-through) \r\n");
					ret = __pgprot_modify(ulOldProt, L_PTE_MT_MASK, L_PTE_MT_WRITETHROUGH);
					break;

				case 2:
					dprintk("_vmem_get_pgprot (dma-coherent) \r\n");
					ret = pgprot_dmacoherent(ulOldProt);
					break;

				case 3:
					dprintk("_vmem_get_pgprot (write-combine) \r\n");
					ret = pgprot_writecombine(ulOldProt);
					break;

				case 4:
				case 5:
				case 6:
				default:
					dprintk("_vmem_get_pgprot (default: non-cached) \r\n");
					ret = pgprot_noncached(ulOldProt);
					break;
			}
		}
		else{
			dprintk("_vmem_get_pgprot (default: non-cached) \r\n");
			ret = pgprot_noncached(ulOldProt);
		}

		mutex_unlock(&mem_mutex);
		return ret;
	}
	
//	mutex_unlock(&mem_mutex);
	return 0;
}
EXPORT_SYMBOL(vmem_get_pgprot);
/////////////////////////////////////////

#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
static unsigned int _vmem_request_phyaddr_dec_ext23(unsigned int request_size, vputype type)
{
	unsigned int curr_phyaddr;

	if(sz_ext_remained_mem < request_size)
	{
		err("type[%d] : insufficient memory : remain(0x%x), request(0x%x) \n", type, sz_ext_remained_mem, request_size);
		return 0;
	}

	if(type == VPU_DEC_EXT2)
	{
		curr_phyaddr = ptr_ext_front_addr_mem;
		ptr_ext_front_addr_mem += request_size;
		sz_ext_front_used_mem += request_size;
		
		dtailk_mem_dec_ext23("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, curr_phyaddr, ptr_ext_front_addr_mem, request_size, sz_ext_front_used_mem);
	}
	else//(type == VPU_DEC_EXT3)
	{
		ptr_ext_rear_addr_mem -= request_size;
		curr_phyaddr = ptr_ext_rear_addr_mem;
		sz_ext_rear_used_mem += request_size;
		
		dtailk_mem_dec_ext23("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, curr_phyaddr, ptr_ext_rear_addr_mem + request_size, request_size, sz_ext_rear_used_mem);
	}

	dtailk_mem_dec_ext23("type[%d] : mem usage = 0x%x/0x%x = Dec_ext2/3(0x%x + 0x%x) !! \n", type, sz_ext_front_used_mem+sz_ext_rear_used_mem, pmap_video_ext.size, sz_ext_front_used_mem, sz_ext_rear_used_mem);
		
	sz_ext_remained_mem -= request_size;
	
	return curr_phyaddr;
}

static int _vmem_release_phyaddr_dec_ext23(unsigned int phyaddr, unsigned int size, vputype type)
{
	//calc remained_size and ptr_addr_mem
	if(type == VPU_DEC_EXT2)
	{	
		dtailk_mem_dec_ext23("type[%d] : release ptr_ext_front_addr_mem = 0x%x -> 0x%x(0x%x - 0x%x) !! \n", type, ptr_ext_front_addr_mem, ptr_ext_front_addr_mem - size, ptr_ext_front_addr_mem, size);

		ptr_ext_front_addr_mem -= size;
		sz_ext_front_used_mem -= size;
		
		if(ptr_ext_front_addr_mem != phyaddr)
		{
			err("type[%d] :: ptr_ext_front_addr_mem release-mem order!! 0x%x != 0x%x \n", type, ptr_ext_front_addr_mem, phyaddr);
		}
	}
	else//(type == VPU_DEC_EXT3)
	{
		dtailk_mem_dec_ext23("type[%d] : release ptr_ext_rear_addr_mem = 0x%x -> 0x%x(0x%x + 0x%x)!! \n", type, ptr_ext_rear_addr_mem, ptr_ext_rear_addr_mem + size, ptr_ext_rear_addr_mem, size);
		
		if(ptr_ext_rear_addr_mem != phyaddr)
		{
			err("type[%d] :: ptr_ext_rear_addr_mem release-mem order!! 0x%x != 0x%x \n", type, ptr_ext_rear_addr_mem, phyaddr);
		}		
		
		ptr_ext_rear_addr_mem += size;
		sz_ext_rear_used_mem -= size;
	}

	sz_ext_remained_mem += size;
	
	return 0;
}
#endif

#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
static unsigned int _vmem_request_phyaddr_enc_ext(unsigned int request_size, vputype type)
{
	unsigned int curr_phyaddr;
	unsigned int enc_idx = type - VPU_ENC_EXT;

	if(sz_enc_ext_remained_mem[enc_idx] < request_size)
	{
		err("type[%d] : insufficient memory : remain(0x%x), request(0x%x) \n", type, sz_enc_ext_remained_mem[enc_idx], request_size);
		return -1;
	}

	curr_phyaddr = ptr_enc_ext_addr_mem[enc_idx];
	ptr_enc_ext_addr_mem[enc_idx] += request_size;
	sz_enc_ext_used_mem[enc_idx] += request_size;
	sz_enc_ext_remained_mem[enc_idx] -= request_size;

	dtailk_mem_enc_ext234("type[%d] : alloc : [%d] = 0x%x ~ 0x%x, 0x%x!!, used 0x%x/0x%x \n", type, enc_idx, curr_phyaddr, ptr_enc_ext_addr_mem[enc_idx], request_size, sz_enc_ext_used_mem[enc_idx], sz_enc_ext_remained_mem[enc_idx]);

	return curr_phyaddr;
}

static int _vmem_release_phyaddr_enc_ext(unsigned int phyaddr, unsigned int size, vputype type)
{
	unsigned int enc_idx = type - VPU_ENC_EXT;

	dtailk_mem_enc_ext234("type[%d] : release ptr_enc_ext_addr_mem = 0x%x -> 0x%x(0x%x - 0x%x) !! \n", type, ptr_enc_ext_addr_mem[enc_idx], ptr_enc_ext_addr_mem[enc_idx] - size, ptr_enc_ext_addr_mem[enc_idx], size);

	ptr_enc_ext_addr_mem[enc_idx] -= size;
	sz_enc_ext_used_mem[enc_idx] -= size;

	if(ptr_enc_ext_addr_mem[enc_idx] != phyaddr)
	{
		err("type[%d] :: ptr_enc_ext_addr_mem release-mem order!! 0x%x != 0x%x \n", type, ptr_enc_ext_addr_mem[enc_idx], phyaddr);
	}

	sz_enc_ext_remained_mem[enc_idx] += size;

	return 0;
}
#endif

static int _vmem_check_allocation_available(char check_type, unsigned int request_size, vputype type)
{
	switch(check_type)
	{
		case 0: // Check total memory which can allocate. 
			if(sz_remained_mem < request_size)
			{
				err("type[%d] : insufficient memory : remain(0x%x), request(0x%x) \n", type, sz_remained_mem, request_size);
				return -1;
			}
			break;

		case 1: // Check encoder memory.
			if( sz_enc_mem < (sz_rear_used_mem + request_size) )
			{
				err("type[%d] : insufficient memory : total(0x%x) for only encoder, allocated(0x%x), request(0x%x) \n", type, sz_enc_mem, sz_rear_used_mem, request_size);
				return -1;
			}
			break;

		case 2: // Check remainning except encoder memory.
			if( (pmap_video.size - sz_enc_mem - VPU_SW_ACCESS_REGION_SIZE) < (sz_ext_used_mem + sz_front_used_mem + request_size) )
			{
				err("type[%d] : insufficient memory : total(0x%x) except encoder, allocated(0x%x), request(0x%x) \n", type, (pmap_video.size - sz_enc_mem), (sz_ext_used_mem + sz_front_used_mem), request_size);
				return -1;
			}
			break;

		default:
			break;
	}

	return 0;
}

static unsigned int _vmem_request_phyaddr(unsigned int request_size, vputype type)
{
	unsigned int curr_phyaddr;

#ifdef CONFIG_VPU_ALLOC_MEM_IN_SPECIFIC_SEQUENCE
	if(_vmem_check_allocation_available(0, request_size, type))
		return 0;

	if(type == VPU_DEC || type == VPU_ENC)
	{
		curr_phyaddr = ptr_front_addr_mem;
		ptr_front_addr_mem += request_size;
		sz_front_used_mem += request_size;
		
		detailk_mem_dec("type[%d] : alloc front = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, curr_phyaddr, ptr_front_addr_mem, request_size, sz_front_used_mem);
	}
	else if(type == VPU_DEC_EXT || type == VPU_ENC_EXT)
	{
		ptr_rear_addr_mem -= request_size;
		curr_phyaddr = ptr_rear_addr_mem;
		sz_rear_used_mem += request_size;
		
		detailk_mem_dec("type[%d] : alloc rear = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, ptr_rear_addr_mem + request_size, curr_phyaddr, request_size, sz_rear_used_mem);
	}	
#else
	if(only_decmode)
	{
		if(_vmem_check_allocation_available(0, request_size, type))
			return 0;
	}
	else
	{
		if(type == VPU_ENC)
		{
			if(_vmem_check_allocation_available(1, request_size, type) < 0)
				return 0;
		}
		else // for Decoder_ext or Decoder.
		{
			if(type == VPU_DEC_EXT) // regarding to encoder.
			{
				if(_vmem_check_allocation_available(2, request_size, type) < 0)
					return 0;
			}
			else // only for 1st Decoder.
			{
				if( vmgr_get_close(VPU_DEC_EXT) && vmgr_get_close(VPU_ENC)
#ifdef CONFIG_SUPPORT_TCC_HEVC
					&& hmgr_get_close(VPU_DEC_EXT) && hmgr_get_close(VPU_ENC)
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
					&& vp9mgr_get_close(VPU_DEC_EXT) && vp9mgr_get_close(VPU_ENC)
#endif
#ifdef CONFIG_SUPPORT_TCC_JPU
					&& jmgr_get_close(VPU_DEC_EXT) && jmgr_get_close(VPU_ENC)
#endif
				)
				{
					if(_vmem_check_allocation_available(0, request_size, type) < 0)
						return 0;
				}
				else // in case Decoder only is opened with Decoder_ext or Encoder.
				{
					if(_vmem_check_allocation_available(2, request_size, type) < 0)
						return 0;
				}
			}
		}
	}

	if(type == VPU_DEC)
	{
		curr_phyaddr = ptr_front_addr_mem;
		ptr_front_addr_mem += request_size;
		sz_front_used_mem += request_size;
		
		detailk_mem_dec("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, curr_phyaddr, ptr_front_addr_mem, request_size, sz_front_used_mem);
	}
	else if(type == VPU_DEC_EXT)
	{
		if(only_decmode)
		{
			ptr_rear_addr_mem -= request_size;
			curr_phyaddr = ptr_rear_addr_mem;
			sz_rear_used_mem += request_size;
			
			detailk_mem_dec("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, ptr_rear_addr_mem + request_size, curr_phyaddr, request_size, sz_rear_used_mem);
		}
		else
		{
			ptr_ext_addr_mem -= request_size;
			curr_phyaddr = ptr_ext_addr_mem;
			sz_ext_used_mem += request_size;
			
			detailk_mem_dec("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, ptr_ext_addr_mem + request_size, curr_phyaddr, request_size, sz_ext_used_mem);
		}
	}
	else//VPU_ENC
	{	
		if(only_decmode){
			err("type[%d] : There is no memory for encoder because of only decoder mode.\n", type );
			return 0;
		}
		
		ptr_rear_addr_mem -= request_size;
		curr_phyaddr = ptr_rear_addr_mem;
		sz_rear_used_mem += request_size;

		detailk_mem_dec("type[%d] : alloc = 0x%x ~ 0x%x, 0x%x!!, used 0x%x \n", type, ptr_rear_addr_mem + request_size, curr_phyaddr, request_size, sz_rear_used_mem);
	}
#endif
	detailk_mem_dec("type[%d] : mem usage = 0x%x/0x%x = Dec(0x%x + 0x%x) + Enc(0x%x) !! \n", type, sz_front_used_mem+sz_rear_used_mem+sz_ext_used_mem, pmap_video.size, sz_front_used_mem, sz_ext_used_mem, sz_rear_used_mem);
		
	sz_remained_mem -= request_size;
	
	return curr_phyaddr;
}

static int _vmem_release_phyaddr(unsigned int phyaddr, unsigned int size, vputype type)
{
#ifdef CONFIG_VPU_ALLOC_MEM_IN_SPECIFIC_SEQUENCE
	if(type == VPU_DEC || type == VPU_ENC)
	{	
		detailk_mem_dec("type[%d] : release ptr_front_addr_mem = 0x%x -> 0x%x(0x%x - 0x%x) !! \n", ptr_front_addr_mem, ptr_front_addr_mem - size, ptr_front_addr_mem, size);

		ptr_front_addr_mem -= size;
		sz_front_used_mem -= size;
		
		if(ptr_front_addr_mem != phyaddr)
		{
			err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_front_addr_mem, phyaddr);
		}
	}
	else if(type == VPU_DEC_EXT || type == VPU_ENC_EXT)
	{
		detailk_mem_dec("type[%d] : release ptr_front_addr_mem = 0x%x -> 0x%x(0x%x + 0x%x)!! \n", ptr_rear_addr_mem, ptr_rear_addr_mem + size, ptr_rear_addr_mem, size);
		
		if(ptr_rear_addr_mem != phyaddr)
		{
			err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_rear_addr_mem, phyaddr);
		}		
		
		ptr_rear_addr_mem += size;
		sz_rear_used_mem -= size;
	}
#else
	//calc remained_size and ptr_addr_mem
	if(type == VPU_DEC)
	{	
		detailk_mem_dec("type[%d] : release ptr_front_addr_mem = 0x%x -> 0x%x(0x%x - 0x%x) !! \n", type, ptr_front_addr_mem, ptr_front_addr_mem - size, ptr_front_addr_mem, size);

		ptr_front_addr_mem -= size;
		sz_front_used_mem -= size;
		
		if(ptr_front_addr_mem != phyaddr)
		{
			err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_front_addr_mem, phyaddr);
		}
	}
	else if(type == VPU_DEC_EXT)
	{
		if(only_decmode)
		{
			detailk_mem_dec("type[%d] : release ptr_front_addr_mem = 0x%x -> 0x%x(0x%x + 0x%x)!! \n", type, ptr_rear_addr_mem, ptr_rear_addr_mem + size, ptr_rear_addr_mem, size);
			
			if(ptr_rear_addr_mem != phyaddr)
			{
				err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_rear_addr_mem, phyaddr);
			}		
			
			ptr_rear_addr_mem += size;
			sz_rear_used_mem -= size;
		}
		else
		{
			detailk_mem_dec("type[%d] : release ptr_ext_addr_mem = 0x%x -> 0x%x(0x%x + 0x%x)!! \n", type, ptr_ext_addr_mem, ptr_ext_addr_mem + size, ptr_ext_addr_mem, size);
			
			if(ptr_ext_addr_mem != phyaddr)
			{
				err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_ext_addr_mem, phyaddr);
			}		
			
			ptr_ext_addr_mem += size;
			sz_ext_used_mem -= size;
		}	
	}
	else//VPU_ENC
	{
		detailk_mem_dec("type[%d] : release ptr_front_addr_mem = 0x%x -> 0x%x(0x%x + 0x%x)!! \n", type, ptr_rear_addr_mem, ptr_rear_addr_mem + size, ptr_rear_addr_mem, size);
	
		if(ptr_rear_addr_mem != phyaddr)
		{
			err("type[%d] :: release-mem order!! 0x%x != 0x%x \n", type, ptr_rear_addr_mem, phyaddr);
		}		

		ptr_rear_addr_mem += size;
		sz_rear_used_mem -= size;
	}
#endif

	sz_remained_mem += size;
	
	return 0;
}

int _vmem_alloc_dedicated_buffer(void)
{	
	int ret = 0;
	unsigned int hevc_alloc_size = 0;
	unsigned int jpu_alloc_size = 0;
	unsigned int vpu_alloc_size = 0;
	unsigned int vp9_alloc_size = 0;
#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
	unsigned int alloc_ps_size = 0;
	unsigned int alloc_slice_size = 0;
#endif

#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
	if( gsHevcWork_memInfo.kernel_remap_addr == 0 )
	{
	// HEVC WORK BUFFER
		gsHevcWork_memInfo.request_size = HEVC_WORK_BUF_SIZE;
		if( gsHevcWork_memInfo.request_size )
		{
			gsHevcWork_memInfo.phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size;
			gsHevcWork_memInfo.kernel_remap_addr	= (unsigned int)ioremap_nocache(gsHevcWork_memInfo.phy_addr, PAGE_ALIGN(gsHevcWork_memInfo.request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc HEVC workbuffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsHevcWork_memInfo.phy_addr, gsHevcWork_memInfo.kernel_remap_addr, gsHevcWork_memInfo.request_size);

			if (gsHevcWork_memInfo.kernel_remap_addr == 0) {
				err("[0x%x] HEVC workbuffer remap ALLOC_MEMORY failed.\n", gsHevcWork_memInfo.kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			//memset((void*)gsHevcWork_memInfo.kernel_remap_addr, 0x00, HEVC_WORK_BUF_SIZE);
			hevc_alloc_size = gsHevcWork_memInfo.request_size;
		}
	}
	else{
		err("[0x%x] HEVC-WorkBuff : already remapped? \n", gsHevcWork_memInfo.kernel_remap_addr );
	}
#endif

#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
	if( gsVp9Work_memInfo.kernel_remap_addr == 0 )
	{
	// VP9 WORK BUFFER
		gsVp9Work_memInfo.request_size = VP9_WORK_BUF_SIZE;
		if( gsVp9Work_memInfo.request_size )
		{
			gsVp9Work_memInfo.phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size;
			gsVp9Work_memInfo.kernel_remap_addr	= (unsigned int)ioremap_nocache(gsVp9Work_memInfo.phy_addr, PAGE_ALIGN(gsVp9Work_memInfo.request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VP9 workbuffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVp9Work_memInfo.phy_addr, gsVp9Work_memInfo.kernel_remap_addr, gsVp9Work_memInfo.request_size);

			if (gsVp9Work_memInfo.kernel_remap_addr == 0) {
				err("[0x%x] VP9 workbuffer remap ALLOC_MEMORY failed.\n", gsVp9Work_memInfo.kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			//memset((void*)gsVp9Work_memInfo.kernel_remap_addr, 0x00, VP9_WORK_BUF_SIZE);
			hevc_alloc_size = gsVp9Work_memInfo.request_size;
		}
	}
	else{
		err("[0x%x] VP9-WorkBuff : already remapped? \n", gsVp9Work_memInfo.kernel_remap_addr );
	}
#endif

#ifdef CONFIG_SUPPORT_TCC_JPU
	if( gsJpuWork_memInfo.kernel_remap_addr == 0 )
	{
	// JPU WORK BUFFER
		gsJpuWork_memInfo.request_size = JPU_WORK_BUF_SIZE;
		if( gsJpuWork_memInfo.request_size )
		{
			gsJpuWork_memInfo.phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size;
			gsJpuWork_memInfo.kernel_remap_addr	= (unsigned int)ioremap_nocache(gsJpuWork_memInfo.phy_addr, PAGE_ALIGN(gsJpuWork_memInfo.request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc JPU workbuffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsJpuWork_memInfo.phy_addr, gsJpuWork_memInfo.kernel_remap_addr, gsJpuWork_memInfo.request_size);

			if (gsJpuWork_memInfo.kernel_remap_addr == 0) {
				err("[0x%x] JPU workbuffer remap ALLOC_MEMORY failed.\n", gsJpuWork_memInfo.kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			//memset((void*)gsJpuWork_memInfo.kernel_remap_addr, 0x00, JPU_WORK_BUF_SIZE);
			jpu_alloc_size = gsJpuWork_memInfo.request_size;
		}
	}
	else{
		err("[0x%x] JPU-WorkBuff : already remapped? \n", gsJpuWork_memInfo.kernel_remap_addr );
	}
#endif

	if( gsVpuWork_memInfo.kernel_remap_addr == 0 )
	{
	// VPU WORK BUFFER
		gsVpuWork_memInfo.request_size = VPU_WORK_BUF_SIZE;
		if( gsVpuWork_memInfo.request_size )
		{
			gsVpuWork_memInfo.phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size;
			gsVpuWork_memInfo.kernel_remap_addr	= (unsigned int)ioremap_nocache(gsVpuWork_memInfo.phy_addr, PAGE_ALIGN(gsVpuWork_memInfo.request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU workbuffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVpuWork_memInfo.phy_addr, gsVpuWork_memInfo.kernel_remap_addr, gsVpuWork_memInfo.request_size);

			if (gsVpuWork_memInfo.kernel_remap_addr == 0) {
				err("[0x%x] VPU workbuffer remap ALLOC_MEMORY failed.\n", gsVpuWork_memInfo.kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			//memset((void*)gsVpuWork_memInfo.kernel_remap_addr, 0x00, WORK_CODE_PARA_BUF_SIZE);
			vpu_alloc_size = gsVpuWork_memInfo.request_size;
		}
	}
	else{
		err("[0x%x] VPU-WorkBuff : already remapped? \n", gsVpuWork_memInfo.kernel_remap_addr );
	}

#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
	if( gsVpuEncSeqheader_memInfo.kernel_remap_addr == 0 )
	{
	// SEQ-HEADER BUFFER FOR ENCODER
		gsVpuEncSeqheader_memInfo.request_size 		= ENC_HEADER_BUF_SIZE;
		if( gsVpuEncSeqheader_memInfo.request_size )
		{
			gsVpuEncSeqheader_memInfo.phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size;
		
			gsVpuEncSeqheader_memInfo.kernel_remap_addr	= (unsigned int)ioremap_nocache(gsVpuEncSeqheader_memInfo.phy_addr, PAGE_ALIGN(gsVpuEncSeqheader_memInfo.request_size/*-PAGE_SIZE*/));
			
			dprintk_mem("alloc VPU seqheader_mem Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVpuEncSeqheader_memInfo.phy_addr, gsVpuEncSeqheader_memInfo.kernel_remap_addr, gsVpuEncSeqheader_memInfo.request_size);
			
			if (gsVpuEncSeqheader_memInfo.kernel_remap_addr == 0) {
				err("[0x%x] VPU seqheader_mem remap ALLOC_MEMORY failed.\n", gsVpuEncSeqheader_memInfo.kernel_remap_addr );
				ret = -1;
				goto Error;
			}
		}
	}
	else{
		err("[0x%x] SeqHeader : already remapped? \n", gsVpuEncSeqheader_memInfo.kernel_remap_addr );
	}
#endif

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
	if( gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0 )
	{
		gsPs_memInfo[VPU_DEC_EXT2].request_size 			= PAGE_ALIGN(PS_SAVE_SIZE);
		if( gsPs_memInfo[VPU_DEC_EXT2].request_size )
		{
			gsPs_memInfo[VPU_DEC_EXT2].phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size + alloc_ps_size + alloc_slice_size;

			gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsPs_memInfo[VPU_DEC_EXT2].phy_addr, PAGE_ALIGN(gsPs_memInfo[VPU_DEC_EXT2].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Ps[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC_EXT2, gsPs_memInfo[VPU_DEC_EXT2].phy_addr, gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr, gsPs_memInfo[VPU_DEC_EXT2].request_size);

			if (gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0) {
				err("[0x%x] VPU Ps[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC_EXT2, gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			alloc_ps_size = gsPs_memInfo[VPU_DEC_EXT2].request_size;
		}
	}
	else{
		err("[0x%x] Ps[%d] : already remapped? \n", VPU_DEC_EXT2, gsPs_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
	}

	if( gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0 )
	{
		gsSlice_memInfo[VPU_DEC_EXT2].request_size 			= PAGE_ALIGN(SLICE_SAVE_SIZE);
		if( gsSlice_memInfo[VPU_DEC_EXT2].request_size )
		{
			gsSlice_memInfo[VPU_DEC_EXT2].phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size + alloc_ps_size + alloc_slice_size;

			gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsSlice_memInfo[VPU_DEC_EXT2].phy_addr, PAGE_ALIGN(gsSlice_memInfo[VPU_DEC_EXT2].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Slice[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC_EXT2, gsSlice_memInfo[VPU_DEC_EXT2].phy_addr, gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr, gsSlice_memInfo[VPU_DEC_EXT2].request_size);

			if (gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0) {
				err("[0x%x] VPU Slice[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC_EXT2, gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			alloc_slice_size = gsSlice_memInfo[VPU_DEC_EXT2].request_size;
		}
	}
	else{
		err("[0x%x] Slice[%d] : already remapped? \n", VPU_DEC_EXT2, gsSlice_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
	}

	if( gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0 )
	{
		gsStream_memInfo[VPU_DEC_EXT2].request_size 			= PAGE_ALIGN(LARGE_STREAM_BUF_SIZE);
		if( gsStream_memInfo[VPU_DEC_EXT2].request_size )
		{
			gsStream_memInfo[VPU_DEC_EXT2].phy_addr 			= ptr_rear_addr_mem + hevc_alloc_size + jpu_alloc_size + vpu_alloc_size + vp9_alloc_size + alloc_ps_size + alloc_slice_size;

			gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsStream_memInfo[VPU_DEC_EXT2].phy_addr, PAGE_ALIGN(gsStream_memInfo[VPU_DEC_EXT2].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Stream[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC_EXT2, gsStream_memInfo[VPU_DEC_EXT2].phy_addr, gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr, gsStream_memInfo[VPU_DEC_EXT2].request_size);

			if (gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr == 0) {
				err("[0x%x] VPU Stream[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC_EXT2, gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
		}
	}
	else{
		err("[0x%x] Stream[%d] : already remapped? \n", VPU_DEC_EXT2, gsStream_memInfo[VPU_DEC_EXT2].kernel_remap_addr );
	}

	alloc_ps_size = alloc_slice_size = 0;

	if( gsPs_memInfo[VPU_DEC].kernel_remap_addr == 0 )
	{
		gsPs_memInfo[VPU_DEC].request_size 			= PAGE_ALIGN(PS_SAVE_SIZE);
		if( gsPs_memInfo[VPU_DEC].request_size )
		{
			gsPs_memInfo[VPU_DEC].phy_addr 			= ptr_ext_rear_addr_mem + alloc_ps_size + alloc_slice_size;

			gsPs_memInfo[VPU_DEC].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsPs_memInfo[VPU_DEC].phy_addr, PAGE_ALIGN(gsPs_memInfo[VPU_DEC].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Ps[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC, gsPs_memInfo[VPU_DEC].phy_addr, gsPs_memInfo[VPU_DEC].kernel_remap_addr, gsPs_memInfo[VPU_DEC].request_size);

			if (gsPs_memInfo[VPU_DEC].kernel_remap_addr == 0) {
				err("[0x%x] VPU Ps[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC, gsPs_memInfo[VPU_DEC].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			alloc_ps_size = gsPs_memInfo[VPU_DEC].request_size;
		}
	}
	else{
		err("[0x%x] Ps[%d] : already remapped? \n", VPU_DEC, gsPs_memInfo[VPU_DEC].kernel_remap_addr );
	}
	
	if( gsSlice_memInfo[VPU_DEC].kernel_remap_addr == 0 )
	{
		gsSlice_memInfo[VPU_DEC].request_size 			= PAGE_ALIGN(SLICE_SAVE_SIZE);
		if( gsSlice_memInfo[VPU_DEC].request_size )
		{
			gsSlice_memInfo[VPU_DEC].phy_addr 			= ptr_ext_rear_addr_mem + alloc_ps_size + alloc_slice_size;

			gsSlice_memInfo[VPU_DEC].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsSlice_memInfo[VPU_DEC].phy_addr, PAGE_ALIGN(gsSlice_memInfo[VPU_DEC].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Slice[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC, gsSlice_memInfo[VPU_DEC].phy_addr, gsSlice_memInfo[VPU_DEC].kernel_remap_addr, gsSlice_memInfo[VPU_DEC].request_size);

			if (gsSlice_memInfo[VPU_DEC].kernel_remap_addr == 0) {
				err("[0x%x] VPU Slice[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC, gsSlice_memInfo[VPU_DEC].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
			alloc_slice_size = gsSlice_memInfo[VPU_DEC].request_size;
		}
	}
	else{
		err("[0x%x] Slice[%d] : already remapped? \n", VPU_DEC, gsSlice_memInfo[VPU_DEC].kernel_remap_addr );
	}

	if( gsStream_memInfo[VPU_DEC].kernel_remap_addr == 0 )
	{
		gsStream_memInfo[VPU_DEC].request_size 			= PAGE_ALIGN(LARGE_STREAM_BUF_SIZE);
		if( gsStream_memInfo[VPU_DEC].request_size )
		{
			gsStream_memInfo[VPU_DEC].phy_addr 			= ptr_ext_rear_addr_mem + alloc_ps_size + alloc_slice_size;

			gsStream_memInfo[VPU_DEC].kernel_remap_addr	= (unsigned int)ioremap_nocache(gsStream_memInfo[VPU_DEC].phy_addr, PAGE_ALIGN(gsStream_memInfo[VPU_DEC].request_size/*-PAGE_SIZE*/));

			dprintk_mem("alloc VPU Stream[%d] Info buffer :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", VPU_DEC, gsStream_memInfo[VPU_DEC].phy_addr, gsStream_memInfo[VPU_DEC].kernel_remap_addr, gsStream_memInfo[VPU_DEC].request_size);

			if (gsStream_memInfo[VPU_DEC].kernel_remap_addr == 0) {
				err("[0x%x] VPU Stream[%d] remap ALLOC_MEMORY failed.\n", VPU_DEC, gsStream_memInfo[VPU_DEC].kernel_remap_addr );
				ret = -1;
				goto Error;
			}
		}
	}
	else{
		err("[0x%x] Stream[%d] : already remapped? \n", VPU_DEC, gsStream_memInfo[VPU_DEC].kernel_remap_addr );
	}	
#endif

Error:
	return ret;
}

void _vmem_free_dedicated_buffer(void)
{
	// Memory Management!!
#ifdef CONFIG_SUPPORT_TCC_JPU
	if( gsJpuWork_memInfo.kernel_remap_addr != 0 )
	{
		dprintk("free JPU workbuffer:: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsJpuWork_memInfo.phy_addr, gsJpuWork_memInfo.kernel_remap_addr, gsJpuWork_memInfo.request_size);
		iounmap((void*)gsJpuWork_memInfo.kernel_remap_addr);
		memset(&gsJpuWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
	}
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
	if( gsHevcWork_memInfo.kernel_remap_addr != 0 )
	{
		dprintk("free HEVC workbuffer:: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsHevcWork_memInfo.phy_addr, gsHevcWork_memInfo.kernel_remap_addr, gsHevcWork_memInfo.request_size);
		iounmap((void*)gsHevcWork_memInfo.kernel_remap_addr);
		memset(&gsHevcWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
	}
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
	if( gsVp9Work_memInfo.kernel_remap_addr != 0 )
	{
		dprintk("free VP9 workbuffer:: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVp9Work_memInfo.phy_addr, gsVp9Work_memInfo.kernel_remap_addr, gsVp9Work_memInfo.request_size);
		iounmap((void*)gsVp9Work_memInfo.kernel_remap_addr);
		memset(&gsVp9Work_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
	}
#endif

#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)	
	if( gsVpuEncSeqheader_memInfo.kernel_remap_addr != 0 )
	{
		dprintk("free SeqHeader :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVpuEncSeqheader_memInfo.phy_addr, gsVpuEncSeqheader_memInfo.kernel_remap_addr, gsVpuEncSeqheader_memInfo.request_size);
		iounmap((void*)gsVpuEncSeqheader_memInfo.kernel_remap_addr);
		memset(&gsVpuEncSeqheader_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
	}
#endif

	if( gsVpuWork_memInfo.kernel_remap_addr != 0 )
	{
		dprintk("free workbuffer:: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsVpuWork_memInfo.phy_addr, gsVpuWork_memInfo.kernel_remap_addr, gsVpuWork_memInfo.request_size);
		iounmap((void*)gsVpuWork_memInfo.kernel_remap_addr);
		memset(&gsVpuWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
	}

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
	{
		int type = 0;

		for(type = 0; type < VPU_INST_MAX; type++)
		{
			if( gsPs_memInfo[type].kernel_remap_addr != 0 )
			{
				dprintk("free PS :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsPs_memInfo[type].phy_addr, gsPs_memInfo[type].kernel_remap_addr, gsPs_memInfo[type].request_size);
				iounmap((void*)gsPs_memInfo[type].kernel_remap_addr);
				memset(&gsPs_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
			}
			if( gsSlice_memInfo[type].kernel_remap_addr != 0 )
			{
				dprintk("free Slice :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsSlice_memInfo[type].phy_addr, gsSlice_memInfo[type].kernel_remap_addr, gsSlice_memInfo[type].request_size);
				iounmap((void*)gsSlice_memInfo[type].kernel_remap_addr);
				memset(&gsSlice_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
			}
			if( gsStream_memInfo[type].kernel_remap_addr != 0 )
			{
				dprintk("free Stream :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", gsStream_memInfo[type].phy_addr, gsStream_memInfo[type].kernel_remap_addr, gsStream_memInfo[type].request_size);
				iounmap((void*)gsStream_memInfo[type].kernel_remap_addr);
				memset(&gsStream_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
			}
		}
	}
#endif
}


static unsigned int _vmem_request_workbuff_phyaddr(int codec_type, unsigned int *remapped_addr)
{
	unsigned int phy_address = 0x00;
#ifdef CONFIG_SUPPORT_TCC_JPU // M-Jpeg
	if(codec_type == STD_MJPG)
	{
		*remapped_addr = gsJpuWork_memInfo.kernel_remap_addr;	
		phy_address = gsJpuWork_memInfo.phy_addr;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
	if(codec_type == STD_HEVC)
	{
		*remapped_addr = gsHevcWork_memInfo.kernel_remap_addr;	
		phy_address = gsHevcWork_memInfo.phy_addr;
	}
	else
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
	if(codec_type == STD_VP9)
	{
		*remapped_addr = gsVp9Work_memInfo.kernel_remap_addr;	
		phy_address = gsVp9Work_memInfo.phy_addr;
	}
	else
#endif
	{
		*remapped_addr = gsVpuWork_memInfo.kernel_remap_addr;	
		phy_address = gsVpuWork_memInfo.phy_addr;
	}

	return phy_address;
}

static unsigned int _vmem_request_seqheader_buff_phyaddr(unsigned int *remapped_addr, unsigned int *request_size)
{
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)	
	*remapped_addr = gsVpuEncSeqheader_memInfo.kernel_remap_addr;
	*request_size = gsVpuEncSeqheader_memInfo.request_size;
	
	return gsVpuEncSeqheader_memInfo.phy_addr;
#else
	return 0;
#endif
}

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
static unsigned int _vmem_request_ps_buff_phyaddr(int type, unsigned int *remapped_addr, unsigned int *request_size)
{
	*remapped_addr = gsPs_memInfo[type].kernel_remap_addr;
	*request_size = gsPs_memInfo[type].request_size;

	return gsPs_memInfo[type].phy_addr;
}

static unsigned int _vmem_request_slice_buff_phyaddr(int type, unsigned int *remapped_addr, unsigned int *request_size)
{
	*remapped_addr = gsSlice_memInfo[type].kernel_remap_addr;
	*request_size = gsSlice_memInfo[type].request_size;

	return gsSlice_memInfo[type].phy_addr;
}

static unsigned int _vmem_request_stream_buff_phyaddr(int type, unsigned int *remapped_addr, unsigned int *request_size)
{
	*remapped_addr = gsStream_memInfo[type].kernel_remap_addr;
	*request_size = gsStream_memInfo[type].request_size;

	return gsStream_memInfo[type].phy_addr;
}
#endif

void _vmem_init_memory_info(void)
{
	pmap_get_info("video", &pmap_video);

	ptr_front_addr_mem = pmap_video.base;
	sz_remained_mem = pmap_video.size;

	ptr_rear_addr_mem = ptr_front_addr_mem + sz_remained_mem - VPU_SW_ACCESS_REGION_SIZE;
	sz_remained_mem -= VPU_SW_ACCESS_REGION_SIZE;

#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
	sz_enc_mem = ENC_MEM_LEN;
	only_decmode = 0;
#else
	sz_enc_mem = 0;
	only_decmode = 1;
#endif
	ptr_ext_addr_mem = ptr_rear_addr_mem - sz_enc_mem;
	sz_front_used_mem = sz_rear_used_mem = sz_ext_used_mem = 0;

#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
// Additional decoder :: ext2,3
	pmap_get_info("video_ext", &pmap_video_ext);

	ptr_ext_front_addr_mem = pmap_video_ext.base;
	sz_ext_remained_mem = pmap_video_ext.size;
#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
	ptr_ext_rear_addr_mem = ptr_ext_front_addr_mem + sz_ext_remained_mem - (PAGE_ALIGN(PS_SAVE_SIZE) + PAGE_ALIGN(SLICE_SAVE_SIZE) + PAGE_ALIGN(LARGE_STREAM_BUF_SIZE));
	sz_ext_remained_mem -= (PAGE_ALIGN(PS_SAVE_SIZE) + PAGE_ALIGN(SLICE_SAVE_SIZE) + PAGE_ALIGN(LARGE_STREAM_BUF_SIZE));
#else
	ptr_ext_rear_addr_mem = ptr_ext_front_addr_mem + sz_ext_remained_mem;
#endif
	sz_ext_front_used_mem = sz_ext_rear_used_mem = 0;
#endif
#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
// Additional encoder :: ext, ext2, ext3
	pmap_get_info("enc_ext", &pmap_enc_ext[0]);
	ptr_enc_ext_addr_mem[0] = pmap_enc_ext[0].base;
	sz_enc_ext_remained_mem[0] = pmap_enc_ext[0].size;
	sz_enc_ext_used_mem[0] = 0;
#endif
#if defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
	pmap_get_info("enc_ext2", &pmap_enc_ext[1]);
	ptr_enc_ext_addr_mem[1] = pmap_enc_ext[1].base;
	sz_enc_ext_remained_mem[1] = pmap_enc_ext[1].size;
	sz_enc_ext_used_mem[1] = 0;
#endif
#if defined(CONFIG_VENC_CNT_4)
	pmap_get_info("enc_ext3", &pmap_enc_ext[2]);
	ptr_enc_ext_addr_mem[2] = pmap_enc_ext[2].base;
	sz_enc_ext_remained_mem[2] = pmap_enc_ext[2].size;
	sz_enc_ext_used_mem[2] = 0;
#endif

	memset(vmem_allocated_count, 0x00, sizeof(vmem_allocated_count));
	memset(vmem_alloc_info, 0x00, sizeof(vmem_alloc_info));
	memset(vdec_used, 0x00, sizeof(vdec_used));
	memset(venc_used, 0x00, sizeof(venc_used));
}

int vmem_proc_alloc_memory(int codec_type, MEM_ALLOC_INFO_t *alloc_info, vputype type)
{
	mutex_lock(&mem_mutex);
	dprintk("type[%d]-buffer[%d] : vmgr_proc_alloc_memory!! \n", type, alloc_info->buffer_type);

/******************************** START :: Dedicated memory region   **************************************/
#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
  #ifdef FIXED_PS_SLICE_BUFFER
	if( alloc_info->buffer_type == BUFFER_PS )
	{
		if( (alloc_info->phy_addr = _vmem_request_ps_buff_phyaddr( type, &(alloc_info->kernel_remap_addr), &(alloc_info->request_size)) ) != 0 )
		{
			goto Success;
		}
	}
	else if( alloc_info->buffer_type == BUFFER_SLICE )
	{
		if( (alloc_info->phy_addr = _vmem_request_slice_buff_phyaddr( type, &(alloc_info->kernel_remap_addr), &(alloc_info->request_size)) ) != 0 )
		{
			goto Success;
		}
	}
	else
  #endif
  #ifdef FIXED_STREAM_BUFFER
	if( alloc_info->buffer_type == BUFFER_STREAM )
	{
		if( (alloc_info->phy_addr = _vmem_request_stream_buff_phyaddr( type, &(alloc_info->kernel_remap_addr), &(alloc_info->request_size)) ) != 0 )
		{
			goto Success;
		}
	}
  #endif
#endif
	if( alloc_info->buffer_type == BUFFER_WORK )
	{
		if( (alloc_info->phy_addr = _vmem_request_workbuff_phyaddr( codec_type, &(alloc_info->kernel_remap_addr)) ) == 0 )
		{
			err("type[%d]-buffer[%d] : [0x%x] BUFFER_WORK ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
			goto Error;
		}
	}
	else if( alloc_info->buffer_type == BUFFER_SEQHEADER && type == VPU_ENC ) //Encoder
	{
		if( (alloc_info->phy_addr = _vmem_request_seqheader_buff_phyaddr( &(alloc_info->kernel_remap_addr), &(alloc_info->request_size) )) == 0 )
		{
			err("type[%d]-buffer[%d] : [0x%x] BUFFER_SEQHEADER ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
			goto Error;
		}
	}
/******************************** END :: Dedicated memory region   **************************************/
	else
	{
#ifdef CONFIG_VPU_ALLOC_MEM_IN_SPECIFIC_SEQUENCE
		if( (alloc_info->phy_addr = _vmem_request_phyaddr( alloc_info->request_size, type)) == 0 )
		{
			err("type[%d]-buffer[%d] : [0x%x] ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
			goto Error;
		}
#else
		if( type == VPU_DEC_EXT2 || type == VPU_DEC_EXT3 )
		{
#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
			if( (alloc_info->phy_addr = _vmem_request_phyaddr_dec_ext23( alloc_info->request_size, type)) == 0 )
			{
				err("type[%d]-buffer[%d] : [0x%x] ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
				goto Error;
			}
#endif
		}
		else if( type == VPU_ENC_EXT || type == VPU_ENC_EXT2 || type == VPU_ENC_EXT3 )
		{
#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
			if( (alloc_info->phy_addr = _vmem_request_phyaddr_enc_ext( alloc_info->request_size, type)) == 0 )
			{
				err("type[%d]-buffer[%d] : [0x%x] ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
				goto Error;
			}
#endif
		}
		else
		{
			if( (alloc_info->phy_addr = _vmem_request_phyaddr( alloc_info->request_size, type)) == 0 )
			{
				err("type[%d]-buffer[%d] : [0x%x] ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr );
				goto Error;
			}
		}
#endif
		alloc_info->kernel_remap_addr = 0x0;
		if( alloc_info->buffer_type != BUFFER_FRAMEBUFFER )
		{
			alloc_info->kernel_remap_addr = (unsigned int)ioremap_nocache(alloc_info->phy_addr, PAGE_ALIGN(alloc_info->request_size/*-PAGE_SIZE*/));
			if (alloc_info->kernel_remap_addr == 0) {
				err("type[%d]-buffer[%d] : phy[0x%x - 0x%x] remap ALLOC_MEMORY failed.\n", type, alloc_info->buffer_type, alloc_info->phy_addr, PAGE_ALIGN(alloc_info->request_size/*-PAGE_SIZE*/) );
				memcpy(&(vmem_alloc_info[type][vmem_allocated_count[type]]), alloc_info, sizeof(MEM_ALLOC_INFO_t));
				vmem_allocated_count[type] += 1;
				goto Error;
			}
		}

		dprintk_mem("type[%d]-buffer[%d] : alloc addr[%d] :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", type, alloc_info->buffer_type, vmem_allocated_count[type], alloc_info->phy_addr, alloc_info->kernel_remap_addr, alloc_info->request_size);
		memcpy(&(vmem_alloc_info[type][vmem_allocated_count[type]]), alloc_info, sizeof(MEM_ALLOC_INFO_t));
		vmem_allocated_count[type] += 1;
	}

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
Success:
#endif
	if( alloc_info->buffer_type == BUFFER_STREAM )
		_vmem_set_page_type(alloc_info->phy_addr, 1);

	mutex_unlock(&mem_mutex);	
	return 0;
	
Error:
	mutex_unlock(&mem_mutex);
	return -EFAULT;
}

int vmem_proc_free_memory(vputype type)
{
	mutex_lock(&mem_mutex);
	{
		int i;

		dprintk("type[%d] : vmgr_proc_free_memory!! \n", type);

		for(i=vmem_allocated_count[type]; i>0; i--)
		{	
			//if(vmem_alloc_info[type][i-1].kernel_remap_addr != 0)
			{
				if( vmem_alloc_info[type][i-1].kernel_remap_addr != 0 )
					iounmap((void*)vmem_alloc_info[type][i-1].kernel_remap_addr);
#ifdef CONFIG_VPU_ALLOC_MEM_IN_SPECIFIC_SEQUENCE
				_vmem_release_phyaddr(vmem_alloc_info[type][i-1].phy_addr, vmem_alloc_info[type][i-1].request_size, type);
#else
				if(type == VPU_DEC_EXT2 || type == VPU_DEC_EXT3){
	#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
					_vmem_release_phyaddr_dec_ext23(vmem_alloc_info[type][i-1].phy_addr, vmem_alloc_info[type][i-1].request_size, type);
	#endif
				}
				else if( type == VPU_ENC_EXT || type == VPU_ENC_EXT2 || type == VPU_ENC_EXT3 ){
	#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
					_vmem_release_phyaddr_enc_ext(vmem_alloc_info[type][i-1].phy_addr, vmem_alloc_info[type][i-1].request_size, type);
	#endif
				}
				else{
					_vmem_release_phyaddr(vmem_alloc_info[type][i-1].phy_addr, vmem_alloc_info[type][i-1].request_size, type);
				}
#endif
				dprintk_mem("type[%d]-buffer[%d] : free addr[%d] :: phy = 0x%x, remap = 0x%x, size = 0x%x !! \n", type, vmem_alloc_info[type][i-1].buffer_type, i-1, vmem_alloc_info[type][i-1].phy_addr, vmem_alloc_info[type][i-1].kernel_remap_addr, vmem_alloc_info[type][i-1].request_size);
			}
		}

		vmem_allocated_count[type] = 0;
		memset(vmem_alloc_info[type], 0x00, sizeof(MEM_ALLOC_INFO_t));
	}
	mutex_unlock(&mem_mutex);

	return 0;
}

unsigned int vmem_get_free_memory(vputype type)
{
	if( type == VPU_DEC )
	{
		if( vmgr_get_close(VPU_DEC_EXT) && vmgr_get_close(VPU_ENC)
#ifdef CONFIG_SUPPORT_TCC_HEVC
			&& hmgr_get_close(VPU_DEC_EXT) && hmgr_get_close(VPU_ENC)
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9
			&& vp9mgr_get_close(VPU_DEC_EXT) && vp9mgr_get_close(VPU_ENC)
#endif
#ifdef CONFIG_SUPPORT_TCC_JPU
			&& jmgr_get_close(VPU_DEC_EXT) && jmgr_get_close(VPU_ENC)
#endif
		)
		{
			return sz_remained_mem;
		}
		else
		{
			if(only_decmode)
				return sz_remained_mem;
			else
				return (pmap_video.size - sz_enc_mem - VPU_SW_ACCESS_REGION_SIZE) - (sz_ext_used_mem + sz_front_used_mem);
		}
	}
	else if( type == VPU_DEC_EXT )
	{
		if(only_decmode)
			return sz_remained_mem;
		else
			return (pmap_video.size - sz_enc_mem - VPU_SW_ACCESS_REGION_SIZE) - (sz_ext_used_mem + sz_front_used_mem);
	}
	else if( type == VPU_ENC )
	{
		return sz_enc_mem-sz_rear_used_mem;
	}
	else if( type == VPU_DEC_EXT2 || type == VPU_DEC_EXT3 )
	{
#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
		return sz_ext_remained_mem;
#else
		return 0;
#endif
	}
	else if( type == VPU_ENC_EXT || type == VPU_ENC_EXT2 || type == VPU_ENC_EXT3 )
	{
#if defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)
		return sz_enc_ext_remained_mem[type-VPU_ENC_EXT];
#else
		return 0;
#endif
	}

	return sz_remained_mem;
}
EXPORT_SYMBOL(vmem_get_free_memory);

unsigned int vmem_get_freemem_size(vputype type)
{
	if( type <= VPU_DEC_EXT3 )
	{
		printk("type[%d] mem info for vpu :: remain(0x%x : 0x%x), used mem(0x%x/0x%x, 0x%x : 0x%x/0x%x) \n",
					type,
					sz_remained_mem,
#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
					sz_ext_remained_mem,
#else
					0,
#endif
					sz_front_used_mem, sz_ext_used_mem, sz_rear_used_mem,
#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
					sz_ext_front_used_mem, sz_ext_rear_used_mem
#else
					0, 0
#endif
		);
	}
	else if( /*VPU_ENC <= type && */type < VPU_MAX )
	{
		printk("type[%d] mem info for vpu :: remain(0x%x : 0x%x : 0x%x : 0x%x), used mem(0x%x : 0x%x : 0x%x : 0x%x) \n",
					type,
					sz_enc_mem - sz_rear_used_mem,
#if defined(CONFIG_VENC_CNT_4)
					sz_enc_ext_remained_mem[0],	sz_enc_ext_remained_mem[1], sz_enc_ext_remained_mem[2],
#elif defined(CONFIG_VENC_CNT_3)
					sz_enc_ext_remained_mem[0],	sz_enc_ext_remained_mem[1],	0,
#elif defined(CONFIG_VENC_CNT_2)
					sz_enc_ext_remained_mem[0],	0, 0,
#else
					0, 0, 0,
#endif
					sz_rear_used_mem,
#if defined(CONFIG_VENC_CNT_4)
					sz_enc_ext_used_mem[0],	sz_enc_ext_used_mem[1],	sz_enc_ext_used_mem[3]
#elif defined(CONFIG_VENC_CNT_3)
					sz_enc_ext_used_mem[0],	sz_enc_ext_used_mem[1],	0
#elif defined(CONFIG_VENC_CNT_2)
					sz_enc_ext_used_mem[0],	0, 0
#else
					0, 0, 0
#endif
		);
	}
	else{
		err("mem_info :: unKnown type(%d)\n", type);
		return 0;
	}

	return vmem_get_free_memory(type);
}

void vmem_reinit(void)
{
	mutex_lock(&mem_mutex);
	if((vmgr_opened() == 0)
#ifdef CONFIG_SUPPORT_TCC_JPU
		&& (jmgr_opened() == 0)
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
		&& (hmgr_opened() == 0)
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
		&& (vp9mgr_opened() == 0)
#endif
	){
		_vmem_init_memory_info();
	}
	mutex_unlock(&mem_mutex);
}

void vmem_init(void)
{
	mutex_init(&mem_mutex);

//////////////////////////////////////
// Memory Management!!
	memset(&gsVpuWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
#ifdef CONFIG_SUPPORT_TCC_JPU
	memset(&gsJpuWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
#endif
#ifdef CONFIG_SUPPORT_TCC_HEVC // HEVC
	memset(&gsHevcWork_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
#endif
#ifdef CONFIG_SUPPORT_TCC_VP9 // VP9
	memset(&gsVp9Work_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
#endif
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)	
	memset(&gsVpuEncSeqheader_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t));
#endif

#if defined(CONFIG_TEST_VPU_DRAM_INTLV)
	memset(&gsPs_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t)*VPU_INST_MAX);
	memset(&gsSlice_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t)*VPU_INST_MAX);
	memset(&gsStream_memInfo, 0x00, sizeof(MEM_ALLOC_INFO_t)*VPU_INST_MAX);
#endif

	printk("_vmem_init_memory_info (work_total : 0x%x) \n", VPU_SW_ACCESS_REGION_SIZE);
	_vmem_init_memory_info();
	_vmem_alloc_dedicated_buffer();
//////////////////////////////////////
}

void vmem_deinit(void)
{
	_vmem_free_dedicated_buffer();
}

void vmem_set_only_decode_mode(int bDec_only)
{
	mutex_lock(&mem_mutex);
	{	
		if( vmem_allocated_count[VPU_DEC_EXT] == 0 && vmem_allocated_count[VPU_ENC] == 0){
#if defined(CONFIG_VENC_CNT_1) || defined(CONFIG_VENC_CNT_2) || defined(CONFIG_VENC_CNT_3) || defined(CONFIG_VENC_CNT_4)	
			only_decmode = bDec_only;
#else
			only_decmode = 1;
#endif
			printk("Changed alloc_mode(%d) \n", only_decmode);
		}
		else{
			dprintk("can't change mode. (%d/%d) \n", vmem_allocated_count[VPU_DEC_EXT], vmem_allocated_count[VPU_ENC]);
		}
	}
	mutex_unlock(&mem_mutex);
}

void vdec_get_instance(int *nIdx)
{	
	mutex_lock(&mem_mutex);
	{
		int i, nInstance = -1;

		if( *nIdx >= 0 && *nIdx < VPU_INST_MAX ) // check instance that they want.
		{
			if( vdec_used[*nIdx] == 0 && (vmem_get_free_memory(*nIdx+VPU_DEC) > 0) )
				nInstance = *nIdx;
		}

		if( nInstance < 0 )
		{
			for(i=0; i<VPU_INST_MAX; i++) {
				if(vdec_used[i] == 0){
					nInstance = i;
					break;
				}
			}
		}

		if(nInstance >= 0)
			vdec_used[nInstance] = 1;
		else
			err("failed to get new instance for decoder(%d) \n", nInstance);

		*nIdx = nInstance;
	}
	mutex_unlock(&mem_mutex);
}

void vdec_check_instance_available(unsigned int *nAvailable_Inst)
{	
	mutex_lock(&mem_mutex);
	{
		int nAvailable_Instance = 0;
		int i = 0;

		for(i=0; i<VPU_INST_MAX; i++) {
			if(vdec_used[i] == 0 && (vmem_get_free_memory(i+VPU_DEC) > 0)){
				nAvailable_Instance++;
			}
		}
		*nAvailable_Inst = nAvailable_Instance;
	}
	mutex_unlock(&mem_mutex);
}

void vdec_clear_instance(int nIdx)
{
	mutex_lock(&mem_mutex);
	{
		if(nIdx >= 0)
			vdec_used[nIdx] = 0;
		else
			err("failed to clear instance for decoder(%d) \n", nIdx);

	}
	mutex_unlock(&mem_mutex);
}
EXPORT_SYMBOL(vdec_clear_instance);

void venc_get_instance(int *nIdx)
{	
	mutex_lock(&mem_mutex);
	{
		int i, nInstance = -1;

		if( *nIdx >= 0 && *nIdx < VPU_INST_MAX ) // check instance that they want.
		{
			if( venc_used[*nIdx] == 0 && vmem_get_free_memory(*nIdx+VPU_ENC) > 0 )
				nInstance = *nIdx;
		}

		if( nInstance < 0 )
		{
			for(i=0; i<VPU_INST_MAX; i++) {
				if(venc_used[i] == 0){
					nInstance = i;
					break;
				}
			}
		}

		if(nInstance >= 0)
			venc_used[nInstance] = 1;
		else
			err("failed to get new instance for encoder(%d) \n", nInstance);

		*nIdx = nInstance;
	}
	mutex_unlock(&mem_mutex);
}

void venc_check_instance_available(unsigned int *nAvailable_Inst)
{	
	mutex_lock(&mem_mutex);
	{
		int nAvailable_Instance = 0;
		int i = 0;

		for(i=0; i<VPU_INST_MAX; i++) {
			if(venc_used[i] == 0 && (vmem_get_free_memory(i+VPU_ENC) > 0)){
				nAvailable_Instance++;
			}
		}
		*nAvailable_Inst = nAvailable_Instance;
	}
	mutex_unlock(&mem_mutex);
}


void venc_clear_instance(int nIdx)
{
	mutex_lock(&mem_mutex);
	{
		if(nIdx >= 0)
			venc_used[nIdx] = 0;
		else
			err("failed to clear instance for encoder(%d) \n", nIdx);

	}
	mutex_unlock(&mem_mutex);
}
EXPORT_SYMBOL(venc_clear_instance);

