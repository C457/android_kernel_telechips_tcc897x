/****************************************************************************
 *   FileName    : jpu_mgr_sys.c
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

//#define CLK_JPU_DEBUG

#define dprintk(msg...)	//printk( "TCC_JPU_MGR_SYS: " msg);
#define detailk(msg...) //printk( "TCC_JPU_MGR_SYS: " msg);
#define err(msg...) printk("TCC_JPU_MGR_SYS[Err]: "msg);
#define info(msg...) printk("TCC_JPU_MGR_SYS[Info]: "msg);

static struct clk *vbus_clk = NULL;
static struct clk *jpu_vbus_clk = NULL; // for pwdn and vBus.

struct jmgr_clk_table {
	unsigned char clk_idx;
	unsigned char enabled;
};

static struct jmgr_clk_table gtJpuClkTable[JPU_MAX] = 
{
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
};

extern int tccxxx_sync_player(int sync);
static int cache_droped = 0;

//////////////////////////////////////////////////////////////////////////////
#if defined(CONFIG_CLOCK_TABLE)
static unsigned int gtJpuSizeTable[] =
{
	720 * 480 , //D1,
	1280 * 720, //HD720P,
	1920 * 1080, //HD1080P,
};
#define SIZE_TABLE_MAX	(sizeof(gtJpuSizeTable)/sizeof(unsigned int))

static struct func_clktbl_t *vpu_2nd_clktbl = NULL;
static struct func_clktbl_t *vpu_clktbl = NULL;

static int _Get_Index_sizeTable(unsigned int Image_Size)
{
	int i;

	//exception process in case of error-size in parser.!!
	if(Image_Size < 64*64)
		Image_Size = 1920*1080;

	for(i=0; i<SIZE_TABLE_MAX; i++)
	{
		if( gtJpuSizeTable[i] >= Image_Size)
		{
			return i;
		}
	}
	
	return (SIZE_TABLE_MAX -1);
}

static int _Get_NextClock(int cur_index)
{
	if(cur_index < (SIZE_TABLE_MAX-1))
		return cur_index+1;

	return (SIZE_TABLE_MAX-1);
}

int jmgr_calcClock(CONTENTS_INFO info)
{
	int index;

	if(!cache_droped)
	{
		tccxxx_sync_player(1);
		cache_droped = 1;
	}

	index = _Get_Index_sizeTable(info.width * info.height);

	if( info.type <= VPU_DEC_EXT3 )
	{
		if( info.framerate > 60 )
			index = _Get_NextClock(SIZE_TABLE_MAX-1);
		else if( info.framerate > 30 )
			index = _Get_NextClock(index);
	}

	if(info.isSWCodec)
		index = _Get_NextClock(SIZE_TABLE_MAX-1);

	gtJpuClkTable[info.type].clk_idx = index;

	return index;
}

static int _Adjust_clock(int type, unsigned int bitrate, int selected_idx, int opened_instance)
{
	int out_2nd_idx;

	if( type <= VPU_DEC_EXT3 ) {
		out_2nd_idx = 0;
		if(bitrate > 30)
			out_2nd_idx = 3;
		else if(bitrate > 20)
			out_2nd_idx = 2;
		else if(bitrate > 10)
			out_2nd_idx = 1;

		if(opened_instance > 1)
			out_2nd_idx = 3;

		switch (selected_idx) {
			case 0:
				vpu_2nd_clktbl = clocktable_get("vpu_480p_clktbl");
				break;
			case 1:
				vpu_2nd_clktbl = clocktable_get("vpu_720p_clktbl");
				break;
			case 2:
			default:
				vpu_2nd_clktbl = clocktable_get("vpu_1080p_clktbl");
				break;
		}
		if (IS_ERR(vpu_2nd_clktbl)) {
			err("vpu secondary clocktable get failed\n");
			vpu_2nd_clktbl = NULL;
		}
		else {
			if (out_2nd_idx >= vpu_2nd_clktbl->num_clktbls)
				out_2nd_idx = vpu_2nd_clktbl->num_clktbls - 1;
		}

		return out_2nd_idx;
	}

	return -1;
}

int jmgr_setClock(int in_idx, CONTENTS_INFO info, unsigned char dev_opened_cnt, unsigned char clk_limitation)
{
	int out_idx, out_2nd_idx;
	int bus_priority_changed = 0;

	if( vpu_clktbl == NULL )
		return 0;
	
	out_idx = in_idx;
	dprintk("[%d,%d]\n[%d,%d]\n[%d,%d]\n[%d,%d]\n[%d,%d]\n", gtJpuClkTable[0].clk_idx, gtJpuClkTable[0].enabled,
				gtJpuClkTable[1].clk_idx, gtJpuClkTable[1].enabled, gtJpuClkTable[2].clk_idx, gtJpuClkTable[2].enabled,
				gtJpuClkTable[3].clk_idx, gtJpuClkTable[3].enabled, gtJpuClkTable[4].clk_idx, gtJpuClkTable[4].enabled);
	
	if(dev_opened_cnt >= 3) //for V2IP + Decode or UVC.
		out_idx = _Get_NextClock(SIZE_TABLE_MAX-1);
	else if(dev_opened_cnt >= 2) //for V2IP
	{
		int i, max_idx;

		max_idx = in_idx;
		for(i=0; i<VPU_MAX; i++)
		{
			if(gtJpuClkTable[i].enabled)
			{
				if(gtJpuClkTable[i].clk_idx > max_idx){
					dprintk("changed[%d] :: %d -> %d\n", i, max_idx, gtJpuClkTable[i].clk_idx);
					max_idx = gtJpuClkTable[i].clk_idx;					
				}
				else
				{
					dprintk("not-changed [%d] :: %d !-> %d\n", i, max_idx, gtJpuClkTable[i].clk_idx);
				}
			}
			else
			{
				dprintk("not-enabled [%d] :: %d !-> %d\n", i, max_idx, gtJpuClkTable[i].clk_idx);
			}
		}		
		out_idx = _Get_NextClock(max_idx);
		dprintk("selected idx = %d, final_idx = %d\n", max_idx, out_idx);
	}

	if (vpu_clktbl) {
		if (out_idx >= vpu_clktbl->num_clktbls)
			out_idx = vpu_clktbl->num_clktbls-1;
	}

	if (vpu_2nd_clktbl) {
		if (vpu_2nd_clktbl->count)
			clocktable_ctrl(vpu_2nd_clktbl, 0, CLKTBL_DISABLE);
		clocktable_put(vpu_2nd_clktbl);
		vpu_2nd_clktbl = NULL;
	}

	out_2nd_idx = _Adjust_clock(info.type, info.bitrate, out_idx, dev_opened_cnt);
	bus_priority_changed = jmgr_BusPrioritySetting(BUS_FOR_VIDEO, info.type);

	if( vpu_2nd_clktbl )
	{
		printk("[%d]-Clk_idx[%d] :: BusPriority(%d) -[%d/%d]-[%d/%d] :: res = %d x %d, fps: %d, mbps: %d \n", info.type, out_idx, bus_priority_changed,
					(vpu_clktbl->clktbls[out_idx].cpu_clk > vpu_2nd_clktbl->clktbls[out_idx].cpu_clk) ? vpu_clktbl->clktbls[out_idx].cpu_clk/1000 : vpu_2nd_clktbl->clktbls[out_idx].cpu_clk/1000, 
					(vpu_clktbl->clktbls[out_idx].mem_clk > vpu_2nd_clktbl->clktbls[out_idx].mem_clk) ? vpu_clktbl->clktbls[out_idx].mem_clk/1000 : vpu_2nd_clktbl->clktbls[out_idx].mem_clk/1000, 
					(vpu_clktbl->clktbls[out_idx].vbus_clk > vpu_2nd_clktbl->clktbls[out_idx].vbus_clk) ? vpu_clktbl->clktbls[out_idx].vbus_clk/1000 : vpu_2nd_clktbl->clktbls[out_idx].vbus_clk/1000, 
					(vpu_clktbl->clktbls[out_idx].hevc_c_clk > vpu_2nd_clktbl->clktbls[out_idx].hevc_c_clk) ? vpu_clktbl->clktbls[out_idx].hevc_c_clk/1000 : vpu_2nd_clktbl->clktbls[out_idx].hevc_c_clk/1000, 
					info.width, info.height, info.framerate, info.bitrate);
	}
	else
	{
		printk("[%d]-Clk_idx[%d] :: BusPriority(%d) -[%d/%d]-[%d/%d] :: res = %d x %d, fps: %d, mbps: %d \n", info.type, out_idx, bus_priority_changed,
					vpu_clktbl->clktbls[out_idx].cpu_clk/1000, 
					vpu_clktbl->clktbls[out_idx].mem_clk/1000, 
					vpu_clktbl->clktbls[out_idx].vbus_clk/1000,
					vpu_clktbl->clktbls[out_idx].hevc_c_clk/1000,
					info.width, info.height, info.framerate, info.bitrate);

	}
	
	if (vpu_clktbl)
		clocktable_ctrl(vpu_clktbl, out_idx, CLKTBL_ENABLE);
	if (vpu_2nd_clktbl)
		clocktable_ctrl(vpu_2nd_clktbl, out_2nd_idx, CLKTBL_ENABLE);	

	return 0;
}
#endif

void jmgr_readyClock(unsigned char count, unsigned int vpu_base)
{
#ifdef CONFIG_CLOCK_TABLE
	if(count == 0)
	{
		vpu_clktbl = clocktable_get("vpu_clktbl");
		if (IS_ERR(vpu_clktbl)){
			vpu_clktbl = NULL;
		}
		else{
			info("Will use clktbl for TCC893x \n");
		}
	}
#endif
}

void jmgr_clearClock(unsigned char count)
{
#if defined(CONFIG_CLOCK_TABLE)
	if(count == 0)
	{
		if (vpu_2nd_clktbl)
			clocktable_ctrl(vpu_2nd_clktbl, 0, CLKTBL_DISABLE);
		if (vpu_clktbl)
			clocktable_ctrl(vpu_clktbl, 0, CLKTBL_DISABLE);
	}
#endif
}

void jmgr_set_clkstatus(vputype type, int on)
{
	dprintk(" @@@@@@@@@@@ jmgr_set_clkstatus %d - %d \n", type, on);

	gtJpuClkTable[type].enabled = on;
}

void jmgr_log_clkstatus(unsigned char closed, unsigned char opened_count, unsigned int accumulated_count)
{
	if(closed){
		printk("jmgr_release Out!! %d'th, total = %d  - DEC(%d/%d/%d/%d):ENC(%d/%d/%d/%d) \n", opened_count, accumulated_count,
				gtJpuClkTable[0].enabled, gtJpuClkTable[1].enabled, gtJpuClkTable[2].enabled, gtJpuClkTable[3].enabled,
				gtJpuClkTable[4].enabled, gtJpuClkTable[5].enabled, gtJpuClkTable[6].enabled, gtJpuClkTable[7].enabled);
	}
	else{
		dprintk("jmgr_open Out!! %d'th, total = %d  - DEC(%d/%d/%d/%d):ENC(%d/%d/%d/%d) \n", opened_count, accumulated_count,
				gtJpuClkTable[0].enabled, gtJpuClkTable[1].enabled, gtJpuClkTable[2].enabled, gtJpuClkTable[3].enabled,
				gtJpuClkTable[4].enabled, gtJpuClkTable[5].enabled, gtJpuClkTable[6].enabled, gtJpuClkTable[7].enabled);
	}
}

void jmgr_enable_clock(void)
{
	if (vbus_clk)
		clk_prepare_enable(vbus_clk);	
	if (jpu_vbus_clk)
		clk_prepare_enable(jpu_vbus_clk);
#ifdef CLK_JPU_DEBUG
	printk("jpu clk(0x%x) -> enabled :: VBUS(0x%x) / PWDN@VBus(0x%x) / SWR@VBus(0x%x)\n", HWCLK_BASE, 
			vetc_reg_read(HWCLK_BASE, VBUS),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x00),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x04));
#endif
}

void jmgr_disable_clock(void)
{	
	if (jpu_vbus_clk)
		clk_disable_unprepare(jpu_vbus_clk);
#if !defined(VBUS_CLK_ALWAYS_ON)
	if (vbus_clk)
		clk_disable_unprepare(vbus_clk);	
#endif
	
#ifdef CLK_JPU_DEBUG
	printk("jpu clk(0x%x) -> disabled :: VBUS(0x%x) / PWDN@VBus(0x%x) / SWR@VBus(0x%x)\n", HWCLK_BASE, 
			vetc_reg_read(HWCLK_BASE, VBUS),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x00),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x04));
#endif	
}

void jmgr_get_clock(struct device_node *node)
{
    if(node == NULL) {
        printk("device node is null\n");
    }

    vbus_clk = of_clk_get(node, 0);
    BUG_ON(IS_ERR(vbus_clk));

	jpu_vbus_clk = of_clk_get(node, 1);
	BUG_ON(IS_ERR(jpu_vbus_clk));

}

void jmgr_put_clock(void)
{
	if (vbus_clk) {
		clk_put(vbus_clk);
		vbus_clk = NULL;
	}	
	if (jpu_vbus_clk) {
		clk_put(jpu_vbus_clk);
		jpu_vbus_clk = NULL;
	}
}

void jmgr_enable_irq(unsigned int irq)
{
	enable_irq(irq);
}

void jmgr_disable_irq(unsigned int irq)
{
	disable_irq(irq);
}

void jmgr_free_irq(unsigned int irq, void * dev_id) {
	free_irq(irq, dev_id);
}

int jmgr_request_irq(unsigned int irq, irqreturn_t (*handler)(int irq, void *dev_id), unsigned long frags, const char * device, void * dev_id) {
    return request_irq(irq, handler, frags, device, dev_id);
}
unsigned long jmgr_get_int_flags(void)
{
	return IRQ_INT_TYPE;
}

void jmgr_init_interrupt(void)
{
}

int jmgr_BusPrioritySetting(int mode, int type)
{
	return 0;
}

void jmgr_status_clear(unsigned int base_addr)
{
}

void jmgr_init_variable(void)
{
	cache_droped = 0;
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC jpu devices driver");
MODULE_LICENSE("GPL");
#endif
