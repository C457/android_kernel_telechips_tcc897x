/****************************************************************************
 *   FileName    : hevc_mgr_sys.c
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
#ifdef CONFIG_SUPPORT_TCC_HEVC

#include "hevc_mgr_sys.h"

//#define CLK_HEVC_DEBUG

#define dprintk(msg...)	//printk( "TCC_HEVC_MGR_SYS: " msg);
#define detailk(msg...) //printk( "TCC_HEVC_MGR_SYS: " msg);
#define err(msg...) printk("TCC_HEVC_MGR_SYS[Err]: "msg);
#define info(msg...) printk("TCC_HEVC_MGR_SYS[Info]: "msg);

static struct clk *vbus_clk = NULL;
static struct clk *hevc_c_clk = NULL;
static struct clk *hevc_v_clk = NULL;
static struct clk *hevc_b_clk = NULL;
static struct clk *hevc_vbus_clk = NULL; // for pwdn and vBus.
#if defined(CONFIG_ARCH_TCC898X)
static struct clk *hevc_vcore_clk = NULL; // for pwdn and vBus.
#endif

struct hmgr_clk_table {
	unsigned char clk_idx;
	unsigned char enabled;
};

static struct hmgr_clk_table gtHevcClkTable[HEVC_MAX] = 
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
static unsigned int gtHevcSizeTable[] =
{
	720 * 480 , //D1,
	1280 * 720, //HD720P,
	1920 * 1080, //HD1080P,
};
#define SIZE_TABLE_MAX	(sizeof(gtHevcSizeTable)/sizeof(unsigned int))

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
		if( gtHevcSizeTable[i] >= Image_Size)
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

int hmgr_calcClock(CONTENTS_INFO info)
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

	gtHevcClkTable[info.type].clk_idx = index;

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

int hmgr_setClock(int in_idx, CONTENTS_INFO info, unsigned char dev_opened_cnt, unsigned char clk_limitation)
{
	int out_idx, out_2nd_idx;
	int bus_priority_changed = 0;

	if( vpu_clktbl == NULL )
		return 0;
	
	out_idx = in_idx;
	dprintk("[%d,%d]\n[%d,%d]\n[%d,%d]\n[%d,%d]\n[%d,%d]\n", gtHevcClkTable[0].clk_idx, gtHevcClkTable[0].enabled,
				gtHevcClkTable[1].clk_idx, gtHevcClkTable[1].enabled, gtHevcClkTable[2].clk_idx, gtHevcClkTable[2].enabled,
				gtHevcClkTable[3].clk_idx, gtHevcClkTable[3].enabled, gtHevcClkTable[4].clk_idx, gtHevcClkTable[4].enabled);
	
	if(dev_opened_cnt >= 3) //for V2IP + Decode or UVC.
		out_idx = _Get_NextClock(SIZE_TABLE_MAX-1);
	else if(dev_opened_cnt >= 2) //for V2IP
	{
		int i, max_idx;

		max_idx = in_idx;
		for(i=0; i<VPU_MAX; i++)
		{
			if(gtHevcClkTable[i].enabled)
			{
				if(gtHevcClkTable[i].clk_idx > max_idx){
					dprintk("changed[%d] :: %d -> %d\n", i, max_idx, gtHevcClkTable[i].clk_idx);
					max_idx = gtHevcClkTable[i].clk_idx;					
				}
				else
				{
					dprintk("not-changed [%d] :: %d !-> %d\n", i, max_idx, gtHevcClkTable[i].clk_idx);
				}
			}
			else
			{
				dprintk("not-enabled [%d] :: %d !-> %d\n", i, max_idx, gtHevcClkTable[i].clk_idx);
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
	bus_priority_changed = hmgr_BusPrioritySetting(BUS_FOR_VIDEO, info.type);

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

void hmgr_readyClock(unsigned char count, unsigned int vpu_base)
{
	if(count == 0)
	{
#ifdef CONFIG_CLOCK_TABLE
		vpu_clktbl = clocktable_get("vpu_clktbl");
		if (IS_ERR(vpu_clktbl)){
			vpu_clktbl = NULL;
		}
		else{
			info("Will use clktbl for TCC893x \n");
		}
#endif
	}
}

void hmgr_clearClock(unsigned char count)
{
	if(count == 0)
	{
#if defined(CONFIG_CLOCK_TABLE)
		if (vpu_2nd_clktbl)
			clocktable_ctrl(vpu_2nd_clktbl, 0, CLKTBL_DISABLE);
		if (vpu_clktbl)
			clocktable_ctrl(vpu_clktbl, 0, CLKTBL_DISABLE);
#endif
	}
}

void hmgr_set_clkstatus(vputype type, int on)
{
	dprintk(" @@@@@@@@@@@ hmgr_set_clkstatus %d - %d \n", type, on);

	gtHevcClkTable[type].enabled = on;
}

void hmgr_log_clkstatus(unsigned char closed, unsigned char opened_count, unsigned int accumulated_count)
{
	if(closed){
		printk("hmgr_release Out!! %d'th, total = %d  - DEC(%d/%d/%d/%d) \n", opened_count, accumulated_count,
					gtHevcClkTable[0].enabled, gtHevcClkTable[1].enabled, gtHevcClkTable[2].enabled, gtHevcClkTable[3].enabled);
	}
	else{
		dprintk("hmgr_open Out!! %d'th, total = %d  - DEC(%d/%d/%d/%d) \n", opened_count, accumulated_count,
					gtHevcClkTable[0].enabled, gtHevcClkTable[1].enabled, gtHevcClkTable[2].enabled, gtHevcClkTable[3].enabled);
	}
}

#ifdef VBUS_QOS_MATRIX_CTL
inline void vbus_matrix(void)
{
	vetc_reg_write(0x15444100, 0x00, 0x0); // PRI Core0 read - port0
	vetc_reg_write(0x15444100, 0x04, 0x0); // PRI Core0 write - port0

	vetc_reg_write(0x15445100, 0x00, 0x3); // PROC read - port1
	vetc_reg_write(0x15445100, 0x04, 0x3); // PROC read - port1

	vetc_reg_write(0x15446100, 0x00, 0x2); // SDMA read - port2
	vetc_reg_write(0x15446100, 0x04, 0x2); // SDMA read - port2

	vetc_reg_write(0x15447100, 0x00, 0x0); // SECON Core0 read - port3
	vetc_reg_write(0x15447100, 0x04, 0x0); // SECON Core0 read - port3

	vetc_reg_write(0x15449100, 0x00, 0x1); // DMA read - port4
	vetc_reg_write(0x15449100, 0x04, 0x1); // DMA read - port4

	vetc_reg_write(0x1544A100, 0x00, 0x0); // PRI Core1 read - port5
	vetc_reg_write(0x1544A100, 0x04, 0x0); // PRI Core1 read - port5

	vetc_reg_write(0x1544B100, 0x00, 0x0); // SECON Core1 read - port6
	vetc_reg_write(0x1544B100, 0x04, 0x0); // SECON Core1 read - port6
}
#endif

void hmgr_enable_clock(void)
{
	if (vbus_clk)
		clk_prepare_enable(vbus_clk);
	if (hevc_c_clk)
		clk_prepare_enable(hevc_c_clk);
	if (hevc_v_clk)
		clk_prepare_enable(hevc_v_clk);
	if (hevc_b_clk)
		clk_prepare_enable(hevc_b_clk);
	if (hevc_vbus_clk)
		clk_prepare_enable(hevc_vbus_clk);
#if defined(CONFIG_ARCH_TCC898X)
	if (hevc_vcore_clk)
		clk_prepare_enable(hevc_vcore_clk);
#endif
		
#ifdef CLK_HEVC_DEBUG
	printk("hevc clk(0x%x) -> enabled :: VBUS(0x%x) / HEVC(0x%x,0x%x,0x%x) / PWDN@VBus(0x%x) / SWR@VBus(0x%x)\n", HWCLK_BASE, 
			vetc_reg_read(HWCLK_BASE, VBUS),
			vetc_reg_read(HWCLK_BASE, HEVC_C),
			vetc_reg_read(HWCLK_BASE, HEVC_V),
			vetc_reg_read(HWCLK_BASE, HEVC_B),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x00),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x04));
#endif

#ifdef VBUS_QOS_MATRIX_CTL
	vbus_matrix();
#endif
}

void hmgr_disable_clock(void)
{
#if defined(CONFIG_ARCH_TCC898X)
	if (hevc_vcore_clk)
		clk_disable_unprepare(hevc_vcore_clk);
#endif
	if (hevc_vbus_clk)
		clk_disable_unprepare(hevc_vbus_clk);
	if (hevc_b_clk)
		clk_disable_unprepare(hevc_b_clk);
	if (hevc_v_clk)
		clk_disable_unprepare(hevc_v_clk);
	if (hevc_c_clk)
		clk_disable_unprepare(hevc_c_clk);
#if !defined(VBUS_CLK_ALWAYS_ON)
	if (vbus_clk)
		clk_disable_unprepare(vbus_clk);
#endif
#ifdef CLK_HEVC_DEBUG
	printk("hevc clk(0x%x) -> disabled :: VBUS(0x%x) / HEVC(0x%x,0x%x,0x%x) / PWDN@VBus(0x%x) / SWR@VBus(0x%x)\n", HWCLK_BASE, 
			vetc_reg_read(HWCLK_BASE, VBUS),
			vetc_reg_read(HWCLK_BASE, HEVC_C),
			vetc_reg_read(HWCLK_BASE, HEVC_V),
			vetc_reg_read(HWCLK_BASE, HEVC_B),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x00),
			vetc_reg_read(HwVIDEOBUSCONFIG_BASE, 0x04));
#endif	
}

void hmgr_get_clock(struct device_node *node)
{
    if(node == NULL) {
        printk("device node is null\n");
    }

	vbus_clk = of_clk_get(node, 0);
	BUG_ON(IS_ERR(vbus_clk));

    hevc_c_clk = of_clk_get(node, 1);
    BUG_ON(IS_ERR(hevc_c_clk));

	hevc_v_clk = of_clk_get(node, 2);
    BUG_ON(IS_ERR(hevc_v_clk));

	hevc_b_clk = of_clk_get(node, 3);
    BUG_ON(IS_ERR(hevc_b_clk));

	hevc_vbus_clk = of_clk_get(node, 4);
	BUG_ON(IS_ERR(hevc_vbus_clk));

#if defined(CONFIG_ARCH_TCC898X)
	hevc_vcore_clk = of_clk_get(node, 5);
	BUG_ON(IS_ERR(hevc_vcore_clk));
#endif
}

void hmgr_put_clock(void)
{
	if (hevc_c_clk) {
		clk_put(hevc_c_clk);
		hevc_c_clk = NULL;
	}
	if (hevc_v_clk) {
		clk_put(hevc_v_clk);
		hevc_v_clk = NULL;
	}
	if (hevc_b_clk) {
		clk_put(hevc_b_clk);
		hevc_b_clk = NULL;
	}
	if (hevc_vbus_clk) {
		clk_put(hevc_vbus_clk);
		hevc_vbus_clk = NULL;
	}
#if defined(CONFIG_ARCH_TCC898X)
	if (hevc_vcore_clk) {
		clk_put(hevc_vcore_clk);
		hevc_vcore_clk = NULL;
	}
#endif
	if (vbus_clk) {
		clk_put(vbus_clk);
		vbus_clk = NULL;
	}
}

void hmgr_enable_irq(unsigned int irq)
{
	enable_irq(irq);
}

void hmgr_disable_irq(unsigned int irq)
{
	disable_irq(irq);
}

void hmgr_free_irq(unsigned int irq, void * dev_id) {
	free_irq(irq, dev_id);
}

int hmgr_request_irq(unsigned int irq, irqreturn_t (*handler)(int irq, void *dev_id), unsigned long frags, const char * device, void * dev_id) {
    return request_irq(irq, handler, frags, device, dev_id);
}
unsigned long hmgr_get_int_flags(void)
{
	return IRQ_INT_TYPE;
}

void hmgr_init_interrupt(void)
{
}

int hmgr_BusPrioritySetting(int mode, int type)
{
	return 0;
}

void hmgr_status_clear(unsigned int base_addr)
{
}

void hmgr_init_variable(void)
{
	cache_droped = 0;
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC hevc devices driver");
MODULE_LICENSE("GPL");
#endif
