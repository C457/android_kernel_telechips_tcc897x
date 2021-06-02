/****************************************************************************
 *   FileName    : vp9_mgr_sys.h
 *   Description : chip dependent codes
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


#ifndef _VP9MGR_SYS_H_
#define _VP9MGR_SYS_H_

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

#define BUS_FOR_NORMAL  0
#define BUS_FOR_VIDEO   1

#if defined(CONFIG_CLOCK_TABLE)
extern int vp9mgr_calcClock(CONTENTS_INFO info);
extern int vp9mgr_setClock(int in_idx, CONTENTS_INFO info, unsigned char dev_opened_cnt, unsigned char clk_limitation);
#endif

extern void vp9mgr_readyClock(unsigned char count, unsigned int vpu_base);
extern void vp9mgr_clearClock(unsigned char count);

extern void vp9mgr_set_clkstatus(vputype type, int on);
extern void vp9mgr_log_clkstatus(unsigned char closed, unsigned char opened_count, unsigned int accumulated_count);

extern void vp9mgr_enable_clock(void);
extern void vp9mgr_disable_clock(void);
extern void vp9mgr_get_clock(struct device_node *node);
extern void vp9mgr_put_clock(void);

extern void vp9mgr_enable_irq(unsigned int irq);
extern void vp9mgr_disable_irq(unsigned int irq);
extern void vp9mgr_free_irq(unsigned int irq, void * dev_id);
extern int  vp9mgr_request_irq(unsigned int irq, irqreturn_t (*handler)(int , void *), unsigned long frags, const char * device, void * dev_id);
extern unsigned long vp9mgr_get_int_flags(void);

extern void vp9mgr_init_interrupt(void);
extern int vp9mgr_BusPrioritySetting(int mode, int type);
extern void vp9mgr_status_clear(unsigned int base_addr);
extern int vp9mgr_hw_reset(void);

extern void vp9mgr_init_variable(void);
#endif
