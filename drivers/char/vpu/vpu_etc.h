/****************************************************************************
 *   FileName    : vpu_etc.h
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
#include <linux/clk.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/io.h>
#include <mach/tcc_video_common.h>
#else
#include <video/tcc/tcc_types.h>
#include <video/tcc/tcc_video_common.h>
#endif

#ifndef _VETC_H_
#define _VETC_H_

#ifdef CONFIG_VPU_TIME_MEASUREMENT
extern int vetc_GetTimediff_ms(struct timeval time1, struct timeval time2);
#endif

extern unsigned int vetc_reg_read(unsigned int base_addr, unsigned int addr);
extern void vetc_reg_write(unsigned int base_addr, unsigned int addr, unsigned int data);
extern void vetc_dump_reg_all(unsigned int base_addr, unsigned char *str);
extern void vetc_reg_init(unsigned int base_addr);

#endif

