/* linux/arch/arm/mach-tcc897x/cpu_early_init.S
 *
 * Copyright (C) 2010 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/sram_map.h>

//=================================================================
// __cpu_early_init
//=================================================================
	.global __cpu_early_init
__cpu_early_init:
	stmfd	sp!, {r0-r2, lr}

#if 0 //defined(CONFIG_LCD_HDMI1920X1080) // 1080p UI ?
	//CPU Qos
	ldr     r0, =0xf4142100
	ldr     r1, =0x00000080        // combined ot
	ldr     r2, =0x000006FF        // outstanding = 6ff
	str     r1, [r0, #0x0C]
	str     r2, [r0, #0x14]

	//GPU Qos
	ldr     r0, =0xf4144100
	ldr     r1, =0x00000080        // combined ot
	ldr     r2, =0x000006FF        // outstanding = 6ff
	str     r1, [r0, #0x0C]
	str     r2, [r0, #0x14]

	//VBUS Qos
	ldr     r0, =0xf4145100
	ldr     r1, =0x00000080        // combined ot
	ldr     r2, =0x000006ff        // outstanding = 6ff
	str     r1, [r0, #0x0C]
	str     r2, [r0, #0x14]

	//VBUS Qos
	ldr     r0, =0xf4146100
	ldr     r1, =0x00000080        // combined ot
	ldr     r2, =0x000006ff        // outstanding = 6ff
	str     r1, [r0, #0x0C]
	str     r2, [r0, #0x14]
#endif

	ldmfd	sp!, {r0-r2, pc}

