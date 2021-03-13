/*
 * include/asm-arm/arch-tcc88xx/entry-macro.S
 *
 * Author : <linux@telechips.com>
 * Created: August 30, 2010
 * Description: Low-level IRQ helper macros for TCC-based platforms
 *
 * Copyright (C) 2010 Telechips, Inc.
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>
#include <mach/irqs.h>

	.macro  disable_fiq
	.endm 

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

#include <asm/hardware/entry-macro-gic.S>

