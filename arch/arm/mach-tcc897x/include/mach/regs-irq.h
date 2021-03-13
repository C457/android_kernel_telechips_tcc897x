/*
 * Copyright (C) 2013 Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public Licens
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-
 */

#ifndef __ASM_ARCH_REGS_IRQ_H
#define __ASM_ARCH_REGS_IRQ_H

#define TCC893X_PA_PIC_BASE		0x74100000
#define TCC893X_VA_PIC_BASE		IO_VIRT(TCC893X_PA_PIC_BASE)

#define TCC893X_PIC_IEN0		0x000
#define TCC893X_PIC_IEN1		0x004
#define TCC893X_PIC_CLR0		0x008
#define TCC893X_PIC_CLR1		0x00C
#define TCC893X_PIC_POL0		0x038
#define TCC893X_PIC_POL1		0x03C
#define TCC893X_PIC_MODE0		0x060
#define TCC893X_PIC_MODE1		0x064
#define TCC893X_PIC_MODEA0		0x078
#define TCC893X_PIC_MODEA1		0x07C
#define TCC893X_PIC_INTMSK0		0x100
#define TCC893X_PIC_INTMSK1		0x104
#define TCC893X_PIC_ALLMSK		0x108

#define TCC893X_PA_VIC_BASE		0x74100200

#endif	/*__ASM_ARCH_REGS_IRQ */
