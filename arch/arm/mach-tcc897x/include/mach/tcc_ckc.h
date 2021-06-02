/****************************************************************************
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 ****************************************************************************/

#ifndef _PLATFORM_TCC_CKC_H_
#define _PLATFORM_TCC_CKC_H_

#include <dt-bindings/clock/tcc897x-clks.h>

#define XIN_CLK_RATE	(24*1000*1000)	// 24MHz
#define XTIN_CLK_RATE	32768	// 32.768kHz
#define HDMI_CLK_RATE	(27*1000*1000)
#define HDMI_PCLK_RATE	27	// dummy value for set lcdc peri source to hdmi pclk
#define EXT0_CLK_RATE	(24*1000*1000)	// 24MHz
#define EXT1_CLK_RATE	(24*1000*1000)	// 24MHz

#define CKC_DISABLE	0
#define CKC_ENABLE	1
#define CKC_NOCHANGE	2


/* PLL channel index */
enum {
	PLL_0=0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_DIV_0,
	PLL_DIV_1,
	PLL_DIV_2,
	PLL_DIV_3,
	PLL_DIV_4,
	PLL_XIN,
	PLL_XTIN,
};

typedef struct {
	unsigned int	fpll;
	unsigned int	en;
	unsigned int	p;
	unsigned int	m;
	unsigned int	s;
	unsigned int	src;
} tPMS;

typedef struct {
	unsigned int	freq;
	unsigned int	en;
	unsigned int	config;
	unsigned int	sel;
} tCLKCTRL;

typedef struct {
	unsigned int	periname;
	unsigned int	freq;
	unsigned int	md;
	unsigned int	en;
	unsigned int	sel;
	unsigned int	div;
} tPCLKCTRL;

extern void tcc_ckc_backup_regs(unsigned int clk_down);
extern void tcc_ckc_restore_regs(void);
extern unsigned long tca_ckc_get_nand_iobus_clk(void);
extern int tcc_ckc_set_hdmi_audio_src(unsigned int src_id);
extern int tcc_ckc_adjust_audio_clk(unsigned int aud_id, int value);
extern void tcc_ckc_restore_audio_clk(unsigned int aud_id);

#endif /* _PLATFORM_TCC_CKC_H_ */
