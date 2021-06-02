/****************************************************************************
 * mach/pmu_wakeup_src.h
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

#include "pmu_wakeup.h"
#include <video/tcc/gpio.h>

#ifndef __MACH_PMU_WAKEUP_SRC_H__
#define __MACH_PMU_WAKEUP_SRC_H__

enum {
	PMU_WKUP0_TSADC_UPDOWN = 0,
	PMU_WKUP0_TSADC_STOP_WKU,
	PMU_WKUP0_TSADC_WAKEUP,
	PMU_WKUP0_RTC_WAKEUP,
	PMU_WKUP0_nIRQOUT00,
	PMU_WKUP0_nIRQOUT01,		// 5
	PMU_WKUP0_nIRQOUT02,
	PMU_WKUP0_nIRQOUT03,
	PMU_WKUP0_GP_B11,
	PMU_WKUP0_GP_B12,
	PMU_WKUP0_GP_B13,		// 10
	PMU_WKUP0_GP_B14,
	PMU_WKUP0_GP_B15,
	PMU_WKUP0_GP_B28,
	PMU_WKUP0_GP_C00,
	PMU_WKUP0_GP_C15,		// 15
	PMU_WKUP0_GP_C18,
	PMU_WKUP0_GP_C19,
	PMU_WKUP0_GP_C20,
	PMU_WKUP0_GP_C21,
	PMU_WKUP0_GP_C22,		// 20
	PMU_WKUP0_GP_C23,
	PMU_WKUP0_GP_C24,
	PMU_WKUP0_GP_C25,
	PMU_WKUP0_GP_C27,
	PMU_WKUP0_GP_C28,		// 25
	PMU_WKUP0_GP_C29,
	PMU_WKUP0_GP_D08,
	PMU_WKUP0_GP_D09,
	PMU_WKUP0_GP_D12,
	PMU_WKUP0_GP_D13,		// 30
	PMU_WKUP0_GP_D14,
};

enum {
	PMU_WKUP1_GP_D20 = 0,
	PMU_WKUP1_GP_E31,
	PMU_WKUP1_GP_F15,
	PMU_WKUP1_GP_F16,
	PMU_WKUP1_GP_F17,
	PMU_WKUP1_GP_F18,		// 5
	PMU_WKUP1_GP_F19,
	PMU_WKUP1_GP_F20,
	PMU_WKUP1_GP_F21,
	PMU_WKUP1_GP_F22,
	PMU_WKUP1_GP_F23,		// 10
	PMU_WKUP1_GP_F29,
	PMU_WKUP1_GP_G05,
	PMU_WKUP1_GP_G08,
	PMU_WKUP1_GP_G09,
	PMU_WKUP1_GP_G10,		// 15
	PMU_WKUP1_GP_G11,
	PMU_WKUP1_GP_G12,
	PMU_WKUP1_GP_G13,
	PMU_WKUP1_GP_G14,
	PMU_WKUP1_GP_G16,		// 20
	PMU_WKUP1_GP_G17,
	PMU_WKUP1_GP_G18,
	PMU_WKUP1_GP_G19,
	PMU_WKUP1_GP_HDMI00,
	PMU_WKUP1_GP_HDMI01,		// 25
	PMU_WKUP1_GP_SD06,
	PMU_WKUP1_GP_SD07,
	PMU_WKUP1_GP_SD08,
	PMU_WKUP1_GP_SD09,
	PMU_WKUP1_GP_SD10,		// 30
	PMU_WKUP1_REMOCON,
};

static struct wakeup_source_ wakeup_src[] = {
	{ PMU_WKUP0_TSADC_UPDOWN,	-1 },	// 0
	{ PMU_WKUP0_TSADC_STOP_WKU,	-1 },	
	{ PMU_WKUP0_TSADC_WAKEUP,	-1 },
	{ PMU_WKUP0_RTC_WAKEUP,		-1 },
	{ PMU_WKUP0_nIRQOUT00,		-1 },
	{ PMU_WKUP0_nIRQOUT01,		-1 },	// 5
	{ PMU_WKUP0_nIRQOUT02,		-1 },
	{ PMU_WKUP0_nIRQOUT03,		-1 },
	{ PMU_WKUP0_GP_B11,	TCC_GPB(11) },
	{ PMU_WKUP0_GP_B12,	TCC_GPB(12) },
	{ PMU_WKUP0_GP_B13,	TCC_GPB(13) },	// 10
	{ PMU_WKUP0_GP_B14,	TCC_GPB(14) },
	{ PMU_WKUP0_GP_B15,	TCC_GPB(15) },
	{ PMU_WKUP0_GP_B28,	TCC_GPB(28) },
	{ PMU_WKUP0_GP_C00,	TCC_GPC(0) },
	{ PMU_WKUP0_GP_C15,	TCC_GPC(15) },	// 15
	{ PMU_WKUP0_GP_C18,	TCC_GPC(18) },
	{ PMU_WKUP0_GP_C19,	TCC_GPC(19) },
	{ PMU_WKUP0_GP_C20,	TCC_GPC(20) },
	{ PMU_WKUP0_GP_C21,	TCC_GPC(21) },
	{ PMU_WKUP0_GP_C22,	TCC_GPC(22) },	// 20
	{ PMU_WKUP0_GP_C23,	TCC_GPC(23) },
	{ PMU_WKUP0_GP_C24,	TCC_GPC(24) },
	{ PMU_WKUP0_GP_C25,	TCC_GPC(25) },
	{ PMU_WKUP0_GP_C27,	TCC_GPC(27) },
	{ PMU_WKUP0_GP_C28,	TCC_GPC(28) },	// 25
	{ PMU_WKUP0_GP_C29,	TCC_GPC(29) },
	{ PMU_WKUP0_GP_D08,	TCC_GPD(8) },
	{ PMU_WKUP0_GP_D09,	TCC_GPD(9) },
	{ PMU_WKUP0_GP_D12,	TCC_GPD(12) },
	{ PMU_WKUP0_GP_D13,	TCC_GPD(13) },	// 30
	{ PMU_WKUP0_GP_D14,	TCC_GPD(14) },
	{ PMU_WKUP1_GP_D20,	TCC_GPD(20) },	// 32	// 0
	{ PMU_WKUP1_GP_E31,	TCC_GPE(31) },
	{ PMU_WKUP1_GP_F15,	TCC_GPF(15) },
	{ PMU_WKUP1_GP_F16,	TCC_GPF(16) },
	{ PMU_WKUP1_GP_F17,	TCC_GPF(17) },
	{ PMU_WKUP1_GP_F18,	TCC_GPF(18) },	// 37	// 5
	{ PMU_WKUP1_GP_F19,	TCC_GPF(19) },
	{ PMU_WKUP1_GP_F20,	TCC_GPF(20) },
	{ PMU_WKUP1_GP_F21,	TCC_GPF(21) },
	{ PMU_WKUP1_GP_F22,	TCC_GPF(22) },
	{ PMU_WKUP1_GP_F23,	TCC_GPF(23) },	// 42	// 10
	{ PMU_WKUP1_GP_F29,	TCC_GPF(29) },
	{ PMU_WKUP1_GP_G05,	TCC_GPG(5) },
	{ PMU_WKUP1_GP_G08,	TCC_GPG(8) },
	{ PMU_WKUP1_GP_G09,	TCC_GPG(9) },
	{ PMU_WKUP1_GP_G10,	TCC_GPG(10) },	// 47	// 15
	{ PMU_WKUP1_GP_G11,	TCC_GPG(11) },
	{ PMU_WKUP1_GP_G12,	TCC_GPG(12) },
	{ PMU_WKUP1_GP_G13,	TCC_GPG(13) },
	{ PMU_WKUP1_GP_G14,	TCC_GPG(14) },
	{ PMU_WKUP1_GP_G16,	TCC_GPG(16) },	// 52	// 20
	{ PMU_WKUP1_GP_G17,	TCC_GPG(17) },
	{ PMU_WKUP1_GP_G18,	TCC_GPG(18) },
	{ PMU_WKUP1_GP_G19,	TCC_GPG(19) },
	{ PMU_WKUP1_GP_HDMI00,	TCC_GPHDMI(0) },
	{ PMU_WKUP1_GP_HDMI01,	TCC_GPHDMI(1) },	// 57	// 25
	{ PMU_WKUP1_GP_SD06,	TCC_GPSD(6) },
	{ PMU_WKUP1_GP_SD07,	TCC_GPSD(7) },
	{ PMU_WKUP1_GP_SD08,	TCC_GPSD(8) },
	{ PMU_WKUP1_GP_SD09,	TCC_GPSD(9) },
	{ PMU_WKUP1_GP_SD10,	TCC_GPSD(10) },	// 62	// 30
	{ PMU_WKUP1_REMOCON,		-1 },
};

#endif /* __MACH_PMU_WAKEUP_SRC_H__ */
