/****************************************************************************
 * arch/arm/mach-tcc893x/tcc_sram.c
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

#include <linux/module.h>
#include <mach/tcc_sram.h>
#if defined(CONFIG_ARM_TRUSTZONE)
#include <mach/smc.h>
#endif

extern void tcc_resume_from_sram(void);
extern void tcc_suspend_to_sram(unsigned mode);

void tcc_sram_init(void)
{
	memset((void *)sram_p2v(PMU_WAKEUP_ADDR), 0x0, PMU_WAKEUP_SIZE);

	memcpy((void*)sram_p2v(SRAM_RESUME_FUNC_ADDR),		(void*)tcc_resume_from_sram,		SRAM_RESUME_FUNC_SIZE);
	memcpy((void*)sram_p2v(SRAM_SUSPEND_FUNC_ADDR),		(void*)tcc_suspend_to_sram,		SRAM_SUSPEND_FUNC_SIZE);

#if defined(CONFIG_ARM_TRUSTZONE)	/* Secure SRAM Boot - hjbae */
	_tz_smc(SMC_CMD_FINALIZE, 0, 0, 0);
	_tz_smc(SMC_CMD_BLOCK_SW_JTAG, 0, 0, 0);
#endif
}
EXPORT_SYMBOL(tcc_sram_init);
