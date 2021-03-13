/*
 * Telechips TCC893x Flattened Device Tree enabled machine
 *
 * Copyright (c) 2014 Telechips Co., Ltd.
 *		http://www.telechips.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/of_platform.h>
#include <linux/serial_core.h>

#include <asm/mach/arch.h>
#include <mach/iomap.h>
#ifdef CONFIG_DAUDIO
#include <mach/gpio.h>
#endif

#include "common.h"

#include <plat/platsmp.h>

static struct of_dev_auxdata tcc897x_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("telechips,vioc-fb", TCC_PA_VIOC,
		       "tccfb", NULL),
	OF_DEV_AUXDATA("telechips,tcc897x-sdhc", TCC897X_PA_SDMMC0,
		       "tcc-sdhc.0", NULL),
	OF_DEV_AUXDATA("telechips,tcc897x-sdhc", TCC897X_PA_SDMMC2,
		       "tcc-sdhc.2", NULL),
	OF_DEV_AUXDATA("telechips,tcc897x-sdhc", TCC897X_PA_SDMMC3,
		       "tcc-sdhc.3", NULL),
	OF_DEV_AUXDATA("telechips,nand-v8", TCC897X_PA_NFC,
		       "tcc_nand", NULL),
	OF_DEV_AUXDATA("telechips,tcc-ehci", TCC_PA_EHCI,
		       "tcc-ehci", NULL),
	OF_DEV_AUXDATA("telechips,dwc_otg", TCC_PA_DWC_OTG,
			   "dwc_otg", NULL),
	OF_DEV_AUXDATA("telechips,i2c", TCC_PA_I2C0,
		       "i2c.0", NULL),
	OF_DEV_AUXDATA("telechips,i2c", TCC_PA_I2C1,
		       "i2c.1", NULL),
	OF_DEV_AUXDATA("telechips,i2c", TCC_PA_I2C2,
		       "i2c.2", NULL),
	OF_DEV_AUXDATA("telechips,i2c", TCC_PA_I2C3,
		       "i2c.3", NULL),
	OF_DEV_AUXDATA("telechips,wmixer_drv", 0, "wmixer0", NULL),
	OF_DEV_AUXDATA("telechips,wmixer_drv", 1, "wmixer1", NULL),
	OF_DEV_AUXDATA("telechips,scaler_drv", 1, "scaler1", NULL),
	OF_DEV_AUXDATA("telechips,scaler_drv", 3, "scaler3", NULL),
	OF_DEV_AUXDATA("telechips,tcc_wdma", 0, "wdma", NULL),
	OF_DEV_AUXDATA("telechips,tcc_overlay", 0, "overlay", NULL),
	{},
};

static void __init tcc897x_dt_init(void)
{
	platform_device_register_simple("tcc-cpufreq", -1, NULL, 0);
	of_platform_populate(NULL, of_default_bus_match_table,
			     tcc897x_auxdata_lookup, NULL);
#if defined(CONFIG_DAUDIO) //set GPIO C 29 HIGH
    //tcc_gpio_config(TCC_GPC(29), GPIO_FN0|GPIO_OUTPUT|GPIO_HIGH);
#endif
}

static char const *tcc897x_dt_compat[] __initconst = {
	"telechips,tcc897x",
	NULL
};

DT_MACHINE_START(TCC897X_DT, "Telechips TCC897x (Flattened Device Tree)")
	.init_irq	= tcc_irq_dt_init,
	.smp		= smp_ops(tcc_smp_ops),
	.smp_init	= smp_init_ops(tcc_smp_init_ops),
	.map_io		= tcc_map_common_io,
	.init_early	= tcc_init_early,
	.init_machine	= tcc897x_dt_init,
	.dt_compat	= tcc897x_dt_compat,
	.restart        = tcc897x_restart,
	.reserve	= tcc_mem_reserve,
MACHINE_END


