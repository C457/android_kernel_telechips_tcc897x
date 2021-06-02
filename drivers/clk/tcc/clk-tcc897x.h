/****************************************************************************
 * mach/clk.h
 * Copyright (C) 2015 Telechips Inc.
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
#include <dt-bindings/clock/tcc897x-clks.h>

/*******************************
 * Define special options of clk.
 *   - parents, flags, etc.
 * must keep idx order
 *******************************/
/*	{ name,		parent_name,	idx,		flags },	*/
static struct _tcc_clk_data tcc_fbus_data[] = {
	{ "cpu0",	NULL,		FBUS_CPU0,	0 },
	{ "cpu1",	NULL,		FBUS_CPU1,	0 },
	{ "mem_bus",	NULL,		FBUS_MEM,	CLK_SET_RATE_GATE|CLK_IGNORE_UNUSED },
	{ "ddi_bus",	NULL,		FBUS_DDI,	CLK_SET_RATE_GATE },
	{ "mali_clk",	NULL,		FBUS_GPU,	0 },
	{ "io_bus",	NULL,		FBUS_IO,	CLK_SET_RATE_GATE|CLK_IGNORE_UNUSED },
	{ "vpu_bus",	NULL,		FBUS_VBUS,	CLK_SET_RATE_GATE },
	{ "boda",	NULL,		FBUS_CODA,	CLK_SET_RATE_GATE },
	{ "hsio_bus",	NULL,		FBUS_HSIO,	CLK_SET_RATE_GATE },
	{ "smu_bus",	NULL,		FBUS_SMU,	CLK_SET_RATE_GATE|CLK_IGNORE_UNUSED },
	{ "g2d_bus",	NULL,		FBUS_G2D,	CLK_SET_RATE_GATE },
#ifdef CONFIG_SUPPORT_TCC_NSK
	{ "cm_bus",	NULL,		FBUS_CMBUS,	CLK_SET_RATE_GATE|CLK_IGNORE_UNUSED },
#else
	{ "cm_bus",	NULL,		FBUS_CMBUS,	CLK_IGNORE_UNUSED },
#endif

	{ "hevc_c",	NULL,		FBUS_HEVC_VCE,	CLK_SET_RATE_GATE },
	{ "hevc_v",	"hevc_c",	FBUS_HEVC_VCPU,	CLK_SET_RATE_GATE },
	{ "hevc_b",	"hevc_c",	FBUS_HEVC_BPU,	CLK_SET_RATE_GATE },
};

static struct _tcc_clk_data tcc_peri_data[] = {
	{ "timerz",     NULL,           PERI_TCZ,       CLK_IGNORE_UNUSED },
	{ "cmbus_cnt",	NULL,		PERI_CMBUS_CNT,	CLK_IGNORE_UNUSED },
};

static struct _tcc_clk_data tcc_isoip_top_data[] = {
	{ "otp_phy",	NULL,		ISOIP_TOP_OTP,	CLK_IGNORE_UNUSED },
	{ "rtc_phy",	NULL,		ISOIP_TOP_RTC,	CLK_IGNORE_UNUSED },
	{ "pll_phy",	NULL,		ISOIP_TOP_PLL,	CLK_IGNORE_UNUSED },
};

static struct _tcc_clk_data tcc_iobus_data[] = {
	{ "sdhc0_hclk",	"sdhc",		IOBUS_SDMMC0,	0 },
	{ "sdhc2_hclk",	"sdhc",		IOBUS_SDMMC2,	0 },
	{ "sdhc3_hclk",	"sdhc",		IOBUS_SDMMC3,	0 },
	{ "sdhc",	"io_bus",	IOBUS_SDMMC,	0 },
	{ "dma0",	"dma",		IOBUS_DMA0,	CLK_IGNORE_UNUSED },
	{ "dma1",	"dma",		IOBUS_DMA1,	CLK_IGNORE_UNUSED },
	{ "dma2",	"dma",		IOBUS_DMA2,	CLK_IGNORE_UNUSED },
	{ "dma",	"io_bus",	IOBUS_DMA,	CLK_IGNORE_UNUSED },
	{ "i2cs2",	"i2c",		IOBUS_I2C_S2,	0 },
	{ "edi",	"edi_p",	IOBUS_EDICFG,	CLK_IGNORE_UNUSED },
	{ "edi_p",	"io_bus",	IOBUS_EDI,	CLK_IGNORE_UNUSED },
	{ "io_shulden",	"io_bus",	IOBUS_PROT,	CLK_IGNORE_UNUSED },
#ifdef CONFIG_SND_TCC_AUDIO_DSP//8971 EVM
	{ "adma0",	"audio0",	IOBUS_ADMA0,	CLK_IGNORE_UNUSED },
	{ "dai0_hclk",	"adma0",	IOBUS_DAI0,	CLK_IGNORE_UNUSED },
	{ "spdif0_h",	"adma0",	IOBUS_SPDIF0,	CLK_IGNORE_UNUSED },
	{ "audio0",	"io_bus",	IOBUS_AUDIO0,	CLK_IGNORE_UNUSED },
#else    
	{ "adma0",	"audio0",	IOBUS_ADMA0,	0 },
	{ "dai0_hclk",	"adma0",	IOBUS_DAI0,	0 },
	{ "spdif0_h",	"adma0",	IOBUS_SPDIF0,	0 },
	{ "audio0",	"io_bus",	IOBUS_AUDIO0,	0 },
#endif    
	{ "adma3",	"audio3",	IOBUS_ADMA3,	0 },
	{ "dai3_hclk",	"adma3",	IOBUS_DAI3,	0 },
	{ "spdif3_h",	"adma3",	IOBUS_SPDIF3,	0 },
	{ "audio3",	"io_bus",	IOBUS_AUDIO3,	0 },
	{ "i2c0_hclk",	"i2c",		IOBUS_I2C_M0,	0 },
	{ "i2c1_hclk",	"i2c",		IOBUS_I2C_M1,	0 },
	{ "i2c2_hclk",	"i2c",		IOBUS_I2C_M2,	0 },
	{ "i2c3_hclk",	"i2c",		IOBUS_I2C_M3,	0 },
	{ "i2cs0",	"i2c",		IOBUS_I2C_S0,	0 },
	{ "i2cs1",	"i2c",		IOBUS_I2C_S1,	0 },
	{ "i2c",	"io_bus",	IOBUS_I2C,	0 },
	{ "uart0_hclk",	"uart",		IOBUS_UART0,	0 },
	{ "uart1_hclk",	"uart",		IOBUS_UART1,	0 },
	{ "uart2_hclk",	"uart",		IOBUS_UART2,	0 },
	{ "uart3_hclk",	"uart",		IOBUS_UART3,	0 },
	{ "uart4_hclk",	"uart",		IOBUS_UART4,	0 },
	{ "uart5_hclk",	"uart",		IOBUS_UART5,	0 },
	{ "uart6_hclk",	"uart",		IOBUS_UART6,	0 },
	{ "uart7_hclk",	"uart",		IOBUS_UART7,	0 },
	{ "uart",	"io_bus",	IOBUS_UART,	0 },
	{ "nfc",	"io_bus",	IOBUS_NFC,	CLK_IGNORE_UNUSED },
	{ "gpsb0_hclk",	"gpsb",		IOBUS_GPSB0,	0 },
	{ "gpsb1_hclk",	"gpsb",		IOBUS_GPSB1,	0 },
	{ "gpsb2_hclk",	"gpsb",		IOBUS_GPSB2,	0 },
	{ "gpsb",	"io_bus",	IOBUS_GPSB,	0 },
	{ "i2cs3",	"i2c",		IOBUS_I2C_S3,	0 },
	{ "i2c4_hclk",	"i2c",		IOBUS_I2C_M4,	0 },
	{ "i2c5_hclk",	"i2c",		IOBUS_I2C_M5,	0 },
	{ "i2c6_hclk",	"i2c",		IOBUS_I2C_M6,	0 },
	{ "i2c7_hclk",	"i2c",		IOBUS_I2C_M7,	0 },
	{ "dmac0",	"dmac",		IOBUS_DMAC0,	0 },
	{ "dmac1",	"dmac",		IOBUS_DMAC1,	0 },
	{ "dmac2",	"dmac",		IOBUS_DMAC2,	0 },
	{ "dmac",	"io_bus",	IOBUS_DMAC,	0 },
	{ "adma1",	"audio1",	IOBUS_ADMA1,	0 },
	{ "dai1_hclk",	"adma1",	IOBUS_DAI1,	0 },
	{ "spdif1_h",	"adma1",	IOBUS_SPDIF1,	0 },
	{ "audio1",	"io_bus",	IOBUS_AUDIO1,	0 },
#ifdef CONFIG_SND_TCC_AUDIO_DSP//8971 LCN
	{ "adma2",	"audio2",	IOBUS_ADMA2,	CLK_IGNORE_UNUSED },
	{ "dai2_hclk",	"adma2",	IOBUS_DAI2,	CLK_IGNORE_UNUSED },
	{ "spdif2_h",	"adma2",	IOBUS_SPDIF2,	0 },
	{ "audio2",	"io_bus",	IOBUS_AUDIO2,	CLK_IGNORE_UNUSED },
#else    
    { "adma2",	"audio2",	IOBUS_ADMA2,	0 },
	{ "dai2_hclk",	"adma2",	IOBUS_DAI2,	0 },
	{ "spdif2_h",	"adma2",	IOBUS_SPDIF2,	0 },
	{ "audio2",	"io_bus",	IOBUS_AUDIO2,	0 },
#endif
};


static struct tcc_clks_type tcc_fbus_clks = {
	NULL, FBUS_MAX, tcc_fbus_data, ARRAY_SIZE(tcc_fbus_data), CLK_IS_ROOT
};

static struct tcc_clks_type tcc_peri_clks = {
	NULL, PERI_MAX, tcc_peri_data, ARRAY_SIZE(tcc_peri_data), CLK_IS_ROOT
};

static struct tcc_clks_type tcc_iobus_clks = {
	"io_bus", IOBUS_MAX, tcc_iobus_data, ARRAY_SIZE(tcc_iobus_data), CLK_SET_RATE_GATE
};

static struct tcc_clks_type tcc_isoip_top_clks = {
	NULL, ISOIP_TOP_MAX, tcc_isoip_top_data, ARRAY_SIZE(tcc_isoip_top_data), CLK_SET_RATE_GATE
};

static struct tcc_clks_type tcc_hsiobus_clks = {
	"hsio_bus", HSIOBUS_MAX, NULL, 0, CLK_SET_RATE_GATE
};

static struct tcc_clks_type tcc_ddibus_clks = {
	"ddi_bus", DDIBUS_MAX, NULL, 0, CLK_SET_RATE_GATE
};

static struct tcc_clks_type tcc_vpubus_clks = {
	"vpu_bus", VIDEOBUS_MAX, NULL, 0, CLK_SET_RATE_GATE
};

