/*
 * linux/driver/net/tcc_gmac/tca_gmac.h
 * 	
 * Author : Telechips <linux@telechips.com>
 * Created : June 22, 2010
 * Description : This is the driver for the Telechips MAC 10/100/1000 on-chip Ethernet controllers.  
 *               Telechips Ethernet IPs are built around a Synopsys IP Core.
 *
 * Copyright (C) 2010 Telechips
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */ 

#ifndef _TCA_GMAC_H_
#define _TCA_GMAC_H_

#include <mach/bsp.h>
#include <mach/gpio.h>
#include <mach/iomap.h>

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/phy.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/system_info.h>

#define ECID0_OFFSET		(0x290)
#define ECID1_OFFSET		(0x294)
#define ECID2_OFFSET		(0x298)
#define ECID3_OFFSET		(0x29C)

#define GMACDLY0_OFFSET		(0x2000)
#define GMACDLY1_OFFSET		(0x2004)
#define GMACDLY2_OFFSET		(0x2008)
#define GMACDLY3_OFFSET		(0x200C)
#define GMACDLY4_OFFSET		(0x2010)
#define GMACDLY5_OFFSET		(0x2014)
#define GMACDLY6_OFFSET		(0x2018)

#define GMAC0_CFG2_OFFSET	(0x0054)
#define GMAC0_CFG3_OFFSET	(0x0058)
#define GMAC0_CFG4_OFFSET	(0x005C)
#define GMAC0_CFG0_OFFSET	(0x0060)
#define GMAC0_CFG1_OFFSET	(0x0064)
#define GMAC1_CFG0_OFFSET	(0x0068)
#define GMAC1_CFG1_OFFSET	(0x006C)
#define GMAC1_CFG2_OFFSET	(0x0070)
#define GMAC1_CFG3_OFFSET	(0x0074)
#define GMAC1_CFG4_OFFSET	(0x0078)

// GMAC CFG0 
#define CFG0_TR					(1<<31)
#define CFG0_TXDIV_SHIFT		(19)
#define CFG0_TXDIV_MASK			(0x7f<<CFG0_TXDIV_SHIFT)
#define CFG0_TXCLK_SEL_SHIFT	(16)
#define CFG0_TXCLK_SEL_MASK		(0x3<<CFG0_TXCLK_SEL_SHIFT)
#define CFG0_RR					(1<<15)
#define CFG0_RXDIV_SHIFT		(4)
#define CFG0_RXDIV_MASK			(0x3f<<CFG0_RXDIV_SHIFT)
#define CFG0_RXCLK_SEL_SHIFT	(0)
#define CFG0_RXCLK_SEL_MASK		(0x3<<CFG0_RXCLK_SEL_SHIFT)

// GMAC CFG1 
#define CFG1_CE					(1<<31)
#define CFG1_CC					(1<<30)
#define CFG1_PHY_INFSEL_SHIFT	(18)
#define CFG1_PHY_INFSEL_MASK	(0x7<<CFG1_PHY_INFSEL_SHIFT)
#define CFG1_FCTRL				(1<<17)
#define CFG1_TCO				(1<<16)

// GMAC CFG2
#define GPI_SHIFT				(0)
#define GPO_SHIFT				(4)
#define PTP_AUX_TS_TRIG_SHIFT	(8)
#define PTP_PPS_TRIG_SHIFT		(12)


struct gmac_dt_info_t {
	unsigned index;
	phy_interface_t phy_inf;
	struct clk *hsio_clk;
	struct clk *gmac_clk;
	struct clk *ptp_clk;
	struct clk *gmac_hclk;
	unsigned phy_on;
	unsigned phy_rst;
	u32 txclk_i_dly;
	u32 txclk_i_inv;
	u32 txclk_o_dly;
	u32 txclk_o_inv;
	u32 txen_dly;
	u32 txer_dly;
	u32 txd0_dly;
	u32 txd1_dly;
	u32 txd2_dly;
	u32 txd3_dly;
	u32 txd4_dly;
	u32 txd5_dly;
	u32 txd6_dly;
	u32 txd7_dly;
	u32 rxclk_i_dly;
	u32 rxclk_i_inv;
	u32 rxdv_dly;
	u32 rxer_dly;
	u32 rxd0_dly;
	u32 rxd1_dly;
	u32 rxd2_dly;
	u32 rxd3_dly;
	u32 rxd4_dly;
	u32 rxd5_dly;
	u32 rxd6_dly;
	u32 rxd7_dly;
	u32 crs_dly;
	u32 col_dly;
};

int tca_gmac_init(struct device_node *np, struct gmac_dt_info_t *dt_info);
void tca_gmac_clk_enable(struct gmac_dt_info_t *dt_info);
void tca_gmac_clk_disable(struct gmac_dt_info_t *dt_info);
unsigned int tca_gmac_get_hsio_clk(struct gmac_dt_info_t *dt_info);
phy_interface_t tca_gmac_get_phy_interface(struct gmac_dt_info_t *dt_info);
void tca_gmac_phy_pwr_on(struct gmac_dt_info_t *dt_info);
void tca_gmac_phy_pwr_off(struct gmac_dt_info_t *dt_info);
void tca_gmac_phy_reset(struct gmac_dt_info_t *dt_info);
void tca_gmac_tunning_timing(struct gmac_dt_info_t *dt_info, volatile void *ioaddr);
void tca_gmac_portinit(struct gmac_dt_info_t *dt_info, volatile void *ioaddr);
void IO_UTIL_ReadECID (unsigned ecid[]);
int tca_get_mac_addr_from_ecid(char *mac_addr);

#endif /*_TCA_GMAC_H_*/
