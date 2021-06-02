/* linux/arch/arm/mach-tcc92xx/include/mach/tcc_board_hdmi.h
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

#ifndef __TCC_HDMI_H
#define __TCC_HDMI_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/switch.h>

typedef enum{
	TCC_HDMI_PWR_INIT,
	TCC_HDMI_PWR_ON,		
	TCC_HDMI_PWR_OFF,
	TCC_HDMI_PWR_V5_ON,
	TCC_HDMI_PWR_V5_OFF,
	TCC_HDMI_PWR_MAX
}tcc_hdmi_power_s;

struct tcc_hdmi_platform_data {
	unsigned int open_cnt;
	struct device *pdev;

	int (*set_power)(struct device *dev, tcc_hdmi_power_s pwr);
};


struct tcc_hpd_platform_data {
	unsigned hpd_port;
};


#define TCC_HDMI_ON     1
#define TCC_HDMI_OFF    0

struct hdmi_gpio_switch_data {
	struct switch_dev sdev;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	unsigned int state_val;
	struct work_struct work;
	void (*send_hdp_event)(void *pswitch_data, unsigned int hpd_state);
};

#endif
