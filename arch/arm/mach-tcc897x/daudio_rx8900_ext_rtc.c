/*
 * Copyright 2013 Linear Technology Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/i2c.h>


#define RTC_I2C_ADDR 	0x64

static struct i2c_board_info __initdata i2c_rtc_devices[] = {
	{
		I2C_BOARD_INFO("rx8900", RTC_I2C_ADDR>>1),
		.irq = -1,
	},
};


void __init daudio_init_ext_rtc(void)
{
	printk("daudio_init_rx8900_ext_rtc\n");

	i2c_register_board_info(0, i2c_rtc_devices, ARRAY_SIZE(i2c_rtc_devices));
}

subsys_initcall(daudio_init_ext_rtc);
	


