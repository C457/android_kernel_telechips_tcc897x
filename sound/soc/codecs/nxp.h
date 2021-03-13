/*
 * Soc Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on nxp.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _NXP_H
#define _NXP_H

/* nxp register space */

#define NXP_SYSCLK	0
#define NXP_DAI		0

struct nxp_setup_data {
	int            spi;
	int            i2c_bus;
	unsigned short i2c_address;
};

#endif
