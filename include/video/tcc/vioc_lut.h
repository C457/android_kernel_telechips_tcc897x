/*
 * linux/include/video/tcc/vioc_lut.h
 * Author:  <linux@telechips.com>
 * Created: June 10, 2016
 * Description: TCC VIOC h/w block 
 *
 * Copyright (C) 2008-2009 Telechips
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef __VIOC_LUT_H__
#define	__VIOC_LUT_H__

#define	VIOC_LUT_DEV0			0
#define	VIOC_LUT_DEV1			1
#define	VIOC_LUT_DEV2			2
#define	VIOC_LUT_COMP0		3
#define	VIOC_LUT_COMP1		4
#define	VIOC_LUT_COMP2		5
#define	VIOC_LUT_COMP3		6

#define LUT_COLOR_DEPTH		10
#define LUT_TABLE_SIZE			(1 << LUT_COLOR_DEPTH)


extern void tcc_set_lut_table(unsigned int lut_n, unsigned int *table);
extern void tcc_set_lut_plugin(unsigned int lut_n, unsigned int plugComp);
extern int tcc_get_lut_plugin (unsigned int lut_n);
extern void tcc_set_lut_enable(unsigned int lut_n, unsigned int enable);
extern int tcc_get_lut_enable(unsigned int lut_n);

#endif
