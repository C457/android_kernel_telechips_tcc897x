/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#ifndef _CAM_CLOCK_H_ 
#define _CAM_CLOCK_H_

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#define CIF_MCLOCK	0
#define VIOC_CLOCK	1

struct clock_data {
	struct clk		*cif_mclk;	
	struct clk 		*vioc_clk;	
	unsigned int	cif_mclk_rate;	
};

extern void cam_clock_get(struct tcc_camera_device * vdev);
extern void cam_clock_clkrate(struct tcc_camera_device * vdev);
extern void cam_clock_put(struct tcc_camera_device * vdev);
extern void cam_clock_enable(struct tcc_camera_device * vdev, int clk_index);
extern void cam_clock_disable(struct tcc_camera_device * vdev, int clk_index);
#endif

