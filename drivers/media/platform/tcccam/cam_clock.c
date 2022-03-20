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

#include "tcc_camera_device.h"
#include "cam_clock.h"
#include "sensor_if.h"

void cam_clock_get(struct tcc_camera_device * vdev)
{
	struct device_node * cam_np = vdev->camera_np;

	if(cam_np)
	{
		vdev->clk_data.cif_mclk = of_clk_get(cam_np, 0);
		vdev->clk_data.vioc_clk = of_clk_get(cam_np, 1);
	}
	
}

void cam_clock_clkrate(struct tcc_camera_device * vdev)
{
	struct device_node * cam_np = vdev->camera_np;
	struct device_node	*module_np;
	
#if defined(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT)

	if(vdev->CameraID)
	{
		module_np = of_find_node_by_name(cam_np,FRONT_MODULE_NODE);
	}
	else
	{
		module_np = of_find_node_by_name(cam_np,BACK_MODULE_NODE);
	}
	
#else

	if(vdev->data.cam_info == DAUDIO_CAMERA_LVDS || vdev->data.cam_info == DAUDIO_ADAS_PRK || vdev->data.cam_info == DAUDIO_DVRS_RVM)
		module_np = of_find_node_by_name(cam_np,LVDS_MODULE_NODE);
	else
		module_np = of_find_node_by_name(cam_np,MODULE_NODE);

#endif

	if(module_np)
	{
		of_property_read_u32(module_np, "clock-frequency", &vdev->clk_data.cif_mclk_rate);
	}
	else
	{
		printk("could not find sensor module node!! \n");
	}
	
}

void cam_clock_put(struct tcc_camera_device * vdev)
{
	clk_put(vdev->clk_data.cif_mclk);
	clk_put(vdev->clk_data.vioc_clk);	
}

void cam_clock_enable(struct tcc_camera_device * vdev, int clk_index)
{
	switch(clk_index){
		case CIF_MCLOCK :
			clk_set_rate(vdev->clk_data.cif_mclk, vdev->clk_data.cif_mclk_rate*100);
			printk("VIDEO IN Camera CLK :: MCLK = %d \n", vdev->clk_data.cif_mclk_rate*100);
			
			clk_prepare_enable(vdev->clk_data.cif_mclk);
			break;
			
		case VIOC_CLOCK :
			clk_prepare_enable(vdev->clk_data.vioc_clk);
			printk("VIOC clock status(enable cnt: %d, prepare cnt: %d)\n", \
					__clk_get_enable_count(vdev->clk_data.vioc_clk), \
					__clk_get_prepare_count(vdev->clk_data.vioc_clk));
			break;
	};
}

void cam_clock_disable(struct tcc_camera_device * vdev, int clk_index)
{
	switch(clk_index){
		case CIF_MCLOCK :
			clk_disable_unprepare(vdev->clk_data.cif_mclk);
			break;
			
		case VIOC_CLOCK :
			clk_disable_unprepare(vdev->clk_data.vioc_clk);
			printk("VIOC clock status(enable cnt: %d, prepare cnt: %d)\n", \
					__clk_get_enable_count(vdev->clk_data.vioc_clk), \
					__clk_get_prepare_count(vdev->clk_data.vioc_clk));
			break;
	};
}

