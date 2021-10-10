
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

#include <generated/autoconf.h>

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#if defined(CONFIG_ARCH_TCC898X)
#include <video/tcc/tcc_gpu_align.h>
#include <video/tcc/tcc_cam_ioctrl.h>
#include <video/tcc/vioc_deintls.h>
#else
#include <mach/bsp.h>
#include <mach/tcc_gpu_align.h>
#include <mach/tcc_cam_ioctrl.h>
#include <mach/vioc_deintls.h>
#endif
#include <soc/tcc/pmap.h>

#include "sensor_if.h"
#include "tcc_cam.h"
#include "camera_core.h"
#include "cam_reg.h"
#include "tcc_cam_i2c.h"
#include "tcc_camera_device.h"

static int debug	= 1;
#define TAG		"tcc_cam"
#define dprintk(msg...)	if(debug) { printk(TAG ": " msg); }
#define log(msg...)	if(debug) { printk(TAG ": %s - ", __func__); printk(msg); }
#define FUNCTION_IN	log("In\n");
#define FUNCTION_OUT	log("Out\n");

void cif_dma_hw_reg(unsigned char frame_num, struct tcc_camera_device * vdev);

void cif_set_frameskip(struct tcc_camera_device * vdev, unsigned char skip_count, unsigned char locked)
{
	vdev->skip_frm = skip_count;
	vdev->frame_lock = locked;
}
EXPORT_SYMBOL(cif_set_frameskip);

int cif_get_capturedone(struct tcc_camera_device * vdev)
{
	if(vdev->data.cif_cfg.cap_status == CAPTURE_DONE) {
		mdelay(5);
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(cif_get_capturedone);

static void cif_data_init(struct tcc_camera_device * vdev)
{
	// Default Setting
	vdev->data.cif_cfg.polarity_pclk 		= vdev->tcc_sensor_info.p_clock_pol;
	vdev->data.cif_cfg.polarity_vsync	 	= vdev->tcc_sensor_info.v_sync_pol;
	vdev->data.cif_cfg.polarity_href 		= vdev->tcc_sensor_info.h_sync_pol;
	vdev->data.cif_cfg.polarity_de	 		= vdev->tcc_sensor_info.de_pol;
	vdev->data.cif_cfg.field_bfield_low		= vdev->tcc_sensor_info.field_bfield_low;
	vdev->data.cif_cfg.gen_field_en    		= vdev->tcc_sensor_info.gen_field_en;
	vdev->data.cif_cfg.conv_en           	= vdev->tcc_sensor_info.conv_en;
	vdev->data.cif_cfg.hsde_connect_en   	= vdev->tcc_sensor_info.hsde_connect_en;
	vdev->data.cif_cfg.vs_mask           	= vdev->tcc_sensor_info.vs_mask;
	vdev->data.cif_cfg.input_fmt         	= vdev->tcc_sensor_info.input_fmt;
	vdev->data.cif_cfg.data_order        	= vdev->tcc_sensor_info.data_order;
	vdev->data.cif_cfg.intl_en           	= vdev->tcc_sensor_info.intl_en;
	vdev->data.cif_cfg.intpl_en          	= vdev->tcc_sensor_info.intpl_en;
	vdev->data.cif_cfg.zoom_step			= 0;
	vdev->data.cif_cfg.base_buf				= vdev->data.cif_buf.addr;
	vdev->data.cif_cfg.pp_num				= 0; // TCC_CAMERA_MAX_BUFNBRS;
	vdev->data.cif_cfg.fmt					= vdev->tcc_sensor_info.format;
	vdev->data.cif_cfg.order422 			= CIF_YCBYCR;
	vdev->data.cif_cfg.oper_mode 			= OPER_PREVIEW;
	vdev->data.cif_cfg.cap_status 			= CAPTURE_NONE;
	vdev->data.cif_cfg.retry_cnt			= 0;
}

static irqreturn_t cif_cam_isr(int irq, void *client_data)
{
	struct tcc_camera_device * vdev = (struct tcc_camera_device *)client_data;
	struct tccxxx_cif_buffer *curr_buf, *next_buf;
	unsigned int next_num, stride;
	VIOC_WDMA*	pWDMABase = (VIOC_WDMA*)vdev->vioc.wdma.address;

	if (is_vioc_intr_activatied(vdev->data.vioc_intr.id, vdev->data.vioc_intr.bits) == false)
		return IRQ_NONE;

	// preview operation.
	if(vdev->data.cif_cfg.oper_mode == OPER_PREVIEW)
	{
		if(pWDMABase->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFF_MASK)
		{
			if(vdev->skip_frm == 0 && !vdev->frame_lock) 
			{
				if(vdev->prev_buf != NULL)
					list_move_tail(&vdev->prev_buf->buf_list, &(vdev->data.done_list));

				vdev->prev_buf = vdev->data.buf + vdev->data.cif_cfg.now_frame_num;
				vdev->prev_num = vdev->data.cif_cfg.now_frame_num;

				next_buf = list_entry(vdev->data.list.next->next, struct tccxxx_cif_buffer, buf_list);
				next_num = next_buf->v4lbuf.index;
				
				// exception process!!
				if(next_buf->buf_list.next != vdev->data.list.next && vdev->prev_buf != next_buf) 
				{ 
					if(vdev->prev_num != vdev->prev_buf->v4lbuf.index) 
					{
						printk("Frame num mismatch :: true num	:: %d \n", vdev->prev_num);
						printk("Frame num mismatch :: false num :: %d \n", vdev->prev_buf->v4lbuf.index);
						vdev->prev_buf->v4lbuf.index = vdev->prev_num ;
					}

					if(next_num > vdev->data.cif_cfg.pp_num || next_num < 0) return IRQ_NONE;

					cif_dma_hw_reg(next_num, vdev);

					stride = ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
					
					vdev->prev_buf->v4lbuf.bytesused = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (vdev->data.cif_cfg.main_set.target_y / 2);
					vdev->prev_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
					vdev->prev_buf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

					/* bfield Process */
					/**********  VERY IMPORTANT ***************************/
					/* if we want to operate temporal deinterlacer mode,  */
					/* initial 3 field are operated by spatial,           */
					/* 	then change the temporal mode in next fields.     */
					/******************************************************/
#if 0
					if(vdev->data.cif_cfg.intl_en)
					{
						if((vdev->frm_cnt == 1) && (vdev->bfield == 0)) 
						{
							dprintk("Deintl Initialization\n");
							VIOC_VIQE_SetDeintlMode((VIQE *)vdev->vioc.viqe.address, 2);
						}
						else
						{
							vdev->field_cnt++;
						}
						
						// end fied of bottom field
						if(vdev->bfield == 1) 
						{ 
							vdev->bfield = 0;
							vdev->frm_cnt++;
						}
						else
						{
							vdev->bfield = 1;
						}

						if(vdev->skip_frm == 0)
						{ 
							vdev->skip_frm++;
						}
						else
						{
							vdev->skip_frm = 1;
						}
					}
#endif
				}
				else
				{
					vdev->prev_buf = NULL;
					dprintk("no-buf change... wakeup!! \n");
					vdev->skipped_frm++;
				}
				
				vdev->data.wakeup_int = 1;
				wake_up_interruptible(&(vdev->data.frame_wait));
			}
			else 
			{
				if(vdev->skip_frm > 0)
					vdev->skip_frm--;
				else
					vdev->skip_frm = 0;
			}
			VIOC_WDMA_SetImageUpdate((VIOC_WDMA*)vdev->vioc.wdma.address); // update WDMA
			VIOC_WDMA_ClearEOFF((VIOC_WDMA*)vdev->vioc.wdma.address); // clear EOFF
		}
		return IRQ_HANDLED;
	}

	// capture operation.
	if(vdev->data.cif_cfg.oper_mode == OPER_CAPTURE) 
	{ 
		if(pWDMABase->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFR_MASK) 
		{
			if(vdev->skip_frm == 0 && !vdev->frame_lock) 
			{
				VIOC_WDMA_SetIreqMask((VIOC_WDMA*)vdev->vioc.wdma.address, VIOC_WDMA_IREQ_ALL_MASK, 0x1);
				VIOC_WDMA_SetImageDisable((VIOC_WDMA*)vdev->vioc.wdma.address);
				VIOC_VIN_SetEnable((VIOC_VIN *)vdev->vioc.vin.address, OFF); // disable VIN

				vdev->data.cif_cfg.now_frame_num = 0;
				curr_buf = vdev->data.buf + vdev->data.cif_cfg.now_frame_num;

				curr_buf->v4lbuf.bytesused = vdev->data.cif_cfg.main_set.target_x*vdev->data.cif_cfg.main_set.target_y*2;

				curr_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
				curr_buf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

				list_move_tail(&curr_buf->buf_list, &(vdev->data.done_list));
				vdev->data.cif_cfg.cap_status = CAPTURE_DONE;
				wake_up_interruptible(&(vdev->data.frame_wait)); //POLL

				VIOC_WDMA_SetIreqStatus((VIOC_WDMA*)vdev->vioc.wdma.address, VIOC_WDMA_IREQ_ALL_MASK, 0x1);
			}
			else 
			{
				if(vdev->skip_frm > 0) {
					printk("capture frame skip. skip count is %d. \n", vdev->skip_frm);
					vdev->skip_frm--;
				}
				else {
					vdev->skip_frm = 0;
				}
				VIOC_WDMA_SetImageUpdate((VIOC_WDMA*)vdev->vioc.wdma.address); // update WDMA
			}
		}
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}

static int cif_cleanup(struct tcc_camera_device * vdev)
{  
	return sensor_if_cleanup(vdev);
}

void cif_scaler_calc(struct tcc_camera_device * vdev)
{
	enum cifoper_mode mode = vdev->data.cif_cfg.oper_mode;
	unsigned int off_x = 0, off_y = 0, width = 0, height = 0;

	if(mode == OPER_PREVIEW 	|| 	\
		(mode == OPER_CAPTURE && vdev->data.cif_cfg.main_set.target_x < vdev->tcc_sensor_info.cam_capchg_width)) 
	{
		off_x  = vdev->tcc_sensor_info.preview_zoom_offset_x;
		off_y  = vdev->tcc_sensor_info.preview_zoom_offset_y;
		width  = vdev->tcc_sensor_info.preview_w;
		height = vdev->tcc_sensor_info.preview_h;
	}
	else
	{
		off_x  = vdev->tcc_sensor_info.capture_zoom_offset_x;
		off_y  = vdev->tcc_sensor_info.capture_zoom_offset_y;
		width  = vdev->tcc_sensor_info.capture_w;
		height = vdev->tcc_sensor_info.capture_h;
	}

	vdev->data.cif_cfg.main_set.win_hor_ofst = off_x * vdev->data.cif_cfg.zoom_step;
	vdev->data.cif_cfg.main_set.win_ver_ofst = off_y * vdev->data.cif_cfg.zoom_step;
	vdev->data.cif_cfg.main_set.source_x 	= width  - (off_x * vdev->data.cif_cfg.zoom_step) * 2;
	vdev->data.cif_cfg.main_set.source_y 	= height - (off_y * vdev->data.cif_cfg.zoom_step) * 2;
	dprintk("%s():  src_x=%d, src_y=%d, off_x=%d, off_y=%d, zoom_step=%d. \n", __FUNCTION__, vdev->data.cif_cfg.main_set.source_x, \
			vdev->data.cif_cfg.main_set.source_y, off_x, off_y, vdev->data.cif_cfg.zoom_step);
	dprintk("%s():  scaler_x=%d, scaler_y=%d, scaler_off_x=%d, scaler_off_y=%d, tgt_x=%d, tgt_y=%d. \n", __FUNCTION__, \
			vdev->data.cif_cfg.main_set.scaler_x, vdev->data.cif_cfg.main_set.scaler_y, vdev->data.cif_cfg.main_set.win_hor_ofst, \
			vdev->data.cif_cfg.main_set.win_ver_ofst, vdev->data.cif_cfg.main_set.target_x, vdev->data.cif_cfg.main_set.target_y);

	VIOC_VIN_SetImageSize((VIOC_VIN *)vdev->vioc.vin.address, width, height);
	VIOC_VIN_SetImageOffset((VIOC_VIN *)vdev->vioc.vin.address, 0, 0, 0);
	VIOC_VIN_SetImageCropSize((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.main_set.source_x, vdev->data.cif_cfg.main_set.source_y);
	VIOC_VIN_SetImageCropOffset((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.main_set.win_hor_ofst, vdev->data.cif_cfg.main_set.win_ver_ofst);
	dprintk("VIN WxH[%dx%d] Crop Start XxY[%dx%d] \n", vdev->data.cif_cfg.main_set.source_x, vdev->data.cif_cfg.main_set.source_y, 	\
													vdev->data.cif_cfg.main_set.win_hor_ofst, vdev->data.cif_cfg.main_set.win_ver_ofst);
}

void cif_dma_hw_reg(unsigned char frame_num, struct tcc_camera_device * vdev)
{	
	vdev->data.cif_cfg.now_frame_num = frame_num;

	if(vdev->pix_format.pixelformat == V4L2_PIX_FMT_YVU420) {
#ifdef CONFIG_VIDEO_V4L2_VIDEO_X_MEMORY_ALLOC
		VIOC_WDMA_SetImageBase((VIOC_WDMA*)vdev->vioc.wdma.address, (unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Y,
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cb, 
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cr); 
#else
		// For Making the format android wants, we had to change between Cb and Cr.
		VIOC_WDMA_SetImageBase((VIOC_WDMA*)vdev->vioc.wdma.address, (unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Y,
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cr, 
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cb);
#endif
	}
	else {
		VIOC_WDMA_SetImageBase((VIOC_WDMA*)vdev->vioc.wdma.address, (unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Y,
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cb,
										(unsigned int)vdev->data.cif_cfg.preview_buf[frame_num].p_Cb);
	}
}

void cif_preview_dma_set(struct tcc_camera_device * vdev)
{
	unsigned char i;
	dma_addr_t base_addr = vdev->data.cif_cfg.base_buf;
	unsigned int y_offset;
	unsigned int uv_offset = 0;
	unsigned int total_off = 0;
	unsigned int stride;

	vdev->data.cif_cfg.now_frame_num = 0;

	stride 		= ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
	y_offset 	= stride * vdev->data.cif_cfg.main_set.target_y;
	uv_offset 	= ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (vdev->data.cif_cfg.main_set.target_y/2);
	total_off = y_offset + uv_offset * 2; //yuv

  	total_off = PAGE_ALIGN(total_off);
	for(i=0; i < vdev->data.cif_cfg.pp_num; i++)
	{
		vdev->data.cif_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN( base_addr + total_off*i);
		vdev->data.cif_cfg.preview_buf[i].p_Cb = (unsigned int)vdev->data.cif_cfg.preview_buf[i].p_Y + y_offset;
		vdev->data.cif_cfg.preview_buf[i].p_Cr = (unsigned int)vdev->data.cif_cfg.preview_buf[i].p_Cb + uv_offset;
	}
}

void cif_capture_dma_set(struct tcc_camera_device * vdev)
{
	dma_addr_t base_addr;
	unsigned int y_offset;
	unsigned int uv_offset = 0;
	unsigned int target_width, target_height;

	base_addr = vdev->data.cif_cfg.base_buf;
	target_width  = vdev->data.cif_cfg.main_set.target_x;
	target_height = vdev->data.cif_cfg.main_set.target_y;

	y_offset = target_width*target_height;
		
	if(vdev->data.cif_cfg.fmt == M420_ZERO)
	    uv_offset = (target_width * target_height) / 2;
	else
	    uv_offset = (target_width * target_height) / 4;

	VIOC_WDMA_SetImageBase((VIOC_WDMA*)vdev->vioc.wdma.address, (unsigned int)vdev->data.cif_cfg.capture_buf.p_Y, \
																(unsigned int)vdev->data.cif_cfg.capture_buf.p_Cb, \
																(unsigned int)vdev->data.cif_cfg.capture_buf.p_Cr);
}

void cif_set_port(struct tcc_camera_device * vdev)
{
	struct device_node	*cam_np = vdev->camera_np;
	struct device_node *port_np;
	unsigned int nUsingPort;
#if defined(CONFIG_ARCH_TCC898X)
	unsigned int nCIF_8bit_connect;
#endif
	unsigned int *cifport_addr;

	port_np = of_parse_phandle(cam_np, "camera_port", 0);
	of_property_read_u32_index(cam_np,"camera_port",1,&nUsingPort);
#if defined(CONFIG_ARCH_TCC898X)	
	of_property_read_u32_index(cam_np,"camera_port",2,&nCIF_8bit_connect);
#endif
	cifport_addr = of_iomap(port_np, 0);

	dprintk("cifport register = 0x%x \n",*cifport_addr);
#if defined(CONFIG_ARCH_TCC898X)
	BITCSET(*cifport_addr, 0xFFE77777, nUsingPort << (vdev->vioc.vin.index*4) | nCIF_8bit_connect << (20+vdev->vioc.vin.index));
#else
	BITCSET(*cifport_addr, 0x00077777, nUsingPort << (vdev->vioc.vin.index*4));
#endif
}

void tccxxx_parse_vioc_dt_data(struct tcc_camera_device * vdev)
{
	struct device_node * cam_np = vdev->camera_np;
	struct device_node * vioc_np;
	int lut_offset;

	if(cam_np)
	{
		vioc_np = of_parse_phandle(cam_np, "camera_wmixer", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_wmixer", 1, &vdev->vioc.wmixer.index);
			vdev->vioc.wmixer.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.wmixer.index);
		}
		else
		{
			printk("could not find camera wmixer node!! \n");
		}
		
		vioc_np = of_parse_phandle(cam_np, "camera_wdma", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_wdma", 1, &vdev->vioc.wdma.index);
			vdev->vioc.wdma.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.wdma.index); 
		}
		else
		{
			printk("could not find camera wdma node!! \n");
		}
		
		vioc_np = of_parse_phandle(cam_np, "camera_videoin", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_videoin", 2, &vdev->vioc.vin.index);
			vdev->vioc.vin.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.vin.index);

			switch(vdev->vioc.vin.index)
			{
				case 0:
				default:
					vdev->vioc.sc_plugin_pos = VIOC_SC_VIN_00;
					break;
				case 1:
					vdev->vioc.sc_plugin_pos = VIOC_SC_VIN_01;
					break;
			}
			of_property_read_u32_index(cam_np,"camera_videoin", 1, &lut_offset);			
			vdev->vioc.lut.address  = (unsigned int*)of_iomap(vioc_np, vdev->vioc.vin.index + lut_offset);
		}
		else
		{
			printk("could not find video input node!! \n");
		}
		
		vioc_np = of_parse_phandle(cam_np, "camera_scaler", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_scaler", 1, &vdev->vioc.scaler.index);
			vdev->vioc.scaler.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.scaler.index);
			switch(vdev->vioc.scaler.index)
			{
				case 0:
				default:
					vdev->vioc.sc_channel_num = VIOC_SC0;
					break;
				case 1:
					vdev->vioc.sc_channel_num = VIOC_SC1;
					break;
				case 2:
					vdev->vioc.sc_channel_num = VIOC_SC2;
					break;
				case 3:
					vdev->vioc.sc_channel_num = VIOC_SC3;
					break;
			}			
		}
		else
		{
			printk("could not find camera scaler node!! \n");
		}

		vioc_np = of_parse_phandle(cam_np, "camera_config", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_config", 1, &vdev->vioc.config.index);
			vdev->vioc.config.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.config.index);
		}
		else
		{
			printk("could not find camera irqe node!! \n");
		}

		vioc_np = of_parse_phandle(cam_np, "camera_viqe", 0);
		if(vioc_np) 
		{
			of_property_read_u32_index(cam_np,"camera_viqe", 1, &vdev->vioc.viqe.index);
			vdev->vioc.viqe.address = (unsigned int*)of_iomap(vioc_np, vdev->vioc.viqe.index);
		}
		else
		{
			printk("could not find camera viqe node!! \n");
		}
	}
	else
	{
		printk("could not find camera platform node!! \n");
	}
}

int tccxxx_vioc_vin_main(struct tcc_camera_device * vdev)
{
	uint width, height, offs_width, offs_height; 
	
	if(vdev->data.cif_cfg.oper_mode == OPER_PREVIEW
	|| (vdev->data.cif_cfg.oper_mode == OPER_CAPTURE && vdev->data.cif_cfg.main_set.target_x < vdev->tcc_sensor_info.cam_capchg_width)) 
	{
		offs_width 		= vdev->tcc_sensor_info.preview_zoom_offset_x;
		offs_height 	= vdev->tcc_sensor_info.preview_zoom_offset_y;
		width 			= vdev->tcc_sensor_info.preview_w;
		height 			= vdev->tcc_sensor_info.preview_h;
	}
	else 
	{
		offs_width 		= vdev->tcc_sensor_info.capture_zoom_offset_x;
		offs_height 	= vdev->tcc_sensor_info.capture_zoom_offset_y;
		width 			= vdev->tcc_sensor_info.capture_w;
		height 			= vdev->tcc_sensor_info.capture_h;
	}

	vdev->data.cif_cfg.main_set.win_hor_ofst = offs_width  * vdev->data.cif_cfg.zoom_step;
	vdev->data.cif_cfg.main_set.win_ver_ofst = offs_height * vdev->data.cif_cfg.zoom_step;
	vdev->data.cif_cfg.main_set.source_x 	= width  - (offs_width  * vdev->data.cif_cfg.zoom_step) * 2;
	vdev->data.cif_cfg.main_set.source_y 	= height - (offs_height * vdev->data.cif_cfg.zoom_step) * 2;
	dprintk("%s():  width=%d, height=%d, offset_x=%d, offset_y=%d. \n", __FUNCTION__, vdev->data.cif_cfg.main_set.source_x, \
			vdev->data.cif_cfg.main_set.source_y, vdev->data.cif_cfg.main_set.win_hor_ofst, vdev->data.cif_cfg.main_set.win_ver_ofst);

	if(vdev->vioc.wmixer.index == 5) 
		VIOC_CONFIG_WMIXPath(WMIX50, OFF);	// VIN01 means LUT of VIN0 component
	else if(vdev->vioc.wmixer.index == 6) 
		VIOC_CONFIG_WMIXPath(WMIX60, OFF);	// VIN11 means LUT of VIN1 component

	VIOC_WMIX_SetUpdate((VIOC_WMIX *)vdev->vioc.wmixer.address);

	VIOC_VIN_SetSyncPolarity((VIOC_VIN *)vdev->vioc.vin.address, !(vdev->data.cif_cfg.polarity_href),	!(vdev->data.cif_cfg.polarity_vsync), \
			vdev->data.cif_cfg.field_bfield_low, vdev->data.cif_cfg.polarity_de, vdev->data.cif_cfg.gen_field_en,!(vdev->data.cif_cfg.polarity_pclk));
	VIOC_VIN_SetCtrl((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.conv_en, vdev->data.cif_cfg.hsde_connect_en, \
			vdev->data.cif_cfg.vs_mask, vdev->data.cif_cfg.input_fmt, vdev->data.cif_cfg.data_order);
	VIOC_VIN_SetInterlaceMode((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.intl_en, vdev->data.cif_cfg.intpl_en);
	VIOC_VIN_SetImageSize((VIOC_VIN *)vdev->vioc.vin.address, width, height);
	VIOC_VIN_SetImageOffset((VIOC_VIN *)vdev->vioc.vin.address, 128, 0, 0);
	VIOC_VIN_SetImageCropSize((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.main_set.source_x, vdev->data.cif_cfg.main_set.source_y);
	VIOC_VIN_SetImageCropOffset((VIOC_VIN *)vdev->vioc.vin.address, vdev->data.cif_cfg.main_set.win_hor_ofst, vdev->data.cif_cfg.main_set.win_ver_ofst);
	VIOC_VIN_SetY2RMode((VIOC_VIN *)vdev->vioc.vin.address, 2);

	VIOC_VIN_SetY2REnable((VIOC_VIN *)vdev->vioc.vin.address, OFF);
	VIOC_VIN_SetLUT((VIOC_VIN *)vdev->vioc.vin.address, vdev->vioc.lut.address);
	VIOC_VIN_SetLUTEnable((VIOC_VIN *)vdev->vioc.vin.address, OFF, OFF, OFF);
	VIOC_VIN_SetEnable((VIOC_VIN *)vdev->vioc.vin.address, ON);

	return 0;
}

int tccxxx_vioc_vin_wdma_set(struct tcc_camera_device * vdev)
{
	uint dw, dh; 	// destination image size
	uint fmt; 		// image format

	dw = vdev->data.cif_cfg.main_set.target_x;
	dh = vdev->data.cif_cfg.main_set.target_y;
	
	if(vdev->data.cif_cfg.oper_mode == OPER_CAPTURE)
	{
		fmt = VIOC_IMG_FMT_YUV420SEP;
		cif_capture_dma_set(vdev);
	}
	else
	{
		if(vdev->pix_format.pixelformat == V4L2_PIX_FMT_YVU420)	
			fmt = VIOC_IMG_FMT_YUV420SEP;
		else
			fmt = VIOC_IMG_FMT_YUV420IL1;
	}

	dprintk("%s():  WDMA size[%dx%d], format[%d]. \n", __FUNCTION__, dw, dh, fmt);
	
	VIOC_WDMA_SetImageFormat((VIOC_WDMA*)vdev->vioc.wdma.address, fmt);
	VIOC_WDMA_SetImageSize((VIOC_WDMA*)vdev->vioc.wdma.address, dw, dh);
	
	if((vdev->pix_format.pixelformat == V4L2_PIX_FMT_YVU420 && (dw/2)%C_STRIDE_ALIGN!=0) && vdev->data.cif_cfg.oper_mode == OPER_PREVIEW)
	{
		VIOC_WDMA_SetImageOffset_withYV12((VIOC_WDMA*)vdev->vioc.wdma.address,dw);
	}
	else
	{
		VIOC_WDMA_SetImageOffset((VIOC_WDMA*)vdev->vioc.wdma.address, fmt, dw);
	}
	if(vdev->data.cif_cfg.oper_mode == OPER_CAPTURE) 
	{
		dprintk("While capture, frame by frame mode. \n");
		VIOC_WDMA_SetImageEnable((VIOC_WDMA*)vdev->vioc.wdma.address, OFF);	// operating start in 1 frame
		VIOC_WDMA_ClearEORF((VIOC_WDMA*)vdev->vioc.wdma.address); // clear EORF bit
		VIOC_WDMA_SetIreqMask((VIOC_WDMA*)vdev->vioc.wdma.address, VIOC_WDMA_IREQ_EOFR_MASK, 0x0);
	}
	else
	{
		dprintk("While preview, continuous mode. \n");
		VIOC_WDMA_SetImageEnable((VIOC_WDMA*)vdev->vioc.wdma.address, ON);	// operating start in 1 frame
		VIOC_WDMA_ClearEOFF((VIOC_WDMA*)vdev->vioc.wdma.address); // clear EOFF bit
		VIOC_WDMA_SetIreqMask((VIOC_WDMA*)vdev->vioc.wdma.address, VIOC_WDMA_IREQ_EOFF_MASK, 0x0);
	}
	cif_dma_hw_reg(0, vdev);

	return 0;
}

int tccxxx_vioc_scaler_set(struct tcc_camera_device * vdev)
{
#if defined(FEATURE_PREVIEW_RATIO_720P) 	
	uint preview_ratio = 0;
#endif
	uint dw, dh;			// destination size in SCALER
	uint width, height, off_x, off_y;
	uint mw, mh;			// image size in WMIX

	if(vdev->data.cif_cfg.oper_mode == OPER_PREVIEW
	|| (vdev->data.cif_cfg.oper_mode == OPER_CAPTURE && vdev->data.cif_cfg.main_set.target_x < vdev->tcc_sensor_info.cam_capchg_width)) 
	{
		width 		= vdev->tcc_sensor_info.preview_w;
		height 		= vdev->tcc_sensor_info.preview_h;
		off_x 		= vdev->tcc_sensor_info.preview_zoom_offset_x;
		off_y 		= vdev->tcc_sensor_info.preview_zoom_offset_y;
	}
	else
	{
		width 		= vdev->tcc_sensor_info.capture_w;
		height 		= vdev->tcc_sensor_info.capture_h;
		off_x 		= vdev->tcc_sensor_info.capture_zoom_offset_x;
		off_y 		= vdev->tcc_sensor_info.capture_zoom_offset_y;
	}

	dw = vdev->data.cif_cfg.main_set.target_x;
	dh = vdev->data.cif_cfg.main_set.target_y;

	if(vdev->sc_plugin_status)
	{
		VIOC_CONFIG_PlugOut(vdev->vioc.sc_channel_num);
		vdev->sc_plugin_status = OFF;
	}

// modify to 720p preview that mismatch ratio
#if defined(FEATURE_PREVIEW_RATIO_720P) 

	// check to ratio 4:3
	if((vdev->data.cif_cfg.oper_mode == OPER_PREVIEW) && ((dw*3/4) != dh)) 
	{
		preview_ratio = 1; 
		dprintk("%s():  mismatch preview ratio. \n", __FUNCTION__); 
	}
	
	if((vdev->data.cif_cfg.oper_mode == OPER_PREVIEW) && (dw > width)) 
	{
		width  = dw; 
		height = width * 3 / 4; 
	}

#endif

	/* PlugIn path 
     *
	 * VIOC_SC_VIN_00  : VIN0 - SCALE - WMIX
	 * VIOC_SC_VIN_01  : VIN1 - SCALE - WMIX
	 *
	 * VIOC_SC_WDMA_05 : VIN0 - WMIX - SCALE
	 * VIOC_SC_WDMA_06 : VIN0 - WMIX - SCALE
	 * VIOC_SC_WDMA_07 : VIN1 - WMIX - SCALE
 	 * VIOC_SC_WDMA_08 : VIN1 - WMIX - SCALE
	 *
	 */
	switch(vdev->vioc.sc_plugin_pos)
	{
		case VIOC_SC_VIN_00:
		case VIOC_SC_VIN_01:
		default:
			mw = vdev->data.cif_cfg.main_set.target_x;
			mh = vdev->data.cif_cfg.main_set.target_y;
			break;

		case VIOC_SC_WDMA_05:
		case VIOC_SC_WDMA_06:
		case VIOC_SC_WDMA_07:
		case VIOC_SC_WDMA_08:
			mw 	= vdev->tcc_sensor_info.preview_w;
			mh 	= vdev->tcc_sensor_info.preview_h;
			break;
	}

	#if defined (CONFIG_VIDEO_TCCXX_ATV) //http://gerrit.daudio/#/c/22079/ 2016.6.07 mhjung merge
	{
		#if defined(CONFIG_VIDEO_ATV_SENSOR_TVP5150) /*|| defined(CONFIG_VIDEO_ATV_SENSOR_DAUDIO)*/
               if (vdev->data.cam_info== DAUDIO_CAMERA_CMMB) {     //for CMMB
                       mw = dw + DAUDIO_CMMB_CROP_X;
                       mh = dh + DAUDIO_CMMB_CROP_Y;
               } else {
				   if ((vdev->data.cam_info == DAUDIO_CAMERA_AUX) && (data->cif_cfg.encode == 1))
				   		mh = dh + 12;
				   else
			   			mh = dh + 6;
				   mw = dw;
               }
		#else 

			mw = dw;
			mh = dh + 6;
		#endif		
	}
	#endif

	VIOC_CONFIG_PlugIn(vdev->vioc.sc_channel_num, vdev->vioc.sc_plugin_pos); 
	VIOC_SC_SetBypass((VIOC_SC *)vdev->vioc.scaler.address, OFF); 	
	
#if defined (CONFIG_VIDEO_TCCXX_ATV)

	VIOC_SC_SetDstSize((VIOC_SC *)vdev->vioc.scaler.address, mw, mh); 
	VIOC_SC_SetOutPosition((VIOC_SC *)vdev->vioc.scaler.address, (mw-dw), (mh-dh));
//	VIOC_SC_SetOutPosition((VIOC_SC *)vdev->vioc.scaler.address, 0, 0);

#else

	#if defined(FEATURE_PREVIEW_RATIO_720P) 
	if(preview_ratio) 
	{ 
		VIOC_SC_SetDstSize((VIOC_SC *)vdev->vioc.scaler.address, width, height); 
		VIOC_SC_SetOutPosition((VIOC_SC *)vdev->vioc.scaler.address, ((width-dw)/2), ((height-dh)/2)); 
	}
	else 
	#endif		
	{ 
		VIOC_SC_SetDstSize((VIOC_SC *)vdev->vioc.scaler.address, (dw + 2), (dh + 2)); 
		VIOC_SC_SetOutPosition((VIOC_SC *)vdev->vioc.scaler.address, 1, 1);
	}

#endif
	
	VIOC_SC_SetOutSize((VIOC_SC *)vdev->vioc.scaler.address, dw, dh); 
	VIOC_SC_SetUpdate((VIOC_SC *)vdev->vioc.scaler.address); 

	vdev->sc_plugin_status = ON;

	return 0;
}

int tccxxx_vioc_viqe_main(struct tcc_camera_device *vdev)
{
	struct device_node	*cam_np = vdev->camera_np, * handler_np = NULL;
	VIQE * pVIQE = (VIQE *)vdev->vioc.viqe.address;

	unsigned int	viqe_width	= 0;
	unsigned int	viqe_height	= 0;
	unsigned int	format		= FMT_FC_YUV420;
	unsigned int	bypass_deintl	= VIOC_VIQE_DEINTL_MODE_2D;
	unsigned int	offset			= vdev->tcc_sensor_info.preview_w*vdev->tcc_sensor_info.preview_h* 2 * 2;
	unsigned int	deintl_base0	= vdev->pmap_viqe.base;
	unsigned int	deintl_base1	= deintl_base0 + offset;
	unsigned int	deintl_base2	= deintl_base1 + offset;
	unsigned int	deintl_base3	= deintl_base2 + offset;
	unsigned int	cdf_lut_en	= OFF;
	unsigned int	his_en		= OFF;
	unsigned int	gamut_en	= OFF;
	unsigned int	d3d_en		= OFF;
	unsigned int	deintl_en	= ON;
	unsigned int*	viqe_set_reg1 = NULL;
	unsigned int*	viqe_set_reg2 = NULL;

	if(!(handler_np = of_parse_phandle(cam_np, "camera_viqe_set", 0))) 
	{
		printk("could not find cam_viqe_set node!! \n");

	}
	else
	{
		viqe_set_reg1 = (unsigned int*)of_iomap(handler_np, 0);
		viqe_set_reg2 = (unsigned int*)of_iomap(handler_np, 1);
	
		BITCSET(*viqe_set_reg1,1<<3,1<<3);
		BITCSET(*viqe_set_reg2,1<<8 | 1<<9 , 1<<8 | 1<<9);
	}
	
	VIOC_VIQE_SetControlRegister(pVIQE, viqe_width, viqe_height, format);
	VIOC_VIQE_SetDeintlRegister(pVIQE, format, OFF, viqe_width, viqe_height, bypass_deintl,	\
					deintl_base0, deintl_base1, deintl_base2, deintl_base3);
	VIOC_VIQE_SetControlEnable(pVIQE, cdf_lut_en, his_en, gamut_en, d3d_en, deintl_en);

	// YUV422 Format
	BITCSET(pVIQE->cDEINTL_COMP.nVIQE_FC_MISC.nREG, 0x0000f000, 			(1 << 12)); 		// 0x160
	BITCSET(pVIQE->cDEINTL.nDI_FMT,					0x00000001, 			(1 <<  0)); 		// 0x2E8
	
	// Like Weave Mode
	BITCSET(pVIQE->cDEINTL.nDI_ENGINE0, 			0xffffffff, 			0x0204ff08);		// 0x284
	BITCSET(pVIQE->cDEINTL.nDI_ENGINE4, 			0xffffffff, 			0x124f2582);		// 0x294
	BITCSET(pVIQE->cDEINTL.nDI_CTRL,	((1<<5)|(1<<4)|(1<<0)), ((0<<5)|(0<<4)|(0<<0)));		// 0x280
	BITCSET(pVIQE->cDEINTL.nDI_ENGINE3, 			0xfff00000, 				(0<<20));		// 0x290

	return 0;
}

static int tcc_cif_pmap_ctrl(int video_nr, pmap_t *pmap, unsigned int width, unsigned int height, int total_buf)
{
	int ret = 0;

	/* get pmap v4l2_videoX */
	char name[36];
	sprintf(name, "v4l2_video%d", video_nr);
//	strcpy(name, "video");
	pmap_get_info(name, pmap);
	dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
		name, pmap->base, pmap->base + pmap->size, pmap->size);

	return ret;
}

int tccxxx_cif_buffer_set(struct tcc_camera_device * vdev, struct v4l2_requestbuffers *req)
{
	int ret = 0, i;
	struct tccxxx_cif_buffer *buf = NULL;
	unsigned int y_offset = 0, uv_offset = 0, stride = 0;
	unsigned int buff_size = 0;

	// routine for tccvin
	if(req->reserved[0] == 0)
	{
		ret = tcc_cif_pmap_ctrl/*pdata->cif_pmap_ctrl*/(vdev->vfd->minor, &vdev->pmap_preview, 
						vdev->tcc_sensor_info.preview_w, vdev->tcc_sensor_info.preview_h, TCC_CAMERA_MIN_BUFNBRS);
		if (ret)
			return -ENOMEM;

		vdev->data.cif_cfg.base_buf = vdev->pmap_preview.base;

		vdev->data.buf = vdev->data.static_buf;
		vdev->data.cif_buf.bytes = PAGE_ALIGN(vdev->pmap_preview.size);
		vdev->data.cif_buf.addr = vdev->pmap_preview.base;
		vdev->data.cif_cfg.now_frame_num = 0;

		if (req->count > TCC_CAMERA_MAX_BUFNBRS) 
			req->count = TCC_CAMERA_MAX_BUFNBRS;
		if (req->count < 0)
			req->count = TCC_CAMERA_MIN_BUFNBRS;

		stride = ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
		y_offset = stride * vdev->data.cif_cfg.main_set.target_y;
		uv_offset = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (vdev->data.cif_cfg.main_set.target_y/2);

		if(vdev->data.cif_cfg.fmt == M420_ZERO)	
			buff_size = PAGE_ALIGN(y_offset*2);
		else								
			buff_size = PAGE_ALIGN(y_offset + y_offset/2);
		
		vdev->data.buf->v4lbuf.length = buff_size;

retry:
		if (vdev->data.buf->v4lbuf.length * req->count > vdev->pmap_preview.size) {
			dprintk("REQUBUF error\n");
			req->count--;
			if (req->count <= 0) {
				printk("reqbufs: count invalid\n");
				return -1;
			}
			goto retry;
		}

		memset(vdev->data.static_buf, 0x00, req->count * sizeof(struct tccxxx_cif_buffer));

		vdev->data.done_list.prev = vdev->data.done_list.next = &(vdev->data.done_list);
		vdev->data.list.prev 	 = vdev->data.list.next 		= &(vdev->data.list);

		for (vdev->data.n_sbufs = 0; vdev->data.n_sbufs < req->count; (vdev->data.n_sbufs++)) {
			struct tccxxx_cif_buffer *buf = &(vdev->data.static_buf[vdev->data.n_sbufs]);
		
			INIT_LIST_HEAD(&buf->buf_list);
		
			buf->v4lbuf.length = buff_size;
			buf->mapcount = 0;
			buf->cam = &vdev->data;
			buf->v4lbuf.index = vdev->data.n_sbufs;
			buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf->v4lbuf.field = V4L2_FIELD_NONE;
			buf->v4lbuf.memory = V4L2_MEMORY_MMAP;
		
			/* Offset: must be 32-bit even on a 64-bit system. 
			 * videobuf-dma-sg just uses the length times the index,
			 * but the spec warns against doing just that - vma merging problems.
			 * So we leave a gap between each pair of buffers.
			 */
			//buf->v4lbuf.m.offset = 2 * index * buf->v4lbuf.length;	
		}
		dprintk("-----------------------------------------------------\n");
		dprintk("v4l2_buffer: 0x%x * %d = 0x%x\n", buff_size, vdev->data.n_sbufs, buff_size * vdev->data.n_sbufs);
		
		vdev->data.cif_cfg.pp_num = vdev->data.n_sbufs;
		req->count = vdev->data.cif_cfg.pp_num;

		for(i = 0; i < vdev->data.cif_cfg.pp_num; i++) {
			vdev->data.cif_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN(vdev->data.cif_cfg.base_buf + buff_size * i);
			vdev->data.cif_cfg.preview_buf[i].p_Cb = (unsigned int)vdev->data.cif_cfg.preview_buf[i].p_Y + y_offset;
			vdev->data.cif_cfg.preview_buf[i].p_Cr = (unsigned int)vdev->data.cif_cfg.preview_buf[i].p_Cb + uv_offset;
			
			dprintk("	  [%d]		0x%08x 0x%08x 0x%08x\n", i,
				vdev->data.cif_cfg.preview_buf[i].p_Y, vdev->data.cif_cfg.preview_buf[i].p_Cb, vdev->data.cif_cfg.preview_buf[i].p_Cr);
		}
		dprintk("-----------------------------------------------------\n");
	}
	else // routine for tcccam
	{
		if(req->count == 0) 
		{
			vdev->data.cif_cfg.now_frame_num = 0;

			stride = ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
			y_offset = stride * vdev->data.cif_cfg.main_set.target_y;

			if(vdev->data.cif_cfg.fmt == M420_ZERO) 	
				buff_size = PAGE_ALIGN(y_offset*2);
			else 								
				buff_size = PAGE_ALIGN(y_offset + y_offset/2);
		
			vdev->data.buf->v4lbuf.length = buff_size;

			memset(vdev->data.static_buf, 0x00, req->count * sizeof(struct tccxxx_cif_buffer));

			vdev->data.done_list.prev = vdev->data.done_list.next = &(vdev->data.done_list);
			vdev->data.list.prev 	 = vdev->data.list.next 		= &(vdev->data.list);
			vdev->data.cif_cfg.base_buf = (unsigned int)req->reserved[0];
		}

		buf = &(vdev->data.static_buf[req->count]);

		INIT_LIST_HEAD(&buf->buf_list);

		buf->mapcount 		= 0;
		buf->cam 			= &vdev->data;
		buf->v4lbuf.index 	= req->count;
		buf->v4lbuf.type 	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf->v4lbuf.field 	= V4L2_FIELD_NONE;
		buf->v4lbuf.memory 	= V4L2_MEMORY_MMAP;
		buf->v4lbuf.length 	= buff_size;

		stride = ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
		y_offset = stride * vdev->data.cif_cfg.main_set.target_y;
		
		uv_offset = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (vdev->data.cif_cfg.main_set.target_y/2);
		
		vdev->data.cif_cfg.preview_buf[req->count].p_Y  = (unsigned int)req->reserved[0];
		vdev->data.cif_cfg.preview_buf[req->count].p_Cb = (unsigned int)vdev->data.cif_cfg.preview_buf[req->count].p_Y + y_offset;
		vdev->data.cif_cfg.preview_buf[req->count].p_Cr = (unsigned int)vdev->data.cif_cfg.preview_buf[req->count].p_Cb + uv_offset;
			
		vdev->data.cif_cfg.pp_num = req->count;
	}
	return 0;
}

void tccxxx_set_camera_addr(struct tcc_camera_device * vdev,int index, unsigned int addr, unsigned int cameraStatus)
{
	unsigned int y_offset = 0, uv_offset = 0, stride = 0;
	
	if(cameraStatus == 1 /* MODE_PREVIEW */) 
	{
		stride = ALIGNED_BUFF(vdev->data.cif_cfg.main_set.target_x, L_STRIDE_ALIGN);
		y_offset = stride * vdev->data.cif_cfg.main_set.target_y;
		uv_offset = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (vdev->data.cif_cfg.main_set.target_y/2);
			
		vdev->data.cif_cfg.preview_buf[index].p_Y  = addr;
		vdev->data.cif_cfg.preview_buf[index].p_Cb = (unsigned int)vdev->data.cif_cfg.preview_buf[index].p_Y + y_offset;
		vdev->data.cif_cfg.preview_buf[index].p_Cr = (unsigned int)vdev->data.cif_cfg.preview_buf[index].p_Cb + uv_offset;
	
	}
	else if(cameraStatus == 3 /* MODE_CAPTURE */) 
	{
		y_offset = vdev->data.cif_cfg.main_set.target_x * vdev->data.cif_cfg.main_set.target_y;
		
		if(vdev->data.cif_cfg.fmt == M420_ZERO)
			uv_offset = vdev->data.cif_cfg.main_set.target_x * vdev->data.cif_cfg.main_set.target_y / 2;
		else
			uv_offset = vdev->data.cif_cfg.main_set.target_x * vdev->data.cif_cfg.main_set.target_y / 4;

		vdev->data.cif_cfg.capture_buf.p_Y  = addr;
		vdev->data.cif_cfg.capture_buf.p_Cb = (unsigned int)vdev->data.cif_cfg.capture_buf.p_Y + y_offset;
		vdev->data.cif_cfg.capture_buf.p_Cr = (unsigned int)vdev->data.cif_cfg.capture_buf.p_Cb + uv_offset;
	}
}

int tccxxx_cif_start_stream(struct tcc_camera_device * vdev) {
	VIOC_IREQ_CONFIG *	pViocConfig	= (VIOC_IREQ_CONFIG *)vdev->vioc.config.address;
	VIOC_SWRESET_Component	deinterlacer	= VIOC_CONFIG_VIQE;	// init
	int			idxDeinterlacer = 0;			// init
	
	dprintk("%s - In\n", __func__);
	
	// size info
	dprintk("src: %d * %d\n", vdev->data.cif_cfg.main_set.source_x,		vdev->data.cif_cfg.main_set.source_y);
	dprintk("ofs: %d * %d\n", vdev->data.cif_cfg.main_set.win_hor_ofst,	vdev->data.cif_cfg.main_set.win_ver_ofst);
	dprintk("scl: %d * %d\n", vdev->data.cif_cfg.main_set.scaler_x,		vdev->data.cif_cfg.main_set.scaler_y);
	dprintk("tgt: %d * %d\n", vdev->data.cif_cfg.main_set.target_x,		vdev->data.cif_cfg.main_set.target_y);

	vdev->data.cif_cfg.oper_mode  = OPER_PREVIEW;
	vdev->data.cif_cfg.cap_status = CAPTURE_NONE;

	
	// select deinterlacer
	if(vdev->data.cif_cfg.intl_en) {
		deinterlacer	= VIOC_CONFIG_VIQE;		// default
		idxDeinterlacer = vdev->vioc.viqe.index;	// default
	}

	sensor_if_change_mode(vdev,OPER_PREVIEW);
	mdelay(100);
	
	// VIOC reset
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WDMA,	vdev->vioc.wdma.index,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WMIXER,	vdev->vioc.wmixer.index, VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_SCALER,	vdev->vioc.scaler.index, VIOC_CONFIG_RESET);
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset(pViocConfig, deinterlacer,	idxDeinterlacer,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_VIN,	vdev->vioc.vin.index,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_VIN,	vdev->vioc.vin.index,	 VIOC_CONFIG_CLEAR);
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset(pViocConfig, deinterlacer,	idxDeinterlacer,	 VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_SCALER,	vdev->vioc.scaler.index, VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WMIXER,	vdev->vioc.wmixer.index, VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WDMA,	vdev->vioc.wdma.index,	 VIOC_CONFIG_CLEAR);
	
	vdev->data.stream_state = STREAM_ON;

	vdev->skipped_frm 	= 0;
	vdev->gZoomStep 	= 0;
	vdev->prev_buf 		= NULL;

	tccxxx_vioc_vin_main(vdev);

	if(vdev->data.cif_cfg.intl_en) {
		unsigned int plug_type	= (deinterlacer	== VIOC_CONFIG_VIQE) ? VIOC_VIQE : VIOC_DEINTLS;
		unsigned int plug_value	= (vdev->vioc.vin.index == 0) ? VIOC_VIQE_VIN_00 : VIOC_VIQE_VIN_01;
		
//		vdev->frm_cnt 		= 0;
//		vdev->bfield 		= 0;
//		vdev->field_cnt 	= 0;

		VIOC_CONFIG_PlugIn(plug_type, plug_value);

		switch(deinterlacer) {
		case VIOC_CONFIG_VIQE:
			tccxxx_vioc_viqe_main(vdev);
			break;
		case VIOC_CONFIG_DEINTS:
#if defined (CONFIG_ARCH_TCC897X)
			VIOC_DEINTLS_SetDeIntlMode(vdev->vioc.deintls.address, 3);
#endif
			break;
		default :
			//do nothing...
			break;
		}
		
		vdev->deint_plugin_status = ON;
	}

	tccxxx_vioc_scaler_set(vdev);

	tccxxx_vioc_vin_wdma_set(vdev);
	mdelay(10);
	
	vdev->cam_streaming++;
	vdev->preview_method = PREVIEW_V4L2;
	log("cam_streaming: %d, preview_method: %d\n", vdev->cam_streaming, vdev->preview_method);
	
	dprintk("%s - Out\n", __func__);
	return 0;
}

int tccxxx_cif_stop_stream(struct tcc_camera_device * vdev)
{
	VIOC_IREQ_CONFIG * pViocConfig = (VIOC_IREQ_CONFIG *)vdev->vioc.config.address;
	VIOC_SWRESET_Component	deinterlacer	= VIOC_CONFIG_VIQE;	// init
	int idxDeinterlacer = 0;			// init
	int idxLoop;
	unsigned int status;
	
	dprintk("%s - In\n", __func__);

	mutex_lock(&(vdev->data.lock));
	
	vdev->cam_streaming--;
	if(0 < vdev->cam_streaming) {
		printk("[%s] vdev->cam_streaming : %d \n", __FUNCTION__, vdev->cam_streaming);
		return 0;
	}

	// select deinterlacer
	if(vdev->data.cif_cfg.intl_en) {
		deinterlacer	= VIOC_CONFIG_VIQE;		// default
		idxDeinterlacer = vdev->vioc.viqe.index;	// default
	}

	vdev->gZoomStep = vdev->data.cif_cfg.zoom_step;
	vdev->data.cif_cfg.zoom_step = 0;

	VIOC_WDMA_SetIreqMask((VIOC_WDMA *)vdev->vioc.wdma.address, VIOC_WDMA_IREQ_ALL_MASK, 0x1);	// disable WDMA interrupt
	VIOC_WDMA_SetImageDisable((VIOC_WDMA *)vdev->vioc.wdma.address);

	for(idxLoop=0; idxLoop<20; idxLoop++) {
		VIOC_WDMA_GetStatus((VIOC_WDMA *)vdev->vioc.wdma.address, &status);
		if(status & VIOC_WDMA_IREQ_EOFR_MASK)
			break;
		else
			msleep(1);
	}

	if(vdev->sc_plugin_status) {
		VIOC_CONFIG_PlugOut(vdev->vioc.sc_channel_num);
		vdev->sc_plugin_status = OFF;
	}

	if(vdev->deint_plugin_status) {
		if(deinterlacer == VIOC_CONFIG_VIQE)
			VIOC_CONFIG_PlugOut(VIOC_VIQE);
		else
			VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
		vdev->deint_plugin_status = OFF;
	}
	
	VIOC_VIN_SetEnable((VIOC_VIN *)vdev->vioc.vin.address, OFF); // disable VIN

	// VIOC reset
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WDMA,	vdev->vioc.wdma.index,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WMIXER,	vdev->vioc.wmixer.index, VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_SCALER,	vdev->vioc.scaler.index, VIOC_CONFIG_RESET);
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset(pViocConfig, deinterlacer,	idxDeinterlacer,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_VIN,	vdev->vioc.vin.index,	 VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_VIN,	vdev->vioc.vin.index,	 VIOC_CONFIG_CLEAR);
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset(pViocConfig, deinterlacer,	idxDeinterlacer,	 VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_SCALER,	vdev->vioc.scaler.index, VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WMIXER,	vdev->vioc.wmixer.index, VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(pViocConfig, VIOC_CONFIG_WDMA,	vdev->vioc.wdma.index,	 VIOC_CONFIG_CLEAR);

	vdev->data.stream_state = STREAM_OFF;	

	mutex_unlock(&(vdev->data.lock));
	
	dprintk("\n\n SKIPPED FRAME = %d \n\n", vdev->skipped_frm);
	dprintk("%s - Out\n", __func__);

	return 0;
}

int tccxxx_cif_capture(int quality,struct tcc_camera_device * vdev)
{
	VIOC_SWRESET_Component	deinterlacer	= VIOC_CONFIG_VIQE;	// init
	unsigned int plug_type = (deinterlacer == VIOC_CONFIG_VIQE) ? VIOC_VIQE : VIOC_DEINTLS;
	int idxDeinterlacer = 0;			// init
	int skip_frame = 0;

	dprintk("%s Start!! \n", __FUNCTION__);

	if(vdev->data.cif_cfg.intl_en) {
		deinterlacer	= VIOC_CONFIG_VIQE;		// default
		idxDeinterlacer	= vdev->vioc.viqe.index;	// default
		plug_type = (deinterlacer == VIOC_CONFIG_VIQE) ? VIOC_VIQE : VIOC_DEINTLS;
	}
	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_WDMA,vdev->vioc.wdma.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_WMIXER,vdev->vioc.wmixer.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_SCALER,vdev->vioc.scaler.index,VIOC_CONFIG_RESET);	
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,deinterlacer,idxDeinterlacer,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_VIN,vdev->vioc.vin.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_VIN,vdev->vioc.vin.index,VIOC_CONFIG_CLEAR);
	if(vdev->data.cif_cfg.intl_en)
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,deinterlacer,idxDeinterlacer,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_SCALER,vdev->vioc.scaler.index,VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_WMIXER,vdev->vioc.wmixer.index,VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)vdev->vioc.config.address,VIOC_CONFIG_WDMA,vdev->vioc.wdma.index,VIOC_CONFIG_CLEAR);		
	
	vdev->data.cif_cfg.oper_mode = OPER_CAPTURE;
	vdev->data.cif_cfg.zoom_step = vdev->gZoomStep;
	dprintk("%s() -  Zoom Step is %d. \n", __func__, vdev->data.cif_cfg.zoom_step);
	
	memset(&(vdev->data.cif_cfg.jpg_info), 0x00, sizeof(TCCXXX_JPEG_ENC_DATA));

	if(vdev->data.cif_cfg.main_set.target_x >= vdev->tcc_sensor_info.cam_capchg_width && !(vdev->data.cif_cfg.retry_cnt)) 
	{
		sensor_if_change_mode(vdev, OPER_CAPTURE);
	}

	// for Rotate Capture
	vdev->data.cif_cfg.jpg_info.start_phy_addr = vdev->data.cif_cfg.base_buf;
	
	//capture config
	if(vdev->data.cif_cfg.retry_cnt) 	
	{
		skip_frame = 0;
	}
	else
	{
		skip_frame = vdev->tcc_sensor_info.capture_skip_frame;
	}

	cif_set_frameskip(vdev,skip_frame, 0);	
	tccxxx_vioc_vin_main(vdev);
	tccxxx_vioc_scaler_set(vdev);
	tccxxx_vioc_vin_wdma_set(vdev);
	vdev->data.cif_cfg.cap_status = CAPTURE_NONE;

	dprintk("%s End!! \n", __FUNCTION__);

	return 0;
}
		

int tccxxx_cif_set_zoom(unsigned char arg, struct tcc_camera_device * vdev)
{
	vdev->data.cif_cfg.zoom_step = arg;

	if(vdev->data.stream_state != STREAM_OFF) 
	{
		dprintk("zoom level = %d. \n", vdev->data.cif_cfg.zoom_step);
		cif_scaler_calc(vdev);
	}

	return 0;
}

int tccxxx_cif_set_resolution(struct tcc_camera_device * vdev, unsigned int pixel_fmt, unsigned short width, unsigned short height)
{
	if(pixel_fmt == V4L2_PIX_FMT_YUYV)			vdev->data.cif_cfg.fmt = M420_ZERO;  // yuv422
	else if(pixel_fmt == V4L2_PIX_FMT_RGB32)	vdev->data.cif_cfg.fmt = ARGB8888;  
	else if(pixel_fmt == V4L2_PIX_FMT_RGB565) vdev->data.cif_cfg.fmt = RGB565;
	else 										vdev->data.cif_cfg.fmt = M420_ODD; 	// yuv420

	vdev->data.cif_cfg.main_set.target_x = width;
	vdev->data.cif_cfg.main_set.target_y = height;

	return 0;
}

int tccxxx_cif_irq_request(struct tcc_camera_device * vdev)
{
	struct device_node *cam_np = vdev->camera_np;
	struct device_node *irq_np;
	int ret = -1, irq_num;

	dprintk("[%s] in \n", __func__);

	irq_np = of_parse_phandle(cam_np, "camera_wdma", 0);
	
	if(irq_np) 
	{
		of_property_read_u32_index(cam_np,"camera_wdma", 1, &(vdev->vioc.wdma.index));
		irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.wdma.index);

		vdev->data.vioc_intr.id = VIOC_INTR_WD0 + vdev->vioc.wdma.index;
		vdev->data.vioc_intr.bits = ((1 << VIOC_WDMA_INTR_EOFR) | (1 << VIOC_WDMA_INTR_EOFF));
		vioc_intr_clear(vdev->data.vioc_intr.id, vdev->data.vioc_intr.bits);
		ret = request_irq(irq_num, cif_cam_isr, IRQF_SHARED, CAM_NAME, vdev);
	}

	dprintk("[%s] out \n", __func__);
	return ret;
}

void tccxxx_cif_irq_free(struct tcc_camera_device * vdev)
{
	struct device_node *cam_np = vdev->camera_np;
	struct device_node *irq_np;
	int irq_num;
	
	irq_np = of_parse_phandle(cam_np, "camera_wdma", 0);
	if(irq_np) 
	{
		of_property_read_u32_index(cam_np,"camera_wdma", 1, &(vdev->vioc.wdma.index));
		irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.wdma.index);
		free_irq(irq_num, vdev);
	}
}

int tccxxx_cif_open(struct tcc_camera_device * vdev)
{
	int ret = 0;

	vdev->data.done_list.next = &(vdev->data.done_list);
	vdev->data.list.next = &(vdev->data.list);

	return ret;
}

int tccxxx_cif_close(struct tcc_camera_device * vdev)
{
	mutex_destroy(&(vdev->data.lock));
	cif_cleanup(vdev);

	if(vdev->sensor_status == ENABLE)
		vdev->sensor_status  = DISABLE;

	if(vdev->cam_irq)
		tccxxx_cif_irq_free(vdev);

	vdev->cam_irq = DISABLE;

	return 0;
}

int tcc_get_sensor_info(struct tcc_camera_device * vdev, int index)
{
	sensor_if_set(vdev,index);
	return 0;
}

int tccxxx_cif_init(struct tcc_camera_device * vdev) {
	dprintk("%s - sensor_status = %d\n", __func__, vdev->sensor_status);

	if(vdev->sensor_status == DISABLE) {
		memset(&vdev->data,0x00,sizeof(struct TCCxxxCIF));

		vdev->data.buf = vdev->data.static_buf;
		vdev->sensor_status  = ENABLE;
		vdev->deint_plugin_status = OFF;
		vdev->sc_plugin_status = OFF;
		if(vdev->camera_np) {
			tccxxx_parse_vioc_dt_data(vdev);
			cif_set_port(vdev);
		} else {
			printk("%s - could not find camera node!! \n", __func__);
			return -ENODEV;	
		}
			
		/* Init the camera IF */
		cif_data_init(vdev);

		if(vdev->data.cif_cfg.intl_en)
			pmap_get_info("jpeg_raw", &vdev->pmap_viqe);
		
		init_waitqueue_head(&vdev->data.frame_wait);

		spin_lock_init(&vdev->sensor_lock);
		spin_lock_init(&vdev->data.dev_lock);
		INIT_LIST_HEAD(&vdev->data.list);
		INIT_LIST_HEAD(&vdev->data.done_list);	
		mutex_init(&vdev->data.lock);
	}
	return 0;
}

