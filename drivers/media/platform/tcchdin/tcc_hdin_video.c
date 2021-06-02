
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
 
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include <video/tcc/vioc_ireq.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_vin.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_viqe.h>
#include <video/tcc/vioc_cam_plugin.h>
#include <video/tcc/vioc_intr.h>
#include <video/tcc/tcc_gpu_align.h>

#include <soc/tcc/pmap.h>

#include "tcc_hdin_main.h"
#include "tcc_hdin_ctrl.h"
#include "tcc_hdin_video.h"


static int debug	   = 0;
#define dprintk(msg...)	if (debug) { printk( "hdin_video: " msg); }

#define USE_VIQE

static void hdin_data_init(void *priv)
{
	struct tcc_hdin_device *dev = 	(struct tcc_hdin_device *)priv;
	struct TCC_HDIN *data = &dev->data;
	struct device_node *hdin_np = dev->np;
	struct device_node *module_np;

	//Default Setting
	data->hdin_cfg.base_buf			= data->hdin_buf.addr;
	data->hdin_cfg.pp_num			= 0; 

	data->hdin_cfg.fmt				= M420_EVEN;    

	data->hdin_cfg.main_set.target_x = 0;
	data->hdin_cfg.main_set.target_y = 0;

	//vin parameter
	module_np = of_find_node_by_name(hdin_np, MODULE_NODE);
	
	if(module_np)
	{
		of_property_read_u32_index(module_np,"polarity_hsync", 0, &dev->vioc.polarity_hsync);
		of_property_read_u32_index(module_np,"polarity_vsync", 0, &dev->vioc.polarity_vsync);
		of_property_read_u32_index(module_np,"field_bfield_low", 0, &dev->vioc.field_bfield_low);
		of_property_read_u32_index(module_np,"polarity_data_en", 0, &dev->vioc.polarity_data_en);
		of_property_read_u32_index(module_np,"gen_field_en", 0, &dev->vioc.gen_field_en);
		of_property_read_u32_index(module_np,"polarity_pclk", 0, &dev->vioc.polarity_pclk);
		of_property_read_u32_index(module_np,"conv_en", 0, &dev->vioc.conv_en);
		of_property_read_u32_index(module_np,"hsde_connect_en", 0, &dev->vioc.hsde_connect_en);
		of_property_read_u32_index(module_np,"vs_mask", 0, &dev->vioc.vs_mask);
		of_property_read_u32_index(module_np,"input_fmt", 0, &dev->vioc.input_fmt);
		of_property_read_u32_index(module_np,"data_order", 0, &dev->vioc.data_order);
		of_property_read_u32_index(module_np,"intpl_en", 0, &dev->vioc.intpl_en);
	}

}


int hdin_vioc_vin_main(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = 	vdev;
	uint width, height; 

	width 			= dev->input_width;
	height 			= dev->input_height;

	if(dev->vioc.wmixer.index == 5) 
		VIOC_CONFIG_WMIXPath(WMIX50, OFF); 		// VIN01 means LUT of VIN0 component
	else if(dev->vioc.wmixer.index == 6) 
		VIOC_CONFIG_WMIXPath(WMIX60, OFF);      // VIN11 means LUT of VIN1 component
	
	VIOC_WMIX_SetUpdate((VIOC_WMIX *)dev->vioc.wmixer.address);
	VIOC_VIN_SetSyncPolarity((VIOC_VIN *)dev->vioc.vin.address, dev->vioc.polarity_hsync, dev->vioc.polarity_vsync, \
		dev->vioc.field_bfield_low, dev->vioc.polarity_data_en, dev->vioc.gen_field_en, dev->vioc.polarity_pclk);		
	VIOC_VIN_SetCtrl((VIOC_VIN *)dev->vioc.vin.address, dev->vioc.conv_en, dev->vioc.hsde_connect_en, dev->vioc.vs_mask, \
		dev->vioc.input_fmt, dev->vioc.data_order);
	VIOC_VIN_SetInterlaceMode((VIOC_VIN *)dev->vioc.vin.address, dev->hdin_interlaced_mode, dev->vioc.intpl_en);
	VIOC_VIN_SetImageSize((VIOC_VIN *)dev->vioc.vin.address, width, height);
	VIOC_VIN_SetImageOffset((VIOC_VIN *)dev->vioc.vin.address, 0, 0, 0);
	VIOC_VIN_SetImageCropSize((VIOC_VIN *)dev->vioc.vin.address, width, height);
	VIOC_VIN_SetImageCropOffset((VIOC_VIN *)dev->vioc.vin.address, 0, 0);
	VIOC_VIN_SetY2RMode((VIOC_VIN *)dev->vioc.vin.address, 2);
	VIOC_VIN_SetY2REnable((VIOC_VIN *)dev->vioc.vin.address, OFF);
	VIOC_VIN_SetLUT((VIOC_VIN *)dev->vioc.vin.address, dev->vioc.lut.address);
	VIOC_VIN_SetLUTEnable((VIOC_VIN *)dev->vioc.vin.address, OFF, OFF, OFF);
	VIOC_VIN_SetEnable((VIOC_VIN *)dev->vioc.vin.address, ON);

	return 0;
}

int hdin_vioc_scaler_set(struct tcc_hdin_device *vdev)
{
	unsigned int dw, dh;			// destination size in SCALER

	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;


	dw = data->hdin_cfg.main_set.target_x;
	dh = data->hdin_cfg.main_set.target_y;

	if(dw == dev->input_width && dh == dev->input_height)
	{
		if(dev->sc_plugin_status)
		{
			VIOC_CONFIG_PlugOut(dev->vioc.sc_channel_num);
			dev->sc_plugin_status = OFF;
		}
		return 0;
	}

	VIOC_CONFIG_PlugIn(dev->vioc.sc_channel_num, dev->vioc.sc_plugin_pos);	// PlugIn position in SCALER
	VIOC_SC_SetBypass((VIOC_SC *)dev->vioc.scaler.address, OFF);

	if(dev->hdin_interlaced_mode){
		VIOC_SC_SetDstSize((VIOC_SC *)dev->vioc.scaler.address, dw, (dh+6));		// set destination size in scaler
		VIOC_SC_SetOutPosition((VIOC_SC *)dev->vioc.scaler.address, 0, 3);			// set output position in scaler
	}
	else
	{
		VIOC_SC_SetDstSize((VIOC_SC *)dev->vioc.scaler.address, dw + 2, dh + 2);		// set destination size in scaler
		VIOC_SC_SetOutPosition((VIOC_SC *)dev->vioc.scaler.address, 1, 1);	
	}

	VIOC_SC_SetOutSize((VIOC_SC *)dev->vioc.scaler.address, dw, dh);		// set output size in scaer
	VIOC_SC_SetUpdate((VIOC_SC *)dev->vioc.scaler.address);					// Scaler update

	dev->sc_plugin_status = ON;

	dprintk("VIOC SC%d(%dx%d)\n", dev->vioc.sc_channel_num, dw, dh);
	return 0;
}


int hdin_vioc_vin_wdma_set(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;
	
	uint dw, dh; 	// destination image size
	uint fmt; 		// image format

	dw = data->hdin_cfg.main_set.target_x;
	dh = data->hdin_cfg.main_set.target_y;

	fmt = VIOC_IMG_FMT_YUV420SEP;

	dprintk("%s():  WDMA size[%dx%d], format[%d]. \n", __FUNCTION__, dw, dh, fmt);
	
	VIOC_WDMA_SetImageFormat((VIOC_WDMA *)dev->vioc.wdma.address, fmt);
	VIOC_WDMA_SetImageSize((VIOC_WDMA *)dev->vioc.wdma.address, dw, dh);
	VIOC_WDMA_SetImageOffset((VIOC_WDMA *)dev->vioc.wdma.address, fmt, dw);
	VIOC_WDMA_SetImageRGBSwapMode((VIOC_WDMA *)dev->vioc.wdma.address, SWAP_RBG);
	VIOC_WDMA_SetImageEnable((VIOC_WDMA *)dev->vioc.wdma.address, ON);	// operating start in 1 frame
	vioc_intr_clear(data->vioc_intr->id, data->vioc_intr->bits);
	vioc_intr_enable(data->vioc_intr->id, data->vioc_intr->bits);

	return 0;
}

#ifdef USE_VIQE	
int hdin_vioc_viqe_main(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct device_node	*hdin_np = dev->np, * handler_np = NULL;
	
	unsigned int	viqe_width	= 0;
	unsigned int	viqe_height	= 0;
	unsigned int	format		= FMT_FC_YUV420;
	unsigned int	bypass_deintl	= VIOC_VIQE_DEINTL_MODE_3D;
	unsigned int	offset			= dev->input_width*dev->input_height* 2 * 2;
	unsigned int	deintl_base0	= dev->viqe_area;
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

	if(!(handler_np = of_parse_phandle(hdin_np, "hdin_viqe_set", 0))) 
	{
		printk("could not find camera_viqe_set node!! \n");

	}
	else
	{
		viqe_set_reg1 = (unsigned int*)of_iomap(handler_np, 0);
		viqe_set_reg2 = (unsigned int*)of_iomap(handler_np, 1);
	
		BITCSET(*viqe_set_reg1,1<<3,1<<3);
		BITCSET(*viqe_set_reg2,1<<8 | 1<<9 , 1<<8 | 1<<9);
	}
	
	VIOC_VIQE_SetControlRegister((VIQE *)dev->vioc.viqe.address, viqe_width, viqe_height, format);
	VIOC_VIQE_SetDeintlRegister((VIQE *)dev->vioc.viqe.address, format, OFF, viqe_width, viqe_height, bypass_deintl,	\
					deintl_base0, deintl_base1, deintl_base2, deintl_base3);
	VIOC_VIQE_SetControlEnable((VIQE *)dev->vioc.viqe.address, cdf_lut_en, his_en, gamut_en, d3d_en, deintl_en);
	
	return 0;
}
#endif

void hdin_set_port(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = 	vdev;	
	struct device_node	*hdin_np = dev->np;
	struct device_node *port_np;
	unsigned int nUsingPort,nCIF_8bit_connect;
	unsigned int *cifport_addr;

	port_np = of_parse_phandle(hdin_np,"hdin_port", 0);
	of_property_read_u32_index(hdin_np,"hdin_port",1,&nUsingPort);
	of_property_read_u32_index(hdin_np,"hdin_port",2,&nCIF_8bit_connect);
	
	cifport_addr = of_iomap(port_np, 0);

	BITCSET(*cifport_addr, 0xFFE77777, nUsingPort << (dev->vioc.vin.index*4) | nCIF_8bit_connect << 20);

}

int hdin_video_start_stream(struct tcc_hdin_device *vdev)
{

	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;

	dprintk("%s Start!! \n", __FUNCTION__);

	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_WDMA,dev->vioc.wdma.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_WMIXER,dev->vioc.wmixer.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_SCALER,dev->vioc.scaler.index,VIOC_CONFIG_RESET);	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_VIN,dev->vioc.vin.index,VIOC_CONFIG_RESET);
	
	if(dev->hdin_interlaced_mode)
	{
#ifdef USE_VIQE		
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_VIQE,dev->vioc.viqe.index,VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_VIQE,dev->vioc.viqe.index,VIOC_CONFIG_CLEAR);
#else
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_DEINTS,dev->vioc.viqe.index,VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_DEINTS,dev->vioc.viqe.index,VIOC_CONFIG_CLEAR);
#endif
	}
	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_VIN,dev->vioc.vin.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_SCALER,dev->vioc.scaler.index,VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_WMIXER,dev->vioc.wmixer.index,VIOC_CONFIG_CLEAR);	
	VIOC_CONFIG_SWReset((VIOC_IREQ_CONFIG *)dev->vioc.config.address,VIOC_CONFIG_WDMA,dev->vioc.wdma.index,VIOC_CONFIG_CLEAR);	


	data->stream_state = STREAM_ON;

	dev->skipped_frm 	= 0;
	dev->prev_buf 		= NULL;

	hdin_vioc_vin_main(dev);
	hdin_vioc_scaler_set(dev);

	if(dev->deint_plugin_status && !dev->hdin_interlaced_mode)
	{
#ifdef USE_VIQE			
		VIOC_CONFIG_PlugOut(VIOC_VIQE);
#else
		VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
#endif
	}
	
	if(dev->hdin_interlaced_mode)
	{
#ifdef USE_VIQE	
		VIOC_CONFIG_PlugIn(VIOC_VIQE, VIOC_VIQE_VIN_00);
		hdin_vioc_viqe_main(dev);
#else
		VIOC_CONFIG_PlugIn(VIOC_DEINTLS, VIOC_VIQE_VIN_00);
#endif
		dev->deint_plugin_status = ON;
	}

	hdin_vioc_vin_wdma_set(dev);

	dprintk("%s End!! \n", __FUNCTION__);
	return 0;
}

int hdin_video_stop_stream(struct tcc_hdin_device *vdev)
{	
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;

	dprintk("%s Start!! \n", __FUNCTION__);

	mutex_lock(&data->lock);

	vioc_intr_disable(data->vioc_intr->id, data->vioc_intr->bits);
	VIOC_WDMA_SetImageDisable((VIOC_WDMA *)dev->vioc.wdma.address);

	VIOC_VIN_SetEnable((VIOC_VIN *)dev->vioc.vin.address, OFF); // disable VIN
	
	if(dev->deint_plugin_status)
	{
#ifdef USE_VIQE			
		VIOC_CONFIG_PlugOut(VIOC_VIQE);
#else
		VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
#endif
	}

	if(dev->sc_plugin_status)
	{
		VIOC_CONFIG_PlugOut(dev->vioc.sc_channel_num);
		dev->sc_plugin_status = OFF;
	}
	mutex_unlock(&data->lock);

	data->stream_state = STREAM_OFF;	
	dprintk("\n\n SKIPPED FRAME = %d \n\n", dev->skipped_frm);

	dprintk("%s End!! \n", __FUNCTION__);

	return 0;
}


int hdin_video_buffer_set(struct tcc_hdin_device *vdev,struct v4l2_requestbuffers *req)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;
	struct tcc_hdin_buffer *buf = NULL;
	unsigned int y_offset = 0, uv_offset = 0, stride = 0;
	unsigned int buff_size = 0;

	if(req->count == 0) {
		data->hdin_cfg.now_frame_num = 0;

		stride = ALIGNED_BUFF(data->hdin_cfg.main_set.target_x, L_STRIDE_ALIGN);
		y_offset = stride * data->hdin_cfg.main_set.target_y;

		if(data->hdin_cfg.fmt == M420_ZERO) 	buff_size = PAGE_ALIGN(y_offset*2);
		else 								buff_size = PAGE_ALIGN(y_offset + y_offset/2);

		data->buf->v4lbuf.length = buff_size;

		memset(data->static_buf, 0x00, req->count * sizeof(struct tcc_hdin_buffer));

		data->done_list.prev = data->done_list.next = &data->done_list;
		data->list.prev 	 = data->list.next 		= &data->list;
		data->hdin_cfg.base_buf = (unsigned int)req->reserved[0];
	}

	buf = &(data->static_buf[req->count]);

	INIT_LIST_HEAD(&buf->buf_list);

	buf->mapcount 		= 0;
	buf->device 		= data;
	buf->v4lbuf.index 	= req->count;
	buf->v4lbuf.type 	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field 	= V4L2_FIELD_NONE;
	buf->v4lbuf.memory 	= V4L2_MEMORY_MMAP;
	buf->v4lbuf.length 	= buff_size;


	stride = ALIGNED_BUFF(data->hdin_cfg.main_set.target_x, L_STRIDE_ALIGN);
	y_offset = stride * data->hdin_cfg.main_set.target_y;
	uv_offset = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (data->hdin_cfg.main_set.target_y/2);

	data->hdin_cfg.preview_buf[req->count].p_Y  = (unsigned int)req->reserved[0];
	data->hdin_cfg.preview_buf[req->count].p_Cb = (unsigned int)data->hdin_cfg.preview_buf[req->count].p_Y + y_offset;
	data->hdin_cfg.preview_buf[req->count].p_Cr = (unsigned int)data->hdin_cfg.preview_buf[req->count].p_Cb + uv_offset;

	data->hdin_cfg.pp_num = req->count;

	return 0;
}

void hdin_video_set_addr(struct tcc_hdin_device *vdev,int index, unsigned int addr)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;
	unsigned int y_offset = 0, uv_offset = 0, stride = 0;

	stride = ALIGNED_BUFF(data->hdin_cfg.main_set.target_x, L_STRIDE_ALIGN);
	y_offset = stride * data->hdin_cfg.main_set.target_y;
	uv_offset = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (data->hdin_cfg.main_set.target_y/2);

	data->hdin_cfg.preview_buf[index].p_Y  = addr;
	data->hdin_cfg.preview_buf[index].p_Cb = (unsigned int)data->hdin_cfg.preview_buf[index].p_Y + y_offset;
	data->hdin_cfg.preview_buf[index].p_Cr = (unsigned int)data->hdin_cfg.preview_buf[index].p_Cb + uv_offset;
}

int hdin_video_set_resolution(struct tcc_hdin_device *vdev,unsigned int pixel_fmt, unsigned short width, unsigned short height)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;

	if(pixel_fmt == V4L2_PIX_FMT_YUYV)  data->hdin_cfg.fmt = M420_ZERO;  // yuv422
	else if(pixel_fmt == V4L2_PIX_FMT_RGB32) data->hdin_cfg.fmt = ARGB8888;  
	else if(pixel_fmt == V4L2_PIX_FMT_RGB565) data->hdin_cfg.fmt = RGB565;
	else 								data->hdin_cfg.fmt = M420_ODD; 	// yuv420

	data->hdin_cfg.main_set.target_x = width;
	data->hdin_cfg.main_set.target_y = height;

	return 0;
}


void hdin_dma_hw_reg(struct tcc_hdin_device *vdev,unsigned char frame_num)
{
	struct tcc_hdin_device *dev = 	vdev;
	struct TCC_HDIN *data = &dev->data;
	
	data->hdin_cfg.now_frame_num = frame_num;

	VIOC_WDMA_SetImageBase((VIOC_WDMA*)dev->vioc.wdma.address, (unsigned int)data->hdin_cfg.preview_buf[frame_num].p_Y, 
																(unsigned int)data->hdin_cfg.preview_buf[frame_num].p_Cr, 
																(unsigned int)data->hdin_cfg.preview_buf[frame_num].p_Cb);
}
static irqreturn_t hdin_video_isr(int irq, void *client_data)
{
	struct tcc_hdin_device *dev = (struct tcc_hdin_device*)client_data;	
	struct TCC_HDIN *data = &dev->data;
	struct tcc_hdin_buffer *next_buf;
	
	unsigned int stride;
	unsigned int next_num;

	if (is_vioc_intr_activatied(data->vioc_intr->id, data->vioc_intr->bits) == false)
			return IRQ_NONE;
	
	if(vioc_intr_get_status(data->vioc_intr->id)) 
	{
		if(dev->skip_frm == 0 && !dev->frame_lock) 
		{
			if(dev->prev_buf != NULL) 
				list_move_tail(&dev->prev_buf->buf_list, &data->done_list);

			dev->prev_buf = data->buf + data->hdin_cfg.now_frame_num;
			dev->prev_num = data->hdin_cfg.now_frame_num;

			next_buf = list_entry(data->list.next->next, struct tcc_hdin_buffer, buf_list);
			next_num = next_buf->v4lbuf.index;

			if(next_buf->buf_list.next != data->list.next && dev->prev_buf != next_buf) 
			{ 
				if(dev->prev_num != dev->prev_buf->v4lbuf.index) 
				{
					printk("Frame num mismatch :: true num	:: %d \n", dev->prev_num);
					printk("Frame num mismatch :: false num :: %d \n", dev->prev_buf->v4lbuf.index);
					dev->prev_buf->v4lbuf.index = dev->prev_num ;
				}

				hdin_dma_hw_reg(dev,next_num);

				stride = ALIGNED_BUFF(data->hdin_cfg.main_set.target_x, L_STRIDE_ALIGN);
				dev->prev_buf->v4lbuf.bytesused = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN) * (data->hdin_cfg.main_set.target_y / 2);
				dev->prev_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
				dev->prev_buf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;
			}
			else 
			{
				dev->prev_buf = NULL;
				dprintk("no-buf change... wakeup!! \n");
				dev->skipped_frm++;
			}
		
			data->wakeup_int = 1;
			wake_up_interruptible(&data->frame_wait);
		}
		else 
		{
			if(dev->skip_frm > 0) 
			{
				printk("skip_frame!!\n");
				dev->skip_frm--;
			}
			else 
			{
				dev->skip_frm = 0;
			}
		}
		VIOC_WDMA_SetImageUpdate((VIOC_WDMA*)dev->vioc.wdma.address); // update WDMA
		vioc_intr_clear(data->vioc_intr->id, data->vioc_intr->bits);
	}

	return IRQ_HANDLED;
}

void hdin_vioc_dt_parse_data(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	struct device_node	*hdin_np = dev->np;
	struct device_node *vioc_np;
	int lut_offset;

	if(hdin_np)
	{
		
		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_wmixer", 0))) 
		{
			printk("could not find hdin wmixer node!! \n");
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_wmixer", 1, &dev->vioc.wmixer.index);
			dev->vioc.wmixer.address = (unsigned int*)of_iomap(vioc_np, dev->vioc.wmixer.index);			
		}
		
		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_wdma", 0))) 
		{
			printk("could not find hdin wdma node!! \n");
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_wdma", 1, &dev->vioc.wdma.index);
			dev->vioc.wdma.address	= (unsigned int*)of_iomap(vioc_np, dev->vioc.wdma.index);
			switch(dev->vioc.wdma.index)
			{
				case 5:
				default:
					dev->vioc.sc_plugin_pos = VIOC_SC_WDMA_05;
					break;
				case 6:
					dev->vioc.sc_plugin_pos = VIOC_SC_WDMA_06;
					break;
			}
		}

		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_scaler", 0))) 
		{
			printk("could not find hdin scaler node!! \n");
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_scaler", 1, &dev->vioc.scaler.index);
			dev->vioc.scaler.address	= (unsigned int*)of_iomap(vioc_np, dev->vioc.scaler.index);
			switch(dev->vioc.scaler.index)
			{
				case 0:
				default:
					dev->vioc.sc_channel_num = VIOC_SC0;
					break;
				case 1:
					dev->vioc.sc_channel_num = VIOC_SC1;
					break;
				case 2:
					dev->vioc.sc_channel_num = VIOC_SC2;
					break;
				case 3:
					dev->vioc.sc_channel_num = VIOC_SC3;
					break;

			}			
		}

		
		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_videoin", 0))) 
		{
			printk("could not find hdin input node!! \n");
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_videoin", 2, &dev->vioc.vin.index);
			dev->vioc.vin.address = (unsigned int*)of_iomap(vioc_np, dev->vioc.vin.index);
			
			of_property_read_u32_index(hdin_np,"hdin_videoin", 1, &lut_offset);			
			dev->vioc.lut.address= (unsigned int*)of_iomap(vioc_np, dev->vioc.vin.index + lut_offset);			
		}
		
		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_config", 0))) 
		{
			printk("could not find hdin irqe node!! \n");			
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_config", 1, &dev->vioc.config.index);
			dev->vioc.config.address= (unsigned int*)of_iomap(vioc_np, dev->vioc.config.index);			
		}

		if(!(vioc_np = of_parse_phandle(hdin_np, "hdin_viqe", 0))) 
		{
			printk("could not find hdin viqe node!! \n");			
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdin_viqe", 1, &dev->vioc.viqe.index);
			dev->vioc.viqe.address = (unsigned int*)of_iomap(vioc_np, dev->vioc.viqe.index);			
		}

		
		if(!(vioc_np = of_parse_phandle(hdin_np, "hdmi_spdif_op", 0)))
		{
			printk("could not find hdmi_spdif_op node!! \n");
		}
		else
		{
			of_property_read_u32_index(hdin_np,"hdmi_spdif_op", 1, &dev->hdmi_spdif_op_value);
			dev->hdmi_spdif_op_addr = (unsigned int*)of_iomap(vioc_np, 0);			
		}			
	}
	else
	{
		printk("could not find hdin platform node!! \n");
	}
}

int hdin_video_irq_request(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	struct TCC_HDIN *data = &dev->data;
	struct device_node *hdin_np = dev->np;
	struct device_node *irq_np;
	int ret = -1, irq_num;
	
	irq_np = of_parse_phandle(hdin_np, "hdin_wdma", 0);
	
	if(irq_np) 
	{
		of_property_read_u32_index(hdin_np,"hdin_wdma", 1, &dev->vioc.wdma.index);
		irq_num = irq_of_parse_and_map(irq_np, dev->vioc.wdma.index);

		data->vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
		if (data->vioc_intr  == 0)
		{
			printk("could not get hdin_data.vioc_intr !! \n");
			return ret;
		}

		data->vioc_intr->id = VIOC_INTR_WD0 + dev->vioc.wdma.index;
		data->vioc_intr->bits = 1<<VIOC_WDMA_INTR_EOFF;
		
		ret = request_irq(irq_num, hdin_video_isr, IRQF_SHARED, DRIVER_NAME,dev);
		if (ret) printk("failed to aquire hdin(wdma %d) request_irq. \n",data->vioc_intr->id);
	}
		
	return ret;
	
}

void hdin_video_irq_free(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	struct TCC_HDIN *data = &dev->data;	
	struct device_node *hdin_np = dev->np;
	struct device_node *irq_np;
	int irq_num;
	
	irq_np = of_parse_phandle(hdin_np, "hdin_wdma", 0);
	
	if(irq_np) 
	{
		of_property_read_u32_index(hdin_np,"hdin_wdma", 1, &dev->vioc.wdma.index);
		irq_num = irq_of_parse_and_map(irq_np, dev->vioc.wdma.index);
		free_irq(irq_num, dev);
		kfree(data->vioc_intr);
	}
		
}

int hdin_video_open(struct tcc_hdin_device *vdev)
{

	struct tcc_hdin_device *dev = vdev;	
	struct TCC_HDIN *data = &dev->data;

	int ret = 0;

	data->done_list.next = &data->done_list;
	data->list.next = &data->list;

	if(!dev->hdin_irq) 
	{
		if((ret = hdin_video_irq_request(dev)) < 0)
		{
			printk("FAILED to aquire hdin irq(%d).\n", ret);
			return ret;
		}
		dev->hdin_irq = 1;
	}
	
	return 0;
}

int hdin_video_close(struct file *file)
{
	struct tcc_hdin_device *dev = video_drvdata(file);
	struct TCC_HDIN *data = &dev->data;


	mutex_destroy(&data->lock);

	hdin_ctrl_cleanup(file);

	if(dev->hdin_opend== ON)
		dev->hdin_opend = OFF;

	if(dev->hdin_irq) 
	{
		hdin_video_irq_free(dev);
	}
	
	dev->hdin_irq = 0;

	return 0;
}

int  hdin_video_init(struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	struct TCC_HDIN *data = &dev->data;
	pmap_t viqe_area;

	dprintk("hdin_video_init!! \n");

	if(dev->hdin_opend == OFF) 
	{
        memset(data,0x00,sizeof(struct TCC_HDIN));
        data->buf = data->static_buf;

		pmap_get_info("video", &viqe_area);
		dev->viqe_area = viqe_area.base;

		dev->hdin_opend = ON;
		dev->deint_plugin_status = OFF;
		dev->sc_plugin_status = OFF;
		dev->current_resolution = -1;
		dev->interrupt_status = 0;
		dev->frame_lock = 0;
		dev->skip_frm = 0;
		
		if(dev->np)
		{
			hdin_vioc_dt_parse_data(dev);
			hdin_set_port(dev);
		}
		else
		{
			printk("could not find hdin node!! \n");
			return -ENODEV;	
		}		
	  
		hdin_data_init((void*)dev);

		mdelay(5);

		init_waitqueue_head(&data->frame_wait);
		spin_lock_init(&data->dev_lock);

		INIT_LIST_HEAD(&data->list);
		INIT_LIST_HEAD(&data->done_list);	
		mutex_init(&data->lock);
	}
	return 0;
}
