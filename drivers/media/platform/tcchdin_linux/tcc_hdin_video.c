
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
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <linux/module.h>

#include <mach/irqs.h>

#include <mach/hardware.h>
#include <asm/io.h>

#include <mach/bsp.h>
#include <mach/tcc_hdin_ioctrl.h>
#include <mach/vioc_cam_plugin.h>

#include <mach/memory.h>
#include <asm/scatterlist.h>
#include <asm/mach-types.h>

#include <soc/tcc/pmap.h>

#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

#include "tcc_hdin_main.h"
#include "tcc_hdin_ctrl.h"
#include "tcc_hdin_video.h"
#include "tcc_hdin_i2c.h"

static int debug	   = 0;
#define dprintk(msg...) if(debug) { printk( "\e[33mhdin_video : \e[0m" msg); }

static unsigned char skip_frm = 0;
static unsigned char skipped_frm = 0;
static unsigned char frame_lock = 0;

struct VIOC_Index vioc_num;

static uint				bfield;
static uint				frm_cnt;

static int hdin_try_bpp(unsigned int pixelformat)
{
	int bpp;

	switch (pixelformat) {
	/* RGB formats */
	case V4L2_PIX_FMT_RGB332:
		bpp = 1;
		break;
	case V4L2_PIX_FMT_RGB444:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555X:
	case V4L2_PIX_FMT_RGB565X:
		bpp = 2;
		break;
	case V4L2_PIX_FMT_BGR666:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		bpp = 3;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		bpp = 4;
		break;

	/* YUV formats */
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_M420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12MT:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		bpp = 1;
		break;

	/* YUV422 Sequential formats */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		bpp = 2;
		break;

	default:
		printk(KERN_ERR DRIVER_NAME ": Not supported pixelformat(0x%x) \n", pixelformat);
		bpp = -EINVAL;
		break;
	}

	return bpp;
}

static void hdin_data_init(struct TCC_HDIN *h)
{
	dprintk("hdin_data_init....................\n");

	//Default Setting
	h->hdin_cfg.base_buf			= h->hdin_buf.addr;
	h->hdin_cfg.pp_num			= 0;

	h->hdin_cfg.fmt				= RGB565;

	h->hdin_cfg.main_set.target_x = 0;
	h->hdin_cfg.main_set.target_y = 0;
}

void hdin_dma_hw_reg(struct TCC_HDIN *h, VIOC_WDMA *pWDMA, unsigned char frame_num)
{
	dprintk("hdin_dma_hw_reg....................(%d)\n", frame_num);

	h->hdin_cfg.now_frame_num = frame_num;
	VIOC_WDMA_SetImageBase(pWDMA, (unsigned int)h->hdin_cfg.preview_buf[frame_num].p_Y,
									(unsigned int)h->hdin_cfg.preview_buf[frame_num].p_Cb,
									(unsigned int)h->hdin_cfg.preview_buf[frame_num].p_Cr);
}

static int hdin_vioc_deintls_set(struct tcc_hdin_device *hdev)
{
	struct tcc_vioc *vioc = &hdev->vioc;
	volatile unsigned long *vioc_deintls = (volatile unsigned long *)tcc_p2v(HwVIOC_DEINTLS);
	int ret = 0;

	dprintk("hdin_vioc_deintls_set....................\n");

	/* reset deintl_s */
	BITSET(vioc->config_addr->uSOFTRESET.nREG[1], 0x1 << 17);
	BITCLR(vioc->config_addr->uSOFTRESET.nREG[1], 0x1 << 17);

	/*soc guide*/
	BITCSET(*vioc_deintls, 0x7, 0x3);

	ret = VIOC_CONFIG_PlugIn(VIOC_DEINTLS, vioc->deintls_plugin);

	return ret == 0 ? 0 : -EFAULT;
}

static int hdin_vioc_viqe_set(struct tcc_hdin_device *hdev)
{
	struct tcc_vioc *vioc = &hdev->vioc;
	unsigned int viqe_deintl_base[4] = {0,};
	int img_size, viqe_w, viqe_h;

	/* viqe config variable */
	int vmisc_tsdu = OFF; 		// 0: viqe size is get from vioc module
	int di_dec_misc_fmt = FMT_FC_YUV422;	// 0: YUV420, 1: YUV422

	dprintk("hdin_vioc_viqe_set....................\n");

	viqe_w = ((PRV_W) >> 3) << 3;	// 8bit align
	viqe_h = ((PRV_H) >> 2) << 2;	// 4bit align

	img_size = (viqe_w * viqe_h * 2);
	pmap_get_info("viqe", &hdev->core.pmap_viqe);
	dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n",
			hdev->core.pmap_viqe.name, hdev->core.pmap_viqe.base, hdev->core.pmap_viqe.base + hdev->core.pmap_viqe.size, hdev->core.pmap_viqe.size);
	viqe_deintl_base[0] = hdev->core.pmap_viqe.base;
	viqe_deintl_base[1] = viqe_deintl_base[0] + img_size;
	viqe_deintl_base[2] = viqe_deintl_base[1] + img_size;
	viqe_deintl_base[3] = viqe_deintl_base[2] + img_size;

	if ((viqe_deintl_base[3] + img_size) > (hdev->core.pmap_viqe.base + hdev->core.pmap_viqe.size)) {
		printk(KERN_ERR DRIVER_NAME": [%s] pamp_viqe no space\n", __func__);
		return -ENOBUFS;
	}

	if (vmisc_tsdu == OFF) {
		viqe_w = 0;
		viqe_h = 0;
	}

	VIOC_VIQE_SetControlRegister(vioc->viqe_addr, viqe_w, viqe_h, di_dec_misc_fmt);
	VIOC_VIQE_SetDeintlRegister(vioc->viqe_addr, di_dec_misc_fmt, vmisc_tsdu, viqe_w, viqe_h,
		VIOC_VIQE_DEINTL_MODE_2D, viqe_deintl_base[0], viqe_deintl_base[1], viqe_deintl_base[2], viqe_deintl_base[3]);
	VIOC_VIQE_SetControlEnable(vioc->viqe_addr,
								OFF,	/* histogram CDF or LUT */
								OFF,	/* histogram */
								OFF,	/* gamut mapper */
								OFF,	/* de-noiser */
								ON		/* de-interlacer */
								);
	VIOC_VIQE_SetImageY2REnable(vioc->viqe_addr, TRUE);
	VIOC_VIQE_SetImageY2RMode(vioc->viqe_addr, 0x02);
	VIOC_CONFIG_PlugIn(VIOC_VIQE, vioc->viqe_plugin);

	return 0;
}

int hdin_vioc_vin_set(struct tcc_hdin_device *hdev, VIOC_VIN *pVIN, VIOC_WMIX *pWMIX)
{
	unsigned int vin_w, vin_h;
	unsigned int offs_w, offs_h, offs_h_intl;

	dprintk("hdin_vioc_vin_set....................\n");

	vin_w = ((PRV_W) >> 3) << 3;	// 8bit align
	vin_h = ((PRV_H) >> 2) << 2;	// 4bit align

	/* crop setting */
	offs_w		= 0;
	offs_h		= 0;
	offs_h_intl	= 0;

	#ifdef HDMIIN_FEATURE_YUV422_MODE
	if(hdev->hdmi_in_Interlaced) {
		VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_VIQE,vioc_num.viqe.index,VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_VIQE,vioc_num.viqe.index,VIOC_CONFIG_CLEAR);
		hdin_vioc_viqe_set(hdev);
	} else {
		/*	check VIQE plug in/out
		 */
		VIOC_PlugInOutCheck plug_state;
		VIOC_CONFIG_Device_PlugState(VIOC_VIQE, &plug_state);
		if((plug_state.connect_statue == VIOC_PATH_CONNECTED)
			&&(plug_state.connect_device == VIOC_VIQE_VIN_00))
			VIOC_CONFIG_PlugOut(VIOC_VIQE);
	}
	#else
	if(hdev->hdmi_in_Interlaced) {
		hdin_vioc_deintls_set(hdev);
	} else {
		/*	check DEINTL_S plug in/out
		 */
		VIOC_PlugInOutCheck plug_state;
		VIOC_CONFIG_Device_PlugState(VIOC_DEINTLS, &plug_state);
		if((plug_state.connect_statue == VIOC_PATH_CONNECTED)
			&&(plug_state.connect_device == VIOC_DEINTLS_VIN_00))
			VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
	}
	#endif

	VIOC_CONFIG_WMIXPath(WMIX50, OFF);
	VIOC_WMIX_SetUpdate(pWMIX);

	/* VIN config */
	VIOC_VIN_SetSyncPolarity(pVIN,
		ON/*Hsync*/,
		ON/*Vsync*/,
		OFF,
		OFF/*DE*/,
		ON/*gen_field_en*/,
		OFF/*pclk*/);

	VIOC_VIN_SetY2RMode(pVIN, 0x2);

	#ifdef HDMIIN_FEATURE_YUV422_MODE
	VIOC_VIN_SetCtrl(pVIN,
		OFF,
		OFF,
		OFF,
		FMT_YUV422_16BIT/*hdmi_in_vin_fmt*/,
		ORDER_BGR);
	if(hdev->hdmi_in_Interlaced)
		VIOC_VIN_SetY2REnable(pVIN, OFF);
	else
		VIOC_VIN_SetY2REnable(pVIN, ON);
	#else
	VIOC_VIN_SetCtrl(pVIN,
		OFF,
		OFF,
		OFF,
		FMT_RGB444_24BIT/*hdmi_in_vin_fmt*/,
		ORDER_RGB);
	VIOC_VIN_SetY2REnable(pVIN, OFF);
	#endif
	VIOC_VIN_SetInterlaceMode(pVIN, hdev->hdmi_in_Interlaced, OFF);

	VIOC_VIN_SetImageSize(pVIN, vin_w, vin_h);
	VIOC_VIN_SetImageOffset(pVIN, offs_w, offs_h, offs_h_intl);

	VIOC_VIN_SetImageCropSize(pVIN, vin_w, vin_h-2);
	VIOC_VIN_SetImageCropOffset(pVIN, 0, 0);

	VIOC_VIN_SetLUT(pVIN, hdev->vioc.vin_lut_addr);
	VIOC_VIN_SetLUTEnable(pVIN, OFF, OFF, OFF);

	VIOC_VIN_SetEnable(pVIN, ON);

	dprintk("VIOC VIN0(%dx%d) - WMIX5(%dx%d)\n", vin_w, vin_h, PRV_W, PRV_H);

	return 0;
}

int hdin_vioc_scaler_set(struct tcc_hdin_device *hdev, VIOC_SC *pSC, unsigned int sc_plugin)
{
	unsigned int dw, dh;			// destination size in SCALER
	unsigned int sc_channel= VIOC_SC0;
	struct TCC_HDIN *h = hdev->h;

	dprintk("hdin_vioc_scaler_set....................\n");

	dw = h->hdin_cfg.main_set.target_x;
	dh = h->hdin_cfg.main_set.target_y;

	if(dw == PRV_W && dh == PRV_H)
	{
		dprintk("not used scaler\n");
		VIOC_CONFIG_PlugOut(sc_channel);
		return 0;
	}

	VIOC_CONFIG_PlugIn(sc_channel, sc_plugin);	// PlugIn position in SCALER
	VIOC_SC_SetBypass(pSC, OFF);

	if(hdev->hdmi_in_Interlaced){
		VIOC_SC_SetDstSize(pSC, dw, (dh+6));		// set destination size in scaler
		VIOC_SC_SetOutPosition(pSC, 0, 6);			// set output position in scaler
	}
	else
	{
		VIOC_SC_SetDstSize(pSC, dw, dh);		// set destination size in scaler
	}

	VIOC_SC_SetOutSize(pSC, dw, dh);		// set output size in scaer
	VIOC_SC_SetUpdate(pSC);					// Scaler update

	dprintk("VIOC SC%d(%dx%d)\n", sc_channel, dw, dh);
	return 0;
}

int hdin_vioc_wdma_set(struct tcc_hdin_device *hdev, VIOC_WDMA *pWDMA, int restart)
{
	unsigned int fmt = 0, dw, dh;
	struct TCC_HDIN *h = hdev->h;

	dprintk("hdin_vioc_wdma_set....................(%d)\n", h->hdin_cfg.fmt);

	switch(h->hdin_cfg.fmt)
	{
	case RGB565:
		fmt = VIOC_IMG_FMT_RGB565;
		break;
	case ARGB8888:
		fmt = VIOC_IMG_FMT_ARGB8888;
		break;
	case YUV420SEP:
		fmt = VIOC_IMG_FMT_YUV420SEP;
		break;
	}

	dw = h->hdin_cfg.main_set.target_x;
	dh = h->hdin_cfg.main_set.target_y;

	if(!restart) {
		hdin_dma_hw_reg(h, pWDMA, h->hdin_cfg.now_frame_num);
		dprintk("VIOC WDMA addr Y[0x%x] U[0x%x] V[0x%x], size[%dx%d]\n"
				,h->hdin_cfg.preview_buf[h->hdin_cfg.now_frame_num].p_Y
				,h->hdin_cfg.preview_buf[h->hdin_cfg.now_frame_num].p_Cb
				,h->hdin_cfg.preview_buf[h->hdin_cfg.now_frame_num].p_Cr
				,dw, dh);
	}

	VIOC_WDMA_SetImageFormat(pWDMA, fmt);
	VIOC_WDMA_SetImageSize(pWDMA, dw, dh);
	VIOC_WDMA_SetImageOffset(pWDMA, fmt, dw);

	if(fmt > VIOC_IMG_FMT_ARGB8888) { /* YUV format */
		VIOC_WDMA_SetImageR2YMode(pWDMA, R2YMD_SDTV_FR);
		VIOC_WDMA_SetImageR2YEnable(pWDMA, ON);
	}
	VIOC_WDMA_SetImageEnable(pWDMA, ON);
	VIOC_WDMA_SetIreqMask(pWDMA, hdev->vioc.wdma_irq_mask, 0x0);
	return 0;
}
#ifdef HDIN_DRV_BYPASS_EN
int hdin_vioc_rdma_set(struct tcc_hdin_device *hdev, VIOC_RDMA *pRDMA)
{
	unsigned int fmt = 0, dw, dh;
	struct TCC_HDIN *h = hdev->h;

	dprintk("hdin_vioc_rdma_set....................(%d)\n", h->hdin_cfg.fmt);

	switch(h->hdin_cfg.fmt)
	{
	case RGB565:
		fmt = VIOC_IMG_FMT_RGB565;
		break;
	case ARGB8888:
		fmt = VIOC_IMG_FMT_ARGB8888;
		break;
	case YUV420SEP:
		fmt = VIOC_IMG_FMT_YUV420SEP;
		break;
	}

	dw = h->hdin_cfg.main_set.target_x;
	dh = h->hdin_cfg.main_set.target_y;

	VIOC_RDMA_SetImageFormat(pRDMA, fmt);
	VIOC_RDMA_SetImageSize(pRDMA, dw, dh);
	VIOC_RDMA_SetImageOffset(pRDMA, fmt, dw);

	if(fmt > VIOC_IMG_FMT_ARGB8888) { /* YUV format */
		VIOC_RDMA_SetImageY2RMode(pRDMA, R2YMD_SDTV_FR);
		VIOC_RDMA_SetImageY2REnable(pRDMA, ON);
	}
	VIOC_RDMA_SetImageDisableNW(pRDMA);
	return 0;
}
#endif
void hdin_set_port(struct tcc_hdin_device *hdev)
{
	struct device_node *port_np;
	unsigned int nUsingPort;
	unsigned int *cifport_addr;

	port_np = of_parse_phandle(hdev->hdin_np, "hdin_port", 0);
	of_property_read_u32_index(hdev->hdin_np,"hdin_port",1,&nUsingPort);
	cifport_addr = of_iomap(port_np, 0);

	switch(nUsingPort)
	{
		case 0: //used port0
			BITCSET(*cifport_addr, 0x00077777, (4 << 16) | (3 << 12) | (2 << 8) | (1 << 4) | nUsingPort);
			break;

		case 1: //used port1
			BITCSET(*cifport_addr, 0x00077777, (0 << 16) | (4 << 12) | (3 << 8) | (2 << 4) | nUsingPort);
			break;

		case 2: //used port2
			BITCSET(*cifport_addr, 0x00077777, (0 << 16) | (4 << 12) | (3 << 8) | (1 << 4) | nUsingPort);
			break;

		case 3: //used port3
			BITCSET(*cifport_addr, 0x00077777, (0 << 16) | (4 << 12) | (2 << 8) | (1 << 4) | nUsingPort);
			break;

		case 4: //used port4
		default:
			BITCSET(*cifport_addr, 0x00077777, (0 << 16) | (3 << 12) | (2 << 8) | (1 << 4) | nUsingPort);
			break;
	}
}

int hdin_video_start_stream(struct tcc_hdin_device *hdev)
{

	struct TCC_HDIN *h = hdev->h;
	struct tcc_vioc *vioc = &hdev->vioc;

	dprintk("%s Start!! \n", __FUNCTION__);

	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_WDMA,vioc_num.wdma.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_WMIXER,vioc_num.wmixer.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_SCALER,vioc_num.scaler.index,VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_VIN,vioc_num.vin.index,VIOC_CONFIG_RESET);

	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_VIN,vioc_num.vin.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_SCALER,vioc_num.scaler.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_WMIXER,vioc_num.wmixer.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_WDMA,vioc_num.wdma.index,VIOC_CONFIG_CLEAR);

	h->stream_state = STREAM_ON;
	vioc->sc_plugin = VIOC_SC_WDMA_05;

	#ifdef HDMIIN_FEATURE_YUV422_MODE
	vioc->viqe_plugin = VIOC_VIQE_VIN_00;
	#else
	vioc->deintls_plugin = VIOC_DEINTLS_VIN_00;
	#endif

	/* init previous buffer info */
	hdev->core.prev_buf = h->buf + h->hdin_cfg.now_frame_num;
	hdev->core.prev_num = h->hdin_cfg.now_frame_num;

	skipped_frm 	= 0;
	frm_cnt 	= 0;
	bfield 	= 0;

	hdin_vioc_vin_set(hdev, vioc->vin_addr, vioc->wmix_addr);
	hdin_vioc_scaler_set(hdev, vioc->sc_addr, vioc->sc_plugin);
	hdin_vioc_wdma_set(hdev, vioc->wdma_addr, OFF);
#ifdef HDIN_DRV_BYPASS_EN
	if(hdev->bypass) {
		VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_RDMA,vioc_num.rdma.index,VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_RDMA,vioc_num.rdma.index,VIOC_CONFIG_CLEAR);

		hdin_vioc_rdma_set(hdev, vioc->rdma_addr);
	} else {
		unsigned int enable = 0;
		VIOC_RDMA_GetImageEnable(vioc->rdma_addr, &enable);
		if(enable) {
			VIOC_RDMA_SetImageDisable(vioc->rdma_addr);

			VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_RDMA,vioc_num.rdma.index,VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(hdev->vioc.config_addr,VIOC_CONFIG_RDMA,vioc_num.rdma.index,VIOC_CONFIG_CLEAR);
		}
	}
#endif
	dprintk("%s End!! \n", __FUNCTION__);
	return 0;
}

int hdin_video_stop_stream(struct tcc_hdin_device *hdev)
{
	struct TCC_HDIN *h = hdev->h;
	struct tcc_vioc *vioc = &hdev->vioc;

	dprintk("%s Start!! \n", __FUNCTION__);

	mutex_lock(&h->lock);

	VIOC_WDMA_SetIreqMask(vioc->wdma_addr, VIOC_WDMA_IREQ_ALL_MASK, 0x1); // Disable WDMA interrupt
	VIOC_WDMA_SetImageDisable(vioc->wdma_addr); // Disable WDMA

	msleep(50);

	VIOC_VIN_SetEnable(vioc->vin_addr, OFF);	// Disable VIN

	msleep(50);

	if(hdev->hdmi_in_Interlaced) {
		#ifdef HDMIIN_FEATURE_YUV422_MODE
		VIOC_CONFIG_PlugOut(VIOC_VIQE);
		#else
		VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
		#endif
	}
#ifdef HDIN_DRV_BYPASS_EN
	if(hdev->bypass)
		VIOC_RDMA_SetImageDisableNW(vioc->rdma_addr);
#endif
	mutex_unlock(&h->lock);

	h->stream_state = STREAM_OFF;
	dprintk("\n\n SKIPPED FRAME = %d \n\n", skipped_frm);

	dprintk("%s End!! \n", __FUNCTION__);

	return 0;
}

void hdin_vioc_dt_parse_data(struct tcc_hdin_device *hdev)
{
	struct device_node *wmixer_np, *wdma_np, *vin_np, *scaler_np, *config_np, *viqe_np, *rdma_np;
	int lut_offset;

	if(hdev->hdin_np)
	{
		wmixer_np = of_parse_phandle(hdev->hdin_np, "hdin_wmixer", 0);
		if(wmixer_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_wmixer", 1, &vioc_num.wmixer.index);
			hdev->vioc.wmix_addr = (VIOC_WMIX*)of_iomap(wmixer_np, vioc_num.wmixer.index);
		}
		else
		{
			printk("could not find hdin wmixer node!! \n");
		}
		
		wdma_np = of_parse_phandle(hdev->hdin_np, "hdin_wdma", 0);
		if(wdma_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_wdma", 1, &vioc_num.wdma.index);
			hdev->vioc.wdma_addr= (VIOC_WDMA*)of_iomap(wdma_np, vioc_num.wdma.index);
		}
		else
		{
			printk("could not find hdin wdma node!! \n");
		}
		
		vin_np = of_parse_phandle(hdev->hdin_np, "hdin_videoin", 0);
		if(vin_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_videoin", 2, &vioc_num.vin.index);
			hdev->vioc.vin_addr= (VIOC_VIN*)of_iomap(vin_np, vioc_num.vin.index);

			of_property_read_u32_index(hdev->hdin_np,"hdin_videoin", 1, &lut_offset);
			hdev->vioc.vin_lut_addr = (unsigned int*)of_iomap(vin_np, vioc_num.vin.index + lut_offset);
		}
		else
		{
			printk("could not find hdin input node!! \n");
		}

		scaler_np = of_parse_phandle(hdev->hdin_np, "hdin_scaler", 0);
		if(scaler_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_scaler", 1, &vioc_num.scaler.index);
			hdev->vioc.sc_addr= (VIOC_SC*)of_iomap(scaler_np, vioc_num.scaler.index);
		}
		else
		{
			printk("could not find hdin scaler node!! \n");
		}

		config_np = of_parse_phandle(hdev->hdin_np, "hdin_config", 0);
		if(config_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_config", 1, &vioc_num.config.index);
			hdev->vioc.config_addr= (VIOC_IREQ_CONFIG*)of_iomap(config_np, vioc_num.config.index);
		}
		else
		{
			printk("could not find hdin irqe node!! \n");
		}

		viqe_np = of_parse_phandle(hdev->hdin_np, "hdin_viqe", 0);
		if(viqe_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_viqe", 1, &vioc_num.viqe.index);
			hdev->vioc.viqe_addr= (VIQE*)of_iomap(viqe_np, vioc_num.viqe.index);
		}
		else
		{
			printk("could not find hdin viqe node!! \n");
		}

		rdma_np = of_parse_phandle(hdev->hdin_np, "hdin_rdma", 0);
		if(rdma_np)
		{
			of_property_read_u32_index(hdev->hdin_np,"hdin_rdma", 1, &vioc_num.rdma.index);
			hdev->vioc.rdma_addr = (VIOC_RDMA*)of_iomap(rdma_np, vioc_num.rdma.index);
		}
		else
		{
			printk("could not find hdin rdma node!! \n");
		}
	}
	else
	{
		printk("could not find hdin platform node!! \n");
	}
}


int hdin_video_buffer_set(struct tcc_hdin_device *hdev, struct v4l2_requestbuffers *req)
{
	struct TCC_HDIN *h = hdev->h;
	dma_addr_t base_addr = h->hdin_cfg.base_buf;
	unsigned int y_offset = 0, uv_offset = 0, total_offset = 0;
	unsigned char i;
	unsigned long total_req_buf_size = 0;

	dprintk("hdin_video_buffer_set....................\n");

	printk("req->count: %d\n", req->count);

	h->hdin_cfg.now_frame_num = 0;

	dprintk("pix.width x pix.height: %d x %d\n", h->hdin_cfg.main_set.target_x, h->hdin_cfg.main_set.target_y);

	if(h->hdin_cfg.fmt == RGB565)
	{
		total_offset = (h->hdin_cfg.main_set.target_x * 2/* bpp */ * h->hdin_cfg.main_set.target_y);
	}
	else if(h->hdin_cfg.fmt == ARGB8888)
	{
		total_offset = (h->hdin_cfg.main_set.target_x * 4/* bpp */ * h->hdin_cfg.main_set.target_y);
	}
	else //yuv420
	{
		y_offset = h->hdin_cfg.main_set.target_x * h->hdin_cfg.main_set.target_y;
		uv_offset = y_offset / 4;
		total_offset = y_offset + uv_offset * 2;
	}

	total_offset = PAGE_ALIGN(total_offset);
	h->buf->v4lbuf.length = total_offset;

retry:
	if(req->count == 1)
	{
#if 0  // ZzaU :: Don't check Buffer in Rolling-Capture.
		if (h->buf->v4lbuf.length > CAPTURE_MEM)
		{
			printk("reqbufs: count invalid\n");
			return -1;
		}
#endif
	}
	else
	{
		if (h->buf->v4lbuf.length * req->count > hdev->core.pmap_preview.size)
		{
			req->count--;
			if (req->count <= 0)
			{
				printk("reqbufs: count invalid\n");
				return -1;
			}
			goto retry;
		}
	}

	memset(h->static_buf, 0x0, req->count * sizeof(struct tcc_hdin_buffer));

	h->done_list.prev = h->done_list.next = &h->done_list;
	h->list.prev = h->list.next = &h->list;

	for(h->n_sbufs = 0; h->n_sbufs < req->count; h->n_sbufs++)
	{
		struct tcc_hdin_buffer *buf = &(h->static_buf[h->n_sbufs]);

		INIT_LIST_HEAD(&buf->buf_list);

		buf->v4lbuf.length = total_offset;
		total_req_buf_size += buf->v4lbuf.length;
		buf->mapcount = 0;
		buf->device = h;
		buf->v4lbuf.index = h->n_sbufs;
		buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf->v4lbuf.field = V4L2_FIELD_NONE;
		buf->v4lbuf.memory = V4L2_MEMORY_MMAP;
	}
	dprintk("-----------------------------------------------------\n");
	dprintk("v4l2_buffer: 0x%x * %d = 0x%x\n", total_offset, h->n_sbufs, (unsigned int)total_req_buf_size);

	h->hdin_cfg.pp_num = h->n_sbufs;
	req->count = h->hdin_cfg.pp_num;

	dprintk("y_off(0x%x) + uv_off(0x%x)*2 = ALIGN(0x%x)\n", y_offset, uv_offset, total_offset);

	/* set base address each buffers
	 */
	dprintk("preview_buf    Y_base     Cb_base    Cr_base\n");
	for (i = 0; i < h->hdin_cfg.pp_num; i++) {
		switch (h->hdin_cfg.fmt) {
		/* YUV format */
		case YUV420SEP:
			h->hdin_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN(base_addr + total_offset * i);
			h->hdin_cfg.preview_buf[i].p_Cb = (unsigned int)h->hdin_cfg.preview_buf[i].p_Y + y_offset;
			h->hdin_cfg.preview_buf[i].p_Cr = (unsigned int)h->hdin_cfg.preview_buf[i].p_Cb + uv_offset;
			break;
		/* RGB format */
		case RGB565:
		case ARGB8888:
			h->hdin_cfg.preview_buf[i].p_Y = (unsigned int)PAGE_ALIGN(base_addr + total_offset * i);
			h->hdin_cfg.preview_buf[i].p_Cb = 0;
			h->hdin_cfg.preview_buf[i].p_Cr = 0;
			break;
		default:
			printk(KERN_ERR "%s: Not supported VIOC format(%d)\n", __func__, h->hdin_cfg.fmt);
			return -EINVAL;
			break;
		}
		dprintk("     [%d]      0x%08x 0x%08x 0x%08x\n", i,h->hdin_cfg.preview_buf[i].p_Y, h->hdin_cfg.preview_buf[i].p_Cb, h->hdin_cfg.preview_buf[i].p_Cr);
	}
	dprintk("-----------------------------------------------------\n");

	return 0;
}

void hdin_video_set_addr(struct tcc_hdin_device *hdev, int index, unsigned int addr)
{
	struct TCC_HDIN *h = hdev->h;
	unsigned int y_offset = 0, uv_offset = 0, stride = 0;

	dprintk("hdin_video_set_addr....................\n");

	if(h->hdin_cfg.fmt == RGB565 || h->hdin_cfg.fmt == ARGB8888)
	{
		stride = ALIGNED_BUFF(h->hdin_cfg.main_set.target_x, 16);
		y_offset = stride * h->hdin_cfg.main_set.target_y;
		uv_offset = stride * h->hdin_cfg.main_set.target_y;

		h->hdin_cfg.preview_buf[index].p_Y  = addr;
		h->hdin_cfg.preview_buf[index].p_Cb = (unsigned int)NULL;
		h->hdin_cfg.preview_buf[index].p_Cr = (unsigned int)NULL;
	}
	else
	{
		stride = ALIGNED_BUFF(h->hdin_cfg.main_set.target_x, 16);
		y_offset = stride * h->hdin_cfg.main_set.target_y;
		uv_offset = ALIGNED_BUFF((stride/2), 16) * (h->hdin_cfg.main_set.target_y/2);

		h->hdin_cfg.preview_buf[index].p_Y  = addr;
		h->hdin_cfg.preview_buf[index].p_Cb = (unsigned int)h->hdin_cfg.preview_buf[index].p_Y + y_offset;
		h->hdin_cfg.preview_buf[index].p_Cr = (unsigned int)h->hdin_cfg.preview_buf[index].p_Cb + uv_offset;
	}
}

int hdin_video_set_resolution(struct tcc_hdin_device *hdev, unsigned int pixel_fmt, unsigned short width, unsigned short height)
{
	struct TCC_HDIN *h = hdev->h;
	dprintk("hdin_video_set_resolution....................(%d)\n", pixel_fmt);

	h->hdin_cfg.bpp = hdin_try_bpp(pixel_fmt);

	if(pixel_fmt == V4L2_PIX_FMT_RGB565)	h->hdin_cfg.fmt = RGB565;
	else if(pixel_fmt == V4L2_PIX_FMT_RGB32)	h->hdin_cfg.fmt = ARGB8888;
	else									h->hdin_cfg.fmt = YUV420SEP; 	// yuv420

	h->hdin_cfg.main_set.target_x = width;
	h->hdin_cfg.main_set.target_y = height;

	return 0;
}

static irqreturn_t hdin_video_isr(int irq, void *dev)
{
	struct tcc_hdin_device *hdev = (struct tcc_hdin_device *)dev;
	struct TCC_HDIN *h = hdev->h;
	struct tcc_vioc *vioc = &hdev->vioc;

	if(vioc->wdma_addr->uIRQSTS.nREG & vioc->wdma_irq_mask)
	{
		if (h->stream_state == STREAM_ON)
		{
			if(skip_frm == 0 && !frame_lock)
			{
				if(hdev->core.prev_buf != NULL) {
					list_move_tail(&hdev->core.prev_buf->buf_list, &h->done_list);
					#ifdef HDIN_DRV_BYPASS_EN
					if(hdev->bypass) {
						VIOC_RDMA_SetImageBase(vioc->rdma_addr, (unsigned int)h->hdin_cfg.preview_buf[hdev->core.prev_num].p_Y,
							(unsigned int)h->hdin_cfg.preview_buf[hdev->core.prev_num].p_Cb,
							(unsigned int)h->hdin_cfg.preview_buf[hdev->core.prev_num].p_Cr);

						if(vioc->rdma_addr->uCTRL.nREG & Hw28)
							VIOC_RDMA_SetImageUpdate(vioc->rdma_addr);
						else
							VIOC_RDMA_SetImageEnable(vioc->rdma_addr);
					} else {
						if(vioc->rdma_addr->uCTRL.nREG & Hw28)
							VIOC_RDMA_SetImageDisableNW(vioc->rdma_addr);
					}
					#endif
				}

				hdev->core.next_buf = list_entry(h->list.next, struct tcc_hdin_buffer, buf_list);
				hdev->core.next_num = hdev->core.next_buf->v4lbuf.index;

				dprintk("hdev->core.prev_num %d hdev->core.next_num %d\n", hdev->core.prev_num, hdev->core.next_num);

				if((&hdev->core.next_buf->buf_list != &h->list ) && (&hdev->core.prev_buf->buf_list != &hdev->core.next_buf->buf_list))
				{
					if(hdev->core.prev_num != hdev->core.prev_buf->v4lbuf.index)
					{
						printk("Frame num mismatch :: true num	:: %d \n", hdev->core.prev_num);
						printk("Frame num mismatch :: false num :: %d \n", hdev->core.prev_buf->v4lbuf.index);
						hdev->core.prev_buf->v4lbuf.index = hdev->core.prev_num ;
					}

					if(h->hdin_cfg.fmt == RGB565)
						hdev->core.prev_buf->v4lbuf.bytesused = h->hdin_cfg.main_set.target_x * h->hdin_cfg.main_set.target_y * 2/* 16bpp */;
					else if(h->hdin_cfg.fmt == ARGB8888)
						hdev->core.prev_buf->v4lbuf.bytesused = h->hdin_cfg.main_set.target_x * h->hdin_cfg.main_set.target_y * 4/* 32bpp */;
					else //YUV420
						hdev->core.prev_buf->v4lbuf.bytesused = (h->hdin_cfg.main_set.target_x * h->hdin_cfg.main_set.target_y * 3) / 2;

					hdin_dma_hw_reg(h, vioc->wdma_addr, hdev->core.next_num);

					/* Update previous buffer info*/
					hdev->core.prev_buf = hdev->core.next_buf;
					hdev->core.prev_num= hdev->core.next_num;

					if((frm_cnt == 1) && (bfield == 0))
					{
						#ifdef HDMIIN_FEATURE_YUV422_MODE
						if(hdev->hdmi_in_Interlaced)
							VIOC_VIQE_SetDeintlMode(vioc->viqe_addr, VIOC_VIQE_DEINTL_MODE_3D);
						#endif
					}

					if(bfield == 1)
					{
						bfield = 0;
						frm_cnt++;
					}
					else
					{
						bfield = 1;
					}

					if(skip_frm == 0)	skip_frm++;
					else				skip_frm = 1;
				}
				else 
				{
					dprintk("no-buf\n");
					hdev->core.prev_buf = NULL;
					skipped_frm++;
				}

				h->wakeup_int = 1;
				wake_up_interruptible(&h->frame_wait);
			}
			else
			{
				if(skip_frm > 0)
				{
					skip_frm--;
				}
				else
				{
					skip_frm = 0;
				}
			}
			BITCSET(vioc->wdma_addr->uCTRL.nREG, Hw16, Hw16);											/* update WDMA */
			BITCSET(vioc->wdma_addr->uIRQSTS.nREG, VIOC_WDMA_IREQ_ALL_MASK, VIOC_WDMA_IREQ_ALL_MASK);	/* clear irq status */
			VIOC_WDMA_SetIreqMask(vioc->wdma_addr, vioc->wdma_irq_mask, 0x0);							/* enable WDMA's irq */
			//VIOC_WDMA_SetImageEnable(vioc->wdma_addr, OFF);	/* if wdma isn't continuous mode */
		}
	}
	return IRQ_HANDLED;
}

int hdin_video_irq_request(struct tcc_hdin_device *hdev)
{
	struct device_node *irq_np;
	int ret = -1, irq_num;

	irq_np = of_parse_phandle(hdev->hdin_np, "hdin_wdma", 0);

	if(irq_np) 
	{
		of_property_read_u32_index(hdev->hdin_np,"hdin_wdma", 1, &vioc_num.wdma.index);
		irq_num = irq_of_parse_and_map(irq_np, vioc_num.wdma.index);

		hdev->vioc.wdma_addr->uIRQSTS.nREG |= VIOC_WDMA_IREQ_ALL_MASK;		// clear interrupt stauts
		VIOC_WDMA_SetIreqMask(hdev->vioc.wdma_addr, VIOC_WDMA_IREQ_ALL_MASK, 0x1);
		ret = request_irq(irq_num, hdin_video_isr, IRQF_SHARED, DRIVER_NAME, hdev);
	}

	return ret;
}

void hdin_video_irq_free(struct tcc_hdin_device *hdev)
{
	struct device_node *irq_np;
	int irq_num;

	irq_np = of_parse_phandle(hdev->hdin_np, "hdin_wdma", 0);

	if(irq_np) 
	{
		of_property_read_u32_index(hdev->hdin_np,"hdin_wdma", 1, &vioc_num.wdma.index);
		irq_num = irq_of_parse_and_map(irq_np, vioc_num.wdma.index);
		free_irq(irq_num, hdev);
	}
}

int hdin_video_open(struct tcc_hdin_device *hdev)
{
	struct TCC_HDIN *h = hdev->h;
	int ret = 0;

	dprintk("hdin_video_open....................\n");

	h->done_list.next = &h->done_list;
	h->list.next = &h->list;

	hdev->vioc.wdma_irq_mask = VIOC_WDMA_IREQ_EOFF_MASK;

	hdev->bypass = 0;

	if(hdev->hdin_np)
	{
		if((ret = hdin_video_irq_request(hdev)) < 0)
		{
			printk("FAILED to aquire hdin irq(%d).\n", ret);
			return ret;
		}
	}
	return 0;
}

int hdin_video_close(struct tcc_hdin_device *hdev)
{
	struct TCC_HDIN *h = hdev->h;
	dprintk("hdin_video_close....................\n");

	mutex_destroy(&h->lock);
	hdin_ctrl_cleanup(hdev);

	if(h->hdin_buf.area != NULL)
		iounmap(h->hdin_buf.area);

	hdin_video_irq_free(hdev);
	kfree(h);

	hdev->bypass = 0;

	return 0;
}

int  hdin_video_init(struct tcc_hdin_device *hdev)
{
	struct TCC_HDIN *h;

	dprintk("hdin_video_init....................\n");

	memset(&hdev->core, 0x00, sizeof(struct tcc_vin_core));

	pmap_get_info("v4l2_hdin", &hdev->core.pmap_preview);
	dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n",
		hdev->core.pmap_preview.name, hdev->core.pmap_preview.base, hdev->core.pmap_preview.base + hdev->core.pmap_preview.size, hdev->core.pmap_preview.size);

	h = kzalloc(sizeof(struct TCC_HDIN), GFP_KERNEL);
	if (h == NULL)
		return -ENOMEM;
	memset(h, 0x00, sizeof(struct TCC_HDIN));
	h->buf = h->static_buf;

	h->hdin_buf.bytes = PAGE_ALIGN(hdev->core.pmap_preview.size);
	h->hdin_buf.addr = hdev->core.pmap_preview.base;
	h->hdin_buf.area = ioremap_nocache(h->hdin_buf.addr, h->hdin_buf.bytes);
	dprintk("reamp : [0x%x - 0x%x] -> [0x%x] \n", h->hdin_buf.addr, h->hdin_buf.bytes, (unsigned int)h->hdin_buf.area);
	if (h->hdin_buf.area == NULL) {
		printk(KERN_ERR "%s: cannot remap buffer\n", DRIVER_NAME);
		return -ENODEV;
	}

	if(hdev->hdin_np)
	{
		hdin_vioc_dt_parse_data(hdev);
		hdin_set_port(hdev);
	}
	else
	{
		printk("could not find hdin node!! \n");
		return -ENODEV;
	}

	hdin_data_init(h);

	mdelay(5);

	init_waitqueue_head(&h->frame_wait);
	spin_lock_init(&h->dev_lock);

	INIT_LIST_HEAD(&h->list);
	INIT_LIST_HEAD(&h->done_list);
	mutex_init(&h->lock);

	hdev->h = h;
	return 0;
}
