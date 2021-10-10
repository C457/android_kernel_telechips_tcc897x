/****************************************************************************
FileName    : kernel/drivers/video/tcc/vioc/tcc_component.c
Description : 

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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fb.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/of_address.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/gpio.h>
#include <mach/tcc_component_ioctl.h>
#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tca_lcdc.h>

#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_global.h>
#include <mach/vioc_config.h>
#include <mach/vioc_scaler.h>

#include <mach/tcc_board_component.h>

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <mach/tcc_vsync_ioctl.h>
#endif

#else
#include <video/tcc/gpio.h>
#include <video/tcc/tcc_component_ioctl.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tca_lcdc.h>

#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_scaler.h>

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <video/tcc/tcc_vsync_ioctl.h>
#endif

#endif

#include "tcc_component.h"
#include "tcc_component_ths8200.h"

extern void tcc_video_post_process(struct tcc_lcdc_image_update *ImageInfo);

extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
									unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);

extern void tca_vioc_displayblock_powerOn(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_powerOff(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_disable(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_ctrl_set(unsigned int outDevice, struct tcc_dp_device *pDisplayInfo, stLTIMING *pstTiming, stLCDCTR *pstCtrl, unsigned int format);
extern void tca_fb_attach_start(struct tccfb_info *info);
extern void tca_fb_attach_stop(struct tccfb_info *info);

/*****************************************************************************

 VARIABLES

******************************************************************************/

extern char fb_power_state;

/* Debugging stuff */
static int debug = 0;
#define dprintk(msg...)	if (debug) { printk( "tcc_component: " msg); }

#define TCC_LCDC1_USE

static int					Component_LCDC_Num = -1;
static int					Component_Disp_Num;
static int					Component_Mode = -1;
static PVIOC_DISP			pComponent_DISP;
static PVIOC_WMIX			pComponent_WMIX;
static PVIOC_RDMA			pComponent_RDMA_UI;
static PVIOC_RDMA			pComponent_RDMA_VIDEO;
static PVIOC_SC				pComponent_SCALER;
static PVIOC_IREQ_CONFIG	pComponent_IREQ; 
static unsigned int 		*pConfigMisc1;

static PVIOC_DISP			pComponent_Attach_DISP;
static PVIOC_WMIX			pComponent_Attach_WMIX;
static PVIOC_RDMA			pComponent_Attach_RDMA_UI;
static PVIOC_RDMA			pComponent_Attach_RDMA_VIDEO;

#define DEVICE_NAME			"component"
#define COMPONENT_MINOR		206

#define COMPONENT_DETECT_GPIO		NULL
#define COMPONENT_DETECT_EINTSEL	0
#define COMPONENT_DETECT_EINTNUM	0
#define COMPONENT_DETECT_EINT		NULL

static struct clk *component_lcdc0_clk;
static struct clk *component_lcdc1_clk;

static char tcc_component_mode = COMPONENT_MAX;
static char tcc_component_attached = 0;
static char tcc_component_starter = 0;

static char component_start = 0;
static char component_plugout = 0;

static int hdmi_hpd_port = 0;

static unsigned int gComponentSuspendStatus = 0;
static unsigned int gLinuxComponentSuspendStatus = 0;

static struct device *pdev_component;

static int component_io_port_num = 0;

#define RDMA_UVI_MAX_WIDTH		3072

/*****************************************************************************

 FUNCTIONS

******************************************************************************/
extern void LCDC_IO_Set (char DD_num,  char DP_num, unsigned bit_per_pixel);
extern void LCDC_IO_Disable (char DP_num, unsigned bit_per_pixel);

#if defined(CONFIG_SWITCH_GPIO_COMPONENT)
static struct platform_device tcc_component_detect_device = {
	.name   = "switch-gpio-component-detect",
	.id             = -1,
	.dev = {
		.platform_data = NULL,
	}, 
};
#endif



/*****************************************************************************
 Function Name : tcc_component_detect()
******************************************************************************/
int tcc_component_detect(void)
{
	int detect = true;

	#if defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS)
		detect = false;
	#elif defined(CONFIG_TCC_DISPLAY_MODE_DUAL_AUTO)
		/* Check the HDMI detection */
		if(gpio_get_value(hdmi_hpd_port))
			detect = false;
	#endif
	
	return detect;
}

/*****************************************************************************
 Function Name : tcc_component_get_spec()
******************************************************************************/
void tcc_component_get_spec(COMPONENT_MODE_TYPE mode, COMPONENT_SPEC_TYPE *spec)
{
	switch(mode)
	{
		case COMPONENT_MODE_NTSC_M:
		case COMPONENT_MODE_NTSC_M_J:
		case COMPONENT_MODE_PAL_M:
		case COMPONENT_MODE_NTSC_443:
		case COMPONENT_MODE_PSEUDO_PAL:
			spec->component_clk = 27*1000*1000;
			spec->component_bus_width = 8;
			spec->component_lcd_width = 720;
			spec->component_lcd_height = 480;
			spec->component_LPW = 128 - 1; 					// line pulse width
			spec->component_LPC = 720 * 2 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->component_LSWC = 116 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->component_LEWC = 32 - 1;					// line end wait clock (the number of dummy pixel clock - 1)

			spec->component_VDB = 0; 						// Back porch Vsync delay
			spec->component_VDF = 0; 						// front porch of Vsync delay

			spec->component_FPW1 = 6 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC1 = 480 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC1 = 30 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC1 = 9 - 1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->component_FPW2 = 6 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC2 = 480 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC2 = 31 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC2 = 8 - 1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;
				
		case COMPONENT_MODE_NTSC_N:
		case COMPONENT_MODE_NTSC_N_J:
		case COMPONENT_MODE_PAL_N:
		case COMPONENT_MODE_PAL_B:
		case COMPONENT_MODE_PAL_G:
		case COMPONENT_MODE_PAL_H:
		case COMPONENT_MODE_PAL_I:
		case COMPONENT_MODE_PSEUDO_NTSC:
			spec->component_clk = 27*1000*1000;
			spec->component_bus_width = 8;
			spec->component_lcd_width = 720;
			spec->component_lcd_height = 576;
			spec->component_LPW = 128 - 1; 					// line pulse width
			spec->component_LPC = 720 * 2 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->component_LSWC = 136 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->component_LEWC = 24 - 1;					// line end wait clock (the number of dummy pixel clock - 1)

			spec->component_VDB = 0; 						// Back porch Vsync delay
			spec->component_VDF = 0; 						// front porch of Vsync delay

			spec->component_FPW1 = 5 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC1 = 576 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC1 = 39 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC1 = 5 - 1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->component_FPW2 = 5 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC2 = 576 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC2 = 40 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC2 = 4 - 1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;

		case COMPONENT_MODE_720P:
			spec->component_clk = 74250*1000;
			spec->component_bus_width = 24;
			spec->component_lcd_width = 1280;
			spec->component_lcd_height = 720;
			spec->component_LPW = 9 - 1; 					// line pulse width
			spec->component_LPC = 1280 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->component_LSWC = 349 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->component_LEWC = 12 - 1;					// line end wait clock (the number of dummy pixel clock - 1)

			spec->component_VDB = 0; 						// Back porch Vsync delay
			spec->component_VDF = 0; 						// front porch of Vsync delay
				
			spec->component_FPW1 = 3 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC1 = 720 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC1 = 26 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC1 = 1 - 1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->component_FPW2 = 3 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC2 = 720 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC2 = 26 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC2 = 1 - 1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;

		case COMPONENT_MODE_1080I:
			spec->component_clk = 74250*1000;
			spec->component_bus_width = 24;
			spec->component_lcd_width = 1920;
			spec->component_lcd_height = 1080;
			spec->component_LPW = 24 - 1; 					// line pulse width
			spec->component_LPC = 1920 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->component_LSWC = 254 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->component_LEWC = 2 - 1;					// line end wait clock (the number of dummy pixel clock - 1)

			spec->component_VDB = 0; 						// Back porch Vsync delay
			spec->component_VDF = 0; 						// front porch of Vsync delay
				
			spec->component_FPW1 = 5*2 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC1 = 540*2 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC1 = 15*2 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC1 = 2.5*2 - 1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->component_FPW2 = 5*2 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->component_FLC2 = 540*2 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->component_FSWC2 = 15.5*2 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->component_FEWC2 = 2*2 - 1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;

		default:
			break;
	}
}

/*****************************************************************************
 Function Name : tcc_component_set_lcd2tv()
******************************************************************************/
void tcc_component_set_lcd2tv(COMPONENT_MODE_TYPE mode)
{
	COMPONENT_SPEC_TYPE spec;
	stLTIMING ComponentTiming;
	stLCDCTR LcdCtrlParam;

	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;

	volatile PVIOC_IREQ_CONFIG pIREQConfig;
	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)pComponent_IREQ;

	dprintk("%s, mode=%d\n", __func__, mode);
	
	tcc_component_get_spec(mode, &spec);
	
#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)

#else
	/* set io ports for component output */
	if(Component_LCDC_Num == component_io_port_num)
	{
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 28), (1 << 29));	//LCD2_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 26), (1 << 26));	//LCD1_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 24), (0 << 24));	//LCD0_SEL
		dprintk("%s, CFG_MISC1: 0x%08x\n", __func__, pIREQConfig->uMISC1.nREG);
	}
	else
	{
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 28), (1 << 29));	//LCD2_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 26), (0 << 26));	//LCD1_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 24), (1 << 24));	//LCD0_SEL
		dprintk("%s, CFG_MISC1: 0x%08x\n", __func__, pIREQConfig->uMISC1.nREG);
	}
#endif
	
	dprintk("%s : component_io_port_num = %d, spec.component_bus_width = %d\n", __func__,component_io_port_num, spec.component_bus_width );
	LCDC_IO_Set(component_io_port_num, component_io_port_num, spec.component_bus_width);

	ComponentTiming.lpw = spec.component_LPW;
	ComponentTiming.lpc = spec.component_LPC + 1;
	ComponentTiming.lswc = spec.component_LSWC + 1;
	ComponentTiming.lewc = spec.component_LEWC + 1;
	
	ComponentTiming.vdb = spec.component_VDB;
	ComponentTiming.vdf = spec.component_VDF;
	ComponentTiming.fpw = spec.component_FPW1;
	ComponentTiming.flc = spec.component_FLC1;
	ComponentTiming.fswc = spec.component_FSWC1;
	ComponentTiming.fewc = spec.component_FEWC1;
	ComponentTiming.fpw2 = spec.component_FPW2;
	ComponentTiming.flc2 = spec.component_FLC2;
	ComponentTiming.fswc2 = spec.component_FSWC2;
	ComponentTiming.fewc2 = spec.component_FEWC2;
	
	memset(&LcdCtrlParam, 0, sizeof(LcdCtrlParam));

	switch(mode)
	{
		case COMPONENT_MODE_NTSC_M:
		case COMPONENT_MODE_NTSC_M_J:
		case COMPONENT_MODE_PAL_M:
		case COMPONENT_MODE_NTSC_443:
		case COMPONENT_MODE_PSEUDO_PAL:
		case COMPONENT_MODE_NTSC_N:
		case COMPONENT_MODE_NTSC_N_J:
		case COMPONENT_MODE_PAL_N:
		case COMPONENT_MODE_PAL_B:
		case COMPONENT_MODE_PAL_G:
		case COMPONENT_MODE_PAL_H:
		case COMPONENT_MODE_PAL_I:
		case COMPONENT_MODE_PSEUDO_NTSC:
			break;

		case COMPONENT_MODE_720P:
			//LcdCtrlParam.r2ymd = 3;
			LcdCtrlParam.ckg = 1;
			LcdCtrlParam.id= 0;
			LcdCtrlParam.iv = 1;
			LcdCtrlParam.ih = 1;
			LcdCtrlParam.ip = 1;
			LcdCtrlParam.pxdw = 12;
			LcdCtrlParam.ni = 1;
			break;

		case COMPONENT_MODE_1080I:
			//LcdCtrlParam.r2ymd = 3;
			#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
			LcdCtrlParam.advi = 0;
			#else
			LcdCtrlParam.advi = 1;
			#endif
			LcdCtrlParam.ckg = 1;
			LcdCtrlParam.id= 0;
			LcdCtrlParam.iv = 0;
			LcdCtrlParam.ih = 1;
			LcdCtrlParam.ip = 1;
			LcdCtrlParam.pxdw = 12;
			LcdCtrlParam.ni = 0;
			LcdCtrlParam.tv = 1;
			break;

		default:
			break;
	}
	
	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
		LcdCtrlParam.r2ymd = 0;
	#else
		LcdCtrlParam.r2ymd = 3;
	#endif

	tccfb_info = info->par;

	#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
		dp_device = &tccfb_info->pdata.Sdp_data;
	#else
	if(tccfb_info->pdata.Mdp_data.DispNum == Component_LCDC_Num) {
		dp_device = &tccfb_info->pdata.Mdp_data;
	}
	else if(tccfb_info->pdata.Sdp_data.DispNum == Component_LCDC_Num) {
		dp_device = &tccfb_info->pdata.Sdp_data;
		dp_device->DispOrder = 1;
	}
	#endif

	if(dp_device)
	{
		tca_vioc_displayblock_disable(dp_device);

		dp_device->DispDeviceType = TCC_OUTPUT_COMPONENT;
		if(tcc_component_attached) {
			dp_device->FbUpdateType = FB_ATTACH_UPDATE;
		}
		else {
			dp_device->FbUpdateType = FB_SC_RDMA_UPDATE;
			dp_device->sc_num0 = VIOC_SC2;
		}

		#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
			dp_device->DispOrder = 1;	//Set to Sub Display

			#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
				dp_device->FbUpdateType = FB_SC_RDMA_UPDATE;
				dp_device->sc_num0 = VIOC_SC4;
			#else
				dp_device->FbUpdateType = FB_RDMA_UPDATE;
			#endif//

			#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
				if(dp_device->FbUpdateType == FB_SC_RDMA_UPDATE)	{
					if(VIOC_CONFIG_PlugOut(dp_device->sc_num0) == VIOC_PATH_DISCONNECTED)
					{
						VIOC_SC_SetSWReset(dp_device->sc_num0, dp_device->rdma_info[RDMA_FB].blk_num, 0xFF);
						VIOC_CONFIG_PlugIn(dp_device->sc_num0, dp_device->rdma_info[RDMA_FB].blk_num);
					}
				}
			#endif
		#endif//CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB
		tca_vioc_displayblock_powerOn(dp_device);
		tca_vioc_displayblock_ctrl_set(VIOC_OUTCFG_HDVENC, dp_device, &ComponentTiming, &LcdCtrlParam, 0);
	}

#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
		//Set LCDC1 to Component
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 28), (1 << 29));	//LCD2_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 26), (0 << 26));	//LCD1_SEL
		BITCSET(pIREQConfig->uMISC1.nREG, (3 << 24), (1 << 24));	//LCD0_SEL
		dprintk("%s, CFG_MISC1: 0x%08x\n", __func__, pIREQConfig->uMISC1.nREG);
#endif


	
}

/*****************************************************************************
 Function Name : tcc_component_get_lcdsize()
******************************************************************************/
void tcc_component_get_lcdsize(unsigned int *width, unsigned int *height)
{
	unsigned int lcdsize;
		
	lcdsize = pComponent_DISP->uLSIZE.nREG;

	*width = lcdsize & 0x0000FFFF;
	*height = lcdsize >>16;
}

static int onthefly_using;

// 0 : 3 : layer enable/disable 
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
extern int enabled_LastFrame;

void tcc_plugout_for_component(int ch_layer)
{
	unsigned int iSCType;

	#if defined(CONFIG_ARCH_TCC893X)
		#if defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8937S) || defined(CONFIG_MACH_TCC8930ST)
		iSCType = VIOC_SC1;
		#else
		iSCType = VIOC_SC3;
		#endif
	#else
		iSCType = VIOC_SC1;
	#endif /* CONFIG_ARCH_TCC893X */

	if(ISSET(onthefly_using, 1<<ch_layer))
	{
		VIOC_CONFIG_PlugOut(iSCType);
		BITCLR(onthefly_using, 1 << ch_layer);
	}

}
EXPORT_SYMBOL(tcc_plugout_for_component);
extern unsigned int LastFrame;
#endif

/*****************************************************************************
 Function Name : tcc_component_update()
******************************************************************************/
void tcc_component_update(struct tcc_lcdc_image_update *ImageInfo)
{
	VIOC_DISP * pDISPBase;
	VIOC_WMIX * pWMIXBase;
	VIOC_RDMA * pRDMABase;
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	VIOC_RDMA * pRDMABase_temp;
#endif		
	VIOC_SC *pSC;
	
	unsigned int lcd_width = 0, lcd_height = 0;

	unsigned int iSCType;
	unsigned int nRDMB;
	unsigned int interlace_output = 0;

	pSC = pComponent_SCALER;
	iSCType = VIOC_SC1;
	
	dprintk("%s enable:%d, layer:%d, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", __func__, ImageInfo->enable, ImageInfo->Lcdc_layer,
			ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);
	
	ImageInfo->Lcdc_layer = 3;

	pDISPBase = (VIOC_DISP*)pComponent_DISP;
	pWMIXBase = (VIOC_WMIX*)pComponent_WMIX;		
	pRDMABase = (VIOC_RDMA*)pComponent_RDMA_VIDEO;		
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	pRDMABase_temp  = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA05);
#endif		
		
	if(!ImageInfo->enable)	{
		volatile PVIOC_IREQ_CONFIG pIREQConfig;
		pIREQConfig = (volatile PVIOC_IREQ_CONFIG)pComponent_IREQ;

		VIOC_RDMA_SetImageDisable(pRDMABase);		

		nRDMB = Component_LCDC_Num*4 + ImageInfo->Lcdc_layer;
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<nRDMB), (0x1<<nRDMB)); // rdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<nRDMB), (0x0<<nRDMB)); // rdma reset
		
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
		if(enabled_LastFrame){
			VIOC_RDMA_SetImageDisable(pRDMABase_temp);	
		}
#endif	

		if(ImageInfo->MVCframeView != 1)
		{
			if(ISSET(onthefly_using, 1<<ImageInfo->Lcdc_layer))
			{
				dprintk("%s scaler_%d is plugged out!!\n", __func__, iSCType);
			
				VIOC_CONFIG_PlugOut(iSCType);
				BITCLR(onthefly_using, 1 << ImageInfo->Lcdc_layer);

				BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+iSCType)), (0x1<<(28+iSCType))); // scaler reset
				BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+iSCType)), (0x0<<(28+iSCType))); // scaler reset

			}		
		}
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
		LastFrame = 0;
#endif
		return;
	}	

	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);
	
	if((!lcd_width) || (!lcd_height)){
		printk(" %s Error :: lcd_width %d, lcd_height %d, hdmi_lcdc=%d, enable=%d\n", __func__,lcd_width, lcd_height, Component_LCDC_Num, ImageInfo->enable);
		return;
	}

	if((ImageInfo->Lcdc_layer >= 4) || (ImageInfo->fmt >TCC_LCDC_IMG_FMT_MAX))
		return;

	#if defined(CONFIG_TCC_VIDEO_DISPLAY_BY_VSYNC_INT)
		if(ImageInfo->outputMode != OUTPUT_COMPONENT)
			return;
	#endif

	//dprintk("%s lcdc:%d, pRDMA:0x%08x, pWMIX:0x%08x, pDISP:0x%08x, addr0:0x%08x\n", __func__, hdmi_lcdc, pRDMABase, pWMIXBase, pDISPBase, ImageInfo->addr0);

#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
	if( !(pDISPBase->uCTRL.nREG & HwDISP_NI )) {//interlace mode
		interlace_output = 1;
	}
#endif

	if((ImageInfo->MVCframeView != 1) && ImageInfo->on_the_fly)
	{
		unsigned int RDMA_NUM;
		RDMA_NUM = Component_LCDC_Num ? (ImageInfo->Lcdc_layer + VIOC_SC_RDMA_04) : ImageInfo->Lcdc_layer;

		VIOC_SC_SetSrcSize(pSC, ImageInfo->Frame_width, ImageInfo->Frame_height);
		VIOC_SC_SetDstSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set destination size in scaler
		VIOC_SC_SetOutSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set output size in scaer

		if(ImageInfo->Frame_width==ImageInfo->Image_width && ImageInfo->Frame_height==ImageInfo->Image_height) {
			VIOC_SC_SetBypass (pSC, true);
			dprintk("%s scaler_%d is plug in SetBypass ON \n", __func__, iSCType);
		}else {
			VIOC_SC_SetBypass (pSC, false);
			dprintk("%s scaler_%d is plug in SetBypass OFF \n", __func__, iSCType);
		}

		if(!onthefly_using)
		{ 
			VIOC_RDMA_SetImageDisable(pRDMABase);
			dprintk("%s scaler_%d is plug in RDMA %d \n", __func__, RDMA_NUM, iSCType);
			BITSET(onthefly_using, 1 << ImageInfo->Lcdc_layer);
			VIOC_CONFIG_PlugIn (iSCType, RDMA_NUM);	
		}
	}
	else
	{
		if(ISSET(onthefly_using, 1<<ImageInfo->Lcdc_layer))
		{
			dprintk("%s scaler_%d is plug out  \n", __func__, iSCType);
			VIOC_RDMA_SetImageDisable(pRDMABase);
			VIOC_CONFIG_PlugOut(iSCType);
			BITCLR(onthefly_using, 1 << ImageInfo->Lcdc_layer);
		}
	}

	if(onthefly_using)
		VIOC_SC_SetUpdate (pSC);
		
	// position
	VIOC_WMIX_SetPosition(pWMIXBase, ImageInfo->Lcdc_layer, ImageInfo->offset_x, ImageInfo->offset_y);
	VIOC_WMIX_SetUpdate(pWMIXBase);	
		
	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
		VIOC_RDMA_SetImageY2REnable(pRDMABase, false);
	#else
		if(ImageInfo->fmt >= TCC_LCDC_IMG_FMT_UYVY && ImageInfo->fmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)
		{
			VIOC_RDMA_SetImageY2REnable(pRDMABase, true);
			VIOC_RDMA_SetImageY2RMode(pRDMABase, 1); /* Y2RMode Default 0 (Studio Color) */

			if( ImageInfo->Frame_width <= RDMA_UVI_MAX_WIDTH )
				VIOC_RDMA_SetImageUVIEnable(pRDMABase, true);
			else
				VIOC_RDMA_SetImageUVIEnable(pRDMABase, false);
		}
	#endif

	if(ImageInfo->one_field_only_interlace)
		VIOC_RDMA_SetImageOffset(pRDMABase, ImageInfo->fmt, ImageInfo->Frame_width*2);
	else
		VIOC_RDMA_SetImageOffset(pRDMABase, ImageInfo->fmt, ImageInfo->Frame_width);

	VIOC_RDMA_SetImageFormat(pRDMABase, ImageInfo->fmt);

	if( !( ((ImageInfo->crop_left == 0) && (ImageInfo->crop_right == ImageInfo->Frame_width)) &&  ((ImageInfo->crop_top == 0) && (ImageInfo->crop_bottom == ImageInfo->Frame_height)))  )
	{
		unsigned int addr_Y = ImageInfo->addr0;
		unsigned int addr_U = ImageInfo->addr1;
		unsigned int addr_V = ImageInfo->addr2;
		
		dprintk(" Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);

		tccxxx_GetAddress(ImageInfo->fmt, (unsigned int)ImageInfo->addr0, ImageInfo->Frame_width, ImageInfo->Frame_height, 		
								ImageInfo->crop_left, ImageInfo->crop_top, &addr_Y, &addr_U, &addr_V);		

		if(ImageInfo->one_field_only_interlace)
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->crop_right - ImageInfo->crop_left, (ImageInfo->crop_bottom - ImageInfo->crop_top)/2);
		else
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->crop_right - ImageInfo->crop_left, ImageInfo->crop_bottom - ImageInfo->crop_top);
		VIOC_RDMA_SetImageBase(pRDMABase, addr_Y, addr_U, addr_V);
	}
	else
	{	
		dprintk(" don't Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);
		if(ImageInfo->one_field_only_interlace)
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->Frame_width, ImageInfo->Frame_height/2);
		else
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->Frame_width, ImageInfo->Frame_height);
		VIOC_RDMA_SetImageBase(pRDMABase, ImageInfo->addr0, ImageInfo->addr1, ImageInfo->addr2);		
	}
		
	VIOC_RDMA_SetImageIntl(pRDMABase, interlace_output);

	VIOC_WMIX_SetUpdate(pWMIXBase);	
	
	VIOC_RDMA_SetImageEnable(pRDMABase);

	tcc_video_post_process(ImageInfo);
}

/*****************************************************************************
 Function Name : tcc_component_end()
******************************************************************************/
void tcc_component_end(void)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	unsigned int component_bus_width = 24;
	
	dprintk("%s, LCDC_Num = %d\n", __func__, Component_LCDC_Num);

	//tcc_component_set_interrupt(false, Component_LCDC_Num);

 	#ifdef TCC_COMPONENT_IC_THS8200
		//ths8200_power(0);
		ths8200_reset();
	#endif

	/* Clear the start flag */
	component_start = 0;
	
	if(component_plugout)
		component_plugout = 0;

	tccfb_info = info->par;

	if(tccfb_info->pdata.Mdp_data.DispNum == Component_LCDC_Num)
		dp_device = &tccfb_info->pdata.Mdp_data;
	else if(tccfb_info->pdata.Sdp_data.DispNum == Component_LCDC_Num)
		dp_device = &tccfb_info->pdata.Sdp_data;
	else
		return;

	tca_vioc_displayblock_disable(dp_device);
	tca_vioc_displayblock_powerOff(dp_device);
	dp_device->DispDeviceType = TCC_OUTPUT_NONE;

	LCDC_IO_Disable(component_io_port_num, component_bus_width);

}

/*****************************************************************************
 Function Name : tcc_component_start()
******************************************************************************/
void tcc_component_start(TCC_COMPONENT_MODE_TYPE mode)
{
	printk("%s mode=%d, lcdc_num=%d\n", __func__, mode, Component_LCDC_Num);

	tcc_component_mode = mode;

	switch(mode)
	{
		case COMPONENT_NTST_M:
			Component_Mode = COMPONENT_MODE_NTSC_M;
			break;
		case COMPONENT_PAL:
			Component_Mode = COMPONENT_MODE_PAL_B;
			break;
		case COMPONENT_720P:
			Component_Mode = COMPONENT_MODE_720P;
			break;
		case COMPONENT_1080I:
			Component_Mode = COMPONENT_MODE_1080I;
			break;
		default:
			break;
	}

	tcc_component_set_lcd2tv(Component_Mode);
 
	#ifdef TCC_COMPONENT_IC_THS8200
		ths8200_enable(Component_Mode, tcc_component_starter);
	#endif

	/* Set the start flag */
	component_start = 1;
}

/*****************************************************************************
 Function Name : tcc_component_clock_onoff()
******************************************************************************/
void tcc_component_clock_onoff(char OnOff)
{
	dprintk("%s, Onoff = %d \n", __func__, OnOff);
}

#ifdef CONFIG_PM_RUNTIME
static int component_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_component);

	return 0;
}

static int component_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_component);

	return 0;
}
#endif

static int component_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n", __func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_component->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
		//printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_component->power.usage_count, pdev_component->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			component_disable();
			break;
		case FB_BLANK_UNBLANK:
			component_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}

/*****************************************************************************
 Function Name : tcc_component_attach()
******************************************************************************/
void tcc_component_attach(char lcdc_num, char mode, char starter_flag)
{
	char component_mode;

	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;

	dprintk("%s, lcdc_num=%d, mode=%d, starter=%d\n", __func__, lcdc_num, mode, starter_flag);
	
	/* reset previous display path */
	if(starter_flag)
	{
		VIOC_DISP_TurnOff(pComponent_Attach_DISP);
		VIOC_DISP_Wait_DisplayDone(pComponent_Attach_DISP);
		VIOC_RDMA_SetImageDisable(pComponent_Attach_RDMA_UI);
		VIOC_RDMA_SetImageDisable(pComponent_Attach_RDMA_VIDEO);
	}

	tccfb_info = info->par;

	tcc_component_attached = 1;
	tcc_component_starter = starter_flag;

	Component_LCDC_Num = lcdc_num;
	
	if(mode >= COMPONENT_MAX)
		component_mode = COMPONENT_720P;
	else
		component_mode = mode;

	tcc_component_clock_onoff(true);
	tcc_component_start(component_mode);

	tca_fb_attach_start(tccfb_info);
}

/*****************************************************************************
 Function Name : tcc_component_detach()
******************************************************************************/
void tcc_component_detach(void)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;

	printk("%s\n", __func__);
	
	tccfb_info = info->par;

	tcc_component_end();

	tca_fb_attach_stop(tccfb_info);

	tcc_component_attached = 0;
	tcc_component_starter = 0;

	Component_LCDC_Num = Component_Disp_Num;
}

/*****************************************************************************
 Function Name : tcc_component_ioctl()
******************************************************************************/
static long tcc_component_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	TCC_COMPONENT_START_TYPE start;
	struct tcc_lcdc_image_update update;			
	
	switch(cmd)
	{
		case TCC_COMPONENT_IOCTL_HPD_SWITCH_STATUS	:
		{
			#if defined(CONFIG_SWITCH_GPIO_COMPONENT)
			unsigned int enable = 0;
			//dprintk(KERN_INFO "ioctl(TCC_COMPONENT_IOCTL_HPD_SWITCH_STATUS)\n");

			if(copy_from_user(&enable, (void *)arg, sizeof(enable)))
				return -EFAULT;

			
			if(tcc_component_detect_device.dev.platform_data != NULL)
			{
				struct component_gpio_switch_data	*switch_data = tcc_component_detect_device.dev.platform_data;

				if(enable) {
					switch_data->send_component_event(switch_data, TCC_COMPONENT_ON);
					printk(KERN_INFO "ioctl(TCC_COMPONENT_IOCTL_HPD_SWITCH_STATUS), enable = %d\n", enable);
				} else {
					switch_data->send_component_event(switch_data, TCC_COMPONENT_OFF);
					printk(KERN_INFO "ioctl(TCC_COMPONENT_IOCTL_HPD_SWITCH_STATUS), enable = %d\n", enable);
				}
			}
			#endif//
			break;
		}
		case TCC_COMPONENT_IOCTL_START:
			dprintk("%s [TCC_COMPONENT_IOCTL_START] \n", __func__);
			if(copy_from_user(&start, (void *)arg, sizeof(start)))
				return -EFAULT;

			if(tcc_component_attached)
				tcc_component_detach();

			tcc_component_start(start.mode);
 			break;

		case TCC_COMPONENT_IOCTL_UPDATE:
			if(copy_from_user(&update, (void *)arg, sizeof(update)))
				return -EFAULT;

			tcc_component_update(&update);									
			break;
			
		case TCC_COMPONENT_IOCTL_END:
			dprintk("%s [TCC_COMPONENT_IOCTL_END] \n", __func__);
			#if defined(CONFIG_SWITCH_GPIO_COMPONENT)
			if(tcc_component_detect_device.dev.platform_data != NULL)
			{
				struct component_gpio_switch_data	*switch_data = tcc_component_detect_device.dev.platform_data;
				switch_data->send_component_event(switch_data, TCC_COMPONENT_OFF);
			}
			#endif//

			tcc_component_end();
 			break;

		case TCC_COMPONENT_IOCTL_PROCESS:
			break;

		case TCC_COMPONENT_IOCTL_ATTACH:
			#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
				if(copy_from_user(&start, (void *)arg, sizeof(start)))
					return -EFAULT;

				tcc_component_mode = start.mode;

				if(tcc_component_attached)
					tcc_component_detach();

				if(start.lcdc)
					tcc_component_attach(0, start.mode, 0);
				else
					tcc_component_attach(1, start.mode, 0);
			#endif
			break;

		case TCC_COMPONENT_IOCTL_DETACH:
			#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
				tcc_component_detach();
			#endif
			break;
			
		case TCC_COMPONENT_IOCTL_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *)arg))
				return -EFAULT;

			dprintk(KERN_INFO "COMPONENT: ioctl(TCC_COMPONENT_IOCTL_BLANK :  %d )\n", cmd);

			component_blank(cmd);
			break;

		}
			
		case TCC_COPONENT_IOCTL_GET_SUSPEND_STATUS:
		{
			//dprintk(KERN_INFO "COMPONENT: ioctl(TCC_COPONENT_IOCTL_GET_SUSPEND_STATUS : %d )\n", gComponentSuspendStatus);

			if (gLinuxComponentSuspendStatus)
			{
				put_user(gLinuxComponentSuspendStatus, (unsigned int __user*)arg);
				gLinuxComponentSuspendStatus = 0;
			}
			else
			{
				put_user(gComponentSuspendStatus,(unsigned int __user*)arg);
			}

			break;			
		}
			
		default:
			printk("%d, Unsupported IOCTL!!!\n", cmd);      
			break;			
	}

	return 0;
}

/*****************************************************************************
 Function Name : tcc_component_open()
******************************************************************************/
static int tcc_component_open(struct inode *inode, struct file *filp)
{	
	dprintk("%s  \n", __func__);

	return 0;
}

/*****************************************************************************
 Function Name : tcc_component_release()
******************************************************************************/
static int tcc_component_release(struct inode *inode, struct file *file)
{
	dprintk("%s  \n", __func__);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
int component_runtime_suspend(struct device *dev)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	
	printk("%s:  \n", __FUNCTION__);

	if(component_start) {
		tcc_component_end();

		tccfb_info = info->par;

		if(tccfb_info->pdata.Mdp_data.DispNum == Component_LCDC_Num)
			dp_device = &tccfb_info->pdata.Mdp_data;
		else if(tccfb_info->pdata.Sdp_data.DispNum == Component_LCDC_Num)
			dp_device = &tccfb_info->pdata.Sdp_data;

		if(dp_device)
		{
			tca_vioc_displayblock_disable(dp_device);
			tca_vioc_displayblock_powerOff(dp_device);
			dp_device->DispDeviceType = TCC_OUTPUT_NONE;
		}
	}

	gComponentSuspendStatus = 1;

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int component_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	gComponentSuspendStatus = 0;

	return 0;
}

#endif

#if defined(CONFIG_PM)
static int component_suspend(struct device *dev)
{
#ifndef CONFIG_ANDROID //PJJ
	/* Linux Platform */
	tcc_component_end();
	gLinuxComponentSuspendStatus = 1;
#endif//
	dprintk(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}

static int component_resume(struct device *dev)
{
	dprintk(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}
#endif

static struct file_operations tcc_component_fops = 
{
	.owner          = THIS_MODULE,
	.unlocked_ioctl = tcc_component_ioctl,
	.open           = tcc_component_open,
	.release        = tcc_component_release,	
};

static struct miscdevice tcc_component_misc_device =
{
    .minor = COMPONENT_MINOR,
    .name  = "component",
    .fops  = &tcc_component_fops,
};

#ifdef CONFIG_OF
static int component_parse_dt(struct device_node *np)
{
	struct device_node *np_fb_child, *np_fb, *np_fb_1st, *np_fb_2nd, *np_hpd;
	int index = 0, ret = 0;

	/* get the number of io port - [0]:GPIO_B, [1]:GPIO_E */
	of_property_read_u32(np, "io_port_num", &component_io_port_num);

	/* get the information of component device node */
	component_lcdc0_clk = of_clk_get(np, 0);
	BUG_ON(component_lcdc0_clk == NULL);
	component_lcdc1_clk = of_clk_get(np, 1);
	BUG_ON(component_lcdc1_clk == NULL);

	np_fb_child = of_parse_phandle(np, "scaler", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np, "scaler", 1, &index);
		pComponent_SCALER = (PVIOC_SC)of_iomap(np_fb_child, index);
	} else {
		printk("%s, could not find scaler node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np, "config", 0);
	if(np_fb_child) {
		pComponent_IREQ = (PVIOC_IREQ_CONFIG)of_iomap(np_fb_child, 0);
		pConfigMisc1 = (unsigned int *)(pComponent_IREQ + 0x84);
	} else {
		printk("%s, could not find irq_config node\n", __func__);
		ret = -ENODEV;
	}

	/* get the information of vioc-fb device node */
	np_fb = of_find_compatible_node(NULL, NULL, "telechips,vioc-fb");

	if(of_property_read_u32(np_fb, "telechips,fbdisplay_num", &Component_Disp_Num)) {
		pr_err("%s, could not find fbdisplay_num\n", __func__);
		ret = -ENODEV;
	}

	#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
		component_io_port_num = 0;
		Component_Disp_Num = 1;
	#endif


	if(Component_Disp_Num) {
		np_fb_1st = of_find_node_by_name(np_fb, "fbdisplay1");
		np_fb_2nd = of_find_node_by_name(np_fb, "fbdisplay0");
	}
	else {
		np_fb_1st = of_find_node_by_name(np_fb, "fbdisplay0");
		np_fb_2nd = of_find_node_by_name(np_fb, "fbdisplay1");
	}

	Component_LCDC_Num = Component_Disp_Num;

	/* get register address for main output */
	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,disp", 0);
	if(np_fb_child) {
		pComponent_DISP = (VIOC_DISP *)of_iomap(np_fb_child, 0);
	} else {
		pr_err( "%s, could not find disp node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,wmixer", 0);
	if(np_fb_child) {
		pComponent_WMIX = (VIOC_WMIX *)of_iomap(np_fb_child, 0);
	} else {
		pr_err( "%s, could not find disp wmixer\n", __func__);
		ret = -ENODEV;
	}
	
	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,rdma", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np_fb_1st, "telechips,rdma", 1+0, &index);
		pComponent_RDMA_UI = (VIOC_RDMA *)of_iomap(np_fb_child, index);
		of_property_read_u32_index(np_fb_1st, "telechips,rdma", 1+3, &index);
		pComponent_RDMA_VIDEO = (VIOC_RDMA *)of_iomap(np_fb_child, index);
	} else {
		pr_err( "%s, could not find disp rdma\n", __func__);
		ret = -ENODEV;
	}

	/* get register address for attached output */
	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,disp", 0);
	if(np_fb_child) {
		pComponent_Attach_DISP = (VIOC_DISP *)of_iomap(np_fb_child, 0);
	} else {
		pr_err( "%s, could not find disp node for attached output\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,wmixer", 0);
	if(np_fb_child) {
		pComponent_Attach_WMIX = (VIOC_WMIX *)of_iomap(np_fb_child, 0);
	} else {
		pr_err( "%s, could not find disp wmixer for attached output\n", __func__);
		ret = -ENODEV;
	}
	
	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,rdma", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np_fb_2nd, "telechips,rdma", 1+0, &index);
		pComponent_Attach_RDMA_UI = (VIOC_RDMA *)of_iomap(np_fb_child, index);
		of_property_read_u32_index(np_fb_2nd, "telechips,rdma", 1+3, &index);
		pComponent_Attach_RDMA_VIDEO = (VIOC_RDMA *)of_iomap(np_fb_child, index);
	} else {
		pr_err( "%s, could not find disp rdma for attached output\n", __func__);
		ret = -ENODEV;
	}

	/* get the information of hpd node */
	#if defined(CONFIG_ARCH_TCC893X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc893x-hdmi-hpd");
	#elif defined(CONFIG_ARCH_TCC896X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc896x-hdmi-hpd");
	#elif defined(CONFIG_ARCH_TCC897X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc897x-hdmi-hpd");
	#endif
	if(np_hpd) {
		hdmi_hpd_port = of_get_gpio(np_hpd, 0);
		if(gpio_is_valid(hdmi_hpd_port)) 
		{
			printk("%s, hpd port: 0x%02x\n", __func__, hdmi_hpd_port);
			gpio_request(hdmi_hpd_port, "component-hdmi-hpd");
			gpio_direction_input(hdmi_hpd_port);
		}
		else
		{
			printk("%s, err to get hpd port\n", __func__);
			hdmi_hpd_port = -1;
		}
	} else {
		printk( "%s, could not find hpd node\n", __func__);
		ret = -ENODEV;
	}

	dprintk("%s, Component_LCDC_Num = %d \n", __func__, Component_Disp_Num);

	return ret;
}
#else
static int component_parse_dt(struct device_node *np)
{
}
#endif

static int component_probe(struct platform_device *pdev)
{
	printk("%s\n", __func__);

	pdev_component = &pdev->dev;
	
	/* parse device tree */
	component_parse_dt(pdev->dev.of_node);

	if(misc_register(&tcc_component_misc_device))
	{
	    printk(KERN_WARNING "COMPONENT: Couldn't register device 10, %d.\n", COMPONENT_MINOR);
	    return -EBUSY;
	}
	
#if defined(CONFIG_SWITCH_GPIO_COMPONENT)
	platform_device_register(&tcc_component_detect_device);
#endif//


#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_component);	
	pm_runtime_enable(pdev_component);  
	pm_runtime_get_noresume(pdev_component);  //increase usage_count 
#endif

	return 0;
}

static int component_remove(struct platform_device *pdev)
{
	printk("%s LCDC:%d \n", __func__, Component_LCDC_Num);

	misc_deregister(&tcc_component_misc_device);

	if(component_lcdc0_clk)
		clk_put(component_lcdc0_clk);
	if(component_lcdc1_clk)
		clk_put(component_lcdc1_clk);

	return 0;
}

static const struct dev_pm_ops component_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = component_runtime_suspend,
	.runtime_resume = component_runtime_resume,
#else
	.suspend = component_suspend,
	.resume	= component_resume,
#endif
};

#ifdef CONFIG_OF
static struct of_device_id component_of_match[] = {
	{ .compatible = "telechips,tcc893x-component" },
	{ .compatible = "telechips,tcc896x-component" },
	{ .compatible = "telechips,tcc897x-component" },
	{}
};
MODULE_DEVICE_TABLE(of, component_of_match);
#endif

static struct platform_driver tcc_component_driver = {
	.probe	= component_probe,
	.remove	= component_remove,
	.driver	= {
		.name	= "tcc_component",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &component_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(component_of_match),
#endif
	},
};

/*****************************************************************************
 Function Name : tcc_component_init()
******************************************************************************/
int __init tcc_component_init(void)
{
	return platform_driver_register(&tcc_component_driver);
}

/*****************************************************************************
 Function Name : tcc_component_cleanup()
******************************************************************************/
void __exit tcc_component_exit(void)
{
	platform_driver_unregister(&tcc_component_driver);
}

module_init(tcc_component_init);
module_exit(tcc_component_exit);

MODULE_AUTHOR("Telechips");
MODULE_DESCRIPTION("Telechips COMPONENT Out driver");
MODULE_LICENSE("GPL");

