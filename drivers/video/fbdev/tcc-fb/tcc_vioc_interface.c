/*
 * linux/drivers/video/tcc/tccfb_interface.c
 *
 * Based on:    Based on s3c2410fb.c, sa1100fb.c and others
 * Author:  <linux@telechips.com>
 * Created: June 10, 2008
 * Description: TCC LCD Controller Frame Buffer Driver
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
#include <linux/err.h>
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
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/of_address.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#ifdef CONFIG_PM
#include "tcc_hibernation_logo.h"
#include <linux/pm.h>
#endif
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>

#include <mach/tca_lcdc.h>
#include <mach/tcc_fb.h>
#include <mach/tccfb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/TCC_LCD_Interface.h>
#include <mach/tcc_grp_ioctrl.h>
#include <mach/tcc_scaler_ctrl.h>
#include <mach/vioc_scaler.h>
#include <mach/tcc_composite_ioctl.h>
#include <mach/tcc_component_ioctl.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_config.h>
#include <mach/vioc_viqe.h>
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
#include <mach/vioc_mc.h>
#include "vioc/tca_map_converter.h"
#endif

#include <mach/tca_display_config.h>
#include <mach/tccfb_address.h>

#include <mach/vioc_global.h>
#include <mach/vioc_api.h>
#include <mach/vioc_ireq.h>
#else
#include <video/tcc/irqs.h>			//TODO: remove
#include <video/tcc/tcc_types.h>

#include <video/tcc/tca_lcdc.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/TCC_LCD_Interface.h>
#include <video/tcc/tcc_grp_ioctrl.h>
#include <video/tcc/tcc_scaler_ctrl.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/tcc_composite_ioctl.h>
#include <video/tcc/tcc_component_ioctl.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_viqe.h>
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
#include <video/tcc/vioc_mc.h>
#include "vioc/tca_map_converter.h"
#endif

#include <video/tcc/tca_display_config.h>
#include <video/tcc/tccfb_address.h>

#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_ireq.h>
#endif

#if defined(CONFIG_TCC_VIOCMG)
#include <video/tcc/viocmg.h>
#endif

#if defined (CONFIG_VIDEO_TCC_VOUT)
#include <video/tcc/tcc_vout_v4l2.h>
extern unsigned int tcc_vout_get_status(void);
#endif
#if defined(CONFIG_ANDROID)
#define AnD_FB_SC	(VIOC_SC2)
#define UI_CHROMA_EN	(0)
#elif defined(CONFIG_PLATFORM_AVN)
#define AnD_FB_SC	(VIOC_SC0)
#define UI_CHROMA_EN	(0)
#elif defined(CONFIG_PLATFORM_STB)
#define AnD_FB_SC	(VIOC_SC2)
#define UI_CHROMA_EN	(1)
#else
#define AnD_FB_SC	(VIOC_SC2)
#define UI_CHROMA_EN	(0)
#endif

#ifdef CONFIG_DAUDIO_KK
#if defined(CONFIG_USE_DISPLAY_FB_LOCK)
extern unsigned int fb_lock;
#endif
#endif

/* Debugging stuff */
//#define DISPLAY_TIMING_DUMP
static int debug = 0;
#define dprintk(msg...)	if (debug) { printk( "VIOC_I: " msg); }
 
struct device_node *ViocScaler_np;
struct device_node *ViocConfig_np;

#if defined(CONFIG_HIBERNATION) 
extern void fb_quickboot_resume(struct tccfb_info *info);

#if defined(CONFIG_USING_LAST_FRAMEBUFFER)
extern void fb_quickboot_lastframe_display_release(void);
#endif
#endif//CONFIG_HIBERNATION

extern TCC_LAYER_MODE Output_LayerMode;

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
extern VIOC_RDMA * pLastFrame_RDMABase;
extern int enabled_LastFrame;
extern void tcc_plugout_for_composite(int ch_layer);
extern void tcc_plugout_for_component(int ch_layer);
extern void tcc_video_clear_last_frame(VIOC_RDMA *rdma, bool reset);
#endif	
#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
extern void tcc_video_frame_backup(struct tcc_lcdc_image_update *Image);
#endif
#ifdef CONFIG_DISPLAY_EXT_FRAME
extern int tcc_ctrl_ext_frame(char enable);
#endif

extern void hdmi_stop(void);
extern int hdmi_get_VBlank(void);

int tccxxx_grp_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int tccxxx_grp_release(struct inode *inode, struct file *filp);
int tccxxx_grp_open(struct inode *inode, struct file *filp);

#define DEV_SCALER1	"dev/scaler1"

static struct file *scaler_filp = NULL;

static struct inode g2d_inode;
static struct file g2d_filp;

static int (*g2d_ioctl) ( struct file *, unsigned int, unsigned long);
static int (*g2d_open) (struct inode *, struct file *);
static int (*g2d_release) (struct inode *, struct file *);

static char *fb_scaler_pbuf0;
static char *fb_scaler_pbuf1;
static char *fb_g2d_pbuf0;
static char *fb_g2d_pbuf1;

static tcc_display_mouse mouse_data;
static tcc_display_resize resize_data;
static tcc_display_divide divide_data;
static tcc_display_resize output_attach_resize_data;

#ifdef CONFIG_ANDROID
#define ATTACH_SCALER VIOC_SC0
#else
#define ATTACH_SCALER VIOC_SC3
#endif//

static tcc_display_attach		attach_data;
static tcc_display_attach_intr	attach_intr;

#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
static tcc_display_resize secondary_display_resize_data;	//composite , component
#endif//

static char disable_video_composition = false;	// for disabling video composition : 160502 inbest

static void tca_fb_get_hdmi_configs(unsigned int format, unsigned *pxdw, unsigned *r2y, unsigned *swapbf) {
        switch(format)
        {
                case 1:
                        // YCC444
                        printk("tca_fb_get_hdmi_configs-YCC444\r\n");
                        *pxdw = 12;
                        *r2y = 1;
                        *swapbf = 2;
                        break;
                case 2:
                        // YCC422
                        printk("tca_fb_get_hdmi_configs-YCC422\r\n");
                        *pxdw = 21;
                        *r2y = 1;
                        *swapbf = 5;
                        break;
                case 3:
                        // YCC420
                        printk("tca_fb_get_hdmi_configs-YCC420\r\n");
                        *pxdw = 27;
                        *r2y = 1;
                        *swapbf = 0;
                        break;
                case 0:
                default:
                        printk("tca_fb_get_hdmi_configs-RGB\r\n");
                        // RGB || HDMI_V1.4
                        *pxdw = 12;
                        *r2y = 0;
                        *swapbf = 0;
                        break;
        }
}

void tca_fb_mem_scale_init(void)
{
	pmap_t pmap;

	pmap_get_info("fb_scale0", &pmap);
	fb_scaler_pbuf0 = (char *) pmap.base;
	pmap_get_info("fb_scale1", &pmap);
	fb_scaler_pbuf1 = (char *) pmap.base;

	pmap_get_info("fb_g2d0", &pmap);
	fb_g2d_pbuf0 = (char *) pmap.base;
	pmap_get_info("fb_g2d1", &pmap);
	fb_g2d_pbuf1 = (char *) pmap.base;

	#if defined(CONFIG_TCC_GRAPHIC)
	g2d_ioctl = tccxxx_grp_ioctl;
	g2d_open = tccxxx_grp_open;
	g2d_release = tccxxx_grp_release;
	#else
	g2d_ioctl = NULL;
	g2d_open = NULL;
	g2d_release = NULL;
	#endif

	pr_info("%s scaler buffer:0x%p 0x%p g2d buffer : 0x%p, 0x%p size:%d \n", __func__, fb_scaler_pbuf0, fb_scaler_pbuf1, fb_g2d_pbuf0, fb_g2d_pbuf1, pmap.size);
}


unsigned int tca_get_scaler_num(TCC_OUTPUT_TYPE Output, unsigned int Layer)
{
	unsigned int iSCNum = VIOC_SC3;

	#ifdef CONFIG_ANDROID

	if(Output == TCC_OUTPUT_LCD)
		iSCNum = VIOC_SC3;
	else if(Output == TCC_OUTPUT_HDMI || Output == TCC_OUTPUT_COMPOSITE || Output == TCC_OUTPUT_COMPONENT)
		iSCNum = VIOC_SC1;

	#else
		#ifdef CONFIG_PLATFORM_STB
			if (Layer == RDMA_VIDEO_SUB)
				return VIOC_SC0;
		#else
			return VIOC_SC1;
		#endif//

	#endif//

	return iSCNum;
}

#if 0
void vioc_display_device_reset(unsigned int device_num, struct tcc_dp_device *pDisplayDevice)
{
	#define DD0_DMA_CONNECT_CHECK(PlugInOut) 	((PlugInOut.connect_device == VIOC_SC_RDMA_00)|| (PlugInOut.connect_device == VIOC_SC_RDMA_01) \
													||(PlugInOut.connect_device == VIOC_SC_RDMA_02) || (PlugInOut.connect_device == VIOC_SC_RDMA_03) \
													|| (PlugInOut.connect_device == VIOC_SC_WDMA_00))
													
	#define DD1_DMA_CONNECT_CHECK(PlugInOut) 	((PlugInOut.connect_device == VIOC_SC_RDMA_04)|| (PlugInOut.connect_device == VIOC_SC_RDMA_05) \
													||(PlugInOut.connect_device == VIOC_SC_RDMA_06) || (PlugInOut.connect_device == VIOC_SC_RDMA_07)  \
													|| (PlugInOut.connect_device == VIOC_SC_WDMA_01))

	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();
	volatile VIOC_DISP  *DISPBackup, *pDISPBackup;
	volatile VIOC_WMIX *WMIXBackup, *pWMIX_Addr;

	volatile VIOC_RDMA  *RDMABackup[RDMA_MAX_NUM], *pRDMA_Addr[RDMA_MAX_NUM];

	volatile unsigned char WMIXER_N, RDMA_N[RDMA_MAX_NUM];
	
	VIOC_PlugInOutCheck VIOC_PlugIn[VIOC_VIQE + 1];
	volatile unsigned char VIOC_PlugIn_reset[VIOC_VIQE + 1];
	volatile VIOC_SC SC0_Backup, SC1_Backup, SC2_Backup, SC3_Backup;
	volatile VIOC_SC *pSC0_Addr, *pSC1_Addr, *pSC2_Addr, *pSC3_Addr;

	volatile VIQE ViqeBackup, *pViqe_Addr;
	volatile unsigned char i;

	for(i = 0; i <= VIOC_VIQE; i++)
	{
		VIOC_PlugIn_reset[i] = false;

		VIOC_CONFIG_Device_PlugState((unsigned int)i, &VIOC_PlugIn[i]);

		if(VIOC_PlugIn[i].enable)
		{
			if(device_num) {
				if(DD1_DMA_CONNECT_CHECK(VIOC_PlugIn[i]))
					VIOC_PlugIn_reset[i] = true;
			}
			else	{
				if(DD0_DMA_CONNECT_CHECK(VIOC_PlugIn[i]))
					VIOC_PlugIn_reset[i] = true;
			}
		}
	}

	pDISPBackup = pDisplayDevice->ddc_info.virt_addr;
	
	pWMIX_Addr = pDisplayDevice->wmixer_info.virt_addr;
	
	WMIXER_N = pDisplayDevice->wmixer_info.blk_num;

	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		pRDMA_Addr[i] = pDisplayDevice->rdma_info[i].virt_addr;
		RDMA_N[i] = pDisplayDevice->rdma_info[i].blk_num;
	}

	// backup RDMA_N, WMIXER, DisplayDevice register
	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		RDMABackup[i] = kmalloc(sizeof(VIOC_RDMA), GFP_KERNEL);
		memcpy((void *)RDMABackup[i], (void *)pRDMA_Addr[i], sizeof(VIOC_RDMA));
	}

	DISPBackup = kmalloc(sizeof(VIOC_DISP), GFP_KERNEL);
	memcpy((void *)DISPBackup, (void *)pDISPBackup, sizeof(VIOC_DISP));

	WMIXBackup = kmalloc(sizeof(VIOC_WMIX), GFP_KERNEL);
	memcpy((void *)WMIXBackup, (void *)pWMIX_Addr, sizeof(VIOC_WMIX));

	pSC0_Addr = VIOC_SC_GetAddress(VIOC_SC0);
	pSC1_Addr = VIOC_SC_GetAddress(VIOC_SC1);
	pSC2_Addr = VIOC_SC_GetAddress(VIOC_SC2);
	pSC3_Addr = VIOC_SC_GetAddress(VIOC_SC3);
	pViqe_Addr = VIOC_VIQE_GetAddress();
	
	if(VIOC_PlugIn_reset[VIOC_SC0] && (pSC0_Addr != NULL))
		SC0_Backup = *(VIOC_SC *)pSC0_Addr;

	if(VIOC_PlugIn_reset[VIOC_SC1] && (pSC1_Addr != NULL))
		SC1_Backup = *(VIOC_SC *)pSC1_Addr;

	if(VIOC_PlugIn_reset[VIOC_SC2]&& (pSC2_Addr != NULL))
		SC2_Backup = *(VIOC_SC *)pSC2_Addr;

	if(VIOC_PlugIn_reset[VIOC_SC3] && (pSC3_Addr != NULL))
		SC3_Backup = *(VIOC_SC *)pSC3_Addr;

	if(VIOC_PlugIn_reset[VIOC_VIQE] && (pViqe_Addr != NULL))
		ViqeBackup = *(VIQE *)pViqe_Addr;	
	
	// h/w block reset
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(20+device_num)), (0x1<<(20+device_num))); // disp reset

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(0+device_num)), (0x1<<(0+device_num))); // wdma reset

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(9+WMIXER_N)), (0x1<<(9+WMIXER_N))); // wmix reset

	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA_N[i]), (0x1<<RDMA_N[i])); // rdma reset
	}


	if(VIOC_PlugIn_reset[VIOC_SC0])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC0)), (0x1<<(28+VIOC_SC0))); // scaler 0 reset

	if(VIOC_PlugIn_reset[VIOC_SC1])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC1)), (0x1<<(28+VIOC_SC1))); // scaler 1 reset

	if(VIOC_PlugIn_reset[VIOC_SC2])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC2)), (0x1<<(28+VIOC_SC2))); // scaler 2 reset

	if(VIOC_PlugIn_reset[VIOC_SC3])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC3)), (0x1<<(28+VIOC_SC3))); // scaler 3 reset

	if(VIOC_PlugIn_reset[VIOC_VIQE])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(16)), (0x1<<(16))); // VIQE reset

		
	// h/w block release reset 
	if(VIOC_PlugIn_reset[VIOC_VIQE])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(16)), (0x0<<(16))); // VIQE reset

	if(VIOC_PlugIn_reset[VIOC_SC3])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC3)), (0x0<<(28+VIOC_SC3))); // scaler 3 reset

	if(VIOC_PlugIn_reset[VIOC_SC2])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC2)), (0x0<<(28+VIOC_SC2))); // scaler 2 reset

	if(VIOC_PlugIn_reset[VIOC_SC1])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC1)), (0x0<<(28+VIOC_SC1))); // scaler 1 reset

	if(VIOC_PlugIn_reset[VIOC_SC0])
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+VIOC_SC0)), (0x0<<(28+VIOC_SC0))); // scaler 0 reset


	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA_N[i]), (0x0<<RDMA_N[i])); // rdma reset 
	}
 	
	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(9+WMIXER_N)), (0x0<<(9+WMIXER_N))); // wmix reset release

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(0+device_num)), (0x0<<(0+device_num))); // wdma reset release

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(0+device_num)), (0x0<<(0+device_num))); // wdma reset release

	BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(20+device_num)), (0x0<<(20+device_num))); // disp reset release


	// restore VIQE, SCALER , RDMA_N, WMIXER, DisplayDevice register
	if(VIOC_PlugIn_reset[VIOC_VIQE] && (pViqe_Addr != NULL)) {
		*(VIQE *)pViqe_Addr = ViqeBackup;
	}

	if(VIOC_PlugIn_reset[VIOC_SC3] && (pSC3_Addr != NULL)) {
		*(VIOC_SC *)pSC3_Addr = SC3_Backup;
		VIOC_API_SCALER_SetUpdate(VIOC_SC3); // scaler 3 update
	}

	if(VIOC_PlugIn_reset[VIOC_SC2] && (pSC2_Addr != NULL)) {
		*(VIOC_SC *)pSC2_Addr = SC2_Backup;
		VIOC_API_SCALER_SetUpdate(VIOC_SC2); // scaler 2 update		
	}
	
	if(VIOC_PlugIn_reset[VIOC_SC1] && (pSC1_Addr != NULL)) {
		*(VIOC_SC *)pSC1_Addr= SC1_Backup;
		VIOC_API_SCALER_SetUpdate(VIOC_SC1); // scaler 1 update
	}

	if(VIOC_PlugIn_reset[VIOC_SC0] && (pSC0_Addr != NULL)) {
		*(VIOC_SC *)pSC0_Addr = SC0_Backup;
		VIOC_API_SCALER_SetUpdate(VIOC_SC0); // scaler 0 update
	}
		
	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		memcpy((void *)pRDMA_Addr[i], (void *)RDMABackup[i], sizeof(VIOC_RDMA));

	}

	memcpy((void *)pWMIX_Addr, (void *)WMIXBackup, sizeof(VIOC_WMIX));
 
	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		VIOC_RDMA_SetImageUpdate((VIOC_RDMA *)pRDMA_Addr[i]);
	}
 
	VIOC_WMIX_SetUpdate((VIOC_WMIX *)pWMIX_Addr);

	memcpy((void *)pDISPBackup, (void *)DISPBackup, sizeof(VIOC_DISP));
	kfree((void *)DISPBackup);
	kfree((void *)WMIXBackup);
	for(i = 0; i < RDMA_MAX_NUM; i++)	{
		kfree((void *)RDMABackup[i]);
	}

	printk("VIOC_sc0:%d VIOC_sc1:%d VIOC_sc2:%d ", VIOC_PlugIn_reset[0], VIOC_PlugIn_reset[1], VIOC_PlugIn_reset[2]);
    	printk(" VIOC_sc3:%d VIOC_VIQE:%d  \n", VIOC_PlugIn_reset[3], VIOC_PlugIn_reset[4]);
}
#endif

irqreturn_t tca_main_display_handler(int irq, void *dev_id)
{
	struct tccfb_info *fbdev = dev_id;
	unsigned int dispblock_state;

	if (dev_id == NULL) {
		pr_err("%s irq: %d dev_id:%p \n",__func__, irq, dev_id);
		return IRQ_HANDLED;
	}
	dispblock_state = vioc_intr_get_status(fbdev->pdata.lcdc_number);

	fbdev->vsync_timestamp = ktime_get();

	dprintk("%s  sync_struct.state:0x%x dispblock_state:0x%x, fbdev:%p \n",
				__func__, fbdev->pdata.Mdp_data.pandisp_sync.state, dispblock_state, fbdev);

	if (dispblock_state & (1<<VIOC_DISP_INTR_RU)) {
		vioc_intr_clear(fbdev->pdata.lcdc_number, (1<<VIOC_DISP_INTR_RU));
		if (fbdev->active_vsync)
			schedule_work(&fbdev->vsync_work);
		if (fbdev->pdata.Mdp_data.pandisp_sync.state == 0) {
			fbdev->pdata.Mdp_data.pandisp_sync.state = 1;
			wake_up_interruptible(&fbdev->pdata.Mdp_data.pandisp_sync.waitq);
		}
#if defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION)
		if(VIOC_DISP_Get_TurnOnOff(fbdev->pdata.Mdp_data.ddc_info.virt_addr)) {
			if(dispblock_state & (1<<VIOC_DISP_INTR_FU)) {
//				pr_err("%s FIFO UNDERUN STATUS:0x%x \n",__func__, dispblock_state);
				vioc_intr_clear(fbdev->pdata.lcdc_number, (1<<VIOC_DISP_INTR_FU));
			}
		} else {
			//Don't care FIFO under-run, when DISP is off.
		}
#endif
	}
#if defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION)
	if(dispblock_state & (1<<VIOC_DISP_INTR_DD)) {
		vioc_intr_clear(fbdev->pdata.lcdc_number, (1<<VIOC_DISP_INTR_DD));

		if (fbdev->pdata.Mdp_data.disp_dd_sync.state == 0) {
			fbdev->pdata.Mdp_data.disp_dd_sync.state = 1;
			wake_up_interruptible(&fbdev->pdata.Mdp_data.disp_dd_sync.waitq);
		}

		pr_info("%s DISABEL DONE Lcdc_num:%d 0x%p  STATUS:0x%x  \n",
			__func__,fbdev->pdata.lcdc_number, fbdev->pdata.Mdp_data.ddc_info.virt_addr, dispblock_state);
	}
#endif

	if(dispblock_state & (1<<VIOC_DISP_INTR_SREQ)) {
		vioc_intr_clear(fbdev->pdata.lcdc_number, (1<<VIOC_DISP_INTR_SREQ));
	}
	
	return IRQ_HANDLED;
}

int tca_main_interrupt_reg(char SetClear, struct tccfb_info *info)
{
	int ret = 0;
//	pr_info("%s SetClear:%d lcdc_num:%d %d  INT_VIOC_DEV1:%d \n",__func__, SetClear, info->pdata.lcdc_number, info->pdata.Mdp_data.ddc_info.irq_num, INT_VIOC_DEV1);

	if(SetClear)	{
		vioc_intr_clear(info->pdata.lcdc_number, 0x39);
		ret = request_irq( info->pdata.Mdp_data.ddc_info.irq_num, tca_main_display_handler, IRQF_SHARED, "TCC_MAIN_D", info);

		vioc_intr_enable(info->pdata.lcdc_number, VIOC_DISP_INTR_DISPLAY);
	}
	else {
		vioc_intr_disable(info->pdata.lcdc_number, VIOC_DISP_INTR_DISPLAY);
		free_irq( info->pdata.Mdp_data.ddc_info.irq_num, info);
	}

	return ret;
}
EXPORT_SYMBOL(tca_main_interrupt_reg);

static irqreturn_t tca_sub_display_handler(int irq, void *dev_id)
{
	struct tcc_dp_device *pDisplayInfo = dev_id;
	unsigned int dispblock_state;

	if(dev_id == NULL)	{
		pr_err("%s irq: %d dev_id:%p \n",__func__, irq, dev_id);
		return IRQ_HANDLED;
	}
	
	dispblock_state = vioc_intr_get_status(pDisplayInfo->DispNum);
	if(dispblock_state & VIOC_DISP_IREQ_RU_MASK) {
		vioc_intr_clear(pDisplayInfo->DispNum, (1<<VIOC_DISP_INTR_RU));

		if(pDisplayInfo->pandisp_sync.state == 0)		{
			pDisplayInfo->pandisp_sync.state = 1;
			wake_up_interruptible(&pDisplayInfo->pandisp_sync.waitq);
		}

		#if !(defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION) || defined(TCC_VIDEO_DISPLAY_BY_VSYNC_INT))
		tca_lcdc_interrupt_onoff(false, pDisplayInfo->ddc_info->blk_num);
		#endif

#if defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION)
		if(VIOC_DISP_Get_TurnOnOff(pDisplayInfo->ddc_info.virt_addr)) {
			if(dispblock_state & (1<<VIOC_DISP_INTR_FU)) {
//				pr_err("%s FIFO UNDERUN STATUS:0x%x \n",__func__, dispblock_state);
				vioc_intr_clear(pDisplayInfo->ddc_info.blk_num, (1<<VIOC_DISP_INTR_FU));
			}
		} else {
			//Don't care FIFO under-run, when DISP is off.
		}
#endif
	}

#if defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION)
		if(dispblock_state & (1<<VIOC_DISP_INTR_DD)) {
			vioc_intr_clear(pDisplayInfo->ddc_info.blk_num, (1<<VIOC_DISP_INTR_DD));

			if (pDisplayInfo->disp_dd_sync.state == 0) {
				pDisplayInfo->disp_dd_sync.state = 1;
				wake_up_interruptible(&pDisplayInfo->disp_dd_sync.waitq);
			}

			pr_info("%s DISABEL DONE Lcdc_num:%d 0x%p  STATUS:0x%x \n",
				__func__,pDisplayInfo->ddc_info.blk_num, pDisplayInfo->ddc_info.virt_addr, dispblock_state);
		}
#endif

	return IRQ_HANDLED;
}

void tca_vioc_displayblock_powerOn(struct tcc_dp_device *pDisplayInfo)
{
	int ret = 0;
	int vioc_clock_value = 0;
	struct device_node *np_output = NULL;

	// get and set the value of vioc clock
	if(pDisplayInfo->DispDeviceType == TCC_OUTPUT_HDMI)
	{
		#if defined(CONFIG_ARCH_TCC896X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc896x-hdmi");
		#elif defined(CONFIG_ARCH_TCC897X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc897x-hdmi");
		#else
			np_output = of_find_compatible_node(NULL, NULL, "telechips,dw-hdmi-tx");
		#endif
	}
	else if(pDisplayInfo->DispDeviceType == TCC_OUTPUT_COMPOSITE)
	{
		#if defined(CONFIG_ARCH_TCC896X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc896x-composite");
		#elif defined(CONFIG_ARCH_TCC897X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc897x-composite");
		#else
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc-composite");
		#endif
	}
	else if(pDisplayInfo->DispDeviceType == TCC_OUTPUT_COMPONENT)
	{
		#if defined(CONFIG_ARCH_TCC896X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc896x-component");
		#elif defined(CONFIG_ARCH_TCC897X)
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc897x-component");
		#else
			np_output = of_find_compatible_node(NULL, NULL, "telechips,tcc-component");
		#endif
	}

	if(np_output)
	{
		of_property_read_u32(np_output, "vioc-frequency", &vioc_clock_value);
		clk_set_rate(pDisplayInfo->vioc_clock, vioc_clock_value);
	}
				
	pr_info("%s lcdc:%d device:%d lcdc_clock:%d\n", __func__, pDisplayInfo->ddc_info.blk_num, pDisplayInfo->DispDeviceType, vioc_clock_value);

	clk_prepare_enable(pDisplayInfo->vioc_clock);
	clk_prepare_enable(pDisplayInfo->ddc_clock);

	if(pDisplayInfo->DispOrder != DD_MAIN)	{
		ret = request_irq(pDisplayInfo->ddc_info.irq_num, tca_sub_display_handler, IRQF_SHARED, 
					"SUB_LCDC", pDisplayInfo);
		if(ret < 0)
			pr_err("Error %s:%d  ret:%d \n", __func__,__LINE__ ,ret);
	}
}

void tca_vioc_displayblock_powerOff(struct tcc_dp_device *pDisplayInfo)
{
	pr_info("%s lcdc:%d\n", __func__, pDisplayInfo->ddc_info.blk_num);

	if(pDisplayInfo->DispOrder != DD_MAIN)
		free_irq(pDisplayInfo->ddc_info.irq_num, pDisplayInfo);

	clk_disable_unprepare(pDisplayInfo->ddc_clock);
	clk_disable_unprepare(pDisplayInfo->vioc_clock);

	if(g2d_filp.private_data != NULL) {
		if (g2d_release != NULL) {
			g2d_release((struct inode *)&g2d_inode, (struct file *)&g2d_filp);
		}
	}
	g2d_filp.private_data = NULL;

	if(scaler_filp)
		filp_close(scaler_filp, 0);
	scaler_filp = NULL;
}

void tca_vioc_displayblock_disable(struct tcc_dp_device *pDisplayInfo)
{
	pDisplayInfo->FbPowerState = false;

	pr_info("%s lcdc:%d onoff:%d\n", __func__, pDisplayInfo->ddc_info.blk_num, VIOC_DISP_Get_TurnOnOff(pDisplayInfo->ddc_info.virt_addr));

	// Disable Display R2Y.
	VIOC_DISP_SetR2Y(pDisplayInfo->ddc_info.virt_addr, 0);
        
#ifdef CONFIG_VIDEO_TCC_VOUT
	VIOC_RDMA_SetImageDisableNW(pDisplayInfo->rdma_info[RDMA_FB].virt_addr);
	VIOC_RDMA_SetImageDisableNW(pDisplayInfo->rdma_info[RDMA_3D].virt_addr);
	if(tcc_vout_get_status() != TCC_VOUT_RUNNING) {
		VIOC_RDMA_SetImageDisableNW(pDisplayInfo->rdma_info[RDMA_VIDEO_3D].virt_addr);
		VIOC_RDMA_SetImageDisableNW(pDisplayInfo->rdma_info[RDMA_VIDEO].virt_addr);
	}
#else
	{
		int i = 0;
		for(i = 0; i < RDMA_MAX_NUM; i ++)
			VIOC_RDMA_SetImageDisableNW(pDisplayInfo->rdma_info[i].virt_addr);
	}
#endif

	if(VIOC_DISP_Get_TurnOnOff(pDisplayInfo->ddc_info.virt_addr)) {
		VIOC_DISP_TurnOff(pDisplayInfo->ddc_info.virt_addr);
		VIOC_DISP_sleep_DisplayDone(pDisplayInfo->ddc_info.virt_addr);
	}

	VIOC_DISP_SWReset(pDisplayInfo->ddc_info.blk_num);

	if(pDisplayInfo->FbUpdateType == FB_MVC_UPDATE) {
		printk("%s : pDisplayInfo->FbUpdateType = %d\n", __func__, pDisplayInfo->FbUpdateType);

		if(tca_get_RDMA_PlugIn_to_Scaler(pDisplayInfo->rdma_info[RDMA_3D].blk_num) != -1) //scaler plug in status check
		{
			if(VIOC_CONFIG_PlugOut(VIOC_SC3) == VIOC_PATH_DISCONNECTED) {
				VIOC_SC_SetSWReset(VIOC_SC3, pDisplayInfo->rdma_info[RDMA_3D].blk_num, 0xFF);
				printk("%s : Scale3 is reset!!!\n", __func__);
			}
		}

		if(tca_get_RDMA_PlugIn_to_Scaler(pDisplayInfo->rdma_info[RDMA_FB].blk_num) != -1) //scaler plug in status check
		{
			if(VIOC_CONFIG_PlugOut(AnD_FB_SC) == VIOC_PATH_DISCONNECTED) {
				VIOC_SC_SetSWReset(AnD_FB_SC, pDisplayInfo->rdma_info[RDMA_FB].blk_num, 0xFF);
				printk("%s : Scaler %d is reset!!!\n", __func__, AnD_FB_SC);
			}
		}
	}

	#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)|| defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
	#else
	VIOC_WMIX_SetSWReset(pDisplayInfo->wmixer_info.blk_num, pDisplayInfo->rdma_info[RDMA_FB].blk_num, pDisplayInfo->wdma_info.blk_num);
	VIOC_WMIX_SetSWReset(pDisplayInfo->wmixer_info.blk_num, pDisplayInfo->rdma_info[RDMA_3D].blk_num, pDisplayInfo->wdma_info.blk_num);
	#ifdef CONFIG_VIDEO_TCC_VOUT
	if(tcc_vout_get_status() != TCC_VOUT_RUNNING) 
	{
		VIOC_WMIX_SetSWReset(pDisplayInfo->wmixer_info.blk_num, pDisplayInfo->rdma_info[RDMA_VIDEO_3D].blk_num, pDisplayInfo->wdma_info.blk_num);
		VIOC_WMIX_SetSWReset(pDisplayInfo->wmixer_info.blk_num, pDisplayInfo->rdma_info[RDMA_VIDEO].blk_num, pDisplayInfo->wdma_info.blk_num);
	}
	#endif
	#endif

}

#ifdef DISPLAY_TIMING_DUMP
static void tca_vioc_displaytiming_dump(struct tcc_dp_device *pDisplayInfo) {
        VIOC_DISP *pDISP = (VIOC_DISP *)pDisplayInfo->ddc_info.virt_addr;

        volatile unsigned int reg = pDISP->uCTRL.nREG;
        volatile unsigned int val, val1, val2, val3, val4;
        printk("FB Timing Dump\r\n");
        printk(" SYNC VSYNC(%s), HSYNC(%s), DE(%s) PCLK(%s)\r\n", 
                (reg & (1<<13))?"Low":"High",  (reg & (1<<12))?"Low":"High", (reg & (1<<14))?"Low":"High", (reg & (1<<1))?"Low":"High");

        // LPC
        val = pDISP->uLHTIME1.nREG;
        val &= 0x00003FFF;
        val += 1;

        // LPW - HSYNC WIDTH
        val1 = pDISP->uLHTIME1.nREG;
        val1 &= 0x01FF0000;
        val1 >>= 16;
        val1 += 1;

        // LSWC
        val2 = pDISP->uLHTIME2.nREG;
        val2 &= 0x01FF0000;
        val2 >>=16;
        val2 += 1;              

        //LEWC - HSYNC EDGE DELAY
        val3 = pDISP->uLHTIME2.nREG;
        val3 &= 0x000001FF;
        val3 += 1;
        
        val4 = val1 + val2 +val3;

        printk("HSYNC ACTIVE(%d), DELAY(%d), SYNC(%d), BLANK(%d)\r\n", val,  val3, val1,  val4);


        // FLC
        val = pDISP->uLVTIME1.nREG;
        val &= 0x00003FFF;
        val += 1;

        // FPW - VSYNC WIDTH
        val1 = pDISP->uLVTIME1.nREG;
        val1 &= 0x3F0000;
        val1 >>= 16;
        val1 += 1;

        // FSWC
        val2 = pDISP->uLVTIME2.nREG;
        val2 &= 0x01FF0000;
        val2 >>=16;
        val2 += 1;


        //FEWC - VSYNC EDGE DELAY
        val3 = pDISP->uLVTIME2.nREG;
        val3 &= 0x000001FF;
        val3 += 1;

        val4 = val1 + val2 +val3;

	
        printk("VSYNC ACTIVE(%d), DELAY(%d), SYNC(%d), BLANK(%d)\r\n", val,  val3, val1,  val4);
}
#endif

void tca_vioc_displayblock_timing_set(unsigned int outDevice, struct tcc_dp_device *pDisplayInfo, struct lcdc_timimg_parms_t *mode)
{
	unsigned int width, height;
	VIOC_DISP *pDISP = pDisplayInfo->ddc_info.virt_addr;
	VIOC_WMIX * pWMIX = pDisplayInfo->wmixer_info.virt_addr;	
	stLCDCTR stCtrlParam;
	stLTIMING stTimingParam;
	unsigned int rdma_en = 0;
	#if defined(CONFIG_ARCH_TCC898X)
	unsigned int swapbf = 0;
	#endif

	if(mode->dp) {
		width = (mode->lpc + 1)/2;
		height = mode->flc + 1;
	}
	else	{
		width = (mode->lpc + 1);
		height = mode->flc + 1;
	}

	if(pDisplayInfo->DispDeviceType == TCC_OUTPUT_COMPOSITE)
		width = width/2;

	stTimingParam.lpw= mode->lpw;
	stTimingParam.lpc= mode->lpc + 1;
	stTimingParam.lswc= mode->lswc+ 1;
	stTimingParam.lewc= mode->lewc+ 1;
	
	stTimingParam.vdb = mode->vdb;
	stTimingParam.vdf = mode->vdf;
	stTimingParam.fpw = mode->fpw;
	stTimingParam.flc = mode->flc;
	stTimingParam.fswc = mode->fswc;
	stTimingParam.fewc = mode->fewc;
	stTimingParam.fpw2 = mode->fpw2;
	stTimingParam.flc2 = mode->flc2;
	stTimingParam.fswc2 = mode->fswc2;
	stTimingParam.fewc2 = mode->fewc2;
	
	VIOC_DISP_SWReset( pDisplayInfo->ddc_info.blk_num);
	VIOC_DISP_SetTimingParam(pDISP, &stTimingParam);

	memset(&stCtrlParam, 0x00, sizeof(stCtrlParam));
	stCtrlParam.id= mode->id;
	stCtrlParam.iv= mode->iv;
	stCtrlParam.ih= mode->ih;
	stCtrlParam.ip= mode->ip;
	stCtrlParam.clen = 0;
	stCtrlParam.r2y = 0;
	//stCtrlParam.pxdw = 12; //RGB888
	stCtrlParam.dp = mode->dp;
	stCtrlParam.ni = mode->ni;
        
#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
    if(mode->ni == 0)
            stCtrlParam.advi = 0;
#else
    #if defined(CONFIG_ARCH_TCC898X)
    stCtrlParam.advi = mode->ni;
    #else
    if(mode->ni == 0)
            stCtrlParam.advi = 1;
    #endif
#endif
	stCtrlParam.tv = mode->tv;

	#if 0//defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) : pjj delete please
		if(output_starter_state || hdmi_get_hdmimode() == DVI)
			stCtrlParam.pxdw = 12; //RGB888
		else
			stCtrlParam.pxdw = 8; //YCbCr
	#else
		stCtrlParam.pxdw = 12; //RGB888
	#endif

    #if defined(CONFIG_ARCH_TCC898X)
    if(outDevice == VIOC_OUTCFG_HDMI) {
            tca_fb_get_hdmi_configs(mode->format, &stCtrlParam.pxdw, &stCtrlParam.r2y, &swapbf);
    }
    #endif
	
	VIOC_DISP_SetControlConfigure(pDISP, &stCtrlParam);
    #if defined(CONFIG_ARCH_TCC898X)
    VIOC_DISP_SetSwapbf(pDISP, swapbf);
    #endif
	VIOC_DISP_SetSize (pDISP, width, height);
	VIOC_DISP_SetBGColor(pDISP, 0, 0 , 0);

	#if 0//defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) : pjj delete please
		if(output_starter_state || hdmi_get_hdmimode() == DVI)
			VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0xff);
		else
			VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x80, 0x80, 0x00);
	#else
		#if defined(CONFIG_ARCH_TCC898X)
		VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0x3ff);
		#else
		VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0xff);
		#endif
	#endif

        #if defined(CONFIG_TCC_REAR_CAMERA_DRV)
        printk(" \r\n\r\n%s (%d) SKIP OVP\r\n", __func__, __LINE__);
        #else
	VIOC_WMIX_SetOverlayPriority(pWMIX, 24);
        #endif
	
	VIOC_WMIX_SetSize(pWMIX, width, height);
	VIOC_WMIX_SetUpdate (pWMIX);

	VIOC_OUTCFG_SetOutConfig(outDevice, pDisplayInfo->ddc_info.blk_num);

	VIOC_RDMA_GetImageEnable(pDisplayInfo->rdma_info[RDMA_FB].virt_addr, &rdma_en);
	if(rdma_en)
		VIOC_RDMA_SetImageDisable(pDisplayInfo->rdma_info[RDMA_FB].virt_addr);

	#if	defined(CONFIG_HDMI_FB_ROTATE_90)||defined(CONFIG_HDMI_FB_ROTATE_180)||defined(CONFIG_HDMI_FB_ROTATE_270)
		pDisplayInfo->FbUpdateType = FB_SC_G2D_RDMA_UPDATE;
	#else
		#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
			pDisplayInfo->FbUpdateType = FB_SC_M2M_RDMA_UPDATE; /* YUV output */
		#else
			pDisplayInfo->FbUpdateType = FB_SC_RDMA_UPDATE;
		#endif
	#endif

	printk("mode->framepacking = %d\n", mode->framepacking);

	if(mode->framepacking == 1) {
		//pDisplayInfo->MVCMode = 1;
		pDisplayInfo->FbUpdateType = FB_MVC_UPDATE;
	}

// patch for stb presentation window.
	if(pDisplayInfo->FbUpdateType == FB_SC_RDMA_UPDATE) {
		pDisplayInfo->sc_num0 = AnD_FB_SC;
	}
	else if(pDisplayInfo->FbUpdateType == FB_MVC_UPDATE) {
		pDisplayInfo->sc_num0 = AnD_FB_SC;
		pDisplayInfo->sc_num1 = VIOC_SC3;
	}

	if(VIOC_CONFIG_PlugOut(pDisplayInfo->sc_num0) != VIOC_PATH_DISCONNECTED)
		VIOC_SC_SetSWReset(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num, 0xFF);

	if(pDisplayInfo->FbUpdateType == FB_SC_RDMA_UPDATE)	{
		VIOC_CONFIG_PlugIn(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num);
	}
	else  if(pDisplayInfo->FbUpdateType == FB_SC_M2M_RDMA_UPDATE) {
		if (!scaler_filp) {
			scaler_filp = filp_open(DEV_SCALER1, O_RDWR, 0666);
			if (IS_ERR(scaler_filp))
				scaler_filp = NULL;
		}
	}
	else if(pDisplayInfo->FbUpdateType == FB_SC_G2D_RDMA_UPDATE)	{
		if (g2d_open != NULL) {
			g2d_open((struct inode *)&g2d_inode, (struct file *)&g2d_filp);
		}
		if (!scaler_filp) {
			scaler_filp = filp_open(DEV_SCALER1, O_RDWR, 0666);
			if (IS_ERR(scaler_filp))
				scaler_filp = NULL;
		}
	}
	else if(pDisplayInfo->FbUpdateType == FB_MVC_UPDATE) {
		if(VIOC_CONFIG_PlugOut(pDisplayInfo->sc_num0) != VIOC_PATH_DISCONNECTED)
			VIOC_SC_SetSWReset(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num, 0xFF);

		if(VIOC_CONFIG_PlugOut(pDisplayInfo->sc_num1) != VIOC_PATH_DISCONNECTED)
			VIOC_SC_SetSWReset(pDisplayInfo->sc_num1, pDisplayInfo->rdma_info[RDMA_3D].blk_num, 0xFF);

		VIOC_CONFIG_PlugIn(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num);
		VIOC_CONFIG_PlugIn(pDisplayInfo->sc_num1, pDisplayInfo->rdma_info[RDMA_3D].blk_num);
	}
		
	pDisplayInfo->FbPowerState = true;
		
	vioc_intr_enable(pDisplayInfo->DispNum, VIOC_DISP_INTR_DISPLAY);

	#ifdef DISPLAY_TIMING_DUMP
	tca_vioc_displaytiming_dump(pDisplayInfo);
	#endif

	pr_info("%s displayN:%d non-interlaced:%d w:%d h:%d FbUpdateType:%d \n", __func__, pDisplayInfo->ddc_info.blk_num, mode->ni, width, height, pDisplayInfo->FbUpdateType ); 			
}

void tca_vioc_displayblock_ctrl_set(unsigned int outDevice, struct tcc_dp_device *pDisplayInfo, stLTIMING *pstTiming, stLCDCTR *pstCtrl, unsigned int format)
{
	unsigned int width, height;
	VIOC_DISP *pDISP = pDisplayInfo->ddc_info.virt_addr;
	VIOC_WMIX * pWMIX = pDisplayInfo->wmixer_info.virt_addr;	
	unsigned int rdma_en = 0;
	unsigned int swapbf = 0;
	
	if(pstCtrl->dp) {
		width = pstTiming->lpc/2;
		height = pstTiming->flc + 1;
	}
	else	{
		width = pstTiming->lpc;
		height = pstTiming->flc + 1;
	}

	if(pDisplayInfo->DispDeviceType == TCC_OUTPUT_COMPOSITE)
		width = width/2;

	VIOC_DISP_SWReset(pDisplayInfo->ddc_info.blk_num);
	VIOC_DISP_SetTimingParam(pDISP, pstTiming);

        if(outDevice == VIOC_OUTCFG_HDMI) {
                tca_fb_get_hdmi_configs(format, &pstCtrl->pxdw, &pstCtrl->r2y, &swapbf);
        }
        
	VIOC_DISP_SetControlConfigure(pDISP, pstCtrl);
        #if defined(CONFIG_ARCH_TCC898X)
        VIOC_DISP_SetSwapbf(pDISP, swapbf);
        #endif
	VIOC_DISP_SetSize(pDISP, width, height);
	VIOC_DISP_SetBGColor(pDISP, 0, 0 , 0);

	#if 0//defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) : pjj delete please
		if(output_starter_state || hdmi_get_hdmimode() == DVI)
			VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0xff);
		else
			VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x80, 0x80, 0x00);
	#else
		#if defined(CONFIG_ARCH_TCC898X)
		VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0x3ff);
		#else
		VIOC_WMIX_SetBGColor(pWMIX, 0x00, 0x00, 0x00, 0xff);
		#endif
	#endif

	#if defined(CONFIG_TCC_VIOCMG)
	viocmg_set_wmix_ovp(VIOCMG_CALLERID_FB, pDisplayInfo->wmixer_info.blk_num, viocmg_get_main_display_ovp());
	#else
	VIOC_WMIX_SetOverlayPriority(pWMIX, 24);
	#endif
	
	VIOC_WMIX_SetSize(pWMIX, width, height);
	VIOC_WMIX_SetUpdate (pWMIX);

	VIOC_OUTCFG_SetOutConfig(outDevice, pDisplayInfo->ddc_info.blk_num);

	VIOC_RDMA_GetImageEnable(pDisplayInfo->rdma_info[RDMA_FB].virt_addr, &rdma_en);
	if(rdma_en)
		VIOC_RDMA_SetImageDisable(pDisplayInfo->rdma_info[RDMA_FB].virt_addr);

	if(pDisplayInfo->FbUpdateType != FB_ATTACH_UPDATE)
	{
		#if	defined(CONFIG_HDMI_FB_ROTATE_90)||defined(CONFIG_HDMI_FB_ROTATE_180)||defined(CONFIG_HDMI_FB_ROTATE_270)
			pDisplayInfo->FbUpdateType = FB_SC_G2D_RDMA_UPDATE;
		#else
			#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
				pDisplayInfo->FbUpdateType = FB_SC_M2M_RDMA_UPDATE; /* YUV output */
			#else
				if(pDisplayInfo->FbUpdateType != FB_RDMA_UPDATE)
					pDisplayInfo->FbUpdateType = FB_SC_RDMA_UPDATE;
			#endif
		#endif
		// patch for stb presentation window.
		if(pDisplayInfo->FbUpdateType == FB_SC_RDMA_UPDATE) {
			if(pDisplayInfo->DispOrder == 0)
				pDisplayInfo->sc_num0 = AnD_FB_SC;
			#ifdef CONFIG_COMPONENT_PRESENTATION_WINDOW_DISPLAY_RESIZE
			else if(pDisplayInfo->DispOrder == 1)
				pDisplayInfo->sc_num0 = VIOC_SC4;
			#endif//
		}
		else if(pDisplayInfo->FbUpdateType == FB_MVC_UPDATE) {
			pDisplayInfo->sc_num0 = AnD_FB_SC;
			pDisplayInfo->sc_num1 = VIOC_SC3;
		}
			if(pDisplayInfo->FbUpdateType == FB_SC_RDMA_UPDATE) {
				if(VIOC_CONFIG_PlugOut(pDisplayInfo->sc_num0) != VIOC_PATH_DISCONNECTED)
					VIOC_SC_SetSWReset(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num, 0xFF);

				VIOC_CONFIG_PlugIn(pDisplayInfo->sc_num0, pDisplayInfo->rdma_info[RDMA_FB].blk_num);
			}
		else  if(pDisplayInfo->FbUpdateType == FB_SC_M2M_RDMA_UPDATE) {
			if (!scaler_filp) {
				scaler_filp = filp_open(DEV_SCALER1, O_RDWR, 0666);
				if (IS_ERR(scaler_filp))
					scaler_filp = NULL;
			}
		}
		else if(pDisplayInfo->FbUpdateType == FB_SC_G2D_RDMA_UPDATE)	{
			if (g2d_open != NULL) {
				g2d_open((struct inode *)&g2d_inode, (struct file *)&g2d_filp);
			}
			if (!scaler_filp) {
				scaler_filp = filp_open(DEV_SCALER1, O_RDWR, 0666);
				if (IS_ERR(scaler_filp))
					scaler_filp = NULL;
			}
		}
	}

	pDisplayInfo->FbPowerState = true;

	vioc_intr_enable(pDisplayInfo->DispNum, VIOC_DISP_INTR_DISPLAY);

	pr_info("%s displayN:%d non-interlaced:%d w:%d h:%d FbUpdateType:%d \n", __func__, pDisplayInfo->ddc_info.blk_num, pstCtrl->ni, width, height, pDisplayInfo->FbUpdateType); 			
}

void tca_fb_rdma_active_var(unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	unsigned int  fmt = 0;
	unsigned int lcd_width, lcd_height, img_width, img_height;

	unsigned int alpha_type = 0, alpha_blending_en = 0;
	unsigned int chromaR, chromaG, chromaB, chroma_en = 0;
	unsigned int lcd_pos_x = 0, lcd_pos_y = 0, lcd_layer = RDMA_FB;

	PVIOC_RDMA pRDMA = (PVIOC_RDMA)pdp_data->rdma_info[RDMA_FB].virt_addr;
	PVIOC_WMIX pWMIX = (PVIOC_WMIX)pdp_data->wmixer_info.virt_addr;
	
	if(var->bits_per_pixel == 32) 	{
		chroma_en = UI_CHROMA_EN;
		alpha_type = 1;
		alpha_blending_en = 1;
		fmt = TCC_LCDC_IMG_FMT_RGB888;
	}
	else if(var->bits_per_pixel == 16)	{
		chroma_en = 1;
		alpha_type = 0;
		alpha_blending_en = 0;
		fmt = TCC_LCDC_IMG_FMT_RGB565; 
	}
	else	{
		pr_err("%s: bits_per_pixel : %d Not Supported BPP!\n", __FUNCTION__, var->bits_per_pixel);
	}

	chromaR = chromaG = chromaB = 0;

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);

	//pr_info("%s: %d base_addr:0x%x w:%d h:%d offset:%d lcd:%d - %d \n", __FUNCTION__, pdp_data->ddc_info.blk_num, base_addr, var->xres, var->yres, var->yoffset, lcd_width, lcd_height);

	img_width = var->xres;
	img_height = var->yres;

	if(img_width > lcd_width)	
		img_width = lcd_width;
	
	if(img_height > lcd_height)	
		img_height = lcd_height;

	/* display 3d ui : side-by-side, top-and-bottom */
	if(divide_data.enable)
	{
		if(divide_data.count == 0)
		{
			pRDMA		= (PVIOC_RDMA)pdp_data->rdma_info[RDMA_FB].virt_addr;
			lcd_layer	= RDMA_FB;
		}
		else
		{
			pRDMA		= (PVIOC_RDMA)pdp_data->rdma_info[RDMA_3D].virt_addr;
			lcd_layer	= RDMA_3D;
		}

		/* side-by-side : left/right */
		if(divide_data.mode == 0)
		{
			lcd_width	= lcd_width/2;

			if(divide_data.count == 0)
				lcd_pos_x	= lcd_pos_x/2;
			else
				lcd_pos_x	= lcd_pos_x/2 + lcd_width;
		}
		/* top-and-bottom : top/bottom */
		else
		{
			lcd_height	= lcd_height/2;

			if(divide_data.count == 0)
				lcd_pos_y	= lcd_pos_y/2;
			else
				lcd_pos_y	= lcd_pos_y/2 + lcd_height;
		}
	}

	// default framebuffer 
	VIOC_WMIX_SetPosition(pWMIX, lcd_layer, lcd_pos_x, lcd_pos_y);
	//overlay setting
	#ifdef CONFIG_ARCH_TCC898X
	VIOC_WMIX_SetChromaKey(pWMIX, lcd_layer, chroma_en, chromaR, chromaG, chromaB, 0x3FF, 0x3FF, 0x3FF);
	#else
	VIOC_WMIX_SetChromaKey(pWMIX, lcd_layer, chroma_en, chromaR, chromaG, chromaB, 0xF8, 0xFC, 0xF8);
	#endif//

	#if defined(CONFIG_SUPPORT_2D_COMPRESSION)
		if(CONFIG_ADDITIONAL_TS_BUFFER_LINES != 0 && var->reserved[2] != 0 )	{
			//var->reserved[2] = //Compressed mode...
			//var->reserved[3] = //1 if it has alpha, otherwise 0...
			//#define ALIGNED(len, mul) ( ( (unsigned int)len + (mul-1) ) & ~(mul-1) )
			unsigned int *remap_addr;
			unsigned int ts_address = base_addr +  (var->xres * var->yres * (var->bits_per_pixel/8));

			remap_addr = (unsigned int *)ioremap_nocache(ts_address, 4096);
			if( remap_addr != NULL && remap_addr[0] != 0xffffffff ){
				VIOC_RDMA_DEC_CTRL(pRDMA, ts_address, (var->xres*var->yres/64)/2, var->reserved[3], 0xf);
				VIOC_RDMA_DEC_EN(pRDMA, 1); // decompress mode : 1, bypass mode : 0
				//printk("======> FBC for DEC100 - 0x%x/0x%x - %d \n", base_addr, ts_address, var->reserved[3]);
			}
			else{
				VIOC_RDMA_DEC_EN(pRDMA, 0);
			}
			iounmap((void*)remap_addr);
		}
	#endif
	
	VIOC_RDMA_SetImageFormat(pRDMA, fmt);					//fmt

	VIOC_RDMA_SetImageOffset(pRDMA, fmt, img_width);		//offset	
	VIOC_RDMA_SetImageSize(pRDMA, img_width, img_height);	//size	
	VIOC_RDMA_SetImageBase(pRDMA, base_addr, 0, 0);

	VIOC_RDMA_SetImageAlphaSelect(pRDMA, alpha_type);
	VIOC_RDMA_SetImageAlphaEnable(pRDMA, alpha_blending_en);

	VIOC_WMIX_SetUpdate(pWMIX);	
	VIOC_RDMA_SetImageEnable(pRDMA);


//temp pjj because fifo under have to enable rdma before display device on.
	if(pdp_data->DispOrder == DD_MAIN)
		VIOC_DISP_TurnOn(pdp_data->ddc_info.virt_addr);
}

void tca_fb_sc_rdma_active_var(unsigned int base_addr, struct fb_var_screeninfo *var,struct tcc_dp_device *pdp_data)
{
	unsigned int  fmt = 0,  interlace_output = 0;
	unsigned int lcd_width, lcd_height, img_width, img_height;
	unsigned int lcd_pos_x = 0, lcd_pos_y = 0, lcd_layer = RDMA_FB;

	unsigned int alpha_type = 0, alpha_blending_en = 0;
	unsigned int chromaR, chromaG, chromaB, chroma_en = 0;
  	tcc_display_resize *pResize= &resize_data;
	PVIOC_SC pSC;

	PVIOC_RDMA pRDMA = (PVIOC_RDMA)pdp_data->rdma_info[RDMA_FB].virt_addr;
	PVIOC_WMIX pWMIX = (PVIOC_WMIX)pdp_data->wmixer_info.virt_addr;
	
	if(var->bits_per_pixel == 32) 	{
		chroma_en = UI_CHROMA_EN;
		alpha_type = 1;
		alpha_blending_en = 1;
		fmt = TCC_LCDC_IMG_FMT_RGB888;
	}
	else if(var->bits_per_pixel == 16)	{
		chroma_en = 1;
		alpha_type = 0;
		alpha_blending_en = 0;
		fmt = TCC_LCDC_IMG_FMT_RGB565; 
	}
	else	{
		pr_err("%s: bits_per_pixel : %d Not Supported BPP!\n", __FUNCTION__, var->bits_per_pixel);
	}
	chromaR = chromaG = chromaB = 0;

	img_width = var->xres;
	img_height = var->yres;

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);

	#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	if((pdp_data->DispOrder == DD_SUB) && ((pdp_data->DispDeviceType== TCC_OUTPUT_COMPONENT) || (pdp_data->DispDeviceType== TCC_OUTPUT_COMPOSITE)))
	  	pResize = &secondary_display_resize_data;
	else
		pResize = &resize_data;
	#endif//CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB

	
	//pr_info("%s: display order:%d block:%d base_addr:0x%x w:%d h:%d offset:%d lcd:%d - %d \n", __FUNCTION__, pdp_data->DispOrder, pdp_data->ddc_info.blk_num, base_addr, var->xres, var->yres, var->yoffset, lcd_width, lcd_height);	
	//pr_info("scaler num %d  l:%d r : %d u:%d d:%d \n", pdp_data->sc_num0, pResize->resize_left , pResize->resize_right, pResize->resize_up , pResize->resize_down);	

	if((!lcd_height) || (!lcd_width))	
		return;

	lcd_width	-= (pResize->resize_left + pResize->resize_right) * 8;
	lcd_height	-= (pResize->resize_up + pResize->resize_down) * 4;
	lcd_pos_x	= pResize->resize_left * 8;
	lcd_pos_y	= pResize->resize_up * 4;

	/* display 3d ui : side-by-side, top-and-bottom */
	if(divide_data.enable)
	{
		if(divide_data.count == 0)
		{
			pSC			= (VIOC_SC *)VIOC_SC_GetAddress( pdp_data->sc_num0);
			pRDMA		= (PVIOC_RDMA)pdp_data->rdma_info[RDMA_FB].virt_addr;
			lcd_layer	= RDMA_FB;
		}
		else
		{
			pSC			= (VIOC_SC *)VIOC_SC_GetAddress( pdp_data->sc_num1);
			pRDMA		= (PVIOC_RDMA)pdp_data->rdma_info[RDMA_3D].virt_addr;
			lcd_layer	= RDMA_3D;
		}

		/* side-by-side : left/right */
		if(divide_data.mode == 0)
		{
			lcd_width	= lcd_width/2;

			if(divide_data.count == 0)
				lcd_pos_x	= lcd_pos_x/2;
			else
				lcd_pos_x	= lcd_pos_x/2 + lcd_width;
		}
		/* top-and-bottom : top/bottom */
		else
		{
			lcd_height	= lcd_height/2;

			if(divide_data.count == 0)
				lcd_pos_y	= lcd_pos_y/2;
			else
				lcd_pos_y	= lcd_pos_y/2 + lcd_height;
		}

		VIOC_RDMA_SetImageY2RMode(pRDMA, 0); /* Y2RMode Default 0 (Studio Color) */
	}
	else
	{
		pSC = (PVIOC_SC)VIOC_SC_GetAddress( pdp_data->sc_num0);
	}

	if (pSC != NULL) {
		#if 0
		if((lcd_height == img_height) && (lcd_width == img_width))
			VIOC_SC_SetBypass (pSC, ON);
		else
		#endif
			VIOC_SC_SetBypass (pSC, OFF);

		VIOC_SC_SetSrcSize(pSC, img_width, img_height);
		VIOC_SC_SetDstSize (pSC, lcd_width, lcd_height);			// set destination size in scaler
		VIOC_SC_SetOutSize (pSC, lcd_width, lcd_height);			// set output size in scaer
	}

	// default framebuffer 
	VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, lcd_layer, lcd_pos_x, lcd_pos_y);

	VIOC_RDMA_SetImageIntl(pRDMA, interlace_output);

	VIOC_RDMA_SetImageFormat(pRDMA, fmt );				//fmt

	VIOC_RDMA_SetImageOffset(pRDMA, fmt, img_width  );		//offset	
	VIOC_RDMA_SetImageSize(pRDMA, img_width, img_height  );	//size	
	VIOC_RDMA_SetImageBase(pRDMA, base_addr, 0, 0);

	#ifdef CONFIG_ARCH_TCC898X
	VIOC_WMIX_SetChromaKey(pWMIX, lcd_layer, chroma_en, chromaR, chromaG, chromaB, 0x3FF, 0x3FF, 0x3FF);
	#else
	VIOC_WMIX_SetChromaKey(pWMIX, lcd_layer, chroma_en, chromaR, chromaG, chromaB, 0xF8, 0xFC, 0xF8);
	#endif//

	VIOC_RDMA_SetImageAlphaSelect(pRDMA, alpha_type);
	VIOC_RDMA_SetImageAlphaEnable(pRDMA, alpha_blending_en);

	#if defined(CONFIG_SUPPORT_2D_COMPRESSION)
		if(CONFIG_ADDITIONAL_TS_BUFFER_LINES != 0 && var->reserved[2] != 0 )	{
			//var->reserved[2] = //Compressed mode...
			//var->reserved[3] = //1 if it has alpha, otherwise 0...
			//#define ALIGNED(len, mul) ( ( (unsigned int)len + (mul-1) ) & ~(mul-1) )
			unsigned int *remap_addr;
			unsigned int ts_address = base_addr +  (var->xres * var->yres * (var->bits_per_pixel/8));

			remap_addr = (unsigned int *)ioremap_nocache(ts_address, 4096);
			if( remap_addr != NULL && remap_addr[0] != 0xffffffff ){
				VIOC_RDMA_DEC_CTRL(pRDMA, ts_address, (var->xres*var->yres/64)/2, var->reserved[3], 0xf);
				VIOC_RDMA_DEC_EN(pRDMA, 1); // decompress mode : 1, bypass mode : 0
				//printk("======> FBC for DEC100 - 0x%x/0x%x - %d \n", base_addr, ts_address, var->reserved[3]);
			}
			else{
				VIOC_RDMA_DEC_EN(pRDMA, 0);
			}
			iounmap((void*)remap_addr);
		}
	#endif

	if (pSC != NULL) {
		VIOC_SC_SetUpdate (pSC);	// Scaler update
	}
	VIOC_WMIX_SetUpdate(pWMIX);	
	VIOC_RDMA_SetImageEnable(pRDMA);
}

unsigned int tca_fb_sc_m2m(unsigned int base_addr, struct fb_var_screeninfo *var, unsigned int dest_x, unsigned int dest_y)
{
	SCALER_TYPE fbscaler;
	static char sc_buf_index = 0;
	
//	pr_info("%s: base_addr:0x%x w:%d h:%d offset:%d lcd:%d - %d \n", __FUNCTION__, base_addr, var->xres, var->yres, var->yoffset, dest_x, dest_y);	

	memset(&fbscaler, 0x00, sizeof(SCALER_TYPE));
	fbscaler.responsetype = SCALER_POLLING;
	fbscaler.viqe_onthefly = 0;

	fbscaler.src_Yaddr = (char *)base_addr;

	if(var->bits_per_pixel == 32)
		fbscaler.src_fmt = SCALER_ARGB8888;
	else
		fbscaler.src_fmt = SCALER_RGB565;

	fbscaler.src_ImgWidth = var->xres;
	fbscaler.src_ImgHeight = var->yres;

	fbscaler.src_winLeft = 0;
	fbscaler.src_winTop = 0;
	fbscaler.src_winRight = var->xres;
	fbscaler.src_winBottom = var->yres;
		
	if(sc_buf_index)
		fbscaler.dest_Yaddr = fb_scaler_pbuf1;	// destination image address
	else
		fbscaler.dest_Yaddr = fb_scaler_pbuf0;	// destination image address

	sc_buf_index = !sc_buf_index;

	fbscaler.dest_fmt = fbscaler.src_fmt;
	fbscaler.dest_ImgWidth = dest_x;		// destination image width
	fbscaler.dest_ImgHeight = dest_y; 		// destination image height

	fbscaler.dest_winLeft = 0;
	fbscaler.dest_winTop = 0;
	fbscaler.dest_winRight = dest_x;
	fbscaler.dest_winBottom = dest_y;

	if (scaler_filp)
		scaler_filp->f_op->unlocked_ioctl(scaler_filp, TCC_SCALER_IOCTRL_KERENL, (unsigned long)&fbscaler);

	return (unsigned int)fbscaler.dest_Yaddr;
}

void tca_fb_sc_m2m_rdma_active_var(unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	struct fb_var_screeninfo sc_var;
	unsigned int lcd_width, lcd_height;
	unsigned int targetAddr;
  	tcc_display_resize *pResize = &resize_data;
	
	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);
//	pr_info("%s: %d base_addr:0x%x w:%d h:%d offset:%d lcd:%d - %d \n", __FUNCTION__, pdp_data->ddc_info.blk_num, base_addr, var->xres, var->yres, var->yoffset, lcd_width, lcd_height);	

	#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	if((pdp_data->DispOrder == DD_SUB) && ((pdp_data->DispDeviceType== TCC_OUTPUT_COMPONENT) || (pdp_data->DispDeviceType== TCC_OUTPUT_COMPOSITE)))
	  	pResize = &secondary_display_resize_data;
	else
		pResize = &resize_data;
	#endif//CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	lcd_width	-= (pResize->resize_left + pResize->resize_right) * 8;
	lcd_height	-= (pResize->resize_up + pResize->resize_down) * 4;
	
	/* display 3d ui : side-by-side, top-and-bottom */
	if(divide_data.enable)
	{
		/* side-by-side : left/right */
		if(divide_data.mode == 0)
			lcd_width	= lcd_width/2;
		/* top-and-bottom : top/bottom */
		else
			lcd_height	= lcd_height/2;
	}

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		targetAddr = tca_fb_sc_m2m(base_addr, var, lcd_width, lcd_height);
	#else
		if(lcd_width != var->xres || lcd_height != var->yres)
			targetAddr = tca_fb_sc_m2m(base_addr, var, lcd_width, lcd_height);
		else
			targetAddr = base_addr;
	#endif

	sc_var.xres = lcd_width;
	sc_var.yres = lcd_height;
	sc_var.bits_per_pixel = var->bits_per_pixel;

	tca_fb_rdma_active_var((unsigned int)targetAddr, &sc_var, pdp_data);
}

unsigned int tca_fb_g2d_m2m(unsigned int base_addr, struct fb_var_screeninfo * var, unsigned int g2d_rotate)
{
	static unsigned int g2d_buf_index;
	G2D_BITBLIT_TYPE g2d_p;
	
	memset(&g2d_p, 0x00, sizeof(G2D_BITBLIT_TYPE));

//	pr_info("%s: %d \n", __func__, g2d_rotate);	

	g2d_p.responsetype = G2D_POLLING;
	g2d_p.src0 = (unsigned int)base_addr;
	
	if(var->bits_per_pixel == 32)
		g2d_p.srcfm.format = GE_RGB888;
	else
		g2d_p.srcfm.format = GE_RGB565;

	g2d_p.srcfm.data_swap = 0;
	g2d_p.src_imgx = var->xres;
	g2d_p.src_imgy = var->yres;

	g2d_p.ch_mode = g2d_rotate;

	g2d_p.crop_offx = 0;
	g2d_p.crop_offy = 0;
	g2d_p.crop_imgx = var->xres;
	g2d_p.crop_imgy = var->yres;

	if(g2d_buf_index)
		g2d_p.tgt0 = (unsigned int)fb_g2d_pbuf1;	// destination image address
	else
		g2d_p.tgt0 = (unsigned int)fb_g2d_pbuf0;	// destination image address

	g2d_buf_index = !g2d_buf_index;
	g2d_p.tgtfm.format = g2d_p.srcfm.format;

	// destinaion f
	g2d_p.tgtfm.data_swap = 0;
	
	if((g2d_rotate == ROTATE_270) || (g2d_rotate == ROTATE_90)) 	{
		g2d_p.dst_imgx = g2d_p.src_imgy;
		g2d_p.dst_imgy = g2d_p.src_imgx;
	}
	else		{
		g2d_p.dst_imgx = g2d_p.src_imgx;
		g2d_p.dst_imgy = g2d_p.src_imgy;
	}
	
	g2d_p.dst_off_x = 0;
	g2d_p.dst_off_y = 0;
	g2d_p.alpha_value = 255;

	g2d_p.alpha_type = G2d_ALPHA_NONE;

	if (g2d_ioctl != NULL) {
		g2d_ioctl((struct file *)&g2d_filp, TCC_GRP_ROTATE_IOCTRL_KERNEL, (unsigned long)&g2d_p);
	}

	return g2d_p.tgt0;
}

void tca_fb_sc_g2d_rdma_active_var(unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	struct fb_var_screeninfo tar_var;
	unsigned int targetAddr, rotate, lcd_width, lcd_height;
  	tcc_display_resize *pResize = &resize_data;

	#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	if((pdp_data->DispOrder == DD_SUB) && ((pdp_data->DispDeviceType== TCC_OUTPUT_COMPONENT) || (pdp_data->DispDeviceType== TCC_OUTPUT_COMPOSITE)))
	  	pResize = &secondary_display_resize_data;
	else
		pResize = &resize_data;
	#endif//CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB

	
	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);	
//	pr_info("%s: %d base_addr:0x%x w:%d h:%d offset:%d lcd:%d - %d \n", __FUNCTION__, pdp_data->ddc_info.blk_num, base_addr, var->xres, var->yres, var->yoffset, lcd_width, lcd_height);	

	lcd_width	-= (pResize->resize_left + pResize->resize_right) * 8;
	lcd_height	-= (pResize->resize_up + pResize->resize_down) * 4;
	
	#if	defined(CONFIG_HDMI_FB_ROTATE_180)
		rotate = ROTATE_180;
	#elif defined(CONFIG_HDMI_FB_ROTATE_270)
		rotate = ROTATE_270;
	#elif defined(CONFIG_HDMI_FB_ROTATE_90)
		rotate = ROTATE_90;
	#else 
		rotate = 0;
	#endif//
		
	tar_var = *var;

	if((lcd_width * lcd_height) > (var->xres * var->yres))
	{
		targetAddr = tca_fb_g2d_m2m(base_addr, var, rotate);
		if((rotate == ROTATE_90) || (rotate == ROTATE_270)) {
			tar_var.xres = var->yres;
			tar_var.yres = var->xres;
		}
		targetAddr = tca_fb_sc_m2m(targetAddr, &tar_var, lcd_width, lcd_height);
	}
	else
	{
		targetAddr = tca_fb_sc_m2m(base_addr, var, lcd_width, lcd_height);
		tar_var.xres = lcd_width;
		tar_var.yres = lcd_height;

		targetAddr = tca_fb_g2d_m2m(targetAddr, &tar_var, rotate);
		if((rotate == ROTATE_90) || (rotate == ROTATE_270)) {
			tar_var.xres = lcd_height;
			tar_var.yres = lcd_width;
		}
	}

	tca_fb_rdma_active_var(targetAddr, &tar_var, pdp_data);
}

void tca_fb_mvc_active_var(unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	unsigned int interlace_output = 0;
	unsigned int lcd_width = 0, lcd_height = 0;
	unsigned int img_width = 0, img_height = 0, ifmt = 0;
	unsigned int chromaR = 0, chromaG = 0, chromaB = 0;
	unsigned int chroma_en = 0, alpha_blending_en = 0, alpha_type = 0;
	unsigned int iVBlank = 0;

	SCALER_TYPE fbscaler;
	PVIOC_SC pSC0 = (VIOC_SC *)VIOC_SC_GetAddress( AnD_FB_SC);
	PVIOC_SC pSC1 = (VIOC_SC *)VIOC_SC_GetAddress( VIOC_SC3);

	memset(&fbscaler, 0x00, sizeof(SCALER_TYPE));

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);

	if((!lcd_width) || (!lcd_height))
	{
		dprintk(" %s - Err: width=%d, height=%d, bpp=%d, LCD W:%d H:%d\n", __func__, var->xres, var->yres, var->bits_per_pixel, lcd_width, lcd_height);
		return;
	}

	dprintk(" %s width=%d, height=%d, bpp=%d\n", __func__, var->xres, var->yres, var->bits_per_pixel);

	if(var->bits_per_pixel == 32)
	{
		chroma_en = UI_CHROMA_EN;
		alpha_type = 1;
		alpha_blending_en = 1;
		ifmt = TCC_LCDC_IMG_FMT_RGB888;
	}
	else
	{
		chroma_en = 1;
		alpha_type = 0;
		alpha_blending_en = 0;
		ifmt = TCC_LCDC_IMG_FMT_RGB565;
	}

	img_width = var->xres;
	img_height = var->yres;

	iVBlank = hdmi_get_VBlank();
	lcd_height = (lcd_height - iVBlank)/2;

	if (pSC0 != NULL && pSC1 != NULL) {
		if( lcd_width == 1280 && lcd_height == 720) {
			VIOC_SC_SetBypass (pSC0, ON);
			VIOC_SC_SetBypass (pSC1, ON);
		} else {
			VIOC_SC_SetBypass (pSC0, OFF);
			VIOC_SC_SetBypass (pSC1, OFF);
		}

		VIOC_SC_SetSrcSize(pSC0, img_width, img_height);
		VIOC_SC_SetDstSize (pSC0, lcd_width, lcd_height);	// set destination size in scaler
		VIOC_SC_SetOutSize (pSC0, lcd_width, lcd_height);	// set output size in scaer
		VIOC_SC_SetUpdate (pSC0);							// scaler update

		VIOC_SC_SetSrcSize(pSC1, img_width, img_height);
		VIOC_SC_SetDstSize (pSC1, lcd_width, lcd_height);	// set destination size in scaler
		VIOC_SC_SetOutSize (pSC1, lcd_width, lcd_height);	// set output size in scaer
		VIOC_SC_SetUpdate (pSC1);							// scaler update
	}

	//--------------------------------------
	// set RDMA for 1st UI
	//--------------------------------------
	VIOC_RDMA_SetImageUVIEnable(pdp_data->rdma_info[RDMA_FB].virt_addr, true);

	// size
	VIOC_RDMA_SetImageSize(pdp_data->rdma_info[RDMA_FB].virt_addr, img_width, img_height);

	// format
	VIOC_RDMA_SetImageFormat(pdp_data->rdma_info[RDMA_FB].virt_addr, ifmt);
	//if ( ifmt>= TCC_LCDC_IMG_FMT_YUV420SP && ifmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)	{
	//	VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[RDMA_FB].virt_addr, true);
	//	VIOC_RDMA_SetImageY2RMode(pdp_data->rdma_info[RDMA_FB].virt_addr, 0); /* Y2RMode Default 0 (Studio Color) */
	//} else {
		VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[RDMA_FB].virt_addr, 0);
	//}

	VIOC_RDMA_SetImageIntl(pdp_data->rdma_info[RDMA_FB].virt_addr, interlace_output);

	//offset
	VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[RDMA_FB].virt_addr, ifmt, img_width);

	// alpha & chroma key color setting
	VIOC_RDMA_SetImageAlphaSelect(pdp_data->rdma_info[RDMA_FB].virt_addr, true);
	VIOC_RDMA_SetImageAlphaEnable(pdp_data->rdma_info[RDMA_FB].virt_addr, true);

	// Update base addr
	VIOC_RDMA_SetImageBase(pdp_data->rdma_info[RDMA_FB].virt_addr, base_addr, 0, 0);

	//--------------------------------------
	// set RDMA for 2nd UI
	//--------------------------------------
	VIOC_RDMA_SetImageUVIEnable(pdp_data->rdma_info[RDMA_3D].virt_addr, true);
			
	// size
	VIOC_RDMA_SetImageSize(pdp_data->rdma_info[RDMA_3D].virt_addr, img_width, img_height);

	// format
	VIOC_RDMA_SetImageFormat(pdp_data->rdma_info[RDMA_3D].virt_addr, ifmt);
	//if ( ifmt>= TCC_LCDC_IMG_FMT_YUV420SP && ifmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)	{
	//	VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[RDMA_3D].virt_addr, true);
	//	VIOC_RDMA_SetImageY2RMode(pdp_data->rdma_info[RDMA_3D].virt_addr, 0); /* Y2RMode Default 0 (Studio Color) */
	//} else {
		VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[RDMA_3D].virt_addr, 0);
	//}

	VIOC_RDMA_SetImageIntl(pdp_data->rdma_info[RDMA_3D].virt_addr, interlace_output);

	//offset
	VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[RDMA_3D].virt_addr, ifmt, img_width);

	// alpha & chroma key color setting
	VIOC_RDMA_SetImageAlphaSelect(pdp_data->rdma_info[RDMA_3D].virt_addr, true);
	VIOC_RDMA_SetImageAlphaEnable(pdp_data->rdma_info[RDMA_3D].virt_addr, true);

	// Update base addr
	VIOC_RDMA_SetImageBase(pdp_data->rdma_info[RDMA_3D].virt_addr, base_addr, 0, 0);

	//--------------------------------------
	// Set WMIX
	//--------------------------------------
	VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, 0, 0, 0);
	VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, 1, 0, lcd_height+iVBlank);

	#if defined(CONFIG_ARCH_TCC898X)
	VIOC_WMIX_SetBGColor(pdp_data->wmixer_info.virt_addr, 0x3ff, 0x3ff, 0x3ff, 0x3ff);
	#else
	VIOC_WMIX_SetBGColor(pdp_data->wmixer_info.virt_addr, 0xff, 0xff, 0xff, 0xff);
	#endif

	#ifdef CONFIG_ARCH_TCC898X
	VIOC_WMIX_SetChromaKey(pdp_data->wmixer_info.virt_addr, 0, chroma_en, chromaR, chromaG, chromaB, 0x3FF, 0x3FF, 0x3FF);
	VIOC_WMIX_SetChromaKey(pdp_data->wmixer_info.virt_addr, 1, chroma_en, chromaR, chromaG, chromaB, 0x3FF, 0x3FF, 0x3FF);
	#else
	VIOC_WMIX_SetChromaKey(pdp_data->wmixer_info.virt_addr, 0, chroma_en, chromaR, chromaG, chromaB, 0xF8, 0xFC, 0xF8);
	VIOC_WMIX_SetChromaKey(pdp_data->wmixer_info.virt_addr, 1, chroma_en, chromaR, chromaG, chromaB, 0xF8, 0xFC, 0xF8);
	#endif//

	VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);

	//--------------------------------------
	// enable RDMA for 1st and 2nd UI
	//--------------------------------------
	VIOC_RDMA_SetImageEnable(pdp_data->rdma_info[RDMA_FB].virt_addr);
	VIOC_RDMA_SetImageEnable(pdp_data->rdma_info[RDMA_3D].virt_addr);

}

void tca_fb_attach_update(struct tcc_dp_device *pMdp_data, struct tcc_dp_device *pSdp_data)
{
	unsigned int img_width, img_height, base_addr,attach_posx,attach_posy;
	
	if(attach_data.flag == 0)
		return;
		
	if(attach_data.index)
		base_addr = (unsigned int)attach_data.pbuf1;
	else
		base_addr = (unsigned int)attach_data.pbuf0;

	attach_data.index = !attach_data.index;
	
	/* get the size of the attached output */
	VIOC_DISP_GetSize(pSdp_data->ddc_info.virt_addr, &img_width, &img_height);

	/* default framebuffer */
	VIOC_WMIX_SetPosition(pSdp_data->wmixer_info.virt_addr, RDMA_FB, 0, 0);
	/* overlay setting */
	#ifdef CONFIG_ARCH_TCC898X
	VIOC_WMIX_SetChromaKey(pSdp_data->wmixer_info.virt_addr, RDMA_FB, 0, 0, 0, 0, 0x3FF, 0x3FF, 0x3FF);
	#else
	VIOC_WMIX_SetChromaKey(pSdp_data->wmixer_info.virt_addr, RDMA_FB, 0, 0, 0, 0, 0xF8, 0xFC, 0xF8);
	#endif//
	VIOC_RDMA_SetImageFormat(pSdp_data->rdma_info[RDMA_FB].virt_addr, TCC_LCDC_IMG_FMT_RGB888);				//fmt
	VIOC_RDMA_SetImageOffset(pSdp_data->rdma_info[RDMA_FB].virt_addr, TCC_LCDC_IMG_FMT_RGB888, img_width);	//offset	
	VIOC_RDMA_SetImageSize(pSdp_data->rdma_info[RDMA_FB].virt_addr, img_width, img_height);					//size	
	VIOC_RDMA_SetImageBase(pSdp_data->rdma_info[RDMA_FB].virt_addr, base_addr, 0, 0);

	VIOC_RDMA_SetImageAlphaSelect(pSdp_data->rdma_info[RDMA_FB].virt_addr, 0);
	VIOC_RDMA_SetImageAlphaEnable(pSdp_data->rdma_info[RDMA_FB].virt_addr, 0);

	if(attach_data.index)
		base_addr = (unsigned int)attach_data.pbuf1;
	else
		base_addr = (unsigned int)attach_data.pbuf0;

	attach_posx = output_attach_resize_data.resize_left * 8;
	attach_posy = output_attach_resize_data.resize_up * 4;
	
	VIOC_WDMA_SetImageBase(pMdp_data->wdma_info.virt_addr, base_addr+attach_posx*4+(img_width)*attach_posy*4, 0, 0);
	VIOC_WDMA_SetImageUpdate(pMdp_data->wdma_info.virt_addr);
	VIOC_WMIX_SetUpdate(pSdp_data->wmixer_info.virt_addr);	

	VIOC_RDMA_SetImageEnable(pSdp_data->rdma_info[RDMA_FB].virt_addr);

	if(pSdp_data->FbPowerState)
		VIOC_DISP_TurnOn(pSdp_data->ddc_info.virt_addr);
}

void tca_fb_attach_update_flag(void)
{
	if(attach_data.flag)
	{
		if(attach_data.update_flag == 0)
			attach_data.update_flag = 1;
	}
}

static irqreturn_t tca_fb_attach_handler(int irq, void *dev_id)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = info->par;
	struct tcc_dp_device *pMdp_data = &tccfb_info->pdata.Mdp_data;
	struct tcc_dp_device *pSdp_data = &tccfb_info->pdata.Sdp_data;
	unsigned int WDMA_IRQ_Status;

	if (!is_vioc_intr_activatied(attach_intr.vioc_intr->id, (1<<attach_intr.vioc_intr->bits))) {
		return IRQ_NONE;
	}

	WDMA_IRQ_Status = vioc_intr_get_status(attach_intr.vioc_intr->id);
 	dprintk("%s IRQ_STATUS:0x%x\n", __func__, WDMA_IRQ_Status);

	if(WDMA_IRQ_Status & (1<<attach_intr.vioc_intr->bits)) {
		dprintk("%s VIOC_WDMA_IREQ_EOFR_MASK \n", __func__);

		tca_fb_attach_update_flag();

		if(attach_data.flag)
		{
			if(attach_data.update_start)
			{
				tca_fb_attach_update(pMdp_data, pSdp_data);
				attach_data.update_start = 0;
			}

			if(attach_data.update_flag)
			{
				attach_data.update_flag = 0;
				attach_data.update_start = 1;
			}
		}

		vioc_intr_clear(attach_intr.vioc_intr->id, (1<<attach_intr.vioc_intr->bits));
	}

	return IRQ_HANDLED;
}

void tca_fb_attach_start(struct tccfb_info *info)
{
	PVIOC_WDMA	pWDMA;
	PVIOC_SC 	pSC;
	unsigned int main_wd, main_ht, attach_wd, attach_ht, scaler_num, ret;
	unsigned int   attach_posx = 0, attach_posy = 0;
	pmap_t pmap;

	struct tcc_dp_device *pMdp_data = &info->pdata.Mdp_data;
	struct tcc_dp_device *pSdp_data = &info->pdata.Sdp_data;

	printk("%s\n", __func__);

	/* set the buffer for attached output */		
	#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)|| defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
	pmap_get_info("output_attach", &pmap);
	#else
	pmap_get_info("viqe", &pmap);
	#endif

	attach_data.pbuf0 = (char *) pmap.base;
	attach_data.pbuf1 = (char *) attach_data.pbuf0 + 2*1024*1024;
	
	/* set WDMA register */
	pWDMA = (VIOC_WDMA *)pMdp_data->wdma_info.virt_addr;

	/* enable WDMA interrupt handler */
	if(pMdp_data->DispNum)
		attach_intr.vioc_intr->id = VIOC_INTR_WD1;
	else
		attach_intr.vioc_intr->id = VIOC_INTR_WD0;
	attach_intr.vioc_intr->bits = VIOC_WDMA_INTR_EOFR;

	vioc_intr_disable(attach_intr.vioc_intr->id, (1<<attach_intr.vioc_intr->bits));
	ret = request_irq(INT_LCDC, tca_fb_attach_handler, IRQF_SHARED, "TCC_WDMA", &attach_intr);
	vioc_intr_enable(attach_intr.vioc_intr->id, (1<<attach_intr.vioc_intr->bits));

	/* set the attached WMIX path to Bypass */
	if(pMdp_data->DispNum)
		VIOC_CONFIG_WMIXPath(WMIX00, 0);
	else
		VIOC_CONFIG_WMIXPath(WMIX10, 0);
	
	/* set the scaler information */		
	scaler_num = ATTACH_SCALER;	
	
	pSC = (VIOC_SC *)VIOC_SC_GetAddress( scaler_num);

	/* get the resolution of main output */
	VIOC_DISP_GetSize(pMdp_data->ddc_info.virt_addr, &main_wd, &main_ht);

	/* get the resolution of attached output */
	VIOC_DISP_GetSize(pSdp_data->ddc_info.virt_addr, &attach_wd, &attach_ht);

	attach_wd -= (output_attach_resize_data.resize_left + output_attach_resize_data.resize_right) * 8;
	attach_ht -= (output_attach_resize_data.resize_up + output_attach_resize_data.resize_down) * 4;
	attach_posx = output_attach_resize_data.resize_left * 8;
	attach_posy = output_attach_resize_data.resize_up * 4;

	/* set SCALER before plugging-in */
	if (pSC != NULL) {
		VIOC_SC_SetBypass(pSC, OFF);
		VIOC_SC_SetSrcSize(pSC, main_wd, main_ht);
		VIOC_SC_SetDstSize(pSC, attach_wd, attach_ht);
		VIOC_SC_SetOutSize(pSC, attach_wd, attach_ht);
		VIOC_SC_SetUpdate(pSC);
	}

	/* plug the scaler in WDMA */		
	if(attach_data.plugin == 0)
	{
		if(pMdp_data->DispNum)
			VIOC_CONFIG_PlugIn(scaler_num, VIOC_SC_WDMA_01);
		else
			VIOC_CONFIG_PlugIn(scaler_num, VIOC_SC_WDMA_00);

		attach_data.plugin = 1;
	}

	/* set the control register of WDMA */
	VIOC_WDMA_SetImageR2YEnable(pWDMA, 0);
	VIOC_WDMA_SetImageSize(pWDMA, attach_wd, attach_ht);
	attach_wd += (output_attach_resize_data.resize_left + output_attach_resize_data.resize_right) * 8;
	VIOC_WDMA_SetImageOffset(pWDMA, TCC_LCDC_IMG_FMT_RGB888, attach_wd);
	VIOC_WDMA_SetImageFormat(pWDMA, TCC_LCDC_IMG_FMT_RGB888);
	VIOC_WDMA_SetImageBase(pWDMA, (unsigned int)attach_data.pbuf0+attach_posx*4+attach_wd*attach_posy*4, 0, 0);

	if((pWDMA->uCTRL.nREG & HwDMA_IEN) == 0)
		VIOC_WDMA_SetImageEnable(pWDMA, 1 /* ON */);
	else
		VIOC_WDMA_SetImageUpdate(pWDMA);

	attach_data.flag = 1;
	attach_data.index = 0;
	attach_data.update_flag = 0;
	attach_data.update_start = 0;
	attach_data.update_started = 0;
}

void tca_fb_attach_stop(struct tccfb_info *info)
{
	PVIOC_WDMA	pWDMA;
	unsigned int scaler_num; 

	struct tcc_dp_device *pMdp_data = &info->pdata.Mdp_data;
	
	printk("%s\n", __func__);

	if(attach_data.flag == 0)
		return;

	/* set WDMA register */
	pWDMA = (VIOC_WDMA *)pMdp_data->wdma_info.virt_addr;

	/* disable WDMA */
	VIOC_WDMA_SetImageDisable(pWDMA);

	/* disable WDMA interrupt handler */
	vioc_intr_disable(attach_intr.vioc_intr->id, (1<<attach_intr.vioc_intr->bits));
	free_irq(INT_LCDC, &attach_intr);

	/* set the scaler number */		
	scaler_num = ATTACH_SCALER;
	
	/* plug-out scaler */
	if(attach_data.plugin)
	{
		if(VIOC_CONFIG_PlugOut(scaler_num) != VIOC_PATH_DISCONNECTED)
		{
			if(pMdp_data->DispNum) {
				VIOC_SC_SetSWReset(scaler_num, VIOC_SC_WDMA_00, 0xFF);
			} else {
				VIOC_SC_SetSWReset(scaler_num, VIOC_SC_WDMA_01, 0xFF);
			}
		}

		attach_data.plugin = 0;
	}

	memset((void*)&attach_data, 0x00, sizeof(tcc_display_attach));
}

unsigned char tca_fb_mouse_data_out(unsigned int data_in, unsigned char yuv, unsigned char threshold_use)
{
	unsigned char data_out;
	unsigned char threshold_value = 128;

	//[19] = underflow, [18:17] = overflow
	if(data_in & (0x01 << 19))
		data_out = 0;
	else if ((data_in & (0x01 << 18)) || (data_in & (0x01 << 17)))
		data_out = 0xFF;
	else
		data_out = ((data_in >> 9) & 0x00FF);

	if((yuv == 0) && (threshold_use == 1))
	{
		if(data_out > threshold_value)
			data_out = threshold_value;
	}

	return data_out;
}

void tca_fb_mouse_set_icon(tcc_mouse_icon *mouse_icon)
{
    int i, j, k;
    int output_mode, hdmi_mode;
    char *dst, *src = mouse_icon->buf;

    if(!mouse_data.index)
        dst = (char *)mouse_data.vbuf1;
    else
        dst = (char *)mouse_data.vbuf2;

    mouse_data.width = ((mouse_icon->width + 3) >> 2) << 2;
    mouse_data.height = ((mouse_icon->height + 1)>> 1) << 1;

    for(i=0; i<mouse_icon->height; i++)
    {
        for(j=0; j<mouse_icon->width; j++)
        {
            *(dst+3) = *(src+3);
            *(dst+2) = *(src+0);
            *(dst+1) = *(src+1);
            *(dst+0) = *(src+2);
            dst+=4;
            src+=4;
        }

        for(k=0; k<(mouse_data.width - mouse_icon->width); k++)
        {
            *(dst+3) = 0;
            *(dst+2) = 0;
            *(dst+1) = 0;
            *(dst+0) = 0;
            dst+=4;
        }
    }

	hdmi_mode	= (mouse_icon->option & 0x00ff) >> 0; /* 0: HDMI, 1: DVI */
	output_mode	= (mouse_icon->option & 0xff00) >> 8; /* 1: HDMI, 2: CVBS, 3:Component */

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
			if(output_mode != OUTPUT_HDMI || hdmi_mode != 1)
		#else
			if(output_mode == OUTPUT_COMPOSITE)
		#endif
			{
				unsigned int wYMULR, wYMULG, wYMULB;
				unsigned int wUMULR, wUMULG, wUMULB;
				unsigned int wVMULR, wVMULG, wVMULB;
				unsigned char wYADDR, wUADDR, wVADDR;
				unsigned int wY_tmp, wU_tmp, wV_tmp;
				unsigned char wA, wR, wG, wB;
				unsigned char wY, wU, wV;

			    if(!mouse_data.index)
			        dst = (char *)mouse_data.vbuf1;
			    else
			        dst = (char *)mouse_data.vbuf2;

				wYMULR = 0x06d; //	0.213
				wYMULG = 0x16e; //	0.715
				wYMULB = 0x025; //	0.072

				wUMULR = 0x03c; //	0.117
				wUMULG = 0x0ca; //	0.394
				wUMULB = 0x106; //	0.511

				wVMULR = 0x106; //	0.511
				wVMULG = 0x0ee; //	0.464
				wVMULB = 0x018; //	0.047

				wYADDR = 0x0;	//	0
				wUADDR = 0x80;	//	128
				wVADDR = 0x80;	//	128

			    for(i=0; i<mouse_icon->height; i++)
			    {
			        for(j=0; j<mouse_icon->width; j++)
			        {
			            wA = *(dst+3);
			            wR = *(dst+2);
			            wG = *(dst+1);
			            wB = *(dst+0);

						wY_tmp = (wYADDR << 9) + (wYMULR * wR) + (wYMULG * wG) + (wYMULB * wB);
						wU_tmp = (wUADDR << 9) - (wUMULR * wR) - (wUMULG * wG) + (wUMULB * wB);
						wV_tmp = (wVADDR << 9) + (wVMULR * wR) - (wVMULG * wG) - (wVMULB * wB);

						if(output_mode == OUTPUT_COMPOSITE)
							wY = tca_fb_mouse_data_out(wY_tmp, 0, 1);
						else
							wY = tca_fb_mouse_data_out(wY_tmp, 0, 0);

						wU = tca_fb_mouse_data_out(wU_tmp, 0, 0);
						wV = tca_fb_mouse_data_out(wV_tmp, 0, 0);

						*(dst+3) = wA;
						*(dst+2) = wY;
						*(dst+1) = wU;
						*(dst+0) = wV;

			            dst+=4;
			        }

			        for(k=0; k<(mouse_cursor_width-mouse_icon->width); k++)
			        {
						*(dst+3) = 0x00;	//Alpha
						*(dst+2) = 0x00;	//Y
						*(dst+1) = 0x80;	//Cb
						*(dst+0) = 0x80;	//Cr
			            dst+=4;
					}
				}
			}
	#endif

    mouse_data.index = !mouse_data.index;
}

int tca_fb_mouse_move(unsigned int width, unsigned int height, tcc_mouse *mouse, struct tcc_dp_device *pdp_data)
{
	PVIOC_DISP pDISPBase = pdp_data->ddc_info.virt_addr;
	PVIOC_WMIX pWMIXBase = pdp_data->wmixer_info.virt_addr;
	PVIOC_RDMA pRDMABase = pdp_data->rdma_info[RDMA_MOUSE].virt_addr;
	
	unsigned int lcd_width, lcd_height, lcd_w_pos,lcd_h_pos, mouse_x, mouse_y;
	unsigned int interlace_output = 0, display_width, display_height;
	unsigned int wmix_channel = 0;
  	tcc_display_resize *pResize = &resize_data;
	
	if((pDISPBase == NULL) || (pWMIXBase == NULL) || (pRDMABase == NULL)) {
		//pr_err("%s - VIOC is not valid, RDMA:0x%08x, WMIX:0x%08x, DISP:0x%08x\n", __func__, pDISPBase, pWMIXBase, pRDMABase);
		return 0;
	}

	wmix_channel = pdp_data->rdma_info[RDMA_MOUSE].blk_num;

	#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
		if( !(pDISPBase->uCTRL.nREG & HwDISP_NI )) {//interlace mode
			interlace_output = 1;
		}
	#endif

	//dprintk("%s RDMA:0x%08x, WMIX:0x%08x, DISP:0x%08x\n", __func__, pRDMABase, pWMIXBase, pDISPBase);

	#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	if((pdp_data->DispOrder == DD_SUB) && (pdp_data->DispDeviceType== TCC_OUTPUT_COMPONENT))
	  	pResize = &secondary_display_resize_data;
	else
		pResize = &resize_data;
	#endif//CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB

	
	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);
	
	if((!lcd_width) || (!lcd_height))
		return -1;

	lcd_width	-= (pResize->resize_left + pResize->resize_right) << 3;
	lcd_height	-= (pResize->resize_up + pResize->resize_down) << 2;

	mouse_x = (unsigned int)(lcd_width * mouse->x / width);
	mouse_y = (unsigned int)(lcd_height *mouse->y / height);

	if( mouse_x > lcd_width - mouse_data.width )
		display_width = lcd_width - mouse_x;
	else
		display_width = mouse_data.width;

	if( mouse_y > lcd_height - mouse_data.height )
		display_height = lcd_height - mouse_y;
	else
		display_height = mouse_data.height;

	VIOC_RDMA_SetImageY2REnable(pRDMABase, false);

	VIOC_RDMA_SetImageOffset(pRDMABase, TCC_LCDC_IMG_FMT_RGB888, mouse_data.width);
	VIOC_RDMA_SetImageFormat(pRDMABase, TCC_LCDC_IMG_FMT_RGB888);

	mouse_x += pResize->resize_left << 3;
	if(interlace_output)
		mouse_y += pResize->resize_up << 1;
	else
		mouse_y += pResize->resize_up << 2;

	lcd_w_pos = mouse_x;
	lcd_h_pos = mouse_y;

	dprintk("%s lcd_width:%d, lcd_height:%d, lcd_w_pos:%d, lcd_h_pos:%d\n\n", __func__, lcd_width, lcd_height, lcd_w_pos, lcd_h_pos);
	
	// position
	VIOC_WMIX_SetPosition(pWMIXBase, wmix_channel, lcd_w_pos, lcd_h_pos);

	VIOC_RDMA_SetImageSize(pRDMABase, display_width, display_height);
	VIOC_RDMA_SetImageIntl(pRDMABase, interlace_output);

	#if !defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
	if(interlace_output)
		VIOC_WMIX_SetPosition(pWMIXBase, wmix_channel,  lcd_w_pos, (lcd_h_pos/2) );
	else
		VIOC_WMIX_SetPosition(pWMIXBase, wmix_channel, lcd_w_pos, lcd_h_pos);
	#endif

	#if defined(CONFIG_TCC_VIOCMG)
	viocmg_set_wmix_ovp(VIOCMG_CALLERID_FB, pdp_data->wmixer_info.blk_num, 8);
	#else
	VIOC_WMIX_SetOverlayPriority(pWMIXBase, 8);
	#endif

	// alpha & chroma key color setting
	VIOC_RDMA_SetImageAlphaSelect(pRDMABase, 1);
	VIOC_RDMA_SetImageAlphaEnable(pRDMABase, 1);
	
	// image address
	if(mouse_data.index)
		VIOC_RDMA_SetImageBase(pRDMABase, (unsigned int)mouse_data.pbuf1, 0, 0);
	else
		VIOC_RDMA_SetImageBase(pRDMABase, (unsigned int)mouse_data.pbuf2, 0, 0);

	VIOC_WMIX_SetUpdate(pWMIXBase);
	VIOC_RDMA_SetImageEnable(pRDMABase);

	#if defined(CONFIG_TCC_DISPLAY_MODE_USE) && !defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT)
		tca_fb_attach_update_flag();
	#endif

	return 1;
}

void tca_fb_resize_set_value(tcc_display_resize resize_value, TCC_OUTPUT_TYPE output)
{
	printk( "%s : resize_up = %d, resize_down = %d, resize_left = %d, resize_right = %d\n", __func__, 
			resize_value.resize_up, resize_value.resize_down, resize_value.resize_left, resize_value.resize_right );
#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
	if((output == TCC_OUTPUT_COMPONENT) || (output == TCC_OUTPUT_COMPOSITE))
		memcpy((void *)&secondary_display_resize_data, (void *)&resize_value, sizeof(tcc_display_resize));
	else
#endif//CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB		
		memcpy((void *)&resize_data, (void *)&resize_value, sizeof(tcc_display_resize));
}

void tca_fb_output_attach_resize_set_value(tcc_display_resize resize_value)
{
	printk( "%s : output_attach resize_up = %d, resize_down = %d, resize_left = %d, resize_right = %d\n", __func__, 
			resize_value.resize_up, resize_value.resize_down, resize_value.resize_left, resize_value.resize_right );
	
	memcpy((void *)&output_attach_resize_data, (void *)&resize_value, sizeof(tcc_display_resize));
}
int tca_fb_divide_set_mode(struct tcc_dp_device *pdp_data, char enable, char mode)
{

	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();
	unsigned int iSCType;


	divide_data.enable = enable;
	divide_data.mode = mode;
	
	#if defined(CONFIG_ANDROID) || defined(CONFIG_PLATFORM_STB)
		iSCType = VIOC_SC3;
	#else
		iSCType = VIOC_SC0;
	#endif//
	
	if(enable)
	{
		divide_data.fbtype = pdp_data->FbUpdateType;
		pdp_data->FbUpdateType = FB_DIVIDE_UPDATE;
		VIOC_CONFIG_PlugIn(iSCType, pdp_data->rdma_info[RDMA_3D].blk_num);
	}
	else
	{
		pdp_data->FbUpdateType = divide_data.fbtype;
		
		VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[RDMA_3D].virt_addr);

		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[RDMA_3D].blk_num, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[RDMA_3D].blk_num, VIOC_CONFIG_CLEAR);

		if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[RDMA_3D].blk_num) != -1) //scaler plug in status check
		{
			pr_info("scaler %d plug out \n", iSCType);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);
			VIOC_CONFIG_PlugOut(iSCType);

		}
	}
	
	printk("%s, enable=%d, mode=%d, fbtype=%d\n", __func__, enable, mode, divide_data.fbtype);

	return 0;
}

int tca_fb_divide_get_status(void)
{
	return divide_data.enable;
}

void tca_fb_divide_active_var(unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	/* Update Left/Top screen */
	divide_data.count = 0;
	if(divide_data.fbtype == FB_SC_M2M_RDMA_UPDATE)
		tca_fb_sc_m2m_rdma_active_var(base_addr, var, pdp_data);
	else
		tca_fb_sc_rdma_active_var(base_addr, var, pdp_data);
			
	/* Update Right/Bottom screen */
	divide_data.count = 1;
	if(divide_data.fbtype == FB_SC_M2M_RDMA_UPDATE)
		tca_fb_sc_m2m_rdma_active_var(base_addr, var, pdp_data);
	else
		tca_fb_sc_rdma_active_var(base_addr, var, pdp_data);
}

// pjj modify need
#define RDMA_UVI_MAX_WIDTH             3072


void tca_lcdc_set_onthefly(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo)
{
	unsigned int iSCType = 0;
	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();

	if((pdp_data == NULL) || (ImageInfo == NULL)){
		pr_err("ERROR %s pdp_data:%p ImageInfo:%p\n", __func__, pdp_data, ImageInfo);
		return;
	}
	
	if(ImageInfo->MVCframeView == 1)
	{
		dprintk("%s is returned because of mvc frame view\n", __func__);
		return;
	}

	if(Output_LayerMode == TCC_LAYER_SHOW) {
		if(ImageInfo->on_the_fly)
		{
			iSCType = tca_get_scaler_num(pdp_data->DispDeviceType, ImageInfo->Lcdc_layer);

			if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) == -1)
			{
				pr_info(" %s  scaler %u is plug in RDMA %d \n",__func__, iSCType, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
				VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);

				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);

				VIOC_CONFIG_PlugIn (iSCType, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);

				#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
				if(ImageInfo->private_data.optional_info[16] == true)
				{
					pr_info("Map converter  %d \n",pdp_data->ddc_info.blk_num);

					if( pdp_data->ddc_info.blk_num == 1) {
						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, 1, VIOC_CONFIG_RESET);
						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, 1, VIOC_CONFIG_CLEAR);
						tca_set_MC_Connect_to_RDMA(1, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 1);
					}
					else {
						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, 0, VIOC_CONFIG_RESET);
						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, 0, VIOC_CONFIG_CLEAR);
						tca_set_MC_Connect_to_RDMA(0, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 1);
					}
				}
				else
				{
					if(pdp_data->ddc_info.blk_num == 1)
						tca_set_MC_Connect_to_RDMA(1, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 0);
					else
						tca_set_MC_Connect_to_RDMA(0, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 0);
				}
				#endif//CONFIG_ARCH_TCC_MAP_CONVERTER
			}
		}
		else
		{
			if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1)
			{
				iSCType = (unsigned int)tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);

				#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
				{
					unsigned int MC_NUM;
					if(tca_get_MC_Connect_to_RDMA(&MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num))
					{
						tca_map_convter_onoff(MC_NUM, 0);
						tca_map_convter_wait_done(MC_NUM);

						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
						VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);
					}
				}
				#endif//CONFIG_ARCH_TCC_MAP_CONVERTER

				pr_info(" %s  scaler %d is plug out RDMA:%d \n",__func__, iSCType, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
				VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);

				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_CLEAR);

				if(iSCType <= VIOC_SC3)
				{
					VIOC_CONFIG_PlugOut(iSCType);
					VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
					VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);
				}
			}
		}
	}
}

void tcc_video_post_process(struct tcc_lcdc_image_update *ImageInfo)
{
	if( ImageInfo->Lcdc_layer != RDMA_VIDEO )
		return;

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	if(enabled_LastFrame)
		tcc_video_clear_last_frame(pLastFrame_RDMABase, false);
#endif
#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
	if(ImageInfo != NULL)
		tcc_video_frame_backup(ImageInfo);
#endif
#ifdef CONFIG_DISPLAY_EXT_FRAME
	tcc_ctrl_ext_frame(0);
#endif
}

void tca_scale_display_update(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo)
{
	VIOC_SC *pSC;
	unsigned int iSCType;
	unsigned int lcd_width = 0, lcd_height = 0;
	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	struct device_node *ViocMapCV_np;
	VIOC_MC *HwVIOC_MC01;
	unsigned int MC_NUM;
	#endif//
	volatile PVIOC_IREQ_CONFIG pIREQConfig;
	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)of_iomap(ViocConfig_np, 0);

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(ViocMapCV_np == NULL)
		return;
	
	HwVIOC_MC01= (VIOC_MC *)of_iomap(ViocMapCV_np, 1);
	#endif//

	if (pdp_data == NULL)
		return;

	//pr_info("%s enable:%d, layer:%d, addr:0x%x, ts:%d, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", __func__, ImageInfo->enable, ImageInfo->Lcdc_layer, ImageInfo->addr0, ImageInfo->time_stamp,
	//		ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);

	if((ImageInfo->Lcdc_layer >= RDMA_MAX_NUM) || (ImageInfo->fmt >TCC_LCDC_IMG_FMT_MAX)){
		pr_err("LCD :: lcdc:%d, enable:%d, layer:%d, addr:0x%x, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", pdp_data->ddc_info.blk_num, ImageInfo->enable, ImageInfo->Lcdc_layer, ImageInfo->addr0, 
				ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);
		return;
	}

	// for disabling video composition : 160502 inbest
	if(Output_LayerMode == TCC_LAYER_HIDE && ImageInfo->Lcdc_layer == RDMA_VIDEO)
	{
		if(!disable_video_composition)
		{
			//pr_info("%s HIDE!!! Layer:%d (%08x/%dmsec)\n", __func__, ImageInfo->Lcdc_layer, ImageInfo->addr0, ImageInfo->time_stamp);
			#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
			if(tca_get_MC_Connect_to_RDMA(&MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num))
			{
				tca_map_convter_onoff(MC_NUM, 0);
				tca_set_MC_Connect_to_RDMA(MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 0);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);
			}
			#endif

			VIOC_RDMA_SetImageDisableNW(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_CLEAR);

			if(ImageInfo->MVCframeView != 1){
				if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1) //scaler plug in status check
				{
					iSCType = tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
					VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
					VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);
					VIOC_CONFIG_PlugOut(iSCType);
					pr_info("scaler %d plug out \n", iSCType);
				}
			}
			disable_video_composition = true;
		}
		return;
	}

	// for disabling video composition : 160502 inbest
	if(ImageInfo->Lcdc_layer == RDMA_VIDEO) {
		//pr_info("%s SHOW!!! Layer:%d (%08x/%dmsec)\n", __func__, ImageInfo->Lcdc_layer, ImageInfo->addr0, ImageInfo->time_stamp);
		disable_video_composition = false;
	}

	if(!ImageInfo->enable)	{
	//map converter 
	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
		if(tca_get_MC_Connect_to_RDMA(&MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num))		{
			tca_map_convter_onoff(MC_NUM, 0);
			tca_map_convter_wait_done(MC_NUM);
			tca_set_MC_Connect_to_RDMA(MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 0);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);			
		}
	#endif//
		VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);	

		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_CLEAR);
	
		if(ImageInfo->MVCframeView != 1){
			
			if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1) //scaler plug in status check
			{
				iSCType = tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);
				VIOC_CONFIG_PlugOut(iSCType);
				pr_info("scaler %d plug out \n", iSCType);
			}
		}
		return;
	}	

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);
	if((!lcd_width) || (!lcd_height)){
		pr_err("%s LCD :: Error :: lcd_width %d, lcd_height %d \n", __func__,lcd_width, lcd_height);
		return;
	}

	/* scaler */
	iSCType = tca_get_scaler_num(pdp_data->DispDeviceType, ImageInfo->Lcdc_layer);
	pSC = (VIOC_SC *)VIOC_SC_GetAddress( iSCType);
	if (pSC != NULL) {
		if((ImageInfo->MVCframeView != 1) && ImageInfo->on_the_fly)
		{		
			VIOC_SC_SetSrcSize(pSC, ImageInfo->Frame_width, ImageInfo->Frame_height);
			VIOC_SC_SetDstSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set output size in scaer

			if(((ImageInfo->crop_right - ImageInfo->crop_left) == ImageInfo->Image_width) && ((ImageInfo->crop_bottom - ImageInfo->crop_top) == ImageInfo->Image_height)){
				VIOC_SC_SetBypass (pSC, ON);
				dprintk(" %s  scaler 1 is plug in SetBypass ON \n",__func__);
			}else {
				VIOC_SC_SetBypass (pSC, OFF);
				dprintk(" %s  scaler 1 is plug in SetBypass OFF \n",__func__);
			}

			if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) == -1) //scaler plug in status check
			{ 
				VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);
				dprintk(" %s  scaler 1 is plug in RDMA %d \n",__func__, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
				VIOC_CONFIG_PlugIn (iSCType, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);	
			}
		}
		else
		{
			if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1)
			{
				dprintk(" %s  scaler 1 is plug out \n",__func__);
				VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);
				VIOC_CONFIG_PlugOut(iSCType);
			}
		}

		if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1)
			VIOC_SC_SetUpdate (pSC);
	}

#if 0
	// position
	VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, ImageInfo->Lcdc_layer, 0, 0);


	VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, true);
	VIOC_RDMA_SetImageY2RMode(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, 0); /* Y2RMode Default 0 (Studio Color) */

	VIOC_RDMA_SetImageUVIEnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, false);
	VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->fmt, ImageInfo->Frame_width);
	VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, lcd_width, lcd_height);
	VIOC_RDMA_SetImageFormat(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->fmt);

	VIOC_RDMA_SetImageBase(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->addr0, ImageInfo->addr1, ImageInfo->addr2);		
	
	VIOC_RDMA_SetImageIntl(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, 0);

	VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);	

	VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);			
 	VIOC_RDMA_SetImageEnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);



#else	
	// position
	VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, ImageInfo->Lcdc_layer, ImageInfo->offset_x, ImageInfo->offset_y);
	VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);	

	//map converter 
	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	if(ImageInfo->private_data.optional_info[16] == true)
	{
		if(pdp_data->ddc_info.blk_num == 1)
			tca_set_MC_Connect_to_RDMA(1, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 1);
		else
			tca_set_MC_Connect_to_RDMA(0, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, 1);

		tca_map_convter_set(pdp_data->ddc_info.blk_num, ImageInfo);
		
		VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);	
		
		#if	0
		{
			int i;
			unsigned int *MC1_add = (unsigned int*)HwVIOC_MC01;
				printk("0x%p: 0x%08x \n", &pIREQConfig->uMC.nREG, pIREQConfig->uMC.nREG);

			for(i = 0; i < sizeof(VIOC_MC) / 4; i++)
				printk("0x%p:  0x%08x \n", (MC1_add + i), *(MC1_add + i));
		}
		#endif//
	}
	else
	#endif//
	{
		if(ImageInfo->fmt == TCC_LCDC_IMG_FMT_COMP)
			VIOC_CONFIG_PlugIn(VIOC_FCDEC0, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num);
		else if(ImageInfo->fmt >= TCC_LCDC_IMG_FMT_UYVY && ImageInfo->fmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)
		{
			VIOC_RDMA_SetImageY2REnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, true);
			VIOC_RDMA_SetImageY2RMode(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, 1); /* Y2RMode Default 0 (Studio Color) */

			if( ImageInfo->Frame_width <= RDMA_UVI_MAX_WIDTH )
				VIOC_RDMA_SetImageUVIEnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, true);
			else
				VIOC_RDMA_SetImageUVIEnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, false);
		}

		if(ImageInfo->one_field_only_interlace)
			VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->fmt, ImageInfo->Frame_width*2);
		else
			VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->fmt, ImageInfo->Frame_width);

		VIOC_RDMA_SetImageFormat(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->fmt);
		VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->crop_right - ImageInfo->crop_left, ImageInfo->crop_bottom - ImageInfo->crop_top);

		if( !( ((ImageInfo->crop_left == 0) && (ImageInfo->crop_right == ImageInfo->Frame_width)) &&  ((ImageInfo->crop_top == 0) && (ImageInfo->crop_bottom == ImageInfo->Frame_height)))  )
		{

			int addr_Y = (unsigned int)ImageInfo->addr0;
			int addr_U = (unsigned int)ImageInfo->addr1;
			int addr_V = (unsigned int)ImageInfo->addr2;

			dprintk(" Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);
	        
			tccxxx_GetAddress(ImageInfo->fmt, (unsigned int)ImageInfo->addr0, ImageInfo->Frame_width, ImageInfo->Frame_height, 		
									ImageInfo->crop_left, ImageInfo->crop_top, &addr_Y, &addr_U, &addr_V);		

			if(ImageInfo->one_field_only_interlace)
				VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->crop_right - ImageInfo->crop_left, (ImageInfo->crop_bottom - ImageInfo->crop_top)/2);
			else
				VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->crop_right - ImageInfo->crop_left, ImageInfo->crop_bottom - ImageInfo->crop_top);

			VIOC_RDMA_SetImageBase(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, addr_Y, addr_U, addr_V);
		}
		else
		{	
			dprintk(" don't Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);
			if(ImageInfo->one_field_only_interlace)
				VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->Frame_width, ImageInfo->Frame_height/2);
			else
				VIOC_RDMA_SetImageSize(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->Frame_width, ImageInfo->Frame_height);

			VIOC_RDMA_SetImageBase(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, ImageInfo->addr0, ImageInfo->addr1, ImageInfo->addr2);		
		}
			
		VIOC_RDMA_SetImageIntl(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr, 0);


		VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);	

	 	VIOC_RDMA_SetImageEnable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);
	}


	tcc_video_post_process(ImageInfo);


#endif//
}

void tca_mvc_display_update(char hdmi_lcdc, struct tcc_lcdc_image_update *ImageInfo)
{
	VIOC_DISP * pDISPBase;
	VIOC_WMIX * pWMIXBase;
	VIOC_RDMA * pRDMABase0;
	VIOC_RDMA * pRDMABase1;

	unsigned int lcd_width = 0, lcd_height = 0, iVBlank = 0;

	dprintk("%s enable:%d, layer:%d, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", __func__, ImageInfo->enable, ImageInfo->Lcdc_layer,
			ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);


	if(hdmi_lcdc)
	{
		pDISPBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
		pWMIXBase = (VIOC_WMIX*)tcc_p2v(HwVIOC_WMIX1);		
		pRDMABase0 = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA06);
		pRDMABase1 = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA07);
	}
	else
	{
		pDISPBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);
		pWMIXBase = (VIOC_WMIX*)tcc_p2v(HwVIOC_WMIX0);
		pRDMABase0 = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA02);
		pRDMABase1 = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA03);
	}

	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);
	
	if((!lcd_width) || (!lcd_height))
	{
		printk("%s - lcd width and hight is not normal!!!!\n", __func__);
		return;
	}

	iVBlank = hdmi_get_VBlank();

	if(!ImageInfo->enable)	{
		VIOC_RDMA_SetImageDisable(pRDMABase0);
		VIOC_RDMA_SetImageDisable(pRDMABase1);
		printk("%s - Image Info is not enable, so RDAMA is disable.\n", __func__);
		return;
	}

	//dprintk("%s lcdc:%d,pRDMA0:0x%08x pRDMA1:0x%08x, pWMIX:0x%08x, pDISP:0x%08x, addr0:0x%08x\n", __func__, hdmi_lcdc, pRDMABase0, pRDMABase1, pWMIXBase, pDISPBase, ImageInfo->addr0);
	//dprintk("%s enable:%d, layer:%d, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", __func__, ImageInfo->enable, ImageInfo->Lcdc_layer,
	//		ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);

	if(ImageInfo->fmt >= TCC_LCDC_IMG_FMT_UYVY && ImageInfo->fmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)
	{
		VIOC_RDMA_SetImageY2REnable(pRDMABase0, true);
		VIOC_RDMA_SetImageY2RMode(pRDMABase0, 0); /* Y2RMode Default 0 (Studio Color) */

		if( ImageInfo->Frame_width <= RDMA_UVI_MAX_WIDTH )
			VIOC_RDMA_SetImageUVIEnable(pRDMABase0, true);
		else
			VIOC_RDMA_SetImageUVIEnable(pRDMABase0, false);
	}

	VIOC_RDMA_SetImageOffset(pRDMABase0, ImageInfo->fmt, ImageInfo->Frame_width);
	VIOC_RDMA_SetImageFormat(pRDMABase0, ImageInfo->fmt);

	VIOC_WMIX_SetPosition(pWMIXBase, 2, 0, 0);
	VIOC_WMIX_SetPosition(pWMIXBase, 3, 0, 0);

	VIOC_RDMA_SetImageSize(pRDMABase0, ImageInfo->Image_width, ImageInfo->Image_height);
	VIOC_RDMA_SetImageBase(pRDMABase0, ImageInfo->addr0, ImageInfo->addr1, ImageInfo->addr2);		

	VIOC_RDMA_SetImageIntl(pRDMABase0, false);

	if(ImageInfo->fmt >= TCC_LCDC_IMG_FMT_UYVY && ImageInfo->fmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)
	{
		VIOC_RDMA_SetImageY2REnable(pRDMABase1, true);
		VIOC_RDMA_SetImageY2RMode(pRDMABase1, 0); /* Y2RMode Default 0 (Studio Color) */

		if( ImageInfo->Frame_width <= RDMA_UVI_MAX_WIDTH )
			VIOC_RDMA_SetImageUVIEnable(pRDMABase1, true);
		else
			VIOC_RDMA_SetImageUVIEnable(pRDMABase1, false);

	}

	VIOC_RDMA_SetImageOffset(pRDMABase1, ImageInfo->fmt, ImageInfo->Frame_width);
	VIOC_RDMA_SetImageFormat(pRDMABase1, ImageInfo->fmt);

	// position
	VIOC_WMIX_SetPosition(pWMIXBase, ImageInfo->Lcdc_layer, 0, ImageInfo->Frame_height + iVBlank);

	VIOC_RDMA_SetImageSize(pRDMABase1, ImageInfo->Image_width, ImageInfo->Image_height);
	VIOC_RDMA_SetImageBase(pRDMABase1, ImageInfo->MVC_Base_addr0, ImageInfo->MVC_Base_addr1, ImageInfo->MVC_Base_addr2);

	VIOC_RDMA_SetImageIntl(pRDMABase1, false);

	VIOC_WMIX_SetBGColor(pWMIXBase, 0x00, 0x00, 0x00, 0x00);

	VIOC_RDMA_SetImageEnable(pRDMABase0);
	VIOC_RDMA_SetImageEnable(pRDMABase1);

	VIOC_WMIX_SetUpdate(pWMIXBase);
}

void tca_fb_rdma_pandisplay(unsigned int layer, unsigned int base_addr, struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	int scN = 0;
	unsigned int  fmt = 0;
	unsigned int lcd_width, lcd_height, img_width, img_height;

	unsigned int alpha_type = 0, alpha_blending_en = 0;
	unsigned int chromaR, chromaG, chromaB, chroma_en = 0;
	PVIOC_SC pSC;

	PVIOC_RDMA pRDMA = (PVIOC_RDMA)pdp_data->rdma_info[layer].virt_addr;
	PVIOC_WMIX pWMIX = (PVIOC_WMIX)pdp_data->wmixer_info.virt_addr;
	
	if(var->bits_per_pixel == 32) 	{
		chroma_en = UI_CHROMA_EN;
		alpha_type = 1;
		alpha_blending_en = 1;
		fmt = TCC_LCDC_IMG_FMT_RGB888;
	}
	else if(var->bits_per_pixel == 16)	{
		chroma_en = 1;
		alpha_type = 0;
		alpha_blending_en = 0;
		fmt = TCC_LCDC_IMG_FMT_RGB565; 
	}
	else	{
		pr_err("%s: bits_per_pixel : %d Not Supported BPP!\n", __FUNCTION__, var->bits_per_pixel);
	}

	chromaR = chromaG = chromaB = 0;

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);

	img_width = var->xres;
	img_height = var->yres;

	scN = tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[layer].blk_num);

	if(scN < 0)	{
		if(img_width > lcd_width)	
			img_width = lcd_width;
		
		if(img_height > lcd_height)	
			img_height = lcd_height;
	}
	else
	{
		pSC = (VIOC_SC *)VIOC_SC_GetAddress(scN);
		if (pSC != NULL) {
			if((lcd_height == img_height) && (lcd_width == img_width))
				VIOC_SC_SetBypass (pSC, ON);
			else
				VIOC_SC_SetBypass (pSC, OFF);

			VIOC_SC_SetDstSize (pSC, lcd_width, lcd_height);			// set destination size in scaler
			VIOC_SC_SetOutSize (pSC, lcd_width, lcd_height);			// set output size in scaer
			VIOC_SC_SetUpdate (pSC);
		}
	}


	// default framebuffer 
	VIOC_WMIX_SetPosition(pWMIX, layer, 0, 0);
	//overlay setting
	#ifdef CONFIG_ARCH_TCC898X
	VIOC_WMIX_SetChromaKey(pWMIX, layer, chroma_en, chromaR, chromaG, chromaB, 0x3FF, 0x3FF, 0x3FF);
	#else
	VIOC_WMIX_SetChromaKey(pWMIX, layer, chroma_en, chromaR, chromaG, chromaB, 0xF8, 0xFC, 0xF8);
	#endif//


	VIOC_RDMA_SetImageFormat(pRDMA, fmt);					//fmt

	VIOC_RDMA_SetImageOffset(pRDMA, fmt, img_width);		//offset	
	VIOC_RDMA_SetImageSize(pRDMA, img_width, img_height);	//size	
	VIOC_RDMA_SetImageBase(pRDMA, base_addr, 0, 0);

	VIOC_RDMA_SetImageAlphaSelect(pRDMA, alpha_type);
	VIOC_RDMA_SetImageAlphaEnable(pRDMA, alpha_blending_en);

	VIOC_WMIX_SetUpdate(pWMIX);	
	VIOC_RDMA_SetImageEnable(pRDMA);

}

void tccfb1_set_par(struct tccfb_info *fbi,  struct fb_var_screeninfo *var)
{
	int ret;
	unsigned int lcd_width, lcd_height, rdmaN, en_rdma;
	 struct tcc_dp_device *pdp_data = &fbi->pdata.Mdp_data;

	VIOC_DISP_GetSize(pdp_data->ddc_info.virt_addr, &lcd_width, &lcd_height);

	if(fbi->fb->node == 0)
		rdmaN = RDMA_FB;
	else if(fbi->fb->node == 1)
		rdmaN = RDMA_FB1;
	else
		rdmaN = RDMA_FB;

	if((lcd_width != var->xres) || (lcd_height != var->yres))
	{
		//scaler plug in check
		if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[rdmaN].blk_num) < 0)
		{
			VIOC_PlugInOutCheck VIOC_PlugIn;

			ret = VIOC_CONFIG_Device_PlugState(VIOC_SC0, &VIOC_PlugIn);
			VIOC_RDMA_GetImageEnable(pdp_data->rdma_info[rdmaN].virt_addr, &en_rdma);

			if (ret == VIOC_DEVICE_CONNECTED) {
				if (!en_rdma && !VIOC_PlugIn.enable) {
					pr_info("fb %d : display scaler 1 plug in \n", fbi->fb->node);
					VIOC_CONFIG_PlugIn(VIOC_SC0, pdp_data->rdma_info[rdmaN].blk_num);
				} else {
					pr_err("ERR fb %d : scaler 1 plug in error RDMA enable:%d  SCALER enable:%d\n",
						fbi->fb->node, en_rdma ,VIOC_PlugIn.enable);
				}
			}
		}
	}

}

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
void tcc_video_rdma_off(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo, int outputMode, int interlaced)
{
	unsigned int iSCType = 0;
	volatile PVIOC_IREQ_CONFIG pIREQConfig;

	iSCType = tca_get_scaler_num(pdp_data->DispDeviceType, ImageInfo->Lcdc_layer);
	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)of_iomap(ViocConfig_np, 0);

	VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[ImageInfo->Lcdc_layer].virt_addr);	
	//map converter 
	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	{
		unsigned int MC_NUM;
		if(tca_get_MC_Connect_to_RDMA(&MC_NUM, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num))		{
			tca_map_convter_onoff(MC_NUM, 0);
			tca_map_convter_wait_done(MC_NUM);

			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);			
		}
	}
	#endif//
	if( !interlaced && (outputMode == TCC_OUTPUT_LCD || outputMode == TCC_OUTPUT_HDMI) )
	{
		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_RDMA, pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num, VIOC_CONFIG_CLEAR);

		if(tca_get_RDMA_PlugIn_to_Scaler(pdp_data->rdma_info[ImageInfo->Lcdc_layer].blk_num) != -1) //scaler plug in status check
		{
			pr_info("tcc_video_rdma_off for last-frame :: scaler %u plug out \n", iSCType);
			VIOC_CONFIG_PlugOut(iSCType);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(pIREQConfig, VIOC_CONFIG_SCALER, iSCType, VIOC_CONFIG_CLEAR);
		}
	}
#if defined(CONFIG_FB_TCC_COMPOSITE)
	else if(outputMode == TCC_OUTPUT_COMPOSITE){
		#if defined(CONFIG_ANDROID) || defined(CONFIG_PLATFORM_AVN)
		tcc_plugout_for_composite(RDMA_VIDEO);
		#else
		tcc_plugout_for_composite(ImageInfo->Lcdc_layer);//linux stb
		#endif//
	}
#endif
#if defined(CONFIG_FB_TCC_COMPONENT)
	else if(outputMode == TCC_OUTPUT_COMPONENT){
		#if defined(CONFIG_ANDROID) || defined(CONFIG_PLATFORM_AVN)
		tcc_plugout_for_component(RDMA_VIDEO);
		#else
		tcc_plugout_for_component(ImageInfo->Lcdc_layer); //linux stb
		#endif//
	}
#endif
}
#endif



void tca_fb_vsync_activate(struct tcc_dp_device *pdata)
{
	#ifdef CONFIG_FB_TCC_USE_VSYNC_INTERRUPT
	VIOC_DISP_SetStatus(pdata->ddc_info.virt_addr, 0x3F);
	pdata->pandisp_sync.state = 0;
	vioc_intr_enable(pdata->DispNum, VIOC_DISP_INTR_DISPLAY);
//	VIOC_DISP_SetStatus(fbi->pdata.Mdp_data.ddc_info.virt_addr, 0x3F);
//	fbi->pdata.Mdp_data.pandisp_sync.state = 0;
//	tca_lcdc_interrupt_onoff(true, fbi->pdata.lcdc_number);
	#endif//
}
EXPORT_SYMBOL(tca_fb_vsync_activate);

void tca_fb_wait_for_vsync(struct tcc_dp_device *pdata)
{
	#ifdef CONFIG_FB_TCC_USE_VSYNC_INTERRUPT
	int ret = 1;

	tca_fb_vsync_activate(pdata);
	if(pdata->pandisp_sync.state == 0)
		ret = wait_event_interruptible_timeout(pdata->pandisp_sync.waitq, pdata->pandisp_sync.state == 1, msecs_to_jiffies(50));

	if(!ret)	{
	 	printk("  [%s %d]: tcc_setup_interrupt timed_out!! \n",__func__, ret);
	}
	#endif //CONFIG_FB_TCC_USE_VSYNC_INTERRUPT
}
EXPORT_SYMBOL(tca_fb_wait_for_vsync);



/* tccfb_activate_var
 * activate (set) the controller from the given framebuffer
 * information
*/
void tca_fb_activate_var(unsigned int dma_addr,  struct fb_var_screeninfo *var, struct tcc_dp_device *pdp_data)
{
	
	switch(pdp_data->FbUpdateType)
	{
		case FB_RDMA_UPDATE:
			tca_fb_rdma_active_var(dma_addr, var, pdp_data);
			break;

		case FB_SC_RDMA_UPDATE:
			tca_fb_sc_rdma_active_var(dma_addr, var, pdp_data);
			break;

		case FB_SC_M2M_RDMA_UPDATE:
			tca_fb_sc_m2m_rdma_active_var(dma_addr, var, pdp_data);
			break;

		case FB_SC_G2D_RDMA_UPDATE:
			tca_fb_sc_g2d_rdma_active_var(dma_addr, var, pdp_data);
			break;

		case FB_DIVIDE_UPDATE:
			tca_fb_divide_active_var(dma_addr, var, pdp_data);
			break;
			
		case FB_MVC_UPDATE:
			tca_fb_mvc_active_var(dma_addr, var, pdp_data);
			break;
			
		default:
			break;
	}
	
	pdp_data->FbBaseAddr = dma_addr;

	if(pdp_data->FbPowerState)
		VIOC_DISP_TurnOn(pdp_data->ddc_info.virt_addr);

	#if defined(CONFIG_TCC_DISPLAY_MODE_USE) && !defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT)|| defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
		tca_fb_attach_update_flag();
	#endif
}
EXPORT_SYMBOL(tca_fb_activate_var);


/* tcafb_activate_var
 *
 * activate (set) the controller from the given framebuffer
 * information
*/
void tcafb_activate_var(struct tccfb_info *fbi,  struct fb_var_screeninfo *var)
{
	unsigned int BaseAddr;

 	BaseAddr = fbi->map_dma + var->xres * var->yoffset * (var->bits_per_pixel/8);

	if(fbi->pdata.Mdp_data.FbPowerState)
		tca_fb_activate_var(BaseAddr, var, &fbi->pdata.Mdp_data);

	#if !defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
		#if !defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_MID) && !defined(CONFIG_TCC_DISPLAY_MODE_USE)
		if(fbi->pdata.Sdp_data.FbPowerState)
			tca_fb_activate_var(BaseAddr, var, &fbi->pdata.Sdp_data);
		#endif
	#endif
}


/* tccfb_pan_display
 *
 * pandisplay (set) the controller from the given framebuffer
 * information
*/
int tca_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct tccfb_info *fbi = info->par;
	int ret = 0;

	/* activate this new configuration */
   	if(info->node == 0)	{

		if((fbi->pdata.Mdp_data.FbPowerState)
			#if defined(CONFIG_USE_DISPLAY_FB_LOCK)
			&& (!fb_lock)
			#endif
		)
		{
		
			tcafb_activate_var(fbi, var);
			tca_fb_vsync_activate(&fbi->pdata.Mdp_data);

			#ifdef CONFIG_FB_TCC_USE_VSYNC_INTERRUPT
			if(fbi->pdata.Mdp_data.pandisp_sync.state == 0)
				ret = wait_event_interruptible_timeout(fbi->pdata.Mdp_data.pandisp_sync.waitq, fbi->pdata.Mdp_data.pandisp_sync.state == 1, msecs_to_jiffies(50));

			if(!ret)	{
			 	printk("  [%s %d]: tcc_setup_interrupt timed_out!! \n",__func__, ret);
			}
			#endif //CONFIG_FB_TCC_USE_VSYNC_INTERRUPT

		}
	}
	else
	{
		unsigned int layer, base_addr;

		base_addr = fbi->map_dma + var->xres * var->yoffset * (var->bits_per_pixel/8);

		if(info->node == 1)
			layer = RDMA_FB1;
		else
			return -1;

		tca_fb_rdma_pandisplay(RDMA_FB1,base_addr, var, &fbi->pdata.Mdp_data);
	}
	return ret;
}
EXPORT_SYMBOL(tca_fb_pan_display);



static volatile VIOC_RDMA pRDMA_BackUp;
static volatile VIOC_WMIX pWMIX_BackUp;



#ifdef CONFIG_PM_RUNTIME
void tca_fb_runtime_suspend(struct tccfb_info *fbi)
{
	int ret;
	struct tcc_dp_device *pdp_data = &fbi->pdata.Mdp_data;

	pr_info("### %s:  \n", __FUNCTION__);
 


	#ifdef CONFIG_ANDROID
	VIOC_RDMA_SetImageDisable(pdp_data->rdma_info[RDMA_FB].virt_addr);
	#else
	VIOC_RDMA_SetImageDisableNW(pdp_data->rdma_info[RDMA_FB].virt_addr);
	VIOC_DISP_TurnOff(pdp_data->ddc_info.virt_addr);
	#endif//
	pdp_data->disp_dd_sync.state = 0;
	ret = wait_event_interruptible_timeout(pdp_data->disp_dd_sync.waitq, pdp_data->disp_dd_sync.state == 1, msecs_to_jiffies(30));
	if(ret == 0) 
		pr_info("%s:  DISABLE DONE wait for  ms \n", __FUNCTION__);	
	
	vioc_intr_disable(fbi->pdata.lcdc_number, VIOC_DISP_INTR_DISPLAY);
	
	clk_disable_unprepare(pdp_data->vioc_clock);
	clk_disable_unprepare(pdp_data->ddc_clock);
}
EXPORT_SYMBOL(tca_fb_runtime_suspend);
#endif
void tca_fb_runtime_resume(struct tccfb_info *fbi)
{
	struct tcc_dp_device *pdp_data = &fbi->pdata.Mdp_data;

	printk("### %s:  \n", __FUNCTION__);
		
	clk_prepare_enable(pdp_data->vioc_clock);
	clk_prepare_enable(pdp_data->ddc_clock);

	vioc_intr_enable(fbi->pdata.lcdc_number, VIOC_DISP_INTR_DISPLAY);
	tca_fb_vsync_activate(pdp_data);
}
EXPORT_SYMBOL(tca_fb_runtime_resume);


/* suspend and resume support for the lcd controller */
int tca_fb_suspend(struct device *dev)
{
	pr_info("FB: %s: \n", __FUNCTION__);
	return 0;
}
EXPORT_SYMBOL(tca_fb_suspend);

int tca_fb_resume(struct device *dev)
{
	struct platform_device *fb_device = container_of(dev, struct platform_device, dev);
	struct tccfb_info	   *fbi = platform_get_drvdata(fb_device);

	#if defined(CONFIG_PLATFORM_STB) && !defined(CONFIG_ANDROID)
	tca_fb_runtime_resume(fbi);
	#endif

	#if defined(CONFIG_HIBERNATION)
	fb_quickboot_resume(fbi);
	#endif
	
	return 0;
}
EXPORT_SYMBOL(tca_fb_resume);


void tca_vsync_enable(struct tccfb_info *dev, int on)
{
	dprintk("## VSYNC(%d)", on);

	dev->active_vsync = on;
}
EXPORT_SYMBOL(tca_vsync_enable);

int tca_fb_init(struct tccfb_info *fbi)
{
	printk(KERN_INFO " %s (built)\n", __func__);
	tca_fb_mem_scale_init();

	ViocScaler_np = of_find_compatible_node(NULL, NULL, "telechips,scaler");
	if(ViocScaler_np == NULL)
		pr_err("cann't find scaler\n");
	
	ViocConfig_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_config");
	if(ViocConfig_np == NULL)
		pr_err("cann't find vioc config \n");
		
	clk_prepare_enable(fbi->pdata.Mdp_data.vioc_clock);
	clk_prepare_enable(fbi->pdata.Mdp_data.ddc_clock);
	
	//initialize the buffers for mouse cursor
	{
		dma_addr_t	Gmap_dma;	/* physical */
		u_char *	Gmap_cpu;	/* virtual */

		Gmap_cpu = dma_alloc_writecombine(0, MOUSE_CURSOR_BUFF_SIZE, &Gmap_dma, GFP_KERNEL);
		memset(Gmap_cpu, 0x00, MOUSE_CURSOR_BUFF_SIZE);

		mouse_data.pbuf1 = (char *)Gmap_dma;
		mouse_data.pbuf2 = (char *)(Gmap_dma + MOUSE_CURSOR_BUFF_SIZE/2);
		mouse_data.vbuf1 = (char *)Gmap_cpu;
		mouse_data.vbuf2 = (char *)(Gmap_cpu + MOUSE_CURSOR_BUFF_SIZE/2);
		mouse_data.index = 0;
	}

	memset((void *)&resize_data, 0x00, sizeof(tcc_display_resize));

	attach_intr.vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!attach_intr.vioc_intr)
		return -ENOMEM;

#if defined(CONFIG_PLATFORM_STB) && !defined(CONFIG_ANDROID)
        {
        	VIOC_DISP *pDISP = fbi->pdata.Mdp_data.ddc_info.virt_addr;
        	VIOC_RDMA *pRDMA = fbi->pdata.Mdp_data.rdma_info[RDMA_FB].virt_addr;
        	VIOC_DISP_TurnOff(pDISP);
        	VIOC_RDMA_SetImageDisable(pRDMA);
        }
#endif

	return 0;
}
EXPORT_SYMBOL(tca_fb_init);

void tca_fb_exit(void)
{
	pr_info(" %s \n", __func__);

#if defined(CONFIG_HIBERNATION) && defined(CONFIG_USING_LAST_FRAMEBUFFER)
	fb_quickboot_lastframe_display_release();
#endif
}
EXPORT_SYMBOL(tca_fb_exit);
