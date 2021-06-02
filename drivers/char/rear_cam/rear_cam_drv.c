
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
#include <linux/err.h>
#include <linux/irq.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif


#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <mach/bsp.h>
#include <mach/tca_lcdc.h>

#include <linux/cpufreq.h>
#include <linux/wait.h>
#include <linux/kthread.h>

#include <soc/tcc/pmap.h>

#include <mach/vioc_ireq.h>

#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_global.h>
#include <mach/vioc_vin.h>
#include <mach/vioc_viqe.h>
#include <mach/vioc_config.h>
#include <mach/vioc_api.h>
#include <mach/vioc_intr.h>

#if defined(CONFIG_TCC_REAR_CAMERA_CPU_WITH_OVERSCAN)
#include <mach/vioc_scaler.h>
#endif
#include <mach/tcc_fb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_scaler.h>

//#include <asm/system.h>


#include <linux/gpio.h>
#include <mach/io.h>
#include <mach/bsp.h>
#include "tcc_hw_event.h"
#include "rear_cam_drv.h"


#include <video/tcc/viocmg.h>

#define REAR_CAM_DEBUG 		       0
#define dprintk(format, msg...)         if(REAR_CAM_DEBUG) { printk("[REAR_CAM] "format"", ##msg); }
#define dfnprintk(format, msg...)       if(REAR_CAM_DEBUG) { printk("[REAR_CAM] %s: "format"", __func__,  ##msg); }

#define CAMERA_PREVIEW_WIDTH            720
#define CAMERA_PREVIEW_HEIGHT           240

//#define CONFIG_TCC_REAR_CAMERA_USE_SCALER

static struct work_struct rearcam_work_q;


DECLARE_WAIT_QUEUE_HEAD(rear_event_wait);

void RCAM_WDMASetHandler( char onoff);
void RCAM_ReConfigVIOC(void);

static int RCAM_CheckAliveThread(void *arg);
static int handover = 0;

static int RCAM_parse_camear_hardware(struct device_node *np)
{
        int result = -1;
        unsigned int val;
        enum of_gpio_flags flags;
        struct device_node *np_camera, *np_phandle;

        dprintk("RCAM_parse_camear_hardware\r\n");
        
        np_camera = of_parse_phandle(np, "telechips,rear-camera", 0);
        if(np_camera < 1) {
                pr_err("count not find telechips,rear-camera\r\n");
                result = -ENODEV;
                goto END_PROCESS;                
        }
        
        // RDMA
        result = of_property_read_u32_index(np,"telechips,in_wmix_rdma",1,&val);
        if(result) {
                result = -ENODEV;
                pr_err( "could not find  index from in_wmix_rdma\n");
                goto END_PROCESS;
        }
        np_phandle = of_parse_phandle(np, "telechips,in_wmix_rdma", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_port\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_wmix_rdma = (unsigned int)of_iomap(np_phandle, val);

        // display rdma..!!
        rear_cam_soc->reg.lcd_wmix_rdma = (unsigned int)of_iomap(np_phandle, viocmg_get_rear_cam_display_rdma_id());
        
        // VIN
        np_phandle = of_parse_phandle(np_camera, "camera_videoin", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_videoin (%d)\n", np_phandle);
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_wmix_cam = of_iomap(np_phandle, viocmg_get_rear_cam_vin_id());

        
        // VIQE
        np_phandle = of_parse_phandle(np_camera, "camera_viqe", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_viqe\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_viqe = of_iomap(np_phandle, 0);

        // SCALER
        np_phandle = of_parse_phandle(np_camera, "camera_scaler", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_viqe\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_scaler = of_iomap(np_phandle, viocmg_get_rear_cam_scaler_id());

        // WMIXER
        np_phandle = of_parse_phandle(np_camera, "camera_wmixer", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_wmixer\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_wmix = of_iomap(np_phandle, viocmg_get_rear_cam_vin_wmix_id());
        
        // WDMA
        np_phandle = of_parse_phandle(np_camera, "camera_wdma", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_wdma\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.in_wmix_wdma = of_iomap(np_phandle, viocmg_get_rear_cam_vin_wdma_id());
        dprintk(" >> in_wmix_wdma = 0x%x\r\n", rear_cam_soc->reg.in_wmix_wdma);

        // WDMA IRQ
        rear_cam_soc->in_wmix_wdma_irq = irq_of_parse_and_map(np_phandle, viocmg_get_rear_cam_vin_wdma_id());
        

        // CONFIG
        np_phandle = of_parse_phandle(np_camera, "camera_config", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find camera_config\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.vioc_config = of_iomap(np_phandle, 0);



        // CAMERA PORT MAPPING
        np_phandle = of_parse_phandle(np_camera, "camera_port", 0);
        if(np_phandle < 1) {
                result = -ENODEV;
                pr_err( "could not find ddicfg\n");
                goto END_PROCESS;
        }
        rear_cam_soc->reg.camera_port = of_iomap(np_phandle, 0);


        result = of_property_read_u32_index(np_camera,"camera_port",1,&val);
        rear_cam_soc->camera_port = val;

                
        rear_cam_soc->gear_port = of_get_named_gpio_flags(np, "gear-gpios", 0, &flags);
        if(gpio_is_valid(rear_cam_soc->gear_port)) {
				result = of_property_read_u32(np,"telechips,gear_port_active_level",&val);
				if(result) { pr_err("cann't parse telechips,gear_port_active_level\r\n"); goto END_PROCESS; }
				rear_cam_soc->gear_port_active_level = val;

                dprintk(" >>rear cam gear_port = 0x%x, cam gear_port_active_level=%d\r\n", rear_cam_soc->gear_port, rear_cam_soc->gear_port_active_level);
                result = gpio_request(rear_cam_soc->gear_port, "Rear Camera IRQ");
                if(result) {
                        dev_err(rear_cam_soc->dev, "error mapping gear port gpio\r\n");
                        goto END_PROCESS;
                }
                // need gpio_free

                result = gpio_direction_input(rear_cam_soc->gear_port);
                if(result) {
                        dev_err(rear_cam_soc->dev, "error input setting to gear port gpio\r\n");
                        goto END_PROCESS;
                }
                
                rear_cam_soc->gear_port_irq = gpio_to_irq(rear_cam_soc->gear_port);
        }
        else {
                result = -ENODEV;
                pr_err( "invalid gpio port\n");
                goto END_PROCESS;
        }        
        
        dprintk(" >>rear cam gear_port_active_level = %d, irq=0x%x\r\n", rear_cam_soc->gear_port_active_level, rear_cam_soc->gear_port_irq);
        
        result = 0;
        
END_PROCESS:
        return result;
}



static int RCAM_parse_camear_misc(struct device_node *np)
{
        int result = -1;
        unsigned int val;
        enum of_gpio_flags flags;

        dprintk("RCAM_parse_camear_misc\r\n");

        result = of_property_read_u32(np,"telechips,preview_x",&val);
        if(result) { pr_err("cann't parse telechips,preview_x\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_x = val;


        result = of_property_read_u32(np,"telechips,preview_y",&val);
        if(result) { pr_err("cann't parse telechips,preview_y\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_y = val;

        result = of_property_read_u32(np,"telechips,preview_width",&val);
        if(result) { pr_err("cann't parse telechips,preview_width\r\n"); goto END_PROCESS; }
        rear_cam_soc->lcd_width = rear_cam_soc->preview_width = rear_cam_soc->draw_preview_width = val;

        result = of_property_read_u32(np,"telechips,preview_height",&val);
        if(result) { pr_err("cann't parse telechips,preview_height\r\n"); goto END_PROCESS; }
        rear_cam_soc->lcd_height = rear_cam_soc->preview_height = rear_cam_soc->draw_preview_height = val;

        result = of_property_read_u32(np,"telechips,preview_format",&val);
        if(result) { pr_err("cann't parse telechips,preview_format\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_format = val;

        result = of_property_read_u32(np,"telechips,preview_additional_width",&val);
        if(result) { pr_err("cann't parse telechips,preview_additional_width\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_additional_width = val;

        result = of_property_read_u32(np,"telechips,preview_additional_height",&val);
        if(result) { pr_err("cann't parse telechips,preview_additional_height\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_additional_height = val;

	result = of_property_read_u32(np,"telechips,preview_crop_x",&val);
        if(result) { pr_err("cann't parse telechips,preview_crop_x\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_crop_x = val;//rear_cam_soc->preview_x = val;

	  result = of_property_read_u32(np,"telechips,preview_crop_y",&val);
        if(result) { pr_err("cann't parse telechips,preview_crop_y\r\n"); goto END_PROCESS; }
        rear_cam_soc->preview_crop_y = val;//rear_cam_soc->preview_x = val;

        result = of_property_read_u32(np,"telechips,parking_line_x",&val);
        if(result) { pr_err("cann't parse telechips,parking_line_x\r\n"); goto END_PROCESS; }
        rear_cam_soc->parking_line_x = val;

        result = of_property_read_u32(np,"telechips,parking_line_y",&val);
        if(result) { pr_err("cann't parse telechips,parking_line_y\r\n"); goto END_PROCESS; }
        rear_cam_soc->parking_line_y = val;

        result = of_property_read_u32(np,"telechips,parking_line_width",&val);
        if(result) { pr_err("cann't parse telechips,parking_line_width\r\n"); goto END_PROCESS; }
        rear_cam_soc->parking_line_width = rear_cam_soc->draw_parking_line_width = val;

        result = of_property_read_u32(np,"telechips,parking_line_height",&val);
        if(result) { pr_err("cann't parse telechips,parking_line_height\r\n"); goto END_PROCESS; }
        rear_cam_soc->parking_line_height = rear_cam_soc->draw_parking_line_height = val;

        result = of_property_read_u32(np,"telechips,parking_line_format",&val);
        if(result) { pr_err("cann't parse telechips,parking_line_format\r\n"); goto END_PROCESS; }
        rear_cam_soc->parking_line_format = val;

        dprintk(" >> parking line size(%dx%d)\r\n", rear_cam_soc->parking_line_width, rear_cam_soc->parking_line_height);

        
END_PROCESS:
        return result;

}


static int RCAM_Parse_DT(struct platform_device *pdev)
{
        int ret;

        ret = RCAM_parse_camear_hardware(pdev->dev.of_node);
        if(ret < 0) { goto END_PROCESS;}
        ret = RCAM_parse_camear_misc(pdev->dev.of_node);
        
END_PROCESS:

        return ret;
}

static int RCAM_Process_Pmap(struct platform_device *pdev)
{
        
        int ret;        
        pmap_t pmap_data;
        unsigned int pg_pffset;

        ret = pmap_get_info("rearcamera", &pmap_data);

        if(ret && pmap_data.size > 0) {
                dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
                        pmap_data.name, pmap_data.base, pmap_data.base + pmap_data.size, pmap_data.size);
        }
        else {
                printk("error empty memory for rear-camera\r\n");
                ret = -ENODEV;
                goto END_PROCESS;
        }

        pg_pffset =  (((rear_cam_soc->preview_width * rear_cam_soc->preview_height) << 2) + 4095) & (~4095);
        
        rear_cam_soc->mPreviewBufAddr[0] = pmap_data.base;
        rear_cam_soc->mPreviewBufAddr[1] = pmap_data.base +pg_pffset;

        if(pmap_data.size < (pg_pffset << 1)) {
                pr_err(" >> %s  preview buffer is small %d, need %d bytes  \n", __func__,pmap_data.size,(pg_pffset << 1));  
                ret = -ENODEV;
                goto END_PROCESS;
        }

        ret = pmap_get_info("parking_gui", &pmap_data);
        if( ret && (pmap_data.size > 0)){
                dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
                        pmap_data.name, pmap_data.base, pmap_data.base + pmap_data.size, pmap_data.size);

		  pg_pffset = ((pmap_data.size/2) + 4095) & (~4095); //(((rear_cam_soc->parking_line_width * rear_cam_soc->parking_line_height) << 2) + 4096) & (~4095);
                rear_cam_soc->mLineBufAddr[0] = pmap_data.base;
                rear_cam_soc->mLineBufAddr[1] = pmap_data.base+pg_pffset;
                if(pmap_data.size < (pg_pffset << 1)){
                        dprintk(" >> %s  parking-guied-line buffer is small %d, need %d bytes  \n", __func__,pmap_data.size,(pg_pffset << 1));                           
                }
                
                dprintk("%s  rear_cam_soc->mPreviewBufAddr 0x%x 0x%x  \n", __func__,
                        (unsigned int)rear_cam_soc->mPreviewBufAddr[0],(unsigned int)rear_cam_soc->mPreviewBufAddr[1]);
                dprintk("%s  rear_cam_soc->mLineBufAddr 0x%x 0x%x  \n", __func__,
                        (unsigned int)rear_cam_soc->mLineBufAddr[0],(unsigned int)rear_cam_soc->mLineBufAddr[1]);

        }
        else {
                pr_err("error..!! empty parking gui memory..!!\r\n");
                ret = -ENODEV;
                goto END_PROCESS;
        }

        if(viocmg_is_feature_rear_cam_use_viqe()) {
                // bootloader need viqe memory when viqe feature is enabled.
                ret = pmap_get_info("rearcamera_viqe", &pmap_data);
                if(ret) {
                        dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
                                pmap_data.name, pmap_data.base, pmap_data.base + pmap_data.size, pmap_data.size);
                        pg_pffset = (CAMERA_PREVIEW_WIDTH * CAMERA_PREVIEW_HEIGHT) << 3;
                        if(pmap_data.size < pg_pffset) {
                                pr_err(" >> %s  preview buffer is small %d, need %d bytes  \n", __func__, pmap_data.size,(pg_pffset));   
                                ret  = -ENODEV;
                                goto END_PROCESS;
                        }
                        rear_cam_soc->mViqeMemory =  pmap_data.base;
                }
                else {
                        pr_err("error..!! empty viqe memory..!!\r\n");
                        ret = -ENODEV;
                        goto END_PROCESS;
                }
        }
        ret = 0;
        
END_PROCESS:
        return ret;
}

static void RCAM_Show(long OnOff, unsigned int booting_mode)
{
        VIOC_RDMA *pRDMA;
        mutex_lock(&rear_cam_soc->mShowMutex);

        pRDMA = (VIOC_RDMA *)rear_cam_soc->reg.lcd_wmix_rdma;

        dprintk("+RCAM_Show\r\n");

        if(booting_mode == RCAM_COLD_BOOT || handover == 0) {
#if defined (USING_EARLY_CAM_CM4)
//				if(OnOff){
					dprintk("RCAM_Show RCAM_COLD_BOOT ENABLE\r\n");
					viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_PREPARE);
					viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
//				}
//				else {
//					viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_STOP);
//				}
#else
				if(OnOff){
						if(viocmg_get_rear_cam_mode() > VIOCMG_REAR_MODE_STOP) {
								dprintk("RCAM_Show RCAM_COLD_BOOT ENABLE\r\n");
								viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_PREPARE);
								viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
						}
						else {
								dprintk("RCAM_Show RCAM_COLD_BOOT DISABLE\r\n");
								viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_PREPARE);
								RCAM_ReConfigVIOC();
								RCAM_PreviewSetPosition(rear_cam_soc->preview_x, rear_cam_soc->preview_y);
								RCAM_SetPreviewVIOC();
								viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
						}
				}
				else {
						RCAM_ReleaseVideo();
						viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_STOP);
						VIOC_RDMA_SetImageDisable(pRDMA);
				}
#endif
        }
        else {
                if(OnOff){
                        if(viocmg_get_rear_cam_mode()==VIOCMG_REAR_MODE_STOP) {
                                viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_PREPARE);
                                RCAM_ReConfigVIOC();
                                RCAM_PreviewSetPosition(rear_cam_soc->preview_x, rear_cam_soc->preview_y);
                                RCAM_SetPreviewVIOC();
                                viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
                        }
                }
                else {
                        if(viocmg_get_rear_cam_mode() > VIOCMG_REAR_MODE_STOP) {
                                RCAM_ReleaseVideo();
                                viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_STOP);
                                VIOC_RDMA_SetImageDisable(pRDMA);
                        }
                }     
        }
        dprintk("-RCAM_Show\r\n");
        mutex_unlock(&rear_cam_soc->mShowMutex);
}

static unsigned int RCAM_GetGearStatus(void)
{
        unsigned int val = 0;
        unsigned int gear_status = 0;
                
        gear_status = gpio_get_value(rear_cam_soc->gear_port);
        if(rear_cam_soc->gear_port_active_level == gear_status) {
                val = 1;
        }
        else {
                val = 0;
        }
        return val;
}


static unsigned int RCAM_GetDmaStatus(void)
{
        unsigned int vin_enable, wdma_enable ;

        vin_enable = VIOC_VIN_IsEnable((VIOC_VIN *)rear_cam_soc->reg.in_wmix_cam);
        wdma_enable = VIOC_WDMA_IsImageEnable((VIOC_WDMA *)rear_cam_soc->reg.in_wmix_wdma);
        return (vin_enable && wdma_enable );
}


void RCAM_OnOff(long OnOff)
{
        if (OnOff ){
                rear_cam_soc->mRearEventStatus = 1;
        }
        else{
                rear_cam_soc->mRearEventStatus = 0;
        }

        RCAM_Show(rear_cam_soc->mRearEventStatus, RCAM_AFTER_BOOT);
}


void RCAM_LINE_OnOff(long OnOff)
{
	VIOC_RDMA *parkingline_rdma;
	VIOC_WMIX *vin_wmix;

	unsigned int layer          = 0x0;
	unsigned int key_R          = 0xFF;
	unsigned int key_G          = 0xFF;
	unsigned int key_B          = 0xFF;
	unsigned int key_mask_R     = 0xF8;
	unsigned int key_mask_G     = 0xF8;
	unsigned int key_mask_B     = 0xF8;

	unsigned int buffer_index = rear_cam_soc->mLineBufIndex;

	if(viocmg_is_feature_rear_cam_use_parking_line() &&
                rear_cam_soc->mLineBufAddr[buffer_index] != 0){
		parkingline_rdma = (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
		vin_wmix = (VIOC_WMIX*)rear_cam_soc->reg.in_wmix;

                mutex_lock(&rear_cam_soc->mMutex);
                if (OnOff ){
                        rear_cam_soc->mLineBufEnabled = 1;
                        VIOC_RDMA_SetImageFormat(parkingline_rdma, rear_cam_soc->parking_line_format);
                        VIOC_RDMA_SetImageSize(parkingline_rdma, rear_cam_soc->parking_line_width, rear_cam_soc->parking_line_height);
                        VIOC_RDMA_SetImageOffset(parkingline_rdma, rear_cam_soc->parking_line_format, rear_cam_soc->parking_line_width);
                        VIOC_RDMA_SetImageBase(parkingline_rdma, rear_cam_soc->mLineBufAddr[buffer_index], (unsigned int)NULL, (unsigned int)NULL);
                        VIOC_RDMA_SetImageEnable(parkingline_rdma);
                        
                        VIOC_WMIX_SetChromaKey(vin_wmix, layer, ON, key_R, key_G, key_B, key_mask_R, key_mask_G, key_mask_B);
                        VIOC_WMIX_SetUpdate(vin_wmix);
		}
		else{
			rear_cam_soc->mLineBufEnabled = 0;
                        VIOC_RDMA_SetImageDisable(parkingline_rdma);
		}
                mutex_unlock(&rear_cam_soc->mMutex);
	}
}

#if 0
void RCAM_LineBufUpdate(unsigned long bufnum)
{
        VIOC_RDMA *parkingline_rdma;
        unsigned long baseBufferForLine_pa[2];

        dprintk("%s, bufnum=%d\n", __func__, (unsigned int)bufnum);

        if(viocmg_is_feature_rear_cam_use_parking_line()) {
                if(bufnum < 2){
                        parkingline_rdma = (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
                        baseBufferForLine_pa[0] = rear_cam_soc->mLineBufAddr[0];
                        baseBufferForLine_pa[1] = rear_cam_soc->mLineBufAddr[1];
                        mutex_lock(&rear_cam_soc->mMutex);
                        rear_cam_soc->mLineBufIndex = bufnum;
                        VIOC_RDMA_SetImageBase(parkingline_rdma, baseBufferForLine_pa[bufnum], baseBufferForLine_pa[bufnum],   baseBufferForLine_pa[bufnum]);
                        VIOC_RDMA_SetImageUpdate(parkingline_rdma);
                        mutex_unlock(&rear_cam_soc->mMutex);
                }
        }
}
#endif

void RCAM_LineBufUpdate(RCAM_LINE_BUFFER_UPDATE_INFO * pInfo)
{
        VIOC_RDMA *parkingline_rdma;
        unsigned long baseBufferForLine_pa[2];

        dprintk("%s, bufnum=%d\n", __func__, (unsigned int)pInfo->mBufnum);

        if(viocmg_is_feature_rear_cam_use_parking_line()) {
                if(pInfo->mBufnum < 2){
		          VIOC_WMIX* 	pWMIXBase 	= (VIOC_WMIX *)rear_cam_soc->reg.in_wmix;
                        parkingline_rdma = (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
                        baseBufferForLine_pa[0] = rear_cam_soc->mLineBufAddr[0];
                        baseBufferForLine_pa[1] = rear_cam_soc->mLineBufAddr[1];
                        mutex_lock(&rear_cam_soc->mMutex);

			/* parking line buff change */
			rear_cam_soc->mLineBufIndex = pInfo->mBufnum;
			VIOC_RDMA_SetImageBase(parkingline_rdma, baseBufferForLine_pa[pInfo->mBufnum], baseBufferForLine_pa[pInfo->mBufnum],   baseBufferForLine_pa[pInfo->mBufnum]);

			/* parking line size set(width, height, offset) */
			RCAM_LineSetSize(&(pInfo->mSize));

			/* parking line position set */
			rear_cam_soc->parking_line_x = pInfo->mPos.line_x; //rcam_line_position_info->line_x;
			rear_cam_soc->parking_line_y = pInfo->mPos.line_y; //rcam_line_position_info->line_y;

			rear_cam_soc->draw_parking_line_width = rear_cam_soc->parking_line_width;
			rear_cam_soc->draw_parking_line_height = rear_cam_soc->parking_line_height;

			if((rear_cam_soc->lcd_width - rear_cam_soc->parking_line_x) < (rear_cam_soc->parking_line_width))
				rear_cam_soc->draw_parking_line_width = rear_cam_soc->lcd_width - rear_cam_soc->parking_line_x;

			if((rear_cam_soc->lcd_height - rear_cam_soc->parking_line_y) < (rear_cam_soc->parking_line_height))
				rear_cam_soc->draw_parking_line_height = rear_cam_soc->lcd_height - rear_cam_soc->parking_line_y;

			if(rear_cam_soc->mRearEventStatus == 1) {
				unsigned int wdma_intr_id = VIOC_INTR_WD0 + viocmg_get_rear_cam_vin_wdma_id();
				int retval =0;

				VIOC_RDMA_SetImageSize(parkingline_rdma, rear_cam_soc->draw_parking_line_width, rear_cam_soc->draw_parking_line_height);
				VIOC_RDMA_SetImageOffset(parkingline_rdma, rear_cam_soc->parking_line_format, rear_cam_soc->parking_line_width);

				VIOC_WMIX_SetPosition(pWMIXBase, 1, rear_cam_soc->parking_line_x,  rear_cam_soc->parking_line_y);

				printk(" >>request_irq\r\n");
				vioc_intr_enable(wdma_intr_id,  (1<<VIOC_WDMA_INTR_EOFR));

				rear_cam_soc->wdma_wakeup_int = 0;
				if(!(retval = wait_event_interruptible_timeout(rear_cam_soc->wdma_isr_wait, rear_cam_soc->wdma_wakeup_int == 1, msecs_to_jiffies(50)))) {
					pr_err("time-out wdma interrupt\r\n");
					printk("wait_event_interruptible_timeout return value : %d \n", retval);
				}

				 //disable_irq(rear_cam_soc->in_wmix_wdma_irq);
				vioc_intr_disable(wdma_intr_id,  (1<<VIOC_WDMA_INTR_EOFR));

				printk("wmda_wakeup_int : %d \n", rear_cam_soc->wdma_wakeup_int);

				VIOC_RDMA_SetImageUpdate(parkingline_rdma);
				VIOC_WMIX_SetUpdate(pWMIXBase);
			}
			mutex_unlock(&rear_cam_soc->mMutex);
		}
	}
}


void RCAM_ReleaseVideo()
{
	VIOC_VIN* 	pVINBase 	= (VIOC_VIN *)rear_cam_soc->reg.in_wmix_cam;
	VIOC_RDMA* 	pRDMABase 	= (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
	VIOC_WMIX* 	pWMIXBase 	= (VIOC_WMIX *)rear_cam_soc->reg.in_wmix;
	VIOC_WDMA* 	pWDMABase       = (VIOC_WDMA *)rear_cam_soc->reg.in_wmix_wdma;
	VIQE*           pVIQE           = (VIQE *)rear_cam_soc->reg.in_viqe;

        unsigned int is_viqe_used = viocmg_is_feature_rear_cam_use_viqe();

        mutex_lock(&rear_cam_soc->mMutex);

        dfnprintk("+\r\n");
        
        // Disable WDMA
        VIOC_WDMA_SetImageDisable(pWDMABase);
        mdelay(40); // wait 1 frame..!!

        dfnprintk("L(%d)\r\n", __LINE__);
        if(is_viqe_used){
                VIOC_CONFIG_PlugOut(VIOC_VIQE);
                VIOC_VIQE_SetControlEnable(pVIQE, OFF, OFF, OFF, OFF, OFF);
        }
        else {
                VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
        }

        dfnprintk("L(%d)\r\n", __LINE__);
        VIOC_CONFIG_PlugOut(VIOC_SC2);

#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
	 VIOC_CONFIG_PlugOut(VIOC_SC3);
#endif
	 
        BITCSET(pVINBase->uVIN_CTRL.nREG, 1, 0); 			// disable VIN0
        mdelay(1);

        dfnprintk("L(%d)\r\n", __LINE__);
        VIOC_RDMA_SetImageDisable(pRDMABase);
        mdelay(1);
        mutex_unlock(&rear_cam_soc->mMutex);

        dfnprintk("-\r\n");
}

void RCAM_WaitEvent()
{
        unsigned long currStatus;
        currStatus = rear_cam_soc->mRearEventStatus;
	
        wait_event_interruptible_timeout(rear_event_wait,rear_cam_soc->mRearEventStatus != currStatus,500 );
}

static irqreturn_t RCAM_isr(int irq, void *dev_id)
{
        unsigned long temp;
        
        dprintk(" >>ISR\r\n");
        
        RCAM_ExternalINT_POL_change();

        schedule_work(&rearcam_work_q);
        
        dprintk(" >>-ISR\r\n");

        return IRQ_HANDLED;
}


static irqreturn_t RCAM_WDMA_isr(int irq, void *dev_id)
{
        unsigned int wdma_id = VIOC_INTR_WD0 + viocmg_get_rear_cam_vin_wdma_id();
        unsigned int wdma_status, wdma_mask = (1 << VIOC_WDMA_INTR_EOFR);

        if (!is_vioc_intr_activatied(wdma_id, wdma_mask) || !is_vioc_intr_unmasked(wdma_id, wdma_mask)) {
                return IRQ_NONE;
        }
        
        wdma_status = vioc_intr_get_status(wdma_id);
        if(wdma_status & (1<<VIOC_WDMA_INTR_EOFR)) {
                dprintk("RCAM_WDMA_isr+\r\n");
                rear_cam_soc->wdma_wakeup_int = 1;
                wake_up_interruptible(&rear_cam_soc->wdma_isr_wait);

                /* clear interrupt status */
                vioc_intr_clear(wdma_id, (1<<VIOC_WDMA_INTR_EOFR));
                dprintk("RCAM_WDMA_isr-\r\n");
        }

        
        return IRQ_HANDLED;    
}


static void RCAM_SetWDMA_Isr(int OnOff)
{
        int ret;
        unsigned int wdma_intr_id = VIOC_INTR_WD0 + viocmg_get_rear_cam_vin_wdma_id();
        
        if(OnOff)
        {
                vioc_intr_disable(wdma_intr_id,  (1<<VIOC_WDMA_INTR_EOFR));
                vioc_intr_clear(wdma_intr_id, VIOC_WDMA_INT_MASK);
                
                synchronize_irq(rear_cam_soc->in_wmix_wdma_irq);
                
                request_irq(rear_cam_soc->in_wmix_wdma_irq, RCAM_WDMA_isr, IRQF_SHARED, "TCC_REAR_WDMA_EVENT", rear_cam_soc);
        }
        else
        {
                free_irq(rear_cam_soc->in_wmix_wdma_irq, rear_cam_soc);
        }
}


static void RCAM_SetExternalEvent(int OnOff)
{
	if(OnOff)
	{
                int gear_status = gpio_get_value(rear_cam_soc->gear_port);
                synchronize_irq(rear_cam_soc->gear_port_irq);
		request_irq(rear_cam_soc->gear_port_irq, RCAM_isr, gear_status?IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH, "TCC_REAR_EVENT", rear_cam_soc);
                //disable_irq(rear_cam_soc->in_wmix_wdma_irq);
	}
	else
	{
		free_irq(rear_cam_soc->gear_port_irq, RCAM_isr);
	}
	dprintk("%s, onoff=%d\n", __func__, OnOff);
}

static void rearcam_work_thread(struct work_struct *work)
{
        mdelay(10); // remove flickering noise;
		RCAM_IsPreviewShow(RCAM_AFTER_BOOT);
}


void RCAM_DeInitialize(void)
{
        kthread_stop(rear_cam_soc->mAliveThread);

        RCAM_SetWDMA_Isr(0);

        RCAM_SetRearEvent(0);

		flush_work(&rearcam_work_q);
}


unsigned long RCAM_Initialize(struct platform_device *pdev)
{
        int ret = -1;

        #if defined(CONFIG_ARCH_TCC893X) // please check it..!
        volatile PVIOC_IREQ_CONFIG pConfig = (volatile PVIOC_IREQ_CONFIG)tcc_p2v((unsigned int)HwVIOC_CONFIG);
        if(ISSET(pConfig->uMISC1.nREG, 1 << 23)) {
                BITCLR(pConfig->uMISC1.nREG, 1 << 23);
        }
        #endif

        dprintk("RCAM_Initialize\r\n");
        
        INIT_WORK(&rearcam_work_q, rearcam_work_thread);
        init_waitqueue_head(&rear_cam_soc->wdma_isr_wait);

        mutex_init(&rear_cam_soc->mMutex);
        mutex_init(&rear_cam_soc->mShowMutex);

        // link API
        viocmg_set_rear_cam_gearstatus_api((void*)RCAM_GetGearStatus);
        viocmg_set_rear_cam_dmastatus_api((void*)RCAM_GetDmaStatus);
        
        ret = RCAM_Parse_DT(pdev);
        if(ret < 0) {
                goto END_PROCESS; 
        }

        ret = RCAM_Process_Pmap(pdev);    
        if(ret < 0) {
                goto END_PROCESS; 
        }

        dprintk(" >>save default wmix\r\n");
        viocmg_save_wmix_default();

        RCAM_SetWDMA_Isr(1);

		if(viocmg_is_feature_rear_cam_use_parking_line())
				rear_cam_soc->mLineBufEnabled = 1;

        RCAM_IsPreviewShow(RCAM_COLD_BOOT);
        
        RCAM_SetRearEvent(1);

        rear_cam_soc->mCurrAliveCount =0;
        rear_cam_soc->mFileOpenCount = 0;

        rear_cam_soc->mAliveThreadRun = 1;

        rear_cam_soc->mAliveThread = kthread_create(RCAM_CheckAliveThread, NULL, "RCAM_CheckAliveThread");

        if(IS_ERR(rear_cam_soc->mAliveThread)) {
                ret = PTR_ERR(rear_cam_soc->mAliveThread);
        }
        else {
                wake_up_process(rear_cam_soc->mAliveThread);
                ret = 0;
        }
        
END_PROCESS:
	return ret;
}

void RCAM_SetRearEvent(int OnOff)
{
	RCAM_SetExternalEvent(OnOff);		
	if(OnOff){
		init_waitqueue_head(&rear_event_wait);
		dprintk("RCAM_SetRearEvent On \n");
	}
	else{
		dprintk("RCAM_SetRearEvent Off \n");
	}
}

void RCAM_IsPreviewShow(unsigned int booting)
{
        unsigned int gear_status;

        dfnprintk("+\r\n");
        if(!rear_cam_soc->mSuspend){
		gear_status = gpio_get_value(rear_cam_soc->gear_port);
                dprintk("RCAM_IsPreviewShow gear state = %d\r\n", (rear_cam_soc->gear_port_active_level == gear_status));
                if(rear_cam_soc->gear_port_active_level == gear_status) {
                        rear_cam_soc->mRearEventStatus = 1;
                }
                else {
                        rear_cam_soc->mRearEventStatus = 0;
                }

                RCAM_Show(rear_cam_soc->mRearEventStatus, booting);
        }	
        dfnprintk("-\r\n");
}

void RCAM_SetPreviewVIOC(void)
{
	VIOC_RDMA* pRDMA;

        pRDMA = (VIOC_RDMA *)rear_cam_soc->reg.lcd_wmix_rdma;
        mutex_lock(&rear_cam_soc->mMutex);
        VIOC_RDMA_SetImageFormat(pRDMA, rear_cam_soc->preview_format);
        VIOC_RDMA_SetImageSize(pRDMA, (unsigned int)rear_cam_soc->draw_preview_width,(unsigned int)rear_cam_soc->draw_preview_height);
        VIOC_RDMA_SetImageOffset(pRDMA, rear_cam_soc->preview_format, (unsigned int)rear_cam_soc->preview_width);
        VIOC_RDMA_SetImageBase(pRDMA, rear_cam_soc->mPreviewBufAddr[0], (unsigned int)NULL, (unsigned int)NULL);
        VIOC_RDMA_SetImageEnable(pRDMA);
        mutex_unlock(&rear_cam_soc->mMutex);
}

void RCAM_PreviewUpdate(unsigned int buf_address)
{
        VIOC_RDMA* pRDMA;

        pRDMA = (VIOC_RDMA *)rear_cam_soc->reg.lcd_wmix_rdma;
        VIOC_RDMA_SetImageBase(pRDMA, buf_address, (unsigned int)NULL, (unsigned int)NULL);
        VIOC_RDMA_SetImageUpdate(pRDMA);
}

void RCAM_SetScalerOverscan(unsigned long overscan_position_x, unsigned long overscan_position_y, unsigned long overscan_width_factor, unsigned long overscan_height_factor)
{
        VIOC_SC* pSCBase;

        if(rear_cam_soc->preview_x  != overscan_position_x ||
                rear_cam_soc->preview_y != overscan_position_y ||
                rear_cam_soc->preview_additional_width != overscan_width_factor ||
                rear_cam_soc->preview_additional_height != overscan_height_factor)
        {
                pSCBase = (VIOC_SC *)tcc_p2v(HwVIOC_SC2);
                dprintk(" >>RCAM_SetScalerOverscan(%ld, %ld, %ld, %ld)\r\n", overscan_position_x, overscan_position_y, overscan_width_factor, overscan_height_factor);
                VIOC_SC_SetDstSize(pSCBase, rear_cam_soc->preview_width+overscan_width_factor, rear_cam_soc->preview_height+overscan_height_factor);
                VIOC_SC_SetOutPosition(pSCBase, overscan_position_x, overscan_position_y);
                VIOC_SC_SetOutSize(pSCBase, (unsigned int)rear_cam_soc->preview_width,(unsigned int)rear_cam_soc->preview_height);
                VIOC_SC_SetUpdate(pSCBase);
        }
        rear_cam_soc->preview_x = overscan_position_x;
        rear_cam_soc->preview_y = overscan_position_y;
        rear_cam_soc->preview_additional_width = overscan_width_factor;
        rear_cam_soc->preview_additional_height = overscan_height_factor;

}

int tcc_camera_set_rdma(VIOC_RDMA* pRDMA, unsigned int base_addr, int width, int height, int fmt, int onoff)
{
        VIOC_RDMA_SetImageFormat(pRDMA, fmt);
        VIOC_RDMA_SetImageSize(pRDMA, width, height);
        VIOC_RDMA_SetImageOffset(pRDMA, fmt, width);
        VIOC_RDMA_SetImageBase(pRDMA, base_addr, (unsigned int)NULL, (unsigned int)NULL);

        if(onoff && rear_cam_soc->mLineBufEnabled)
                VIOC_RDMA_SetImageEnable(pRDMA);
        else
                VIOC_RDMA_SetImageDisable(pRDMA);

        return 0;
}

int tcc_camera_set_vin(VIOC_VIN* pVIN)
{
        unsigned int is_viqe_used = viocmg_is_feature_rear_cam_use_viqe();
        dfnprintk("+\r\n");
        VIOC_VIN_SetSyncPolarity(pVIN, OFF, OFF, OFF, OFF, OFF, OFF);
        VIOC_VIN_SetCtrl(pVIN, ON, OFF, OFF, FMT_YUV422_8BIT, ORDER_RGB);

        VIOC_VIN_SetInterlaceMode(pVIN, ON, OFF);

        VIOC_VIN_SetImageSize(pVIN, CAMERA_PREVIEW_WIDTH, CAMERA_PREVIEW_HEIGHT);
        VIOC_VIN_SetImageOffset(pVIN, 0, 0, 0);

        VIOC_VIN_SetY2RMode(pVIN, 2);
        VIOC_VIN_SetY2REnable(pVIN, is_viqe_used?OFF:ON);
        VIOC_VIN_SetLUTEnable(pVIN, OFF, OFF, OFF);
        VIOC_VIN_SetEnable(pVIN, ON);

        return 0;
}

int tcc_camera_set_viqe(VIQE* pVIQE, char bUseVIQE)
{
        unsigned int plugin_val;
        unsigned int offset = CAMERA_PREVIEW_WIDTH * CAMERA_PREVIEW_HEIGHT *2;
        dfnprintk("+\r\n");
        if(bUseVIQE) {
                unsigned int deintl_base0 = rear_cam_soc->mViqeMemory; 
                unsigned int deintl_base1 = deintl_base0 + (offset);
                unsigned int deintl_base2 = deintl_base1 + (offset);
                unsigned int deintl_base3 = deintl_base2 + (offset);

                printk("viqe mem=0x%x\r\n", rear_cam_soc->mViqeMemory);

                VIOC_CONFIG_PlugOut(VIOC_VIQE);
                switch(viocmg_get_rear_cam_vin_id())
                {
                        case 0:
                                plugin_val = 0xA; // VIOC_VIQE_VIN00
                                break;
                        case 1:
                                plugin_val = 0xC; // VIOC_VIQE_VIN01
                                break;
                        case 2:
                                plugin_val = 0x8; // VIOC_VIQE_VIN02
                                break;
                        case 3:
                                plugin_val = 0x9; // VIOC_VIQE_VIN03
                                break;
                }
                
                VIOC_CONFIG_PlugIn(VIOC_VIQE, plugin_val);
                VIOC_VIQE_SetDeintlMode(pVIQE, VIOC_VIQE_DEINTL_MODE_2D);
                VIOC_VIQE_SetControlRegister(pVIQE, 0, 0, FMT_FC_YUV420);
                VIOC_VIQE_SetDeintlRegister(pVIQE, FMT_FC_YUV420, OFF, 0, 0, VIOC_VIQE_DEINTL_MODE_3D, (unsigned int)deintl_base0,
                        (unsigned int)deintl_base1, (unsigned int)deintl_base2, (unsigned int)deintl_base3);
                VIOC_VIQE_InitDeintlCoreVinMode(pVIQE);
                VIOC_VIQE_SetY2R(pVIQE, TRUE);
                VIOC_VIQE_SetControlEnable(pVIQE, OFF, OFF, OFF, OFF, ON);
        } else {
                VIOC_CONFIG_PlugIn(VIOC_DEINTLS, 10 /*VIOC_VIQE_VIN_00*/);
        }
        return 0;
}

int tcc_camera_set_wmix(VIOC_WMIX* pWMIX, int width, int height)
{
        unsigned int key_R          = 0xFF;
        unsigned int key_G          = 0xFF;
        unsigned int key_B          = 0xFF;
        unsigned int key_mask_R     = 0xF8;
        unsigned int key_mask_G     = 0xF8;
        unsigned int key_mask_B     = 0xF8;

        dfnprintk("+\r\n");
        
        VIOC_WMIX_SetSize(pWMIX, width, height);

	 VIOC_WMIX_SetPosition(pWMIX, 0, 0,0);//rear_cam_soc->preview_x, rear_cam_soc->preview_y);
	
	 
        if(viocmg_is_feature_rear_cam_use_parking_line()) {
                VIOC_WMIX_SetChromaKey(pWMIX, 0, ON, key_R, key_G, key_B, key_mask_R, key_mask_G, key_mask_B);
                VIOC_WMIX_SetPosition(pWMIX, 1, rear_cam_soc->parking_line_x, rear_cam_soc->parking_line_y);      
        }
        VIOC_WMIX_SetUpdate(pWMIX);

        return 0;
}


int tcc_camera_set_scaler(VIOC_SC* pSC, unsigned long overscan_position_x, unsigned long overscan_position_y, unsigned int overscan_width_factor, unsigned int overscan_height_factor, int width, int height)
{
        unsigned int plugin_val;
        unsigned int scaler_id = viocmg_get_rear_cam_scaler_id();

        dfnprintk("+\r\n");
        
        switch(viocmg_get_rear_cam_vin_id())
        {
                case 0:
                        plugin_val = 0x10; // VIOC_SC_VIN00
                        break;
                case 1:
                        plugin_val = 0x12; // VIOC_SC_VIN01
                        break;
                case 2:
                        plugin_val = 0xC; // VIOC_SC_VIN02
                        break;
                case 3:
                        plugin_val = 0xE; // VIOC_SC_VIN03
                        break;
        }

        VIOC_SC_SetBypass(pSC, OFF);
        VIOC_SC_SetDstSize(pSC, width+overscan_width_factor, height+overscan_height_factor);
        VIOC_SC_SetOutPosition(pSC, overscan_position_x, overscan_position_y);
        VIOC_SC_SetOutSize(pSC, width, height);
        VIOC_SC_SetUpdate(pSC);
        VIOC_CONFIG_PlugIn(VIOC_SC0 + scaler_id, plugin_val);

        return 0;
}

#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
int tcc_preview_set_scaler(VIOC_SC* pSC, unsigned long overscan_position_x, unsigned long overscan_position_y, unsigned int overscan_width_factor, unsigned int overscan_height_factor, int width, int height)
{
        unsigned int plugin_val;

        dfnprintk("+\r\n");
        
        switch(viocmg_get_rear_cam_vin_id())
        {
                case 0:
                        plugin_val = 0x19; // VIOC_SC_VIN00
                        break;
                case 1:
                        plugin_val = 0x1B; // VIOC_SC_VIN01
                        break;
        }

        VIOC_SC_SetBypass(pSC, OFF);
        VIOC_SC_SetDstSize(pSC, width+overscan_width_factor, height+overscan_height_factor);
        VIOC_SC_SetOutPosition(pSC, overscan_position_x, overscan_position_y);
        VIOC_SC_SetOutSize(pSC, width, height);
        VIOC_SC_SetUpdate(pSC);
        VIOC_CONFIG_PlugIn(VIOC_SC3, plugin_val);

        return 0;
}
#endif

int tcc_camera_set_wdma(VIOC_WDMA* pWDMA, unsigned int base_addr, int width, int height, int fmt)
{
        dfnprintk("+\r\n");
        VIOC_WDMA_SetImageFormat(pWDMA, fmt);
        VIOC_WDMA_SetImageSize(pWDMA, width, height);
        VIOC_WDMA_SetImageOffset(pWDMA, fmt, width);
        VIOC_WDMA_SetImageBase(pWDMA, base_addr, (unsigned int)NULL, (unsigned int)NULL);
        VIOC_WDMA_SetImageEnable(pWDMA, ON);
        return 0;
}

void tcc_camera_viqe_modechange(void)
{
        VIQE* pVIQE = (VIQE *)rear_cam_soc->reg.in_viqe;

        VIOC_VIQE_SetDeintlMode(pVIQE, VIOC_VIQE_DEINTL_MODE_3D);
}

void RCAM_ReConfigVIOC(void)
{
        VIOC_VIN* 		pVINBase 	= (VIOC_VIN *)rear_cam_soc->reg.in_wmix_cam;
        VIOC_RDMA* 		pRDMABase 	= (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
        VIQE* 			pVIQEBase 	= (VIQE *)rear_cam_soc->reg.in_viqe;
        VIOC_SC* 		pSCBase 	= (VIOC_SC *)rear_cam_soc->reg.in_scaler;
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
        VIOC_SC* 		pSCBase2 	= (VIOC_SC *)tcc_p2v(HwVIOC_SC3);
#endif
        VIOC_WMIX* 		pWMIXBase 	= (VIOC_WMIX *)rear_cam_soc->reg.in_wmix;
        VIOC_WDMA* 		pWDMABase 	= (VIOC_WDMA *)rear_cam_soc->reg.in_wmix_wdma;
        DDICONFIG* 		pDDIConfig 	= (DDICONFIG *)rear_cam_soc->reg.camera_port;	
        VIOC_IREQ_CONFIG* 	pIREQConfig 	= (VIOC_IREQ_CONFIG *)rear_cam_soc->reg.vioc_config;

        unsigned int buffer_index = rear_cam_soc->mLineBufIndex;
        unsigned int parking_addr = rear_cam_soc->mLineBufAddr[buffer_index];//cm3_get_pg_addr();

        unsigned int vin_shift = 24+viocmg_get_rear_cam_vin_id();
        unsigned int vin_rdma_shift = viocmg_get_rear_cam_vin_rdma_id();
        unsigned int vin_wmix_shift = 9+viocmg_get_rear_cam_vin_wmix_id();
        unsigned int vin_scaler_shift = 30; // fixed!!
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER      
        unsigned int lcd_scaler_shift = 31;
#endif
        unsigned int vin_wdma_shift = viocmg_get_rear_cam_vin_wdma_id();
        int loop;
        unsigned int is_viqe_used = viocmg_is_feature_rear_cam_use_viqe();

        dprintk("RCAM_ReConfigVIOC+\r\n");

		dprintk("RCAM Register Address dump \n");
		dprintk("VIOC_VIN = 0x%x \n", pVINBase);
		dprintk("VIOC_RDMA = 0x%x \n" , pRDMABase);
		dprintk("VIOC_VIQE = 0x%x \n" , pVIQEBase);

        dprintk("parking line[%d] = 0x%x\r\n", buffer_index, parking_addr);
        
        RCAM_ReleaseVideo();

        mutex_lock(&rear_cam_soc->mMutex);

        // vioc camera path reset
        dprintk(" >> REG1  SET (1 << %d)\r\n", vin_wdma_shift);
        BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (1<<vin_wdma_shift), (1<<vin_wdma_shift)); // WDMA reset
        dprintk(" >> REG1  SET (1 << %d)\r\n", vin_wmix_shift);
        BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (1<<vin_wmix_shift), (1<<vin_wmix_shift));     // WMIX reset
        dprintk(" >> REG0  SET (1 << %d)\r\n", vin_scaler_shift);
        BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_scaler_shift), (1<<vin_scaler_shift)); // SCALER2 reset
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER        
        BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (1<<lcd_scaler_shift), (1<<lcd_scaler_shift)); // SCALER3 reset
#endif       

        dfnprintk("L(%d)\r\n", __LINE__);
        if(is_viqe_used)
                BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (1<<16), (1<<16));                     // VIQE reset
        else
                BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (1<<17), (1<<17));                     // DINTS reset
        dprintk(" >> REG0  SET (1 << %d)\r\n", vin_shift);
        BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_shift), (1<<vin_shift));               // VIN reset
        dprintk(" >> REG0  SET (1 << %d)\r\n", vin_rdma_shift);
        BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_rdma_shift), (1<<vin_rdma_shift));     // RDMA reset
        
        dprintk(" >> REG0  CLEAR (0 << %d)\r\n", vin_rdma_shift);
        BITCLR(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_rdma_shift));     // RDMA reset clear 
        dprintk(" >> REG0  CLEAR (0 << %d)\r\n", vin_shift);
        BITCLR(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_shift));               // VIN reset clear
        dfnprintk("L(%d)\r\n", __LINE__);
        if(is_viqe_used)
                BITCLR(pIREQConfig->uSOFTRESET.nREG[1], (1<<16));                     // VIQE reset clear
        else
                BITCLR(pIREQConfig->uSOFTRESET.nREG[1], (1<<17));                     // DINTS reset clear
        dprintk(" >> REG0  CLEAR (0 << %d)\r\n", vin_scaler_shift);
        BITCLR(pIREQConfig->uSOFTRESET.nREG[0], (1<<vin_scaler_shift)); // SCALER reset clear
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER        
        BITCLR(pIREQConfig->uSOFTRESET.nREG[0], (1<<lcd_scaler_shift)); // SCALER reset clear
#endif        
        dprintk(" >> REG1  CLEAR (0 << %d)\r\n", vin_wmix_shift);
        BITCLR(pIREQConfig->uSOFTRESET.nREG[1], (1<<vin_wmix_shift));     // WMIX reset clear
        dprintk(" >> REG1  CLEAR (0 << %d)\r\n", vin_wdma_shift);
        BITCLR(pIREQConfig->uSOFTRESET.nREG[1], (1<< vin_wdma_shift));   // WDMA reset clear

        // cam-4 port configuration
        dprintk(" RCAM_ReConfigVIOC camera port\r\n");
        BITCSET(rear_cam_soc->reg.camera_port, 0x00077777, rear_cam_soc->camera_port << (viocmg_get_rear_cam_vin_id() << 2));

        if(viocmg_is_feature_rear_cam_use_parking_line()) {
                /* Global alpha settings  of parking guide-line*/
                VIOC_RDMA_SetImageAlphaEnable(pRDMABase, 1);
                VIOC_RDMA_SetImageAlpha(pRDMABase, 0xff,0xff);

               // tcc_camera_set_rdma(pRDMABase, parking_addr, rear_cam_soc->parking_line_width , rear_cam_soc->parking_line_height, rear_cam_soc->parking_line_format,1);
				
		  if(rear_cam_soc->parking_line_width != rear_cam_soc->draw_parking_line_width || rear_cam_soc->parking_line_height != rear_cam_soc->draw_parking_line_height)
		  	{
		  	tcc_camera_set_rdma(pRDMABase, parking_addr, rear_cam_soc->draw_parking_line_width , rear_cam_soc->draw_parking_line_height, rear_cam_soc->parking_line_format,1);
		  	VIOC_RDMA_SetImageOffset(pRDMABase, rear_cam_soc->parking_line_format, rear_cam_soc->parking_line_width);
		  	}
		  else
		  	tcc_camera_set_rdma(pRDMABase, parking_addr, rear_cam_soc->parking_line_width , rear_cam_soc->parking_line_height, rear_cam_soc->parking_line_format,1);
		  	
        }

        tcc_camera_set_vin(pVINBase);
        tcc_camera_set_viqe(pVIQEBase, is_viqe_used);
        tcc_camera_set_scaler(pSCBase, 
                       // rear_cam_soc->preview_x, 
                       // rear_cam_soc->preview_y, 
                        rear_cam_soc->preview_crop_x,
                        rear_cam_soc->preview_crop_y,
                        rear_cam_soc->preview_additional_width, 
                        rear_cam_soc->preview_additional_height, 
                        #ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
		          rear_cam_soc->lcd_width,
		          rear_cam_soc->lcd_height
			   #else
                        rear_cam_soc->preview_width, 
                        rear_cam_soc->preview_height
			   #endif
			   );
        VIOC_WMIX_SetOverlayPriority(pWMIXBase, 5); // internal wmix (ovp is fixed)

        tcc_camera_set_wmix(pWMIXBase, 
			   #ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
		          rear_cam_soc->lcd_width,
		          rear_cam_soc->lcd_height
			   #else
		          rear_cam_soc->draw_preview_width, 
		          rear_cam_soc->draw_preview_height
			   #endif
			   );
		
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER
	 tcc_preview_set_scaler(pSCBase2, rear_cam_soc->preview_crop_x, rear_cam_soc->preview_crop_y, rear_cam_soc->preview_additional_width, rear_cam_soc->preview_additional_height, rear_cam_soc->preview_width, rear_cam_soc->preview_height);
#endif    

        tcc_camera_set_wdma(pWDMABase, rear_cam_soc->mPreviewBufAddr[0], 
                                rear_cam_soc->preview_width, 
                                rear_cam_soc->preview_height, 
                                rear_cam_soc->preview_format);
        if(is_viqe_used) {
                unsigned int wdma_intr_id = VIOC_INTR_WD0 + viocmg_get_rear_cam_vin_wdma_id();
                
                dprintk(">>%s enable VIOC_WDMA_INTR_EOFR\r\n", __func__);
                
                //vioc_intr_disable(wdma_intr_id,  VIOC_WDMA_INT_MASK);
                //vioc_intr_clear(wdma_intr_id, VIOC_WDMA_INT_MASK);

                dprintk(" >>wait synchronize_irq\r\n");
                //synchronize_irq(rear_cam_soc->in_wmix_wdma_irq);

                dprintk(" >>request_irq\r\n");
                vioc_intr_enable(wdma_intr_id,  (1<<VIOC_WDMA_INTR_EOFR));
                //request_irq(rear_cam_soc->in_wmix_wdma_irq, RCAM_WDMA_isr, IRQF_SHARED, "TCC_REAR_WDMA_EVENT", rear_cam_soc);
                //enable_irq(rear_cam_soc->in_wmix_wdma_irq);


                // wait 3 fields
                for(loop=0;loop<3;loop++) {
                        rear_cam_soc->wdma_wakeup_int = 0;                        
                        if(!wait_event_interruptible_timeout(rear_cam_soc->wdma_isr_wait, rear_cam_soc->wdma_wakeup_int == 1, msecs_to_jiffies(50))) {
                                pr_err("time-out wdma interrupt\r\n");
                        }
                }

                //disable_irq(rear_cam_soc->in_wmix_wdma_irq);
                vioc_intr_disable(wdma_intr_id,  (1<<VIOC_WDMA_INTR_EOFR));
                
                //free_irq(rear_cam_soc->in_wmix_wdma_irq, RCAM_WDMA_isr);
                
                // VIQE 3D Mode needs 60ms delay (3frame). 
                printk("VIQE 3D MODE..!!\r\n");
                
                tcc_camera_viqe_modechange();

                dprintk(">>%s disaqble VIOC_WDMA_INTR_EOFR\r\n", __func__);
                
        }
        
        mutex_unlock(&rear_cam_soc->mMutex);

        dprintk("RCAM_ReConfigVIOC-\r\n");
        return 0;
}

void RCAM_CheckAlive(void)
{
        
        int i;
        unsigned int temp;
        VIOC_WDMA* pWDMABase = (VIOC_WDMA *)rear_cam_soc->reg.in_wmix_wdma;
        
        mutex_lock(&rear_cam_soc->mShowMutex);

		if(handover) {
	        //dprintk(">>RCAM_CheckAlive+\r\n");
	        if(viocmg_get_rear_cam_mode() > VIOCMG_REAR_MODE_STOP) {
	                if(rear_cam_soc->mPrevAliveBase == pWDMABase->uCBASE){
	                    rear_cam_soc->mCurrAliveCount++;
	                }
	                else {
	                        rear_cam_soc->mPrevAliveBase = pWDMABase->uCBASE;
	                }

                if(rear_cam_soc->mCurrAliveCount >= 2){
                        //dprintk("--------> RCAM_CheckAlive %x %x \n",pWDMABase->uCBASE, pWDMABase->uBASE0);
                        rear_cam_soc->mCurrAliveCount =0;
                        RCAM_ReConfigVIOC();
                }
        }
        else {
                rear_cam_soc->mCurrAliveCount = 0;
                rear_cam_soc->mPrevAliveBase = 0;
        }
        //dprintk(">>RCAM_CheckAlive-\r\n");
		}
        mutex_unlock(&rear_cam_soc->mShowMutex);
}


//#define RCAM_REINIT_TEST 
//#define RCAM_LOOP_TEST

static int RCAM_CheckAliveThread(void *arg)
{
        #if defined(RCAM_REINIT_TEST)
        int ntime = 0;
        #endif

        #if defined(RCAM_LOOP_TEST)
        int force_enable = 0;
        unsigned long dealy_jiffies = jiffies + msecs_to_jiffies(4500);
        #endif
        dprintk(" >>RCAM_CheckAliveThread start..!!!\r\n");

        while(1){
                // check whether we should exit or not
                if(kthread_should_stop() || !rear_cam_soc->mAliveThreadRun){
                        break;
                }

                RCAM_CheckAlive();
                
                // this delay will influence the CPU usage and response latency
                msleep(100);
                
                #if defined(RCAM_LOOP_TEST)

                if(time_after(jiffies, dealy_jiffies)) {
                        printk("FORCE RCAM_SHOW(%d)\r\n", force_enable);
                        RCAM_Show(force_enable, RCAM_AFTER_BOOT);
                        force_enable = !force_enable;         
                        dealy_jiffies = jiffies + msecs_to_jiffies(4500);
                }
                #if defined(RCAM_REINIT_TEST)
                if(ntime++ > 50) {
                        mutex_lock(&rear_cam_soc->mShowMutex);
                        if(viocmg_get_rear_cam_mode() > VIOCMG_REAR_MODE_STOP)
                                RCAM_ReConfigVIOC();
                        mutex_unlock(&rear_cam_soc->mShowMutex);
                        ntime = 0;
                }
                #endif
                #endif
                
        }
        dprintk(" >>RCAM_CheckAliveThread stop..!!!\r\n");
        return 0;
}


void RCAM_GetLineBufAddrByIndex(RCAM_LINE_BUFFER_INFO * rcam_line_buffer_info)
{
	unsigned long index;

	index = rcam_line_buffer_info->index;

	//rcam_line_buffer_info->index = rear_cam_soc->mLineBufIndex;
	//rear_cam_soc->mLineBufIndex = rcam_line_buffer_info->index;
	rcam_line_buffer_info->rear_linebuf_addr = rear_cam_soc->mLineBufAddr[index];
	rcam_line_buffer_info->buffer_size = rear_cam_soc->mLineBufAddr[1] - rear_cam_soc->mLineBufAddr[0];

			   
	dprintk("[tcc_rear_cam] index = %ld\nlineBufAddr = 0x%x\nbufferSize = %ld \n", rcam_line_buffer_info->index, rcam_line_buffer_info->rear_linebuf_addr, rcam_line_buffer_info->buffer_size);
}

void RCAM_LineSetSize(RCAM_LINE_SIZE_INFO * rcam_line_size_info)
{
	VIOC_RDMA* pRDMABase 		= (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;

	rear_cam_soc->draw_parking_line_width = rear_cam_soc->parking_line_width = rcam_line_size_info->width;
	rear_cam_soc->draw_parking_line_height = rear_cam_soc->parking_line_height = rcam_line_size_info->height;

	if(rear_cam_soc->mRearEventStatus == 1) {
		VIOC_RDMA_SetImageSize(pRDMABase, rear_cam_soc->parking_line_width, rear_cam_soc->parking_line_height);
		VIOC_RDMA_SetImageOffset(pRDMABase, rear_cam_soc->parking_line_format, rear_cam_soc->parking_line_width);
	}
}
	

void RCAM_LineSetPosition(RCAM_LINE_POSITION_INFO * rcam_line_position_info)
{
       VIOC_WMIX* 	pWMIXBase 	= (VIOC_WMIX *)rear_cam_soc->reg.in_wmix; 
	VIOC_RDMA* pRDMABase 		= (VIOC_RDMA *)rear_cam_soc->reg.in_wmix_rdma;
			

	rear_cam_soc->parking_line_x = rcam_line_position_info->line_x;
	rear_cam_soc->parking_line_y = rcam_line_position_info->line_y;

	rear_cam_soc->draw_parking_line_width = rear_cam_soc->parking_line_width;
	rear_cam_soc->draw_parking_line_height = rear_cam_soc->parking_line_height;

	if((rear_cam_soc->lcd_width - rear_cam_soc->parking_line_x) < (rear_cam_soc->parking_line_width))
		rear_cam_soc->draw_parking_line_width = rear_cam_soc->lcd_width - rear_cam_soc->parking_line_x;

	if((rear_cam_soc->lcd_height - rear_cam_soc->parking_line_y) < (rear_cam_soc->parking_line_height))
		rear_cam_soc->draw_parking_line_height = rear_cam_soc->lcd_height - rear_cam_soc->parking_line_y;

	if(rear_cam_soc->mRearEventStatus == 1) {
		VIOC_RDMA_SetImageSize(pRDMABase, rear_cam_soc->draw_parking_line_width, rear_cam_soc->draw_parking_line_height);
		VIOC_RDMA_SetImageOffset(pRDMABase, rear_cam_soc->parking_line_format, rear_cam_soc->parking_line_width);
		VIOC_RDMA_SetImageUpdate(pRDMABase);
					
		VIOC_WMIX_SetPosition(pWMIXBase, 1, rear_cam_soc->parking_line_x,  rear_cam_soc->parking_line_y);
		VIOC_WMIX_SetUpdate(pWMIXBase);
	}
}

void RCAM_PreviewSetPosition(unsigned int preview_position_x, unsigned int preview_position_y)
{
	VIOC_WMIX* pWMIX = (VIOC_WMIX *)tcc_p2v(HwVIOC_WMIX0);// viocmg_get_(VIOC_WMIX*)rear_cam_soc->reg.lcd_wmix;

//	if(rear_cam_soc->mRearEventStatus == 1) {
		if(preview_position_x + rear_cam_soc->preview_width > rear_cam_soc->lcd_width)
			rear_cam_soc->draw_preview_width = rear_cam_soc->lcd_width - preview_position_x;
		else
			rear_cam_soc->draw_preview_width = rear_cam_soc->preview_width;
		
		if(preview_position_y + rear_cam_soc->preview_height > rear_cam_soc->lcd_height)
			rear_cam_soc->draw_preview_height = rear_cam_soc->lcd_height - preview_position_y;
		else
			rear_cam_soc->draw_preview_height = rear_cam_soc->preview_height;

		rear_cam_soc->preview_x = preview_position_x;
		rear_cam_soc->preview_y = preview_position_y;

	if(rear_cam_soc->mRearEventStatus == 1) {
		RCAM_SetPreviewVIOC();
		
		VIOC_WMIX_SetPosition(pWMIX, viocmg_get_rear_cam_display_rdma_id()%4, preview_position_x, preview_position_y);

		VIOC_WMIX_SetUpdate(pWMIX);
	}
}

void RCAM_PreviewSetSize(unsigned int width, unsigned int height)
{
#ifdef CONFIG_TCC_REAR_CAMERA_USE_SCALER	
	VIOC_SC*		pSC = (VIOC_SC *)tcc_p2v(HwVIOC_SC3);
#else
	VIOC_SC* 		pSC	= (VIOC_SC *)rear_cam_soc->reg.in_scaler;
#endif
	VIOC_WMIX* 		pWMIXBase 	= (VIOC_WMIX *)rear_cam_soc->reg.in_wmix;
	VIOC_WDMA* 	pWDMABase 	= (VIOC_WDMA *)rear_cam_soc->reg.in_wmix_wdma;


//	if(rear_cam_soc->mRearEventStatus == 1) {
		dprintk("RCAM_PreviewSetSize start!! \n");
		dprintk("preview width : %d, preview height : %d \n", width, height);

	        VIOC_WDMA_SetImageDisable(pWDMABase);
	        mdelay(40); // wait 1 frame..!!

#ifndef CONFIG_TCC_REAR_CAMERA_USE_SCALER	
	       rear_cam_soc->mStatus &= (~RCAM_ST_PARKING_LINE);
	       RCAM_LINE_OnOff(0);
#endif

		if(width > rear_cam_soc->lcd_width - rear_cam_soc->preview_x|| height > rear_cam_soc->lcd_height - rear_cam_soc->preview_y) {
			dprintk("Preview size is bigger than lcd.. \n");
			width = rear_cam_soc->lcd_width;
			height = rear_cam_soc->lcd_height;
		}
		
		rear_cam_soc->preview_width = width;
		rear_cam_soc->preview_height = height;

		RCAM_PreviewSetPosition(rear_cam_soc->preview_x, rear_cam_soc->preview_y);

	if(rear_cam_soc->mRearEventStatus == 1) {
		VIOC_SC_SetDstSize(pSC, width + rear_cam_soc->preview_additional_width, height + rear_cam_soc->preview_additional_height);
		VIOC_SC_SetOutPosition(pSC, rear_cam_soc->preview_crop_x, rear_cam_soc->preview_crop_y);
	       VIOC_SC_SetOutSize(pSC, width, height);
		   
		VIOC_WDMA_SetImageSize(pWDMABase, width, height);
		VIOC_WDMA_SetImageOffset(pWDMABase, rear_cam_soc->preview_format, width);
		
#ifndef CONFIG_TCC_REAR_CAMERA_USE_SCALER	
		VIOC_WMIX_SetSize(pWMIXBase, rear_cam_soc->preview_width, rear_cam_soc->preview_height);
		VIOC_WMIX_SetUpdate(pWMIXBase);
#endif

	       VIOC_SC_SetUpdate(pSC);

		VIOC_WDMA_SetImageEnable(pWDMABase, ON);

		//RCAM_SetPreviewVIOC();
		//RCAM_PreviewSetPosition(rear_cam_soc->preview_x, rear_cam_soc->preview_y);
	}
	dprintk("RCAM_PreviewSetSize end!!\n");
}
#if defined (USING_EARLY_CAM_CM4)
int RCAM_Handover(int cm4_gear_status)
{
	unsigned int gear_status;
	dfnprintk("+\r\n");

	mutex_lock(&rear_cam_soc->mShowMutex);

	//mdelay(1000); // for handover test

	gear_status = gpio_get_value(rear_cam_soc->gear_port);

	if((rear_cam_soc->gear_port_active_level == gear_status) && (cm4_gear_status == 0)) {
		RCAM_ReConfigVIOC();
		RCAM_PreviewSetPosition(rear_cam_soc->preview_x, rear_cam_soc->preview_y);
		RCAM_SetPreviewVIOC();
		viocmg_set_rear_cam_mode(VIOCMG_CALLERID_REAR_CAM, VIOCMG_REAR_MODE_RUN);
	}

	handover = 1;

	dfnprintk("gear state = %d\r\n", (rear_cam_soc->gear_port_active_level == gear_status));
	dfnprintk("cm4 gear state = %d \n", cm4_gear_status);

	dfnprintk("-\r\n");
    mutex_unlock(&rear_cam_soc->mShowMutex);

	RCAM_IsPreviewShow(RCAM_AFTER_BOOT);

	return 0;
}
#endif
