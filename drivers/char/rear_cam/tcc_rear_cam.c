
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/fs.h> 

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

#include <mach/bsp.h>
#include <mach/irqs.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_config.h>

#include <mach/vioc_scaler.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_api.h>
#include <mach/vioc_disp.h>
#include <linux/poll.h>
#include <mach/tccfb_ioctrl.h>
//#include "tcc_wdma.h"
#include <mach/tcc_wdma_ioctrl.h>
#include <mach/tcc_fb.h>
#include <mach/tca_display_config.h>


#if defined(CONFIG_ARCH_TCC892X)
#include <mach/vioc_plugin_tcc892x.h>
#endif

#if defined(CONFIG_ARCH_TCC893X)
#include <mach/vioc_plugin_tcc893x.h>
#endif

#include <soc/tcc/pmap.h>
#include <video/tcc/viocmg.h>


#if defined(CONFIG_TCC_REAR_CAMERA_DRV) && !defined(CONFIG_TCC_VIOCMG)
#error "REAR CAMERA NEED CONFIG_TCC_VIOCMG"
#endif


#ifdef CONFIG_TCC_REAR_CAMERA_DRV
#include "tcc_hw_event.h"

#include "rear_cam_drv.h"
#include "tcc_rear_cam_ioctl.h"

#define REAR_CAM_DEBUG 		0
#define dprintk(msg...) 	if(REAR_CAM_DEBUG) { printk("REAR_CAM: " msg); }
#define DEVICE_NAME             "rear_cam_ctrl"

static unsigned	int rcamMajorNum;
struct device* p_rcam_dev;

static char banner[] __initdata = KERN_INFO "TCC Rear Camera Driver Initializing. \n";
static struct class *rcam_class;
static int device_error;

struct rear_cam_soc *rear_cam_soc;

extern int range_is_allowed(unsigned long pfn, unsigned long size);

static int tccxxx_rcam_mmap(struct file *file, struct vm_area_struct *vma)
{
        if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
                printk(KERN_ERR	"tccxxx_rcam_mmap: this address is not allowed. \n");
                return -EAGAIN;
        }

        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
                printk(KERN_ERR	"tccxxx_rcam_mmap: this address is not allowed. \n");
                return -EAGAIN;
        }

        vma->vm_ops	=  NULL;
		vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

        return 0;
}


long tccxxx_rcam_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        static unsigned long status_temp;

        switch(cmd) {

                case RCAM_DIRECT_ON:
                        RCAM_OnOff(1);
                        break;

                case RCAM_DIRECT_OFF:
                        RCAM_OnOff(0);
                        break;

                case RCAM_LINE_ON:
                        rear_cam_soc->mStatus |= (RCAM_ST_PARKING_LINE);
                        RCAM_LINE_OnOff(1);
                        break;

                case RCAM_LINE_OFF:
                        rear_cam_soc->mStatus &= (~RCAM_ST_PARKING_LINE);
                        RCAM_LINE_OnOff(0);
                        break;

                case RCAM_LINEBUF_ADDR:
                        if(copy_to_user((void __user *)arg,(const void *)&rear_cam_soc->mLineBufAddr[0], sizeof(unsigned long)) != 0)
                                return -EFAULT;				
                        break;
		  case RCAM_LINEBUF_ADDR_WITH_INDEX:
			{
			   RCAM_LINE_BUFFER_INFO rcam_line_buffer_info;

			   if(copy_from_user((void *)&rcam_line_buffer_info, (const void *)arg, sizeof(RCAM_LINE_BUFFER_INFO)) < 0){
                                        printk(" >> failed copy_from_user\r\n");
                                        return -EFAULT; 
                        }
			   RCAM_GetLineBufAddrByIndex(&rcam_line_buffer_info);
			   
                        if(copy_to_user((void __user *)arg,(const void *)&rcam_line_buffer_info, sizeof(RCAM_LINE_BUFFER_INFO)) != 0)
                                return -EFAULT;		
                	}  
			   break;
                case RCAM_RELEASE_VIDEO:
                        RCAM_ReleaseVideo();
                        rear_cam_soc->mStatus |= (RCAM_ST_RELEASE_VIDEO);
                        break;

                case RCAM_WAIT_REAR_EVENT:
                        RCAM_WaitEvent();
                        status_temp =  rear_cam_soc->mStatus | rear_cam_soc->mRearEventStatus;
                        if(copy_to_user((void __user *)arg,(const void *)&status_temp, sizeof(unsigned long)) != 0)
                                return -EFAULT;
                        break;

                case RCAM_GET_REAR_STATUS:
                        status_temp =  (viocmg_get_rear_cam_mode() == VIOCMG_REAR_MODE_RUN)?1:0;
                        if(copy_to_user((void __user *)arg,(const void *)&status_temp, sizeof(unsigned long)) != 0)
                                return -EFAULT;
                        break;
                case RCAM_REAR_EVENT_SET:
                        RCAM_SetRearEvent(1);
                        rear_cam_soc->mStatus |= (RCAM_ST_RUN);
                break;

                case RCAM_REAR_EVENT_UNSET:
                        RCAM_SetRearEvent(0);
                        rear_cam_soc->mStatus &= (~RCAM_ST_RUN);
                        break;
                case RCAM_LINE_UPDATE:
			  {
				  RCAM_LINE_BUFFER_UPDATE_INFO rcam_line_buffer_update_info;
				  printk("RCAM_LINE_UPDATE !!\n");

				  if(copy_from_user((void *)&rcam_line_buffer_update_info, (const void *)arg, sizeof(RCAM_LINE_BUFFER_UPDATE_INFO)) < 0){
	                                printk(" >> failed copy_from_user\r\n");
	                                return -EFAULT;
	                        }
	                        RCAM_LineBufUpdate(&rcam_line_buffer_update_info);
			   }
                        break;

                case RCAM_SCALE_OVERSCAN:
                        {
                                RCAM_SCALE_OVERSCAN_INFO rcam_scale_overscan;
                                if(copy_from_user((void *)&rcam_scale_overscan, (const void *)arg, sizeof(RCAM_SCALE_OVERSCAN_INFO)) < 0){
                                        printk(" >> failed copy_from_user\r\n");
                                        return -EFAULT; 
                                }
                                RCAM_SetScalerOverscan(rcam_scale_overscan.overscan_position_x, rcam_scale_overscan.overscan_position_y, rcam_scale_overscan.overscan_width_factor, rcam_scale_overscan.overscan_height_factor);
                        }
                        break;
		  case RCAM_LINE_SET_POSITION:
		  	  {
				RCAM_LINE_POSITION_INFO rcam_line_position_info;

				
				if(copy_from_user((void *)&rcam_line_position_info, (const void *)arg, sizeof(RCAM_LINE_POSITION_INFO)) < 0){
						printk(" >> failed copy_from_user\r\n");
                                        	return -EFAULT; 
                            }

				RCAM_LineSetPosition(&rcam_line_position_info);		   
		  	}	
			break;
		case RCAM_LINE_SET_SIZE:
			{
				RCAM_LINE_SIZE_INFO rcam_line_size_info;
				dprintk("RCAM_LINE_SET_SIZE\n");

				if(copy_from_user((void *)&rcam_line_size_info, (const void *)arg, sizeof(RCAM_LINE_SIZE_INFO)) < 0){
					printk(" >> failed copy_from_user\r\n");
                                   return -EFAULT; 
                            }

				dprintk("width : %ld height : %ld\n", rcam_line_size_info.width, rcam_line_size_info.height);
				RCAM_LineSetSize(&rcam_line_size_info);		
			}
			break;
		case RCAM_PREVIEW_SET_POSITION:
			{
				RCAM_PREVIEW_POSITION_INFO rcam_preview_position_info;
					
				if(copy_from_user((void *)&rcam_preview_position_info, (const void *)arg, sizeof(RCAM_PREVIEW_POSITION_INFO)) < 0){
						printk(" >> failed copy_from_user\r\n");
	                                       	return -EFAULT; 
                           	}
				RCAM_PreviewSetPosition(rcam_preview_position_info.preview_position_x, rcam_preview_position_info.preview_position_y);
			}
			break;
		case RCAM_PREVIEW_SET_SIZE:
			{
				RCAM_PREVIEW_SIZE_INFO rcam_preview_size_info;

				dprintk("RCAM_PREVIEW_SET_SIZE start\n");
				
				if(copy_from_user((void *)&rcam_preview_size_info, (const void *)arg, sizeof(RCAM_PREVIEW_SIZE_INFO)) < 0){
						printk(" >> failed copy_from_user\r\n");
	                                       	return -EFAULT; 
                           	}
				
				RCAM_PreviewSetSize(rcam_preview_size_info.preview_width, rcam_preview_size_info.preview_height);
				dprintk("RCAM_PREVIEW_SET_SIZE end\n");
			}
			break;
#ifdef USING_EARLY_CAM_CM4
		case RCAM_HANDOVER:
		{
			int cm4_gear_status;
			dprintk("RCAM_HANDOVER start \n");

			if(copy_from_user((void *)&cm4_gear_status, (const void *)arg, sizeof(int)) < 0){
				printk(" >> failed copy_from_user\r\n");
				return -EFAULT;
			}

			return RCAM_Handover(cm4_gear_status);
		}
#endif
		default:
                        printk(KERN_ALERT "not supported RCAM IOCTL(0x%x). \n", cmd);
                break;			
        }

        return 0;
}


int tccxxx_rcam_release(struct inode *inode, struct file *filp)
{
        int ret = 0;
        dprintk("rcam_release \n");
        rear_cam_soc->mFileOpenCount = 0;
        return ret;
}


int tccxxx_rcam_open(struct inode *inode, struct file *filp)
{	
        int ret = 0;

        if(rear_cam_soc->mFileOpenCount < 1){
                dprintk("rcam_open \n");
                rear_cam_soc->mFileOpenCount++;
        }
        else
                dprintk("rcam_open : too much file open\n");

        return ret;	
}

ssize_t tccxxx_rcam_write( struct file *filp, const char *buf, size_t count, loff_t *f_pos )
{
        dprintk("rcam_write Null\n");

        return count;
}

ssize_t tccxxx_rcam_read( struct file *filp, char *buf, size_t count, loff_t *f_pos )
{
	unsigned long temp;
	
	dprintk("rcam_read status \n");

	temp = rear_cam_soc->mStatus |rear_cam_soc->mRearEventStatus;

	if(copy_to_user( buf, (char*)&temp, sizeof(unsigned long)) != 0)
		return -EFAULT;

	return sizeof(unsigned long);
}


static struct file_operations tcc_rcam_fops = {
	.owner 				= THIS_MODULE,
	.unlocked_ioctl 	= tccxxx_rcam_ioctl,
	.mmap 			= tccxxx_rcam_mmap,
	.open 			= tccxxx_rcam_open,
	.release 	        = tccxxx_rcam_release,
	.read			= tccxxx_rcam_read,
	.write 			= tccxxx_rcam_write
};



void tccxxx_rcam_cleanup(void)
{
	device_destroy(rcam_class, MKDEV(rcamMajorNum, 0));
	class_destroy(rcam_class);
	unregister_chrdev(rcamMajorNum, DEVICE_NAME);

	return;
}

int tccxxx_rcam_init(struct platform_device *pdev)
{
	int ret = -1;

	printk(banner);

	rcamMajorNum = register_chrdev(0, DEVICE_NAME, &tcc_rcam_fops);
	if (rcamMajorNum < 0) {
		printk("%s: device failed with %d\n", __func__, rcamMajorNum);
	}
	else{
		rcam_class = class_create(THIS_MODULE, DEVICE_NAME);
		if (IS_ERR(rcam_class)) {
			goto END_PROCESS_1;
		}
		else {
                        p_rcam_dev = device_create(rcam_class,NULL,MKDEV(rcamMajorNum, 0),NULL,DEVICE_NAME);

			if(p_rcam_dev == NULL){
                                goto END_PROCESS_2;
			}
			else {
				ret = RCAM_Initialize(pdev);
				if(ret < 0){
                                        printk("%s: RCAM_Initialize fail \n", __func__);
                                        goto END_PROCESS_3;
				}
			}
		}
	}
        ret = 0;
        
        return ret;
        
END_PROCESS_3:
        tccxxx_rcam_cleanup();
        
END_PROCESS_2:
        class_destroy(rcam_class);
        
END_PROCESS_1:
        unregister_chrdev(rcamMajorNum, DEVICE_NAME);
        
	return ret;
}

/* Platform device Initialize */

static int tcc_rcam_suspend(struct platform_device *pdev, pm_message_t state)
{
        rear_cam_soc->mSuspend = 1;
        viocmg_set_rear_cam_suspend(1);
        RCAM_ReleaseVideo();
        return 0;
}

static int tcc_rcam_resume(struct platform_device *pdev)
{
        rear_cam_soc->mSuspend = 0;
        viocmg_set_rear_cam_suspend(0);
        RCAM_ExternalINT_POL_change();
        RCAM_IsPreviewShow(RCAM_COLD_BOOT);
	return 0;
}

static int tcc_rcam_probe(struct platform_device *pdev)
{
	int ret = 0;
       
	device_error = 1;

        if(!viocmg_is_feature_rear_cam_enable()) {
                ret = -ENODEV;
                pr_err(" >> rear camera feature is not enabled.!!!\r\n");
                goto END_PROCESS;
        }
        
        rear_cam_soc = devm_kzalloc(&pdev->dev, sizeof(struct rear_cam_soc),GFP_KERNEL);
        if (!rear_cam_soc) {
                printk("viocmg: out of memory\r\n");
                ret = -ENOMEM;
                goto END_PROCESS;
        }

        platform_set_drvdata(pdev, rear_cam_soc);

        rear_cam_soc->dev = &pdev->dev;
                
	ret = tccxxx_rcam_init(pdev);
	if(ret < 0){
                pr_err("tccxxx_rcam_init error\n");
		goto END_PROCESS;
	}
	device_error = 0;	

        ret = 0;

        printk("tcc_rcam_probe..!!\r\n");

        return ret;
        
END_PROCESS:
        if(rear_cam_soc)
                devm_kfree(&pdev->dev, rear_cam_soc);
        
	return ret;
}

static int tcc_rcam_remove(struct platform_device *pdev)
{	if(device_error == 0){	
		dprintk("tcc_rcam_remove\n");
		tccxxx_rcam_cleanup();
                if(rear_cam_soc)
                        devm_kfree(&pdev->dev, rear_cam_soc);
	}
	return 0;
}


#ifdef CONFIG_OF
static struct of_device_id rear_camera_of_match[] = {
        {.compatible = "telechips,rear-camera-drv"},
        {}
};
MODULE_DEVICE_TABLE(of,rear_camera_of_match);
#endif


static struct platform_driver tcc_rcam_driver = {
	.probe		= tcc_rcam_probe,
	.remove		= tcc_rcam_remove,
	.suspend	= tcc_rcam_suspend,
	.resume 	= tcc_rcam_resume,
	.driver		= {
		.name = "tcc-rcam",
		.owner = THIS_MODULE,
                #ifdef CONFIG_OF
                .of_match_table = of_match_ptr(rear_camera_of_match),
                #endif
                },
};


static int __init tcc_rcam_init(void)
{
        printk(KERN_INFO " %s\n", __func__);
	return platform_driver_register(&tcc_rcam_driver);
}

static void __exit tcc_rcam_exit(void)
{
	platform_driver_unregister(&tcc_rcam_driver);
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC REAR CAM Driver");
MODULE_LICENSE("LGPL");

/*
arch_initcall(tccxxx_rcam_init);
module_exit(tccxxx_rcam_cleanup);
*/
//postcore_initcall(tcc_rcam_init);
arch_initcall(tcc_rcam_init);
module_exit(tcc_rcam_exit);
#endif 

