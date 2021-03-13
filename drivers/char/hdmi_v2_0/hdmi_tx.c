/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_tx.c
*  \brief       HDMI TX controller driver
*  \details   
*  \version     1.0
*  \date        2014-2015
*  \copyright
This source code contains confidential information of Telechips.
Any unauthorized use without a written  permission  of Telechips including not 
limited to re-distribution in source  or binary  form  is strictly prohibited.
This source  code is  provided "AS IS"and nothing contained in this source 
code  shall  constitute any express  or implied warranty of any kind, including
without limitation, any warranty of merchantability, fitness for a   particular 
purpose or non-infringement  of  any  patent,  copyright  or  other third party 
intellectual property right. No warranty is made, express or implied, regarding 
the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability 
arising from, out of or in connection with this source  code or the  use in the 
source code. 
This source code is provided subject  to the  terms of a Mutual  Non-Disclosure 
Agreement between Telechips and Company.
*******************************************************************************/
#include "include/hdmi_includes.h"
#include "include/hdmi_log.h"
#include "include/irq_handlers.h"
#include "include/proc_fs.h"
#include "include/hdmi_ioctls.h"

#include "hdmi_api_lib/include/core/irq.h"
#include "hdmi_api_lib/include/phy/phy.h"

#include <linux/clk.h>

unsigned int hdmi_log_level = SNPS_WARN;


/** @short License information */
MODULE_LICENSE("GPL v2");
/** @short Author information */
MODULE_AUTHOR("Telechips");
/** @short Device description */
MODULE_DESCRIPTION("HDMI_TX module driver");
/** @short Device version */
MODULE_VERSION("1.0");


//MISC Funcations
extern int dwc_hdmi_misc_register(struct hdmi_tx_dev *dev);
extern int dwc_hdmi_misc_deregister(struct hdmi_tx_dev *dev);

// API Functions
extern void dwc_hdmi_api_register(struct hdmi_tx_dev *dev);



/**
 * @short List of the devices
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/**
 * @short List of allocated memory
 * Linked list that contains the allocated memory
 */
static struct mem_alloc *alloc_list;

/**
 * @short Map memory blocks
 * @param[in,out] main HDMI TX structure
 * @param[in] Device node pointer
 * @return Return -ENOMEM if one of the blocks is not mapped and 0 if all
 * blocks are mapped successful.
 */
int
of_parse_hdmi_dt(struct hdmi_tx_dev *dev, struct device_node *node){
        int ret = 0;
        unsigned int of_loop;

        struct device_node *ddibus_np, *ckc_np;
        
	// Map DWC HDMI TX Core
	dev->dwc_hdmi_tx_core_io = of_iomap(node, PROTO_HDMI_TX_CORE);
	if(!dev->dwc_hdmi_tx_core_io){
		pr_err("%s:Unable to map resource\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;

	}

	// Map HDCP
	dev->hdcp_io = of_iomap(node, PROTO_HDMI_TX_HDCP);       // HDCP_ADDRESS,
	if(!dev->hdcp_io){
		pr_err("%s:Unable to map hdcp_base_addr resource\n", FUNC_NAME);
		ret = -ENODEV;
                goto end_process;
	}
        
	// Map HDMI TX PHY interface
	dev->hdmi_tx_phy_if_io = of_iomap(node, PROTO_HDMI_TX_PHY); // TXPHY_IF_ADDRESS,
	if(!dev->hdmi_tx_phy_if_io){
		pr_err("%s:Unable to map hdmi_tx_phy_if_base_addr resource\n",
				FUNC_NAME);
                ret = -ENODEV;
                goto end_process;

	}
        
        

        // Find DDI_BUS Node
        ddibus_np = of_find_compatible_node(NULL, NULL, "telechips,ddi_config");
        if(!ddibus_np) {
                pr_err("%s:Unable to map ddibus resource\n",
                                FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }

        // Map DDI_Bus interface                
        dev->ddibus_io = of_iomap(ddibus_np, 0);
        if(!dev->ddibus_io){
                pr_err("%s:Unable to map ddibus_io base address resource\n",
                                FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }
		
        ckc_np = of_find_compatible_node(NULL, NULL, "telechips,ckc");
        if(!ckc_np) {
                pr_err("%s:Unable to map ckc resource\n",
                                FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }
		
        dev->io_bus = of_iomap(ckc_np, 5);
        if(!dev->io_bus){
                pr_err("%s:Unable to map io_bus base address resource\n",
                                FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }

		// Parse Interrupt
        for(of_loop = HDMI_IRQ_TX_CORE; of_loop < HDMI_IRQ_TX_MAX; of_loop++) {
                dev->irq[of_loop] = of_irq_to_resource(node, of_loop, NULL);
        }

        // Parse Clock 
        for(of_loop = HDMI_CLK_INDEX_PHY27M; of_loop < HDMI_CLK_INDEX_MAX; of_loop++) {
                dev->clk[of_loop] = of_clk_get(node, of_loop);
                if (IS_ERR(dev->clk[of_loop])) {
                        pr_err("%s:Unable to map hdmi clock (%d)\n", 
                                FUNC_NAME, of_loop);
                        ret = -ENODEV;
                        goto end_process;
                }
        }

        // Parse Clock Freq
        ret = of_property_read_u32(node, "clock-frequency", &dev->clk_freq_pclk);
        if(ret < 0) {
                pr_err("%s:Unable to map hdmi pclk frequency\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }


        ret = of_property_read_u32(node, "audio_src_offset", &dev->audio_offset);
        if(ret < 0) {
                pr_err("%s:Unable to map audio source offset\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }



        // HDMI Display Device
        of_property_read_u32(node, "DisplayDevice", &dev->display_device);
        
        // Update the verbose level
        of_property_read_u32(node, "verbose", &dev->verbose);
        if(dev->verbose >= VERBOSE_BASIC)
                        pr_info("%s: verbose level is %d\n", FUNC_NAME, dev->verbose);
        
end_process:

	return ret;
}


/**
 * @short Map memory blocks
 * @param[in,out] main HDMI TX structure
 * @param[in] Device node pointer
 * @return Return -ENOMEM if one of the blocks is not mapped and 0 if all
 * blocks are mapped successful.
 */
int
of_parse_i2c_mapping(struct device_node *node){
        int ret = 0;
        struct device_node *i2c_np;
        volatile void __iomem   *hdmi_i2c_io = NULL;
        unsigned int i2c_mapping_reg, i2c_mapping_mask, i2c_mapping_val;
        volatile unsigned int val;
        
        // Map HDMI I2C Mapping interface
        i2c_np = of_parse_phandle(node, "hdmi_i2c_mapping", 0);
        if (!i2c_np) {
                pr_err("%s:Unable to map i2c_mapping node\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }
      
        // Map HDMI TX PHY interface
        hdmi_i2c_io = of_iomap(i2c_np, 1); // I2C Port Config,
        if(!hdmi_i2c_io){
                pr_err("%s:Unable to map i2c mapping resource\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }
        ret = of_property_read_u32_index(node, "hdmi_i2c_mapping_val", 0, &i2c_mapping_reg);
        if(ret < 0) {
                pr_err("%s:Unable to map i2c_mapping_reg\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }

        ret = of_property_read_u32_index(node, "hdmi_i2c_mapping_val", 1, &i2c_mapping_mask);
        if(ret < 0) {
                pr_err("%s:Unable to map i2c_mapping_mask\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }

        ret = of_property_read_u32_index(node, "hdmi_i2c_mapping_val", 2, &i2c_mapping_val);
        if(ret < 0) {
                pr_err("%s:Unable to map i2c_mapping_val\n", FUNC_NAME);
                ret = -ENODEV;
                goto end_process;
        }

        //pr_info("of_parse_i2c_mapping reg=0x%x, mask=0x%x, val=0x%x\r\n", i2c_mapping_reg, i2c_mapping_mask, i2c_mapping_val);
        val = ioread32((void*)(hdmi_i2c_io + i2c_mapping_reg));
        val &= ~i2c_mapping_mask;
        val |= i2c_mapping_val;
        iowrite32(val, (void*)(hdmi_i2c_io + i2c_mapping_reg));
        
        
end_process:

        if(hdmi_i2c_io)
                iounmap(hdmi_i2c_io);
      
        return ret;
}


/**
 * @short Release memory blocks
 * @param[in,out] main HDMI TX structure
 * @return Void
 */
void
release_memory_blocks(struct hdmi_tx_dev *dev){
        if(dev->ddibus_io)
                iounmap(dev->ddibus_io);

        if(dev->hdmi_tx_phy_if_io)
                iounmap(dev->hdmi_tx_phy_if_io);

        if(dev->hdcp_io)
                iounmap(dev->hdcp_io);

        if(dev->dwc_hdmi_tx_core_io)
                iounmap(dev->dwc_hdmi_tx_core_io);
}

/**
 * @short Allocate memory
 * @param[in] info String to associate with memory allocated
 * @param[in] size Size of the memory to allocate
 * @param[in,out] allocated Pointer to the structure that contains the info
 * about the allocation
 * @return Void
 */
void *
alloc_mem(char *info, size_t size, struct mem_alloc *allocated){
        struct mem_alloc *new = NULL;
        int *return_pnt = NULL;

        // first time
        if(alloc_list == NULL){
                alloc_list = kzalloc(sizeof(struct mem_alloc), GFP_KERNEL);
                if(!alloc_list){
                        printk( KERN_ERR "%s:Couldn't create alloc_list\n",
                        FUNC_NAME);
                        return NULL;
                }
                alloc_list->instance = 0;
                alloc_list->info = "allocation list - instance 0";
                alloc_list->size = 0;
                alloc_list->pointer = NULL;
                alloc_list->last = NULL;
                alloc_list->prev = NULL;
        }

        // alloc pretended memory
        return_pnt = kzalloc(size, GFP_KERNEL);
        if(!return_pnt){
                printk( KERN_ERR "%s:Couldn't allocate memory: %s\n",
                FUNC_NAME, info);
                return NULL;
        }

        // alloc memory for the infostructure
        new = kzalloc(sizeof(struct mem_alloc), GFP_KERNEL);
        if(!new){
                printk( KERN_ERR "%s:Couldn't allocate memory for the "
                "alloc_mem\n", FUNC_NAME);
                kfree(return_pnt);
                return NULL;
        }

        new->instance = ++alloc_list->instance;
        new->info = info;
        new->size = size;
        alloc_list->size += size;
        new->pointer = return_pnt;
        if(alloc_list->last == NULL){
                new->prev = alloc_list;	// First instance
        }
        else{
                new->prev = alloc_list->last;
        }
        alloc_list->last = new;
        new->last = new;

        return return_pnt;
}

/**
 * @short Free all memory
 * This was implemented this way so that all memory allocated was de-allocated
 * and to avoid memory leaks.
 * @return Void
 */
void
free_all_mem(void){
        if(alloc_list != NULL){
                while(alloc_list->instance != 0){
                        struct mem_alloc *this;
                        this = alloc_list->last;
                        // cut this from list
                        alloc_list->last = this->prev;
                        alloc_list->instance--;
                        alloc_list->size -= this->size;
                        // free allocated memory
                        kfree(this->pointer);
                        // free this memory
                        printk( KERN_INFO "%s:Freeing: %s\n",
                        FUNC_NAME, this->info);
                        kfree(this);
                }
                kfree(alloc_list);
                alloc_list = NULL;
        }
}

/**
 * @short Initialization routine - Entry point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */



static int
hdmi_tx_init(struct platform_device *pdev){
        int error = 0;
        int i, ret = 0;
        struct hdmi_tx_dev *dev = NULL;

        pr_info("****************************************\n");
        pr_info("%s:Installing SNPS HDMI module\n", FUNC_NAME);
        pr_info("****************************************\n");

        pr_info("%s:Device registration\n", FUNC_NAME);
        dev = alloc_mem("HDMI TX Device", sizeof(struct hdmi_tx_dev), NULL);
        if(!dev){
                pr_err("%s:Could not allocated hdmi_tx_dev\n", FUNC_NAME);
                return -ENOMEM;
        }

        // Zero the device
        memset(dev, 0, sizeof(struct hdmi_tx_dev));

        // Update the device node
        dev->parent_dev = &pdev->dev;

        dev->device_name = "HDMI_TX";
        pr_info("%s:Driver's name '%s'\n", FUNC_NAME, dev->device_name);

        spin_lock_init(&dev->slock);

        dev->videoParam = (void*)devm_kzalloc(dev->parent_dev, sizeof(videoParams_t), GFP_KERNEL);
        dev->audioParam = (void*)devm_kzalloc(dev->parent_dev, sizeof(audioParams_t), GFP_KERNEL);
        dev->productParam = (void*)devm_kzalloc(dev->parent_dev, sizeof(productParams_t), GFP_KERNEL);
        dev->hdcpParam = (void*)devm_kzalloc(dev->parent_dev, sizeof(hdcpParams_t), GFP_KERNEL);

        // always data enable polarity is 1. 
        dev->hdmi_tx_ctrl.data_enable_polarity = 1;
        dev->hdmi_tx_ctrl.csc_on = 1;
        dev->hdmi_tx_ctrl.audio_on = 1;


        // Initialize IRQ Enable List..
        INIT_LIST_HEAD(&dev->irq_enable_entry.list);
        
        // Map memory blocks
        pr_info("%s:Map memory blocks\n", FUNC_NAME);
        if(of_parse_hdmi_dt(dev, pdev->dev.of_node) < 0){
                pr_info("%s:Map memory blocks failed\n", FUNC_NAME);
                goto free_mem;
        }

        if(of_parse_i2c_mapping(pdev->dev.of_node) < 0){
                pr_info("%s:Map i2c mapping failed\n", FUNC_NAME);
                goto free_mem;
        }


        #if defined(CONFIG_REGULATOR)
        //dev->regulator = regulator_get(dev->parent_dev, "hdmi_voltage");
        #endif

        // Disable All Interrupts
        dwc_init_interrupts(dev);
        
        // Proc file system
        pr_info("%s:Init proc file system @ /proc/hdmi_tx\n", FUNC_NAME);
        proc_interface_init(dev);

        // Now that everything is fine, let's add it to device list
        list_add_tail(&dev->devlist, &devlist_global);


        dwc_hdmi_api_register(dev);

        // Register MISC Driver. 
        dwc_hdmi_misc_register(dev);


        return ret;

free_mem:
        if(dev->videoParam) {
                devm_kfree(dev->parent_dev, dev->videoParam);
                dev->videoParam = NULL;
        }
        if(dev->audioParam) {
                devm_kfree(dev->parent_dev, dev->audioParam);
                dev->audioParam = NULL;
        }
        if(dev->productParam) {
                devm_kfree(dev->parent_dev, dev->productParam);
                dev->productParam = NULL;
        }
        if(dev->hdcpParam) {
                devm_kfree(dev->parent_dev, dev->hdcpParam);
                dev->hdcpParam = NULL;

        }
        release_memory_blocks(dev);
        free_all_mem();
        return error;
}


/**
 * @short Exit routine - Exit point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int
hdmi_tx_exit(struct platform_device *pdev){
        struct hdmi_tx_dev *dev;
        struct list_head *list;

        pr_info("**************************************\n");
        pr_info("%s:Removing SNPS HDMI module\n", FUNC_NAME);
        pr_info("**************************************\n");

        while(!list_empty(&devlist_global)){
                list = devlist_global.next;
                list_del(list);
                dev = list_entry(list, struct hdmi_tx_dev, devlist);

                if(dev == NULL){
                        continue;
                }

                if(dev->videoParam) {
                        devm_kfree(dev->parent_dev, dev->videoParam);
                        dev->videoParam = NULL;
                }

                if(dev->audioParam) {
                        devm_kfree(dev->parent_dev, dev->audioParam);
                        dev->audioParam = NULL;
                }
                
                if(dev->productParam) {
                        devm_kfree(dev->parent_dev, dev->productParam);
                        dev->productParam = NULL;
                }
                if(dev->hdcpParam) {
                        devm_kfree(dev->parent_dev, dev->hdcpParam);
                        dev->hdcpParam = NULL;
                
                }
                pr_info("%s:Remove proc file system\n", FUNC_NAME);
                proc_interface_remove(dev);

                dwc_deinit_interrupts(dev);

                // Deregister HDMI Misc driver. 
                dwc_hdmi_misc_deregister(dev);

                // Release memory blocks
                pr_info("%s:Release memory blocks\n", FUNC_NAME);
                release_memory_blocks(dev);



                free_all_mem();
        }
        return 0;
}

/** 
 * @short of_device_id structure
 */
static const struct of_device_id dw_hdmi_tx[] = {
        { .compatible =	"telechips,dw-hdmi-tx" },
        { }
};
MODULE_DEVICE_TABLE(of, dw_hdmi_tx);

/** 
 * @short Platform driver structure
 */
static struct platform_driver __refdata dwc_hdmi_tx_pdrv = {
        .remove = hdmi_tx_exit,
        .probe = hdmi_tx_init,
        .driver = {
                .name = "telechips,dw-hdmi-tx",
                .owner = THIS_MODULE,
                .of_match_table = dw_hdmi_tx,
        },
};

static __init int dwc_hdmi_init(void)
{
        printk("%s \n", __FUNCTION__);
        return platform_driver_register(&dwc_hdmi_tx_pdrv);
}

static __exit void dwc_hdmi_exit(void)
{
        printk("%s \n", __FUNCTION__);
        platform_driver_unregister(&dwc_hdmi_tx_pdrv);
}

module_init(dwc_hdmi_init);
module_exit(dwc_hdmi_exit);

