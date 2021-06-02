/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        hdmi_cec.c
*  \brief       HDMI CEC controller driver
*  \details   
*  \version     1.0
*  \date        2016
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

#include"include/hdmi_cec.h"
#include"include/hdmi_cec_misc.h"
#include"include/tcc_cec_interface.h"

#include<linux/clk.h>

/** @short License information */
MODULE_LICENSE("GPL v2");
/** @short Author information */
MODULE_AUTHOR("Telechips");
/** @short Device description */
MODULE_DESCRIPTION("HDMI_CEC module driver");
/** @short Device version */
MODULE_VERSION("1.0");

/**
 * @short List of the devices
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

extern void * alloc_mem(char *info, size_t size, struct mem_alloc *allocated);
extern void free_all_mem(void);

static int hdmi_cec_probe(struct platform_device *pdev){
        int error = 0;
        int i, ret = 0;
        struct cec_device *dev = NULL;

        printk("%s:Device registration\n", __func__);
        dev = alloc_mem("HDMI CEC Device", sizeof(struct cec_device), NULL);
        if(!dev){
                pr_err("%s:Could not allocated hdmi_tx_dev\n", __func__);
                return -ENOMEM;
        }

        // Zero the device
        memset(dev, 0, sizeof(struct cec_device));

        // Update the device node
        dev->parent_dev = &pdev->dev;

        dev->device_name = "HDMI_CEC";
		
        printk("%s:Driver's name '%s'\n", __func__, dev->device_name);



		if(pdev->dev.of_node){
			dev->cec_core_io = of_iomap(pdev->dev.of_node, 0);
			dev->cec_irq = irq_of_parse_and_map(pdev->dev.of_node,0);
			dev->cec_wake_up_irq = irq_of_parse_and_map(pdev->dev.of_node,1);
			
			for(i = HDMI_CLK_CEC_INDEX_CORE; i < HDMI_CLK_CEC_INDEX_MAX; i++)
				dev->clk[i] = of_clk_get(pdev->dev.of_node,i);

			if(dev->clk[HDMI_CLK_CEC_INDEX_CORE]) {
				clk_set_rate(dev->clk[HDMI_CLK_CEC_INDEX_CORE], HDMI_CEC_CORE_CLK_RATE);
				clk_prepare_enable(dev->clk[HDMI_CLK_CEC_INDEX_CORE]);
			}

			if(dev->clk[HDMI_CLK_CEC_INDEX_SFR]) {
				clk_set_rate(dev->clk[HDMI_CLK_CEC_INDEX_SFR], HDMI_CEC_SFR_CLK_RATE);
				clk_prepare_enable(dev->clk[HDMI_CLK_CEC_INDEX_SFR]);
			}

		} else {
			printk("%s: device node is null!!!",__func__);
			goto free_mem;
		}

		hdmi_cec_misc_register(dev);


		if(hdmi_cec_request_irq(dev) < 0)
		{
			printk("%s: IRQ request fail!!!",__func__);
			goto free_mem;
		}
	

		TccCECInterface_Init();

		if(dev->clk[HDMI_CLK_CEC_INDEX_CORE])
			clk_disable(dev->clk[HDMI_CLK_CEC_INDEX_CORE]);

		if(dev->clk[HDMI_CLK_CEC_INDEX_SFR])
			clk_disable(dev->clk[HDMI_CLK_CEC_INDEX_SFR]);

        // Now that everything is fine, let's add it to device list
        list_add_tail(&dev->devlist, &devlist_global);
		
        return ret;

free_mem:

        if(dev->cec_core_io)
        	iounmap(dev->cec_core_io);

        free_all_mem();
        return error;
}


/**
 * @short Exit routine - Exit point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int hdmi_cec_remove(struct platform_device *pdev){
    struct cec_device *dev;
    struct list_head *list;

     while(!list_empty(&devlist_global)){
        list = devlist_global.next;
        list_del(list);
        dev = list_entry(list, struct cec_device , devlist);

        if(dev == NULL){
                continue;
        }

        hdmi_cec_misc_deregister(dev);

		free_irq(dev->cec_irq, dev);

        if(dev->cec_core_io)
        	iounmap(dev->cec_core_io);

        free_all_mem();
     }
        return 0;
}


/** 
 * @short of_device_id structure
 */
static const struct of_device_id dw_hdmi_cec[] = {
        { .compatible =	"telechips,dw-hdmi-cec" },
        { }
};
MODULE_DEVICE_TABLE(of, dw_hdmi_cec);

/** 
 * @short Platform driver structure
 */
static struct platform_driver __refdata dwc_hdmi_cec_pdrv = {
        .remove = hdmi_cec_remove,
        .probe = hdmi_cec_probe,
        .driver = {
                .name = "telechips,dw-hdmi-cec",
                .owner = THIS_MODULE,
                .of_match_table = dw_hdmi_cec,
        },
};

static __init int dwc_hdmi_cec_init(void)
{
        printk("%s \n", __FUNCTION__);
        return platform_driver_register(&dwc_hdmi_cec_pdrv);
}

static __exit void dwc_hdmi_cec_exit(void)
{
        printk("%s \n", __FUNCTION__);
        platform_driver_unregister(&dwc_hdmi_cec_pdrv);
}

module_init(dwc_hdmi_cec_init);
module_exit(dwc_hdmi_cec_exit);