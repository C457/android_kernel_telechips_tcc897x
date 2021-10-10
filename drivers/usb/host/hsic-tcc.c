/*
 * linux/drivers/usb/host/hsic-tcc.c
 *
 * Description: EHCI HCD (Host Controller Driver) for USB.
 *              Bus Glue for Telechips-SoC
 *
 *  Copyright (C) 2009 Atmel Corporation,
 *                     Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 *  Modified for Telechips SoC from ehci-atmel.c
 *                     by Telechips Team Linux <linux@telechips.com> 25-01-2011
 *
 *  Based on various ehci-*.c drivers
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include "tcc-hcd.h"
#include <asm/io.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/dma-mapping.h>
#include "ehci.h"

#define DRIVER_DESC "USB 2.0 'Enhanced' Host Controller (HSIC) Driver"

static const char hcd_name[] = "hsic-tcc";

#define tcc_ehci_readl(r)		readl(r)
#define tcc_ehci_writel(v,r)		writel(v,r)

typedef struct _HSICPHYCFG
{
	volatile unsigned int       bcfg;
	volatile unsigned int       pcfg0;
	volatile unsigned int       pcfg1;
	volatile unsigned int       pcfg2;
	volatile unsigned int       pcfg3;
	volatile unsigned int       pcfg4;
	volatile unsigned int       sts;
	volatile unsigned int       lcfg0;
	volatile unsigned int       lcfg1;
} HSICPHYCFG, *PHSICPHYCFG;

struct tcc_ehci_hcd {
	struct ehci_hcd *ehci;
	struct device			*dev;
	int	hosten_ctrl_able;
	int host_en_gpio;
	int sq_level_ctl;

	int vbus_ctrl_able;
	int vbus_gpio;

	unsigned int phy_type;
	unsigned int hcd_id;
	unsigned int vbus_status;

	int vbus_source_ctrl;
	struct regulator *vbus_source;

	struct clk *hclk;
	struct clk *pclk;
	struct clk *phy_clk;
	unsigned int core_clk_rate;
	unsigned int core_clk_rate_phy;
	struct clk *isol;

	struct tcc_usb_phy *phy;
	struct usb_phy *transceiver;
	int host_resumed;
	int port_resuming;

	/* USB PHY */
	void __iomem		*phy_regs;		/* device memory/io */
	resource_size_t		phy_rsrc_start;	/* memory/io resource start */
	resource_size_t		phy_rsrc_len;	/* memory/io resource length */

	unsigned int hcd_tpl_support;	/* TPL support */
};

extern unsigned long usb_hcds_loaded;
extern const struct hc_driver* get_ehci_hcd_driver(void);

static int tcc_ehci_parse_dt(struct platform_device *pdev, struct tcc_ehci_hcd *tcc_ehci);
static void tcc_USB20HPHY_cfg(struct tcc_ehci_hcd *tcc_ehci);

static void tcc_ehci_phy_ctrl(struct tcc_ehci_hcd *tcc_ehci, int on_off)
{
	if(on_off == ON) {
		if(tcc_ehci->isol) {
			if(clk_prepare_enable(tcc_ehci->isol) != 0) {
				dev_err(tcc_ehci->dev,
				"can't do usb 2.0 phy enable\n");
			}
		}
		if(tcc_ehci->phy_clk) {
			if(clk_prepare_enable(tcc_ehci->phy_clk) != 0) {
				dev_err(tcc_ehci->dev,
					"can't do usb 2.0 phy clk clock enable\n");
			}
			clk_set_rate(tcc_ehci->phy_clk, tcc_ehci->core_clk_rate_phy);
		}
	}
	else if(on_off == OFF) {
		if(tcc_ehci->phy_clk) {
			clk_disable_unprepare(tcc_ehci->phy_clk);
		}
		if(tcc_ehci->isol) {
			clk_disable_unprepare(tcc_ehci->isol);
		}
	}
}

/*-------------------------------------------------------------------------*/
static int tcc_ehci_clk_ctrl(struct tcc_ehci_hcd *tcc_ehci, int on_off)
{
	if(on_off == ON) {
		if(tcc_ehci->hclk) {
			if(clk_prepare_enable(tcc_ehci->hclk) != 0) {
				dev_err(tcc_ehci->dev,
					"can't do usb 2.0 hclk clock enable\n");
				return -1;
			}
		}
		if(tcc_ehci->pclk) {
			if(clk_prepare_enable(tcc_ehci->pclk) != 0) {
				dev_err(tcc_ehci->dev,
					"can't do usb 2.0 hclk clock enable\n");
				return -1;
			}
			clk_set_rate(tcc_ehci->pclk, tcc_ehci->core_clk_rate);
		}
		//printk("usb host 2.0 clk rate: %d\n", tcc_ehci->core_clk_rate);
	} else {
		if(tcc_ehci->hclk) 
			clk_disable_unprepare(tcc_ehci->hclk);
		if(tcc_ehci->pclk)
			clk_disable_unprepare(tcc_ehci->pclk);
	}

	return 0;
}

static int tcc_ehci_vbus_ctrl(struct tcc_ehci_hcd *tcc_ehci, int on_off);
static ssize_t ehci_tcc_vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tcc_ehci_hcd *tcc_ehci = dev_get_drvdata(dev);

	return sprintf(buf, "ehci vbus - %s\n",(tcc_ehci->vbus_status) ? "on":"off");
}

static ssize_t ehci_tcc_vbus_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct tcc_ehci_hcd *tcc_ehci = dev_get_drvdata(dev);

	if (!strncmp(buf, "on", 2)) {
		tcc_ehci_vbus_ctrl(tcc_ehci, ON);
	}

	if (!strncmp(buf, "off", 3)) {
		tcc_ehci_vbus_ctrl(tcc_ehci, OFF);
	}

	return count;
}

static DEVICE_ATTR(vbus, S_IRUGO | S_IWUSR, ehci_tcc_vbus_show, ehci_tcc_vbus_store);

/* Group all the eHCI SQ attributes */
static struct attribute *usb_sq_attrs[] = {
		&dev_attr_vbus.attr,
		NULL,
};

static struct attribute_group usb_sq_attr_group = {
	.name = NULL,	/* we want them in the same directory */
	.attrs = usb_sq_attrs,
};

/*-------------------------------------------------------------------------*/
static void tcc_HSIC_PHY_init(struct tcc_ehci_hcd *tcc_ehci)
{
	int i;
	PHSICPHYCFG hsic_phy = tcc_ehci->phy_regs;

	clk_reset(tcc_ehci->hclk, 1);
	udelay(10);
	clk_reset(tcc_ehci->hclk, 0);

	// Reset PHY Registers
	tcc_ehci_writel(0x83000015, &hsic_phy->pcfg0);
	tcc_ehci_writel(0x0000003A, &hsic_phy->pcfg1);
	tcc_ehci_writel(0x00000090, &hsic_phy->pcfg2);
	tcc_ehci_writel(0x00000000, &hsic_phy->pcfg3);
	tcc_ehci_writel(0x00001400, &hsic_phy->pcfg4);
	tcc_ehci_writel(0x30048020, &hsic_phy->lcfg0);
	tcc_ehci_writel(0x00000000, &hsic_phy->lcfg1);

	clk_reset(tcc_ehci->hclk, 1);
	// Set the POR
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->pcfg0) | Hw31, &hsic_phy->pcfg0);
	// Set the Core Reset
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->lcfg0) & 0xCFFFFFFF, &hsic_phy->lcfg0);

	// Wait 10 usec
	udelay(10);

	// Release POR
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->pcfg0) & ~(Hw31), &hsic_phy->pcfg0);
	// Clear SIDDQ
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->pcfg0) & ~(Hw24), &hsic_phy->pcfg0);
	// Set Phyvalid en
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->pcfg4) | Hw30, &hsic_phy->pcfg4);
	// Set DP/DM (pull down)
	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->pcfg4) | 0x1400, &hsic_phy->pcfg4);

	tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->lcfg0) | 0x30000000, &hsic_phy->lcfg0);

	clk_reset(tcc_ehci->hclk, 0);

	// Wait Phy Valid Interrupt
	i = 0;
	while (i < 10000) {
	   if ((tcc_ehci_readl(&hsic_phy->pcfg0) & Hw21)) break;
	   i++;
	   udelay(5);
	}
	printk("PHY valid check %s\x1b[0m\n",i>=9999?"fail!":"pass.");

	// Release Core Reset
	//tcc_ehci_writel(tcc_ehci_readl(&hsic_phy->lcfg0) & 0x30000000, &hsic_phy->lcfg0);
}

static void tcc_USB20HPHY_cfg(struct tcc_ehci_hcd *tcc_ehci)
{
	tcc_HSIC_PHY_init(tcc_ehci);
}

static int tcc_ehci_power_ctrl(struct tcc_ehci_hcd *tcc_ehci, int on_off)
{
	int err = 0;

	if (tcc_ehci->vbus_source_ctrl == 1) {
		if(on_off == ON) {
			if (tcc_ehci->vbus_source) {
				err = regulator_enable(tcc_ehci->vbus_source);
				if(err) {
					dev_err(tcc_ehci->dev,
						"can't enable vbus source\n");
					return err;
				}
			}
		}
	}

	if (tcc_ehci->hosten_ctrl_able == 1) {
		err = gpio_direction_output(tcc_ehci->host_en_gpio, 1);	/* Don't control gpio_hs_host_en because this power also supported in USB core. */
		if(err) {
			dev_err(tcc_ehci->dev,
				"can't enable host\n");
			return err;
		}
	}

	if (tcc_ehci->vbus_source_ctrl == 1) {
		if(on_off == OFF)
			if (tcc_ehci->vbus_source)
				regulator_disable(tcc_ehci->vbus_source);
	}
	
	return err;
}

static int tcc_ehci_vbus_ctrl(struct tcc_ehci_hcd *tcc_ehci, int on_off)
{
	int err = 0;

	if (tcc_ehci->vbus_ctrl_able == 1) {
		err = gpio_direction_output(tcc_ehci->vbus_gpio, on_off);
		if(err) {
			dev_err(tcc_ehci->dev,
				"can't enable vbus\n");
			return err;
		}
		tcc_ehci->vbus_status = on_off;
	}
	
	return err;
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int tcc_ehci_suspend(struct device *dev)
{
	struct tcc_ehci_hcd *tcc_ehci =	dev_get_drvdata(dev);
	struct usb_hcd *hcd = ehci_to_hcd(tcc_ehci->ehci);
	bool do_wakeup = device_may_wakeup(dev);

	ehci_suspend(hcd, do_wakeup);

	/* Telechips specific routine */
	tcc_ehci_phy_ctrl(tcc_ehci, OFF);
	tcc_ehci_clk_ctrl(tcc_ehci, OFF);
	tcc_ehci_vbus_ctrl(tcc_ehci, OFF);
	tcc_ehci_power_ctrl(tcc_ehci, OFF);

	return 0;
}

static int tcc_ehci_resume(struct device *dev)
{
	struct tcc_ehci_hcd *tcc_ehci =	dev_get_drvdata(dev);
	struct usb_hcd *hcd = ehci_to_hcd(tcc_ehci->ehci);

	/* Telechips specific routine */
	tcc_ehci_power_ctrl(tcc_ehci, ON);
	tcc_ehci_phy_ctrl(tcc_ehci, ON);
	tcc_ehci_vbus_ctrl(tcc_ehci, ON);
	tcc_ehci_clk_ctrl(tcc_ehci, ON);
	tcc_USB20HPHY_cfg(tcc_ehci);

	ehci_resume(hcd, false);

	return 0;
}

static const struct dev_pm_ops ehci_tcc_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(tcc_ehci_suspend,tcc_ehci_resume)
};

#define EHCI_TCC_PMOPS &ehci_tcc_pmops
#else
#define EHCI_TCC_PMOPS NULL
#endif	/* CONFIG_PM */

static int ehci_tcc_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct tcc_ehci_hcd *tcc_ehci;
	const struct hc_driver *driver = get_ehci_hcd_driver();
	struct resource *res;
	struct resource *res1;
	int irq;
	int retval;

	if (usb_disabled())
		return -ENODEV;

	retval = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (retval)
		return retval;

	tcc_ehci = devm_kzalloc(&pdev->dev, sizeof(struct tcc_ehci_hcd), GFP_KERNEL);
	if (!tcc_ehci) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	/* Parsing the device table */
	retval = tcc_ehci_parse_dt(pdev, tcc_ehci);
	if(retval != 0){
		if(retval != -1)
			printk(KERN_ERR "ehci-tcc: Device table parsing failed\n");
		retval = -EIO;
		goto fail_create_hcd;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_create_hcd;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	platform_set_drvdata(pdev, tcc_ehci);
	tcc_ehci->dev = &(pdev->dev);

	/* USB ECHI Base Address*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_request_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;
	hcd->regs = devm_ioremap(&pdev->dev, res->start, hcd->rsrc_len);
	
	/* USB PHY (UTMI) Base Address*/
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_request_resource;
	}
	tcc_ehci->phy_rsrc_start = res1->start;
	tcc_ehci->phy_rsrc_len = res1->end - res1->start + 1;
	tcc_ehci->phy_regs = devm_ioremap(&pdev->dev, res1->start, tcc_ehci->phy_rsrc_len);

	tcc_ehci_clk_ctrl(tcc_ehci, ON);
	/* USB HS Phy Enable */
	tcc_ehci_phy_ctrl(tcc_ehci, ON);

	/* USB HOST Power Enable */
	if (tcc_ehci_power_ctrl(tcc_ehci, ON) != 0) {
		printk(KERN_ERR "ehci-tcc: USB HOST VBUS Ctrl failed\n");
		retval = -EIO;
		goto fail_request_resource;
	}
	
	tcc_USB20HPHY_cfg(tcc_ehci);

	tcc_ehci_vbus_ctrl(tcc_ehci, ON);

	/* ehci setup */
	tcc_ehci->ehci = hcd_to_ehci(hcd);
	tcc_ehci->ehci->caps = hcd->regs;		/* registers start at offset 0x0 */
	tcc_ehci->ehci->regs = hcd->regs + HC_LENGTH(tcc_ehci->ehci, ehci_readl(tcc_ehci->ehci, &tcc_ehci->ehci->caps->hc_capbase));
	tcc_ehci->ehci->hcs_params = ehci_readl(tcc_ehci->ehci, &tcc_ehci->ehci->caps->hcs_params);	/* cache this readonly data; minimize chip reads */

	/* TPL Support Set */
	hcd->tpl_support = tcc_ehci->hcd_tpl_support;
	
	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;
	//disp_ehci(res->start);

	retval = sysfs_create_group(&pdev->dev.kobj, &usb_sq_attr_group);
	if (retval < 0) {
		printk(KERN_ERR "Cannot register USB SQ sysfs attributes: %d\n",
		       retval);
		goto fail_add_hcd;
	}

	return retval;

fail_add_hcd:
	tcc_ehci_clk_ctrl(tcc_ehci, OFF);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n",
		dev_name(&pdev->dev), retval);
	
	return retval;
}

static int __exit ehci_tcc_drv_remove(struct platform_device *pdev)
{
	struct tcc_ehci_hcd *tcc_ehci = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tcc_ehci->ehci);

    sysfs_remove_group(&pdev->dev.kobj, &usb_sq_attr_group);

	//ehci_shutdown(hcd);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	tcc_ehci_phy_ctrl(tcc_ehci, OFF);
	tcc_ehci_clk_ctrl(tcc_ehci, OFF);
	tcc_ehci_vbus_ctrl(tcc_ehci, OFF);
	tcc_ehci_power_ctrl(tcc_ehci, OFF);

	gpio_free(tcc_ehci->host_en_gpio);
	gpio_free(tcc_ehci->vbus_gpio);	

	if (tcc_ehci->vbus_source) {
		regulator_disable(tcc_ehci->vbus_source);
		regulator_put(tcc_ehci->vbus_source);
	}
	
	return 0;
}

static const struct of_device_id tcc_hsic_match[] = {
	{ .compatible = "telechips,tcc-hsic" },
	{},
};
MODULE_DEVICE_TABLE(of, tcc_hsic_match);

static int tcc_ehci_parse_dt(struct platform_device *pdev, struct tcc_ehci_hcd *tcc_ehci)
{
	int err = 0;

	//===============================================
	// Check Host enable pin	
	//===============================================
	if (of_find_property(pdev->dev.of_node, "hosten-ctrl-able", 0)) {
		tcc_ehci->hosten_ctrl_able = 1;

		tcc_ehci->host_en_gpio = of_get_named_gpio(pdev->dev.of_node,
						"hosten-gpio", 0);
		if(!gpio_is_valid(tcc_ehci->host_en_gpio)){
			dev_err(&pdev->dev, "can't find dev of node: host en gpio\n");
			return -ENODEV;
		}

		err = gpio_request(tcc_ehci->host_en_gpio, "host_en_gpio");
		if(err) {
			dev_err(&pdev->dev, "can't requeest host_en gpio\n");
			return err;
		}
	} else {
		tcc_ehci->hosten_ctrl_able = 0;	// can not control vbus
	}

	//===============================================
	// Check vbus enable pin	
	//===============================================
	if (of_find_property(pdev->dev.of_node, "vbus-ctrl-able", 0)) {
		tcc_ehci->vbus_ctrl_able = 1;

		tcc_ehci->vbus_gpio = of_get_named_gpio(pdev->dev.of_node,
						"vbus-gpio", 0);
		if(!gpio_is_valid(tcc_ehci->vbus_gpio)) {
			dev_err(&pdev->dev, "can't find dev of node: vbus gpio\n");
			return -ENODEV;
		}

		err = gpio_request(tcc_ehci->vbus_gpio, "vbus_gpio");
		if(err) {
			dev_err(&pdev->dev, "can't requeest vbus gpio\n");
			return err;
		}
	} else {
		tcc_ehci->vbus_ctrl_able = 0;	// can not control vbus
	}

	//===============================================
	// Check VBUS Source enable	
	//===============================================
	if (of_find_property(pdev->dev.of_node, "vbus-source-ctrl", 0)) {
		tcc_ehci->vbus_source_ctrl = 1;

		tcc_ehci->vbus_source = regulator_get(&pdev->dev, "vdd_v5p0");
		if (IS_ERR(tcc_ehci->vbus_source)) {
			dev_err(&pdev->dev, "failed to get ehci vdd_source\n");
			tcc_ehci->vbus_source = NULL;
		}
	} else {
		tcc_ehci->vbus_source_ctrl = 0;
	}
	
	tcc_ehci->hclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(tcc_ehci->hclk))
		tcc_ehci->hclk = NULL;

	tcc_ehci->isol = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_ehci->isol))
		tcc_ehci->isol = NULL;

	tcc_ehci->pclk = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(tcc_ehci->pclk))
		tcc_ehci->pclk = NULL;

	tcc_ehci->phy_clk = of_clk_get(pdev->dev.of_node, 3);
	if (IS_ERR(tcc_ehci->phy_clk))
		tcc_ehci->phy_clk = NULL;

	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &tcc_ehci->core_clk_rate);
	of_property_read_u32(pdev->dev.of_node, "clock-frequency-phy", &tcc_ehci->core_clk_rate_phy);

	return err;		
}

static void tcc_ehci_hcd_shutdown(struct platform_device *pdev)
{
	struct tcc_ehci_hcd *tcc_ehci = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tcc_ehci->ehci);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver hsic_tcc_driver = {
	.probe		= ehci_tcc_drv_probe,
	.remove		= __exit_p(ehci_tcc_drv_remove),
	.shutdown	= tcc_ehci_hcd_shutdown,
	.driver = {
		.name	= "tcc-hsic",
		.owner	= THIS_MODULE,
		.pm		= EHCI_TCC_PMOPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tcc_hsic_match),
#endif
	}
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE ("GPL");

static int __init tcc_ehci_hcd_init(void)
{
	int retval = 0;
	
	set_bit(USB_EHCI_LOADED, &usb_hcds_loaded);
	retval = platform_driver_register(&hsic_tcc_driver);
	if (retval < 0)
		clear_bit(USB_EHCI_LOADED, &usb_hcds_loaded);

	return retval;
}
module_init(tcc_ehci_hcd_init);

static void __exit tcc_ehci_hcd_cleanup(void)
{
	platform_driver_unregister(&hsic_tcc_driver);
	clear_bit(USB_EHCI_LOADED, &usb_hcds_loaded);
}
module_exit(tcc_ehci_hcd_cleanup);

