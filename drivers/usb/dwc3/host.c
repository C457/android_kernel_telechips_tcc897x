/**
 * host.c - DesignWare USB3 DRD Controller Host Glue
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/usb/xhci_pdriver.h>

#include "core.h"

/*
 * Registers should always be accessed with double word or quad word accesses.
 *
 * Some xHCI implementations may support 64-bit address pointers.  Registers
 * with 64-bit address pointers should be written to with dword accesses by
 * writing the low dword first (ptr[0]), then the high dword (ptr[1]) second.
 * xHCI implementations that do not support 64-bit address pointers will ignore
 * the high dword, and write order is irrelevant.
 */
static inline u64 dwc3_host_read_64(__le64 __iomem *regs)
{
	__u32 __iomem *ptr = (__u32 __iomem *) regs;
	u64 val_lo = readl(ptr);
	u64 val_hi = readl(ptr + 1);
	return val_lo + (val_hi << 32);
}
static inline void dwc3_host_write_64(const u64 val, __le64 __iomem *regs)
{
	__u32 __iomem *ptr = (__u32 __iomem *) regs;
	u32 val_lo = lower_32_bits(val);
	u32 val_hi = upper_32_bits(val);

	writel(val_lo, ptr);
	writel(val_hi, ptr + 1);
}

void dwc3_host_save_registers(struct dwc3 *dwc)
{
	printk("[%s:%d]\n", __func__, __LINE__);
	dwc->s3.command = readl(&dwc->op_regs->command);
	dwc->s3.dev_nt = readl(&dwc->op_regs->dev_notification);
	dwc->s3.dcbaa_ptr = dwc3_host_read_64(&dwc->op_regs->dcbaa_ptr);
	dwc->s3.config_reg = readl(&dwc->op_regs->config_reg);
	//dwc->s3.erst_size = readl(&dwc->ir_set->erst_size);
	//dwc->s3.erst_base = xhci_read_64(dwc, &dwc->ir_set->erst_base);
	//dwc->s3.erst_dequeue = xhci_read_64(dwc, &dwc->ir_set->erst_dequeue);
	//dwc->s3.irq_pending = readl(&dwc->ir_set->irq_pending);
	//dwc->s3.irq_control = readl(&dwc->ir_set->irq_control);

	dwc->s3_saved = 1;
}

void dwc3_host_restore_registers(struct dwc3 *dwc)
{
	if (dwc->s3_saved){
		printk("[%s:%d]\n", __func__, __LINE__);
		writel(dwc->s3.command, &dwc->op_regs->command);
		writel(dwc->s3.dev_nt, &dwc->op_regs->dev_notification);
		dwc3_host_write_64(dwc->s3.dcbaa_ptr, &dwc->op_regs->dcbaa_ptr);
		writel(dwc->s3.config_reg, &dwc->op_regs->config_reg);
		//writel(dwc->s3.erst_size, &dwc->ir_set->erst_size);
		//xhci_write_64(dwc, dwc->s3.erst_base, &dwc->ir_set->erst_base);
		//xhci_write_64(dwc, dwc->s3.erst_dequeue, &dwc->ir_set->erst_dequeue);
		//writel(dwc->s3.irq_pending, &dwc->ir_set->irq_pending);
		//writel(dwc->s3.irq_control, &dwc->ir_set->irq_control);
	}
}


int dwc3_host_init(struct dwc3 *dwc)
{
	struct platform_device	*xhci;
	struct usb_xhci_pdata	pdata;
	int			ret;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(dwc->dev, "couldn't allocate xHCI device\n");
		ret = -ENOMEM;
		goto err0;
	}

	dma_set_coherent_mask(&xhci->dev, dwc->dev->coherent_dma_mask);

	xhci->dev.parent	= dwc->dev;
	xhci->dev.dma_mask	= dwc->dev->dma_mask;
	xhci->dev.dma_parms	= dwc->dev->dma_parms;

	dwc->xhci = xhci;

	ret = platform_device_add_resources(xhci, dwc->xhci_resources,
						DWC3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(dwc->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	memset(&pdata, 0, sizeof(pdata));

#ifdef CONFIG_DWC3_HOST_USB3_LPM_ENABLE
	pdata.usb3_lpm_capable = 1;
#endif

	ret = platform_device_add_data(xhci, &pdata, sizeof(pdata));
	if (ret) {
		dev_err(dwc->dev, "couldn't add platform data to xHCI device\n");
		goto err1;
	}

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(dwc->dev, "failed to register xHCI device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(xhci);

err0:
	return ret;
}

void dwc3_host_exit(struct dwc3 *dwc)
{
	platform_device_unregister(dwc->xhci);
}
