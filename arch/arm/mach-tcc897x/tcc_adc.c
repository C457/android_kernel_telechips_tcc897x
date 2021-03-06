/****************************************************************************
 * tcc_adc.c
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <asm/io.h>

#include <mach/tcc_adc.h>

#ifdef CONFIG_DAUDIO_INFO
#include <mach/daudio_info.h>
#endif

#define adc_dbg(_adc, msg...) dev_dbg((_adc)->dev, msg)

#define adc_readl		__raw_readl
#define adc_writel		__raw_writel

struct adc_device {
	struct device		*dev;
	struct clk		*fclk;
	struct clk		*hclk;		
	struct clk		*phyclk;		
	struct tcc_adc_client	*cur;
	void __iomem		*regs;
	void __iomem		*pmu_regs;
	u32			is_12bit_res;
	u32			delay;
	u32			ckin;
};

static struct mutex		lock;
static struct adc_device	*adc_dev;
static spinlock_t		adc_spin_lock;

static void adc_pmu_power_ctrl(struct adc_device *adc, int pwr_on)
{
	unsigned reg_values;
	BUG_ON(!adc);
	reg_values = adc_readl(adc->pmu_regs);
	reg_values &= ~((1<<PMU_TSADC_PWREN_SHIFT) | (1<<PMU_TSADC_STOP_SHIFT));

	if (pwr_on) {
		clk_prepare_enable(adc->phyclk);
		reg_values |= (1<<PMU_TSADC_PWREN_SHIFT) | (0<<PMU_TSADC_STOP_SHIFT);
		adc_writel(reg_values, adc->pmu_regs);
	}
	else {
		reg_values |= (0<<PMU_TSADC_PWREN_SHIFT) | (1<<PMU_TSADC_STOP_SHIFT);
		adc_writel(reg_values, adc->pmu_regs);
		clk_prepare_enable(adc->phyclk);
	}
#if 0
	BITSET(pPMU->CONTROL, 1<<16/*HwPMU_CONTROL_APEN*/);
	BITCLR(pPMU->CONTROL, 1<<17/*HwPMU_CONTROL_ASTM*/);
	BITCLR(pPMU->CONTROL, 1<<18/*HwPMU_CONTROL_AISO*/);
#endif
}

static void adc_power_ctrl(struct adc_device *adc, int pwr_on)
{
	unsigned reg_values;

	if (pwr_on) {
		unsigned long clk_rate;
		int ps_val;
		adc_pmu_power_ctrl(adc, 1);
		
		clk_prepare_enable(adc->fclk);
		clk_prepare_enable(adc->hclk);

		/* adc delay */
		reg_values = adc_readl(adc->regs+ADCDLY_REG);
		reg_values = (reg_values&(~ADCDLY_DELAY_MASK))|ADCDLY_DELAY(adc->delay);
		adc_writel(reg_values, adc->regs+ADCDLY_REG);

		/* adc control */
		clk_rate = clk_get_rate(adc->fclk);
		ps_val = (clk_rate+adc->ckin/2)/adc->ckin - 1;
		BUG_ON((ps_val < 4) || (ps_val > ADCCON_PS_VAL_MASK));
		reg_values = ADCCON_PS_EN | ADCCON_PS_VAL(ps_val) | ADCCON_STBY;
		if (adc->is_12bit_res)
			reg_values |= ADCCON_12BIT_RES;
		adc_writel(reg_values, adc->regs+ADCCON_REG);

		/* adctsc control */
		reg_values = adc_readl(adc->regs+ADCTSC_REG);
		reg_values = ((reg_values&(~ADCTSC_MASK))| ADCTSC_PUON | ADCTSC_XPEN | ADCTSC_YPEN);
		adc_writel(reg_values, adc->regs+ADCTSC_REG);

		//pTSADC->ADCCON.bREG.STBY = 0;
	}
	else {
		/* set stand-by mode */
		reg_values = adc_readl(adc->regs+ADCCON_REG) | ADCCON_STBY;
		adc_writel(reg_values, adc->regs+ADCCON_REG);

		if(adc->fclk)
			clk_disable_unprepare(adc->fclk);
		if(adc->hclk)
			clk_disable_unprepare(adc->hclk);

		adc_pmu_power_ctrl(adc, 0);
	}
}

static inline unsigned long adc_read(struct adc_device *adc, int ch)
{
	unsigned long data;
	unsigned reg_values;

	/* change operation mode */
	reg_values = adc_readl(adc->regs+ADCCON_REG);
	reg_values &= ~ADCCON_STBY;
	adc_writel(reg_values, adc->regs+ADCCON_REG);

	if (ch > ADC_CH10)
		ch = 0;
	else {
		/* clear input channel select */
		reg_values &= ~(ADCCON_ASEL(15));
		adc_writel(reg_values, adc->regs+ADCCON_REG);
		mdelay(1);
	}

	reg_values |= ADCCON_ASEL(ch)|ADCCON_EN_ST;
	adc_writel(reg_values, adc->regs+ADCCON_REG);
	/* Wait for Start Bit Cleared */
	while (adc_readl(adc->regs+ADCCON_REG) & ADCCON_EN_ST)
		ndelay(5);
	/* Wait for ADC Conversion Ended */
	while (!(adc_readl(adc->regs+ADCCON_REG) & ADCCON_E_FLG))
		ndelay(5);

	/* Read Measured data */
	if (adc->is_12bit_res)
		data = adc_readl(adc->regs+ADCDAT0_REG)&0xFFF;
	else
		data = adc_readl(adc->regs+ADCDAT0_REG)&0x3FF;

	/* clear input channel select. & disable en_st */
	reg_values &= ~(ADCCON_ASEL(15)|ADCCON_EN_ST);
	adc_writel(reg_values, adc->regs+ADCCON_REG);

	/* chagne stand-by mode */
	reg_values |= ADCCON_STBY;
	adc_writel(reg_values, adc->regs+ADCCON_REG);

	return data;
}

unsigned long tcc_adc_getdata(struct tcc_adc_client *client)
{
	struct adc_device *adc = adc_dev;
	BUG_ON(!client);

	if (!adc) {
		printk(KERN_ERR "%s: failed to find adc device\n", __func__);
		return -EINVAL;
	}

	spin_lock(&adc_spin_lock);
	client->data = adc_read(adc, client->channel);
	spin_unlock(&adc_spin_lock);

	return client->data;
}
EXPORT_SYMBOL_GPL(tcc_adc_getdata);

struct tcc_adc_client *tcc_adc_register(struct device *dev, int ch)
{
	struct tcc_adc_client *client;

	BUG_ON(!dev);
	/* adc ch6/7/8/9 are touchscreen chanells. */
//	BUG_ON(ch>ADC_CH5 && ch<ADC_CH10);
//	BUG_ON(ch>ADC_TOUCHSCREEN && ch<ADC_CH10);
	BUG_ON(ch>ADC_CH10);

	client = devm_kzalloc(dev, sizeof(struct tcc_adc_client), GFP_KERNEL);
	if (!client) {
		 dev_err(dev, "no memory for adc client\n");
		 return NULL;
	}
	client->dev = dev;
	client->channel = ch;
	return client;
}
EXPORT_SYMBOL_GPL(tcc_adc_register);

void tcc_adc_release(struct tcc_adc_client *client)
{
	if (client) {
		devm_kfree(client->dev, client);
		client = NULL;
	}
}
EXPORT_SYMBOL_GPL(tcc_adc_release);

spinlock_t *get_adc_spinlock(void)
{
	return &adc_spin_lock;
}
EXPORT_SYMBOL_GPL(get_adc_spinlock);

static int tcc_adc_probe(struct platform_device *pdev)
{
	struct adc_device *adc;
	u32 clk_rate;
	int ret = -ENODEV;
	printk("\x1b[1;33m[%s:%d]\x1b[0m\n", __func__, __LINE__);

	adc = devm_kzalloc(&pdev->dev, sizeof(struct adc_device), GFP_KERNEL);
	if (adc == NULL) {
		dev_err(&pdev->dev, "failed to allocate adc_device\n");
		return -ENOMEM;
	}
	adc->dev = &pdev->dev;

	/* get adc/pmu register addresses */
	adc->regs = of_iomap(pdev->dev.of_node, 0);
	if (!adc->regs) {
		dev_err(&pdev->dev, "failed to get adc registers\n");
		ret = -ENXIO;
		goto err_get_reg_addrs;
	}
	adc->pmu_regs = of_iomap(pdev->dev.of_node, 1);
	if (!adc->pmu_regs) {
		dev_err(&pdev->dev, "failed to get pmu registers\n");
		ret = -ENXIO;
		goto err_get_reg_addrs;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk_rate);
	if (ret) {
		dev_err(&pdev->dev, "Can not get clock frequency value\n");
		goto err_get_property;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "ckin-frequency", &adc->ckin);
	if (ret) {
		dev_err(&pdev->dev, "Can not get adc ckin value\n");
		goto err_get_property;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "adc-delay", &adc->delay);
	if (ret) {
		dev_err(&pdev->dev, "Can not get ADC Dealy value\n");
		goto err_get_property;
	}
	adc->is_12bit_res = of_property_read_bool(pdev->dev.of_node, "adc-12bit-resolution");

	/* get peri/io_hclk/pmu_ipisol clocks */
	adc->fclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(adc->fclk))
		goto err_fclk;
	else
		clk_set_rate(adc->fclk, (unsigned long)clk_rate);
	adc->hclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(adc->hclk))
		goto err_hclk;
	adc->phyclk = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(adc->phyclk))
		goto err_phyclk;

	platform_set_drvdata(pdev, adc);

	mutex_init(&lock);
	spin_lock_init(&adc_spin_lock);

	adc_power_ctrl(adc, 1);
	dev_info(&pdev->dev, "attached driver\n");
	adc_dev = adc;

#if defined(CONFIG_DAUDIO_INFO) && !defined(CONFIG_HIBERNATION)
	printk(KERN_ERR "%s\n", __func__);
	get_daudio_rev();
#endif

	return 0;

err_phyclk:
	clk_put(adc->hclk);
err_hclk:
	clk_put(adc->fclk);
err_fclk:

err_get_property:

err_get_reg_addrs:
	devm_kfree(&pdev->dev, adc);
	return ret;
}

static int tcc_adc_remove(struct platform_device *pdev)
{
	struct adc_device *adc = platform_get_drvdata(pdev);
	adc_power_ctrl(adc, 0);
	devm_kfree(&pdev->dev, adc);
	return 0;
}

static int tcc_adc_suspend(struct device *dev)
{
	struct adc_device *adc = dev_get_drvdata(dev);
	adc_power_ctrl(adc, 0);
	return 0;
}

static int tcc_adc_resume(struct device *dev)
{
	struct adc_device *adc = dev_get_drvdata(dev);
	adc_power_ctrl(adc, 1);
#if defined(CONFIG_DAUDIO_INFO) && defined(CONFIG_HIBERNATION)
	printk(KERN_ERR "%s\n", __func__);
	get_daudio_rev();
#endif
	return 0;
}

static SIMPLE_DEV_PM_OPS(adc_pm_ops, tcc_adc_suspend, tcc_adc_resume);

#ifdef CONFIG_OF
static const struct of_device_id tcc_adc_dt_ids[] = {
	{.compatible = "telechips,adc",},
	{}
};
#else
#define tcc_adc_dt_ids NULL
#endif

static struct platform_driver tcc_adc_driver = {
	.driver	= {
		.name	= "tcc-adc",
		.owner	= THIS_MODULE,
		.pm	= &adc_pm_ops,
		.of_match_table	= of_match_ptr(tcc_adc_dt_ids),
	},
	.probe	= tcc_adc_probe,
	.remove	= tcc_adc_remove,
};

module_platform_driver(tcc_adc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Telechips Corporation");

