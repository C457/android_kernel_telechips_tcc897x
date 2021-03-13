/* ICTC driver skeleton */

/*
 * TODO : 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <linux/ictc.h>

#define ictc_readl		__raw_readl
#define ictc_writel		__raw_writel

#define ICTC_EN					(1<<31)
#define TCLK_EN					(1<<20)
#define FLT_CNT_EN				(1<<19)
#define TO_CNT_EN				(1<<18)
#define FEDGE_CNT_EN			(1<<17)
#define PD_CMP_CNT_EN			(1<<16)

#define FLT_CNT_FULL_IRQ		(1<<0)
#define TO_CNT_FULL_IRQ			(1<<1)
#define FEDGE_CNT_FULL_IRQ		(1<<2)
#define CMP_DT_ERR_IRQ			(1<<3)
#define CMP_PRD_ERR_IRQ			(1<<4)
#define PD_CMP_CNT_FULL_IRQ		(1<<5)

static struct ictc_platform_data ictc_resource;
static void __iomem *ictc_en_base;
static void __iomem *ictc_ctrl_base;
static void __iomem	*ictc_irq_base;
static u32	f_in_sel;

static irqreturn_t ictc_interrupt_handler(int irq, void *dev_id)
{
	printk("\x1b[1;35m[%s:%d] ICTC interrupt handler \x1b[0m\n", __func__, __LINE__);

	if( ictc_readl(ictc_irq_base)&FLT_CNT_FULL_IRQ )
		printk("FLT_CNT_FULL_IRQ \n");
	if( ictc_readl(ictc_irq_base)&TO_CNT_FULL_IRQ )
		printk("TO_CNT_FULL_IRQ \n");
	if( ictc_readl(ictc_irq_base)&FEDGE_CNT_FULL_IRQ )
		printk("FEDGE_CNT_FULL_IRQ\n");
	if( ictc_readl(ictc_irq_base)&CMP_DT_ERR_IRQ )
		printk("CMP_DT_ERR_IRQ\n");
	if( ictc_readl(ictc_irq_base)&CMP_PRD_ERR_IRQ )
		printk("CMP_PRD_ERR_IRQ\n");
	if( ictc_readl(ictc_irq_base)&PD_CMP_CNT_FULL_IRQ )
		printk("PD_CMP_CNT_FULL_IRQ\n");
	
	return IRQ_NONE;
}

static void ictc_start(void)
{
	printk("\x1b[1;31m[%s:%d]\x1b[0m\n", __func__, __LINE__);

	// Pre-Scaler Counter Enable
	ictc_writel( ictc_readl(ictc_en_base)|TCLK_EN, ictc_en_base );
	// Filter Counter Enable
	ictc_writel( ictc_readl(ictc_en_base)|FLT_CNT_EN, ictc_en_base );
	// TimeOut Counter Enable
	ictc_writel( ictc_readl(ictc_en_base)|TO_CNT_EN, ictc_en_base );
	// Falling Edge Counter Enable
	ictc_writel( ictc_readl(ictc_en_base)|FEDGE_CNT_EN, ictc_en_base );
	// Period/Duty CMP Counter Enable
	ictc_writel( ictc_readl(ictc_en_base)|PD_CMP_CNT_EN, ictc_en_base );
	
	// start ICTC
	ictc_writel( ictc_readl(ictc_en_base)|ICTC_EN, ictc_en_base );
}

static void ictc_stop(void)
{
	printk("\x1b[1;34m[%s:%d]\x1b[0m\n", __func__, __LINE__);

	// stop ICTC
	ictc_writel( ictc_readl(ictc_en_base)&(~ICTC_EN), ictc_en_base );
}

static int ictc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned int irq = platform_get_irq(pdev, 0);
	int ret = 0;

	printk("\x1b[1;32m[%s:%d]ICTC driver init \x1b[0m\n", __func__, __LINE__);

	if (!np) {
	    dev_err(&pdev->dev, "no platform data\n");
	    return -EINVAL;
	}
	
	ictc_en_base	= of_iomap(np, 0);
	ictc_ctrl_base	= of_iomap(np, 1);
	ictc_irq_base	= of_iomap(np, 2);

	if( of_property_read_u32(np, "f_in_sel", &f_in_sel) )
	{
		printk("%s: Can't read f_in_sel\n", __func__);
		f_in_sel = 0;
	}

	// set F_IN_SEL
	ictc_writel( 0, ictc_ctrl_base);
	ictc_writel( ictc_readl(ictc_ctrl_base)|f_in_sel, ictc_ctrl_base );

	ret = request_irq(irq, ictc_interrupt_handler, IRQF_SHARED, "ICTC", &ictc_resource);

	ictc_start();
	
	return 0;
}

static int ictc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ICTC_of_match[] = {
	{ .compatible = "telechips,ictc" },
	{}
};
MODULE_DEVICE_TABLE(of, ICTC_of_match);

static struct platform_driver ictc_driver = {
	.probe	= ictc_probe,
	.remove	= ictc_remove,
	.driver = {
		.name	= "ictc",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ICTC_of_match),
	}
};
module_platform_driver(ictc_driver);

static int __init ictc_init(void)
{
	platform_driver_register(&ictc_driver);
}

static int __init ictc_exit(void)
{
	ictc_stop();
	platform_driver_unregister(&ictc_driver);
}

module_init(ictc_init);
module_exit(ictc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Telechips Corporation");
MODULE_ALIAS("platform:ictc");
