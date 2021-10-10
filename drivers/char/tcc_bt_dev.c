/*
 * File:		drivers/char/tcc_bt_dev.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/device.h>

#include <asm/io.h>
#include <linux/delay.h>
#include <linux/tcc_bt_dev.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

int tcc_gpio_config(unsigned gpio, unsigned flags);

#if defined(CONFIG_ANDROID) && defined(CONFIG_PLATFORM_AVN) && defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK)
#include <mach/tcc_ckc.h>
#if defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK_0)
#define DAI_CLK_CTRL_AUD_ID PERI_ADAI0
#elif defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK_1)
#define DAI_CLK_CTRL_AUD_ID PERI_ADAI1
#elif defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK_2)
#define DAI_CLK_CTRL_AUD_ID PERI_ADAI2
#elif defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK_3)
#define DAI_CLK_CTRL_AUD_ID PERI_ADAI3
#endif
#define DAI_CLK_CTRL_STEP_SIZE 512
#endif

#ifndef ON
#define ON		1
#endif

#ifndef OFF
#define OFF		0
#endif

static int bt_reset_port = (int)NULL;

#define DEV_NAME "tcc_bt_dev"
static struct class *bt_dev_class;

typedef struct {
	int module;		// 0x12:CSR, 0x34:Broadcom
	int TMP1;
	int TMP2;
} tcc_bt_info_t;

static int tcc_bt_dev_open(struct inode *inode, struct file *file)
{
//	printk("[## BT ##] tcc_bt_dev_open\n");
	return 0;
}

static int tcc_bt_dev_release(struct inode *inode, struct file *file)
{
//	printk("[## BT ##] tcc_bt_dev_release\n");
	return 0;
}

static int tcc_bt_get_info(tcc_bt_info_t* arg)
{
	tcc_bt_info_t *info_t;
	int module_t;
	unsigned long ret;

	info_t = (tcc_bt_info_t *)arg;
	ret = copy_from_user(info_t, (tcc_bt_info_t *)arg, sizeof(tcc_bt_info_t));

	module_t = 0;

#if defined (CONFIG_TCC_CSR_BC0406_MODULE_SUPPORT)
	module_t = 0x12;
#elif defined (CONFIG_TCC_RDA_587X_MODULE_SUPPORT)
	module_t = 0x56;
#elif defined (CONFIG_TCC_BRCM_BCM4330_MODULE_SUPPORT)
	module_t = 0x34;
#elif defined (CONFIG_TCC_ATHEROS_AR3002_MODULE_SUPPORT)
	module_t = 0x78;
#endif

	printk("[## BT ##] module[0x%x]\n", module_t);

	info_t->module = module_t;

	ret = copy_to_user((tcc_bt_info_t *)arg, info_t, sizeof(tcc_bt_info_t));

	return 0;
}

static void tcc_bt_dev_gpio_init(void)
{
    printk("[## BT ##] tcc_bt_dev_gpio_init");

    tcc_gpio_config(bt_reset_port, (1 << 24));
    //mdelay(10);
    gpio_direction_output(bt_reset_port, 1);
    //mdelay(10);
}

static void tcc_bt_dev_reset_control(int value)
{
    printk("[## BT ##] tcc_bt_dev_reset_control");

    if(value != BT_DEV_GPIO_LOW)
        gpio_set_value(bt_reset_port, BT_DEV_GPIO_HI);
    else
        gpio_set_value(bt_reset_port, BT_DEV_GPIO_LOW);
}

static long tcc_bt_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int *parm1;
	int ret = 0;

	memset(&parm1, 0, sizeof(int));

	switch(cmd) {
		case IOCTL_BT_DEV_SPECIFIC:
			printk("[## BT ##] IOCTL_BT_DEV_SPECIFIC cmd[%x]\n", cmd);
			tcc_bt_get_info((tcc_bt_info_t*)&arg);
			break;

#if defined(CONFIG_ANDROID) && defined(CONFIG_PLATFORM_AVN) && defined(CONFIG_TCC_BT_DEV_CTRL_AUDIO_CLK)
		case IOCTL_BT_DEV_DAI_CLK_RAISE:
			printk("[## BT ##] IOCTL_BT_DEV_DAI_CLK_RAISE cmd[%x]\n", cmd);
			ret = tcc_ckc_adjust_audio_clk(DAI_CLK_CTRL_AUD_ID, DAI_CLK_CTRL_STEP_SIZE);
			break;

		case IOCTL_BT_DEV_DAI_CLK_REDUCE:
			printk("[## BT ##] IOCTL_BT_DEV_DAI_CLK_REDUCE cmd[%x]\n", cmd);
			ret = tcc_ckc_adjust_audio_clk(DAI_CLK_CTRL_AUD_ID, -DAI_CLK_CTRL_STEP_SIZE);
			break;

		case IOCTL_BT_DEV_DAI_CLK_RESTORE:
				printk("[## BT ##] IOCTL_BT_DEV_DAI_CLK_RESTORE cmd[%x]\n", cmd);
				tcc_ckc_restore_audio_clk(DAI_CLK_CTRL_AUD_ID);
			break;
#endif
		case IOCTL_BT_DEV_CLOCK_LIMIT:
			printk("[## BT ##] tcc_bt_dev_ioctl cmd[%x] arg[%x]\n", cmd, (int)arg);
			break;

        case IOCTL_BT_DEV_CTRL_INIT:
        	printk("[## BT ##] IOCTL_BT_DEV_CTRL_INIT cmd[%x]\n", cmd);
            tcc_bt_dev_gpio_init();
			break;

        case IOCTL_BT_DEV_CTRL_RESET:
            printk("[## BT ##] IOCTL_BT_DEV_CTRL_RESET cmd[%x]\n", cmd);
                     
	        if(arg != 0)
	            tcc_bt_dev_reset_control(BT_DEV_GPIO_HI);
	        else
	            tcc_bt_dev_reset_control(BT_DEV_GPIO_LOW);
			break;

		default :
			printk("[## BT ##] tcc_bt_dev_ioctl cmd[%x] arg[%x]\n", cmd, (int)arg);
			break;
	}

	return ret;
}

struct file_operations tcc_bt_dev_ops = {
	.owner				= THIS_MODULE,
	.unlocked_ioctl		= tcc_bt_dev_ioctl,
	.open				= tcc_bt_dev_open,
	.release			= tcc_bt_dev_release,
};

static struct of_device_id tcc_bluetooth_dt_match[] = {
#ifdef CONFIG_DAUDIO_KK
	{ .compatible = "telechips, tcc-bt-device", },
#else
	{ .compatible = "telechips, tcc-bluetooth", },
#endif
	{ },
};
MODULE_DEVICE_TABLE(of, tcc_bluetooth_dt_match);

static int tcc_bt_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;		
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = register_chrdev(BT_DEV_MAJOR_NUM, DEV_NAME, &tcc_bt_dev_ops);

	bt_dev_class = class_create(THIS_MODULE, DEV_NAME);
	device_create(bt_dev_class, NULL, MKDEV(BT_DEV_MAJOR_NUM, BT_DEV_MINOR_NUM), NULL, DEV_NAME);

	if(ret < 0){
		printk("[## BT ##] [%d]fail to register the character device\n", ret);
		return ret;
	}
	match = of_match_device(tcc_bluetooth_dt_match, &pdev->dev);
	if(!match)
	{
		printk("[## BT ##] of_match_device fail %s \n", __func__);
		return -EINVAL;
	}
			
	bt_reset_port = (int)NULL;
	if(np)
	{
		bt_reset_port = of_get_named_gpio(np, "bt_reset-gpio", 0);
		if(!gpio_is_valid(bt_reset_port)) {
			printk("[## BT ##] %s: err to get gpios: ret:%x\n", __func__, bt_reset_port);
			return -EINVAL;
		}
	}
	return 0;
}

static int tcc_bt_remove(struct platform_device *pdev)
{
	printk("[## BT ##] cleanup_module\n");
	unregister_chrdev(BT_DEV_MAJOR_NUM, DEV_NAME);
	return 0;
}

static struct platform_driver bluetooth_gpio_driver = {
	.probe		= tcc_bt_probe,
	.remove		= tcc_bt_remove,
	.driver 	= {
		.name 			= DEV_NAME,
		.owner 			= THIS_MODULE,
		.of_match_table = tcc_bluetooth_dt_match,
	},
};

static struct platform_device *bluetooth_gpio_device;

int __init tcc_bt_init(void)
{
	int retval;
	
	printk("[## BT ##] init_module\n");
#ifndef CONFIG_OF
	bluetooth_gpio_device = platform_device_alloc("bluetooth_gpio", -1);

	if(!bluetooth_gpio_device)
		return -ENOMEM;

	retval = platform_device_add(bluetooth_gpio_device);
	if(retval < 0)
		return retval;
#endif
	retval =  platform_driver_register(&bluetooth_gpio_driver);
#ifndef CONFIG_OF
	if(retval < 0)
		platform_device_unregister(bluetooth_gpio_device);
#endif
	return retval;
}

void __exit tcc_bt_exit(void)
{
	printk("[## BT ##] cleanup_module\n");
	platform_device_unregister(bluetooth_gpio_device);
	platform_driver_unregister(&bluetooth_gpio_driver);
}

module_init(tcc_bt_init);
module_exit(tcc_bt_exit);

MODULE_AUTHOR("Telechips Inc. linux@telechips.com");
MODULE_DESCRIPTION("TCC_BT_DEV");
MODULE_LICENSE("GPL");
