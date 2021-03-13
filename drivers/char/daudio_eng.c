#include <asm/io.h>
#include <asm/setup.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <mach/reg_physical.h>
#include <mach/structures_smu_pmu.h>
#include <asm/uaccess.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>

#include <mach/daudio_eng.h>
#include <mach/daudio_settings.h>

static int daudio_eng_debug = 0;

#define dbg(f, a...)  if (daudio_eng_debug) printk(KERN_INFO "[cleinsoft em] " f, ##a)
#define __func dbg("%s\n", __func__);

static struct class *daudio_eng_class;
static struct device *daudio_eng_device;

em_setting_info daudio_em_info;

#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend daudio_eng_early_suspend;
#endif

#if defined(CONFIG_SND_SOC_FM1288)
static DEFINE_MUTEX(daudio_eng_lock);
#endif

static DEFINE_MUTEX(daudio_eng_setting_lock);

#if defined(CONFIG_DAUDIO_INFO_CMDLINE)
#define parse_board_em_setting_info_id(em_info_id,value) \
	daudio_em_info.id = value
#define parse_board_em_setting_info_mode(em_info_mode,value) \
	daudio_em_info.mode = value

static int __init parse_cmdline_em_setting_info_id(char* sz)
{
	int id_value;
	sscanf(sz, "%d", &id_value);
	parse_board_em_setting_info_id(id, id_value);

	dbg("em_setting_info id : 0x%llx \n", daudio_em_info.id);

	return 0;
}

static int __init parse_cmdline_em_setting_info_mode(char* sz)
{
	int mode_value;
	sscanf(sz, "%d", &mode_value);
	parse_board_em_setting_info_mode(mode, mode_value);

	dbg("em_setting_info mode : 0x%x \n", daudio_em_info.mode);

	return 0;
}

__setup("daudio_info_id=", parse_cmdline_em_setting_info_id);
__setup("daudio_info_mode=", parse_cmdline_em_setting_info_mode);

#else
static int __init parse_tag_em_setting(const struct tag *tag)
{
	em_setting_info *info = (em_setting_info *)(&tag->u);

	daudio_em_info.id = info->id;
	daudio_em_info.mode = info->mode;

	dbg("em_setting_info id: 0x%llx, mode: 0x%x\n",
			daudio_em_info.id, daudio_em_info.mode);

	return 0;
}
__tagtable(ATAG_DAUDIO_EM_SETTING, parse_tag_em_setting);
#endif

static ssize_t daudio_eng_read(struct file *file, char *buf, size_t count, loff_t *offset)
{
	volatile PGPION pGPIO_B = (PGPION)tcc_p2v(HwGPIOB_BASE);
	em_setting_info em_info;
	int ret;

	__func;
#if defined(CONFIG_SND_SOC_FM1288)
	mutex_lock(&daudio_eng_lock);
	dbg("uart port output enable b19 : 0x%X\n", pGPIO_B->GPEN.bREG.GP19);
	dbg("uart port output enable b20 : 0x%X\n", pGPIO_B->GPEN.bREG.GP20);
	dbg("uart port func b19 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN19);
	dbg("uart port func b20 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN20);
	mutex_unlock(&daudio_eng_lock);
#endif
	mutex_lock(&daudio_eng_setting_lock);
	ret = read_em_setting(&em_info);
	if (ret) {
		dbg("em_setting_info id: 0x%llx, mode: 0x%x\n", em_info.id, em_info.mode);
	}
	mutex_unlock(&daudio_eng_setting_lock);
	return 0;
}

static ssize_t daudio_eng_write(struct file *file, const char *buf, size_t count, loff_t *offset)
{
	char value = 0;
	em_setting_info em_info;
	__func;

	mutex_lock(&daudio_eng_setting_lock);
	em_info.mode = -1;

	//for debug
	if (count) {
		value = buf[0];

		if (value == 0x30) {		//'0' disable
			em_info.mode = 0;
		} else if (value == 0x31) {	//'1' enable
			em_info.mode = 1;
		}

		if (em_info.mode >= 0)
			write_em_setting(&em_info);
	}
	mutex_unlock(&daudio_eng_setting_lock);

	return count;
}

static long daudio_eng_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, size = 0, err = 0;
	int v = 0;
	int __user *p = (int __user *)arg;
	em_setting_info em_info;
	
	volatile PGPION pGPIO_B = (PGPION)tcc_p2v(HwGPIOB_BASE);

	__func;

	if (_IOC_TYPE(cmd) != ENGMODE_IOC_MAGIC) return -EINVAL;

	if (_IOC_NR(cmd) >= ENGMODE_IOC_MAX ) return -EINVAL;

	size = _IOC_SIZE(cmd);

	if(size){
		if(_IOC_DIR(cmd) & _IOC_READ)
			err = !access_ok(VERIFY_WRITE, (void*)arg, size);
		else if(_IOC_DIR(cmd) & _IOC_WRITE)
			err = !access_ok(VERIFY_READ, (void*)arg, size);
		
		if(err)  return err;
	}

	switch(cmd){
#if defined(CONFIG_SND_SOC_FM1288)
		case ENG_IOC_FM1288_UART_SET:
			dbg("ENG_IOC_FM1288_UART_SET\n");
			get_user(v, p);
			
			mutex_lock(&daudio_eng_lock);
			if(v == FUNCTION){
				tcc_gpio_config(TCC_GPB(19), GPIO_FN10);
				tcc_gpio_config(TCC_GPB(20), GPIO_FN10);
			}
			else if(v == INPUT){
				tcc_gpio_config(TCC_GPB(19), GPIO_FN0|GPIO_INPUT);
				tcc_gpio_config(TCC_GPB(20), GPIO_FN0|GPIO_INPUT);
			}
			else ret =  -EINVAL;
			mutex_unlock(&daudio_eng_lock);

			dbg("uart port b19 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN19);
			dbg("uart port b20 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN20);
			break;
		case ENG_IOC_FM1288_UART_GET:
			dbg("ENG_IOC_FM1288_UART_GET\n");
			
			mutex_lock(&daudio_eng_lock);
			if(pGPIO_B->GPFN2.bREG.GPFN19 && pGPIO_B->GPFN2.bREG.GPFN20) v = FUNCTION;
			else if(pGPIO_B->GPFN2.bREG.GPFN19 == 0x00 && pGPIO_B->GPFN2.bREG.GPFN20 == 0x00){
				if(pGPIO_B->GPEN.bREG.GP19 == 0x00 && pGPIO_B->GPEN.bREG.GP20 == 0x00){
					v = INPUT;
				}else v = INVALID;
			}
			else v = INVALID;
			mutex_unlock(&daudio_eng_lock);

			put_user(v, p);
			dbg("uart port output enable b19 : 0x%X\n", pGPIO_B->GPEN.bREG.GP19);
			dbg("uart port output enable b20 : 0x%X\n", pGPIO_B->GPEN.bREG.GP20);
			dbg("uart port func b19 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN19);
			dbg("uart port func b20 : 0x%X\n", pGPIO_B->GPFN2.bREG.GPFN20);
			break;
#endif			

		case ENG_IOC_SETTING_SET:
			dbg("ENG_IOC_SETTING_SET\n");

			mutex_lock(&daudio_eng_setting_lock);
			copy_from_user(&em_info, p, sizeof(em_setting_info));
			ret = write_em_setting(&em_info);
			copy_to_user(p, &em_info, sizeof(em_setting_info));
			if (!ret) {
				ret = -EINVAL;
			}
			mutex_unlock(&daudio_eng_setting_lock);
			break;

		case ENG_IOC_SETTING_GET:
			dbg("ENG_IOC_SETTING_GET\n");

			mutex_lock(&daudio_eng_setting_lock);
			copy_from_user(&em_info, p, sizeof(em_setting_info));
			ret = read_em_setting(&em_info);
			copy_to_user(p, &em_info, sizeof(em_setting_info));
			if (!ret) {
				ret = -EINVAL;
			}
			mutex_unlock(&daudio_eng_setting_lock);
			break;

		default:
			printk(KERN_ERR "Invalid command : %d\n", cmd);
			ret = -EINVAL;
	}
	return ret;
}

static int daudio_eng_open(struct inode *inode, struct file *file)
{
	__func;
	return 0;
}

static int daudio_eng_release(struct inode *inode, struct file *file)
{
	__func;
	return 0;
}

static struct file_operations daudio_eng_fops =
{
	.owner          = THIS_MODULE,
	.unlocked_ioctl = daudio_eng_ioctl,
	.open           = daudio_eng_open,
	.release        = daudio_eng_release,
	.read			= daudio_eng_read,
	.write			= daudio_eng_write,
};

static void eng_early_suspend(struct early_suspend *h)
{
	__func;
}

static void eng_late_resume(struct early_suspend *h)
{
	__func;
}

static __init int daudio_eng_init(void)
{
	int ret = register_chrdev(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_DEV, &daudio_eng_fops);

	dbg("%s %d\n", __func__, ret);

	if (ret < 0)
	{
		printk(KERN_ERR "%s failed to register_chrdev\n", __func__);
		ret = -EIO;
		goto out;
	}

	daudio_eng_class = class_create(THIS_MODULE, ENGMODE_CTRL_DEV);
	if (IS_ERR(daudio_eng_class))
	{
		ret = PTR_ERR(daudio_eng_class);
		goto out_chrdev;
	}

	daudio_eng_device = device_create(daudio_eng_class, NULL, MKDEV(DAUDIO_ENG_MAJOR, 0), NULL, ENGMODE_CTRL_DEV);
	if (IS_ERR(daudio_eng_device))
	{
		ret = PTR_ERR(daudio_eng_device);
		goto out_class;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	daudio_eng_early_suspend.suspend = eng_early_suspend;
	daudio_eng_early_suspend.resume = eng_late_resume;
	daudio_eng_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&daudio_eng_early_suspend);
#endif

	goto out;

out_class:
	class_destroy(daudio_eng_class);
out_chrdev:
	unregister_chrdev(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_DEV);
out:
	return ret;
}

static __exit void daudio_eng_exit(void)
{
	device_destroy(daudio_eng_class, MKDEV(DAUDIO_ENG_MAJOR, 0));
	class_destroy(daudio_eng_class);
	unregister_chrdev(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_DEV);
}

module_init(daudio_eng_init);
module_exit(daudio_eng_exit);

MODULE_AUTHOR("Cleinsoft");
MODULE_DESCRIPTION("D-Audio Engineering Mode Driver");
MODULE_LICENSE("GPL");
