#include "../melfas_mip4/mip4_ts.h"
#include "../siw/touch_sw17700.h"
#include"../atmel/atmel_mxt336S.h"

#include <mach/vioc_lut.h>
#include <mach/vioc_blk.h>
#include <mach/tcc_lut_ioctl.h>

#ifdef CONFIG_DAUDIO_KK
#include <mach/mobis_nvdata_handler.h>
#elif defined(CONFIG_WIDE_PE_COMMON)
#include <mobis/mobis_nvdata_handler.h>
#endif

#undef VERIFY_OCTAL_PERMISSIONS
#define VERIFY_OCTAL_PERMISSIONS(perms) (perms)

#define DRIVER_NAME     "serdes_recovery"
#define SERDES_DEVICE_NAME "serdes_recovery"

#define SERDES_RESET_TIME 4000

EXPORT_SYMBOL(mobis_touch_update_check);
EXPORT_SYMBOL(mobis_touch_update_complete);

struct serdes_info {
        struct i2c_client *client;
struct {
                struct delayed_work reset_dwork;
                struct mutex lock;
} serdes;
};

struct serdes_info *g_info = NULL;

int red_value = 255;
int green_value = 255;
int blue_value = 255;

int isLVDS(void)
{
	if(get_oemtype())
	{
		if(get_montype())
		{
			switch(daudio_lcd_version())
			{
				case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE :
					return 1;
					break;
				default :
					return 0;
					break;
			}
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 1;
	}

	return 0;
}

int mobis_touch_update_check(void)
{
        char buffer[2] = {0x00, 0x00};
        mobis_misc_nvdata_read(MOBIS_NVDATA_MISC_TOUCH_UPDATE, buffer, 1);
        if(buffer[0] == 0x1)
        {
                printk("Touch Firmware Update Set\n",__func__);
                // TODO touch firmware update and clear misc data
                return 1;
        }
        else
        {
                printk("Touch Firmware nothing todo....\n",__func__);
                return 0;
        }
}
void mobis_touch_update_complete(void)
{
        char buffer[2] = {0x00, 0x00};
        memset(buffer, 0x00, sizeof(buffer));
        mobis_misc_nvdata_write(MOBIS_NVDATA_MISC_TOUCH_UPDATE, buffer, 1);

        printk("Touch Firmware Update Complete\n",__func__);
}

inline void serdes_reset_dwork_stop(struct serdes_info *info)
{
        cancel_delayed_work(&info->serdes.reset_dwork);
}

inline void serdes_reset_dwork_start(struct serdes_info *info)
{
        serdes_reset_dwork_stop(info);
        schedule_delayed_work(&info->serdes.reset_dwork,msecs_to_jiffies(SERDES_RESET_TIME));
}

static void serdes_reset_dwork(struct work_struct *work)
{
	int lcd_ver = 0;
	struct serdes_info *info = container_of(work, struct serdes_info, serdes.reset_dwork);

	lcd_ver = daudio_lcd_version();

	printk("%s, GPIO_B24 = %d, GPIO_B13 = %d, GPIO_B8 = %d,  daudio_lcd_version() = %d \n", __func__, get_oemtype(), get_montype(), get_factorycheck(), lcd_ver);
#ifdef CONFIG_TOUCHSCREEN_ONEBIN
	if(get_oemtype())
	{
		if(get_montype())
                {
			switch(lcd_ver)
			{
				case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_LG:
	                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_2_LG:
				case DAUDIOKK_LCD_OI_DISCONNECTED:
					printk("%s mip4_init()\n",__func__);
					mip4_init();
                                        break;
				case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS_LG:
					printk("%s sw17700_init()\n",__func__);
					sw17700_init();
					break;
				case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE:
					printk("%s mxt336s_init()\n",__func__);
                                        mxt336s_init();
				default:
					break;
			}
                }
                else
                {
			switch(lcd_ver)
                        {
				case DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si_LG:
					printk("%s mip4_init()\n",__func__);
                                        mip4_init();
                                        break;
				case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS_LG:
					if(!get_factorycheck())
					{
						printk("%s mip4_init()\n",__func__);
						mip4_init();
						break;
					}
					printk("%s sw17700_init()\n",__func__);
                                        sw17700_init();
					break;
				case DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si_BOE:
					printk("%s mxt336s_init()\n",__func__);
                                        mxt336s_init();
				default:
					break;
                        }
                }
	}
	else
	{
		printk("%s mxt336s_init()\n",__func__);
		mxt336s_init();
	}
#endif
}

static ssize_t set_rmmod(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int state;
        struct i2c_client *client = to_i2c_client(dev);
        struct serdes_info *info = i2c_get_clientdata(client);

        sscanf(buf, "%d", &state);
#ifdef CONFIG_TOUCHSCREEN_ONEBIN
        if(state == 1)
        {
                mip4_cleanup();
                printk("mip4_cleanup [Done]\n");
        }
        else if(state == 2)
        {
                sw17700_cleanup();
                printk("sw17700_cleanup [Done]\n");
        }
#endif
        return count;
}

static ssize_t set_insmod(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int state;
        struct i2c_client *client = to_i2c_client(dev);
        struct serdes_info *info = i2c_get_clientdata(client);

        sscanf(buf, "%d", &state);
#ifdef CONFIG_TOUCHSCREEN_ONEBIN
        if(state == 1)
        {
                mip4_init();
                printk("mip4_init [Done]\n");
        }
        else if(state == 2)
        {
                sw17700_init();
                printk("sw17700_init [Done]\n");
        }
#endif
        return count;
}

static ssize_t _set_bluelight(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int i = 0;

	int lut_dev_select = 0;

        unsigned int BGR_Gamma[256];

        sscanf(buf, "%d %d %d", &red_value, &green_value, &blue_value);

        printk("%s Start %d %d %d\n",__func__,red_value, green_value, blue_value);

	if(isLVDS())
		lut_dev_select = LUT_DEV1;
	else
		lut_dev_select = LUT_DEV0;

	tcc_set_lut_enable(TVC_LUT(lut_dev_select), true);

        for(i = 0; i < 256; i++)
        {
                BGR_Gamma[i] = ((int)(blue_value*i/255) << 16) | ((int)(green_value*i/255) << 8) | (int)(red_value*i/255);
        }

        tcc_set_lut_table(TVC_LUT(lut_dev_select), BGR_Gamma);

        return count;
}

static ssize_t _show_bluelight(struct device *dev, struct device_attribute *attr, const char *buf)
{
	printk("%s Start %d %d %d\n",__func__,red_value, green_value, blue_value);
	snprintf(buf, 255, "%d %d %d\n", red_value, green_value, blue_value);
}

static ssize_t _show_fw_update(struct device *dev, struct device_attribute *attr, const char *buf)
{
        char buffer[2] = {0x00, 0x00};

        printk("%s [Start]\n",__func__);

        mobis_misc_nvdata_read(MOBIS_NVDATA_MISC_TOUCH_UPDATE, buffer, 1);

        if(buffer[0] == 0x01)
                printk("Fw Update State : Set\n");
        else if(buffer[0] == 0x00)
                printk("Fw Update State : Cancel\n");
        else
                printk("Fw Update State : Wrong\n");
}
static ssize_t _set_fw_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int state;
        char buffer[2] = {0x00, 0x00};

        printk("%s [Start]\n",__func__);

        sscanf(buf, "%d", &state);

        if(state == 1)
        {
                printk("Set Fw Update Next boot\n");
                memset(buffer, 0x01, sizeof(buffer));
                mobis_misc_nvdata_write(MOBIS_NVDATA_MISC_TOUCH_UPDATE, buffer, 1);
        }
        else if(state == 0)
        {
                printk("Cancel Fw Update\n");
                memset(buffer, 0x00, sizeof(buffer));
                mobis_misc_nvdata_write(MOBIS_NVDATA_MISC_TOUCH_UPDATE, buffer, 1);
        }
        else
        {
                printk("Input wrong value : %d\n",state);
        }
        return count;
}

static DEVICE_ATTR(rmmod, S_IRUGO | S_IWUGO, NULL, set_rmmod);
static DEVICE_ATTR(insmod, S_IRUGO | S_IWUGO, NULL, set_insmod);
static DEVICE_ATTR(bluelight, S_IRUGO | S_IWUGO , _show_bluelight, _set_bluelight);
static DEVICE_ATTR(fw_update, S_IRUGO | S_IWUGO ,_show_fw_update, _set_fw_update);

static struct attribute *serdes_attrs[] = {
	&dev_attr_rmmod.attr,
        &dev_attr_insmod.attr,
	&dev_attr_bluelight.attr,
	&dev_attr_fw_update.attr,
	NULL,
};

static const struct attribute_group serdes_attr_group = {
        .attrs = serdes_attrs,
};

static int serdes_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct serdes_info *info;
	int ret;

	printk("%s [Start]\n",__func__);

	info = devm_kzalloc(&client->dev, sizeof(struct serdes_info), GFP_KERNEL);

        g_info = info;

        info->client = client;

	i2c_set_clientdata(client, info);

	if(sysfs_create_group(&client->dev.kobj, &serdes_attr_group)) {
		printk("%s [ERROR] sysfs_create_group\n", __func__);
	}

	if (sysfs_create_link(NULL, &client->dev.kobj, "serdes_drv")) {
		printk("%s [ERROR] sysfs_create_link\n", __func__);
	}

	INIT_DELAYED_WORK(&info->serdes.reset_dwork, serdes_reset_dwork);
        serdes_reset_dwork_start(info);

        return 0;
}

static int serdes_remove(struct i2c_client *client)
{
	printk("%s [Start]\n",__func__);
	return 0;
}

static const struct of_device_id serdes_of_match_table[] = {
        { .compatible = "serdes,serdes_recovery", },
        { },
};
MODULE_DEVICE_TABLE(of, serdes_of_match_table);

static const struct i2c_device_id serdes_id[] = {
        {SERDES_DEVICE_NAME, 0},
        {},
};
MODULE_DEVICE_TABLE(i2c, serdes_id);

static struct i2c_driver serdes_driver = {
	.id_table = serdes_id,
	.probe = serdes_probe,
	.remove = serdes_remove,
	.driver = {
		.name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
		.of_match_table = serdes_of_match_table,
	},
};

static int __init serdes_init(void)
{
	int err;
	printk("%s Start \n", __func__);

	err = i2c_add_driver(&serdes_driver);
	if(err)
		printk("serdes driver i2c driver add failed (errno = %d)\n", err);
	return err;
}
module_init(serdes_init);

static void __exit serdes_cleanup(void)
{
}
module_exit(serdes_cleanup);

