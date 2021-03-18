/*
 * MELFAS MIP4 Touchscreen
 *
 * Copyright (C) 2000-2017 MELFAS Inc.
 *
 *
 * mip4_ts.c
 *
 * Version : 2017.05.23
 */

#include "mip4_ts.h"
#include <mach/daudio_info.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <mach/daudio.h>
/*
 * Start-up run command
 */

void mip4_ts_restart(struct mip4_ts_info *info);
int mip4_ts_startup_run(struct mip4_ts_info *info);
int mip4_ts_config(struct mip4_ts_info *info);
static irqreturn_t mip4_ts_interrupt(int irq, void *dev_id);
int mip4_ts_i2c_read(struct mip4_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);

static struct mip4_ts_info g_info;

static bool LCD_TOUCH_RESET_NEED = 0;
static bool LCD_DES_RESET_NEED = 0;
static int LCD_TOUCH_INT_CHECK = 0;
static int LCD_VER = 0;
/* 2018.3.19
LCD_VER 0 : INTEGRATED 10.25 1920X720 MELFAS LGD In-cell
	3 : DEPARTED 10.25 1920X720 MELFAS LGD In-cell
	4 : DEPARTED 12.3 1920X720 MELFAS LGD In-cell
	10 : INTEGRATED No Monitor 
*/

/* for debugging, enable DEBUG_TRACE */

int debug;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");


#define SER_SIZE 71
#define SER_RETRY_SIZE 19
#define SER_RETRY_SIZE_12_3 6
#define MIP4_SERDES_RESET_TIME 3000
#define DES_ADDRESS 0x48
#define SER_ADDRESS 0x40

struct mip4_ts_info *info_mip4 ;




static u16 serdes_config [SER_SIZE][3] = {
	{0x6a, 0x0010, 0x11}, /* Reset */
	{0x40, 0x0010, 0x11},
	{0x40, 0x20F5, 0x00},
	{0x40, 0x0001, 0x88},
	{0x6a, 0x01CE, 0xDF},

	{0x40, 0x0140, 0x20},
	{0x40, 0x0002, 0x03},
	{0x6a, 0x0140, 0x20},
	{0x6a, 0x0002, 0x03},
	{0x40, 0x020C, 0x04},

	{0x40, 0x020D, 0x44},
	{0x40, 0x020E, 0x04},
	{0x6a, 0x0212, 0x83},
	{0x6a, 0x0213, 0x64},
	{0x6a, 0x0214, 0x44},

	{0x6a, 0x0215, 0x84},
	{0x6a, 0x0216, 0xA5},
	{0x6a, 0x0217, 0x05},
	{0x40, 0x020F, 0x83},
	{0x40, 0x0210, 0xA5},

	{0x40, 0x0211, 0x45},
	{0x6a, 0x020C, 0x84},
	{0x6a, 0x020D, 0xA2},
	{0x6a, 0x020E, 0x02},
	{0x40, 0x0206, 0x83},

	{0x40, 0x0207, 0xA2},
	{0x40, 0x0208, 0x42},
	{0x6a, 0x020F, 0x84},
	{0x6a, 0x0210, 0xA3},
	{0x6a, 0x0211, 0x03},

	{0x40, 0x0209, 0x83},
	{0x40, 0x020A, 0xA3},
	{0x40, 0x020B, 0x43},
	{0x6a, 0x0209, 0x8C},
	{0x6a, 0x020A, 0xA1},

	{0x6a, 0x020B, 0x41},
	{0x40, 0x0203, 0x8B},
	{0x40, 0x0204, 0xA1},
	{0x40, 0x0205, 0x41},
	{0x6a, 0x0200, 0x81},

	{0x6a, 0x0206, 0x8C},
	{0x6a, 0x0207, 0xA0},
	{0x6a, 0x0208, 0x40},
	{0x40, 0x0200, 0x0B},
	{0x40, 0x0201, 0xA0},

	{0x40, 0x0202, 0x00},
	{0x6a, 0x021B, 0x8C},
	{0x6a, 0x021C, 0xA6},
	{0x6a, 0x021D, 0x46},
	{0x40, 0x0212, 0x8B},

	{0x40, 0x0213, 0xA6},
	{0x40, 0x0214, 0x06},
	{0x40, 0x0215, 0x84},
	{0x40, 0x0216, 0xA7},
	{0x40, 0x0217, 0x07},

	{0x6a, 0x021E, 0x83},
	{0x6a, 0x021F, 0xA7},
	{0x6a, 0x0220, 0x47},
	{0x40, 0x0218, 0x8C},
	{0x40, 0x0219, 0xA8},

	{0x40, 0x021A, 0x48},
	{0x6a, 0x0218, 0x8B},
	{0x6a, 0x0219, 0xA8},
	{0x6a, 0x021A, 0x48},
	{0x40, 0x021B, 0x83},

	{0x40, 0x021C, 0xA9},
	{0x40, 0x021D, 0x49},
	{0x6a, 0x0221, 0x84},
	{0x6a, 0x0222, 0x29},
	{0x6a, 0x0223, 0x09},

	{0x40, 0x20F5, 0x01},
};

static u16 serdes_re_try[SER_RETRY_SIZE][3] = {
	{DES_ADDRESS, 0x01CE, 0xDF},
	{DES_ADDRESS, 0x0140, 0x20},
	{DES_ADDRESS, 0x0002, 0x43},

	{DES_ADDRESS, 0x0219, 0x64},
	{DES_ADDRESS, 0x021A, 0x44},
	{DES_ADDRESS, 0x0213, 0xA6},
	{DES_ADDRESS, 0x0214, 0x06},

	{DES_ADDRESS, 0x0222, 0xA3},
	{DES_ADDRESS, 0x0223, 0x03},
	{DES_ADDRESS, 0x0216, 0xA7},
	{DES_ADDRESS, 0x0217, 0x07},
	
	{DES_ADDRESS, 0x0218, 0x83},
	{DES_ADDRESS, 0x021E, 0x90},
	{DES_ADDRESS, 0x0212, 0x84},
	{DES_ADDRESS, 0x0221, 0x84},

	{DES_ADDRESS, 0x0215, 0x8C},
	{DES_ADDRESS, 0x0206, 0x80},
	{DES_ADDRESS, 0x0227, 0x80},
	
	{DES_ADDRESS, 0x0010, 0x31},
};
static u16 serdes_link [1][2] = {
	{SER_ADDRESS, 0x0013},
};

static u16 serdes_re_try_12_3[SER_RETRY_SIZE_12_3][3] = {
        {DES_ADDRESS, 0x14, 0x05},
        {DES_ADDRESS, 0x02, 0x0F},
        {DES_ADDRESS, 0x15, 0x70},
        {DES_ADDRESS, 0x0F, 0x01},
        {DES_ADDRESS, 0x06, 0x00},

        {DES_ADDRESS, 0x06, 0x08},
};


static u16 serdes_link_12_3[1][2] = {
	{DES_ADDRESS, 0x01},
};



static int serdes_i2c_init(struct i2c_client *client)
{
	int ret;
	int i;
	u8 buf[3];
	int retry;
	unsigned short addr;

	addr = client->addr;

	printk("-----------%s\n",__func__);

	for(i=0; i<SER_SIZE; i++)
	{
		retry = 0;
		client->addr = serdes_config[i][0];
		buf[0] = (serdes_config[i][1] >> 8) & 0xff;
		buf[1] = serdes_config[i][1] & 0xff;
		buf[2] = serdes_config[i][2];
retry_write:
		ret =  i2c_master_send(client, buf, 3);
		if(ret!=3) {
			if(retry<5){
				dev_err(&client->dev, "%s: i2c retry\n", __func__);
				msleep(25);
				retry++;
				goto retry_write;
			} else {
				dev_err(&client->dev, "%s: i2c send failed (%d), {0x%x, 0x%x %x, 0x%x}\n", __func__, ret, client->addr, buf[0], buf[1], buf[2]);
				ret = -EIO;
				return ret;
			}
		} else {
			ret = 0;
		}
	}
	printk("-----------%s Success\n",__func__);
	client->addr = addr;
	return 0;
}

static int serdes_i2c_retry(struct i2c_client *client)
{
	int ret;
	int i;
	u8 buf[3];
	int retry;
	unsigned short addr;

	addr = client->addr;

	printk("-----------%s\n",__func__);

	for(i=0; i<((LCD_VER ==4) ? SER_RETRY_SIZE_12_3 : SER_RETRY_SIZE); i++)
	{
		retry = 0;
		if(LCD_VER == 4)
		{	
			client->addr = serdes_re_try_12_3[i][0];
			buf[0] = serdes_re_try_12_3[i][1];
			buf[1] = serdes_re_try_12_3[i][2];

			ret = i2c_master_send(client, buf, 2);	
		}
		else
		{
			client->addr = serdes_re_try[i][0];
			buf[0] = (serdes_re_try[i][1] >> 8) & 0xff;
			buf[1] = serdes_re_try[i][1] & 0xff;
			buf[2] = serdes_re_try[i][2];

			ret =  i2c_master_send(client, buf, 3);
		}
			
		if(ret!=((LCD_VER==4) ? 2 : 3)) {
			dev_err(&client->dev, "%s: i2c fail\n", __func__);
			goto out_i2c;
		}
	}
	printk("-----------%s Success\n",__func__);
out_i2c:	
	client->addr = addr;
	return 0;
}


static int serdes_i2c_read(struct i2c_client *client)
{
	int ret;
	int i;
	u8 buf[2];
	u8 buf2[2];
	int retry;
	unsigned short addr;
	
	addr = client->addr;


	if(LCD_VER == 4)
	{
		client->addr = serdes_link_12_3[0][0];
		buf[1] = serdes_link_12_3[0][1];
		
		ret = i2c_master_send(client, buf, 1);	

		if(ret != 1)
		{
			dev_info(&client->dev, "%s LCD_DES_RESET_NEED Set 1\n",__func__);
			LCD_DES_RESET_NEED =1;
		}
	}
	else
	{
		client->addr = serdes_link[0][0];
		buf[0] = (serdes_link[0][1] >> 8) & 0xff;
		buf[1] = serdes_link[0][1] & 0xff;
		//buf[2] = (u8 *) value;
		ret =  i2c_master_send(client, buf, 2);
		ret = i2c_master_recv(client, buf2, 3);
	

#if 1
		if(buf2[0] != 0xde)
		{
			printk(KERN_ERR "---------%s: i2c read  (%d), {0x%x, 0x%x ,0x%x,0x%x}\n", __func__, ret, client->addr, buf2[0], buf2[1],buf2[2]);
			LCD_DES_RESET_NEED = 1;
		}
#endif	
	}

	client->addr = addr;
	return ((LCD_VER==4) ? ret : buf2[0]);
}


inline void mip4_serdes_reset_dwork_stop(struct mip4_ts_info *info)
{
	cancel_delayed_work(&info->serdes.reset_dwork);
}

inline void mip4_serdes_reset_dwork_start(struct mip4_ts_info *info)
{
	mip4_serdes_reset_dwork_stop(info);
	schedule_delayed_work(&info->serdes.reset_dwork,
			msecs_to_jiffies(MIP4_SERDES_RESET_TIME));
}



static void mip4_serdes_reset(struct mip4_ts_info *info,struct mip4_ts_info *info_mip4)
{
	u8 val;
	int ret = 0,i = 0;

	mutex_lock(&info->serdes.lock);
	
	val = serdes_i2c_read(info->client);

	
	if((val == ((LCD_VER==4) ? 1 : 0xde)) && LCD_DES_RESET_NEED == 1)
	{
		printk(KERN_ERR "---------%s: DISPLAY & TouchIC Recovery !!! \n", __func__);
		printk(KERN_ERR "%s mip4_read_byte :0x[%x]!!!\r\n", __func__, val);
		printk(KERN_ERR "%s LCD_DES_RESET_NEED value :(%d) !!!\r\n", __func__, LCD_DES_RESET_NEED);

		serdes_i2c_retry(info->client);

		if(info_mip4->irq_enabled) {
			disable_irq(info_mip4->client->irq);
			info_mip4->irq_enabled = false;
		}		
		
		mip4_ts_restart(info_mip4);

		ret = mip4_ts_config(info_mip4);
		
		if (ret) {
			LCD_DES_RESET_NEED = 1;
			printk(KERN_ERR "%s [ERROR] mip4_ts_config\n", __func__);
		}
		else
			{
				printk(KERN_ERR "%s write LCD_DES_RESET 0x[%x]\r\n", __func__, val);
				LCD_DES_RESET_NEED = 0;
			}
			
		mip4_ts_config_input(info_mip4); 

		if(!info_mip4->irq_enabled) {
			enable_irq(info_mip4->client->irq);
			info_mip4->irq_enabled = true;
		}
			
	}
	else
	{
		if(val != ((LCD_VER==4) ? 1 : 0xde))
		{
			printk(KERN_ERR "---------%s: HDMI Connect ERROR !!! \n", __func__);
		}
		
	}
	for(i=0; i<3; i++) //serdes conn check 20180622 mhjung
	{
		val = serdes_i2c_read(info->client);

		if(val != ((LCD_VER==4) ? 1 : 0xde))
		{
			if (i == 2)  
			{
				set_serdes_conn_check(2);
			}	
		
			msleep(30);
		}else{
				set_serdes_conn_check(0);
				break;
		}
	
	}
	// mxt->client->addr=addr;
	mutex_unlock(&info->serdes.lock);
}

static void mip4_touch_reset(struct mip4_ts_info *info)
{
	int ret = 0;
	int retry = I2C_RETRY_COUNT;

	if(LCD_TOUCH_INT_CHECK >= 2)
	{
		LCD_TOUCH_INT_CHECK = 0;
	}
	else
	{
		LCD_TOUCH_RESET_NEED = 1;
	}
	
	if(LCD_TOUCH_RESET_NEED == 1)
	{
		dev_info(&info->client->dev, "%s LCD_TOUCH_RESET [START]",__func__);

		if(info->irq_enabled) {
			disable_irq(info->client->irq);
			info->irq_enabled = false;
		}		
		
		mip4_ts_restart(info);

		while(retry--)
		{
			ret = mip4_ts_config(info);

			if(ret == 0)
				break;
			msleep(200);
			printk("msleep(200)\n");
		}

		if(ret !=0)
		{
			dev_err(&info->client->dev, "%s [ERROR]\n", __func__);	
			return;
		}
		
		mip4_ts_config_input(info); 

		if(!info->irq_enabled) {
			enable_irq(info->client->irq);
			info->irq_enabled = true;
		}
		LCD_TOUCH_RESET_NEED = 0;
		LCD_TOUCH_INT_CHECK = 0;
		
		dev_info(&info->client->dev, "%s LCD_TOUCH_RESET [DONE]",__func__);
	}

}

static void mip4_serdes_reset_dwork(struct work_struct *work)
{
	struct input_dev *dev;
	struct mip4_ts_info *info_serdes = container_of(work, struct mip4_ts_info, serdes.reset_dwork);
	
	LCD_VER = daudio_lcd_version();

	if(LCD_VER == 4 || LCD_VER == 3)// 3 : 10.25 departed 4 : 12.3 departed 
	{	
		mip4_serdes_reset(info_serdes,info_mip4);
		
		if(LCD_DES_RESET_NEED != 1)
		{
			mip4_touch_reset(info_mip4);
		}
	}
	else
	{
		mip4_touch_reset(info_mip4);
	}

	mip4_serdes_reset_dwork_start(info_serdes);
}

int mip4_ts_startup_run(struct mip4_ts_info *info)
{
	u8 wbuf[4];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_RUN;
	wbuf[2] = 1;
	if (mip4_ts_i2c_write(info, wbuf, 3)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Start-up process
 */
int mip4_ts_startup(struct mip4_ts_info *info)
{
	int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

#if 0
	reinit_completion(&info->startup_done);
	dev_dbg(&info->client->dev, "%s - startup : reinit_completion\n", __func__);
#else
	init_completion(&info->startup_done);
	dev_dbg(&info->client->dev, "%s - startup : init_completion\n", __func__);
#endif

	mip4_ts_power_on(info);
	msleep(30);

	if (!info->irq_enabled) {
		enable_irq(info->client->irq);
		info->irq_enabled = true;
	}

	/* Wait start-up interrupt */
	dev_dbg(&info->client->dev, "%s - startup : wait_for_completion_interruptible_timeout[%d]\n", __func__, STARTUP_TIMEOUT);
	ret = wait_for_completion_interruptible_timeout(&info->startup_done, msecs_to_jiffies(STARTUP_TIMEOUT));
	if (ret > 0) {
		dev_dbg(&info->client->dev, "%s - startup : completed[%d]\n", __func__, jiffies_to_msecs(ret));
	} else {
		dev_dbg(&info->client->dev, "%s - startup : timeout[%d]\n", __func__, ret);

#if USE_FW_RECOVERY
		/* Restore firmware */
		dev_dbg(&info->client->dev, "%s - flash_fail_type : critical\n", __func__);
		ret = mip4_ts_fw_restore_critical_section(info);
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_fw_restore_critical_section\n", __func__);
		}
#endif /* USE_FW_RECOVERY */
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
 * Reset chip
 */
void mip4_ts_reset(struct mip4_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mip4_ts_power_off(info);

	mip4_ts_power_on(info);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/*
 * Restart chip
 */
void mip4_ts_restart(struct mip4_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mip4_ts_power_off(info);

#if USE_STARTUP_WAITING
	mip4_ts_startup(info);
#else
	mip4_ts_power_on(info);
#endif /* USE_STARTUP_WAITING */

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/*
 * I2C Read
 */
int mip4_ts_i2c_read(struct mip4_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	struct i2c_msg msg[] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

	while (retry--) {
		res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

		if (res == ARRAY_SIZE(msg)) {
			goto exit;
		} else if (res < 0) {
			dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
		} else {
			dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%zu] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
		}
	}

	goto error_reboot;

error_reboot:
#if RESET_ON_I2C_ERROR
	//mip4_ts_restart(info);
#endif /* RESET_ON_I2C_ERROR */
	return 1;

exit:
	return 0;
}

/*
 * I2C Write
 */
int mip4_ts_i2c_write(struct mip4_ts_info *info, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;
	printk("%s\n",__func__);

	while (retry--) {
		res = i2c_master_send(info->client, write_buf, write_len);

		if (res == write_len) {
			goto exit;
		} else if (res < 0) {
			dev_err(&info->client->dev, "%s [ERROR] i2c_master_send - errno [%d]\n", __func__, res);
		} else {
			dev_err(&info->client->dev, "%s [ERROR] i2c_master_send - write[%d] result[%d]\n", __func__, write_len, res);
		}
	}

	goto error_reboot;

error_reboot:
#if RESET_ON_I2C_ERROR
	//mip4_ts_restart(info);
#endif /* RESET_ON_I2C_ERROR */
	return 1;

exit:
	return 0;
}

/*
 * Enable device
 */
int mip4_ts_enable(struct mip4_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (info->enabled) {
		dev_err(&info->client->dev, "%s [ERROR] device already enabled\n", __func__);
		goto exit;
	}

	mutex_lock(&info->lock);

#if USE_WAKEUP_GESTURE
	mip4_ts_set_power_state(info, MIP4_CTRL_POWER_ACTIVE);

	if (wake_lock_active(&info->wake_lock)) {
		wake_unlock(&info->wake_lock);
		dev_dbg(&info->client->dev, "%s - wake_unlock\n", __func__);
	}

	info->gesture_wakeup_mode = false;
	dev_dbg(&info->client->dev, "%s - gesture wake-up mode : off\n", __func__);
#else
#if USE_STARTUP_WAITING
	mip4_ts_startup(info);
#else
	if(info->init){
		info->init = false;
	}else{
		mip4_ts_power_on(info);
	}
	

	if (!info->irq_enabled) {
		enable_irq(info->client->irq);
		info->irq_enabled = true;
	}
#endif /* USE_STARTUP_WAITING */
#endif /* USE_WAKEUP_GESTURE */

	info->enabled = true;
	info->i2c_error_cnt = 0;

	mutex_unlock(&info->lock);

exit:
	dev_info(&info->client->dev, MIP4_TS_DEVICE_NAME" - Enabled\n");

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
 * Disable device
 */
int mip4_ts_disable(struct mip4_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (!info->enabled) {
		dev_err(&info->client->dev, "%s [ERROR] device already disabled\n", __func__);
		goto exit;
	}

	mutex_lock(&info->lock);

#if USE_WAKEUP_GESTURE
	info->wakeup_gesture_code = 0;

	mip4_ts_set_wakeup_gesture_type(info, MIP4_EVENT_GESTURE_ALL);
	mip4_ts_set_power_state(info, MIP4_CTRL_POWER_LOW);

	info->gesture_wakeup_mode = true;
	dev_dbg(&info->client->dev, "%s - gesture wake-up mode : on\n", __func__);

	if (!wake_lock_active(&info->wake_lock)) {
		wake_lock(&info->wake_lock);
		dev_dbg(&info->client->dev, "%s - wake_lock\n", __func__);
	}
#else
	disable_irq(info->client->irq);
	info->irq_enabled = false;

	mip4_ts_power_off(info);
#endif /* USE_WAKEUP_GESTURE */

	mip4_ts_clear_input(info);

	info->enabled = false;

	mutex_unlock(&info->lock);

exit:
	dev_info(&info->client->dev, MIP4_TS_DEVICE_NAME" - Disabled\n");

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

#if USE_FB_NOTIFY
/*
 * FB notifier callback
 */
static int mip4_ts_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct mip4_ts_info *info = container_of(self, struct mip4_ts_info, fb_notifier);
	struct fb_event *event_data = data;
	unsigned int blank;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Check event */
	//if (event != FB_EARLY_EVENT_BLANK) {
	if (event != FB_EVENT_BLANK) {
		dev_dbg(&info->client->dev, "%s - skip event [0x%02X]\n", __func__, event);
		goto exit;
	}

	/* Check blank */
	blank = *(int *)event_data->data;

	dev_dbg(&info->client->dev, "%s - blank[0x%02X]\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
	    /* Enable */
		dev_dbg(&info->client->dev, "%s - FB_BLANK_UNBLANK\n", __func__);
		dev_info(&info->client->dev, "%s - Display : On\n", __func__);

		break;

	case FB_BLANK_POWERDOWN:
	    /* Disable */
		dev_dbg(&info->client->dev, "%s - FB_BLANK_POWERDOWN\n", __func__);
		dev_info(&info->client->dev, "%s - Display : Off\n", __func__);

		break;

	default:
		dev_dbg(&info->client->dev, "%s - skip blank [0x%02X]\n", __func__, blank);
		break;
	}

exit:
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}
#endif /* USE_FB_NOTIFY */

#if USE_INPUT_OPEN_CLOSE
/*
 * Open input device
 */
static int mip4_ts_input_open(struct input_dev *dev)
{
	struct mip4_ts_info *info = input_get_drvdata(dev);

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (info->init == true) {
		info->init = false;
	} else {
		mip4_ts_enable(info);
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
 * Close input device
 */
static void mip4_ts_input_close(struct input_dev *dev)
{
	struct mip4_ts_info *info = input_get_drvdata(dev);

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mip4_ts_disable(info);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}
#endif /* USE_INPUT_OPEN_CLOSE */

/*
 * Get ready status
 */
int mip4_ts_get_ready_status(struct mip4_ts_info *info)
{
	u8 wbuf[16];
	u8 rbuf[16];
	int ret = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_READY_STATUS;
	if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, 1)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read\n", __func__);
		goto error;
	}
	ret = rbuf[0];

	/* Check status */
	if ((ret == MIP4_CTRL_STATUS_NONE) || (ret == MIP4_CTRL_STATUS_LOG) || (ret == MIP4_CTRL_STATUS_READY)) {
		//dev_dbg(&info->client->dev, "%s - status[0x%02X]\n", __func__, ret);
	} else {
		dev_err(&info->client->dev, "%s [ERROR] Unknown status[0x%02X]\n", __func__, ret);
		goto error;
	}

	if (ret == MIP4_CTRL_STATUS_LOG) {
		/* Skip log event */
		wbuf[0] = MIP4_R0_LOG;
		wbuf[1] = MIP4_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if (mip4_ts_i2c_write(info, wbuf, 3)) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
		}
	}

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return ret;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Read chip firmware version
 */
int mip4_ts_get_fw_version(struct mip4_ts_info *info, u8 *ver_buf)
{
	u8 rbuf[8];
	u8 wbuf[2];
	int i;

	wbuf[0] = MIP4_R0_INFO;
	wbuf[1] = MIP4_R1_INFO_VERSION_BOOT;
	if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, 8)) {
		goto error;
	};

	for (i = 0; i < FW_MAX_SECT_NUM; i++) {
		ver_buf[0 + i * 2] = rbuf[1 + i * 2];
		ver_buf[1 + i * 2] = rbuf[0 + i * 2];
	}

	return 0;

error:
	for (i = 0; i < (FW_MAX_SECT_NUM * 2); i++) {
		ver_buf[i] = 0xFF;
	}

	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Read chip firmware version for u16
 */
int mip4_ts_get_fw_version_u16(struct mip4_ts_info *info, u16 *ver_buf_u16)
{
	u8 rbuf[8];
	int i;

	if (mip4_ts_get_fw_version(info, rbuf)) {
		goto error;
	}

	for (i = 0; i < FW_MAX_SECT_NUM; i++) {
		ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
	}

	return 0;

error:
	for (i = 0; i < FW_MAX_SECT_NUM; i++) {
		ver_buf_u16[i] = 0xFFFF;
	}

	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

#if (CHIP_MODEL != CHIP_NONE)
/*
 * Read bin(file) firmware version
 */
int mip4_ts_get_fw_version_from_bin(struct mip4_ts_info *info, u8 *ver_buf)
{
	const struct firmware *fw;
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	request_firmware(&fw, FW_PATH_INTERNAL, &info->client->dev);

	if (!fw) {
		dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		goto error;
	}

	if (mip4_ts_bin_fw_version(info, fw->data, fw->size, ver_buf)) {
		for (i = 0; i < (FW_MAX_SECT_NUM * 2); i++) {
			ver_buf[i] = 0xFF;
		}
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_bin_fw_version\n", __func__);
		goto error;
	}

	release_firmware(fw);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}
#endif /* CHIP_MODEL */

/*
 * Set power state
 */
int mip4_ts_set_power_state(struct mip4_ts_info *info, u8 mode)
{
	u8 wbuf[3];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	dev_dbg(&info->client->dev, "%s - mode[%u]\n", __func__, mode);

	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_POWER_STATE;
	wbuf[2] = mode;
	if (mip4_ts_i2c_write(info, wbuf, 3)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Set wake-up gesture type
 */
int mip4_ts_set_wakeup_gesture_type(struct mip4_ts_info *info, u32 type)
{
	u8 wbuf[6];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	dev_dbg(&info->client->dev, "%s - type[%08X]\n", __func__, type);

	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_GESTURE_TYPE;
	wbuf[2] = (type >> 24) & 0xFF;
	wbuf[3] = (type >> 16) & 0xFF;
	wbuf[4] = (type >> 8) & 0xFF;
	wbuf[5] = type & 0xFF;
	if (mip4_ts_i2c_write(info, wbuf, 6)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Disable ESD alert
 */
int mip4_ts_disable_esd_alert(struct mip4_ts_info *info)
{
	u8 wbuf[4];
	u8 rbuf[4];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_DISABLE_ESD_ALERT;
	wbuf[2] = 1;
	if (mip4_ts_i2c_write(info, wbuf, 3)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
		goto error;
	}

	if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, 1)) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read\n", __func__);
		goto error;
	}

	if (rbuf[0] != 1) {
		dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Alert event handler - ESD
 */
static int mip4_ts_alert_handler_esd(struct mip4_ts_info *info, u8 *rbuf)
{
	u8 frame_cnt = rbuf[1];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

	if (frame_cnt == 0) {
		/* Panel defect, not ESD */
		info->esd_cnt++;
		dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

		if (info->disable_esd == true) {
			mip4_ts_disable_esd_alert(info);
			info->esd_cnt = 0;
		} else if (info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
			if (!mip4_ts_disable_esd_alert(info)) {
				info->disable_esd = true;
				info->esd_cnt = 0;
			}
		} else {
			mip4_ts_restart(info);
		}
	} else {
		/* ESD detected */
		info->esd_cnt = 0;
		mip4_ts_restart(info);
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
 * Alert event handler - Wake-up
 */
static int mip4_ts_alert_handler_wakeup(struct mip4_ts_info *info, u8 *rbuf)
{
	int gesture_code = rbuf[1];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (mip4_ts_gesture_wakeup_event_handler(info, gesture_code)) {
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Alert event handler - Input type
 */
static int mip4_ts_alert_handler_inputtype(struct mip4_ts_info *info, u8 *rbuf)
{
	u8 input_type = rbuf[1];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	switch (input_type) {
	case 0:
		dev_dbg(&info->client->dev, "%s - Input type : Finger\n", __func__);
		break;
	case 1:
		dev_dbg(&info->client->dev, "%s - Input type : Glove\n", __func__);
		break;
	default:
		dev_err(&info->client->dev, "%s - Input type : Unknown[%d]\n", __func__, input_type);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Alert event handler - Image
 */
static int mip4_ts_alert_handler_image(struct mip4_ts_info *info, u8 *rbuf)
{
	u8 image_type = rbuf[1];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	switch (image_type) {
#if USE_WAKEUP_GESTURE
	case MIP4_IMG_TYPE_GESTURE:
		if (mip4_ts_gesture_wakeup_event_handler(info, 1)) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_wakeup_event_handler\n", __func__);
			goto error;
		}
		if (mip4_ts_get_image(info, rbuf[1])) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_get_image\n", __func__);
			goto error;
		}
		break;
#endif /* USE_WAKEUP_GESTURE */
	default:
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Alert event handler - Flash failure
 */
int mip4_ts_alert_handler_flash(struct mip4_ts_info *info, u8 *data)
{
#if USE_FW_RECOVERY
	int ret = 0;
	int retry = 1;
	u8 sections[64];
	int i_byte = 0;
	int i_bit = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	print_hex_dump(KERN_ERR, MIP4_TS_DEVICE_NAME " Sections : ", DUMP_PREFIX_OFFSET, 16, 1, data, 8, false);

	for (i_byte = 0; i_byte < 8; i_byte++) {
		dev_info(&info->client->dev, "%s - packet #%d [0x%02X]\n", __func__, i_byte, data[i_byte]);

		for (i_bit = 0; i_bit < 8; i_bit++) {
			sections[i_byte * 8 + i_bit] = (data[i_byte] >> i_bit) & 0x01;
			dev_info(&info->client->dev, "%s - section #%d [%d]\n", __func__, i_byte * 8 + i_bit, sections[i_byte * 8 + i_bit]);
		}
	}

	ret = mip4_ts_fw_restore_from_kernel(info, sections, retry);

	if (ret < fw_err_none) {
		dev_err(&info->client->dev, "%s - fw_err[%d]\n", __func__, ret);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
#else
	return 0;
#endif /* USE_FW_RECOVERY */
}

/*
 * Alert event handler - SRAM failure
 */
static int mip4_ts_alert_handler_sram(struct mip4_ts_info *info, u8 *data)
{
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	info->sram_addr_num = (unsigned int) (data[0] | (data[1] << 8));
	dev_info(&info->client->dev, "%s - sram_addr_num [%d]\n", __func__, info->sram_addr_num);

	if (info->sram_addr_num > 8) {
		dev_err(&info->client->dev, "%s [ERROR] sram_addr_num [%d]\n", __func__, info->sram_addr_num);
		goto error;
	}

	for (i = 0; i < info->sram_addr_num; i++) {
		info->sram_addr[i] = data[2 + 4 * i] | (data[2 + 4 * i + 1] << 8) | (data[2 + 4 * i + 2] << 16) | (data[2 + 4 * i + 3] << 24);
		dev_info(&info->client->dev, "%s - sram_addr #%d [0x%08X]\n", __func__, i, info->sram_addr[i]);
	}
	for (i = info->sram_addr_num; i < 8; i++) {
		info->sram_addr[i] = 0;
		dev_info(&info->client->dev, "%s - sram_addr #%d [0x%08X]\n", __func__, i, info->sram_addr[i]);
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Alert event handler - Custom
 */
static int __maybe_unused mip4_ts_alert_handler_f1(struct mip4_ts_info *info, u8 *rbuf, u8 size)
{
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
#endif

#if USE_AOT
	if (rbuf[1] == 2) {
		if (mip4_ts_aot_event_handler(info, rbuf, size)) {
			dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
		}
	}
#endif /* USE_AOT */

#if USE_LPWG
	if (mip4_ts_lpwg_event_handler(info, rbuf, size)) {
		dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	}
#endif /* USE_LPWG */
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
#endif
	return 0;
}

/*
 * Interrupt handler
 */
static irqreturn_t mip4_ts_interrupt(int irq, void *dev_id)
{
	struct mip4_ts_info *info = dev_id;
	u8 wbuf[8];
	u8 rbuf[256];
	unsigned int size = 0;
	u8 category = 0;
	u8 alert_type = 0;
	bool startup = false;
	u8 flash_fail_type = flash_fail_none;
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
#endif

#if USE_STARTUP_WAITING
	/* Start-up interrupt */
	if (info->startup_init == false) {
		dev_dbg(&info->client->dev, "%s - skip\n", __func__);
		goto exit;
	}
	if (!completion_done(&info->startup_done)) {
		startup = true;
		dev_dbg(&info->client->dev, "%s - start-up\n", __func__);
	}
#endif /* USE_STARTUP_WAITING */

	/* Read packet info */
	wbuf[0] = MIP4_R0_EVENT;
	wbuf[1] = MIP4_R1_EVENT_PACKET_INFO;
	if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, 1)) {
		dev_err(&info->client->dev, "%s [ERROR] Read packet info\n", __func__);
		if (!startup) {
			info->i2c_error_cnt++;
			goto error;
		} else {
			flash_fail_type = flash_fail_critical;
			goto startup_exit;
		}
	}

	size = (rbuf[0] & 0x3F);
	category = ((rbuf[0] >> 6) & 0x1);
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s - packet info : size[%d] category[%d]\n", __func__, size, category);
#endif

	/* Check packet size */
	if ((size <= 0) || (size > 200)) {
		dev_err(&info->client->dev, "%s [ERROR] Packet size [%d]\n", __func__, size);
		if (!startup) {
			info->i2c_error_cnt++;
			goto error;
		} else {
			flash_fail_type = flash_fail_critical;
			goto startup_exit;
		}
	}

	/* Read packet data */
	wbuf[0] = MIP4_R0_EVENT;
	wbuf[1] = MIP4_R1_EVENT_PACKET_DATA;
	if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, size)) {
		dev_err(&info->client->dev, "%s [ERROR] Read packet data\n", __func__);
		if (!startup) {
			info->i2c_error_cnt++;
			goto error;
		} else {
			flash_fail_type = flash_fail_critical;
			goto startup_exit;
		}
	}

	/* Event handler */
	if (startup == false) {
		LCD_TOUCH_INT_CHECK ++;
		if (category == 0) {
			/* Input event */
			info->esd_cnt = 0;

			mip4_ts_input_event_handler(info, size, rbuf);
		} else {
			/* Alert event */
			alert_type = rbuf[0];
#if USE_INT_DBG		
			dev_dbg(&info->client->dev, "%s - alert type[%d]\n", __func__, alert_type);
#endif
			switch (alert_type) {
			case MIP4_ALERT_ESD:
				if (mip4_ts_alert_handler_esd(info, rbuf)) {
					goto error;
				}
				break;
			case MIP4_ALERT_WAKEUP:
				if (mip4_ts_alert_handler_wakeup(info, rbuf)) {
					goto error;
				}
				break;
			case MIP4_ALERT_INPUT_TYPE:
				if (mip4_ts_alert_handler_inputtype(info, rbuf)) {
					goto error;
				}
				break;
			case MIP4_ALERT_IMAGE:
				if (mip4_ts_alert_handler_image(info, rbuf)) {
					goto error;
				}
				break;
			case MIP4_ALERT_FLASH_FAILURE:
				if (mip4_ts_alert_handler_flash(info, &rbuf[1])) {
					goto error;
				}
				break;
			case MIP4_ALERT_SRAM_FAILURE:
				if (mip4_ts_alert_handler_sram(info, &rbuf[1])) {
					goto error;
				}
				break;
			case MIP4_ALERT_BOOT_SUCCEEDED:
				mip4_ts_startup_config(info);
				mip4_ts_startup_run(info);
				break;
			case MIP4_ALERT_F1:
				if (mip4_ts_alert_handler_f1(info, rbuf, size)) {
					goto error;
				}
				break;
			default:
				dev_err(&info->client->dev, "%s [ERROR] Unknown alert type[%d]\n", __func__, alert_type);
				goto error;
			}
		}
	} else {
#if USE_STARTUP_WAITING
		/* Start-up */
		if (category == 1) {
			alert_type = rbuf[0];
			dev_dbg(&info->client->dev, "%s - alert type[%d]\n", __func__, alert_type);

			switch (alert_type) {
			case MIP4_ALERT_FLASH_FAILURE:
				flash_fail_type = flash_fail_section;
				break;
			case MIP4_ALERT_BOOT_SUCCEEDED:
				flash_fail_type = flash_fail_none;
				break;
			default:
				dev_err(&info->client->dev, "%s [ERROR] Unknown alert type[%d]\n", __func__, alert_type);
				flash_fail_type = flash_fail_critical;
				break;
			}
		} else {
			dev_err(&info->client->dev, "%s [ERROR] Unknown catetory[%d]\n", __func__, category);
			flash_fail_type = flash_fail_critical;
		}
		goto startup_exit;
#endif /* USE_STARTUP_WAITING */
	}

#if USE_STARTUP_WAITING
exit:
#endif /* USE_STARTUP_WAITING */
	info->i2c_error_cnt = 0;
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
#endif
	return IRQ_HANDLED;

startup_exit:
#if USE_STARTUP_WAITING
	complete_all(&info->startup_done);
	dev_dbg(&info->client->dev, "%s - startup : complete_all\n", __func__);

	if (alert_type == MIP4_ALERT_BOOT_SUCCEEDED) {
		mip4_ts_startup_config(info);
		mip4_ts_startup_run(info);
	}

#if USE_FW_RECOVERY
	if (flash_fail_type == flash_fail_critical) {
		mip4_ts_fw_restore_critical_section(info);
	} else if (flash_fail_type == flash_fail_section) {
		mip4_ts_alert_handler_flash(info, &rbuf[1]);
	}
#endif /* USE_FW_RECOVERY */
#endif /* USE_STARTUP_WAITING */
#if USE_INT_DBG
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
#endif
	return IRQ_HANDLED;

error:
#if RESET_ON_EVENT_ERROR
	//dev_err(&info->client->dev, "%s - Reset on event error\n", __func__);
	//mip4_ts_restart(info);
#endif /* RESET_ON_EVENT_ERROR */
#if USE_STARTUP_WAITING

#endif /* USE_STARTUP_WAITING */
#if USE_FW_RECOVERY
	if (info->i2c_error_cnt >= I2C_ERR_CNT_FOR_FW_RECOVERY) {
		mip4_ts_fw_restore_critical_section(info);
	}
#endif /* USE_FW_RECOVERY */

	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return IRQ_HANDLED;
}

/*
 * Config module
 */
int mip4_ts_config(struct mip4_ts_info *info)
{
	u8 wbuf[4];
	u8 rbuf[16];
	int ret = 0;
	int retry = I2C_RETRY_COUNT;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Product name */
	wbuf[0] = MIP4_R0_INFO;
	wbuf[1] = MIP4_R1_INFO_PRODUCT_NAME;
	ret = mip4_ts_i2c_read(info, wbuf, 2, rbuf, 16);
	if (ret) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read\n", __func__);
		goto error;
	}

	memcpy(info->product_name, rbuf, 16);
	dev_dbg(&info->client->dev, "%s - product_name[%s]\n", __func__, info->product_name);

	/* Firmware version */
	ret = mip4_ts_get_fw_version(info, rbuf);
	if (ret) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read\n", __func__);
		goto error;
	}

	memcpy(info->fw_version, rbuf, 8);
	dev_info(&info->client->dev, "%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);

	/* Sometimes chip didn't enough time to read f/w info after chip reset.
	(f/w version read 00.00 00.00 00.00 00.00)
	So add 200ms delay, and retry to read f/w info(from ESD test 18/5/2)*/
	if((info->fw_version[0] == 0)&&(info->fw_version[1] == 0)) 
	{
		printk("%s - F/W not correct\n",__func__);
		goto error;
	}

	/* Resolution */
	wbuf[0] = MIP4_R0_INFO;
	wbuf[1] = MIP4_R1_INFO_RESOLUTION_X;
	ret = mip4_ts_i2c_read(info, wbuf, 2, rbuf, 14);
	if (ret) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read\n", __func__);
		goto error;
	}

	info->max_x = (rbuf[0]) | (rbuf[1] << 8);
	info->max_y = (rbuf[2]) | (rbuf[3] << 8);
	dev_dbg(&info->client->dev, "%s - max_x[%d] max_y[%d]\n", __func__, info->max_x, info->max_y);

	info->ppm_x = rbuf[12];
	info->ppm_y = rbuf[13];
	dev_dbg(&info->client->dev, "%s - ppm_x[%d] ppm_y[%d]\n", __func__, info->ppm_x, info->ppm_y);

	/* Node info */
	info->node_x = rbuf[4];
	info->node_y = rbuf[5];
	info->node_key = rbuf[6];
	dev_dbg(&info->client->dev, "%s - node_x[%d] node_y[%d] node_key[%d]\n", __func__, info->node_x, info->node_y, info->node_key);

	/* Key info */
	if (info->node_key > 0) {
		/* Enable touchkey */
		info->key_enable = true;
		info->key_num = info->node_key;
	}

	/* Protocol */
	wbuf[0] = MIP4_R0_EVENT;
	wbuf[1] = MIP4_R1_EVENT_FORMAT;
	while (retry--) {
		if (mip4_ts_i2c_read(info, wbuf, 2, rbuf, 3)) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_read - event format\n", __func__);
		} else {
			info->event_format = rbuf[0] | (rbuf[1] << 8);
			info->event_size = rbuf[2];
			if (info->event_size <= 0) {
				dev_err(&info->client->dev, "%s [ERROR] event_size[%d]\n", __func__, info->event_size);
				goto error;
			}
			dev_dbg(&info->client->dev, "%s - event_format[%d] event_size[%d]\n", __func__, info->event_format, info->event_size);
			break;
		}
	}
	if (retry < 0) {
		dev_err(&info->client->dev, "%s [ERROR] event format - retry limit\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Config platform
 */
static int mip4_ts_config_platform(struct mip4_ts_info *info)
{
#ifdef CONFIG_ACPI
	struct acpi_device *a_dev;
	acpi_status a_status;
#endif /* CONFIG_ACPI */
	int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

#ifdef CONFIG_ACPI
	/* ACPI */
	dev_dbg(&info->client->dev, "%s - ACPI\n", __func__);

	a_status = acpi_bus_get_device(ACPI_HANDLE(&info->client->dev), &a_dev);
	if (ACPI_SUCCESS(a_status)) {
		if (strncmp(dev_name(&a_dev->dev), ACPI_ID, 8) != 0) {
			dev_err(&info->client->dev, "%s [ERROR] ACPI_ID mismatch [%s]\n", __func__, dev_name(&a_dev->dev));
			ret = -EINVAL;
			goto exit;
		}
	}
#else

#ifdef CONFIG_OF
	/* Devicetree */
	if (&info->client->dev.of_node) {
		dev_dbg(&info->client->dev, "%s - Devicetree\n", __func__);

		ret = mip4_ts_parse_devicetree(&info->client->dev, info);
		if (ret) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_parse_devicetree\n", __func__);
			goto exit;
		}
	} else {
		dev_err(&info->client->dev, "%s [ERROR] of_node\n", __func__);
		goto exit;
	}
#else
	/* Platform device */
	dev_dbg(&info->client->dev, "%s - Platform device\n", __func__);

	info->pdata = dev_get_platdata(&info->client->dev);
	if (info->pdata == NULL) {
		ret = -EINVAL;
		dev_err(&info->client->dev, "%s [ERROR] dev_get_platdata\n", __func__);
		goto exit;
	}

	info->gpio_intr = info->pdata->gpio_intr;
	info->gpio_ce = info->pdata->gpio_ce;
	info->gpio_vd33_en = info->pdata->gpio_vd33_en;

	ret = mip4_ts_config_gpio(info);
	if (ret) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_config_gpio\n", __func__);
		goto exit;
	}
#endif /* CONFIG_OF */

#endif /* CONFIG_ACPI */

exit:
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return ret;
}

#if (CHIP_MODEL != CHIP_NONE)
#if USE_FW_RECOVERY
/*
 * Restore critical section of firmware from kernel built-in firmware file
 */
int mip4_ts_fw_restore_critical_section(struct mip4_ts_info *info)
{
	int ret = 0;
	int i;
	u8 sections[FW_SECTION_NUM];
	int retry = 1;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Restore firmware */
	memset(sections, 0, FW_SECTION_NUM);
	for (i = 0; i < FW_CRITICAL_SECTION_NUM; i++) {
		sections[i] = 1;
	}

	ret = mip4_ts_fw_restore_from_kernel(info, sections, retry);
	if (ret < 0) {
		dev_err(&info->client->dev, "%s [ERROR] mms_fw_restore_from_kernel\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/*
 * Restore firmware from kernel built-in firmware file
 */
int mip4_ts_fw_restore_from_kernel(struct mip4_ts_info *info, u8 *sections, int retry)
{
	const char *fw_name = FW_PATH_INTERNAL;
	const struct firmware *fw;
	int ret = fw_err_none;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	//mutex_lock(&info->lock);
	mip4_ts_clear_input(info);

	print_hex_dump(KERN_ERR, MIP4_TS_DEVICE_NAME " Sections : ", DUMP_PREFIX_OFFSET, 16, 1, sections, FW_SECTION_NUM, false);

	/* Get firmware */
	request_firmware(&fw, fw_name, &info->client->dev);
	if (!fw) {
		dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		ret = fw_err_file_open;
		goto error;
	}

	/* Restore firmware */
	while (retry > 0) {
		ret = mip4_ts_restore_fw(info, fw->data, fw->size, sections);
		dev_err(&info->client->dev, "%s - mip4_ts_restore_fw[%d]\n", __func__, ret);
		if (ret >= fw_err_none) {
			break;
		}

		retry--;
	}
	if (retry <= 0) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_restore_fw failed\n", __func__);
		ret = fw_err_download;
	} else {
		ret = fw_err_none;
	}

	release_firmware(fw);

	if (ret < fw_err_none) {
		goto error;
	}

	//mutex_unlock(&info->lock);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	//mutex_unlock(&info->lock);

	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return ret;
}
#endif

/*
 * Update firmware from kernel built-in firmware file
 */
int mip4_ts_fw_update_from_kernel(struct mip4_ts_info *info)
{
	const char *fw_name = FW_PATH_INTERNAL;
	const struct firmware *fw;
	int retry = 1;
	int ret = fw_err_none;
	u16 ver_chip[FW_MAX_SECT_NUM];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Disable IRQ */
	mutex_lock(&info->lock);
	disable_irq(info->client->irq);
	mip4_ts_clear_input(info);

	/* Get firmware */
	if(LCD_VER == 4)
	{
		request_firmware(&fw, FW_PATH_INTERNAL_12_3, &info->client->dev);
	}
	else
	{
		request_firmware(&fw, FW_PATH_INTERNAL_10_25, &info->client->dev);	
		
		mip4_ts_get_fw_version_u16(info, ver_chip);

		if((ver_chip[3]&0xF000)==0xF000&&ver_chip[3]!=0xFFFF)
		{
			request_firmware(&fw, FW_PATH_INTERNAL_10_25_plastic, &info->client->dev);
		}
	}

	if (!fw) {
		dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		ret = fw_err_file_open;
		goto error;
	}

	/* Update firmware */
	do {
		ret = mip4_ts_flash_fw(info, fw->data, fw->size, false, true);
		if (ret >= fw_err_none) {
			break;
		}
	} while (--retry);

	if (!retry) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_flash_fw failed\n", __func__);
		ret = fw_err_download;
	}

	release_firmware(fw);

	/* Enable IRQ */
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret < fw_err_none) {
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return ret;
}

/*
 * Update firmware from external storage
 */
int mip4_ts_fw_update_from_storage(struct mip4_ts_info *info, char *path, bool force)
{
	struct file *fp;
	mm_segment_t old_fs;
	size_t fw_size, nread;
	unsigned char *fw_data;
	int ret = fw_err_none;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Disable IRQ */
	mutex_lock(&info->lock);
	disable_irq(info->client->irq);
	mip4_ts_clear_input(info);

	/* Get firmware */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(path, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		dev_err(&info->client->dev, "%s [ERROR] file_open - path[%s]\n", __func__, path);
		ret = fw_err_file_open;
		goto error;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (fw_size > 0) {
		/* Read firmware */
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
		dev_dbg(&info->client->dev, "%s - path[%s] size[%zu]\n", __func__, path, fw_size);

		if (nread != fw_size) {
			dev_err(&info->client->dev, "%s [ERROR] vfs_read - size[%zu] read[%zu]\n", __func__, fw_size, nread);
			ret = fw_err_file_read;
		} else {
			/* Update firmware */
			ret = mip4_ts_flash_fw(info, fw_data, fw_size, force, true);
		}

		kfree(fw_data);
	} else {
		dev_err(&info->client->dev, "%s [ERROR] fw_size[%zu]\n", __func__, fw_size);
		ret = fw_err_file_read;
	}

	filp_close(fp, current->files);

error:
	set_fs(old_fs);

	/* Enable IRQ */
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret < fw_err_none) {
		dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	} else {
		dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	}

	return ret;
}

/*
 * Sysfs - firmware update
 */
static ssize_t mip4_ts_sys_fw_update(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mip4_ts_info *info = i2c_get_clientdata(client);
	int result = 0;
	u8 data[255];
	int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	mip4_serdes_reset_dwork_stop(info);

	ret = mip4_ts_fw_update_from_kernel(info);

	switch (ret) {
	case fw_err_none:
		snprintf(data, sizeof(data), "F/W update success.\n");
		break;
	case fw_err_uptodate:
		snprintf(data, sizeof(data), "F/W is already up-to-date.\n");
		break;
	case fw_err_download:
		snprintf(data, sizeof(data), "F/W update failed : Download error\n");
		break;
	case fw_err_file_type:
		snprintf(data, sizeof(data), "F/W update failed : File type error\n");
		break;
	case fw_err_file_open:
		snprintf(data, sizeof(data), "F/W update failed : File open error[%s]\n", info->fw_path_ext);
		break;
	case fw_err_file_read:
		snprintf(data, sizeof(data), "F/W update failed : File read error\n");
		break;
	default:
		snprintf(data, sizeof(data), "F/W update failed.\n");
		break;
	}

	/* Re-config driver */
	mip4_ts_config(info);
	mip4_ts_config_input(info);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	mip4_serdes_reset_dwork_start(info);

	result = snprintf(buf, 255, "%s\n", data);
	return result;
}


static ssize_t show_debug(struct device *dev,struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "CURRENT DEBUG MODE : %d\n"
                                        "0:NONE\n" "1:INFO\n"
                                        "2:MESSAGES\n" "3:TRACE\n", debug);
}

static ssize_t set_debug(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        int state;
        struct i2c_client *client;
        struct mxt_data *mxt;

        client = to_i2c_client(dev);
        mxt = i2c_get_clientdata(client);

        sscanf(buf, "%d", &state);
        if (state >= DEBUG_NONE && state <= DEBUG_TRACE) {
                debug = state;
                pr_info("\tCURRENT DEBUG MODE : %d\n"
                                "\t0:NONE\n" "\t1:INFO\n"
                                "\t2:MESSAGES\n" "\t3:TRACE\n", debug);
        }
        else {
                return -EINVAL;
        }

        return count;
}




static DEVICE_ATTR(fw_update, S_IRUGO, mip4_ts_sys_fw_update, NULL);
static DEVICE_ATTR(8_debug,( S_IWUSR | S_IWGRP ) | S_IRUGO, show_debug, set_debug);

#endif /* CHIP_MODEL */

/*
 * Sysfs attr info
 */
static struct attribute *mip4_ts_attrs[] = {
#if (CHIP_MODEL != CHIP_NONE)
	&dev_attr_fw_update.attr,
#endif /* CHIP_MODEL */
	&dev_attr_8_debug.attr,
	NULL,
};

/*
 * Sysfs attr group info
 */
static const struct attribute_group mip4_ts_attr_group = {
	.attrs = mip4_ts_attrs,
};


static ssize_t mip4_fops_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
        printk("%s\n", __func__);
        return 0;
}

static ssize_t mip4_fops_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
        printk("%s\n", __func__);
        return 0;
}

static int mip4_fops_open(struct inode *inode, struct file *file)
{
        return 0;
}

static int mip4_fops_release(struct inode *inode, struct file *file)
{
        return 0;
}


static long mip4_fops_ioctl(struct file *file, unsigned int cmd, unsigned long _arg)
{
/*	struct mip4_ts_info *info = &g_info;
	u8 wbuf[4];

	printk("%s [Start]\n",__func__);

	switch(cmd){
		
		case DAUDIO_TOUCHIOC_CHECK_RE_CAL:
			printk("%s DAUDIO_TOUCHIOC_CHECK_RE_CAL\n",__func__);
			wbuf[0] = MIP4_R0_CTRL;
			wbuf[1] = MIP4_R1_CTRL_RECALIBRATE;
			wbuf[2] = 1;
			mip4_ts_i2c_write(info, wbuf, 3);
			break;
		default:
			break;
	}
*/
	return 0;
}

static const struct file_operations mip4_misc_fops = {
        /**
        * @ legolamp@cleinsoft
        * @ date 2014.05.01
        * @ add mxt336S_fops_write, mxt336S_fops_read, mxt336S_fops_ioctl
        **/
        .unlocked_ioctl = mip4_fops_ioctl,
        .write                  = mip4_fops_write,
        .read                   = mip4_fops_read,
        .open                   = mip4_fops_open,
        .release                = mip4_fops_release,
};

static struct miscdevice mip4_misc = {
        .name = "mxt_misc",
        .fops = &mip4_misc_fops,
        .minor = MISC_DYNAMIC_MINOR,

        .mode = 0660,
};

/*
 * Initialize driver
 */
static int mip4_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mip4_ts_info *info = &g_info;
	struct input_dev *input_dev;
	int ret = 0;	
	
	dev_dbg(&client->dev, "%s [START]\n", __func__);

	/* Check I2C functionality */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s [ERROR] i2c_check_functionality\n", __func__);
		ret = -EIO;
		goto error_i2c;
	}

	/* Init info data */
	info = devm_kzalloc(&client->dev, sizeof(struct mip4_ts_info), GFP_KERNEL);
	input_dev = devm_input_allocate_device(&client->dev);
	if (!info || !input_dev) {
		dev_err(&client->dev, "%s [ERROR]\n", __func__);
		ret = -ENOMEM;
		goto error_info;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->init = true;
	info->power = -1;
	info->irq_enabled = false;
	info->fw_path_ext = kstrdup(FW_PATH_EXTERNAL, GFP_KERNEL);
	mutex_init(&info->lock);
	init_completion(&info->startup_done);
	info->startup_init = false;

	/* Config platform */
	ret = mip4_ts_config_platform(info);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] mip4_ts_config_platform\n", __func__);
		goto error_info;
	}

	/* Config input device */
	info->input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));
	info->input_dev->phys = info->phys;
	info->input_dev->id.bustype = BUS_I2C;
	info->input_dev->id.vendor = 0x13C5;
	info->input_dev->dev.parent = &client->dev;
#if USE_INPUT_OPEN_CLOSE
	info->input_dev->open = mip4_ts_input_open;
	info->input_dev->close = mip4_ts_input_close;
#endif /* USE_INPUT_OPEN_CLOSE */

	info_mip4 = info;
	
	/* Set info data */
	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);

	/* Config regulator */
	mip4_ts_config_regulator(info);

	mip4_ts_power_off(info);

	/* Firmware update */
#if USE_AUTO_FW_UPDATE
	mip4_ts_power_on(info);

	LCD_VER = daudio_lcd_version();

	if(LCD_VER != 10)//Condition : Connect Monitor
	{

		if(LCD_VER == 3 || LCD_VER == 4)//Condition : Departed Monitor
		{
			if(serdes_i2c_read(info->client) == (LCD_VER==4 ? 1: 0xde))//Condition : Deserializer response
			{
				ret = mip4_ts_fw_update_from_kernel(info);
			}
			else
			{
				printk("%s [ERROR] LCD_VER : %d Deserializer doesn't response\n ",__func__,LCD_VER);
			}
		}
		else//Condition : Integrated Monitor
		{
			ret = mip4_ts_fw_update_from_kernel(info);
		}
		if (ret) {
			dev_err(&client->dev, "%s [ERROR] mip4_ts_fw_update_from_kernel\n", __func__);
		}
	}
//	mip4_ts_power_off(info);
#endif /* USE_AUTO_FW_UPDATE */

	/* Set interrupt handler */
	//ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, mip4_ts_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, MIP4_TS_DEVICE_NAME, info);
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, mip4_ts_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, MIP4_TS_DEVICE_NAME, info);

	if (ret) {
		dev_err(&client->dev, "%s [ERROR] request_threaded_irq\n", __func__);
		goto error_irq;
	}

	disable_irq(info->client->irq);
	info->irq_enabled = false;
	info->startup_init = true;

	/* Enable device */
	mip4_ts_enable(info);

	/* Config module */
	ret = mip4_ts_config(info);
	
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] mip4_ts_config\n", __func__);
		mip4_ts_disable(info);
		LCD_TOUCH_RESET_NEED = 1;
	}

	/* Config input interface */
	mip4_ts_config_input(info);

	/* Register input device */
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "%s [ERROR] input_register_device\n", __func__);
		ret = -EIO;
		goto error_device;
	}

	misc_register(&mip4_misc);

#if USE_WAKEUP_GESTURE
	/* Wake-lock for wake-up gesture mode */
	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, "mip4_ts_wake_lock");
#endif /* USE_WAKEUP_GESTURE */

#if USE_FB_NOTIFY
	info->fb_notifier.notifier_call = mip4_ts_fb_notifier_callback;
	fb_register_client(&info->fb_notifier);
#endif /* USE_FB_NOTIFY */

#if USE_DEV
	/* Create dev node (optional) */
	if (mip4_ts_dev_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mip4_ts_dev_create\n", __func__);
	}

	info->class = class_create(THIS_MODULE, MIP4_TS_DEVICE_NAME);
	device_create(info->class, NULL, info->mip4_ts_dev, NULL, MIP4_TS_DEVICE_NAME);
#endif /* USE_DEV */

#if USE_SYS
	/* Create sysfs for development functions (optional) */
	if (mip4_ts_sysfs_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mip4_ts_sysfs_create\n", __func__);
	}
#endif /* USE_SYS */

#if USE_CMD
	/* Create sysfs for command functions (optional) */
	if (mip4_ts_sysfs_cmd_create(info)) {
		dev_err(&client->dev, "%s [ERROR] mip4_ts_sysfs_cmd_create\n", __func__);
	}
#endif /* USE_CMD */

	/* Create sysfs */
	if (sysfs_create_group(&client->dev.kobj, &mip4_ts_attr_group)) {
		dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
	}
	if (sysfs_create_link(NULL, &client->dev.kobj, MIP4_TS_DEVICE_NAME)) {
		dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
	}


//#if defined(INCLUDE_MAX_SER_DES)
	mutex_init(&info->serdes.lock);
	INIT_DELAYED_WORK(&info->serdes.reset_dwork, mip4_serdes_reset_dwork);
	mip4_serdes_reset_dwork_start(info);
//#endif

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);
	dev_info(&client->dev, "MELFAS " CHIP_NAME " Touchscreen\n");
	return 0;

error_irq:
	free_irq(info->irq, info);
error_device:
	input_unregister_device(info->input_dev);
error_info:
error_i2c:
	dev_dbg(&client->dev, "%s [ERROR]\n", __func__);
	dev_err(&client->dev, "MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
	return ret;
}

/*
 * Remove driver
 */
static int mip4_ts_remove(struct i2c_client *client)
{
	struct mip4_ts_info *info = i2c_get_clientdata(client);

#if USE_CMD
	mip4_ts_sysfs_cmd_remove(info);
#endif /* USE_CMD */

#if USE_SYS
	mip4_ts_sysfs_remove(info);
#endif /* USE_SYS */

	sysfs_remove_group(&info->client->dev.kobj, &mip4_ts_attr_group);
	sysfs_remove_link(NULL, MIP4_TS_DEVICE_NAME);

#if USE_DEV
	device_destroy(info->class, info->mip4_ts_dev);
	class_destroy(info->class);
#endif /* USE_DEV */

//#if defined(INCLUDE_MAX_SER_DES)
	mip4_serdes_reset_dwork_stop(info);
	mutex_destroy(&info->serdes.lock);
//#endif

#if USE_FB_NOTIFY
	fb_unregister_client(&info->fb_notifier);
#endif /* USE_FB_NOTIFY */

	input_unregister_device(info->input_dev);

	return 0;
}

#ifdef CONFIG_PM
/*
 * Device suspend event handler
 */
int mip4_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mip4_ts_info *info = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	mip4_ts_disable(info);

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);

	return 0;
}

/*
 * Device resume event handler
 */
int mip4_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mip4_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	mip4_ts_enable(info);

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);

	return ret;
}
                  

/*
 * PM info
 */
const struct dev_pm_ops mip4_ts_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mip4_ts_suspend, mip4_ts_resume)
};
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
/*
 * Devicetree match table
 */
static const struct of_device_id mip4_ts_of_match_table[] = {
	{ .compatible = "melfas,mip4_ts", },
	{ },
};
MODULE_DEVICE_TABLE(of, mip4_ts_of_match_table);
#endif /* CONFIG_OF */

#ifdef CONFIG_ACPI
/*
 * ACPI match table
 */
static const struct acpi_device_id mip4_ts_acpi_match_table[] = {
	{ACPI_ID, 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, mip4_ts_acpi_match_table);
#endif /* CONFIG_ACPI */

/*
 * I2C Device ID
 */
static const struct i2c_device_id mip4_ts_id[] = {
	{MIP4_TS_DEVICE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mip4_ts_id);

/*
 * I2C driver info
 */
static struct i2c_driver mip4_ts_driver = {
	.id_table = mip4_ts_id,
	.probe = mip4_ts_probe,
	.remove = mip4_ts_remove,
	.driver = {
		.name = MIP4_TS_DEVICE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mip4_ts_of_match_table),
#endif /* CONFIG_OF */
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(mip4_ts_acpi_match_table),
#endif /* CONFIG_ACPI */
#ifdef CONFIG_PM
		.pm = &mip4_ts_pm_ops,
#endif /* CONFIG_PM */
	},
};

static int __init mip4_init(void){
	int err;

	LCD_VER = daudio_lcd_version();

	printk("%s, GPIO_B24 = %d, GPIO_B13 = %d,  daudio_lcd_version() = %d \n", __func__, gpio_get_value(TCC_GPB(24)), gpio_get_value(TCC_GPB(13)), daudio_lcd_version());

	switch(LCD_VER){
		case 0:
		case 3:
		case 4:
#ifndef CONFIG_LCD_RESOLUTION_1280_720
		case 10:
#endif
			err = i2c_add_driver(&mip4_ts_driver);
			if(err)
				printk("mip4 touch driver i2c driver add failed (errno = %d)\n",err);

			return err;

			break;

		default:
			return -ENODEV;

			break;

	}
}

module_init(mip4_init);
//module_i2c_driver(mip4_ts_driver);

MODULE_DESCRIPTION("MELFAS MIP4 Touchscreen");
MODULE_VERSION("2017.05.23");
MODULE_AUTHOR("Sangwon Jee <jeesw@melfas.com>");
MODULE_LICENSE("GPL");

