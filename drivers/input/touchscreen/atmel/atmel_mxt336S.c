/*
 * atmel_mxt336S.c - Atmel maXTouch Touchscreen Controller
 *
 **/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/atmel_mxt336S.h>
#include <linux/uaccess.h>
#include "atmel_mxt336S_cfg.h"
#include "device_config_mxt336S.h"
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <mach/daudio.h>
#include <linux/completion.h>
//#include <mach/daudio_pinctl.h>
//#include <mach/daudio_info.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include <mach/daudio_info.h>

#define DRIVER_NAME	"mxt336s"

#ifdef CONFIG_ARCH_MXC
#include <mach/hardware.h>
#endif
#include <linux/miscdevice.h>

#define	TS_100S_TIMER_INTERVAL	1

#define _MXT_REGUP_NOOP		0
#define _MXT_REGUP_RUNNING	1
#define _MXT_REGUP_END		2

#define AUTO_CAL_DISABLE	0
#define AUTO_CAL_ENABLE		1

/* amplitude limitation */
#define AMPLITUDE_LIMIT 250

#define RESET_DURATION			25
#define RESET_DELAY				28

#if defined(CONFIG_DAUDIO)
#if defined(CONFIG_DAUDIO_20140220)
#define HW_RESET_GPIO	TCC_GPF(2)
#endif
#endif

#define DEBUG_MXT_336S	1

#if (DEBUG_MXT_336S)
#define VPRINTK(fmt, args...) printk(KERN_INFO "[mXT336S-AT] " fmt, ##args)
#else
#define VPRINTK(args...) do {} while(0)
#endif

unsigned int touch_type;

enum {
	MXT_UP_CMD_BEGIN  = 1,
	/* auto firmware upgrade */
	MXT_UP_AUTO = MXT_UP_CMD_BEGIN,
	/* lock for the user firmware upgrade */
	MXT_UP_LOCK,
	/* unlock for the user firmware upgrade */
	MXT_UP_UNLOCK,
	/* force fail test. this is for mxt_resume
	 * if you want to this test,
	 *  debug value should not be equal to DEBUG_NONE
	 * */
	/* check the current status and firmware upgrade */
	MXT_UP_CHECK_AND_AUTO,
	MXT_UP_CMD_END,
};

enum {
	MXT_FIRMUP_FLOW_LOCK = (1 << 1),
	MXT_FIRMUP_FLOW_UPGRADE = (1 << 2),
	MXT_FIRMUP_FLOW_UNLOCK = (1 << 3),
};

enum {
	MXT_FIRM_STATUS_STATBLE = 0,
	MXT_FIRM_STATUS_START_STABLE,
	MXT_FIRM_STATUS_START_UNSTABLE,
	MXT_FIRM_STATUS_SUCCESS,
	MXT_FIRM_STATUS_FAIL,
};

#ifdef MXT_FIRMUP_ENABLE
#define MXT_FIRM_CLEAR(status) (status = MXT_FIRM_STATUS_STATBLE)
#define MXT_FIRM_UPGRADING_STABLE(status) \
	(status == MXT_FIRM_STATUS_START_STABLE)
#define MXT_FIRM_UPGRADING_UNSTABLE(status) \
	(status == MXT_FIRM_STATUS_START_UNSTABLE)
#define MXT_FIRM_UPGRADING(status) (MXT_FIRM_UPGRADING_STABLE(status) || \
		MXT_FIRM_UPGRADING_UNSTABLE(status))
#define MXT_FIRM_STABLE(status) \
	(status == MXT_FIRM_STATUS_STATBLE || status == MXT_FIRM_STATUS_SUCCESS)
#else
#define MXT_FIRM_CLEAR(status)
#define MXT_FIRM_UPGRADING_STABLE(status) (0)
#define MXT_FIRM_UPGRADING_UNSTABLE(status) (0)
#define MXT_FIRM_UPGRADING(status)	(0)
#define MXT_FIRM_STABLE(status)		(1)
#endif

#undef TSP_DEBUG_MESSAGE

#define MXT_SW_RESET_TIME	400
#define I2C_RETRY_COUNT 	3
#define MXT_INIT_FIRM_ERR 	1

/* static bool cal_check_flag; */
static u8 facesup_message_flag_T9;
/* check touch flag header byte */
static u16 MXT_MSG_T37_CMD_TOUCH_HEAD[2] = {0xF3, 0x00};
static u8 check_hardkey_cnt = 0;
/* error logging system */
static u16 gtouch_ic_error = 0;
static u16 gtouch_ic_reset_cnt = 0;

static struct atmel_touch_version prev_ver;
static struct atmel_touch_version nvram_ver;
static struct atmel_touch_version new_ver;
static struct atmel_touch_self_test test;
static struct completion pinfault_done;

static struct mxt_data g_mxt;

static int LCD_VER = 0;

extern int daudio_lcd_version(void);


#define ABS(x, y)		((x < y) ? (y - x) : (x - y))

#ifdef TSP_DEBUG_MESSAGE
#define MAX_MSG_LOG_SIZE	512
struct {
	u8 id[MAX_MSG_LOG_SIZE];
	u8 status[MAX_MSG_LOG_SIZE];
	u16 xpos[MAX_MSG_LOG_SIZE];
	u16 ypos[MAX_MSG_LOG_SIZE];
	u8 area[MAX_MSG_LOG_SIZE];
	u8 amp[MAX_MSG_LOG_SIZE];
	u16 cnt;
} msg_log;
#endif

/* level of debugging messages */
#define DEBUG_NONE		0
#define DEBUG_INFO      1
#define DEBUG_MESSAGES  2
#define DEBUG_TRACE     3
#define CONFIG_KERNEL_DEBUG_SEC 1

// #define DEBUG_JIG

//#define REMOVE_DEBUG_CODE
#ifndef REMOVE_DEBUG_CODE
#define mxt_debug_info(mxt, fmt, arg...) \
	if (debug >= DEBUG_INFO) \
		dev_info(&mxt->client->dev, fmt, ## arg);

#define mxt_debug_msg(mxt, fmt, arg...) \
	if (debug >= DEBUG_MESSAGES) \
		dev_info(&mxt->client->dev, fmt, ## arg);

#define mxt_debug_trace(mxt, fmt, arg...) \
	if (debug >= DEBUG_TRACE) \
		dev_info(&mxt->client->dev, fmt, ## arg);
#else
#define mxt_debug_msg(p...) do {} while (0);
#define mxt_debug_info(p...) do {} while (0);
#define mxt_debug_trace(p...) do {} while (0);
#endif

/* for debugging,  enable DEBUG_TRACE */
static int debug = DEBUG_NONE;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

static void mxt_release_all_fingers(struct mxt_data *mxt);

/* \brief check the chip's calibration */
static void check_chip_calibration(struct mxt_data *mxt);
static void check_chip_calibration_t100(u8 *message, struct mxt_data *mxt);

/* \brief calibration may be good. and check the condition one more time */
static void cal_maybe_good(struct mxt_data *mxt);

/* \brief calibrate chip */
static int calibrate_chip(struct mxt_data *mxt);

/* \brief check the palm-suppression */

static void check_chip_palm(struct mxt_data *mxt);

/* \brief check the touch flag */
static int check_touch_flag(struct mxt_data *mxt, int *touch, int *anti_touch);

/* \brief write the initial registers
 *
 * @param mxt struct mxt_data pointer
 * @return if success return 0, otherwise < 0
 */
static int mxt_write_init_registers(struct mxt_data *mxt);

/* \brief run re-cal if there'is no release msg */
static void mxt_supp_ops_dwork(struct work_struct *work);

/* \brief stop the suppression operation */
static inline void mxt_supp_ops_stop(struct mxt_data *mxt);

static void process_t62_message(u8 *message, struct mxt_data *mxt);

static void process_t6_message(u8 *message, struct mxt_data *mxt);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif

#define MXT_100MS_TIMER_LIMIT	10
#define MXT_100MS_ERROR_LIMIT	5

#if TS_100S_TIMER_INTERVAL
static struct workqueue_struct *ts_100s_tmr_workqueue;
static void ts_100ms_timeout_handler(unsigned long data);
static void ts_100ms_timer_start(struct mxt_data *mxt);
static void ts_100ms_timer_stop(struct mxt_data *mxt);
static void ts_100ms_timer_init(struct mxt_data *mxt);
static void ts_100ms_tmr_work(struct work_struct *work);
#endif
static void ts_100ms_timer_clear(struct mxt_data *mxt);
static void ts_100ms_timer_enable(struct mxt_data *mxt);

/* \brief initialize mxt336S
 * @param client struct i2c_client pointer
 * @param mxt struct mxt_data pointer
 * @return if < 0 error, 0 is ok, 1 is firmware upgrade
 */
static int mxt336s_initialize(struct i2c_client *client, struct mxt_data *mxt);
static int mxt_identify(struct i2c_client *client, struct mxt_data *mxt);
static int mxt_read_object_table(
	struct i2c_client *client, struct mxt_data *mxt);

/* \brief get the current configuration version */
static int mxt_get_version(
	struct mxt_data *mxt, struct atmel_touch_version *ver);

/* \brief if the current configuration version is higher than previous one,
 *	bakcup to nvram */
static void mxt_update_backup_nvram(
	struct mxt_data *mxt, struct atmel_touch_version *old_ver);

#ifdef MXT_FIRMUP_ENABLE
static int set_mxt_auto_update_exe(struct mxt_data *mxt, int cmd, int state, char* path);
#endif

static int _check_recalibration(struct mxt_data *mxt);
static inline void _disable_auto_cal(struct mxt_data *mxt, int locked);
/* \brief delayed work
 * If enter hardware key, check touch, anti-touch
 * if tch !=0 and atch !=0, execute re-calibration function.
 */
static void mxt_cal_timer_dwork(struct work_struct *work);
static int maybe_good_count;

static int mxt336s_resume_reset(struct i2c_client *client);

/* Brief
 * It is used instead of T25(self test)
 * This will check touch ic status.
 */
#define MXT_CHK_TOUCH_IC_TIME		(4000)
#define MXT_CHK_PALM_TOUCH_TIME     (3000)
static void mxt_check_touch_ic_timer_dwork(struct work_struct *work);
static void mxt_check_touch_ic_timer_start(struct mxt_data *mxt);
static inline void mxt_check_touch_ic_timer_stop(struct mxt_data *mxt);


static int mxt_write_onebyte_register(struct i2c_client *client, u8 addr, u8 value);

const u8 *maxtouch_family = "maXTouch";
const u8 *mxt336S_variant  = "mXT336S";

u8 *object_type_name[MXT_MAX_OBJECT_TYPES] = {
	[5] = "GEN_MESSAGEPROCESSOR_T5",
	[6] = "GEN_COMMANDPROCESSOR_T6",
	[7] = "GEN_POWERCONFIG_T7",
	[8] = "GEN_ACQUIRECONFIG_T8",
	[9] = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "GPIOPWN_CONFIG_T19",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[42] = "PROCI_TOUCHSUPPRESSION_T42",
	[43] = "HID_CONFI_T43",
	[44] = "MESSAGE_COUNTER_T44",
	[46] = "SPT_CTECONFIG_T46",
	[47] = "PROCI_STYLUS_T47",
	[62] = "PROCG_NOISESUPPRESSION_T62",
	[52] = "PROXIMITY_KEY_T52",
	[53] = "DATA_SOURCE_T53",
	[55] = "ADAPTIVETHRESHOLD_T55",
	[56] = "SHIEDLESS_T56",
	[57] = "EXTRATOUCHSCREENDATA_T57",
};

static struct multi_touch_info {
	uint16_t size;
	int16_t pressure;
	/* pressure(-1) : reported
	 * pressure(0) : released
	 * pressure(> 0) : pressed */
	int16_t x;
	int16_t y;
	/* int16_t component; */
} mtouch_info[MXT_MAX_NUM_TOUCHES];

static struct mxt_palm_info {
	u8 facesup_message_flag;
	bool palm_check_timer_flag;
	bool palm_release_flag;
} palm_info = {
	.palm_release_flag = true,
};
/* 128 byte is enough to mxt336S */
#define MXT_MSG_BUFF_SIZE 256
static u8 mxt_message_buf[MXT_MSG_BUFF_SIZE];


// 2015/08/21, YG 분리형 모니터 부팅시 모니터 연결 해제 상황 체크 변수
static int mxt_initialize_device = true;

#define MXT_RESERVED_T255			255u
#define MXT_CRC_TIMEOUT 1000	/* MSEC */

static void mxt_read_message_reportid(struct mxt_data *mxt, uint8_t reportid, uint32_t *crc);
static int mxt_t6_reportall_command(struct mxt_data *mxt);
static uint8_t type_to_report_id(struct mxt_data *mxt, uint8_t object_type, uint8_t instance);
int mxt_read_no_delay_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);
static void mxt_read_config_crc(struct mxt_data *mxt, uint32_t *crc);
static int mxt_wait_touch_flag(struct mxt_data *mxt, uint16_t addr_t37, uint16_t addr_t6, unsigned char cmd);
static int mxt_read_touch_flag(struct mxt_data *mxt, uint16_t addr_t37,	unsigned char page, char *buff, int buff_size);
static int check_touch_count(struct mxt_data *mxt, int *touch,	int *anti_touch);

static void mxt_read_config_crc(struct mxt_data *mxt, uint32_t *crc)
{
	uint8_t status;
	uint8_t t6_rid;

	// disable the interrupt in order to read the message manually
	// __interrupt_disable();

	/* read pending message */
	mxt_read_message_reportid(mxt, MXT_RESERVED_T255, crc);

	/* Send Report all command */
	status = mxt_t6_reportall_command(mxt);

	/* Read command processor message */
	t6_rid = type_to_report_id(mxt, MXT_GEN_COMMANDPROCESSOR_T6, 0);

	mxt_read_message_reportid(mxt, t6_rid, crc);

	return;
}

static int mxt_t6_reportall_command(struct mxt_data *mxt)
{
	int ret;

	ret = mxt_write_byte(mxt->client,
						 MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
						 MXT_ADR_T6_REPORTALL,
						 0x01);
	/* 2012_0523 :
	 *  - ATMEL : please, log the below code */
	mxt_debug_info(mxt, "%s()\n", __func__);
	/* 2012_0515 :
	 *  - ATMEL : 100ms needs to backup to nvram */
	msleep(50);

	return ret;
}

static void mxt_read_message_reportid(struct mxt_data *mxt, uint8_t reportid, uint32_t *crc)
{
	struct  i2c_client *client;
	u8      *message = NULL;
	u16     message_length;
	u16     message_addr;
	int     error;
	uint16_t try_cnt;
	uint16_t fail_count;

	fail_count = mxt->report_id_count * 2;

	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	message = mxt_message_buf;

	if (likely(message_length < MXT_MSG_BUFF_SIZE)) {
		message = mxt_message_buf;
	}
	else {
		dev_err(&client->dev,
				"[TSP] Message length larger than 256 bytes "
				"not supported\n");
		return ;
	}

	mutex_lock(&mxt->msg_lock);

	for(try_cnt = 0; try_cnt < fail_count; try_cnt++) {
		error = mxt_read_no_delay_block(client, message_addr, message_length, message);
		if (error >= 0) {
			if (message[0] == MXT_RESERVED_T255) {
				break;
			}
			else if(message[0] == reportid) {
				*crc =  (uint32_t)message[2] | ((uint32_t)message[3] << 8) | ((uint32_t)message[4] << 16);
				//VPRINTK("message_buf_checksum = 0x%x, 0x%x, 0x%x\n", message[2], message[3], message[4]);
				break;
			}
			else {
				//break;
			}
		}
		else {
			break;
		}
	}

	mutex_unlock(&mxt->msg_lock);

	return;
}


static uint8_t type_to_report_id(struct mxt_data *mxt, uint8_t object_type, uint8_t instance)
{
	uint8_t report_id = 1;
	uint8_t report_id_found = 0;

	while((report_id <= mxt->report_id_count) && (report_id_found == 0)) {
		if((mxt->rid_map[report_id].object == object_type) &&
				(mxt->rid_map[report_id].instance == instance)) {
			report_id_found = 1;
			break;
		}
		else {
			report_id++;
		}
	}

	if (report_id_found) {
		return(report_id);
	}
	else {
		return 0xff;
	}
}

static int backup_to_nv(struct mxt_data *mxt)
{
	int ret;

	/* backs up settings to the non-volatile memory */
	/* = Command Processor T6 =
	 * BACKUPNV Field
	 * This field backs up settings to the non-volatile memory (NVM).
	 * Once the device has processed this command it generates a status
	 * message containing the new NVM checksum.
	 * Write value: 0x55
	 **/
	/* 2012_0504 :
	 *  - Configuration parameters maximum writes 10,000 */
	ret = mxt_write_byte(mxt->client,
						 MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
						 MXT_ADR_T6_BACKUPNV,
						 0x55);
	/* 2012_0523 :
	 *  - ATMEL : please, log the below code */
	mxt_debug_info(mxt, "%s()\n", __func__);
	/* 2017_0120 :
	 *  - ATMEL : 400ms needs to backup to nvram */
	msleep(400);
	return ret;
}

static void hw_reset_gpio(struct mxt_data* mxt)
{
#if 1// !defined(INCLUDE_TOUCH_MXT_1189T)
	unsigned RESET_GPIO = mxt->pdata->gpio_reset;

	int touch_scl = mxt->pdata->gpio_scl;
	int touch_sda = mxt->pdata->gpio_sda;
	int gpio_enable = 0;
	int gpio_ret = 0;


	if (touch_scl < 0 || touch_sda < 0)
		gpio_enable = -1;

	if (gpio_enable != -1) {

		gpio_ret = tcc_gpio_config(touch_scl, GPIO_FN0 | GPIO_OUTPUT | GPIO_LOW);
		if (gpio_ret < 0)
			printk(KERN_ERR "[%s:%d] scl -> gpio error value (0x%x) \r\n", __func__, __LINE__, gpio_ret);

		gpio_ret = tcc_gpio_config(touch_sda, GPIO_FN0 | GPIO_OUTPUT | GPIO_LOW);
		if (gpio_ret < 0)
			printk(KERN_ERR "[%s:%d] sda -> gpio error value (0x%x) \r\n", __func__, __LINE__, gpio_ret);
	}

	gpio_request(RESET_GPIO, NULL);
	gpio_direction_output(RESET_GPIO, 0);
	msleep(RESET_DURATION);

	if (gpio_enable != -1) {

			gpio_ret = tcc_gpio_config(touch_scl, GPIO_FN10 | GPIO_OUTPUT | GPIO_LOW);
			if (gpio_ret < 0)
				printk(KERN_ERR "[%s:%d] gpio -> scl error value (0x%x) \r\n", __func__, __LINE__, gpio_ret);

			gpio_ret = tcc_gpio_config(touch_sda, GPIO_FN10 | GPIO_OUTPUT | GPIO_LOW);
			if (gpio_ret < 0)
				printk(KERN_ERR "[%s:%d] gpio -> sda error value (0x%x) \r\n", __func__, __LINE__, gpio_ret);

	}

	gpio_direction_output(RESET_GPIO, 1);
	msleep(RESET_DELAY);
#else


#if defined(INCLUDE_CURVED_TI_SERDES)
	unsigned short addr = mxt->client->addr;
	printk(KERN_ERR "[%s:%d] TI Serdes Device(0x58), Register(0x20), Value(0x91, 0x99)!!!\r\n", __func__, __LINE__);
	mxt->client->addr = 0x58;
	mxt_write_onebyte_register(mxt->client, 0x20, 0x91);
	msleep(RESET_DURATION);
	mxt_write_onebyte_register(mxt->client, 0x20, 0x99);
	msleep(RESET_DELAY);
	mxt->client->addr = addr;

#else  // NORMAL 1189T WIDE
	unsigned short addr = mxt->client->addr;
	printk(KERN_ERR "[%s:%d] TI Serdes Device(0x58), Register(0x20)!!!\r\n", __func__, __LINE__);
	mxt->client->addr = 0x58;
	mxt_write_onebyte_register(mxt->client, 0x20, 0x01);
	msleep(RESET_DURATION);
	mxt_write_onebyte_register(mxt->client, 0x20, 0x09);
	msleep(RESET_DELAY);
	mxt->client->addr = addr;

#endif // NORMAL 1189T WIDE
#endif

}

void hw_reset_chip(struct mxt_data *mxt)
{
	int ret = 0;
	int loop = 0;

	VPRINTK("[%s]", __func__);

	for (loop = 0 ; loop < 10; loop++) {
		hw_reset_gpio(mxt);
		ret = mxt_identify(mxt->client, mxt);
		if (ret >= 0) {
			mxt_debug_info(mxt, "[TSP] mxt_identify Sucess");
			ret = mxt_read_object_table(mxt->client, mxt);
			if (ret >= 0) {
				mxt_debug_info(mxt, "[TSP] mxt_read_object_table Sucess");
				break;
			}
			else
				dev_err(&mxt->client->dev, "[TSP] mxt_read_object_table Fail");
		}
		else
			dev_err(&mxt->client->dev, "[TSP] mxt_identify Fail ");
	}

	if (loop < 10) {
		mxt_release_all_fingers(mxt);
		calibrate_chip(mxt);
		mxt->cal_check_flag = 1;
	}

	if(mxt->check_auto_cal_flag == AUTO_CAL_DISABLE)
		_disable_auto_cal(mxt, true);
}

/**
  * @ legolamp@cleinsoft
  * @ date 2014.11.20
  * Add Check jig detect func
  **/
static int check_jig_detection(struct mxt_data *mxt)
{
	int jig_det = 0;
	int gpio_jig_det = mxt->pdata->irq_jig_gpio;
	if (gpio_jig_det < 0) {
		return -EFAULT;
	}
#ifdef DEBUG_JIG
	mxt_debug_msg(mxt, "[TSP] TOUCH JIG DETECT : (%s)\n"
				  , gpio_get_value(gpio_jig_det) ? "Disconnected" : "Connected");
#endif

	if (mxt) {
		if (gpio_get_value(gpio_jig_det) == 0)
			jig_det = 1;
		else
			jig_det = 0;
	}
	else {
#ifdef DEBUG_JIG
		mxt_debug_msg(mxt, "[TSP] %s - mxt object is empty\n", __func__);
#endif
		return -EINVAL;
	}

	return jig_det;
}

/**
  * @ legolamp@cleinsoft
  * @ date 2014.11.20
  * Add gpio_func_enable input/output
  **/
static void gpio_func_enable(struct mxt_data* mxt, u8 enable)
{
	int gpio_i2s_scl = mxt->pdata->gpio_scl;
	int gpio_i2s_sda = mxt->pdata->gpio_sda;

	if (gpio_i2s_scl < 0) {
		return ;//-EFAULT;
	}

	if (gpio_i2s_sda < 0) {
		return ;//-EFAULT;
	}

	if(enable) {
		tcc_gpio_config(gpio_i2s_scl, GPIO_FN7 | GPIO_OUTPUT | GPIO_LOW);
		tcc_gpio_config(gpio_i2s_sda, GPIO_FN7 | GPIO_OUTPUT | GPIO_LOW);
	}
	else {
		tcc_gpio_config(gpio_i2s_scl, GPIO_FN0 | GPIO_INPUT );
		tcc_gpio_config(gpio_i2s_sda, GPIO_FN0 | GPIO_INPUT );
	}
	return ;
}

/**
  * @ legolamp@cleinsoft
  * @ date 2014.11.20
  * Add update jig detection func
  * return jig_deetection status
  **/
static int update_jig_detection(struct mxt_data *mxt)
{
	int jig_det = check_jig_detection(mxt);

	if(jig_det < 0)
		return false;

	if(jig_det != mxt->jig_detected) {
		if (jig_det == MXT_JIG_DETECT_DISABLE) {
			enable_irq(mxt->irq);
			gpio_func_enable(mxt, (jig_det ? 0 : 1));
		}
		else if (jig_det == MXT_JIG_DETECT_ENABLE) {
			disable_irq(mxt->irq);
			gpio_func_enable(mxt, (jig_det ? 0 : 1));
		}
	}
#ifdef DEBUG_JIG
	else
		mxt_debug_msg(mxt, "Status not change\n");
#endif

	mxt->jig_detected = jig_det;
	return jig_det;
}
int try_sw_reset_chip(struct mxt_data *mxt, u8 mode)
{
	int err;

	err = sw_reset_chip(mxt, mode);
	if (err) {
		hw_reset_chip(mxt);
		return 1;
	}

	return 0;
}

int sw_reset_chip(struct mxt_data *mxt, u8 mode)
{
	u8 data;
	u16 addr;
	int retry = I2C_RETRY_COUNT, err;
	struct i2c_client *client = mxt->client;

	mxt_debug_trace(mxt, "[TSP] Reset chip Reset mode (%d)", mode);

	/* Command Processor T6 (GEN_COMMANDPROCESSOR_T6)
	 * RESET Field
	 * This field forces a reset of the device if a nonzero value
	 * is written. If 0xA5 is written to this field, the device
	 * resets into bootloader mode.
	 * Write value: Nonzero (normal) or 0xA5 (bootloader) */

	if (mode == RESET_TO_NORMAL)
		data = 0x01;  /* non-zero value */
	else if (mode == RESET_TO_BOOTLOADER)
		data = 0xA5;
	else {
		dev_info(&client->dev, "[TSP] Invalid reset mode(%d)", mode);
		return -EINVAL;
	}

	if (MXT_FIRM_STABLE(mxt->firm_status_data) ||
			MXT_FIRM_UPGRADING_STABLE(mxt->firm_status_data))
		addr = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
			   MXT_ADR_T6_RESET;
	else {
		/* if the firmware is not stable, we don't know about
		 * the address from the objects. so, 0x11F is hard-coding */
		addr = 0x11F + MXT_ADR_T6_RESET;
		return -EIO;
	}

	do {
		err = mxt_write_byte(client, addr, data);
		if (err == 0)
			break;
	}
	while (retry--);
	return err;
}

/* 2012_0504 : ATMEL recommends that the s/w reset after h/w reset
 * doesn't need to be run */
static int mxt_force_reset(struct mxt_data *mxt)
{
	int cnt;

	if (debug >= DEBUG_MESSAGES)
		pr_info("[TSP] %s has been called!", __func__);

	hw_reset_chip(mxt);

	for (cnt = 10; cnt > 0; cnt--) {
		/* soft reset */
		if (sw_reset_chip(mxt, RESET_TO_NORMAL) == 0)
			break;
	}
	if (cnt == 0) {
		dev_err(&mxt->client->dev, "[TSP] mxt_force_reset failed!!!");
		return -1;
	}
	msleep(250);  /* 250ms */
	if (debug >= DEBUG_MESSAGES)
		pr_info("[TSP] %s success!!!", __func__);

	return 0;
}

#if defined(MXT_DRIVER_FILTER)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
	static int tcount[MXT_MAX_NUM_TOUCHES] = {0, };
	static u16 pre_x[MXT_MAX_NUM_TOUCHES][4] = {{0}, };
	static u16 pre_y[MXT_MAX_NUM_TOUCHES][4] = {{0}, };
	int coff[4] = {0, };
	int distance = 0;

	if (detect)
		tcount[id] = 0;

	pre_x[id][tcount[id] % 4] = *px;
	pre_y[id][tcount[id] % 4] = *py;

	if (tcount[id] > 3) {
		distance = abs(pre_x[id][(tcount[id] - 1) % 4] - *px)
				   + abs(pre_y[id][(tcount[id] - 1) % 4] - *py);

		coff[0] = (u8)(4 + distance / 5);
		if (coff[0] < 8) {
			coff[0] = max(4, coff[0]);
			coff[1] = min((10 - coff[0]), (coff[0] >> 1) + 1);
			coff[2] = min((10 - coff[0] - coff[1]), (coff[1] >> 1) + 1);
			coff[3] = 10 - coff[0] - coff[1] - coff[2];

			*px = (u16)((*px * (coff[0])
						 + pre_x[id][(tcount[id] - 1) % 4] * (coff[1])
						 + pre_x[id][(tcount[id] - 2) % 4] * (coff[2])
						 + pre_x[id][(tcount[id] - 3) % 4] * (coff[3])) / 10);
			*py = (u16)((*py * (coff[0])
						 + pre_y[id][(tcount[id] - 1) % 4] * (coff[1])
						 + pre_y[id][(tcount[id] - 2) % 4] * (coff[2])
						 + pre_y[id][(tcount[id] - 3) % 4] * (coff[3])) / 10);
		}
		else {
			*px = (u16)((*px * 4 + pre_x[id][(tcount[id] - 1) % 4]) / 5);
			*py = (u16)((*py * 4 + pre_y[id][(tcount[id] - 1) % 4]) / 5);
		}
	}
	tcount[id]++;
}
#endif  /* MXT_DRIVER_FILTER */

static void mxt_release_all_fingers(struct mxt_data *mxt)
{
	int id;

	mxt_debug_trace(mxt, "[TSP] %s\n", __func__);

	/* if there is any un-reported message, report it */
	for (id = 0 ; id < MXT_MAX_NUM_TOUCHES ; ++id) {
		if (mtouch_info[id].pressure == -1)
			continue;

		/* force release check*/
		mtouch_info[id].pressure = 0;

		/* ADD TRACKING_ID*/
		//REPORT_MT(id, mtouch_info[id].x, mtouch_info[id].y,
		//		mtouch_info[id].pressure, mtouch_info[id].size);
	if(touch_type == 5 || touch_type == 6) //1189T, 641T
	{
		/* 160615 - release touch */
		//input_report_key(mxt->input, BTN_TOUCH, 0);
		input_mt_slot(mxt->input, id);
		input_mt_report_slot_state(mxt->input, MT_TOOL_FINGER, false);
		/* END  */
//		input_mt_sync(mxt->input);
	}
	else
	{
		input_report_key(mxt->input, BTN_TOUCH, 0);
		input_mt_sync(mxt->input);
	}

#ifdef CONFIG_KERNEL_DEBUG_SEC
		mxt_debug_msg(mxt, "[TSP] r (%d,%d) %d\n",
					  mtouch_info[id].x,
					  mtouch_info[id].y, id);
#endif
		if (mtouch_info[id].pressure == 0)
			mtouch_info[id].pressure = -1;
	}

	input_sync(mxt->input);

}
#define mxt_release_all_keys(mxt) do {} while (0);

static inline void mxt_prerun_reset(struct mxt_data *mxt, bool r_irq)
{
#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_stop(mxt);
#endif
	mxt_check_touch_ic_timer_stop(mxt);
	mxt_supp_ops_stop(mxt);
	mxt_release_all_fingers(mxt);
	mxt_release_all_keys(mxt);
	if (r_irq)
		disable_irq(mxt->client->irq);
}

static inline void mxt_postrun_reset(struct mxt_data *mxt, bool r_irq)
{
	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* Add check_touch_ic_timer_start
	**/
	mxt_check_touch_ic_timer_start(mxt);
	if (r_irq)
		enable_irq(mxt->client->irq);
}

static int _check_recalibration(struct mxt_data *mxt)
{
	int rc = 0;
	int tch_ch = 0 , atch_ch = 0;
	dev_info(&mxt->client->dev, "%s\n", __func__);

	mutex_lock(&mxt->msg_lock);
	rc = check_touch_count(mxt, &tch_ch, &atch_ch);
	if (!rc) {
		if ((tch_ch != 0) || (atch_ch != 0)) {
			rc = -EPERM;
			dev_info(&mxt->client->dev, "tch_ch:%d, atch_ch:%d\n",
					 tch_ch, atch_ch);
			ts_100ms_timer_stop(mxt);
			mxt_check_touch_ic_timer_stop(mxt);
			mxt_supp_ops_stop(mxt);
			calibrate_chip(mxt);
		}
		else
			rc = 0;
	}
	else {
		maybe_good_count = 1;
		dev_err(&mxt->client->dev, "failed to read touch count\n");
		if (rc != -ETIME) {
			/* i2c io error */
			mxt_prerun_reset(mxt, false);
			hw_reset_chip(mxt);
			mxt_postrun_reset(mxt, false);
		}
		else {
			ts_100ms_timer_stop(mxt);
			calibrate_chip(mxt);
		}
	}
	mutex_unlock(&mxt->msg_lock);

	return rc;
}

static inline void _disable_auto_cal(struct mxt_data *mxt, int locked)
{
	if( NULL != mxt->object_table ) {
		u16 addr;

		addr = MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8);
		if (!locked)
			mutex_lock(&mxt->msg_lock);
		mxt_write_byte(mxt->client, addr + MXT_ADR_T8_TCHAUTOCAL, 0);
		if (!locked)
			mutex_unlock(&mxt->msg_lock);
	}
}

/* Returns object address in mXT chip, or zero if object is not found */
u16 get_object_address(uint8_t object_type,
					   uint8_t instance,
					   struct mxt_object *object_table,
					   int max_objs, int *object_link)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;

	struct mxt_object obj;

	/* check by object_link */
	/* 2012_0508 : Insert the below code. It's faster than while loop */
	if (object_link) {
		obj = object_table[object_link[object_type]];
		if (obj.type == object_type) {
			if (obj.instances >= instance) {
				address = obj.chip_addr + (obj.size + 1) * instance;
				return address;
			}
		}
	}

	while ((object_table_index < max_objs) && !address_found) {
		obj = object_table[object_table_index];
		if (obj.type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj.instances >= instance) {
				address = obj.chip_addr + (obj.size + 1) * instance;
			}
			else {
				return 0;
			}
		}
		object_table_index++;
	}

	return address;
}

/* Returns object size in mXT chip, or zero if object is not found */
u16 get_object_size(uint8_t object_type, struct mxt_object *object_table,
					int max_objs, int *object_link)
{
	uint8_t object_table_index = 0;
	struct mxt_object obj;

	/* check by object_link */
	/* 2012_0508 : Insert the below code. It's faster than while loop */
	if (object_link) {
		obj = object_table[object_link[object_type]];
		if (obj.type == object_type)
			return obj.size;
	}

	while (object_table_index < max_objs) {
		obj = object_table[object_table_index];
		if (obj.type == object_type)
			return obj.size;
		object_table_index++;
	}
	return 0;
}

u16 get_object_instance(uint8_t object_type, struct mxt_object *object_table,
						int max_objs, int *object_link)
{
	uint8_t object_table_index = 0;
	struct mxt_object obj;

	/* check by object_link */
	/* 2012_0508 : Insert the below code. It's faster than while loop */
	if (object_link) {
		obj = object_table[object_link[object_type]];
		if (obj.type == object_type)
			return obj.instances;
	}

	while (object_table_index < max_objs) {
		obj = object_table[object_table_index];
		if (obj.type == object_type)
			return obj.instances;
		object_table_index++;
	}
	return 0;
}


/*
* Reads one byte from given address from mXT chip (which requires
* writing the 16-bit address pointer first).
*/

int mxt_read_byte(struct i2c_client *client, u16 addr, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr = cpu_to_le16(addr);
	// struct mxt_data *mxt;

	// mxt = i2c_get_clientdata(client);

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = (u8 *) value;
	if  (i2c_transfer(adapter, msg, 2)) {
		// mxt->last_read_addr = addr;
		return 0;
	}
	else {
		/* In case the transfer failed, set last read addr to Invalid
		 * address, so that the next reads won't get confused.  */
		// mxt->last_read_addr = -1;
		return -EIO;
	}
}

/*
* Reads a block of bytes from given address from mXT chip. If we are
* reading from message window, and previous read was from message window,
* there's no need to write the address pointer: the mXT chip will
* automatically set the address pointer back to message window start.
*/

int mxt_read_no_delay_block(struct i2c_client *client,
							u16 addr,
							u16 length,
							u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	// struct mxt_data *mxt;
	int res = 0;

#if 0
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
				(addr == mxt->msg_proc_addr)) {
#if 1
			if  (i2c_master_recv(client, value, length) == length)
#else
			int count = i2c_master_recv(client, value, length);
			VPRINTK("%s i2c_master_recv count:%d length:%d\n", __func__, count, length);
			if  (count == length)
#endif
				return 0;
			else {
				dev_err(&client->dev, "1. [TSP] read failed\n");
				return -EIO;
			}
		}
		else {
			mxt->last_read_addr = addr;
		}
	}
#endif

	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	if  ((res = i2c_transfer(adapter, msg, 2))) {
		// mxt->last_read_addr = addr;

		return 0;
	}
	else {
		// mxt->last_read_addr = -1;
		return -EIO;
	}
}

/* PRODUCT_PRODUCT == yg_us, yg_ca,
   used deserialize device,
   this device needs to delay while mxt_read_block */

int mxt_read_block(struct i2c_client *client,
				   u16 addr,
				   u16 length,
				   u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	// struct mxt_data *mxt;
	int res = 0;

#if 0
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) && (addr == mxt->msg_proc_addr)) {

			if  (i2c_master_recv(client, value, length) == length) {
				return 0;
			}
			else {
				dev_err(&client->dev, "1. [TSP] read failed\n");
				return -EIO;
			}
		}
		else {
			mxt->last_read_addr = addr;
		}
	}
#endif

	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	if  ((res = i2c_transfer(adapter, msg, 2))) {
		// mxt->last_read_addr = addr;
#if defined(CONFIG_DISASSEMBLED_MONITOR)
		udelay(100);
#endif

		return 0;
	}
	else {
		// mxt->last_read_addr = -1;
#if defined(CONFIG_DISASSEMBLED_MONITOR)
		udelay(100);
#endif
		return -EIO;
	}
}

/* Reads a block of bytes from current address from mXT chip. */
int mxt_read_block_wo_addr(struct i2c_client *client,
						   u16 length,
						   u8 *value)
{
	if (i2c_master_recv(client, value, length) == length)
		return length;
	else {
		dev_err(&client->dev, "[TSP] read failed\n");
		return -EIO;
	}
}

/* Writes one byte to given address in mXT chip. */
int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	int count = 0;

	struct {
		__le16 le_addr;
		u8 data;
	} i2c_byte_transfer;

#if 0
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
#endif

	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;

	if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		return 0;
	else {
		VPRINTK("[%s] write byte error : %d\n", __func__, count );
		return -EIO;
	}
}

static int mxt_write_onebyte_register(struct i2c_client *client, u8 addr, u8 value)
{
	int ret = 0;

	struct {
		u8 le_addr;
		u8 data;
	} i2c_byte_transfer;

#if 0
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
#endif

	i2c_byte_transfer.le_addr = addr;
	i2c_byte_transfer.data = value;

	ret = i2c_master_send(client, (u8 *) &i2c_byte_transfer, 2);
	if (ret == 2)
		return 0;
	else {
		VPRINTK("[%s] write byte error : %d\n", __func__, ret );
		return -EIO;
	}
}


/* Writes a block of bytes (max 256) to given address in mXT chip. */
int mxt_write_block(struct i2c_client *client,
					u16 addr,
					u16 length,
					u8 *value)
{
	int i;
	struct {
		__le16	le_addr;
		u8	data[256];

	} i2c_block_transfer;


	// struct mxt_data *mxt;

	if (length > 256)
		return -EINVAL;
#if 0
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
#endif

	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;

	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);

#if defined(CONFIG_DISASSEMBLED_MONITOR)
	udelay(100);
#endif

	if (i == (length + 2))
		return length;
	else {
		VPRINTK("[%s] write block error : %d \n!!!\n", __func__, i );
		return -EIO;
	}
}

/* TODO: make all other access block until the read has been done? Otherwise
an arriving message for example could set the ap to message window, and then
the read would be done from wrong address! */

/* Writes the address pointer (to set up following reads). */
int mxt_write_ap(struct i2c_client *client, u16 ap)
{

	__le16	le_ap = cpu_to_le16(ap);

#if 0
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	mxt_debug_info(mxt, "[TSP] Address pointer set to %d\n", ap);
#endif

	if (i2c_master_send(client, (u8 *) &le_ap, 2) == 2)
		return 0;
	else
		return -EIO;
}

static u32 mxt_CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u16 data_word;

	data_word = (u16) ((u16) (byte2 << 8u) | byte1);
	result = ((crc << 1u) ^ (u32) data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

int calculate_infoblock_crc(u32 *crc_result, struct mxt_data *mxt)
{
	u32 crc = 0;
	u16 crc_area_size;
	u8 *mem;
	int i;

	int error;
	struct i2c_client *client;

	client = mxt->client;

	crc_area_size = MXT_ID_BLOCK_SIZE +
					mxt->device_info.num_objs * MXT_OBJECT_TABLE_ELEMENT_SIZE;

	mem = kmalloc(crc_area_size, GFP_KERNEL);

	if (mem == NULL) {
		dev_err(&client->dev, "[TSP] Error allocating memory\n");
		return -ENOMEM;
	}

	error = mxt_read_block(client, 0, crc_area_size, mem);
	if (error < 0) {
		kfree(mem);
		return error;
	}

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = mxt_CRC_24(crc, *(mem + i), *(mem + i + 1));

	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = mxt_CRC_24(crc, *(mem + i), 0);

	kfree(mem);

	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);
	return 1;
}

int calculate_drv_config_crc(uint32_t *crc_result, struct mxt_data *mxt)
{
	u32 crc = 0;
	u16 crc_area_size;
	u8 *mem;
	int error;
	struct i2c_client *client;
	u16 last_obj = 0, i;
	static u16 off_addr = 0;

	client = mxt->client;

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		if(mxt->object_table[i].chip_addr > mxt->object_table[last_obj].chip_addr) {
			last_obj = i;
		}

		if(mxt->object_table[i].type == 38) {
			off_addr = mxt->object_table[i].chip_addr + mxt->object_table[i].instances * mxt->object_table[i].size;
		}
	}

	crc_area_size = mxt->object_table[last_obj].chip_addr + mxt->object_table[last_obj].instances * mxt->object_table[last_obj].size;

	mem = kmalloc(crc_area_size, GFP_KERNEL);

	if (mem == NULL) {
		dev_err(&client->dev, "[TSP] Error allocating memory\n");
		return -ENOMEM;
	}
	memset(mem, 0, crc_area_size);

	error =	mxt_config_settings(mxt, mem, TOMEM, touch_type);
	if (error < 0) {
		kfree(mem);
		return error;
	}

	for (i = off_addr; i < (crc_area_size - 1); i = i + 2)
		crc = mxt_CRC_24(crc, *(mem + i), *(mem + i + 1));

	/* If uneven size, pad with zero */
	if ((crc_area_size - off_addr) & 0x0001)
		crc = mxt_CRC_24(crc, *(mem + i), 0);

	kfree(mem);

	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);
	return 1;
}

static void process_T9_message(u8 *message, struct mxt_data *mxt)
{
	struct input_dev *input;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 report_id;
	u8 touch_id;  /* to identify each touches. starts from 0 to 15 */
	u8 pressed_or_released = 0;
	static int prev_touch_id = -1;
	int i;
	u16 chkpress = 0;
	u8 touch_message_flag = 0;

	input = mxt->input;
	status = message[MXT_MSG_T9_STATUS];

	report_id = message[0];
	if (likely(mxt->report_id_count > 0))
		touch_id = report_id - mxt->rid_map[report_id].first_rid;
	else
		touch_id = report_id - 2;

	if (unlikely(touch_id >= MXT_MAX_NUM_TOUCHES)) {
		//dev_err(&mxt->client->dev,"[TSP] Invalid touch_id (toud_id=%d)",touch_id);
		return;
	}

	/* calculate positions of x, y */
	/* TOUCH_MULTITOUCHSCREEN_T9
	 * [2] XPOSMSB		X position MSByte
	 * [3] YPOSMSB		Y position MSByte
	 * [4] XYPOSLSB		X position lsbits | Y position lsbits
	 * */
	xpos = message[2];
	xpos = xpos << 4;
	xpos = xpos | (message[4] >> 4);
#ifndef INCLUDE_LCD_RESOLUTION_1280_720
	xpos >>= 2;
#endif

	ypos = message[3];
	ypos = ypos << 4;
	ypos = ypos | (message[4] & 0x0F);
	ypos >>= 2;

	/* 2012_0612
	 * ATMEL suggested amplitude limitation
	 */
	mxt_debug_trace(mxt, "Touch amplitude value %d\n",
					message[MXT_MSG_T9_TCHAMPLITUDE]);
	if (message[MXT_MSG_T9_TCHAMPLITUDE] >= AMPLITUDE_LIMIT) {
		dev_err(&mxt->client->dev, "amplitude is over:%d/%d\n",
				message[MXT_MSG_T9_TCHAMPLITUDE],
				AMPLITUDE_LIMIT);
		return;
	}

	/**
	 * @author sjpark@cleinsoft
	 * @date 2013/10/07
	 * apply patch code to fix the release message loss.
	 **/
	if (status & MXT_MSGB_T9_RELEASE) {   /* case 2: released */
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;
	}
	else if (status & MXT_MSGB_T9_DETECT) {   /* case 1: detected */
		touch_message_flag = 1;
		mtouch_info[touch_id].pressure =
			message[MXT_MSG_T9_TCHAMPLITUDE];
		mtouch_info[touch_id].x = (int16_t)xpos;
		mtouch_info[touch_id].y = (int16_t)ypos;

		if (status & MXT_MSGB_T9_PRESS) {
			pressed_or_released = 1;  /* pressed */
#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(1, touch_id,
								&mtouch_info[touch_id].x,
								&mtouch_info[touch_id].y);
#endif
		}
		else if (status & MXT_MSGB_T9_MOVE) {
#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(0, touch_id,
								&mtouch_info[touch_id].x,
								&mtouch_info[touch_id].y);
#endif
		}
	}
	else if (status & MXT_MSGB_T9_SUPPRESS) {     /* case 3: suppressed */
		mxt_debug_info(mxt, "[TSP] Palm(T9) Suppressed !!!\n");
		facesup_message_flag_T9 = 1;
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;
	}
	else {
		dev_err(&mxt->client->dev,
				"[TSP] Unknown status (0x%x)", status);

		if (facesup_message_flag_T9 == 1) {
			facesup_message_flag_T9 = 0;
			mxt_debug_info(mxt, "[TSP] Palm(T92) Suppressed !!!\n");
		}
	}
	/* only get size , id would use TRACKING_ID*/
	/* TCHAREA Field
	 *  Reports the size of the touch area in terms of the number of
	 *  channels that are covered by the touch.
	 *  A reported size of zero indicates that the reported touch is
	 *  a stylus from the Stylus T47 object linked to this Multiple
	 *  Touch Touchscreen T9.
	 *  A reported size of 1 or greater indicates that the reported
	 *  touches are from a finger touch (or a large object, such as
	 *  a palm or a face). For example, the area ... */
	mtouch_info[touch_id].size = message[MXT_MSG_T9_TCHAREA];

	if (prev_touch_id >= touch_id || pressed_or_released) {
		for (i = 0; i < MXT_MAX_NUM_TOUCHES  ; ++i) {
			if (mtouch_info[i].pressure == -1)
				continue;

			mxt_debug_trace(mxt, "[TSP]id:%d, x:%d, y:%d\n", i,
							mtouch_info[i].x,
							mtouch_info[i].y);
			if (mtouch_info[i].pressure == 0) {
				mtouch_info[i].size = 0;
				input_mt_slot(mxt->input, i);
				input_mt_report_slot_state(mxt->input, MT_TOOL_FINGER, false);
				mtouch_info[i].pressure = -1;
				//printk("[%s] released  :  size - %d / pressure - %d / touch - %d \n", __func__ , mtouch_info[i].size , mtouch_info[i].pressure, i);
			}
			else {
				//printk("[%s] prossed  :  size - %d / pressure - %d / touch - %d\n", __func__ , mtouch_info[i].size , mtouch_info[i].pressure , i);
				input_mt_slot(mxt->input, i);
				input_mt_report_slot_state(mxt->input, MT_TOOL_FINGER, true);
				input_report_abs(mxt->input, ABS_MT_POSITION_X,	mtouch_info[i].x);
				input_report_abs(mxt->input, ABS_MT_POSITION_Y,	mtouch_info[i].y);
				chkpress++;
			}
			//	printk("por X : %d / por Y : %d \n",mtouch_info[i].x, mtouch_info[i].y);
		}
		input_sync(input);  /* TO_CHK: correct position? */
	}
	prev_touch_id = touch_id;

#ifdef CONFIG_KERNEL_DEBUG_SEC
	/* simple touch log */
	if ((debug >= DEBUG_MESSAGES) && (debug < DEBUG_TRACE)) {
		if (status & MXT_MSGB_T9_SUPPRESS) {
			pr_info("\n[cliensoft mxt336s] Suppress\n");
			pr_info("\n[cliensoft mxt336s] Release |x:%3d, y:%3d| No.%d |\n",
					xpos, ypos, touch_id);
		}
		else {
			if (status & MXT_MSGB_T9_PRESS) {
				pr_info("\n[cliensoft mxt336s] Press   |x:%3d, y:%3d| No.%d | amp=%d | "
						"Area=%d |\n", xpos, ypos, touch_id,
						message[MXT_MSG_T9_TCHAMPLITUDE],
						message[MXT_MSG_T9_TCHAREA]);
			}
			else if (status & MXT_MSGB_T9_RELEASE) {
				pr_info("\n[cliensoft mxt336s] Release |x:%3d, y:%3d| No.%d |\n",
						xpos, ypos, touch_id);
			}
		}
	}

	/* detail touch log */
	if (debug >= DEBUG_TRACE) {
		char msg[64] = { 0};
		char info[64] = { 0};
		if (status & MXT_MSGB_T9_SUPPRESS) {
			strcpy(msg, "[cliensoft mxt336s] Suppress");
		}
		else {
			if (status & MXT_MSGB_T9_DETECT) {
				strcpy(msg, "[cliensoft mxt336s]");
				if (status & MXT_MSGB_T9_PRESS)
					strcat(msg, " Press  ");
				if (status & MXT_MSGB_T9_MOVE)
					strcat(msg, " Move   ");
				if (status & MXT_MSGB_T9_AMP)
					strcat(msg, " AMP    ");
				if (status & MXT_MSGB_T9_VECTOR)
					strcat(msg, " VECTOR  ");
				strcat(msg, " -> ");
			}
			else if (status & MXT_MSGB_T9_RELEASE) {
				strcpy(msg, "[cliensoft mxt336s] Release -> ");
			}
			else {
				strcpy(msg, "[!] Unknown status -> ");
			}
		}
		sprintf(info, "| id=%d | x:%3d, y:%3d | amp=%2d | size=%2d |",
				touch_id, xpos, ypos, message[MXT_MSG_T9_TCHAMPLITUDE],
				message[MXT_MSG_T9_TCHAREA]);

		strcat(msg, info);
		pr_info("\n %s \n", msg);
	}
#endif

	if (mxt->cal_check_flag == 1) {
		/* If chip has recently calibrated and there are any touch or
		 * face suppression messages, we must confirm if
		 * the calibration is good. */
		if (touch_message_flag) {
			/* 2012_0517
			 *  - ATMEL : no need */
			/*if (mxt->ts_100ms_tmr.timer_flag == MXT_DISABLE)
				ts_100ms_timer_enable(mxt);*/
			if (mxt->mxt_time_point == 0)
				mxt->mxt_time_point = jiffies;
			check_chip_calibration(mxt);
		}
	}
	return;
}

#define MXT_T100_DETECT 0x80
//#define MXT_T100_TYPE_MASK 0x80
//#define MXT_T100_TYPE_STYLUS 0x80

static void process_T100_message(u8 *message, struct mxt_data *mxt)
{
	struct input_dev *input;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 report_id;
	u8 touch_id;  /* to identify each touches. starts from 0 to 15 */
	u8 pressed_or_released = 0;
	static int prev_touch_id = -1;
	int i;
	u16 chkpress = 0;
	u8 touch_message_flag = 0;

	input = mxt->input;
	status = message[0x01];

	report_id = message[0];

	if(report_id == mxt->rid_map[report_id].first_rid) {
		if(mxt->cal_check_flag == 1) {
			if((message[1] & 0x80) == 0x80) {
				if(mxt->mxt_time_point == 0)
					mxt->mxt_time_point = jiffies;
				check_chip_calibration_t100(message, mxt);
			}
		}


		if (message[1] == 0x40) { // suppression
			palm_info.palm_release_flag = false;
			mxt_debug_info(mxt, "[TSP] Palm Suppressed !!! \n");
			/* 0506 LBK */
			if (palm_info.facesup_message_flag
					&& mxt->ts_100ms_tmr.timer_flag)
				return;
			check_chip_palm(mxt);

			if (palm_info.facesup_message_flag) {
				mxt_check_touch_ic_timer_stop(mxt);
				/* 100ms timer Enable */
				ts_100ms_timer_enable(mxt);
				mxt_debug_info(mxt,
							   "[TSP] Palm Timer start !!!\n");
			}
		}
		else if(message[1] == 0x00) {  // suppression release
			mxt_debug_info(mxt, "[TSP] Palm Released !!!\n");
			palm_info.palm_release_flag = true;
			palm_info.facesup_message_flag = 0;

			/* 100ms timer disable */
			ts_100ms_timer_clear(mxt);
		}

		return;
	}

	touch_id = report_id - mxt->rid_map[report_id].first_rid - 2;

	mxt_debug_trace(mxt,"report_id:%u, touch_id:%u\n",report_id, touch_id);

	if (unlikely(touch_id >= MXT_MAX_NUM_TOUCHES)) {
		//dev_err(&mxt->client->dev,"[TSP] Invalid touch_id (toud_id=%d)",touch_id);
		return;
	}



	/* calculate positions of x, y */
	/* TOUCH_MULTITOUCHSCREEN_T9
	 * [2] XPOSMSB		X position MSByte
	 * [3] YPOSMSB		Y position MSByte
	 * [4] XYPOSLSB		X position lsbits | Y position lsbits
	 * */

	xpos = (message[3] << 8) | message[2];
	ypos = (message[5] << 8) | message[4];


	/* 2012_0612
	 * ATMEL suggested amplitude limitation
	 */

	/* only get size , id would use TRACKING_ID*/
	/* TCHAREA Field
	 *  Reports the size of the touch area in terms of the number of
	 *  channels that are covered by the touch.
	 *  A reported size of zero indicates that the reported touch is
	 *  a stylus from the Stylus T47 object linked to this Multiple
	 *  Touch Touchscreen T9.
	 *  A reported size of 1 or greater indicates that the reported
	 *  touches are from a finger touch (or a large object, such as
	 *  a palm or a face). For example, the area ... */
	mtouch_info[touch_id].size = 0x3;

#if 0
	dev_info(&mxt->client->dev,
			 "[%u] status:%02X x:%u y:%u \n",
			 message[0], //touch_id,
			 status,
			 xpos, ypos
			);
#endif

	input_mt_slot(mxt->input, 0);


	if ((status & 0x05) == 0x05) {   /* case 2: released */
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;

		mxt_debug_trace(mxt,"status:%02X  released!!! \n", status);

	}
	else if (status & 0x80) {   /* case 1: detected */
		touch_message_flag = 1;
		mtouch_info[touch_id].pressure =
			//message[MXT_MSG_T9_TCHAMPLITUDE];
			10;
		mtouch_info[touch_id].x = (int16_t)xpos;
		mtouch_info[touch_id].y = (int16_t)ypos;

		mxt_debug_trace(mxt,"status:%02X  detected!!! \n", status);

		if ((status & 0x04) == 0x04) {
			pressed_or_released = 1;  /* pressed */

			mxt_debug_trace(mxt,"status:%02X  pressed!!! \n", status);
#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(1, touch_id,
								&mtouch_info[touch_id].x,
								&mtouch_info[touch_id].y);
#endif
		}
		else if ((status & 0x01) == 0x01) {

			mxt_debug_trace(mxt,"status:%02X  moved!!! \n", status);
#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(0, touch_id,
								&mtouch_info[touch_id].x,
								&mtouch_info[touch_id].y);
#endif
		}
	}
	else if ((status & 0x03) == 0x03 ) {  /*case 3 : suppressed */
		mxt_debug_info(mxt, "[TSP] Palm(T9) Suppressed !@#$!@#%!@^!@!!!\n");
		facesup_message_flag_T9 = 1;
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;
	}

#if 1
	if (prev_touch_id >= touch_id || pressed_or_released) {
		for (i = 0; i < MXT_MAX_NUM_TOUCHES  ; ++i) {
			if (mtouch_info[i].pressure == -1)
				continue;

			mxt_debug_trace(mxt, "[TSP]id:%d, x:%d, y:%d\n", i,
							mtouch_info[i].x,
							mtouch_info[i].y);
			if (mtouch_info[i].pressure == 0) {
				mtouch_info[i].size = 0;
				input_mt_slot(mxt->input, i);
				input_mt_report_slot_state(mxt->input, MT_TOOL_FINGER, false);
				mtouch_info[i].pressure = -1;
				//printk("[%s] released  :  size - %d / pressure - %d / touch - %d \n", __func__ , mtouch_info[i].size , mtouch_info[i].pressure, i);
			}
			else {
				//printk("[%s] prossed  :  size - %d / pressure - %d / touch - %d\n", __func__ , mtouch_info[i].size , mtouch_info[i].pressure , i);
				input_mt_slot(mxt->input, i);
				input_mt_report_slot_state(mxt->input, MT_TOOL_FINGER, true);
				input_report_abs(mxt->input, ABS_MT_POSITION_X,	mtouch_info[i].x);
				input_report_abs(mxt->input, ABS_MT_POSITION_Y,	mtouch_info[i].y);
				chkpress++;
			}
			//	printk("por X : %d / por Y : %d \n",mtouch_info[i].x, mtouch_info[i].y);
		}
		input_sync(input);  /* TO_CHK: correct position? */
	}
	prev_touch_id = touch_id;

#endif


#ifdef CONFIG_KERNEL_DEBUG_SEC

        /* simple touch log */
        if ((debug >= DEBUG_MESSAGES) && (debug < DEBUG_TRACE)) {
                if ((status & MXT_MSGB_T100_SUPPRESS)==MXT_MSGB_T100_SUPPRESS) {
                        pr_info("\n[cliensoft mxt336s] Suppress\n");
                        pr_info("\n[cliensoft mxt336s] Release |x:%3d, y:%3d| No.%d |\n",
                                        xpos, ypos, touch_id);
                }
                else {
			if((status & MXT_MSGB_T100_RELEASE)==MXT_MSGB_T100_RELEASE) {
                                pr_info("\n[cliensoft mxt336s] Release |x:%3d, y:%3d| No.%d |\n",
                                                xpos, ypos, touch_id);
                        }
                        else if ((status & MXT_MSGB_T100_PRESS)==MXT_MSGB_T100_PRESS) {
                                pr_info("\n[cliensoft mxt336s] Press   |x:%3d, y:%3d| No.%d |\n"
                                                , xpos, ypos, touch_id);
                        }
                }
        }

        /* detail touch log */
        if (debug >= DEBUG_TRACE) {
                char msg[64] = { 0};
                char info[64] = { 0};
                if ((status & MXT_MSGB_T100_SUPPRESS)==MXT_MSGB_T100_SUPPRESS) {
                        strcpy(msg, "[cliensoft mxt336s] Suppress");
                }
                else {
                        if ((status & MXT_MSGB_T100_DETECT)==MXT_MSGB_T100_DETECT) {
                                strcpy(msg, "[cliensoft mxt336s]");
                                if ((status & MXT_MSGB_T100_PRESS)==MXT_MSGB_T100_PRESS)
                                        strcat(msg, " Press  ");
                                if ((status & MXT_MSGB_T100_MOVE)==MXT_MSGB_T100_MOVE)
                                        strcat(msg, " Move   ");
//                                if (status & MXT_MSGB_T100_AMP)
//                                        strcat(msg, " AMP    ");
//                                if (status & MXT_MSGB_T100_VECTOR)
//                                        strcat(msg, " VECTOR  ");
                                strcat(msg, " -> ");
                        }
                        else if ((status & MXT_MSGB_T100_RELEASE)==MXT_MSGB_T100_RELEASE) {
                                strcpy(msg, "[cliensoft mxt336s] Release -> ");
                        }
                        else {
                                strcpy(msg, "[!] Unknown status -> ");
                        }
                }
                sprintf(info, "| id=%d | x:%3d, y:%3d ",
                                touch_id, xpos, ypos);

                strcat(msg, info);
                pr_info("\n %s \n", msg);
        }
#endif

	return;
}

#if defined(INCLUDE_LCD_TOUCHKEY)

static const struct mxt_tsp_key tsp_keys[] = {
	{ TOUCH_9KEY_VOLUMEDOWN, KEY9_VOLUMEDOWN},
	{ TOUCH_9KEY_VOLUMEUP, KEY9_VOLUMEUP },
	{ TOUCH_9KEY_NAVI, KEY9_NAVI },
	{ TOUCH_9KEY_MAP, KEY9_MAP },
	{ TOUCH_9KEY_RADIO, KEY9_RADIO },
	{ TOUCH_9KEY_MEDIA, KEY9_MEDIA },
	{ TOUCH_9KEY_CUSTOM, KEY9_CUSTOM },
	{ TOUCH_9KEY_SETUP, KEY9_SETUP },
	{ TOUCH_9KEY_POWER, KEY9_POWER },
	{ TOUCH_KEY_NULL, KEY_INVALID }
};
static unsigned int mxt_tsp_get_keycode(u16 id)
{
	int i;
	for (i = 0; tsp_keys[i].id != TOUCH_KEY_NULL; i++) {
		if (tsp_keys[i].id == id) {
			return tsp_keys[i].keycode;
		}
	}
	return KEY_INVALID;
}
static void process_T15_message(u8 *message, struct mxt_data *mxt)
{
#if 1  //20160923 mhjung touchkey 
	struct	input_dev *input = mxt->input_key;
	static u16  tsp_keyvalue = false;
	unsigned int tsp_keycode = KEY_INVALID;
	int pressed;

	mxt_debug_trace(mxt, "%s\n", __func__);
	if (message[MXT_MSG_T15_STATUS] & MXT_MSGB_T15_DETECT) {
		if (tsp_keyvalue != TOUCH_KEY_NULL) {
			input_report_key(input,
							 mxt_tsp_get_keycode(tsp_keyvalue), KEY_RELEASE);
			input_sync(input);
			mxt_debug_info(mxt,
						   "[TSP_KEY] TOUCH KEY FORCE RELEASE  : 0x%x\n", tsp_keyvalue);
			tsp_keyvalue = TOUCH_KEY_NULL;
		}
		//tsp_keyvalue = message[MXT_MSG_T15_KEYSTATE];
		tsp_keyvalue = message[MXT_MSG_T15_KEYSTATE] | message[MXT_MSG_T15_KEYSTATE + 1] << 8;
		tsp_keycode = mxt_tsp_get_keycode(tsp_keyvalue);

		pressed = 1;
		if (tsp_keycode == KEY_INVALID) {
			dev_info(&mxt->client->dev,
					 "[TSP_KEY] Unknown key (pressed) : 0x%x\n", message[MXT_MSG_T15_KEYSTATE]);
			tsp_keyvalue = TOUCH_KEY_NULL;
		}
		mxt_debug_info(mxt,
					   "[TSP_KEY] TOUCH KEY PRESS : 0x%x\n", message[MXT_MSG_T15_KEYSTATE]);
	}
	else {   /* TOUCH KEY RELEASE */
		tsp_keycode = mxt_tsp_get_keycode(tsp_keyvalue);
		pressed = 0;
		if (tsp_keycode == KEY_INVALID) {
			dev_info(&mxt->client->dev,
					 "[TSP_KEY] Unknown key (released) : 0x%x\n", message[MXT_MSG_T15_KEYSTATE]);
		}
		mxt_debug_info(mxt,
					   "[TSP_KEY] TOUCH KEY RELEASE : 0x%x\n", tsp_keyvalue);
		tsp_keyvalue = TOUCH_KEY_NULL;
	}
	if (tsp_keycode != KEY_INVALID) {
		input_report_key(input, tsp_keycode, pressed);
		input_sync(input);
	}
	return;
#else
	struct	input_dev *input;
	static u8  tsp_keyvalue = false;
	// static u8  tsp_keystatus = false;
	input = mxt->input;

	// tsp_keystatus = message[MXT_MSG_T15_STATUS];

	/* touch key is 7 key array,
	/* TOUCH KEY PRESS */
	if (message[MXT_MSG_T15_STATUS] & MXT_MSGB_T15_DETECT) {
		/* defence code, if there is any Pressed key,
		 * force release!! */
		if (tsp_keyvalue != TOUCH_KEY_NULL) {
			switch (tsp_keyvalue) {
				case TOUCH_7KEY_POWER:
					input_report_key(mxt->input,
									 KEY7_POWER, KEY_RELEASE);
					break;
				case TOUCH_7KEY_HOME:
					input_report_key(mxt->input,
									 KEY7_HOME, KEY_RELEASE);
					break;
				case TOUCH_7KEY_VOLUMEDOWN:
					input_report_key(mxt->input,
									 KEY7_VOLUMEDOWN, KEY_RELEASE);
					break;
				case TOUCH_7KEY_VOLUMEUP:
					input_report_key(mxt->input,
									 KEY7_VOLUMEUP,	KEY_RELEASE);
					break;
				case TOUCH_7KEY_SEEKDOWN:
					input_report_key(mxt->input,
									 KEY7_SEEKDOWN,	KEY_RELEASE);
					break;
				case TOUCH_7KEY_TRACKUP:
					input_report_key(mxt->input,
									 KEY7_TRACKUP,	KEY_RELEASE);
					break;
				default:
#if 0
					dev_info(&mxt->client->dev,
							 "[TSP_KEY] default1 :  0x%x\n", tsp_keyvalue);
#endif
					break;
			}
//#ifdef CONFIG_KERNEL_DEBUG_SEC
#if 0
			dev_info(&mxt->client->dev,
					 "[TSP_KEY] TOUCH KEY FORCE RELEASE  : 0x%x\n", tsp_keyvalue);
#endif
			tsp_keyvalue = TOUCH_KEY_NULL;
		}

		switch (message[MXT_MSG_T15_KEYSTATE]) {
			case 0x01:
				input_report_key(mxt->input,
								 KEY7_POWER, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_POWER;
				break;
			case 0x02:
				input_report_key(mxt->input,
								 KEY7_HOME, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_HOME;
				break;
			case 0x04:
				input_report_key(mxt->input,
								 KEY7_VOLUMEDOWN, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_VOLUMEDOWN;
				break;
			case 0x08:
				input_report_key(mxt->input,
								 KEY7_VOLUMEUP, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_VOLUMEUP;
				break;
			case 0x10:
				input_report_key(mxt->input,
								 KEY7_SEEKDOWN, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_SEEKDOWN;
				break;
			case 0x20:
				input_report_key(mxt->input,
								 KEY7_TRACKUP, KEY_PRESS);
				tsp_keyvalue = TOUCH_7KEY_TRACKUP;
				break;
			default:
				dev_info(&mxt->client->dev,
						 "[TSP_KEY] default2 : 0x%x\n", message[MXT_MSG_T15_KEYSTATE]);
				break;
		}
#if 1
		dev_info(&mxt->client->dev,
				 "[TSP_KEY] TOUCH KEY PRESS : 0x%x\n", message[MXT_MSG_T15_KEYSTATE]);
#endif

	}
	else {   /* TOUCH KEY RELEASE */
		switch (tsp_keyvalue) {
			case TOUCH_7KEY_POWER:
				input_report_key(mxt->input,
								 KEY7_POWER, KEY_RELEASE);
				break;
			case TOUCH_7KEY_HOME:
				input_report_key(mxt->input,
								 KEY7_HOME, KEY_RELEASE);
				break;
			case TOUCH_7KEY_VOLUMEDOWN:
				input_report_key(mxt->input,
								 KEY7_VOLUMEDOWN, KEY_RELEASE);
				break;
			case TOUCH_7KEY_VOLUMEUP:
				input_report_key(mxt->input,
								 KEY7_VOLUMEUP, KEY_RELEASE);
				break;
			case TOUCH_7KEY_SEEKDOWN:
				input_report_key(mxt->input,
								 KEY7_SEEKDOWN, KEY_RELEASE);
				break;
			case TOUCH_7KEY_TRACKUP:
				input_report_key(mxt->input,
								 KEY7_TRACKUP, KEY_RELEASE);
				break;
			default:
				dev_info(&mxt->client->dev,
						 "[TSP_KEY] default3 : 0x%x\n", tsp_keyvalue);
				break;
		}
//#ifdef CONFIG_KERNEL_DEBUG_SEC
#if 1
		dev_info(&mxt->client->dev,
				 "[TSP_KEY] TOUCH KEY RELEASE : 0x%x\n", tsp_keyvalue);
#endif

		tsp_keyvalue = TOUCH_KEY_NULL;
	}
	return;
#endif
}
#else
#define process_T15_message(msg, mxt) do {} while (0)
#endif

static void process_T42_message(u8 *message, struct mxt_data *mxt)
{
	struct	input_dev *input;
	u8  status = false;

	input = mxt->input;
	status = message[MXT_MSG_T42_STATUS];

	/* Message Data for PROCI_TOUCHSUPPRESSION_T42 */
	/* Byte Field  Bit7   Bit6  Bit5    Bit4 Bit3   Bit2 Bit1    Bit0 */
	/* 1	STATUS <---- Reserved ------------------------->     TCHSUP */
	/* TCHSUP: Indicates that touch suppression has been applied to
	 * the linked Multiple Touch Touchscreen T9 object. This field
	 * is set to 1 if suppression is active, and 0 otherwise. */

	if (message[MXT_MSG_T42_STATUS] == MXT_MSGB_T42_TCHSUP) {
		palm_info.palm_release_flag = false;
		mxt_debug_info(mxt, "[TSP] Palm(T42) Suppressed !!! \n");
		/* 0506 LBK */
		if (palm_info.facesup_message_flag
				&& mxt->ts_100ms_tmr.timer_flag)
			return;
		check_chip_palm(mxt);

		if (palm_info.facesup_message_flag) {
			mxt_check_touch_ic_timer_stop(mxt);
			/* 100ms timer Enable */
			ts_100ms_timer_enable(mxt);
			mxt_debug_info(mxt,
						   "[TSP] Palm(T42) Timer start !!!\n");
		}
	}
	else {
		mxt_debug_info(mxt, "[TSP] Palm(T42) Released !!!\n");
		palm_info.palm_release_flag = true;
		palm_info.facesup_message_flag = 0;

		/* 100ms timer disable */
		ts_100ms_timer_clear(mxt);
	}
	return;
}

static void process_t62_message(u8 *message, struct mxt_data *mxt)
{
	u8 state;
	int f_version;

	state = message[4];
	f_version = (mxt->device_info.major * 10) + (mxt->device_info.minor);

	if (state == 0x05)
		printk(KERN_INFO "[TSP] NOISESUPPRESSION_T62\n");
}

static void process_t6_message(u8 *message, struct mxt_data *mxt)
{
	u8 status;
	struct i2c_client *client;
	status = message[1];
	client = mxt->client;

	if (status & MXT_MSGB_T6_COMSERR) {
		dev_err(&client->dev,
				"[TSP] maXTouch checksum error\n");
	}
	if (status & MXT_MSGB_T6_CFGERR) {
cfgerr:
		dev_err(&client->dev, "[TSP] maXTouch "
				"configuration error\n");
		mxt_prerun_reset(mxt, false);
		/* 2012_0510 : ATMEL */
		/*  - keep the below flow */
		if (!try_sw_reset_chip(mxt, RESET_TO_NORMAL))
			msleep(MXT_SW_RESET_TIME);
		/* re-configurate */
		mxt_config_settings(mxt, NULL, TODEV, touch_type);
		/* backup to nv memory */
		backup_to_nv(mxt);
		/* forces a reset of the chipset */
		if (!try_sw_reset_chip(mxt, RESET_TO_NORMAL))
			msleep(MXT_SW_RESET_TIME);

		mxt_postrun_reset(mxt, false);
	}
	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.23
	* T6_message of calibration case is changed
	* "calibration in progress" -> "calibration is complete"
	**/
	if (status == MXT_MSGB_T6_CAL) {
		mxt_debug_info(mxt, "[TSP] maXTouch calibration is "
					   "complete(cal_check_flag = %d)"
					   "\n", mxt->cal_check_flag);
		mxt->mxt_time_point = 0;
		mxt->cal_check_flag = 1;

		if (mxt->check_auto_cal_flag == AUTO_CAL_ENABLE)
			maybe_good_count = 0;

		if (mxt->cal_check_flag == 0) {
			/* ATMEL_DEBUG 0406 */
			mxt_check_touch_ic_timer_stop(mxt);
			mxt->cal_check_flag = 1;
		}
		else if (mxt->cal_check_flag == 55) {
			/* fot SIGERR test */
			mxt->cal_check_flag = 0;
			status |= MXT_MSGB_T6_SIGERR;
			dev_info(&client->dev, "SIGERR test\n");
		}
		else if (mxt->cal_check_flag == 56) {
			mxt->cal_check_flag = 0;
			dev_info(&client->dev, "CFGERR test\n");
			goto cfgerr;
		}
	}
	if (status & MXT_MSGB_T6_SIGERR) {

		dev_err(&client->dev,
				"[TSP] maXTouch acquisition error\n");
		mxt_prerun_reset(mxt, false);
		hw_reset_chip(mxt);
		mxt_postrun_reset(mxt, false);
	}

	if (status & MXT_MSGB_T6_OFL)
		dev_err(&client->dev,
				"[TSP] maXTouch cycle overflow\n");

	if (status & MXT_MSGB_T6_RESET)
		mxt_debug_msg(mxt, "[TSP] maXTouch chip reset\n");

	if (status == MXT_MSG_T6_STATUS_NORMAL)
		mxt_debug_msg(mxt, "[TSP] maXTouch status normal\n");

	/* 2012_0523 :
	 * - ATMEL : please, log the below code */
	mxt_debug_msg(mxt, "T6[0~4] : %02X %02X %02X %02X %02X\n", message[0],
				  message[1], message[2], message[3], message[4]);
}

// #define UNKNOWN_MES_TEST

static int UNKNOWN_MES = 0;
static bool LCD_DES_RESET_NEED = 0;

#ifdef UNKNOWN_MES_TEST
int err_test_count = 0;
#endif

int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;

	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  event;
	u8  length;
	u8  report_id;

	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];
//	printk("[TSP] process_message 0: (0x%x) 1:(0x%x) object:(%d)\n",message[0],message[1],object);
	mxt_debug_trace(mxt, "process_message 0: (0x%x) 1:(0x%x) object:(%d)",
					message[0], message[1], object);
#ifdef UNKNOWN_MES_TEST
	err_test_count++;
	if(err_test_count > 100 && err_test_count < 130)
		object = 0xFF;
#endif

	switch (object) {
		case MXT_PROCG_NOISESUPPRESSION_T62:
			UNKNOWN_MES = 0;
			process_t62_message(message, mxt);
			break;

		case MXT_GEN_COMMANDPROCESSOR_T6:
			UNKNOWN_MES = 0;
			process_t6_message(message, mxt);
			break;

		case MXT_TOUCH_MULTITOUCHSCREEN_T9:
			UNKNOWN_MES = 0;
			process_T9_message(message, mxt);
			break;

		case MXT_TOUCH_KEYARRAY_T15:
			UNKNOWN_MES = 0;
			process_T15_message(message, mxt);
			break;

		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			UNKNOWN_MES = 0;
			process_T42_message(message, mxt);
			mxt_debug_msg(mxt, "[TSP] Receiving Touch "
						  "suppression msg\n");
			break;

		case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			UNKNOWN_MES = 0;
			mxt_debug_msg(mxt, "[TSP] Receiving one-touch gesture msg\n");

			event = message[MXT_MSG_T24_STATUS] & 0x0F;
			xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
				   ((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
			ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
				   ((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
			xpos >>= 2;
			ypos >>= 2;

			switch (event) {
				case MT_GESTURE_RESERVED:
					break;
				case MT_GESTURE_PRESS:
					break;
				case MT_GESTURE_RELEASE:
					break;
				case MT_GESTURE_TAP:
					break;
				case MT_GESTURE_DOUBLE_TAP:
					break;
				case MT_GESTURE_FLICK:
					break;
				case MT_GESTURE_DRAG:
					break;
				case MT_GESTURE_SHORT_PRESS:
					break;
				case MT_GESTURE_LONG_PRESS:
					break;
				case MT_GESTURE_REPEAT_PRESS:
					break;
				case MT_GESTURE_TAP_AND_PRESS:
					break;
				case MT_GESTURE_THROW:
					break;
				default:
					break;
			}
			break;
		case MXT_SPT_SELFTEST_T25:
			UNKNOWN_MES = 0;
			mxt_debug_msg(mxt, "[TSP] Receiving Self-Test msg\n");

			if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK) {
				sprintf(test.pinfault, "%d", MXT_SELF_TEST_SUCCESS); // GT system IM002A-321
				mxt_debug_msg(mxt, "Pinfault Test Success :0x%x, 0x%x, 0x%x, 0x%x \n",
							  message[MXT_MSG_T25_STATUS], message[MXT_MSG_T25_STATUS + 1],
							  message[MXT_MSG_T25_STATUS + 2], message[MXT_MSG_T25_STATUS + 3]);
			}
			else {
				sprintf(test.pinfault, "%d", MXT_SELF_TEST_FAIL); // GT system IM002A-321
				mxt_debug_msg(mxt, "Pinfault Test Fail :0x%x, 0x%x, 0x%x, 0x%x \n",
							  message[MXT_MSG_T25_STATUS], message[MXT_MSG_T25_STATUS + 1],
							  message[MXT_MSG_T25_STATUS + 2], message[MXT_MSG_T25_STATUS + 3]);
			}
			complete(&pinfault_done);// GT system IM002A-321
			break;

		case MXT_SPT_CTECONFIG_T46:
			UNKNOWN_MES = 0;
			mxt_debug_msg(mxt, "[TSP] Receiving CTE message...\n");
			status = message[MXT_MSG_T46_STATUS];
			if (status & MXT_MSGB_T46_CHKERR)
				dev_err(&client->dev,
						"[TSP] maXTouch: Power-Up CRC failure\n");

			break;

		case MXT_SHIELDLESS_T56:
			UNKNOWN_MES = 0;
			break;


		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			UNKNOWN_MES = 0;

			process_T100_message(message, mxt);

			break;

		case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
			UNKNOWN_MES = 0;

			break;

		case MXT_SPT_TIMER_T61:
			UNKNOWN_MES = 0;
			//dev_info(&client->dev,
			//		"[TSP] object :T61!!!\n");

			break;

		case MXT_PROCG_NOISESUPPRESSION_T72:
			UNKNOWN_MES = 0;
			dev_info(&client->dev,
					 "[TSP] object: T72!!!\n");
			break;

		case 80:
			UNKNOWN_MES = 0;
			//dev_info(&client->dev,
			//		"[TSP] object: T80!!!\n");
			break;

		case MXT_SPT_SELFCAPGLOBALCONFIG_T109:
			UNKNOWN_MES = 0;
			
			break;



		default:
			dev_info(&client->dev,
					 "[TSP] maXTouch: Unknown message! %d\n", object);

#if defined(HDMI_1920_720_12_3)
			UNKNOWN_MES++;

			if(UNKNOWN_MES == 5) {
				LCD_DES_RESET_NEED = 1;
				hw_reset_chip(mxt);
				UNKNOWN_MES = 0;
#ifdef UNKNOWN_MES_TEST
				err_test_count = 0;
#endif
			}
#endif
			break;
	}

	return 0;
}

#ifdef MXT_THREADED_IRQ
/* Processes messages when the interrupt line (CHG) is asserted. */

static void mxt_threaded_irq_handler(struct mxt_data *mxt)
{
	struct	i2c_client *client;

	u8	*message = NULL;
	u16	message_length;
	u16	message_addr;
	u8	report_id = 0;
	u8	object;
	int	error;
	bool	need_reset = false;
	int	i;
	int	loop_cnt;
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;
	if (likely(message_length < MXT_MSG_BUFF_SIZE)) {
		message = mxt_message_buf;
	}
	else {
		dev_err(&client->dev,
				"[TSP] Message length larger than 256 bytes "
				"not supported\n");
		return;
	}

	mutex_lock(&mxt->msg_lock);

	/* 2012_0517
	 *  - ATMEL : there is no case over 10 count
	 * 2012_0517
	 *  - tom : increase to 30 */
#define MXT_MSG_LOOP_COUNT 30
	loop_cnt = MXT_MSG_LOOP_COUNT;

	/* mxt_debug_trace(mxt, "[TSP] maXTouch worker active:\n"); */
	do {
		/* Read next message */
		mxt->message_counter++;
		/* Reread on failure! */
		for (i = I2C_RETRY_COUNT; i > 0; i--) {
			/* note: changed message_length to 8 in ver0.9 */
			/* 2012_0510 : ATMEL
			 * - if you want to reduce the message size,
			 *   you can do it.
			 * - but, recommend to use message_length
			 * - if want to reduce the size,
			 *    see MESSAGEPROCESSOR_T5 and MULTITOUCHSCREEN_T9 */
			error = mxt_read_no_delay_block(client,
											message_addr,
											message_length, message);
			if (error >= 0) {
				//I2C success
				//printk("%s message: %c addr: %d length: %d\n", __func__, message, message_addr, message_length);
				break;
			}
			mxt->read_fail_counter++;
			/* Register read failed */
			dev_info(&client->dev, "[TSP] Failure reading "
					 "maxTouch device(%d)\n", i);
		}
		if (i == 0) {
			dev_err(&client->dev, "[TSP] i2c error!\n");
			need_reset = true;
			break;
		}

		report_id = message[0];
		if (debug >= DEBUG_MESSAGES) {
			int p;
			char msg[64] = {0, };

			p = sprintf(msg, "(id:%d)[%d] ",
						report_id, mxt->message_counter);
			for (i = 0; i < message_length; i++)
				p += sprintf(msg + p, "%02X ", message[i]);
			sprintf(msg + p, "\n");

			//dev_info(&client->dev, "%s\n", msg);
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			if (unlikely(debug > DEBUG_NONE)) {
				/* this is for debugging.
				 * so, run only debug is higher than
				 *  DEBUG_NONE */
				for (i = 0; i < message_length; i++)
					mxt->last_message[i] = message[i];

				if (down_interruptible(&mxt->msg_sem)) {
					dev_warn(&client->dev,
							 "[TSP] mxt_worker Interrupted "
							 "while waiting for msg_sem!\n");
					kfree(message);
					mutex_unlock(&mxt->msg_lock);
					return;
				}
				mxt->new_msgs = 1;
				up(&mxt->msg_sem);
				wake_up_interruptible(&mxt->msg_queue);
			}
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}

		if (!loop_cnt--) {
			dev_err(&client->dev, "%s() over %d count\n",
					__func__, MXT_MSG_LOOP_COUNT);
			/* there's no way except POR */
			mxt_prerun_reset(mxt, false);
			hw_reset_chip(mxt);
			mxt_postrun_reset(mxt, false);

			mutex_unlock(&mxt->msg_lock);
			return;
		}

	}
	while ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0));

	mutex_unlock(&mxt->msg_lock);
	mxt_debug_trace(mxt, "loop_cnt : %d\n", MXT_MSG_LOOP_COUNT - loop_cnt);

	/* TSP reset */
	if (need_reset) {
		u8 val = 0;
		/* go to the boot mode for checking the current mode */
		mxt_change_i2c_addr(mxt, true);
		if (i2c_master_recv(client, &val, 1) != 1) {
			dev_err(&client->dev, "reset the chip\n");
			/* back to the application mode */
			mxt_change_i2c_addr(mxt, false);

			mxt_prerun_reset(mxt, false);
			hw_reset_chip(mxt);
			mxt_postrun_reset(mxt, false);
		}
		else {
			/* this chip needs to be upgraded.
			 * plase wait to upgarde by power on.
			 * there is only one possibility which makes
			 *  this problem.
			 *  - first, failed to upgrade(shutdown and so on)
			 *  - second, resume by FASTBOOT
			 *  because, irq has already registered, and h/w power
			 *  on reset generates the first interrupt.
			 * so, normal boot never happend.
			 * TODO : after fastboot is stable, I'll check again this.
			 * */
			dev_info(&client->dev, "wait to upgrade!\n");
			/* back to the application mode */
			mxt_change_i2c_addr(mxt, false);
		}
	}
}

static irqreturn_t mxt_threaded_irq(int irq, void *_mxt)
{
	struct	mxt_data *mxt = _mxt;
	mxt->irq_counter++;
	mxt_threaded_irq_handler(mxt);
	return IRQ_HANDLED;
}

#else
/* Processes messages when the interrupt line (CHG) is asserted. */

static void mxt_worker(struct work_struct *work)
{
	struct	mxt_data *mxt;
	struct	i2c_client *client;

	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error;
	int	i;

	message = NULL;
	mxt = container_of(work, struct mxt_data, dwork.work);
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev,
					"[TSP] Error allocating memory\n");
			return;
		}
	}
	else {
		dev_err(&client->dev,
				"[TSP] Message length larger than "
				"256 bytes not supported\n");
	}

	mxt_debug_trace(mxt, "[TSP] maXTouch worker active:\n");
	do {
		/* Read next message */
		mxt->message_counter++;
		/* Reread on failure! */
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			/* note: changed message_length to 8 in ver0.9 */
			error = mxt_read_no_delay_block(client, message_addr,
											8/*message_length*/, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			pr_alert("[TSP] mXT: message read failed!\n");
			/* Register read failed */
			dev_err(&client->dev,
					"[TSP] Failure reading maxTouch device\n");
		}

		report_id = message[0];
		if (debug >= DEBUG_TRACE) {
			pr_info("[TSP] %s message [%08x]:",
					REPORT_ID_TO_OBJECT_NAME(report_id),
					mxt->message_counter
				   );
			for (i = 0; i < message_length; i++) {
				pr_info("0x%02x ", message[i]);
			}
			pr_info("\n");
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {

			for (i = 0; i < message_length; i++)
				mxt->last_message[i] = message[i];

			if (down_interruptible(&mxt->msg_sem)) {
				pr_warning("[TSP] mxt_worker Interrupted "
						   "while waiting for msg_sem!\n");
				kfree(message);
				return;
			}
			mxt->new_msgs = 1;
			up(&mxt->msg_sem);
			wake_up_interruptible(&mxt->msg_queue);
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}

	}
	while ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0));

	kfree(message);
}

u8 mxt_valid_interrupt(void)
{
	/* TO_CHK: how to implement this function? */
	return 1;
}

/*
* The maXTouch device will signal the host about a new message by asserting
* the CHG line. This ISR schedules a worker routine to read the message when
* that happens.
*/

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct	mxt_data *mxt = _mxt;
	unsigned long	flags;
	mxt->irq_counter++;
	spin_lock_irqsave(&mxt->lock, flags);

	if (mxt_valid_interrupt()) {
		/* Send the signal only if falling edge generated the irq. */
		cancel_delayed_work(&mxt->dwork);
		schedule_delayed_work(&mxt->dwork, 0);
		mxt->valid_irq_counter++;
	}
	else {
		mxt->invalid_irq_counter++;
	}
	spin_unlock_irqrestore(&mxt->lock, flags);

	return IRQ_HANDLED;
}
#endif

/**
 * @ legolamp@cleinsoft
 * @ date 2014.11.20
 * jig detect interrupt - falling edge
 **/
#ifdef MXT_JIG_DETECT
static irqreturn_t mxt_jig_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = (struct mxt_data*)_mxt;

	update_jig_detection(mxt);

	return IRQ_HANDLED;
}
#endif

/* Function to write a block of data to any address on touch chip. */

#define I2C_PAYLOAD_SIZE 254

static ssize_t show_config_info(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	/**
	 * @ legolamp@cleinsoft
	 * @ date 2014.06.03
	 * this func is debug configure register
	 **/
	int i;
	int object_type;
	u16 obj_addr, obj_size;
	u8 message_buff[128];
	struct i2c_client *client;
	struct mxt_data *mxt;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	sscanf(buf, "%d", &object_type);
	obj_addr = MXT_BASE_ADDR(object_type);
	obj_size = MXT_GET_SIZE(object_type);
	mxt_read_block(client, obj_addr, obj_size, message_buff);

	for(i = 0; i < obj_size; i++)
		dev_info(&mxt->client->dev, "T%d_CONFIG [%d]:%d\n",
				 object_type, i, message_buff[i]);

	return count;
}

/*
* Sets up a read from mXT chip. If we want to read config data from user space
* we need to use this first to tell the address and byte count, then use
* get_config to read the data.
*/

static ssize_t set_ap(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t count)
{

	int i;
	struct i2c_client *client;
	struct mxt_data *mxt;
	u16 ap;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	if (count < 3) {
		/* Error, ap needs to be two bytes, plus 1 for size! */
		pr_err("[TSP] set_ap needs to arguments: address pointer "
			   "and data size");
		return -EIO;
	}

	ap = (u16) * ((u16 *)buf);
	i = mxt_write_ap(client, ap);
	mxt->bytes_to_read_ap = (u16) * (buf + 2);
	return count;

}

static ssize_t show_deltas(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	s16     *delta, *pdelta;
	s16     size, read_size;
	u16     diagnostics;
	u16     debug_diagnostics;
	char    *bufp;
	int     x, y;
	int     error;
	u16     *val;
	struct mxt_object obj;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	/* Allocate buffer for delta's */
	size = mxt->device_info.num_nodes * sizeof(__u16);
	pdelta = delta = kzalloc(size, GFP_KERNEL);
	if (!delta) {
		sprintf(buf, "insufficient memory\n");
		return strlen(buf);
	}

	diagnostics = T6_REG(MXT_ADR_T6_DIAGNOSTICS);
	obj = mxt->object_table[mxt->object_link[MXT_ADR_T6_DIAGNOSTICS]];
	if (obj.type == 0) {
		dev_err(&client->dev, "[TSP] maXTouch: Object T6 not found\n");
		kfree(pdelta);
		return 0;
	}

	debug_diagnostics = T37_REG(2);
	obj = mxt->object_table[mxt->object_link[MXT_DEBUG_DIAGNOSTICS_T37]];
	if (obj.type == 0) {
		kfree(pdelta);
		dev_err(&client->dev, "[TSP] maXTouch: Object T37 not found\n");
		return 0;
	}

	/* Configure T37 to show deltas */
	error = mxt_write_byte(client, diagnostics, MXT_CMD_T6_DELTAS_MODE);
	if (error) {
		dev_err(&client->dev, "[TSP] failed to write(%d)\n", error);
		kfree(pdelta);
		return error;
	}

	while (size > 0) {
		read_size = size > 128 ? 128 : size;
		error = mxt_read_block(client,
							   debug_diagnostics,
							   read_size,
							   (__u8 *) delta);
		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
					"[TSP] maXTouch: Error reading delta object\n");
		}
		delta += (read_size / 2);
		size -= read_size;
		/* Select next page */
		mxt_write_byte(client, diagnostics, MXT_CMD_T6_PAGE_UP);
	}

	bufp = buf;
	val  = (s16 *) delta;
	for (x = 0; x < mxt->device_info.x_size; x++) {
		for (y = 0; y < mxt->device_info.y_size; y++)
			bufp += sprintf(bufp, "%05d  ",
							(s16) le16_to_cpu(*val++));
		bufp -= 2;	/* No spaces at the end */
		bufp += sprintf(bufp, "\n");
	}
	bufp += sprintf(bufp, "\n");

	kfree(pdelta);
	return strlen(buf);
}

static int do_show_references(struct mxt_data *mxt)
{
	u16 diagnostics;
	u16 debug_diagnostics;

	int error;
	int loop_page = 0;
	int loop_byte = 0;
	int node_value = 0;
	int result = MXT_SELF_TEST_SUCCESS;

	int	min_diff_value = 0;
	int max_diff_value = 0;

	struct mxt_object obj;

	u8 read_buf[TOUCH_REF_READ_SIZE] = {0x00,};

	VPRINTK("[%s] \n", __func__);

	diagnostics =  T6_REG(MXT_ADR_T6_DIAGNOSTICS);
	obj = mxt->object_table[mxt->object_link[MXT_ADR_T6_DIAGNOSTICS]];
	if (obj.type == 0) {
		dev_err(&mxt->client->dev, "[TSP] maXTouch: Object T6 not found\n");
		return 0;
	}

	debug_diagnostics = T37_REG(0);
	obj = mxt->object_table[mxt->object_link[MXT_DEBUG_DIAGNOSTICS_T37]];
	if (obj.type == 0) {
		dev_err(&mxt->client->dev, "[TSP] maXTouch: Object T37 not found\n");
		return 0;
	}

	/* Configure T37 to show references */
	mxt_write_byte(mxt->client, diagnostics, MXT_CMD_T6_REFERENCES_MODE);
	// IC firmware cycle is about 15msec.
	mdelay(30);

	/* Should check for error */
	for (loop_page = 0; loop_page < 6; loop_page++) {

		error = mxt_read_block(mxt->client, debug_diagnostics, TOUCH_REF_READ_SIZE, (u8 *)read_buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&mxt->client->dev, "[TSP] maXTouch: Error " "reading reference object\n");
			break;
		}

		for (loop_byte = 0; loop_byte < TOUCH_REF_READ_SIZE; loop_byte += 2) {
			if ((loop_page == 5) && (loop_byte > 32))
				break;

			// need mode & page number check
			if (loop_byte == 0) {
				if (read_buf[loop_byte + 1] != loop_page) {
					result = MXT_SELF_TEST_RETRY;
					dev_err(&mxt->client->dev, "Mode Page Read Error, page (%d), read page (%d) \n",
							loop_page, read_buf[loop_byte + 1]);
					break;
				}
			}
			else {
				node_value = 0;
				node_value = (read_buf[loop_byte + 1] << 8);
				node_value += read_buf[loop_byte];

				if ((loop_page == 5) && ( (loop_byte >= 6) && (loop_byte <= 32) )) {
					if (node_value < TOUCH_MIN_RANGE_X17 || node_value > TOUCH_MAX_RANGE) {
						dev_err(&mxt->client->dev, "[Page(%d+1) - %d byte] FAIL  0x%x, 0x%x - %d \n",
								loop_page, loop_byte, read_buf[loop_byte], read_buf[loop_byte + 1], node_value);
						result = MXT_SELF_TEST_SUCCESS;
					}
				}
				else {

					if ( (min_diff_value == 0) && (max_diff_value == 0) ) {
						min_diff_value = node_value;
						max_diff_value = node_value;
					}
					else {
						if (node_value < min_diff_value)
							min_diff_value = node_value;
						else if (node_value > max_diff_value)
							max_diff_value = node_value;
					}

					if (node_value < TOUCH_MIN_RANGE || node_value > TOUCH_MAX_RANGE) {
						dev_err(&mxt->client->dev, "[Page(%d+1) - %d byte] FAIL  0x%x, 0x%x - %d \n",
								loop_page, loop_byte, read_buf[loop_byte], read_buf[loop_byte + 1], node_value);
						result = MXT_SELF_TEST_SUCCESS;
					}
				}
			}
		}

		/* Select next page */
		mxt_write_byte(mxt->client, diagnostics, MXT_CMD_T6_PAGE_UP);
		mdelay(30);		// IC firmware cycle is about 15msec.
	}

	VPRINTK("[%s] Min Node Value (%d), Max Node Value (%d) \n", __func__, min_diff_value, max_diff_value);

	return result;
}

static int do_show_pinfault(struct mxt_data *mxt)
{
	u8 t_buff[2] = {0, };
	int ret = -1;

	// Initialize code setting
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25), 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25) + 1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25) + 26, 0);

	// IC firmware cycle is about 15msec.
	mdelay(30);

	t_buff[0] = 0x03;
	t_buff[1] = 0x12;

	ret = mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25) + 26, 200);
	if (ret < 0) {
		dev_err(&mxt->client->dev, "[%s] ret (%d) \n", __func__, ret);
		return -EIO;
	}

	ret = mxt_write_block(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25), 2, t_buff);
	if (ret < 0) {
		dev_err(&mxt->client->dev, "[%s] ret (%d) [TSP] mxt_write_block failed!\n", __func__, ret);
		return -EIO;
	}

	return SUCCESS;
}

static ssize_t show_references(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;

	int result = 0;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	result = do_show_references(mxt);

	switch (result) {
		case MXT_SELF_TEST_SUCCESS:
			sprintf(buf, "Reference TEST SUCCESS \n");
			break;
		case MXT_SELF_TEST_FAIL:
			sprintf(buf, "Reference TEST FAIL \n");
			break;
		case MXT_SELF_TEST_RETRY:
			sprintf(buf, "Reference TEST Retry \n");
			break;
	}

	return strlen(buf);
}

static ssize_t show_pin_fault(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct i2c_client *client;
	struct mxt_data *mxt;
	int ret = 0;

	VPRINTK( "%s \n", __func__);

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	ret = do_show_pinfault(mxt);

	if (ret == SUCCESS)
		sprintf(buf, "Pinfault TEST SUCCESS \r\n");
	else
		sprintf(buf, "Pinfault TEST FAIL  \r\n");

	return strlen(buf);
}

static ssize_t show_device_info(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	struct atmel_touch_version ver;
	char *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	bufp = buf;
	bufp += sprintf(bufp,
					"Family:\t\t\t[0x%02x] %s\n",
					mxt->device_info.family_id,
					mxt->device_info.family
				   );
	bufp += sprintf(bufp,
					"Variant:\t\t[0x%02x] %s\n",
					mxt->device_info.variant_id,
					mxt->device_info.variant
				   );
	bufp += sprintf(bufp,
					"Firmware version:\t[%2d.%d],Build:0x%02X\n",
					mxt->device_info.major,
					mxt->device_info.minor,
					mxt->device_info.build
				   );
	bufp += sprintf(bufp,
					"%d Sensor nodes:\t[X=%3d, Y=%3d]\n",
					mxt->device_info.num_nodes,
					mxt->device_info.x_size,
					mxt->device_info.y_size
				   );
	bufp += sprintf(bufp,
					"Reported resolution:\t[X=%d, Y=%d]\n",
					mxt->pdata->max_x,
					mxt->pdata->max_y
				   );

	memset(&ver, 0x00, sizeof(struct atmel_touch_version));
	mxt_get_version(mxt, &ver);
	bufp += sprintf(bufp,
					"Configuration:\t[%s]\n",
					ver.conf
				   );

	return strlen(buf);
}

static ssize_t show_stat(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	char *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	bufp = buf;
#ifndef MXT_THREADED_IRQ
	bufp += sprintf(bufp,
					"Interrupts:\t[VALID=%d ; INVALID=%d]\n",
					mxt->valid_irq_counter,
					mxt->invalid_irq_counter);
#endif
	bufp += sprintf(bufp, "Messages:\t[%d]\n", mxt->message_counter);
	bufp += sprintf(bufp, "Read Failures:\t[%d]\n", mxt->read_fail_counter);
	return strlen(buf);
}

/* \brief set objnum_show which is the number for showing
 *	the specific object block
 */
static ssize_t set_object_info(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	if (sscanf(buf, "%i", &state) != 1)
		return -EINVAL;

	mxt->objnum_show = state;

	return count;
}

static ssize_t show_ic_error(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (gtouch_ic_error == 1)
		sprintf(buf, "1");
	else
		sprintf(buf, "0");

	return strlen(buf);
}

static ssize_t show_object_info(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	int i, match_id = -1, size = 0;
	char *bufp;
	struct i2c_client *client;
	struct mxt_data	*mxt;
	struct mxt_object *object_table;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);
	object_table = mxt->object_table;

	bufp = buf;

	bufp += sprintf(bufp, "maXTouch: %d Objects\n",
					mxt->device_info.num_objs);

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		if (object_table[i].type != 0) {
			bufp += sprintf(bufp,
							"Type:\t\t[%d]: %s\n",
							object_table[i].type,
							object_type_name[object_table[i].type]);
			bufp += sprintf(bufp,
							"Address:\t0x%04X\n",
							object_table[i].chip_addr);
			bufp += sprintf(bufp,
							"Size:\t\t%d Bytes\n",
							object_table[i].size);
			bufp += sprintf(bufp,
							"Instances:\t%d\n",
							object_table[i].instances);
			bufp += sprintf(bufp,
							"Report Id's:\t%d\n\n",
							object_table[i].num_report_ids);

			if (mxt->objnum_show == object_table[i].type) {
				match_id = mxt->objnum_show;
				size = object_table[i].size;
			}
		}
	}

	/* show registers of the object number */
	bufp += sprintf(bufp, "object number for show:%d\n", mxt->objnum_show);
	if (match_id != -1) {
		int reg_ok = 0;
		u8 *regbuf = kzalloc(size, GFP_KERNEL);

		if (0 == mxt_get_object_values(mxt, match_id)) {
			if (0 == mxt_copy_object(mxt, regbuf, match_id))
				reg_ok = 1;
		}
		else if (0 == mxt_get_objects(mxt, regbuf, size, match_id))
			reg_ok = 1;

		if (reg_ok) {
			bufp += sprintf(bufp, "   ");
			for (i = 0; i < 16; i++)
				bufp += sprintf(bufp, "%02X ", i);
			bufp += sprintf(bufp, "\n%02X:", 0);
			for (i = 0; i < size; i++) {
				if ((i != 0) && ((i % 16) == 0))
					bufp += sprintf(bufp, "\n%02X:",
									i / 16);
				bufp += sprintf(bufp, "%02X ", (u8)regbuf[i]);
			}
			bufp += sprintf(bufp, "\n");
		}
		kfree(regbuf);
	}

	return strlen(buf);
}

static ssize_t show_messages(struct device *dev,
							 struct device_attribute *attr,
							 char *buf)
{
	struct i2c_client *client;
	struct mxt_data   *mxt;
	struct mxt_object *object_table;
	int   i;
	__u8  *message;
	__u16 message_len;
	__u16 message_addr;

	char  *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);
	object_table = mxt->object_table;

	if (debug <= DEBUG_NONE) {
		dev_info(&client->dev, "enable debug mode\n");
		return -EINVAL;
	}

	bufp = buf;

	message = kmalloc(mxt->message_size, GFP_KERNEL);
	if (message == NULL) {
		pr_warning("Error allocating memory!\n");
		return -ENOMEM;
	}

	message_addr = mxt->msg_proc_addr;
	message_len = mxt->message_size;
	bufp += sprintf(bufp,
					"Reading Message Window [0x%04x]\n",
					message_addr);

	/* Acquire the lock. */
	if (down_interruptible(&mxt->msg_sem)) {
		mxt_debug_info(mxt, "[TSP] mxt: Interrupted while "
					   "waiting for mutex!\n");
		kfree(message);
		return -ERESTARTSYS;
	}

	while (mxt->new_msgs == 0) {
		/* Release the lock. */
		up(&mxt->msg_sem);
		if (wait_event_interruptible(mxt->msg_queue, mxt->new_msgs)) {
			mxt_debug_info(mxt, "[TSP] mxt: Interrupted while "
						   "waiting for new msg!\n");
			kfree(message);
			return -ERESTARTSYS;
		}

		/* Acquire the lock. */
		if (down_interruptible(&mxt->msg_sem)) {
			mxt_debug_info(mxt, "[TSP] mxt: Interrupted while "
						   "waiting for mutex!\n");
			kfree(message);
			return -ERESTARTSYS;
		}
	}

	for (i = 0; i < mxt->message_size; i++)
		message[i] = mxt->last_message[i];

	mxt->new_msgs = 0;

	/* Release the lock. */
	up(&mxt->msg_sem);

	for (i = 0; i < message_len; i++)
		bufp += sprintf(bufp, "0x%02x ", message[i]);
	bufp--;
	bufp += sprintf(bufp, "\t%s\n", REPORT_ID_TO_OBJECT_NAME(message[0]));

	kfree(message);
	return strlen(buf);
}


static ssize_t show_report_id(struct device *dev,
							  struct device_attribute *attr,
							  char *buf)
{
	struct i2c_client    *client;
	struct mxt_data      *mxt;
	struct report_id_map *report_id;
	int                  i;
	int                  object;
	char                 *bufp;

	client    = to_i2c_client(dev);
	mxt       = i2c_get_clientdata(client);
	report_id = mxt->rid_map;

	bufp = buf;
	for (i = 0 ; i < mxt->report_id_count ; i++) {
		object = report_id[i].object;
		bufp += sprintf(bufp, "Report Id [%02d], object [%02d]:\t%s\n ",
						i, object, object_type_name[object]);
	}
	return strlen(buf);
}

static ssize_t show_debug(struct device *dev,
						  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "CURRENT DEBUG MODE : %d\n"
					"0:NONE\n" "1:INFO\n"
					"2:MESSAGES\n" "3:TRACE\n", debug);
}

static ssize_t set_debug(struct device *dev,
						 struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	struct i2c_client *client;
	struct mxt_data *mxt;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	sscanf(buf, "%d", &state);
	if (state >= DEBUG_NONE	&& state <= DEBUG_TRACE) {
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

/* \brief : clear the whole registers */
static void mxt_clear_nvram(struct mxt_data *mxt)
{
	int i;
	struct i2c_client *client;

	client = mxt->client;

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		int ret;
		u16 obj_addr, obj_size;
		u8 *dummy_data;

		obj_addr = mxt->object_table[i].chip_addr;
		obj_size = mxt->object_table[i].size;

		mxt_debug_trace(mxt, "obj_addr:%02X, size:%d\n",
						obj_addr, obj_size);

		dummy_data = kzalloc(obj_size, GFP_KERNEL);
		if (!dummy_data)
			dev_err(&client->dev, "%s()[%d] : no mem\n",
					__func__, __LINE__);
		ret = mxt_write_block(
				  mxt->client, obj_addr, obj_size, dummy_data);
		if (ret < 0)
			dev_err(&client->dev, "%s()[%d] : i2c err(%d)\n",
					__func__, __LINE__, ret);
		kfree(dummy_data);
	}

}

static ssize_t show_test(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0: calibration\n1: h/w reset\n2: nvram backup\n"
					"3: nvram clear\n4: cal_check_flag\n"
					"5: i2c error\n6: sigerr\n"
					"7-8: auto-cal enable/disable \n"
					"9-10: touch ic timer\n11: CFGERR\n"
					"12: tch_atch\n");
}

static ssize_t set_test(struct device *dev,
						struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	struct i2c_client *client;
	struct mxt_data *mxt;
	int tch_ch = 0, atch_ch = 0;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	sscanf(buf, "%d", &state);

	if (state >= 1 && state <= 2)
		mxt_prerun_reset(mxt, true);

	if (state == 0) {
		/* calibration */
		calibrate_chip(mxt);
	}
	else if (state == 1) {
		/* h/w reset chip */
		hw_reset_chip(mxt);
		msleep(MXT_SW_RESET_TIME);
	}
	else if (state == 2) {
		/* nvram_backup  */
		backup_to_nv(mxt);
	}
	else if (state == 3) {
		/* nvram clear */
		mxt_clear_nvram(mxt);
		backup_to_nv(mxt);
	}
	else if (state == 4) {
		/* re-cal test */
		mxt->cal_check_flag = 1;
	}
	else if (state == 5) {
		/* i2c force error */
		/* invalid i2c-slave address */
		client->addr = 0x28;
	}
	else if (state == 6) {
		/* SIGERR test */
		mxt->cal_check_flag = 55;
		sw_reset_chip(mxt, RESET_TO_NORMAL);
		msleep(MXT_SW_RESET_TIME);
	}
	else if (state == 7) {
		u16 addr;
		/* auto-cal test */
		addr = MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8);
		/* 25 menas (25 * 200) ms */
		mxt_write_byte(client, addr + MXT_ADR_T8_TCHAUTOCAL, 25);
	}
	else if (state == 8) {
		/* auto-cal disable */
		_disable_auto_cal(mxt, false);
		mxt->check_auto_cal_flag = AUTO_CAL_DISABLE;
	}
	else if (state == 9) {
		mxt_check_touch_ic_timer_stop(mxt);
		mxt_check_touch_ic_timer_start(mxt);
	}
	else if (state == 10) {
		mxt_check_touch_ic_timer_stop(mxt);
	}
	else if (state == 11) {
		/* CFGERR test */
		mxt->cal_check_flag = 56;
		sw_reset_chip(mxt, RESET_TO_NORMAL);
		msleep(MXT_SW_RESET_TIME);
	}
	else if(state == 12) {
		check_touch_count(mxt, &tch_ch, &atch_ch);
	}
	else {
		pr_info("0: calibration\n1: h/w reset\n2: nvram backup\n"
				"3: nvram clear\n4: cal_check_flag\n"
				"5: i2c error\n6: sigerr\n"
				"7-8: auto-cal enable/disable \n"
				"9-10: touch ic timer\n11: CFGERR\n"
				"12: tch_atch\n");
		return -EINVAL;
	}

	if (state >= 1 && state <= 2)
		mxt_postrun_reset(mxt, true);

	return count;
}

#ifdef MXT_FIRMUP_ENABLE
static ssize_t show_firmware(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);
	u8 val[7];

	mxt_read_block(mxt->client, MXT_ADDR_INFO_BLOCK, 7, (u8 *)val);
	mxt->device_info.major = ((val[2] >> 4) & 0x0F);
	mxt->device_info.minor = (val[2] & 0x0F);
	mxt->device_info.build	= val[3];

	return snprintf(buf, PAGE_SIZE,
					"Atmel %s Firmware version [%d.%d] Build 0x%02X\n",
					mxt336S_variant,
					mxt->device_info.major,
					mxt->device_info.minor,
					mxt->device_info.build);
}

static ssize_t store_firmware(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	int state = 0, ret = 0;
	int cmd;
	char path[DAUDIO_PATH_LEN] = {0};
	struct mxt_data *mxt = dev_get_drvdata(dev);

	if (sscanf(buf, "%d:%s", &state, path) != 2 ||
			(state < MXT_UP_CMD_BEGIN || state >= MXT_UP_CMD_END)) {
		dev_err(dev, "[TSP] Invalid argument : %s\n"
				"Usage : echo cmd:fw_path > /xxx/02_firmware_up\n"
				"cmd : 1 - firmware update using the firmware class (fixed location : /etc/firmware/mxt336s.fw\n"
				"('fw_path' is completely meaningless at this mode)\n"
				"2 - external location firmware update (2:/xxx/xxx/mxt336s.fw)\n", buf);
		return -EINVAL;
	}

	if (state == MXT_UP_AUTO || state == MXT_UP_CHECK_AND_AUTO) {
		if (state == MXT_UP_CHECK_AND_AUTO) {
			/* test the chip status */
			if (MXT_FIRM_STABLE(mxt->firm_status_data)) {
				dev_info(dev, "[TSP] the firmware is stable\n");
				return count;
			}

			/* test whether firmware is out of order */
			mxt_change_i2c_addr(mxt, true);
			ret = mxt_identify(mxt->client, mxt);
			if (-EIO == ret) {
				dev_err(dev, "[TSP] there's no mxt336S "
						"in your board!\n");
				return ret;
			}
		}

		cmd = MXT_FIRMUP_FLOW_LOCK |
			  MXT_FIRMUP_FLOW_UPGRADE | MXT_FIRMUP_FLOW_UNLOCK;
	}

	ret = set_mxt_auto_update_exe(mxt, cmd, state, path);

	if (ret < 0)
		count = ret;

	return count;
}
#endif

static ssize_t show_firm_status(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
					MXT_FIRM_STABLE(mxt->firm_status_data) ? "stable" :
					"unstable");
}

static void reg_update_dwork(struct work_struct *work)
{
	struct	mxt_data *mxt = NULL;
	mxt = container_of(work, struct mxt_data, reg_update.dwork.work);

	mutex_lock(&mxt->reg_update.lock);
	if (mxt->reg_update.flag != _MXT_REGUP_NOOP) {
		del_timer(&mxt->reg_update.tmr);
		mxt->reg_update.flag = _MXT_REGUP_NOOP;

		enable_irq(mxt->client->irq);
		wake_unlock(&mxt->wakelock);
	}
	mutex_unlock(&mxt->reg_update.lock);

}

/* \brief to check whether the requested update is over or dead
 *
 * after trying to update the registers, this function would be called
 * after 10 seconds. if running still, change th flags and enable irq
 * */
static void reg_update_handler(unsigned long data)
{
	struct mxt_data *mxt = (struct mxt_data *)data;

	dev_info(&mxt->client->dev, "timeout!");

	schedule_delayed_work(&mxt->reg_update.dwork, 0);
}

/* \brief to update the new registers by atmel generated files
 * simple flow (..SECTION.. ; one section string)
 *	@ echo "[BEGIN]" > registers
 *		@ echo "[BEGINV]" > registers (for print register info)
 *	@ echo ..SECTION.. > registers
 *	@ echo ..SECTION.. > registers
 *	@ ..
 *	@ echo "[END]" > registers
 * */
static ssize_t store_registers(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

#define _CHECK_UPDATE_FLAG(flag_, err)					\
	do {								\
		if (mxt->reg_update.flag != flag_) {			\
			count = 0;					\
			dev_err(dev, "%s", err);			\
			goto store_registers_err0;			\
		}							\
	} while (0);

	if (!MXT_FIRM_STABLE(mxt->firm_status_data)) {
		dev_err(dev, "[TSP] firmware is not stable");
		return -ENXIO;
	}

	mutex_lock(&mxt->reg_update.lock);
	if (!strncmp(buf, "[BEGIN]", strlen("[BEGIN]"))) {
		int len;
		_CHECK_UPDATE_FLAG(_MXT_REGUP_NOOP, "[TSP] alerady running\n");
		dev_info(dev, "[TSP] start the reigter update\n");
		len = strlen("[BEGIN]");
		/* "[BEGIN]V" means verbose */
		if ((strlen(buf) > len) && (buf[len] == 'V'))
			mxt->reg_update.verbose = 1;
		else
			mxt->reg_update.verbose = 0;
		mxt->reg_update.flag = _MXT_REGUP_RUNNING;

		mxt_get_version(mxt, &prev_ver);

		/* init a timer for time-out */
		init_timer(&(mxt->reg_update.tmr));
		mxt->reg_update.tmr.data = (unsigned long)(mxt);
		mxt->reg_update.tmr.function = reg_update_handler;
		mod_timer(&mxt->reg_update.tmr,
				  jiffies + msecs_to_jiffies(1000 * 10));

#if TS_100S_TIMER_INTERVAL
		ts_100ms_timer_stop(mxt);
#endif
		mxt_check_touch_ic_timer_stop(mxt);
		mxt_supp_ops_stop(mxt);

		wake_lock(&mxt->wakelock);
		disable_irq(mxt->client->irq);
	}
	else if (!strncmp(buf, "[END]", strlen("[END]"))) {
		_CHECK_UPDATE_FLAG(_MXT_REGUP_RUNNING,
						   "[TSP] Now, no-running stats\n");
		mxt->reg_update.flag = _MXT_REGUP_NOOP;
		del_timer(&mxt->reg_update.tmr);
		dev_info(dev, "[TSP] end the reigter update\n");

		mxt_update_backup_nvram(mxt, &prev_ver);

		if (!try_sw_reset_chip(mxt, RESET_TO_NORMAL))
			msleep(MXT_SW_RESET_TIME);

		enable_irq(mxt->client->irq);
		wake_unlock(&mxt->wakelock);
	}
	else {
		_CHECK_UPDATE_FLAG(_MXT_REGUP_RUNNING, "[TSP] please, start\n");
		if (0 != mxt_load_registers(mxt, buf, count)) {
			count = 0;
			dev_err(dev, "[TSP] failed to update registers\n");
		}
	}

store_registers_err0:
	mutex_unlock(&mxt->reg_update.lock);
	return count;
}

#ifdef MXT_FIRMUP_ENABLE
static int set_mxt_auto_update_exe(struct mxt_data *mxt, int cmd, int state, char* path)
{
	int ret = 0;
	struct i2c_client *client = mxt->client;

	mxt_debug_trace(mxt, "%s : %d\n", __func__, cmd);

	/* lock status */
	if (cmd & MXT_FIRMUP_FLOW_LOCK) {
		wake_lock(&mxt->wakelock);
		disable_irq(client->irq);
#if TS_100S_TIMER_INTERVAL
		ts_100ms_timer_stop(mxt);
#endif
		mxt_check_touch_ic_timer_stop(mxt);
		mxt_supp_ops_stop(mxt);

		if (MXT_FIRM_STABLE(mxt->firm_status_data))
			mxt->firm_status_data = MXT_FIRM_STATUS_START_STABLE;
		else
			mxt->firm_status_data = MXT_FIRM_STATUS_START_UNSTABLE;
	}

	if (cmd & MXT_FIRMUP_FLOW_UPGRADE) {
		ret = mxt_load_firmware(&client->dev,
								MXT336S_FIRMWARE, path, state );
	}
	if (cmd & MXT_FIRMUP_FLOW_UNLOCK) {

		if ((cmd & MXT_FIRMUP_FLOW_UPGRADE) && ret < 0) {
			enable_irq(client->irq);
			wake_unlock(&mxt->wakelock);
			return ret;
		}

		/* 2012.08.16
		 * mdelay(100)
		 * --> after firmware update, slave i2c addr bootloader mode.
		 * request delay time to ATMEL.
		 */
		mdelay(3000);

		/* anyway, It's success to upgrade the requested firmware
		 * even if identify/read_object_table will fail */
		mxt->firm_status_data = MXT_FIRM_STATUS_SUCCESS;

		/* chip reset and re-initialize */
		hw_reset_chip(mxt);

		ret = mxt_identify(client, mxt);
		if (ret >= 0) {
			mxt_debug_info(mxt, "[TSP] mxt_identify Sucess");

			ret = mxt_read_object_table(client, mxt);
			if (ret >= 0) {
				mxt_debug_info(mxt,
							   "[TSP] mxt_read_object_table Sucess");
			}
			else
				dev_err(&client->dev,
						"[TSP] mxt_read_object_table Fail");
		}
		else
			dev_err(&client->dev,
					"[TSP] mxt_identify Fail ");

		if (ret >= 0) {
			/* mxt->mxt_status = true; */
			mxt_write_init_registers(mxt);
		}

		enable_irq(client->irq);
		wake_unlock(&mxt->wakelock);
	}

	return ret;
}
#endif

static int chk_obj(u8 type, int version)
{
	int f_ver;
	f_ver = version;
	if (f_ver <= 10) {
		switch (type) {
			case MXT_GEN_MESSAGEPROCESSOR_T5:
			case MXT_GEN_COMMANDPROCESSOR_T6:
			case MXT_GEN_POWERCONFIG_T7:
			case MXT_GEN_ACQUIRECONFIG_T8:
			case MXT_TOUCH_MULTITOUCHSCREEN_T9:
			case MXT_TOUCH_KEYARRAY_T15:
			case MXT_SPT_COMMSCONFIG_T18:
			case MXT_SPT_GPIOPWM_T19:
			case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			case MXT_SPT_SELFTEST_T25:
			case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
			case MXT_DEBUG_DIAGNOSTICS_T37:
			case MXT_USER_INFO_T38:
			case MXT_PROCI_GRIPSUPPRESSION_T40:
			case MXT_PROCI_TOUCHSUPPRESSION_T42:
			case MXT_SPT_CTECONFIG_T46:
			case MXT_PROCI_STYLUS_T47:
			case MXT_PROCG_NOISESUPPRESSION_T48:
			case MXT_MESSAGECOUNT_T44:
			case MXT_TOUCH_PROXIMITY_T52:
				return 0;
			default:
				return -1;
		}
	}
	else if (f_ver <= 21) {
		switch (type) {
			case MXT_GEN_MESSAGEPROCESSOR_T5:
			case MXT_GEN_COMMANDPROCESSOR_T6:
			case MXT_GEN_POWERCONFIG_T7:
			case MXT_GEN_ACQUIRECONFIG_T8:
			case MXT_TOUCH_MULTITOUCHSCREEN_T9:
			case MXT_TOUCH_KEYARRAY_T15:
			case MXT_SPT_COMMSCONFIG_T18:
			case MXT_SPT_GPIOPWM_T19:
			case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			case MXT_SPT_SELFTEST_T25:
			case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
			case MXT_DEBUG_DIAGNOSTICS_T37:
			case MXT_USER_INFO_T38:
			case MXT_PROCI_GRIPSUPPRESSION_T40:
			case MXT_PROCI_TOUCHSUPPRESSION_T42:
			case MXT_SPT_CTECONFIG_T46:
			case MXT_PROCI_STYLUS_T47:
			case MXT_PROCG_NOISESUPPRESSION_T48:
			case MXT_MESSAGECOUNT_T44:
			case MXT_TOUCH_PROXIMITY_T52:
			case MXT_DATA_SOURCE_T53:
			case MXT_ADAPTIVE_THRESHOLD_T55:
			case MXT_SHIELDLESS_T56:
			case MXT_EXTRA_TOUCH_SCREEN_DATA_T57:
			case MXT_PROCG_NOISESUPPRESSION_T62:
				return 0;
			default:
				return -1;
		}
	}
	return 1;
}

static ssize_t show_message(struct device *dev,
							struct device_attribute *attr,
							char *buf)
{
#ifdef TSP_DEBUG_MESSAGE
	struct mxt_data *mxt = NULL;
	char *bufp = NULL;
	unsigned short msg_tail_cnt = 0;
	mxt = dev_get_drvdata(dev);

	bufp = buf;
	bufp += sprintf(bufp,
					"Show recently touch message, msg_log.cnt = %d\n",
					msg_log.cnt);
	msg_tail_cnt = msg_log.cnt + 1;
	msg_tail_cnt &= (MAX_MSG_LOG_SIZE - 1);

	do {
		bufp += sprintf(bufp, "%d,\t", msg_log.id[msg_tail_cnt]);
		bufp += sprintf(bufp, "%x,\t", msg_log.status[msg_tail_cnt]);
		bufp += sprintf(bufp, "%d,\t", msg_log.xpos[msg_tail_cnt]);
		bufp += sprintf(bufp, "%d,\t", msg_log.ypos[msg_tail_cnt]);
		bufp += sprintf(bufp, "%d,\t", msg_log.area[msg_tail_cnt]);
		bufp += sprintf(bufp, "%d\n", msg_log.amp[msg_tail_cnt]);

		msg_tail_cnt++;
	}
	while (msg_tail_cnt != msg_log.cnt);

	for (msg_log.cnt = 0; msg_log.cnt < 128; msg_log.cnt++) {
		msg_log.id[msg_log.cnt] = 0;
		msg_log.status[msg_log.cnt] = 0;
		msg_log.xpos[msg_log.cnt] = 0;
		msg_log.ypos[msg_log.cnt] = 0;
		msg_log.area[msg_log.cnt] = 0;
		msg_log.amp[msg_log.cnt] = 0;
	}
	msg_log.cnt = 0;
#endif
	return strlen(buf);

}

static ssize_t show_object(struct device *dev,
						   struct device_attribute *attr, char *buf)
{
	struct mxt_data *mxt;
	struct mxt_object  *object_table;

	int count = 0;
	int i, j;
	int version;
	u8 val;

	mxt = dev_get_drvdata(dev);
	object_table = mxt->object_table;

	version = (mxt->device_info.major * 10) + (mxt->device_info.minor);

	pr_info("maXTouch: %d Objects\n",
			mxt->device_info.num_objs);

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		u8 obj_type = object_table[i].type;

		if (chk_obj(obj_type, version))
			continue;

		count += sprintf(buf + count, "%s: %d addr %d bytes",
						 object_type_name[obj_type], MXT_BASE_ADDR(obj_type),
						 object_table[i].size);

		for (j = 0; j < object_table[i].size; j++) {
			mxt_read_byte(mxt->client,
						  MXT_BASE_ADDR(obj_type) + (u16)j,
						  &val);
			if (!(j % 8))
				count += sprintf(buf + count, "\n%02X : ", j);
			count += sprintf(buf + count, " %02X", val);
		}

		count += sprintf(buf + count, "\n\n");
	}

	/* debug only */
	/*
	count += sprintf(buf + count, "%s: %d bytes\n", "debug_config_T0", 32);

	for (j = 0; j < 32; j++) {
		count += sprintf(buf + count,
			"  Byte %2d: 0x%02x (%d)\n",
			j,
			mxt->debug_config[j],
			mxt->debug_config[j]);
	}
	* */

	return count;
}

static ssize_t store_object(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *mxt;
	/*	struct mxt_object	*object_table;//TO_CHK: not used now */
	struct i2c_client *client;

	unsigned int type, offset, val;
	u16 chip_addr;
	int ret;

	mxt = dev_get_drvdata(dev);
	client = mxt->client;

	if ((sscanf(buf, "%u %u %u", &type, &offset, &val) != 3)
			|| (type >= MXT_MAX_OBJECT_TYPES)) {
		pr_err("Invalid values");
		return -EINVAL;
	}

	mxt_debug_info(mxt, "[TSP] Object type: %u, Offset: %u, Value: %u\n",
				   type, offset, val);

	/* debug only */
	/*
	count += sprintf(buf + count, "%s: %d bytes\n", "debug_config_T0", 32);
	*/

	if (type == 0) {
		/* mxt->debug_config[offset] = (u8)val; */
	}
	else {
		chip_addr = get_object_address(type, 0, mxt->object_table,
									   mxt->device_info.num_objs, mxt->object_link);
		if (chip_addr == 0) {
			dev_err(&client->dev,
					"[TSP] Invalid object type(%d)!", type);
			return -EIO;
		}

		ret = mxt_write_byte(client, chip_addr + (u16)offset, (u8)val);
		if (ret < 0) {
			dev_err(&client->dev,
					"[TSP] store_object result: (%d)\n",
					ret);
			return ret;
		}
	}

	return count;
}


/* Register sysfs files */
static DEVICE_ATTR(0_device_info, S_IRUGO, show_device_info, NULL);
#ifdef MXT_FIRMUP_ENABLE
static DEVICE_ATTR(1_firm_status,  S_IRUSR | S_IRGRP, show_firm_status, NULL);
static DEVICE_ATTR(2_firmware_up, (S_IWUSR | S_IWGRP) | S_IRUGO, show_firmware, store_firmware);
#endif

static DEVICE_ATTR(3_config_info,	S_IWUSR, NULL, show_config_info);
static DEVICE_ATTR(4_references,	S_IRUGO, show_references, NULL);
static DEVICE_ATTR(5_pinfault,		S_IRUGO, show_pin_fault, NULL);
static DEVICE_ATTR(6_deltas,		S_IRUGO, show_deltas,	 NULL);
static DEVICE_ATTR(7_stat,			S_IRUGO, show_stat,		 NULL);
static DEVICE_ATTR(8_debug,			( S_IWUSR | S_IWGRP ) | S_IRUGO, show_debug, set_debug);
static DEVICE_ATTR(9_test,			( S_IWUSR | S_IWGRP ) | S_IRUGO, show_test, set_test);
static DEVICE_ATTR(10_ic_error,		S_IRUGO, show_ic_error,		 NULL);
static DEVICE_ATTR(object, S_IWUSR | S_IRUGO, show_object, store_object);

static struct attribute *maxTouch_attributes[] = {
	&dev_attr_0_device_info.attr,
#ifdef MXT_FIRMUP_ENABLE
	&dev_attr_1_firm_status.attr,
	&dev_attr_2_firmware_up.attr,
#endif
	&dev_attr_3_config_info.attr,
	&dev_attr_4_references.attr,
	&dev_attr_5_pinfault.attr,
	&dev_attr_6_deltas.attr,
	&dev_attr_7_stat.attr,
	&dev_attr_8_debug.attr,
	&dev_attr_9_test.attr,
	&dev_attr_10_ic_error.attr,
	&dev_attr_object.attr,
	NULL,
};

static struct attribute_group maxtouch_attr_group = {
	.attrs = maxTouch_attributes,
};

/* This function sends a calibrate command to the maXTouch chip.
* While calibration has not been confirmed as good, this function sets
* the ATCHCALST and ATCHCALSTHR to zero to allow a bad cal to always recover
* Returns WRITE_MEM_OK if writing the command to touch chip was successful.
*/
unsigned char not_yet_count;
/* extern gen_acquisitionconfig_t8_config_t acquisition_config; */
/* extern int mxt_acquisition_config(struct mxt_data *mxt); */
static void mxt_supp_ops_dwork(struct work_struct *work)
{
	struct mxt_data *mxt = NULL;
	mxt = container_of(work, struct mxt_data, supp_ops.dwork.work);

	dev_info(&mxt->client->dev,
			 "%s() suppression time-out\n", __func__);
	calibrate_chip(mxt);
}
static inline void mxt_supp_ops_stop(struct mxt_data *mxt)
{
	cancel_delayed_work(&mxt->supp_ops.dwork);
}
static void mxt_cal_timer_dwork(struct work_struct *work)
{
	int rc;
	struct mxt_data *mxt = NULL;

	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.23
	* Limit hardkey normal operation counting is 3
	**/
	check_hardkey_cnt += 1;
	if(check_hardkey_cnt > 2)
		maybe_good_count = 1;

	mxt = container_of(work, struct mxt_data, cal_timer.dwork.work);

	dev_info(&mxt->client->dev,
			 "%s() cal-timer start !!!\n", __func__);
	rc = _check_recalibration(mxt);

}
static void mxt_check_touch_ic_timer_dwork(struct work_struct *work)
{
	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* Check infoblock - i2c_error
	* Compare Family ID
	**/

	int error;
	u8 buf[2] = {0, };
	struct mxt_data *mxt = NULL;
	struct i2c_client *client;
	mxt = container_of(work, struct mxt_data,
					   check_touch_ic_timer.dwork.work);
	client = mxt->client;

#if 0
// #if defined(INCLUDE_SER_DES)
	if(LCD_DES_RESET_NEED == 1) {
		unsigned short addr = mxt->client->addr;
		u8 val;

		mxt->client->addr = 0x6C;
		mxt_read_byte(mxt->client, 0x14, &val);
		val = (val & 0xf7); //D3-> 0
		val = (val | 0x04); //D2->1
		printk(KERN_ERR "LCD_DES_RESET[%x]\r\n", val);
		mxt_write_onebyte_register(mxt->client, 0x14, val);
		mxt->client->addr = addr;
		LCD_DES_RESET_NEED = 0;
	}
#endif

	/**
	 * @ legolamp@cleinsoft
	 * @ date 2014.11.20
	 * Check jig detection status
	 **/
#ifdef MXT_JIG_DETECT
	error = update_jig_detection(mxt);
	if(error == 0) { // no detect jig
#endif
		error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, 1, (u8 *)buf);

		if (error < 0) {
			//exception
			dev_err(&mxt->client->dev, "[TSP:%d] %s() hw-reset by (%d), count (%d)\n", __LINE__, __func__, error, gtouch_ic_reset_cnt);
			mxt_prerun_reset(mxt, true);
			hw_reset_chip(mxt);
			mxt_postrun_reset(mxt, true);
#if defined(HDMI_1920_720_12_3)
			LCD_DES_RESET_NEED = 1;
#endif
			// Touch Error Logging
			gtouch_ic_reset_cnt++;
			if (gtouch_ic_reset_cnt > 10) {
				gtouch_ic_reset_cnt = 10;
				gtouch_ic_error = 1;
			}
			return;
		}
		else {
			if (buf[0] == MAXTOUCH_FAMILYID||buf[0] == MAXTOUCH_FAMILYID_S) {
				gtouch_ic_reset_cnt = 0;

				/* 2015/08/21, YG_US 분리형 모니터 부팅시 터치 IC 연결 안된 상황 복구 코드*/
				if (mxt_initialize_device == false) {
					printk(KERN_ERR "[%s:%d] mxt_initialize false........ \r\n", __func__, __LINE__);

					mxt_initialize_device = true;

					mxt_prerun_reset(mxt, true);
					hw_reset_chip(mxt);
#if defined(HDMI_1920_720_12_3)
					LCD_DES_RESET_NEED = 1;
#endif
					mxt_postrun_reset(mxt, true);

					enable_irq(mxt->irq);
				}
				//printk("%s Touch IC OK !\n", __func__);
			}
			else {
				dev_err(&mxt->client->dev, "[TSP:%d] %s() hw-reset by (%d), count (%d)\n", __LINE__, __func__, error, gtouch_ic_reset_cnt);
				mxt_prerun_reset(mxt, true);
				hw_reset_chip(mxt);
#if defined(HDMI_1920_720_12_3)
				LCD_DES_RESET_NEED = 1;
#endif
				mxt_postrun_reset(mxt, true);
				// Touch Error Logging
				gtouch_ic_reset_cnt++;
				if (gtouch_ic_reset_cnt > 10) {
					gtouch_ic_reset_cnt = 10;
					gtouch_ic_error = 1;
				}
				return;
			}
		}
#ifdef MXT_JIG_DETECT
	}
#endif
	cancel_delayed_work(&mxt->check_touch_ic_timer.dwork);
	schedule_delayed_work(&mxt->check_touch_ic_timer.dwork,
						  msecs_to_jiffies(MXT_CHK_TOUCH_IC_TIME));
}
static void mxt_check_touch_ic_timer_start(struct mxt_data *mxt)
{
	cancel_delayed_work(&mxt->check_touch_ic_timer.dwork);
	schedule_delayed_work(&mxt->check_touch_ic_timer.dwork,
						  msecs_to_jiffies(MXT_CHK_TOUCH_IC_TIME));
	mxt->check_touch_ic_timer.flag = MXT_CHECK_TOUCH_IC_FLAG_ON;
}
static inline void mxt_check_touch_ic_timer_stop(struct mxt_data *mxt)
{
	cancel_delayed_work(&mxt->check_touch_ic_timer.dwork);
	mxt->check_touch_ic_timer.flag = MXT_CHECK_TOUCH_IC_FLAG_OFF;
}
static int check_touch_flag(struct mxt_data *mxt, int *touch, int *anti_touch)
{
	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* we have had the first touchscreen or face suppression message
	* after a calibration - check the sensor state and try to confirm if
	* cal was good or bad
	**/
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	int tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i, j;
	uint8_t x_line_limit;
	struct i2c_client *client = mxt->client;

	/* start read touch flag / anti-touch flag command */
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) + MXT_ADR_T6_DIAGNOSTICS, 0xF3);

	/* get the address of the diagnostic object so we can get the data we need */
	diag_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTICS_T37);

	/* SAP_Sleep(10); */
	msleep(10);

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	memset( data_buffer , 0xFF, sizeof( data_buffer ) );

	/* read_mem(diag_address, 2,data_buffer); */
	mxt_read_block(client, diag_address, 3, data_buffer);
	/* wait for diagnostic object to update */
	while(!((data_buffer[0] == MXT_MSG_T37_CMD_TOUCH_HEAD[0])
			&& (data_buffer[1] == MXT_MSG_T37_CMD_TOUCH_HEAD[1]))) {

		if(data_buffer[0] == MXT_MSG_T37_CMD_TOUCH_HEAD[0]) {
			if( data_buffer[1] == 0x01) {
				/* Page down */
				data_byte = 0x02;
				mxt_write_byte(mxt->client,
							   MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) + MXT_ADR_T6_DIAGNOSTICS, data_byte);

				/* SAP_Sleep(5); */
				msleep(10);
			}
		}
		else {
			msleep(10);
		}
		/* wait for data to be valid  */
		if(try_ctr > 50) {
			/* Failed! */
			pr_err("[TSP] Diagnostic Data did not update!!\n");
			break;
		}

		try_ctr++; /* timeout counter */

		/* read_mem(diag_address, 2,data_buffer); */
		mxt_read_block(client, diag_address, 3, data_buffer);

		mxt_debug_info(mxt, "[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
	}

	if(try_ctr > 50) {
		pr_err("[TSP] %s, Diagnostic Data did not update over 3 !! %d\n", __func__, try_ctr);
		/* send page up command so we can detect when data updates next time,
		 * page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;
		/* write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte); */
		mxt_write_byte(mxt->client,
					   MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) + MXT_ADR_T6_DIAGNOSTICS, data_byte);
		return -1;
	}
	/* data is ready - read the detection flags */
	/* read_mem(diag_address, 100, data_buffer); */
	mxt_read_block(client, diag_address, 100, data_buffer);

	/* data array is 24 x 14 bits for each set of flags, 2 byte header,
		48 bytes for touch flags 48 bytes for antitouch flags*/
	/* count up the channels/bits if we recived the data properly */
	if((data_buffer[0] == MXT_MSG_T37_CMD_TOUCH_HEAD[0])
			&& (data_buffer[1] == MXT_MSG_T37_CMD_TOUCH_HEAD[1])) {
		x_line_limit = 24;

		/* double the limit as the array is in bytes not words */
		x_line_limit = x_line_limit << 1;

		/* count the channels and print the flags to the log */
		for(i = 0; i < x_line_limit; i += 2) { /*check X lines - data is in words so increment 2 at a time */
			/* count how many bits set for this row */
			for(j = 0; j < 8; j++) {
				/* create a bit mask to check against */
				check_mask = 1 << j;

				/* check detect flags */
				if(data_buffer[2 + i] & check_mask) {
					tch_ch++;
				}
				if(data_buffer[3 + i] & check_mask) {
					tch_ch++;

					/* 14, 15 bits is don't care */
					if((check_mask == 0x40) || (check_mask == 0x80)) {
						VPRINTK("[%s] : 14, 15 bits is don't care \n", __func__);
						tch_ch -= 1;
					}
				}

				/* check anti-detect flags */
				if(data_buffer[50 + i] & check_mask) {
					atch_ch++;
				}
				if(data_buffer[51 + i] & check_mask) {
					atch_ch++;
				}
			}
		}
		/* print how many channels we counted */
		mxt_debug_info(mxt, "[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);
		dev_info(&mxt->client->dev, "tch_ch:%d, atch_ch:%d\n", tch_ch, atch_ch);
		/* send page up command so we can detect when data updates next time,
		 * page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;
		/* write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte); */
		mxt_write_byte(mxt->client,
					   MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) + MXT_ADR_T6_DIAGNOSTICS, data_byte);
	}
	*touch = tch_ch;
	*anti_touch = atch_ch;

	return 0;
}

static int check_touch_count(struct mxt_data *mxt, int *touch,
							 int *anti_touch)
{
	int rc = 0, i;
	uint16_t addr_t6, addr_t37;
	int buff_size = 0;
	unsigned char buff[100] = {0, };

	addr_t6 = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6);
	addr_t37 = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTICS_T37);
	//buff_size = 2 + T9_XSIZE * ((T9_YSIZE + 7) / 8);
	buff_size = 2 + 32 * ((20 + 7) / 8);	//641T(X:32, Y:20)

	for (i = 0; i < 2; i++) {
		unsigned char cmd;
		if (0 == i)
			cmd = 0xF3;
		else
			cmd = 0x01;

		rc = mxt_wait_touch_flag(mxt, addr_t37, addr_t6, cmd);
		if (rc < 0) {
			dev_info(&mxt->client->dev, "1.rc:%d\n", rc);
			goto out;
		}

		rc = mxt_read_touch_flag(mxt, addr_t37,	(unsigned char)i, buff, buff_size);
		if (rc < 0)
			goto out;

		if (0 == i)
			*touch = rc;
		else
			*anti_touch = rc;
	}

	return 0;
out:
	*touch = *anti_touch = 0;
	return rc;
}

static int mxt_wait_touch_flag(struct mxt_data *mxt, uint16_t addr_t37,
							   uint16_t addr_t6, unsigned char cmd)
{
	int rc = 0;
	unsigned char page = 0;
	char buff[2] = {0,};
	int retry_count = 50;
	struct i2c_client *client = mxt->client;
	unsigned char t48_tchthr;
	//u16 t48_addr = MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48);

	if (0xF3 == cmd) {
		/* we have had the first touchscreen or face suppression message
		 * after a calibration - check the sensor state and try to confirm if
		 * cal was good or bad
		 * get touch flags from the chip using the diagnostic object
		 * write command to command processor to get touch flags -
		 * 0xF3 Command required to do this */
		rc = mxt_write_byte(mxt->client, addr_t6 + MXT_ADR_T6_DIAGNOSTICS, cmd);
		/* SAP_Sleep(5); */
		page = 0;
		msleep(5);
	}
	else if (0x01 == cmd /* || 0x02 == cmd */) {
		/* page-up or page-down */
		page = 1;
		rc = mxt_write_byte(mxt->client, addr_t6 + MXT_ADR_T6_DIAGNOSTICS, cmd);
	}
	else {
		return -EINVAL;
	}

	if (rc < 0) {
		dev_err(&client->dev, "%s() err:%d\n", __func__, rc);
		return -EIO;
	}

	/* wait for diagnostic object to update */
	buff[0] = buff[1] = 0xFF;
	do {
		rc = mxt_read_block(mxt->client, addr_t37, 2, buff);
		if ((buff[0] == 0xF3 && buff[1] == page))
			break;
		msleep(2);
	}
	while (--retry_count && !rc);

	if (retry_count <= 0 || rc) {
		if (retry_count <= 0)
			rc = -ETIME;
		dev_err(&client->dev, "%s(), retry_count:%d"
				" rc:%d\n", __func__, retry_count, rc);
	}

	return rc;
}

static int mxt_read_touch_flag(struct mxt_data *mxt, uint16_t addr_t37,
							   unsigned char page, char *buff, int buff_size)
{
	int rc = 0, i, j;
	int count = 0;
	unsigned char mask;
	struct i2c_client *client = mxt->client;

	rc = mxt_read_block(client, addr_t37, buff_size, buff);

	if (rc) {
		dev_err(&client->dev, "%s() rc:%d\n", __func__, rc);
		goto out;
	}

	if (buff[0] != 0xF3 || buff[1] != page) {
		dev_err(&client->dev, "%s() buff:0x%02X 0x%02X, page:%d\n",
				__func__, buff[0], buff[1], page);
		rc = -EPERM;
		goto out;
	}

	for (i = 2; i < buff_size; i++) {
		for (j = 0; j < 8; j++) {
			mask = 1 << j;
			if (mask & buff[i])
				count++;
		}
	}
	rc = count;

out:
	return rc;
}

static int calibrate_chip(struct mxt_data *mxt)
{
	int ret = 0;
	u8 data = 1u;
	struct i2c_client *client = mxt->client;

	VPRINTK("[%s]\n", __func__);
	palm_info.facesup_message_flag = 0;
	ts_100ms_timer_clear(mxt);
	not_yet_count = 0;

	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.23
	* Set flag about Auto-Calibration
	**/
	mxt->check_auto_cal_flag = AUTO_CAL_DISABLE;
	maybe_good_count = 0;
	check_hardkey_cnt = 0;

	/* restore settings to the local structure so that when we confirm them
	 * cal is good we can correct them in the chip this must be done before
	 * returning send calibration command to the chip change calibration
	 * suspend settings to zero until calibration confirmed good */
	if( NULL != mxt->object_table ) {
		ret = mxt_write_byte(client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6)
							 + MXT_ADR_T6_CALIBRATE, data);
	}
	else {
		ret = -1;
	}

	if (ret < 0) {
		dev_err(&client->dev, "[TSP] faild to run CALRIBRATE!\n");
		return ret;
	}
	else {
		/* set flag for calibration lockup recovery if cal command was
		 * successful set flag to show we must still confirm if
		 * calibration was good or bad */
		mxt->cal_check_flag = 1u;
		/* 2012.08.02
		 * Request debugging log (ATMEL) */
		mxt_debug_info(mxt, "[TSP] Calibration request "
					   "by host!\n");
	}

	/* 2012_0510
	 *  - ATMEL : It needs 1ms to settle chip.
	 *  but mxt224 takes 10ms. the below code is work-around of the case.
	 *  if the below code does not make some problem, use it */
	msleep(120);

	return ret;
}

static void check_chip_palm(struct mxt_data *mxt)
{
	int tch_ch, atch_ch;
	struct i2c_client *client = mxt->client;

	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* Optimize_reading touch_flag & antitouch_flag from mxt540E to mxt336S
	* Add check_touch_flag func
	* Change facesup_message_flag condition
	**/

	if (debug >= DEBUG_INFO)
		dev_info(&mxt->client->dev, "%s\n", __func__);

	//check_touch_flag(mxt, &tch_ch, &atch_ch);
	check_touch_count(mxt, &tch_ch, &atch_ch);

	if ((tch_ch > 0) && (atch_ch > 0)) {
		/* multi-touch or palm */
		/* TBD in DAUDIO */
		palm_info.facesup_message_flag = 1;
	}
	else if ((tch_ch > 0) && (atch_ch == 0)) {
		/* common case(single touch + small sized palm)
		 * if tch_ch is normal */
		/* NORMAL PALM TOUCH in DAUDIO */
		palm_info.facesup_message_flag = 2;
	}
	else if ((tch_ch == 0) && (atch_ch > 0)) {
		/* abnoraml case */
		/* TODO : does it really need for us?? */
		palm_info.facesup_message_flag = 3;
		dev_warn(&client->dev, "%s() tch_ch:%d, atch_ch:%d\n",
				 __func__, tch_ch, atch_ch);
	}
	else {
		palm_info.facesup_message_flag = 4;
	}

	if (palm_info.facesup_message_flag == 1 || palm_info.facesup_message_flag == 2) {
		palm_info.palm_check_timer_flag = true;
		/**
		* @ legolamp@cleinsoft
		* @ date 2014.06.23
		* If normal palm touch condition is disable hardkey func
		**/
		if(check_hardkey_cnt > 2)
			maybe_good_count = 1;
		else
			maybe_good_count = 0;
	}
	mxt_debug_info(mxt, "[TSP] Touch suppression State:%d/%d\n",
				   palm_info.facesup_message_flag,
				   palm_info.palm_check_timer_flag ? 1 : 0);
}


static void check_chip_calibration_t100(u8 *message, struct mxt_data *mxt)
{
	uint8_t status;
	uint8_t finger_cnt = 0;
	uint16_t tch_area;
	uint16_t atch_area;

	uint8_t cal_thr = 10;
	uint8_t num_of_antitouch = 2;

#if 0
	sprintf(print_buff, "message:%02x %02x %02x %02x %02x %02x %02x %02x %02x", \
			message[0], message[1], message[2], message[3], message[4], message[5], message[6], message[7], message[8]);
	SerialUSB.println(print_buff);
#endif

	//0: ID
	//1: status
	status = message[1];
	//2: finger_cnt;
	finger_cnt = message[2];
	//3-4: tch_area
	tch_area = (message[3] | (message[4] << 8));
	//5-6: atch_area
	atch_area = (message[5] | (message[6] << 8));

	if(status == 0) {
		return;
	}

	if((tch_area) && (atch_area == 0)) {
		// normal
		if ((finger_cnt >= 2) && (tch_area <= 3)) {
			calibrate_chip(mxt);
		}
		else {
			not_yet_count = 0;
			cal_maybe_good(mxt);
		}
	}
	else if(atch_area > ((finger_cnt * num_of_antitouch) + 2)) {
		/* atch_ch is too many!! (finger_cnt) */
		calibrate_chip(mxt);
	}
	else if ((tch_area + cal_thr) <= atch_area) {
		//dev_info(&client->dev, "%s()\n FAIL2 tch_ch:%d, atch_ch:%d, cal_thr:%d", __func__, tch_ch, atch_ch, cal_thr);

		/* atch_ch is too many!! (except finger_cnt) */
		/* cal was bad - must recalibrate and check afterwards */
		calibrate_chip(mxt);
	}
	else {
		not_yet_count++;

		//mxt_debug_info(mxt, "[TSP] calibration was not decided yet, not_yet_count = %d\n", not_yet_count);

		if ((tch_area == 0) && (atch_area == 0)) {
			not_yet_count = 0;
		}
		else if (not_yet_count >= 3) {
			/* 2012_0517 : kykim 30 -> 3 */
			//dev_info(&client->dev, [TSP] not_yet_count over 3, re-calibrate!!\n");
			not_yet_count = 0;
			calibrate_chip(mxt);
		}
	}
}

static void check_chip_calibration(struct mxt_data *mxt)
{
	int tch_ch, atch_ch;
	uint8_t i;
	struct i2c_client *client = mxt -> client;
	uint8_t finger_cnt = 0;
	uint8_t cal_thr = 10;
	uint8_t num_of_antitouch = 2;

	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* Optimize_reading touch_flag & antitouch_flag from mxt540E to mxt336S
	* add check_touch_flag func
	**/

	if (debug >= DEBUG_INFO)
		dev_info(&mxt->client->dev, "%s\n", __func__);

	//check_touch_flag(mxt, &tch_ch, &atch_ch);
	check_touch_count(mxt, &tch_ch, &atch_ch);

	for (i = 0 ; i < MXT_MAX_NUM_TOUCHES ; ++i) {
		if ( mtouch_info[i].pressure == -1 )
			continue;
		finger_cnt++;
	}

	if (mxt->cal_check_flag != 1) {
		mxt_debug_info(mxt, "[TSP] check_chip_calibration just return!! finger_cnt = %d\n", finger_cnt);
		return;
	}

	if ((tch_ch) && (atch_ch == 0)) {
		/* normal */
		if ((finger_cnt >= 2) && (tch_ch <= 3)) {
			/* tch_ch is the number of node.
			 * if finger_cnt is over 2 and tch_ch is
			 * less than 3,
			 * this means something is wrong. */
			calibrate_chip(mxt);
			/* 100ms timer disable */
			ts_100ms_timer_clear(mxt);
		}
		else {
			cal_maybe_good(mxt);
			not_yet_count = 0;
		}

	}
	else if (atch_ch > ((finger_cnt * num_of_antitouch) + 2)) {
		dev_info(&client->dev, "%s()\n FAIL1 tch_ch:%d, "
				 "atch_ch:%d, finger_cnt:%d, "
				 "num_of_antitouch : %d\n",
				 __func__, tch_ch, atch_ch,
				 finger_cnt, num_of_antitouch);
		/* atch_ch is too many!! (finger_cnt) */
		calibrate_chip(mxt);
		/* 100ms timer disable */
		ts_100ms_timer_clear(mxt);
	}
	else if ((tch_ch + cal_thr) <= atch_ch) {
		dev_info(&client->dev, "%s()\n FAIL2 tch_ch:%d, "
				 "atch_ch:%d, cal_thr:%d", __func__,
				 tch_ch, atch_ch, cal_thr);
		/* atch_ch is too many!! (except finger_cnt) */
		/* cal was bad -
		 *	must recalibrate and check afterwards */
		calibrate_chip(mxt);
		/* 100ms timer disable */
		ts_100ms_timer_clear(mxt);
	}
	else {
		mxt->cal_check_flag = 1u;
		/* 2012_0517
		 * ATMEL - no need */
		not_yet_count++;
		mxt_debug_info(mxt, "[TSP] calibration was not decided"
					   " yet, not_yet_count = %d\n",
					   not_yet_count);
		if ((tch_ch == 0) && (atch_ch == 0)) {
			not_yet_count = 0;
		}
		else if (not_yet_count >= 3) {
			/* 2012_0517 : kykim 30 -> 3 */
			dev_info(&client->dev,
					 "[TSP] not_yet_count over "
					 "3, re-calibrate!!\n");
			not_yet_count = 0;
			calibrate_chip(mxt);
			/* 100ms timer disable */
			ts_100ms_timer_clear(mxt);
		}
	}
}

static void cal_maybe_good(struct mxt_data *mxt)
{
	mxt_debug_trace(mxt, "[TSP] %s()\n", __func__);
	/* Check if the timer is enabled */
	if (mxt->mxt_time_point != 0) {
		/* Check if the timer timedout of 0.3seconds */
		unsigned long t_next, t_now;
		struct i2c_client *client = mxt->client;
		t_now = jiffies;
		t_next = mxt->mxt_time_point + msecs_to_jiffies(300);
		if (time_after(t_now, t_next)) {
			dev_info(&client->dev, "(%d)good(%d)\n", maybe_good_count
					 , jiffies_to_msecs(t_now - mxt->mxt_time_point));
			/* Cal was good - don't need to check any more */
			mxt->mxt_time_point = 0;
			mxt->cal_check_flag = 0;
			/* Disable the timer */
			ts_100ms_timer_clear(mxt);
			mxt_check_touch_ic_timer_stop(mxt);
			mxt_check_touch_ic_timer_start(mxt);

		}
		else {
			mxt->cal_check_flag = 1u;
		}
	}
	else {
		mxt->cal_check_flag = 1u;
	}
}

#if TS_100S_TIMER_INTERVAL
/******************************************************************************/
/* 0512 Timer Rework by LBK                            */
/******************************************************************************/
static void ts_100ms_timeout_handler(unsigned long data)
{
	struct mxt_data *mxt = (struct mxt_data *)data;
	mxt->ts_100ms_tmr.p_ts_timeout_tmr = NULL;
	queue_work(ts_100s_tmr_workqueue, &mxt->ts_100ms_tmr.tmr_work);
}

static void ts_100ms_timer_start(struct mxt_data *mxt)
{
	if (mxt->ts_100ms_tmr.p_ts_timeout_tmr != NULL)
		del_timer(mxt->ts_100ms_tmr.p_ts_timeout_tmr);
	mxt->ts_100ms_tmr.p_ts_timeout_tmr = NULL;

	/* 100ms */
	mxt->ts_100ms_tmr.ts_timeout_tmr.expires =
		jiffies + msecs_to_jiffies(100);
	mxt->ts_100ms_tmr.p_ts_timeout_tmr = &mxt->ts_100ms_tmr.ts_timeout_tmr;
	add_timer(&mxt->ts_100ms_tmr.ts_timeout_tmr);
}

static void ts_100ms_timer_stop(struct mxt_data *mxt)
{
	if (mxt->ts_100ms_tmr.p_ts_timeout_tmr)
		del_timer(mxt->ts_100ms_tmr.p_ts_timeout_tmr);
	mxt->ts_100ms_tmr.p_ts_timeout_tmr = NULL;
}

static void ts_100ms_timer_init(struct mxt_data *mxt)
{
	init_timer(&(mxt->ts_100ms_tmr.ts_timeout_tmr));
	mxt->ts_100ms_tmr.ts_timeout_tmr.data = (unsigned long)(mxt);
	mxt->ts_100ms_tmr.ts_timeout_tmr.function = ts_100ms_timeout_handler;
	mxt->ts_100ms_tmr.p_ts_timeout_tmr = NULL;
}

static void ts_100ms_tmr_work(struct work_struct *work)
{
	struct mxt_100ms_timer *p100ms_tmr = NULL;
	struct mxt_data *mxt = container_of(work,
										struct mxt_data, ts_100ms_tmr.tmr_work);

	p100ms_tmr = &mxt->ts_100ms_tmr;
	p100ms_tmr->timer_ticks++;

	mxt_debug_trace(mxt, "[TSP] 100ms T %d\n", p100ms_tmr->timer_ticks);
	mutex_lock(&mxt->msg_lock);
	/* Palm but Not touch message */
	if (palm_info.facesup_message_flag) {
		mxt_debug_info(mxt, "[TSP] facesup_message_flag = %d\n",
					   palm_info.facesup_message_flag);
		check_chip_palm(mxt);
	}
	if ((p100ms_tmr->timer_flag == MXT_ENABLE) &&
			(p100ms_tmr->timer_ticks < p100ms_tmr->timer_limit)) {
		ts_100ms_timer_start(mxt);
		palm_info.palm_check_timer_flag = false;
	}
	else {
		int cal = 0;
		/* 2012_0523
		 *  - ATMEL : remove "palm_info.facesup_message_flag == 1" */
		if (palm_info.palm_check_timer_flag
				&& (palm_info.facesup_message_flag == 3)
				&& (palm_info.palm_release_flag == false)) {
			dev_info(&mxt->client->dev,
					 "[TSP](%s) calibrate_chip\n", __func__);
			calibrate_chip(mxt);
			palm_info.palm_check_timer_flag = false;
			cal = 1;
		}

		/* 2012_0521 :
		 *  - add the below code : there' is no way to recover if
		 *   the upper code don't filter the suppression */
		if (!cal && p100ms_tmr->timer_ticks > 0 &&
				(p100ms_tmr->timer_ticks >= p100ms_tmr->timer_limit)) {

			mxt_debug_trace(mxt, "go in suppression wait state\n");
			mxt_supp_ops_stop(mxt);
			schedule_delayed_work(&mxt->supp_ops.dwork,
								  msecs_to_jiffies(MXT_CHK_PALM_TOUCH_TIME));
		}

		p100ms_tmr->timer_flag = MXT_DISABLE;
		p100ms_tmr->timer_ticks = 0;
		p100ms_tmr->timer_limit = MXT_100MS_TIMER_LIMIT;
		p100ms_tmr->error_limit = 0;
	}
	mutex_unlock(&mxt->msg_lock);
}
#endif

static void ts_100ms_timer_clear(struct mxt_data *mxt)
{
	mxt->ts_100ms_tmr.timer_flag = MXT_DISABLE;
	mxt->ts_100ms_tmr.timer_ticks = 0;
	mxt->ts_100ms_tmr.timer_limit = MXT_100MS_TIMER_LIMIT;
	mxt->ts_100ms_tmr.error_limit = 0;
#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_stop(mxt);
#endif
}

static void ts_100ms_timer_enable(struct mxt_data *mxt)
{
	mxt->ts_100ms_tmr.timer_flag = MXT_ENABLE;
	mxt->ts_100ms_tmr.timer_ticks = 0;
	mxt->ts_100ms_tmr.timer_limit = MXT_100MS_TIMER_LIMIT;
	mxt->ts_100ms_tmr.error_limit = 0;
#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_start(mxt);
#endif
}

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int mxt336s_initialize(struct i2c_client *client, struct mxt_data *mxt)
{
	int error = 0;
	int retry_count = I2C_RETRY_COUNT;

	if (!client || !mxt) {
		dev_info(&client->dev, "client:%p, mxt:%p\n",
				 client, mxt);
		return -EINVAL;
	}

	error = mxt_identify(client, mxt);

	if (error < 0) {
#ifdef MXT_FIRMUP_ENABLE
		/* change to i2c slave address for boot mode
		 * this is for checking the upgrade failure.
		 * If module is shutdowned during upgrade.
		 *  module's i2c address may has boot-mode slave address
		 * */
		/* change to boot-mode i2c slave address */
		mxt_change_i2c_addr(mxt, true);
		error = mxt_identify(client, mxt);
		if (-EIO == error) {
			dev_err(&client->dev, "[TSP] ATMEL Chip could not "
					"be identified. error = %d\n", error);
			goto mxt_init_out;
		}
		else {
			/* back to the application mode
			 * It has tow possibilities, one is another atmel device
			 * , the other is the upgrade-failure */
			mxt_change_i2c_addr(mxt, false);
		}

		return MXT_INIT_FIRM_ERR;
#else
		dev_err(&client->dev, "[TSP] ATMEL Chip could not "
				"be identified. error = %d\n", error);
		goto mxt_init_out;
#endif
	}

	while (retry_count > 0) {
		error = mxt_read_object_table(client, mxt);
		if (error < 0)
			if (retry_count == 0)
				dev_err(&client->dev, "[TSP] ATMEL Chip could not read"
						" object table. error = %d\n", error);
			else
				retry_count--;
		else
			goto mxt_init_out;
	}

mxt_init_out:
#if defined(INCLUDE_TOUCH_MXT_1189T)
	if (error < 0) {
		printk("[buffalo]Touch driver is reloading...... \n");
		return MXT_INIT_FIRM_ERR;
	}
	else
#endif
		return error;
}

static int mxt_identify(struct i2c_client *client, struct mxt_data *mxt)
{
	u8 buf[7];
	int error;
	int identified;
	int retry_count = I2C_RETRY_COUNT;
	int val;

	identified = 0;

retry_i2c:
	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, 7, (u8 *)buf);

	if (error < 0) {
		mxt->read_fail_counter++;
		if (retry_count--) {
			dev_info(&client->dev, "[TSP] Warning: "
					 "To wake up touch-ic in "
					 "deep sleep, retry i2c "
					 "communication!");
			msleep(30);  /* delay 30ms */
			goto retry_i2c;
		}
		dev_err(&client->dev, "[TSP] Failure accessing "
				"maXTouch device\n");
		return -EIO;
	}

	mxt->device_info.family_id  = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major	    = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor      = (buf[2] & 0x0F);
	mxt->device_info.build	    = buf[3];
	mxt->device_info.x_size	    = buf[4];
	mxt->device_info.y_size	    = buf[5];
	mxt->device_info.num_objs   = buf[6];
	mxt->device_info.num_nodes =
		mxt->device_info.x_size * mxt->device_info.y_size;


	dev_info(&mxt->client->dev,
			 "[TSP] Atmel Family ID[0x%X] variant ID[0x%X] major[0x%X] minor[0x%X] build [0x%X]\n",
			 mxt->device_info.family_id,
			 mxt->device_info.variant_id,
			 mxt->device_info.major,
			 mxt->device_info.minor,
			 mxt->device_info.build);

	if (mxt->device_info.family_id == MAXTOUCH_FAMILYID) {
		printk("%s T family : 0x%X\n",__func__,mxt->device_info.family_id);
		strcpy(mxt->device_info.family, maxtouch_family);
	}
	else if (mxt->device_info.family_id == MAXTOUCH_FAMILYID_S) {
		printk("%s S family : 0x%X\n",__func__,mxt->device_info.family_id);
                strcpy(mxt->device_info.family, maxtouch_family);
        }
	else {
		dev_err(&client->dev,
				"[TSP] maXTouch Family ID [0x%x] not supported\n",
				mxt->device_info.family_id);
		identified = -ENXIO;
	}

	val = mxt->device_info.major * 10 + mxt->device_info.minor;

	if (	((mxt->device_info.variant_id == MXT1189T_FIRM_VER1_VARIANTID)) ||
		((mxt->device_info.variant_id == MXT1189T_FIRM_VER2_VARIANTID))) {
		touch_type = 5;
		printk("%s MXT1189T, touch_type : %d\n",__func__,touch_type);
		sprintf(mxt->device_info.variant, "%s[%02X]", mxt336S_variant,
				mxt->device_info.variant_id);
	}
	else if (    ((mxt->device_info.variant_id == MXT641T_FIRM_VER1_VARIANTID)) ||
                ((mxt->device_info.variant_id == MXT641T_FIRM_VER2_VARIANTID))) {
		touch_type = 6;
		printk("%s MXT641T, touch_type : %d\n",__func__,touch_type);
                sprintf(mxt->device_info.variant, "%s[%02X]", mxt336S_variant,
                                mxt->device_info.variant_id);
        }
	else if (    ((mxt->device_info.variant_id == MXT336S_FIRM_VER1_VARIANTID)) ||
                ((mxt->device_info.variant_id == MXT336S_FIRM_VER2_VARIANTID))) {
		touch_type = 7;
		printk("%s MXT336S, touch_type : %d\n",__func__,touch_type);
                sprintf(mxt->device_info.variant, "%s[%02X]", mxt336S_variant,
                                mxt->device_info.variant_id);
        }
	else {
		dev_err(&client->dev,
				"[TSP] maXTouch Variant ID [0x%x] not supported\n",
				mxt->device_info.variant_id);
		identified = -ENXIO;
	}

	mxt_debug_msg(mxt,
				  "[TSP] Atmel %s.%s Firmware version [%d.%d] Build:[%d]\n",
				  mxt->device_info.family,
				  mxt->device_info.variant,
				  mxt->device_info.major,
				  mxt->device_info.minor,
				  mxt->device_info.build);
	mxt_debug_msg(mxt,
				  "[TSP] Atmel %s.%s Configuration [X: %d] x [Y: %d]\n",
				  mxt->device_info.family,
				  mxt->device_info.variant,
				  mxt->device_info.x_size,
				  mxt->device_info.y_size);
	mxt_debug_msg(mxt, "[TSP] number of objects: %d\n",
				  mxt->device_info.num_objs);

	return identified;
}

/*
* Reads the object table from maXTouch chip to get object data like
* address, size, report id.
*/
static int mxt_read_object_table(struct i2c_client *client,
								 struct mxt_data *mxt)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     crc_calculated;
	int	i;
	int	error;

	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;

	struct mxt_object *object_table;

	mxt_debug_trace(mxt, "[TSP] maXTouch driver get configuration\n");

	if(mxt->object_table != NULL) {
		kfree(mxt->object_table);
		mxt->object_table = NULL;
	}
	mxt->object_table = kzalloc(sizeof(struct mxt_object) *
								mxt->device_info.num_objs,
								GFP_KERNEL);
	if (mxt->object_table == NULL) {
		dev_err(&mxt->client->dev,
				"[TSP] maXTouch: Memory allocation failed!\n");
		return -ENOMEM;
	}
	object_table = mxt->object_table;

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		mxt_debug_trace(mxt, "[TSP] <%d>Reading maXTouch at [0x%04x]: ",
						i, object_info_address);

		error = mxt_read_block(client, object_info_address,
							   MXT_OBJECT_TABLE_ELEMENT_SIZE, (u8 *)buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
					"[TSP] maXTouch Object %d "
					"could not be read\n", i);
			return -EIO;
		}

		object_type		=  buf[0];
		object_address		= (buf[2] << 8) + buf[1];
		object_size		=  buf[3] + 1;
		object_instances	=  buf[4] + 1;
		object_report_ids	=  buf[5];

		mxt_debug_trace(mxt, "[TSP] Type=%03d, Address=0x%04x, "
						"Size=0x%02x, %d instances, %d report id's\n",
						object_type,
						object_address,
						object_size,
						object_instances,
						object_report_ids);

		if (object_type > MXT_MAX_OBJECT_TYPES) {
			/* Unknown object type */
			dev_err(&client->dev,
					"[TSP] maXTouch object type [%d] "
					"not recognized\n", object_type);
			VPRINTK("%s object[%d] type: %d address: %d size: %d instances: %d report_ids: %d\n"
					, __func__, (i + 1), object_type, object_address, object_size,
					object_instances, object_report_ids);
			return -ENXIO;
		}

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;
		mxt->object_link[object_type] = i;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	if(mxt->rid_map != NULL) {
		kfree(mxt->rid_map);
		mxt->rid_map = NULL;
	}
	/* allocate for report_id 0, even if not used */
	mxt->rid_map = kzalloc(sizeof(struct report_id_map) * \
						   (report_id_count + 1), GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		pr_warning("[TSP] maXTouch: Can't allocate memory!\n");
		return -ENOMEM;
	}

	if(mxt->last_message != NULL) {
		kfree(mxt->last_message);
		mxt->last_message = NULL;
	}
	mxt->last_message = kzalloc(mxt->message_size, GFP_KERNEL);
	if (mxt->last_message == NULL) {
		pr_warning("[TSP] maXTouch: Can't allocate memory!\n");
		return -ENOMEM;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
		dev_err(&client->dev,
				"[TSP] Too many maXTouch report id's [%d]\n",
				report_id_count);
		return -ENXIO;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
				object_instance < object_table[i].instances;
				object_instance++) {
			first_report_id = report_id;
			for (object_report_id = 0;
					object_report_id <
					object_table[i].num_report_ids;
					object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, MXT_CRC_NUM, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "[TSP] Error reading CRC\n");
		return -EIO;
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	calculate_infoblock_crc(&crc_calculated, mxt);

	mxt_debug_trace(mxt, "[TSP] Reported info block CRC = 0x%6X\n\n", crc);
	mxt_debug_trace(mxt, "[TSP] Calculated info block CRC = 0x%6X\n\n",	crc_calculated);
	if (crc == crc_calculated)
		mxt->info_block_crc = crc;
	else {
		mxt->info_block_crc = 0;
		dev_err(&mxt->client->dev, "[TSP] maXTouch: info block CRC invalid!\n");
	}

	if (0) {
		dev_info(&client->dev, "[TSP] maXTouch: %d Objects\n",
				 mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "[TSP] Type:\t\t\t[%d]: %s\n", object_table[i].type,
					 object_type_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",	object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",	object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",	object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",	object_table[i].num_report_ids);
		}
	}

	return 0;
}

#define MXT_ADR_T38_CFG_CONF_POS	MXT_ADR_T38_USER0
#define MXT_ADR_T38_CFG_CONF_SIZE	8
#define MXT_CONFIG_STRING			"XX.XXX.XX."
#define MXT_CONFIG_STR_LENG			8

static int mxt_get_version(struct mxt_data *mxt, struct atmel_touch_version *ver)
{
	int ret = 0;
	int retry = I2C_RETRY_COUNT, version = 0;
	struct i2c_client *client = mxt->client;

	/* firmware version */
	version = mxt->device_info.major * 10;
	version += mxt->device_info.minor;
	sprintf(ver->firm, "%03d", version);

	/* configuration version */
	/* see T38_USERDATA0 of device_config_mxt336S.h */
	do {
		ret = mxt_read_block(client, MXT_BASE_ADDR(MXT_USER_INFO_T38)
							 + MXT_ADR_T38_CFG_CONF_POS,
							 MXT_ADR_T38_CFG_CONF_SIZE, ver->conf);
		if (!ret)
			break;
	}
	while (retry--);

	ver->conf[MXT_ADR_T38_CFG_CONF_SIZE] = '\0';
	if (ret < 0) {
		dev_err(&client->dev, "%s() ret=%d\n", __func__, ret);
		sprintf(ver->conf, "XX.XXX.XX.XXXXXX.FAIL");
	}

	return ret;
}

static int mxt_convert_version(const char *buff)
{
	long version = 0;

	char *end = NULL;
	version = simple_strtoul(buff, &end, 10);

	return version;
}

extern unsigned char cfg_1189T_T38[64];
extern unsigned char cfg_641T_T38[64];
static int mxt_get_conf_version(struct mxt_data *mxt)
{
	char buff[9] = {0, };

	printk("%s, touch_type : %d\n", __func__, touch_type);

	if (touch_type >= TOUCH_TYPE_MAX) {
		printk(KERN_ERR "[%s] touch_type is (%d) \r\n", __func__, touch_type);
		touch_type = 0;
	}

	switch(touch_type){
		case 5 :
			memcpy(buff, cfg_1189T_T38, 8);
			break;
		case 6 :
			memcpy(buff, cfg_641T_T38, 8);
			break;
		case 7 :
			buff[0] = t38_userdata0[touch_type];
		        buff[1] = t38_userdata1[touch_type];
		        buff[2] = t38_userdata2[touch_type];
		        buff[3] = t38_userdata3[touch_type];
		        buff[4] = t38_userdata4[touch_type];
		        buff[5] = t38_userdata5[touch_type];
		        buff[6] = t38_userdata6[touch_type];
		        buff[7] = t38_userdata7[touch_type];
		        buff[8] = '\0';
			break;
		default :
			printk("%s wrong touch type %d\n",__func__,touch_type);
			break;
	}	

	return mxt_convert_version(buff);
}

static void mxt_update_backup_nvram(
	struct mxt_data *mxt, struct atmel_touch_version *old_ver)
{
	int error;
	int old_version = 0, new_version = 0;

	if (old_ver->conf[17] == 'F' &&
			old_ver->conf[18] == 'A') {
		/* failed to read the configuration version */
		return;
	}

	/* '\0' means that this is the first time.
	 *  so, write the configuration to the NVRAM */
	if (old_ver->conf[0] == '\0') {
		mxt_debug_info(mxt, "[TSP] nvram is clear\n");
		backup_to_nv(mxt);
		return;
	}

	mxt_debug_trace(mxt, "[TSP] prev conf-version:%s\n", old_ver->conf);

	/* get the current version from T38 */
	error = mxt_get_version(mxt, &new_ver);
	if (!error) {
		char *end = NULL;

		/* old version */
		if (!strncmp(old_ver->conf,
					 ""MXT_CONFIG_STRING"",
					 MXT_CONFIG_STR_LENG)) {
			old_version = simple_strtoul(
							  old_ver->conf + MXT_CONFIG_STR_LENG,
							  &end, 10);
			old_version *= 100;
			old_version += simple_strtoul(end + 1, &end, 10);
		}
		/* new version */
		if (!strncmp(new_ver.conf, ""MXT_CONFIG_STRING"",
					 MXT_CONFIG_STR_LENG)) {
			new_version = simple_strtoul(
							  new_ver.conf + MXT_CONFIG_STR_LENG,
							  &end, 10);
			new_version *= 100;
			new_version += simple_strtoul(end + 1, &end, 10);
		}

		mxt_debug_trace(mxt,
						"[TSP] new conf-version:%s\n", new_ver.conf);

		if (new_version > old_version) {
			/* if new_version is higher than old_version,
			 * write the configuration to the NVRAM */
			mxt_debug_info(mxt, "backup to nvram\n");
			backup_to_nv(mxt);
		}
	}
}
static int mxt_write_init_registers(struct mxt_data *mxt)
{
	int error;
	int drv_version = 0, nvm_version = 0;
	uint32_t drv_crc;
	uint32_t nvm_conf_checksum = 0;
	u32 retry_loop = 0;
	struct i2c_client *client = mxt->client;

	//printk("%s, touch_type : %d\n",__func__, touch_type);

	/* get the current version from T38 registers */
	error = mxt_get_version(mxt, &nvram_ver);
	if (error < 0) {
		dev_err(&client->dev, "failed to read version\n");
		return error;
	}
	/* driver's configuration version */
	drv_version = mxt_get_conf_version(mxt);
	/* nvram's configuration version */
	nvm_version = mxt_convert_version(nvram_ver.conf);

	mxt_read_config_crc(mxt, &nvm_conf_checksum);
	//VPRINTK("nvm_conf_checksum=0x%X\n", nvm_conf_checksum);

	calculate_drv_config_crc(&drv_crc, mxt);
	VPRINTK("checksum:nvm=0x%X,drv=0x%X\n", nvm_conf_checksum, drv_crc);
	VPRINTK("version:nvm=%d,drv=%d,touch_type=%d\n",
			nvm_version, drv_version, touch_type);

	//if ((drv_version != nvm_version) || (drv_crc != nvm_conf_checksum)) {
	if (drv_version != nvm_version) {
mxt_setting_loop:
		error = mxt_config_settings(mxt, NULL, TODEV, touch_type);

		if (error < 0) {
			dev_err(&client->dev, "[TSP] (%s):"
					"failed to update config\n", __func__);
			return error;
		}
		backup_to_nv(mxt);

		mxt_read_config_crc(mxt, &nvm_conf_checksum);
		VPRINTK("nvm_conf_checksum=0x%X\n", nvm_conf_checksum);
		if(drv_crc != nvm_conf_checksum) {
			VPRINTK("backup_to_nv verify error !!!(nvm_conf_checksum=0x%X, drv_crc=0x%X(retry=%d)\n", nvm_conf_checksum, drv_crc, retry_loop + 1);
			retry_loop++;
			if(retry_loop < 3)
				goto mxt_setting_loop;
		}

		sw_reset_chip(mxt, RESET_TO_NORMAL);
		msleep(100);
	}
	else
		VPRINTK("[%s] config setting is recent version.\n", __func__);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h)
{
	struct	mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);

#ifndef MXT_SLEEP_POWEROFF
	u8 cmd_sleep[2] = { 0};
	u16 addr;
#endif

	mxt_debug_info(mxt, "[TSP] mxt_early_suspend has been called!");
#if defined(MXT_FIRMUP_ENABLE)
	/*start firmware updating : not yet finished*/
	while (MXT_FIRM_UPGRADING(mxt->firm_status_data)) {
		mxt_debug_info(mxt, "[TSP] mxt firmware is Downloading : "
					   "mxt suspend must be delayed!");
		msleep(1000);
	}
#endif
	disable_irq(mxt->client->irq);
	disable_irq(mxt->jig_irq);
#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_stop(mxt);
#endif
	mxt_check_touch_ic_timer_stop(mxt);
	mxt_supp_ops_stop(mxt);
	mxt_release_all_fingers(mxt);
	mxt_release_all_keys(mxt);

	/* global variable initialize */
	mxt->ts_100ms_tmr.timer_flag = MXT_DISABLE;
	mxt->ts_100ms_tmr.timer_ticks = 0;
	mxt->mxt_time_point = 0;
#ifdef MXT_SLEEP_POWEROFF
	if (mxt->pdata->suspend_platform_hw != NULL)
		mxt->pdata->suspend_platform_hw(mxt->pdata);
#else
	/*
	* a setting of zeros to IDLEACQINT and ACTVACQINT
	* forces the chip set to enter Deep Sleep mode.
	*/
	addr = get_object_address(MXT_GEN_POWERCONFIG_T7,
							  0, mxt->object_table, mxt->device_info.num_objs,
							  mxt->object_link);
	mxt_debug_info(mxt, "[TSP] addr: 0x%02x, buf[0]=0x%x, buf[1]=0x%x",
				   addr, cmd_sleep[0], cmd_sleep[1]);
	mxt_write_block(mxt->client, addr, 2, (u8 *)cmd_sleep);
#endif
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct	mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);

	mxt_debug_info(mxt, "[TSP] mxt_late_resumehas been called!");

	mxt336s_resume_reset(mxt->client);

	if( MXT_FIRM_STATUS_FAIL == mxt->firm_status_data ) {
		dev_err(&mxt->client->dev, "[TSP] MXT FIRM STATUS FAIL. resume reset.\n");
		mxt336s_resume_reset(mxt->client);

		if( MXT_FIRM_STATUS_FAIL != mxt->firm_status_data ) {
			enable_irq(mxt->irq);
		}
	}

#if defined(CONFIG_DISASSEMBLED_MONITOR)
	{
		unsigned short addr = mxt->client->addr;

		printk(KERN_ERR "[%s:%d] Serdes Device(0x64), Register(0x0C), value(0xA0) \r\n", __func__, __LINE__);
		mxt->client->addr = 0x64;
		mxt_write_onebyte_register(mxt->client, 0x0c, 0xA0);
		mxt->client->addr = addr;
	}
#endif

#ifdef MXT_SLEEP_POWEROFF
	if (mxt->pdata->resume_platform_hw != NULL)
		mxt->pdata->resume_platform_hw(mxt->pdata);
#else
	for (cnt = 10; cnt > 0; cnt--) {
		if (mxt_power_config(mxt) < 0)
			continue;
		if (sw_reset_chip(mxt, RESET_TO_NORMAL) == 0)  /* soft reset */
			break;
	}
	if (cnt == 0) {
		pr_err("[TSP] mxt_late_resume failed!!!");
		return;
	}
#endif

	/* metal_suppression_chk_flag = true; */
	/* mxt->mxt_status = true; */

	if( NULL != mxt->object_table ) {
		mxt_write_init_registers(mxt);
	}
	enable_irq(mxt->client->irq);
	enable_irq(mxt->jig_irq);

	/**
	* @ legolamp@cleinsoft
	* @ date 2014.06.03
	* Add mxt_check_touch_ic_timer_start - resume mode
	**/
	mxt_check_touch_ic_timer_start(mxt);
}
#endif

static ssize_t mxt336S_fops_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	VPRINTK("%s\n", __func__);
	return 0;
}

static ssize_t mxt336S_fops_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	VPRINTK("%s\n", __func__);
	return 0;
}

static int mxt336S_fops_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxt336S_fops_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mxt336S_fops_ioctl(struct file *file,
							   unsigned int cmd, unsigned long _arg)
{
	int ret = 0;
	unsigned int value;
	struct mxt_data *mxt = &g_mxt;
	struct atmel_touch_version ver;
	u16 addr;

	if (NULL == mxt->object_table || check_jig_detection(mxt))
		return 0;

	if (_IOC_TYPE(cmd) != DAUDIO_TOUCH_IOC_MAGIC_V1) {
		pr_info("%s(): this is not mine.\n", __func__);
		//return -ENOTTY;
	}

	switch (cmd) {

		case DAUDIO_TOUCHIOC_S_MODE_VERSION:
			/* call this for the firmware version */
			disable_irq(mxt->irq);

			mxt_get_version(mxt, &ver);

			ret = copy_to_user((void __user *)_arg,
							   &ver, sizeof(struct atmel_touch_version));

			enable_irq(mxt->irq);
			if (ret)
				ret = -EFAULT;
			break;

		case DAUDIO_TOUCHIOC_S_MODE_LOCK:
			/* call this before firmware-update */
			//mxt->last_read_addr = -1;
			mxt_check_touch_ic_timer_stop(mxt);
			mxt_prerun_reset(mxt, true);
			break;

		case DAUDIO_TOUCHIOC_S_MODE_UNLOCK:
			/* call this after firmware-update */
			mxt_check_touch_ic_timer_start(mxt);
			enable_irq(mxt->irq);
			break;

		case DAUDIO_TOUCHIOC_DISABLE_AUTO_CAL:
			/* Dong-Ui-Ham(Accept Button) */
			VPRINTK("[%s] Disable Auto_cal %d\n", __func__, mxt->check_auto_cal_flag);
			/* auto-cal disable */
			_disable_auto_cal(mxt, false);
			mxt->check_auto_cal_flag = AUTO_CAL_DISABLE;
			break;

		case DAUDIO_TOUCHIOC_CHECK_RE_CAL:
			if(debug >= DEBUG_MESSAGES)
				VPRINTK("[%s] Check re-calibration\n", __func__);
			if ((mxt->check_auto_cal_flag == AUTO_CAL_DISABLE) && (maybe_good_count == 0)) {
				cancel_delayed_work(&mxt->cal_timer.dwork);
				queue_delayed_work(ts_100s_tmr_workqueue, &mxt->cal_timer.dwork,
								   msecs_to_jiffies(200));
			}
			break;

		case DAUDIO_TOUCHIOC_SENSITIVITY:
			addr = MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T62);
			value = (unsigned int)_arg;
			if (value < 16) {
				mutex_lock(&mxt->msg_lock);
				mxt_write_byte(mxt->client,
							   addr + 0x27, value + 32);
				mutex_unlock(&mxt->msg_lock);
			}
			else {
				pr_info("Out of range : (%u)\n", value);
				return -EFAULT;
			}
			break;

		case DAUDIO_TOUCHIOC_REFERENCES_TEST:
			/* call this for the Reference test */
			disable_irq(mxt->irq);
			ret = do_show_references(mxt);

			switch (ret) {
				case MXT_SELF_TEST_SUCCESS:
					sprintf(test.references, "%d", MXT_SELF_TEST_SUCCESS);
					break;
				case MXT_SELF_TEST_FAIL:
					sprintf(test.references, "%d", MXT_SELF_TEST_FAIL);
					break;
				case MXT_SELF_TEST_RETRY:
					sprintf(test.references, "%d", MXT_SELF_TEST_RETRY);
					break;
			}

			if(copy_to_user((void __user *)_arg,
							&test, sizeof(struct atmel_touch_self_test)))
				return -EFAULT;

			enable_irq(mxt->irq);
			break;

		case DAUDIO_TOUCHIOC_PINFAULT_TEST:
			/* call this for the Pinfault test */
			disable_irq(mxt->irq);

			ret = do_show_pinfault(mxt);

			if (ret < 0)
				VPRINTK("T25 Write Init Data Fail\n");
			else
				VPRINTK("T25 Write Init Data Success \n");
			enable_irq(mxt->irq);
			if(!wait_for_completion_timeout(&pinfault_done, msecs_to_jiffies(1000))) {
				VPRINTK("Pinfault TEST Timeout!!!\n");
				sprintf(test.pinfault, "%d", MXT_SELF_TEST_TIMEOUT);
			}

			if(copy_to_user((void __user *)_arg,
							&test, sizeof(struct atmel_touch_self_test)))
				return -EFAULT;

			//enable_irq(mxt->irq);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}


static const struct file_operations mxt336S_misc_fops = {
	/**
	* @ legolamp@cleinsoft
	* @ date 2014.05.01
	* @ add mxt336S_fops_write, mxt336S_fops_read, mxt336S_fops_ioctl
	**/
	.unlocked_ioctl = mxt336S_fops_ioctl,
	.write			= mxt336S_fops_write,
	.read 			= mxt336S_fops_read,
	.open 			= mxt336S_fops_open,
	.release 		= mxt336S_fops_release,
};

static struct miscdevice mxt336S_misc = {
	.name = "mxt_misc",
	.fops = &mxt336S_misc_fops,
	.minor = MISC_DYNAMIC_MINOR,

	.mode = 0660,
};

#ifdef CONFIG_OF
static struct mxt_platform_data *mxt336s_parse_dt(struct i2c_client *client) {
	struct mxt_platform_data *pdata;
	struct device_node *np;
	int gpio;

	if (!client->dev.of_node)
		return ERR_PTR(-ENODEV);

	np = client->dev.of_node;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	gpio = of_get_named_gpio(np, "int-gpios", 0);
	if (!gpio_is_valid(gpio)) {
		pr_err("Failed to get gpio: %d\n", gpio);
		goto err;
	}
	pdata->irq_gpio = gpio;

	gpio = of_get_named_gpio(np, "jig-gpios", 0);
	if (!gpio_is_valid(gpio)) {
		pr_err("Failed to get gpio: %d\n", gpio);
		goto err;
	}
	pdata->irq_jig_gpio = gpio;
	pdata->irq_jig_num	= gpio_to_irq(pdata->irq_jig_gpio);

#if defined(HDMI_1920_720_12_3)
        pdata->max_x = 1919;
        pdata->max_y = 719;
#else
#if defined(INCLUDE_LCD_RESOLUTION_1280_720)
        pdata->max_x = 1279;
        pdata->max_y = 719;
#else
	of_property_read_u32(np, "max-x", &pdata->max_x);
	of_property_read_u32(np, "max-y", &pdata->max_y);
#endif
#endif

	pdata->gpio_reset = of_get_named_gpio(np, "por-gpios", 0);
	pdata->gpio_scl = of_get_named_gpio(np, "scl-gpios", 0);
	pdata->gpio_sda = of_get_named_gpio(np, "sda-gpios", 0);

	pr_info("irq=%x, jig=%x, reset=%x, scl=%x, sda=%x, max-x=%d, max-y=%d\n",
			pdata->irq_gpio, pdata->irq_jig_gpio, pdata->gpio_reset, pdata->gpio_scl, pdata->gpio_sda, pdata->max_x, pdata->max_y);

	return pdata;

err:
	if (pdata) {
		devm_kfree(&client->dev, pdata);
	}
	return NULL;
}
#else
static struct mxt_platform_data *mxt336s_parse_dt(struct i2c_client *client) {
	struct mxt_platform_data *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->irq_gpio		= TCC_GPF(21);
	pdata->irq_jig_gpio	= TCC_GPG(4);
	pdata->irq_jig_num	= gpio_to_irq(pdata->irq_jig_gpio);
	pdata->max_x = 799;
	pdata->max_y = 479;

	pdata->gpio_reset = TCC_GPF(20);
	pdata->gpio_scl = TCC_GPG(16);
	pdata->gpio_sda = TCC_GPG(15);

	pr_info("irq=%x, jig=%x, reset=%x, scl=%x, sda=%x\n",
			pdata->irq_gpio, pdata->irq_jig_gpio, pdata->gpio_reset, pdata->gpio_scl, pdata->gpio_sda);

	return pdata;
}
#endif

static int mxt336s_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	struct mxt_data          *mxt = &g_mxt;
	struct mxt_platform_data *pdata;
	struct input_dev         *input;
	int error;
	int i;

	client->dev.platform_data = dev_get_platdata(&client->dev);
	if (!client->dev.platform_data) {
		client->dev.platform_data = mxt336s_parse_dt(client);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* Initialize Platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "[TSP] platform data is required!\n");
		return -EINVAL;
	}
	mxt->pdata = pdata;
	client->irq = gpio_to_irq(pdata->irq_gpio);

	VPRINTK("driver name:%s, addr:%04x, irq:%d\n", client->name, client->addr, client->irq);

	init_completion(&pinfault_done);
	mxt->client = client;
	hw_reset_gpio(mxt);

	msleep(103);
	VPRINTK("[%s] reset high !!!\n", __func__ );

	/* Allocate structure - we need it to identify device */
	// mxt->last_read_addr = -1;
	mxt->jig_detected = 0;
	input = input_allocate_device();
	if (!mxt || !input) {
		dev_err(&client->dev, "[TSP] insufficient memory\n");
		error = -ENOMEM;
		goto err_after_kmalloc;
	}
#if defined(INCLUDE_LCD_TOUCHKEY)
	mxt->input_key = input_allocate_device();
	if (!mxt->input_key) {
		dev_err(&client->dev, "[TSP] insufficient memory\n");
		error = -ENOMEM;
		goto err_after_kmalloc;
	}
#endif
	mxt->message_counter   = 0;
	// mxt->client = client;
	mxt->input  = input;

	/* Default-configuration for auto-cal is 'enable' */
	mxt->check_auto_cal_flag = AUTO_CAL_ENABLE;

	if (pdata->init_platform_hw)
		pdata->init_platform_hw(mxt);

	/* mxt336S regulator config */
	i2c_set_clientdata(client, mxt);
	MXT_FIRM_CLEAR(mxt->firm_status_data);
	error = mxt336s_initialize(client, mxt);
	if (error < 0) {
		printk(KERN_ERR "[%s:%d] mxt336s_initialize error (%d) \r\n", __func__, __LINE__, error);
		goto err_after_get_regulator;
	}
#ifdef MXT_FIRMUP_ENABLE
	else if (error == MXT_INIT_FIRM_ERR) {
		printk(KERN_ERR "[%s:%d] mxt336s_initialize, MXT_INIT_FIRM_ERR error (%d) \r\n", __func__, __LINE__, error);
		dev_info(&client->dev, "[TSP] invalid device or firmware crash!\n");
		mxt->firm_status_data = MXT_FIRM_STATUS_FAIL;
		mxt_initialize_device = false;
		mxt->msg_proc_addr = 0xffff;
//		goto err_after_get_regulator;
	}
#endif

#if !defined(INCLUDE_TOUCH_MXT_1189T) && defined(CONFIG_DISASSEMBLED_MONITOR)
	{
		unsigned short addr = mxt->client->addr;

		printk(KERN_ERR "[%s:%d] Serdes Device(0x64), Register(0x0C), value(0xA0) \r\n", __func__, __LINE__);
		mxt->client->addr = 0x64;
		mxt_write_onebyte_register(mxt->client, 0x0c, 0xA0);
		mxt->client->addr = addr;
	}
#endif

	init_waitqueue_head(&mxt->msg_queue);
	sema_init(&mxt->msg_sem, 1);
#ifndef MXT_THREADED_IRQ
	spin_lock_init(&mxt->lock);
#endif
	mutex_init(&mxt->reg_update.lock);
	mutex_init(&mxt->msg_lock);

	snprintf(mxt->phys_name,
			 sizeof(mxt->phys_name),
			 "%s/input0",
			 dev_name(&client->dev));

	input->name = "mxt336s";
	input->phys = mxt->phys_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

#if defined(INCLUDE_LCD_TOUCHKEY)
	snprintf(mxt->phys_name_key,
			 sizeof(mxt->phys_name_key),
			 "%s/input0",
			 dev_name(&client->dev));
	mxt->input_key->name = "mxt1189t";
	mxt->input_key->phys = mxt->phys_name_key;
	mxt->input_key->id.bustype = BUS_I2C;
	mxt->input_key->dev.parent = &client->dev;
#endif
	if (debug >= DEBUG_INFO) {
		dev_info(&client->dev,
				 "[TSP] maXTouch name: \"%s\"\n", input->name);
		dev_info(&client->dev,
				 "[TSP] maXTouch phys: \"%s\"\n", input->phys);
		dev_info(&client->dev, "[TSP] maXTouch driver "
				 "setting abs parameters\n");
	}
	/* single touch */

	input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS);
	input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
#if defined(INCLUDE_LCD_TOUCHKEY)
	mxt->input_key->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	for (i = 0; tsp_keys[i].id != TOUCH_KEY_NULL; i++) {
		set_bit(tsp_keys[i].keycode, mxt->input_key->keybit);
	}
#endif
	/* multi touch */
	input_mt_init_slots(input, 2, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, mxt->pdata->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, mxt->pdata->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES - 1, 0, 0);

	input_set_drvdata(input, mxt);

	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev,
				"[TSP] Failed to register input device\n");
		goto err_after_get_regulator;
	}
#if defined(INCLUDE_LCD_TOUCHKEY)
	error = input_register_device(mxt->input_key);
	if (error < 0) {
		dev_err(&client->dev,
				"[TSP] Failed to register input device\n");
		goto err_after_input_register;
	}
#endif

	if (MXT_FIRM_STABLE(mxt->firm_status_data)
			&& mxt_write_init_registers(mxt) < 0) {
		printk(KERN_ERR "[%s:%d] \r\n", __func__, __LINE__);
//		goto err_after_input_register;
	}


	/* _SUPPORT_MULTITOUCH_ */
	for (i = 0; i < MXT_MAX_NUM_TOUCHES ; i++)
		mtouch_info[i].pressure = -1;

	error = misc_register(&mxt336S_misc);
	if (error < 0) {
		dev_err(&client->dev, "[TSP] failed to register"
				" misc driver(%d)\n", error);
		goto err_after_input_register;
	}

	/* Allocate the interrupt */
	mxt->irq = client->irq;
	mxt->irq_counter = 0;
	if (mxt->irq) {
		/* Try to request IRQ with falling edge first. This is
		 * not always supported. If it fails, try with any edge. */
#ifdef MXT_THREADED_IRQ
		error = request_threaded_irq(mxt->irq,
									 NULL,
									 mxt_threaded_irq,
									 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
									 client->dev.driver->name,
									 mxt);
		if (error < 0) {
			error = request_threaded_irq(mxt->irq,
										 NULL,
										 mxt_threaded_irq,
										 IRQF_DISABLED,
										 client->dev.driver->name,
										 mxt);
		}

		if( MXT_FIRM_STATUS_FAIL == mxt->firm_status_data ) {
			dev_err(&client->dev, "[TSP] MXT FIRM STATUS FAIL; disable_irq.\n");
			disable_irq(mxt->irq);
		}
#else
		mxt->valid_irq_counter = 0;
		mxt->invalid_irq_counter = 0;
		error = request_irq(mxt->irq,
							mxt_irq_handler,
							IRQF_TRIGGER_FALLING,
							client->dev.driver->name,
							mxt);
		if (error < 0) {
			error = request_irq(mxt->irq,
								mxt_irq_handler,
								0,
								client->dev.driver->name,
								mxt);
		}
#endif

		/**
		 * @ legolamp@cleinsoft
		 * @ date 2014.11.20
		 * Allocate the jig interrupt
		 **/
#ifdef MXT_JIG_DETECT
		mxt->jig_irq = mxt->pdata->irq_jig_num;
		if (mxt->jig_irq) {
			error = request_irq(mxt->jig_irq,
								mxt_jig_irq_handler,
								IRQ_TYPE_EDGE_FALLING | IRQF_DISABLED,
								"JIG_IRQ42",
								mxt);
			if (error < 0) {
				error = request_irq(mxt->jig_irq,
									mxt_jig_irq_handler,
									0,
									"JIG_IRQ42",
									mxt);
			}
		}
#endif
		if (error < 0) {
			dev_err(&client->dev,
					"[TSP] failed to allocate irq %d\n",
					mxt->irq);
			goto err_after_misc_register;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&mxt->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	error = sysfs_create_group(&client->dev.kobj, &maxtouch_attr_group);
	if (error) {
		pr_err("[TSP] fail sysfs_create_group\n");
		goto err_after_earlysuspend;
	}
	error = sysfs_create_link(NULL, &client->dev.kobj, DRIVER_NAME);
	if (error) {
                pr_err("[TSP] fail sysfs_create_link\n");
                goto err_after_earlysuspend;
        }
	

#if TS_100S_TIMER_INTERVAL
	INIT_WORK(&mxt->ts_100ms_tmr.tmr_work, ts_100ms_tmr_work);

	ts_100s_tmr_workqueue =
		create_singlethread_workqueue("ts_100_tmr_workqueue");
	if (!ts_100s_tmr_workqueue) {
		pr_err("unabled to create touch tmr work queue\n");
		error = -1;
		goto err_after_attr_group;
	}
	ts_100ms_timer_init(mxt);
#endif
	INIT_DELAYED_WORK(&mxt->reg_update.dwork, reg_update_dwork);

	INIT_DELAYED_WORK(&mxt->supp_ops.dwork, mxt_supp_ops_dwork);
	INIT_DELAYED_WORK(&mxt->cal_timer.dwork, mxt_cal_timer_dwork);
	INIT_DELAYED_WORK(&mxt->check_touch_ic_timer.dwork,
					  mxt_check_touch_ic_timer_dwork);

	VPRINTK("[%s] INIT_DELAYED_WORK 2\n", __func__);
	mxt_check_touch_ic_timer_stop(mxt);
	mxt_check_touch_ic_timer_start(mxt);

	wake_lock_init(&mxt->wakelock, WAKE_LOCK_SUSPEND, "touch");
	/* if (MXT_FIRM_STABLE(mxt->firm_status_data))
		mxt->mxt_status = true; */

	VPRINTK("[%s] Disable Auto_cal %d\n", __func__, mxt->check_auto_cal_flag);
	_disable_auto_cal(mxt, false);
	mxt->check_auto_cal_flag = AUTO_CAL_DISABLE;

	mxt->cal_check_flag = 1;
	//mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_SPT_TIMER_T61)+0x1,0x1);

	return 0;

err_after_attr_group:
	sysfs_remove_group(&client->dev.kobj, &maxtouch_attr_group);

err_after_earlysuspend:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&mxt->early_suspend);
#endif

err_after_misc_register:
	misc_deregister(&mxt336S_misc);

err_after_input_register:
#if defined(INCLUDE_LCD_TOUCHKEY)
	if (mxt->input_key) {
		input_free_device(mxt->input_key);
	}
#endif
	input_free_device(input);

err_after_get_regulator:

err_after_kmalloc:
	if (mxt != NULL) {
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
	}

	return error;
}

static int mxt336s_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;
#ifdef CONFIG_OF
	struct platform_data* pdata;
#endif

	mxt = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	wake_lock_destroy(&mxt->wakelock);
	unregister_early_suspend(&mxt->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_stop(mxt);
#endif
	mxt_check_touch_ic_timer_stop(mxt);

	/* Close down sysfs entries */
	sysfs_remove_group(&client->dev.kobj, &maxtouch_attr_group);

	/* Release IRQ so no queue will be scheduled */
	if (mxt->irq)
		free_irq(mxt->irq, mxt);

#ifdef MXT_JIG_DETECT
	if (mxt->jig_irq)
		free_irq(mxt->jig_irq, mxt);
#endif

	/* deregister misc device */
	misc_deregister(&mxt336S_misc);
#if defined(INCLUDE_LCD_TOUCHKEY)
	input_unregister_device(mxt->input_key);
#endif

	input_unregister_device(mxt->input);

	if (mxt != NULL) {
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
	}

	i2c_set_clientdata(client, NULL);

#ifdef CONFIG_OF
	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		devm_kfree(&client->dev, pdata);
		client->dev.platform_data = NULL;
	}
#endif

	return 0;
}

static int mxt336s_resume_reset(struct i2c_client *client)
{
	int error;
	struct mxt_data *mxt = i2c_get_clientdata(client);

	VPRINTK("[%s]\n", __func__);
	dev_dbg(&client->dev, "resume multi-touch\n");

	disable_irq(mxt->client->irq);

	hw_reset_chip(mxt);

	/* init variables */
	mxt->read_fail_counter = 0;
	mxt->message_counter = 0;
	// mxt->last_read_addr = -1;
#ifndef MXT_THREADED_IRQ
	mxt->valid_irq_counter = 0;
	mxt->invalid_irq_counter = 0;
#endif
	/* metal_suppression_chk_flag = false; */

	/* Default-configuration for auto-cal is 'enable' */
	mxt->check_auto_cal_flag = AUTO_CAL_ENABLE;
	maybe_good_count = 0;

	MXT_FIRM_CLEAR(mxt->firm_status_data);
	error = mxt336s_initialize(client, mxt);
	if (error < 0)
		goto err_resume;
#ifdef MXT_FIRMUP_ENABLE
	else if (error == MXT_INIT_FIRM_ERR) {
		dev_info(&client->dev,
				 "[TSP] invalid device or firmware crash!\n");
		mxt->firm_status_data = MXT_FIRM_STATUS_FAIL;
	}
#endif

#ifndef MXT_TUNNING_ENABLE
	if (MXT_FIRM_STABLE(mxt->firm_status_data)
			&& mxt_write_init_registers(mxt) < 0) {
		dev_err(&client->dev, "[TSP] failed to initialize "
				"registers in resume\n");
		goto err_resume;
	}
#endif /* MXT_TUNNING_ENABLE */
	if (MXT_FIRM_STABLE(mxt->firm_status_data))
		enable_irq(mxt->client->irq);
	return 0;

err_resume:
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
	kfree(mxt->last_message);
	return 0;

}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int mxt336s_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

#if TS_100S_TIMER_INTERVAL
	ts_100ms_timer_stop(mxt);
#endif
	mxt_check_touch_ic_timer_stop(mxt);
	mxt_supp_ops_stop(mxt);

	while (MXT_FIRM_UPGRADING(mxt->firm_status_data)) {
		dev_info(&client->dev, "[TSP] mxt firmware is Downloading : "
				 "mxt suspend must be delayed!");
		msleep(1000);
	}

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(mxt->irq);

	return 0;
}

static int mxt336s_resume(struct i2c_client *client)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(mxt->irq);

	mxt336s_resume_reset(client);
	return 0;
}
#else
#define mxt336s_suspend NULL

#define mxt336s_resume	NULL
#endif

static const struct i2c_device_id mxt336s_idtable[] = {
	{DRIVER_NAME, 0,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt336s_idtable);

#ifdef CONFIG_OF
static const struct of_device_id mxt336s_match[] = {
	{.compatible = "atmel,mxt336s", },
	{ },
};
MODULE_DEVICE_TABLE(of, mxt336s_match);
#endif

static struct i2c_driver mxt336s_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mxt336s_match,
#endif
	},

	.id_table	= mxt336s_idtable,
	.probe		= mxt336s_probe,
	.remove		= mxt336s_remove,
	.suspend	= mxt336s_suspend,
	.resume		= mxt336s_resume,
};


static int __init mxt336s_init(void)
{
	int err;

	LCD_VER = daudio_lcd_version();
	touch_type = daudio_lcd_version();
	
	printk("%s, GPIO_B24 = %d, GPIO_B13 = %d,  daudio_lcd_version() = %d \n", __func__, gpio_get_value(TCC_GPB(24)), gpio_get_value(TCC_GPB(13)), daudio_lcd_version());
	
	switch(LCD_VER) {
		case 1:
		case 2:
		case 5:
		case 6:
	#ifdef INCLUDE_LCD_RESOLUTION_1280_720
		case 10:
	#endif
			err = i2c_add_driver(&mxt336s_driver);
			if (err)
				pr_err("Adding mXT336S-AT driver failed2 (errno = %d)\n", err);

			return err;
			break;

		default:
			return -ENODEV;
			break;
	}

}
module_init(mxt336s_init);

static void __exit mxt336s_cleanup(void)
{
	i2c_del_driver(&mxt336s_driver);
}
module_exit(mxt336s_cleanup);

MODULE_DESCRIPTION("Driver for Atmel mXT336S-AT Touchscreen Controller");
MODULE_AUTHOR("Cleinsoft");
MODULE_LICENSE("GPL");
