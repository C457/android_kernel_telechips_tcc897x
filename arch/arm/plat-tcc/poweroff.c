/****************************************************************************
 * plat-tcc/poweroff.c
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
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <mach/bsp.h>
#include <mach/io.h>
//#include <mach/system.h>
#include <asm/system_misc.h>
#include <asm/mach/arch.h>
#ifdef CONFIG_DAUDIO_KK
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>

typedef struct _RECOVERY_MESSAGE {
	char command[32];
	char status[32];
	char recovery[1024];
} RECOVERY_MESSAGE;

#define MISC_PARTITION_PATH "/dev/block/platform/bdm/by-name/misc"
#endif

//+NATIVE_PLATFORM shutdown via micom
// Add feature for sending message to micom daemon about shutdown.
// The micom daemon will send micom packet(SYSTEM_OFF_COMPLETE_C) to micom.

// Uncomment below line to enable the feature
#define FEATURE_SEND_SHUTDOWN_MSG_TO_MICOM
#ifdef FEATURE_SEND_SHUTDOWN_MSG_TO_MICOM
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/termios.h>
#include <asm/ioctls.h>
#include <linux/serial.h>
#include <linux/poll.h>

static void shutdown_prepare(void);
#endif
//-NATIVE_PLATFORM

#define nop_delay(x) for(cnt=0 ; cnt<x ; cnt++){ \
		__asm__ __volatile__ ("nop\n"); }

uint32_t restart_reason = 0x776655AA;

static void tcc_pm_power_off(void)
{
#ifdef CONFIG_RTC_DISABLE_ALARM_FOR_PWROFF_STATE		//Disable the RTC Alarm during the power off state
	{
		extern volatile int tca_alarm_disable(unsigned int rtcbaseaddress);
		tca_alarm_disable(io_p2v(HwRTC_BASE));
	}
#endif
#ifdef CONFIG_REGULATOR_DA9062
    {
        extern void da9062_power_off(void);
        da9062_power_off();
    }
#endif

	while(1);
}

#ifdef CONFIG_DAUDIO_KK
/*===========================================================================
FUNCTION
===========================================================================*/
#ifdef FACTORYRST_CMDLINE
int panic_factoryrst_dsp(void)
{
	int fd;
	int result = 0;
	mm_segment_t fs;
	static char *spPanicdebug="factory-reset-by-panic";
	static char read_Msg[32];
	int factoryrst_flag = 0;

	fs=get_fs();
	set_fs(KERNEL_DS);

	fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
	if(fd >= 0) {
		memset(&read_Msg, 0, sizeof(read_Msg));
		sys_lseek(fd, 3072, SEEK_SET);
		result = sys_read(fd, (char const *)&read_Msg, sizeof(read_Msg));
		if (result >= 0 && !strncmp(read_Msg, spPanicdebug, sizeof(read_Msg)) )
		{
		    printk(KERN_ERR "[KDEBUG] %s:%d ! panic MSG read : %s \r\n", __func__, __LINE__,read_Msg);
			factoryrst_flag = 1;
		}
		else
		    printk(KERN_ERR "[KDEBUG] %d panic MSG read, error (%d) \r\n", __LINE__, fd);
    }

	sys_close(fd);
	set_fs(fs);
	sys_sync();
	return factoryrst_flag;
}
#endif
void panic_repeat_debug(void)
{
	int fd;
	int result = 0;
	mm_segment_t fs;
	static char *spPanicdebug="factory-reset-by-panic";
	static char read_Msg[32], write_Msg[32];

	memset(&write_Msg, 0, sizeof(write_Msg));
	strlcpy(write_Msg, spPanicdebug, sizeof(write_Msg));

	fs=get_fs();
	set_fs(KERNEL_DS);

	fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
	if(fd >= 0) {
		sys_lseek(fd, 3072, SEEK_SET);
		result = sys_write(fd, (char const *)&write_Msg, sizeof(write_Msg));
		if (result >= 0)
		    printk(KERN_ERR "[KDEBUG] %s:%d ! panic MSG write : %s \r\n", __func__, __LINE__,write_Msg);
		else
		    printk(KERN_ERR "[KDEBUG] %d panic MSG write, error (%d) \r\n", __LINE__, fd);
	}

	sys_close(fd);
	set_fs(fs);

	fs=get_fs();
	set_fs(KERNEL_DS);

	fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
	if(fd >= 0) {
		memset(&read_Msg, 0, sizeof(read_Msg));
		sys_lseek(fd, 3072, SEEK_SET);
		result = sys_read(fd, (char const *)&read_Msg, sizeof(read_Msg));
		if (result >= 0 && !strncmp(read_Msg, spPanicdebug, sizeof(read_Msg)) )
		    printk(KERN_ERR "[KDEBUG] %s:%d ! panic MSG read : %s \r\n", __func__, __LINE__,read_Msg);
		else
		    printk(KERN_ERR "[KDEBUG] %d panic MSG read, error (%d) \r\n", __LINE__, fd);
    }

	sys_close(fd);
	set_fs(fs);
	sys_sync();
}

int panic_count_check(int clear)
{
	int fd = -1;
	int result = 0;
	mm_segment_t fs;

	static char read_count = 0, write_count = 0;

	fs=get_fs();
	set_fs(KERNEL_DS);

	fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
	if(fd >= 0) {
		memset(&read_count, 0, sizeof(read_count));
		sys_lseek(fd, 2048, SEEK_SET);
		result = sys_read(fd, (char const *)&read_count, sizeof(read_count));
		if (result >= 0)
			printk(KERN_ERR "[KDEBUG] %s:%d ! panic count read : %d \r\n", __func__, __LINE__,read_count);
		else
			printk(KERN_ERR "[KDEBUG] %d panic count read, error (%d) \r\n", __LINE__, fd);

		sys_close(fd);
	}
	else
		printk(KERN_ERR "[KDEBUG] %d sys_open failed:fd(%d) PATH(%s) \r\n", __LINE__, fd, MISC_PARTITION_PATH);

	set_fs(fs);

	memset(&write_count, 0, sizeof(write_count));
	if (clear == false)
		write_count = read_count + 1;
	else
		write_count = 1;

	fs=get_fs();
	set_fs(KERNEL_DS);

	fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
	if(fd >= 0) {
		sys_lseek(fd, 2048, SEEK_SET);
		result = sys_write(fd, (char const *)&write_count, sizeof(write_count));
		if (result >= 0)
			printk(KERN_ERR "[KDEBUG] %s:%d ! panic count write : %d \r\n", __func__, __LINE__,write_count);
		else
			printk(KERN_ERR "[KDEBUG] %d panic count write, error (%d) \r\n", __LINE__, fd);

		sys_close(fd);
	}
	else
		printk(KERN_ERR "[KDEBUG] %d sys_open failed:fd(%d) PATH(%s) \r\n", __LINE__, fd, MISC_PARTITION_PATH);

	set_fs(fs);
	sys_sync();
	return write_count;
}

void restart_reason_misc_write_with_message(uint32_t spReason_write)
{
	static char *spReason[]={"", "boot-bootloader", "boot-recovery", "boot-force_normal"};
	static char *spRecovery="recovery\n--wipe_data\n";

	int fd = -1;
	int loop = 0;
	int result = 0;
	mm_segment_t fs;
	RECOVERY_MESSAGE write_Msg,read_Msg;

	memset(&write_Msg, 0, sizeof(write_Msg));
	strlcpy(write_Msg.command, spReason[spReason_write], sizeof(write_Msg.command));
	strlcpy(write_Msg.recovery, spRecovery, sizeof(write_Msg.recovery));

	for (loop = 0; loop < 100; loop++) {
		/* Write MISC reboot parameter */
		fs=get_fs();
		set_fs(KERNEL_DS);

		fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
		if(fd >= 0) {
			sys_lseek(fd, 0, SEEK_SET);
			result = sys_write(fd, (char const *)&write_Msg, sizeof(write_Msg));

			if(result >= 0)
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG writing...) \r\n", __func__, __LINE__);
			else
                printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG writing error (%d)) \r\n", __func__, __LINE__, result);

			sys_close(fd);
		}
		else
			printk(KERN_ERR "[KDEBUG] %s:%d sys_open failed:fd(%d) PATH(%s) \r\n", __func__, __LINE__, fd, MISC_PARTITION_PATH);

		set_fs(fs);

		/* Read MISC reboot paramter */
		fs=get_fs();
		set_fs(KERNEL_DS);

		fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
		if (fd >= 0) {
			memset(&read_Msg, 0, sizeof(read_Msg));
			sys_lseek(fd, 0, SEEK_SET);
			result = sys_read(fd, (char const *)&read_Msg, sizeof(read_Msg));
			if (result >= 0) {
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG read : %s \r\n", __func__, __LINE__,read_Msg.command);
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG read recovery: %s \r\n", __func__, __LINE__,read_Msg.recovery);
				if (strncmp(spReason[spReason_write],read_Msg.command,strlen(spReason[spReason_write])) == 0) {
					printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG check OK \r\n", __func__, __LINE__);
					sys_close(fd);
					set_fs(fs);
					break;
				}
			}

			sys_close(fd);
		}
		else
			printk(KERN_ERR "[KDEBUG] %s:%d sys_open failed:fd(%d) PATH(%s) \r\n", __func__, __LINE__, fd, MISC_PARTITION_PATH);

		set_fs(fs);
	}
	sys_sync();
}

void restart_reason_misc_write(uint32_t spReason_write)
{
	static char *spReason[]={"", "boot-bootloader", "boot-recovery", "boot-force_normal", "boot-panic", "boot-android"};
	int fd;
	int loop = 0;
	int result = 0;
	mm_segment_t fs;
	RECOVERY_MESSAGE write_Msg,read_Msg;

	memset(&write_Msg, 0, sizeof(write_Msg));
	strlcpy(write_Msg.command, spReason[spReason_write], sizeof(write_Msg.command));

	for (loop = 0; loop < 100; loop++) {
		/* Write MISC reboot parameter */
		fs=get_fs();
		set_fs(KERNEL_DS);

		fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
		if(fd >= 0) {
			sys_lseek(fd, 0, SEEK_SET);
			result = sys_write(fd, (char const *)&write_Msg, sizeof(write_Msg));

			if(result >= 0)
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG writing...) \r\n", __func__, __LINE__);
			else
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG writing error (%d)) \r\n", __func__, __LINE__, result);

		}
		else
			printk(KERN_ERR "[KDEBUG] %s Not open, error (%d) \r\n", MISC_PARTITION_PATH, fd);

		sys_close(fd);
		set_fs(fs);

		sys_sync();
		sys_sync();
		sys_sync();

		/* Read MISC reboot paramter */
		fs=get_fs();
		set_fs(KERNEL_DS);

		fd=sys_open(MISC_PARTITION_PATH, O_RDWR, 0);
		if (fd >= 0) {
			memset(&read_Msg, 0, sizeof(read_Msg));
			sys_lseek(fd, 0, SEEK_SET);
			result = sys_read(fd, (char const *)&read_Msg, sizeof(read_Msg));
			if (result >= 0) {
				printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG read : %s \r\n", __func__, __LINE__,read_Msg.command);
				if (strncmp(spReason[spReason_write],read_Msg.command,strlen(spReason[spReason_write])) == 0) {
					printk(KERN_ERR "[KDEBUG] %s:%d ! REOCVERY MSG check OK \r\n", __func__, __LINE__);
					sys_close(fd);
					set_fs(fs);
					break;
				}
			} else
				printk(KERN_ERR "[KDEBUG] %d Not open, error (%d) \r\n", __LINE__, fd);
		}
		sys_close(fd);
		set_fs(fs);

	}
}
#endif
static int tcc_reboot_call(struct notifier_block *this, unsigned long code, void *cmd)
{
	/* XXX: convert reboot mode value because USTS register
	 * hold only 8-bit value
	 */
	if (code == SYS_RESTART) {
		if (cmd) {
			if (!strcmp(cmd, "bootloader"))
				restart_reason = 1;	/* fastboot mode */
			else if (!strcmp(cmd, "recovery"))
				restart_reason = 2;	/* recovery mode */
//+[TCCQB] QuickBoot Skip Mode
			else if (!strcmp(cmd, "force_normal"))
				restart_reason = 3;	/* skip quickboot mode */
//-[TCCQB]
//
			else
				restart_reason = 5;
		} else
			restart_reason = 5;
#ifdef CONFIG_DAUDIO_KK
		if( (restart_reason >= 0) && (restart_reason <= 5) )
			restart_reason_misc_write(restart_reason);
#endif			
	}
#ifdef CONFIG_DAUDIO_KK
	printk("restart_reason(%d)\n", restart_reason);
#endif
	return NOTIFY_DONE;
}

static struct notifier_block tcc_reboot_notifier = {
	.notifier_call = tcc_reboot_call,
};
static int __init tcc_poweroff_init(void)
{
	pm_power_off = tcc_pm_power_off;

#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	if (machine_desc->restart)
#else
	if (machine_desc->restart && machine_desc)
#endif
		arm_pm_restart = machine_desc->restart;

	register_reboot_notifier(&tcc_reboot_notifier);

//+NATIVE_PLATFORM shutdown via micom
#ifdef FEATURE_SEND_SHUTDOWN_MSG_TO_MICOM
	pm_power_off_prepare = shutdown_prepare;	
#endif
//-NATIVE_PLATFORM

	return 0;
}
__initcall(tcc_poweroff_init);

//+NATIVE_PLATFORM shutdown via micom
#ifdef FEATURE_SEND_SHUTDOWN_MSG_TO_MICOM
#define MICOM_TTY_DEV "/dev/ttyTCC2"
#define SHUTDOWN_PACKET_LENGTH 10

static struct file *tty_file;

static long tty_ioctl(struct file *f, unsigned op, unsigned long param)
{
	if (f->f_op->unlocked_ioctl)
		return f->f_op->unlocked_ioctl(f, op, param);

	return -ENOSYS;
}

static int tty_open(void) 
{
	int result = 0;
	tty_file = filp_open(MICOM_TTY_DEV, O_RDWR, S_IRUSR|S_IWUSR);
	if (IS_ERR(tty_file))
	{
		result = (int)PTR_ERR(tty_file);
		printk(KERN_ERR "open error %s - err:%d\n", MICOM_TTY_DEV, result);
	}
	return result;
}

static void tty_close(void)
{
	if (!IS_ERR(tty_file) && (tty_file != 0))
		filp_close(tty_file, 0);
}

static void tty_setspeed(void)
{
	int result = -1;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	{
		struct termios settings;

		tty_ioctl(tty_file, TCGETS, (unsigned long)&settings);
		settings.c_iflag = 0;
		settings.c_oflag = 0;
		settings.c_lflag = 0;
		settings.c_cflag = CLOCAL | CS8 | CREAD;
		settings.c_cc[VMIN] = 0;
		settings.c_cc[VTIME] = 0;
		settings.c_cflag |= B115200;

		result = tty_ioctl(tty_file, TCSETS, (unsigned long)&settings);

	}

	set_fs(oldfs);

	printk(KERN_ERR "tty_setspeed result:%d\n", result);
}

static int tty_write(unsigned char *buf, int count)
{
	int result;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	tty_file->f_pos = 0;
	result = tty_file->f_op->write(tty_file, buf, count, &tty_file->f_pos);
	set_fs(oldfs);
	return result;	
}

static unsigned int calc_xor_checksum(unsigned char *data, int count)
{
    int i;
    uint8_t sum;
    int length = count-1;

    sum = data[0];

    for(i=1; i<length; i++){
        sum ^= data[i];
    }

    return sum;
}
static unsigned char packet_buf[10];
static int send_shutdown(void)
{
    // Request reset for test.
    // packet_buf[0] = 0xff; // sof
    // packet_buf[1] = 0x8a; // sender id : CID_SYSTEM
    // packet_buf[2] = 0x01; // receiver id : MID_SYSTEM
    // packet_buf[3] = 0x01; // type : APP
    // packet_buf[4] = 0x01; // funcH : MID_SYSTEM
    // packet_buf[5] = 0x1c; // funcL : 0x1c
    // packet_buf[6] = 0x00; // payload length High
    // packet_buf[7] = 0x02; // payload length Low
    // packet_buf[8] = 0x00; // data : Normal

    // Send SYSTEM_OFF_COMPLETE_C packet with System off data(0x02)
    packet_buf[0] = 0xff; // sof
    packet_buf[1] = 0x8a; // sender id : CID_SYSTEM
    packet_buf[2] = 0x01; // receiver id : MID_SYSTEM
    packet_buf[3] = 0x01; // type : APP
    packet_buf[4] = 0x01; // funcH : MID_SYSTEM
    packet_buf[5] = 0x09; // funcL : 0x09
    packet_buf[6] = 0x00; // payload length High
    packet_buf[7] = 0x02; // payload length Low
    packet_buf[8] = 0x02; // data : System Off
    packet_buf[9] = calc_xor_checksum(packet_buf, 10); // checksum

    return tty_write(packet_buf, SHUTDOWN_PACKET_LENGTH);	
}

static void shutdown_prepare(void)
{
	int result;
	result = tty_open();
	if (result >= 0) {
		printk(KERN_ERR "tty open successful\n");		
		tty_setspeed();
    
	    result = send_shutdown();

	    printk(KERN_ERR "send_shutdown result:%d\n", result);

	    tty_close();

	    mdelay(100);

	}
}

#endif
//-NATIVE_PLATFORM
