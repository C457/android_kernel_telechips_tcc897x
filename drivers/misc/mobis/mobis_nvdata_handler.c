/*
 *  mobis_nvdata_handler.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/signal.h>
#include <linux/personality.h>
#include <linux/kallsyms.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/kexec.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
//#include <asm/system.h>
#include <asm/unistd.h>
#include <asm/traps.h>
#include <asm/unwind.h>

#include <linux/fs.h>
#include <linux/syscalls.h>
//#include <mobis/mobis_nvdata_handler.h>
#include <mach/mobis_nvdata_handler.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>

// ------------------------- For MISC NV data ---------------------------------------------------//

static int mobis_misc_nvdata_raw_read(int offset, char* buf, int size)
{
	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();

	if(size == 0)
		return 0;

	set_fs(KERNEL_DS);
	h_file = sys_open(MOBIS_NVDATA_MISC_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_read( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't read misc NVDATA. read size(%d)\n", ret);
			sys_close(h_file);
			set_fs(oldfs);
			return ret;
		}
		sys_close(h_file);
	}
	else
	{
		printk("Can't open misc nvdata partition handle = %d.\n",h_file);
		set_fs(oldfs);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

static int mobis_misc_nvdata_raw_write(int offset, char* buf, int size)
{
	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();


	if(size == 0)
		return 0;

	set_fs(KERNEL_DS);
	h_file = sys_open(MOBIS_NVDATA_MISC_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_write( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't write misc NVDATA. write size(%d)\n", ret);
			sys_close(h_file);
			set_fs(oldfs);
			return ret;
		}
		sys_fsync(h_file);
		sys_close(h_file);
	}
	else
	{
		printk("Can't open misc NVDATA partition handle = %d.\n",h_file);
		set_fs(oldfs);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

int mobis_misc_nvdata_read(enum mobis_misc_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	return mobis_misc_nvdata_raw_read(pos,buf,size);
}

int mobis_misc_nvdata_write(enum mobis_misc_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	if(pos < MOBIS_NVDATA_MISC_RGB_COLOR_TUNE)
	{
		printk("Less than 4096 bytes is prohibited.\n");
		return -EINVAL;
	}
	return mobis_misc_nvdata_raw_write(pos,buf,size);
}

// ------------------------- For setting NV data -----------------------------------------------------//
static int mobis_settings_nvdata_raw_read(int offset, char* buf, int size)
{
	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();

	if(size == 0)
		return 0;

	set_fs(KERNEL_DS);
	h_file = sys_open(MOBIS_NVDATA_SETTINGS_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_read( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't read settings NVDATA. read size(%d)\n", ret);
			sys_close(h_file);
			set_fs(oldfs);
			return ret;
		}
		sys_close(h_file);
	}
	else
	{
		printk("Can't open settings nvdata partition handle = %d.\n",h_file);
		set_fs(oldfs);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

static int mobis_settings_nvdata_raw_write(int offset, char* buf, int size)
{
	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();

	if(size == 0)
		return 0;

	set_fs(KERNEL_DS);
	h_file = sys_open(MOBIS_NVDATA_SETTINGS_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_write( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't write settings NVDATA. write size(%d)\n", ret);
			sys_close(h_file);
			set_fs(oldfs);
			return ret;
		}
		sys_fsync(h_file);
		sys_close(h_file);
	}
	else
	{
		printk("Can't open settings NVDATA partition handle = %d.\n",h_file);
		set_fs(oldfs);
		return 0;
	}
	set_fs(oldfs);

	return size;
}



int mobis_settings_nvdata_read(enum mobis_settings_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	return mobis_settings_nvdata_raw_read(pos,buf,size);
}

int mobis_settings_nvdata_write(enum mobis_settings_nvdata_offset offset, char* buf, int size)

{
	int pos = (int)offset;
	return mobis_settings_nvdata_raw_write(pos,buf,size);
}
//==========================================================================================

#define MAX_SIZE 100

static DEFINE_MUTEX(mobis_nvdata_lock);

static int offset;
static int size;
static char mobis_misc_nvdata_buf[MAX_SIZE];
static char mobis_settings_nvdata_buf[MAX_SIZE];


/***********************************
read funcs
***********************************/

static ssize_t mobis_misc_nvdata_raw_read_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret = 0;
	mutex_lock(&mobis_nvdata_lock);
	ret = sprintf(buf, "%s", mobis_misc_nvdata_buf);
	mutex_unlock(&mobis_nvdata_lock);
	return ret;
}

static ssize_t mobis_misc_nvdata_raw_read_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	mutex_lock(&mobis_nvdata_lock);
	sscanf(buf, "%d %d", &offset, &size);
	mobis_misc_nvdata_raw_read(offset, mobis_misc_nvdata_buf, size);
	printk("offset=%d, buf=%s, size=%d\n", offset, mobis_misc_nvdata_buf, size);
	mutex_unlock(&mobis_nvdata_lock);
	return count;
}

static ssize_t mobis_settings_nvdata_raw_read_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret = 0;
	mutex_lock(&mobis_nvdata_lock);
	ret = sprintf(buf, "%s", mobis_settings_nvdata_buf);
	mutex_unlock(&mobis_nvdata_lock);
	return ret;
}

static ssize_t mobis_settings_nvdata_raw_read_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	mutex_lock(&mobis_nvdata_lock);
	sscanf(buf, "%d %d", &offset, &size);
	mobis_settings_nvdata_raw_read(offset, mobis_settings_nvdata_buf, size);
	printk("offset=%d, buf=%s, size=%d\n", offset, mobis_settings_nvdata_buf, size);
	mutex_unlock(&mobis_nvdata_lock);
	return count;
}

/***********************************
//write funcs
***********************************/

static ssize_t mobis_misc_nvdata_raw_write_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	char bufTmp[MAX_SIZE] = { 0, };

	mutex_lock(&mobis_nvdata_lock);
	sscanf(buf, "%d %s %d", &offset, bufTmp, &size);
	printk("offset=%d, buf=%s, size=%d\n", offset, bufTmp, size);
	mobis_misc_nvdata_raw_write(offset, bufTmp, size);
	mutex_unlock(&mobis_nvdata_lock);
	return count;
}

static ssize_t mobis_settings_nvdata_raw_write_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	char bufTmp[MAX_SIZE] = { 0, };

	mutex_lock(&mobis_nvdata_lock);
	sscanf(buf, "%d %s %d", &offset, bufTmp, &size);
	printk("offset=%d, buf=%s, size=%d\n", offset, bufTmp, size);
	mobis_settings_nvdata_raw_write(offset, bufTmp, size);
	mutex_unlock(&mobis_nvdata_lock);
	return count;
}

static struct kobj_attribute mobis_misc_nvdata_raw_read_attribute =
	__ATTR(misc_raw_read, 0664, mobis_misc_nvdata_raw_read_show, mobis_misc_nvdata_raw_read_store);
static struct kobj_attribute mobis_settings_nvdata_raw_read_attribute =
	__ATTR(settings_raw_read, 0664, mobis_settings_nvdata_raw_read_show, mobis_settings_nvdata_raw_read_store);

static struct kobj_attribute mobis_misc_nvdata_raw_write_attribute =
	__ATTR(misc_raw_write, 0200, NULL, mobis_misc_nvdata_raw_write_store);
static struct kobj_attribute mobis_settings_nvdata_raw_write_attribute =
	__ATTR(settings_raw_write, 0200, NULL, mobis_settings_nvdata_raw_write_store);

static struct attribute *attrs[] = {
	&mobis_misc_nvdata_raw_read_attribute.attr,
	&mobis_settings_nvdata_raw_read_attribute.attr,
	&mobis_misc_nvdata_raw_write_attribute.attr,
	&mobis_settings_nvdata_raw_write_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name ="nvdata",
	.attrs = attrs,
};


static struct kobject *mobis_nvdata_kobj;

static int __init mobis_nvdata_handler_init(void)
{
	int retval;

	mobis_nvdata_kobj = kobject_create_and_add("mobis_nvdata", kernel_kobj);
	if (!mobis_nvdata_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(mobis_nvdata_kobj, &attr_group);
	if (retval)
		kobject_put(mobis_nvdata_kobj);

	return retval;
}

static void __exit mobis_nvdata_handler_exit(void)
{
	kobject_put(mobis_nvdata_kobj);
}

/* sample code
static void mobis_nvdata_test_code(void)
{
	char test_buffer[3] = { 0xFF, 0xE8, 0xF2 };	// 255, 232, 242
	mobis_misc_nvdata_write(MOBIS_NVDATA_MISC_RGB_COLOR_TUNE, test_buffer, 3);
	memset(test_buffer, 0x00, sizeof(test_buffer));
	mobis_misc_nvdata_read(MOBIS_NVDATA_MISC_RGB_COLOR_TUNE, test_buffer, 3);
	printk("R=%d, G=%d, B=%d\n", test_buffer[0], test_buffer[1], test_buffer[2]);
}
*/

module_init(mobis_nvdata_handler_init);
module_exit(mobis_nvdata_handler_exit);

MODULE_AUTHOR("System platform team <kibum.lee@mobis.co.kr>");
MODULE_DESCRIPTION("MOBIS nvdata handler driver");
MODULE_LICENSE("GPL");

