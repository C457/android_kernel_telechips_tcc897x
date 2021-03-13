/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        proc_fs.c
*  \brief       HDMI TX controller driver
*  \details   
*  \version     1.0
*  \date        2014-2015
*  \copyright
This source code contains confidential information of Telechips.
Any unauthorized use without a written  permission  of Telechips including not 
limited to re-distribution in source  or binary  form  is strictly prohibited.
This source  code is  provided "AS IS"and nothing contained in this source 
code  shall  constitute any express  or implied warranty of any kind, including
without limitation, any warranty of merchantability, fitness for a   particular 
purpose or non-infringement  of  any  patent,  copyright  or  other third party 
intellectual property right. No warranty is made, express or implied, regarding 
the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability 
arising from, out of or in connection with this source  code or the  use in the 
source code. 
This source code is provided subject  to the  terms of a Mutual  Non-Disclosure 
Agreement between Telechips and Company.
*******************************************************************************/
#include "include/hdmi_includes.h"
#include "include/proc_fs.h"

#define DEBUGFS_BUF_SIZE 4096

static struct file_operations proc_fops_hdcp_keys;
static struct file_operations proc_fops_hpd;
static struct file_operations proc_fops_hdcp22;


ssize_t
proc_write_hdcp_keys(struct file *filp, const char __user *buffer, size_t cnt,
		loff_t *off_set){
        //struct hdmi_tx_dev *dev = PDE_DATA(file_inode(filp));
	pr_info("%s:TDB\n", FUNC_NAME);
	return cnt;
}

ssize_t proc_write_hdcp22(struct file *filp, const char __user *buffer, size_t cnt,
		loff_t *off_set){
	int ret;
	uint32_t hdcp22 = 0;
        struct hdmi_tx_dev *dev = PDE_DATA(file_inode(filp));
	// Check size of the input buffer
	if(cnt != sizeof(uint32_t)){
//		if(dev->verbose >= VERBOSE_IO)
			pr_err("%s:Sizes do not match [%d != %d]\n", FUNC_NAME,
				(int)cnt, (int)sizeof(uint32_t));
		return 0;
	}

	// Copy buffers
	ret = copy_from_user(&hdcp22, buffer, cnt);

	dev->hdcp22 = hdcp22;

	if(dev->verbose >= VERBOSE_IO)
		pr_info("%s:HDCP 2.2 = %d\n", FUNC_NAME, dev->hdcp22);

	return 0;
}

/**
 * hpd functions
 */
ssize_t proc_read_hpd(struct file *filp, char __user *usr_buf, size_t cnt, loff_t *off_set)
{
        ssize_t size;
        struct hdmi_tx_dev *dev = PDE_DATA(file_inode(filp));     
        
        char *hpd_buf = devm_kzalloc(dev->parent_dev, DEBUGFS_BUF_SIZE, GFP_KERNEL);
        size = sprintf(hpd_buf, "%d\n", dev->hpd);
        size = simple_read_from_buffer(usr_buf, cnt,  off_set, hpd_buf, size);
        devm_kfree(dev->parent_dev, hpd_buf);
	return size;
}

int
proc_open(struct inode *inode, struct file *filp){
	try_module_get(THIS_MODULE);
        //struct hdmi_tx_dev *dev = PDE_DATA(file_inode(filp));
	return 0;
}

int
proc_close(struct inode *inode, struct file *filp){
        //struct hdmi_tx_dev *dev = PDE_DATA(file_inode(filp));
	module_put(THIS_MODULE);
	return 0;
}

void proc_interface_init(struct hdmi_tx_dev *dev){
	if(dev == NULL){
		pr_err("%s:Device is null\n", FUNC_NAME);
		return;
	}

	dev->hdmi_proc_dir = proc_mkdir("hdmi_tx", NULL);
	if(dev->hdmi_proc_dir == NULL){
		pr_err("%s:Could not create file system @ /proc/hdmi_tx\n",
			FUNC_NAME);
	}

	// Clear file operations
	memset(&proc_fops_hpd, 0, sizeof(struct file_operations));
	memset(&proc_fops_hdcp_keys, 0, sizeof(struct file_operations));
	memset(&proc_fops_hpd, 0, sizeof(struct file_operations));
        
	// HPD
	pr_info("%s:Installing /proc/hdmi_tx/hpd file\n", FUNC_NAME);
	proc_fops_hpd.owner = THIS_MODULE;
	proc_fops_hpd.open = proc_open;
	proc_fops_hpd.release = proc_close;
	proc_fops_hpd.read = proc_read_hpd;
	dev->hdmi_proc_hpd = proc_create_data("hpd", S_IFREG | S_IRUGO,
			dev->hdmi_proc_dir, &proc_fops_hpd, dev);
	if(dev->hdmi_proc_hpd == NULL){
		pr_err("%s:Could not create file system @"
				" /proc/hdmi_tx/hpd\n", FUNC_NAME);
	}
        
        // HDCP Keys
        pr_info("%s:Installing /proc/hdmi_tx/hdcp_keys file\n", FUNC_NAME);
        proc_fops_hdcp_keys.owner = THIS_MODULE;
        proc_fops_hdcp_keys.open = proc_open;
        proc_fops_hdcp_keys.release = proc_close;
        proc_fops_hdcp_keys.write = proc_write_hdcp_keys;
        dev->hdmi_proc_hdcp_keys = proc_create_data("hdcp_keys", S_IFREG | S_IWUGO,
                                         dev->hdmi_proc_dir,
                                         &proc_fops_hdcp_keys, dev);
        if(dev->hdmi_proc_hdcp_keys == NULL){
                pr_err("%s:Could not create file system @"
                                " /proc/hdmi_tx/hdcp_keys\n", FUNC_NAME);
        }
        
	// HDCP 2.2
	pr_info("%s:Installing /proc/hdmi_tx/hdcp22 file\n", FUNC_NAME);
	proc_fops_hdcp22.owner = THIS_MODULE;
	proc_fops_hdcp22.open = proc_open;
	proc_fops_hdcp22.release = proc_close;
	proc_fops_hdcp22.write = proc_write_hdcp22;
	dev->hdmi_proc_hdcp22 = proc_create_data("hdcp22", S_IFREG | S_IWUGO,
			dev->hdmi_proc_dir, &proc_fops_hpd, dev);
	if(dev->hdmi_proc_hdcp22 == NULL){
		pr_err("%s:Could not create file system @"
				" /proc/hdmi_tx/hdcp22\n", FUNC_NAME);
	}


}

void proc_interface_remove(struct hdmi_tx_dev *dev){

	if(dev->hdmi_proc_hdcp_keys != NULL)
		proc_remove(dev->hdmi_proc_hdcp_keys);

	if(dev->hdmi_proc_dir != NULL)
		proc_remove(dev->hdmi_proc_dir);

	if(dev->hdmi_proc_hpd != NULL)
		proc_remove(dev->hdmi_proc_hpd);

	if(dev->hdmi_proc_hdcp22 != NULL)
		proc_remove(dev->hdmi_proc_hdcp22);
}
