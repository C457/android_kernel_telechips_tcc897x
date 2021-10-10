/*
 * drivers/char/tcc_mem.c
 *
 * TCC MEM driver
 *
 * Copyright (C) 2009 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/syscalls.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
#include <asm/pgtable.h>
#endif
#include <asm/mach-types.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/tcc_mem_ioctl.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tcc_vsync_ioctl.h>
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <mach/tccfb.h>
#include <mach/vioc_rdma.h>
#endif
#else
#include <video/tcc/tcc_mem_ioctl.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tcc_vsync_ioctl.h>
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <video/tcc/tccfb.h>
#include <video/tcc/vioc_rdma.h>
#endif
#endif

#include "vpu_comm.h"
#include "vpu_devices.h"

#define dprintk(msg...)	if (0) { printk( "VPU_MEM: " msg); }


extern int set_displaying_index(unsigned long arg);
extern int get_displaying_index(int nInst);
extern int set_buff_id(unsigned long arg);
extern int get_buff_id(int nInst);
extern void tcc_vsync_set_deinterlace_mode(int mode);
extern int range_is_allowed(unsigned long pfn, unsigned long size);
extern int vmem_get_pgprot(unsigned long ulOldProt, unsigned long ulPageOffset);

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include "../../video/fbdev/tcc-fb/tcc_vsync.h"
extern int tcc_video_last_frame(tcc_video_disp *p, struct stTcc_last_frame iLastFrame, struct tcc_lcdc_image_update *lastUpdated, int idx);
#endif

long vmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;

	switch(cmd)
	{
		case TCC_VIDEO_SET_DISPLAYING_IDX:
		{
			ret = set_displaying_index(arg);
		}
		break;

		case TCC_VIDEO_GET_DISPLAYING_IDX:
		{
			ret = get_displaying_index((int)arg);
		}
		break;

		case TCC_VIDEO_SET_DISPLAYED_BUFFER_ID:
		{
			ret = set_buff_id(arg);
		}
		break;

		case TCC_VIDEO_GET_DISPLAYED_BUFFER_ID:
		{
			ret = get_buff_id((int)arg);
		}
		break;

		case TCC_LCDC_VIDEO_KEEP_LASTFRAME:
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
		{
			struct stTcc_last_frame lastframe_info;
			
			if (copy_from_user((void*)&lastframe_info, (const void*)arg, sizeof(struct stTcc_last_frame)))
				ret = -EFAULT;
			else
			{
				struct tcc_lcdc_image_update lastUpdated;

				if((ret = tcc_video_last_frame(NULL, lastframe_info, &lastUpdated, 0)) > 0)
					tcc_vsync_set_deinterlace_mode(0);

			}
		}
#endif
		break;

		default:
			printk("Unsupported cmd(0x%x) for vmem_ioctl. \n", cmd);
			ret = -EFAULT;
			break;
	}

	return ret;
}

static unsigned int vmem_poll(struct file *filp, poll_table *wait)
{
	vpu_comm_data_t *vpu_data = (vpu_comm_data_t *)filp->private_data;

	if (vpu_data == NULL){
		return -EFAULT;
	}

	if (vpu_data->count > 0)
	{
		vpu_data->count--;
		return POLLIN;
	}

	poll_wait(filp, &(vpu_data->wq), wait);

	if (vpu_data->count > 0)
	{
		vpu_data->count--;
		return POLLIN;
	}

	return 0;
}

static int vmem_open(struct inode *inode, struct file *filp)
{
	return 0;
}


static int vmem_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int vmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "mem_mmap: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = vmem_get_pgprot(vma->vm_page_prot, vma->vm_pgoff);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk("mem_mmap :: remap_pfn_range failed\n");
		return -EAGAIN;
	}

	vma->vm_ops 	= NULL;
	vma->vm_flags	|= VM_IO;
	vma->vm_flags   |= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static struct file_operations vdev_mem_fops = {
	.owner				= THIS_MODULE,
	.open				= vmem_open,
	.release			= vmem_release,
	.mmap				= vmem_mmap,
	.unlocked_ioctl		= vmem_ioctl,
	.poll				= vmem_poll,
};

static struct miscdevice vmem_misc_device =
{
    MISC_DYNAMIC_MINOR,
    MEM_NAME,
    &vdev_mem_fops,
};

int vmem_probe(struct platform_device *pdev)
{
    if (misc_register(&vmem_misc_device))
    {
        printk(KERN_WARNING "VPU memoder: Couldn't register device.\n");
        return -EBUSY;
    }

	return 0;
}

int vmem_remove(struct platform_device *pdev)
{
    misc_deregister(&vmem_misc_device);
	return 0;
}

