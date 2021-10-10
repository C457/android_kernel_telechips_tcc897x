/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <asm/processor.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>

#include "tcc_hdin_main.h"
#include "tcc_hdin_ctrl.h"
#include "tcc_hdin_video.h"

#include <linux/clk.h>
#include <linux/jiffies.h>

static int debug	   = 0;
#define dprintk(msg...)	if(debug) { printk( "\e[33mhdin_main : \e[0m" msg); }

extern unsigned long volatile __jiffy_data jiffies;

static int check_int = 1; // check interrupt
static int video_nr = -1;

int hdin_videobuf_g_fmt(struct tcc_hdin_device *hdev, struct v4l2_format *fmt)
{
	memset(&fmt->fmt.pix, 0, sizeof (fmt->fmt.pix));
	fmt->fmt.pix = hdev->pix;

	return 0;
}

int hdin_videobuf_s_fmt(struct tcc_hdin_device *hdev, struct v4l2_format *fmt)
{
	hdev->pix.width	= fmt->fmt.pix.width;
	hdev->pix.height	= fmt->fmt.pix.height;
	hdev->pix.pixelformat = fmt->fmt.pix.pixelformat;

	return hdin_video_set_resolution(hdev, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width, fmt->fmt.pix.height);
}

int hdin_videobuf_querycap(struct tcc_hdin_device *hdev, struct v4l2_capability *cap)
{
	memset(cap, 0, sizeof(struct v4l2_capability));

	strlcpy(cap->driver, hdev->name, sizeof(cap->driver));
	strlcpy(cap->card, hdev->vfd->name, sizeof(cap->card));

	cap->bus_info[0] = '\0';
	cap->version = KERNEL_VERSION(3, 10, 37);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
						V4L2_CAP_STREAMING;
	return 0;
}

int hdin_videobuf_reqbufs(struct tcc_hdin_device *hdev, struct v4l2_requestbuffers *req)
{

	if(req->memory != V4L2_MEMORY_MMAP && req->memory != V4L2_MEMORY_USERPTR \
		&& req->memory != V4L2_MEMORY_OVERLAY) 
	{
		printk("reqbufs: memory type invalid\n");
		return -EINVAL;
	}

	if(req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) 
	{
		printk("reqbufs: video type invalid\n");
		return -EINVAL;
	}

	return hdin_video_buffer_set(hdev, req);
}

int hdin_videobuf_querybuf(struct tcc_hdin_device *hdev, struct v4l2_buffer *buf)
{
	struct TCC_HDIN *h = hdev->h;
	struct v4l2_pix_format pix = hdev->pix;
	struct tcc_hdin_buffer *hdin_buf = h->buf + buf->index;
	int index = buf->index;

	if (index < 0 || index > h->hdin_cfg.pp_num) {
		printk(KERN_WARNING "querybuf error : index : %d / %d",index, h->hdin_cfg.pp_num);
		return -EINVAL;
	}

	memset(buf, 0, sizeof(*buf));
	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->index = index;
	buf->flags = V4L2_BUF_FLAG_MAPPED;
	buf->flags |= hdin_buf->v4lbuf.flags;
	buf->field = V4L2_FIELD_NONE;
	buf->timestamp = hdin_buf->v4lbuf.timestamp;
	buf->sequence = hdin_buf->v4lbuf.sequence;
	buf->memory = V4L2_MEMORY_MMAP;

	if (h->hdin_cfg.fmt == YUV420SEP)
		buf->length = PAGE_ALIGN((pix.width * pix.height * 3) / 2);
	else if (h->hdin_cfg.fmt >= RGB565)
		buf->length = PAGE_ALIGN(pix.width * h->hdin_cfg.bpp * pix.height);

	hdin_buf->v4lbuf.index = index;
	hdin_buf->v4lbuf.length = buf->length;
	hdin_buf->v4lbuf.m.offset = buf->index * buf->length;	//data->cif_cfg.preview_buf[index].p_Y;
	buf->m.offset = hdin_buf->v4lbuf.m.offset;

	dprintk("buf->m.offset(0x%x), buf->length(0x%x), h->hdin_cfg.base_buf(0x%x)\n",
		buf->m.offset, buf->length, h->hdin_cfg.base_buf);

	/* NOTE: add base address. (team app. wanted)
	 */
	hdin_buf->v4lbuf.m.offset += h->hdin_cfg.base_buf;
	buf->m.offset = hdin_buf->v4lbuf.m.offset;

	dprintk("<%d :: [PA]0x%x / flag: 0x%x  >\n", index, (unsigned int)(hdin_buf->v4lbuf.m.offset), (unsigned int)(buf->flags));
	return 0;
}

int hdin_videobuf_qbuf(struct tcc_hdin_device *hdev, struct v4l2_buffer *buf)
{
	struct TCC_HDIN *h = hdev->h;
	struct tcc_hdin_buffer *hdin_buf = h->buf + buf->index;

	if(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;

	if((buf->index < 0) || (buf->index > h->hdin_cfg.pp_num))
		return -EAGAIN;

	hdin_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;

	list_add_tail(&hdin_buf->buf_list, &h->list);

	dprintk("[qbuf] %d -> list\n", hdin_buf->v4lbuf.index);
	return 0;
}

int hdin_videobuf_dqbuf(struct tcc_hdin_device *hdev, struct v4l2_buffer *buf, struct file *file )
{
	struct TCC_HDIN *h = hdev->h;
	struct tcc_hdin_buffer *hdin_buf;

	if(list_empty(&h->done_list)) {
		if(file->f_flags & O_NONBLOCK){
			printk("file->f_flags Fail!!\n");
			return -EAGAIN;
		}

		h->wakeup_int = 0;
		if(wait_event_interruptible_timeout(h->frame_wait, h->wakeup_int == 1, msecs_to_jiffies(500)) <= 0) {
			printk("wait_event_interruptible_timeout 500ms!!\n");
			if(check_int == 1)
				check_int = 0;
			return -EFAULT;
		} else if(check_int == 0) {
			printk("interruptible flag is 0 to 1 \n");
			check_int = 1; 
		}

		if(list_empty(&h->done_list)) {
			printk("It needs list_empty!!\n");
			return -ENOMEM;
		}
	}

	hdin_buf = list_entry(h->done_list.next, struct tcc_hdin_buffer, buf_list);
	list_del(h->done_list.next);

	hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
	memcpy(buf, &(hdin_buf->v4lbuf),sizeof(struct v4l2_buffer));
	dprintk("[dqbuf] %d -> list\n", hdin_buf->v4lbuf.index);

	return 0;
}

int hdin_videobuf_streamon(struct tcc_hdin_device *hdev, enum v4l2_buf_type *type)
{
	if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;
	return hdin_video_start_stream(hdev);
}

int hdin_videobuf_streamoff(struct tcc_hdin_device *hdev, enum v4l2_buf_type * type)
{
	struct TCC_HDIN *h = hdev->h;
	struct tcc_hdin_buffer *hdin_buf;

	int ret_val;

	if(*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EAGAIN;

	ret_val = hdin_video_stop_stream(hdev);

	while(!list_empty(&h->done_list)) {
		hdin_buf = list_entry(h->done_list.next, struct tcc_hdin_buffer, buf_list);
		list_del(&hdin_buf->buf_list);
		hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		hdin_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;

		list_add_tail(&hdin_buf->buf_list, &h->list);
	}
	h->done_list.next = &h->done_list;
	return ret_val;
}

int hdin_videobuf_set_address(struct tcc_hdin_device *hdev, struct v4l2_requestbuffers *req)
{
	hdin_video_set_addr(hdev, req->count, req->reserved[0]);
	return 0;
}

int hdin_interrupt_check(struct tcc_hdin_device *hdev, int* arg)
{
	struct TCC_HDIN *h = hdev->h;
	int value = 0;

	if(h->stream_state != STREAM_OFF){
		value = hdin_ctrl_get_resolution(hdev);
		if(hdev->cur_resolution != value && value != 0){
			hdev->cur_resolution = value;
			hdin_video_stop_stream(hdev);
			return OFF;
		}
	}

	if(check_int == 0){
		value = hdin_ctrl_get_resolution(hdev);
		if(value != 0){
			hdev->cur_resolution = value;
			return ON;
		}
	}
	return check_int;
}

static int hdin_main_module_open(struct tcc_hdin_device *hdev)
{
	int ret = -1;

	dprintk("hdin_main_module_open \n");

	if(!hdev->enabled){
		/* setup vioc component */
		//hdin_video_path_mapping(hdev);
		if ((ret = hdin_ctrl_init(hdev)) < 0)
		{
			printk(KERN_ERR DRIVER_NAME ": cannot initialize sensor\n");
			goto error;
		}

		if ((ret = hdin_video_init(hdev)) < 0)
		{
			printk(KERN_ERR DRIVER_NAME ": cannot initialize interface hardware\n");
			goto error;
		}

		if (hdin_video_open(hdev))
		{
			printk (KERN_ERR DRIVER_NAME ": HDIN configuration failed\n");
			goto error;
		}
	}
	return 0;
error :
	hdin_ctrl_cleanup(hdev);
	return -ENODEV;	
}

static long hdin_v4l2_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct tcc_hdin_device *hdev = video_drvdata(file);

	switch (cmd) {
	case VIDIOC_G_FMT:
		return hdin_videobuf_g_fmt(hdev, (struct v4l2_format*)arg);

	case VIDIOC_S_FMT:
		return hdin_videobuf_s_fmt(hdev, (struct v4l2_format*)arg);

	case VIDIOC_QUERYCAP:
		return hdin_videobuf_querycap(hdev, (struct v4l2_capability *)arg);

	case VIDIOC_REQBUFS:
		return hdin_videobuf_reqbufs(hdev, (struct v4l2_requestbuffers *)arg);

	case VIDIOC_QUERYBUF:
		return hdin_videobuf_querybuf(hdev, (struct v4l2_buffer *)arg);

	case VIDIOC_QBUF:
		return hdin_videobuf_qbuf(hdev, (struct v4l2_buffer *)arg);

	case VIDIOC_DQBUF:
		return hdin_videobuf_dqbuf(hdev, (struct v4l2_buffer *)arg, file);

	case VIDIOC_STREAMON:
		return hdin_videobuf_streamon(hdev, (enum v4l2_buf_type *)arg);

	case VIDIOC_STREAMOFF:
		return hdin_videobuf_streamoff(hdev, (enum v4l2_buf_type *)arg);

	case VIDIOC_USER_SET_CAMINFO_TOBEOPEN:
		return hdin_main_module_open(hdev);

	case VIDIOC_USER_GET_MAX_RESOLUTION:
		{
			int res = 0x00;
			res = hdin_ctrl_get_resolution(hdev);
			if(hdev->cur_resolution != res)
			{
				dprintk("change video source\n");
				hdev->cur_resolution = res;
			}
			return res;
		}

	case VIDIOC_USER_GET_SENSOR_FRAMERATE:
		return hdin_ctrl_get_fps((int *)arg);

	case VIDIOC_USER_SET_CAMERA_ADDR:
		return hdin_videobuf_set_address(hdev, (struct v4l2_requestbuffers *)arg);

	case VIDIOC_USER_INT_CHECK: // 20131018 swhwang, for check video frame interrupt
		return hdin_interrupt_check(hdev, (int *)arg);

	case VIDIOC_USER_AUDIO_SR_CHECK:
		return hdin_ctrl_get_audio_samplerate(hdev, (int)arg);

	case VIDIOC_USER_AUDIO_TYPE_CHECK:
		return hdin_ctrl_get_audio_type(hdev, (int)arg);
#ifdef HDIN_DRV_BYPASS_EN
	case VIDIOC_USER_GET_VIDEO_MODE:
		*(int *)arg = hdev->bypass;
		break;

	case VIDIOC_USER_SET_VIDEO_MODE:
		hdin_ctrl_set_bypass_mode(hdev, (int *)arg);
		break;

	case VIDIOC_USER_SET_HDMI_AUDIO:
		writel(*(unsigned int *)arg, IOMEM(HDMI_I2SREG(0x08)));
		break;

	case VIDIOC_USER_GET_HDMI_AUDIO:
		*(unsigned int *)arg = readl(IOMEM(HDMI_I2SREG(0x08)));
		break;
#endif
	default:
		printk(KERN_ERR DRIVER_NAME ": unrecognized ioctl(0x%08x)\n", cmd);
		ret = -1;
		break;
	}

	return ret;
}

static long hdin_main_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, hdin_v4l2_do_ioctl);
}

extern int range_is_allowed(unsigned long pfn, unsigned long size);

static int hdin_main_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "hdin: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		return -EAGAIN;
	}

	vma->vm_ops 	= NULL;
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

	return 0;
}


static int hdin_main_release(struct file *file)
{
	struct tcc_hdin_device *hdev = NULL;

	dprintk("hdin release....\n");

	hdev = video_drvdata(file);
	if(hdev == NULL)
		return -ENODEV;

	hdin_video_close(hdev);
	hdin_clock_disable(hdev);
	return 0;
}

static int hdin_main_open(struct file *file)
{
	int minor = video_devdata(file)->minor;

	dprintk("hdin open.... \n");

	struct tcc_hdin_device *hdev = video_drvdata(file);
	if(hdev == NULL){
		printk(KERN_ERR DRIVER_NAME": drvdata is null \n");
		return -ENODEV;
	}

	if (!hdev->vfd || hdev->vfd->minor != minor){
		printk(KERN_ERR DRIVER_NAME ": minor number mismatch (%d != %d)\n",  hdev->vfd->minor, minor);
		return -ENODEV;
	}

	hdin_clock_enable(hdev);

	return 0;
}

static struct v4l2_file_operations hdmi_receiver_fops =
{
	.owner			= THIS_MODULE,
	.unlocked_ioctl = hdin_main_ioctl,
	.mmap			= hdin_main_mmap,
	.open			= hdin_main_open,
	.release		= hdin_main_release
};

static int hdin_main_probe(struct platform_device *pdev)
{
	struct tcc_hdin_device *hdev = NULL;

	printk("__________hdin_main_probe \n");

	hdev = kzalloc(sizeof(struct tcc_hdin_device), GFP_KERNEL);
	if (hdev == NULL)
		return -ENOMEM;
	strlcpy(hdev->name, pdev->name, sizeof(hdev->name));

	if(pdev->dev.of_node)
	{
		hdev->hdin_np = pdev->dev.of_node;
		hdin_clock_get(hdev);

		hdin_ctrl_get_gpio(hdev);
	}
	else
	{
		printk("could not find hdin node!! \n");
		return -ENODEV;	
	}

	hdev->vfd = video_device_alloc();
	if (!hdev->vfd)
	{
		printk(KERN_ERR DRIVER_NAME": could not allocate video device struct\n");
		return -ENOMEM;
	}

	hdev->vfd->release = video_device_release;

	strlcpy(hdev->vfd->name, DRIVER_NAME, sizeof(hdev->vfd->name));

	hdev->vfd->fops = &hdmi_receiver_fops;
	hdev->vfd->minor = -1;

	if (video_register_device(hdev->vfd, VFL_TYPE_GRABBER, video_nr) < 0)
	{
		printk(KERN_ERR DRIVER_NAME ": could not register Video for Linux device\n");
		video_device_release(hdev->vfd);
		return -ENODEV;
	}

	/* mapping private data */
	video_set_drvdata(hdev->vfd, hdev);
	platform_set_drvdata(pdev, hdev);
	hdev->dev = &pdev->dev;

	printk(KERN_INFO DRIVER_NAME ": registered device video%d [v4l2]\n", hdev->vfd->minor);

	return 0;
}

static int hdin_main_remove(struct platform_device *pdev)
{
	struct tcc_hdin_device *hdev = platform_get_drvdata(pdev);

	dprintk("hdin_main_remove \n");

	video_unregister_device(hdev->vfd);
	video_device_release(hdev->vfd);
	hdin_clock_put(hdev);

	if(hdev->h)
		kfree(hdev->h);
	kfree(hdev);
	return 0;
}

int hdin_main_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tcc_hdin_device *hdev = platform_get_drvdata(pdev);
	hdin_ctrl_key_disable(hdev);
	hdin_ctrl_rst_disable(hdev);
	hdin_ctrl_pwr_disable(hdev);

	return 0;
}

int hdin_main_resume(struct platform_device *pdev)
{
	struct tcc_hdin_device *hdev = platform_get_drvdata(pdev);
	hdin_ctrl_pwr_enable(hdev);
	hdin_ctrl_rst_enable(hdev);
	hdin_ctrl_key_enable(hdev);

	return 0;
}

static struct of_device_id hdin_of_match[] = {
	{ .compatible = "telechips-hdin" },
	{}
};

static struct platform_driver hdmi_receiver_driver = {
	.driver = {
		.name 	= DRIVER_NAME,
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdin_of_match)
	},
	.probe 		= hdin_main_probe,
	.remove 	= hdin_main_remove,
	.suspend    = hdin_main_suspend,
	.resume     = hdin_main_resume
};

void __exit hdin_core_exit(void)
{
	dprintk("hdin_core_exit \n");
	return;
}

int __init hdin_core_init(void)
{
	dprintk("hdin init \n");
	return 0;
}

module_init(hdin_core_init);
module_exit(hdin_core_exit);
module_platform_driver(hdmi_receiver_driver);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
