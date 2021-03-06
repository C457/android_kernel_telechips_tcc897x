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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/jiffies.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>

#include "tcc_hdin_main.h"
#include "tcc_hdin_ctrl.h"
#include "tcc_hdin_video.h"


static int debug	   = 0;
#define dprintk(msg...)	if(debug) { printk( "hdin_main : " msg); }

int hdin_videobuf_g_fmt(struct v4l2_format *fmt, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;

	memset(&fmt->fmt.pix, 0, sizeof (fmt->fmt.pix));
	fmt->fmt.pix = dev->pix_format;
	
	return 0;
}

int hdin_videobuf_s_fmt(struct v4l2_format *fmt, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	
	dev->pix_format.width	= fmt->fmt.pix.width;
	dev->pix_format.height	= fmt->fmt.pix.height;
	dev->pix_format.pixelformat = fmt->fmt.pix.pixelformat;

	return hdin_video_set_resolution(dev, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width, fmt->fmt.pix.height);
}

int hdin_videobuf_querycap(struct v4l2_capability *cap, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	
	memset(cap, 0, sizeof(struct v4l2_capability));
	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, dev->vfd->name, sizeof(cap->card));
	
	cap->bus_info[0] = '\0';
	cap->version = KERNEL_VERSION(3, 18, 00);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |				
						V4L2_CAP_VIDEO_OVERLAY |
						V4L2_CAP_READWRITE | 
						V4L2_CAP_STREAMING;
	return 0;
}

int hdin_videobuf_reqbufs(struct v4l2_requestbuffers *req, struct tcc_hdin_device *vdev)
{	
	struct tcc_hdin_device *dev = vdev;

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

	return hdin_video_buffer_set(dev,req);
}

int hdin_videobuf_qbuf(struct v4l2_buffer *buf, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	struct TCC_HDIN *data = &dev->data;
	struct tcc_hdin_buffer *hdin_buf = data->buf + buf->index;

	if(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;

	if((buf->index < 0) || (buf->index > data->hdin_cfg.pp_num))
		return -EAGAIN;

	if(hdin_buf->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED)
		hdin_buf->v4lbuf.flags |= ~V4L2_BUF_FLAG_QUEUED;

	if(hdin_buf->v4lbuf.flags & V4L2_BUF_FLAG_DONE)
		hdin_buf->v4lbuf.flags |= ~V4L2_BUF_FLAG_DONE;

	hdin_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;

	list_add_tail(&hdin_buf->buf_list, &data->list);

	return 0;			
}

int hdin_videobuf_dqbuf(struct v4l2_buffer *buf, struct file *file)
{
	struct tcc_hdin_device *dev = video_drvdata(file);
	struct TCC_HDIN *data = &dev->data;
	struct tcc_hdin_buffer *hdin_buf;

	if(list_empty(&data->done_list)){
		
		if(file->f_flags & O_NONBLOCK){
			printk("file->f_flags Fail!!\n");
			return -EAGAIN;
		}
			
		data->wakeup_int = 0;
		
		if(wait_event_interruptible_timeout(data->frame_wait, data->wakeup_int == 1, msecs_to_jiffies(500)) <= 0)
		{
			if(dev->interrupt_status == 1)	dev->interrupt_status = 0; 
			
			printk("wait wdma interrupt... wait time is 500ms!!\n");
	
			return -EFAULT;
		} 

		if(list_empty(&data->done_list))
		{
			printk("It needs list_empty!!\n");
			return -ENOMEM;
		}

	}

	hdin_buf = list_entry(data->done_list.next, struct tcc_hdin_buffer, buf_list);
	list_del(data->done_list.next);
	hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
	memcpy(buf, &(hdin_buf->v4lbuf),sizeof(struct v4l2_buffer));

	if(dev->interrupt_status == 0)
	{
		dev->interrupt_status = 1; 
		printk("interruptible flag is 0 to 1 \n");		
	}
	
	return 0;
}

int hdin_videobuf_streamon(enum v4l2_buf_type *type, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	
	if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;
	
	return hdin_video_start_stream(dev);
}

int hdin_videobuf_streamoff(enum v4l2_buf_type * type, struct tcc_hdin_device *vdev)
{
	
	struct tcc_hdin_device *dev = vdev;	
	struct TCC_HDIN *data = &dev->data;	
	struct tcc_hdin_buffer *hdin_buf;		
	int ret_val;

	if(*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EAGAIN;

	ret_val = hdin_video_stop_stream(dev);

	while(!list_empty(&data->done_list))
	{
		hdin_buf = list_entry(data->done_list.next, struct tcc_hdin_buffer, buf_list);
		list_del(&hdin_buf->buf_list);
		hdin_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		hdin_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
		list_add_tail(&hdin_buf->buf_list, &data->list);
	}
	
	data->done_list.next = &data->done_list;

	return ret_val;
}

int hdin_videobuf_set_camera_addr(struct v4l2_requestbuffers *req, struct tcc_hdin_device *vdev)
{
	struct tcc_hdin_device *dev = vdev;
	
	hdin_video_set_addr(dev, req->count, req->reserved[0]);
	
	return 0;
}

int hdin_interrupt_check(int* arg, struct file *file)
{
	struct tcc_hdin_device *dev = video_drvdata(file);
	struct TCC_HDIN *data = &dev->data;	
	int value = 0;
	
	if(data->stream_state != STREAM_OFF)
	{
		if((value = hdin_ctrl_get_resolution(file)) == 0x00)
		{
			dev->current_resolution = -1;
			return 0;
		}

		if(dev->current_resolution == -1)
		{
			dev->current_resolution = value;
			return 0;
		}
		
		if(dev->current_resolution != value && value != 0)
		{
			dev->current_resolution  = value;
			hdin_video_stop_stream(dev);
			return 0;
		}
	}
	
	return dev->interrupt_status;
}

static int hdin_main_module_open(struct file *file)
{
	struct tcc_hdin_device *dev = video_drvdata(file);
	int ret;

	dprintk("hdin_main_module_open \n");
	
	if ((ret = hdin_ctrl_init(file)) < 0) 
	{
		printk(KERN_ERR DRIVER_NAME ": cannot initialize sensor\n");
		goto error;
	}	
	
	if ((ret = hdin_video_init(dev)) < 0) 
	{
		printk(KERN_ERR DRIVER_NAME ": cannot initialize interface hardware\n");
		goto error;
	}	

	if ((ret = hdin_video_open(dev)) < 0)
	{
		printk (KERN_ERR DRIVER_NAME ": HDIN configuration failed\n");
		goto error;
	}

	return 0;

error :

	hdin_ctrl_cleanup(file);
	return -ENODEV;	
}
static long hdin_v4l2_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct tcc_hdin_device *vdev = video_drvdata(file);

	switch (cmd) {

		case VIDIOC_G_FMT:
			return hdin_videobuf_g_fmt((struct v4l2_format*)arg,vdev);

		case VIDIOC_S_FMT:
			return hdin_videobuf_s_fmt((struct v4l2_format*)arg,vdev);

		case VIDIOC_QUERYCAP:
			return hdin_videobuf_querycap((struct v4l2_capability *)arg,vdev);

		case VIDIOC_REQBUFS:
			return hdin_videobuf_reqbufs((struct v4l2_requestbuffers *)arg,vdev);  

		case VIDIOC_QBUF:
			return hdin_videobuf_qbuf((struct v4l2_buffer *)arg,vdev);						

		case VIDIOC_DQBUF:
			return hdin_videobuf_dqbuf((struct v4l2_buffer *)arg, file);						
			
		case VIDIOC_STREAMON:
			return hdin_videobuf_streamon((enum v4l2_buf_type *)arg,vdev);

		case VIDIOC_STREAMOFF:
			return hdin_videobuf_streamoff((enum v4l2_buf_type *)arg,vdev);

		case VIDIOC_USER_SET_CAMINFO_TOBEOPEN:
			return hdin_main_module_open(file);

		case VIDIOC_USER_GET_MAX_RESOLUTION:
			return hdin_ctrl_get_resolution(file);

		case VIDIOC_USER_GET_SENSOR_FRAMERATE:
			return hdin_ctrl_get_fps((int *)arg);

		case VIDIOC_USER_SET_CAMERA_ADDR:
			return hdin_videobuf_set_camera_addr((struct v4l2_requestbuffers *)arg, vdev);
			
		case VIDIOC_USER_INT_CHECK: // 20131018 swhwang, for check video frame interrupt
			return hdin_interrupt_check((int *)arg, file);
			
		case VIDIOC_USER_AUDIO_SR_CHECK:
			return hdin_ctrl_get_audio_samplerate(vdev);
			
		case VIDIOC_USER_AUDIO_TYPE_CHECK:
			return hdin_ctrl_get_audio_type(vdev);

		case VIDIOC_USER_AUDIO_OUTPUT_CHECK:
			return hdin_ctrl_get_audio_output_signal(vdev);

	}

	return 0;
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
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_RESERVED;
	
	return 0;
}


static int hdin_main_release(struct file *file)
{
	struct tcc_hdin_device *vdev = video_drvdata(file);

	dprintk("hdin release....\n");

	hdin_video_close(file);

	clk_disable_unprepare(vdev->hdin_clk);	

	return 0;
}

static int hdin_main_open(struct file *file)
{
	struct tcc_hdin_device *vdev = video_drvdata(file);

	dprintk("hdin open.... \n");	

	if(vdev == NULL) return -ENODEV;

	if (!vdev->vfd || (vdev->vfd->minor == -1))
		return -ENODEV;

	clk_prepare_enable(vdev->hdin_clk);

	return 0;
	
}

static struct v4l2_file_operations hdmi_receiver_fops = 
{
	.owner          = THIS_MODULE,
	.unlocked_ioctl = hdin_main_ioctl,
	.mmap           = hdin_main_mmap,
	.open           = hdin_main_open,
	.release        = hdin_main_release
};

static int hdin_main_probe(struct platform_device *pdev)
{
	struct tcc_hdin_device	*vdev;

	printk("__________hdin_main_probe \n");

	vdev = kzalloc(sizeof(struct tcc_hdin_device),GFP_KERNEL);
	if(vdev == NULL)
		return -ENOMEM;

	if(pdev->dev.of_node)
	{
		vdev->np = pdev->dev.of_node;

		if(vdev->np)
		{
			vdev->hdin_clk= of_clk_get(vdev->np, 0);
		}		

		hdin_ctrl_get_gpio(vdev);
	}
	else
	{
		printk("could not find hdin node!! \n");
		return -ENODEV;	
	}

	vdev->vfd = video_device_alloc();
	
	if (!vdev->vfd) 
	{
		printk(KERN_ERR DRIVER_NAME": could not allocate video device struct\n");
		return -ENOMEM;

	}

	vdev->vfd->v4l2_dev = kzalloc(sizeof(struct v4l2_device), GFP_KERNEL);

	if(v4l2_device_register(&pdev->dev, vdev->vfd->v4l2_dev) < 0){
		printk(KERN_ERR "%s : [error] v4l2 register failed\n", pdev->name);
		return -ENODEV;
	}

	
 	vdev->vfd->release = video_device_release;

 	strlcpy(vdev->vfd->name, DRIVER_NAME, sizeof(vdev->vfd->name));
 	
 	vdev->vfd->fops = &hdmi_receiver_fops;
 	vdev->vfd->minor = -1;

	if (video_register_device(vdev->vfd, VFL_TYPE_GRABBER, -1) < 0) 
	{
		printk(KERN_ERR DRIVER_NAME ": could not register Video for Linux device\n");
		video_device_release(vdev->vfd);
		return -ENODEV;
	}

	video_set_drvdata(vdev->vfd, vdev);
	platform_set_drvdata(pdev, vdev);

	printk(KERN_INFO DRIVER_NAME ": registered device video%d [v4l2]\n", vdev->vfd->minor);

	return 0;

}

static int hdin_main_remove(struct platform_device *pdev)
{
	struct tcc_hdin_device	*vdev = platform_get_drvdata(pdev);
	
	dprintk("hdin_main_remove \n");

	if(vdev->hdin_clk)
		clk_put(vdev->hdin_clk);

	video_unregister_device(vdev->vfd);
	video_device_release(vdev->vfd);
	kfree(vdev->vfd->v4l2_dev);
	kfree(vdev);

	return 0;
}

int hdin_main_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tcc_hdin_device	*vdev = platform_get_drvdata(pdev);
	
	hdin_ctrl_port_disable(vdev->gpio.key_port);
	hdin_ctrl_port_disable(vdev->gpio.rst_port);	
	hdin_ctrl_port_disable(vdev->gpio.pwr_port);	

	return 0;
}

int hdin_main_resume(struct platform_device *pdev)
{
	struct tcc_hdin_device	*vdev = platform_get_drvdata(pdev);	
	
	hdin_ctrl_port_enable(vdev->gpio.pwr_port);
	hdin_ctrl_port_enable(vdev->gpio.rst_port);
	hdin_ctrl_port_enable(vdev->gpio.key_port);

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

