/*
 * drivers/media/video/tccvin/tcc_vin_v4l2.c
 *
 * Copyright (C) 2008 Telechips, Inc. 
 *
 * Video-for-Linux (Version 2) camera capture driver for Telechisp SoC.
 *
 * leverage some code from CEE distribution 
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */
 
#include <linux/kernel.h>
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
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include "tcc_vin.h"
#include "tcc_vin_core.h"
#include "tcc_vin_hw.h"
#include <mach/gpio.h>
#include <mach/tcc_cam_ioctrl.h>	// for private ioctl

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include <media/v4l2-device.h>

#ifdef CONFIG_TCC_VIN_DEBUG
#define dprintk(fmt, args...) printk("\e[33m[vin_v4l2]%s(%d) \e[0m" fmt, __func__, __LINE__, ## args);
#else
#define dprintk(fmt, args...)
#endif

extern int tcc_is_camera_enable;
extern int Get_Index_sizeTable(unsigned int Image_Size);
extern int cif_set_i2c_name(int id, const char * cam_name);
extern int cif_get_client(struct tcc_video_device *vdev);

/* Not used clock limit table */
//#if defined(CONFIG_CPU_FREQ)
//extern struct tcc_freq_table_t gtCameraClockLimitTable[];
//extern struct tcc_freq_table_t gtISPCameraClockLimitTable[];
//#endif

// TODO: earysuspend crash tcc_battery
//#ifdef CONFIG_HAS_EARLYSUSPEND
//static struct early_suspend early_suspend;
//static void tcc92xx_camera_early_suspend(struct early_suspend *h)
//{
//}
//static void tcc92xx_camera_late_resume(struct early_suspend *h)
//{
//}
//#endif

extern int cif_i2c_init(struct tcc_video_device *vdev);
extern int cif_i2c_exit(struct tcc_video_device *vdev);

struct device_node * np_camera;

#ifdef CONFIG_TCC_VIN_DEBUG
static void dump_vdev(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	dprintk("---------- tcc_video_device(%d) infomation ----------\n", vdev->video_nr);
	dprintk("name: %s\n", vdev->name);
	dprintk("pmap base: 0x%08x\n", vdev->core.pmap_preview.base);
	dprintk("     size: 0x%08x\n", vdev->core.pmap_preview.size);
	dprintk("\n");

	if (vdev->vioc.vin_demux.id == VIOC_NULL) {
		dprintk("vin_demux  id: not use\n");
	} else {
		if (vdev->vioc.vin_demux.id < VIOC_VD_MP_00) {
			dprintk("vin_demux  id: external port%d\n", vdev->vioc.vin_demux.id - VIOC_VD_EP_00);
		} else {
			dprintk("vin_demux  id: multiplex port%d\n", vdev->vioc.vin_demux.id - VIOC_VD_MP_00);
		}
	}
	dprintk("\n");

	dprintk("vin        id: vin%d (%d)\n", vdev->vioc.vioc_num.vin.index);//vdev->vioc.vin.id - VIOC_VIN_00, vdev->vioc.vin.id);
	dprintk("          idx: %d\n", vdev->vioc.vin.idx);
	dprintk("    phys_addr: 0x%08x\n", vdev->vioc.vin.phys_addr);
	dprintk("    s/w reset: 0x%08x\n", vdev->vioc.vin.swrst_bit);
	dprintk("\n");

	if (vdev->vioc.viqe.id == VIOC_NULL) {
		dprintk("viqe       id: not use\n");
	} else {
		dprintk("viqe       id: viqe%d (%d)\n", vdev->vioc.viqe.id - VIOC_VIQE_00, vdev->vioc.viqe.id);
		dprintk("          idx: %d\n", vdev->vioc.viqe.idx);
		dprintk("    phys_addr: 0x%08x\n", vdev->vioc.viqe.phys_addr);
		dprintk("    s/w reset: 0x%08x\n", vdev->vioc.viqe.swrst_bit);
	}
	dprintk("\n");

	if (vdev->vioc.deintls.id == VIOC_NULL) {
		dprintk("deintls    id: not use\n");
	} else {
		dprintk("deintls    id: deintls%d (%d)\n", vdev->vioc.deintls.id - VIOC_DEINTLS_00, vdev->vioc.deintls.id);
		dprintk("          idx: %d\n", vdev->vioc.deintls.idx);
		dprintk("    phys_addr: 0x%08x\n", vdev->vioc.deintls.phys_addr);
		dprintk("    s/w reset: 0x%08x\n", vdev->vioc.deintls.swrst_bit);
	}
	dprintk("\n");

	if (vdev->vioc.sc.id == VIOC_NULL) {
		dprintk("sc         id: not use\n");
	} else {
		dprintk("sc         id: sc%d (%d)\n", vdev->vioc.sc.id - VIOC_SC_00, vdev->vioc.sc.id);
		dprintk("          idx: %d\n", vdev->vioc.sc.idx);
		dprintk("    phys_addr: 0x%08x\n", vdev->vioc.sc.phys_addr);
		dprintk("    s/w reset: 0x%08x\n", vdev->vioc.sc.swrst_bit);
	}
	dprintk("\n");

	dprintk("wmix       id: wmix%d (%d)\n", vdev->vioc.wmix.id - VIOC_WMIX_00, vdev->vioc.wmix.id);
	dprintk("          idx: %d\n", vdev->vioc.wmix.idx);
	dprintk("    phys_addr: 0x%08x\n", vdev->vioc.wmix.phys_addr);
	dprintk("    s/w reset: 0x%08x\n", vdev->vioc.wmix.swrst_bit);
	dprintk("\n");

	dprintk("wdma       id: wdma%d (%d)\n", vdev->vioc.wdma.id - VIOC_WDMA_00, vdev->vioc.wdma.id);
	dprintk("          idx: %d\n", vdev->vioc.wdma.idx);
	dprintk("    phys_addr: 0x%08x\n", vdev->vioc.wdma.phys_addr);
	dprintk("    s/w reset: 0x%08x\n", vdev->vioc.wdma.swrst_bit);
	dprintk("     transfer: %s mode\n", vdev->vioc.wdma_continuous_mode ? "continuous" : "frame-by-frame");
	dprintk("\n");

	dprintk("vioc       wdma irq: %d\n", vdev->vioc.wdma.irq);
	if (vdev->vioc.vsync_ecc)
		dprintk("(vsync ecc) vin irq: %d\n", vdev->vioc.vin.irq);
	dprintk("          sc_plugin: 0x%x\n", vdev->vioc.sc_plugin);
	dprintk("        viqe_plugin: 0x%x\n", vdev->vioc.viqe_plugin);
	dprintk("\n");

	dprintk("sinfo        name: %s\n", vdev->sinfo->name);
	dprintk("     in_data bits: %d\n", vdev->sinfo->nr_data_bits);
	dprintk("         i2c_addr: 0x%x\n", vdev->sinfo->i2c_addr);
	dprintk("            prv_w: %d\n", vdev->sinfo->prv_w);
	dprintk("            prv_h: %d\n", vdev->sinfo->prv_h);
	dprintk("        framerate: %d\n", vdev->sinfo->framerate);
	dprintk("             mclk: %d\n", vdev->sinfo->mclk);
	dprintk("         pol_pclk: %d\n", vdev->sinfo->polarity_pclk);
	dprintk("        pol_hsync: %d\n", vdev->sinfo->polarity_hsync);
	dprintk("        pol_vsync: %d\n", vdev->sinfo->polarity_vsync);
	dprintk("         de2hsync: %d\n", vdev->sinfo->de2hsync);
	dprintk("   interface_type: %d\n", vdev->sinfo->interface_type);
	dprintk("        scan_type: %d\n", vdev->sinfo->scan_type);
	dprintk("     de_interlace: %d\n", vdev->sinfo->de_interlace);
	dprintk("   frame_skip_irq: %d\n", vdev->sinfo->frame_skip_irq);
	dprintk("   frame_skip_vin: %d\n", vdev->sinfo->frame_skip_vin);
	dprintk("        zoom_type: %d\n", vdev->sinfo->zoom_type);
	dprintk("\n");

	dprintk("sensor_type: %d\n", pdata->sensor_type);
	dprintk("cif port_nr: %d\n", pdata->port_nr);
	dprintk("  clko_name: %s\n", pdata->port->clko_name);

	if (vdev->cif_i2c_client) {
		dprintk(" i2c_client: %s\n", vdev->cif_i2c_client->name);
		dprintk("   i2c_addr: 0x%x\n", vdev->cif_i2c_client->addr);
	}
	dprintk("-----------------------------------------------------\n");
}
#endif

static int tcc_videobuf_inputenum(struct v4l2_input *input)
{
	/* default handler assumes 1 video input (the camera) */
	int index = input->index;

	memset(input, 0, sizeof(*input));
	input->index = index;

	if (index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int tcc_videobuf_g_input(unsigned int *input)
{
	*input = 0;
	
	return 0;
}

static int tcc_videobuf_s_input(unsigned int *input)
{
	if (*input > 0)
		return -EINVAL;

	return 0;
}

static int tcc_videobuf_g_param(struct tcc_video_device *vdev, struct v4l2_streamparm *gparam)
{
	struct TCCxxxCIF *h = vdev->h;
	memset(gparam,0x00,sizeof(*gparam));
	gparam->parm.capture.capturemode = h->cif_cfg.oper_mode;
	return 0;
}

static int tcc_videobuf_s_param(struct v4l2_streamparm *sparam)
{
	return 0;
}

static int tcc_videobuf_enum_fmt(struct v4l2_fmtdesc *fmt)
{	
	return sensor_if_enum_pixformat(fmt);
}

static int tcc_videobuf_try_fmt(struct v4l2_format *fmt)
{	
	//return sensor_try_format(&fmt->fmt.pix);
	return 0;
}

static int tcc_videobuf_g_fmt(struct v4l2_pix_format pix, struct v4l2_format *fmt)
{
	/* get the current format */
	memset(&fmt->fmt.pix, 0, sizeof (fmt->fmt.pix));
	fmt->fmt.pix = pix;
	
	return 0;
}

static int tcc_videobuf_s_fmt(struct tcc_video_device *vdev, struct v4l2_format *fmt)
{
	unsigned int temp_sizeimage = 0;
	temp_sizeimage = vdev->pix.sizeimage;
	vdev->pix.width = fmt->fmt.pix.width;
	vdev->pix.height = fmt->fmt.pix.height;
	return vincore_set_resolution(vdev, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width, fmt->fmt.pix.height);
}

static int tcc_videobuf_querycap(struct tcc_video_device *vdev, struct v4l2_capability *cap)
{
	//memset(cap, 0, sizeof(struct v4l2_capability));
	strlcpy(cap->driver, vdev->name, sizeof(cap->driver));
	strlcpy(cap->card, vdev->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->version = KERNEL_VERSION(3, 0, 8);
	cap->capabilities = V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_STREAMING;
	return 0;
}

static int tcc_videobuf_g_fbuf(struct v4l2_framebuffer fbuf, struct v4l2_framebuffer *fargbuf)
{
	memcpy(fargbuf, &fbuf, sizeof(struct v4l2_framebuffer));
	return 0;
}

static int tcc_videobuf_s_fbuf(struct v4l2_framebuffer fbuf, struct v4l2_framebuffer *fargbuf)
{
	fbuf.base = fargbuf->base;
	fbuf.fmt = fargbuf->fmt;
	return 0;
}

static int tcc_videobuf_reqbufs(struct tcc_video_device *vdev, struct v4l2_requestbuffers *req)
{
	if(req->memory != V4L2_MEMORY_MMAP && req->memory != V4L2_MEMORY_USERPTR \
		&& req->memory != V4L2_MEMORY_OVERLAY) {
		printk("reqbufs: memory type invalid\n");
		return -EINVAL;
	}
	
	if(req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk("reqbufs: video type invalid\n");
		return -EINVAL;
	}

	return vincore_set_bufs(vdev, req);
}

static int tcc_videobuf_querybuf(struct tcc_video_device *vdev, struct v4l2_buffer *buf)
{
	struct TCCxxxCIF *h = vdev->h;
	struct v4l2_pix_format pix = vdev->pix;
	struct tccxxx_cif_buffer *cif_buf = h->buf + buf->index;
	int index = buf->index;	

	if (index < 0 || index > h->cif_cfg.pp_num) {
		printk(KERN_WARNING "querybuf error : index : %d / %d",index, h->cif_cfg.pp_num);
		return -EINVAL;
	}
	
	memset(buf, 0, sizeof(*buf));
	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->index = index;
	buf->flags = V4L2_BUF_FLAG_MAPPED;
	buf->flags |= cif_buf->v4lbuf.flags;
	buf->field = V4L2_FIELD_NONE;
	buf->timestamp = cif_buf->v4lbuf.timestamp;
	buf->sequence = cif_buf->v4lbuf.sequence;
	buf->memory = V4L2_MEMORY_MMAP;

	dprintk("pix.width x height: %d x %d\n", pix.width, pix.height);

	if (h->cif_cfg.pfmt == TCC_PFMT_YUV420)
		buf->length = PAGE_ALIGN(pix.width * pix.height * 3 / 2);
	else if (h->cif_cfg.pfmt == TCC_PFMT_YUV422)
		buf->length = PAGE_ALIGN(pix.width * pix.height * 2);
	else if (h->cif_cfg.pfmt == TCC_PFMT_RGB)
		buf->length = PAGE_ALIGN(pix.width * h->cif_cfg.bpp * pix.height);

	#if defined(CONFIG_ARCH_TCC892X)
	if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO) {
		/* If application want offset informations then
		 * application can use below values (optional).
		 * This value was set by vincore_set_bufs().
		 */
		buf->length = vdev->core.offset_total;
		buf->sequence = vdev->core.offset_y;
		buf->reserved = vdev->core.offset_uv;
	}
	#endif

	cif_buf->v4lbuf.index = index;
	cif_buf->v4lbuf.length = buf->length;
	cif_buf->v4lbuf.m.offset = buf->index * buf->length;	//data->cif_cfg.preview_buf[index].p_Y;
	buf->m.offset = cif_buf->v4lbuf.m.offset;

	dprintk("buf->m.offset(0x%x), buf->length(0x%x), data->cif_cfg.base_buf(0x%x)\n", 
		buf->m.offset, buf->length, h->cif_cfg.base_buf);

	/* NOTE: add base address. (team app. wanted)
	 */
	cif_buf->v4lbuf.m.offset += h->cif_cfg.base_buf;
	buf->m.offset = cif_buf->v4lbuf.m.offset;

	dprintk("<%d :: [PA]0x%x / flag: 0x%x  >\n", index, (unsigned int)(cif_buf->v4lbuf.m.offset), (unsigned int)(buf->flags));		
	return 0;
}

static int tcc_videobuf_qbuf(struct tcc_video_device *vdev, struct v4l2_buffer *buf)
{
	struct TCCxxxCIF *h = vdev->h;
	struct tccxxx_cif_buffer *cif_buf = h->buf + buf->index;

//	dprintk("__ in \n");
	
	spin_lock_irqsave(&h->dev_lock, h->dev_lock_flag);

	if(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;

	if((buf->index < 0) || (buf->index > h->cif_cfg.pp_num))
		return -EAGAIN;

	cif_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	cif_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;

	list_add_tail(&cif_buf->buf_list, &h->list);

	spin_unlock_irqrestore(&h->dev_lock, h->dev_lock_flag);

	printk("[qbuf] %d -> list\n", cif_buf->v4lbuf.index);
#ifdef DEBUG_BUF
	printk("[qbuf] %d -> list\n", cif_buf->v4lbuf.index);
#endif
	return 0;			
}

static int tcc_videobuf_dqbuf(struct tcc_video_device *vdev, struct v4l2_buffer *buf, struct file *file)
{
	struct TCCxxxCIF *h = vdev->h;
	struct tccxxx_cif_buffer *cif_buf;
	int i = 0;

	if(h->cif_cfg.esd_restart) vincore_restart(vdev);

	if(list_empty(&h->done_list)) {
		if(file->f_flags & O_NONBLOCK){
			printk("file->f_flags Fail!!\n");
			return -EAGAIN;
		}
			
retry:
		h->wakeup_int = 0;
		if(wait_event_interruptible_timeout(h->frame_wait, h->wakeup_int == 1, msecs_to_jiffies(500)) <= 0){
			printk("wait_event_interruptible_timeout 500ms!!\n");
			return -EFAULT;
		}

		/* Should probably recheck !list_empty() here */
		if(list_empty(&h->done_list)){
			//printk("get frame(retry %d)\n", i);
			if (i++ < 2)
				goto retry;
			printk("It needs list_empty\n");
			return -ENOMEM;
		}
	}

#ifdef DEBUG_BUF
	{
		struct list_head *p;
		struct tccxxx_cif_buffer *tbuf;
		printk("[dqbuf] done_list: ");
		list_for_each(p, &h->done_list) {
			tbuf = list_entry(p, struct tccxxx_cif_buffer, buf_list);
			printk("[%d]", tbuf->v4lbuf.index);
		}
		printk("\n");
	}
#endif

	cif_buf = list_entry(h->done_list.next, struct tccxxx_cif_buffer, buf_list);
	list_del(h->done_list.next);

	cif_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	cif_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;

	memcpy(buf, &(cif_buf->v4lbuf),sizeof(struct v4l2_buffer));

//	dprintk("__ out \n");
	
	return 0;
}

static void tcc_set_clock(struct v4l2_pix_format pix, unsigned char cpu_max)
{
//#if defined(CONFIG_CPU_FREQ)
//	int index;
//	struct tcc_freq_table_t clk_table;
//
//	dprintk(" tcc_set_clock() in \n");
//
//	index = Get_Index_sizeTable(pix.width * pix.height);
//	dprintk("clock index = %d \n", index);
//
//	memcpy(&clk_table, &gtCameraClockLimitTable[index], sizeof(struct tcc_freq_table_t));
//
//	#if defined(CONFIG_VIDEO_TCC_ENABLE_MAX_CLK)
//	clk_table.mem_freq = gtCameraClockLimitTable[(NUM_FREQS -1)].mem_freq;
//	#endif
//
//	if(cpu_max)
//		clk_table.cpu_freq = 600000;
//
//	dprintk("Camera Clk_idx[%d]-[%d/%d]-[%d/%d] :: res = %d x %d\n", index, clk_table.cpu_freq/1000, clk_table.mem_freq/1000, 
//						clk_table.vbus_freq/10000, clk_table.vcod_freq/10000, pix.width, pix.height);
//
//	tcc_cpufreq_set_limit_table(&clk_table, TCC_FREQ_LIMIT_CAMERA, 1);
//
//	dprintk(" tcc_set_clock() out \n");
//#endif
}

static void tcc_reset_clock(void)
{
//#if defined(CONFIG_CPU_FREQ)
//	dprintk("tcc_reset_clock in \n");
//	tcc_cpufreq_set_limit_table(&gtCameraClockLimitTable[0], TCC_FREQ_LIMIT_CAMERA, 0);
//	dprintk("tcc_reset_clock out \n");
//#endif
}

static int tcc_videobuf_streamon(struct tcc_video_device *vdev, enum v4l2_buf_type *type)
{
	if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EAGAIN;
	
	tcc_set_clock(vdev->pix, 0);
	return vincore_start_stream(vdev);
}

//stop capture
static int tcc_videobuf_streamoff(struct tcc_video_device *vdev, enum v4l2_buf_type *type )
{	
	struct TCCxxxCIF *h = vdev->h;	
	struct tccxxx_cif_buffer *cif_buf;		

	int ret_val;

	if(*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EAGAIN;

	ret_val = vincore_stop_stream(vdev);

	while(!list_empty(&h->done_list)) {
		cif_buf = list_entry(h->done_list.next, struct tccxxx_cif_buffer, buf_list);

		list_del(&cif_buf->buf_list);

		cif_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		cif_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;

		list_add_tail(&cif_buf->buf_list, &h->list);
	}
	h->done_list.next = &h->done_list;

	return ret_val;
}

static int camera_core_sensor_open(struct tcc_video_device *vdev)
{
	int ret;

	dprintk("__ in \n");
	
	/* setup vioc compoent */
	vioc_vin_path_mapping(vdev);

#if defined(CONFIG_TCC_REAR_CAMERA_DRV)

	ret = strncmp(vdev->name, "tcc-vin-video0",14);
	if(ret == 0){
		dprintk(" %s is not initialize for rear_cam\n",vdev->name);
		vdev->sinfo->init_mode = SINIT_ZOMBI_OPEN; 
	}
#endif

	/* initialize the sensor and define a default capture format cam->pix */
	if (vdev->sinfo->init_mode < SINIT_ZOMBI_OPEN) {
		ret = sensor_if_init(vdev);
		if (ret < 0) {
			printk(KERN_ERR "%s: cannot initialize sensor\n", vdev->name);
			goto error;
		}
		if (vdev->sinfo->init_mode == SINIT_ZOMBI)
			vdev->sinfo->init_mode = SINIT_ZOMBI_OPEN;
	}
	
	// ioremapﾽￃ probe﾿ﾡﾼﾭ ﾼ￶ￇ￠ﾽￃ virtual address problem￀ﾸﾷￎ ￀ￌﾵ﾿.!!
	ret = vincore_init(vdev);
	if (ret < 0) {
		printk(KERN_ERR "%s: cannot initialize interface hardware\n", vdev->name);
		goto error;
	}	

	ret = vincore_open(vdev);
	if (ret < 0) {
		printk (KERN_ERR "%s: Camera IF configuration failed\n", vdev->name);
		goto error;
	}

	dprintk("out \n");
	return 0;
error:
	sensor_if_deinit(vdev);
	return -ENODEV;	
}


/* ---------------------------------------------------------------------------- */

static long camera_core_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct tcc_video_device *vdev = video_drvdata(file);

	switch (cmd) {
		case VIDIOC_ENUMINPUT:
			return tcc_videobuf_inputenum((struct v4l2_input *)arg);
			
		case VIDIOC_G_INPUT:
			return tcc_videobuf_g_input((unsigned int*)arg);

		case VIDIOC_S_INPUT:
			return tcc_videobuf_s_input((unsigned int*)arg);
		
		case VIDIOC_G_PARM:
			return tcc_videobuf_g_param(vdev, (struct v4l2_streamparm*)arg);

		case VIDIOC_S_PARM:
			return tcc_videobuf_s_param((struct v4l2_streamparm*)arg);
		
		case VIDIOC_ENUM_FMT:
			return tcc_videobuf_enum_fmt((struct v4l2_fmtdesc*)arg);

		case VIDIOC_TRY_FMT:
			return tcc_videobuf_try_fmt((struct v4l2_format*)arg);

		case VIDIOC_G_FMT:
			return tcc_videobuf_g_fmt(vdev->pix, (struct v4l2_format*)arg);

		case VIDIOC_S_FMT:
			return tcc_videobuf_s_fmt(vdev, (struct v4l2_format*)arg);

		case VIDIOC_QUERYCTRL:
			return sensor_if_query_control(vdev, (struct v4l2_queryctrl *)arg);

		case VIDIOC_G_CTRL:
			return sensor_if_get_control(vdev, (struct v4l2_control *)arg);

		case VIDIOC_S_CTRL:
			return sensor_if_set_control(vdev, (struct v4l2_control *)arg, 0);

		case VIDIOC_QUERYCAP:
			return tcc_videobuf_querycap(vdev, (struct v4l2_capability *)arg);

		case VIDIOC_G_FBUF: /* Get the frame buffer parameters */
			return tcc_videobuf_g_fbuf(vdev->fbuf, (struct v4l2_framebuffer *)arg);

		case VIDIOC_S_FBUF: /* set the frame buffer parameters */
			return tcc_videobuf_s_fbuf(vdev->fbuf, (struct v4l2_framebuffer *)arg);

		case VIDIOC_REQBUFS:
			return tcc_videobuf_reqbufs(vdev, (struct v4l2_requestbuffers *)arg);            

		case VIDIOC_QUERYBUF:
			return tcc_videobuf_querybuf(vdev, (struct v4l2_buffer *)arg);			

		case VIDIOC_QBUF:
			return tcc_videobuf_qbuf(vdev, (struct v4l2_buffer *)arg);						

		case VIDIOC_DQBUF:
			return tcc_videobuf_dqbuf(vdev, (struct v4l2_buffer *)arg, file);						
			
		case VIDIOC_STREAMON:
			return tcc_videobuf_streamon(vdev, (enum v4l2_buf_type *)arg);

		case VIDIOC_STREAMOFF:
			return tcc_videobuf_streamoff(vdev, (enum v4l2_buf_type *)arg);

		case VIDIOC_OVERLAY:
			return -EINVAL;

		case VIDIOC_ENUMSTD:
		case VIDIOC_G_STD:
		case VIDIOC_S_STD:
		case VIDIOC_QUERYSTD:
		{
			/* Digital cameras don't have an analog video standard, 
			 * so we don't need to implement these ioctls.
			 */
			 return -EINVAL;
		}
		case VIDIOC_G_AUDIO:
		case VIDIOC_S_AUDIO:
		case VIDIOC_G_AUDOUT:
		case VIDIOC_S_AUDOUT:
		{
			/* we don't have any audio inputs or outputs */
			return -EINVAL;
		}

		case VIDIOC_G_JPEGCOMP:
		case VIDIOC_S_JPEGCOMP:
		{
			/* JPEG compression is not supported */
			return -EINVAL;
		}

		case VIDIOC_G_TUNER:
		case VIDIOC_S_TUNER:
		case VIDIOC_G_MODULATOR:
		case VIDIOC_S_MODULATOR:
		case VIDIOC_G_FREQUENCY:
		case VIDIOC_S_FREQUENCY:
		{
			/* we don't have a tuner or modulator */
			return -EINVAL;
		}

		case VIDIOC_ENUMOUTPUT:
		case VIDIOC_G_OUTPUT:
		case VIDIOC_S_OUTPUT:
		{
			/* we don't have any video outputs */
			return -EINVAL;
		}

		case VIDIOC_USER_GET_MAX_RESOLUTION:
		{
			return -EINVAL;
		}

		case VIDIOC_USER_GET_SENSOR_FRAMERATE:
			return sensor_if_get_sensor_framerate(vdev, (int *)arg);

		default:
		{
			/* unrecognized ioctl */
			return -ENOIOCTLCMD;
		}
	}

	return 0;
}

/*
 *  file operations
 */
static unsigned int camera_core_poll(struct file *file, struct poll_table_struct *wait)
{
	struct tcc_video_device *vdev = video_drvdata(file);
	struct TCCxxxCIF *h = vdev->h;

	poll_wait(file, &(h->frame_wait), wait);	

	if(h->cif_cfg.cap_status == CAPTURE_DONE)
	{
		dprintk("POLL IN ! \r\n");
		dprintk("CAM CLOCK :: initialized to 0 \n");
		//tcc_reset_clock();
		return POLLIN;
	}
	else if(h->cif_cfg.cap_status == CAPTURE_OVERFLOW)
	{
		dprintk("POLL ERR ! \r\n");		
		printk("CAM CLOCK :: initialized to 0 \n");
		//tcc_reset_clock();

		return POLLERR;
	}
	//else //cap_status = CAPTURE_NO_INT;
	//	return 0;

	//if (!list_empty(&(h->done_list)))
	//	return POLLIN | POLLRDNORM;

	dprintk("NO_INT ! \r\n"); 

	return 0;
}

extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int camera_core_mmap(struct file *file, struct vm_area_struct *vma)
{
	if (range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
		printk("[vin_v4l2]%s range_is_allowed() failed\n", __func__);
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk("[vin_v4l2]%s remap_pfn_range() failed\n", __func__);
		return -EAGAIN;
	}

	vma->vm_ops = NULL;
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
//	vma->vm_flags |= VM_RESERVED;

	return 0;
}

static long camera_core_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, camera_core_do_ioctl);
}

static int camera_core_release(struct file *file)
{
	struct tcc_video_device *vdev = NULL;

	vdev = video_drvdata(file);
	if (vdev == NULL)
		return -ENODEV;

	vincore_close(vdev);
	tcc_reset_clock();

	clk_disable(vdev->vioc_clk);

	if (vdev->sinfo->init_mode == SINIT_NORMAL)
		sensor_if_deinit(vdev);

	dprintk("camera_core_release \n");
	return 0;
}

static int camera_core_open(struct file *file)
{
	int ret = -1;
	struct tcc_video_device *vdev = video_drvdata(file);

	dprintk("__ in \n");
	
	if (vdev == NULL)
		return -ENODEV;

	if (!vdev->vfd || (vdev->vfd->minor == -1)) {
		ret = -ENODEV;
		goto err;
	}
	
	cif_get_client(vdev);

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
	{
		struct tcc_cif_platform_data *pdata = vdev->dev->platform_data;
		if (pdata->port->det_gpio != CIF_PORT_NULL) {
			vdev->det_status = gpio_get_value(pdata->port->det_gpio);
			if (vdev->det_status) {
				printk("%s: image sensor plug-out\n", vdev->name);
				ret = -ENODEV;
				goto err;
			}
		}
	}
#endif

	clk_prepare_enable(vdev->vioc_clk);
	tcc_set_clock(vdev->pix, 0);

	ret = camera_core_sensor_open(vdev);
	if (ret < 0)
		goto err;

#ifdef CONFIG_TCC_VIN_DEBUG
	//dump_vdev(vdev);
#endif

	dprintk("__out \n");
	
	return	0;
err:
	printk("%s: error(%d)\n", __func__, ret);
	return ret;
}

static struct v4l2_file_operations camera_core_fops = 
{
	.owner          = THIS_MODULE,
	.poll           = camera_core_poll,
	.unlocked_ioctl = camera_core_ioctl,
	.mmap           = camera_core_mmap,
	.open           = camera_core_open,
	.release        = camera_core_release
};

static int __init camera_core_probe(struct platform_device *pdev)
{
	int	ret = 0;
	struct tcc_video_device *vdev = NULL;
	struct tcc_cif_platform_data *pdata;
	struct sensor_info *sinfo;
    struct pinctrl *pinctrl;
	int result, val = -1;
	const char * cam_i2c_name;
	
	dprintk("camera_core_probe \n");

	vdev = kzalloc(sizeof(struct tcc_video_device), GFP_KERNEL);
	if (vdev == NULL)
		return -ENOMEM;

	strlcpy(vdev->name, pdev->name, sizeof(vdev->name));
	dprintk("pdev->name : %s \n", pdev->name);

	/* initialize the video_device struct */
	vdev->vfd = video_device_alloc();
	if (!vdev->vfd) {
		printk(KERN_ERR "%s: could not allocate video device struct\n", vdev->name);
		ret = -ENOMEM;
		goto err0;
	}
	
	if(v4l2_device_register(&pdev->dev, &vdev->v4l2_dev) < 0){
		printk(KERN_ERR "%s : [error] v4l2 register failed\n", pdev->name);
		ret = -ENODEV;
		goto err0;
	}

	vdev->vfd->v4l2_dev = &vdev->v4l2_dev;
	
 	vdev->vfd->release = video_device_release;

 	strlcpy(vdev->vfd->name, vdev->name, sizeof(vdev->vfd->name));
 	/*vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;*/
 	
 	/* need to register for a VID_HARDWARE_* ID in videodev.h */
 	vdev->vfd->fops = &camera_core_fops;
 	vdev->vfd->minor = -1;

	result = of_property_read_u32(pdev->dev.of_node,"telechips,sensor_type",&val);
	if(result) { pr_err("cann't parse telechips,sensor_type\r\n"); goto err0; }
		vdev->sensor_type = val;
		
	result = of_property_read_u32(pdev->dev.of_node,"telechips,id",&val);
	if(result) { pr_err("cann't parse telechips,id\r\n"); goto err0; }
		pdev->id = val;

	/* video device minor 
	 * -1 ==> auto assign, X ==> /dev/videoX 
	 */
	vdev->video_nr = pdev->id;
	if (video_register_device(vdev->vfd, VFL_TYPE_GRABBER, vdev->video_nr) < 0) {
		printk(KERN_ERR "%s: could not register Video for Linux device\n", vdev->name);
		ret = -ENODEV;
		goto err1;
	}

	/* mapping private data */
	video_set_drvdata(vdev->vfd, vdev);	// struct tcc_video_device *vdev = video_drvdata(struct file *file);
	platform_set_drvdata(pdev, vdev);	// struct tcc_video_device *vdev = platform_get_drvdata(struct platform_device *pdev);
	vdev->dev = &pdev->dev;				// pdev->dev.platform_data == struct tcc_cif_platform_data
	pdata = vdev->dev->platform_data;
	
	printk(KERN_INFO "%s: registered device video%d [v4l2]\n", vdev->name, vdev->vfd->minor);

//#ifdef CONFIG_HAS_EARLYSUSPEND
//	early_suspend.suspend = tcc92xx_camera_early_suspend;
//	early_suspend.resume  = tcc92xx_camera_late_resume;
//	early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
//	register_early_suspend(&early_suspend);
//#endif

	/* vioc_clk: vioc block clock
	 * cifmc_clk: cif mclk (CIF_CLKO)
     */	
     np_camera = of_parse_phandle(pdev->dev.of_node, "telechips,cif_camera", 0);
		if(np_camera < 1) {
			pr_err("can't find telechips,cif_camera\r\n");
			return NULL;
		}

	vdev->cifmc_clk = of_clk_get(np_camera, 0);
	vdev->vioc_clk = of_clk_get(np_camera, 1);
#if 0
	vdev->vioc_clk = clk_get(NULL, "lcdc");
	if (pdata->port->clko != CIF_PORT_NULL)
		vdev->cifmc_clk = clk_get(NULL, pdata->port->clko_name);
	else
		vdev->cifmc_clk = NULL;
#endif 


	/* create & mapping sensor_info
	 */
	sinfo = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (sinfo == NULL)
		goto err1;
	vdev->sinfo = sinfo;
	sensor_if_mapping(sinfo, vdev->sensor_type);

	/* initialize cif i2c 
	 */
	of_property_read_string(vdev->dev->of_node, "telechips,sensor_name", &cam_i2c_name);
	dprintk("____ %s \n", cam_i2c_name);
	
	cif_set_i2c_name(pdev->id, cam_i2c_name);
	
#if 0
	if (pdata->port->i2c_core != I2C_NONE) {
		/* for the management of devices that share a one sensor */ 
		if (!(pdata->cif_shared_sensor_ctrl != NULL
				&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_I2C, 1, NULL))) {
			cif_i2c_init(vdev);

			/* set shared i2c_client */
			if (pdata->cif_shared_sensor_ctrl)
				ret = pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_SET_I2C, 1, vdev->cif_i2c_client);
				dprintk("tcc_video_device%d i2c: name(%s), addr(0x%x)\n", 
						vdev->video_nr, vdev->cif_i2c_client->name, vdev->cif_i2c_client->addr);
		} else {
			if (pdata->cif_shared_sensor_ctrl) {
				vdev->cif_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
				ret = pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_GET_I2C, 1, vdev->cif_i2c_client);
				dprintk("tcc_video_device%d i2c: name(%s), addr(0x%x)\n", 
					vdev->video_nr, vdev->cif_i2c_client->name, vdev->cif_i2c_client->addr);
			}
		}

		if (ret) {
			printk("%s: shared_i2c_client failed\n", vdev->name);
			goto err1;
		}
	}
#endif

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
	/* initialize detection code (sensor plug-in/out)
	 */
	if (pdata->port->det_gpio != CIF_PORT_NULL) {
		ret = detect_sensor_init(vdev);
		if (ret) {
			printk("%s: detect code init failed\n", vdev->name);
			goto err1;
		}
	}
#endif

	return 0;
err1:
	video_device_release(vdev->vfd);
err0:
	return ret;
}

static int camera_core_remove(struct platform_device *pdev)
{
	struct tcc_video_device *vdev = platform_get_drvdata(pdev);
	struct tcc_cif_platform_data *pdata = pdev->dev.platform_data;

	dprintk("camera_core_remove \n");

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
	/* de-initialize detection code (sensor plug-in/out)
	 */
	if (pdata->port->det_gpio != CIF_PORT_NULL) {
		detect_sensor_deinit(vdev);
	}
#endif

#if 0
	/* for the management of devices that share a one sensor */ 
	if (!(pdata->cif_shared_sensor_ctrl != NULL
			&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_I2C, 0, NULL))) {
		cif_i2c_exit(vdev);
	} else {
		if (pdata->cif_shared_sensor_ctrl)
			kfree(vdev->cif_i2c_client);
	}
#endif

	video_unregister_device(vdev->vfd);
	video_device_release(vdev->vfd);
#if 0
	clk_put(vdev->vioc_clk);
	if (vdev->cifmc_clk)
		clk_put(vdev->cifmc_clk);
#endif
	if (vdev->h)
		kfree(vdev->h);
	kfree(vdev->sinfo->v4l2_ctrl);
	kfree(vdev->sinfo);
	kfree(vdev);

	return 0;
}

static unsigned int cifport_reg_backup;
static int camera_core_suspend(struct platform_device *pdev, pm_message_t state)
{
	DDICONFIG *pDDIConfig;
	struct device_node * ddi_config_np;

	ddi_config_np = of_parse_phandle(pdev->dev.of_node, "telechips,ddi_config", 0);
	if(ddi_config_np < 1) {
		pr_err("can't find telechips,ddi_config\r\n");
		return NULL;
	}

	pDDIConfig = (DDICONFIG *)of_iomap(ddi_config_np, 0);

	cifport_reg_backup = pDDIConfig->CIFPORT.nREG;
	
	dprintk("\n");
	return 0;
}

static int camera_core_resume(struct platform_device *pdev)
{	
	DDICONFIG *pDDIConfig;
	struct device_node * ddi_config_np;
	
	ddi_config_np = of_parse_phandle(pdev->dev.of_node, "telechips,ddi_config", 0);
	if(ddi_config_np < 1) {
		pr_err("can't find telechips,ddi_config\r\n");
		return NULL;
	}
	
	pDDIConfig = (DDICONFIG *)of_iomap(ddi_config_np, 0);

	BITCSET(pDDIConfig->CIFPORT.nREG, 0x00077777, cifport_reg_backup);	
	dprintk("\n");
	return 0;
}

static struct of_device_id cif_of_match[] = {
	{ .compatible = "telechips,cif_info" },
	{ .compatible = "telechips,cif2_info" },
}; 

#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
static struct platform_driver v4l2_driver_video0 = {
	.driver = {
		.name 	= "tcc-vin-video0",
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cif_of_match[0]),
	},
	.probe 		= camera_core_probe,
	.remove 	= camera_core_remove,
	.suspend 	= camera_core_suspend,
	.resume 	= camera_core_resume
};
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
static struct platform_driver v4l2_driver_video1 = {
	.driver = {
		.name 	= "tcc-vin-video1",
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cif_of_match[1]),

	},
	.probe 		= camera_core_probe,
	.remove 	= camera_core_remove,
	.suspend 	= camera_core_suspend,
	.resume 	= camera_core_resume
};
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
static struct platform_driver v4l2_driver_video2 = {
	.driver = {
		.name 	= "tcc-vin-video2",
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(cif_of_match),
	},
	.probe 		= camera_core_probe,
	.remove 	= camera_core_remove,
	.suspend 	= camera_core_suspend,
	.resume 	= camera_core_resume
};
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
static struct platform_driver v4l2_driver_video3 = {
	.driver = {
		.name 	= "tcc-vin-video3",
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(cif_of_match),
	},
	.probe 		= camera_core_probe,
	.remove 	= camera_core_remove,
	.suspend 	= camera_core_suspend,
	.resume 	= camera_core_resume
};
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
static struct platform_driver v4l2_driver_video4 = {
	.driver = {
		.name 	= "tcc-vin-video4",
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(cif_of_match),
	},
	.probe 		= camera_core_probe,
	.remove 	= camera_core_remove,
	.suspend 	= camera_core_suspend,
	.resume 	= camera_core_resume
};
#endif

int __init camera_core_init(void)
{
	int ret = 0, err = 0;
	dprintk("camera_core_init \n");


	if (unlikely(!tcc_is_camera_enable)) {
		printk("Camera Feature is dead in bootloader! please, check!\n");
		return -ENODEV;
	}
	return 0;
}

void __exit camera_core_cleanup(void)
{
	dprintk("camera_core_cleanup \n");

	if (unlikely(!tcc_is_camera_enable)) {
		printk("Camera Feature is dead in bootloader! please, check!\n");
	}
}

MODULE_AUTHOR("Telechips Inc.");
MODULE_DESCRIPTION("VIOC VIN driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
module_platform_driver(v4l2_driver_video0);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
module_platform_driver(v4l2_driver_video1);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
module_platform_driver(v4l2_driver_video2);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
module_platform_driver(v4l2_driver_video3);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
module_platform_driver(v4l2_driver_video4);
#endif
module_init(camera_core_init);
module_exit(camera_core_cleanup);
MODULE_DEVICE_TABLE(of, cif_of_match);
