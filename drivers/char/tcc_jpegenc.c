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
#include <asm/processor.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/syscalls.h>
#include <linux/poll.h>

#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/tcc_scaler_ioctrl.h>
#else
#include <video/tcc/tcc_scaler_ioctrl.h>
#endif

#include "tcc_jpeg.h"
#include "tcc_jpegenc_app.h"

#define TCC_JPEGENC_MAJOR_ID 211
#define TCC_JPEGENC_MINOR_ID 1
#define TCC_JPEGENC_DEVICE_NAME "jpegenc"

#define dprintk(msg...)	if (0) { printk( "TCC_JPEGENC: " msg); }

typedef struct _int_data_t {
	wait_queue_head_t wq;
	spinlock_t lock;
	unsigned int count;
	
	struct mutex io_mutex;

	unsigned char irq_reged;
	unsigned char dev_opened;
} int_data_t;
static int_data_t jpgenc_data;

JPEG_ENC_INFO  gJPEG_ENC_Info;
JPEG_BUFFER_CTRL gJPEGEnc_Buffer_Info;

static TCCXXX_JPEG_ENC_DATA enc_data;
static unsigned int gJpegEncodeDone = 0;

void *pJpegenc_remapped_rawdata, *pJpegenc_remapped_streamdata, *pJpegenc_remapped_header;
#ifndef CONFIG_SUPPORT_TCC_JPU
static struct clk *vcore_clk;
static struct clk *jpege_clk;
#endif
static unsigned int jpu_base_addr;

static void tccxx_jpgenc_set_clock(unsigned char init)
{
#ifndef CONFIG_SUPPORT_TCC_JPU
	if(init)
	{
		clk_prepare_enable(vcore_clk);
		clk_prepare_enable(jpege_clk);
		clk_disable_unprepare(vcore_clk); 
	}
	else
	{
		clk_prepare_enable(vcore_clk);
	}
#endif
}

void tccxx_jpgenc_reset_clock(unsigned char deinit)
{
#ifndef CONFIG_SUPPORT_TCC_JPU	
	if(deinit)
	{
		clk_disable_unprepare(jpege_clk);
	}
	else
	{
		clk_disable_unprepare(vcore_clk);			
	}
#endif
}
static int tccxx_enc_meminit(void *data)
{
	TCCXXX_JPEG_ENC_DATA *Enc_data;
	unsigned int raw_phy_addr, raw_size;
	unsigned int stream_phy_addr, stream_size;
	unsigned int header_phy_addr, header_size;
	#if (1)
	unsigned int jpeg_header_size = 0x30000;
	#else
	pmap_t pmap_jpeg_header;
	pmap_get_info("jpeg_header", &pmap_jpeg_header);
	#endif

	if(pJpegenc_remapped_header != NULL) {
		iounmap(pJpegenc_remapped_header);
		pJpegenc_remapped_header = NULL;
	}
	
	if(pJpegenc_remapped_rawdata != NULL) {
		iounmap(pJpegenc_remapped_rawdata);
		pJpegenc_remapped_rawdata = NULL;
	}
	
	if(pJpegenc_remapped_streamdata != NULL) {
		iounmap(pJpegenc_remapped_streamdata);
		pJpegenc_remapped_streamdata = NULL;
	}
	
	Enc_data = (TCCXXX_JPEG_ENC_DATA *)data;

	//Target is Raw-data in case of Decode.
	raw_phy_addr = Enc_data->source_addr;
	if(Enc_data->src_format == ENC_INPUT_422)
		raw_size = Enc_data->width * Enc_data->height * 2;
	else
		raw_size = Enc_data->width * Enc_data->height * 3 / 2;

	//Source is Stream-data in case of Decode.
	stream_phy_addr = Enc_data->target_addr;
	stream_size 	= Enc_data->target_size - jpeg_header_size;
	header_phy_addr = stream_phy_addr + stream_size;
	header_size		= jpeg_header_size;

	if((raw_phy_addr && raw_size) > 0) {
		pJpegenc_remapped_rawdata = (void *)ioremap_nocache(raw_phy_addr, PAGE_ALIGN(raw_size/*-PAGE_SIZE*/));
		if(pJpegenc_remapped_rawdata == NULL) {
			printk(KERN_ALERT "[raw] can not remap for jpeg\n");
			return -EFAULT;
		}	
	} else {
		pJpegenc_remapped_rawdata = NULL;
	}

	if((stream_phy_addr && stream_size) > 0) {
		pJpegenc_remapped_streamdata = (void *)ioremap_nocache(stream_phy_addr, PAGE_ALIGN(stream_size/*-PAGE_SIZE*/));
		if(pJpegenc_remapped_streamdata == NULL) {
			printk(KERN_ALERT "[stream] can not remap for jpeg\n");
			return -EFAULT;
		}
	} else {
		pJpegenc_remapped_streamdata = NULL;
	}

	if((header_phy_addr && header_size) > 0) {
		pJpegenc_remapped_header = (void *)ioremap_nocache(header_phy_addr, PAGE_ALIGN(header_size/*-PAGE_SIZE*/));
		if(pJpegenc_remapped_header == NULL) {
			printk(KERN_ALERT "[header] can not remap for jpeg\n");
			return -EFAULT;
		}
	} else {
		pJpegenc_remapped_header = NULL;
	}

	dprintk("raw   :  phy[0x%x ~ 0x%x], virt[0x%x ~ 0x%x], size = 0x%x \n",raw_phy_addr, 					\
								raw_phy_addr+raw_size, (unsigned int)pJpegenc_remapped_rawdata, 			\
								(unsigned int)pJpegenc_remapped_rawdata+raw_size, raw_size);
	dprintk("stream:  phy[0x%x ~ 0x%x], virt[0x%x ~ 0x%x], size = 0x%x \n",stream_phy_addr, 				\
								stream_phy_addr+stream_size, (unsigned int)pJpegenc_remapped_streamdata, 	\
								(unsigned int)pJpegenc_remapped_streamdata+stream_size, stream_size);
	dprintk("header:  phy[0x%x ~ 0x%x], virt[0x%x ~ 0x%x], size = 0x%x \n",header_phy_addr, 				\
								header_phy_addr + header_size, (unsigned int)pJpegenc_remapped_header, 		\
								(unsigned int)pJpegenc_remapped_header + header_size, header_size);

	gJPEGEnc_Buffer_Info.pBaseRawDataAddr 			= (void*)raw_phy_addr;
	gJPEGEnc_Buffer_Info.pBaseRawDataSize 			= raw_size;
	gJPEGEnc_Buffer_Info.pBaseBitstreamDataAddr 	= (void*)stream_phy_addr;
	gJPEGEnc_Buffer_Info.pBaseBitstreamDataSize 	= stream_size;
	gJPEGEnc_Buffer_Info.pBaseHeaderDataAddr 		= (void*)(stream_phy_addr + stream_size - jpeg_header_size);
	gJPEGEnc_Buffer_Info.pBaseHeaderDataSize 		= jpeg_header_size;

	return 0;
}

static int tccxx_exifheader_meminit(void *data)
{
	unsigned int header_phy_addr, header_size;
	unsigned int stream_phy_addr, stream_size;	
	TCCXXX_JPEG_ENC_EXIF_DATA *thumb_data;

	thumb_data = (TCCXXX_JPEG_ENC_EXIF_DATA*)data;
	if (thumb_data == NULL) {
		pr_err("%s: jpeg header data is NULL\n", __func__);
		return -EFAULT;
	}
	
	if(pJpegenc_remapped_streamdata != NULL)
	{
		iounmap(pJpegenc_remapped_streamdata);
		pJpegenc_remapped_streamdata = NULL;
	}
	
	if(pJpegenc_remapped_header != NULL)
	{
		iounmap(pJpegenc_remapped_header);
		pJpegenc_remapped_header = NULL;
	}
	
	header_phy_addr = thumb_data->header_addr;
	header_size = thumb_data->header_size;

	if(header_phy_addr && header_size > 0)
	{
		pJpegenc_remapped_header = (void *)ioremap_nocache(header_phy_addr, PAGE_ALIGN(header_size));
		if (pJpegenc_remapped_header == NULL) {
			printk(KERN_ALERT "[header] can not remap for jpeg\n");
			return -EFAULT;
		}
	}
	else
	{
		pJpegenc_remapped_header = NULL;
	}

	stream_phy_addr = thumb_data->thumbjpg_addr;
	stream_size = thumb_data->thumbjpg_size;

	if(stream_phy_addr && stream_size > 0)
	{
		pJpegenc_remapped_streamdata = (void *)ioremap_nocache(stream_phy_addr, PAGE_ALIGN(stream_size));
		if (pJpegenc_remapped_streamdata == NULL) {
			printk(KERN_ALERT "[stream] can not remap for jpeg\n");
			return -EFAULT;
		}
	}
	else
	{
		pJpegenc_remapped_streamdata = NULL;
	}

	dprintk("ExifHeader	:: phy[0x%x ~ 0x%x], virt[0x%x ~ 0x%x], size = 0x%x \n",header_phy_addr, header_phy_addr+header_size, (unsigned int)pJpegenc_remapped_header, (unsigned int)pJpegenc_remapped_header+header_size, header_size);
	dprintk("ThumbJpeg	:: phy[0x%x ~ 0x%x], virt[0x%x ~ 0x%x], size = 0x%x \n",stream_phy_addr, stream_phy_addr+stream_size, (unsigned int)pJpegenc_remapped_streamdata, (unsigned int)pJpegenc_remapped_streamdata+stream_size, stream_size);

	return 0;
}

static int tccxx_enc_start(void)
{
	int ret = 0;

	if(enc_data.normal_op && jpgenc_data.dev_opened > 1) {
		dprintk("JpegDevice(Enc) is already using \n");
		return -EFAULT;
	}

	ret = tccxx_enc_meminit(&enc_data);
	dprintk("src: 0x%x, size: %d x %d !!\n", enc_data.source_addr, enc_data.width, enc_data.height);

	if(ret >= 0) {
		gJpegEncodeDone = 0;
	}

	return ret;
}


static int tccxx_enc_finish(void)
{
	void *HeaderBuffer;
	int ret = 0;

	ret = JPEG_ENCODE_Get_Result(enc_data.width, enc_data.height, &(enc_data.bitstream_size), &(enc_data.header_size));

	dprintk("enc info :: %d = %d + %d \n", enc_data.header_size + enc_data.bitstream_size, enc_data.header_size, enc_data.bitstream_size);
	if(ret >= 0) {
		HeaderBuffer = pJpegenc_remapped_header;
		memcpy(pJpegenc_remapped_streamdata+enc_data.bitstream_size, HeaderBuffer, enc_data.header_size);

		enc_data.thumb_size = 0; //thumb is in header!!
		enc_data.bitstream_offset = 0;
		enc_data.header_offset = enc_data.bitstream_size;
	}

	return ret;
}


static int tccxx_make_exifheader(TCCXXX_JPEG_ENC_EXIF_DATA* arg)
{
	int ret = 0;
	TCCXXX_JPEG_ENC_EXIF_DATA thumb_data;

	if(copy_from_user(&thumb_data, arg, sizeof(TCCXXX_JPEG_ENC_EXIF_DATA)))
		return -EFAULT;

	ret = tccxx_exifheader_meminit(&thumb_data);
	
 	dprintk("ExifHeader stream: 0x%x [%d x %d], thumb: 0x%x - 0x%x[%d x %d] \n",
					thumb_data.bitstream_size, thumb_data.bitstream_width, thumb_data.bitstream_height,
					thumb_data.thumbjpg_addr, thumb_data.thumbjpg_size, thumb_data.thumb_width, thumb_data.thumb_height);

	if(ret >= 0)
	{	
		if(thumb_data.thumbjpg_size > 64*1024) //thumb is not jpeg but yuv. so we need to encode.
		{		
			printk("Err :: ExifHeader Max size over \n");
			return -EFAULT;
		}

		thumb_data.header_size = JPEG_Make_ExifHeader(&thumb_data, (unsigned int)pJpegenc_remapped_header);
	}
	dprintk("Header :: addr = 0x%x, size = 0x%x \n", thumb_data.header_addr, thumb_data.header_size);

	if(copy_to_user(arg, &thumb_data, sizeof(TCCXXX_JPEG_ENC_EXIF_DATA)))
		return -EFAULT;
	
	return ret;
}


/****************************************************************************
	IRQ_JPEG_ENC_Handler()

	Description : 
	Parameter : NONE
	Return : NONE
****************************************************************************/

extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int tcc_jpegenc_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		printk(KERN_ERR  "jpegenc: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		return -EAGAIN;
	}

	vma->vm_ops		= NULL;
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static long tcc_jpegenc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int_data_t *jpg_data = (int_data_t *)file->private_data;
	int wait_ms = 10000;
	int ret = 0;


	switch(cmd) {
		case TCC_JPEGE_IOCTL_ENC:
			{
				mutex_lock(&jpg_data->io_mutex);
								
				if(copy_from_user(&enc_data, (TCCXXX_JPEG_ENC_DATA*)arg, sizeof(TCCXXX_JPEG_ENC_DATA))) {
					mutex_unlock(&jpg_data->io_mutex);
					return -EFAULT;
				}

				jpg_data->count = 0;
				tccxx_jpgenc_set_clock(0);

				if((ret = tccxx_enc_start()) >= 0) {
					if(enc_data.width <= 640 && enc_data.height <= 640) //exception proc::  jpeg error in case of specific size!!
						wait_ms = 100;
					
					ret = wait_event_interruptible_timeout(jpg_data->wq, jpg_data->count > 0, msecs_to_jiffies(wait_ms));
					if(ret <= 0) {
						printk("[%d]: jpegenc[%d x %d] timed_out!! \n", ret, enc_data.width, enc_data.height);
						ret = -EFAULT;
					} else {
						if(!gJpegEncodeDone) {
							ret = -EINVAL;
						} else {
							if((ret = tccxx_enc_finish()) >= 0) {
								if (copy_to_user((TCCXXX_JPEG_ENC_DATA*)arg, &enc_data, sizeof(TCCXXX_JPEG_ENC_DATA)))
									ret = -EFAULT;
							}
						}
					}
					
				}

				tccxx_jpgenc_reset_clock(0);
				mutex_unlock(&jpg_data->io_mutex);
			}
			break;

		case TCC_JPEGE_IOCTL_MAKE_EXIF:
			ret = tccxx_make_exifheader((TCCXXX_JPEG_ENC_EXIF_DATA*)arg);
			break;

		case TCC_JPEGE_IOCTL_USABLE:
			{
				unsigned int usable = 0;
				
				if(jpgenc_data.dev_opened == 1)
					usable = 1;
					
				if (copy_to_user((unsigned int*)arg, &usable, sizeof(unsigned int)))
					ret = -EFAULT;
			}
			break;

		case TCC_JPU_IOCTL_GET_BASE_ADDR:
			{
				if (copy_to_user((unsigned int *)arg, &jpu_base_addr, sizeof(unsigned int))) {
					ret = -EFAULT;
				}
			}
			break;

		default:
			printk(KERN_ALERT "Not Supported JPEG_ENC_IOCTL(%d) \n", cmd);
			break;
	}

	return ret;
}

static int tcc_jpegenc_release(struct inode *inode, struct file *filp)
{
	jpgenc_data.dev_opened--;

	dprintk("tcc_jpegenc_release(%d) \n", jpgenc_data.dev_opened);

	if(!jpgenc_data.dev_opened)
	{
		if(jpgenc_data.irq_reged)
		{
			jpgenc_data.irq_reged = 0;
		}
			
		if(pJpegenc_remapped_header != NULL)
			iounmap(pJpegenc_remapped_header);

		if(pJpegenc_remapped_rawdata != NULL)
			iounmap(pJpegenc_remapped_rawdata);
		
		if(pJpegenc_remapped_streamdata != NULL)
			iounmap(pJpegenc_remapped_streamdata);

		pJpegenc_remapped_header = NULL;
		pJpegenc_remapped_rawdata = NULL;
		pJpegenc_remapped_streamdata = NULL;
	}

	tccxx_jpgenc_reset_clock(1);
	dprintk("tcc_jpegenc_release(%d) \n", jpgenc_data.dev_opened);
	
	return 0;
}

static int tcc_jpegenc_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	dprintk("tcc_jpegenc_open!!\n");
	
	if(!jpgenc_data.irq_reged)
	{
		jpgenc_data.irq_reged = 1;
	}
	tccxx_jpgenc_set_clock(1);
	
	filp->private_data = &jpgenc_data;
	jpgenc_data.dev_opened++;
	dprintk("tcc_jpegenc_open(%d) \n", jpgenc_data.dev_opened);

	return ret;	
	
}

static int tcc_jpegenc_probe(struct platform_device *pdev)
{
	
	printk("_______tcc_jpeg_enc_probe start!!! \n");
	
	if(pdev->dev.of_node)
	{
#ifndef CONFIG_SUPPORT_TCC_JPU
		vcore_clk = of_clk_get(pdev->dev.of_node, 0);
		jpege_clk = of_clk_get(pdev->dev.of_node, 1);
#endif
		of_property_read_u32(pdev->dev.of_node, "jpu_base_addr", &jpu_base_addr);
	}
	else
	{
		printk("could not find jpeg enc node!! \n");
		return -ENODEV;	
	}
	return 0;	
}

static int tcc_jpegenc_remove(struct platform_device *pdev)
{
	return 0;	
}

static struct of_device_id jpegenc_of_match[] = {
	{ .compatible = "telechips-jpegenc" },
	{}
};
MODULE_DEVICE_TABLE(of, jpegenc_of_match);

static struct platform_driver tcc_jpegenc_driver = {
	.driver = {
		.name 	= TCC_JPEGENC_DEVICE_NAME,
		.owner 	= THIS_MODULE,
		.of_match_table = of_match_ptr(jpegenc_of_match),
	},
	.probe		= tcc_jpegenc_probe,
	.remove		= tcc_jpegenc_remove
};

static struct file_operations tcc_jpegenc_fops = 
{
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= tcc_jpegenc_ioctl,
	.mmap			= tcc_jpegenc_mmap,
	.open			= tcc_jpegenc_open,
	.release		= tcc_jpegenc_release,
};

void __exit tcc_jpegenc_cleanup(void)
{
	platform_driver_unregister(&tcc_jpegenc_driver);
	unregister_chrdev(TCC_JPEGENC_MAJOR_ID, TCC_JPEGENC_DEVICE_NAME);
#ifndef CONFIG_SUPPORT_TCC_JPU
	clk_put(vcore_clk);
	clk_put(jpege_clk);
#endif
	dprintk(" ===========> tcc_jpegenc_cleanup \n");

	return;
}


static char banner[] __initdata = KERN_INFO "TCCXX JPEG Encoder driver initializing\n";
static struct class *jpegenc_class;

int __init tcc_jpegenc_init(void)
{
	printk(banner);

	memset(&jpgenc_data, 0, sizeof(int_data_t));
	spin_lock_init(&(jpgenc_data.lock));
	init_waitqueue_head(&(jpgenc_data.wq));
	
	mutex_init(&jpgenc_data.io_mutex);
	
	register_chrdev(TCC_JPEGENC_MAJOR_ID, TCC_JPEGENC_DEVICE_NAME, &tcc_jpegenc_fops);
	platform_driver_register(&tcc_jpegenc_driver);
	
	jpegenc_class = class_create(THIS_MODULE, TCC_JPEGENC_DEVICE_NAME);
	device_create(jpegenc_class,NULL,MKDEV(TCC_JPEGENC_MAJOR_ID,TCC_JPEGENC_MINOR_ID),NULL,TCC_JPEGENC_DEVICE_NAME);

	pJpegenc_remapped_header = NULL;
	pJpegenc_remapped_rawdata = NULL;
	pJpegenc_remapped_streamdata = NULL;

	return 0;
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCCXX JPEG Encoder driver");
MODULE_LICENSE("GPL");
module_init(tcc_jpegenc_init);
module_exit(tcc_jpegenc_cleanup);
