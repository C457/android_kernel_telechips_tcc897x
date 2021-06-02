/*
 *	tcc_dxb_internal.c
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */

#include "tcc_dxb_internal.h"

#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#ifdef INTERNAL_INTPUT_THREAD
#include <linux/kfifo.h>
#endif

#include <linux/net.h>
#include <linux/socket.h>
#include <linux/in.h>

#include "../tcc_hwdemux/tca_hwdemux_tsif.h"

#define LOG_TAG "[TCC_DXB_INTERNAL]"
static int dev_debug = 1;

/* For Debugging */
module_param(dev_debug, int, 0644);
MODULE_PARM_DESC(dev_debug, "Turn on/off device debugging (default:off).");

#define dprintk(msg...) 								\
{														\
		if (dev_debug)									\
				printk(KERN_INFO LOG_TAG "(D)" msg);	\
}

#define eprintk(msg...) 								\
{														\
		printk(KERN_INFO LOG_TAG " (E) " msg);			\
}

/* Informations */
#define DXB_INTERNAL_NAME "tcc_dxb_internal"
#define _RTP_HEADER_LENGTH		(12)
#define _ADDRESS_ANY			(0x00000000)
#define _TS_PACKET				(188)
#define _PACKET					(1316)
#define _NUM_PATCKET			(128)
#define _DMA_BUF_LEN			(_PACKET*_NUM_PATCKET)	//(it * _NUM_OF_DMA_BUF) must less than (1024 * 1024 * 4)
#define _NUM_OF_DMA_BUF			(24)
#define _SOCKET_RECV_SIZE		(4096*32)				//it must less than (4096 * 32)
#define _SOCKET_RECV_BUF_SIZE	(256*640*8)

static struct mutex dev_mutex;
static int gNumOfOpen = 0;
static struct class *tcc_dxb_internal_class;
static int giMajorDevID = 0;
static int hwdmx_num = 0;

#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
struct tcc_dxb_internal_drvdata_t **drvdata = NULL;
#endif

//internal input thread
#ifdef INTERNAL_INTPUT_THREAD
static struct task_struct *ii_thread;
static struct kfifo kff;
static struct mutex kff_mutex;
typedef struct kfifo_ele_t {
	unsigned dmx_id;
	unsigned char *dma_buf_vir_addr;
	unsigned int dma_buf_phy_addr;
	int total_size;
} KFIFO_ele_t;
#else
static struct mutex hwdmx_mutex;
#endif

/****************************************************************************************/
#ifdef TCC_DXB_INTERNAL_FILE_OUT
#define _PATH "/data/tdi_out.ts"
struct file *mfile;
static struct file* file_open(const char* path, int flags, int rights) {
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}

static void file_close(struct file* file) {
	filp_close(file, NULL);
}

static int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);
	set_fs(oldfs);
	return ret;
}
#endif
/****************************************************************************************/

static int tcc_dxb_internal_sock_create(int family, int type, int proto, struct socket **res) {
	dprintk("%s\n", __FUNCTION__);
	return sock_create_kern(family, type, proto, res);
}

static int tcc_dxb_internal_setsockopt(struct socket *sock, int level, int optname, char *optval, unsigned int optlen) {
	dprintk("%s\n", __FUNCTION__);
	return kernel_setsockopt(sock, level, optname, optval, optlen);
}

static int tcc_dxb_internal_getsockopt(struct socket *sock, int level, int optname, char *optval, int *optlen) {
	dprintk("%s\n", __FUNCTION__);
	return kernel_getsockopt(sock, level, optname, optval, optlen);
}

static int tcc_dxb_internal_bind(struct socket *sock, struct sockaddr *addr, int addrlen) {
	dprintk("%s\n", __FUNCTION__);
	return kernel_bind(sock, addr, addrlen);
}

static int tcc_dxb_internal_sock_recvmsg(struct socket *sock, struct msghdr *msg, size_t size, int flags) {
	//dprintk("%s\n", __FUNCTION__);
	return sock_recvmsg(sock, msg, size, flags);
}

static int tcc_dxb_internal_receive(struct socket* sock, struct sockaddr_in* addr, unsigned char* buf, int len) {
	struct msghdr msg;
	struct iovec iov;
	int size = 0;

	if (sock->sk==NULL) return 0;

	iov.iov_base = buf;
	iov.iov_len = len;

	msg.msg_flags = MSG_DONTWAIT;
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
   
	size = tcc_dxb_internal_sock_recvmsg(sock,&msg,len,msg.msg_flags);

	return size;
}

static int tcc_dxb_internal_rtp_parse (struct tcc_dxb_internal_t *tdi, unsigned char *pucPacket, unsigned int uiPacketSize, unsigned char **pucPayload, unsigned int *puiPayloadSize)
{
	unsigned int rdpos = 0;
	if (pucPacket[0] == 0x47)// && !(uiPacketSize % 188))
	{		
		*pucPayload = pucPacket;
		*puiPayloadSize = uiPacketSize;
		return 0;
	}
	tdi->gRTPStatus.Len = uiPacketSize;
	tdi->gRTPStatus.Version = (*(pucPacket + rdpos) & 0xc0) >> 6;
	tdi->gRTPStatus.Padding = (*(pucPacket + rdpos) & 0x20) >> 5;
	tdi->gRTPStatus.Extension = (*(pucPacket + rdpos) & 0x10) >> 4;
	tdi->gRTPStatus.CSRC_Cnt = (*(pucPacket + rdpos) & 0x0f);
	rdpos += 1;

	tdi->gRTPStatus.Marker = (*(pucPacket + rdpos) & 0x80) >> 7;
	tdi->gRTPStatus.PayloadType = (*(pucPacket + rdpos) & 0x7f);
	rdpos += 1;

	memcpy (&tdi->gRTPStatus.SequenceNo, (pucPacket + rdpos), 2);
	tdi->gRTPStatus.SequenceNo = ntohs(tdi->gRTPStatus.SequenceNo);
	rdpos += 2;

#if 1
	if(tdi->gRTPStatus.SequenceNo && tdi->gRTPStatus.prev_SequenceNo && tdi->gRTPStatus.SequenceNo != (tdi->gRTPStatus.prev_SequenceNo+1))
	{
		dprintk("%s prev_SequenceNo = %d SequenceNo = %d \n", __FUNCTION__, tdi->gRTPStatus.prev_SequenceNo, tdi->gRTPStatus.SequenceNo);
	}
	tdi->gRTPStatus.prev_SequenceNo = tdi->gRTPStatus.SequenceNo;
#endif

	memcpy (&tdi->gRTPStatus.TimeStamp, (pucPacket + rdpos), 4);
	tdi->gRTPStatus.TimeStamp = ntohl(tdi->gRTPStatus.TimeStamp);
	rdpos += 4;

	memcpy (&tdi->gRTPStatus.SSRC, (pucPacket + rdpos), 4);
	tdi->gRTPStatus.SSRC = ntohl(tdi->gRTPStatus.SSRC);
	rdpos += 4;

	if (tdi->gRTPStatus.CSRC_Cnt)
	{
		int 	  i;

		for (i = 0; i < tdi->gRTPStatus.CSRC_Cnt; i++)
		{
			if (i < 15) {
				memcpy (&tdi->gRTPStatus.CSRC[i], (pucPacket + rdpos), 4);
				tdi->gRTPStatus.TimeStamp = ntohl(tdi->gRTPStatus.TimeStamp);
			}
			rdpos += 4;
		}
	}

	if (tdi->gRTPStatus.Extension)
	{
		unsigned short type = 0;
		unsigned short length = 0;

		memcpy (&type, (pucPacket + rdpos), 2);
		type = ntohs(type);
		rdpos += 2;

		memcpy (&length, (pucPacket + rdpos), 2);
		length = ntohs(length);
		rdpos += 2;

		if ((rdpos+length) >= uiPacketSize) return -1;
		if (length > 512) memcpy (tdi->gRTPStatus.ExtensionHeader, (pucPacket + rdpos), 512);
		else memcpy (tdi->gRTPStatus.ExtensionHeader, (pucPacket + rdpos), length);
		rdpos += length;
	}
	
	tdi->gRTPStatus.Payload = pucPacket + rdpos;
	tdi->gRTPStatus.PayloadLen = tdi->gRTPStatus.Len - rdpos;

	*pucPayload = tdi->gRTPStatus.Payload;
	*puiPayloadSize = tdi->gRTPStatus.PayloadLen;
	return 0;
}


/* For Threading */
static int tcc_dxb_internal_thread(void *arg) {
	struct tcc_dxb_internal_t *tdi = (struct tcc_dxb_internal_t *)arg;
	unsigned char *dma_buf_temp = NULL;
	unsigned char *recieve_buf_temp = NULL;
	int size = 0;
	unsigned int total_size = 0;
	unsigned int buf_count = 0;
#ifndef	INTERNAL_INTPUT_THREAD
	int status = -1;
#else
	struct kfifo_ele_t kff_ele;
#endif

	dprintk("%s START Vir_addr:0x%p, Phy_addr:0x%X\n", __FUNCTION__, tdi->dma_buf_vir_addr, tdi->dma_buf_phy_addr);

	dma_buf_temp = tdi->dma_buf_vir_addr;
	recieve_buf_temp = tdi->sock_recieve_buf;
	while(!kthread_should_stop()) {
		memset(tdi->sock_recieve_buf, 0x0, _SOCKET_RECV_SIZE);
		size = tcc_dxb_internal_receive(tdi->sock, &tdi->addr, tdi->sock_recieve_buf, _SOCKET_RECV_SIZE);
		if(signal_pending(current)) 	break;
		if(size == -EAGAIN) {
			msleep(1);
			continue;
		}
		else if(size <= 0) {
			eprintk("sock_recvmsg failed: %d\n", size);
			msleep(10);
			continue;
		}
		else {
			if(tcc_dxb_internal_rtp_parse(tdi, tdi->sock_recieve_buf, size, &recieve_buf_temp, &size)){
				eprintk("receive packet parse error!!\n");
				msleep(10);
				continue;
			}
			if(recieve_buf_temp[0] != 0x47) {
				eprintk("ts packet sync byte error!!\n");
				msleep(10);
				continue;
			}
			memcpy(dma_buf_temp, recieve_buf_temp, size);
			dma_buf_temp += size;
			total_size += size;
			if(total_size >= (_DMA_BUF_LEN-_PACKET)) {
				if(total_size%_TS_PACKET != 0) {
					eprintk("receive packet size error!!\n");
					goto TCC_DXB_INTERNAL_THREAD_FINALLY;
				}
#ifndef INTERNAL_INTPUT_THREAD
				mutex_lock(&hwdmx_mutex);
				status = tca_tsif_input_internal(tdi->dmx_id, tdi->dma_buf_phy_addr+(buf_count*_DMA_BUF_LEN), total_size);
				mutex_unlock(&hwdmx_mutex);
				if(status) {
					eprintk("hwdemux internal input error[0x%X]!!\n", status);
				}
				{
//					unsigned char *temp = tdi->dma_buf_vir_addr+(buf_count*_DMA_BUF_LEN);
//					eprintk("###jktest0 total_size:%d [0x%.2X%.2X%.2X...0x%.2X]\n", total_size, temp[0], temp[1], temp[2], temp[188]);
				}
#else
				kff_ele.dmx_id				= tdi->dmx_id;
				kff_ele.dma_buf_phy_addr	= tdi->dma_buf_phy_addr+(buf_count*_DMA_BUF_LEN);
				kff_ele.dma_buf_vir_addr	= tdi->dma_buf_vir_addr+(buf_count*_DMA_BUF_LEN);
				kff_ele.total_size			= total_size;
				if(kfifo_is_full(&kff)) {
					eprintk("kfifo is full!!!\n");
					goto TCC_DXB_INTERNAL_THREAD_FINALLY;
				}
				mutex_lock(&kff_mutex);
				kfifo_in(&kff, &kff_ele, sizeof(struct kfifo_ele_t));
				mutex_unlock(&kff_mutex);
#endif
TCC_DXB_INTERNAL_THREAD_FINALLY:
				buf_count++;
				buf_count %= _NUM_OF_DMA_BUF;
				dma_buf_temp = tdi->dma_buf_vir_addr+(buf_count*_DMA_BUF_LEN);
				total_size = 0;
				memset(dma_buf_temp, 0x0, _DMA_BUF_LEN);
			}
		}
	}
	dprintk("%s END\n", __FUNCTION__);
	return 0;
}

#ifdef INTERNAL_INTPUT_THREAD
static int tcc_dxb_internal_input_thread(void *arg) {
	struct kfifo_ele_t kff_ele = {0, };
	int status = -1;
#ifdef TCC_DXB_INTERNAL_FILE_OUT
	unsigned long long ppos = 0;

	mfile = file_open(_PATH, O_RDWR|O_CREAT , S_IRUSR|S_IWUSR);
#endif
	dprintk("%s START\n", __FUNCTION__);
	while(!kthread_should_stop()) {
		if(signal_pending(current)) 	break;
		if(kfifo_is_empty(&kff)) {
			msleep(1);
			continue;
		}
		if(kfifo_out(&kff, &kff_ele, sizeof(struct kfifo_ele_t))) {
#ifdef TCC_DXB_INTERNAL_FILE_OUT
			file_write(mfile, ppos, kff_ele.dma_buf_vir_addr, kff_ele.total_size);
			ppos += kff_ele.total_size;
#endif
			status = tca_tsif_input_internal(kff_ele.dmx_id, kff_ele.dma_buf_phy_addr, kff_ele.total_size);
			if(status) {
				eprintk("hwdemux internal input error[0x%X]!!\n", status);
			}
			{
//				eprintk("###jktest1 total_size:%d [0x%.2X%.2X%.2X...0x%.2X]\n", kff_ele.total_size, kff_ele.dma_buf_vir_addr[0], kff_ele.dma_buf_vir_addr[1], kff_ele.dma_buf_vir_addr[2], kff_ele.dma_buf_vir_addr[188]);
			}
		}
	}
#ifdef TCC_DXB_INTERNAL_FILE_OUT
	file_close(mfile);
#endif
	dprintk("%s END\n", __FUNCTION__);
	return 0;
}
#endif
static void tcc_dxb_internal_start_feed(struct tcc_dxb_internal_t *tdi) {
	dprintk("%s\n", __FUNCTION__);
	if(tdi == NULL) {
		eprintk("%s instance is null !!!\n", __FUNCTION__);
		return;
	}
	memset(&tdi->gRTPStatus, 0x0, sizeof(struct tcc_rtp_t));
	if(tdi->thread == NULL) {
		tdi->thread = (struct task_struct *)kthread_run(tcc_dxb_internal_thread, (void *)tdi, "tcc_dxb_internal_thread%d", gNumOfOpen);
	}
}

static void tcc_dxb_internal_stop_feed(struct tcc_dxb_internal_t *tdi) {
	dprintk("%s\n", __FUNCTION__);
	if(tdi == NULL) {
		return;
	}
	if(tdi->thread) {
		kthread_stop(tdi->thread);
		tdi->thread = NULL;
	}
}

static void tcc_dxb_internal_clean(struct tcc_dxb_internal_t *tdi) {
	tcc_dxb_internal_stop_feed(tdi);
	if(tdi->sock != NULL) {
		tcc_dxb_internal_setsockopt(tdi->sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char *)&tdi->req, sizeof(tdi->req));
		sock_release(tdi->sock);
		tdi->sock = NULL;
	}
}

/* File Operations*/
static int tcc_dxb_internal_start(struct tcc_dxb_internal_t *tdi, struct sockaddr_in addr) {
	int ret = -1;
	int bReuse = 0x1;
	unsigned int sock_RecvBufSize = _SOCKET_RECV_BUF_SIZE;
	unsigned int optlen = sizeof(sock_RecvBufSize);

	dprintk("%s\n", __FUNCTION__);
	if(tdi == NULL) {
		eprintk("%s instance is null !!!\n", __FUNCTION__);
		return -1;
	}
	ret = tcc_dxb_internal_sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &tdi->sock);
	if(ret < 0) {
		eprintk("sock_create() failed: %d\n", ret);
		return ret;
	}
	ret = tcc_dxb_internal_setsockopt(tdi->sock, SOL_SOCKET, SO_REUSEADDR, (char *)&bReuse, sizeof(bReuse));
	if(ret < 0) {
		eprintk("kernel_setsockopt() for SO_REUSEADDR failed: %d\n", ret);
		return ret;
	}
	memset(&tdi->addr, 0, sizeof(struct sockaddr_in));

	tdi->addr.sin_family			= AF_INET;
	tdi->addr.sin_addr.s_addr		= addr.sin_addr.s_addr;
	tdi->addr.sin_port 			= addr.sin_port;

	ret = tcc_dxb_internal_bind(tdi->sock, (struct sockaddr *)&tdi->addr, sizeof(struct sockaddr));
	if (ret < 0) {
		eprintk("kernel_bind() failed: %d\n", ret);
		return ret;
	}

	tdi->req.imr_multiaddr.s_addr = addr.sin_addr.s_addr;
	tdi->req.imr_interface.s_addr = _ADDRESS_ANY;
	ret = tcc_dxb_internal_setsockopt(tdi->sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&tdi->req, sizeof(tdi->req));
	if (ret < 0) {
		eprintk("kernel_setsockopt() for IP_ADD_MEMBERSHIP failed: %d\n", ret);
		return ret;
	}

	ret = tcc_dxb_internal_setsockopt(tdi->sock, SOL_SOCKET, SO_RCVBUF, (char *)&sock_RecvBufSize, sizeof(sock_RecvBufSize));
	if(ret < 0) {
		eprintk("kernel_setsockopt() for SO_RCVBUF failed: %d\n", ret);
		return ret;
	}
	ret = tcc_dxb_internal_getsockopt(tdi->sock, SOL_SOCKET, SO_RCVBUF, (char *)&sock_RecvBufSize, &optlen);
	if(ret < 0) {
		eprintk("kernel_getsockopt() for SO_RCVBUF failed: %d\n", ret);
	}
	else {
		dprintk("kernel_getsockopt() for SO_RCVBUF size:%d\n", sock_RecvBufSize);
	}
	return 0;
}

static int tcc_dxb_internal_stop(struct tcc_dxb_internal_t *tdi) {
	int ret = -1;

	dprintk("%s\n", __FUNCTION__);
	if(tdi == NULL) {
		eprintk("%s instance is null !!!\n", __FUNCTION__);
		return ret;
	}
	if(tdi->sock != NULL) {
		tcc_dxb_internal_setsockopt(tdi->sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char *)&tdi->req, sizeof(tdi->req));
		sock_release(tdi->sock);
		tdi->sock = NULL;
	}
	return 0;
}

static int backup_tsif_interface = 0;
static int tcc_dxb_internal_open(struct inode *inode, struct file *filp) {
	struct tcc_dxb_internal_t *tdi = NULL;
	unsigned minor_devID = iminor(filp->f_path.dentry->d_inode);

	dprintk("%s:[0x%p]\n", __FUNCTION__, filp);

	mutex_lock(&dev_mutex);
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	if(!((drvdata != NULL) && (drvdata[minor_devID] != NULL) && (drvdata[minor_devID]->state == 0))) {
		eprintk("%s already open\n", __FUNCTION__);
		mutex_unlock(&dev_mutex);
		return -1;
	}
#endif
	if(gNumOfOpen++ == 0) {
		backup_tsif_interface = tca_tsif_get_interface();
#ifdef INTERNAL_INTPUT_THREAD
		if(kfifo_alloc(&kff, 1024, GFP_KERNEL)) {
			eprintk("%s fifo create fail\n", __FUNCTION__);
			if(gNumOfOpen-- == 1)
				tca_tsif_set_interface(backup_tsif_interface);
			mutex_unlock(&dev_mutex);
			return -1;
		}
		if(ii_thread == NULL) {
			ii_thread = (struct task_struct *)kthread_run(tcc_dxb_internal_input_thread, (void *)tdi, "tcc_dxb_internal_input_thread");
		}
		if(ii_thread == NULL) {
			eprintk("%s internal_input_thread fail\n", __FUNCTION__);
			if(gNumOfOpen-- == 1)
				tca_tsif_set_interface(backup_tsif_interface);
			kfifo_free(&kff);
			mutex_unlock(&dev_mutex);
			return -1;
		}
#endif
	}
	tca_tsif_set_interface(2);
	tdi = (struct tcc_dxb_internal_t *)kmalloc(sizeof(struct tcc_dxb_internal_t), GFP_KERNEL);
	if(tdi == NULL) {
		eprintk("%s tcc_dxb_internal_t kmalloc failed %dbytes\n", __FUNCTION__, sizeof(struct tcc_dxb_internal_t));
		if(gNumOfOpen-- == 1) {
			tca_tsif_set_interface(backup_tsif_interface);
#ifdef INTERNAL_INTPUT_THREAD
			if(ii_thread != NULL) {
				kthread_stop(ii_thread);
				ii_thread = NULL;
			}
			kfifo_free(&kff);
#endif
		}
		mutex_unlock(&dev_mutex);
		return -1;
	}
	memset(tdi, 0x0, sizeof(struct tcc_dxb_internal_t));
	tdi->dmx_id = minor_devID;
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	tdi->dma_buf_vir_addr = drvdata[minor_devID]->dma_vir_buf;
	tdi->dma_buf_phy_addr = drvdata[minor_devID]->dma_phy_buf;
	tdi->sock_recieve_buf = drvdata[minor_devID]->sock_buf;
	drvdata[minor_devID]->state = 1;
#else
	tdi->dma_buf_vir_addr = dma_alloc_coherent(0, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, &tdi->dma_buf_phy_addr, GFP_KERNEL);
	if(tdi->dma_buf_vir_addr == NULL) {
		eprintk("%s dma alloc failed %dbytes\n", __FUNCTION__, _DMA_BUF_LEN*_NUM_OF_DMA_BUF);
		if(gNumOfOpen-- == 1) {
			tca_tsif_set_interface(backup_tsif_interface);
#ifdef INTERNAL_INTPUT_THREAD
			if(ii_thread != NULL) {
				kthread_stop(ii_thread);
				ii_thread = NULL;
			}
			kfifo_free(&kff);
#endif
		}
		if(tdi != NULL) {
			kfree(tdi);
		}
		mutex_unlock(&dev_mutex);
		return -1;
	}
	tdi->sock_recieve_buf = (unsigned char *)kmalloc(_SOCKET_RECV_SIZE, GFP_KERNEL);
	if(tdi->sock_recieve_buf == NULL) {
		eprintk("%s sock_recieve_buf kmalloc failed %dbytes\n", __FUNCTION__, _SOCKET_RECV_SIZE);
		if(gNumOfOpen-- == 1) {
			tca_tsif_set_interface(backup_tsif_interface);
#ifdef INTERNAL_INTPUT_THREAD
			if(ii_thread != NULL) {
				kthread_stop(ii_thread);
				ii_thread = NULL;
			}
			kfifo_free(&kff);
#endif
		}
		if(tdi->dma_buf_vir_addr) {
			dma_free_coherent(0, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, tdi->dma_buf_vir_addr, tdi->dma_buf_phy_addr);
		}
		if(tdi != NULL) {
			kfree(tdi);
		}
		mutex_unlock(&dev_mutex);
		return -1;
	}
#endif
	tdi->write_buf_count = 0;
	tdi->write_ptr_pos = 0;


	filp->private_data = (void *)tdi;
	mutex_unlock(&dev_mutex);

	return 0;
}

static int tcc_dxb_internal_release(struct inode *inode, struct file *filp) {
	struct tcc_dxb_internal_t *tdi = (struct tcc_dxb_internal_t *)filp->private_data;
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	unsigned minor_devID = iminor(filp->f_path.dentry->d_inode);
#endif
	dprintk("%s\n", __FUNCTION__);

	mutex_lock(&dev_mutex);
	if(tdi != NULL) {
		tcc_dxb_internal_clean(tdi);
	}
	if(gNumOfOpen-- == 1) {
		tca_tsif_set_interface(backup_tsif_interface);
#ifdef INTERNAL_INTPUT_THREAD
		if(ii_thread != NULL) {
			kthread_stop(ii_thread);
			ii_thread = NULL;
		}
		kfifo_free(&kff);
#endif
	}
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	if(drvdata != NULL && drvdata[minor_devID] != NULL) {
		drvdata[minor_devID]->state = 0;
	}
#else
	if(tdi->sock_recieve_buf)
		kfree(tdi->sock_recieve_buf);
	if(tdi->dma_buf_vir_addr) {
		dma_free_coherent(0, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, tdi->dma_buf_vir_addr, tdi->dma_buf_phy_addr);
	}
#endif
	if(tdi != NULL) {
		kfree(tdi);
	}
	mutex_unlock(&dev_mutex);
	
	return 0;
}

static long tcc_dxb_internal_ioctl(struct file *filp, unsigned cmd, unsigned long arg) {
	struct tcc_dxb_internal_t *tdi = (struct tcc_dxb_internal_t *)filp->private_data;
	int ret = -2;

	dprintk("%s:0x%X\n", __FUNCTION__, cmd);
	mutex_lock(&dev_mutex);
	switch(cmd) {
		case TCC_DXB_INTERNAL_START:
		{
			struct sockaddr_in addr;
			if(copy_from_user((void *)&addr, (const void *)arg, sizeof(struct sockaddr_in))) {
				ret = -EFAULT;
				break;
			}
			ret = tcc_dxb_internal_start(tdi, addr);
			if(ret) {
				tcc_dxb_internal_clean(tdi);
				break;
			}
			tcc_dxb_internal_start_feed(tdi);
			ret = 0;
			break;
		}
		break;
		case TCC_DXB_INTERNAL_END:
		{
			tcc_dxb_internal_stop_feed(tdi);
			tcc_dxb_internal_stop(tdi);
			ret = 0;
			break;
		}
		break;
		case TCC_DXB_INTERNAL_GET_DMX_ID:
		{
			ret = tdi->dmx_id;
		}
		break;
		default :
		{
			ret = 0;
			break;
		}
		break;
	}
	mutex_unlock(&dev_mutex);
	return ret;
}

static int tcc_dxb_internal_mmap(struct file *filp, struct vm_area_struct *vma) {
	struct tcc_dxb_internal_t *tdi = (struct tcc_dxb_internal_t *)filp->private_data;
	int ret;

	dprintk("%s\n", __FUNCTION__);

	if (!(tdi->dma_buf_phy_addr)) {
		eprintk("%s memory error\n", __FUNCTION__);
		return -EAGAIN;
	}
	if ((vma->vm_end - vma->vm_start) != _DMA_BUF_LEN*_NUM_OF_DMA_BUF) {
		eprintk("%s size error buffer_size[%d] requested_size[%lu]\n", __FUNCTION__, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, vma->vm_end-vma->vm_start);
		return -EAGAIN;
	}
	ret = remap_pfn_range(vma, vma->vm_start, (unsigned long)tdi->dma_buf_phy_addr, vma->vm_end - vma->vm_start, PAGE_SHARED);
	if (ret != 0) {
		eprintk("%s remap_pfn_range error ret[%d]\n", __FUNCTION__, ret);
		return -EAGAIN;
	}
	return 0;
}

#ifdef TCC_DXB_INTERNAL_WRITE
static int tcc_dxb_internal_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos) {
	struct tcc_dxb_internal_t *tdi = (struct tcc_dxb_internal_t *)filp->private_data;
	unsigned long missing;
	int status = 0;

	mutex_lock(&dev_mutex);
	if(tdi->dma_buf_vir_addr==NULL) {
		mutex_unlock(&dev_mutex);
		return -EFAULT;
	}
	if(count > _DMA_BUF_LEN) {
		mutex_unlock(&dev_mutex);
		return -EFAULT;
	}

	if((tdi->write_ptr_pos + count) >= _DMA_BUF_LEN*_NUM_OF_DMA_BUF)
		tdi->write_ptr_pos = 0;

	missing = copy_from_user(tdi->dma_buf_vir_addr+ tdi->write_ptr_pos, buf, count);
	if(missing == 0) {
		status = tca_tsif_input_internal(tdi->dmx_id, tdi->dma_buf_phy_addr + tdi->write_ptr_pos, count);
		if(status == 0) {
			status = count;
			tdi->write_ptr_pos+= count;
		}
		else
			eprintk("%s[0x%X][v:0x%p][p:0x%X] internal input error\n", __FUNCTION__, status, tdi->dma_buf_vir_addr, tdi->dma_buf_phy_addr);
	}
	else
		status = -EFAULT;
	mutex_unlock(&dev_mutex);
	return status;
}
#endif

static struct file_operations tcc_dxb_internal_fops = {
	.owner			= THIS_MODULE,
	.open			= tcc_dxb_internal_open,
#ifdef TCC_DXB_INTERNAL_WRITE
	.write			= tcc_dxb_internal_write,
#endif
	.release		= tcc_dxb_internal_release,
	.unlocked_ioctl = tcc_dxb_internal_ioctl,
	.mmap			= tcc_dxb_internal_mmap,
};

/* Module Operations */
static int tcc_dxb_internal_probe(struct platform_device *pdev) {
	struct device *dev = &(pdev->dev);
	int res = 0;
	int i = 0;

	of_property_read_u32(dev->of_node, "hwdmx_num", &hwdmx_num);

	dprintk("%s number_of_device[%d]\n", __FUNCTION__, hwdmx_num);

	res = register_chrdev(0, DXB_INTERNAL_NAME, &tcc_dxb_internal_fops);
	if(res < 0)
		return res; 
	giMajorDevID = res;
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	drvdata = (struct tcc_dxb_internal_drvdata_t **)kmalloc(sizeof(struct tcc_dxb_internal_drvdata_t *)*hwdmx_num, GFP_KERNEL);
#endif
	tcc_dxb_internal_class = class_create(THIS_MODULE, DXB_INTERNAL_NAME);
	for(i=0;i<hwdmx_num;i++) {
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
		drvdata[i] = (struct tcc_dxb_internal_drvdata_t *)kmalloc(sizeof(struct tcc_dxb_internal_drvdata_t), GFP_KERNEL);
		if(drvdata[i] == NULL) {
			eprintk("%s drvdata kmalloc failed\n", __FUNCTION__);
		}
#ifdef CONFIG_ARM64
		drvdata[i]->dma_vir_buf = (unsigned char *)dma_alloc_coherent(&pdev->dev, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, &drvdata[i]->dma_phy_buf, GFP_KERNEL);
#else
		drvdata[i]->dma_vir_buf = (unsigned char *)dma_alloc_coherent(0, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, &drvdata[i]->dma_phy_buf, GFP_KERNEL);
#endif
		if(drvdata[i]->dma_vir_buf == NULL) {
			eprintk("%s dma alloc failed %dbytes\n", __FUNCTION__, _DMA_BUF_LEN*_NUM_OF_DMA_BUF);
		}
		drvdata[i]->sock_buf = (unsigned char *)kmalloc(_SOCKET_RECV_SIZE, GFP_KERNEL);
		if(drvdata[i]->sock_buf == NULL) {
			eprintk("%s buf kmalloc failed %dbytes\n", __FUNCTION__, _SOCKET_RECV_SIZE);
		}
		drvdata[i]->state = 0;
#endif
		device_create(tcc_dxb_internal_class, NULL, MKDEV(giMajorDevID, i), NULL, "tcc_dxb_internal%d", i);
	}

	mutex_init(&dev_mutex);
#ifdef INTERNAL_INTPUT_THREAD
	mutex_init(&kff_mutex);
#else
	mutex_init(&hwdmx_mutex);
#endif

	return 0;
}

static int tcc_dxb_internal_remove(struct platform_device *pdev) {
	int i = 0;

	dprintk("%s number_of_device[%d]\n", __FUNCTION__, hwdmx_num);
	for(i=0;i<hwdmx_num;i++) {
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
		if(drvdata[i] != NULL) {
			drvdata[i]->state = 0;
			if(drvdata[i]->sock_buf != NULL) {
				kfree(drvdata[i]->sock_buf);
			}
			if(drvdata[i]->dma_vir_buf != NULL) {
				dma_free_coherent(0, _DMA_BUF_LEN*_NUM_OF_DMA_BUF, drvdata[i]->dma_vir_buf, drvdata[i]->dma_phy_buf);
			}
			kfree(drvdata[i]);
		}
#endif
		device_destroy(tcc_dxb_internal_class, MKDEV(giMajorDevID, i));
	}
#ifdef TCC_DXB_INTERNAL_PRE_ALLOC
	if(drvdata != NULL) {
		kfree(drvdata);
		drvdata = NULL;
	}
#endif
	class_destroy(tcc_dxb_internal_class);
	unregister_chrdev(giMajorDevID, DXB_INTERNAL_NAME);

	mutex_destroy(&dev_mutex);
#ifdef INTERNAL_INTPUT_THREAD
	mutex_destroy(&kff_mutex);
#else
	mutex_destroy(&hwdmx_mutex);
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tcc_dxb_internal_of_match[] = {
	{.compatible = "telechips,tcc_dxb_internal", },
	{ },
};
MODULE_DEVICE_TABLE(of, tcc_dxb_internal_of_match);
#endif

static struct platform_driver tcc_dxb_internal = {
	.probe  = tcc_dxb_internal_probe,
	.remove = tcc_dxb_internal_remove,
	.driver = {
				.name   = DXB_INTERNAL_NAME,
				.owner  = THIS_MODULE,
#ifdef CONFIG_OF
				.of_match_table = tcc_dxb_internal_of_match,
#endif
	},
};

static int __init tcc_dxb_internal_module_init(void) {
	dprintk("%s success\n", __FUNCTION__);

	platform_driver_register(&tcc_dxb_internal);

	return 0;
}

static void __exit tcc_dxb_internal_module_exit(void) {
	platform_driver_unregister(&tcc_dxb_internal);

	dprintk("%s\n", __FUNCTION__);
}

module_init(tcc_dxb_internal_module_init);
module_exit(tcc_dxb_internal_module_exit);

MODULE_DESCRIPTION("TCCxxx Dxb Internal TS Input Module");
MODULE_AUTHOR("Telechips Inc.");
MODULE_LICENSE("GPL");
