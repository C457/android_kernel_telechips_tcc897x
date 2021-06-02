/*
 * linux/drivers/spi/tcc_hwdemux_tsif.c
 *
 * Copyright (C) 2010 Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/spi/tcc_tsif.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include <soc/tcc/pmap.h>

#include "tcc_hwdemux/tca_hwdemux_tsif.h"

#include <asm/io.h>

#include "tsdemux/TSDEMUX_sys.h"
#include "tcc_dxb_drv/HWDemux_bin.h"

#define USE_REV_MEMORY
#define USE_DMA_BUFFER
#define USE_STATIC_DMA_BUFFER

#undef  TSIF_DMA_SIZE
#define TSIF_DMA_SIZE (188*10000)
#define SECTION_DMA_SIZE (512*1024)

#define HWDMX_NUM         2

#define	MAX_PCR_CNT 2
//#define TS_PACKET_CHK_MODE

static const char fw_name[] = "tcc_tsif/HWDemux.bin";
static struct device *gpdev = NULL;
static DEFINE_SPINLOCK(timer_lock);

extern struct class *tsif_class;

struct ts_demux_feed_struct {
    int is_active; //0:don't use external demuxer, 1:use external decoder
    int index;
    int call_decoder_index;
    int (*ts_demux_decoder)(char *p1, int p1_size, char *p2, int p2_size, int devid);
};

struct tcc_hwdmx_handle {
    struct ts_demux_feed_struct ts_demux_feed_handle;
    struct tcc_tsif_handle tsif_ex_handle;
    struct tcc_tsif_pid_param ts_pid;
    int pcr_pid[MAX_PCR_CNT];
    int read_buff_offset;
    int write_buff_offset;
    int end_buff_offset;
    int loop_count;
    int state;
    unsigned char * mem;
    wait_queue_head_t wait_q;
    unsigned int dma_mode; //0:normal mode (not mpeg2ts mode), 1:mpeg2ts mode(length 188, start with 0x47, or other identifier(tdmb-tsif))
    unsigned int use_pid_filter; //0:don't use pid filter, 1:use pid filter(linuxtv), 2:use pid filter(non linuxtv - old way)

    int use_tsif_export_ioctl;
};

struct tcc_tsif_pri_handle {
    int open_cnt;
    u32 drv_major_num;

    struct mutex mutex;
    struct tcc_hwdmx_handle demux[HWDMX_NUM];
    struct timer_list timer;

#if defined(USE_REV_MEMORY)
    pmap_t pmap_tsif;
    void *mem_base;
#endif
#if defined(USE_DMA_BUFFER)
    struct tea_dma_buf *static_dma_buffer[HWDMX_NUM];
#endif
	struct tca_tsif_port_config port_cfg[HWDMX_NUM];
	struct device *dev[HWDMX_NUM];
};

static struct tcc_tsif_pri_handle tsif_ex_pri;

static int tcc_hwdmx_tsif_init(struct firmware *fw, struct tcc_hwdmx_handle *demux, int id);
static void tcc_hwdmx_tsif_deinit(struct tcc_hwdmx_handle *demux);

int tcc_hwdmx_tsif_probe(struct platform_device *pdev);
int tcc_hwdmx_tsif_remove(struct platform_device *pdev);
int tcc_hwdmx_tsif_open(struct inode *inode, struct file *filp);
int tcc_hwdmx_tsif_release(struct inode *inode, struct file *filp);
static long tcc_hwdmx_tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int tcc_hwdmx_tsif_ioctl_ex(struct file *filp, unsigned int cmd, unsigned long arg);
ssize_t tcc_hwdmx_tsif_read(struct file *filp, char *buf, size_t len, loff_t *ppos);
ssize_t tcc_hwdmx_tsif_write(struct file *filp, const char *buf, size_t len, loff_t *ppos);
unsigned int tcc_hwdmx_tsif_poll(struct file *filp, struct poll_table_struct *wait);
int tcc_hwdmx_tsif_set_external_tsdemux(struct file *filp, int (*decoder)(char *p1, int p1_size, char *p2, int p2_size, int devid), int max_dec_packet, int devid);
int tcc_hwdmx_tsif_buffer_flush(struct file *filp, unsigned long x, int size);

struct file_operations tcc_hwdmx_tsif_fops = {
    .owner          = THIS_MODULE,
    .read           = tcc_hwdmx_tsif_read,
    .write          = tcc_hwdmx_tsif_write,
    .unlocked_ioctl = tcc_hwdmx_tsif_ioctl,
    .open           = tcc_hwdmx_tsif_open,
    .release        = tcc_hwdmx_tsif_release,
    .poll           = tcc_hwdmx_tsif_poll,
};

extern int tsif_major_num;
extern int gpsb_tsif_num;
extern int hwdmx_tsif_num;
extern int *pTsif_mode;

#ifdef TS_PACKET_CHK_MODE
typedef	struct {
	struct ts_packet_chk_t* next;
	
	unsigned short pid;
	unsigned char cc;
	
	unsigned long long cnt;
	unsigned int err;
} ts_packet_chk_t;


typedef	struct {
	unsigned long long total_cnt;
	unsigned int total_err;
	unsigned int packet_cnt;

	#define DEBUG_CHK_TIME		10	//sec
	unsigned int cur_time;
	unsigned int prev_time;
	unsigned int debug_time;
	struct timeval time;

  	ts_packet_chk_t* packet;
} ts_packet_chk_info_t;
ts_packet_chk_info_t*	ts_packet_chk_info = NULL;

static void itv_ts_cc_debug(int mod) 
{
	if(ts_packet_chk_info != NULL) {
		if(ts_packet_chk_info->packet != NULL) {
			printk("\n[total:%llu / err:%d (%d sec)]\n", ts_packet_chk_info->total_cnt, ts_packet_chk_info->total_err, (ts_packet_chk_info->debug_time * DEBUG_CHK_TIME));
			
			if(mod) {
				ts_packet_chk_t* tmp = NULL;
				
				tmp = ts_packet_chk_info->packet;
				do {
					printk("\t\tpid:0x%04x => cnt:%llu err:%d\n", tmp->pid, tmp->cnt, tmp->err);
					tmp = tmp->next;
				} while(tmp != NULL);
			}
		}
	}
}

static void itv_ts_cc_remove(unsigned short pid)
{
	ts_packet_chk_t* tmp = NULL;
	ts_packet_chk_t* tmp_prev = NULL;

	if(ts_packet_chk_info->packet) {
		tmp = ts_packet_chk_info->packet;
		while(1) {
			if(tmp->pid == pid) {
				if(tmp->next == NULL) {
					if(tmp_prev == NULL) {		//1st
						ts_packet_chk_info->packet = NULL;
						kfree(tmp);
					} else {					//last
						tmp_prev->next = NULL;
						kfree(tmp);
					}
				} else {
					tmp_prev = tmp;
					tmp = tmp->next;

					ts_packet_chk_info->packet = tmp;
					kfree(tmp_prev);
				}
				
				break;
			} else {
				tmp_prev = tmp;
				tmp = tmp->next;

				if(tmp == NULL) {
					break;
				}
			}
		}
	}
}

static void itv_ts_cc_check(unsigned char* buf)
{
	unsigned short pid;
	unsigned char cc;

	ts_packet_chk_t* tmp = NULL;

	pid = ((*(unsigned char*)(buf + 1) & 0x1f) << 8) | *(unsigned char*)(buf + 2);
	cc 	= *(unsigned char*)(buf + 3) & 0x0f;
	
	if(ts_packet_chk_info != NULL) {
		ts_packet_chk_info->total_cnt++;

		if(ts_packet_chk_info->packet == NULL) {
			tmp = (ts_packet_chk_t*)kmalloc(sizeof(ts_packet_chk_t), GFP_ATOMIC);
			if(tmp == NULL) {
				printk("\t ts_packet_chk_t mem alloc err..\n");
			}

			memset(tmp, 0x0, sizeof(ts_packet_chk_t));
			tmp->pid = pid;
			tmp->cc	= cc;
			tmp->next = NULL;
			tmp->cnt++;
			ts_packet_chk_info->packet = tmp;
			ts_packet_chk_info->packet_cnt++;
			
			printk("\t>>>> create[%d] : 0x%04x / %02d\n", ts_packet_chk_info->packet_cnt, tmp->pid, tmp->cc);
		} else {
			unsigned char new = 0;
			unsigned int temp;
	
			tmp = ts_packet_chk_info->packet;
			while(1) {
				if(tmp->pid == pid) {
					tmp->cnt++;

					if(tmp->pid != 0x1fff) {
						if(cc > tmp->cc) {
							temp = tmp->err;
							tmp->err += ((cc - tmp->cc + 0xf) % 0xf) - 1;
							if(temp != tmp->err) {
								ts_packet_chk_info->total_err += tmp->err - temp;
								printk("\t(%dmin) pid:0x%04x => cnt:%llu err:%d [%d -> %d]\n", ts_packet_chk_info->debug_time, tmp->pid, tmp->cnt, tmp->err, tmp->cc, cc);
							}
							tmp->cc = cc;
						} else if(cc < tmp->cc) {
							temp = tmp->err;
							tmp->err += (0x10 - tmp->cc + cc) - 1;
							if(temp != tmp->err) {
								ts_packet_chk_info->total_err += tmp->err - temp;
								printk("\t(%dmin) pid:0x%04x => cnt:%llu err:%d [%d -> %d]\n", ts_packet_chk_info->debug_time, tmp->pid, tmp->cnt, tmp->err, tmp->cc, cc);
							}
							tmp->cc = cc;
						}
					}
					break;
				} else {
					tmp = tmp->next;
					
					if(tmp == NULL) {
						new = 1;
						break;
					}
				}
			}

			if(new) {
				ts_packet_chk_t* tmp_ = NULL;
	
				tmp = (ts_packet_chk_t*)kmalloc(sizeof(ts_packet_chk_t), GFP_ATOMIC);
				if(tmp == NULL) {
					printk("\t ts_packet_chk_t mem alloc err..\n");
				}

				memset(tmp, 0x0, sizeof(ts_packet_chk_t));
				tmp->pid = pid;
				tmp->cc	= cc;
				tmp->next = NULL;
				tmp->cnt++;
				
				ts_packet_chk_info->packet_cnt++;
				tmp_ = ts_packet_chk_info->packet;
				do {
					if(tmp_->next == NULL) {
						tmp_->next= tmp;
						break;
					} else {
						tmp_ = tmp_->next;
					}
				} while(1);

				printk("\t>>>> create[%d] : 0x%04x / %02d\n", ts_packet_chk_info->packet_cnt, tmp->pid, tmp->cc);
			}
		}

		do_gettimeofday(&ts_packet_chk_info->time);
		ts_packet_chk_info->cur_time = ((ts_packet_chk_info->time.tv_sec * 1000) & 0x00ffffff) + (ts_packet_chk_info->time.tv_usec / 1000);
		if((ts_packet_chk_info->cur_time - ts_packet_chk_info->prev_time) > (DEBUG_CHK_TIME * 1000)) {
			//itv_ts_cc_debug(0);
			itv_ts_cc_debug(1);

			ts_packet_chk_info->prev_time = ts_packet_chk_info->cur_time;
			ts_packet_chk_info->debug_time++;
		}
	}
}
#endif	//#ifdef TS_PACKET_CHK_MODE

#if 0
static void tcc_tsif_timer_handler(unsigned long data)
{
    struct tcc_hwdmx_handle *demux;
    int i;

    for(i=0; i< HWDMX_NUM; i++)
    {
        demux = &tsif_ex_pri.demux[i];

        if(demux->state == 1 && 0 < demux->pcr_pid[0] && demux->pcr_pid[0] < 0x1FFF)
        {
            tca_tsif_get_stc(&demux->tsif_ex_handle);
        }
    }
    if (tsif_ex_pri.open_cnt) {
        tsif_ex_pri.timer.expires = jiffies + msecs_to_jiffies(1000);//msecs_to_jiffies(125);
        add_timer(&tsif_ex_pri.timer);
    }
}
#endif

#ifdef CONFIG_OF
static void tcc_hwdmx_tsif_parse_dt(struct device_node *np, struct tca_tsif_port_config *pcfg)
{
    of_property_read_u32(np, "tsif-port", &pcfg->tsif_port);
    of_property_read_u32(np, "tsif-id", &pcfg->tsif_id);
}
#else
static void tcc_hwdmx_tsif_parse_dt(struct device_node *np, struct tca_tsif_port_config *pcfg)
{
}
#endif

static int tcc_hwdmx_tsif_update_stc(struct tcc_hwdmx_handle *demux, char *p1, int p1_size, char *p2, int p2_size)
{
    int i;
    int dmxid = demux->tsif_ex_handle.dmx_id;

    for (i = 1; i < MAX_PCR_CNT; i++)
    {
        if (0 < demux->pcr_pid[i] && demux->pcr_pid[i] < 0x1FFFF)
        {
            if (p1)
                TSDEMUX_MakeSTC((unsigned char *)p1, p1_size, demux->pcr_pid[i], i + dmxid*MAX_PCR_CNT);
            if (p2)
                TSDEMUX_MakeSTC((unsigned char *)p2, p2_size, demux->pcr_pid[i], i + dmxid*MAX_PCR_CNT);
        }
    }
    return 0;
}

static int tcc_hwdmx_tsif_parse_packet(struct tcc_hwdmx_handle *demux, char *pBuffer, int updated_buff_offset, int end_buff_offset)
{
    int ret = 0;
    char *p1 = NULL, *p2 = NULL;
    int p1_size = 0, p2_size = 0;
    int packet_th = demux->ts_demux_feed_handle.call_decoder_index;
    if(++demux->ts_demux_feed_handle.index >= packet_th) {

        if( updated_buff_offset > demux->write_buff_offset ) {
            p1 = (char *)(pBuffer + demux->write_buff_offset);
            p1_size = updated_buff_offset - demux->write_buff_offset;
        } else if (end_buff_offset == demux->write_buff_offset) {
            p1 = (char *)pBuffer;
            p1_size = updated_buff_offset;
        } else {
            p1 = (char *)(pBuffer + demux->write_buff_offset);
            p1_size = end_buff_offset - demux->write_buff_offset;

            p2 = (char *)pBuffer;
            p2_size = updated_buff_offset;
        }

        if (p1 == NULL || p1_size < 188) return 0;

        tcc_hwdmx_tsif_update_stc(demux, p1, p1_size, p2, p2_size);

        if(demux->ts_demux_feed_handle.ts_demux_decoder(p1, p1_size, p2, p2_size, demux->tsif_ex_handle.dmx_id) == 0)
        {
            demux->write_buff_offset = updated_buff_offset;
            demux->ts_demux_feed_handle.index = 0;
        }

#ifdef TS_PACKET_CHK_MODE
        {
            int i;
            if(p1 && p1_size)
            {
                for(i=0; i<p1_size; i+= 188)
    				itv_ts_cc_check(p1+i);
            }

            if(p2 && p2_size)
            {
                for(i=0; i<p2_size; i+= 188)
    				itv_ts_cc_check(p2+i);
            }
        }
#endif
        
    }
    return ret;
}

static int tcc_hwdmx_tsif_push_buffer(struct tcc_hwdmx_handle *demux, char *pBuffer, int updated_buff_offset, int end_buff_offset)
{
    char *p1 = NULL, *p2 = NULL;
    int p1_size = 0, p2_size = 0;
    int ret = 0;

    demux->loop_count++;
    if(demux->loop_count) {
        demux->loop_count = 0;

        if(updated_buff_offset >= end_buff_offset)
            updated_buff_offset = 0;

        demux->end_buff_offset = end_buff_offset;
        if( updated_buff_offset >= demux->write_buff_offset ){
            p1 = (char *)(pBuffer + demux->write_buff_offset);
            p1_size = updated_buff_offset - demux->write_buff_offset;
        } else {
            p1 = (char *)(pBuffer + demux->write_buff_offset);
            p1_size = end_buff_offset - demux->write_buff_offset;

            p2 = (char *)pBuffer;
            p2_size = updated_buff_offset;
        }

        tcc_hwdmx_tsif_update_stc(demux, p1, p1_size, p2, p2_size);

		demux->write_buff_offset = updated_buff_offset;

        wake_up(&(demux->wait_q));
    }
    return ret;
}

static int tcc_hwdmx_tsif_updated_callback(unsigned int dmxid, unsigned int ftype, unsigned int fid, unsigned int value1, unsigned int value2, unsigned int bErrCRC)
{
    struct tcc_hwdmx_handle *demux = &tsif_ex_pri.demux[dmxid];
    unsigned int uiSTC;
    int ret = 0;

    switch(ftype)
    {
        case 0: // HW_DEMUX_SECTION
        {
            if (demux->ts_demux_feed_handle.is_active != 0)
            {
                //printk("0x%x, 0x%x, 0x%x\n", demux->tsif_ex_handle.dma_buffer->v_sec_addr, value1, demux->tsif_ex_handle.dma_buffer->dma_sec_addr);
                ret = demux->ts_demux_feed_handle.ts_demux_decoder((char *)fid,
                                                                   bErrCRC,
                                                                   demux->tsif_ex_handle.dma_buffer->v_sec_addr + value1,
                                                                   value2 - value1,
                                                                   demux->tsif_ex_handle.dmx_id);
            }
            break;
        }
        case 1: // HW_DEMUX_TS
        {
            if (demux->ts_demux_feed_handle.is_active != 0)
            {
                ret = tcc_hwdmx_tsif_parse_packet(demux, demux->tsif_ex_handle.dma_buffer->v_addr, value1, value2);
            }
            else
            {
                ret = tcc_hwdmx_tsif_push_buffer(demux, demux->tsif_ex_handle.dma_buffer->v_addr, value1, value2);
            }
            break;
        }
        case 2: // HW_DEMUX_PES
        {
            break;
        }
        case 3: // HW_DEMUX_PCR
        {
            //bErrCRC is error(ms) for PCR & STC
            //printk("%s: STC Error (%d)ms \n", __func__, bErrCRC);
            uiSTC = (unsigned int)value2 & 0x80000000;
            uiSTC |= (unsigned int)value1 >> 1;
            TSDEMUX_UpdatePCR(uiSTC / 45, dmxid*MAX_PCR_CNT);
            break;
        }
        default:
        {
            printk("Invalid parameter: Filter Type : %d\n", ftype);
            break;
        }
    }

    return ret;
}

int tcc_hwdmx_tsif_set_external_tsdemux(struct file *filp, int (*decoder)(char *p1, int p1_size, char *p2, int p2_size, int devid), int max_dec_packet, int devid)
{
    struct tcc_hwdmx_handle *demux = filp->private_data;

    if(max_dec_packet == 0) {
        //turn off external decoding
        //memset(&demux->ts_demux_feed_handle, 0x0, sizeof(struct ts_demux_feed_struct));
        demux->ts_demux_feed_handle.is_active = 0;
        return 0;
    }
    if(demux->ts_demux_feed_handle.call_decoder_index != max_dec_packet || demux->ts_demux_feed_handle.is_active == 0) {
        demux->ts_demux_feed_handle.ts_demux_decoder = decoder;
        demux->ts_demux_feed_handle.index = 0;
        demux->ts_demux_feed_handle.call_decoder_index = max_dec_packet; //every max_dec_packet calling isr, call decoder
        demux->ts_demux_feed_handle.is_active = 1;
        printk("%s::%d::max_dec_packet[%d]int_packet[%d]\n", __func__, __LINE__, demux->ts_demux_feed_handle.call_decoder_index, demux->tsif_ex_handle.dma_intr_packet_cnt);
    }
    return 0;
}

int tcc_hwdmx_tsif_buffer_flush(struct file *filp, unsigned long x, int size)
{
	struct tcc_hwdmx_handle *demux = filp->private_data;
	unsigned long offset;

#if defined(USE_REV_MEMORY)
	if (tsif_ex_pri.mem_base)
	{
		return 0;
	}
#endif

	if ((unsigned long)demux->tsif_ex_handle.dma_buffer->v_addr <= x)
	{
		offset = x - (unsigned long)demux->tsif_ex_handle.dma_buffer->v_addr;
		if (offset < demux->tsif_ex_handle.dma_buffer->buf_size)
		{
			x = demux->tsif_ex_handle.dma_buffer->dma_addr;
			dma_sync_single_for_cpu(0, x + offset, size, DMA_FROM_DEVICE);
			return 0;
		}
	}
	if ((unsigned long)demux->tsif_ex_handle.dma_buffer->v_sec_addr <= x)
	{
		offset = x - (unsigned long)demux->tsif_ex_handle.dma_buffer->v_sec_addr;
		if (offset < demux->tsif_ex_handle.dma_buffer->buf_sec_size)
		{
			x = demux->tsif_ex_handle.dma_buffer->dma_sec_addr;
			dma_sync_single_for_cpu(0, x + offset, size, DMA_FROM_DEVICE);
			return 0;
		}
	}
	return -1;
}

static char is_use_tsif_export_ioctl(struct tcc_hwdmx_handle *demux)
{
	return demux->use_tsif_export_ioctl;
}

static ssize_t tcc_hwdmx_tsif_copy_from_user(struct tcc_hwdmx_handle *demux, void *dest, void *src, size_t copy_size)
{
	int ret = 0;
	if(is_use_tsif_export_ioctl(demux) == 1) {
		memcpy(dest, src, copy_size);
	} else {
		ret = copy_from_user(dest, src, copy_size);
	}
	return ret;
}

static ssize_t tcc_hwdmx_tsif_copy_to_user(struct tcc_hwdmx_handle *demux, void *dest, void *src, size_t copy_size)
{
	int ret = 0;
	if(is_use_tsif_export_ioctl(demux) == 1) {
		memcpy(dest, src, copy_size);
	} else {
		ret = copy_to_user(dest, src, copy_size);
	}
	return ret;
}

static int tcc_hwdmx_tsif_calculate_readable_cnt(struct tcc_hwdmx_handle *demux)
{
	int dma_recv_size = 0;
	unsigned int read_buff_offset, write_buff_offset, end_buff_offset;
	int ret = 0;
	
	if(demux){
        read_buff_offset = demux->read_buff_offset;
        write_buff_offset = demux->write_buff_offset;
		end_buff_offset = demux->end_buff_offset;

		if(write_buff_offset >= read_buff_offset) {
		    dma_recv_size = write_buff_offset - read_buff_offset;
	    } else { 
		    dma_recv_size = (end_buff_offset - read_buff_offset) + write_buff_offset;
	    }

		if(dma_recv_size > 0){
			ret = dma_recv_size / TSIF_PACKET_SIZE;
		}
		return ret;
	}
    return 0;
}

static int tcc_hwdmx_tsif_get_readable_cnt(struct tcc_hwdmx_handle *demux)
{
    int readable_cnt = 0;
    if (demux) {
        readable_cnt = tcc_hwdmx_tsif_calculate_readable_cnt(demux);
    }
    return readable_cnt;
}

ssize_t tcc_hwdmx_tsif_read(struct file *filp, char *buf, size_t len, loff_t *ppos)
{
    struct tcc_hwdmx_handle *demux = filp->private_data;
    ssize_t ret = 0;

    char *packet_data = NULL;
    char *pBuffer = demux->tsif_ex_handle.dma_buffer->v_addr;

    int valid_cnt = tcc_hwdmx_tsif_get_readable_cnt(demux);

    if(valid_cnt > 0)
    {
        int request_cnt = len / TSIF_PACKET_SIZE;
        int copy_byte = request_cnt * TSIF_PACKET_SIZE;
        if(request_cnt <= valid_cnt)
        {
            unsigned int read_buff_offset = demux->read_buff_offset;
            unsigned int end_buff_offset = demux->end_buff_offset;

            if((read_buff_offset + copy_byte) < end_buff_offset)
            {
                packet_data = (char *)(pBuffer + read_buff_offset);
                if(packet_data) {
                    if (copy_to_user(buf, packet_data, copy_byte)) {
                        return -EFAULT;
                    }
                    ret = copy_byte;
                    demux->read_buff_offset += ret;
                }
            }
            else
            {
                int p1_size, p2_size;
                p1_size = end_buff_offset - read_buff_offset;
                packet_data = (char *)(pBuffer + read_buff_offset);
                if(packet_data){
                    if (copy_to_user(buf, packet_data, p1_size)) {
                        return -EFAULT;
                    }
                }
                p2_size = copy_byte - p1_size;
                packet_data = (char *)(pBuffer);
                if(packet_data && (p2_size > 0)){
                    if (copy_to_user(buf+p1_size, packet_data, p2_size)) {
                        return -EFAULT;
                    }
                    ret = p1_size + p2_size;
                }
                else
                    ret = p1_size;
                demux->read_buff_offset = p2_size;
            }
        }
    }

    return ret;
}

unsigned int tcc_hwdmx_tsif_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct tcc_hwdmx_handle *demux = filp->private_data;

    if (tcc_hwdmx_tsif_get_readable_cnt(demux) > 0) {
        return  (POLLIN | POLLRDNORM);
    }

    poll_wait(filp, &(demux->wait_q), wait);

    if (tcc_hwdmx_tsif_get_readable_cnt(demux) > 0) {
        return  (POLLIN | POLLRDNORM);
    }
    return 0;
}

ssize_t tcc_hwdmx_tsif_write(struct file *filp, const char *buf, size_t len, loff_t *ppos)
{
    return 0;
}

static long tcc_hwdmx_tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct tcc_hwdmx_handle *demux = filp->private_data;
    int ret = 0;

    switch (cmd) {
    case IOCTL_TSIF_DMA_START:
        {
            int i;
            struct tcc_tsif_param param;
            if (tcc_hwdmx_tsif_copy_from_user(demux, &param, (void *)arg, sizeof(struct tcc_tsif_param))) {
                printk("cannot copy from user tcc_tsif_param in IOCTL_TSIF_DMA_START !!! \n");
                return -EFAULT;
            }
            if (demux->state == 1) {
                demux->state = 0;
                tcc_hwdmx_tsif_deinit(demux);
            }
            demux->dma_mode = param.dma_mode;
            for (i = 0; i < HWDMX_NUM; i++)
            {
                if (demux == &tsif_ex_pri.demux[i])
                    break;
            }
            /* This call means this is not linux tv mode
             * This is old way, We can decide HWDemux working mode by use_pid_filter
             */
            if( demux->use_pid_filter == 2 )
                demux->use_pid_filter = 1;
            else
                demux->use_pid_filter = 0;

            demux->state = 1;
            tcc_hwdmx_tsif_init(NULL, demux, i);
        }
        break;			
    case IOCTL_TSIF_DMA_STOP:
        {
            demux->state = 0;
            demux->dma_mode = 0;
            demux->use_pid_filter = 0;
            tcc_hwdmx_tsif_deinit(demux);
        }
        break;			
    case IOCTL_TSIF_GET_MAX_DMA_SIZE:
        {
        }
        break;        
    case IOCTL_TSIF_SET_PID:
        {
            struct tcc_tsif_pid_param pid_param;
            struct tcc_tsif_filter filter_param;
            int i, j;

            if (tcc_hwdmx_tsif_copy_from_user(demux, (void *)&pid_param, (void*)arg, sizeof(struct tcc_tsif_pid_param)))
                return -EFAULT;

            // check changed pid
            for (i = 0; i < demux->ts_pid.valid_data_cnt; i++)
            {
                for (j = 0; j < pid_param.valid_data_cnt; j++)
                {
                    if (demux->ts_pid.pid_data[i] == pid_param.pid_data[j])
                    {
                        demux->ts_pid.pid_data[i] = pid_param.pid_data[j] = -1;
                        break;
                    }
                }
            }

            filter_param.f_id = 0;
            filter_param.f_type = 1;
            filter_param.f_size = 0;
            filter_param.f_comp = NULL;
            filter_param.f_mask = NULL;
            filter_param.f_mode = NULL;

            // delete pid
            for (i = 0; i < demux->ts_pid.valid_data_cnt; i++)
            {
                if (demux->ts_pid.pid_data[i] != -1)
                {
                    filter_param.f_pid = demux->ts_pid.pid_data[i];
                    ret = tca_tsif_remove_filter(&demux->tsif_ex_handle, &filter_param);
                }
            }

            // add pid
            for (i = 0; i < pid_param.valid_data_cnt; i++)
            {
                if (pid_param.pid_data[i] != -1)
                {
                    filter_param.f_pid = pid_param.pid_data[i];
                    ret = tca_tsif_add_filter(&demux->tsif_ex_handle, &filter_param);
                }
            }

            // save pid
            if (tcc_hwdmx_tsif_copy_from_user(demux, (void *)&demux->ts_pid, (void*)arg, sizeof(struct tcc_tsif_pid_param)))
                return -EFAULT;
            
            demux->use_pid_filter = 2; //use pid filter(not linuxtv)
        } 
        break;		    
    case IOCTL_TSIF_DXB_POWER:
    	{
    	}
        break;
    case IOCTL_TSIF_ADD_PID:
        {
            struct tcc_tsif_filter param;

            if (tcc_hwdmx_tsif_copy_from_user(demux, (void *)&param, (void*)arg, sizeof(struct tcc_tsif_filter)))
                return -EFAULT;

            ret = tca_tsif_add_filter(&demux->tsif_ex_handle, &param);
        }
        break;
    case IOCTL_TSIF_REMOVE_PID:
        {
            struct tcc_tsif_filter param;

            if (tcc_hwdmx_tsif_copy_from_user(demux, (void *)&param, (void*)arg, sizeof(struct tcc_tsif_filter)))
                return -EFAULT;

            ret = tca_tsif_remove_filter(&demux->tsif_ex_handle, &param);
        }
        break;
    case IOCTL_TSIF_SET_PCRPID:
        {
            struct tcc_tsif_pcr_param param;
            int dmxid = demux->tsif_ex_handle.dmx_id;

            if (tcc_hwdmx_tsif_copy_from_user(demux, (void *)&param, (void*)arg, sizeof(struct tcc_tsif_pcr_param)))
                return -EFAULT;

            if (param.index >= MAX_PCR_CNT) return -EFAULT;

            if(param.pcr_pid < 0x1FFF)
            {
                TSDEMUX_Open(param.index + dmxid*MAX_PCR_CNT);
            }
            demux->pcr_pid[param.index] = param.pcr_pid;
            if (param.index == 0) {
                ret = tca_tsif_set_pcrpid(&demux->tsif_ex_handle, demux->pcr_pid[param.index]);
            }
        }		
        break;
    case IOCTL_TSIF_GET_STC:
        {
            unsigned int uiSTC, index;
            unsigned long flags = 0;
            int dmxid = demux->tsif_ex_handle.dmx_id;

            if (tcc_hwdmx_tsif_copy_from_user(demux, (void*)&index, (void*)arg, sizeof(int)))
                return - EFAULT;

            index += (dmxid*MAX_PCR_CNT);
            spin_lock_irqsave(&timer_lock, flags);
            uiSTC = TSDEMUX_GetSTC(index);
            spin_unlock_irqrestore(&timer_lock, flags);
            if (tcc_hwdmx_tsif_copy_to_user(demux, (void *)arg, (void *)&uiSTC, sizeof(int))) {
                printk("cannot copy to user tcc_tsif_param in IOCTL_TSIF_GET_PCR !!! \n");
                return -EFAULT;
            }
        }
        break;
    case IOCTL_TSIF_RESET:
        {
        }
        break;
    case IOCTL_TSIF_HWDMX_START:
        {
            demux->write_buff_offset = 0;
            demux->read_buff_offset = 0;
            demux->end_buff_offset = 0;
            demux->loop_count = 0;
            tca_tsif_start(&demux->tsif_ex_handle);
        }
        break;
    case IOCTL_TSIF_HWDMX_STOP:
        {
            tca_tsif_stop(&demux->tsif_ex_handle);
        }
        break;
    default:
        printk("tcc-tsif : unrecognized ioctl (0x%X)\n", cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

int tcc_hwdmx_tsif_ioctl_ex(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct tcc_hwdmx_handle *demux = (filp == NULL) ? NULL : filp->private_data;

	if (demux != NULL)
	{
		demux->use_tsif_export_ioctl = 1;
		ret = tcc_hwdmx_tsif_ioctl(filp, cmd, arg);
		demux->use_tsif_export_ioctl = 0;
	}
	return ret;
}

#if defined(USE_DMA_BUFFER)
static int tcc_hwdmx_tsif_dma_buffer_alloc(struct tea_dma_buf *dma)
{
    dma->buf_size = TSIF_DMA_SIZE;
    dma->v_addr = dma_alloc_writecombine(0, dma->buf_size, &dma->dma_addr, GFP_KERNEL);
    if (dma->v_addr == NULL)
        return -1;

    dma->buf_sec_size = SECTION_DMA_SIZE;
    dma->v_sec_addr = dma_alloc_writecombine(0, dma->buf_sec_size, &dma->dma_sec_addr, GFP_KERNEL);
    if (dma->v_sec_addr == NULL)
    {
        dma_free_writecombine(0, dma->buf_size, dma->v_addr, dma->dma_addr);
        return -1;
    }

    printk("tsif : dma ts buffer alloc @0x%X(Phy=0x%X), size:%d\n", (unsigned int)dma->v_addr, (unsigned int)dma->dma_addr, dma->buf_size);
    printk("tsif : dma sec buffer alloc @0x%X(Phy=0x%X), size:%d\n", (unsigned int)dma->v_sec_addr, (unsigned int)dma->dma_sec_addr, dma->buf_sec_size);

    return 0;
}

static int tcc_hwdmx_tsif_dma_buffer_free(struct tea_dma_buf *dma)
{
    printk("tsif : dma ts buffer free @0x%X(Phy=0x%X), size:%d\n", (unsigned int)dma->v_addr, (unsigned int)dma->dma_addr, dma->buf_size);
    printk("tsif : dma sec buffer free @0x%X(Phy=0x%X), size:%d\n", (unsigned int)dma->v_sec_addr, (unsigned int)dma->dma_sec_addr, dma->buf_sec_size);

    if (dma->v_addr)
        dma_free_writecombine(0, dma->buf_size, dma->v_addr, dma->dma_addr);

    if (dma->v_sec_addr)
        dma_free_writecombine(0, dma->buf_sec_size, dma->v_sec_addr, dma->dma_sec_addr);

    return 0;
}

#if defined(USE_STATIC_DMA_BUFFER)
static void tsif_hwdmx_dma_buffer_init(void)
{
    int  i;

    for (i = 0; i < HWDMX_NUM; i++)
    {
        tsif_ex_pri.static_dma_buffer[i] = kmalloc(sizeof(struct tea_dma_buf), GFP_KERNEL);
        if (tsif_ex_pri.static_dma_buffer[i])
        {
            if (tcc_hwdmx_tsif_dma_buffer_alloc(tsif_ex_pri.static_dma_buffer[i]) != 0)
            {
                printk("memory alloc fail! (%d)\n", i);
                kfree(tsif_ex_pri.static_dma_buffer[i]);
                tsif_ex_pri.static_dma_buffer[i] = NULL;
            }
        }
    }
}

static void tsif_hwdmx_dma_buffer_deinit(void)
{
    int i;
    for (i = 0; i < HWDMX_NUM; i++)
    {
        if(tsif_ex_pri.static_dma_buffer[i])
        {
            tcc_hwdmx_tsif_dma_buffer_free(tsif_ex_pri.static_dma_buffer[i]);
            kfree(tsif_ex_pri.static_dma_buffer[i]);
            tsif_ex_pri.static_dma_buffer[i] = NULL;
        }
    }
}
#endif//USE_STATIC_DMA_BUFFER

#endif//USE_DMA_BUFFER

static struct tea_dma_buf* tcc_hwdmx_tsif_alloc_buffer(int id)
{
    struct tea_dma_buf *dma_buffer = kmalloc(sizeof(struct tea_dma_buf), GFP_KERNEL);

    if (dma_buffer)
    {
        #if defined(USE_REV_MEMORY)
        if (tsif_ex_pri.mem_base)
        {
            int iSize = tsif_ex_pri.pmap_tsif.size / HWDMX_NUM;
            int iOffset = id*iSize;

            dma_buffer->buf_size = iSize - SECTION_DMA_SIZE;
            dma_buffer->v_addr = tsif_ex_pri.mem_base + iOffset + SECTION_DMA_SIZE;
            dma_buffer->dma_addr = tsif_ex_pri.pmap_tsif.base + iOffset + SECTION_DMA_SIZE;
            dma_buffer->buf_sec_size = SECTION_DMA_SIZE;
            dma_buffer->v_sec_addr = tsif_ex_pri.mem_base + iOffset;
            dma_buffer->dma_sec_addr = tsif_ex_pri.pmap_tsif.base + iOffset;
            return dma_buffer;
        }
        #endif

        #if defined(USE_DMA_BUFFER)
        if (tsif_ex_pri.static_dma_buffer[id])
        {
            dma_buffer->buf_size = tsif_ex_pri.static_dma_buffer[id]->buf_size;
            dma_buffer->v_addr = tsif_ex_pri.static_dma_buffer[id]->v_addr;
            dma_buffer->dma_addr = tsif_ex_pri.static_dma_buffer[id]->dma_addr;
            dma_buffer->buf_sec_size = tsif_ex_pri.static_dma_buffer[id]->buf_sec_size;
            dma_buffer->v_sec_addr = tsif_ex_pri.static_dma_buffer[id]->v_sec_addr;
            dma_buffer->dma_sec_addr = tsif_ex_pri.static_dma_buffer[id]->dma_sec_addr;
            return dma_buffer;
        }
        else
        {
            if (tcc_hwdmx_tsif_dma_buffer_alloc(dma_buffer) == 0)
            {
                return dma_buffer;
            }
        }
        #endif

        kfree(dma_buffer);
    }
    return NULL;
}

static void tcc_hwdmx_tsif_free_buffer(struct tcc_hwdmx_handle *demux, struct tea_dma_buf *dma_buffer)
{
    #if defined(USE_REV_MEMORY)
    if (tsif_ex_pri.mem_base)
    {
        goto EXIT_FREE_BUFFER;
    }
    #endif

    #if defined(USE_DMA_BUFFER)
    if (tsif_ex_pri.static_dma_buffer[demux->tsif_ex_handle.dmx_id] == NULL)
    {
        tcc_hwdmx_tsif_dma_buffer_free(dma_buffer);
    }
    #endif

#if defined(USE_REV_MEMORY)
EXIT_FREE_BUFFER:
#endif
    kfree(dma_buffer);
}

static int tcc_hwdmx_tsif_init(struct firmware *fw, struct tcc_hwdmx_handle *demux, int id)
{
    struct tea_dma_buf *dma_buffer;
    struct pinctrl *pinctrl;

    if (id >= HWDMX_NUM)
        return -ENOMEM;

    memset(&demux->tsif_ex_handle, 0, sizeof(tcc_tsif_handle_t));
    memcpy((void*)&demux->tsif_ex_handle.port_cfg, (void*)&tsif_ex_pri.port_cfg[id], sizeof(struct tca_tsif_port_config));

    demux->tsif_ex_handle.dmx_id = id;

    dma_buffer = tcc_hwdmx_tsif_alloc_buffer(id);
    if (dma_buffer == NULL) {
        return -ENOMEM;
    }

    printk("tcc893x_tsif[%d] : dma buffer alloc @0x%X(Phy=0x%X), size:%d\n", id, (unsigned int)dma_buffer->v_addr, (unsigned int)dma_buffer->dma_addr, dma_buffer->buf_size);
    printk("tcc893x_tsif[%d] : dma sec buffer alloc @0x%X(Phy=0x%X), size:%d\n",id, (unsigned int)dma_buffer->v_sec_addr, (unsigned int)dma_buffer->dma_sec_addr, dma_buffer->buf_sec_size);
    demux->tsif_ex_handle.dma_buffer = dma_buffer;
    if (fw && fw->size)
    {
        demux->tsif_ex_handle.fw_data = fw->data;
        demux->tsif_ex_handle.fw_size = fw->size;
    }
    else
    {
        demux->tsif_ex_handle.fw_data = NULL;
        demux->tsif_ex_handle.fw_size = 0;
    }

    demux->tsif_ex_handle.dma_mode = demux->dma_mode;
    //Check working mode
    if( demux->dma_mode == 1 && demux->use_pid_filter == 1) 
    {
        //mpeg2ts mode
        demux->tsif_ex_handle.working_mode = 1; //tsif for isdbt & dvbt
    }
    else
    {
        //not mepg2ts mode
        demux->tsif_ex_handle.working_mode = 0; //tsif for tdmb
    }

	pinctrl = pinctrl_get_select(tsif_ex_pri.dev[demux->tsif_ex_handle.dmx_id], "active");
	if(IS_ERR(pinctrl))
		printk("%s : pinctrl active error[0x%p]\n", __func__, pinctrl);

	printk("%s-id[%d]:dma_mode[%d]use_pid_filter[%d]working_mod[%d]port[%d]pinctrl[0x%p]\n",
	__func__, demux->tsif_ex_handle.dmx_id, demux->tsif_ex_handle.dma_mode, demux->use_pid_filter,demux->tsif_ex_handle.working_mode,
	demux->tsif_ex_handle.port_cfg.tsif_port, pinctrl);

    if (tca_tsif_init(&demux->tsif_ex_handle)) {
        printk("%s: tca_tsif_init error !!!!!\n", __func__);
        if(dma_buffer) {
            tcc_hwdmx_tsif_free_buffer(demux, dma_buffer);
            demux->tsif_ex_handle.dma_buffer = NULL;
        }
        tca_tsif_clean(&demux->tsif_ex_handle);
        return -EBUSY;
    }

    demux->write_buff_offset = 0;
    demux->read_buff_offset = 0;
    demux->end_buff_offset = 0;
    demux->loop_count = 0;

    tca_tsif_buffer_updated_callback(&demux->tsif_ex_handle, tcc_hwdmx_tsif_updated_callback);

    return 0;
}

static void tcc_hwdmx_tsif_deinit(struct tcc_hwdmx_handle *demux)
{
    struct tea_dma_buf *dma_buffer = demux->tsif_ex_handle.dma_buffer;
    struct pinctrl *pinctrl;

    if(demux->mem)
        vfree(demux->mem);
    demux->mem = NULL;

    demux->write_buff_offset = 0;
    demux->read_buff_offset = 0;
    demux->end_buff_offset = 0;
    demux->loop_count = 0;

	pinctrl = pinctrl_get_select(tsif_ex_pri.dev[demux->tsif_ex_handle.dmx_id], "idle");
	if(IS_ERR(pinctrl))
		printk("%s : pinctrl idle error[0x%p]\n", __func__, pinctrl);
	else
		pinctrl_put(pinctrl);

    tca_tsif_clean(&demux->tsif_ex_handle);

    if(dma_buffer) {
        tcc_hwdmx_tsif_free_buffer(demux, dma_buffer);
        demux->tsif_ex_handle.dma_buffer = NULL;
    }
}

int tcc_hwdmx_tsif_open(struct inode *inode, struct file *filp)
{
    struct tcc_hwdmx_handle *demux;
    int major_number = 0, minor_number = 0;
    int i, err;
    const struct firmware *fw = NULL;

#ifdef REQUEST_FIRMWARE
#else
		struct firmware stFirmwareIncluded;
#endif

    if (inode)
    {
        major_number = imajor(inode);
        minor_number = iminor(inode);
    }

    mutex_lock(&(tsif_ex_pri.mutex));

    if (filp)
        filp->f_op = &tcc_hwdmx_tsif_fops;

    if (major_number == 0 && minor_number < HWDMX_NUM)
    {
        i = minor_number;
        demux = &tsif_ex_pri.demux[i];
    }
    else
    {
        for (i = 0; i < HWDMX_NUM; i++)
        {
            demux = &tsif_ex_pri.demux[i];
            if (demux->state == 0)
                break;
        }
    }

    if (demux->state != 0)
    {
        mutex_unlock(&(tsif_ex_pri.mutex));
        return -EBUSY;
    }

#if 1
    if (tsif_ex_pri.open_cnt == 0 && filp->private_data == NULL && gpdev)
    {
#ifdef REQUEST_FIRMWARE
		printk("[%s:%d] request_firmware\n", __func__,__LINE__);
        err = request_firmware(&fw, fw_name, gpdev);
        if (err != 0)
        {
            mutex_unlock(&(tsif_ex_pri.mutex));
            printk("Failed to load[ID:%d, Err:%d]\n", i, err);
            return -EBUSY;
        }
		printk("[%s:%d] request_firmware done\n", __func__,__LINE__);
#else 
		stFirmwareIncluded.data = HWDemux_bin;
		stFirmwareIncluded.size = sizeof(HWDemux_bin);
		printk("[%s] HWDemux_bin[]= %p, size %d\n", __func__, stFirmwareIncluded.data, stFirmwareIncluded.size );
		fw = &stFirmwareIncluded;
#endif
        filp->private_data = (void *)fw;
    }
#endif

    demux->dma_mode = 1; //0:normal mode (not mpeg2ts mode), 1:mpeg2ts mode(length 188, start with 0x47, or other identifier(tdmb-tsif))
    demux->use_pid_filter = 1;

    err = tcc_hwdmx_tsif_init((struct firmware *)filp->private_data, demux, i);

#ifdef REQUEST_FIRMWARE
    if (fw)
    {
        release_firmware(fw);
    }
#endif

    if (err != 0)
    {
        mutex_unlock(&(tsif_ex_pri.mutex));
        return -EBUSY;
    }

    demux->state = 1;
    filp->private_data = demux;

    init_waitqueue_head(&(demux->wait_q));

    tsif_ex_pri.open_cnt++;

    mutex_unlock(&(tsif_ex_pri.mutex));

#ifdef TS_PACKET_CHK_MODE
if (tsif_ex_pri.open_cnt == 1)
{
	ts_packet_chk_info = (ts_packet_chk_info_t*)kmalloc(sizeof(ts_packet_chk_info_t), GFP_ATOMIC);
	if(ts_packet_chk_info == NULL) {
		printk("\t ts_packet_chk_info_t mem alloc err..\n");
	}
	memset(ts_packet_chk_info, 0x0, sizeof(ts_packet_chk_info_t));
}
#endif

    return 0;
}


int tcc_hwdmx_tsif_release(struct inode *inode, struct file *filp)
{
    struct tcc_hwdmx_handle *demux = filp->private_data;

   	mutex_lock(&(tsif_ex_pri.mutex));   	   
    if (tsif_ex_pri.open_cnt > 0)
    {
        if (demux->state == 1)
        {
            tsif_ex_pri.open_cnt--;

            demux->state = 0;
            tcc_hwdmx_tsif_deinit(demux);
        }
    }
    mutex_unlock(&(tsif_ex_pri.mutex));
#ifdef TS_PACKET_CHK_MODE
if (tsif_ex_pri.open_cnt == 0)
{
	ts_packet_chk_t* tmp = NULL;
	ts_packet_chk_t* tmp_ = NULL;

	if(ts_packet_chk_info != NULL) {
		if(ts_packet_chk_info->packet != NULL) {

			itv_ts_cc_debug(1);
			
			tmp = ts_packet_chk_info->packet;
			do {
				tmp_ = tmp;
				tmp = tmp->next;
				kfree(tmp_);
			} while(tmp != NULL);
		}
		kfree(ts_packet_chk_info);
		ts_packet_chk_info = NULL;
	}
}
#endif
    return 0;
}

int tcc_hwdmx_tsif_probe(struct platform_device *pdev)
{
	int id = -1;

	if(hwdmx_tsif_num==0) {
		memset(&tsif_ex_pri, 0, sizeof(struct tcc_tsif_pri_handle));
		mutex_init(&(tsif_ex_pri.mutex));
		tsif_ex_pri.drv_major_num = tsif_major_num;

		printk("[%s:%d] major number = %d\n", __func__, __LINE__, tsif_ex_pri.drv_major_num);
	}

#ifdef CONFIG_OF
	tcc_hwdmx_tsif_parse_dt(pdev->dev.of_node, &tsif_ex_pri.port_cfg[hwdmx_tsif_num]);
#endif
	pdev->id = id = tsif_ex_pri.port_cfg[hwdmx_tsif_num].tsif_id;

	gpdev = device_create(tsif_class, NULL, MKDEV(tsif_ex_pri.drv_major_num, id), NULL, TSIF_DEV_NAMES, id);

    pTsif_mode[id] = TSIF_MODE_HWDMX;
    tsif_ex_pri.dev[hwdmx_tsif_num] = &pdev->dev;
    hwdmx_tsif_num++;

	if(hwdmx_tsif_num==1) {
		#if defined(USE_REV_MEMORY)
	    if (pmap_get_info("tsif", &tsif_ex_pri.pmap_tsif))
	    {
	        if (tsif_ex_pri.pmap_tsif.size > 0)
	        {
	            tsif_ex_pri.mem_base = ioremap_nocache(tsif_ex_pri.pmap_tsif.base, tsif_ex_pri.pmap_tsif.size);
	            if (tsif_ex_pri.mem_base)
	            {
	                printk("tsif_ex : pmap(PA: 0x%08x, VA: 0x%08x, SIZE: 0x%x)\n", tsif_ex_pri.pmap_tsif.base, (unsigned int)tsif_ex_pri.mem_base, tsif_ex_pri.pmap_tsif.size);
	                return 0;
	            }
	        }
	    }
	    #endif

	    #if defined(USE_DMA_BUFFER) && defined(USE_STATIC_DMA_BUFFER)
	    tsif_hwdmx_dma_buffer_init();
	    #endif
	}

    return 0;
}

int tcc_hwdmx_tsif_remove(struct platform_device *pdev)
{
	if(hwdmx_tsif_num==1) {
	    #if defined(USE_DMA_BUFFER) && defined(USE_STATIC_DMA_BUFFER)
	    tsif_hwdmx_dma_buffer_deinit();
	    #endif

	    #if defined(USE_REV_MEMORY)
	    if (tsif_ex_pri.mem_base)
	    {
	        iounmap(tsif_ex_pri.mem_base);
	    }
	    #endif
    }

	pTsif_mode[pdev->id] = 0;
	hwdmx_tsif_num--;

	device_destroy(tsif_class, MKDEV(tsif_major_num, pdev->id));

	if(hwdmx_tsif_num==0) {
		 mutex_destroy(&(tsif_ex_pri.mutex));
	}

    pdev = NULL;
    return 0;
}

EXPORT_SYMBOL(tcc_hwdmx_tsif_fops);
EXPORT_SYMBOL(tcc_hwdmx_tsif_probe);
EXPORT_SYMBOL(tcc_hwdmx_tsif_remove);
EXPORT_SYMBOL(tcc_hwdmx_tsif_open);
EXPORT_SYMBOL(tcc_hwdmx_tsif_release);
EXPORT_SYMBOL(tcc_hwdmx_tsif_ioctl_ex);
/*
EXPORT_SYMBOL(tcc_hwdmx_tsif_read);
EXPORT_SYMBOL(tcc_hwdmx_tsif_write);
EXPORT_SYMBOL(tcc_hwdmx_tsif_poll);
*/
EXPORT_SYMBOL(tcc_hwdmx_tsif_set_external_tsdemux);
EXPORT_SYMBOL(tcc_hwdmx_tsif_buffer_flush);

