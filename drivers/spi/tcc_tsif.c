/*
 * linux/drivers/spi/tcc_tsif.c
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>

#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/atomic.h>

//#include <mach/clock.h>
//#include <mach/bsp.h>
//#include <mach/gpio.h>
#include <linux/spi/tcc_tsif.h>
#include "tca_tsif.h"
#include "tsdemux/TSDEMUX_sys.h"

#define USE_STATIC_DMA_BUFFER
//#define TS_PACKET_CHK_MODE

struct tea_dma_buf *g_static_dma;
static struct clk *tsif_clk;
char gcTempTSBuff[MPEG_PACKET_SIZE*4];

extern struct class *tsif_class;

#define	MAX_PCR_CNT 2
struct tcc_tsif_pri_handle {
    wait_queue_head_t wait_q;
    struct mutex mutex;
    int open_cnt;
	u32 tsif_port;
	u32 tsif_channel;
	u32 reg_base;
	u32 drv_major_num;
	u32 pcr_pid[MAX_PCR_CNT];
	u32 bus_num;
	u32 irq_no;
	u32 resync_count;
	u32 packet_read_count;
	const char *name;
};

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
			printk("[total:%llu / err:%d (%d sec)]\n", ts_packet_chk_info->total_cnt, ts_packet_chk_info->total_err, (ts_packet_chk_info->debug_time * DEBUG_CHK_TIME));
			
			if(mod) {
				ts_packet_chk_t* tmp = NULL;
				
				tmp = ts_packet_chk_info->packet;
				do {
					printk("\tpid:0x%04x => cnt:%llu err:%d\n", tmp->pid, tmp->cnt, tmp->err);
					tmp = tmp->next;
				} while(tmp != NULL);
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
			
			//printk("\t>>>> create[%d] : 0x%04x / %02d\n", ts_packet_chk_info->packet_cnt, tmp->pid, tmp->cc);
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
			itv_ts_cc_debug(0);

			ts_packet_chk_info->prev_time = ts_packet_chk_info->cur_time;
			ts_packet_chk_info->debug_time++;
		}
	}
}
#endif	//#ifdef TS_PACKET_CHK_MODE
static tcc_tsif_handle_t tsif_ex_handle;
static struct tcc_tsif_pri_handle tsif_ex_pri;
static char use_tsif_export_ioctl = 0;
static struct task_struct *g_check_thread=NULL;
static int giStreeamDumpMode = 0; //0:Normal mode, 1:Stream Dump mode

static ssize_t tcc_tsif_read(struct file *filp, char *buf, size_t len, loff_t *ppos);
static ssize_t tcc_tsif_write(struct file *filp, const char *buf, size_t len, loff_t *ppos);
static int tcc_tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int tcc_tsif_open(struct inode *inode, struct file *filp);
static int tcc_tsif_release(struct inode *inode, struct file *filp);
static unsigned int tcc_tsif_poll(struct file *filp, struct poll_table_struct *wait);

struct file_operations tcc_tsif_ex_fops = {
    .owner          = THIS_MODULE,
    .read           = tcc_tsif_read,
    .write          = tcc_tsif_write,
    .unlocked_ioctl = tcc_tsif_ioctl,
    .open           = tcc_tsif_open,
    .release        = tcc_tsif_release,
    .poll           = tcc_tsif_poll,
};

struct ts_demux_feed_struct
{
    int is_active; //0:don't use external demuxer, 1:use external decoder
    int index;
    int call_decoder_index;
    int (*ts_demux_decoder)(char *p1, int p1_size, char *p2, int p2_size, int devid)
};

static struct ts_demux_feed_struct ts_demux_feed_handle;
static int g_tsif_isr_called = 0, g_packet_checker_valid = 0;
static int tsif_resync(struct tcc_tsif_handle *H);
static int tsif_packet_checker(void *arg)
{
    unsigned int resync_call = 0, read_packet_th = 0;
    g_tsif_isr_called = 0;
    printk("%s : started !!!\n",__func__);
	while(!kthread_should_stop())
	{
	    if(g_packet_checker_valid == 0)
        {
            ssleep(2);
	        continue;
        }

	    if(tsif_ex_handle.match_pids_count)
        {
            ssleep(1);
            if(ts_demux_feed_handle.is_active)
                read_packet_th = ts_demux_feed_handle.call_decoder_index;
            else
                read_packet_th =  tsif_ex_pri.packet_read_count;
            //printk("%s : %d:%d:%d:%d\n",__func__,g_tsif_isr_called,tsif_ex_handle.match_pids_count, read_packet_th, resync_call); 		
            if(g_tsif_isr_called == 0)
            {
                if(tsif_ex_handle.match_pids_count && read_packet_th)
                {
                    if( resync_call >= 2 && resync_call <= 4)
                    {
                        printk("%s : no tsif data !!!\n",__func__);
                        tsif_resync(&tsif_ex_handle);   
                    }
                    
                    resync_call++;
                }
                else
                    resync_call = 0;
            }
            else
                resync_call = 0;
        }
        else
        {
            resync_call = 0;
            msleep(10);
        }
        g_tsif_isr_called = 0;
	}
    printk("%s : ended !!!\n",__func__);
	return 0;
} 


int tcc_ex_tsif_set_external_tsdemux(struct file *filp, int (*decoder)(char *p1, int p1_size, char *p2, int p2_size, int devid), int max_dec_packet)
{
	if(max_dec_packet == 0)
	{
		//turn off external decoding
		memset(&ts_demux_feed_handle, 0x0, sizeof(struct ts_demux_feed_struct));
		return 0;
	}
	if(ts_demux_feed_handle.call_decoder_index != max_dec_packet)
	{
		ts_demux_feed_handle.is_active = 1;
		ts_demux_feed_handle.ts_demux_decoder = decoder;
		ts_demux_feed_handle.index = 0;
		if(tsif_ex_handle.dma_intr_packet_cnt < max_dec_packet)
			ts_demux_feed_handle.call_decoder_index = max_dec_packet; //every max_dec_packet calling isr, call decoder
		else
			ts_demux_feed_handle.call_decoder_index = 1;
		printk("%s::%d::max_dec_packet[%d]int_packet[%d]\n", __func__, __LINE__, ts_demux_feed_handle.call_decoder_index, tsif_ex_handle.dma_intr_packet_cnt);
	}

    return 0;
}

unsigned long tcc_ex_tsif_virt_to_phys(struct file *filp, unsigned long x)
{
	return 0;
}

static char is_use_tsif_export_ioctl(void)
{
	return use_tsif_export_ioctl;
}

static ssize_t show_port(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "tsif port : %d\n", tsif_ex_pri.tsif_port);

}
static ssize_t store_port(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 port;

	port = simple_strtoul(buf, (char **)NULL, 16);
	/* valid port: 0xC, 0xD, 0xE */
	if (port > 20 ) {
		printk("tcc-tsif: invalid port! (use 0xc/d/e)\n");
		return -EINVAL;
	}

	tsif_ex_pri.tsif_port = port;
	return count;

}
static DEVICE_ATTR(tcc_port, S_IRUSR|S_IWUSR, show_port, store_port);

static int __init tsif_ex_drv_probe(struct platform_device *pdev)
{
    int ret = 0;
	int irq = -1;
    struct resource *regs = NULL;
    struct resource *port = NULL;

    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs) {
        return -ENXIO;
    }
    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        return -ENXIO;
    }
    port = platform_get_resource(pdev, IORESOURCE_IO, 0);
    if (!port) {
        return -ENXIO;
    }

	tsif_ex_pri.bus_num =  pdev->id;
	tsif_ex_pri.irq_no = irq;
	tsif_ex_pri.reg_base = regs->start;
	tsif_ex_pri.tsif_port = port->start;
	tsif_ex_pri.name = port->name;

	printk("%s:[%s][%d]\n",__func__, tsif_ex_pri.name, irq);
    tsif_clk = clk_get(NULL, tsif_ex_pri.name);

	platform_set_drvdata(pdev, &tsif_ex_handle);

	/* TODO: device_remove_file(&pdev->dev, &dev_attr_tcc_port); */
	ret = device_create_file(&pdev->dev, &dev_attr_tcc_port);

	//printk("[%s]%d: init port:%d re:%d\n", pdev->name, gTSIFCH, tsif_ex_pri.gpio_port, ret);
	return 0;
}


/*
 * NOTE: before suspend, you must close tsif.
 */
static int tsif_ex_drv_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}
static int tsif_ex_drv_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tsif_ex_platform_driver = {
	.driver = {
		.name	= "tcc-tsif_ex",
		.owner	= THIS_MODULE,
	},
	.suspend = tsif_ex_drv_suspend,
	.resume  = tsif_ex_drv_resume,
};


static void tea_free_dma_linux(struct tea_dma_buf *tdma, struct device *dev)
{
    if(g_static_dma){        
        return;
    }
    if(tdma) {
        if(tdma->v_addr != 0) {
            dma_free_writecombine(dev, tdma->buf_size, tdma->v_addr, tdma->dma_addr);

			printk("tcc-tsif_ex : dma buffer free @0x%X(Phy=0x%X), size:%d\n", (unsigned int)tdma->v_addr, (unsigned int)tdma->dma_addr, tdma->buf_size);
        }
        memset(tdma, 0, sizeof(struct tea_dma_buf));
    }
}

static int tea_alloc_dma_linux(struct tea_dma_buf *tdma, unsigned int size, struct device *dev)
{
    int ret = -1;

    if(g_static_dma){        
        tdma->buf_size = g_static_dma->buf_size;
        tdma->v_addr = g_static_dma->v_addr;
        tdma->dma_addr = g_static_dma->dma_addr;
        return 0;
    }

    if(tdma) {
        tea_free_dma_linux(tdma,dev);

        tdma->buf_size = size;
        tdma->v_addr = dma_alloc_writecombine(dev, tdma->buf_size, &tdma->dma_addr, GFP_KERNEL);

        ret = tdma->v_addr ? 0 : 1;

		printk("tcc-tsif_ex : dma buffer alloc @0x%X(Phy=0x%X), size:%d\n", (unsigned int)tdma->v_addr, (unsigned int)tdma->dma_addr, tdma->buf_size);
    }

    return ret;
} 

static int tsif_resync(struct tcc_tsif_handle *H)
{
    printk("%s\n",__func__);
    tsif_ex_handle.dma_stop(&tsif_ex_handle);
    clk_disable(tsif_clk);
    //msleep(1);
    clk_enable(tsif_clk); 
    //msleep(1);
    H->hw_init(H);
    H->tsif_set(H);
    printk("%s, pids[%d]\n",__func__, H->match_pids_count);
    if(H->match_pids_count)
    {
        unsigned int i, pids[32] = {0, };
        for(i = 0; i< H->match_pids_count; i++)
            pids[i] = H->match_pids[i];

        tca_tsif_register_pids(H, pids, H->match_pids_count);
    }
    H->q_pos = H->cur_q_pos = 0;
	H->dma_phy_rdpos 	= H->rx_dma.dma_addr;
	H->dma_phy_curpos 	= H->dma_phy_rdpos;
    H->dma_start(H);   
    return 0;
}

static void debug_dump(char *data, int size)
{
    int i;
    printk("===================");
    for(i=0; i<size; i++)
    {
        if(!(i%32))
            printk("\n");
        printk("0x%02X, ", data[i]);        
    }
    printk("\n===================\n");
}


#ifdef   SUPPORT_GDMA
static int tsif_calculate_readable_cnt(struct tcc_tsif_handle *handle)
{
	int dma_recv_size = 0;
	unsigned int dma_phy_rdpos, dma_phy_curpos;
	unsigned int uidma_rdpos_offset, uidma_curpos_offset;
	int ret = 0;
	
	if(handle){
        dma_phy_rdpos = handle->dma_phy_rdpos;
        dma_phy_curpos = handle->dma_phy_curpos;
        uidma_rdpos_offset = dma_phy_rdpos-handle->rx_dma.dma_addr;
        uidma_curpos_offset = dma_phy_curpos-handle->rx_dma.dma_addr;

		if( uidma_curpos_offset ) {
    	    if(uidma_curpos_offset >= uidma_rdpos_offset) {
			    dma_recv_size = uidma_curpos_offset - uidma_rdpos_offset;
		    } else { 
			    dma_recv_size = (handle->dma_total_size - uidma_rdpos_offset) + uidma_curpos_offset;
		    }
		}	
		if(dma_recv_size > 0){
			ret = dma_recv_size / TSIF_PACKET_SIZE;
		}
		return ret;
	}
    return 0;
}

static int tsif_get_readable_cnt(struct tcc_tsif_handle *handle)
{
    if (handle) {
        int readable_cnt = 0;
        readable_cnt = tsif_calculate_readable_cnt(handle);
        return readable_cnt;
    }
	return 0;    
}

static int tsif_check_sync_byte(struct tcc_tsif_handle *handle)
{
    if (handle) {
        static int g_prev_qpos = 0;
        int q_pos;
        int readable_cnt = 0;
        q_pos = handle->dma_phy_rdpos-handle->rx_dma.dma_addr;
        readable_cnt = tsif_calculate_readable_cnt(handle);
        //check data validation
        if(readable_cnt){
            if(q_pos < handle->dma_total_size){
                char *sync_byte = (char *)tsif_ex_handle.rx_dma.v_addr + q_pos;
                if(tsif_ex_handle.mpeg_ts == (Hw0|Hw1))
                {
                    if( *sync_byte != 0x47){                                  
                        printk("call tsif-resync, readable[%d][0x%X][0x%X] : [0x%X][0x%X]!!!!\n", readable_cnt, handle->dma_phy_rdpos, handle->dma_phy_curpos, sync_byte[0], sync_byte[1]);
#if 0                        
                        if(q_pos > 6 && q_pos < 7900)
                        {
                            char *start_ptr = (char *)tsif_ex_handle.rx_dma.v_addr;
                            int i;
                            printk("**********************");
                            //debug_dump(sync_byte - 256, 512);
                            for(i = 0; i<q_pos; i++)
                            {
                                if(!(i%32))
                                    printk("\n");
                                printk("0x%02X, ", start_ptr[i*188]);
                                if(start_ptr[i*188] != 0x47)
                                    break;
                                
                            }
                            printk("\n[%d]**********************\n", i);
                            debug_dump(&start_ptr[i*188] - 512, 1024);

                        }
#endif                        
                        tsif_resync(handle);
                        return 0;
                    }
                }
                else if(tsif_ex_handle.mpeg_ts == Hw0)
                {            
                    if(q_pos != g_prev_qpos)
                    {
                        g_prev_qpos = q_pos;
                        if((*sync_byte != 0x15) && (*(sync_byte+3) != 0x5c)){
                            /* In case of tcc351x, 0x15 is sync byte at tdmb.
                             * But all packet doen't have 0x15. but it must show
                             * within 4kbyte. max 8k/188 = 44
                             */ 
                            if(++tsif_ex_pri.resync_count >= 44){
                                printk("call tsif-resync, readable[%d][0x%X][0x%X][0x%X] : [0x%X][0x%X]!!!!\n", readable_cnt, handle->dma_phy_rdpos, handle->dma_phy_curpos, q_pos,sync_byte[0], sync_byte[1]);
                                tsif_ex_pri.resync_count = 0;
                                tsif_resync(handle);
                                return 0;
                            }
                        }
                        else
                            tsif_ex_pri.resync_count = 0;
                    }
                }
            }
            return readable_cnt;
        }
    }
    return 0;
}

static int tsif_ex_check_stc(tcc_tsif_handle_t *handle)
{
    struct tcc_tsif_pri_handle *h_pri = (struct tcc_tsif_pri_handle *)handle->private_data;
    int search_pos, search_size_bytes;
    int i;
    unsigned int dma_phy_curpos;
    int idma_curpos_offset;

    dma_phy_curpos = handle->dma_phy_curpos;
    idma_curpos_offset = dma_phy_curpos-handle->rx_dma.dma_addr;

    search_pos = idma_curpos_offset-MPEG_PACKET_SIZE*4;
    if(search_pos > 0)
    {
        search_size_bytes = MPEG_PACKET_SIZE*handle->dma_intr_packet_cnt;
        for(i=0; i< MAX_PCR_CNT; i++) {
    	    if(h_pri->pcr_pid[i] < 0x1FFF)
	    	    TSDEMUX_MakeSTC((unsigned char *)handle->rx_dma.v_addr + search_pos, search_size_bytes, h_pri->pcr_pid[i], i );
        }
    }
    return 0;
}

static irqreturn_t tsif_ex_dma_handler(int irq, void *dev_id)
{
    tcc_tsif_handle_t *handle = (tcc_tsif_handle_t *)dev_id;
    struct tcc_tsif_pri_handle *h_pri = (struct tcc_tsif_pri_handle *)handle->private_data;
    int	i, fCheckSTC;

    handle->tsif_isr(handle);
    g_tsif_isr_called = 1;
	if(ts_demux_feed_handle.is_active)
	{
	    //in case of SUPPORT_GDMA mode, only external mode is supported.
		if (h_pri->open_cnt > 0) 
		{
			if(ts_demux_feed_handle.ts_demux_decoder)
			{
				int packet_th = 1;
				for (i=0; i < MAX_PCR_CNT; i++) {
					if (h_pri->pcr_pid[i] < 0x1FFF) {
						packet_th = ts_demux_feed_handle.call_decoder_index;
						break;
					}
				}


#if 0                                
				fCheckSTC = 0;
				for (i=0; i < MAX_PCR_CNT; i++) {
					if (h_pri->pcr_pid[i] < 0x1FFF) {
						fCheckSTC = 1;
						break;
					}
				}

				if(fCheckSTC) //such as scanning, not av playing
				{
				    packet_th = ts_demux_feed_handle.call_decoder_index;
				    tsif_ex_check_stc(handle);
				}
#endif
				if(giStreeamDumpMode)
                    packet_th = 512;
	
				if(++ts_demux_feed_handle.index >= packet_th)
				{
#define     TSSIZE(X) ((X)/MPEG_PACKET_SIZE*MPEG_PACKET_SIZE)		    
					char *p1 = NULL, *p2 = NULL;
					int p1_size = 0, p2_size = 0, remained = 0;
					unsigned int dma_phy_rdpos, dma_phy_curpos;
					unsigned int uidma_rdpos_offset, uidma_curpos_offset;

                    dma_phy_rdpos = handle->dma_phy_rdpos;
                    dma_phy_curpos = handle->dma_phy_curpos;
                    uidma_rdpos_offset = dma_phy_rdpos-handle->rx_dma.dma_addr;
                    uidma_curpos_offset = dma_phy_curpos-handle->rx_dma.dma_addr;

                    //printk("%d:[%d][%d][%d]\n", __LINE__,uidma_rdpos_offset, uidma_curpos_offset, TSSIZE(uidma_curpos_offset));
					if( uidma_curpos_offset > uidma_rdpos_offset )
                    {
                        p1 = (char *)handle->rx_dma.v_addr+uidma_rdpos_offset;
                        p1_size = TSSIZE(uidma_curpos_offset-uidma_rdpos_offset);
                        if(p1_size < MPEG_PACKET_SIZE)
                            return IRQ_HANDLED;
                        dma_phy_rdpos += p1_size;
                        //printk("%d:[%X][%d]\n", __LINE__,p1, p1_size);
                    }
                    else
                    {
                        //copy remained bytes
                        p1 = (char *)handle->rx_dma.v_addr + uidma_rdpos_offset;
                        remained = handle->dma_total_size - uidma_rdpos_offset;
                        if(remained >= MPEG_PACKET_SIZE)
                        {
                            p1_size = TSSIZE(remained);
                            if(ts_demux_feed_handle.ts_demux_decoder(p1, p1_size, NULL, 0, 0) != 0)
                            {
                          	    printk("%s:error occured [%d]!!!\n", __func__, p1_size);
                          	    return IRQ_HANDLED;

                            }
                            remained = remained - p1_size;
                            p1 = (char *)handle->rx_dma.v_addr + uidma_rdpos_offset + p1_size;
                            //dma_phy_rdpos = dma_phy_curpos;
                            if(remained == 0)
                            {                            
	    				        handle->dma_phy_rdpos = handle->rx_dma.dma_addr; //Start from beginning
                            }
                            else
                                handle->dma_phy_rdpos += p1_size;
                        }

                        if(uidma_curpos_offset<MPEG_PACKET_SIZE)
                            return IRQ_HANDLED;

                        memcpy(&gcTempTSBuff[0], p1, remained);
                        memcpy(&gcTempTSBuff[remained], (char *)handle->rx_dma.v_addr, MPEG_PACKET_SIZE-remained);
                        p1 = &gcTempTSBuff[0];
                        p1_size = MPEG_PACKET_SIZE;

                        uidma_rdpos_offset = MPEG_PACKET_SIZE-remained;
                        dma_phy_rdpos = handle->rx_dma.dma_addr+uidma_rdpos_offset;
                        p2 = NULL;
                        p2_size = 0;
                        if( uidma_curpos_offset > uidma_rdpos_offset )
                        {
                            p2 = (char *)handle->rx_dma.v_addr+uidma_rdpos_offset;
                            p2_size = TSSIZE(uidma_curpos_offset-uidma_rdpos_offset);
                            dma_phy_rdpos += p2_size;                            
                            //printk("%d:[%p][%d]\n", __LINE__,p2, p2_size);                 
                        }
                    }
                    for (i=0; i < MAX_PCR_CNT; i++) {
                        if (h_pri->pcr_pid[i] < 0x1FFF) {
                            TSDEMUX_MakeSTC((unsigned char *)p1, p1_size, h_pri->pcr_pid[i], i );
                        }
                    }

                    if(ts_demux_feed_handle.ts_demux_decoder(p1, p1_size, p2, p2_size, 0) == 0)
                    {
                        handle->dma_phy_rdpos = dma_phy_rdpos;
                        ts_demux_feed_handle.index = 0;
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
            }
        }        
    }
    else
    {
        if (h_pri->open_cnt > 0)
        {

            fCheckSTC = 0;
            for (i=0; i < MAX_PCR_CNT; i++) {
                if (h_pri->pcr_pid[i] < 0x1FFF) {
                    fCheckSTC = 1;
                    break;
                }
            }
            //Check PCR & Make STC
            if(fCheckSTC)
            {	
                tsif_ex_check_stc(handle);
            }
            //check read count & wake_up wait_q
            if( tsif_calculate_readable_cnt(handle) >= tsif_ex_pri.packet_read_count)
                wake_up(&(h_pri->wait_q));
        }
    }
    return IRQ_HANDLED;
}

static ssize_t tcc_tsif_read(struct file *filp, char *buf, size_t len, loff_t *ppos)
{
	int readable_cnt;

	readable_cnt = tsif_check_sync_byte(&tsif_ex_handle); 
	if ( len < (readable_cnt * TSIF_PACKET_SIZE) ) {
		int copy_byte = 0;
		int copy_cnt = 0;
        if(tsif_ex_pri.packet_read_count != len/TSIF_PACKET_SIZE)
           printk("packet_read_count = %d !!!\n",len/TSIF_PACKET_SIZE); 
        tsif_ex_pri.packet_read_count = len/TSIF_PACKET_SIZE;

		copy_cnt = len / TSIF_PACKET_SIZE;
		copy_byte = copy_cnt * TSIF_PACKET_SIZE;
		
		if (copy_cnt <= readable_cnt) {		
			int dma_phy_rdpos;
			int dma_phy_curpos;
			int offset;

			dma_phy_rdpos = tsif_ex_handle.dma_phy_rdpos;
    	    dma_phy_curpos = tsif_ex_handle.dma_phy_curpos;

			if((dma_phy_rdpos + copy_byte) < tsif_ex_handle.dma_phy_maxaddr) {
				offset = dma_phy_rdpos - tsif_ex_handle.rx_dma.dma_addr;
				
				if(copy_to_user(buf, (tsif_ex_handle.rx_dma.v_addr + offset), copy_byte)) {
					return -EFAULT;
				}

        	    tsif_ex_handle.dma_phy_rdpos = dma_phy_rdpos + copy_byte;
			} else {
				int first_size;
				int second_size;
			
				offset = dma_phy_rdpos - tsif_ex_handle.rx_dma.dma_addr;
			
				first_size 	= tsif_ex_handle.dma_phy_maxaddr - dma_phy_rdpos;
				if(copy_to_user(buf, (tsif_ex_handle.rx_dma.v_addr + offset), first_size)) {
					return -EFAULT;
				}
			
				second_size	= copy_byte - first_size;
				if(copy_to_user((buf + first_size), tsif_ex_handle.rx_dma.v_addr, second_size)) {
					return -EFAULT;
				}
	
    	        tsif_ex_handle.dma_phy_rdpos = tsif_ex_handle.rx_dma.dma_addr + second_size;
			}
			return copy_byte;
		}
	}

	return 0;
}
#else
static int tsif_calculate_readable_cnt(struct tcc_tsif_handle *H)
{
     if (H) {
        int dma_pos = H->cur_q_pos;
        int q_pos = H->q_pos;
        int readable_cnt = 0;
        if (dma_pos > q_pos) {
            readable_cnt = dma_pos - q_pos;
        } else if (dma_pos < q_pos) {
            readable_cnt = H->dma_total_packet_cnt - q_pos;
            readable_cnt += dma_pos;
        } 
        return readable_cnt;
     }
     return 0;
}

static int tsif_get_readable_cnt(struct tcc_tsif_handle *H)
{
    if (H) {
        int q_pos = H->q_pos;
        int readable_cnt = 0;
        readable_cnt = tsif_calculate_readable_cnt(H);
        //check data validation
        if(readable_cnt){
            if(q_pos < H->dma_total_packet_cnt){
                char *sync_byte = (char *)tsif_ex_handle.rx_dma.v_addr + q_pos * TSIF_PACKET_SIZE;
                if(tsif_ex_handle.mpeg_ts == (Hw0|Hw1))
                {
                    if( *sync_byte != 0x47){                                  
                        printk("call tsif-resync, readable[%d][%d][%d] : [0x%X][0x%X]!!!!\n", readable_cnt, q_pos, H->cur_q_pos, sync_byte[0], sync_byte[1]);
#if 0                        
                        if(q_pos > 6 && q_pos < 7900)
                        {
                            char *start_ptr = (char *)tsif_ex_handle.rx_dma.v_addr;
                            int i;
                            printk("**********************");
                            //debug_dump(sync_byte - 256, 512);
                            for(i = 0; i<q_pos; i++)
                            {
                                if(!(i%32))
                                    printk("\n");
                                printk("0x%02X, ", start_ptr[i*188]);
                                if(start_ptr[i*188] != 0x47)
                                    break;
                                
                            }
                            printk("\n[%d]**********************\n", i);
                            debug_dump(&start_ptr[i*188] - 512, 1024);

                        }
#endif                        
                        tsif_resync(H);
                        return 0;
                    }
                }
                else if(tsif_ex_handle.mpeg_ts == Hw0)
                {             
                    if( (*sync_byte != 0x15) && (*(sync_byte+3) != 0x5c) ){
                        /* In case of tcc351x, 0x15 is sync byte at tdmb.
                         * But all packet doen't have 0x15. but it must show
                         * within 4kbyte. max 8k/188 = 44
                         */ 
                        if(++tsif_ex_pri.resync_count >= 44){
                            printk("call tsif-resync, readable[%d] !!!!\n", readable_cnt);
                            tsif_ex_pri.resync_count = 0;
                            tsif_resync(H);
                            return 0;
                        }
                    }
                    else
                        tsif_ex_pri.resync_count = 0;
                }
            }
        }
        return readable_cnt;
    }
	return 0;
}

static int tsif_ex_check_stc(tcc_tsif_handle_t *handle)
{
    struct tcc_tsif_pri_handle *h_pri = (struct tcc_tsif_pri_handle *)handle->private_data;
    int start_pos, search_pos, search_size;
    int i;

    start_pos = handle->cur_q_pos - handle->dma_intr_packet_cnt;
    if(start_pos > 0 )
    {
        search_pos = start_pos;
        search_size = handle->dma_intr_packet_cnt;
    }	
    else
    {
        search_pos = 0;
        search_size = handle->cur_q_pos;
    }	
    for(i=0; i< MAX_PCR_CNT; i++) {
	    if(h_pri->pcr_pid[i] < 0x1FFF)
		TSDEMUX_MakeSTC((unsigned char *)handle->rx_dma.v_addr + search_pos*TSIF_PACKET_SIZE, search_size*TSIF_PACKET_SIZE, h_pri->pcr_pid[i], i );
    }
    return 0;
}

static irqreturn_t tsif_ex_dma_handler(int irq, void *dev_id)
{
    tcc_tsif_handle_t *handle = (tcc_tsif_handle_t *)dev_id;
    struct tcc_tsif_pri_handle *h_pri = (struct tcc_tsif_pri_handle *)handle->private_data;
    int	i, fCheckSTC;

    handle->tsif_isr(handle);

    g_tsif_isr_called = 1;
	if(ts_demux_feed_handle.is_active)
	{
		if (h_pri->open_cnt > 0) 
		{
			if(ts_demux_feed_handle.ts_demux_decoder)
			{
				int packet_th = 1;
				if( handle->cur_q_pos == handle->q_pos )
                {
					return IRQ_HANDLED;
                }

				fCheckSTC = 0;
				for (i=0; i < MAX_PCR_CNT; i++) {
					if (h_pri->pcr_pid[i] < 0x1FFF) {
						fCheckSTC = 1;
						break;
					}
				}
				if(fCheckSTC) //such as scanning, not av playing
				{
				    packet_th = ts_demux_feed_handle.call_decoder_index;
				    tsif_ex_check_stc(handle);
				}
				if(giStreeamDumpMode)
                    packet_th = 512;
	
				if(++ts_demux_feed_handle.index >= packet_th)
				{
					char *p1 = NULL, *p2 = NULL;
					int p1_size = 0, p2_size = 0;	

					if( handle->cur_q_pos > handle->q_pos )
                    {
                        p1 = (char *)handle->rx_dma.v_addr + handle->q_pos*TSIF_PACKET_SIZE;
                        p1_size = (handle->cur_q_pos - handle->q_pos)*TSIF_PACKET_SIZE;
                    }
                    else
                    {
                        p1 = (char *)handle->rx_dma.v_addr + handle->q_pos*TSIF_PACKET_SIZE;
                        p1_size = (handle->dma_total_packet_cnt - handle->q_pos)*TSIF_PACKET_SIZE;

                        p2 = (char *)handle->rx_dma.v_addr;
                        p2_size = handle->cur_q_pos*TSIF_PACKET_SIZE;
                    }
					if(ts_demux_feed_handle.ts_demux_decoder(p1, p1_size, p2, p2_size, 0) == 0)
					{
					    handle->q_pos = handle->cur_q_pos;
						ts_demux_feed_handle.index = 0;
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
			}
		}
        return IRQ_HANDLED;
	}

  	if (h_pri->open_cnt > 0)
   	{

		fCheckSTC = 0;
		for (i=0; i < MAX_PCR_CNT; i++) {
			if (h_pri->pcr_pid[i] < 0x1FFF) {
				fCheckSTC = 1;
				break;
			}
		}
		//Check PCR & Make STC
		if(fCheckSTC)
		{	
			tsif_ex_check_stc(handle);
		}
		//check read count & wake_up wait_q
		if( tsif_calculate_readable_cnt(handle) >= tsif_ex_pri.packet_read_count)
			wake_up(&(h_pri->wait_q));
	}
	return IRQ_HANDLED;
}

static ssize_t tcc_tsif_read(struct file *filp, char *buf, size_t len, loff_t *ppos)
{
    int readable_cnt = 0, copy_cnt = 0;
    int copy_byte = 0;
    readable_cnt = tsif_get_readable_cnt(&tsif_ex_handle);
    if (readable_cnt > 0) {

        if(tsif_ex_pri.packet_read_count != len/TSIF_PACKET_SIZE)
           printk("packet_read_count = %d !!!\n",len/TSIF_PACKET_SIZE); 
        tsif_ex_pri.packet_read_count = len/TSIF_PACKET_SIZE;

        copy_byte = readable_cnt * TSIF_PACKET_SIZE;
        if (copy_byte > len) {
            copy_byte = len;
        }

        copy_byte -= copy_byte % TSIF_PACKET_SIZE;
        copy_cnt = copy_byte / TSIF_PACKET_SIZE;
        copy_cnt -= copy_cnt % tsif_ex_handle.dma_intr_packet_cnt;
        copy_byte = copy_cnt * TSIF_PACKET_SIZE;

        if (copy_cnt >= tsif_ex_handle.dma_intr_packet_cnt) {
            int offset = tsif_ex_handle.q_pos * TSIF_PACKET_SIZE;
            if (copy_cnt > tsif_ex_handle.dma_total_packet_cnt - tsif_ex_handle.q_pos) {
                int first_copy_byte = (tsif_ex_handle.dma_total_packet_cnt - tsif_ex_handle.q_pos) * TSIF_PACKET_SIZE;
                int first_copy_cnt = first_copy_byte / TSIF_PACKET_SIZE;
                int second_copy_byte = (copy_cnt - first_copy_cnt) * TSIF_PACKET_SIZE;

                if (copy_to_user(buf, tsif_ex_handle.rx_dma.v_addr + offset, first_copy_byte)) {
                    return -EFAULT;
                }
                if (copy_to_user(buf + first_copy_byte, tsif_ex_handle.rx_dma.v_addr, second_copy_byte)) {
                    return -EFAULT;
                }

                tsif_ex_handle.q_pos = copy_cnt - first_copy_cnt;
            } else {
                if (copy_to_user(buf, tsif_ex_handle.rx_dma.v_addr + offset, copy_byte)) {
                    return -EFAULT;
                }

                tsif_ex_handle.q_pos += copy_cnt;
                if (tsif_ex_handle.q_pos >= tsif_ex_handle.dma_total_packet_cnt) {
                    tsif_ex_handle.q_pos = 0;
                }
            }
            return copy_byte;
        }
    }
    return 0;
}
#endif

static unsigned int tcc_tsif_poll(struct file *filp, struct poll_table_struct *wait)
{
    if (tsif_get_readable_cnt(&tsif_ex_handle) >= tsif_ex_pri.packet_read_count) {
		return  (POLLIN | POLLRDNORM);
    }

    poll_wait(filp, &(tsif_ex_pri.wait_q), wait);
    if (tsif_get_readable_cnt(&tsif_ex_handle) >= tsif_ex_pri.packet_read_count) {
		return  (POLLIN | POLLRDNORM);
    }
    return 0;
}

static ssize_t tcc_tsif_write(struct file *filp, const char *buf, size_t len, loff_t *ppos)
{
    return 0;
}

static ssize_t tcc_tsif_copy_from_user(void *dest, void *src, size_t copy_size)
{
	int ret = 0;
	if(is_use_tsif_export_ioctl() == 1) {
		memcpy(dest, src, copy_size);
	} else {
		ret = copy_from_user(dest, src, copy_size);
	}
	return ret;
}

static ssize_t tcc_tsif_copy_to_user(void *dest, void *src, size_t copy_size)
{
	int ret = 0;
	if(is_use_tsif_export_ioctl() == 1) {
		memcpy(dest, src, copy_size);
	} else {
		ret = copy_to_user(dest, src, copy_size);
	}
	return ret;
}

static int tcc_tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	//printk("%s[0x%X] - in\n", __func__, cmd);
    switch (cmd) {
    case IOCTL_TSIF_DMA_START:
    case IOCTL_TSIF_HWDMX_START:
        {
            struct tcc_tsif_param param;
            int clk_polarity,  valid_polarity, sync_polarity, msb_first, mpeg_ts;
            if (tcc_tsif_copy_from_user(&param, (void *)arg, sizeof(struct tcc_tsif_param))) {
                printk("cannot copy from user tcc_tsif_param in IOCTL_TSIF_DMA_START !!! \n");
                return -EFAULT;
            }

            if (((TSIF_PACKET_SIZE * param.ts_total_packet_cnt) > tsif_ex_handle.dma_total_size)
                || (param.ts_total_packet_cnt <= 0)) {
                printk("so big ts_total_packet_cnt !!! \n");
                param.ts_total_packet_cnt = tsif_ex_handle.dma_total_size /(TSIF_PACKET_SIZE*param.ts_intr_packet_cnt);
            }
            if(param.ts_total_packet_cnt > 0x1fff) //Max packet is 0x1fff(13bit)
                param.ts_total_packet_cnt = 0x1fff;

            if(tsif_ex_handle.dma_stop(&tsif_ex_handle) == 0)
                tsif_ex_handle.clear_fifo_packet(&tsif_ex_handle);

            if(param.dma_mode == 1)
                tsif_ex_handle.mpeg_ts |= Hw0;
            else
                tsif_ex_handle.mpeg_ts = 0;

            if(param.mode & SPI_CPOL)
               clk_polarity = 0x01;    //1:falling edge 0:rising edge
            else
               clk_polarity = 0x00;    //1:falling edge 0:rising edge

            sync_polarity = 0x00;   //0:sync high active 1:sync low active
            if(param.mode & SPI_CS_HIGH)
            {
                valid_polarity = 0x01;  //1:valid high active 0:valid low active
                if(tsif_ex_handle.mpeg_ts == 0)
                    sync_polarity = 0x00;   //0:sync high active 1:sync low active
            }
            else
            {
                valid_polarity = 0x00;  //1:valid high active 0:valid low active
                if(tsif_ex_handle.mpeg_ts == 0)
                    sync_polarity = 0x01;   //0:sync high active 1:sync low active
            }

            if(param.mode & SPI_LSB_FIRST)
           	    msb_first = 0x00;       //1:msb first, 0:lsb first
            else
               	msb_first = 0x01;       //1:msb first, 0:lsb first

            if( tsif_ex_handle.clk_polarity != clk_polarity ||\
                tsif_ex_handle.valid_polarity != valid_polarity ||\
                tsif_ex_handle.sync_polarity != sync_polarity ||\
                tsif_ex_handle.msb_first != msb_first)
            {
                tsif_ex_handle.msb_first = msb_first;       //1:msb first, 0:lsb first
                tsif_ex_handle.clk_polarity = clk_polarity;    //1:falling edge 0:rising edge
                tsif_ex_handle.valid_polarity = valid_polarity;  //1:valid high active 0:valid low active
                tsif_ex_handle.sync_polarity = sync_polarity;   //0:sync high active 1:sync low active
                tsif_ex_handle.tsif_set(&tsif_ex_handle);
            }

            tsif_ex_handle.dma_total_packet_cnt = param.ts_total_packet_cnt;
            tsif_ex_handle.dma_intr_packet_cnt = param.ts_intr_packet_cnt;
            tsif_ex_pri.packet_read_count = tsif_ex_handle.dma_intr_packet_cnt;
            tsif_ex_handle.q_pos = tsif_ex_handle.cur_q_pos = 0;
            printk("interrupt packet count [%u]\n", tsif_ex_handle.dma_intr_packet_cnt);
            tsif_ex_handle.dma_start(&tsif_ex_handle);                        
        }
        break;			
    case IOCTL_TSIF_DMA_STOP:
    case IOCTL_TSIF_HWDMX_STOP:
            if(tsif_ex_handle.dma_stop(&tsif_ex_handle) == 0)
                tsif_ex_handle.clear_fifo_packet(&tsif_ex_handle);
        break;			
    case IOCTL_TSIF_GET_MAX_DMA_SIZE:
        {
            struct tcc_tsif_param param;
            param.ts_total_packet_cnt = tsif_ex_handle.dma_total_size / TSIF_PACKET_SIZE;
            param.ts_intr_packet_cnt = 1;
            if (tcc_tsif_copy_from_user((void *)arg, (void *)&param, sizeof(struct tcc_tsif_param))) {
                printk("cannot copy to user tcc_tsif_param in IOCTL_TSIF_GET_MAX_DMA_SIZE !!! \n");
                return -EFAULT;
            }
        }
        break;        
    case IOCTL_TSIF_SET_PID:
        {
            struct tcc_tsif_pid_param param;
            if (tcc_tsif_copy_from_user(&param, (void *)arg, sizeof(struct tcc_tsif_pid_param))) {
                printk("cannot copy from user tcc_tsif_pid_param in IOCTL_TSIF_SET_PID !!! \n");
                return -EFAULT;
            }
            ret = tca_tsif_register_pids(&tsif_ex_handle, param.pid_data, param.valid_data_cnt);
            if(param.valid_data_cnt == 1 && param.pid_data[0] == 0x2000)
            {
               //0x2000 is stream dump pid
               giStreeamDumpMode = 1; //Stream dump mode
               printk("[tsif]Stream dump mode !!!\n");
            }
            else
               giStreeamDumpMode = 0; //Normal mode
        } 
        break;		    
    case IOCTL_TSIF_DXB_POWER:
        break;

    case IOCTL_TSIF_SET_PCRPID:
	{
		struct tcc_tsif_pcr_param pcr_param;

		if (tcc_tsif_copy_from_user((void *)&pcr_param, (void*)arg, sizeof(struct tcc_tsif_pcr_param)))
			return -EFAULT;
		if (pcr_param.index >= MAX_PCR_CNT ) return -EFAULT;
		tsif_ex_pri.pcr_pid[pcr_param.index] = pcr_param.pcr_pid;
		printk("Set PCR PID[0x%X]\n", tsif_ex_pri.pcr_pid[pcr_param.index]);
		if( tsif_ex_pri.pcr_pid[pcr_param.index] < 0x1FFF)
			TSDEMUX_Open(pcr_param.index);
        }		
        break;
    case IOCTL_TSIF_GET_STC:
        {
            unsigned int uiSTC, index;

	    if (tcc_tsif_copy_from_user((void*)&index, (void*)arg, sizeof(int)))
		    return - EFAULT;
	    if (index >= MAX_PCR_CNT) return -EFAULT;
            uiSTC = TSDEMUX_GetSTC(index);
            //printk("STC %d\n", uiSTC);
            if (tcc_tsif_copy_to_user((void *)arg, (void *)&uiSTC, sizeof(int))) {
                printk("cannot copy to user tcc_tsif_param in IOCTL_TSIF_GET_PCR !!! \n");
                return -EFAULT;
            }
        }
        break;
    case IOCTL_TSIF_RESET:
        tsif_resync(&tsif_ex_handle);
        break;
    default:
        printk("tcc-tsif : unrecognized ioctl (0x%X)\n", cmd);
        ret = -EINVAL;
        break;
    }
	//printk("%s[0x%X] - out\n", __func__, cmd);
    return ret;
}

static int tcc_tsif_init(void)
{
    int ret = 0, tsif_channel = 0;
    memset(&tsif_ex_handle, 0, sizeof(tcc_tsif_handle_t));   
    tsif_channel = 0;
   	if(!strcmp(tsif_ex_pri.name,"tsif1"))
        tsif_channel = 1;
    else if(!strcmp(tsif_ex_pri.name,"tsif2"))
        tsif_channel = 2;

    if (tca_tsif_init(&tsif_ex_handle,
                        (volatile struct tcc_tsif_regs *)tsif_ex_pri.reg_base,
                        tea_alloc_dma_linux,
                        tea_free_dma_linux,
                        TSIF_DMA_SIZE,
                        tsif_channel,
                        0,
                        0,
                        tsif_ex_pri.tsif_port)) {
        printk("%s: tca_tsif_init error !!!!!\n", __func__);
        ret = -EBUSY;
        goto err_tsif;
    }
    giStreeamDumpMode = 0; //Normal mode
    tsif_ex_handle.serial_mode = 0x01;     //1:serialmode 0:parallel mode
    tsif_ex_handle.private_data = (void *)&tsif_ex_pri;
    init_waitqueue_head(&(tsif_ex_pri.wait_q));

    tsif_ex_handle.dma_total_packet_cnt = tsif_ex_handle.dma_total_size / TSIF_PACKET_SIZE;
#ifdef  SUPPORT_GDMA    
    //GDMA cannot change endian, therefore we change endian in TSIF
    tsif_ex_handle.big_endian = 0x01;	     //1:big endian, 0:little endian
#else    
    tsif_ex_handle.big_endian = 0x00;	     //1:big endian, 0:little endian
#endif   
    tsif_ex_handle.dma_intr_packet_cnt = 1;
    tsif_ex_handle.hw_init(&tsif_ex_handle);
    tsif_ex_handle.clear_fifo_packet(&tsif_ex_handle);
    tsif_ex_handle.dma_stop(&tsif_ex_handle);
    tsif_ex_handle.msb_first = 0x01;       //1:msb first, 0:lsb first
    tsif_ex_handle.clk_polarity = 0x00;    //1:falling edge 0:rising edge
    tsif_ex_handle.valid_polarity = 0x01;  //1:valid high active 0:valid low active
    tsif_ex_handle.sync_polarity = 0x00;   //0:sync high active 1:sync low active
    tsif_ex_handle.sync_delay = 0x00;
	tsif_ex_handle.mpeg_ts = 0;
    tsif_ex_handle.tsif_set(&tsif_ex_handle);
	
    ret = request_irq(tsif_ex_handle.irq, tsif_ex_dma_handler, IRQF_SHARED, tsif_ex_pri.name, &tsif_ex_handle);
	if (ret) { 
		goto err_irq; 
	}
	return 0;

err_irq:
	free_irq(tsif_ex_handle.irq, &tsif_ex_handle);
	
err_tsif:
	tca_tsif_clean(&tsif_ex_handle);
	return ret;
}

static void tcc_tsif_deinit(void)
{
	free_irq(tsif_ex_handle.irq, &tsif_ex_handle);
    tca_tsif_clean(&tsif_ex_handle);
}


static int tcc_tsif_open(struct inode *inode, struct file *filp)
{
    int ret = 0;	
    int	i;

    if (tsif_ex_pri.open_cnt == 0) {
        tsif_ex_pri.open_cnt++;
    } else {
        return -EBUSY;
    }

   	if(tsif_clk == NULL)
        return -EBUSY;

   	if(filp)
   	   	filp->f_op = &tcc_tsif_ex_fops;

    clk_enable(tsif_clk);
    mutex_lock(&(tsif_ex_pri.mutex));
    for(i=0; i < MAX_PCR_CNT; i++)
   	tsif_ex_pri.pcr_pid[i] = 0xFFFF;
    tsif_ex_pri.resync_count = 0;
    tcc_tsif_init();
   	g_packet_checker_valid = 1;
    if(g_check_thread == NULL){ 
        g_check_thread = (struct task_struct *)kthread_run(tsif_packet_checker, NULL, "tsif packet checker");
    }

    mutex_unlock(&(tsif_ex_pri.mutex));
#ifdef TS_PACKET_CHK_MODE
	ts_packet_chk_info = (ts_packet_chk_info_t*)kmalloc(sizeof(ts_packet_chk_info_t), GFP_ATOMIC);
	if(ts_packet_chk_info == NULL) {
		printk("\t ts_packet_chk_info_t mem alloc err..\n");
	}
	memset(ts_packet_chk_info, 0x0, sizeof(ts_packet_chk_info_t));
#endif	 
	return ret;
}


static int tcc_tsif_release(struct inode *inode, struct file *filp)
{
    if (tsif_ex_pri.open_cnt > 0) {
        tsif_ex_pri.open_cnt--;
    }	

    if(tsif_ex_pri.open_cnt == 0)
    {
   	    mutex_lock(&(tsif_ex_pri.mutex));   	   
		g_packet_checker_valid = 0;
        tsif_ex_handle.dma_stop(&tsif_ex_handle);
        tcc_tsif_deinit();
      	TSDEMUX_Close();
        mutex_unlock(&(tsif_ex_pri.mutex));
        clk_disable(tsif_clk);
#ifdef TS_PACKET_CHK_MODE
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

    }    
    return 0;
}

extern int tsif_major_num;

int tsif_ex_init(void)
{
    int ret = 0;
    int	i;

    g_packet_checker_valid = 0;
    memset(&tsif_ex_pri, 0, sizeof(struct tcc_tsif_pri_handle));
    mutex_init(&(tsif_ex_pri.mutex));
    tsif_ex_pri.drv_major_num = tsif_major_num;
    printk("[%s:%d] major number = %d\n", __func__, __LINE__, tsif_ex_pri.drv_major_num);
    g_static_dma = NULL;
#ifdef      USE_STATIC_DMA_BUFFER
     g_static_dma = kmalloc(sizeof(struct tea_dma_buf), GFP_KERNEL);
     if(g_static_dma)
     {
        g_static_dma->buf_size = TSIF_DMA_SIZE;
        g_static_dma->v_addr = dma_alloc_writecombine(0, g_static_dma->buf_size, &g_static_dma->dma_addr, GFP_KERNEL);
        printk("tcc-tsif_ex : dma buffer alloc @0x%X(Phy=0x%X), size:%d\n", (unsigned int)g_static_dma->v_addr, (unsigned int)g_static_dma->dma_addr, g_static_dma->buf_size);
        if(g_static_dma->v_addr == NULL)
        {
            kfree(g_static_dma);
            g_static_dma = NULL;
        }
     }
#endif    

    ret = platform_driver_probe(&tsif_ex_platform_driver, tsif_ex_drv_probe);

    device_create(tsif_class, NULL, MKDEV(tsif_ex_pri.drv_major_num, 0), NULL, TSIF_DEV_NAMES, 0);

    for(i=0; i < MAX_PCR_CNT; i++)
        tsif_ex_pri.pcr_pid[i] = 0xFFFF;
    return 0;
}

void tsif_ex_exit(void)
{
    if(g_check_thread)
    {	
        g_packet_checker_valid = 0;
        kthread_stop(g_check_thread);
        g_check_thread = NULL; 		
    }

    platform_driver_unregister(&tsif_ex_platform_driver);
	if(tsif_clk)
	{
	    clk_disable(tsif_clk);
    	clk_put(tsif_clk);
	}
	if(g_static_dma)
    {
        dma_free_writecombine(0, g_static_dma->buf_size, g_static_dma->v_addr, g_static_dma->dma_addr);
        kfree(g_static_dma);
        g_static_dma = NULL;
    }
}

int tcc_ex_tsif_open(struct inode *inode, struct file *filp)
{
	return tcc_tsif_open(inode, filp);
}

int tcc_ex_tsif_release(struct inode *inode, struct file *filp)
{
	return tcc_tsif_release(inode, filp);
}

int tcc_ex_tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	use_tsif_export_ioctl = 1;
	ret = tcc_tsif_ioctl(filp, cmd, arg);
	use_tsif_export_ioctl = 0;

	return ret;
}

EXPORT_SYMBOL(tsif_ex_init);
EXPORT_SYMBOL(tsif_ex_exit);
EXPORT_SYMBOL(tcc_ex_tsif_open);
EXPORT_SYMBOL(tcc_ex_tsif_release);
EXPORT_SYMBOL(tcc_ex_tsif_ioctl);
EXPORT_SYMBOL(tcc_ex_tsif_set_external_tsdemux);
EXPORT_SYMBOL(tcc_ex_tsif_virt_to_phys);