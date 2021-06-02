/* 
 * arch/arm/mach-tcc893x/tca_hwdemux_tsif.c
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2008 
 * Description: Telechips Linux H/W Demux Driver
 *
 * Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/bsp.h>
#include <mach/io.h>
#include <mach/tca_hwdemux_tsif.h>
#include <mach/TCC893xHWDemux_cmd.h>
#include <mach/tca_hwdemux_cipher_cmd.h>

#define DEVICE_NAME	"mailbox"

#define MBOX_IRQ    INT_CMBUS_MBOX0

//#define   SUPPORT_INTERNAL_TEST_MODE
//#define SUPPORT_DEBUG_CMHWDEMUX
#ifdef  SUPPORT_DEBUG_CMHWDEMUX
#include "DebugHWDemux.h" //testing f/w of holding at main()
#define CMD_TIMEOUT        msecs_to_jiffies(1000000)
#else
#define CMD_TIMEOUT        msecs_to_jiffies(1000)
#endif

extern unsigned int tcc_chip_rev;
#ifdef CONFIG_CLOCK_TABLE
#include <mach/clocktable.h>
struct func_clktbl_t	*tsif_clktbl = NULL;
#endif
static struct clk *cm_clk = NULL;

enum {
    QUEUE_HW_DEMUX_INIT = 0,
    QUEUE_HW_DEMUX_DEINIT,
    QUEUE_HW_DEMUX_ADD_FILTER,
    QUEUE_HW_DEMUX_DELETE_FILTER,
    QUEUE_HW_DEMUX_GET_STC,
    QUEUE_HW_DEMUX_SET_PCR_PID,
    QUEUE_HW_DEMUX_GET_VERSION,
    QUEUE_HW_DEMUX_INTERNAL_SET_INPUT,
    QUEUE_HW_DEMUX_CHANGE_INTERFACE,
    QUEUE_HW_DEMUX_CIPHER_SET_ALGORITHM,
    QUEUE_HW_DEMUX_CIPHER_SET_KEY,
    QUEUE_HW_DEMUX_CIPHER_SET_IV,
    QUEUE_HW_DEMUX_CIPHER_EXECUTE,
    QUEUE_HW_DEMUX_TYPE_MAX,
};

wait_queue_head_t cmd_queue[4][QUEUE_HW_DEMUX_TYPE_MAX];
static int g_ret = 0, g_ret_done=0;

static FN_UPDATECALLBACK ts_demux_buffer_updated_callback[4];
static FN_NSKCALLBACK nskMsgHandler = NULL;

static struct clk *pktgen_clk[4];

static int g_interface = 0; // 0: serial tsif, 1: parallel tsif, 2: internal

static int g_initCount = 0;
static struct mutex tsdmx_mutex; 
static int tsdmx_lock_init(void)
{
    mutex_init(&tsdmx_mutex);
    return 0;
}

int tsdmx_lock_on(void)
{
    mutex_lock(&tsdmx_mutex);
    return 0;
}
EXPORT_SYMBOL(tsdmx_lock_on);

int tsdmx_lock_off(void)
{
    mutex_unlock(&tsdmx_mutex);
    return 0;
}
EXPORT_SYMBOL(tsdmx_lock_off);

int TSDMX_Init(struct tcc_tsif_handle *h)
{
    int ret = 0;
    ARG_TSDMXINIT Arg;

    printk("\n%s:%d:0x%08X:0x%08X:0x%08X\n",__func__, __LINE__, h->dma_buffer->dma_addr, (unsigned int)h->dma_buffer->v_addr, h->dma_buffer->buf_size);
    Arg.uiDMXID  = h->dmx_id;

    Arg.uiMode = HW_DEMUX_NORMAL; //HW_DEMUX_BYPASS
    
    Arg.uiTSRingBufAddr = h->dma_buffer->dma_addr;
    Arg.uiTSRingBufSize = h->dma_buffer->buf_size;
    Arg.uiSECRingBufAddr = h->dma_buffer->dma_sec_addr;
    Arg.uiSECRingBufSize = h->dma_buffer->buf_sec_size;
    Arg.uiChipID = 0; //Default BX
    Arg.uiChipID = 1;
#if 0
    Arg.uiChipID = 1; //Default BX
    if(tcc_chip_rev == 0xA)
        Arg.uiChipID = 0; //AX
    printk("ChipID[%d]AX:0,BX:1\n", Arg.uiChipID);
#endif    
    switch(h->working_mode)
    {
        case 0x00: //tsif for tdmb
            Arg.uiTSIFInterface = HW_DEMUX_SPI;
            break;
        case 0x02:
            Arg.uiTSIFInterface = HW_DEMUX_INTERNAL;
            break;     

        default: //0x01 for tsif of isdbt & dvbt
            if(h->serial_mode == 1)
                Arg.uiTSIFInterface = HW_DEMUX_TSIF_SERIAL;
            else
                Arg.uiTSIFInterface = HW_DEMUX_TSIF_PARALLEL;
            break;
    }
#ifdef  SUPPORT_INTERNAL_TEST_MODE
    Arg.uiTSIFInterface = HW_DEMUX_INTERNAL;
#endif
    if(g_interface==2)
        Arg.uiTSIFInterface = HW_DEMUX_INTERNAL;
    
    tsdmx_lock_on();
    Arg.uiTSIFCh = h->dmx_id;
    Arg.uiTSIFPort = h->port_cfg.tsif_port;

    //TSIf Polarity : TSP-Sync Pulse(Bit0) : 0(hight active), TVEP-Valid Data(Bit1) : 1(high active), TCKP(Bit2) : 0(riging edge of TSCLK)
    Arg.uiTSIFPol = Hw1;    

    //DEMOD_MODULE_LGDT3305
    //Arg.uiTSIFPol |= Hw2;	//TCKP(Bit2) : 1(falling edge of TSCLK)

    g_ret_done = 0;
    TSDMXCMD_Init(&Arg);
    if((ret = wait_event_timeout((cmd_queue[Arg.uiDMXID][QUEUE_HW_DEMUX_INIT]),g_ret_done==1,CMD_TIMEOUT)) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    if(ret == 0)
        g_initCount++;
    tsdmx_lock_off();
    return ret;
}


int TSDMX_DeInit(unsigned int uiDMXID)
{
    int ret = 0;

    if (g_initCount == 0)
        return ret;

    tsdmx_lock_on();
    g_initCount--;
    g_ret_done = 0;
    TSDMXCMD_DeInit(uiDMXID);
    if((ret = wait_event_timeout((cmd_queue[uiDMXID][QUEUE_HW_DEMUX_DEINIT]),g_ret_done==1,CMD_TIMEOUT)) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}

int TSDMX_ADD_Filter(unsigned int uiDMXID, struct tcc_tsif_filter *feed)
{
    int ret = 0;
    ARG_TSDMX_ADD_FILTER Arg;
    tsdmx_lock_on();
    Arg.uiDMXID = uiDMXID;
    Arg.uiTYPE = feed->f_type;
    Arg.uiPID = feed->f_pid;
    if(feed->f_type == HW_DEMUX_SECTION) {
        printk("pid : %d, size = %d\n", feed->f_pid, feed->f_size);
        if(feed->f_size>16) {
            printk("!!! filter size is over 16 then it sets to 16.\n");
            feed->f_size = 16; //HWDMX can support less than 16 bytes filter size.
        }
#if 0        
        {
            int i;
            for(i=0; i<feed->f_size; i++)
                printk("[%d]C[0x%X]M[0x%X]M[0x%X]\n",i, feed->f_comp[i],feed->f_mask[i],feed->f_mode[i]);
        }
#endif        
        Arg.uiFSIZE = feed->f_size;
        Arg.uiFID = feed->f_id;
        if(feed->f_size*3 <= 24) {
            Arg.uiTotalIndex = 1;
        } else {
            Arg.uiTotalIndex = 2; //max filter size 16*3=48, 2 add filter commands is enough to send it.
        }
        memset(Arg.uiVectorData, 0x00, 20*4);
        memcpy(((unsigned char*)Arg.uiVectorData), feed->f_comp, feed->f_size);
        memcpy(((unsigned char*)Arg.uiVectorData) + (feed->f_size), feed->f_mask, feed->f_size);
        memcpy(((unsigned char*)Arg.uiVectorData) + (feed->f_size*2), feed->f_mode, feed->f_size);
        Arg.uiVectorIndex = 0;
        for(Arg.uiCurrentIndex=0; Arg.uiCurrentIndex<Arg.uiTotalIndex; Arg.uiCurrentIndex ++) {
            g_ret_done = 0;
            TSDMXCMD_ADD_Filter(&Arg);
            if(wait_event_timeout((cmd_queue[Arg.uiDMXID][QUEUE_HW_DEMUX_ADD_FILTER]),g_ret_done==1,CMD_TIMEOUT) != 0)
                ret = g_ret;
            else
            {
                if(g_ret_done==0)
                {
                    printk("%s:time-out\n", __func__);
                    ret =  -2; //time out
                }
            }
            if(ret != 0)
                break;
        }
    } else {
        g_ret_done = 0;
        TSDMXCMD_ADD_Filter(&Arg);
        if(wait_event_timeout((cmd_queue[Arg.uiDMXID][QUEUE_HW_DEMUX_ADD_FILTER]),g_ret_done==1,CMD_TIMEOUT) != 0)
            ret = g_ret;
        else
        {
            if(g_ret_done==0)
            {
                printk("%s:time-out\n", __func__);
                ret =  -2; //time out
            }
        }
    }
    tsdmx_lock_off();
    return ret;
}

int TSDMX_DELETE_Filter(unsigned int uiDMXID, struct tcc_tsif_filter *feed)
{
    int ret = 0;

    ARG_TSDMX_DELETE_FILTER Arg;
    tsdmx_lock_on();
    Arg.uiDMXID = uiDMXID;
    Arg.uiTYPE = feed->f_type;
    if(feed->f_type == HW_DEMUX_SECTION)
        Arg.uiFID = feed->f_id;
    else
        Arg.uiFID = 0;
    Arg.uiPID = feed->f_pid;

    g_ret_done = 0;
    TSDMXCMD_DELETE_Filter(&Arg);
    if(wait_event_timeout((cmd_queue[Arg.uiDMXID][QUEUE_HW_DEMUX_DELETE_FILTER]),g_ret_done==1,CMD_TIMEOUT) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}

long long TSDMX_GET_STC(unsigned int uiDMXID)
{
    int ret;
    tsdmx_lock_on();
    ret = TSDMXCMD_GET_STC(uiDMXID);
    tsdmx_lock_off();
    return ret;
}

int TSDMX_SET_PCR_PID(unsigned int uiDMXID, unsigned int pid)
{
    int ret = 0;

    ARG_TSDMX_SET_PCR_PID Arg;
    tsdmx_lock_on();
    Arg.uiDMXID = uiDMXID;
    Arg.uiPCRPID = pid;
    g_ret_done = 0;
    TSDMXCMD_SET_PCR_PID(&Arg);
    if(wait_event_timeout((cmd_queue[Arg.uiDMXID][QUEUE_HW_DEMUX_SET_PCR_PID]),g_ret_done==1,CMD_TIMEOUT) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}

int TSDMX_GET_Version(unsigned int uiDMXID, unsigned int uiTimeOut)
{
    int ret = 0;
    tsdmx_lock_on();
    g_ret_done = 0;
    TSDMXCMD_GET_VERSION(uiDMXID);
    if((ret = wait_event_timeout((cmd_queue[uiDMXID][QUEUE_HW_DEMUX_GET_VERSION]),g_ret_done==1, uiTimeOut)) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}

int TSDMX_SET_InBuffer(unsigned int uiDMXID, unsigned int uiBufferAddr, unsigned int uiBufferSize)
{
    int ret = 0;
    ARG_TSDMX_SET_IN_BUFFER Arg;
	if (g_initCount == 0)
        return ret;
    tsdmx_lock_on();
    Arg.uiDMXID = uiDMXID;
    Arg.uiInBufferAddr = uiBufferAddr;
    Arg.uiInBufferSize = uiBufferSize;
    g_ret_done = 0;
    TSDMXCMD_SET_IN_BUFFER(&Arg);
    if((ret = wait_event_timeout((cmd_queue[uiDMXID][QUEUE_HW_DEMUX_INTERNAL_SET_INPUT]),g_ret_done==1,CMD_TIMEOUT)) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}

int TSDMX_Change_Interface(unsigned int uiDMXID, unsigned int uiTSIFInterface)
{
    int ret = 0;

    if (g_initCount == 0)
        return ret;

    printk("[DEMUX #%d] Change Interface\n", uiDMXID);

    tsdmx_lock_on();
    g_ret_done = 0;
    TSDMXCMD_CHANGE_INTERFACE(uiDMXID, uiTSIFInterface);
    if((ret = wait_event_timeout((cmd_queue[uiDMXID][QUEUE_HW_DEMUX_CHANGE_INTERFACE]),g_ret_done==1,CMD_TIMEOUT)) != 0)
        ret = g_ret;
    else
    {
        if(g_ret_done==0)
        {
            printk("%s:time-out\n", __func__);
            ret =  -2; //time out
        }
    }
    tsdmx_lock_off();
    return ret;
}


/*****************************************************************************
* Function Name : static void TSDMX_CMUnloadBinary(void);
* Description : CM Code Un-Loading
* Arguments :
******************************************************************************/
static void TSDMX_CMUnloadBinary(void)
{
    volatile PCM_TSD_CFG pTSDCfg = (volatile PCM_TSD_CFG)tcc_p2v(HwCORTEXM4_TSD_CFG_BASE);
    BITSET(pTSDCfg->CM_RESET.nREG, Hw1|Hw2); //m3 no reset
    printk("%s:%d\n",__func__, __LINE__);
}

/*****************************************************************************
* Function Name : static void TSDMX_CMLoadBinary(void);
* Description : CM Code Loading
* Arguments :
******************************************************************************/
static void TSDMX_CMLoadBinary(const char *fw_data, unsigned int fw_size)
{
    volatile unsigned int * pCodeMem = (volatile unsigned int *)tcc_p2v(HwCORTEXM4_CODE_MEM_BASE);
    volatile PCM_TSD_CFG pTSDCfg = (volatile PCM_TSD_CFG)tcc_p2v(HwCORTEXM4_TSD_CFG_BASE);
    /* M4 has Bit band alias regions at 0x20000000, 0x40000000.
     * We should remap those ranges because it is spectial area.
     * Therefore 
     * Remap 0x20000000 -> 0x60000000, 0x40000000 -> 0x80000000 in CM4
     */
#if 0    
    pTSDCfg->REMAP0.bREG.ICODE = 0x0;
    pTSDCfg->REMAP0.bREG.DCODE = 0x0;
    pTSDCfg->REMAP0.bREG.REMAP_2 = 0x2;
    pTSDCfg->REMAP0.bREG.REMAP_3 = 0x3;
    pTSDCfg->REMAP0.bREG.REMAP_4 = 0x4;
    pTSDCfg->REMAP0.bREG.REMAP_5 = 0x5;
    pTSDCfg->REMAP0.bREG.REMAP_6 = 0x4; // 0x6xxxxxxx -> 0x4xxxxxxx
    pTSDCfg->REMAP0.bREG.REMAP_7 = 0x7;

    pTSDCfg->REMAP1.bREG.REMAP_8 = 0x8;
    pTSDCfg->REMAP1.bREG.REMAP_9 = 0x9;
    pTSDCfg->REMAP1.bREG.REMAP_A = 0xA;
    pTSDCfg->REMAP1.bREG.REMAP_B = 0xB;
    pTSDCfg->REMAP1.bREG.REMAP_C = 0xC;
    pTSDCfg->REMAP1.bREG.REMAP_D = 0xD;
    pTSDCfg->REMAP1.bREG.REMAP_E = 0xE;
    pTSDCfg->REMAP1.bREG.REMAP_F = 0xF;
#else
    BITCSET(pTSDCfg->REMAP0.nREG, 0xFFFFFFFF, 0x00234547);
    BITCSET(pTSDCfg->REMAP1.nREG, 0xFFFFFFFF, 0x89ABCDEF);
#endif    
    TSDMX_CMUnloadBinary();
#ifndef     SUPPORT_DEBUG_CMHWDEMUX
    if(fw_data && fw_size > 0)
    {   
        memcpy((void *)pCodeMem, (void *)fw_data, fw_size);
    }
    else
        printk("Using previous loading the firmware\n");
#else
    memcpy((void *)pCodeMem, DebugHWDemux, sizeof(DebugHWDemux));
#endif    
    BITCLR(pTSDCfg->CM_RESET.nREG, Hw1|Hw2); //m3 reset
}

/*****************************************************************************
* Function Name : static void TSDMX_CMConfigure(void);
* Description : MailBox register init
* Arguments :
******************************************************************************/
static void TSDMX_CMConfigure(void)
{
    volatile PMAILBOX pMailBoxMain = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);
    volatile PMAILBOX pMailBoxSub = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX1_BASE);
    volatile PCM_TSD_CFG pTSDCfg = (volatile PCM_TSD_CFG)tcc_p2v(HwCORTEXM4_TSD_CFG_BASE);
    BITSET(pMailBoxMain->uMBOX_CTL_016.nREG, Hw0|Hw1|Hw4|Hw5|Hw6);
    BITSET(pMailBoxSub->uMBOX_CTL_016.nREG, Hw0|Hw1|Hw4|Hw5|Hw6);
    BITSET(pTSDCfg->IRQ_MASK_POL.nREG, Hw16|Hw22); //IRQ_MASK_POL, IRQ, FIQ(CHANGE POLARITY)
}

static irqreturn_t MailBox_Handler(int irq, void *dev)
{	
    unsigned int value1, value2;
    volatile int nReg[8];
    unsigned char cmd, dmxid;
    volatile PMAILBOX pMailBox = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);

    nReg[0] = pMailBox->uMBOX_RX0.nREG;
    nReg[1] = pMailBox->uMBOX_RX1.nREG; // Updated Position
    nReg[2] = pMailBox->uMBOX_RX2.nREG; // End Buffer Position
    nReg[3] = pMailBox->uMBOX_RX3.nREG;
    nReg[4] = pMailBox->uMBOX_RX4.nREG;
    nReg[5] = pMailBox->uMBOX_RX5.nREG;
    nReg[6] = pMailBox->uMBOX_RX6.nREG;
    nReg[7] = pMailBox->uMBOX_RX7.nREG;

    cmd = (nReg[0]&0xFF000000)>>24;
    dmxid = (nReg[0]&0x00FF0000)>>16;

    //if( cmd != 0xE1 && cmd != 0xF1)
    //    printk("cmd = 0x%X dmxid = 0x%X\n", cmd, dmxid);
    switch(cmd)
    {
        case HW_DEMUX_INIT: // DEMUX_INIT
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_INIT]));
            break;

        case HW_DEMUX_DEINIT: // DEMUX_DEINIT
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_DEINIT]));
            break;

        case HW_DEMUX_ADD_FILTER: // DEMUX_ADD_FILTER
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_ADD_FILTER]));
            break;

        case HW_DEMUX_DELETE_FILTER: // DEMUX_DELETE_FILTER
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_DELETE_FILTER]));
            break;

        case HW_DEMUX_GET_STC: // DEMUX_GET_STC
            {
                int filterType = 3;
                value1 = nReg[1]; // STC Base
                value2 = nReg[2]; // Bit 31: STC Extension
                if(ts_demux_buffer_updated_callback[dmxid])
                {
                    ts_demux_buffer_updated_callback[dmxid](dmxid, filterType, 0, nReg[1], nReg[2], nReg[3]);
                }
            }
            break;

        case HW_DEMUX_SET_PCR_PID: // DEMUX_SET_PCR_PID
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_SET_PCR_PID]));
            break;
        case HW_DEMUX_GET_VERSION: // HW_DEMUX_GET_VERSION
            g_ret = (nReg[0]&0x0000FFFF);
            printk("[HWDMX]Version[%X] Date[%X]\n", nReg[1], nReg[2]);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_GET_VERSION]));
            break;            
        case HW_DEMUX_INTERNAL_SET_INPUT: // HW_DEMUX_INTERNAL_SET_INPUT
            g_ret = (nReg[0]&0x0000FFFF);
            g_ret_done = 1;
            wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_INTERNAL_SET_INPUT]));
            break;            

        case HW_DEMUX_BUFFER_UPDATED: // DEMUX_BUFFER_UPDATED
            {
                unsigned int filterType = (nReg[0]&0x0000FF00)>>8;
                unsigned int filterID = nReg[0]&0x000000FF;
                unsigned int bErrCRC = (nReg[3]&0xFF000000)>>24;
                value1 = nReg[1];     // ts: end point of updated buffer(offset), sec: start point of updated buffer(offset)
                value2 = nReg[2] + 1; // ts: end point of buffer(offset),         sec: end point of updated buffer(offset)
                if (filterType == HW_DEMUX_TS)
                {
                    value1++;
                }
                else
                {
#ifdef     SUPPORT_DEBUG_CMHWDEMUX
                //    if(filterType == HW_DEMUX_SECTION)
                //        printk("Section:[0x%X][0x%X][0x%X][0x%X][0x%08X]\n", nReg[0], nReg[1], nReg[2], nReg[3], nReg[4]); 
#endif                    
                }
                if(ts_demux_buffer_updated_callback[dmxid])
                {
                    ts_demux_buffer_updated_callback[dmxid](dmxid, filterType, filterID, value1, value2, bErrCRC);
                }
            }
            break;
        case HW_DEMUX_DEBUG_DATA:
            {
                unsigned char str[9];
                unsigned int *p = (unsigned int *)str;
                p[0] = nReg[1];
                p[1] = nReg[2];
                str[8] = 0;
                printk("[HWDMX]%s :[0x%X][0x%X][0x%X][0x%X][0x%X].\n", str, nReg[3], nReg[4], nReg[5], nReg[6], nReg[7]);
            }
            break;

		case HW_CM_CIPHER_SET_ALGORITHM:
			g_ret = (nReg[0]&0x0000FFFF);
			g_ret_done = 1;
			wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_CIPHER_SET_ALGORITHM]));
			break;

		case HW_CM_CIPHER_SET_KEY:
			g_ret = (nReg[0]&0x0000FFFF);
			g_ret_done = 1;
			wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_CIPHER_SET_KEY]));
			break;

		case HW_CM_CIPHER_SET_IV:
			g_ret = (nReg[0]&0x0000FFFF);
			g_ret_done = 1;
			wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_CIPHER_SET_IV]));
			break;

		case HW_CM_CIPHER_EXECUTE:
			g_ret = (nReg[0]&0x0000FFFF);
			g_ret_done = 1;
			wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_CIPHER_EXECUTE]));
			break;

		case HW_DEMUX_CHANGE_INTERFACE:
			g_ret = (nReg[0]&0x0000FFFF);
			g_ret_done = 1;
			wake_up(&(cmd_queue[dmxid][QUEUE_HW_DEMUX_CHANGE_INTERFACE]));
			break;

		default:
			// NSK2 cmd handling.
			if (0xB0 <= cmd && cmd <= 0xCF) {
				if (nskMsgHandler != NULL) {
					nskMsgHandler(irq, dev, nReg);
				}
			}
			break;
    }

    return IRQ_HANDLED;
}

int tca_tsif_set_port(struct tcc_tsif_handle *h)
{
    int ret = 0;

    unsigned int port = h->port_cfg.tsif_port;
    unsigned int func = h->port_cfg.tsif_func;
    printk("%s : select port => %d, func => %d\n", __func__, port, func);

    tcc_gpio_config(h->port_cfg.pin0, GPIO_FN(func));
    tcc_gpio_config(h->port_cfg.pin1, GPIO_FN(func));
    tcc_gpio_config(h->port_cfg.pin2, GPIO_FN(func));
    tcc_gpio_config(h->port_cfg.pin3, GPIO_FN(func));
    if (h->serial_mode == 0)
    {
        tcc_gpio_config(h->port_cfg.pin4, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin5, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin6, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin7, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin8, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin9, GPIO_FN(func));
        tcc_gpio_config(h->port_cfg.pin10, GPIO_FN(func));
    }

    return ret;
}

int tca_tsif_release_port(struct tcc_tsif_handle *h)
{
#if 0
    volatile unsigned long* TSIFPORT = (volatile unsigned long *)tcc_p2v(HwTSIF_TSCHS_BASE);
#endif
    int ret = 0;
    unsigned int port = h->port_cfg.tsif_port;
    unsigned int func = 0;

    printk("%s : select port => %d\n", __func__, port);

    tcc_gpio_config(h->port_cfg.pin0, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
    tcc_gpio_config(h->port_cfg.pin1, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
    tcc_gpio_config(h->port_cfg.pin2, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
    tcc_gpio_config(h->port_cfg.pin3, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
    if (h->serial_mode == 0)
    {
        tcc_gpio_config(h->port_cfg.pin4, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin5, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin6, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin7, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin8, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin9, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(h->port_cfg.pin10, GPIO_FN(func)|GPIO_OUTPUT|GPIO_LOW);
    }

    return ret;
}

int tca_tsif_clock_enable(struct tcc_tsif_handle *h)
{
    if(cm_clk == NULL) 
    {
        cm_clk = clk_get(NULL, "cm_bus");
        if(IS_ERR(cm_clk)) 
        {
            printk("can't find cm_bus clk driver!\n");
            cm_clk = NULL;
        }
        else 
        {
#ifdef CONFIG_CLOCK_TABLE
           	tsif_clktbl = clocktable_get("tsif_clktbl");
	    	if (IS_ERR(tsif_clktbl))
            {
                clk_disable(cm_clk);
                clk_put(cm_clk);        
                cm_clk = NULL;
		    	tsif_clktbl = NULL;
                printk("can't find tsif_clktbl clk table!\n");
		    	return 1;
            }
    		else
	    		clocktable_ctrl(tsif_clktbl, 0, CLKTBL_ENABLE);
#else
            clk_set_rate(cm_clk, 200*1000*1000);
#endif            
            clk_enable(cm_clk);	
        }
    }
    if(cm_clk)
        return 0;
    else
        return 1;
}

int tca_tsif_clock_disable(struct tcc_tsif_handle *h)
{   
#ifndef     SUPPORT_DEBUG_CMHWDEMUX
    if(!cm_clk) 
    {
#ifdef CONFIG_CLOCK_TABLE
		if (tsif_clktbl) 
		{
			clocktable_ctrl(tsif_clktbl, 0, CLKTBL_DISABLE);
			clocktable_put(tsif_clktbl);
			tsif_clktbl = NULL;
		}
#endif        
        clk_disable(cm_clk);
        clk_put(cm_clk);        
        cm_clk = NULL;

    }
#endif  
    return 0;
}

int tca_tsif_pkg_enable(struct tcc_tsif_handle *h)
{
    char szBuf[128];
    sprintf(szBuf, "cmbus_tsif%d", h->dmx_id);
    if(pktgen_clk[h->dmx_id] == NULL) 
    {
        pktgen_clk[h->dmx_id] = clk_get(NULL, szBuf);
        if(IS_ERR(pktgen_clk[h->dmx_id])) 
        {
            printk("can't find %s clk driver!\n", szBuf);
            pktgen_clk[h->dmx_id] = NULL;
        }
        else
        {
            clk_set_rate(pktgen_clk[h->dmx_id], 27*1000*1000);	
            clk_enable(pktgen_clk[h->dmx_id]);
        }
    }
    if(pktgen_clk[h->dmx_id])
        return 0;
    else
        return 1;
}

int tca_tsif_pkg_disable(struct tcc_tsif_handle *h)
{
    if(pktgen_clk[h->dmx_id]) 
    {
        clk_disable(pktgen_clk[h->dmx_id]);
        clk_put(pktgen_clk[h->dmx_id]);
        pktgen_clk[h->dmx_id] = NULL;
    }
    return 0;
}

int tca_tsif_sys_enable(struct tcc_tsif_handle *h)
{
    int dmx_id, cmd_id, ret;
    for(dmx_id=0; dmx_id<4;dmx_id++) 
    {
        for(cmd_id=QUEUE_HW_DEMUX_INIT; cmd_id<QUEUE_HW_DEMUX_TYPE_MAX;cmd_id++) 
        {
            init_waitqueue_head(&cmd_queue[dmx_id][cmd_id]);
        }
    }
    tsdmx_lock_init();
    TSDMX_CMConfigure();
    TSDMX_CMLoadBinary(h->fw_data, h->fw_size);
    ret = request_irq(MBOX_IRQ, MailBox_Handler, IRQ_TYPE_LEVEL_LOW | IRQF_DISABLED, DEVICE_NAME, NULL);
    msleep(100); //Wait for CM Booting
    return 0;
}
int tca_tsif_sys_disable(struct tcc_tsif_handle *h)
{
    TSDMX_CMUnloadBinary();
    free_irq(MBOX_IRQ, NULL);
    return 0;
}

int tca_tsif_port_enable(struct tcc_tsif_handle *h)
{
    if(h->dmx_id == 0)
    {
        h->serial_mode = 1;

        if(g_interface == 1) // parallel TSIF
        {
            h->serial_mode = 0; // 1:Serial Mode,  0:Parallel Mode
        }
    }
    else if(h->dmx_id == 1)
    {
        //Caution!!!
        //Port setting for demux1
        //h->tsif_port_num = ?
        //h->serial_mode = ? //1:serialmode 0:parallel mode
        //If you set a port properly, we should remove below "return 0"
        printk("%s : return[%d]!\n", __func__, h->dmx_id);
        return 0;
    }
    tca_tsif_set_port(h);
    return 0;
}
int tca_tsif_port_disable(struct tcc_tsif_handle *h)
{
    if(h->dmx_id == 1)
    {
        //Caution!!
        //Because we didn't set port for demux1, we should return without
        //doing anything.
        printk("%s : return[%d]!\n", __func__, h->dmx_id);
        return 0;
    }
	tca_tsif_release_port(h);
    return 0;
}

void tca_tsif_set_interface(int mode) // mode(0: serial, 1: parallel, 2: internal)
{
    g_interface = mode;//(mode == 0) ? 0 : 1;

    TSDMX_Change_Interface(0, mode);
}

int tca_tsif_get_interface(void)
{
    return g_interface;
}

int tca_tsif_init(struct tcc_tsif_handle *h)
{
    int ret = 0;
    if(h == NULL)
        return 1;
    if(tca_tsif_clock_enable(h) != 0)
        return 1;
    if(tca_tsif_pkg_enable(h) != 0)
        return 1;
    if(g_initCount == 0) 
    {
        tca_tsif_sys_enable(h);
#ifdef      SUPPORT_DEBUG_CMHWDEMUX        
        while(1)
        {
            ret = TSDMX_GET_Version(h->dmx_id, msecs_to_jiffies(10000)); //Getting f/w version
            if(ret == 0)
                break;
        }
#else        
        ret = TSDMX_GET_Version(h->dmx_id, CMD_TIMEOUT); //Getting f/w version
#endif        
    }
    tca_tsif_port_enable(h);
    ret = TSDMX_Init(h);
    return ret;
}

void tca_tsif_clean(struct tcc_tsif_handle *h)
{
    int ret = 0;
    if(h==NULL)
        return;

    ts_demux_buffer_updated_callback[h->dmx_id] = NULL;
    tca_tsif_pkg_disable(h);
    ret = TSDMX_DeInit(h->dmx_id);
    tca_tsif_port_disable(h);
    if(g_initCount == 0) 
    {
        tca_tsif_sys_disable(h);
        tca_tsif_clock_disable(h);
    }	
    printk("TSDMX_DeInit Command Result : %d\n", ret);
    
}

int tca_tsif_start(struct tcc_tsif_handle *h)
{
	int ret = 0;

	if(h) {
	    ret = TSDMX_Init(h);
 	}
	
	return ret;
}

int tca_tsif_stop(struct tcc_tsif_handle *h)
{
	int ret = 0;

	if(h) {
	    ret = TSDMX_DeInit(h->dmx_id);
 	}

	return ret;
}

int tca_tsif_register_pids(struct tcc_tsif_handle *h, unsigned int *pids, unsigned int count)
{
    printk("[DEMUX #%d]tca_tsif_register_pids\n", h->dmx_id);
    return 0;
}

int tca_tsif_can_support(void)
{
    return 1;
}

int tca_tsif_buffer_updated_callback(struct tcc_tsif_handle *h, FN_UPDATECALLBACK buffer_updated)
{
    ts_demux_buffer_updated_callback[h->dmx_id] = buffer_updated;
    return 0;
}

int tca_tsif_set_nskcallback(FN_NSKCALLBACK _nskMsgHandler)
{
	nskMsgHandler = _nskMsgHandler;
	return 0;
}

int tca_tsif_set_pcrpid(struct tcc_tsif_handle *h, unsigned int pid)
{
    printk("[DEMUX #%d]tca_tsif_set_pcrpid(pid=%d)\n", h->dmx_id, pid);
    return TSDMX_SET_PCR_PID(h->dmx_id, pid);
}

long long tca_tsif_get_stc(struct tcc_tsif_handle *h)
{
    //printk("[DEMUX #%d]tca_tsif_get_stc\n", h->dmx_id);
    return TSDMX_GET_STC(h->dmx_id);
}

int tca_tsif_add_filter(struct tcc_tsif_handle *h, struct tcc_tsif_filter *feed)
{
    printk("[DEMUX #%d]tca_tsif_add_filter(type=%d, pid=%d)\n", h->dmx_id, feed->f_type, feed->f_pid);
    return TSDMX_ADD_Filter(h->dmx_id, feed);
}

int tca_tsif_remove_filter(struct tcc_tsif_handle *h, struct tcc_tsif_filter *feed)
{
    printk("[DEMUX #%d]tca_tsif_remove_filter(type=%d, pid=%d)\n", h->dmx_id, feed->f_type, feed->f_pid);
    return TSDMX_DELETE_Filter(h->dmx_id, feed);
}

int tca_tsif_input_internal(unsigned int dmx_id, unsigned int phy_addr, unsigned int size)
{
    //printk("[DEMUX #%d]tca_tsif_input_internal(buffer=[0x%X], size=%d)\n", dmx_id, phy_addr, size);
    return TSDMX_SET_InBuffer(dmx_id, phy_addr, size);
}

int tca_hwdemux_cipher_Set_Algorithm(stHWDEMUXCIPHER_ALGORITHM *pARG)
{
	int ret = 0;
//	printk("%s, %d  \n", __func__, __LINE__);

	tsdmx_lock_on();
	g_ret_done = 0;
	HWDEMUXCIPHERCMD_Set_Algorithm(pARG);
	if(wait_event_timeout((cmd_queue[pARG->uDemuxid][QUEUE_HW_DEMUX_CIPHER_SET_ALGORITHM]),g_ret_done==1,CMD_TIMEOUT) != 0)
		ret = g_ret;
	else
	{
		if(g_ret_done==0)
		{
			printk("%s:time-out\n", __func__);
			ret =  -2; //time out
		}
	}
	tsdmx_lock_off();
//	printk("%s, %d  \n", __func__, __LINE__);
	return ret;
}

int tca_hwdemux_cipher_Set_Key(stHWDEMUXCIPHER_KEY *pARG)
{
	unsigned int uiTotalIndex=0, i;
	int ret = 0;
//	printk("%s, %d  \n", __func__, __LINE__);

	tsdmx_lock_on();
		
	if(pARG->uLength<=6*4)		//6*4 : ("6 = uMBOX_TX2~uMBOX_TX7", "4 = uMBOX_TX2 = 4byte")
		uiTotalIndex = 1;
	else 
		uiTotalIndex = 2;

	for(i=0; i<uiTotalIndex; i++)
	{	
		g_ret_done = 0;
		HWDEMUXCIPHERCMD_Set_Key(pARG, uiTotalIndex, (i+1));
		if(wait_event_timeout((cmd_queue[pARG->uDemuxid][QUEUE_HW_DEMUX_CIPHER_SET_KEY]),g_ret_done==1,CMD_TIMEOUT) != 0)
			ret = g_ret;
		else
		{
			if(g_ret_done==0)
			{
				printk("%s:time-out\n", __func__);
				ret =  -2; //time out
			}
		}
	}
	tsdmx_lock_off();
//	printk("%s, %d  \n", __func__, __LINE__);
	return ret;

}

int tca_hwdemux_cipher_Set_IV(stHWDEMUXCIPHER_VECTOR *pARG)
{
	int ret = 0;

//	printk("%s, %d  \n", __func__, __LINE__);

	tsdmx_lock_on();
	g_ret_done = 0;
	HWDEMUXCIPHERCMD_Set_IV(pARG);
	if(wait_event_timeout((cmd_queue[pARG->uDemuxid][QUEUE_HW_DEMUX_CIPHER_SET_IV]),g_ret_done==1,CMD_TIMEOUT) != 0)
		ret = g_ret;
	else
	{
		if(g_ret_done==0)
		{
			printk("%s:time-out\n", __func__);
			ret =  -2; //time out
		}
	}
	tsdmx_lock_off();
//	printk("%s, %d  \n", __func__, __LINE__);

	return ret;
}

int tca_hwdemux_cipher_Cipher_Run(stHWDEMUXCIPHER_EXECUTE *pARG)
{
	int ret = 0;
//	printk("%s, %d  \n", __func__, __LINE__);

	tsdmx_lock_on();
	g_ret_done = 0;
	HWDEMUXCIPHERCMD_Cipher_Run(pARG);
	if(wait_event_timeout((cmd_queue[pARG->uDemuxid][QUEUE_HW_DEMUX_CIPHER_EXECUTE]),g_ret_done==1,CMD_TIMEOUT) != 0)
		ret = g_ret;
	else
	{
		if(g_ret_done==0)
		{
			printk("%s:time-out\n", __func__);
			ret =  -2; //time out
		}
	}
	tsdmx_lock_off();
//	printk("%s, %d  \n", __func__, __LINE__);
	return ret;
}


EXPORT_SYMBOL(tca_tsif_set_interface);
EXPORT_SYMBOL(tca_tsif_get_interface);
EXPORT_SYMBOL(tca_tsif_init);
EXPORT_SYMBOL(tca_tsif_clean);
EXPORT_SYMBOL(tca_tsif_start);
EXPORT_SYMBOL(tca_tsif_stop);


EXPORT_SYMBOL(tca_tsif_register_pids);
EXPORT_SYMBOL(tca_tsif_can_support);
EXPORT_SYMBOL(tca_tsif_buffer_updated_callback);
EXPORT_SYMBOL(tca_tsif_set_nskcallback);
EXPORT_SYMBOL(tca_tsif_set_pcrpid);
EXPORT_SYMBOL(tca_tsif_get_stc);
EXPORT_SYMBOL(tca_tsif_add_filter);
EXPORT_SYMBOL(tca_tsif_remove_filter);
EXPORT_SYMBOL(tca_tsif_input_internal);

EXPORT_SYMBOL(tca_hwdemux_cipher_Set_Algorithm);
EXPORT_SYMBOL(tca_hwdemux_cipher_Set_Key);
EXPORT_SYMBOL(tca_hwdemux_cipher_Set_IV);
EXPORT_SYMBOL(tca_hwdemux_cipher_Cipher_Run);
