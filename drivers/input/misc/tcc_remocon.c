/*
 * IR driver for remote controller : tcc_remocon.c
 *
 * Copyright (C) 2010 Telechips, Inc.
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

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/capability.h>
#include <linux/uio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <linux/time.h>
#include "tcc_remocon.h"

//#define ENABLE_DEBUG_LOG
//#define ENABLE_DEBUG_FIFO_DATA

//+[TCCQB] QuickBoot Booting & Making flag
#if defined(CONFIG_HIBERNATION)
extern unsigned int do_hibernate_boot;
#endif
//-[TCCQB]
//

extern struct clk *clk_get(struct device *dev, const char *id);
extern int clk_enable(struct clk *clk);
extern void clk_disable(struct clk *clk);

unsigned long rmt_clk_rate = 0;

#define DEVICE_NAME	"tcc-remote"

#define TCC_IRVERSION	0.001

#define KEY_RELEASED	0
#define KEY_PRESSED		1

#define LOW_MIN_VALUE		LOW_MIN_TCC892X
#define LOW_MAX_VALUE		LOW_MAX_TCC892X
#define HIGH_MIN_VALUE		HIGH_MIN_TCC892X
#define HIGH_MAX_VALUE		HIGH_MAX_TCC892X
#define REPEAT_MIN_VALUE	REPEAT_MIN_TCC892X
#define REPEAT_MAX_VALUE	REPEAT_MAX_TCC892X
#define START_MIN_VALUE		START_MIN_TCC892X
#define START_MAX_VALUE		START_MAX_TCC892X

struct remote_data {
	struct device_node	*remote_np;
	struct clk			*pclk;
	struct clk			*hclk;
	int					 irq_port;
	unsigned int		 core_clk_rate;
};

struct remote_data st_remote_data;

#if defined(ENABLE_DEBUG_LOG)
#define dbg printk
#else /* no debug */
#define dbg(x...) do {} while(0)
#endif

#if defined(ENABLE_DEBUG_FIFO_DATA)
int g_rd_data[1000];
int g_rd_vcnt[1000];
int g_rd_index[1000];
int gi=0;
static struct task_struct *g_remocon_fifo_thread=NULL;
static int fifo_checker(void *arg);
#endif

/*****************************************************************************
*
* structures
*
******************************************************************************/
struct tcc_remote
{
	unsigned int old_key;		//Previous KeyCode
	int status;		//Key Status

	struct input_dev *dev;  
	struct work_struct work_q;
};

typedef struct
{
	unsigned int repeatCount;
	unsigned int BitCnt;	//Data Bit Count
	unsigned long long Buf;	//Data Buffer
	unsigned int Id;	//Remocon ID
	unsigned int Code;	//Parsing Return Value
	unsigned int Stat;	//Remocon Status
}REM_DATA;

static int remote_state = 0;

static struct tcc_remote *rem;
static REM_DATA Rem;

static struct timer_list remocon_timer;

char output_lcdc_onoff = 1;

unsigned int tcc_remocon_GetTickCount(void)
{
	struct timeval tTimeVal;

	do_gettimeofday(&tTimeVal);

	printk("%d sec %d usec\n", (int)tTimeVal.tv_sec, (int)tTimeVal.tv_usec);

	return tTimeVal.tv_sec * 1000 + tTimeVal.tv_usec / 1000;
}

void tcc_remocon_rmcfg (unsigned int value)
{
	PREMOCON_CONFIG pRemoteCFG = (volatile PREMOCON_CONFIG)tcc_p2v(TCC_PA_REMOCON_CONFIG);

	#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
			BITSET(pRemoteCFG->nREG, Hw6); 
			BITCLR(pRemoteCFG->nREG, Hw5);
			BITSET(pRemoteCFG->nREG, Hw4);
			BITCLR(pRemoteCFG->nREG, Hw3);
	switch(value)
	{

		case 001 : //RMISEL is 001b 
			BITCLR(pRemoteCFG->nREG, Hw2); 
			BITCLR(pRemoteCFG->nREG, Hw1); 
			BITSET(pRemoteCFG->nREG, Hw0);
			break;
		case 010 : //RMISEL is 010b
			BITCLR(pRemoteCFG->nREG, Hw2); 
			BITSET(pRemoteCFG->nREG, Hw1); 
			BITCLR(pRemoteCFG->nREG, Hw0);
			break;
		case 011 : //RMISEL is 011b
			BITCLR(pRemoteCFG->nREG, Hw2); 
			BITSET(pRemoteCFG->nREG, Hw1); 
			BITSET(pRemoteCFG->nREG, Hw0);
			break;
		case 100 : //RMISEL is 100b
			BITSET(pRemoteCFG->nREG, Hw2); 
			BITCLR(pRemoteCFG->nREG, Hw1); 
			BITCLR(pRemoteCFG->nREG, Hw0);
			break;
		case 101 : //RMISEL is 101b
			BITSET(pRemoteCFG->nREG, Hw2); 
			BITCLR(pRemoteCFG->nREG, Hw1); 
			BITSET(pRemoteCFG->nREG, Hw0);
			break;
		case 110 : //RMISEL is 110b
			BITSET(pRemoteCFG->nREG, Hw2); 
			BITSET(pRemoteCFG->nREG, Hw1); 
			BITCLR(pRemoteCFG->nREG, Hw0);
			break;
		default :
			break;

	}
	#else
			BITSET(pRemoteCFG->nREG, Hw26);
			BITCLR(pRemoteCFG->nREG, Hw25);
			BITSET(pRemoteCFG->nREG, Hw24);
			BITCLR(pRemoteCFG->nREG, Hw23);
			BITCLR(pRemoteCFG->nREG, Hw22|Hw21);
			BITCLR(pRemoteCFG->nREG, Hw20);
	#endif

	return;
}

#if 0
static void tca_rem_sendremocondata(unsigned int key_type);

/*****************************************************************************
* Function Name : static unsigned int tca_rem_readcode(char ch);
* Description : Decode readCode.
* Arguments : ch - readcode, return value - key type(new or repeat)
******************************************************************************/
static unsigned int tca_rem_readcode(char ch)
{
	switch (Rem.Stat)
	{
		/* Initialize */
		case STATUS0:
			Rem.BitCnt = Rem.Buf = 0;
			Rem.Stat = STATUS1;
			if (ch == 'S')
			{
				Rem.Stat = STATUS2;
			}
			else if (ch == 'R')
			{
				//tcnt=12.5ms
				return REPEAT_KEY;
			}
			break;

		/*Start Bit */
		case  STATUS1:
			if (ch == 'S') 	//tcnt = 15ms
			{
				Rem.Code = 0;
				Rem.Stat = STATUS2;
			}
			//Repeat
			else if (ch == 'R')
			{
				dbg("R: %x\n", Rem.Code);
				//tcnt=12.5ms
				return REPEAT_KEY;
			}
			break;

		/* Data Code Check */
		case STATUS2:
			if (ch == '0')	//Bit '0' = 3.5ms
			{
				Rem.BitCnt++;
			}
			else if (ch == '1')	//Bit '1' = 5.5ms
			{
				Rem.Buf |= (((unsigned long long)1) << Rem.BitCnt);
				Rem.BitCnt++;
			}
			else if (ch == 'S')		//If Receive Start Bit, Return 0;
			{
				return 0;
			}
			else
			{
				Rem.Stat =STATUS0;
			}

			if(Rem.BitCnt == IR_SIGNAL_MAXIMUN_BIT_COUNT)
			{
				dbg("############### BUF=%llx\n", Rem.Buf);
				Rem.Id = (Rem.Buf & IR_ID_MASK);
				dbg("############### ID=%x\n", Rem.Id);
				if (Rem.Buf == REMOCON_REPEAT)
				{
					Rem.Stat = STATUS0;
					return REPEAT_START;
				}
				else if (Rem.Id == REMOCON_ID)
				{
					Rem.Code = (Rem.Buf  & IR_CODE_MASK) >> IR_BUFFER_BIT_SHIFT;
					dbg("############### CODE=%x\n", Rem.Code);
					Rem.Stat = STATUS0;
					return NEW_KEY;
				}
				else
				{
					Rem.Stat = STATUS0;
				}
			}
		break;
	}
	return NO_KEY;
}

/*****************************************************************************
* Function Name : static int tca_rem_process(char ch);
* Description : data processing* Arguments : ch - rawdata, return value(0: success, -1: failed)
******************************************************************************/
static int tca_rem_process(char ch)
{
	int key_type = tca_rem_readcode(ch);
	if (key_type==NO_KEY)	
		return -1;
	tca_rem_sendremocondata(key_type);
	return 0;
}

/*****************************************************************************
* Function Name : static void rem_getrawdata(int *rd_data);
* Description : transform IR input value into rawdata
* Arguments : rd_data - IR input value
******************************************************************************/
static void rem_getrawdata(int *rd_data)
{
	int i=0;
	int low, high;
	char low_bit='x', high_bit='x';
	do
	{
		low = rd_data[i] & 0xffff;
		high = (rd_data[i] >> 16) & 0xffff;
		if ((low > LOW_MIN_VALUE) && (low < LOW_MAX_VALUE))
		{
			low_bit='0';
		}
		else if ((low > HIGH_MIN_VALUE) && (low < HIGH_MAX_VALUE))
		{
			low_bit='1';
		}
		else if ((low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
		{
			low_bit='R';
		}
		else if ((low > START_MIN_VALUE) && (low < START_MAX_VALUE))
		{
			low_bit='S';
		}
		else
		{
			low_bit='E';
		}
		if ((high > LOW_MIN_VALUE) && (high < LOW_MAX_VALUE))
		{
			high_bit='0';
		}
		else if ((high > HIGH_MIN_VALUE) && (high < HIGH_MAX_VALUE))
		{
			high_bit='1';
		}
		else if ((high > REPEAT_MIN_VALUE) && (high < REPEAT_MAX_VALUE))
		{
			high_bit='R';
		}
		else if ((high > START_MIN_VALUE) && (high < START_MAX_VALUE))
		{
			high_bit='S';
		}
		else
		{
			high_bit='E';
		}
		if(low_bit == 'S' || high_bit=='S')
		{
			dbg("\n############### start\n");
		}
		//dbg("%04x|%04x => %c%c\n", low, high, low_bit, high_bit);
		dbg("%04x|%04x => %c%c\n", low, high, low_bit, high_bit);
		if (tca_rem_process(low_bit) == 0 || tca_rem_process(high_bit) == 0)
		{
			break;
		}
	} while(++i < IR_FIFO_READ_COUNT);
}
#endif

/*****************************************************************************
* Function Name : static void remocon_timer_handler(unsigned long data);
* Description : timer event handler for key release.
* Arguments :
******************************************************************************/
static void remocon_timer_handler(unsigned long data)
{
	struct input_dev *dev = rem->dev;

	if(rem->status==KEY_PRESSED)
	{
		input_report_key(dev, rem->old_key, 0);
		input_sync(dev);
		dbg("############### nRem=%d released(timer)\n",rem->old_key);
		rem->status = KEY_RELEASED;
		rem->old_key = NO_KEY;
		Rem.Code = NO_KEY;
		Rem.Buf = 0;
	}
}

/*****************************************************************************
* Function Name : static int tca_rem_getkeycodebyscancode(unsigned short kc);
* Description : transform ScanCode into KeyCode.
* Arguments : kc - scancode
******************************************************************************/
static int tca_rem_getkeycodebyscancode(unsigned short kc)
{
	int i;
	for (i = 0;i < sizeof(key_mapping)/sizeof(SCANCODE_MAPPING);i++)
		if (kc == key_mapping[i].rcode) 
			return key_mapping[i].vkcode;
	return -1;
}

/*****************************************************************************
* Function Name : static void tca_rem_sendremocondata(unsigned int key_type);
* Description : key event is generated.
* Arguments : Data - key type(new or repeat)
******************************************************************************/
static void tca_rem_sendremocondata(unsigned int key_type)
{
	struct input_dev *dev = rem->dev;
	unsigned int nRem, repeatCheck = 1;

	switch(key_type) {
	case NEW_KEY:
		nRem = tca_rem_getkeycodebyscancode(Rem.Code);
		if(nRem == -1 || (output_lcdc_onoff==0 && nRem!=REM_POWER))
			return;
		del_timer(&remocon_timer);
		if(rem->status==KEY_PRESSED) {
			dbg("############### nRem=%d released\n",rem->old_key);
			input_report_key(dev, rem->old_key, 0);
			input_sync(dev);
			rem->status=KEY_RELEASED;
		}
		dbg("############### nRem=%d pressed\n", nRem);
		rem->status = KEY_PRESSED;
		input_report_key(dev, nRem, 1);
		input_sync(dev);
		rem->old_key = nRem;
		Rem.repeatCount = 0;
		break;
	case REPEAT_KEY:
#ifndef CONFIG_VERIZON
		if(output_lcdc_onoff==0 || !isRepeatableKey(rem->old_key))
			return;
#endif
		if(rem->old_key == REM_POWER)
            repeatCheck = 15;

		del_timer(&remocon_timer);
		if(rem->status==KEY_PRESSED && Rem.repeatCount>repeatCheck) {
			dbg("############### nRem=%d repeat=%d\n",rem->old_key, Rem.repeatCount);
			input_event(dev, EV_KEY, rem->old_key, 2);    // repeat event
#ifdef CONFIG_VERIZON
			input_report_key(dev, rem->old_key,1);
#endif
			input_sync(dev);
		}
		Rem.repeatCount++;
		break;
	case REPEAT_START:
		if(output_lcdc_onoff==0 || !isRepeatableKey(rem->old_key))
			return;
		Rem.repeatCount = 0;
		del_timer(&remocon_timer);
	}
	remocon_timer.expires = jiffies + msecs_to_jiffies(175);
	add_timer(&remocon_timer);
}

/*****************************************************************************
* Function Name : static irqreturn_t remocon_handler(int irq, void *dev);
* Description : IR interrupt event handler
* Arguments :
******************************************************************************/
static irqreturn_t remocon_handler(int irq, void *dev)
{	
	int vcnt=0;
	int ii=0;
	int key_type;

	PREMOTECON pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	int rd_data[IR_FIFO_READ_COUNT];
	int low;
	char low_bit='x';
	
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	int delay=0;
#endif
	//tcc_remocon_GetTickCount();

	Rem.Stat = STATUS0;

  #if defined(CONFIG_ARCH_TCC893X)
	if(pRcu->BDD.nREG & Hw0)
	{
		rd_data[0] = pRcu->RDATA.nREG;
	}
  #endif
	for(ii=0; ii<10; ii++)
	{
		vcnt = pRcu->FSTA.nREG & 0x3F;

		if(vcnt < IR_FIFO_READ_COUNT)
		{
			//udelay(10);
		}
		else
		{
			break;
		}
	}

  #if defined(ENABLE_DEBUG_LOG)
  {
	int high=0;
	char high_bit='x';
	int i=0;
	do
	{
		rd_data[i] = pRcu->RDATA.nREG;
	#if defined(ENABLE_DEBUG_FIFO_DATA)
		g_rd_data[gi] = rd_data[i];
		g_rd_index[gi] = i;
		gi++;
		if(gi >= 1000)
			gi = 0;
	#endif

		low = rd_data[i] & 0xffff;
		high = (rd_data[i] >> 16) & 0xffff;

		if ((low > LOW_MIN_VALUE) && (low < LOW_MAX_VALUE))
		{ 		
		  low_bit='0';
#ifdef CONFIG_VERIZON
		  if((i==0) && (low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
		  {
			  low_bit='R';
			  BITSET(pRcu->BDD.nREG, Hw3);
		  } 
#endif
		}
		else if ((low > HIGH_MIN_VALUE) && (low < HIGH_MAX_VALUE))
		{
		  low_bit='1';
		}
		else if ((low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
		{
		  low_bit='R';
		}
		else if ((low > START_MIN_VALUE) && (low < START_MAX_VALUE))
		{
		  low_bit='S';
		}
		else
		{
		  low_bit='E';
		}

		if ((high > LOW_MIN_VALUE) && (high < LOW_MAX_VALUE))
		{
		  high_bit='0';
#ifdef CONFIG_VERIZON
		  if((i==0) && (low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
		  {
			  low_bit='R';
			  BITSET(pRcu->BDD.nREG, Hw3);
		  }
#endif
		}
		else if ((high > HIGH_MIN_VALUE) && (high < HIGH_MAX_VALUE))
		{
		  high_bit='1';
		}
		else if ((high > REPEAT_MIN_VALUE) && (high < REPEAT_MAX_VALUE))
		{
		  high_bit='R';
		}
		else if ((high > START_MIN_VALUE) && (high < START_MAX_VALUE))
		{
		  high_bit='S';
		}
		else
		{
		  high_bit='E';
		}

		if(low_bit == 'S' || high_bit=='S') {
		  dbg("\n############### start\n");
		}
		dbg("%04x|%04x => %c%c\n", low, high, low_bit, high_bit);
	}
	while (++i < IR_FIFO_READ_COUNT);

	dbg("==============================================\n");
	dbg("CMD	   0x%08lx\n", pRcu->CMD.nREG);
	dbg("INPOL   0x%08lx\n", pRcu->INPOL.nREG);
	dbg("STA	   0x%08lx\n", pRcu->STA.nREG);
	dbg("FSTA    0x%08lx\n", pRcu->FSTA.nREG);
	dbg("BDD	   0x%08lx\n", pRcu->BDD.nREG);
	dbg("BDR0    0x%08lx\n", pRcu->BDR0.nREG);
	dbg("BDR1    0x%08lx\n", pRcu->BDR1.nREG);

	dbg("SD	   0x%08lx\n", pRcu->SD.nREG);
	dbg("DBD0    0x%08lx\n", pRcu->DBD0.nREG);
	dbg("DBD1    0x%08lx\n", pRcu->DBD1.nREG);
	dbg("RBD	   0x%08lx\n", pRcu->RBD.nREG);
	dbg("PBD00   0x%08lx\n", pRcu->PBD00.nREG);
	dbg("PBD01   0x%08lx\n", pRcu->PBD01.nREG);
	dbg("PBD10   0x%08lx\n", pRcu->PBD10.nREG);
	dbg("PBD10   0x%08lx\n", pRcu->PBD10.nREG);
	dbg("PBD11   0x%08lx\n", pRcu->PBD11.nREG);
  #if defined(CONFIG_ARCH_TCC896X)||defined(CONFIG_ARCH_TCC897x) || defined(CONFIG_ARCH_TCC898X)
	dbg("PBD20   0x%08lx\n", pRcu->PBD20.nREG);
	dbg("PBD21   0x%08lx\n", pRcu->PBD21.nREG);
	dbg("PBD30   0x%08lx\n", pRcu->PBD30.nREG);
	dbg("PBD31   0x%08lx\n", pRcu->PBD31.nREG);
	dbg("RKD0   0x%08lx\n", pRcu->RKD0.nREG);
	dbg("RKD1   0x%08lx\n", pRcu->RKD1.nREG);
	dbg("PBM0   0x%08lx\n", pRcu->PBM0.nREG);
	dbg("PBM1   0x%08lx\n", pRcu->PBM1.nREG);
  #endif
	dbg("CLKDIV  0x%08lx\n", pRcu->CLKDIV.nREG);
	dbg("==============================================\n");
  }
  #else
	rd_data[0] = pRcu->RDATA.nREG;
	//mdelay(10);
  #endif

	low = rd_data[0] & 0xffff;
	if ((low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
	{
		low_bit='R';
#ifdef CONFIG_VERIZON
		key_type = REPEAT_KEY;
#endif
	}
	else if ((low > START_MIN_VALUE) && (low < START_MAX_VALUE))
	{
		low_bit='S';
	}

  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	if (Rem.Buf != 0 && !(pRcu->BDD.nREG & Hw2) && (low_bit=='S' || low_bit=='R'))
  #else
	if (Rem.Buf != 0 && !(pRcu->BDD.nREG & Hw2) && ((low_bit=='S' || low_bit=='R') && (pRcu->BDR0.nREG == 0x00000000 && pRcu->BDR1.nREG == 0x00000000)))
  #endif
	{
		del_timer(&remocon_timer);

		remocon_timer.expires = jiffies + msecs_to_jiffies(175);//msecs_to_jiffies(150);//msecs_to_jiffies(125);

		add_timer(&remocon_timer);

		RemoconInit(remote_state);
	
		return IRQ_HANDLED;
	}
	else if (Rem.Buf != 0 && (pRcu->BDD.nREG & Hw3 || low_bit=='R'))
	{
		key_type = REPEAT_KEY;
	}
	else
	{
	#if defined(CONFIG_ARCH_TCC893X)
		Rem.Buf = ((pRcu->BDR1.nREG & 0x7) << 29)|(pRcu->BDR0.nREG >> 3);
	#else
		Rem.Buf = ((pRcu->BDR1.nREG & 0x1) << 31)|(pRcu->BDR0.nREG >> 1);
	#endif
		Rem.Id = (Rem.Buf & IR_ID_MASK);
		Rem.Code = (Rem.Buf  & IR_CODE_MASK) >> IR_BUFFER_BIT_SHIFT;
#ifdef CONFIG_VERIZON
		Rem.Id = 0xf123;
		Rem.Code = (Rem.Buf  & IR_CODE_MASK);
#endif

		dbg("############### ID=0x%x\n", Rem.Id);
		
		if (Rem.Id == REMOCON_ID)
		{
			dbg("############### CODE=0x%x\n", Rem.Code);
			Rem.Stat = STATUS0;
			key_type = NEW_KEY;
		}
		else
		{
			dbg("############### INVALID CODE=0x%x\n", Rem.Code);
			key_type = NO_KEY;
			Rem.Code = 0;
		}
	}

	if (key_type!=NO_KEY)
		tca_rem_sendremocondata(key_type);

 	RemoconInit(remote_state);

#if defined(ENABLE_DEBUG_FIFO_DATA)
	{
		volatile int* pPhy = (volatile int*)0xFFC00000;

		if(*pPhy == 0x5a5a5a5a)
		{
			int icheck=0;
			*pPhy = 0;
			while(icheck < 1000);
			{
				low = g_rd_data[icheck] & 0xffff;
				high = (g_rd_data[icheck] >> 16) & 0xffff;
			
				if ((low > LOW_MIN_VALUE) && (low < LOW_MAX_VALUE))
				{		  
					low_bit='0';
				}
				else if ((low > HIGH_MIN_VALUE) && (low < HIGH_MAX_VALUE))
				{
					low_bit='1';
				}
				else if ((low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
				{
					low_bit='R';
				}
				else if ((low > START_MIN_VALUE) && (low < START_MAX_VALUE))
				{
					low_bit='S';
				}
				else
				{
					low_bit='E';
				}

				if ((high > LOW_MIN_VALUE) && (high < LOW_MAX_VALUE))
				{
					high_bit='0';
				}
				else if ((high > HIGH_MIN_VALUE) && (high < HIGH_MAX_VALUE))
				{
					high_bit='1';
				}
				else if ((high > REPEAT_MIN_VALUE) && (high < REPEAT_MAX_VALUE))
				{
					high_bit='R';
				}
				else if ((high > START_MIN_VALUE) && (high < START_MAX_VALUE))
				{
					high_bit='S';
				}
				else
				{
					high_bit='E';
				}

				if(low_bit == 'S' || high_bit=='S') {
					printk("\n############### start\n");
				}
				//dbg("%04x|%04x => %c%c\n", low, high, low_bit, high_bit);
				printk("%d:%04x|%04x => %c%c\n", g_rd_vcnt[icheck], low, high, low_bit, high_bit);
				icheck++;
			}
		}
	}
#endif
	//tcc_remocon_GetTickCount();

	return IRQ_HANDLED;
}

/*****************************************************************************
* Name :  Device register
* Description : This functions register device
* Arguments :
******************************************************************************/
static int remocon_probe(struct platform_device *pdev)
{
	int ret = -1;
	int err = -ENOMEM, i;
	unsigned int get_gpio = 0;
	struct input_dev *input_dev;
	PREMOCON_CONFIG pRemoteCFG = (volatile PREMOCON_CONFIG)tcc_p2v(TCC_PA_REMOCON_CONFIG);
	PREMOTECON pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	if (!pdev->dev.of_node){
		struct clk *remote_clk = clk_get(NULL, "remocon");
		if(IS_ERR(remote_clk)) {
			printk("can't find remocon clk driver!");
			remote_clk = NULL;
		} else {
			clk_enable(remote_clk);

		#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
			BITSET(pRemoteCFG->nREG, Hw6); 
			BITCLR(pRemoteCFG->nREG, Hw5);
			BITSET(pRemoteCFG->nREG, Hw4);
			BITCLR(pRemoteCFG->nREG, Hw3);
			BITCLR(pRemoteCFG->nREG, Hw2); // RMISEL[0:2] : PM remote controller wake up source
			BITSET(pRemoteCFG->nREG, Hw1); // this value is GPIO_C29 (010b)
			BITCLR(pRemoteCFG->nREG, Hw0);
		#else
			BITSET(pRemoteCFG->nREG, Hw26);
			BITCLR(pRemoteCFG->nREG, Hw25);
			BITSET(pRemoteCFG->nREG, Hw24);
			BITCLR(pRemoteCFG->nREG, Hw23);
			BITCLR(pRemoteCFG->nREG, Hw22|Hw21);
			BITCLR(pRemoteCFG->nREG, Hw20);
		#endif
	
		clk_set_rate(remote_clk, 24000000/4);

		rmt_clk_rate = clk_get_rate(remote_clk);
		dbg("############## %s: remote clk_rate = %lu\n", __func__, rmt_clk_rate);
		}
	}
	else{
		st_remote_data.remote_np = pdev->dev.of_node;

		st_remote_data.irq_port = platform_get_irq(pdev, 0);
		st_remote_data.hclk = of_clk_get(st_remote_data.remote_np, 1);
		st_remote_data.pclk = of_clk_get(st_remote_data.remote_np, 0);

		if (IS_ERR(st_remote_data.pclk) ||IS_ERR(st_remote_data.hclk) ) {
			printk(KERN_WARNING "REMOTE: failed to get remote clock\n");
			st_remote_data.pclk = NULL;
			st_remote_data.hclk = NULL;
			return -ENODEV;
		}
		
		if (st_remote_data.pclk && st_remote_data.hclk){
			clk_prepare_enable(st_remote_data.pclk);
			clk_prepare_enable(st_remote_data.hclk);
			
			of_property_read_u32(st_remote_data.remote_np, "clock-frequency", &st_remote_data.core_clk_rate);

			printk("Telechips Remote Controller [irq_port:%d], [core_clk_rate:%d]\n", st_remote_data.irq_port, st_remote_data.core_clk_rate);

			of_property_read_u32(st_remote_data.remote_np, "wakeupsrc",&get_gpio);
			tcc_remocon_rmcfg(get_gpio);
			clk_set_rate(st_remote_data.pclk, st_remote_data.core_clk_rate/4);
				
			rmt_clk_rate = clk_get_rate(st_remote_data.pclk);
			dbg(" %s: remote clk_rate = %lu\n", __func__, rmt_clk_rate);
		}
	}

	rem = kzalloc(sizeof(struct tcc_remote), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!rem || !input_dev)
	{
		err = -ENOMEM;
		goto error_alloc;
	}

	platform_set_drvdata(pdev, rem);

	rem->dev = input_dev;

	rem->dev->name = "telechips_remote_controller";
	rem->dev->phys = DEVICE_NAME;
	rem->dev->evbit[0] = BIT(EV_KEY);
	rem->dev->id.version = TCC_IRVERSION;
	input_dev->dev.parent = &pdev->dev;
	for (i = 0; i < ARRAY_SIZE(key_mapping); i++)
	{
		set_bit(key_mapping[i].vkcode & KEY_MAX, rem->dev->keybit);
	}

	//Init_IR_Port();
	RemoconConfigure ();
	RemoconStatus();
	RemoconDivide(remote_state);		//remocon clk divide and end_cout
	RemoconCommandOpen();
	RemoconIntClear();

	//Init IR variable
	rem->old_key = -1;
	rem->status = KEY_RELEASED;

	init_timer(&remocon_timer);
	remocon_timer.data = (unsigned long)NULL;
	remocon_timer.function = remocon_timer_handler;

{
	long long power_code = 0;
	pRcu->SD.nREG = (START_MAX_VALUE<<16)|START_MIN_VALUE;
	pRcu->DBD0.nREG = (LOW_MAX_VALUE<<16)|LOW_MIN_VALUE;
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	pRcu->DBD1.nREG = (REPEAT_MAX_VALUE<<16)|HIGH_MIN_VALUE;
#else
	pRcu->DBD1.nREG = (HIGH_MAX_VALUE<<16)|HIGH_MIN_VALUE;
#endif
	pRcu->RBD.nREG = (REPEAT_MAX_VALUE<<16)|REPEAT_MIN_VALUE;

#if defined(CONFIG_ARCH_TCC893X)
	power_code = (((long long)SCAN_POWER<<16)|(long long)REMOCON_ID)<<3;
  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	pRcu->PBD00.nREG = 0x00000004;
	pRcu->PBD01.nREG = 0x00000000;
  #else
	pRcu->PBD00.nREG = power_code&0xFFFFFFFF;
	pRcu->PBD01.nREG = (power_code>>32)&0x00000007;
  #endif
#elif defined(CONFIG_ARCH_TCC896X)||defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	power_code = (((long long)SCAN_POWER<<16)|(long long)REMOCON_ID)<<1;
	pRcu->PBD00.nREG = power_code&0xFFFFFFFF;
	pRcu->PBD01.nREG = (power_code>>32)&0xFFFFFFFF;
#endif

	pRcu->PBD10.nREG = 0xFFFFFFFF;
	pRcu->PBD11.nREG = 0xFFFFFFFF;

#if defined(CONFIG_ARCH_TCC896X)||defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	pRcu->PBD20.nREG = 0xFFFFFFFF;
	pRcu->PBD21.nREG = 0xFFFFFFFF;

	pRcu->PBD30.nREG = 0xFFFFFFFF;
	pRcu->PBD31.nREG = 0xFFFFFFFF;

	pRcu->PBM0.nREG = 0x00000000;
	pRcu->PBM1.nREG = 0x00000000;
#endif
}

#if defined(TCC_PM_SLEEP_WFI_USED) && defined(CONFIG_SLEEP_MODE)
	printk("IR request_irq with WFI mode\n");
	ret =  request_irq(st_remote_data.irq_port, remocon_handler, IRQ_TYPE_LEVEL_HIGH | IRQF_DISABLED, DEVICE_NAME, rem);
#else
	printk("IR request_irq\n");
  #if defined(CONFIG_ARCH_TCC893X) || defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	ret =  request_irq(st_remote_data.irq_port, remocon_handler, IRQ_TYPE_LEVEL_HIGH | IRQF_DISABLED, DEVICE_NAME, rem);
  #else
  #endif
#endif

	if (ret)
	{
		printk("IR remote request_irq error\n");
		goto error_irq;
	}

	ret = input_register_device(rem->dev);
	if (ret) 
		goto error_register;

#if defined(ENABLE_DEBUG_FIFO_DATA)
    if(g_remocon_fifo_thread == NULL){ 
        g_remocon_fifo_thread = (struct task_struct *)kthread_run(fifo_checker, NULL, "remocon_fifo_checker");
    }
#endif

	return 0;

error_alloc:
	input_free_device(input_dev);

error_irq:
	free_irq(st_remote_data.irq_port, rem);

error_register:
	input_unregister_device(rem->dev);

	return ret;
}

static int remocon_remove(struct platform_device *pdev)
{
	disable_irq(st_remote_data.irq_port);
	del_timer(&remocon_timer);
	free_irq(st_remote_data.irq_port, rem->dev);
	input_unregister_device(rem->dev);
	kfree(rem);

	if (st_remote_data.pclk && st_remote_data.hclk){
		clk_disable_unprepare(st_remote_data.pclk);
		clk_disable_unprepare(st_remote_data.hclk);
		st_remote_data.pclk = NULL;
		st_remote_data.hclk = NULL;
	}

	return 0;
}

#ifdef CONFIG_PM
static int remocon_suspend(struct platform_device *pdev, pm_message_t state)
{
#if defined(CONFIG_ARCH_TCC893X)
  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	PREMOTECON pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);
	long long power_code = 0;
	power_code = (((long long)SCAN_POWER<<16)|(long long)REMOCON_ID)<<3;
	pRcu->PBD00.nREG = power_code&0xFFFFFFFF;
	pRcu->PBD01.nREG = (power_code>>32)&0x00000007;
  #endif
#endif

#if defined(CONFIG_ARCH_TCC896X)|| defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);
	BITSET(pRcu->BDD.nREG, Hw25); //MISM
	BITCLR(pRcu->BDD.nREG, Hw23); //BDSC
#endif

	return 0;
}

static int remocon_resume(struct platform_device *pdev)
{
#if defined(CONFIG_ARCH_TCC893X) || defined(CONFIG_ARCH_TCC896X)|| defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	PREMOCON_CONFIG pRemoteCFG = (volatile PREMOCON_CONFIG)tcc_p2v(TCC_PA_REMOCON_CONFIG);
	st_remote_data.remote_np = pdev->dev.of_node;
	unsigned int get_gpio;
	#if defined(CONFIG_ARCH_TCC893X)
	PREMOTECON pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);
	long long power_code = 0;
	#endif

//+[TCCQB] Quickboot resume
  #if defined(CONFIG_HIBERNATION)
	if( do_hibernate_boot) {
		//Init_IR_Port();
		RemoconConfigure ();
		RemoconStatus();
		RemoconDivide(remote_state);		//remocon clk divide and end_cout
		RemoconCommandOpen();
		RemoconIntClear();
	}
  #endif
//-[TCCQB]
//

//*
	//Init_IR_Port();
	RemoconConfigure ();
	RemoconStatus();
	RemoconDivide(remote_state);		//remocon clk divide and end_cout
	RemoconCommandOpen();	
	RemoconIntClear();
//*/
  #if defined(CONFIG_ARCH_TCC893X)
	pRcu->SD.nREG = (START_MAX_VALUE<<16)|START_MIN_VALUE;
	pRcu->DBD0.nREG = (LOW_MAX_VALUE<<16)|LOW_MIN_VALUE;
  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	pRcu->DBD1.nREG = (REPEAT_MAX_VALUE<<16)|HIGH_MIN_VALUE;
  #else
	pRcu->DBD1.nREG = (HIGH_MAX_VALUE<<16)|HIGH_MIN_VALUE;
  #endif
	pRcu->RBD.nREG = (REPEAT_MAX_VALUE<<16)|REPEAT_MIN_VALUE;
	power_code = (((long long)SCAN_POWER<<16)|(long long)REMOCON_ID)<<3;
  #endif

  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	 #if defined(CONFIG_ARCH_TCC896X)||defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
			of_property_read_u32(st_remote_data.remote_np, "wakeupsrc",&get_gpio);
			tcc_remocon_rmcfg(get_gpio);
	 #else
			BITSET(pRemoteCFG->nREG, Hw26);
			BITCLR(pRemoteCFG->nREG, Hw25);
			BITSET(pRemoteCFG->nREG, Hw24);
			BITCLR(pRemoteCFG->nREG, Hw23);
			BITCLR(pRemoteCFG->nREG, Hw22|Hw21);
			BITCLR(pRemoteCFG->nREG, Hw20);
	 #endif

		#if defined(CONFIG_ARCH_TCC893X)
			#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
				Rcu->PBD00.nREG = 0x00000004;
				pRcu->PBD01.nREG = 0x00000000;
			#else
				pRcu->PBD00.nREG = power_code&0xFFFFFFFF;
				pRcu->PBD01.nREG = (power_code>>32)&0x00000007;
			#endif
		pRcu->PBD10.nREG = 0xFFFFFFFF;
		pRcu->PBD11.nREG = 0xFFFFFFFF;
    #endif
  #else
	#if defined(CONFIG_ARCH_TCC896X)||defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)

		of_property_read_u32(st_remote_data.remote_np, "wakeupsrc",&get_gpio);
		tcc_remocon_rmcfg(get_gpio);
	#else
		BITCLR(pRemoteCFG->nREG, Hw26);
		BITSET(pRemoteCFG->nREG, Hw25);
		BITCLR(pRemoteCFG->nREG, Hw24);
		BITCLR(pRemoteCFG->nREG, Hw23);
		BITCLR(pRemoteCFG->nREG, Hw22|Hw21|Hw20);	
    #endif

   #if defined(CONFIG_ARCH_TCC893X)
	pRcu->PBD00.nREG = power_code&0xFFFFFFFF;
	pRcu->PBD01.nREG = (power_code>>32)&0x00000007;
  #endif
  #endif
  #if defined(CONFIG_ARCH_TCC893X)
	pRcu->PBD10.nREG = 0xFFFFFFFF;
	pRcu->PBD11.nREG = 0xFFFFFFFF;
#endif
#endif

	remote_state = 0;
	RemoconInit(remote_state);

	return 0;
}
#else
#define remocon_suspend NULL
#define remocon_resume NULL
#endif

#if defined(ENABLE_DEBUG_FIFO_DATA)
static int fifo_checker(void *arg)
{
    printk("%s : started !!!\n",__func__);
	while(!kthread_should_stop())
	{
		volatile int* pPhy = (volatile int*)0xFFC00000;
		if(*pPhy == 0x5a5a5a5a)
		{
			printk("0x5a5a5a5a 0x5a5a5a5a !!!\n");

			int icheck=0;
			int low, high;
			char low_bit, high_bit;
			for(icheck=0; icheck<1000; icheck++)
			{
				low = g_rd_data[icheck] & 0xffff;
				high = (g_rd_data[icheck] >> 16) & 0xffff;
			
				if ((low > LOW_MIN_VALUE) && (low < LOW_MAX_VALUE))
				{		  
					low_bit='0';
				}
				else if ((low > HIGH_MIN_VALUE) && (low < HIGH_MAX_VALUE))
				{
					low_bit='1';
				}
				else if ((low > REPEAT_MIN_VALUE) && (low < REPEAT_MAX_VALUE))
				{
					low_bit='R';
				}
				else if ((low > START_MIN_VALUE) && (low < START_MAX_VALUE))
				{
					low_bit='S';
				}
				else
				{
					low_bit='E';
				}
			
				if ((high > LOW_MIN_VALUE) && (high < LOW_MAX_VALUE))
				{
					high_bit='0';
				}
				else if ((high > HIGH_MIN_VALUE) && (high < HIGH_MAX_VALUE))
				{
					high_bit='1';
				}
				else if ((high > REPEAT_MIN_VALUE) && (high < REPEAT_MAX_VALUE))
				{
					high_bit='R';
				}
				else if ((high > START_MIN_VALUE) && (high < START_MAX_VALUE))
				{
					high_bit='S';
				}
				else
				{
					high_bit='E';
				}
			
				if(low_bit == 'S' || high_bit=='S') {
					dbg("\n############### start\n");
				}
				dbg("0x%08x:%d:%04x|%04x => %c%c\n", g_rd_vcnt[icheck], g_rd_index[icheck], low, high, low_bit, high_bit);
			}
			*pPhy = 0;
		}
		else
		{
			msleep(100);
		}
	}
    printk("%s : ended !!!\n",__func__);
	return 0;
} 
#endif

#ifdef CONFIG_OF
static struct of_device_id tcc_remote_of_match[] = {
	{ .compatible = "telechips,tccxxxx-remote" },
	{}
};
MODULE_DEVICE_TABLE(of, tcc_remote_of_match);
#endif

static struct platform_driver remocon_driver =
{
	.driver	=
	{
		.name	= DEVICE_NAME,
		.owner 	= THIS_MODULE,
		
		.of_match_table = of_match_ptr(tcc_remote_of_match)
	},
	.probe		= remocon_probe,
	.remove		= remocon_remove,
#ifdef CONFIG_PM
	.suspend		= remocon_suspend,
	.resume		= remocon_resume,
#endif
};

static int __init remocon_init(void)
{
	printk("Telechips Remote Controller Driver Init\n");
	return platform_driver_register(&remocon_driver);
}

static void __exit remocon_exit(void)
{
	printk("Telechips Remote Controller Driver Exit \n");
	platform_driver_unregister(&remocon_driver);
}

module_init(remocon_init);
module_exit(remocon_exit);

MODULE_AUTHOR("Linux Team<linux@telechips.com>");
MODULE_DESCRIPTION("IR remote control driver");
MODULE_LICENSE("GPL");
