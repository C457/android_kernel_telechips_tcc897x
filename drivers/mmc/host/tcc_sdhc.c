/*
 * linux/drivers/mmc/host/tcc_sdhc.c
 *
 * Author:  <linux@telechips.com>
 * Created: Octo 18, 2010
 * Description: SD/MMC Host Driver for Telechips Boards.
 *
 * Copyright (C) 2008-2010 Telechips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/irq.h>

#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#ifndef CONFIG_ARM64
#include <asm/mach-types.h>
#endif
//#include <asm/system.h>

//#include <mach/bsp.h>
#include <mach/gpio.h>
//#include <mach/mmc.h>
#include <linux/io.h>
//#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

//#include <mach/iomap.h>
#include "tcc_sdhc.h"

#if defined(CONFIG_ENABLE_TCC_MMC_KPANIC)
#include <linux/mmc/tcc_kpanic.h>
#endif

#define WIFI_POLLING -9999

#undef DEBUG_SD
#ifdef	DEBUG_SD
#define SD_DEBUG(x...)	printk(x)
#else
#define SD_DEBUG(x...)
#endif

#undef DEBUG_CMD
#ifdef	DEBUG_CMD
#define SD_CMD(x...)	printk(x)
#else
#define SD_CMD(x...)
#endif

/* Clock Control */
//#undef SD_CLOCK_CONTROL
//#undef SD_CLOCK_CHANGE

#define DRIVER_NAME		"tcc-sdhc"

#define DETECT_TIMEOUT		(HZ/2)
	
#define SDMMC_FIFO_CNT		1024
#define SDMMC_TIMEOUT_TICKS	(1000*HZ/1000)	/* 1000ms */
#define TCC_MMC_GET_CMD(c) ((c>>24) & 0x3f)

int wifi_stat = 0;
EXPORT_SYMBOL(wifi_stat);

void tcc_mmc_gpio_set_value(unsigned gpio, int value)
{
	gpio_set_value_cansleep(gpio, value);
}

int tcc_mmc_gpio_get_value(unsigned gpio)
{
	return gpio_get_value_cansleep(gpio);
}
static inline int mmc_readw(struct tcc_mmc_host *host, unsigned offset)
{
	return readw(host->base + offset);
}

static inline void mmc_writew(struct tcc_mmc_host *host, u16 b,
			      unsigned offset)
{
	writew(b, host->base + offset);
}

static inline int mmc_readl(struct tcc_mmc_host *host, unsigned offset)
{
    return readl(host->base + offset);
}

static inline void mmc_writel(struct tcc_mmc_host *host, u32 b,
			      unsigned offset)
{
	writel(b, host->base + offset);
}

void tcc_mmc_clock_control(struct tcc_mmc_host *host, int onoff)
{
	uint32_t temp_reg;

	temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);

	if (onoff)
		mmc_writew(host, temp_reg | HwSDCLKSEL_SCK_EN, TCCSDHC_CLOCK_CONTROL);
	else
		mmc_writew(host, temp_reg & ~HwSDCLKSEL_SCK_EN, TCCSDHC_CLOCK_CONTROL);
}

static const struct of_device_id tcc_mmc_of_match[];

static void tcc_mmc_start_command(struct tcc_mmc_host *host, struct mmc_command *cmd);
static void tcc_mmc_tasklet_finish(unsigned long param);
static void init_mmc_host(struct tcc_mmc_host *host, unsigned int clock_rate);

static struct clk *rtc_clk = NULL;
int make_rtc(struct device_node *np)
{
	//tcc_gpio_config(TCC_GPB(29), GPIO_FN12);
    printk("\n\n%s\n\n", __func__);

#ifndef CONFIG_DAUDIO_KK
	if (rtc_clk == NULL) {
		rtc_clk = of_clk_get(np, 2);
		if (IS_ERR(rtc_clk)) {
			printk(KERN_WARNING "%s: failed to get out1 clock\n", __func__);
			rtc_clk = NULL;
			return -ENODEV;
		}
	}

	if (rtc_clk) {
		clk_prepare_enable(rtc_clk);
		clk_set_rate(rtc_clk, 32.8*1000);
		printk("rtc_clk = %d\n", (int)clk_get_rate(rtc_clk));
	}
#endif
    return 0;
}

static int tcc_sw_reset(struct tcc_mmc_host *host, uint8_t rst_bits)
{
	int timeout = 1000;

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return -EHOSTDOWN;
	}

	mmc_writew(host, rst_bits<<8 | mmc_readw(host, TCCSDHC_TIMEOUT_CONTROL),
		   TCCSDHC_TIMEOUT_CONTROL);

	while (--timeout) {
		if (!(mmc_readw(host, TCCSDHC_TIMEOUT_CONTROL) & rst_bits << 8)){
			break;
		}
		/* Must be mdelay not msleep, as called from interrupt context */
		mdelay(1);
	}
	if (!timeout) {
		printk(KERN_ERR "%s%d: timed out waiting for reset\n", DRIVER_NAME, host->controller_id);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Return 1 card inserted, return 0 card not inserted
 */
static int tcc_mmc_get_cd(struct mmc_host *mmc)
{
	if(mmc->slot.cd_irq == WIFI_POLLING)
	{
		return wifi_stat;
	} // sdio Wi-Fi
	else
	{
		return mmc_gpio_get_cd(mmc);
	}
}

/*
 * Card insert/remove timer handler
 */
static void tcc_mmc_poll_event(unsigned long data)
{
	int status= 0;
	struct tcc_mmc_host *host = (struct tcc_mmc_host *) data;

	if (host == NULL) {
		return;// -EHOSTDOWN;
    }

	if(host->mmc->detect_change == 0)
	{
		status = tcc_mmc_get_cd(host->mmc);
	}
	else
	{
		mod_timer(&host->detect_timer, jiffies + DETECT_TIMEOUT);
		return;
	}

	if(host->cd_irq <= 0) // polling only mode
	{
		if((status == 0) && (host->card_inserted == 1)) // remove
		{
			SD_DEBUG("%s%d: card removed\n", DRIVER_NAME, host->controller_id);
			mmc_detect_change(host->mmc, msecs_to_jiffies(0));
			host->card_inserted = status;
		}
		else if((status == 1) && (host->card_inserted == 0)) // insert
		{
			SD_DEBUG("%s%d: card inserted\n", DRIVER_NAME, host->controller_id);
			mmc_detect_change(host->mmc, msecs_to_jiffies(500));
			host->card_inserted = status;
		}
	}

	mod_timer(&host->detect_timer, jiffies + DETECT_TIMEOUT);
}

static char *tcc_mmc_kmap_atomic(struct scatterlist *sg, unsigned long *flags)
{
	if((sg == NULL)||(flags == NULL)) {
		printk("[mmc:NULL] %s(sg:%x, flags:%x)\n", __func__, (u32)sg, (u32)flags);
		return 0;
	}

	local_irq_save(*flags);
	return kmap_atomic(sg_page(sg)) + sg->offset;
}

static void tcc_mmc_kunmap_atomic(void *buffer, unsigned long *flags)
{
	if((buffer == NULL)||(flags == NULL)) {
		printk("[mmc:NULL] %s(buffer:%x, flags:%x)\n", __func__, (u32)buffer, (u32)flags);
		return;
	}

	kunmap_atomic(buffer);
	local_irq_restore(*flags);
}

static int tcc_mmc_adma_table_pre(struct tcc_mmc_host *host, struct mmc_data *data)
{
	int direction;

	u8 *desc;
	u8 *align;
	dma_addr_t addr;
	dma_addr_t align_addr;
	int len, offset;

	struct scatterlist *sg;
	int i;
	char *buffer;
	unsigned long flags;

	if((host == NULL)||(data == NULL)) {
		printk("[mmc:NULL] %s(host:%x, data:%x)\n", __func__, (u32)host, (u32)data);
		return -EHOSTDOWN;
	}

	/*
	 * The spec does not specify endianness of descriptor table.
	 * We currently guess that it is LE.
	 */

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	/*
	 * The ADMA descriptor table is mapped further down as we
	 * need to fill it with data first.
	 */

	host->align_addr = dma_map_single(mmc_dev(host->mmc),
			host->align_buffer, 128 * 4, direction);
	if (dma_mapping_error(mmc_dev(host->mmc), host->align_addr))
		goto fail;
	BUG_ON(host->align_addr & 0x3);

	host->sg_count = dma_map_sg(mmc_dev(host->mmc),
			data->sg, data->sg_len, direction);
	if (host->sg_count == 0)
		goto unmap_align;

	desc = host->adma_desc;
	align = host->align_buffer;

	align_addr = host->align_addr;

	for_each_sg(data->sg, sg, host->sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		/*
		 * The SDHCI specification states that ADMA
		 * addresses must be 32-bit aligned. If they
		 * aren't, then we use a bounce buffer for
		 * the (up to three) bytes that screw up the
		 * alignment.
		 */
		offset = (4 - (addr & 0x3)) & 0x3;
		if (offset) {
			if (data->flags & MMC_DATA_WRITE) {
				buffer = tcc_mmc_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(align, buffer, offset);
				tcc_mmc_kunmap_atomic(buffer, &flags);
			}

			desc[7] = (align_addr >> 24) & 0xff;
			desc[6] = (align_addr >> 16) & 0xff;
			desc[5] = (align_addr >> 8) & 0xff;
			desc[4] = (align_addr >> 0) & 0xff;

			BUG_ON(offset > 65536);

			desc[3] = (offset >> 8) & 0xff;
			desc[2] = (offset >> 0) & 0xff;

			desc[1] = 0x00;
			desc[0] = 0x21; /* tran, valid */

			align += 4;
			align_addr += 4;

			desc += 8;

			addr += offset;
			len -= offset;
		}

		desc[7] = (addr >> 24) & 0xff;
		desc[6] = (addr >> 16) & 0xff;
		desc[5] = (addr >> 8) & 0xff;
		desc[4] = (addr >> 0) & 0xff;

		BUG_ON(len > 65536);

		desc[3] = (len >> 8) & 0xff;
		desc[2] = (len >> 0) & 0xff;

		desc[1] = 0x00;
		desc[0] = 0x21; /* tran, valid */

		desc += 8;

		/*
		 * If this triggers then we have a calculation bug
		 * somewhere. :/
		 */
		WARN_ON((desc - host->adma_desc) > (128 * 2 + 1) * 4);
	}

	desc -= 8;

	/*
	 * Add a terminating entry.
	 */
//	desc[7] = 0;
//	desc[6] = 0;
//	desc[5] = 0;
//	desc[4] = 0;
//	desc[3] = 0;
//	desc[2] = 0;
//	desc[1] = 0x00;
	desc[0] = 0x27; /* nop, end, valid */

	/*
	 * Resync align buffer as we might have changed it.
	 */
	if (data->flags & MMC_DATA_WRITE) {
		dma_sync_single_for_device(mmc_dev(host->mmc),
				host->align_addr, 128 * 4, direction);
	}

	host->adma_addr = dma_map_single(mmc_dev(host->mmc),	host->adma_desc, (128 * 2 + 1) * 4, DMA_TO_DEVICE);

//	printk("ADMA address: 0x%x\n",host->adma_addr);

	if (dma_mapping_error(mmc_dev(host->mmc), host->adma_addr))
		goto unmap_entries;
	BUG_ON(host->adma_addr & 0x3);

	return 0;

unmap_entries:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, direction);
unmap_align:
	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
			128 * 4, direction);
fail:
	return -EINVAL;
}

static void tcc_mmc_adma_table_post(struct tcc_mmc_host *host, struct mmc_data *data)
{
	int direction;

	struct scatterlist *sg;
	int i, size;
	u8 *align;
	char *buffer;
	unsigned long flags;

	if((host == NULL)||(data == NULL)) {
		printk("[mmc:NULL] %s(host:%x, data:%x)\n", __func__, (u32)host, (u32)data);
		return;// -EHOSTDOWN;
	}

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	dma_unmap_single(mmc_dev(host->mmc), host->adma_addr,
			(128 * 2 + 1) * 4, DMA_TO_DEVICE);

	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
			128 * 4, direction);

	if (data->flags & MMC_DATA_READ) {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc), data->sg,
				data->sg_len, direction);

		align = host->align_buffer;

		for_each_sg(data->sg, sg, host->sg_count, i) {
			if (sg_dma_address(sg) & 0x3) {
				size = 4 - (sg_dma_address(sg) & 0x3);

				buffer = tcc_mmc_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(buffer, align, size);
				tcc_mmc_kunmap_atomic(buffer, &flags);

				align += 4;
			}
		}
	}

	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, direction);
}

static void tcc_mmc_finish_data(struct tcc_mmc_host *host)
{
	struct mmc_data *data;
	u16 blocks;

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	if (host->flags & TCC_MMC_REQ_USE_DMA) {
		if (!host->is_direct && !host->is_in_tuning_mode) {
			if (host->flags & TCC_MMC_USE_ADMA) {
				tcc_mmc_adma_table_post(host, data);
			} else {
				dma_unmap_sg(NULL, data->sg, data->sg_len,
					(data->flags & MMC_DATA_READ)?DMA_FROM_DEVICE:DMA_TO_DEVICE);
			}
		} else {
			dma_unmap_sg(NULL, data->sg, data->sg_len,
				(data->flags & MMC_DATA_READ)?DMA_FROM_DEVICE:DMA_TO_DEVICE);
		}
	}

	/*
	 * Controller doesn't count down when in single block mode.
	 */
	if (data->blocks == 1) {
		blocks = (data->error == 0) ? 0 : 1;
	} else {
		blocks = mmc_readw(host, TCCSDHC_BLOCK_COUNT);
	}
	data->bytes_xfered = data->blksz * (data->blocks - blocks);

	if (!data->error && blocks) {
		printk(KERN_ERR "%s: Controller signalled completion even "
				"though there were blocks left.\n",
				mmc_hostname(host->mmc));
		data->error = -EIO;
	}

	/* It must be enabled. When It doesn't sends STOP command */

	if (data->stop && (data->error || !host->mrq->sbc)) {
		if (data->error) {
			tcc_sw_reset(host, HwSD_SRESET_RSTCMD);
			tcc_sw_reset(host, HwSD_SRESET_RSTDAT);
		}
		tcc_mmc_start_command(host, data->stop);
	} else {
		tasklet_schedule(&host->finish_tasklet);
	}
}

//FIXME: Not yet implemented...
static void tcc_transfer_pio(struct tcc_mmc_host *host)
{
	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	BUG_ON(!host->data);
	SD_DEBUG("enter %s  ", __func__);
}

static void tcc_mmc_start_command(struct tcc_mmc_host *host, struct mmc_command *cmd)
{
	u32 resptype;
	u32 cmdtype;
	u32 mask;
	unsigned long timeout;
	int cmd_reg = 0x00000000;
	unsigned int uiIntStatusEn;

	if((host == NULL)||(cmd == NULL)) {
		printk("[mmc:NULL] %s(host:%x, cmd:%x)\n", __func__, (u32)host, (u32)cmd);
		return;// -EHOSTDOWN;
	}

	/* Wait max 10 ms */
	timeout = 10;

	cmdtype = 0;

	mask = HwSD_STATE_NOCMD;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY)) {
		mask |= HwSD_STATE_NODAT;
	}

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (host->mrq->data && (cmd == host->mrq->data->stop)) {
		mask &= ~HwSD_STATE_NODAT;
	}

	while (mmc_readl(host, TCCSDHC_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Controller never released "
					"inhibit bit(s).\n", mmc_hostname(host->mmc));
			cmd->error = -EIO;
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
	}

	mod_timer(&host->timer, jiffies + 10 * HZ);

	host->cmd = cmd;

	switch (mmc_resp_type(cmd)) {
		case MMC_RSP_NONE:
			resptype = 0;
			break;
		case MMC_RSP_R1:
			resptype = 2;
			break;
		case MMC_RSP_R1B:
			resptype = 3;
			break;
		case MMC_RSP_R2:
			resptype = 1;
			break;
		default:
			resptype = 2;
			break;
	}

	uiIntStatusEn = mmc_readl(host, TCCSDHC_INT_ENABLE);

	uiIntStatusEn |= HwSDINT_EN_TDONE | HwSDINT_EN_CDONE;

	if (cmd->data || host->is_in_tuning_mode) {
		host->data = cmd->data;
		host->data_early = 0;

		cmd_reg |= HwSD_COM_TRANS_DATSEL | HwSD_COM_TRANS_DIR;

		if (cmd->data->blocks > 1) {
			cmd_reg |= HwSD_COM_TRANS_MS | HwSD_COM_TRANS_BCNTEN;
			/* SD_IO_RW_EXTENDED means that sdio is used.*/
			if (cmd->opcode != SD_IO_RW_EXTENDED) {
				if (!host->mrq->sbc) { // if a command is not CMD23
					cmd_reg |= HwSD_COM_TRANS_ACMD12;  // It is related STOP command
				}
				else if (host->mrq->sbc) { // if a command is CMD23
					//cmd_reg |= 0x08;
					cmd_reg |= HwSD_COM_TRANS_ACMD23; // auto CMD23 enable
					mmc_writel(host, host->mrq->sbc->arg, TCCSDHC_DMA_ADDRESS);
					if (host->mrq->sbc->arg & 0x80000000)
						printk("\x1b[1;32m %s: 0x%08X \x1b[0m\n", __func__, host->mrq->sbc->arg);
				}
			}
		}

		if (cmd->data->flags & MMC_DATA_WRITE)
			cmd_reg &= ~HwSD_COM_TRANS_DIR;

		if (host->flags & TCC_MMC_USE_DMA)
			host->flags |= TCC_MMC_REQ_USE_DMA;

		if ((host->flags & TCC_MMC_REQ_USE_DMA) && (!host->is_in_tuning_mode))
			cmd_reg |= HwSD_COM_TRANS_DMAEN;

		/*
		 * Always adjust the DMA selection as some controllers
		 * (e.g. JMicron) can't do PIO properly when the selection
		 * is ADMA.
		 */
		if (host->flags & TCC_MMC_USE_DMA) {
			unsigned short ctrl = mmc_readw(host, TCCSDHC_HOST_CONTROL);
			ctrl &= ~HwSD_CTRL_DMA_MASK;

			if (!host->is_direct) {
				if ((host->flags & TCC_MMC_REQ_USE_DMA) && (host->flags & TCC_MMC_USE_ADMA))
					ctrl |= HwSD_CTRL_ADMA32;
				else
					ctrl |= HwSD_CTRL_SDMA;
			} else {
				ctrl |= HwSD_CTRL_SDMA;
			}

			mmc_writew(host, ctrl, TCCSDHC_HOST_CONTROL);
		}

		if ((host->flags & TCC_MMC_REQ_USE_DMA) && (!host->is_in_tuning_mode)) {
			if (!host->is_direct) {
				if (host->flags  & TCC_MMC_USE_ADMA) {
					tcc_mmc_adma_table_pre(host,cmd->data);
					mmc_writel(host, host->adma_addr,
						   TCCSDHC_ADMA_ADDRESS);
				} else {
					int count;
					//count = dma_map_sg(NULL, cmd->data->sg, cmd->data->sg_len,
					count = dma_map_sg(mmc_dev(host->mmc), cmd->data->sg, cmd->data->sg_len,
						(cmd->data->flags & MMC_DATA_READ)?DMA_FROM_DEVICE:DMA_TO_DEVICE);
					BUG_ON(count != 1);

					mmc_writel(host, (u32)sg_dma_address(cmd->data->sg),
						   TCCSDHC_DMA_ADDRESS);
				}
			} else {
				mmc_writel(host, host->req->dma,
					   TCCSDHC_DMA_ADDRESS);
			}
		} else {
			if (!host->is_direct) {
				host->cur_sg = cmd->data->sg;
				host->num_sg = cmd->data->sg_len;
				host->offset = 0;
				host->remain = host->cur_sg->length;
			} else {
				printk(KERN_ERR "%s: only support DMA!!!", mmc_hostname(host->mmc));
			}
		}
		/* reset is_direct */
		//host->is_direct = 0;

	}

	if (((cmd->opcode==SD_IO_RW_DIRECT) && ((cmd->arg & (0x03<<28)) ==0)
				&& ((cmd->arg&(0x1ff<<9)) == (SDIO_CCCR_ABORT<<9)) && (cmd->arg&0x07))
			|| (cmd->opcode==MMC_STOP_TRANSMISSION)) {
		/* Use R5b For CMD52, Function 0, I/O Abort
		 * Need to be revised for handling CMD12
		 */
		cmdtype = HwSD_COM_TRANS_ABORT;
	}

	if (cmd->flags & MMC_RSP_CRC)
		cmd_reg |= HwSD_COM_TRANS_CRCHK;

	if (cmd->flags & MMC_RSP_OPCODE)
		cmd_reg |= HwSD_COM_TRANS_CICHK;

	cmd_reg |= (cmd->opcode << 24) |cmdtype| (resptype << 16);

	/*
	if (mmc_readl(host, TCCSDHC_INT_STATUS) & HwSDINT_STATUS_ERR) {
		mmc_writew(host, mmc_readw(host, TCCSDHC_TIMEOUT_CONTROL) |
			   (HwSD_SRESET_RSTCMD<<8), TCCSDHC_TIMEOUT_CONTROL);
		while (mmc_readw(host, TCCSDHC_TIMEOUT_CONTROL) & HwSD_SRESET_RSTCMD<<8);
			msleep(1);

	}
	*/

	mmc_writel(host, mmc_readl(host, TCCSDHC_INT_STATUS),
		   TCCSDHC_INT_STATUS);

	if (cmd->data) {
		if (host->is_in_tuning_mode)
			mmc_writew(host, cmd->data->blksz, TCCSDHC_BLOCK_SIZE);
		else
			mmc_writew(host, (0x07<<12) | cmd->data->blksz,
					TCCSDHC_BLOCK_SIZE);
		mmc_writew(host, cmd->data->blocks, TCCSDHC_BLOCK_COUNT);
	} else {
		mmc_writel(host, 0, TCCSDHC_DMA_ADDRESS);
		mmc_writew(host, 0, TCCSDHC_BLOCK_SIZE);
		mmc_writew(host, 0, TCCSDHC_BLOCK_COUNT);
	}

	if (host->is_in_tuning_mode) {
		int flags = SG_MITER_ATOMIC | SG_MITER_TO_SG;
		sg_miter_start(&host->sg_miter, cmd->data->sg, cmd->data->sg_len, flags);
		/*printk("[%s] sg_miter_start : sg_len(%d)\n", __func__, cmd->data->sg_len);*/
		cmd_reg |= HwSD_COM_TRANS_DATSEL | HwSD_COM_TRANS_DIR | Hw20 | Hw19;
		mmc_writel(host, 0x0, TCCSDHC_DMA_ADDRESS);
	}
	mmc_writel(host, cmd->arg, TCCSDHC_ARGUMENT);

	/* Enable transfer interrupt sources */
	mmc_writel(host, uiIntStatusEn, TCCSDHC_INT_ENABLE);
	mmc_writel(host, cmd_reg, TCCSDHC_TMODE_COM);

	SD_CMD("%s: CMD%d, arg=%x\n", __func__, cmd->opcode, cmd->arg);
}

static void tcc_mmc_finish_command(struct tcc_mmc_host *host)
{
	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			host->cmd->resp[0] = mmc_readl(host, TCCSDHC_RESPONSE76);
			host->cmd->resp[1] = mmc_readl(host, TCCSDHC_RESPONSE54);
			host->cmd->resp[2] = mmc_readl(host, TCCSDHC_RESPONSE32);
			host->cmd->resp[3] = mmc_readl(host, TCCSDHC_RESPONSE10);

			SD_CMD("%s: R2: resp[0]=0x%08x, resp[1]=0x%08x, resp[2]=0x%08x, resp[3]=0x%08x\n",
					__func__, host->cmd->resp[0], host->cmd->resp[1], host->cmd->resp[2], host->cmd->resp[3]);

			host->cmd->resp[0] = (host->cmd->resp[0] << 8) | ((host->cmd->resp[1] & 0xFF000000) >> 24);
			host->cmd->resp[1] = (host->cmd->resp[1] << 8) | ((host->cmd->resp[2] & 0xFF000000) >> 24);
			host->cmd->resp[2] = (host->cmd->resp[2] << 8) | ((host->cmd->resp[3] & 0xFF000000) >> 24);
			host->cmd->resp[3] <<= 8;

		} else {
			host->cmd->resp[0] = mmc_readl(host, TCCSDHC_RESPONSE10);

			SD_CMD("%s: R1: resp[0]=0x%08x\n",  __func__, host->cmd->resp[0]);
		}
	}

	host->cmd->error = 0;
	if (host->cmd == host->mrq->sbc) {
		host->cmd = NULL;
		tcc_mmc_start_command(host, host->mrq->cmd);
	} else {
		if (host->data && host->data_early)
			tcc_mmc_finish_data(host);

		if (!host->cmd->data)
			tasklet_schedule(&host->finish_tasklet);

		host->cmd = NULL;
	}
}

static void tcc_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;

	if((mmc == NULL)||(host == NULL)||(mrq == NULL)) {
		printk("[mmc:NULL] %s(mmc:%x, host:%x, mrq:%x)\n", __func__, (u32)mmc, (u32)host, (u32)mrq);
		return;// -EHOSTDOWN;
	}

	spin_lock_irqsave(&host->lock, flags);

	if (host->is_clock_control)
		tcc_mmc_clock_control(host, 1);

	BUG_ON(host->mrq != NULL);

	host->mrq = mrq;
	if (!mrq->sbc) {
		if (mrq->stop) {
			mrq->data->stop = NULL;
			mrq->stop = NULL;
		}
	}

	//if (!mmc_gpio_get_cd(mmc)) { // This code should be generated irq lockup when using cd gpio as expand gpio.
	if (!host->card_inserted) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = 0;
			mrq->data->bytes_xfered = mrq->data->blksz *
						  mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		tasklet_schedule(&host->finish_tasklet);
		spin_unlock_irqrestore(&host->lock, flags);
		return;

	} else {
		if (mrq->sbc && !(host->flags & HwSD_COM_TRANS_ACMD23))
			tcc_mmc_start_command(host, mrq->sbc);
		else
			tcc_mmc_start_command(host, mrq->cmd);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

/* High speed mode threshold (Hz).
 *
 * Although high speed mode should be suitable for all speeds not all
 * controller/card combinations are capable of meeting the higher
 * tolerances for (e.g.) clock rise/fall times.  Therefore, default
 * mode is used where possible for improved compatibility. */

#define SDIO_CLOCK_FREQ_HIGH_SPD 25000000

static void tcc_hw_set_high_speed(struct mmc_host *mmc, int hs)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u8 host_ctrl = 0;

	if((mmc == NULL)||(host == NULL)) {
		printk("[mmc:NULL] %s(mmc:%x, host:%x)\n", __func__, (u32)mmc, (u32)host);
		return;// -EHOSTDOWN;
	}

	spin_lock_irqsave(&host->lock, flags);

	host_ctrl= mmc_readw(host, TCCSDHC_HOST_CONTROL);
	host_ctrl &= ~HwSD_POWER_HS;

#if 0 // 1 : falling edge, 0 : rising edge
	if (hs) {
		host_ctrl |= HwSD_POWER_HS;
	}
#endif

	mmc_writew(host, host_ctrl, TCCSDHC_HOST_CONTROL);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void tcc_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	uint32_t temp_reg;
	struct pinctrl *pinctrl;

	long dwMaxClockRate = host->peri_clk;
	int i = 0; /* 2^i is the divisor value */
	u32 clk_div = 0;

	if((mmc == NULL)||(host == NULL)||(ios == NULL)) {
		return;// -EHOSTDOWN;
	}

	if (ios->clock != 0) {
		tcc_hw_set_high_speed(mmc, ios->clock > SDIO_CLOCK_FREQ_HIGH_SPD);
		/* shift MaxClockRate until we find the closest frequency <= target */
		//while ((ios->clock < dwMaxClockRate) && (i < 8)) {
		while ((ios->clock < dwMaxClockRate)) {
			dwMaxClockRate = dwMaxClockRate >> 1;
			i++;
		}

		if(i==0)
			clk_div = 0;
		else
			clk_div = 1<<(i-1);

		host->timing = ios->timing;
		if ( (ios->timing == MMC_TIMING_MMC_DDR52) || (ios->timing == MMC_TIMING_UHS_DDR50) ) {
			printk("[tcc_mmc] Start DDR clock...\n");
			mmc_writel(host, 0x40000, TCCSDHC_ACMD12_ERR);
		}
		dev_dbg(host->dev, "[%s] id[%d] ios->clock : %d, host->peri_clk: %lu, clk_div : %d\n",
		    __func__, host->controller_id, ios->clock, host->peri_clk, clk_div);

		//printk("##### [%s] id[%d] ios->clock : %d, host->peri_clk: %lu, clk_div : %d\n",
		    //__func__, host->controller_id, ios->clock, host->peri_clk, clk_div);

		host->clk_div = clk_div;	/* store divider */

		temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
		mmc_writew(host, temp_reg & ~HwSDCLKSEL_SCK_EN,
			   TCCSDHC_CLOCK_CONTROL);

		udelay(10);

		if(clk_div <= 128)
		{
			mmc_writew(host, (clk_div << 8) | HwSDCLKSEL_INCLK_EN,
				TCCSDHC_CLOCK_CONTROL);
		}
		else // use extend SDCLKSEL register
		{
			if(clk_div > 1023)
				host->clk_div = clk_div = 1023;

			mmc_writew(host, ((clk_div & 0xFF) << 8) | ((clk_div & 0x300) >> 2) | HwSDCLKSEL_INCLK_EN,
				TCCSDHC_CLOCK_CONTROL);
		}

		while (!(mmc_readl(host, TCCSDHC_CLOCK_CONTROL) & HwSDCLKSEL_INCLK_STABLE)) 	{
			udelay(100);
		}
		temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
		//mmc_writew(host, temp_reg|(HwSDCLKSEL_SCK_EN|HwSDCLKSEL_INCLK_EN), TCCSDHC_CLOCK_CONTROL);
		mmc_writew(host, temp_reg | (HwSDCLKSEL_SCK_EN),
			   TCCSDHC_CLOCK_CONTROL);

		udelay(100);
	}

	switch (ios->power_mode) {
		case MMC_POWER_OFF:
//				printk("%s: [%s:MMC_POWER_OFF]\n", mmc_hostname(host->mmc), __func__);
				SD_DEBUG("%s%d: MMC_POWER_OFF\n", DRIVER_NAME, host->controller_id);

				pinctrl = pinctrl_get_select(host->dev, "default");
				if(IS_ERR(pinctrl))
					SD_DEBUG("%s%d: pinctrl select failed\n", DRIVER_NAME, host->controller_id);

				temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
				mmc_writew(host, ((temp_reg & ~HwSDCLKSEL_SCK_EN) & ~HwSDCLKSEL_INCLK_EN),
						TCCSDHC_CLOCK_CONTROL); // disable controller clock

				if(host->cd_irq != WIFI_POLLING) {
					if (gpio_is_valid(host->tcc_hw.vctrl_gpio)) {
						tcc_mmc_gpio_set_value(host->tcc_hw.vctrl_gpio, 1);
						printk("%s: [%s]tcc_mmc_gpio_set_value(host->tcc_hw.vctrl_gpio,1)\n", mmc_hostname(host->mmc), __func__);
					} // switch voltage level from 1.8 to 3.3
					if (gpio_is_valid(host->tcc_hw.pwr_gpio)) {
						tcc_mmc_gpio_set_value(host->tcc_hw.pwr_gpio, 0);
						printk("%s: [%s]tcc_mmc_gpio_set_value(host->tcc_hw.pwr_gpio,0)\n", mmc_hostname(host->mmc), __func__);
					} // power off
				}

				udelay(100);
				break;
		case MMC_POWER_UP:
//				printk("%s: [%s:MMC_POWER_UP]\n", mmc_hostname(host->mmc), __func__);
				dev_dbg(host->dev, "MMC_POWER_UP\n");

				if(host->cd_irq != WIFI_POLLING) {
					if (gpio_is_valid(host->tcc_hw.pwr_gpio)) {
						tcc_mmc_gpio_set_value(host->tcc_hw.pwr_gpio, 1);
						printk("%s: [%s]tcc_mmc_gpio_set_value(host->tcc_hw.pwr_gpio,1)\n", mmc_hostname(host->mmc), __func__);
					} // power on
				}

				if(host->controller_id == 3)
				{
					tcc_gpio_config(TCC_GPA(13), GPIO_FN(0)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(14), GPIO_FN(0)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(15), GPIO_FN(0)|GPIO_SCHMITT_INPUT);

					tcc_gpio_config(TCC_GPA(7), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(8), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(9), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(10), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(11), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
					tcc_gpio_config(TCC_GPA(12), GPIO_FN(3)|GPIO_SCHMITT_INPUT);
				}
				else
				{
					pinctrl = pinctrl_get_select(host->dev, "active");
					if(IS_ERR(pinctrl))
						SD_DEBUG("%s%d: pinctrl select failed\n", DRIVER_NAME, host->controller_id);
				}

				mdelay(100); // It is required because of the stablization of the voltage.

				init_mmc_host(host, mmc->f_max);
				break;
		case MMC_POWER_ON:
				dev_dbg(host->dev, "MMC_POWER_ON\n");
				break;
	}

	switch (ios->bus_width) {
		uint16_t cont_l, cont_h;

		case MMC_BUS_WIDTH_1:
			SD_DEBUG("%s%d: 1 bit mode\n", DRIVER_NAME, host->controller_id);
			cont_l = mmc_readw(host, TCCSDHC_HOST_CONTROL);
			cont_l &= ~(HwSD_POWER_SD4|HwSD_POWER_SD8);
			cont_l |= HwSD_POWER_POW|HwSD_POWER_VOL33;
			mmc_writew(host, cont_l, TCCSDHC_HOST_CONTROL);

			cont_h = mmc_readw(host, TCCSDHC_BLOCK_GAP_CONTROL);
			cont_h &= ~Hw3;
			mmc_writew(host, cont_h, TCCSDHC_BLOCK_GAP_CONTROL);
			break;
		case MMC_BUS_WIDTH_4:
			SD_DEBUG("%s%d: 4 bit mode\n", DRIVER_NAME, host->controller_id);
			cont_l = mmc_readw(host, TCCSDHC_HOST_CONTROL);
			cont_l &= ~HwSD_POWER_SD8;
			cont_l |= HwSD_POWER_POW|HwSD_POWER_VOL33|HwSD_POWER_SD4;
			mmc_writew(host, cont_l, TCCSDHC_HOST_CONTROL);

			cont_h = mmc_readw(host, TCCSDHC_BLOCK_GAP_CONTROL);
			/*cont_h |= Hw3;*/
			mmc_writew(host, cont_h, TCCSDHC_BLOCK_GAP_CONTROL);
			break;
		case MMC_BUS_WIDTH_8:
			SD_DEBUG("%s%d: 8 bit mode\n", DRIVER_NAME, host->controller_id);
			cont_l = mmc_readw(host, TCCSDHC_HOST_CONTROL);
			cont_l |= HwSD_POWER_POW|HwSD_POWER_VOL33|HwSD_POWER_SD8;
			mmc_writew(host, cont_l, TCCSDHC_HOST_CONTROL);

			cont_h = mmc_readw(host, TCCSDHC_BLOCK_GAP_CONTROL);
			cont_h &= ~Hw3;
			mmc_writew(host, cont_h, TCCSDHC_BLOCK_GAP_CONTROL);
			break;
	}

	host->bus_mode = ios->bus_mode;

	mmiowb();
}

static int tcc_mmc_get_ro(struct mmc_host *mmc)
{
	return mmc_gpio_get_ro(mmc);
}

static void tcc_hw_reset(struct mmc_host *mmc)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);

	if (!gpio_is_valid(host->tcc_hw.rst_gpio))
		return;

	printk("%s : eMMC hw reset \n", __func__);

	tcc_mmc_gpio_set_value(host->tcc_hw.rst_gpio, 0);
	mdelay(3);
	tcc_mmc_gpio_set_value(host->tcc_hw.rst_gpio, 1);
	mdelay(3);
}

static void tcc_sdio_hw_enable_int(struct mmc_host *mmc, uint32_t sigs)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	uint32_t stat_en;

	if((mmc == NULL)||(host == NULL)) {
		printk("[mmc:NULL] %s(mmc:%x, host:%x)\n", __func__, (u32)mmc, (u32)host);
		return;// -EHOSTDOWN;
	}

	pr_debug("%s id[%d] iobase[%p], sigs[%x]\n", __FUNCTION__, host->controller_id, host->base, sigs);

	spin_lock_irqsave(&host->lock, flags);

	stat_en=mmc_readl(host, TCCSDHC_INT_ENABLE);
	mmc_writel(host, stat_en | sigs, TCCSDHC_INT_ENABLE);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void tcc_sdio_hw_disable_int(struct mmc_host *mmc, uint32_t sigs)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	uint32_t stat_en;

	if((mmc == NULL)||(host == NULL)) {
		printk("[mmc:NULL] %s(mmc:%x, host:%x)\n", __func__, (u32)mmc, (u32)host);
		return;// -EHOSTDOWN;
	}

	pr_debug("%s id[%d] iobase[%p], sigs[%x]\n", __FUNCTION__, host->controller_id, host->base, sigs);
	spin_lock_irqsave(&host->lock, flags);

	stat_en = mmc_readl(host, TCCSDHC_INT_ENABLE);
	mmc_writel(host, stat_en & ~sigs, TCCSDHC_INT_ENABLE);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void tcc_sdio_enable_card_int(struct mmc_host *mmc)
{
	if(mmc == NULL) {
		printk("[mmc:NULL] %s(mmc:%x)\n", __func__, (u32)mmc);
		return;// -EHOSTDOWN;
	}

	tcc_sdio_hw_enable_int(mmc, HwSDINT_EN_CDINT);
}

static void tcc_sdio_disable_card_int(struct mmc_host *mmc)
{
	if(mmc == NULL) {
		printk("[mmc:NULL] %s(mmc:%x)\n", __func__, (u32)mmc);
		return;// -EHOSTDOWN;
	}

	tcc_sdio_hw_disable_int(mmc, HwSDINT_EN_CDINT);
}

static void tcc_mmc_enable_cd_irq(int cd_irq)
{
	if(!(cd_irq<0))
	  enable_irq(cd_irq);
}

static void tcc_mmc_disable_cd_irq(int cd_irq)
{
	if(!(cd_irq<0))
	  disable_irq_nosync(cd_irq);
}

static void tcc_mmc_check_status(struct tcc_mmc_host *host)
{
	unsigned int status = 0;
	unsigned int irq_type = 0;
	int ret = 0;

	if((host == NULL)||(host->mmc == NULL)) {
		printk("[mmc:NULL] %s(host:%x, host->mmc:%x)\n", __func__, (u32)host, (u32)(host->mmc));
		return;// -EHOSTDOWN;
	}

	dev_dbg(host->dev, "cd detect\n");
	status = tcc_mmc_get_cd(host->mmc);

	//printk("status : %d, card_inserted : %d\n", status, host->card_inserted);

	if(host->card_inserted == status)
	{
		//printk("card_inserted and status are same...cancel cd_irq\n");
		return;
	}

	if(host->card_inserted ^ status)
	{
		if(status) // insert
		{
			mmc_detect_change(host->mmc, msecs_to_jiffies(500));
		}
		else // remove
		{
			mmc_detect_change(host->mmc, 0);
		}
		host->card_inserted = status;

		irq_type = irqd_get_trigger_type(irq_get_irq_data(host->cd_irq));
		irq_type ^= (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW);
		ret = irq_set_irq_type(host->cd_irq, irq_type);

		if (ret)
		{
			printk("%s : error setting irq type\n", __func__);
		}
	}
}

/*
 * ISR for handling card removal
 */
static irqreturn_t tcc_mmc_cd_irq(int irq, void *dev_id)
{
	struct tcc_mmc_host *host = (struct tcc_mmc_host *)dev_id;
	unsigned long flags;

	if(host)
	{
		tcc_mmc_disable_cd_irq(host->cd_irq);

		spin_lock_irqsave(&host->lock, flags);

		tcc_mmc_check_status(host);

		spin_unlock_irqrestore(&host->lock, flags);

		tcc_mmc_enable_cd_irq(host->cd_irq);
	}

	return IRQ_HANDLED;
}

/*
 * Interrupt handling
 */
static void tcc_mmc_read_block_pio(struct tcc_mmc_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 scratch = 0;
	u8 *buf;

	blksize = host->data->blksz;
	chunk = 0;

	local_irq_save(flags);
	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);
		blksize -= len;
		host->sg_miter.consumed = len;
		buf = host->sg_miter.addr;

		while (len) {
			if (chunk == 0) {
				scratch = mmc_readl(host, TCCSDHC_BUFFER);
				chunk = 4;
			}

			*buf = scratch & 0xFF;

			buf++;
			scratch >>= 8;
			chunk--;
			len--;
		}
	}

	sg_miter_stop(&host->sg_miter);
	local_irq_restore(flags);
}

static void tcc_mmc_cmd_irq(struct tcc_mmc_host *host, u32 intmask)
{
	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	BUG_ON(intmask == 0);

	if (!host->cmd) {
		printk(KERN_ERR "%s: Got command interrupt 0x%08x even "
				"though no command operation was in progress.\n",
				mmc_hostname(host->mmc), (unsigned)intmask);
		return;
	}

	if (intmask & HwSDINT_STATUS_CMDTIME) {
		host->cmd->error = -ETIMEDOUT;
	} else if (intmask & (HwSDINT_STATUS_CMDCRC | HwSDINT_STATUS_CMDEND | HwSDINT_STATUS_CINDEX)) {
		host->cmd->error = -EILSEQ;
	}

	if (host->cmd->error) {
		tasklet_schedule(&host->finish_tasklet);
	} else if (intmask & HwSDINT_STATUS_CDONE) {
		tcc_mmc_finish_command(host);
	}
}

static void tcc_mmc_data_irq(struct tcc_mmc_host *host, u32 intmask)
{
	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	BUG_ON(intmask == 0);

	if (!host->data) {
		/*
		 * A data end interrupt is sent together with the response
		 * for the stop command.
		 */
		if (intmask & HwSDINT_STATUS_TDONE)
			return;

		printk(KERN_ERR "%s: Got data interrupt 0x%08x even "
				"though no data operation was in progress.\n",
				mmc_hostname(host->mmc), (unsigned)intmask);
		return;
	}

	if (intmask & HwSDINT_STATUS_DATTIME) {
		host->data->error = -ETIMEDOUT;
	} else if (intmask & (HwSDINT_STATUS_DATCRC | HwSDINT_STATUS_DATEND)) {
		host->data->error = -EILSEQ;
	} else if (intmask & HwSDINT_STATUS_ADMA) {
		host->data->error = -EIO;
	}

	if (host->data->error) {
		tcc_mmc_finish_data(host);
	} else {
		if (intmask & (HwSDINT_STATUS_RDRDY | HwSDINT_STATUS_WRRDY)) {
			/*printk(" \n %x \n ", intmask);*/
			tcc_transfer_pio(host);
		}

		/*
		 * We currently don't do anything fancy with DMA
		 * boundaries, but as we can't disable the feature
		 * we need to at least restart the transfer.
		 */
		//if (intmask & HwSDINT_STATUS_DMA){
		//	mmc_writel(host, mmc_readl(host, TCCSDHC_DMA_ADDRESS), TCCSDHC_DMA_ADDRESS);
		//}

		if (intmask & HwSDINT_STATUS_TDONE) {
			if (host->cmd) {
				/*
				 * Data managed to finish before the
				 * command completed. Make sure we do
				 * things in the proper order.
				 */
				host->data_early = 1;
			} else {
				tcc_mmc_finish_data(host);
			}
		}
	}
}

static irqreturn_t tcc_mmc_interrupt_handler(int irq, void *dev_id)
{
	unsigned int IntStatus;
	irqreturn_t result;
	int cardint = 0;
	struct tcc_mmc_host *host = (struct tcc_mmc_host *) dev_id;
	unsigned long flags;
	int cmd = 0;

	unsigned int errstat;

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return IRQ_HANDLED;// -EHOSTDOWN;
	}

	spin_lock_irqsave(&host->lock, flags);

	IntStatus = mmc_readl(host, TCCSDHC_INT_STATUS);

	if (IntStatus == 0  || IntStatus == 0xffffffff ) {
		result = IRQ_NONE;
		goto out;
	}

#if 0
	if (IntStatus & TCCSDHC_INT_ERR)
	{
		printk("[Interrupt Error] ");
		printk("Normal Interrupt : 0x%08x ", IntStatus);
		printk("ACMD12 : 0x%08x\n",
				mmc_readl(host, TCCSDHC_ACMD12_ERR));
		printk(" Command : 0x%08x ",
				mmc_readl(host, TCCSDHC_TRANSFER_MODE));
		printk(" respons 1/0 : 0x%08x\n", 
				mmc_readl(host, TCCSDHC_RESPONSE10));
	}
#endif

	if(IntStatus & HwSDINT_EN_RDRDY) {
		cmd = TCC_MMC_GET_CMD(mmc_readl(host, TCCSDHC_TMODE_COM));
		if (cmd == MMC_SEND_TUNING_BLOCK ||
				cmd == MMC_SEND_TUNING_BLOCK_HS200) {
			tcc_mmc_read_block_pio(host);
			tcc_sw_reset(host, HwSD_SRESET_RSTCMD);
			tcc_sw_reset(host, HwSD_SRESET_RSTDAT);
			host->tuning_done = 1;
			wake_up(&host->buf_ready_int);
		}
	}

	if (IntStatus & HwSDINT_CMD_MASK) {
		mmc_writel(host, IntStatus & HwSDINT_CMD_MASK,
			   TCCSDHC_INT_STATUS);
		tcc_mmc_cmd_irq(host, IntStatus & HwSDINT_CMD_MASK);
	}

	if (IntStatus & HwSDINT_DATA_MASK) {
		mmc_writel(host, IntStatus & HwSDINT_DATA_MASK,
			   TCCSDHC_INT_STATUS);
		tcc_mmc_data_irq(host, IntStatus & HwSDINT_DATA_MASK);
	}

	IntStatus &= ~(HwSDINT_CMD_MASK |HwSDINT_DATA_MASK);
	IntStatus &= ~HwSDINT_STATUS_ERR;

	if(IntStatus & HwSDINT_STATUS_CDINT) {
		cardint =1;
	}

	IntStatus &= ~ HwSDINT_STATUS_CDINT;

	if (IntStatus) {

		SD_DEBUG(KERN_ERR "%s: Unexpected interrupt 0x%08x.\n", mmc_hostname(host->mmc), IntStatus);

		if(IntStatus & HwSDINT_STATUS_ACMD){
			errstat = mmc_readl(host, TCCSDHC_ACMD12_ERR); // clear AMCD error status register 
		}

		mmc_writel(host, IntStatus, TCCSDHC_INT_STATUS);
	}

	result = IRQ_HANDLED;

	mmiowb();

out:
	spin_unlock_irqrestore(&host->lock, flags);

	if (cardint) {
		mmc_signal_sdio_irq(host->mmc);
	}

	return result;
}

static void tcc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	if (enable) {
		tcc_sdio_enable_card_int(mmc);
	} else {
		tcc_sdio_disable_card_int(mmc);
	}
}
#if defined(CONFIG_ARCH_TCC898X)
static int tcc_mmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);

	disable_irq(host->irq);
	spin_lock(&host->lock);

	printk("[%s] TCC898X_rev0 can't execute auto tuning.\n", __func__);
	printk("[%s] Manual tuning is required.\n", __func__);

	//writel(0x00000000, host->chctrl_base + TCCSDHC_CHCTRL_DLY0);
	//writel(0x00000000, host->chctrl_base + TCCSDHC_CHCTRL_DLY1);
	//writel(0x00000000, host->chctrl_base + TCCSDHC_CHCTRL_DLY2);
	//writel(0x00000000, host->chctrl_base + TCCSDHC_CHCTRL_DLY3);
	//writel(0x00000000, host->chctrl_base + TCCSDHC_CHCTRL_DLY4);

	writel(0x00206100, host->chctrl_base + TCCSDHC_CHCTRL_TAPDLY);

	printk("[%s] TABDLY (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_TAPDLY));

	printk("[%s] DLY0 (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_DLY0));
	printk("[%s] DLY1 (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_DLY1));
	printk("[%s] DLY2 (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_DLY2));
	printk("[%s] DLY3 (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_DLY3));
	printk("[%s] DLY4 (0x%08x)\n", __func__, readl(host->chctrl_base + TCCSDHC_CHCTRL_DLY4));

	spin_unlock(&host->lock);
	enable_irq(host->irq);

	return 0;
}
#else

#define MAX_LOOP 40
static int tcc_mmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	uint32_t reg = 0, tuning_status = 0;
	int err = 0, tuning_count = 0, success_count = 15;
	unsigned int timeout = 0;
	int lowest_delay = 0, highest_delay = 0, avg_delay = 0;
	u8 blksz = 0;
	u8 *blocks;
	struct mmc_command cmd = {0};
	struct mmc_request mrq = {NULL};
	struct mmc_data data = {0};
	struct scatterlist sg;
	unsigned int chctrl_delay = 0;
	const u8 *tuning_data;

	disable_irq(host->irq);
	spin_lock(&host->lock);

	switch (opcode) {
		case MMC_SEND_TUNING_BLOCK_HS200:
			blksz = sizeof(tuning_blk_pattern_8bit);
			tuning_data = tuning_blk_pattern_8bit;
			break;
		case MMC_SEND_TUNING_BLOCK:
			blksz = sizeof(tuning_blk_pattern_4bit);
			tuning_data = tuning_blk_pattern_4bit;
			break;
		default:
			spin_unlock(&host->lock);
			enable_irq(host->irq);
			return 0;
	}

	host->is_in_tuning_mode = true;

#if !defined(CONFIG_ARCH_TCC898X)
	chctrl_delay = (unsigned int)TCCSDMMC_CHCTRL_SD01DELAY + (0x2 * host->controller_id);
#elif defined(CONFIG_ARCH_TCC898X)
	chctrl_delay = 0;
#endif

	reg = readl(host->chctrl_base + chctrl_delay);
	reg &= 0xFFFF0000;
	reg |= (success_count << 8)|(1 << 7); //|(1<<15);
	writel(reg, host->chctrl_base + chctrl_delay);
	printk("[%s] Init tap delay(0x%08x)\n", __func__, readl(host->chctrl_base + chctrl_delay));

	reg = mmc_readl(host, TCCSDHC_ACMD12_ERR);
	reg |= TCCSDHC_EXEC_TUNING;
	mmc_writel(host, reg, TCCSDHC_ACMD12_ERR);
	printk("[%s] Initial Host Control2 Reg(0x%08x)\n",
			__func__, mmc_readl(host, TCCSDHC_ACMD12_ERR));

	// Enable Buffer Read Ready bit
	reg = mmc_readl(host, TCCSDHC_INT_ENABLE);
	mmc_writel(host, reg | TCCSDHC_INT_DATA_AVAIL, TCCSDHC_INT_ENABLE);
	reg = mmc_readl(host, TCCSDHC_SIGNAL_ENABLE);
	mmc_writel(host, reg | TCCSDHC_INT_DATA_AVAIL, TCCSDHC_SIGNAL_ENABLE);

	printk("[%s] Initial Interrupt Enable Reg(0x%08x)\n",
			__func__, mmc_readl(host, TCCSDHC_INT_ENABLE));

	timeout = 150;
	do {
		tuning_status = 0;

		/* prepare tuning command */
		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.retries = 0;
		cmd.data = NULL;
		cmd.data = &data;
		cmd.error = 0;

		data.blksz = blksz;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.sg = &sg;
		data.sg_len = 1;

		blocks = kmalloc(blksz, GFP_KERNEL);
		sg_init_one(&sg, blocks, blksz);

		mrq.cmd = &cmd;
		mrq.data = &data;
		host->mrq = &mrq;

		/* issue tuning command */
		tcc_mmc_start_command(host, &cmd);

		spin_unlock(&host->lock);
		enable_irq(host->irq);

		wait_event_interruptible_timeout(host->buf_ready_int,
				(host->tuning_done == 1),
				msecs_to_jiffies(150));

		disable_irq(host->irq);
		spin_lock(&host->lock);

		if (!memcmp(blocks, tuning_data, blksz))  {
			highest_delay = tuning_count;
			if (lowest_delay == 0)
				lowest_delay = tuning_count;
		} else {
			asm ("nop");
		}

		if(!host->tuning_done) {
			printk("[%s] Timeout waiting for "
					"Buffer Read Ready interrupt during tuninig "
					"procedrue, falling back to fixed sampling "
					"clock\n", __func__);
			err = 0;
			host->cmd = NULL;
			host->mrq = NULL;
			host->data = NULL;
			goto out;
		}

		tuning_status = mmc_readl(host, TCCSDHC_ACMD12_ERR);

		host->cmd = NULL;
		host->mrq = NULL;
		host->tuning_done = 0;
		tuning_count++;

		//if(opcode == MMC_SEND_TUNING_BLOCK)
			//mdelay(1);
	} while (tuning_status & 0x00400000);

out:
	avg_delay = (lowest_delay + highest_delay) / 2;
	printk("[%s] avg_delay(%d), lowest_delay(%d), highest_delay(%d)\n",
			__func__, avg_delay, lowest_delay, highest_delay);
	reg = readl(host->chctrl_base + chctrl_delay);
	reg &= ~(0x3f);
	reg |= avg_delay;

	writel(reg, host->chctrl_base + chctrl_delay);
	printk("[%s] tap delay(0x%08x)\n", __func__, readl(host->chctrl_base + chctrl_delay));

	printk("[%s] Host Control2(0x%08x)\n", __func__, mmc_readl(host, TCCSDHC_ACMD12_ERR));

	host->is_in_tuning_mode = false;

	spin_unlock(&host->lock);
	enable_irq(host->irq);

	return err;
}
#endif

static int get_sdr_mode(int caps, int caps2)
{
	if (caps & MMC_CAP_UHS_SDR12) {
		printk("[mmc] MMC_CAP_UHS_SDR12 is enabled.\n");
		return 0x00080000;
	}
	if (caps & MMC_CAP_UHS_SDR25) {
		printk("[mmc] MMC_CAP_UHS_SDR25 is enabled.\n");
		return 0x00190000;
	}
	if (caps & MMC_CAP_UHS_SDR50) {
		printk("[mmc] MMC_CAP_UHS_SDR50 is enabled.\n");
		return 0x002A0000;
	}
	if (caps & MMC_CAP_UHS_SDR104) {
		printk("[mmc] MMC_CAP_UHS_SDR104 is enabled.\n");
		return 0x000B0000;
	}
	if (caps & MMC_CAP_UHS_DDR50) {
		/* In case of DDR50 for SD UHS-I card,
		 * just enable 1.8v signaling bit here.
		 * UHS mode has to be enabled when DDR timing(MMC_TIMING_UHS_DDR50) is started.
		 */
		printk("[mmc] MMC_CAP_UHS_DDR50 is enabled.\n");
		return 0x00080000;
	}
	if (caps2 & MMC_CAP2_HS200) {
		printk("[mmc] MMC_CAP2_HS200 is enabled.\n");
		return 0x000B0000;
	}
	printk("[mmc] Illegal MMC_CAP_UHS_XXX is set.");

	return 0;
}

static int tcc_mmc_start_signal_voltage_switch(struct mmc_host *mmc,
		struct mmc_ios *ios)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);
	uint32_t temp_reg, temp_val;
	long dwMaxClockRate = host->peri_clk; 
	int i=0;
	u32 clk_div = 0;

	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_180)
		return 0;
	//printk("%s: ios->signal_voltage(%d), ios->clock(%d)\n",
		//__func__, ios->signal_voltage, ios->clock);

	temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
	temp_reg &= ~HwSDCLKSEL_SCK_EN;
	mmc_writew(host, temp_reg, TCCSDHC_CLOCK_CONTROL);

	temp_val = mmc_readw(host, TCCSDHC_HOST_CONTROL);
	temp_val &= ~HwSD_POWER_POW;
	mmc_writew(host, temp_val, TCCSDHC_HOST_CONTROL);

	ios->clock = 400000;
	if (ios->clock != 0) 
	{
		temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
		mmc_writew(host, temp_reg & (~HwSDCLKSEL_SCK_EN | ~HwSDCLKSEL_INCLK_EN),
			   TCCSDHC_CLOCK_CONTROL);

		if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) 
		{
			mmc_writel(host, get_sdr_mode(mmc->caps, mmc->caps2),
					TCCSDHC_ACMD12_ERR);
            if(gpio_is_valid(host->tcc_hw.vctrl_gpio)) {
    			tcc_mmc_gpio_set_value(host->tcc_hw.vctrl_gpio, 0);
            }
			mdelay(15); // [Timing] Host keeps SDCLK low at least 5 ms.
		}

		/*tcc_hw_set_high_speed(mmc, 1);*/
		/* shift MaxClockRate until we find the closest frequency <= target */
		//while ((ios->clock < dwMaxClockRate) && (i < 8)) {
		while ((ios->clock < dwMaxClockRate)) {
			dwMaxClockRate = dwMaxClockRate >> 1;
			i++;
		}

		if(i==0)
			clk_div = 0;
		else
			clk_div = 1<<(i-1);

		host->clk_div = clk_div;	/* store divider */

 		if(clk_div <= 128)
		{
			mmc_writew(host, (clk_div << 8) | HwSDCLKSEL_INCLK_EN,
				TCCSDHC_CLOCK_CONTROL);
		}
		else // use extend SDCLKSEL register
		{
			if(clk_div > 1023)
				host->clk_div = clk_div = 1023;

			mmc_writew(host, ((clk_div & 0xFF) << 8) | ((clk_div & 0x300) >> 2) | HwSDCLKSEL_INCLK_EN,
				TCCSDHC_CLOCK_CONTROL);
		}
		//mmc_writew(host, (clk_div << 8) | HwSDCLKSEL_INCLK_EN,
			   //TCCSDHC_CLOCK_CONTROL);

		while (!(mmc_readl(host, TCCSDHC_CLOCK_CONTROL) &
				HwSDCLKSEL_INCLK_STABLE)) {
			udelay(100);
		}
 	
		temp_reg = mmc_readl(host, TCCSDHC_CLOCK_CONTROL);
		mmc_writew(host, temp_reg | (HwSDCLKSEL_SCK_EN),
			   TCCSDHC_CLOCK_CONTROL);
		udelay(100);
	}

	temp_val = mmc_readw(host, TCCSDHC_HOST_CONTROL);
	temp_val |= HwSD_POWER_POW;
	mmc_writew(host, temp_val | HwSD_POWER_POW, TCCSDHC_HOST_CONTROL);

	return 0;
}

int tcc_mmc_select_drive_strength(unsigned int max_dtr, int host_drv,
		int card_drv)
{
	/*
	 * #define SD_DRIVER_TYPE_B	0x01
	 * #define SD_DRIVER_TYPE_A	0x02
	 * #define SD_DRIVER_TYPE_C	0x04
	 * #define SD_DRIVER_TYPE_D	0x08
	 *
	 * Return Value for Drive Strength
	 * SDR12 - Default /Type B - Func 0x0
	 * SDR25 - High-Speed / Type B - Func 0x1
	 * SDR50 - Type C - Func 0x2
	 * SDR104 - Type D - Func 0x3
	 */

	int drv_caps = host_drv & card_drv;

	if (drv_caps & SD_DRIVER_TYPE_D) {
		printk("[mmc] %s, SD_DRIVER_TYPED_D is supported.\n", __func__);
		return 0x3;
	}
	if (drv_caps & SD_DRIVER_TYPE_C) {
		printk("[mmc] %s, SD_DRIVER_TYPED_C is supported.\n", __func__);
		return 0x2;
	}
	if (drv_caps & SD_DRIVER_TYPE_A) {
		printk("[mmc] %s, SD_DRIVER_TYPED_A is supported.\n", __func__);
		return 0x1;
	}
	if (drv_caps & SD_DRIVER_TYPE_B) {
		printk("[mmc] %s, SD_DRIVER_TYPED_B is supported.\n", __func__);
		return 0x0;
	}
	printk("[mmc] %s, Illegal DRIVER_TYPE is set.\n", __func__);

	return 0;
}

static int tcc_mmc_card_busy(struct mmc_host *mmc)
{
	struct tcc_mmc_host *host = mmc_priv(mmc);

	u32 present_state;
	present_state = mmc_readl(host, TCCSDHC_PRESENT_STATE);

	return !(present_state & 0x00F00000);
}

static struct mmc_host_ops tcc_mmc_ops = {
	.request		= tcc_mmc_request,
	.set_ios		= tcc_mmc_set_ios,
	.get_ro			= tcc_mmc_get_ro,
	.get_cd			= tcc_mmc_get_cd,
	.hw_reset		= tcc_hw_reset,
	.enable_sdio_irq	= tcc_enable_sdio_irq,
	.start_signal_voltage_switch = tcc_mmc_start_signal_voltage_switch,
	.execute_tuning = tcc_mmc_execute_tuning,
	.select_drive_strength = tcc_mmc_select_drive_strength,
	.card_busy = tcc_mmc_card_busy,
};

/*
 * Tasklets
 */
static void tcc_mmc_tasklet_finish(unsigned long param)
{
	struct tcc_mmc_host *host;
	unsigned long flags;
	struct mmc_request *mrq;

	host = (struct tcc_mmc_host *)param;

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	spin_lock_irqsave(&host->lock, flags);

	if(!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if (mrq->cmd->error ||
			(mrq->data && (mrq->data->error ||
						   (mrq->data->stop && mrq->data->stop->error)))) {

		/* Spec says we should do both at the same time, but Ricoh
		   controllers do not like that. */
		tcc_sw_reset(host, HwSD_SRESET_RSTCMD);
		tcc_sw_reset(host, HwSD_SRESET_RSTDAT);
	}


	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	mmiowb();

	/* reset is_direct */
	host->is_direct = 0;

	if (host->is_clock_control)
		tcc_mmc_clock_control(host, 0);

	spin_unlock_irqrestore(&host->lock, flags);
	mmc_request_done(host->mmc, mrq);
}

static void tcc_mmc_timeout_timer(unsigned long data)
{
	struct tcc_mmc_host *host;
	unsigned long flags;

	host = (struct tcc_mmc_host*)data;

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}

	spin_lock_irqsave(&host->lock, flags);

	if (host->mrq) {
		printk(KERN_ERR "%s: Timeout waiting for hardware "
				"interrupt.\n", mmc_hostname(host->mmc));

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			tcc_mmc_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			tasklet_schedule(&host->finish_tasklet);
		}
	}
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void init_mmc_host(struct tcc_mmc_host *host,
		unsigned int clock_rate)
{
	unsigned int temp_val = 0;

	//printk("%s : channel_pa(0x%08x) channel_va(0x%08x)\n",
		//__func__, TCCSDMMC_PA_CHCTRL, tcc_p2v(TCCSDMMC_PA_CHCTRL));
	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return;// -EHOSTDOWN;
	}
	if (host->fclk > 0)
		clk_set_rate(host->fclk, clock_rate);
	else
		clk_set_rate(host->fclk, 25*1000*1000);
	host->peri_clk = clk_get_rate(host->fclk);
	dev_dbg(host->dev, "clock %lu\n", host->peri_clk);

#if 0
	mmc_writel(host, 0x03ff00ff, TCCSDHC_INT_ENABLE);
	mmc_writel(host, 0x03ff01ff, TCCSDHC_SIGNAL_ENABLE);
#else
	mmc_writel(host, 0xfffff0ff, TCCSDHC_INT_ENABLE);
	mmc_writel(host, 0xffffffff, TCCSDHC_SIGNAL_ENABLE);
#endif

	temp_val= mmc_readw(host, TCCSDHC_TIMEOUT_CONTROL)&(0xFF00);
	mmc_writew(host, temp_val | (0x000E), TCCSDHC_TIMEOUT_CONTROL);

	temp_val = mmc_readw(host, TCCSDHC_HOST_CONTROL);
	temp_val &= ~(HwSD_POWER_SD4 | HwSD_POWER_SD8);
	mmc_writew(host, temp_val, TCCSDHC_HOST_CONTROL);
	mmc_writew(host, temp_val | HwSD_POWER_POW, TCCSDHC_HOST_CONTROL);

	mmc_writel(host, 0x00000000, TCCSDHC_ACMD12_ERR);

#if !defined(CONFIG_ARCH_TCC898X)
	temp_val = readl(host->chctrl_base + TCCSDMMC_CHCTRL_SDCTRL);
	temp_val |= 0x00F00000; // disable command conflict
	writel(temp_val, host->chctrl_base + TCCSDMMC_CHCTRL_SDCTRL);

	temp_val = (host->mmc->caps & 
			(MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104)) ?
		0x07070707: 0x0F0F0F0F;
	switch (host->controller_id % 4) 
	{
		case 0:
			writel(0x0F0F0F0F, host->chctrl_base + TCCSDMMC_CHCTRL_SD0CMDDAT);
			writel(0x2CFF9870, host->chctrl_base + TCCSDMMC_CHCTRL_SD0CAPREG0);
			writel(0x00000007, host->chctrl_base + TCCSDMMC_CHCTRL_SD0CAPREG1);

			// DELAY_CON0 manual control
			temp_val = readl(host->chctrl_base + TCCSDMMC_CHCTRL_SD01DELAY);
			if((host->mmc->caps & MMC_CAP_UHS_SDR50) || (host->mmc->caps & MMC_CAP_UHS_SDR104))
			{
				temp_val = 0x000080A4; // SDR MODE: FBEN0(1), DLYCTRL0(0), ITAPEN0(1), IPTAP0(36)
//				printk("[pigfish_debug] %s SDR MODE: set values temp_val[%08x]\n", __func__, temp_val);
			}
			else if(host->mmc->caps & MMC_CAP_UHS_DDR50)
			{
				temp_val = 0x0000808F; // DDR MODE: FBEN0(1), DLYCTRL0(0), ITAPEN0(1), IPTAP0(15)
//				printk("[pigfish_debug] %s DDR MODE: set values temp_val[%08x]\n", __func__, temp_val);
			}
			writel(temp_val, host->chctrl_base + TCCSDMMC_CHCTRL_SD01DELAY);
			temp_val = mmc_readl(host, TCCSDHC_ACMD12_ERR);
			temp_val &= 0xFF3FFFFF; // Disable SCLK, ETUNE bit
			mmc_writel(host, temp_val, TCCSDHC_ACMD12_ERR);
			//DELAY_CON0 manual control

			break;
		case 1:
			writel(temp_val, host->chctrl_base + TCCSDMMC_CHCTRL_SD1CMDDAT);
			writel(0xEDFF9870, host->chctrl_base + TCCSDMMC_CHCTRL_SD1CAPREG0);
			writel(0x00000007, host->chctrl_base + TCCSDMMC_CHCTRL_SD1CAPREG1);
			break;
		case 2:
			writel(0x00000F0F, host->chctrl_base + TCCSDMMC_CHCTRL_SD2CMDDAT);
			writel(0x0CFF9870, host->chctrl_base + TCCSDMMC_CHCTRL_SD2CAPREG0);
			writel(0x00000007, host->chctrl_base + TCCSDMMC_CHCTRL_SD2CAPREG1);
			break;
		case 3:
			writel(0x3F3F3F3F, host->chctrl_base + TCCSDMMC_CHCTRL_SD3CMDDAT);
			writel(0x0CFF9870, host->chctrl_base + TCCSDMMC_CHCTRL_SD3CAPREG0);
			writel(0x00000007, host->chctrl_base + TCCSDMMC_CHCTRL_SD3CAPREG1);
			writel(0x00940094, host->chctrl_base + TCCSDMMC_CHCTRL_SD23DELAY);
			break;
		default:
			printk("%s: failed to init mmc channel configuration...\n", __func__);
			break;
	}
#elif defined(CONFIG_ARCH_TCC898X)
	// It is need to add delay
	writel(0xEDFF9970, host->chctrl_base + TCCSDHC_CHCTRL_CAP0);
	writel(0x00000007, host->chctrl_base + TCCSDHC_CHCTRL_CAP1);
#endif
}

static int tcc_mmc_parse_dt(struct tcc_mmc_host *host, struct device_node *np)
{
	int ret=0;
	int len=0;
	enum of_gpio_flags flags;

	mmc_of_parse(host->mmc);

	if (of_find_property(np, "cap-erase", &len))
		host->mmc->caps |= MMC_CAP_ERASE;
	if (of_find_property(np, "cap-bus-width-test", &len))
		host->mmc->caps |= MMC_CAP_BUS_WIDTH_TEST;
	if (of_find_property(np, "cap-uhs-cmd23", &len))
		host->mmc->caps |= MMC_CAP_CMD23;

	if (of_find_property(np, "cap-1-8V-ddr", &len))
	{
//		printk("[%s:pigfish debug] SET cap-1-8V-ddr\n", __func__);
		host->mmc->caps |= MMC_CAP_1_8V_DDR;
	}

	if (of_find_property(np, "cap-uhs-sdr12", &len))
		host->mmc->caps |= MMC_CAP_UHS_SDR12;
	if (of_find_property(np, "cap-uhs-sdr25", &len))
		host->mmc->caps |= MMC_CAP_UHS_SDR25;
	if (of_find_property(np, "cap-uhs-sdr50", &len))
		host->mmc->caps |= MMC_CAP_UHS_SDR50;
	if (of_find_property(np, "cap-uhs-sdr104", &len))
		host->mmc->caps |= MMC_CAP_UHS_SDR104;
	if (of_find_property(np, "cap-uhs-ddr50", &len))
	{
//		printk("[%s:pigfish debug] SET cap-uhs-ddr50\n", __func__);
		host->mmc->caps |= MMC_CAP_UHS_DDR50;
	}
	if (of_find_property(np, "mmc-hs200-1_8v", &len))
		host->mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR;

	if (of_find_property(np, "cap-uhs-driver_type_a", &len))
		host->mmc->caps |= MMC_CAP_DRIVER_TYPE_A;
	if (of_find_property(np, "cap-uhs-driver_type_c", &len))
		host->mmc->caps |= MMC_CAP_DRIVER_TYPE_C;
	if (of_find_property(np, "cap-uhs-driver_type_d", &len))
		host->mmc->caps |= MMC_CAP_DRIVER_TYPE_D;
	
	if (of_find_property(np, "cap-vdd-165-195", &len))
		host->mmc->ocr_avail |= MMC_VDD_165_195;

	/*
	if (of_find_property(np, "cap2-no-sleep-cmd", &len))
		host->mmc->caps2 |= MMC_CAP2_NO_SLEEP_CMD;
	*/

#if defined(CONFIG_ENABLE_TCC_MMC_KPANIC)
	if (of_find_property(np, "tcc-kpanic", NULL)) {
		host->is_kpanic_card = true;
		init_tcc_mmc_kpanic();
	} else {
		host->is_kpanic_card = false;
	}
#endif

	if (of_find_property(np, "tcc-clock-control", NULL))
		host->is_clock_control = true;
	else
		host->is_clock_control = false;

	dev_info(host->dev, "is_clock_control %s\n",
			 host->is_clock_control ? "true" : "false");

	host->tcc_hw.vctrl_gpio = of_get_named_gpio_flags(np, "vctrl-gpio", 0, &flags);
	if (gpio_is_valid(host->tcc_hw.vctrl_gpio)) {
		ret = devm_gpio_request_one(&host->mmc->class_dev,
				host->tcc_hw.vctrl_gpio, 
				GPIOF_OUT_INIT_HIGH,
				"tcc_sdhc: voltage control gpio");
		if (ret < 0)
			printk("vctrl error!\n");
	}
	
	host->tcc_hw.pwr_gpio = of_get_named_gpio_flags(np, "pwr-gpio", 0, &flags);
	// print pwr_gpio information only SDCard(SdCard only has power control, others does not.)
	if(host->mmc->index == 2)
		printk("%s: [%s]host->tcc_hw.pwr_gpio(%u)\n", mmc_hostname(host->mmc), __func__, host->tcc_hw.pwr_gpio);
	if (gpio_is_valid(host->tcc_hw.pwr_gpio)) {
		tcc_mmc_gpio_set_value(host->tcc_hw.pwr_gpio, 0);

		mdelay(100); // for hardware reset

		ret = devm_gpio_request_one(&host->mmc->class_dev,
				host->tcc_hw.pwr_gpio, 
				GPIOF_OUT_INIT_HIGH,
				"tcc_sdhc: power control gpio");
		if (ret < 0)
			printk("pwr error!\n");
	}

	host->tcc_hw.rst_gpio = of_get_named_gpio_flags(np, "emmc-rst-gpio", 0, &flags);
	if (gpio_is_valid(host->tcc_hw.rst_gpio)) {
		host->mmc->caps |= MMC_CAP_HW_RESET;

		tcc_mmc_gpio_set_value(host->tcc_hw.rst_gpio, 0);

		mdelay(100); // for hardware reset

		ret = devm_gpio_request_one(&host->mmc->class_dev,
				host->tcc_hw.rst_gpio, 
				GPIOF_OUT_INIT_HIGH,
				"tcc_sdhc: reset control gpio");
		if (ret < 0)
			printk("rst error!\n");
	}

	return 0;
}

static int tcc_mmc_probe(struct platform_device *pdev)
{       
	struct mmc_host *mmc;
	struct tcc_mmc_host *host = NULL;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct resource res;
	int ret = 0;

	match = of_match_device(tcc_mmc_of_match, &pdev->dev);
	if (!match) {
		return -EINVAL;
	}

	mmc = mmc_alloc_host(sizeof(struct tcc_mmc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return -ENODEV;// -EHOSTDOWN;
	}

	host->mmc = mmc;
	host->dev = &pdev->dev;

	if (of_property_read_u32(np, "controller-id", &host->controller_id) < 0) {
		printk("\"conrtoller-id\" property is missing...\n");
		goto error;
	}

	if (tcc_mmc_parse_dt(host, np))
		goto error;


	host->fclk = of_clk_get(np, 0);
	if (!host->fclk) {
		printk("%s: SDHC core %d can't get mmc peri clock\n",DRIVER_NAME, host->controller_id);
		goto error;
	}
	host->hclk = of_clk_get(np, 1);
	if (!host->hclk) {
		printk("%s: SDHC core %d can't get mmc hclk clock\n",DRIVER_NAME, host->controller_id);
		goto error;
	}
	clk_prepare_enable(host->fclk);
	clk_prepare_enable(host->hclk);

	/* direct path from FSG */
	host->is_direct = 0;

	host->base = of_iomap(np, 0);
	of_address_to_resource(np,0,&res);
	if (!host->base) {
		dev_err(&pdev->dev, "failed to get sdhc base address\n");
		return -ENOMEM;
	}
//	printk("%s : host[%d]->base(0x%08x/0x%08x)\n",
//			__func__, host->controller_id, (int)host->base, (unsigned int)res.start); 

	host->chctrl_base = of_iomap(np, 1);
	of_address_to_resource(np,1,&res);
	if (!host->chctrl_base) {
		dev_err(&pdev->dev, "failed to get channel control base address\n");
		return -ENOMEM;
	}
//	printk("%s : host[%d]->chctrl_base(0x%08x/0x%08x)\n",
//			__func__, host->controller_id, (int)host->chctrl_base, (unsigned int)res.start);

	host->irq = platform_get_irq(pdev, 0);
	host->cd_irq = host->mmc->slot.cd_irq;

	if (of_find_property(np, "cd-polling", 0))
	{
		host->cd_irq = -1;
	} // only polling mode

	if (of_find_property(np, "wifi-polling", 0))
	{
		host->cd_irq = host->mmc->slot.cd_irq = WIFI_POLLING; // tcm3800
		if (of_find_property(np, "ext32-enable", 0))
		{
			make_rtc(np);
		}
	}
	
//	printk("%s:%d host->cd_irq(%d)\n", __func__, host->controller_id, host->cd_irq);
	host->flags =0;

	host->flags = TCC_MMC_USE_DMA;
	if (of_find_property(np, "cap-adma", 0))
	{
		host->flags |= TCC_MMC_USE_ADMA;
		//host->flags |= HwSD_COM_TRANS_ACMD23;
	}

	mmc->ops = &tcc_mmc_ops;
	mmc->f_min = 100000;
	mmc->ocr_avail |= MMC_VDD_32_33 | MMC_VDD_33_34;

	if (host->flags & TCC_MMC_USE_ADMA) {
		/*
		 * We need to allocate descriptors for all sg entries
		 * (128) and potentially one alignment transfer for
		 * each of those entries.
		 */
		host->adma_desc = kmalloc((128 * 2 + 1) * 4, GFP_KERNEL);
		host->align_buffer = kmalloc(128 * 4, GFP_KERNEL);
		if (!host->adma_desc || !host->align_buffer) {
			kfree(host->adma_desc);
			kfree(host->align_buffer);
			printk(KERN_WARNING "%s: Unable to allocate ADMA "
					"buffers. Falling back to standard DMA.\n",
					mmc_hostname(mmc));
			host->flags &= ~TCC_MMC_USE_ADMA;
		}
	}

	/*
	 * Maximum number of segments. Hardware cannot do scatter lists.
	 */
	if (host->flags & TCC_MMC_USE_DMA) {
		if(host->flags & TCC_MMC_USE_ADMA) {
			mmc->max_segs = 128;
		} else {
			mmc->max_segs = 1;
		}
	} else {
		mmc->max_segs = 128;
	}

	/*
	 * Maximum number of sectors in one transfer. Limited by DMA boundary
	 * size (512KiB).
	 */
	mmc->max_req_size = 524288;

	/*
	 * Maximum segment size. Could be one segment with the maximum number
	 * of bytes.
	 */
	if (host->flags & TCC_MMC_USE_ADMA) {
		mmc->max_seg_size = 65536;
		//mmc->max_seg_size = 524288;
	} else {
		mmc->max_seg_size = mmc->max_req_size;
	}

	/*
	 * Maximum block size. This varies from controller to controller and
	 * is specified in the capabilities register.
	 */
	mmc->max_blk_size = 512;

	/*
	 * Maximum block count.
	 */
	mmc->max_blk_count = 65535;

	platform_set_drvdata(pdev, host);

	spin_lock_init(&host->lock);

	init_mmc_host(host, mmc->f_max);

	// FIXME: Should move?
	mmc_add_host(mmc);

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->finish_tasklet,tcc_mmc_tasklet_finish, (unsigned long)host);

	setup_timer(&host->timer, tcc_mmc_timeout_timer, (unsigned long)host);

	snprintf(host->slot_desc, 16, "tcc-sdhc%d", host->controller_id);
	ret = request_irq(host->irq, tcc_mmc_interrupt_handler,
			IRQF_DISABLED|IRQ_TYPE_LEVEL_HIGH,
			host->slot_desc, host);
	if (ret) {
		dev_err(&pdev->dev, "failed to get IRQ %d, ret1(%d)\n", host->irq, ret);
		goto error;
	}

	//Card detect
	if (host->cd_irq <= 0) {
		/* configuration the SD card detection */
		init_timer(&host->detect_timer);
		host->detect_timer.function = tcc_mmc_poll_event;
		host->detect_timer.data = (unsigned long) host;
		host->detect_timer.expires = jiffies + DETECT_TIMEOUT;
		add_timer(&host->detect_timer);
	}
	else if(host->cd_irq > 0)
	{
		ret = request_irq(host->cd_irq, tcc_mmc_cd_irq,
				IRQF_TRIGGER_LOW | IRQF_DISABLED,
				host->slot_desc, host);

		if (ret) {
			dev_err(&pdev->dev, "failed to get IRQ %d, ret2(%d)\n",
					host->irq, ret);
			goto error;
		}
		tcc_mmc_check_status(host);
	}

	/* initialize the card-insert status */
	host->card_inserted = tcc_mmc_get_cd(host->mmc);
	host->is_in_tuning_mode = false;
	init_waitqueue_head(&host->buf_ready_int);

	// If you want to use the SDIO 3.0 Wi-Fi, the voltage should be changed from 3.3v to 1.8v before the Wi-Fi driver is eabled.
	if ((host->mmc->ocr_avail & MMC_VDD_165_195) && (host->cd_irq == WIFI_POLLING)) {
		mmc_writel(host, get_sdr_mode(mmc->caps, mmc->caps2),
				TCCSDHC_ACMD12_ERR);
		if (gpio_is_valid(host->tcc_hw.vctrl_gpio)) {
			tcc_mmc_gpio_set_value(host->tcc_hw.vctrl_gpio, 0);
			printk("%s: vctrl(%d)\n", __func__,
					tcc_mmc_gpio_get_value(host->tcc_hw.vctrl_gpio));
			mdelay(10); // [Timing] Host keeps SDCLK low at least 5 ms.
		}
	}

	printk("%s: SDHC%d init\n", DRIVER_NAME, host->controller_id);
	return ret;

error:
	printk("%s: SDHC%d failed\n", DRIVER_NAME, host->controller_id);
	return ret;
}

static int tcc_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct tcc_mmc_host *host = mmc_priv(mmc);

		free_irq(host->irq, host);

		mmc_remove_host(mmc);
		del_timer(&host->timer);
		del_timer_sync(&host->timer);
		mmc_free_host(mmc);

		tasklet_kill(&host->finish_tasklet);

		kfree(host->adma_desc);
		kfree(host->align_buffer);

		host->adma_desc = NULL;
		host->align_buffer = NULL;
	}

	return 0;
}

#ifdef CONFIG_PM
static int tcc_mmc_suspend(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct tcc_mmc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;

	if((host == NULL)||(mmc == NULL)) {
		printk("[mmc:NULL] %s(host:%x, mmc:%x)\n", __func__, (u32)host, (u32)mmc);
		return 0;// -EHOSTDOWN;
	}

	if (host && host->suspended)
		return 0;

	if (mmc->card) {
		if (ret == 0)
			host->suspended = 1;
	}

	//disable cd interrupt
	tcc_mmc_disable_cd_irq(host->cd_irq);

	return 0;
}

static int tcc_mmc_resume(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct tcc_mmc_host *host = platform_get_drvdata(pdev);

	if(host == NULL) {
		printk("[mmc:NULL] %s(host:%x)\n", __func__, (u32)host);
		return 0;// -EHOSTDOWN;
	}

	//enable cd interrupt
	tcc_mmc_enable_cd_irq(host->cd_irq);

	if (host && !host->suspended)
	{
		return 0;
	}

	if (host) {
		if (ret == 0)
			host->suspended = 0;
	}

	return ret;
}
#else
#define tcc_mmc_suspend	NULL
#define tcc_mmc_resume	NULL
#endif

static SIMPLE_DEV_PM_OPS(tcc_mmc_dev_pm_ops, tcc_mmc_suspend,
			 tcc_mmc_resume);

#ifdef CONFIG_OF
static const struct of_device_id tcc_mmc_of_match[] = {
	{ .compatible = "telechips,tcc893x-sdhc" },
	{ .compatible = "telechips,tcc896x-sdhc" },
	{ .compatible = "telechips,tcc897x-sdhc" },
	{ .compatible = "telechips,tcc-sdhc" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc_mmc_of_match);
#endif

static struct platform_driver tcc_mmc_driver = {
	.probe		= tcc_mmc_probe,
	.remove		= tcc_mmc_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
		.pm		= &tcc_mmc_dev_pm_ops,
		.of_match_table = of_match_ptr(tcc_mmc_of_match),
	},
};

static int __init tcc_mmc_init(void)
{
	return platform_driver_register(&tcc_mmc_driver);
}

static void __exit tcc_mmc_exit(void)
{
	platform_driver_unregister(&tcc_mmc_driver);
}

module_init(tcc_mmc_init);
module_exit(tcc_mmc_exit);

MODULE_DESCRIPTION("Telechips SD/MMC/SDIO Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Telechips Inc. linux@telechips.com");
