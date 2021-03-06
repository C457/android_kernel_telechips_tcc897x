#ifndef __TCC_SDHC_H__
#define __TCC_SDHC_H__

#include <linux/scatterlist.h>

#ifndef Hw0
#define	Hw31		0x80000000
#define	Hw30		0x40000000
#define	Hw29		0x20000000
#define	Hw28		0x10000000
#define	Hw27		0x08000000
#define	Hw26		0x04000000
#define	Hw25		0x02000000
#define	Hw24		0x01000000
#define	Hw23		0x00800000
#define	Hw22		0x00400000
#define	Hw21		0x00200000
#define	Hw20		0x00100000
#define	Hw19		0x00080000
#define	Hw18		0x00040000
#define	Hw17		0x00020000
#define	Hw16		0x00010000
#define	Hw15		0x00008000
#define	Hw14		0x00004000
#define	Hw13		0x00002000
#define	Hw12		0x00001000
#define	Hw11		0x00000800
#define	Hw10		0x00000400
#define	Hw9			0x00000200
#define	Hw8			0x00000100
#define	Hw7			0x00000080
#define	Hw6			0x00000040
#define	Hw5			0x00000020
#define	Hw4			0x00000010
#define	Hw3			0x00000008
#define	Hw2			0x00000004
#define	Hw1			0x00000002
#define	Hw0			0x00000001
#define HwZERO		0x0
#endif
 
/*********************************************
 * SDMMC BASE ADDRESS
 ********************************************/
#if defined(CONFIG_ARCH_TCC897X)
#define TCC897X_PA_SDMMC0		0x76020000
#define TCC897X_PA_SDMMC1		0x76020200
#define TCC897X_PA_SDMMC2		0x76020400
#define TCC897X_PA_SDMMC3		0x76020600
#define TCCSDMMC_PA_CHCTRL		0x76020800
#elif defined(CONFIG_ARCH_TCC898X)
#define TCC898X_PA_SDMMC0		0x16440000
#define TCC898X_PA_SDMMC1		0x16450000
#define TCC898X_PA_SDMMC2		0x16460000
#define TCCSDMMC_PA_CHCTRL		0x16470000
#endif

/*******************************************************************************
*	SD/SDIO/MMC Host Controller Register Define
********************************************************************************/
/*
 * Controller registers
 */
#define TCCSDHC_DMA_ADDRESS 		0x00
#define TCCSDHC_BLOCK_SIZE			0x04

#define TCCSDHC_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

#define TCCSDHC_BLOCK_COUNT 		0x06
#define TCCSDHC_ARGUMENT			0x08
#define TCCSDHC_TRANSFER_MODE		0x0C
#define  TCCSDHC_TRNS_READ			0x10
#define TCCSDHC_COMMAND 			0x0E
#define TCCSDHC_TMODE_COM			0x0C

#define TCCSDHC_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))

#define TCCSDHC_RESPONSE10			0x10
#define TCCSDHC_RESPONSE32			0x14
#define TCCSDHC_RESPONSE54			0x18
#define TCCSDHC_RESPONSE76			0x1c
#define TCCSDHC_BUFFER				0x20
#define TCCSDHC_PRESENT_STATE		0x24
#define TCCSDHC_HOST_CONTROL		0x28
#define TCCSDHC_POWER_CONTROL		0x29
#define TCCSDHC_BLOCK_GAP_CONTROL	0x2A
#define TCCSDHC_WAKE_UP_CONTROL 	0x2B
#define TCCSDHC_CLOCK_CONTROL		0x2C
#define TCCSDHC_TIMEOUT_CONTROL 	0x2E
#define TCCSDHC_SOFTWARE_RESET		0x2F
#define TCCSDHC_INT_STATUS			0x30
#define  TCCSDHC_INT_ERR			0x00008000
#define TCCSDHC_INT_ENABLE			0x34
#define TCCSDHC_SIGNAL_ENABLE		0x38
#define  TCCSDHC_INT_DATA_AVAIL		0x00000020
#define TCCSDHC_ACMD12_ERR			0x3C
#define  TCCSDHC_EXEC_TUNING		0x00400000
#define  TCCSDHC_TUNED_CLOCK		0x00800000
#define TCCSDHC_CAPABILITIES1		0x40
#define TCCSDHC_CAPABILITIES2		0x44

#define TCCSDHC_ADMA_ERROR			0x54
#define TCCSDHC_ADMA_ADDRESS		0x58

#define TCCSDHC_SLOT_INT			0xFC
#define TCCSDHC_HOST_VERSION		0xFE


#define HwSD_COM_TRANS_ABORT	Hw23+Hw22
#define HwSD_COM_TRANS_DATSEL	Hw21		// data present select
#define HwSD_COM_TRANS_CICHK	Hw20		// command index check enable
#define HwSD_COM_TRANS_CRCHK	Hw19		// command CRC check enable
#define HwSD_COM_TRANS_SPI		Hw7 		// SPI mode
#define HwSD_COM_TRANS_ATACMD	Hw6 		// cmd completion enable for CE-ATA
#define HwSD_COM_TRANS_MS		Hw5 		// multi/single block select
#define HwSD_COM_TRANS_DIR		Hw4 		// data transfer direction select
#define HwSD_COM_TRANS_ACMD23	Hw3			// auto CMD23 enable
#define HwSD_COM_TRANS_ACMD12	Hw2 		// auto CMD12 enable
#define HwSD_COM_TRANS_BCNTEN	Hw1 		// block count enable
#define HwSD_COM_TRANS_DMAEN	Hw0 		// DMA Enable


#define HwSDCLKSEL_DIV_256		0x80
#define HwSDCLKSEL_DIV_128		0x40
#define HwSDCLKSEL_DIV_64		0x20
#define HwSDCLKSEL_DIV_32		0x10
#define HwSDCLKSEL_DIV_16		0x08
#define HwSDCLKSEL_DIV_8		0x04
#define HwSDCLKSEL_DIV_4		0x02
#define HwSDCLKSEL_DIV_2		0x01
#define HwSDCLKSEL_DIV_0		0x00
#define HwSDCLKSEL_SCK_EN		Hw2
#define HwSDCLKSEL_INCLK_STABLE Hw1
#define HwSDCLKSEL_INCLK_EN 	Hw0

#define HwSD_POWER_POW			Hw8 		// SD bus power
#define HwSD_POWER_SD8			Hw5 		// SD 8-bit mode
#define HwSD_POWER_HS			Hw2 		// high speed enable
#define HwSD_POWER_SD4			Hw1 		// data transfer width
#define HwSD_POWER_VOL33		(Hw11+Hw10+Hw9)
#define HwSD_POWER_VOL18		0x00000A00

#define HwSD_CTRL_SDMA			0x00
#define HwSD_CTRL_ADMA32		Hw4
#define HwSD_CTRL_DMA_MASK		Hw4+Hw3

#define HwSD_SRESET_RSTALL		Hw0 	// software reset for All
#define HwSD_SRESET_RSTCMD		Hw1 	// software reset for CMD line
#define HwSD_SRESET_RSTDAT		Hw2 	// software reset for DAT line


// Port Control
#define HwSD_PORTCTRL_CD(X) 		(Hw30 << (X))								// Card Detection for SLOT X. (X = 0 or 1)
#define HwSD_PORTCTRL_WP(X) 		(Hw27 << (X))								// Write Protect for SLOT X. (X = 0 or 1)
#if 0  //remove
#define HwSD_PORTCTRL_SLOT3(X)		((X) * Hw12)								// Port Select for SLOT 3 (X = 0 ~ 7)
#define HwSD_PORTCTRL_SLOT3_MASK	HwSD_PORTCTRL_SLOT3(15)
#define HwSD_PORTCTRL_SLOT2(X)		((X) * Hw8) 								// Port Select for SLOT 2 (X = 0 ~ 7)
#define HwSD_PORTCTRL_SLOT2_MASK	HwSD_PORTCTRL_SLOT2(15)
#define HwSD_PORTCTRL_SLOT1(X)		((X) * Hw4) 								// Port Select for SLOT 1 (X = 0 ~ 7)
#define HwSD_PORTCTRL_SLOT1_MASK	HwSD_PORTCTRL_SLOT1(15)
#define HwSD_PORTCTRL_SLOT0(X)		((X) * Hw0) 								// Port Select for SLOT 0 (X = 0 ~ 7)
#define HwSD_PORTCTRL_SLOT0_MASK	HwSD_PORTCTRL_SLOT0(15)
#endif //remove
#define HwSD_STATE_SDWP 		Hw19
#define HwSD_STATE_NODAT		Hw1 		// data inhibit
#define HwSD_STATE_NOCMD		Hw0 		// command inhibit

#define HwSDINT_STATUS_ADMA		Hw25		// ADMA error
#define HwSDINT_STATUS_ACMD	Hw24
#define HwSDINT_STATUS_DATEND	Hw22		// data end bit error
#define HwSDINT_STATUS_DATCRC	Hw21		// data crc error
#define HwSDINT_STATUS_DATTIME	Hw20		// data timeout error
#define HwSDINT_STATUS_CINDEX	Hw19		// command index error
#define HwSDINT_STATUS_CMDEND	Hw18		// command command end bit error
#define HwSDINT_STATUS_CMDCRC	Hw17		// command crc error
#define HwSDINT_STATUS_CMDTIME	Hw16		// command timeout error
#define HwSDINT_STATUS_ERR		Hw15		// error interrupt
#define HwSDINT_STATUS_CDINT	Hw8 		// card interrupt
#define HwSDINT_STATUS_CDOUT	Hw7 		// card removal
#define HwSDINT_STATUS_CDIN 	Hw6 		// card insertion
#define HwSDINT_STATUS_RDRDY	Hw5 		// buffer read ready
#define HwSDINT_STATUS_WRRDY	Hw4 		// buffer write ready
#define HwSDINT_STATUS_DMA		Hw3 		// DMA interrupt
#define HwSDINT_STATUS_BLKGAP	Hw2 		// block gap event
#define HwSDINT_STATUS_TDONE	Hw1 		// transfer complete
#define HwSDINT_STATUS_CDONE	Hw0 		// command complete

#define HwSDINT_EN_ADMA 		Hw25		// ADMA error signal enable
#define HwSDINT_EN_ACMD12		Hw24		// auto CMD12 error signal enable
#define HwSDINT_EN_CLIMIT		Hw23		// current limit error signal enable
#define HwSDINT_EN_DATEND		Hw22		// data end bit error signal enable
#define HwSDINT_EN_DATCRC		Hw21		// data crc error signal enable
#define HwSDINT_EN_DATTIME		Hw20		// data timeout error signal enable
#define HwSDINT_EN_CINDEX		Hw19		// command index error signal enable
#define HwSDINT_EN_CMDEND		Hw18		// command end bit error signal enable
#define HwSDINT_EN_CMDCRC		Hw17		// command crc error signal enable
#define HwSDINT_EN_CMDTIME		Hw16		// command timeout error signal enable
#define HwSDINT_EN_CDINT		Hw8 		// card interrupt signal enable
#define HwSDINT_EN_CDOUT		Hw7 		// card removal signal enable
#define HwSDINT_EN_CDIN 		Hw6 		// card insertion signal enable
#define HwSDINT_EN_RDRDY		Hw5 		// buffer read ready signal enable
#define HwSDINT_EN_WRRDY		Hw4 		// buffer write ready signal enable
#define HwSDINT_EN_DMA			Hw3 		// DMA interrupt signal enable
#define HwSDINT_EN_BLKGAP		Hw2 		// block gap event signal enable
#define HwSDINT_EN_TDONE		Hw1 		// transfer complete signal enable
#define HwSDINT_EN_CDONE		Hw0 		// command complete signal enable


#define  HwSDINT_NORMAL_MASK	0x00007FFF
#define  HwSDINT_ERROR_MASK 0xFFFF8000

#define  HwSDINT_CMD_MASK	(HwSDINT_EN_CDONE | HwSDINT_EN_CMDTIME | \
		HwSDINT_EN_CMDCRC | HwSDINT_EN_CMDEND | HwSDINT_EN_CINDEX)
		
#define  HwSDINT_DATA_MASK	(HwSDINT_EN_TDONE | HwSDINT_EN_DMA | \
		HwSDINT_EN_RDRDY | HwSDINT_EN_WRRDY | \
		HwSDINT_EN_DATTIME | HwSDINT_EN_DATCRC | \
		HwSDINT_EN_DATEND | HwSDINT_EN_ADMA)

#define TCCSDMMC_CHCTRL_SDCTRL          0x000  /* R/W  0x00000000   Host Controller Control Register */
#define TCCSDMMC_CHCTRL_SDRESERVED      0x004  /* R/W  0x00000000   Reserved Register */
#define TCCSDMMC_CHCTRL_SD0CMDDAT       0x008  /* R/W  0x00000000   SD/MMC0 output delay control register */
#define TCCSDMMC_CHCTRL_SD1CMDDAT       0x00C  /* R/W  0x00000000   SD/MMC1 output delay control register */
#define TCCSDMMC_CHCTRL_SD2CMDDAT       0x010  /* R/W  0x00000000   SD/MMC2 output delay control register */
#define TCCSDMMC_CHCTRL_SD3CMDDAT       0x014  /* R/W  0x00000000   SD/MMC3 output delay control register */
#define TCCSDMMC_CHCTRL_SD0CAPREG0      0x018  /* R/W  0x00000000   SD/MMC0 Capabilities 0 register */
#define TCCSDMMC_CHCTRL_SD0CAPREG1      0x01C  /* R/W  0x00000000   SD/MMC0 Capabilities 1 register */
#define TCCSDMMC_CHCTRL_SD0INITSPD      0x020  /* R/W  0x00000000   SD/MMC0 Initialization & Default Speed Config register */
#define TCCSDMMC_CHCTRL_SD0HIGHSPD      0x024  /* R/W  0x00000000   SD/MMC0 High Speed Config register */
#define TCCSDMMC_CHCTRL_SD0PRESET5      0x028  /* R/W  0x00000000   SD/MMC0 Preset Register5 */
#define TCCSDMMC_CHCTRL_SD0PRESET6      0x02C  /* R/W  0x00000000   SD/MMC0 Preset Register6 */
#define TCCSDMMC_CHCTRL_SD1CAPREG0      0x030  /* R/W  0x00000000   SD/MMC1 Capabilities 0 register */
#define TCCSDMMC_CHCTRL_SD1CAPREG1      0x034  /* R/W  0x00000000   SD/MMC1 Capabilities 1 register */
#define TCCSDMMC_CHCTRL_SD1INITSPD      0x038  /* R/W  0x00000000   SD/MMC1 Initialization & Default Speed Config register */
#define TCCSDMMC_CHCTRL_SD1HIGHSPD      0x03C  /* R/W  0x00000000   SD/MMC1 High Speed Config register */
#define TCCSDMMC_CHCTRL_SD1PRESET5      0x040  /* R/W  0x00000000   SD/MMC1 Preset Register5 */
#define TCCSDMMC_CHCTRL_SD1PRESET6      0x044  /* R/W  0x00000000   SD/MMC1 Preset Register6 */
#define TCCSDMMC_CHCTRL_SD2CAPREG0      0x048  /* R/W  0x00000000   SD/MMC2 Capabilities 0 register */
#define TCCSDMMC_CHCTRL_SD2CAPREG1      0x04C  /* R/W  0x00000000   SD/MMC2 Capabilities 1 register */
#define TCCSDMMC_CHCTRL_SD2INITSPD      0x050  /* R/W  0x00000000   SD/MMC2 Initialization & Default Speed Config register */
#define TCCSDMMC_CHCTRL_SD2HIGHSPD      0x054  /* R/W  0x00000000   SD/MMC2 High Speed Config register */
#define TCCSDMMC_CHCTRL_SD2PRESET5      0x058  /* R/W  0x00000000   SD/MMC2 Preset Register5 */
#define TCCSDMMC_CHCTRL_SD2PRESET6      0x05C  /* R/W  0x00000000   SD/MMC2 Preset Register6 */
#define TCCSDMMC_CHCTRL_SD3CAPREG0      0x060  /* R/W  0x00000000   SD/MMC3 Capabilities 0 register */
#define TCCSDMMC_CHCTRL_SD3CAPREG1      0x064  /* R/W  0x00000000   SD/MMC3 Capabilities 1 register */
#define TCCSDMMC_CHCTRL_SD3INITSPD      0x068  /* R/W  0x00000000   SD/MMC3 Initialization & Default Speed Config register */
#define TCCSDMMC_CHCTRL_SD3HIGHSPD      0x06C  /* R/W  0x00000000   SD/MMC3 High Speed Config register */
#define TCCSDMMC_CHCTRL_SD3PRESET5      0x070  /* R/W  0x00000000   SD/MMC3 Preset Register5 */
#define TCCSDMMC_CHCTRL_SD3PRESET6      0x074  /* R/W  0x00000000   SD/MMC3 Preset Register6 */
#define TCCSDMMC_CHCTRL_SD01DELAY       0x078  /* R/W  0x00000000   SD/MMC0/1 Clock delay controller */
#define TCCSDMMC_CHCTRL_SD23DELAY       0x07C  /* R/W  0x00000000   SD/MMC2/3 Clock delay controller */

// SDMMC 5.1 Channel Control Register
// Channel Control Base + Channel Offset + Value
#define TCCSDHC_CHCTRL_TAPDLY			0x00
#define TCCSDHC_CHCTRL_CAP0				0x04
#define TCCSDHC_CHCTRL_CAP1				0x08
#define TCCSDHC_CHCTRL_PRESET0			0x0C
#define TCCSDHC_CHCTRL_PRESET1			0x10
#define TCCSDHC_CHCTRL_PRESET2			0x14
#define TCCSDHC_CHCTRL_PRESET3			0x18
#define TCCSDHC_CHCTRL_PRESET4			0x1C
#define TCCSDHC_CHCTRL_MAXCRNT			0x20
#define TCCSDHC_CHCTRL_CQ				0x24
#define TCCSDHC_CHCTRL_DLY0				0x28
#define TCCSDHC_CHCTRL_DLY1				0x2C
#define TCCSDHC_CHCTRL_DLY2				0x30
#define TCCSDHC_CHCTRL_DLY3				0x34
#define TCCSDHC_CHCTRL_DLY4				0x38
#define TCCSDHC_CHCTRL_DBG0				0x3C
#define TCCSDHC_CHCTRL_DBG1				0x40
#define TCCSDHC_CHCTRL_DBG2				0x44
#define TCCSDHC_CHCTRL_MON				0x48
#define TCCSDHC_CHCTRL_CD_WP			0x4C

struct mmc_direct_req {
	u32 sector;			/* start sector */
	u32 nr_sectors;		/* # of sectors */
	u32 cmd_flags;		/* direction */
	void *buf;			/* read/write buf from FSG */
	dma_addr_t dma;		/* dma address of buf */
};

struct tcc_mmc_hw_specific {
	unsigned vctrl_gpio;	/* Voltage Control GPIO(e.g. 3.3V <--> 1.8V) */
	unsigned pwr_gpio; /* Power Control GPIO(on, off) */
	unsigned rst_gpio; /* emmc reset */
};

struct tcc_mmc_host {
	int initialized;
	int suspended;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	int data_early:1;				/* Data finished before cmd */

	struct mmc_host *mmc;
	struct device *dev;
	unsigned int controller_id;				/* 16xx chips have 2 MMC blocks */
	unsigned long peri_clk;			/* Controller base clock */
	unsigned int clk_div;			/* base clock divider */
	struct clk *hclk;
	struct clk *fclk;
	void __iomem *base;
	void __iomem *chctrl_base;
	int	irq;
	unsigned char bus_mode;
	unsigned char timing;
	//unsigned char hw_bus_mode;

	char slot_desc[16]; 			/* Name for reservations */

	//unsigned int sg_len;
	//int sg_idx;
	//u16 *buffer;
	//u32 buffer_bytes_left;
	//u32 total_bytes_left;

	struct scatterlist *cur_sg;		/* We're working on this */
	int	num_sg;						/* Entries left */
	int	offset;						/* Offset into current sg */
	int	remain;						/* Bytes left in current */

	int sg_count;					/* Mapped sg entries */

	unsigned char *adma_desc;		/* ADMA descriptor table */
	unsigned char *align_buffer;	/* Bounce buffer */

	dma_addr_t adma_addr;			/* Mapped ADMA descr. table */
	dma_addr_t align_addr;			/* Mapped bounce buffer */

	//short wp_pin;

	short card_inserted;			/* to mark the card is inserted or not */
	short card_changed;
	int cd_irq;
	unsigned int card_insert;
	unsigned int oldstat;

	struct tasklet_struct finish_tasklet;

	spinlock_t lock;				/* Mutex */

	int flags;						/* Host attributes */
#define TCC_MMC_USE_DMA		(1<<0)	/* Host is DMA capable */
#define TCC_MMC_USE_ADMA	(1<<1)	/* Host is ADMA capable */
#define TCC_MMC_REQ_USE_DMA	(1<<2)	/* Use DMA for this req. */

	struct timer_list detect_timer;
	struct timer_list timer;

	int is_direct;
	struct mmc_direct_req *req;

	sector_t phys_nr_sects;			/* physical total sectors */
	sector_t phys_boot_sect;		/* bootloader start sector */
	sector_t phys_kern_sect;		/* kernel start sector */

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_idle;
	struct pinctrl_state *pins_sleep;

	unsigned int tuning_done;
	unsigned int tuning_count; /* Timer count for re-tuning */
	unsigned int tuning_mode; /* Re-tuning mode supported by host */
	wait_queue_head_t buf_ready_int; /* Waitqueue for Buffer Read Ready interrupt */
	struct sg_mapping_iter sg_miter; /* SG state for PIO */
	bool is_in_tuning_mode;

	struct tcc_mmc_hw_specific tcc_hw;
#if defined(CONFIG_ENABLE_TCC_MMC_KPANIC)
	bool is_kpanic_card;
#endif
	bool is_clock_control;
};

void tcc_mmc_clock_control(struct tcc_mmc_host *host, int onoff);
void tcc_mmc_gpio_set_value(unsigned gpio, int value);
int tcc_mmc_gpio_get_value(unsigned gpio);

#endif /*__TCC_SDHC_H__*/
