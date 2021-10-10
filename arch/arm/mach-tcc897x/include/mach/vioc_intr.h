#ifndef __VIOC_DISP_INTR__H__
#define __VIOC_DISP_INTR__H__

struct vioc_intr_type {
	int		id;
	unsigned int	bits;
};

enum {
	VIOC_INTR_DEV0 = 0,
	VIOC_INTR_DEV1,
	VIOC_INTR_DEV2,
	VIOC_INTR_TIMER,
	VIOC_INTR_RD0,
	VIOC_INTR_RD1,		// 5
	VIOC_INTR_RD2,
	VIOC_INTR_RD3,
	VIOC_INTR_RD4,
	VIOC_INTR_RD5,
	VIOC_INTR_RD6,		// 10
	VIOC_INTR_RD7,
	VIOC_INTR_RD8,
	VIOC_INTR_RD9,
	VIOC_INTR_RD10,
	VIOC_INTR_RD11,		// 15
	VIOC_INTR_RD12,
	VIOC_INTR_RD13,
	VIOC_INTR_RD14,
	VIOC_INTR_RD15,
	VIOC_INTR_RD16,		// 20
	VIOC_INTR_RD17,
	VIOC_INTR_MC0,
	VIOC_INTR_MC1,
	VIOC_INTR_MMU,
	VIOC_INTR_FIFO0 = 28,
	VIOC_INTR_FIFO1,
	VIOC_INTR_WD0 = 32,	// 0	// 32
	VIOC_INTR_WD1,
	VIOC_INTR_WD2,
	VIOC_INTR_WD3,		// 3	// 35
	VIOC_INTR_WD4,
	VIOC_INTR_WD5,
	VIOC_INTR_WD6,
	VIOC_INTR_WD7,
	VIOC_INTR_WD8,		// 8	// 40
	VIOC_INTR_SC4 = 43,	// 11	// 43
	VIOC_INTR_VIQE1 = 45,	// 13	// 45
	VIOC_INTR_WMIX0 = 48,	// 16	// 48
	VIOC_INTR_WMIX1,
	VIOC_INTR_WMIX2,	// 18	// 50
	VIOC_INTR_WMIX3,
	VIOC_INTR_WMIX4,
	VIOC_INTR_WMIX5,
	VIOC_INTR_WMIX6,
	VIOC_INTR_VIN0,		// 23	// 55
	VIOC_INTR_VIN1,
	VIOC_INTR_VIN2,
	VIOC_INTR_VIN3,
	VIOC_INTR_VIQE0,
	VIOC_INTR_SC0,		// 28	// 60
	VIOC_INTR_SC1,
	VIOC_INTR_SC2,
	VIOC_INTR_SC3,
	VIOC_INTR_NUM = VIOC_INTR_SC3
};

/* VIOC DEV0/1/2 irqs */
enum vioc_disp_intr_src {
	VIOC_DISP_INTR_FU = 0,	/* LCD output fifo under-run */
	VIOC_DISP_INTR_VSR,	/* VS Rising */
	VIOC_DISP_INTR_VSF,	/* VS Falling */
	VIOC_DISP_INTR_RU,	/* Register Update */
	VIOC_DISP_INTR_DD,	/* Disable Done */
	VIOC_DISP_INTR_SREQ,	/* Device Stop Request */
	VIOC_DISP_INTR_MAX
};

#define VIOC_DISP_INTR_DISPLAY	((1<<VIOC_DISP_INTR_FU)|(1<<VIOC_DISP_INTR_RU)|(1<<VIOC_DISP_INTR_DD))
#define VIOC_DISP_INT_MASK	((1<<VIOC_DISP_INTR_MAX)-1)

/* VIOC RDMA irqs */
#define VIOC_RDMA_INT_MASK	0x7F

/* VIOC WDMA irqs */
enum vioc_wdma_intr_src {
	VIOC_WDMA_INTR_UPD = 0,	/* Register Update */
	VIOC_WDMA_INTR_SREQ,	/* VIOC_WDMA_INTR_EOFF, */
	VIOC_WDMA_INTR_ROL,	/* Rolling */
	VIOC_WDMA_INTR_ENR,	/* Synchronized Enable Rising */
	VIOC_WDMA_INTR_ENF,	/* Synchronized Enable Falling */
	VIOC_WDMA_INTR_EOFR,	/* EOF Rising */
	VIOC_WDMA_INTR_EOFF,	/* EOF Falling */
	VIOC_WDMA_INTR_SEOFR, 	/* Sync EOF Rising */
	VIOC_WDMA_INTR_SEOFF, 	/* Sync EOF Falling */
	VIOC_WDMA_INTR_RESERVED,
	VIOC_WDMA_INTR_MAX
};
#define VIOC_WDMA_INT_MASK	((1<<VIOC_WDMA_INTR_MAX)-1)

/* VIOC SC irqs */
#define VIOC_SC_INT_MASK	0xF

/* VIOC WMIX irqs */
#define VIOC_WMIX_INT_MASK	0x1F



extern int vioc_intr_enable(int id, unsigned mask);
extern int vioc_intr_disable(int id, unsigned mask);
extern unsigned int vioc_intr_get_status(int id);
extern bool check_vioc_irq_status(void __iomem *reg, int id);
extern bool is_vioc_intr_activatied(int id, unsigned mask);
extern int vioc_intr_clear(int id, unsigned mask);
extern void vioc_intr_init(void);
extern bool is_vioc_intr_unmasked(int id, unsigned mask);

#endif /* __VIOC_DISP_INTR__H__ */
