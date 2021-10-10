/*
 * linux/arch/arm/mach-tcc893x/gpio.c
 *
 * Copyright (C) 2011 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>

#ifdef CONFIG_ARCH_TCC897X

#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <asm/mach/irq.h>

#include <mach/reg_physical.h>

#define RMWREG32(addr, startbit, width, val) writel((readl(addr) & ~(((1<<(width)) - 1) << (startbit))) | ((val) << (startbit)), addr)

#define GPIO_REG(reg)	IO_ADDRESS(reg)

typedef struct gpioregs gpioregs;

struct gpioregs
{
	unsigned data;         /* data */
	unsigned out_en;       /* output enable */
	unsigned out_or;       /* OR fnction on output data */
	unsigned out_bic;      /* BIC function on output data */
	unsigned out_xor;      /* XOR function on output data */
	unsigned strength0;    /* driver strength control 0 */
	unsigned strength1;    /* driver strength control 1 */
	unsigned pull_enable;  /* pull-up/down enable */
	unsigned pull_select;  /* pull-up/down select */
	unsigned in_en;        /* input enable */
	unsigned in_type;      /* input type (Shmitt / CMOS) */
	unsigned slew_rate;    /* slew rate */
	unsigned func_select0; /* port configuration 0 */
	unsigned func_select1; /* port configuration 1 */
	unsigned func_select2; /* port configuration 2 */
	unsigned func_select3; /* port configuration 3 */
};

struct board_gpio_irq_config *board_gpio_irqs;
//spinlock_t gpio_lock;
//EXPORT_SYMBOL(gpio_lock);

#ifdef CONFIG_DAUDIO_PIN_CONTROL
int tcc_gpio_clear_ext_intr(unsigned intr)
{
	int extint;
	unsigned int source = 0;
    GPIO *reg = (GPIO *) GPIO_REG(HwGPIO_BASE);

	switch (intr) {
		case INT_EINT0:  extint = EXINT_EI0;  break;
		case INT_EINT1:  extint = EXINT_EI1;  break;
		case INT_EINT2:  extint = EXINT_EI2;  break;
		case INT_EINT3:  extint = EXINT_EI3;  break;
		case INT_EINT4:  extint = EXINT_EI4;  break;
		case INT_EINT5:  extint = EXINT_EI5;  break;
		case INT_EINT6:  extint = EXINT_EI6;  break;
		case INT_EINT7:  extint = EXINT_EI7;  break;
		case INT_EINT8:  extint = EXINT_EI8;  break;
		case INT_EINT9:  extint = EXINT_EI9;  break;
		case INT_EINT10: extint = EXINT_EI10; break;
		case INT_EINT11: extint = EXINT_EI11; break;
		default:
			extint = -1;
			break;
	}

	if (extint < 0)
		return -1;

    if(extint < 4) {
        RMWREG32(&reg->EINTSEL0.nREG, extint*8, 7, source);
    }
    else if(extint < 8) {
        RMWREG32(&reg->EINTSEL1.nREG, (extint-4)*8, 7, source);
    }
    else if(extint < 12) {
        RMWREG32(&reg->EINTSEL2.nREG, (extint-8)*8, 7, source);
    }

	return 0;
}

int tcc_gpio_config_ext_intr(unsigned intr, unsigned source) /* intr: irq num, source: external interrupt source */
{
	int extint;
    GPIO *reg = (GPIO *) GPIO_REG(HwGPIO_BASE);

	switch (intr) {
		case INT_EINT0:  extint = EXINT_EI0;  break;
		case INT_EINT1:  extint = EXINT_EI1;  break;
		case INT_EINT2:  extint = EXINT_EI2;  break;
		case INT_EINT3:  extint = EXINT_EI3;  break;
		case INT_EINT4:  extint = EXINT_EI4;  break;
		case INT_EINT5:  extint = EXINT_EI5;  break;
		case INT_EINT6:  extint = EXINT_EI6;  break;
		case INT_EINT7:  extint = EXINT_EI7;  break;
		case INT_EINT8:  extint = EXINT_EI8;  break;
		case INT_EINT9:  extint = EXINT_EI9;  break;
		case INT_EINT10: extint = EXINT_EI10; break;
		case INT_EINT11: extint = EXINT_EI11; break;
		default:
			extint = -1;
			break;
	}

	if (extint < 0)
		return -1;

    if(extint < 4) {
        RMWREG32(&reg->EINTSEL0.nREG, extint*8, 7, source);
    }
    else if(extint < 8) {
        RMWREG32(&reg->EINTSEL1.nREG, (extint-4)*8, 7, source);
    }
    else if(extint < 12) {
        RMWREG32(&reg->EINTSEL2.nREG, (extint-8)*8, 7, source);
    }

	return 0;
}
#endif //CONFIG_DAUDIO_PIN_CONTROL

static inline gpioregs *tcc893x_gpio_to_reg(unsigned gpio_port)
{
	switch(gpio_port)
	{
		case GPIO_PORTA:
			return (gpioregs *) GPIO_REG(HwGPIOA_BASE);
		case GPIO_PORTB:
			return (gpioregs *) GPIO_REG(HwGPIOB_BASE);
		case GPIO_PORTC:
			return (gpioregs *) GPIO_REG(HwGPIOC_BASE);
		case GPIO_PORTD:
			return (gpioregs *) GPIO_REG(HwGPIOD_BASE);
		case GPIO_PORTE:
			return (gpioregs *) GPIO_REG(HwGPIOE_BASE);
		case GPIO_PORTF:
			return (gpioregs *) GPIO_REG(HwGPIOF_BASE);
		case GPIO_PORTG:
			return (gpioregs *) GPIO_REG(HwGPIOG_BASE);
		case GPIO_PORTHDMI:
			return (gpioregs *) GPIO_REG(HwGPIOHDMI_BASE);
		case GPIO_PORTADC:
			return (gpioregs *) GPIO_REG(HwGPIOADC_BASE);
		case GPIO_PORTSD:
			return (gpioregs *) GPIO_REG(HwGPIOSD_BASE);
		default:
			break;
	}
	return NULL;
}

int tcc_gpio_config(unsigned gpio, unsigned flags)
{
	gpioregs *r = tcc893x_gpio_to_reg((gpio&GPIO_REGMASK));
	unsigned num = gpio & GPIO_BITMASK;
	unsigned bit = (1<<num);

	if(r == NULL)
		return -EINVAL;

	if (flags & GPIO_FN_BITMASK) {
		unsigned fn = ((flags & GPIO_FN_BITMASK) >> GPIO_FN_SHIFT) - 1;

		if (num < 8)
			RMWREG32(&r->func_select0, num*4, 4, fn);
		else if (num < 16)
			RMWREG32(&r->func_select1, (num-8)*4, 4, fn);
		else if (num < 24)
			RMWREG32(&r->func_select2, (num-16)*4, 4, fn);
		else
			RMWREG32(&r->func_select3, (num-24)*4, 4, fn);
	}

	if (flags & GPIO_CD_BITMASK) {
		unsigned cd = ((flags & GPIO_CD_BITMASK) >> GPIO_CD_SHIFT) - 1;

		if (num < 16)
			RMWREG32(&r->strength0, num*2, 2, cd);
		else
			RMWREG32(&r->strength1, (num-16)*2, 2, cd);
	}

	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_HIGH)
			writel(readl(&r->data) | bit, &r->data);
		if (flags & GPIO_LOW)
			writel(readl(&r->data) & (~bit),& r->data);
		writel(readl(&r->out_en) | bit, &r->out_en);
	} else if (flags & GPIO_INPUT) {
		writel(readl(&r->out_en) & (~bit), &r->out_en);
	}

	if (flags & GPIO_PULLUP){
		writel(readl(&r->pull_select) | bit, &r->pull_select);
		writel(readl(&r->pull_enable) | bit, &r->pull_enable);
	} else if (flags & GPIO_PULLDOWN) {
		writel(readl(&r->pull_select) & (~bit), &r->pull_select);
		writel(readl(&r->pull_enable) | bit, &r->pull_enable);
	} else if (flags & GPIO_PULL_DISABLE) {
		writel(readl(&r->pull_enable) & (~bit), &r->pull_enable);	
  	} else if (flags & GPIO_SCHMITT_INPUT) {
		writel(readl(&r->in_type) | bit, &r->in_type);
	} else if (flags & GPIO_CMOS_INPUT) {
		writel(readl(&r->in_type) & (~bit), &r->in_type);
	}

	return 0;
}

EXPORT_SYMBOL(tcc_gpio_config);
#ifdef CONFIG_DAUDIO_PIN_CONTROL
EXPORT_SYMBOL(tcc_gpio_config_ext_intr);
EXPORT_SYMBOL(tcc_gpio_clear_ext_intr);
#endif //CONFIG_DAUDIO_PIN_CONTROL
#else
int tcc_gpio_config(unsigned gpio, unsigned flags)
{
	WARN_ONCE(1, "%s: Do not call tcc_gpio_config function\n", __func__);
	return 0;
}
#endif
