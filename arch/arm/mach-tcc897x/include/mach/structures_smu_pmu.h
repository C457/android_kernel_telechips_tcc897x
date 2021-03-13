/*
 * Copyright (c) 2011 Telechips, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _STRUCTURES_SMU_PMU_H_
#define _STRUCTURES_SMU_PMU_H_


/*******************************************************************************
*
*    TCC896x DataSheet PART 2 SMU & PMU
*
********************************************************************************/


/************************************************************************
*   2. VPIC                                      (Base Addr = 0x74100000)
************************************************************************/

typedef struct {
    unsigned TC0            :1;     //00
    unsigned TC1            :1;     //01
    unsigned SMU_I2C        :1;     //02
    unsigned EINT0          :1;     //03
    unsigned EINT1          :1;     //04
    unsigned EINT2          :1;     //05
    unsigned EINT3          :1;     //06
    unsigned EINT4          :1;     //07
    unsigned EINT5          :1;     //08
    unsigned EINT6          :1;     //09
    unsigned EINT7          :1;     //10
    unsigned EINT8          :1;     //11
    unsigned EINT9          :1;     //12
    unsigned EINT10         :1;     //13
    unsigned EINT11         :1;     //14
    unsigned JPEG1          :1;     //15
    unsigned MALIPPMMU0     :1;     //16
    unsigned MALIGPMMU      :1;     //17
    unsigned DENALI         :1;     //18
    unsigned L2CACHE        :1;     //19
    unsigned LCDC           :1;     //20
    unsigned JPEG0          :1;     //21
    unsigned                :1;     //22
    unsigned VCOD           :1;     //23
    unsigned MALIPP0        :1;     //24
    unsigned MALIGP         :1;     //25
    unsigned MALIPMU        :1;     //26
    unsigned OVERALY        :1;     //27
    unsigned TSADC          :1;     //28
    unsigned GDMA           :1;     //29
    unsigned HSIODMAX       :1;     //30
    unsigned CM3MB          :1;     //31
} PIC0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    PIC0_IDX_TYPE           bREG;
} PIC_SRC0_TYPE;

typedef struct {
    unsigned SDMMC3         :1;     //32
    unsigned SDMMC2         :1;     //33
    unsigned HDMI           :1;     //34
    unsigned MALIPP1        :1;     //35
    unsigned GPSB           :1;     //36
    unsigned IDE            :1;     //37
    unsigned I2C            :1;     //38
    unsigned MPEFEC         :1;     //39
    unsigned MALIPPMMU1     :1;     //40
    unsigned NFC            :1;     //41
    unsigned REMOCON        :1;     //42
    unsigned RTC            :1;     //43
    unsigned SDMMC0         :1;     //44
    unsigned SDMMC1         :1;     //45
    unsigned GDMAHS         :1;     //46
    unsigned UART           :1;     //47
    unsigned USB30          :1;     //48
    unsigned USB20H         :1;     //49
    unsigned RFX            :1;     //50
    unsigned GMAC           :1;     //51
    unsigned CIPHER         :1;     //52
    unsigned TSIF           :1;     //53
    unsigned CM3TSDEMUX     :1;     //54
    unsigned CAN            :1;     //55
    unsigned USBVBON        :1;     //56
    unsigned USBVBOFF       :1;     //57
    unsigned ADMA1          :1;     //58
    unsigned AUDIO1         :1;     //59
    unsigned ADMA0          :1;     //60
    unsigned AUDIO0         :1;     //61
    unsigned CPUBUSDMAX     :1;     //62
    unsigned USB30RTUNE     :1;     //63
} PIC1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    PIC1_IDX_TYPE           bREG;
} PIC_SRC1_TYPE;

typedef struct {
    unsigned IRQ            :1;
    unsigned FIQ            :1;
    unsigned                :30;
} ALLMSK_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    ALLMSK_IDX_TYPE         bREG;
} ALLMSK_TYPE;

typedef struct _PIC{
    volatile PIC_SRC0_TYPE      IEN0;           // 0x000  R/W  0x00000000   Interrupt Enable Register
    volatile PIC_SRC1_TYPE      IEN1;           // 0x004  R/W  0x00010000   Interrupt Enable Register
    volatile PIC_SRC0_TYPE      CLR0;           // 0x008  R/W  0x00000000   Interrupt Clear Register
    volatile PIC_SRC1_TYPE      CLR1;           // 0x00C  R/W  0x00000000   Interrupt Clear Register
    volatile PIC_SRC0_TYPE      STS0;           // 0x010  R    0x00118038   Interrupt Status Register
    volatile PIC_SRC1_TYPE      STS1;           // 0x014  R    0xF0000100   Interrupt Status Register
    volatile PIC_SRC0_TYPE      SEL0;           // 0x018  R/W  0x00000000   IRQ or FIR Selection Register
    volatile PIC_SRC1_TYPE      SEL1;           // 0x01C  R/W  0x00010000   IRQ or FIR Selection Register
    volatile PIC_SRC0_TYPE      SRC0;           // 0x020  R    0x00118038   Source Interrupt Status Register
    volatile PIC_SRC1_TYPE      SRC1;           // 0x024  R    0xF0000100   Source Interrupt Status Register
    volatile PIC_SRC0_TYPE      MSTS0;          // 0x028  R    0x00000000   Masked Status Register
    volatile PIC_SRC1_TYPE      MSTS1;          // 0x02C  R    0x00000000   Masked Status Register
    volatile PIC_SRC0_TYPE      TIG0;           // 0x030  R/W  0x00000000   Test Interrupt Generation Register
    volatile PIC_SRC1_TYPE      TIG1;           // 0x034  R/W  0x00000000   Test Interrupt Generation Register
    volatile PIC_SRC0_TYPE      POL0;           // 0x038  R/W  0x00000000   Interrupt Polarity Register
    volatile PIC_SRC1_TYPE      POL1;           // 0x03C  R/W  0x00000000   Interrupt Polarity Register
    volatile PIC_SRC0_TYPE      IRQ0;           // 0x040  R    0x00000000   IRQ Raw Status Register
    volatile PIC_SRC1_TYPE      IRQ1;           // 0x044  R    0x00000000   IRQ Raw Status Register
    volatile PIC_SRC0_TYPE      FIQ0;           // 0x048  R    Unknown      FIQ Status Register
    volatile PIC_SRC1_TYPE      FIQ1;           // 0x04C  R    Unknown      FIQ Status Register
    volatile PIC_SRC0_TYPE      MIRQ0;          // 0x050  R    0x00000000   Masked IRQ Status Register
    volatile PIC_SRC1_TYPE      MIRQ1;          // 0x054  R    0x00000000   Masked IRQ Status Register
    volatile PIC_SRC0_TYPE      MFIQ0;          // 0x058  R    0x00000000   Masked FIQ Status Register
    volatile PIC_SRC1_TYPE      MFIQ1;          // 0x05C  R    0x00000000   Masked FIQ Status Register
    volatile PIC_SRC0_TYPE      MODE0;          // 0x060  R/W  0x00000000   Trigger Mode Register ? Level or Edge
    volatile PIC_SRC1_TYPE      MODE1;          // 0x064  R/W  0x00000000   Trigger Mode Register ? Level or Edge
    volatile PIC_SRC0_TYPE      SYNC0;          // 0x068  R/W  0xFFFFFFFF   Synchronization Enable Register
    volatile PIC_SRC1_TYPE      SYNC1;          // 0x06C  R/W  0xFFFFFFFF   Synchronization Enable Register
    volatile PIC_SRC0_TYPE      WKEN0;          // 0x070  R/W  0x00000000   Wakeup Event Enable Register
    volatile PIC_SRC1_TYPE      WKEN1;          // 0x074  R/W  0x00000000   Wakeup Event Enable Register
    volatile PIC_SRC0_TYPE      MODEA0;         // 0x078  R/W  0x00000000   Both Edge or Single Edge Register
    volatile PIC_SRC1_TYPE      MODEA1;         // 0x07C  R/W  0x00000000   Both Edge or Single Edge Register
    volatile unsigned long      EI37SEL;        // 0x080  R/W  0x00000000   External INT 3~7 Selection Register
    unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    volatile PIC_SRC0_TYPE      INTMSK0;        // 0x100  R/W  0xFFFFFFFF   Interrupt Output Masking Register
    volatile PIC_SRC1_TYPE      INTMSK1;        // 0x104  R/W  0xFFFFFFFF   Interrupt Output Masking Register
    volatile ALLMSK_TYPE        ALLMSK;         // 0x108  R/W  0x00000003   All Mask Register
} PIC, *PPIC;

typedef struct {
    unsigned VA_            :9;
    unsigned                :22;
    unsigned INV            :1;
} VAFIRQ_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    VAFIRQ_IDX_TYPE         bREG;
} VAFIRQ_TYPE;

typedef struct {
    unsigned VN             :7;
    unsigned                :24;
    unsigned INV            :1;
} VNFIRQ_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    VNFIRQ_IDX_TYPE         bREG;
} VNFIRQ_TYPE;

typedef struct {
    unsigned                :26;
    unsigned IHD            :1;
    unsigned FHD            :1;
    unsigned IFLG           :1;
    unsigned FFLG           :1;
    unsigned FPOL           :1;
    unsigned RCL            :1;
} VCTRL_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    VCTRL_IDX_TYPE          bREG;
} VCTRL_TYPE;

typedef struct _VIC{
    volatile VAFIRQ_TYPE        VAIRQ;          // 0x200  R    0x800000XX   IRQ Vector Register
    volatile VAFIRQ_TYPE        VAFIQ;          // 0x204  R    0x800000XX   FIQ Vector Register
    volatile VNFIRQ_TYPE        VNIRQ;          // 0x208  R    0x800000XX   IRQ Vector Number Register
    volatile VNFIRQ_TYPE        VNFIQ;          // 0x20C  R    0x800000XX   FIQ Vector Number Register
    volatile VCTRL_TYPE         VCTRL;          // 0x210  R/W  0x00000000   Vector Control Register
    unsigned :32; unsigned :32; unsigned :32;
    volatile unsigned long      PRIO0;          // 0x220  R/W  0x03020100   Priorities for Interrupt 0 ~ 3
    volatile unsigned long      PRIO1;          // 0x224  R/W  0x07060504   Priorities for Interrupt 4 ~ 7
    volatile unsigned long      PRIO2;          // 0x228  R/W  0x0B0A0908   Priorities for Interrupt 8 ~ 11
    volatile unsigned long      PRIO3;          // 0x22C  R/W  0x0F0E0D0C   Priorities for Interrupt 12 ~ 15
    volatile unsigned long      PRIO4;          // 0x230  R/W  0x13121110   Priorities for Interrupt 16 ~ 19
    volatile unsigned long      PRIO5;          // 0x234  R/W  0x17161514   Priorities for Interrupt 20 ~ 23
    volatile unsigned long      PRIO6;          // 0x238  R/W  0x1B1A1918   Priorities for Interrupt 24 ~ 27
    volatile unsigned long      PRIO7;          // 0x23C  R/W  0x1F1E1D1C   Priorities for Interrupt 28 ~ 31
    volatile unsigned long      PRIO8;          // 0x240  R/W  0x23222120   Priorities for Interrupt 32 ~ 35
    volatile unsigned long      PRIO9;          // 0x244  R/W  0x27262524   Priorities for Interrupt 36 ~ 39
    volatile unsigned long      PRIO10;         // 0x248  R/W  0x2B2A2928   Priorities for Interrupt 40 ~ 43
    volatile unsigned long      PRIO11;         // 0x24C  R/W  0x2F2E2D2C   Priorities for Interrupt 44 ~ 47
    volatile unsigned long      PRIO12;         // 0x250  R/W  0x33323130   Priorities for Interrupt 48 ~ 51
    volatile unsigned long      PRIO13;         // 0x254  R/W  0x37363534   Priorities for Interrupt 52 ~ 55
    volatile unsigned long      PRIO14;         // 0x258  R/W  0x3B3A3938   Priorities for Interrupt 56 ~ 59
    volatile unsigned long      PRIO15;         // 0x25C  R/W  0x3F3E3D3C   Priorities for Interrupt 60 ~ 63
} VIC, *PVIC;

/************************************************************************
*   6. GPIO & Port Multiplexing                  (Base Addr = 0x74200000)
************************************************************************/

typedef struct {
    unsigned GP00           :1;
    unsigned GP01           :1;
    unsigned GP02           :1;
    unsigned GP03           :1;
    unsigned GP04           :1;
    unsigned GP05           :1;
    unsigned GP06           :1;
    unsigned GP07           :1;
    unsigned GP08           :1;
    unsigned GP09           :1;
    unsigned GP10           :1;
    unsigned GP11           :1;
    unsigned GP12           :1;
    unsigned GP13           :1;
    unsigned GP14           :1;
    unsigned GP15           :1;
    unsigned GP16           :1;
    unsigned GP17           :1;
    unsigned GP18           :1;
    unsigned GP19           :1;
    unsigned GP20           :1;
    unsigned GP21           :1;
    unsigned GP22           :1;
    unsigned GP23           :1;
    unsigned GP24           :1;
    unsigned GP25           :1;
    unsigned GP26           :1;
    unsigned GP27           :1;
    unsigned GP28           :1;
    unsigned GP29           :1;
    unsigned GP30           :1;
    unsigned GP31           :1;
} GPIO_PORT_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_PORT_IDX_TYPE      bREG;
} GPIO_PORT_TYPE;

typedef struct {
    unsigned GPCD00         :2;
    unsigned GPCD01         :2;
    unsigned GPCD02         :2;
    unsigned GPCD03         :2;
    unsigned GPCD04         :2;
    unsigned GPCD05         :2;
    unsigned GPCD06         :2;
    unsigned GPCD07         :2;
    unsigned GPCD08         :2;
    unsigned GPCD09         :2;
    unsigned GPCD10         :2;
    unsigned GPCD11         :2;
    unsigned GPCD12         :2;
    unsigned GPCD13         :2;
    unsigned GPCD14         :2;
    unsigned GPCD15         :2;
} GPIO_CD0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_CD0_IDX_TYPE       bREG;
} GPIO_CD0_TYPE;

typedef struct {
    unsigned GPCD16         :2;
    unsigned GPCD17         :2;
    unsigned GPCD18         :2;
    unsigned GPCD19         :2;
    unsigned GPCD20         :2;
    unsigned GPCD21         :2;
    unsigned GPCD22         :2;
    unsigned GPCD23         :2;
    unsigned GPCD24         :2;
    unsigned GPCD25         :2;
    unsigned GPCD26         :2;
    unsigned GPCD27         :2;
    unsigned GPCD28         :2;
    unsigned GPCD29         :2;
    unsigned GPCD30         :2;
    unsigned GPCD31         :2;
} GPIO_CD1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_CD1_IDX_TYPE       bREG;
} GPIO_CD1_TYPE;

typedef struct {
    unsigned GPFN00         :4;
    unsigned GPFN01         :4;
    unsigned GPFN02         :4;
    unsigned GPFN03         :4;
    unsigned GPFN04         :4;
    unsigned GPFN05         :4;
    unsigned GPFN06         :4;
    unsigned GPFN07         :4;
} GPIO_FN0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_FN0_IDX_TYPE       bREG;
} GPIO_FN0_TYPE;

typedef struct {
    unsigned GPFN08         :4;
    unsigned GPFN09         :4;
    unsigned GPFN10         :4;
    unsigned GPFN11         :4;
    unsigned GPFN12         :4;
    unsigned GPFN13         :4;
    unsigned GPFN14         :4;
    unsigned GPFN15         :4;
} GPIO_FN1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_FN1_IDX_TYPE       bREG;
} GPIO_FN1_TYPE;

typedef struct {
    unsigned GPFN16         :4;
    unsigned GPFN17         :4;
    unsigned GPFN18         :4;
    unsigned GPFN19         :4;
    unsigned GPFN20         :4;
    unsigned GPFN21         :4;
    unsigned GPFN22         :4;
    unsigned GPFN23         :4;
} GPIO_FN2_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_FN2_IDX_TYPE       bREG;
} GPIO_FN2_TYPE;

typedef struct {
    unsigned GPFN24         :4;
    unsigned GPFN25         :4;
    unsigned GPFN26         :4;
    unsigned GPFN27         :4;
    unsigned GPFN28         :4;
    unsigned GPFN29         :4;
    unsigned GPFN30         :4;
    unsigned GPFN31         :4;
} GPIO_FN3_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    GPIO_FN3_IDX_TYPE       bREG;
} GPIO_FN3_TYPE;

typedef struct {
    unsigned GPFN00         :4;
    unsigned GPFN01         :4;
    unsigned GPFN02         :4;
    unsigned GPFN03         :4;
    unsigned                :16;
} HDMI_FN_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    HDMI_FN_IDX_TYPE        bREG;
} HDMI_FN_TYPE;

typedef struct {
    unsigned                :8;
    unsigned GPFN02         :4;
    unsigned GPFN03         :4;
    unsigned GPFN04         :4;
    unsigned GPFN05         :4;
    unsigned                :8;
} ADC_FN_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    ADC_FN_IDX_TYPE         bREG;
} ADC_FN_TYPE;

typedef struct {
    unsigned EINT00SEL      :8;
    unsigned EINT01SEL      :8;
    unsigned EINT02SEL      :8;
    unsigned EINT03SEL      :8;
} EINTSEL0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    EINTSEL0_IDX_TYPE       bREG;
} EINTSEL0_TYPE;

typedef struct {
    unsigned EINT04SEL      :8;
    unsigned EINT05SEL      :8;
    unsigned EINT06SEL      :8;
    unsigned EINT07SEL      :8;
} EINTSEL1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    EINTSEL1_IDX_TYPE       bREG;
} EINTSEL1_TYPE;

typedef struct {
    unsigned EINT08SEL      :8;
    unsigned EINT09SEL      :8;
    unsigned EINT10SEL      :8;
    unsigned EINT11SEL      :8;
} EINTSEL2_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    EINTSEL2_IDX_TYPE       bREG;
} EINTSEL2_TYPE;

typedef struct _GPIO{
    volatile GPIO_PORT_TYPE     GPADAT;         // 0x000  R/W  0x00000000   GPA Data Register
    volatile GPIO_PORT_TYPE     GPAEN;          // 0x004  R/W  0x00000000   GPA Output Enable Register
    volatile GPIO_PORT_TYPE     GPASET;         // 0x008  W    -            OR function on GPA Output Data
    volatile GPIO_PORT_TYPE     GPACLR;         // 0x00C  W    -            BIC function on GPA Output Data
    volatile GPIO_PORT_TYPE     GPAXOR;         // 0x010  W    -            XOR function on GPA Output Data
    volatile GPIO_CD0_TYPE      GPACD0;         // 0x014  R/W  0x00000000   Driver strength Control on GPA
    volatile GPIO_CD1_TYPE      GPACD1;         // 0x018  R/W  0x00000000   Driver strength Control on GPA
    volatile GPIO_PORT_TYPE     GPAPE;          // 0x01C  R/W  0x00000000   Pull-Up/Down Enable function on GPA
    volatile GPIO_PORT_TYPE     GPAPS;          // 0x020  R/W  0x00000000   Pull-Up/Down Selection function on GPA
    volatile GPIO_PORT_TYPE     GPAIEN;         // 0x024  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPA
    volatile GPIO_PORT_TYPE     GPAIS;          // 0x028  R/W  0x00000000   Schimitt input Function on GPA
    volatile GPIO_PORT_TYPE     GPASR;          // 0x02C  R/W  0x00000000   Fast Slew Rate Function GPA
    volatile GPIO_FN0_TYPE      GPAFN0;         // 0x030  R/W  0x00000000   Port Configuration on GPA Output Data
    volatile GPIO_FN1_TYPE      GPAFN1;         // 0x034  R/W  0x00000000   Port Configuration on GPA Output Data
    volatile GPIO_FN2_TYPE      GPAFN2;         // 0x038  R/W  0x00000000   Port Configuration on GPA Output Data
    volatile GPIO_FN3_TYPE      GPAFN3;         // 0x03C  R/W  0x00000000   Port Configuration on GPA Output Data

    volatile GPIO_PORT_TYPE     GPBDAT;         // 0x040  R/W  0x00000000   GPB Data Register
    volatile GPIO_PORT_TYPE     GPBEN;          // 0x044  R/W  0x00000000   GPB Output Enable Register
    volatile GPIO_PORT_TYPE     GPBSET;         // 0x048  W    -            OR function on GPB Output Data
    volatile GPIO_PORT_TYPE     GPBCLR;         // 0x04C  W    -            BIC function on GPB Output Data
    volatile GPIO_PORT_TYPE     GPBXOR;         // 0x050  W    -            XOR function on GPB Output Data
    volatile GPIO_CD0_TYPE      GPBCD0;         // 0x054  R/W  0x00000000   Driver strength Control on GPB
    volatile GPIO_CD1_TYPE      GPBCD1;         // 0x058  R/W  0x00000000   Driver strength Control on GPB
    volatile GPIO_PORT_TYPE     GPBPE;          // 0x05C  R/W  0x00000000   Pull-Up/Down Enable function on GPB
    volatile GPIO_PORT_TYPE     GPBPS;          // 0x060  R/W  0x00000000   Pull-Up/Down Selection function on GPB
    volatile GPIO_PORT_TYPE     GPBIEN;         // 0x064  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPB
    volatile GPIO_PORT_TYPE     GPBIS;          // 0x068  R/W  0x00000000   Schimitt input Function on GPB
    volatile GPIO_PORT_TYPE     GPBSR;          // 0x06C  R/W  0x00000000   Fast Slew Rate Function GPB
    volatile GPIO_FN0_TYPE      GPBFN0;         // 0x070  R/W  0x00000000   Port Configuration on GPB Output Data
    volatile GPIO_FN1_TYPE      GPBFN1;         // 0x074  R/W  0x00000000   Port Configuration on GPB Output Data
    volatile GPIO_FN2_TYPE      GPBFN2;         // 0x078  R/W  0x00000000   Port Configuration on GPB Output Data
    volatile GPIO_FN3_TYPE      GPBFN3;         // 0x07C  R/W  0x00000000   Port Configuration on GPB Output Data

    volatile GPIO_PORT_TYPE     GPCDAT;         // 0x080  R/W  0x00000000   GPC Data Register
    volatile GPIO_PORT_TYPE     GPCEN;          // 0x084  R/W  0x00000000   GPC Output Enable Register
    volatile GPIO_PORT_TYPE     GPCSET;         // 0x088  W    -            OR function on GPC Output Data
    volatile GPIO_PORT_TYPE     GPCCLR;         // 0x08C  W    -            BIC function on GPC Output Data
    volatile GPIO_PORT_TYPE     GPCXOR;         // 0x090  W    -            XOR function on GPC Output Data
    volatile GPIO_CD0_TYPE      GPCCD0;         // 0x094  R/W  0x00000000   Driver strength Control 0 on GPC
    volatile GPIO_CD1_TYPE      GPCCD1;         // 0x098  R/W  0x00000000   Driver strength Control 0 on GPC
    volatile GPIO_PORT_TYPE     GPCPE;          // 0x09C  R/W  0x00000000   Pull-Up/Down Enable function on GPC
    volatile GPIO_PORT_TYPE     GPCPS;          // 0x0A0  R/W  0x00000000   Pull-Up/Down Selection function on GPC
    volatile GPIO_PORT_TYPE     GPCIEN;         // 0x0A4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPC
    volatile GPIO_PORT_TYPE     GPCIS;          // 0x0A8  R/W  0x00000000   Schimitt input Function on GPC
    volatile GPIO_PORT_TYPE     GPCSR;          // 0x0AC  R/W  0x00000000   Fast Slew Rate Function GPC
    volatile GPIO_FN0_TYPE      GPCFN0;         // 0x0B0  R/W  0x00000000   Port Configuration on GPC Output Data
    volatile GPIO_FN1_TYPE      GPCFN1;         // 0x0B4  R/W  0x00000000   Port Configuration on GPC Output Data
    volatile GPIO_FN2_TYPE      GPCFN2;         // 0x0B8  R/W  0x00000000   Port Configuration on GPC Output Data
    volatile GPIO_FN3_TYPE      GPCFN3;         // 0x0BC  R/W  0x00000000   Port Configuration on GPC Output Data

    volatile GPIO_PORT_TYPE     GPDDAT;         // 0x0C0  R/W  0x00000000   GPD Data Register
    volatile GPIO_PORT_TYPE     GPDEN;          // 0x0C4  R/W  0x00000000   GPD Output Enable Register
    volatile GPIO_PORT_TYPE     GPDSET;         // 0x0C8  W    -            OR function on GPD Output Data
    volatile GPIO_PORT_TYPE     GPDCLR;         // 0x0CC  W    -            BIC function on GPD Output Data
    volatile GPIO_PORT_TYPE     GPDXOR;         // 0x0D0  W    -            XOR function on GPD Output Data
    volatile GPIO_CD0_TYPE      GPDCD0;         // 0x0D4  R/W  0x00000000   Driver strength Control 0 on GPD
    volatile GPIO_CD1_TYPE      GPDCD1;         // 0x0D8  R/W  0x00000000   Driver strength Control 0 on GPD
    volatile GPIO_PORT_TYPE     GPDPE;          // 0x0DC  R/W  0x00000000   Pull-Up/Down Enable function on GPD
    volatile GPIO_PORT_TYPE     GPDPS;          // 0x0E0  R/W  0x00000000   Pull-Up/Down Selection function on GPD
    volatile GPIO_PORT_TYPE     GPDIEN;         // 0x0E4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPD
    volatile GPIO_PORT_TYPE     GPDIS;          // 0x0E8  R/W  0x00000000   Schimitt input Function on GPD
    volatile GPIO_PORT_TYPE     GPDSR;          // 0x0EC  R/W  0x00000000   Fast Slew Rate Function GPD
    volatile GPIO_FN0_TYPE      GPDFN0;         // 0x0F0  R/W  0x00000000   Port Configuration on GPD Output Data
    volatile GPIO_FN1_TYPE      GPDFN1;         // 0x0F4  R/W  0x00000000   Port Configuration on GPD Output Data
    volatile GPIO_FN2_TYPE      GPDFN2;         // 0x0F8  R/W  0x00000000   Port Configuration on GPD Output Data
    volatile GPIO_FN3_TYPE      GPDFN3;         // 0x0FC  R/W  0x00000000   Port Configuration on GPD Output Data

    volatile GPIO_PORT_TYPE     GPEDAT;         // 0x100  R/W  0x00000000   GPE Data Register
    volatile GPIO_PORT_TYPE     GPEEN;          // 0x104  R/W  0x00000000   GPE Output Enable Register
    volatile GPIO_PORT_TYPE     GPESET;         // 0x108  W    -            OR function on GPE Output Data
    volatile GPIO_PORT_TYPE     GPECLR;         // 0x10C  W    -            BIC function on GPE Output Data
    volatile GPIO_PORT_TYPE     GPEXOR;         // 0x110  W    -            XOR function on GPE Output Data
    volatile GPIO_CD0_TYPE      GPECD0;         // 0x114  R/W  0x00000000   Driver strength Control 0 on GPE
    volatile GPIO_CD1_TYPE      GPECD1;         // 0x118  R/W  0x00000000   Driver strength Control 0 on GPE
    volatile GPIO_PORT_TYPE     GPEPE;          // 0x11C  R/W  0x00000000   Pull-Up/Down Enable function on GPE
    volatile GPIO_PORT_TYPE     GPEPS;          // 0x120  R/W  0x00000000   Pull-Up/Down Selection function on GPE
    volatile GPIO_PORT_TYPE     GPEIEN;         // 0x124  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPE
    volatile GPIO_PORT_TYPE     GPEIS;          // 0x128  R/W  0x00000000   Schimitt input Function on GPE
    volatile GPIO_PORT_TYPE     GPESR;          // 0x12C  R/W  0x00000000   Fast Slew Rate Function GPE
    volatile GPIO_FN0_TYPE      GPEFN0;         // 0x130  R/W  0x00000000   Port Configuration on GPE Output Data
    volatile GPIO_FN1_TYPE      GPEFN1;         // 0x134  R/W  0x00000000   Port Configuration on GPE Output Data
    volatile GPIO_FN2_TYPE      GPEFN2;         // 0x138  R/W  0x00000000   Port Configuration on GPE Output Data
    volatile GPIO_FN3_TYPE      GPEFN3;         // 0x13C  R/W  0x00000000   Port Configuration on GPE Output Data

    volatile GPIO_PORT_TYPE     GPFDAT;         // 0x140  R/W  0x00000000   GPF Data Register
    volatile GPIO_PORT_TYPE     GPFEN;          // 0x144  R/W  0x00000000   GPF Output Enable Register
    volatile GPIO_PORT_TYPE     GPFSET;         // 0x148  W    -            OR function on GPF Output Data
    volatile GPIO_PORT_TYPE     GPFCLR;         // 0x14C  W    -            BIC function on GPF Output Data
    volatile GPIO_PORT_TYPE     GPFXOR;         // 0x150  W    -            XOR function on GPF Output Data
    volatile GPIO_CD0_TYPE      GPFCD0;         // 0x154  R/W  0x00000000   Driver strength Control 0 on GPF
    volatile GPIO_CD1_TYPE      GPFCD1;         // 0x158  R/W  0x00000000   Driver strength Control 0 on GPF
    volatile GPIO_PORT_TYPE     GPFPE;          // 0x15C  R/W  0x00000000   Pull-Up/Down Enable function on GPF
    volatile GPIO_PORT_TYPE     GPFPS;          // 0x160  R/W  0x00000000   Pull-Up/Down Selection function on GPF
    volatile GPIO_PORT_TYPE     GPFIEN;         // 0x164  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPF
    volatile GPIO_PORT_TYPE     GPFIS;          // 0x168  R/W  0x00000000   Schimitt input Function on GPF
    volatile GPIO_PORT_TYPE     GPFSR;          // 0x16C  R/W  0x00000000   Fast Slew Rate Function GPF
    volatile GPIO_FN0_TYPE      GPFFN0;         // 0x170  R/W  0x00000000   Port Configuration on GPF Output Data
    volatile GPIO_FN1_TYPE      GPFFN1;         // 0x174  R/W  0x00000000   Port Configuration on GP Output Data
    volatile GPIO_FN2_TYPE      GPFFN2;         // 0x178  R/W  0x00000000   Port Configuration on GPF Output Data
    volatile GPIO_FN3_TYPE      GPFFN3;         // 0x17C  R/W  0x00000000   Port Configuration on GPF Output Data

    volatile GPIO_PORT_TYPE     GPGDAT;         // 0x180  R/W  0x00000000   GPG Data Register
    volatile GPIO_PORT_TYPE     GPGEN;          // 0x184  R/W  0x00000000   GPG Output Enable Register
    volatile GPIO_PORT_TYPE     GPGSET;         // 0x188  W    -            OR function on GPG Output Data
    volatile GPIO_PORT_TYPE     GPGCLR;         // 0x18C  W    -            BIC function on GPG Output Data
    volatile GPIO_PORT_TYPE     GPGXOR;         // 0x190  W    -            XOR function on GPG Output Data
    volatile GPIO_CD0_TYPE      GPGCD0;         // 0x194  R/W  0x00000000   Driver strength Control 0 on GPG
    volatile GPIO_CD1_TYPE      GPGCD1;         // 0x198  R/W  0x00000000   Driver strength Control 0 on GPG
    volatile GPIO_PORT_TYPE     GPGPE;          // 0x19C  R/W  0x00000000   Pull-Up/Down Enable function on GPG
    volatile GPIO_PORT_TYPE     GPGPS;          // 0x1A0  R/W  0x00000000   Pull-Up/Down Selection function on GPG
    volatile GPIO_PORT_TYPE     GPGIEN;         // 0x1A4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPG
    volatile GPIO_PORT_TYPE     GPGIS;          // 0x1A8  R/W  0x00000000   Schimitt input Function on GPG
    volatile GPIO_PORT_TYPE     GPGSR;          // 0x1AC  R/W  0x00000000   Fast Slew Rate Function GPG
    volatile GPIO_FN0_TYPE      GPGFN0;         // 0x1B0  R/W  0x00000000   Port Configuration on GPG Output Data
    volatile GPIO_FN1_TYPE      GPGFN1;         // 0x1B4  R/W  0x00000000   Port Configuration on GPG Output Data
    volatile GPIO_FN2_TYPE      GPGFN2;         // 0x1B8  R/W  0x00000000   Port Configuration on GPG Output Data
    volatile GPIO_FN3_TYPE      GPGFN3;         // 0x1BC  R/W  0x00000000   Port Configuration on GPG Output Data

    volatile GPIO_PORT_TYPE     GPHDMIDAT;      // 0x1C0  R/W  0x00000000   GPHDMI Data Register
    volatile GPIO_PORT_TYPE     GPHDMIEN;       // 0x1C4  R/W  0x00000000   GPHDMI Output Enable Register
    volatile GPIO_PORT_TYPE     GPHDMISET;      // 0x1C8  W    -            OR function on GPHDMI Output Data
    volatile GPIO_PORT_TYPE     GPHDMICLR;      // 0x1CC  W    -            BIC function on GPHDMI Output Data
    volatile GPIO_PORT_TYPE     GPHDMIXOR;      // 0x1D0  W    -            XOR function on GPHDMI Output Data
    volatile GPIO_CD0_TYPE      GPHDMICD;       // 0x1D4  R/W  0x00000000   Driver strength Control on GPHDMI
    unsigned :32;
    volatile GPIO_PORT_TYPE     GPHDMIPE;       // 0x1DC  R/W  0x00000000   Pull-Up/Down Enable function on GPHDMI
    volatile GPIO_PORT_TYPE     GPHDMIPS;       // 0x1E0  R/W  0x00000000   Pull-Up/Down Selection function on GPHDMI
    volatile GPIO_PORT_TYPE     GPHDMIIEN;      // 0x1E4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPHDMI
    volatile GPIO_PORT_TYPE     GPHDMIIS;       // 0x1E8  R/W  0x00000000   Schimitt input Function on GPHDMI
    volatile GPIO_PORT_TYPE     GPHDMISR;       // 0x1EC  R/W  0x00000000   Fast Slew Rate Function GPHDMI
    volatile HDMI_FN_TYPE       GPHDMIFN0;      // 0x1F0  R/W  0x00000000   Port Configuration on GPHDMI Output Data
    unsigned :32; unsigned :32; unsigned :32;

    volatile GPIO_PORT_TYPE     GPADCDAT;       // 0x200  R/W  0x00000000   GPADC Data Register
    volatile GPIO_PORT_TYPE     GPADCEN;        // 0x204  R/W  0x00000000   GPADC Output Enable Register
    volatile GPIO_PORT_TYPE     GPADCSET;       // 0x208  W    -            OR function on GPADC Output Data
    volatile GPIO_PORT_TYPE     GPADCCLR;       // 0x20C  W    -            BIC function on GPADC Output Data
    volatile GPIO_PORT_TYPE     GPADCXOR;       // 0x210  W    -            XOR function on GPADC output Data
    volatile GPIO_CD0_TYPE      GPADCCD;        // 0x214  R/W  0x00000000   Driver strength Control on GPADC
    unsigned :32;
    volatile GPIO_PORT_TYPE     GPADCPE;        // 0x21C  R/W  0x00000000   Pull-Up/Down Enable function on GPADC
    volatile GPIO_PORT_TYPE     GPADCPS;        // 0x220  R/W  0x00000000   Pull-Up/Down Selection function on GPADC
    volatile GPIO_PORT_TYPE     GPADCIEN;       // 0x224  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPADC
    volatile GPIO_PORT_TYPE     GPADCIS;        // 0x228  R/W  0x00000000   Schimitt Input Function on GPADC
    volatile GPIO_PORT_TYPE     GPADCSR;        // 0x22C  R/W  0x00000000   Fast Slew Rate Function on GPADC
    volatile ADC_FN_TYPE        GPADCFN0;       // 0x230  R/W  0x00000000   Port Configuration on GPADC Output Data
    unsigned :32; unsigned :32; unsigned :32;

    volatile GPIO_PORT_TYPE     GPSDDAT;        // 0x240
    volatile GPIO_PORT_TYPE     GPSDEN;         // 0x244
    volatile GPIO_PORT_TYPE     GPSDSET;        // 0x248
    volatile GPIO_PORT_TYPE     GPSDCLR;        // 0x24C
    volatile GPIO_PORT_TYPE     GPSDXOR;        // 0x250
    volatile GPIO_CD0_TYPE      GPSDCD;         // 0x254
    unsigned :32;
    volatile GPIO_PORT_TYPE     GPSDPE;         // 0x25C
    volatile GPIO_PORT_TYPE     GPSDPS;         // 0x260
    volatile GPIO_PORT_TYPE     GPSDIEN;        // 0x264
    volatile GPIO_PORT_TYPE     GPSDIS;         // 0x268
    volatile GPIO_PORT_TYPE     GPSDSR;         // 0x26C
    volatile GPIO_FN0_TYPE      GPSDFN0;        // 0x270
    unsigned :32; unsigned :32; unsigned :32;

    volatile EINTSEL0_TYPE      EINTSEL0;       // 0x280  R/W  -            External Interrupt Select Register 0
    volatile EINTSEL1_TYPE      EINTSEL1;       // 0x284  R/W  -            External Interrupt Select Register 1
    volatile EINTSEL2_TYPE      EINTSEL2;       // 0x288  R/W  -            External Interrupt Select Register 2
    unsigned :32;

    volatile unsigned long      ECID0;          // 0x290  R/W  -            CID Output Register
    volatile unsigned long      ECID1;          // 0x294  R    -            CID Serial output data Register
    volatile unsigned long      ECID2;          // 0x298  R    -            CID Parallel output data 0 Register
    volatile unsigned long      ECID3;          // 0x29C  R    -            CID Parallel output data 1 Register
} GPIO, *PGPIO;

typedef struct _GPION{
    volatile GPIO_PORT_TYPE     GPDAT;          // 0x000  R/W  0x00000000   GP Data Register
    volatile GPIO_PORT_TYPE     GPEN;           // 0x004  R/W  0x00000000   GP Output Enable Register
    volatile GPIO_PORT_TYPE     GPSET;          // 0x008  W    -            OR function on GP Output Data
    volatile GPIO_PORT_TYPE     GPCLR;          // 0x00C  W    -            BIC function on GP Output Data
    volatile GPIO_PORT_TYPE     GPXOR;          // 0x010  W    -            XOR function on GP Output Data
    volatile GPIO_CD0_TYPE      GPCD0;          // 0x014  R/W  0x00000000   Driver strength Control on GP
    volatile GPIO_CD1_TYPE      GPCD1;          // 0x018  R/W  0x00000000   Driver strength Control on GP
    volatile GPIO_PORT_TYPE     GPPE;           // 0x01C  R/W  0x00000000   Pull-Up/Down Enable function on GP
    volatile GPIO_PORT_TYPE     GPPS;           // 0x020  R/W  0x00000000   Pull-Up/Down Selection function on GP
    volatile GPIO_PORT_TYPE     GPIEN;          // 0x024  R/W  0xFFFFFFFF   Input Buffer Enable Function on GP
    volatile GPIO_PORT_TYPE     GPIS;           // 0x028  R/W  0x00000000   Schimitt input Function on GP
    volatile GPIO_PORT_TYPE     GPSR;           // 0x02C  R/W  0x00000000   Fast Slew Rate Function GP
    volatile GPIO_FN0_TYPE      GPFN0;          // 0x030  R/W  0x00000000   Port Configuration on GP Output Data
    volatile GPIO_FN1_TYPE      GPFN1;          // 0x034  R/W  0x00000000   Port Configuration on GP Output Data
    volatile GPIO_FN2_TYPE      GPFN2;          // 0x038  R/W  0x00000000   Port Configuration on GP Output Data
    volatile GPIO_FN3_TYPE      GPFN3;          // 0x03C  R/W  0x00000000   Port Configuration on GP Output Data
} GPION, *PGPION;


/************************************************************************
*   etc.
************************************************************************/
//Bruce_893x ???
typedef struct _NAND_PIC{
    volatile unsigned int   IEN0;               // 0x000  R/W  0x00000000   Interrupt Enable0 Register
    volatile unsigned int   IEN1;               // 0x004  R/W  0x00000000   Interrupt Enable1 Register
    volatile unsigned int   CLR0;               // 0x008  R/W  0x00000000   Interrupt Clear0 Register
    volatile unsigned int   CLR1;               // 0x00C  R/W  0x00000000   Interrupt Clear1 Register
    volatile unsigned int   STS0;               // 0x010  R    Unknown      Interrupt Status0 Register
    volatile unsigned int   STS1;               // 0x014  R    Unknown      Interrupt Status1 Register
    volatile unsigned int   SEL0;               // 0x018  R/W  0x00000000   IRQ or FIR Selection0 Register
    volatile unsigned int   SEL1;               // 0x01C  R/W  0x00000000   IRQ or FIR Selection1 Register
    volatile unsigned int   SRC0;               // 0x020  R    Unknown      Source Interrupt Status0 Register
    volatile unsigned int   SRC1;               // 0x024  R    Unknown      Source Interrupt Status1 Register
    volatile unsigned int   MSTS0;              // 0x028  R    0x00000000   Masked Status0 Register
    volatile unsigned int   MSTS1;              // 0x02C  R    0x00000000   Masked Status1 Register
    volatile unsigned int   TIG0;               // 0x030  R/W  0x00000000   Test Interrupt Generation0 Register
    volatile unsigned int   TIG1;               // 0x034  R/W  0x00000000   Test Interrupt Generation1 Register
    volatile unsigned int   POL0;               // 0x038  R/W  0x00000000   Interrupt Polarity0 Register
    volatile unsigned int   POL1;               // 0x03C  R/W  0x00000000   Interrupt Polarity1 Register
    volatile unsigned int   IRQ0;               // 0x040  R    0x00000000   IRQ Raw Status0 Register
    volatile unsigned int   IRQ1;               // 0x044  R    0x00000000   IRQ Raw Status1 Register
    volatile unsigned int   FIQ0;               // 0x048  R    Unknown      FIQ Status0 Register
    volatile unsigned int   FIQ1;               // 0x04C  R    Unknown      FIQ Status1 Register
    volatile unsigned int   MIRQ0;              // 0x050  R    0x00000000   Masked IRQ Status0 Register
    volatile unsigned int   MIRQ1;              // 0x054  R    0x00000000   Masked IRQ Status1 Register
    volatile unsigned int   MFIQ0;              // 0x058  R    0x00000000   Masked FIQ Status0 Register
    volatile unsigned int   MFIQ1;              // 0x05C  R    0x00000000   Masked FIQ Status1 Register
    volatile unsigned int   MODE0;              // 0x060  R/W  0x00000000   Trigger Mode0 Register ? Level or Edge
    volatile unsigned int   MODE1;              // 0x064  R/W  0x00000000   Trigger Mode1 Register ? Level or Edge
    volatile unsigned int   SYNC0;              // 0x068  R/W  0xFFFFFFFF   Synchronization Enable0 Register
    volatile unsigned int   SYNC1;              // 0x06C  R/W  0xFFFFFFFF   Synchronization Enable1 Register
    volatile unsigned int   WKEN0;              // 0x070  R/W  0x00000000   Wakeup Event Enable0 Register
    volatile unsigned int   WKEN1;              // 0x074  R/W  0x00000000   Wakeup Event Enable1 Register
    volatile unsigned int   MODEA0;             // 0x078  R/W  0x00000000   Both Edge or Single Edge0 Register
    volatile unsigned int   MODEA1;             // 0x07C  R/W  0x00000000   Both Edge or Single Edge1 Register
    volatile unsigned int   EI37SEL;            // 0x080  R/W  0x00000000   External INT 3~7Selection Register
    volatile unsigned int   NOTDEFINE0[31];     //-  0x84-0xFC              Reserved
    volatile unsigned int   INTMSK0;            // 0x100  R/W  0xFFFFFFFF   Interrupt Output Masking0 Register
    volatile unsigned int   INTMSK1;            // 0x104  R/W  0xFFFFFFFF   Interrupt Output Masking1 Register
    volatile unsigned int   ALLMSK;             // 0x108  R/W  0x00000003   All Mask Register
} NAND_PIC, *NAND_PPIC;


/************************************************************************
*   6. GPIO & Port Multiplexing (Base Addr = 0x74200000)
************************************************************************/
typedef struct _NAND_GPIO{
    volatile unsigned int   GPADAT;             // 0x000  R/W  0x00000000   GPA Data Register 
    volatile unsigned int   GPAEN;              // 0x004  R/W  0x00000000   GPA Output Enable Register 
    volatile unsigned int   GPASET;             // 0x008  W    -            OR function on GPA Output Data 
    volatile unsigned int   GPACLR;             // 0x00C  W    -            BIC function on GPA Output Data 
    volatile unsigned int   GPAXOR;             // 0x010  W    -            XOR function on GPA Output Data 
    volatile unsigned int   GPACD0;             // 0x014  R/W  0x00000000   Driver strength Control 0 on GPA 
    volatile unsigned int   GPACD1;             // 0x018  R/W  0x00000000   Driver strength Control 1 on GPA 
    volatile unsigned int   GPAPE;              // 0x01C  R/W  0x00000000   Pull-Up/Down Enable function on GPA 
    volatile unsigned int   GPAPS;              // 0x020  R/W  0x00000000   Pull-Up/Down Selection function on GPA 
    volatile unsigned int   GPAIEN;             // 0x024  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPA 
    volatile unsigned int   GPAIS;              // 0x028  R/W  0x00000000   Schimitt Input Function on GPA 
    volatile unsigned int   GPASR;              // 0x02C  R/W  0x00000000   Fast Slew Rate Function GPA 
    volatile unsigned int   GPAFN0;             // 0x030  R/W  0x00000000   Port Configuration on GPA Output Data 
    volatile unsigned int   GPAFN1;             // 0x034  R/W  0x00000000   Port Configuration on GPA Output Data 
    volatile unsigned int   GPAFN2;             // 0x038  R/W  0x00000000   Port Configuration on GPA Output Data 
    volatile unsigned int   GPAFN3;             // 0x03C  R/W  0x00000000   Port Configuration on GPA Output Data 
    volatile unsigned int   GPBDAT;             // 0x040  R/W  0x00000000   GPB Data Register 
    volatile unsigned int   GPBEN;              // 0x044  R/W  0x00000000   GPB Output Enable Register 
    volatile unsigned int   GPBSET;             // 0x048  W    -            OR function on GPB Output Data 
    volatile unsigned int   GPBCLR;             // 0x04C  W    -            BIC function on GPB Output Data 
    volatile unsigned int   GPBXOR;             // 0x050  W    -            XOR function on GPB Output Data 
    volatile unsigned int   GPBCD0;             // 0x054  R/W  0x00000000   Driver strength Control 0 on GPB 
    volatile unsigned int   GPBCD1;             // 0x058  R/W  0x00000000   Driver strength Control 1 on GPB 
    volatile unsigned int   GPBPE;              // 0x05C  R/W  0x00000000   Pull-Up/Down Enable function on GPB 
    volatile unsigned int   GPBPS;              // 0x060  R/W  0x00000000   Pull-Up/Down Selection function on GPB 
    volatile unsigned int   GPBIEN;             // 0x064  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPB 
    volatile unsigned int   GPBIS;              // 0x068  R/W  0x00000000   Schimitt Input Function on GPB 
    volatile unsigned int   GPBSR;              // 0x06C  R/W  0x00000000   Fast Slew Rate Function GPB 
    volatile unsigned int   GPBFN0;             // 0x070  R/W  0x00000000   Port Configuration on GPB Output Data 
    volatile unsigned int   GPBFN1;             // 0x074  R/W  0x00000000   Port Configuration on GPB Output Data 
    volatile unsigned int   GPBFN2;             // 0x078  R/W  0x00000000   Port Configuration on GPB Output Data 
    volatile unsigned int   GPBFN3;             // 0x07C  R/W  0x00000000   Port Configuration on GPB Output Data 
    volatile unsigned int   GPCDAT;             // 0x080  R/W  0x00000000   GPC Data Register 
    volatile unsigned int   GPCEN;              // 0x084  R/W  0x00000000   GPC Output Enable Register 
    volatile unsigned int   GPCSET;             // 0x088  W    -            OR function on GPC Output Data 
    volatile unsigned int   GPCCLR;             // 0x08C  W    -            BIC function on GPC Output Data 
    volatile unsigned int   GPCXOR;             // 0x090  W    -            XOR function on GPC Output Data 
    volatile unsigned int   GPCCD0;             // 0x094  R/W  0x00000000   Driver strength Control 0 on GPC 
    volatile unsigned int   GPCCD1;             // 0x098  R/W  0x00000000   Driver strength Control 1 on GPC 
    volatile unsigned int   GPCPE;              // 0x09C  R/W  0x00000000   Pull-Up/Down Enable function on GPC 
    volatile unsigned int   GPCPS;              // 0x0A0  R/W  0x00000000   Pull-Up/Down Selection function on GPC 
    volatile unsigned int   GPCIEN;             // 0x0A4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPC 
    volatile unsigned int   GPCIS;              // 0x0A8  R/W  0x00000000   Schimitt Input Function on GPC 
    volatile unsigned int   GPCSR;              // 0x0AC  R/W  0x00000000   Fast Slew Rate Function GPC 
    volatile unsigned int   GPCFN0;             // 0x0B0  R/W  0x00000000   Port Configuration on GPC Output Data 
    volatile unsigned int   GPCFN1;             // 0x0B4  R/W  0x00000000   Port Configuration on GPC Output Data 
    volatile unsigned int   GPCFN2;             // 0x0B8  R/W  0x00000000   Port Configuration on GPC Output Data 
    volatile unsigned int   GPCFN3;             // 0x0BC  R/W  0x00000000   Port Configuration on GPC Output Data 
    volatile unsigned int   GPDDAT;             // 0x0C0  R/W  0x00000000   GPD Data Register 
    volatile unsigned int   GPDEN;              // 0x0C4  R/W  0x00000000   GPD Output Enable Register 
    volatile unsigned int   GPDSET;             // 0x0C8  W    -            OR function on GPD Output Data 
    volatile unsigned int   GPDCLR;             // 0x0CC  W    -            BIC function on GPD Output Data 
    volatile unsigned int   GPDXOR;             // 0x0D0  W    -            XOR function on GPD Output Data 
    volatile unsigned int   GPDCD0;             // 0x0D4  R/W  0x00000000   Driver strength Control 0 on GPD 
    volatile unsigned int   GPDCD1;             // 0x0D8  R/W  0x00000000   Driver strength Control 1 on GPD 
    volatile unsigned int   GPDPE;              // 0x0DC  R/W  0x00000000   Pull-Up/Down Enable function on GPD 
    volatile unsigned int   GPDPS;              // 0x0E0  R/W  0x00000000   Pull-Up/Down Selection function on GPD 
    volatile unsigned int   GPDIEN;             // 0x0E4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPD 
    volatile unsigned int   GPDIS;              // 0x0E8  R/W  0x00000000   Schimitt Input Function on GPD 
    volatile unsigned int   GPDSR;              // 0x0EC  R/W  0x00000000   Fast Slew Rate Function GPD 
    volatile unsigned int   GPDFN0;             // 0x0F0  R/W  0x00000000   Port Configuration on GPD Output Data 
    volatile unsigned int   GPDFN1;             // 0x0F4  R/W  0x00000000   Port Configuration on GPD Output Data 
    volatile unsigned int   GPDFN2;             // 0x0F8  R/W  0x00000000   Port Configuration on GPD Output Data 
    volatile unsigned int   GPDFN3;             // 0x0FC  R/W  0x00000000   Port Configuration on GPD Output Data 
    volatile unsigned int   GPEDAT;             // 0x100  R/W  0x00000000   GPE Data Register 
    volatile unsigned int   GPEEN;              // 0x104  R/W  0x00000000   GPE Output Enable Register 
    volatile unsigned int   GPESET;             // 0x108  W    -            OR function on GPE Output Data 
    volatile unsigned int   GPECLR;             // 0x10C  W    -            BIC function on GPE Output Data 
    volatile unsigned int   GPEXOR;             // 0x110  W    -            XOR function on GPE Output Data 
    volatile unsigned int   GPECD0;             // 0x114  R/W  0x00000000   Driver strength Control 0 on GPE 
    volatile unsigned int   GPECD1;             // 0x118  R/W  0x00000000   Driver strength Control 1 on GPE 
    volatile unsigned int   GPEPE;              // 0x11C  R/W  0x00000000   Pull-Up/Down Enable function on GPE 
    volatile unsigned int   GPEPS;              // 0x120  R/W  0x00000000   Pull-Up/Down Selection function on GPE 
    volatile unsigned int   GPEIEN;             // 0x124  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPE 
    volatile unsigned int   GPEIS;              // 0x128  R/W  0x00000000   Schimitt Input Function on GPE 
    volatile unsigned int   GPESR;              // 0x12C  R/W  0x00000000   Fast Slew Rate Function GPE 
    volatile unsigned int   GPEFN0;             // 0x130  R/W  0x00000000   Port Configuration on GPE Output Data 
    volatile unsigned int   GPEFN1;             // 0x134  R/W  0x00000000   Port Configuration on GPE Output Data 
    volatile unsigned int   GPEFN2;             // 0x138  R/W  0x00000000   Port Configuration on GPE Output Data 
    volatile unsigned int   GPEFN3;             // 0x13C  R/W  0x00000000   Port Configuration on GPE Output Data 
    volatile unsigned int   GPFDAT;             // 0x140  R/W  0x00000000   GPF Data Register 
    volatile unsigned int   GPFEN;              // 0x144  R/W  0x00000000   GPF Output Enable Register 
    volatile unsigned int   GPFSET;             // 0x148  W    -            OR function on GPF Output Data 
    volatile unsigned int   GPFCLR;             // 0x14C  W    -            BIC function on GPF Output Data 
    volatile unsigned int   GPFXOR;             // 0x150  W    -            XOR function on GPF Output Data 
    volatile unsigned int   GPFCD0;             // 0x154  R/W  0x00000000   Driver strength Control 0 on GPF 
    volatile unsigned int   GPFCD1;             // 0x158  R/W  0x00000000   Driver strength Control 1 on GPF 
    volatile unsigned int   GPFPE;              // 0x15C  R/W  0x00000000   Pull-Up/Down Enable function on GPF 
    volatile unsigned int   GPFPS;              // 0x160  R/W  0x00000000   Pull-Up/Down Selection function on GPF 
    volatile unsigned int   GPFIEN;             // 0x164  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPF 
    volatile unsigned int   GPFIS;              // 0x168  R/W  0x00000000   Schimitt Input Function on GPF 
    volatile unsigned int   GPFSR;              // 0x16C  R/W  0x00000000   Fast Slew Rate Function GPF 
    volatile unsigned int   GPFFN0;             // 0x170  R/W  0x00000000   Port Configuration on GPF Output Data 
    volatile unsigned int   GPFFN1;             // 0x174  R/W  0x00000000   Port Configuration on GPF Output Data 
    volatile unsigned int   GPFFN2;             // 0x178  R/W  0x00000000   Port Configuration on GPF Output Data 
    volatile unsigned int   GPFFN3;             // 0x17C  R/W  0x00000000   Port Configuration on GPF Output Data 
    volatile unsigned int   GPGDAT;             // 0x180  R/W  0x00000000   GPG Data Register 
    volatile unsigned int   GPGEN;              // 0x184  R/W  0x00000000   GPG Output Enable Register 
    volatile unsigned int   GPGSET;             // 0x188  W    -            OR function on GPG Output Data 
    volatile unsigned int   GPGCLR;             // 0x18C  W    -            BIC function on GPG Output Data 
    volatile unsigned int   GPGXOR;             // 0x190  W    -            XOR function on GPG Output Data 
    volatile unsigned int   GPGCD0;             // 0x194  R/W  0x00000000   Driver strength Control 0 on GPG 
    volatile unsigned int   GPGCD1;             // 0x198  R/W  0x00000000   Driver strength Control 1 on GPG 
    volatile unsigned int   GPGPE;              // 0x19C  R/W  0x00000000   Pull-Up/Down Enable function on GPG 
    volatile unsigned int   GPGPS;              // 0x1A0  R/W  0x00000000   Pull-Up/Down Selection function on GPG 
    volatile unsigned int   GPGIEN;             // 0x1A4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPG 
    volatile unsigned int   GPGIS;              // 0x1A8  R/W  0x00000000   Schimitt Input Function on GPG 
    volatile unsigned int   GPGSR;              // 0x1AC  R/W  0x00000000   Fast Slew Rate Function GPG 
    volatile unsigned int   GPGFN0;             // 0x1B0  R/W  0x00000000   Port Configuration on GPG Output Data 
    volatile unsigned int   GPGFN1;             // 0x1B4  R/W  0x00000000   Port Configuration on GPG Output Data 
    volatile unsigned int   GPGFN2;             // 0x1B8  R/W  0x00000000   Port Configuration on GPG Output Data 
    volatile unsigned int   GPGFN3;             // 0x1BC  R/W  0x00000000   Port Configuration on GPG Output Data 
    volatile unsigned int   GPHDMIDAT;          // 0x1C0  R/W  0x00000000   GPHDMI Data Register     
    volatile unsigned int   GPHDMIEN;           // 0x1C4  R/W  0x00000000   GPHDMI Output Enable Register     
    volatile unsigned int   GPHDMISET;          // 0x1C8  W    -            OR function on GPHDMI Output Data     
    volatile unsigned int   GPHDMICLR;          // 0x1CC  W    -            BIC function on GPHDMI Output Data     
    volatile unsigned int   GPHDMIXOR;          // 0x1D0  W    -            XOR function on GPHDMI Output Data     
    volatile unsigned int   GPHDMICD;           // 0x1D4  R/W  0x00000000   Driver strength Control on GPHDMI 
    volatile unsigned int   Reserved0;          //    
    volatile unsigned int   GPHDMIPE;           // 0x1DC  R/W  0x00000000   Pull-Up/Down Enable function on GPHDMI     
    volatile unsigned int   GPHDMIPS;           // 0x1E0  R/W  0x00000000   Pull-Up/Down Selection function on GPHDMI     
    volatile unsigned int   GPHDMIIEN;          // 0x1E4  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPHDMI     
    volatile unsigned int   GPHDMIIS;           // 0x1E8  R/W  0x00000000   Schimitt Input Function on GPHDMI     
    volatile unsigned int   GPHDMISR;           // 0x1EC  R/W  0x00000000   Fast Slew Rate Function GPHDMI     
    volatile unsigned int   GPHDMIFN0;          // 0x1F0  R/W  0x00000000   Port Configuration on GPHDMI Output Data     
    volatile unsigned int   Reserved1[3];       //    
    volatile unsigned int   GPADCDAT;           // 0x200  R/W  0x00000000   GPADC Data Register     
    volatile unsigned int   GPADCEN;            // 0x204  R/W  0x00000000   GPADC Output Enable Register     
    volatile unsigned int   GPADCSET;           // 0x208  W    -            OR function on GPADC Output Data     
    volatile unsigned int   GPADCCLR;           // 0x20C  W    -            BIC function on GPADC Output Data     
    volatile unsigned int   GPADCXOR;           // 0x210  W    -            XOR function on GPADC Output Data     
    volatile unsigned int   GPADCCD;            // 0x214  R/W  0x00000000   Driver strength Control on GPADC     
    volatile unsigned int   Reserved2;          //    
    volatile unsigned int   GPADCPE;            // 0x21C  R/W  0x00000000   Pull-Up/Down Enable function on GPADC     
    volatile unsigned int   GPADCPS;            // 0x220  R/W  0x00000000   Pull-Up/Down Selection function on GPADC     
    volatile unsigned int   GPADCIEN;           // 0x224  R/W  0xFFFFFFFF   Input Buffer Enable Function on GPADC     
    volatile unsigned int   GPADCIS;            // 0x228  R/W  0x00000000   Schimitt Input Function on GPADC     
    volatile unsigned int   GPADCSR;            // 0x22C  R/W  0x00000000   Fast Slew Rate Function GPADC     
    volatile unsigned int   GPADCFN0;           // 0x230  R/W  0x00000000   Port Configuration on GPADC Output Data     
    volatile unsigned int   Reserved3[3];       //     
    volatile unsigned int   EINTSEL0;           // 0x240  R/W               External Interrupt Select Register 0     
    volatile unsigned int   EINTSEL1;           // 0x244  R/W               External Interrupt Select Register 1     
    volatile unsigned int   EINTSEL2;           // 0x248  R/W               External Interrupt Select Register 2     
    volatile unsigned int   Reserved4;          //    
    volatile unsigned int   ECID0;              // 0x250  R/W               CID output Register 
    volatile unsigned int   ECID1;              // 0x254  R                 CID serial output data Register 
    volatile unsigned int   ECID2;              // 0x258  R    -            CID parallel output data 0 Register 
    volatile unsigned int   ECID3;              // 0x25C  R    -            CID parallel output data 1 Register 
} NAND_GPIO, *NAND_PGPIO;                

typedef struct _NAND_GPION{
    volatile unsigned int   GPDAT;              // 0x000  R/W               GPA Data Register
    volatile unsigned int   GPEN;               // 0x004  R/W               GPA Output Enable Register
    volatile unsigned int   GPSET;              // 0x008  W                 OR function on GPA Output Data
    volatile unsigned int   GPCLR;              // 0x00C  W                 BIC function on GPA Output Data
    volatile unsigned int   GPXOR;              // 0x010  W                 XOR function on GPA Output Data
    volatile unsigned int   GPCD0;              // 0x014  W                 Driver strength Control 0 on GPA Output Data
    volatile unsigned int   GPCD1;              // 0x018  W                 Driver strength Control 1 on GPA Output Data
    volatile unsigned int   GPPD0;              // 0x01C  W                 Pull-Up/Down function on GPA Output Data
    volatile unsigned int   GPPD1;              // 0x020  W                 Pull-Up/Down function on GPA Output Data
    volatile unsigned int   GPFN0;              // 0x024  W                 Port Configuration on GPA Output Data
    volatile unsigned int   GPFN1;              // 0x028  W                 Port Configuration on GPA Output Data
    volatile unsigned int   GPFN2;              // 0x02C  W                 Port Configuration on GPA Output Data
    volatile unsigned int   GPFN3;              // 0x030  W                 Port Configuration on GPA Output Data
} NAND_GPION, *NAND_PGPION;

#endif /* _STRUCTURES_SMU_PMU_H_ */
