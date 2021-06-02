/*
 * IR driver for remote controller : tcc_remocon.h
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

#ifndef __TCC_REMOCON_H__
#define __TCC_REMOCON_H__

#include <linux/types.h>

/* Base Addr */
#if defined (CONFIG_ARCH_TCC897X)
	#define TCC_PA_REMOTECTRL       0x76070000
	#define TCC_PA_REMOCON_CONFIG       0x74400128
#elif defined (CONFIG_ARCH_TCC898X)
	#define TCC_PA_REMOTECTRL       0x16040000
	#define TCC_PA_REMOCON_CONFIG       0x14400128
#else
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TCA_REMOTE__GLOBALS_H__
#define TCA_REMOTE__GLOBALS_H__

#define Hw31		0x80000000
#define Hw30		0x40000000
#define Hw29		0x20000000
#define Hw28		0x10000000
#define Hw27		0x08000000
#define Hw26		0x04000000
#define Hw25		0x02000000
#define Hw24		0x01000000
#define Hw23		0x00800000
#define Hw22		0x00400000
#define Hw21		0x00200000
#define Hw20		0x00100000
#define Hw19		0x00080000
#define Hw18		0x00040000
#define Hw17		0x00020000
#define Hw16		0x00010000
#define Hw15		0x00008000
#define Hw14		0x00004000
#define Hw13		0x00002000
#define Hw12		0x00001000
#define Hw11		0x00000800
#define Hw10		0x00000400
#define Hw9		0x00000200
#define Hw8		0x00000100
#define Hw7		0x00000080
#define Hw6		0x00000040
#define Hw5		0x00000020
#define Hw4		0x00000010
#define Hw3		0x00000008
#define Hw2		0x00000004
#define Hw1		0x00000002
#define Hw0		0x00000001
#define HwZERO		0x00000000

#define ENABLE		1
#define DISABLE		0

#define ON		1
#define OFF		0

#define FALSE		0
#define TRUE		1

#define BITSET(X, MASK)			((X) |= (unsigned int)(MASK))
#define BITSCLR(X, SMASK, CMASK)	((X) = ((((unsigned int)(X)) | ((unsigned int)(SMASK))) & ~((unsigned int)(CMASK))) )
#define BITCSET(X, CMASK, SMASK)	((X) = ((((unsigned int)(X)) & ~((unsigned int)(CMASK))) | ((unsigned int)(SMASK))) )
#define BITCLR(X, MASK)			((X) &= ~((unsigned int)(MASK)) )
#define BITXOR(X, MASK)			((X) ^= (unsigned int)(MASK) )
#define ISZERO(X, MASK)			(!(((unsigned int)(X))&((unsigned int)(MASK))))
#define	ISSET(X, MASK)			((unsigned long)(X)&((unsigned long)(MASK)))

#if defined(CONFIG_ARCH_TCC898X)
#define IO_OFFSET       0xE1000000
#elif defined(CONFIG_ARCH_TCC897X)
#define IO_OFFSET       0x81000000
#else
    #error
#endif
#define io_p2v(pa)      ((pa) + IO_OFFSET)
#define tcc_p2v(pa)     io_p2v(pa)
#if 0
#define BITSET(X, MASK)			((X) |= (unsigned int)(MASK))
#define BITSCLR(X, SMASK, CMASK)	((X) = ((((unsigned int)(X)) | ((unsigned int)(SMASK))) & ~((unsigned int)(CMASK))) )
#define BITCSET(X, CMASK, SMASK)	((X) = ((((unsigned int)(X)) & ~((unsigned int)(CMASK))) | ((unsigned int)(SMASK))) )
#define BITCLR(X, MASK)			((X) &= ~((unsigned int)(MASK)) )
#define BITXOR(X, MASK)			((X) ^= (unsigned int)(MASK) )
#define ISZERO(X, MASK)			(!(((unsigned int)(X))&((unsigned int)(MASK))))
#define	ISSET(X, MASK)			((unsigned long)(X)&((unsigned long)(MASK)))
#if 1
#define IO_PHYS     0x70000000
#define IO_OFFSET   0x81000000  /* Virtual IO = 0xf1000000 */
#define IO_SIZE     0xA000000
#define IO_VIRT     (IO_PHYS + IO_OFFSET)
#endif
#define io_p2v(pa)  ((pa) + IO_OFFSET)
#define tcc_p2v(pa)         io_p2v(pa)
#endif
#endif // __GLOBALS_H__

   /* temporally type */
typedef struct {
	unsigned VALUE          :16;
} TCC_DEF16BIT_IDX_TYPE;

typedef union {
	unsigned short          nREG;
	TCC_DEF16BIT_IDX_TYPE   bREG;
} TCC_DEF16BIT_TYPE;

typedef struct {
	unsigned VALUE          :32;
} TCC_DEF32BIT_IDX_TYPE;

typedef union {
	unsigned long           nREG;
	TCC_DEF32BIT_IDX_TYPE   bREG;
} TCC_DEF32BIT_TYPE;

/*******************************************************************************
*   Remote Control Interface                        (Base Addr = 0x0x16070000)
********************************************************************************/

typedef struct {
    unsigned DATA           :32;
} REMOTE_RDATA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_RDATA_IDX_TYPE   bREG;
} REMOTE_RDATA_TYPE;

typedef struct {
    unsigned FF             :1;
    unsigned EN             :1;
    unsigned CLEAR          :1;
    unsigned TH             :9;
    unsigned WS             :1;
    unsigned DEN            :1;
    unsigned FWEN           :1;
    unsigned                :17;
} REMOTE_CMD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_CMD_IDX_TYPE     bREG;
} REMOTE_CMD_TYPE;

typedef struct {
    unsigned INV            :1;
    unsigned                :7;
    unsigned SCLK           :1;
    unsigned FIL            :1;
    unsigned FT             :2;
    unsigned FCLK           :1;
    unsigned CXTIN          :1;
    unsigned                :18;
} REMOTE_INPOL_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_INPOL_IDX_TYPE   bREG;
} REMOTE_INPOL_TYPE;

typedef struct {
    unsigned ICF            :12;
    unsigned OF             :1;
    unsigned                :19;
} REMOTE_STA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_STA_IDX_TYPE     bREG;
} REMOTE_STA_TYPE;

typedef struct {
    unsigned VCNT           :6;
    unsigned FF             :1;
    unsigned EMP            :1;
    unsigned OVF            :1;
    unsigned                :23;
} REMOTE_FSTA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_FSTA_IDX_TYPE     bREG;
} REMOTE_FSTA_TYPE;

typedef struct {
    unsigned BE             :1;
    unsigned BDDR           :1;
    unsigned DB             :1;
    unsigned RB             :1;
    unsigned SB             :1;
    unsigned BDXD           :18;
    unsigned BDSC           :1;
    unsigned BDDC           :1;
    unsigned                :7;
} REMOTE_BDD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_BDD_IDX_TYPE     bREG;
} REMOTE_BDD_TYPE;

typedef struct {
    unsigned DATA           :32;
} REMOTE_BDR_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_BDR_IDX_TYPE    bREG;
} REMOTE_BDR_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_SD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_SD_IDX_TYPE      bREG;
} REMOTE_SD_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_DBD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_DBD_IDX_TYPE    bREG;
} REMOTE_DBD_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_RBD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_RBD_IDX_TYPE     bREG;
} REMOTE_RBD_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_PBD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_PBD_IDX_TYPE    bREG;
} REMOTE_PBD_TYPE;

typedef struct {
    unsigned END_CNT        :8;
    unsigned                :6;
    unsigned CLK_DIV        :18;
} REMOTE_CLKDIV_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_CLKDIV_IDX_TYPE  bREG;
} REMOTE_CLKDIV_TYPE;

typedef struct _REMOTECON{
    volatile REMOTE_RDATA_TYPE    RDATA;          // 0x000  R  0x00000000   IR Read Data
    volatile REMOTE_CMD_TYPE       CMD;              // 0x004  R/W  0x00000000   Command Register
    volatile REMOTE_INPOL_TYPE     INPOL;           // 0x008  R/W  0x00000000   Input Polarity Inversion Register
    volatile REMOTE_STA_TYPE        STA;              // 0x00C  R/W  0x00000000   Status register
    volatile REMOTE_FSTA_TYPE      FSTA;             // 0x010  R     0x00000000   FIFO Status register
    volatile REMOTE_BDD_TYPE       BDD;              // 0x014  R/W  0x00000000   Bit Duration Decision
    volatile REMOTE_BDR_TYPE     BDR0;             // 0x018  R      0x00000000   IR Bit Data Read0
    volatile REMOTE_BDR_TYPE     BDR1;             // 0x01C  R      0x00000000   IR Bit Data Read1
    volatile REMOTE_SD_TYPE         SD;                 // 0x020  R/W  0x00000000   IR Start Bit Duration High and Low Min/Max
    volatile REMOTE_DBD_TYPE     DBD0;            // 0x024  R/W  0x00000000   IR Data Bit ¡°0¡± Duration High and Low Min/Max
    volatile REMOTE_DBD_TYPE     DBD1;            // 0x028  R/W  0x00000000   IR Data Bit ¡°1¡± Duration High and Low Min/Max
    volatile REMOTE_RBD_TYPE       RBD;              // 0x02c  R/W  0x00000000   IR Repeat Bit Duration High and Low Min/Max
    volatile REMOTE_PBD_TYPE     PBD00;          // 0x030  R/W  0x00000000   Power Button0 Data0
    volatile REMOTE_PBD_TYPE     PBD01;          // 0x034  R/W  0x00000000   Power Button0 Data1
    volatile REMOTE_PBD_TYPE     PBD10;          // 0x038  R/W  0x00000000   Power Button1 Data0
    volatile REMOTE_PBD_TYPE     PBD11;          // 0x03c  R/W  0x00000000   Power Button1 Data1
    volatile REMOTE_PBD_TYPE     PBD20;          // 0x040  R/W  0x00000000   Power Button2 Data0
    volatile REMOTE_PBD_TYPE     PBD21;          // 0x044  R/W  0x00000000   Power Button2 Data1
    volatile REMOTE_PBD_TYPE     PBD30;          // 0x048  R/W  0x00000000   Power Button3 Data0
    volatile REMOTE_PBD_TYPE     PBD31;          // 0x04c  R/W  0x00000000   Power Button3 Data1
    volatile REMOTE_PBD_TYPE     RKD0;            // 0x050  R/W  0x00000000   RepeatKey Data
    volatile REMOTE_PBD_TYPE     RKD1;            // 0x054  R/W  0x00000000   RepeatKey Data
    volatile REMOTE_CLKDIV_TYPE CLKDIV;         // 0x058  R/W  0x00000000   Clock Divide Register
    volatile REMOTE_PBD_TYPE     PBM0;            // 0x05C  R/W  0x00000000   Power Botton Mask
    volatile REMOTE_PBD_TYPE     PBM1;            // 0x060  R/W  0x00000000   Power Botton Mask
} REMOTECON, *PREMOTECON;

typedef struct {
    unsigned RMISEL       :3;
    unsigned RMDIV        :1;
    unsigned RMTE          :1;
    unsigned RMSEL         :2;
    unsigned                    :25;
} REMOCON_CONFIG_TYPE;

typedef union {
    unsigned long           nREG;
    REMOCON_CONFIG_TYPE     bREG;
} REMOCON_CONFIG, *PREMOCON_CONFIG;
#if defined(CONFIG_ARCH_TCC893X)
/*******************************************************************************
*   Remote Control Interface                        (Base Addr = 0x76070000)
********************************************************************************/

typedef struct {
    unsigned DATA           :32;
} REMOTE_RDATA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_RDATA_IDX_TYPE   bREG;
} REMOTE_RDATA_TYPE;

typedef struct {
    unsigned FF             :1;
    unsigned EN             :1;
    unsigned CLEAR          :1;
    unsigned TH             :9;
    unsigned WS             :1;
    unsigned DEN            :1;
    unsigned FWEN           :1;
    unsigned                :17;
} REMOTE_CMD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_CMD_IDX_TYPE     bREG;
} REMOTE_CMD_TYPE;

typedef struct {
    unsigned INV            :1;
    unsigned                :7;
    unsigned SCLK           :1;
    unsigned FIL            :1;
    unsigned FT             :2;
    unsigned FCLK           :1;
    unsigned CXTIN          :1;
    unsigned                :18;
} REMOTE_INPOL_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_INPOL_IDX_TYPE   bREG;
} REMOTE_INPOL_TYPE;

typedef struct {
    unsigned ICF            :12;
    unsigned OF             :1;
    unsigned                :19;
} REMOTE_STA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_STA_IDX_TYPE     bREG;
} REMOTE_STA_TYPE;

typedef struct {
    unsigned VCNT           :6;
    unsigned FF             :1;
    unsigned EMP            :1;
    unsigned OVF            :1;
    unsigned                :23;
} REMOTE_FSTA_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_FSTA_IDX_TYPE     bREG;
} REMOTE_FSTA_TYPE;

typedef struct {
    unsigned BE             :1;
    unsigned BDDR           :1;
    unsigned DB             :1;
    unsigned RB             :1;
    unsigned SB             :1;
    unsigned BDXD           :18;
    unsigned BDSC           :1;
    unsigned BDDC           :1;
    unsigned                :7;
} REMOTE_BDD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_BDD_IDX_TYPE     bREG;
} REMOTE_BDD_TYPE;

typedef struct {
    unsigned DATA           :32;
} REMOTE_BDR0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_BDR0_IDX_TYPE    bREG;
} REMOTE_BDR0_TYPE;

typedef struct {
    unsigned DATA           :32;
} REMOTE_BDR1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_BDR1_IDX_TYPE    bREG;
} REMOTE_BDR1_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_SD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_SD_IDX_TYPE      bREG;
} REMOTE_SD_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_DBD0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_DBD0_IDX_TYPE    bREG;
} REMOTE_DBD0_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_DBD1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_DBD1_IDX_TYPE    bREG;
} REMOTE_DBD1_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_RBD_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_RBD_IDX_TYPE     bREG;
} REMOTE_RBD_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_PBD0_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_PBD0_IDX_TYPE    bREG;
} REMOTE_PBD0_TYPE;

typedef struct {
    unsigned MIN            :16;
    unsigned MAX            :16;
} REMOTE_PBD1_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_PBD1_IDX_TYPE    bREG;
} REMOTE_PBD1_TYPE;

typedef struct {
    unsigned END_CNT        :8;
    unsigned                :6;
    unsigned CLK_DIV        :18;
} REMOTE_CLKDIV_IDX_TYPE;

typedef union {
    unsigned long           nREG;
    REMOTE_CLKDIV_IDX_TYPE  bREG;
} REMOTE_CLKDIV_TYPE;

typedef struct _REMOTECON{
    volatile REMOTE_RDATA_TYPE  RDATA;          // 0x000  R/W  0x00000000   IR Data Transfer Address
    volatile REMOTE_CMD_TYPE    CMD;            // 0x004  R/W  0x00000000   Command Register
    volatile REMOTE_INPOL_TYPE  INPOL;          // 0x008  R/W  0x00000000   Control Register
    volatile REMOTE_STA_TYPE    STA;            // 0x00C  W     0x00000000   Status register
    volatile REMOTE_FSTA_TYPE   FSTA;           // 0x010  R      0x00000000   FIFO Status register
    volatile REMOTE_BDD_TYPE    BDD;            // 0x014  R/W  0x00000000   Bit Duration Decision
	volatile REMOTE_BDR0_TYPE   BDR0;           // 0x018  R      0x00000000   IR Bit Data Read0
	volatile REMOTE_BDR1_TYPE   BDR1;           // 0x01C  R      0x00000000   IR Bit Data Read1
    volatile REMOTE_SD_TYPE     SD;             // 0x020  R/W  0x00000000   IR Start Bit Duration High and Low Min/Max
	volatile REMOTE_DBD0_TYPE   DBD0;           // 0x024  R/W  0x00000000   IR Data Bit ¡°0¡± Duration High and Low Min/Max
	volatile REMOTE_DBD1_TYPE   DBD1;           // 0x028  R/W  0x00000000   IR Data Bit ¡°1¡± Duration High and Low Min/Max
	volatile REMOTE_RBD_TYPE    RBD;            // 0x02c  R/W  0x00000000   IR Repeat Bit Duration High and Low Min/Max
    volatile REMOTE_PBD0_TYPE   PBD00;          // 0x030  R/W  0x00000000   Power Button0 Data0
    volatile REMOTE_PBD0_TYPE   PBD01;          // 0x034  R/W  0x00000000   Power Button0 Data1
    volatile REMOTE_PBD1_TYPE   PBD10;          // 0x038  R/W  0x00000000   Power Button1 Data0
    volatile REMOTE_PBD0_TYPE   PBD11;          // 0x03c  R/W  0x00000000   Power Button1 Data1
    volatile REMOTE_CLKDIV_TYPE CLKDIV;         // 0x040  R      0x00000000   Clock Divide Register
} REMOTECON, *PREMOTECON;

typedef struct {
    unsigned BM              :6;
    unsigned                    :1;
    unsigned TSEL           :1;
    unsigned GCLK_CNT   :8;
    unsigned PKGI           :2;
    unsigned                   :2;
    unsigned RMISEL       :3;
    unsigned RMDIV        :1;
    unsigned RMTE          :1;
    unsigned RMSEL        :2;
    unsigned                   :1;
    unsigned REMAP        :2;
    unsigned                    :1;
    unsigned IOBSEL       :1;
} REMOCON_CONFIG_TYPE;

typedef union {
    unsigned long           nREG;
    REMOCON_CONFIG_TYPE     bREG;
} REMOCON_CONFIG, *PREMOCON_CONFIG;;

#endif
/*******************************************************************************
 Remote KEY value
 This can be found from device/telechips/tccxxx/telechips_remote_controller.kl
********************************************************************************/ 
#define REM_SOFT_LEFT           1
#define REM_SOFT_RIGHT          2
#define REM_HOME                3
#define REM_BACK                4
#define REM_CALL                5
#define REM_ENDCALL             6
#define REM_0                   7
#define REM_1                   8
#define REM_2                   9
#define REM_3                   10
#define REM_4                   11
#define REM_5                   12
#define REM_6                   13
#define REM_7                   14
#define REM_8                   15
#define REM_9                   16
#define REM_STAR                17
#define REM_POUND               18
#define REM_DPAD_UP             19
#define REM_DPAD_DOWN           20
#define REM_DPAD_LEFT           21
#define REM_DPAD_RIGHT          22
#define REM_DPAD_CENTER         23
#define REM_VOLUME_UP           24
#define REM_VOLUME_DOWN         25
#define REM_POWER               26
#define REM_CAMERA              27
#define REM_CLEAR               28
#define REM_A                   29
#define REM_B                   30
#define REM_C                   31
#define REM_D                   32
#define REM_E                   33
#define REM_F                   34
#define REM_G                   35
#define REM_H                   36
#define REM_I                   37
#define REM_J                   38
#define REM_K                   39
#define REM_L                   40
#define REM_M                   41
#define REM_N                   42
#define REM_O                   43
#define REM_P                   44
#define REM_Q                   45
#define REM_R                   46
#define REM_S                   47
#define REM_T                   48
#define REM_U                   49
#define REM_V                   50
#define REM_W                   51
#define REM_X                   52
#define REM_Y                   53
#define REM_Z                   54
#define REM_COMMA               55
#define REM_PERIOD              56
#define REM_ALT_LEFT            57
#define REM_ALT_RIGHT           58
#define REM_SHIFT_LEFT          59
#define REM_SHIFT_RIGHT         60
#define REM_TAB                 61
#define REM_SPACE               62
#define REM_SYM                 63
#define REM_EXPLORER            64
#define REM_ENVELOPE            65
#define REM_ENTER               66
#define REM_DEL                 67
#define REM_GRAVE               68
#define REM_MINUS               69
#define REM_EQUALS              70
#define REM_LEFT_BRACKET        71
#define REM_RIGHT_BRACKET       72
#define REM_BACKSLASH           73
#define REM_SEMICOLON           74
#define REM_APOSTROPHE          75
#define REM_SLASH               76
#define REM_AT                  77
#define REM_NUM                 78
#define REM_HEADSETHOOK         79
#define REM_FOCUS               80
#define REM_PLUS                81
#define REM_MENU                82
#define REM_NOTIFICATION        83
#define REM_SEARCH              84
#define REM_MEDIA_PLAY_PAUSE    85
#define REM_MEDIA_STOP          86
#define REM_MEDIA_NEXT          87
#define REM_MEDIA_PREVIOUS      88
#define REM_MEDIA_REWIND        89
#define REM_MEDIA_FAST_FORWARD  90
#define REM_MUTE                91
#define REM_PAGE_UP             92
#define REM_PAGE_DOWN           93
#define REM_PICTSYMBOLS         94
#define REM_SWITCH_CHARSET      95
#define REM_BUTTON_A            96
#define REM_BUTTON_B            97
#define REM_BUTTON_C            98
#define REM_BUTTON_X            99
#define REM_BUTTON_Y            100
#define REM_BUTTON_Z            101
#define REM_BUTTON_L1           102
#define REM_BUTTON_R1           103
#define REM_BUTTON_L2           104
#define REM_BUTTON_R2           105
#define REM_BUTTON_THUMBL       106
#define REM_BUTTON_THUMBR       107
#define REM_BUTTON_START        108
#define REM_BUTTON_SELECT       109
#define REM_BUTTON_MODE         110
#define REM_ESCAPE              111
#define REM_FORWARD_DEL         112
#define REM_CTRL_LEFT           113
#define REM_CTRL_RIGHT          114
#define REM_CAPS_LOCK           115
#define REM_SCROLL_LOCK         116
#define REM_META_LEFT           117
#define REM_META_RIGHT          118
#define REM_FUNCTION            119
#define REM_SYSRQ               120
#define REM_BREAK               121
#define REM_MOVE_HOME           122
#define REM_MOVE_END            123
#define REM_INSERT              124
#define REM_FORWARD             125
#define REM_MEDIA_PLAY          126
#define REM_MEDIA_PAUSE         127
#define REM_MEDIA_CLOSE         128
#define REM_MEDIA_EJECT         129
#define REM_MEDIA_RECORD        130
#define REM_FUNCTION_F1         131
#define REM_FUNCTION_F2         132
#define REM_FUNCTION_F3         133
#define REM_FUNCTION_F4         134
#define REM_FUNCTION_F5         135
#define REM_FUNCTION_F6         136
#define REM_FUNCTION_F7         137
#define REM_FUNCTION_F8         138
#define REM_FUNCTION_F9         139
#define REM_FUNCTION_F10        140
#define REM_FUNCTION_F11        141
#define REM_FUNCTION_F12        142
#define REM_NUM_LOCK            143
#define REM_NUMPAD_0            144
#define REM_NUMPAD_1            145
#define REM_NUMPAD_2            146
#define REM_NUMPAD_3            147
#define REM_NUMPAD_4            148
#define REM_NUMPAD_5            149
#define REM_NUMPAD_6            150
#define REM_NUMPAD_7            151
#define REM_NUMPAD_8            152
#define REM_NUMPAD_9            153
#define REM_NUMPAD_DIVIDE       154
#define REM_NUMPAD_MULTIPLY     155
#define REM_NUMPAD_SUBTRACT     156
#define REM_NUMPAD_ADD          157
#define REM_NUMPAD_DOT          158
#define REM_NUMPAD_COMMA        159
#define REM_NUMPAD_ENTER        160
#define REM_NUMPAD_EQUALS       161
#define REM_NUMPAD_LEFT_PAREN   162
#define REM_NUMPAD_RIGHT_PAREN  163
#define REM_VOLUME_MUTE         164
#define REM_INFO                165
#define REM_CHANNEL_UP          166
#define REM_CHANNEL_DOWN        167
#define REM_ZOOM_IN             168
#define REM_ZOOM_OUT            169
#define REM_TV                  170
#define REM_WINDOW              171
#define REM_GUIDE               172
#define REM_DVR                 173
#define REM_BOOKMARK            174
#define REM_CAPTIONS            175
#define REM_SETTINGS            176
#define REM_TV_POWER            177
#define REM_TV_INPUT            178
#define REM_STB_POWER           179
#define REM_STB_INPUT           180
#define REM_AVR_POWER           181
#define REM_AVR_INPUT           182
#define REM_PROG_RED            183
#define REM_PROG_GREEN          184
#define REM_PROG_YELLOW         185
#define REM_PROG_BLUE           186
#define REM_APP_SWITCH          187
#define REM_BUTTON_1            188
#define REM_BUTTON_2            189
#define REM_BUTTON_3            190
#define REM_BUTTON_4            191
#define REM_BUTTON_5            192
#define REM_BUTTON_6            193
#define REM_BUTTON_7            194
#define REM_BUTTON_8            195
#define REM_BUTTON_9            196
#define REM_BUTTON_10           197
#define REM_BUTTON_11           198
#define REM_BUTTON_12           199
#define REM_BUTTON_13           200
#define REM_BUTTON_14           201
#define REM_BUTTON_15           202
#define REM_BUTTON_16           203
#define REM_LANGUAGE_SWITCH     204
#define REM_MANNER_MODE         205
#define REM_3D_MODE             206
#define REM_CONTACTS            207
#define REM_CALENDAR            208
#define REM_MUSIC               209
#define REM_CALCULATOR          210

#if 0
#define isRepeatableKey(i)      ((i==REM_DPAD_UP) || (i==REM_DPAD_DOWN) || (i==REM_DPAD_LEFT) || (i==REM_DPAD_RIGHT) || \
                                 (i==REM_ENTER) || (i==REM_POWER) || (i==REM_MEDIA_REWIND) || (i==REM_MEDIA_FAST_FORWARD) || \
                                 (i==REM_MEDIA_PREVIOUS) || (i==REM_MEDIA_NEXT) || \
                                 (i==REM_VOLUME_UP) || (i==REM_VOLUME_DOWN) || (REM_0<=i && i<=REM_9) || (i==REM_HOME))
#endif

#define isRepeatableKey(i)      1
/*******************************************************************************
 Remote controller define
********************************************************************************/ 
#define NO_KEY			0
#define NEW_KEY			1
#define REPEAT_START 	2
#define REPEAT_KEY		3

#define STATUS0			0
#define STATUS1			1
#define STATUS2			2
#define STATUS3			3

/* 1 unit = 1 / [IOBUS/(HwREMOCON->uCLKDIVIDE.bCLKDIVIDE.CLK_DIV)]
              = 1 / [156/(500*20)]  us
              = 64.1us
*/
/* 1 unit = 1 / [IOBUS/(HwREMOCON->uCLKDIVIDE.bCLKDIVIDE.CLK_DIV)]
              = 1 / [165/(500*21)]  us
              = 63.63us
*/
/* 1 unit = 1 / [IOBUS/(HwREMOCON->uCLKDIVIDE.bCLKDIVIDE.CLK_DIV)]
			  = 1 / [165/(500*26)]	us
			  = 78.78us
*/

// Scan-code mapping for keypad
typedef struct {
	unsigned short	rcode;
	unsigned short	vkcode;
}SCANCODE_MAPPING;


/*******************************************************************************
 External variables
********************************************************************************/ 

void RemoconCommandOpen(void);
void RemoconCommandReset(void);
void RemoconConfigure(void);
void RemoconStatus(void);
void RemoconDivide(int state);
void DisableRemocon(void);
void RemoconReset(void);
void RemoconInit(int state);
void RemoconIrqClear(void);
void RemoconIntClear(void);
void RemoconIntWait(void);

extern void tcc_remocon_reset(void);
extern void tcc_remocon_set_io(void);

/*******************************************************************************
 Decide the remote module
 This can be changed with 'make menuconfig' command.
********************************************************************************/ 
#ifdef CONFIG_DARE_IR
	#include "tcc_remocon_DARE_IR.h"

#elif defined(CONFIG_CS_2000_IR_X_CANVAS)
	#include "tcc_remocon_CS_2000_IR_X_CANVAS.h"
#elif defined(CONFIG_VERIZON)
	#include "tcc_remocon_Verizon.h"
#elif defined(CONFIG_NOVATEK_IR)
	#include "tcc_remocon_NOVATEK_IR.h"

#elif defined(CONFIG_JH_4202_1011_03)
	#include "tcc_remocon_jh_4202_1011_03.h"
#elif defined(CONFIG_SAMSUNG_42BIT)
	#include "tcc_remocon_SAMSUNG_42bit.h"
#elif defined(CONFIG_HDS892S)
	#include "tcc_remocon_HDS892S.h"
#elif defined(CONFIG_PREMIER_IR)
	#include "tcc_remocon_premier.h"
#elif defined(CONFIG_YAOJIN_IR)
	#include "tcc_remocon_yaojin.h"
#else
	#error you don not select proper remocon module
#endif

#endif	//__TCC_REMOCON_H__

