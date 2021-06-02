# //+[TCCQB] General register Store / Restore
/****************************************************************************
 * arch/arm/mach-tcc897x/snapshot_asm.S
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 ****************************************************************************/

#include <linux/linkage.h>
#include <mach/sram_map.h>

/*
  CPSR Mode Bit Definition
*/
#define Mode_USR            0x10
#define Mode_FIQ            0x11
#define Mode_IRQ            0x12
#define Mode_SVC            0x13
#define Mode_ABT            0x17
#define Mode_UND            0x1B
#define Mode_SYS            0x1F
#define Mode_MASK           0x1F
#define NOINT               0xC0
#define I_Bit               0x80
#define F_Bit               0x40
.text

#ifdef CONFIG_SNAPSHOT_BOOT
/*
    r0 : ptable phy address
    r1 : mmu & cpu_reg backup area vir address
    r2 : restore func vir address
*/
ENTRY(save_cpu_reg_for_snapshot)
	
	nop
	@ Store General Registers
	push	{r0-r12}

	@ Store Stack Pointer to use it at the end of this function.
	ldr		r3, .Lsaved_stack_pointer
	str		sp, [r3]

    @ Copy arguments to other registers.
    mov     r9, r1
    mov     r10, r2

    ldr     r3, .Lsaved_cpsr
    mrs     r0, cpsr
    str     r0, [r3]
    
    @ Change to system(user) mode
    mov     r1, r0
    orr     r1, r1, #0x1f
    msr     cpsr_c, r1
    
    @ Save User context
    ldr     r3, .Lsaved_context_r0
    stmia   r3, {r0-r14}
    
    @ Go back to original SVC mode
    msr     cpsr_c, r0

    @ Save SVC context
    ldr     r3, .Lsaved_context_r13_svc
	str		r13, [r3]
    ldr     r3, .Lsaved_context_r14_svc
	str		r14, [r3]
    ldr     r3, .Lsaved_spsr_svc
    mrs     r1, spsr
    str     r1, [r3]

    bl      swsusp_save    
    
    mov     r1, r9
    mov     r2, r10


    @ mark shut down mode
    ldr     r3, =0x18C818C8
    str     r3, [r1, #0x00]

    @ save restore func address
    str     r2, [r1, #0x04]
    mov     r3, r1


	/*		 Store Memory Coprocessor Registers			*/
    @ save SCTLR
    mrc     p15, 0, r2, c1, c0, 0
    str     r2, [r3, #0x08]

    @ save TTBR0
    mrc     p15, 0, r2, c2, c0, 0
    str     r2, [r3, #0x0C]

    @ save TTBR1
    mrc     p15, 0, r2, c2, c0, 1
    str     r2, [r3, #0x10]

    @ save TTBCR
    mrc     p15, 0, r2, c2, c0, 2
    str     r2, [r3, #0x14]

    @ save DACR
    mrc     p15, 0, r2, c3, c0, 0
    str     r2, [r3, #0x18]

    @ save DFSR
    mrc     p15, 0, r2, c5, c0, 0
    str     r2, [r3, #0x1C]

    @ save DFAR
    mrc     p15, 0, r2, c6, c0, 0
    str     r2, [r3, #0x20]

    @ save IFSR
    mrc     p15, 0, r2, c5, c0, 1
    str     r2, [r3, #0x24]

    @ save IFAR
    mrc     p15, 0, r2, c6, c0, 2
    str     r2, [r3, #0x28]

    @ save DAFSR
    mrc     p15, 0, r2, c5, c1, 0
    str     r2, [r3, #0x2C]

    @ save IAFSR
    mrc     p15, 0, r2, c5, c1, 1
    str     r2, [r3, #0x30]

    @ save PMRRR
    mrc     p15, 0, r2, c10, c2, 0
    str     r2, [r3, #0x34]

    @ save NMRRR
    mrc     p15, 0, r2, c10, c2, 1
    str     r2, [r3, #0x38]


	/*		 Store General Registers			*/
    @ save svc_reg (spsr)
    str     sp, [r3, #0x60]
    mrs     r2, spsr
    str     r2, [r3, #0x64]

    @ save fiq_reg
    mov     r1, #Mode_FIQ | NOINT       @ Enter FIQ mode, no interrupts
    msr     cpsr, r1
    mrs     r2, spsr                    @ Status Register
	str     r2, [r3, #0x68]
	
	@ Store FIQ mode registers [SleepState_FIQ_SPSR~SleepState_FIQ_LR]
	str     r8, [r3, #0x6C]
	str     r9, [r3, #0x70]
	str     r10,[r3, #0x74]
	str     r11,[r3, #0x78]
	str     r12,[r3, #0x7C]
	str     sp, [r3, #0x80]
	str     lr, [r3, #0x84]

    @ save abort_reg
    mov     r1, #Mode_ABT | NOINT       @ Enter ABT mode, no interrupts
    msr     cpsr, r1
    mrs     r0, spsr                    @ Status Register
	str     r0, [r3, #0x88]

	@ Store ABT mode Registers [SleepState_ABT_SPSR~SleepState_ABT_LR]
	str     sp, [r3, #0x8C]
	str     lr, [r3, #0x90]

    @ save irq_reg
    mov     r1, #Mode_IRQ | NOINT       @ Enter IRQ mode, no interrupts
    msr     cpsr, r1
    mrs     r0, spsr                    @ Status Register
	str     r0, [r3, #0x94]

	@ Store the IRQ Mode Registers [SleepState_IRQ_SPSR~SleepState_IRQ_LR]
	str     sp, [r3, #0x98]
	str     lr, [r3, #0x9C]
    
    @ save undef_reg
    mov     r1, #Mode_UND | NOINT       @ Enter UND mode, no interrupts
    msr     cpsr, r1
    mrs     r0, spsr                    @ Status Register
	str     r0, [r3, #0xA0]

	@ Store the UND mode Registers [SleepState_UND_SPSR~SleepState_UND_LR]
	str     sp, [r3, #0xA4]
	str     lr, [r3, #0xA8]

    @ save usr(sys)_reg
    mov     r1, #Mode_SYS | NOINT       @ Enter SYS mode, no interrupts
    msr     cpsr, r1

	@ Store the SYS mode Registers [SleepState_SYS_SP, SleepState_SYS_LR]
	str     sp, [r3, #0xAC]
	str     lr, [r3, #0xB0]

    @ Return to SVC mode
    mov     r1, #Mode_SVC | NOINT       @ Back to SVC mode, no interrupts
    msr     cpsr, r1

/*	Store SP Register Address... But it saved later at .Lsaved_stack_pointer
    ldr     r2, [sp]
    str     r2, [r3], #4    
*/
    @ invalidate entire tlb & btb & prefetch buffer.
    mov     r0, #0x0
    dsb     sy
    mcr     p15, 0, r0, c8, c7, 0       @ p15,0,r3,c8,c7,0 (itlb+dtlb flush all)
    mcr     p15, 0, r0, c7, c5, 6       @ p15,0,r3,c7,c5,6 (invalidate btb)
    mcr     p15, 0, r0, c7, c5, 4       @ flush prefetch buffer
    dsb     sy
    isb     sy

    @ clean & invalidate cache(L1, L2)
    bl      v7_flush_kern_cache_all

    @ Data synchronization barrier operation
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 4

    nop
	@ Restore Stack Pointer.
	ldr		r0, .Lsaved_stack_pointer
	ldr		sp, [r0]

    @ Restore General Registers
    pop		{r0-r12}
    
    @ Restore return address
    ldr		r0, .Lsaved_context_r14_svc
    ldr		lr, [r0]
    mov		r0, #0
    mov		pc, lr
ENDPROC(save_cpu_reg_for_snapshot)


ENTRY(restore_snapshot_cpu_reg)

    nop
	
    mov	r0, #1
    bl	set_do_snapshot_boot
	
    mov	r0, #0
    bl	set_in_suspend
    
    @ cpu_reg/mmu data backup address
    ldr     r3, =TCC897X_CORE_MMU_DATA_ADDR		// reg[ ]. LK loaded this register data.

    @ restore fiq_reg
    mov     r1, #Mode_FIQ | NOINT
    msr     cpsr, r1
    ldr     r0, [r3, #0x68]
    msr     spsr, r0
    ldr     r8, [r3, #0x6C]
    ldr     r9, [r3, #0x70]
    ldr     r10,[r3, #0x74]
    ldr     r11,[r3, #0x78]
    ldr     r12,[r3, #0x7C]
    ldr     sp, [r3, #0x80]
    ldr     lr, [r3, #0x84]

    @ restore abort_reg
    mov     r1, #Mode_ABT | I_Bit
    msr     cpsr, r1
    ldr     r0, [r3, #0x88]
    msr     spsr, r0
    ldr     sp, [r3, #0x8C]
    ldr     lr, [r3, #0x90]

    @ restore irq_reg
    mov     r1, #Mode_IRQ | I_Bit
    msr     cpsr, r1
    ldr     r0, [r3, #0x94]
    msr     spsr, r0
    ldr     sp, [r3, #0x98]
    ldr     lr, [r3, #0x9C]

    @ restore undef_reg
    mov     r1, #Mode_UND | I_Bit
    msr     cpsr, r1
    ldr     r0, [r3, #0xA0]
    msr     spsr, r0
    ldr     sp, [r3, #0xA4]
    ldr     lr, [r3, #0xA8]

    @ restore usr(sys)_reg
    mov     r1, #Mode_SYS | I_Bit
    msr     cpsr, r1
    ldr     sp, [r3, #0xAC]
    ldr     lr, [r3, #0xB0]

    @ restore svc_reg
    mov     r1, #Mode_SVC | I_Bit
    msr     cpsr, r1
    ldr     r0, [r3, #0x64]
    msr     spsr, r0
    ldr     sp, [r3, #0x60]

    
    @ Restore SVC context
    ldr     r3, .Lsaved_context_r13_svc
	ldr		r13, [r3]
    ldr     r3, .Lsaved_context_r14_svc
	ldr		r14, [r3]
    ldr     r3, .Lsaved_spsr_svc
    ldr     r1, [r3]
    msr     spsr_cxsf, r1
    mrs     r0, cpsr                    @ Save current mode into r0

    @ Change to system(user) mode
    mov     r1, r0
    orr     r1, r1, #0x1f
    msr     cpsr_c, r1

    @ Restore User context
    ldr     r3, .Lsaved_context_r0
    ldmia   r3, {r0-r14}
    ldr     r3, .Lsaved_cpsr
    ldr     r1, [r3]
    msr     cpsr_cxsf, r1

    @ Flush TLB (Invalidate unified TLB unlocked entries)
    mov     r1, #0
    mcr     p15, 0, r1, c8, c7, 0

    @ Set the return value
    @mov     r0, #0

    nop
	@ Restore Stack Pointer to restore r0-r12 registers.
	ldr		r0, .Lsaved_stack_pointer
	ldr		sp, [r0]

    @ Restore General Registers
    pop		{r0-r12}
   
    @ Restore return address
    ldr		r0, .Lsaved_context_r14_svc
    ldr		lr, [r0]
    mov		r0, #0
    mov		pc,	lr
ENDPROC(restore_snapshot_cpu_reg)
.align  4
.Lsaved_context_r0:         .long   saved_context_r0
.Lsaved_cpsr:               .long   saved_cpsr
.Lsaved_context_r13_svc:    .long   saved_context_r13_svc
.Lsaved_context_r14_svc:    .long   saved_context_r14_svc
.Lsaved_spsr_svc:           .long   saved_spsr_svc
.Lsaved_org_cpsr:           .long   saved_org_cpsr
.Lsaved_stack_pointer:      .long   saved_stack_pointer

#endif
# //-[TCCQB]
# //
