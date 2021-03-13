//+[TCCQB] Add for QuickBoot coprocessor store / restore
/*
 *hibernation support specific for ARM
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2006 Rafael J. Wysocki <r...@sisk.pl>
 *
 * Contact: Hiroshi DOYU <hiroshi.d...@nokia.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/io.h>

/* Used in hibernate_asm.S */
#define USER_CONTEXT_SIZE (sizeof(u32) * 15)
unsigned long saved_context_r0[USER_CONTEXT_SIZE];
unsigned long saved_cpsr;
unsigned long saved_context_r14_svc;
unsigned long saved_context_r13_svc;
unsigned long saved_spsr_svc;
unsigned long saved_org_cpsr;
unsigned long saved_stack_pointer;

/*
 * Image of the saved processor state
 *
 * coprocessor 15 registers(RW)
 */
#if defined(CONFIG_ARCH_TCC893X)
struct saved_context_a9 {
	/* CR0 */
	u32 cssr;       /* Cache Size Selection */	
	/* CR1 */
	u32 cr;         /* Control */	
	u32 actlr;
	u32 cacr;
	u32 sder;
	u32 vcr;
	/* CR2 */
	u32 ttb_0r;     /* Translation Table Base 0 */
	u32 ttb_1r;     /* Translation Table Base 1 */
	u32 ttbcr;      /* Translation Talbe Base Control */	
	/* CR3 */
	u32 dacr;       /* Domain Access Control */
	/* CR4 */
	/* CR5 */
	u32 d_fsr;      /* Data Fault Status */
	u32 i_fsr;      /* Instruction Fault Status */
	u32 d_afsr;     /* Data Auxilirary Fault Status */   
	u32 i_afsr;     /* Instruction Auxilirary Fault Status */;
	/* CR6 */
	u32 d_far;      /* Data Fault Address */
	u32 i_far;      /* Instruction Fault Address */       
	/* CR7 */
	u32 par;        /* Physical Address */
	/* CR8 */
	/* CR9 */
	u32 pmcontrolr; /* Performance Monitor Control */
	u32 cesr;       /* Count Enable Set */
	u32 cecr;       /* Count Enable Clear */
	u32 ofsr;       /* Overflow Flag Status */       
	u32 pcsr;       /* Performance Counter Selection */
	u32 ccr;        /* Cycle Count */
	u32 esr;        /* Event Selection */
	u32 pmcountr;
	u32 uer;        /* User Enable */
	u32 iesr;       /* Interrupt Enable Set */
	u32 iecr;       /* Interrupt Enable Clear */	
	/* CR10 */
	u32 d_tlblr;    /* Data TLB Lockdown Register */
	u32 prrr;       /* Primary Region Remap Register */
	u32 nrrr;       /* Normal Memory Remap Register */
	/* CR11 */
	u32 pleuar;
	u32 plepcr;
	/* CR12 */
	u32 vbar;       /* Vector Base Address Register */
	u32 mvbar;       /* Monitor Vector Base Address Register */
	u32 vir;        /* Virtualiztion Interrupt Register */
	/* CR13 */
	u32 fcse;       /* FCSE PID */
	u32 cid;        /* Context ID */       
	u32 urwtpid;    /* User read/write Thread and Process ID */
	u32 urotpid;    /* User read-only Thread and Process ID */
	u32 potpid;     /* Privileged only Thread and Process ID */      
	/* CR15 */
   u32 mtlbar;
}__attribute__((packed));
static struct saved_context_a9 saved_context_a9;

#elif defined(CONFIG_ARCH_TCC896X)
struct saved_context_a15 {
	u32 FCSEIDR;	// FCSE/PID
	u32 TPIDRURO;	// User r/o thread ID
	u32 DACR;		// Domain ID
	u32 TTBCR;		// Translation Table Base ControlRegister
	u32 TTBR1;		// Translation Table Base Reg 1
	u32 TTBR0;		// Translation Table Base Reg 0
	u32 DFSR;		// Data Fault Status Register
	u32 DFAR;		// Data Fault Address Regiser
	u32 IFSR;		// Instruction Fault Status Reg
	u32 IFAR;		// Instruction Fault Address Reg
	u32 ADFSR;		// Auxiliary Data Fault Status Reg
	u32 AIFSR;		// Auxiliary Instruction Fault Status Register
	u32 PRRR;		// Primary Region Remap Register
	u32 NMRR;		// Normal Memory Remap Register
	u32 SCTLR;		// System Control Register
	u32 ACTLR;		// Auxiliary Control Register
	u32 CPACR;		// Coprocessor Access Control Register
	u32 CONTEXTIDR;	// Context ID Register
}__attribute__((packed));
static struct saved_context_a15 saved_context_a15;

#elif defined(CONFIG_ARCH_TCC897X)
struct saved_context_a7 {
	u32 FCSEIDR;	// FCSE/PID
	u32 TPIDRURO;	// User r/o thread ID
	u32 DACR;		// Domain ID
	u32 TTBCR;		// Translation Table Base ControlRegister
	u32 TTBR1;		// Translation Table Base Reg 1
	u32 TTBR0;		// Translation Table Base Reg 0
	u32 DFSR;		// Data Fault Status Register
	u32 DFAR;		// Data Fault Address Regiser
	u32 IFSR;		// Instruction Fault Status Reg
	u32 IFAR;		// Instruction Fault Address Reg
	u32 ADFSR;		// Auxiliary Data Fault Status Reg
	u32 AIFSR;		// Auxiliary Instruction Fault Status Register
	u32 PRRR;		// Primary Region Remap Register
	u32 NMRR;		// Normal Memory Remap Register
	u32 SCTLR;		// System Control Register
	u32 ACTLR;		// Auxiliary Control Register
	u32 CPACR;		// Coprocessor Access Control Register
	u32 CONTEXTIDR;	// Context ID Register
}__attribute__((packed));
static struct saved_context_a7 saved_context_a7;

#endif

#ifdef CONFIG_SNAPSHOT_BOOT
volatile unsigned int do_snapshot_boot = 0;
EXPORT_SYMBOL(do_snapshot_boot);
asmlinkage unsigned int get_do_snapshot_boot(void)
{
	return do_snapshot_boot;
}
EXPORT_SYMBOL(get_do_snapshot_boot);
asmlinkage unsigned int set_do_snapshot_boot(unsigned int set_num)
{
	do_snapshot_boot = set_num;
	return do_snapshot_boot;
}
EXPORT_SYMBOL(set_do_snapshot_boot);
#endif

volatile static int in_suspend __nosavedata = 0;
asmlinkage int get_in_suspend(void)
{
	    return in_suspend;
}
EXPORT_SYMBOL(get_in_suspend);
asmlinkage int set_in_suspend(int set_num)
{
	    in_suspend = set_num;
		return in_suspend;
}
EXPORT_SYMBOL(set_in_suspend);

/* References to section boundaries */
extern const void __nosave_begin, __nosave_end;

/*
 * pfn_is_nosave - check if given pfn is in the 'nosave' section
 */
int pfn_is_nosave(unsigned long pfn)
{
       unsigned long nosave_begin_pfn = __pa_symbol(&__nosave_begin) >> PAGE_SHIFT;
       unsigned long nosave_end_pfn = PAGE_ALIGN(__pa_symbol(&__nosave_end)) >> PAGE_SHIFT;

       return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}

#if defined(CONFIG_ARCH_TCC893X)
static inline void __save_processor_a9_state(struct saved_context_a9 *ctxt)
{
       /* CR0 */
       asm volatile ("mrc p15, 2, %0, c0, c0, 0" : "=r"(ctxt->cssr));
       /* CR1 */
       asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r"(ctxt->cr));
       asm volatile ("mrc p15, 0, %0, c1, c0, 1" : "=r"(ctxt->actlr));      
       asm volatile ("mrc p15, 0, %0, c1, c0, 2" : "=r"(ctxt->cacr));      
//       asm volatile ("mrc p15, 0, %0, c1, c1, 1" : "=r"(ctxt->sder));      
       asm volatile ("mrc p15, 0, %0, c1, c1, 3" : "=r"(ctxt->vcr));      
       /* CR2 */
       asm volatile ("mrc p15, 0, %0, c2, c0, 0" : "=r"(ctxt->ttb_0r));
       asm volatile ("mrc p15, 0, %0, c2, c0, 1" : "=r"(ctxt->ttb_1r));
       asm volatile ("mrc p15, 0, %0, c2, c0, 2" : "=r"(ctxt->ttbcr));    
       /* CR3 */
       asm volatile ("mrc p15, 0, %0, c3, c0, 0" : "=r"(ctxt->dacr));     
       /* CR5 */
       asm volatile ("mrc p15, 0, %0, c5, c0, 0" : "=r"(ctxt->d_fsr));
       asm volatile ("mrc p15, 0, %0, c5, c0, 1" : "=r"(ctxt->i_fsr));
       asm volatile ("mrc p15, 0, %0, c5, c1, 0" : "=r"(ctxt->d_afsr));
       asm volatile ("mrc p15, 0, %0, c5, c1, 1" : "=r"(ctxt->i_afsr));  
       /* CR6 */
       asm volatile ("mrc p15, 0, %0, c6, c0, 0" : "=r"(ctxt->d_far));
       asm volatile ("mrc p15, 0, %0, c6, c0, 2" : "=r"(ctxt->i_far));       
       /* CR7 */
       asm volatile ("mrc p15, 0, %0, c7, c4, 0" : "=r"(ctxt->par));             
       /* CR9 */
       asm volatile ("mrc p15, 0, %0, c9, c12, 0" : "=r"(ctxt->pmcontrolr));   
       asm volatile ("mrc p15, 0, %0, c9, c12, 1" : "=r"(ctxt->cesr));  
       asm volatile ("mrc p15, 0, %0, c9, c12, 2" : "=r"(ctxt->cecr));   
       asm volatile ("mrc p15, 0, %0, c9, c12, 3" : "=r"(ctxt->ofsr));   

       asm volatile ("mrc p15, 0, %0, c9, c12, 5" : "=r"(ctxt->pcsr));
       asm volatile ("mrc p15, 0, %0, c9, c13, 0" : "=r"(ctxt->ccr));
       asm volatile ("mrc p15, 0, %0, c9, c13, 1" : "=r"(ctxt->esr));
       asm volatile ("mrc p15, 0, %0, c9, c13, 2" : "=r"(ctxt->pmcountr));
       asm volatile ("mrc p15, 0, %0, c9, c14, 0" : "=r"(ctxt->uer));
       asm volatile ("mrc p15, 0, %0, c9, c14, 1" : "=r"(ctxt->iesr)); 
       asm volatile ("mrc p15, 0, %0, c9, c14, 2" : "=r"(ctxt->iecr));
       /* CR10 */
       asm volatile ("mrc p15, 0, %0, c10, c0, 0" : "=r"(ctxt->d_tlblr));
       asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(ctxt->prrr));
       asm volatile ("mrc p15, 0, %0, c10, c2, 1" : "=r"(ctxt->nrrr));
       /* CR11 */
       /* CR12 */
       asm volatile ("mrc p15, 0, %0, c12, c0, 0" : "=r"(ctxt->vbar));
       asm volatile ("mrc p15, 0, %0, c12, c0, 1" : "=r"(ctxt->mvbar));
       asm volatile ("mrc p15, 0, %0, c12, c1, 1" : "=r"(ctxt->vir));
       /* CR13 */
       //asm volatile ("mrc p15, 0, %0, c13, c0, 0" : "=r"(ctxt->fcse));
       asm volatile ("mrc p15, 0, %0, c13, c0, 1" : "=r"(ctxt->cid));
       asm volatile ("mrc p15, 0, %0, c13, c0, 2" : "=r"(ctxt->urwtpid));
       asm volatile ("mrc p15, 0, %0, c13, c0, 3" : "=r"(ctxt->urotpid));
       asm volatile ("mrc p15, 0, %0, c13, c0, 4" : "=r"(ctxt->potpid));    
       /* CR15 */
       asm volatile ("mrc p15, 5, %0, c15, c7, 2" : "=r"(ctxt->mtlbar));    
}

static inline void __restore_processor_a9_state(struct saved_context_a9 *ctxt)
{
       asm volatile ("mcr p15, 0, %0, c1, c0, 2" : : "r"(ctxt->cacr));    	//
//	   asm volatile ("mcr p15, 0, %0, c13, c0, 0" : : "r"(ctxt->fcse));
       asm volatile ("mcr p15, 0, %0, c13, c0, 1" : : "r"(ctxt->cid));			//
       asm volatile ("mcr p15, 0, %0, c13, c0, 3" : : "r"(ctxt->urotpid));		//

#if 0
       /* CR0 */
       asm volatile ("mcr p15, 2, %0, c0, c0, 0" : : "r"(ctxt->cssr));     
       /* CR1 */
       asm volatile ("mcr p15, 0, %0, c1, c0, 0" : : "r"(ctxt->cr));
       asm volatile ("mcr p15, 0, %0, c1, c0, 1" : : "r"(ctxt->actlr));    
       asm volatile ("mcr p15, 0, %0, c1, c0, 2" : : "r"(ctxt->cacr));    
//       asm volatile ("mcr p15, 0, %0, c1, c1, 1" : : "r"(ctxt->sder));    
       asm volatile ("mcr p15, 0, %0, c1, c1, 3" : : "r"(ctxt->vcr));    
       /* CR2 */
       asm volatile ("mcr p15, 0, %0, c2, c0, 0" : : "r"(ctxt->ttb_0r));
       asm volatile ("mcr p15, 0, %0, c2, c0, 1" : : "r"(ctxt->ttb_1r));
       asm volatile ("mcr p15, 0, %0, c2, c0, 2" : : "r"(ctxt->ttbcr));      
       /* CR3 */
       asm volatile ("mcr p15, 0, %0, c3, c0, 0" : : "r"(ctxt->dacr));     
       /* CR5 */
       asm volatile ("mcr p15, 0, %0, c5, c0, 0" : : "r"(ctxt->d_fsr));
       asm volatile ("mcr p15, 0, %0, c5, c0, 1" : : "r"(ctxt->i_fsr));
       asm volatile ("mcr p15, 0, %0, c5, c1, 0" : : "r"(ctxt->d_afsr));
       asm volatile ("mcr p15, 0, %0, c5, c1, 1" : : "r"(ctxt->i_afsr)); 
       /* CR6 */
       asm volatile ("mcr p15, 0, %0, c6, c0, 0" : : "r"(ctxt->d_far));
       asm volatile ("mcr p15, 0, %0, c6, c0, 2" : : "r"(ctxt->i_far));      
       /* CR7 */
       asm volatile ("mcr p15, 0, %0, c7, c4, 0" : : "r"(ctxt->par));           
       /* CR9 */
       asm volatile ("mcr p15, 0, %0, c9, c12, 0" : : "r"(ctxt->pmcontrolr));     
       asm volatile ("mcr p15, 0, %0, c9, c12, 1" : : "r"(ctxt->cesr));
       asm volatile ("mcr p15, 0, %0, c9, c12, 2" : : "r"(ctxt->cecr));
       asm volatile ("mcr p15, 0, %0, c9, c12, 3" : : "r"(ctxt->ofsr)); 

       asm volatile ("mcr p15, 0, %0, c9, c12, 5" : : "r"(ctxt->pcsr));
       asm volatile ("mcr p15, 0, %0, c9, c13, 0" : : "r"(ctxt->ccr));
       asm volatile ("mcr p15, 0, %0, c9, c13, 1" : : "r"(ctxt->esr));
       asm volatile ("mcr p15, 0, %0, c9, c13, 2" : : "r"(ctxt->pmcountr));
       asm volatile ("mcr p15, 0, %0, c9, c14, 0" : : "r"(ctxt->uer));
       asm volatile ("mcr p15, 0, %0, c9, c14, 1" : : "r"(ctxt->iesr));
       asm volatile ("mcr p15, 0, %0, c9, c14, 2" : : "r"(ctxt->iecr));
       /* CR10 */
       asm volatile ("mcr p15, 0, %0, c10, c0, 0" : : "r"(ctxt->d_tlblr));
       asm volatile ("mcr p15, 0, %0, c10, c2, 0" : : "r"(ctxt->prrr));
       asm volatile ("mcr p15, 0, %0, c10, c2, 1" : : "r"(ctxt->nrrr));
       /* CR11 */
       /* CR12 */
       asm volatile ("mcr p15, 0, %0, c12, c0, 0" : : "r"(ctxt->vbar));    
       asm volatile ("mcr p15, 0, %0, c12, c0, 1" : : "r"(ctxt->mvbar));    
       asm volatile ("mcr p15, 0, %0, c12, c1, 1" : : "r"(ctxt->vir));    
       /* CR13 */
       //asm volatile ("mcr p15, 0, %0, c13, c0, 0" : : "r"(ctxt->fcse));
       asm volatile ("mcr p15, 0, %0, c13, c0, 1" : : "r"(ctxt->cid));
       asm volatile ("mcr p15, 0, %0, c13, c0, 2" : : "r"(ctxt->urwtpid));
       asm volatile ("mcr p15, 0, %0, c13, c0, 3" : : "r"(ctxt->urotpid));
       asm volatile ("mcr p15, 0, %0, c13, c0, 4" : : "r"(ctxt->potpid));      
       /* CR15 */
       asm volatile ("mcr p15, 5, %0, c15, c7, 2" : : "r"(ctxt->mtlbar));      
#endif
}
#elif defined(CONFIG_ARCH_TCC896X)
static inline void __save_processor_a15_state(struct saved_context_a15 *ctxt)
{
	asm volatile ("mrc p15, 0, %0, c13, c0, 0" : "=r"(ctxt->FCSEIDR)); // FCSE/PID
	asm volatile ("mrc p15, 0, %0, c13, c0, 3" : "=r"(ctxt->TPIDRURO));	// User r/o thread ID

	/*		This Coprocessor Registers will be restored in LK Bootloader. - below -		*/
	asm volatile ("mrc p15, 0, %0, c3, c0, 0" : "=r"(ctxt->DACR)); // Domain ID
	asm volatile ("mrc p15, 0, %0, c2, c0, 2" : "=r"(ctxt->TTBCR)); // Translation Table Base ControlRegister
	asm volatile ("mrc p15, 0, %0, c2, c0, 1" : "=r"(ctxt->TTBR1)); // Translation Table Base Reg 1
	asm volatile ("mrc p15, 0, %0, c2, c0, 0" : "=r"(ctxt->TTBR0)); // Translation Table Base Reg 0
	asm volatile ("mrc p15, 0, %0, c5, c0, 0" : "=r"(ctxt->DFSR)); // Data Fault Status Register
	asm volatile ("mrc p15, 0, %0, c6, c0, 0" : "=r"(ctxt->DFAR)); // Data Fault Address Regiser
	asm volatile ("mrc p15, 0, %0, c5, c0, 1" : "=r"(ctxt->IFSR)); // Instruction Fault Status Reg
	asm volatile ("mrc p15, 0, %0, c6, c0, 2" : "=r"(ctxt->IFAR)); // Instruction Fault Address Reg
	asm volatile ("mrc p15, 0, %0, c5, c1, 0" : "=r"(ctxt->ADFSR));//Auxiliary Data Fault Status Reg
	asm volatile ("mrc p15, 0, %0, c5, c1, 1" : "=r"(ctxt->AIFSR)); // Auxiliary Instruction Fault Status Register
	asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(ctxt->PRRR)); // Primary Region Remap Register
	asm volatile ("mrc p15, 0, %0, c10, c2, 1" : "=r"(ctxt->NMRR)); // Normal Memory Remap Register
	asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r"(ctxt->SCTLR)); // System Control Register
	/*		This Coprocessor Registers will be restored in LK Bootloader. - above -		*/
		
	asm volatile ("mrc p15, 0, %0, c1, c0, 1" : "=r"(ctxt->ACTLR)); // Auxiliary Control Register
	asm volatile ("mrc p15, 0, %0, c1, c0, 2" : "=r"(ctxt->CPACR)); // Coprocessor Access Control Register
	asm volatile ("mrc p15, 0, %0, c13, c0, 1" : "=r"(ctxt->CONTEXTIDR)); // Context ID Register
}

static inline void __restore_processor_a15_state(struct saved_context_a15 *ctxt)
{
/*		Init Coprocessor Registers - below -								*/	
//	asm volatile ("mcr p15, 0, %0, c10, c2, 1" : : "r"(ctxt->ZERO)); // invalidate TLBs
//	asm volatile ("mcr p15, 0, %0, c10, c2, 1" : : "r"(ctxt->ZERO)); // invalidate I cache
//	asm volatile ("mcr p15, 0, %0, c10, c2, 1" : : "r"(ctxt->ZERO)); // set reserved context ID
/*		Init Coprocessor Registers - above -								*/	

/*		Restore Coprocessor Registers - below -								*/
	asm volatile ("mcr p15, 0, %0, c13, c0, 0" : : "r"(ctxt->FCSEIDR)); // FCSE/PID
	asm volatile ("mcr p15, 0, %0, c13, c0, 3" : : "r"(ctxt->TPIDRURO));	// User r/o thread ID
/*		Restore Coprocessor Registers - above -								*/

/*		This Coprocessor Registers are already restored in LK Bootloader.		
	asm volatile ("mcr p15, 0, %0, c3, c0, 0" : : "r"(ctxt->DACR)); // Domain ID
	asm volatile ("mcr p15, 0, %0, c2, c0, 0" : : "r"(ctxt->TTBR0)); // Translation Table Base Reg 0
	asm volatile ("mcr p15, 0, %0, c2, c0, 2" : : "r"(ctxt->TTBCR)); // Translation Table Base ControlRegister
	asm volatile ("mcr p15, 0, %0, c1, c0, 0" : : "r"(ctxt->SCTLR)); // System Control Register
*/

/*		Restore Coprocessor Registers - below -								*/
	asm volatile ("mcr p15, 0, %0, c1, c0, 1" : : "r"(ctxt->ACTLR)); // Auxiliary Control Register
	asm volatile ("mcr p15, 0, %0, c1, c0, 2" : : "r"(ctxt->CPACR)); // Coprocessor Access Control Register
	asm volatile ("mcr p15, 0, %0, c13, c0, 1" : : "r"(ctxt->CONTEXTIDR)); // Context ID Register
/*		Restore Coprocessor Registers - above -								*/
       
	
/*		Init Coprocessor Registers - below -								*/	
//	asm volatile ("mcr p15, 0, %0, c10, c2, 0" : : "r"(PRRR)); // Already restored in LK
//	asm volatile ("mcr p15, 0, %0, c10, c2, 1" : : "r"(NMRR)); // Already restored in LK
/*		Init Coprocessor Registers - above -								*/	
}
#elif defined(CONFIG_ARCH_TCC897X)
static inline void __save_processor_a7_state(struct saved_context_a7 *ctxt)
{
	asm volatile ("mrc p15, 0, %0, c13, c0, 0" : "=r"(ctxt->FCSEIDR)); // FCSE/PID
	asm volatile ("mrc p15, 0, %0, c13, c0, 3" : "=r"(ctxt->TPIDRURO));	// User r/o thread ID
#if 0
	/*		This Coprocessor Registers will be restored in LK Bootloader. - below -		*/
	asm volatile ("mrc p15, 0, %0, c3, c0, 0" : "=r"(ctxt->DACR)); // Domain ID
	asm volatile ("mrc p15, 0, %0, c2, c0, 2" : "=r"(ctxt->TTBCR)); // Translation Table Base ControlRegister
	asm volatile ("mrc p15, 0, %0, c2, c0, 1" : "=r"(ctxt->TTBR1)); // Translation Table Base Reg 1
	asm volatile ("mrc p15, 0, %0, c2, c0, 0" : "=r"(ctxt->TTBR0)); // Translation Table Base Reg 0
	asm volatile ("mrc p15, 0, %0, c5, c0, 0" : "=r"(ctxt->DFSR)); // Data Fault Status Register
	asm volatile ("mrc p15, 0, %0, c6, c0, 0" : "=r"(ctxt->DFAR)); // Data Fault Address Regiser
	asm volatile ("mrc p15, 0, %0, c5, c0, 1" : "=r"(ctxt->IFSR)); // Instruction Fault Status Reg
	asm volatile ("mrc p15, 0, %0, c6, c0, 2" : "=r"(ctxt->IFAR)); // Instruction Fault Address Reg
	asm volatile ("mrc p15, 0, %0, c5, c1, 0" : "=r"(ctxt->ADFSR));//Auxiliary Data Fault Status Reg
	asm volatile ("mrc p15, 0, %0, c5, c1, 1" : "=r"(ctxt->AIFSR)); // Auxiliary Instruction Fault Status Register
	asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(ctxt->PRRR)); // Primary Region Remap Register
	asm volatile ("mrc p15, 0, %0, c10, c2, 1" : "=r"(ctxt->NMRR)); // Normal Memory Remap Register
	asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r"(ctxt->SCTLR)); // System Control Register
	/*		This Coprocessor Registers will be restored in LK Bootloader. - above -		*/
#endif	
	asm volatile ("mrc p15, 0, %0, c1, c0, 1" : "=r"(ctxt->ACTLR)); // Auxiliary Control Register
	asm volatile ("mrc p15, 0, %0, c1, c0, 2" : "=r"(ctxt->CPACR)); // Coprocessor Access Control Register
	asm volatile ("mrc p15, 0, %0, c13, c0, 1" : "=r"(ctxt->CONTEXTIDR)); // Context ID Register
}

static inline void __restore_processor_a7_state(struct saved_context_a7 *ctxt)
{
/*		Restore Coprocessor Registers - below -								*/
	asm volatile ("mcr p15, 0, %0, c13, c0, 0" : : "r"(ctxt->FCSEIDR)); // FCSE/PID
	asm volatile ("mcr p15, 0, %0, c13, c0, 3" : : "r"(ctxt->TPIDRURO));	// User r/o thread ID

/*		Restore Coprocessor Registers - below -								*/
	asm volatile ("mcr p15, 0, %0, c1, c0, 1" : : "r"(ctxt->ACTLR)); // Auxiliary Control Register
	asm volatile ("mcr p15, 0, %0, c1, c0, 2" : : "r"(ctxt->CPACR)); // Coprocessor Access Control Register
	asm volatile ("mcr p15, 0, %0, c13, c0, 1" : : "r"(ctxt->CONTEXTIDR)); // Context ID Register
}
#endif	// CONFIG_ARCH_TCC897X


#if defined(CONFIG_ARCH_TCC893X)
#include <asm/cacheflush.h>

#define SCU_CTRL		0x00
#define TCC_SCU_BASE_ADDR 0x77200000

void restore_scu_state(void)
{
	u32 scu_ctrl;
    void __iomem *scu_base = (void __iomem *)tcc_p2v(TCC_SCU_BASE_ADDR);

#ifdef CONFIG_ARM_ERRATA_764369
	/* Cortex-A9 only */
	scu_ctrl = __raw_readl(scu_base + 0x30);
	if (!(scu_ctrl & 1))
		__raw_writel(scu_ctrl | 0x1, scu_base + 0x30);
#endif

	scu_ctrl = __raw_readl(scu_base + SCU_CTRL);
	/* already enabled? */
	if (!(scu_ctrl & 1)) {
    	scu_ctrl |= 1;
    	__raw_writel(scu_ctrl, scu_base + SCU_CTRL);
    }
	/*
	 * Ensure that the data accessed by CPU0 before the SCU was
	 * initialised is visible to the other CPUs.
	 */
	flush_cache_all();
}
#endif

void save_processor_state(void)
{
	preempt_disable();

#if defined(CONFIG_ARCH_TCC893X)
	__save_processor_a9_state(&saved_context_a9);
#elif defined(CONFIG_ARCH_TCC896X)
	__save_processor_a15_state(&saved_context_a15);
#elif defined(CONFIG_ARCH_TCC897X)
	__save_processor_a7_state(&saved_context_a7);
#endif
}

void restore_processor_state(void)
{
#if defined(CONFIG_ARCH_TCC893X)
	__restore_processor_a9_state(&saved_context_a9);
#elif defined(CONFIG_ARCH_TCC896X)
	__restore_processor_a15_state(&saved_context_a15);
#elif defined(CONFIG_ARCH_TCC897X)
	__restore_processor_a7_state(&saved_context_a7);
#endif

	preempt_enable();
}
//-[TCCQB]
//

