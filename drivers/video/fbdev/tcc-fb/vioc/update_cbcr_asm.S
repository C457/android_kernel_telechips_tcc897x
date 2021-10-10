	.arm
	.align

	.global update_cbcr

vcbcr_addr			.req	r0
p_cbcr_table		.req	r3	
vcbcr_addr_tmp	.req	r6
loop_count			.req	r14


update_cbcr:
		stmdb	sp!,{r4-r6,lr}
@void update_cbcr(unsigned short *vcbcr_addr, unsigned int width, unsigned int height, short *p_cbcr_table) 


@    loop = height * width/8;
	smulbb   r4,r2,r1
	mov			 r6,r0
  movs     r14,r4,lsr #3
  blt				close_update_cbcr
@    while(loop--)
loop_start:

  LDRH      r1,[r0],#2		@int_cbcr = *vcbcr_addr;
  subs			r14,r14,#2		@loop -=2  
  LDRH      r2,[r0],#2    @int_cbcr = *vcbcr_addr;	
  
  mov	    	r1,r1,lsl #1		
  LDRH      r4,[r0],#2		@int_cbcr = *vcbcr_addr;
  mov	    	r2,r2,lsl #1		  

  LDRH     r1,[r3,r1]			@p_cbcr_table[int_cbcr]  
  
  LDRH      r5,[r0],#2    @int_cbcr = *vcbcr_addr;	
  LDRH     r2,[r3,r2]  		@p_cbcr_table[int_cbcr]
  
  mov	    	r4,r4,lsl #1		    
  mov	    	r5,r5,lsl #1		    
  
  STRH     r1,[r6],#2

  LDRH     r4,[r3,r4]			@p_cbcr_table[int_cbcr]

  STRH     r2,[r6],#2

  LDRH     r5,[r3,r5]  		@p_cbcr_table[int_cbcr]

  STRH     r4,[r6],#2
  STRH     r5,[r6],#2
  
  bgt				loop_start

close_update_cbcr:
		ldmia		sp!,{r4-r6,pc}
	
	.end
