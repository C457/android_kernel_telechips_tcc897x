/************************************************************************
 *    Telechips Graphic Engine
 *    ------------------------------------------------
 *
 *    FUNCTION    : Graphic Engine drvier
 *    MODEL       :
 *    CPU NAME    : TCC897X
 *
 *    START DATE    : 2014. 9. 16.
 *    MODIFY DATE :
 *    DEVISION    : DEPT. SYSTEM 3 TEAM
 ************************************************************************/

#include <linux/delay.h>
#include <linux/types.h>

#include <asm/io.h>
#include <mach/bsp.h>
#include <mach/tcc_g2d.h>

#include <linux/interrupt.h>
#include <linux/irq.h>


/*------------------------------------------------------------------
  Gre2d_SetFCh_address
  graphic engine Front End channel address 0,1,2 setting
  -------------------------------------------------------------------*/
void Gre2d_SetFCh_address(void *virt_addr, G2D_CHANNEL ch, unsigned int add0, unsigned int add1, unsigned int add2)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(ch)
	{
		case FCH0_CH:
			BITCSET(pHwOVERLAYMIXER->FCH0_SADDR0.nREG, 0xFFFFFFFF, add0);
			BITCSET(pHwOVERLAYMIXER->FCH0_SADDR1.nREG, 0xFFFFFFFF, add1);
			BITCSET(pHwOVERLAYMIXER->FCH0_SADDR2.nREG, 0xFFFFFFFF, add2);
			break;

		case FCH1_CH:
			BITCSET(pHwOVERLAYMIXER->FCH1_SADDR0.nREG, 0xFFFFFFFF, add0);
			BITCSET(pHwOVERLAYMIXER->FCH1_SADDR1.nREG, 0xFFFFFFFF, add1);
			BITCSET(pHwOVERLAYMIXER->FCH1_SADDR2.nREG, 0xFFFFFFFF, add2);	
			break;

		default:
			break;      
	}
}


/*------------------------------------------------------------------
  Gre2d_SetFCh_position
  graphic engine Front channel position settig

  frameps_x, frameps_y : frame pixsel size
  poffset_x, poffset_y : pixsel offset
  imageps_x, imageps_y : imagme pixel size
  winps_x, winps_y : window pixsel offset
  -------------------------------------------------------------------*/
void Gre2d_SetFCh_position(void *virt_addr, G2D_CHANNEL ch, unsigned int  frameps_x, unsigned int  frameps_y, 
		unsigned int  poffset_x, unsigned int  poffset_y, unsigned int  imageps_x, unsigned int  imageps_y, unsigned int  winps_x, unsigned int  winps_y )
{

	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(ch)
	{
		case FCH0_CH:
			BITCSET(pHwOVERLAYMIXER->FCH0_SFSIZE.nREG,0x0FFF0FFF,((frameps_y<<16) | frameps_x)); // pHwOVERLAYMIXER->FCH0_SFSIZE
			BITCSET(pHwOVERLAYMIXER->FCH0_SOFF.nREG,0x0FFF0FFF,((poffset_y<<16) | poffset_x));
			BITCSET(pHwOVERLAYMIXER->FCH0_SISIZE.nREG,0x0FFF0FFF,((imageps_y<<16) | imageps_x));
			BITCSET(pHwOVERLAYMIXER->FCH0_WOFF.nREG,0x0FFF0FFF,((winps_y<<16) | winps_x));	
			break;

		case FCH1_CH:
			BITCSET(pHwOVERLAYMIXER->FCH1_SFSIZE.nREG,0x0FFF0FFF,((frameps_y<<16) | frameps_x)); // pHwOVERLAYMIXER->FCH0_SFSIZE
			BITCSET(pHwOVERLAYMIXER->FCH1_SOFF.nREG,0x0FFF0FFF,((poffset_y<<16) | poffset_x));
			BITCSET(pHwOVERLAYMIXER->FCH1_SISIZE.nREG,0x0FFF0FFF,((imageps_y<<16) | imageps_x));
			BITCSET(pHwOVERLAYMIXER->FCH1_WOFF.nREG,0x0FFF0FFF,((winps_y<<16) | winps_x));	
			break;

		default:

			break;
	}
}


/*------------------------------------------------------------------
  Gre2d_SetFCh_control
  graphic engine Front channel control setting
mode : flip, rotate
data_form : rgb, yuv, alpha-rgb
-------------------------------------------------------------------*/
void Gre2d_SetFCh_control(void *virt_addr, G2D_CHANNEL ch, G2D_MABC_TYPE MABC, unsigned char LUTE, unsigned char SSUV, G2D_OP_MODE mode, G2D_ZF_TYPE ZF, G2D_FMT_CTRL data_form)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(ch)
	{
		case FCH0_CH:
			BITCSET(pHwOVERLAYMIXER->FCH0_SCTRL.nREG, 0x0F00FFFF, (LUTE<<12)|(SSUV<<11)|((mode<<8)&HwGE_FCHO_OPMODE)
					|(ZF<<5)| (data_form.format& HwGE_FCHO_SDFRM)
					|((data_form.data_swap<<24) & HwGE_FCH_SSB) 
			       );			

			break;

		case FCH1_CH:
			BITCSET(pHwOVERLAYMIXER->FCH1_SCTRL.nREG, 0x0F00FFFF,  (LUTE<<12)|(SSUV<<11)|((mode<<8)&HwGE_FCHO_OPMODE)
					|(ZF<<5)| (data_form.format& HwGE_FCHO_SDFRM)
					|((data_form.data_swap<<24) & HwGE_FCH_SSB)
			       );
			break;

		default:
			break;		
	}

}

/*------------------------------------------------------------------
  Gre2d_SetFCh_Chroma_key
  graphic engine Front channel chroma key Set

  -------------------------------------------------------------------*/
void Gre2d_SetFCh_Chroma_key(void *virt_addr, G2D_CHANNEL ch, unsigned char RY, unsigned char GU, unsigned char BV)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;
	switch(ch)
	{
		case FCH0_CH:
			BITCSET(pHwOVERLAYMIXER->S0_CHROMA.nREG,0x00FFFFFF, (((RY<<16)&0xFF0000) | ((GU<<8)&0xFF00)| (BV&0xFF))); // pHwOVERLAYMIXER->S0_CHROMA
			break;

		default:
			break;      
	}
}


/*------------------------------------------------------------------
  Gre2d_SetFCh_Arithmetic_par
  graphic engine Front channel Arithmetic parameter setting
  -------------------------------------------------------------------*/
void Gre2d_SetFCh_Arithmetic_par(void *virt_addr, G2D_CHANNEL ch, unsigned char RY, unsigned char GU, unsigned char BV)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(ch)
	{
		case FCH0_CH:
			BITCSET(pHwOVERLAYMIXER->S0_PAR.nREG,0x00FFFFFF, (((RY<<16)&0xFF0000) | ((GU<<8)&0xFF00)| (BV&0xFF))); // pHwOVERLAYMIXER->S0_PAR
			break;

		default:
			break;      
	}
}


/*------------------------------------------------------------------
  Gre2d_src_ctrl
  graphic engine sorce control
  -------------------------------------------------------------------*/
void Gre2d_src_ctrl(void *virt_addr, G2D_SRC_CTRL reg)
{
	unsigned int sf_ctrl_reg = 0, sa_ctrl_reg = 0;

	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	// source YUV to RGB converter enable 	sf_ctrl
	sf_ctrl_reg |= (((reg.src0_y2r.src_y2r <<24) & Hw2D_SFCTRL_S0_Y2REN) | 
			((reg.src1_y2r.src_y2r <<25) & Hw2D_SFCTRL_S1_Y2REN) | ((reg.src2_y2r.src_y2r <<26) & Hw2D_SFCTRL_S2_Y2REN));

	// source YUV to RGB coverter type	sf_ctrl
	sf_ctrl_reg |= (((reg.src0_y2r.src_y2r_type<<16) & Hw2D_SFCTRL_S0_Y2RMODE) | 
			((reg.src1_y2r.src_y2r_type <<18) & Hw2D_SFCTRL_S1_Y2RMODE) | ((reg.src2_y2r.src_y2r_type <<20) & Hw2D_SFCTRL_S2_Y2RMODE));

	// source select  sf_ctrl
	sf_ctrl_reg |= ((reg.src_sel_0) & Hw2D_SFCTRL_S0_SEL) |((reg.src_sel_1<<2) & Hw2D_SFCTRL_S1_SEL) 
		| ((reg.src_sel_2<<4) & Hw2D_SFCTRL_S2_SEL) | ((reg.src_sel_3<<6) & Hw2D_SFCTRL_S3_SEL) ;

	// source arithmetic mode 	sa_ctrl
	sa_ctrl_reg |= (((reg.src0_arith ) & Hw2D_SACTRL_S0_ARITHMODE) | 
			((reg.src1_arith<<4) & Hw2D_SACTRL_S1_ARITHMODE) | ((reg.src2_arith<<8) & Hw2D_SACTRL_S2_ARITHMODE));
	// source chroma key enable : for arithmetic	sa_ctrl
	sa_ctrl_reg |= (((reg.src0_chroma_en<<16) & Hw2D_SACTRL_S0_CHROMAEN) | 
			((reg.src1_chroma_en<<17) & Hw2D_SACTRL_S1_CHROMAEN) | ((reg.src2_chroma_en<<18) & Hw2D_SACTRL_S2_CHROMAEN));

	BITCSET(pHwOVERLAYMIXER->SF_CTRL.nREG, 0x0FFFFFFF, sf_ctrl_reg); 
	BITCSET(pHwOVERLAYMIXER->SA_CTRL.nREG, 0x0FFFFFFF, sa_ctrl_reg); 

}


/* -------- Source Operator pattern set ------- */


/*------------------------------------------------------------------
  Gre2d_operator_set
  graphic engine operator 0, 1 setting 
  -------------------------------------------------------------------*/
void Gre2d_operator_set(void *virt_addr, G2D_OP_TYPE op_set, unsigned short alpha , unsigned char RY, unsigned char GU, unsigned char BV)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(op_set)
	{
		case OP_0:
			BITCSET(pHwOVERLAYMIXER->OP0_PAT.nREG, 0x00FFFFFF,((RY<<16)&HwGE_PAT_RY) | ((GU<<8)&HwGE_PAT_GU)| (BV&HwGE_PAT_BV));   // pHwOVERLAYMIXER->OP0_PAT
			BITCSET(pHwOVERLAYMIXER->OP0_ALPHA.nREG,0x0000FFFF, ( (alpha)& HwGE_ALPHA) );
			break;

		default:
			break;
	}

}




/*------------------------------------------------------------------
  Gre2d_operator_ctrl
  graphic engine operator control register setting
  -------------------------------------------------------------------*/

void Gre2d_operator_ctrl(void *virt_addr, G2D_OP_TYPE op_set, G2D_OP_ACON ACON1, G2D_OP_ACON ACON0, G2D_OP_CCON CCON1, G2D_OP_CCON CCON0, G2D_OP_ATUNE ATUNE, G2D_OP1_CHROMA CSEL,GE_ROP_TYPE op )
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	switch(op_set){

		case OP_0:
			BITCSET(pHwOVERLAYMIXER->OP0_CTRL.nREG, 0xFFFFFFFF, (   ( (ACON1<<28)&HwGE_OP_CTRL_ACON1)| ((ACON0<<24)&HwGE_OP_CTRL_ACON0) 
						| ( (CCON1<<21)&HwGE_OP_CTRL_CCON1)| ((CCON0<<16)&HwGE_OP_CTRL_CCON0)
						| ( (ATUNE<<12)&HwGE_OP_CTRL_ATUNE)| ((CSEL<<8)&HwGE_OP_CTRL_CSEL)	
						| (op & HwGE_OP_CTRL_OPMODE)));
			break;

		default:
			break;

	}
}


/*-------- BACK END CHANNEL DESTINATION SETTIG. -------*/

/*------------------------------------------------------------------
  Gre2d_SetBCh_address
  graphic engine BACK End channel address 0,1,2 setting
  -------------------------------------------------------------------*/
void Gre2d_SetBCh_address(void *virt_addr, G2D_CHANNEL ch, unsigned int add0, unsigned int add1, unsigned int add2)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	if(ch == DEST_CH)
	{
		BITCSET(pHwOVERLAYMIXER->BCH_DADDR0.nREG, 0xFFFFFFFF, add0); // pHwOVERLAYMIXER->BCH_DADDR0
		BITCSET(pHwOVERLAYMIXER->BCH_DADDR1.nREG, 0xFFFFFFFF, add1); // pHwOVERLAYMIXER->BCH_DADDR0
		BITCSET(pHwOVERLAYMIXER->BCH_DADDR2.nREG, 0xFFFFFFFF, add2); // pHwOVERLAYMIXER->BCH_DADDR0
	}
}



/*------------------------------------------------------------------
  Gre2d_SetBCh_position
  graphic engine BACK END channel position settig

  frameps_x, frameps_y : frame pixsel size
  poffset_x, poffset_y : pixsel offset
  -------------------------------------------------------------------*/
void Gre2d_SetBCh_position(void *virt_addr, G2D_CHANNEL ch, unsigned int  frameps_x, unsigned int  frameps_y, unsigned int  poffset_x, unsigned int  poffset_y)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	if(ch == DEST_CH)
	{
		BITCSET(pHwOVERLAYMIXER->BCH_DFSIZE.nREG, 0x0FFF0FFF, ((frameps_y<<16) | frameps_x)); // pHwOVERLAYMIXER->BCH_DFSIZE
		BITCSET(pHwOVERLAYMIXER->BCH_DOFF.nREG, 0x0FFF0FFF, ((poffset_y<<16) | poffset_x)); // pHwOVERLAYMIXER->BCH_DOFF
	}   
}




/*------------------------------------------------------------------
  Gre2d_SetBCh_control
  graphic engine Back END channel control setting
  ysel, xsel:
converter_en : format converter enable RGB -> YUV
converter_mode : format converter mode RGB -> YUV
opmode : flip, rotate
data_form : rgb, yuv, alpha-rgb
-------------------------------------------------------------------*/
void Gre2d_SetBCh_control(void *virt_addr, G2D_BCH_CTRL_TYPE *reg)
{
	unsigned int BCH_ctrl_reg = 0;
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	BCH_ctrl_reg |= ((reg->MABC<<21) & HwGE_BCH_DCTRL_MABC);
	BCH_ctrl_reg |= ((reg->ysel<<18) & HwGE_BCH_DCTRL_YSEL);
	BCH_ctrl_reg |= ((reg->xsel<<16) & HwGE_BCH_DCTRL_XSEL);
	BCH_ctrl_reg |= ((reg->converter_en<<15) & HwGE_BCH_DCTRL_CEN);
	BCH_ctrl_reg |= ((reg->converter_mode<<13) & HwGE_BCH_DCTRL_CMODE);
	BCH_ctrl_reg |= ((reg->DSUV<<11) & HwGE_BCH_DCTRL_DSUV);
	BCH_ctrl_reg |= ((reg->opmode<<8) & HwGE_BCH_DCTRL_OPMODE);
	BCH_ctrl_reg |= ((reg->dithering_type<<6) & HwGE_BCH_DCTRL_DOP);
	BCH_ctrl_reg |= ((reg->dithering_en<<5) & HwGE_BCH_DCTRL_DEN);
	BCH_ctrl_reg |= (reg->data_form.format & HwGE_BCH_DCTRL_DDFRM);
	BCH_ctrl_reg |= ((reg->data_form.data_swap<<24) & HwGE_DCH_SSB);

	BITCSET(pHwOVERLAYMIXER->BCH_DCTRL.nREG, 0xFFFFFFFFF, BCH_ctrl_reg); // pHwOVERLAYMIXER->BCH_DCTRL
}

void Gre2d_SetDitheringMatrix(void *virt_addr, unsigned short *Matrix)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	BITCSET(pHwOVERLAYMIXER->BCH_DDMAT0.nREG, 0x1F1F1F1F, Matrix[0] | (Matrix[1]<<8) | (Matrix[2]<<16) | (Matrix[3]<<24)); // pHwOVERLAYMIXER->BCH_DDMAT0
	BITCSET(pHwOVERLAYMIXER->BCH_DDMAT1.nREG, 0x1F1F1F1F, Matrix[4] | (Matrix[5]<<8) | (Matrix[6]<<16) | (Matrix[7]<<24));	
	BITCSET(pHwOVERLAYMIXER->BCH_DDMAT2.nREG, 0x1F1F1F1F, Matrix[8] | (Matrix[9]<<8) | (Matrix[10]<<16) | (Matrix[11]<<24));
	BITCSET(pHwOVERLAYMIXER->BCH_DDMAT3.nREG, 0x1F1F1F1F, Matrix[12] | (Matrix[13]<<8) | (Matrix[14]<<16) | (Matrix[15]<<24));
}


/*------------------------------------------------------------------
  Gre2d_Grp_enable
  graphic engine channel enable control
  -------------------------------------------------------------------*/
void Gre2d_Grp_enable(void *virt_addr, G2D_EN grp_enalbe, unsigned char int_en)
{
	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	BITCSET( pHwOVERLAYMIXER->OM_CTRL.nREG, (HwGE_GE_CTRL_EN|HwGE_GE_INT_EN), ((int_en<<16)|grp_enalbe));
}


/*------------------------------------------------------------------
  Gre2d_Grp_check
  graphic engine transger check
  -------------------------------------------------------------------*/
unsigned int Gre2d_Grp_check(void *virt_addr)
{
	unsigned int contrReg = 0;

	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	contrReg = pHwOVERLAYMIXER->OM_CTRL.nREG; //pHwOVERLAYMIXER->OM_CTRL

	return contrReg;
}



/*------------------------------------------------------------------
  Gre2d_Grp_int_ctrl 
  graphic engine interrupt control 
wr : write / read
int_irq : interrupt request 
int_flg : flag bit
-------------------------------------------------------------------*/
G2D_INT_TYPE Gre2d_Grp_int_ctrl(void *virt_addr, unsigned char wr, G2D_INT_TYPE flag, unsigned char int_irq, unsigned char int_flg)
{
	G2D_INT_TYPE ret_v = 0;

	OVERLAYMIXER *pHwOVERLAYMIXER = (OVERLAYMIXER *)virt_addr;

	if(wr) {
		if(flag & G2D_INT_R_IRQ)
			BITCSET( pHwOVERLAYMIXER->OM_IREQ.nREG, HwGE_GE_IREQ_IRQ, (int_irq ? HwGE_GE_IREQ_IRQ : 0));

		if(flag & G2D_INT_R_FLG)
			BITCSET( pHwOVERLAYMIXER->OM_IREQ.nREG, HwGE_GE_IREQ_FLG, (int_flg ? HwGE_GE_IREQ_FLG : 0));
	}
	else {
		if(ISSET(pHwOVERLAYMIXER->OM_IREQ.nREG, HwGE_GE_IREQ_IRQ))
			ret_v |= G2D_INT_R_IRQ;

		if(ISSET(pHwOVERLAYMIXER->OM_IREQ.nREG, HwGE_GE_IREQ_FLG)) // pHwOVERLAYMIXER->OM_IREQ
			ret_v |= G2D_INT_R_FLG;
	}
	return ret_v;
}



