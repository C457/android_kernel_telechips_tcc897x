#ifndef	_TCC_LCD_INTERFACE_H_
#define	_TCC_LCD_INTERFACE_H_



/*
Description : RGB LCD display port setting
DD_num : Display device block number
DP_num : Display port(GPIO) number {ex  (0: L0_Lxx) or  (1 :L1_Lxx)}
bit_per_pixle : bit per pixel
*/
extern void LCDC_IO_Set (char DD_num,  char DP_num, unsigned bit_per_pixel);

/*
Description : RGB LCD display port disasble (set to normal GPIO)
DP_num : Display port(GPIO) number {ex  (0: L0_Lxx) or  (1 :L1_Lxx)}
bit_per_pixle : bit per pixel
*/
extern void LCDC_IO_Disable (char DP_num, unsigned bit_per_pixel);
#endif //_TCC_LCD_INTERFACE_H_

