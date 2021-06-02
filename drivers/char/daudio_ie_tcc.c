#include <linux/platform_device.h>

#include <mach/daudio.h>
#include <mach/daudio_debug.h>
#include <mach/daudio_ie.h>
#include <mach/io.h>
#include <mach/tcc_lut.h>
#include <mach/tcc_lut_ioctl.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_blk.h>

#include "daudio_ie_tcc.h"

CLOG_INIT(CLOG_TAG_DAUDIO, CLOG_TAG_IE_TCC, CLOG_LEVEL_IE_TCC);

#define DEBUG_IE_LEVEL	get_ie_debug_level()
#define sCLogLevel		(CLOG_LEVEL_IE_TCC <= DEBUG_IE_LEVEL ? DEBUG_IE_LEVEL : CLOG_LEVEL_IE_TCC)

#define MULTI_NUM 1000
#define SATURATION_RANGE   	4 // -X ~ +X
#define CONTRAST_RANGE   	4 // -X ~ +X
static int rgb[257] = {0, };

static int _brightness = -1;
static int _contrast = -1;
static int _hue = -1;
static int _saturation = -1;

static int _video_brightness = -1;
static int _video_contrast = -1;
static int _video_gamma = -1;
static int _video_saturation = -1;

static int _tcc_brightness = 127;
static int _tcc_contrast = 127;
static int _tcc_gamma = 127;

static int set_tcc_disp_ie(void)
{
	VIOC_DISP *pDISP = (VIOC_DISP *)tcc_p2v(HwVIOC_DISP1);
	char contrast = 0;
	char brightness = 0;
	char hue = 0;

	if (_brightness == -1)	_brightness = 127;
	if (_contrast == -1)	_contrast = 127 + 0x20;
	if (_hue == -1)			_hue = 127;

	contrast = (char)(_contrast - 127);
	brightness = (char)(_brightness - 127);
	hue = 0;	//Not support
	CLOG(CLL_CRITICAL,"%s : contrast = %d, brightness =%d , hue =%d \n ", __func__ , contrast, brightness, hue );
	VIOC_DISP_SetColorEnhancement(pDISP, contrast, brightness, hue);

	return SUCCESS;
}
static int set_tcc_video(int _tcc_brightness, int _tcc_contrast, int _tcc_gamma)
{
	int ret = FAIL;
	int vR, vG, vB, i;
	VIOC_LUT *pLUT =(VIOC_LUT*)tcc_p2v(HwVIOC_LUT);

	for (i = 0; i < 256; i++)
	{
		vR = vG = vB = (i + ((_tcc_brightness - 128) * 2));
		vR = vG = vB = ((vR - 128) * (3 + _tcc_contrast * 2)) / 255 + 128;
		//TODO: gamma

		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(vG > 255)    vG = 255;
		if(vG < 0)      vG = 0;
		if(vB > 255)    vB = 255;
		if(vB < 0)      vB = 0;

		rgb[i] = ((vR & 0xff) << 16) | ((vG & 0xff) << 8) | ((vB & 0xff));

		CLOG(CLL_TRACE, "%s count: %d rgb:0x%2x\n", __func__, i, rgb[i]);
	}
	CLOG(CLL_CRITICAL, "%s : brightness=%d , contrast=%d, gamma =%d \n ", __func__, _tcc_brightness, _tcc_contrast, _tcc_gamma );
	
#if defined (CONFIG_VIOC_893X_LUT)
	VIOC_LUT_Set_value(pLUT, LUT_COMP2, rgb);
#else
	tcc_set_lut_table(TVC_LUT(LUT_COMP2), rgb);
#endif

	ret = SUCCESS;

	return ret;
}
static int set_tcc_video_ie(void)
{
	int ret = FAIL;
	int vR, vG, vB, i;
	int fsaturation, fcontrast;
	VIOC_LUT *pLUT =(VIOC_LUT*)tcc_p2v(HwVIOC_LUT);

	CLOG(CLL_CRITICAL, "%s\n", __func__);
	
	if (_video_brightness < 0 || _video_brightness > 255)
		_video_brightness = 127;
	if (_video_contrast < 0 || _video_contrast > 255)
		_video_contrast = 127;
	if (_video_saturation < 0 || _video_saturation > 255)
		_video_saturation = 127;

	for (i = 0; i < 256; i++)
	{
		//contrast brightness
		if(_video_contrast > 127)
			fcontrast = (1 * MULTI_NUM) + ((_video_contrast -127) * MULTI_NUM * (CONTRAST_RANGE - 1)) / 127;
		else
			fcontrast = (_video_contrast * MULTI_NUM) / 127;

		vR = ((i - 16)*fcontrast)/MULTI_NUM + 16;

		//brightness
		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(_video_brightness >= 127)
			vR = vR + ((_video_brightness - 127) * 2);
		else
			vR = vR + ((_video_brightness - 127) * 2) - 1;

		//saturation
		if(_video_saturation > 127)
			fsaturation = (1 * MULTI_NUM) + ((_video_saturation - 127) * MULTI_NUM * (SATURATION_RANGE - 1)) / 127;
		else
			fsaturation = (_video_saturation * MULTI_NUM) / 127;

		//fsaturation = (1* MULTI_NUM)+ ((((_video_saturation - 127) * MULTI_NUM) / 127) * SATURATION_RANGE);

		vG = vB = (((i - 128)*fsaturation * fcontrast)/MULTI_NUM)/MULTI_NUM + 128;

		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(vG > 255)    vG = 255;
		if(vG < 0)      vG = 0;
		if(vB > 255)    vB = 255;
		if(vB < 0)      vB = 0;

		rgb[i] =((vR & 0xff) << 16) | ((vG & 0xff) << 8) | ((vB & 0xff));

		CLOG(CLL_TRACE, "%s count: %d rgb:0x%2x\n", __func__, i, rgb[i]);
	}
	CLOG(CLL_TRACE, "%s _video_brightness:%d  _video_contrast:%d  _video_saturation:%d \n", 
						__func__, _video_brightness, _video_contrast, _video_saturation);
						
#if defined (CONFIG_VIOC_893X_LUT)
	VIOC_LUT_Set_value(pLUT, LUT_COMP1, rgb);
#else
	tcc_set_lut_table(TVC_LUT(LUT_COMP1), rgb);
	tcc_set_lut_table(TVC_LUT(LUT_COMP0), rgb);
#endif

	ret = SUCCESS;
	CLOG(CLL_CRITICAL, "%s,ret = %d \n", __func__ ,ret  );
	return ret;
}

//GT start
static int set_tcc_video_ie_LUT3(void)
{

	int ret = FAIL;
	int vR, vG, vB, i;
	int fsaturation, fcontrast;
	VIOC_LUT *pLUT =(VIOC_LUT*)tcc_p2v(HwVIOC_LUT);

	if(1)//!(pLUT->uCOMP3CFG.bREG.EN))
	{
		CLOG(CLL_CRITICAL, "%s : enable\n", __func__  );
		
#if defined (CONFIG_VIOC_893X_LUT)
		VIOC_LUT_Plugin(pLUT, VIOC_LUT_COMP3, VIOC_SC_RDMA_12);
		VIOC_LUT_Enable (pLUT, VIOC_LUT_COMP3, 1);
#else
		tcc_set_lut_plugin(TVC_LUT(LUT_COMP0), TVC_RDMA(13));
		tcc_set_lut_enable(TVC_LUT(LUT_COMP0), true);		
#endif	

	}

	CLOG(CLL_CRITICAL, "%s\n", __func__);
	
	if (_video_brightness < 0 || _video_brightness > 255)
		_video_brightness = 127;
	if (_video_contrast < 0 || _video_contrast > 255)
		_video_contrast = 127;
	if (_video_saturation < 0 || _video_saturation > 255)
		_video_saturation = 127;

	for (i = 0; i < 256; i++)
	{
		//contrast brightness
		if(_video_contrast > 127)
			fcontrast = (1 * MULTI_NUM) + ((_video_contrast -127) * MULTI_NUM * (CONTRAST_RANGE - 1)) / 127;
		else
			fcontrast = (_video_contrast * MULTI_NUM) / 127;

		vR = ((i - 16)*fcontrast)/MULTI_NUM + 16;

		//brightness
		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(_video_brightness >= 127)
			vR = vR + ((_video_brightness - 127) * 2);
		else
			vR = vR + ((_video_brightness - 127) * 2) - 1;

		//saturation
		if(_video_saturation > 127)
			fsaturation = (1 * MULTI_NUM) + ((_video_saturation - 127) * MULTI_NUM * (SATURATION_RANGE - 1)) / 127;
		else
			fsaturation = (_video_saturation * MULTI_NUM) / 127;

		//fsaturation = (1* MULTI_NUM)+ ((((_video_saturation - 127) * MULTI_NUM) / 127) * SATURATION_RANGE);

		vG = vB = (((i - 128)*fsaturation * fcontrast)/MULTI_NUM)/MULTI_NUM + 128;

		if(vR > 255)    vR = 255;
		if(vR < 0)      vR = 0;
		if(vG > 255)    vG = 255;
		if(vG < 0)      vG = 0;
		if(vB > 255)    vB = 255;
		if(vB < 0)      vB = 0;

		rgb[i] =((vR & 0xff) << 16) | ((vG & 0xff) << 8) | ((vB & 0xff));

		CLOG(CLL_TRACE, "%s count: %d rgb:0x%2x\n", __func__, i, rgb[i]);
	}
	CLOG(CLL_TRACE, "%s _video_brightness:%d  _video_contrast:%d  _video_saturation:%d \n", 
						__func__, _video_brightness, _video_contrast, _video_saturation);

#if defined (CONFIG_VIOC_893X_LUT)
	VIOC_LUT_Set_value(pLUT, LUT_COMP3, rgb);
#else
	tcc_set_lut_table(TVC_LUT(LUT_COMP0), rgb);
#endif

	ret = SUCCESS;
	CLOG(CLL_CRITICAL,"%s,ret = %d\n ", __func__ ,ret  );
	return ret;


}
//GT end

/**
 * @return 1 is argument invalid.
 */
int is_tcc_ie_invalid_arg(int cmd, char value)
{
	return value < 1 || value > 255 ? 1 : 0;
}

int get_tcc_ie(int mode, unsigned char *level)
{

	switch (mode)
	{
		case GET_TCC_BRIGHTNESS:
			*level = _brightness; 
			break;
		case GET_TCC_CONTRAST:
			*level = _contrast;
			break;
		case GET_TCC_HUE:			
			*level = _hue; break;
		case GET_TCC_SATURATION:
			*level = _saturation;
			break;			
 		case GET_TCC_DMB_BRIGHTNESS : //GT system
		case GET_TCC_CMMB_BRIGHTNESS :
		case GET_TCC_USB_BRIGHTNESS :
		case GET_TCC_ISDB_BRIGHTNESS :
			*level = _video_brightness;
			break;
 		case GET_TCC_DMB_CONTRAST: //GT system
		case GET_TCC_CMMB_CONTRAST:
		case GET_TCC_USB_CONTRAST:
		case GET_TCC_ISDB_CONTRAST:

			*level = _video_contrast;
			break;
		case GET_TCC_DMB_GAMMA: //GT system
		case GET_TCC_CMMB_GAMMA:
		case GET_TCC_USB_GAMMA:
		case GET_TCC_ISDB_GAMMA:
 			*level = _video_gamma;
			break;
		case GET_TCC_DMB_SATURATION: //GT system
		case GET_TCC_CMMB_SATURATION:
		case GET_TCC_USB_SATURATION: 			
		case GET_TCC_ISDB_SATURATION: 			
			*level = _video_saturation;
			break;

		default:
			return FAIL;

	}

	return SUCCESS;
}

int set_tcc_ie(int mode, unsigned char level)
{
	int ret = SUCCESS;
	CLOG(CLL_INFO, "%s mode:%d new level:%d\n", __func__, mode, (int)level);

	switch (mode)
	{
		case SET_TCC_BRIGHTNESS:
			if (_brightness != level)
			{
				_brightness = level;
				ret = set_tcc_disp_ie();
			}
			break;

		case SET_TCC_CONTRAST:
			if (_contrast != level)
			{
				_contrast = level;
				ret = set_tcc_disp_ie();
			}
			break;

#if 0	//Not support
		case SET_TCC_HUE:
		case SET_TCC_SATURATION:
			break;
#endif
		
		case SET_TCC_VIDEO_BRIGHTNESS2:
		//case SET_TCC_CMMB_BRIGHTNESS: 
		case SET_TCC_USB_BRIGHTNESS:  
		case SET_TCC_ISDB_BRIGHTNESS:  
			if (_video_brightness != level)
			{
				_video_brightness = level;
				ret = set_tcc_video_ie();
			}
			break;
			case SET_TCC_VIDEO_BRIGHTNESS: 
			case SET_TCC_DMB_BRIGHTNESS: // GT system
			if (_video_brightness != level)
			{
				_video_brightness = level;
				ret = set_tcc_video_ie_LUT3();
			}
			break;

		
		case SET_TCC_VIDEO_CONTRAST2:
		//case SET_TCC_CMMB_CONTRAST:
		case SET_TCC_USB_CONTRAST:
		case SET_TCC_ISDB_CONTRAST:
			if (_video_contrast != level)
			{
				_video_contrast = level;
				ret = set_tcc_video_ie();
			}
			break;
			case SET_TCC_VIDEO_CONTRAST:
			case SET_TCC_DMB_CONTRAST: // GT system
			if (_video_contrast != level)
			{
				_video_contrast = level;
				ret = set_tcc_video_ie_LUT3();
			}
			break;

		
		case SET_TCC_VIDEO_GAMMA2:
		//case SET_TCC_CMMB_GAMMA:
		case SET_TCC_USB_GAMMA:
		case SET_TCC_ISDB_GAMMA:
			if (_video_gamma != level)
			{
				_video_gamma = level;
				ret = set_tcc_video_ie();
			}
			break;
			case SET_TCC_VIDEO_GAMMA:
			case SET_TCC_DMB_GAMMA: // GT system
			if (_video_gamma != level)
			{
				_video_gamma = level;
				ret = set_tcc_video_ie_LUT3();
			}
			break;
		
		case SET_TCC_VIDEO_SATURATION2:
		//case SET_TCC_CMMB_SATURATION:
		case SET_TCC_USB_SATURATION:
		case SET_TCC_ISDB_SATURATION:
			if (_video_saturation != level)
			{
				_video_saturation = level;
				ret = set_tcc_video_ie();
			}
			break;
			case SET_TCC_VIDEO_SATURATION:
			case SET_TCC_DMB_SATURATION:
			if (_video_saturation != level)
			{
				_video_saturation = level;
				ret = set_tcc_video_ie_LUT3();
			}
			break;


		default:
			ret = FAIL;
			break;
	}

	return ret;
}

void init_tcc_video_ie(unsigned char brightness, unsigned char contrast,
		unsigned char gamma, unsigned char saturation)
{
	if (brightness == 0)	brightness = 127;
	if (contrast == 0)		contrast = 127;
	if (gamma == 0)			gamma = 127;
	if (saturation == 0)	saturation = 127;

	CLOG(CLL_CRITICAL, "%s : \n ", __func__ );
	if (brightness != _video_brightness ||
			contrast != _video_contrast ||
			gamma != _video_gamma ||
			saturation != _video_saturation)
	{
		_video_brightness = brightness;
		_video_contrast = contrast;
		_video_gamma = gamma;
		_video_saturation = saturation;
		set_tcc_video(_tcc_brightness, _tcc_contrast, _tcc_gamma);
		set_tcc_video_ie();
		set_tcc_video_ie_LUT3();
	}
}

void init_tcc_ie(unsigned char brightness, unsigned char contrast,
		unsigned char hue, unsigned char saturation)
{
	if (brightness == 0)	brightness = 127;
	if (contrast == 0)		contrast = 127 + 0x20;
	if (hue == 0)			hue = 127;
	if (saturation == 0)	saturation = 127;

	if (brightness != _brightness ||
			contrast != _contrast ||
			hue != _hue ||
			saturation != _saturation)
	{
		_brightness = brightness;
		_contrast = contrast;
		_hue = hue;
		_saturation = saturation;
		set_tcc_disp_ie();
	}
}
