/*!
 ******************************************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2009-2012 Telechips Inc.
  Luther Bldg, 7-20 Sincheon-dong, Songpa-gu, Seoul, Korea
 \endverbatim
 ******************************************************************************************
 */
/*!
 ******************************************************************************************
 *
 * \file
 *		TCC_VP9DEC.h
 * \date
 *		2015/12/01
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		main api
 * \version
 *		0.0.0(2015/12/01) : first beta release
 *
 ******************************************************************************************
 */
#ifndef _TCC_VP9DEC_H_
#define _TCC_VP9DEC_H_

#include "TCCxxxx_VPU_CODEC_COMMON.h"

#define VP9_MAX_NUM_INSTANCE		4

#define RETCODE_VP9ERR_SEQ_HEADER_NOT_FOUND		 31
#define RETCODE_VP9ERR_MIN_RESOLUTION			101
#define RETCODE_VP9ERR_MAX_RESOLUTION			102
#define RETCODE_VP9ERR_BITDEPTH					103
#define RETCODE_VP9ERR_RGB_FORMAT				104
#define RETCODE_VP9ERR_PROFILE					110
//#define RETCODE_VP9ERR_VERSION					111


#define	VP9_LARGE_STREAM_BUF_SIZE	0x200000
#define VP9_ENTROPY_SAVE_SIZE		0x005000

//-----------------------------------------------------------------------------
// decode struct and definition
//-----------------------------------------------------------------------------
typedef struct vp9_pic_crop_t
{
	unsigned int m_iCropLeft;
	unsigned int m_iCropTop;
	unsigned int m_iCropRight;
	unsigned int m_iCropBottom;
} vp9_pic_crop_t;

typedef struct dscale_info_t
{
	int m_iDscaleHeight;					//!< Height of down scaled output picture
	int m_iDscaleWidth;						//!< Width of down scaled output picture
	int m_iDscaleStride;					//!< Stride of down scaled output picture
	unsigned char* m_pDscaleDispOut[2][3];	//!< physical[0] and virtual[1] display  address of Y, Cb, Cr component
	unsigned char* m_pDscaleCurrOut[2][3];	//!< physical[0] and virtual[1] current  address of Y, Cb, Cr component
	unsigned int m_Reserved[17];			//!< Reserved.
} dscale_info_t;

//-----------------------------------------------------------------------------
// data structure to get information necessary to
// start decoding from the decoder (this is an output parameter)
//-----------------------------------------------------------------------------
typedef struct vp9_dec_initial_info_t
{
	int m_iPicWidth;				//!< {(PicX+15)/16} * 16  (this width  will be used while allocating decoder frame buffers. picWidth  is a multiple of 16)
	int m_iPicHeight;				//!< {(PicY+15)/16} * 16  (this height will be used while allocating decoder frame buffers. picHeight is a multiple of 16)
	int m_iMinFrameBufferCount;		//!< the minimum number of frame buffers that are required for decoding. application must allocate at least this number of frame buffers.
	int m_iMinFrameBufferSize;		//!< minimum frame buffer size
	int m_iTempBufferSize;			//!< temp buffer size for segment map & loop filter

	vp9_pic_crop_t m_iVp9PicCrop;		//!< represents rectangular window information in a frame

	//!< Additional Info
	int m_iVersion;					//!< profile of the decoded stream
	int m_iProfile;					//!< level of the decoded stream
	int m_iBitDepth;				//!< when this value is 1, decoded stream will be decoded into both progressive or interlace frame. otherwise, all the frames will be progressive. In H.264, this is frame_mbs_only_flag.
	int m_iReportErrorReason;		//!< reports the reason of 'RETCODE_CODEC_SPECOUT' or 'RETCODE_INVALID_STRIDE' error
	unsigned int m_Reserved[22];	//!< Reserved.
} vp9_dec_initial_info_t;

//!< data structure for initializing Video unit
typedef struct vp9_dec_init_t
{
	codec_addr_t m_RegBaseVirtualAddr;	//!< virtual register base address

	int m_iBitstreamFormat;				//!< bitstream format
	codec_addr_t m_BitstreamBufAddr[2];	//!< bitstream buf address : 128 align
	int m_iBitstreamBufSize;			//!< bitstream buf size	   : multiple of 1024

	//!< Decoding Options
#define VP9_TILED_ENABLE				(1<<0)	//!< TILED Enable
#define VP9_REF_COMPRESS_BYPASS_ENABLE	(1<<1)	//!< Ref. Compress Bypass
#define VP9_10BITS_ENABLE				(1<<2) //!< 10 bits Output Enable
	unsigned int m_uiDecOptFlags;

	codec_addr_t m_EntropySaveBuffer[2];
	int m_iEntropySaveBufferSize;

	//!< VP9 Control
	int m_iFilePlayEnable;				//!< enable file play mode. If this value is set to 0, streaming mode with ring buffer will be used

	int m_iMaxResolution;				//!< maximum resolution limitation option

	//!< Callback Func
	void* (*m_Memcpy ) ( void*, const void*, unsigned int );	//!< memcpy
	void  (*m_Memset ) ( void*, int, unsigned int );			//!< memset
	int   (*m_Interrupt ) ( void );								//!< hw interrupt (return value is always 0)

	unsigned int m_Reserved[18];	//! Reserved.
} vp9_dec_init_t;

typedef struct vp9_dec_input_t
{
	codec_addr_t m_BitstreamDataAddr[2];	//!< bitstream data address
	int m_iBitstreamDataSize;				//!< bitstream data size

	int m_iFrameSearchEnable;				//!< I-frame Search Mode
											//!< If this option is enabled, then decoder performs skipping frame decoding until decoder meet I-frame.
											//!< 0x000 ( Disable )
											//!< 0x001 ( Enable  )

	unsigned int m_Reserved[12];			//!< Reserved.
} vp9_dec_input_t;

typedef struct vp9_dec_buffer_t
{
	codec_addr_t m_FrameBufferStartAddr[2];	//!< physical[0] and virtual[1] address of a frame buffer of the decoder.
	int m_iFrameBufferCount;				//!< allocated frame buffer count

	codec_addr_t m_TempBufferAddr[2];		//!< physical[0] and virtual[1] address of a temp buffer (segment & loopfilter)
	int m_iTempBufferSize;					//!< size of temp buffer

	unsigned int m_Reserved[10];			//!< Reserved.
} vp9_dec_buffer_t;

typedef struct vp9_dec_ring_buffer_setting_in_t
{
	codec_addr_t m_OnePacketBufferAddr;
	unsigned int m_iOnePacketBufferSize;
} vp9_dec_ring_buffer_setting_in_t;

typedef struct vp9_dec_ring_buffer_status_out_t
{
	unsigned int m_iAvailableSpaceInRingBuffer;
} vp9_dec_ring_buffer_status_out_t;

//-----------------------------------------------------------------------------
// data structure to get resulting information from VP9 after decoding a frame
//-----------------------------------------------------------------------------
typedef struct vp9_dec_output_info_t
{
	int m_iPicType;					//!< ( 0- I picture,  1- P picture,  2- B picture )

	int m_iDispOutIdx;				//!< index of output frame buffer
	int m_iDecodedIdx;				//!< index of decoded frame buffer

	int m_iOutputStatus;
	int m_iDecodingStatus;

	int m_iHeight;					//!< Height of input bitstream. In some cases, this value can be different from the height of previous frames.
	int m_iWidth;					//!< Width of input bitstream. In some cases, this value can be different from the height of previous frames.

	int m_iBitDepthLuma;				//!< Luma Bit depth.
	int m_iBitDepthChroma;				//!< Chroma Bit depth.
	int m_iPicStride;				//!< Picture Stride.

	codec_addr_t m_CompressionTableLuma[2];
	codec_addr_t m_CompressionTableChroma[2];

	pic_crop_t m_CropInfo;			//!< Cropping information.

	dscale_info_t m_DscaleInfo;		//!< Down Scale information.

	unsigned int m_Reserved[17];	//!< Reserved.
} vp9_dec_output_info_t;

typedef struct vp9_dec_output_t
{
	vp9_dec_output_info_t m_DecOutInfo;
	unsigned char* m_pDispOut[2][3];	//!< physical[0] and virtual[1] display  address of Y, Cb, Cr component
	unsigned char* m_pCurrOut[2][3];	//!< physical[0] and virtual[1] current  address of Y, Cb, Cr component
	unsigned int   m_Reserved[19];		//!< Reserved.
} vp9_dec_output_t;

/*!
 ******************************************************************************************
 * \brief
 *		TCC_VP9_DEC		: main api function of libvp9dec
 * \param
 *		[in]Op			: decoder operation
 * \param
 *		[in,out]pHandle	: libvp9dec's handle
 * \param
 *		[in]pParam1		: init or input parameter
 * \param
 *		[in]pParam2		: output or info parameter
 * \return
 *		If successful, TCC_VP9_DEC returns 0 or plus. Otherwise, it returns a minus value.
 ******************************************************************************************
 */
codec_result_t
TCC_VP9_DEC( int Op, codec_handle_t* pHandle, void* pParam1, void* pParam2 );

#endif	//_TCC_VP9DEC_H_

