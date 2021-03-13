/*!
 ***********************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2009 Telechips Inc.
  Luther Bldg, 7-20 Sincheon-dong, Songpa-gu, Seoul, Korea
 \endverbatim
 ***********************************************************************
 */
/*!
 ***********************************************************************
 *
 * \file
 *		TCC_HEVCDEC.h
 * \date
 *		2016/03/04
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		main api
 * \version
 *		0.01.0.00(2015/03/07) : modified hevc_dec_initial_info_t structure
 *		0.00.0.00(2015/02/15) : first beta release
 *
 ***********************************************************************
 */
#ifndef _TCC_HEVC_DEC_H_
#define _TCC_HEVC_DEC_H_

#include "TCCxxxx_VPU_CODEC_COMMON.h"

//#define USE_VPU_DISPLAY_MODE	//! use ring buffer

#define HEVC_MAX_NUM_INSTANCE		4

#define RETCODE_HEVCERR_SEQ_HEADER_NOT_FOUND	 31
#define RETCODE_HEVCERR_STRIDE_ZERO_OR_ALIGN8	100
#define RETCODE_HEVCERR_MIN_RESOLUTION			101
#define RETCODE_HEVCERR_MAX_RESOLUTION			102
#define RETCODE_HEVCERR_SEQ_INIT_HANGUP			103
#define RETCODE_HEVCERR_CHROMA_FORMAT			104
#define RETCODE_HEVCERR_PROFILE					110

#define RETCODE_WRAP_AROUND						-10

#define WAVE4_STACK_SIZE			(8*1024)
#define WAVE4_MAX_CODE_BUF_SIZE		(1024*1024)
#define WAVE4_WORKBUF_SIZE			(3*1024*1024)
#define WAVE4_TEMPBUF_SIZE			(1024*1024)

#define WAVE4_SEC_AXI_BUF_SIZE		(128*1024)

#define WAVE4_SIZE_BIT_WORK			(WAVE4_MAX_CODE_BUF_SIZE + WAVE4_STACK_SIZE + WAVE4_TEMPBUF_SIZE + WAVE4_SEC_AXI_BUF_SIZE)
#define WAVE4_WORK_CODE_BUF_SIZE	(WAVE4_SIZE_BIT_WORK + (WAVE4_WORKBUF_SIZE*HEVC_MAX_NUM_INSTANCE))

#define WAVE4_STREAM_BUF_SIZE		0x600000


//------------------------------------------------------------------------------
// decode struct and definition
//------------------------------------------------------------------------------
//! represents rectangular window information in a frame 
typedef struct wave4_pic_crop_t
{
    unsigned int m_iCropLeft;
    unsigned int m_iCropTop;
    unsigned int m_iCropRight;
    unsigned int m_iCropBottom;
} wave4_pic_crop_t;

//-----------------------------------------------------
// data structure to get information necessary to 
// start decoding from the decoder (this is an output parameter)
//-----------------------------------------------------
typedef struct hevc_dec_initial_info_t     
{
	int m_iPicWidth;				//!< {(PicX+15)/16} * 16  (this width  will be used while allocating decoder frame buffers. picWidth  is a multiple of 16)
	int m_iPicHeight;				//!< {(PicY+15)/16} * 16  (this height will be used while allocating decoder frame buffers. picHeight is a multiple of 16)
	unsigned int m_uiFrameRateRes;	//!< decoded picture frame rate residual(number of time units of a clock operating at the frequency[m_iFrameRateDiv] Hz, frameRateInfo = m_uiFrameRateRes/m_uiFrameRateDiv
	unsigned int m_uiFrameRateDiv;	//!< decoded picture frame rate unit number in Hz				
	int m_iMinFrameBufferCount;		//!< the minimum number of frame buffers that are required for decoding. application must allocate at least this number of frame buffers.
	int m_iMinFrameBufferSize;		//!< minimum frame buffer size
	int m_iFrameBufferFormat;		//!< frame buffer format

	wave4_pic_crop_t m_PicCrop;		//!< represents rectangular window information in a frame 

	int m_iFrameBufDelay;			//!< maximum display frame buffer delay to process reordering

	//! Additional Info
	int m_iProfile;					//!< profile of the decoded stream
	int m_iLevel;					//!< level of the decoded stream
	int m_iTier;					//!< 0: Main Tier, 1: High Tier for HEVC
	int m_iAspectRateInfo;			//!< aspect rate information. if this value is 0, then aspect ratio information is not present 
	int m_iReportErrorReason;		//!< reports the reason of 'RETCODE_CODEC_SPECOUT' or 'RETCODE_INVALID_STRIDE' error
	int m_iSourceFormat;			//!< source bit format
	unsigned int m_Reserved[17];	//!< Reserved.
} hevc_dec_initial_info_t;

//! data structure for initializing Video unit
typedef struct hevc_dec_init_t 
{
	codec_addr_t m_BitWorkAddr[2];		//!< physical[0] and virtual[1] address of a working space of the decoder. This working buffer space consists of work buffer, code buffer, and parameter buffer.
	codec_addr_t m_RegBaseVirtualAddr;	//!< virtual address BIT_BASE

	//! Bitstream Info
	int m_iBitstreamFormat;				//!< bitstream format
	codec_addr_t m_BitstreamBufAddr[2];	//!< bitstream buf address : multiple of 4
	int m_iBitstreamBufSize;			//!< bitstream buf size	   : multiple of 1024

	//! Decoding Options
#define WAVE4_WTL_ENABLE			(1<<0) //!< WTL Enable
#define SEC_AXI_BUS_DISABLE_SDRAM	(1<<1) //!< Disable SDRAM for sec. AXI bus
#define WAVE4_10BITS_DISABLE		(1<<3) //!< 10 to 8 bits Output Enable

	unsigned int m_uiDecOptFlags;


	//! VPU Control 
	unsigned int m_bEnableUserData;		//!< If this is set, VPU returns userdata.
	unsigned int m_bCbCrInterleaveMode;	//!< 0 (chroma separate mode   : CbCr data is written in separate frame memories)
										//!< 1 (chroma interleave mode : CbCr data is interleaved in chroma memory)
	int m_iFilePlayEnable;				//!< enable file play mode. If this value is set to 0, streaming mode with ring buffer will be used

	//! Callback Func
	void* (*m_Memcpy ) ( void*, const void*, unsigned int );	//!< memcpy
	void  (*m_Memset ) ( void*, int, unsigned int );			//!< memset
	int   (*m_Interrupt ) ( void );								//!< hw interrupt (return value is always 0)


	unsigned int m_Reserved[18];		//! Reserved.
} hevc_dec_init_t;

typedef struct hevc_dec_input_t 
{
	codec_addr_t m_BitstreamDataAddr[2];//!< bitstream data address
	int m_iBitstreamDataSize;			//!< bitstream data size
	codec_addr_t m_UserDataAddr[2];//!< Picture Layer User-data address
	int m_iUserDataBufferSize;	//!< Picture Layer User-data Size
	int m_iSkipFrameMode;				//!< Skip Frame Mode
										//!< 0 ( Skip disable )
										//!< 1 ( Skip except I(IDR) picture )
										//!< 2 ( Skip B picture : skip if nal_ref_idc==0 in H.264 )
										//!< 3 ( Unconditionally Skip one picture )
	unsigned int m_Reserved[26];		//! Reserved.
} hevc_dec_input_t;

typedef struct hevc_dec_buffer_t
{
	codec_addr_t m_FrameBufferStartAddr[2];	//!< physical[0] and virtual[1] address of a frame buffer of the decoder.
	int m_iFrameBufferCount;				//!< allocated frame buffer count
} hevc_dec_buffer_t;


typedef struct hevc_dec_buffer2_t
{
	codec_addr_t	m_addrFrameBuffer[2][32];		//!< physical[0] and virtual[1] address of a frame buffer of the decoder.
	unsigned long	m_ulFrameBufferCount;			//!< allocated frame buffer count
} hevc_dec_buffer2_t;


typedef struct hevc_dec_ring_buffer_setting_in_t
{
	codec_addr_t m_OnePacketBufferAddr;
	unsigned int m_iOnePacketBufferSize;
} hevc_dec_ring_buffer_setting_in_t;


typedef struct hevc_dec_ring_buffer_status_out_t
{
	unsigned long m_ulAvailableSpaceInRingBuffer;
	codec_addr_t m_ptrReadAddr_PA;
	codec_addr_t m_ptrWriteAddr_PA;
} hevc_dec_ring_buffer_status_out_t;


//-----------------------------------------------------
// data structure to get resulting information from 
// VPU after decoding a frame
//-----------------------------------------------------
typedef struct hevc_dec_MapConv_info_t 
{
	codec_addr_t m_CompressedY[2];
	codec_addr_t m_CompressedCb[2];

	codec_addr_t m_FbcYOffsetAddr[2];
	codec_addr_t m_FbcCOffsetAddr[2];

	unsigned int m_uiLumaStride;
	unsigned int m_uiChromaStride;

	unsigned int m_uiLumaBitDepth;
	unsigned int m_uiChromaBitDepth;

	unsigned int m_uiFrameEndian;

	unsigned int m_Reserved[19];	//! Reserved. 
} hevc_dec_MapConv_info_t;

typedef struct hevc_dec_output_info_t
{
	int m_iPicType;					//!< ( 0- I picture,  1- P picture,  2- B picture )

	int m_iDispOutIdx;				//!< index of output frame buffer
	int m_iDecodedIdx;				//!< index of decoded frame buffer

	int m_iOutputStatus;
	int m_iDecodingStatus;

	int m_iNumOfErrMBs;				//!< number of error macroblocks

	int m_iDecodedHeight;					//!< Height of input bitstream. In some cases, this value can be different from the height of previous frames.
	int m_iDecodedWidth;					//!< Width of input bitstream. In some cases, this value can be different from the height of previous frames.

	int m_iDisplayHeight;					//!< Height of input bitstream. In some cases, this value can be different from the height of previous frames.
	int m_iDisplayWidth;					//!< Width of input bitstream. In some cases, this value can be different from the height of previous frames.

	wave4_pic_crop_t m_DecodedCropInfo;	//!< Cropping information of decoded frame.
	wave4_pic_crop_t m_DisplayCropInfo;	//!< Cropping information of output frame.

	//! User Data Buffer Address
	codec_addr_t m_UserDataAddress[2];	//!< If contents have picture-layer user-data, return it.

	int m_iConsumedBytes;			//!< consumed bytes (only for file play mode, m_iFilePlayEnable=1)

	int m_iInvalidDispCount;		//!< counter of consecutive display index error
	int m_Reserved2;

	hevc_dec_MapConv_info_t m_DispMapConvInfo;	// display compressed frame info. for map converter
	hevc_dec_MapConv_info_t m_CurrMapConvInfo;	// current compressed frame info. for map converter
	hevc_dec_MapConv_info_t m_PrevMapConvInfo;	// previous compressed frame info. for map converter

	unsigned int m_Reserved[12];	//! Reserved. 

} hevc_dec_output_info_t;

typedef struct hevc_dec_output_t 
{
	hevc_dec_output_info_t m_DecOutInfo;
	unsigned char* m_pDispOut[2][3]; //! physical[0] and virtual[1] display  address of Y, Cb, Cr component
	unsigned char* m_pCurrOut[2][3]; //! physical[0] and virtual[1] current  address of Y, Cb, Cr component
	unsigned char* m_pPrevOut[2][3]; //! physical[0] and virtual[1] previous address of Y, Cb, Cr component
} hevc_dec_output_t;

/*!
 ***********************************************************************
 * \brief
 *		TCC_HEVC_DEC	: main api function of libvpudec
 * \param
 *		[in]Op			: decoder operation 
 * \param
 *		[in,out]pHandle	: libvpudec's handle
 * \param
 *		[in]pParam1		: init or input parameter
 * \param
 *		[in]pParam2		: output or info parameter
 * \return
 *		If successful, TCC_HEVC_DEC returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
codec_result_t
TCC_HEVC_DEC( int Op, codec_handle_t* pHandle, void* pParam1, void* pParam2 );

#endif//_TCC_HEVC_DEC_H_