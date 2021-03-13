/****************************************************************************
 *
 * Copyright (C) 2014 Telechips Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions
 * andlimitations under the License.
 ****************************************************************************/

#ifndef __VIOC_BLOCK_H__
#define __VIOC_BLOCK_H__


#define TVC_RDMA(x)		(x)
#define TCV_RDMA_N		(0x20)

#define TVC_WDMA(x)		(TVC_RDMA(TCV_RDMA_N) + x)
#define TCV_WDMA_N		(0x10)

#define TVC_WMIX(x)		(TVC_WDMA(TCV_WDMA_N) + x)
#define TVC_WMIX_N		(0x10)

#define TVC_SCALER(x)	(TVC_WMIX(TVC_WMIX_N) + x)
#define TVC_SCALER_N	(0x10)

#define TVC_LUT(x)		(TVC_SCALER(TVC_SCALER_N) + x)
#define TVC_LUT_N		(0x10)

#define TVC_VIN(x)		(TVC_LUT(TVC_LUT_N) + x)
#define TVC_VIN_N		(0x10)



#endif /* __VIOC_BLOCK_H__ */
