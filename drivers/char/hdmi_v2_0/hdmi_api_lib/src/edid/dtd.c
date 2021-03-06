/*
 * @file:dtd.c
 *
 *
 * Synopsys Inc.
 * SG DWC PT02
 */


#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"
#include "../../../include/hdmi_ioctls.h"


//#include "edid/dtd.h"
//#include "util/log.h"
//#include "util/bit_operation.h"
//#include "util/error.h"
//#include "general_ops.h"
typedef struct supported_dtd{
        u32 refresh_rate;
        dtd_t dtd;
}supported_dtd_t;


static supported_dtd_t _dtd[] = {
        /**
                    mCode       mPixelClock (Hz *1000)     mHImageSize            mVBlanking        mVSyncPolarity
                    |  mLimitedToYcc420 mInterlaced        |   mHSyncOffset       |   mVBorder      |
                    |  |  mYcc420       |   mHactive       |   |   mHSyncPulseWidth   |  mVImageSize|
                    |  |  |  mPixelRepetitionInput    mHBorder |   |     mHSyncPolarity  |  mVSyncOffset
                    |  |  |  |  |       |   |     mHBlanking   |   |     |   mVActive |  |  |    mVSyncPulseWidth
                                                                                                                  */      
        {60000 , {  1, 0, 0, 0, 25200 , 0,  640,  160, 0,  4,   16,  96, 0,  480, 45, 0, 3, 10,  2, 0}},
        {59940 , {  1, 0, 0, 0, 25175 , 0,  640,  160, 0,  4,   16,  96, 0,  480, 45, 0, 3, 10,  2, 0}},
        {60000 , {  2, 0, 0, 0, 27027 , 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0}},
        {59940 , {  2, 0, 0, 0, 27000 , 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0}},
        {60000 , {  3, 0, 0, 0, 27027 , 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {59940 , {  3, 0, 0, 0, 27000 , 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {60000 , {  4, 0, 0, 0, 74250 , 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {59940 , {  4, 0, 0, 0, 74176 , 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {60000 , {  5, 0, 0, 0, 74250 , 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {59940 , {  5, 0, 0, 0, 74176 , 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {60000 , {  6, 0, 0, 1, 27027 , 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {59940 , {  6, 0, 0, 1, 27000 , 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {60000 , {  7, 0, 0, 1, 27027 , 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {59940 , {  7, 0, 0, 1, 27000 , 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {60000 , {  8, 0, 0, 1, 27027 , 0, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {60054 , {  8, 0, 0, 1, 27000 , 0, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {59826 , {  8, 0, 0, 1, 27000 , 0, 1440,  276, 0,  4,   38, 124, 0,  240, 23, 0, 3,  5,  3, 0}},
        {60000 , {  9, 0, 0, 1, 27027 , 0, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {60054 , {  9, 0, 0, 1, 27000 , 0, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {59826 , {  9, 0, 0, 1, 27000 , 0, 1440,  276, 0, 16,   38, 124, 0,  240, 23, 0, 9,  5,  3, 0}},
        {60000 , { 10, 0, 0, 0, 54054 , 1, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0}},
        {59940 , { 11, 0, 0, 0, 54000 , 1, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0}},
        {60000 , { 12, 0, 0, 0, 54054 , 0, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0}},
        {60054 , { 12, 0, 0, 0, 54000 , 0, 2880,  552, 0,  4,   76, 248, 0,  240, 22, 0, 3,  4,  3, 0}},
        {59826 , { 12, 0, 0, 0, 54000 , 0, 2880,  552, 0,  4,   76, 248, 0,  240, 23, 0, 3,  5,  3, 0}},
        {60000 , { 13, 0, 0, 0, 54054 , 0, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0}},
        {60054 , { 13, 0, 0, 0, 54000 , 0, 2880,  552, 0, 16,   76, 248, 0,  240, 22, 0, 9,  4,  3, 0}},
        {59826 , { 13, 0, 0, 0, 54000 , 0, 2880,  552, 0, 16,   76, 248, 0,  240, 23, 0, 9,  5,  3, 0}},
        {60000 , { 14, 0, 0, 0, 54054 , 0, 1440,  276, 0,  4,   32, 124, 0,  480, 45, 0, 3,  9,  6, 0}},
        {59940 , { 14, 0, 0, 0, 54000 , 0, 1440,  276, 0,  4,   32, 124, 0,  480, 45, 0, 3,  9,  6, 0}},
        {60000 , { 15, 0, 0, 0, 54054 , 0, 1440,  276, 0, 16,   32, 124, 0,  480, 45, 0, 9,  9,  6, 0}},
        {59940 , { 15, 0, 0, 0, 54000 , 0, 1440,  276, 0, 16,   32, 124, 0,  480, 45, 0, 9,  9,  6, 0}},
        {60000 , { 16, 0, 0, 0, 148500, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {59940 , { 16, 0, 0, 0, 148352, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {50000 , { 17, 0, 0, 0, 27000 , 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0}},
        {50000 , { 18, 0, 0, 0, 27000 , 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0}},
        {50000 , { 19, 0, 0, 0, 74250 , 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {50000 , { 20, 0, 0, 0, 74250 , 1, 1920,  720, 0, 16,  528,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {50000 , { 21, 0, 0, 1, 27000 , 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0}},
        {50000 , { 22, 0, 0, 1, 27000 , 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0}},
        {50000 , { 23, 0, 0, 1, 27000 , 0, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0}},
        {50000 , { 23, 0, 0, 1, 27000 , 0, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0}},
        {49920 , { 23, 0, 0, 1, 27000 , 0, 1440,  288, 0,  4,   24, 126, 0,  288, 25, 0, 3,  3,  3, 0}},
        {50000 , { 24, 0, 0, 1, 27000 , 0, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0}},
        {50000 , { 24, 0, 0, 1, 27000 , 0, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0}},
        {49920 , { 24, 0, 0, 1, 27000 , 0, 1440,  288, 0, 16,   24, 126, 0,  288, 25, 0, 9,  3,  3, 0}},
        {50000 , { 25, 0, 0, 0, 54000 , 1, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0}},
        {50000 , { 26, 0, 0, 0, 54000 , 1, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0}},
        {50000 , { 27, 0, 0, 0, 54000 , 0, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0}},
        {49920 , { 27, 0, 0, 0, 54000 , 0, 2880,  576, 0,  4,   48, 252, 0,  288, 25, 0, 3,  3,  3, 0}},
        {50000 , { 27, 0, 0, 0, 54000 , 0, 2880,  576, 0,  4,   48, 252, 0,  288, 24, 0, 3,  2,  3, 0}},
        {50000 , { 28, 0, 0, 0, 54000 , 0, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0}},
        {49920 , { 28, 0, 0, 0, 54000 , 0, 2880,  576, 0, 16,   48, 252, 0,  288, 25, 0, 9,  3,  3, 0}},
        {50000 , { 28, 0, 0, 0, 54000 , 0, 2880,  576, 0, 16,   48, 252, 0,  288, 24, 0, 9,  2,  3, 0}},
        {50000 , { 29, 0, 0, 0, 54000 , 0, 1440,  288, 0,  4,   24, 128, 0,  576, 49, 0, 3,  5,  5, 0}},
        {50000 , { 30, 0, 0, 0, 54000 , 0, 1440,  288, 0, 16,   24, 128, 0,  576, 49, 0, 9,  5,  5, 0}},
        {50000 , { 31, 0, 0, 0, 148500, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {24000 , { 32, 0, 0, 0, 74250 , 0, 1920,  830, 0, 16,  638,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {23976 , { 32, 0, 0, 0, 74176 , 0, 1920,  830, 0, 16,  638,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {25000 , { 33, 0, 0, 0, 74250 , 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {30000 , { 34, 0, 0, 0, 74250 , 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {29970 , { 34, 0, 0, 0, 74176 , 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {60000 , { 35, 0, 0, 0, 108108, 0, 2880,  552, 0,  4,   64, 248, 0,  480, 45, 0, 3,  9,  6, 0}},
        {59940 , { 35, 0, 0, 0, 108000, 0, 2880,  552, 0,  4,   64, 248, 0,  480, 45, 0, 3,  9,  6, 0}},
        {60000 , { 36, 0, 0, 0, 108108, 0, 2880,  552, 0, 16,   64, 248, 0,  480, 45, 0, 9,  9,  6, 0}},
        {59940 , { 36, 0, 0, 0, 108100, 0, 2880,  552, 0, 16,   64, 248, 0,  480, 45, 0, 9,  9,  6, 0}},
        {50000 , { 37, 0, 0, 0, 108000, 0, 2880,  576, 0,  4,   48, 256, 0,  576, 49, 0, 3,  5,  5, 0}},
        {50000 , { 38, 0, 0, 0, 108000, 0, 2880,  576, 0, 16,   48, 256, 0,  576, 49, 0, 9,  5,  5, 0}},
        {50000 , { 39, 0, 0, 0, 72000 , 1, 1920,  384, 0, 16,   32, 168, 1,  540, 85, 0, 9, 23,  5, 0}},
        {100000, { 40, 0, 0, 0, 148500, 1, 1920,  720, 0, 16,  528,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {100000, { 41, 0, 0, 0, 148500, 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {100000, { 42, 0, 0, 0, 54000 , 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0}},
        {100000, { 43, 0, 0, 0, 54000 , 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0}},
        {100000, { 44, 0, 0, 1, 54000 , 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0}},
        {100000, { 45, 0, 0, 1, 54000 , 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0}},
        {120000, { 46, 0, 0, 0, 148500, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {119880, { 46, 0, 0, 0, 148352, 1, 1920,  280, 0, 16,   88,  44, 1,  540, 22, 0, 9,  2,  5, 1}},
        {120000, { 47, 0, 0, 0, 148500, 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {120000, { 48, 0, 0, 0, 54054 , 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0}},
        {120000, { 49, 0, 0, 0, 54054 , 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {119880, { 49, 0, 0, 0, 54000 , 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {120000, { 50, 0, 0, 1, 54054 , 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {119880, { 50, 0, 0, 1, 54000 , 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {120000, { 51, 0, 0, 1, 54054 , 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {119880, { 51, 0, 0, 1, 54000 , 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {200000, { 52, 0, 0, 0, 108000, 0,  720,  144, 0,  4,   12,  64, 0,  576, 49, 0, 3,  5,  5, 0}},
        {200000, { 53, 0, 0, 0, 108000, 0,  720,  144, 0, 16,   12,  64, 0,  576, 49, 0, 9,  5,  5, 0}},
        {200000, { 54, 0, 0, 1, 108000, 1, 1440,  288, 0,  4,   24, 126, 0,  288, 24, 0, 3,  2,  3, 0}},
        {200000, { 55, 0, 0, 1, 108000, 1, 1440,  288, 0, 16,   24, 126, 0,  288, 24, 0, 9,  2,  3, 0}},
        {240000, { 56, 0, 0, 0, 108100, 0,  720,  138, 0,  4,   16,  62, 0,  480, 45, 0, 3,  9,  6, 0}},
        {240000, { 57, 0, 0, 0, 108100, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {239760, { 57, 0, 0, 0, 108000, 0,  720,  138, 0, 16,   16,  62, 0,  480, 45, 0, 9,  9,  6, 0}},
        {240000, { 58, 0, 0, 1, 108100, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {239760, { 58, 0, 0, 1, 108000, 1, 1440,  276, 0,  4,   38, 124, 0,  240, 22, 0, 3,  4,  3, 0}},
        {240000, { 59, 0, 0, 1, 108100, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {239760, { 59, 0, 0, 1, 108000, 1, 1440,  276, 0, 16,   38, 124, 0,  240, 22, 0, 9,  4,  3, 0}},
        {24000 , { 60, 0, 0, 0, 59400 , 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {23970 , { 60, 0, 0, 0, 59341 , 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {25000 , { 61, 0, 0, 0, 74250 , 0, 1280, 2680, 0, 16, 2420,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {30000 , { 62, 0, 0, 0, 74250 , 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {29970 , { 62, 0, 0, 0, 74176 , 0, 1280, 2020, 0, 16, 1760,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {120000, { 63, 0, 0, 0, 297000, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {119880, { 63, 0, 0, 0, 296703, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {100000, { 64, 0, 0, 0, 297000, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {50000 , { 68, 0, 0, 0, 74250 , 0, 1280,  700, 0, 16,  440,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {60000 , { 69, 0, 0, 0, 74250 , 0, 1280,  370, 0, 16,  110,  40, 1,  720, 30, 0, 9,  5,  5, 1}},
        {50000 , { 75, 0, 0, 0, 148500, 0, 1920,  720, 0, 16,  528,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {60000 , { 76, 0, 0, 0, 148500, 0, 1920,  280, 0, 16,   88,  44, 1, 1080, 45, 0, 9,  4,  5, 1}},
        {24000 , { 93, 0, 0, 0, 297000, 0, 3840, 1660, 0, 16, 1276,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {25000 , { 94, 0, 0, 0, 297000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {30000 , { 95, 0, 0, 0, 297000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {50000 , { 96, 0, 0, 0, 594000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {60000 , { 97, 0, 0, 0, 594000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {24000 , { 98, 0, 0, 0, 297000, 0, 4096, 1404, 0, 16, 1020,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {30000 , {103, 0, 0, 0, 297000, 0, 3840, 1660, 0, 16, 1276,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {30000 , {104, 0, 0, 0, 297000, 0, 3840, 1440, 0, 16, 1056,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {30000 , {105, 0, 0, 0, 297000, 0, 3840,  560, 0, 16,  176,  88, 1, 2160, 90, 0, 9,  8, 10, 1}},
        {0     , {  0, 0, 0, 0,      0, 0,    0,    0, 0,  0,    0,   0, 0,    0,  0, 0, 0,  0,  0, 0}},

};


/**
 * @short Get the DTD structure that contains the video parameters
 * @param[in] code VIC code to search for
 * @param[in] refreshRate
 * @return returns a pointer to the DTD structure or NULL if not supported.
 * If refreshRate=0 then the first (default) parameters are returned for the VIC code.
 */
static dtd_t * get_dtd(u8 code, u32 refreshRate){
	int i = 0;

	for(i = 0; _dtd[i].dtd.mCode != 0; i++){
		if(_dtd[i].dtd.mCode == code){
			if(!refreshRate){
				return &_dtd[i].dtd;
			}
			if(refreshRate == _dtd[i].refresh_rate){
				return &_dtd[i].dtd;
			}
		}
	}
	return NULL;
}

int hdmi_dtd_fill(dtd_t * dtd, u8 code, u32 refreshRate)
{      
        dtd_t * p_dtd = NULL;
        
	LOG_TRACE();


	p_dtd = get_dtd(code, refreshRate);
	if(p_dtd == NULL){
		LOGGER(SNPS_ERROR, "VIC code [%d] with refresh rate [%dHz] is not supported", code, refreshRate);
		return -1;
	}
	p_dtd->mLimitedToYcc420 = 0;
	p_dtd->mYcc420 = 0;

	memcpy(dtd, p_dtd, sizeof(dtd_t));

	return 0;
}

unsigned int hdmi_dtd_get_refresh_rate(dtd_t *dtd){
        int i = 0;
        for(i = 0; _dtd[i].dtd.mCode != 0; i++){
                if(_dtd[i].dtd.mCode == dtd->mCode){
                        if(dtd->mPixelClock == 0 || _dtd[i].dtd.mPixelClock == dtd->mPixelClock)
                                return _dtd[i].refresh_rate;
                }
        }
        return 0;
}


