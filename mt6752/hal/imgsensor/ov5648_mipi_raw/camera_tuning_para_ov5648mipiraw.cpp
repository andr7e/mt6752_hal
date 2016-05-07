/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_ov5648mipiraw.h"
#include "camera_info_ov5648mipiraw.h"
#include "camera_custom_AEPlinetable.h"
#include "camera_custom_tsf_tbl.h"


const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
      {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      }
    },
    ISPPca: {
#include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
#include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMfbMixer:{{
      0x01FF0001, // MIX3_CTRL_0
      0x00FF0000, // MIX3_CTRL_1
      0xFFFF0000  // MIX3_SPARE
    }},
    ISPMulitCCM:{
      Poly22:{
        60040,    // i4R_AVG
        12975,    // i4R_STD
        74820,    // i4B_AVG
        19537,    // i4B_STD
        942,      // i4R_MAX
        511,      // i4R_MIN
        617,      // i4G_MAX
        582,      // i4G_MIN   
        967,      // i4B_MAX
        645,      // i4B_MIN
        {  // i4P00[9]
            7820000, -2538000, -168000, -1140000, 6056000, 202000, 130000, -3036000, 8020000
        },
        {  // i4P10[9]
            2064555, -1309187, -755257, -279654, -78085, 354988, -135583, 653742, -522159
        },
        {  // i4P01[9]
            601785, -799997, 199389, -423520, 1020, 428621, -214882, -487652, 694593
        },
        {  // i4P20[9]
           0, 0, 0, 0, 0, 0, 0, 0, 0
                },
                { // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
                },
                { // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
                }

      },
      AWBGain:{
        // Strobe
        {
          713,    // i4R
          512,    // i4G
          543    // i4B
        },
        // A
        {
          385,    // i4R
          512,    // i4G
          982    // i4B
        },
        // TL84
        {
          581,    // i4R
          512,    // i4G
          791    // i4B
        },
        // CWF
        {
          676,    // i4R
          512,    // i4G
          873    // i4B
        },
        // D65
        {
          647,    // i4R
          512,    // i4G
          552    // i4B
        },
        // Reserved 1
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 2
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 3
        {
            512,    // i4R
            512,    // i4G
            512    // i4B
        }
      },
      Weight:{
        1, // Strobe
        1, // A
        1, // TL84
        1, // CWF
        1, // D65
        1, // Reserved 1
        1, // Reserved 2
        1  // Reserved 3
      }
    },
    //bInvokeSmoothCCM
    bInvokeSmoothCCM: MFALSE
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,   // u4MinGain, 1024 base =  1x
            10240,  // u4MaxGain, 16x
            100,     // u4MiniISOGain, ISOxx
            64,    // u4GainStepUnit, 1x/8
            35,     // u4PreExpUnit
            30,     // u4PreMaxFrameRate
            35,     // u4VideoExpUnit
            30,     // u4VideoMaxFrameRate
            1024,   // u4Video2PreRatio, 1024 base = 1x
            35,     // u4CapExpUnit
            15,     // u4CapMaxFrameRate
            1024,   // u4Cap2PreRatio, 1024 base = 1x
            35,     // u4Video1ExpUnit
            30,     // u4Video1MaxFrameRate
            1024,   // u4Video12PreRatio, 1024 base = 1x
            35,     // u4Video2ExpUnit
            30,     // u4Video2MaxFrameRate
            1024,   // u4Video22PreRatio, 1024 base = 1x
            35,     // u4Custom1ExpUnit
            30,     // u4Custom1MaxFrameRate
            1024,   // u4Custom12PreRatio, 1024 base = 1x
            35,     // u4Custom2ExpUnit
            30,     // u4Custom2MaxFrameRate
            1024,   // u4Custom22PreRatio, 1024 base = 1x
            35,     // u4Custom3ExpUnit
            30,     // u4Custom3MaxFrameRate
            1024,   // u4Custom32PreRatio, 1024 base = 1x
            35,     // u4Custom4ExpUnit
            30,     // u4Custom4MaxFrameRate
            1024,   // u4Custom42PreRatio, 1024 base = 1x
            35,     // u4Custom5ExpUnit
            30,     // u4Custom5MaxFrameRate
            1024,   // u4Custom52PreRatio, 1024 base = 1x
            24,      // u4LensFno, Fno = 2.8
            350     // u4FocusLength_100x
        },
        // rHistConfig
        {
            4, // 2,   // u4HistHighThres
            40,  // u4HistLowThres
            2,   // u4MostBrightRatio
            1,   // u4MostDarkRatio
            160, // u4CentralHighBound
            20,  // u4CentralLowBound
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]
            {62, 70, 82, 108, 141},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
        },
        // rCCTConfig
        {
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            TRUE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            TRUE,            // bEnableCaptureThres
            TRUE,            // bEnableVideoThres
            TRUE,            // bEnableVideo1Thres
            TRUE,            // bEnableVideo2Thres
            TRUE,            // bEnableCustom1Thres
            TRUE,            // bEnableCustom2Thres
            TRUE,            // bEnableCustom3Thres
            TRUE,            // bEnableCustom4Thres
            TRUE,            // bEnableCustom5Thres
            TRUE,            // bEnableStrobeThres
            50,                // u4AETarget
            50,                // u4StrobeAETarget

            48,                // u4InitIndex
            4,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            2,                 // u4BlackLightStrengthIndex
            2,                 // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
                        -7,               // i4BVOffset delta BV = -2.3
            64,                 // u4PreviewFlareOffset
            64,                 // u4CaptureFlareOffset
            3,                 // u4CaptureFlareThres
            64,                 // u4VideoFlareOffset
            3,                 // u4VideoFlareThres
            64,               // u4CustomFlareOffset
            3,                 //  u4CustomFlareThres
            64,                 // u4StrobeFlareOffset //12 bits
            3,                 // u4StrobeFlareThres // 0.5%
            150,                 // u4PrvMaxFlareThres //12 bit
            0,                 // u4PrvMinFlareThres
            160,                 // u4VideoMaxFlareThres // 12 bit
            0,                 // u4VideoMinFlareThres
            18,                // u4FlatnessThres              // 10 base for flatness condition.
            75,    // u4FlatnessStrength
            //rMeteringSpec
            {
                //rHS_Spec
                {
                    TRUE,//bEnableHistStretch           // enable histogram stretch
                    1024,//u4HistStretchWeight          // Histogram weighting value
                    40, //50, //20,//u4Pcent                      // 1%=10, 0~1000
                    160, //166,//176,//u4Thd                        // 0~255
                    75, //54, //74,//u4FlatThd                    // 0~255

                    120,//u4FlatBrightPcent
                    120,//u4FlatDarkPcent
                    //sFlatRatio
                    {
                        1000,  //i4X1
                        1024,  //i4Y1
                        2400, //i4X2
                        0     //i4Y2
                    },
                    TRUE, //bEnableGreyTextEnhance
                    1800, //u4GreyTextFlatStart, > sFlatRatio.i4X1, < sFlatRatio.i4X2
                    {
                        10,     //i4X1
                        1024,   //i4Y1
                        80,     //i4X2
                        0       //i4Y2
                    }
                },
                //rAOE_Spec
                {
                    TRUE, //bEnableAntiOverExposure
                    1024, //u4AntiOverExpWeight
                    10,    //u4Pcent
                    200,  //u4Thd

                    TRUE, //bEnableCOEP
                    1,    //u4COEPcent
                    106,  //u4COEThd
                    0,  // u4BVCompRatio
                    //sCOEYRatio;     // the outer y ratio
                    {
                        23,   //i4X1
                        1024,  //i4Y1
                        47,   //i4X2
                        0     //i4Y2
                    },
                    //sCOEDiffRatio;  // inner/outer y difference ratio
                    {
                        1500, //i4X1
                        0,    //i4Y1
                        2100, //i4X2
                        1024   //i4Y2
                    }
                },
                //rABL_Spec
                {
                    TRUE,//bEnableBlackLigh
                    1024,//u4BackLightWeigh
                    400,//u4Pcent
                    120,//u4Thd,
                    255, // center luminance
                    256, // final target limitation, 256/128 = 2x
                    //sFgBgEVRatio
                    {
                        2200, //i4X1
                        0,    //i4Y1
                        4000, //i4X2
                        1024   //i4Y2
                    },
                    //sBVRatio
                    {
                        3800,//i4X1
                        0,   //i4Y1
                        5000,//i4X2
                        1024  //i4Y2
                    }
                },
                //rNS_Spec
                {
                    TRUE, // bEnableNightScene
                    5,    //u4Pcent
                    170,  //u4Thd
                    72, //52,   //u4FlatThd

                    200,  //u4BrightTonePcent
                    92, //u4BrightToneThd

                    500,  //u4LowBndPcent
                    5,    //u4LowBndThd
                    26,    //u4LowBndThdLimit

                    50,  //u4FlatBrightPcent;
                    300,   //u4FlatDarkPcent;
                    //sFlatRatio
                    {
                        1200, //i4X1
                        1024, //i4Y1
                        2400, //i4X2
                        0    //i4Y2
                    },
                    //sBVRatio
                    {
                        -500, //i4X1
                        1024,  //i4Y1
                        3000, //i4X2
                        0     //i4Y2
                    },
                    TRUE, // bEnableNightSkySuppresion
                    //sSkyBVRatio
                    {
                        -4000, //i4X1
                        1024, //i4X2
                        -2000,  //i4Y1
                        0     //i4Y2
                    }
                },
                // rTOUCHFD_Spec
                {
                    40, //uMeteringYLowBound;
                    50, //uMeteringYHighBound;
                    40, //uFaceYLowBound;
                    50, //uFaceYHighBound;
                    3,  //uFaceCentralWeight;
                    120,//u4MeteringStableMax;
                    80, //u4MeteringStableMin;
                }
            }, //End rMeteringSpec
            // rFlareSpec
            {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //uPrvFlareWeightArr[16];
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //uVideoFlareWeightArr[16];
                96,                                               //u4FlareStdThrHigh;
                48,                                               //u4FlareStdThrLow;
                0,                                                //u4PrvCapFlareDiff;
                4,                                                //u4FlareMaxStepGap_Fast;
                0,                                                //u4FlareMaxStepGap_Slow;
                1800,                                             //u4FlarMaxStepGapLimitBV;
                0,                                                //u4FlareAEStableCount;
            },
            //rAEMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190, //u4Bright2TargetEnd
                20,   //u4Dark2TargetStart
                90, //u4B2TEnd
                70,  //u4B2TStart
                60,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAEVideoMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAEVideo1MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAEVideo2MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAECustom1MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAECustom2MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAECustom3MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAECustom4MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAECustom5MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },

            //rAEFaceMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190,  //u4Bright2TargetEnd
                10,    //u4Dark2TargetStart
                80, //u4B2TEnd
                30,  //u4B2TStart
                20,  //u4D2TEnd
                60,  //u4D2TStart
            },

            //rAETrackingMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190,  //u4Bright2TargetEnd
                10,    //u4Dark2TargetStart
                80, //u4B2TEnd
                30,  //u4B2TStart
                20,  //u4D2TEnd
                60,  //u4D2TStart
            },
            //rAEAOENVRAMParam =
            {
                1,      // i4AOEStrengthIdx: 0 / 1 / 2
                128,    // u4BVCompRatio
                {
                    {
                        47,  //u4Y_Target
                        10,  //u4AOE_OE_percent
                        160,  //u4AOE_OEBound
                        15,    //u4AOE_DarkBound
                        950,    //u4AOE_LowlightPrecent
                        5,    //u4AOE_LowlightBound
                        100,    //u4AOESceneLV_L
                        150,    //u4AOESceneLV_H
                        40,    //u4AOE_SWHdrLE_Bound
                    },
                    {
                        47,  //u4Y_Target
                        10,  //u4AOE_OE_percent
                        180,  //u4AOE_OEBound
                20,    //u4AOE_DarkBound
                        950,    //u4AOE_LowlightPrecent
                10,    //u4AOE_LowlightBound
                100,    //u4AOESceneLV_L
                150,    //u4AOESceneLV_H
                        40,    //u4AOE_SWHdrLE_Bound
                    },
                    {
                        47,  //u4Y_Target
                        10,  //u4AOE_OE_percent
                        200,  //u4AOE_OEBound
                        25,    //u4AOE_DarkBound
                        950,    //u4AOE_LowlightPrecent
                        15,    //u4AOE_LowlightBound
                        100,    //u4AOESceneLV_L
                        150,    //u4AOESceneLV_H
                        40,    //u4AOE_SWHdrLE_Bound
                    }
                }
            }
        }
    },
        // AWB NVRAM
        {
        {
                // AWB calibration data
                {
                        // rCalGain (calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rDefGain (Default calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rDefGain (Default calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rD65Gain (D65 WB gain: 1.0 = 512)
                        {
                                642,	// u4R
                                512,	// u4G
                                541	    // u4B
                        }
                },
                // Original XY coordinate of AWB light source
                {
                        // Strobe
                        {
							107,
							-146
                        },
                        // Horizon
                        {
							-477,
							-126
                        },
                        // A
                        {
							-357,
							-140
                        },
                        // TL84
                        {
							-112,
							-210
                        },
                        // CWF
                        {
							-96,
							-305
                        },
                        // DNP
                        {
							-61,
							-113
                        },
                        // D65
                        {
							63,
						   -104
                        },
                        // DF
                        {
							94,
							-229
                        }
                },
                // Rotated XY coordinate of AWB light source
                {
                        // Strobe
                        {
							94, -155
                        },
                        // Horizon
                        {
							-486, -85
                        },
                        // A
                        {
							-368, -109
                        },
                        // TL84
                        {
							-130, -200
                        },
                        // CWF
                        {
							-122, -296
                        },
                        // DNP
                        {
							-70, -107
                        },
                        // D65
                        {
							54, -109
                        },
                        // DF
                        {
							74, -236
                        }
                },
                // AWB gain of AWB light source
                {
                        // Strobe
                        {
							721, 512, 540
                        },
                        // Horizon
                        {
							512, 823, 1861
                        },
                        // A
                        {
							512, 687, 1347
                        },
                        // TL84
                        {
							585, 512, 792
                        },
                        // CWF
                        {
							680, 512, 881
                        },
                        // DNP
                        {
							550, 512, 648
                        },
                        // D65
                        {
							642, 512, 541
                        },
                        // DF
                        {
							792, 512, 615
                        }
                },
                // Rotation matrix parameter
                {
                        5,	            // i4RotationAngle
                        255,	        // i4Cos
                        22,	            // i4Sin
                },
                // Daylight locus parameter
                {
                        -148,       	// i4SlopeNumerator
                        128    // i4SlopeDenominator
                },
	            // Predictor gain
                {
                        // i4PrefRatio100
                        101,

                        // DaylightLocus_L
                        {
                            642,    // i4R
                            512,    // i4G
                            541     // i4B
                        },
                        // DaylightLocus_H
                        {
                            539,    // i4R
                            512,    // i4G
                            666     // i4B
                        },
                        // Temporal General
                        {
                            642,    // i4R
                            512,    // i4G
                            541     // i4B
                        }
                },
                // AWB light area
                {
                        // Strobe
                        {
							144, 44, -105, -205
                        },
                        // Tungsten
                        {
							-225, -886, -50, -155
                        },
                        // Warm fluorescent
                        {
							-225, -886, -155, -326
                        },
                        // Fluorescent
                        {
							-85, -225, -69, -260
                        },
                        // CWF
                        {
							-44, -225, -260, -341
                        },
                        // Daylight
                        {
							84, -85, -69, -260
                        },
                        // Shade
                        {
							414, 84, -69, -184
                        },
                        // Daylight Fluorescent
                        {
							84, -44, -260, -341
                        }
                },
                // PWB light areaPWB light area
                {
                        // Reference area
                        {
							414, -886, -50, -341
                        },
                        // Daylight
                        {
							109, -85, -69, -260
                        },
                        // Cloudy daylight
                        {
							209, 34, -69, -260
                        },
                        // Shade
                        {
							309, 34, -69, -260
                        },
                        // Twilight
                        {
							-85, -245, -69, -260
                        },
                        // Fluorescent
                        {
							104, -230, -59, -346
                        },
                        // Warm fluorescent
                        {
							-268, -468, -59, -346
                        },
                        // Incandescent
                        {
							-268, -468, -69, -260
                        },
                        // Gray World
                        {
                                5000,    // i4RightBound
                                -5000,    // i4LeftBound
                                5000,    // i4UpperBound
                                -5000    // i4LowerBound
                        }
                },
                // PWB default gain
                {
                        // Daylight
                        {
							661, 512, 616
                        },
                        // Cloudy daylight
                        {
							757, 512, 525
                        },
                        // Shade
                        {
							805, 512, 488
                        },
                        // Twilight
                        {
							532, 512, 799
                        },
                        // Fluorescent
                        {
							637, 512, 721
                        },
                        // Warm fluorescent
                        {
							438, 512, 1127
                        },
                        // Incandescent
                        {
							414, 512, 1075
                        },
                        // Gray World
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        }
                },
                // AWB preference color
                {
                        // Tungsten
                        {
                                20,	// i4SliderValue
                                5773	// i4OffsetThr
                        },
                        // Warm fluorescent
                        {
                                50,	// i4SliderValue
                                3817	// i4OffsetThr
                        },
                        // Shade
                        {
                                50,	// i4SliderValue
                                410	// i4OffsetThr
                        },
                        // Preference gain: strobe
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: tungsten
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: warm fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: CWF
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: daylight
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: shade
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: daylight fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        }
                },

                // Algorithm Tuning Paramter
                {
                    // AWB Backup Enable
                    FALSE,

                    // AWB LSC Gain
                    {
                        539,        // i4R
                        512,        // i4G
                        666         // i4B
                    },
                    // Parent block weight parameter
                    {
                        TRUE,      // bEnable
                        6           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
                    },
                    // AWB LV threshold for predictor
                    {
                            115, //100,    // i4InitLVThr_L
                            155,//140,    // i4InitLVThr_H
                            100      // i4EnqueueLVThr
                    },
                    // AWB number threshold for temporal predictor
                    {
                            65,     // i4Neutral_ParentBlk_Thr
                        //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                        { 100, 100, 100, 100, 100, 100, 100, 100, 100, 50,  25,   2,   2,   2,   2,   2,   2,   2,   2}  // (%) i4CWFDF_LUTThr
                        },
                    // AWB light neutral noise reduction for outdoor
                    {
                        //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                        // Non neutral
		                { 3,   3,   3,   3,   3,   3,   3,   3,    3,   3,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // Flurescent
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // CWF
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // Daylight
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   2},  // (%)
		                // DF
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    },
                    // AWB feature detection
                    {
                        // Sunset Prop
                        {
                            1,          // i4Enable
                            140, //130,        // i4LVThr
                            {
                                -55,   // i4Sunset_BoundXr_Thr
                                -107    // i4Sunset_BoundYr_Thr
                            },
                            10,         // i4SunsetCountThr
                            0,          // i4SunsetCountRatio_L
                            171         // i4SunsetCountRatio_H
                        },

                        // Shade F Detection
                        {
                            1,          // i4Enable
                            115,//105,        // i4LVThr
                            {

                                -115,   // i4BoundXrThr
                                -109    // i4BoundYrThr
                            },
                            128         // i4DaylightProb
                        },

                        // Shade CWF Detection
                        {
                            1,          // i4Enable
                            110,//95,         // i4LVThr
                            {
                                -122,   // i4BoundXrThr
                                -296    // i4BoundYrThr
                            },
                            128         // i4DaylightProb
                        },

                        // Low CCT
                        {
                            1,          // i4Enable
                            256,        // i4SpeedRatio
                            {
                                -506,          // i4BoundXrThr
                                231        // i4BoundYrThr
                            }
                        }

                    },

                    // AWB Gain Limit
                    {
                        // rNormalLowCCT
                        {
                            1,      // Gain Limit Enable
                            1536 //717     // Gain ratio
                        },
                        // rPrefLowCCT
                        {
                            1,      // Gain Limit Enable
                            1536 //870     // Gain ratio
                        }
                    },

                    // AWB non-neutral probability for spatial and temporal weighting look-up table (Max: 100; Min: 0)
                    {
                        //LV0   1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
                        { 0, 33, 66, 100, 100, 100, 100, 100, 100, 100, 100,  70,  30,  20,   10,   0,   0,   0,   0}
                    },

                    // AWB daylight locus probability look-up table (Max: 100; Min: 0)
                    {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
						


						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0, 0},    // Strobe
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0},  // Tungsten
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0},  // Warm fluorescent
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 70, 40, 20, 0, 0, 0},   // Fluorescent
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 12, 0, 0, 0, 0},     // CWF
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 50, 30, 20}, // Daylight
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0, 0},    // Shade
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 12, 0, 0, 0, 0},     // Daylight fluorescent
    		        }
                },

                // CCT estimation
                {
                        // CCT
                        {
			                2300,	// i4CCT[0]
			                2850,	// i4CCT[1]
			                3750,	// i4CCT[2]
			                5100,	// i4CCT[3]
			                6500 	// i4CCT[4]
		            },
                        // Rotated X coordinate
                        {
                                -540,	// i4RotatedXCoordinate[0]
                                -422,	// i4RotatedXCoordinate[1]
                                -184,	// i4RotatedXCoordinate[2]
                                -124,	// i4RotatedXCoordinate[3]
			                0 	    // i4RotatedXCoordinate[4]
		            }
	            }

        },
                {
                // AWB calibration data
                {
                        // rCalGain (calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rDefGain (Default calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rDefGain (Default calibration gain: 1.0 = 512)
                        {
                                0,	// u4R
                                0,	// u4G
                                0	// u4B
                        },
                        // rD65Gain (D65 WB gain: 1.0 = 512)
                        {
                                642,	// u4R
                                512,	// u4G
                                541	    // u4B
                        }
                },
                // Original XY coordinate of AWB light source
                {
                        // Strobe
                        {
							107,
							-146
                        },
                        // Horizon
                        {
							-477,
							-126
                        },
                        // A
                        {
							-357,
							-140
                        },
                        // TL84
                        {
							-112,
							-210
                        },
                        // CWF
                        {
							-96,
							-305
                        },
                        // DNP
                        {
							-61,
							-113
                        },
                        // D65
                        {
							63,
						   -104
                        },
                        // DF
                        {
							94,
							-229
                        }
                },
                // Rotated XY coordinate of AWB light source
                {
                        // Strobe
                        {
							94, -155
                        },
                        // Horizon
                        {
							-486, -85
                        },
                        // A
                        {
							-368, -109
                        },
                        // TL84
                        {
							-130, -200
                        },
                        // CWF
                        {
							-122, -296
                        },
                        // DNP
                        {
							-70, -107
                        },
                        // D65
                        {
							54, -109
                        },
                        // DF
                        {
							74, -236
                        }
                },
                // AWB gain of AWB light source
                {
                        // Strobe
                        {
							721, 512, 540
                        },
                        // Horizon
                        {
							512, 823, 1861
                        },
                        // A
                        {
							512, 687, 1347
                        },
                        // TL84
                        {
							585, 512, 792
                        },
                        // CWF
                        {
							680, 512, 881
                        },
                        // DNP
                        {
							550, 512, 648
                        },
                        // D65
                        {
							642, 512, 541
                        },
                        // DF
                        {
							792, 512, 615
                        }
                },
                // Rotation matrix parameter
                {
                        5,	            // i4RotationAngle
                        255,	        // i4Cos
                        22,	            // i4Sin
                },
                // Daylight locus parameter
                {
                        -148,       	// i4SlopeNumerator
                        128    // i4SlopeDenominator
                },
	            // Predictor gain
                {
                        // i4PrefRatio100
                        101,

                        // DaylightLocus_L
                        {
                            642,    // i4R
                            512,    // i4G
                            541     // i4B
                        },
                        // DaylightLocus_H
                        {
                            539,    // i4R
                            512,    // i4G
                            666     // i4B
                        },
                        // Temporal General
                        {
                            642,    // i4R
                            512,    // i4G
                            541     // i4B
                        }
                },
                // AWB light area
                {
                        // Strobe
                        {
							144, 44, -105, -205
                        },
                        // Tungsten
                        {
							-225, -886, -50, -155
                        },
                        // Warm fluorescent
                        {
							-225, -886, -155, -326
                        },
                        // Fluorescent
                        {
							-85, -225, -69, -260
                        },
                        // CWF
                        {
							-44, -225, -260, -341
                        },
                        // Daylight
                        {
							84, -85, -69, -260
                        },
                        // Shade
                        {
							414, 84, -69, -184
                        },
                        // Daylight Fluorescent
                        {
							84, -44, -260, -341
                        }
                },
                // PWB light areaPWB light area
                {
                        // Reference area
                        {
							414, -886, -50, -341
                        },
                        // Daylight
                        {
							109, -85, -69, -260
                        },
                        // Cloudy daylight
                        {
							209, 34, -69, -260
                        },
                        // Shade
                        {
							309, 34, -69, -260
                        },
                        // Twilight
                        {
							-85, -245, -69, -260
                        },
                        // Fluorescent
                        {
							104, -230, -59, -346
                        },
                        // Warm fluorescent
                        {
							-268, -468, -59, -346
                        },
                        // Incandescent
                        {
							-268, -468, -69, -260
                        },
                        // Gray World
                        {
                                5000,    // i4RightBound
                                -5000,    // i4LeftBound
                                5000,    // i4UpperBound
                                -5000    // i4LowerBound
                        }
                },
                // PWB default gain
                {
                        // Daylight
                        {
							661, 512, 616
                        },
                        // Cloudy daylight
                        {
							757, 512, 525
                        },
                        // Shade
                        {
							805, 512, 488
                        },
                        // Twilight
                        {
							532, 512, 799
                        },
                        // Fluorescent
                        {
							637, 512, 721
                        },
                        // Warm fluorescent
                        {
							438, 512, 1127
                        },
                        // Incandescent
                        {
							414, 512, 1075
                        },
                        // Gray World
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        }
                },
                // AWB preference color
                {
                        // Tungsten
                        {
                                20,	// i4SliderValue
                                5773	// i4OffsetThr
                        },
                        // Warm fluorescent
                        {
                                50,	// i4SliderValue
                                3817	// i4OffsetThr
                        },
                        // Shade
                        {
                                50,	// i4SliderValue
                                410	// i4OffsetThr
                        },
                        // Preference gain: strobe
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: tungsten
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: warm fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: CWF
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: daylight
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: shade
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        },
                        // Preference gain: daylight fluorescent
                        {
                                512,	// u4R
                                512,	// u4G
                                512	// u4B
                        }
                },

                // Algorithm Tuning Paramter
                {
                    // AWB Backup Enable
                    FALSE,

                    // AWB LSC Gain
                    {
                        539,        // i4R
                        512,        // i4G
                        666         // i4B
                    },
                    // Parent block weight parameter
                    {
                        TRUE,      // bEnable
                        6           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
                    },
                    // AWB LV threshold for predictor
                    {
                            115, //100,    // i4InitLVThr_L
                            155,//140,    // i4InitLVThr_H
                            100      // i4EnqueueLVThr
                    },
                    // AWB number threshold for temporal predictor
                    {
                            65,     // i4Neutral_ParentBlk_Thr
                        //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                        { 100, 100, 100, 100, 100, 100, 100, 100, 100, 50,  25,   2,   2,   2,   2,   2,   2,   2,   2}  // (%) i4CWFDF_LUTThr
                        },
                    // AWB light neutral noise reduction for outdoor
                    {
                        //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                        // Non neutral
		                { 3,   3,   3,   3,   3,   3,   3,   3,    3,   3,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // Flurescent
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // CWF
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
		                // Daylight
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   2},  // (%)
		                // DF
		                { 0,   0,   0,   0,   0,   0,   0,   0,    0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    },
                    // AWB feature detection
                    {
                        // Sunset Prop
                        {
                            1,          // i4Enable
                            140, //130,        // i4LVThr
                            {
                                -55,   // i4Sunset_BoundXr_Thr
                                -107    // i4Sunset_BoundYr_Thr
                            },
                            10,         // i4SunsetCountThr
                            0,          // i4SunsetCountRatio_L
                            171         // i4SunsetCountRatio_H
                        },

                        // Shade F Detection
                        {
                            1,          // i4Enable
                            115,//105,        // i4LVThr
                            {

                                -115,   // i4BoundXrThr
                                -109    // i4BoundYrThr
                            },
                            128         // i4DaylightProb
                        },

                        // Shade CWF Detection
                        {
                            1,          // i4Enable
                            110,//95,         // i4LVThr
                            {
                                -122,   // i4BoundXrThr
                                -296    // i4BoundYrThr
                            },
                            128         // i4DaylightProb
                        },

                        // Low CCT
                        {
                            1,          // i4Enable
                            256,        // i4SpeedRatio
                            {
                                -506,          // i4BoundXrThr
                                231        // i4BoundYrThr
                            }
                        }

                    },

                    // AWB Gain Limit
                    {
                        // rNormalLowCCT
                        {
                            1,      // Gain Limit Enable
                            1536 //717     // Gain ratio
                        },
                        // rPrefLowCCT
                        {
                            1,      // Gain Limit Enable
                            1536 //870     // Gain ratio
                        }
                    },

                    // AWB non-neutral probability for spatial and temporal weighting look-up table (Max: 100; Min: 0)
                    {
                        //LV0   1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
                        { 0, 33, 66, 100, 100, 100, 100, 100, 100, 100, 100,  70,  30,  20,   10,   0,   0,   0,   0}
                    },

                    // AWB daylight locus probability look-up table (Max: 100; Min: 0)
                    {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
						


						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0, 0},    // Strobe
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0},  // Tungsten
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0},  // Warm fluorescent
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 70, 40, 20, 0, 0, 0},   // Fluorescent
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 12, 0, 0, 0, 0},     // CWF
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 50, 30, 20}, // Daylight
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 0, 0, 0, 0},    // Shade
						{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 25, 12, 0, 0, 0, 0},     // Daylight fluorescent
    		        }
                },

                // CCT estimation
                {
                        // CCT
                        {
			                2300,	// i4CCT[0]
			                2850,	// i4CCT[1]
			                3750,	// i4CCT[2]
			                5100,	// i4CCT[3]
			                6500 	// i4CCT[4]
		            },
                        // Rotated X coordinate
                        {
                                -540,	// i4RotatedXCoordinate[0]
                                -422,	// i4RotatedXCoordinate[1]
                                -184,	// i4RotatedXCoordinate[2]
                                -124,	// i4RotatedXCoordinate[3]
			                0 	    // i4RotatedXCoordinate[4]
		            }
		            }
	            }
        },


    // Flash AWB NVRAM
    {
#include INCLUDE_FILENAME_FLASH_AWB_PARA
    },

    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    {
                0,  // isTsfEn
        2,  // tsfCtIdx
        {20, 2000, -110, -110, 512, 512, 512, 0}    // rAWBInput[8]
    },

#include INCLUDE_FILENAME_TSF_PARA
#include INCLUDE_FILENAME_TSF_DATA
};

const NVRAM_CAMERA_FEATURE_STRUCT CAMERA_FEATURE_DEFAULT_VALUE =
{
#include INCLUDE_FILENAME_FEATURE_PARA
};

typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


#define READ_CALIBRATION_FROM_STOCK

#ifdef READ_CALIBRATION_FROM_STOCK

#include "readcalibration.h"

#endif

namespace NSFeature {
  template <>
  UINT32
  SensorInfoSingleton_T::
  impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
  {
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
        sizeof(NVRAM_CAMERA_3A_STRUCT),
        sizeof(NVRAM_CAMERA_SHADING_STRUCT),
        sizeof(NVRAM_LENS_PARA_STRUCT),
        sizeof(AE_PLINETABLE_T),
        0,
        sizeof(CAMERA_TSF_TBL_STRUCT),
        0,
        sizeof(NVRAM_CAMERA_FEATURE_STRUCT)
    };

    if (CameraDataType > CAMERA_NVRAM_DATA_FEATURE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
      return 1;
    }
	
    switch(CameraDataType)
    {
#ifdef READ_CALIBRATION_FROM_STOCK
	  case CAMERA_NVRAM_DATA_ISP:
        readCalibration(CAMERA_NVRAM_DATA_ISP, pDataBuf, sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
        break;
      case CAMERA_NVRAM_DATA_3A:
        readCalibration(CAMERA_NVRAM_DATA_3A,  pDataBuf, sizeof(NVRAM_CAMERA_3A_STRUCT));
        break;
#else
      case CAMERA_NVRAM_DATA_ISP:
        memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
        break;
      case CAMERA_NVRAM_DATA_3A:
        memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
        break;
#endif
      case CAMERA_NVRAM_DATA_SHADING:
        memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
        break;
      case CAMERA_DATA_AE_PLINETABLE:
        memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
        break;
      case CAMERA_DATA_TSF_TABLE:
        memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
        break;
      case CAMERA_NVRAM_DATA_FEATURE:
        memcpy(pDataBuf,&CAMERA_FEATURE_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_FEATURE_STRUCT));
        break;
      default:
        break;
    }
    return 0;
  }};  //  NSFeature


