// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
#ifdef CONFIG_MTK_CAM_PD2139F_EX

    {S5KJN1SQ03PD2139_SENSOR_ID, 0xA0, Common_read_region},
  	{OV13B10PD2139_SENSOR_ID, 0xA4, Common_read_region},
  	{OV13B102APD2139_SENSOR_ID, 0xA4, Common_read_region},
    {IMX355PD2139_SENSOR_ID, 0xA2, Common_read_region},
    {IMX355V1PD2139_SENSOR_ID, 0xA2, Common_read_region},
    {GC08A3PD2139_SENSOR_ID, 0xA2, Common_read_region},
    {GC02M1PD2139_SENSOR_ID, 0xA4, Common_read_region},

#elif defined(CONFIG_MTK_CAM_PD2140F_EX) || defined(CONFIG_MTK_CAM_PD2140IF_EX) || defined(CONFIG_MTK_CAM_PD2140) || defined(CONFIG_MTK_CAM_PD2140JF_EX) || defined(CONFIG_MTK_CAM_PD2213F_EX) || defined(CONFIG_MTK_CAM_PD2140NF_EX)
	/*PD2140 start*/
	{HI1336QTECHPD2140MAIN_SENSOR_ID, 0xA2, Common_read_region},
	{HI1336SUNNYPD2140MAIN_SENSOR_ID, 0xA2, Common_read_region},
	{HI556LCEPD2140FRONT_SENSOR_ID, 0x40, Common_read_region},
	{GC5035TXDPD2140FRONT_SENSOR_ID, 0xA6, Common_read_region},
	{GC5035HLTPD2140FRONT_SENSOR_ID, 0xA6, Common_read_region},
    {GC08A3QTEPD2140MAIN_SENSOR_ID, 0xA4, Common_read_region},
	{GC08A3QTEPD2140FRONT_SENSOR_ID, 0xA8, Common_read_region},
	{GC08A3SUNPD2140MAIN_SENSOR_ID, 0xA4, Common_read_region},
	{GC08A3SUNPD2140FRONT_SENSOR_ID, 0xA8, Common_read_region},
	{GC02M1PD2140_SENSOR_ID, 0xA0, Common_read_region},
	{OV02B10LCEPD2140MACRO_SENSOR_ID, 0xA0, Common_read_region},
	{GC02M1HLTPD2140MACRO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX258QTEPD2140MAIN_SENSOR_ID,  0xA2, Common_read_region},
	{IMX355SUNPD2140FRONT_SENSOR_ID, 0xA8, Common_read_region},
	{IMX355QTEPD2140MAIN_SENSOR_ID, 0xA4, Common_read_region},
#elif defined(CONFIG_MTK_CAM_PD2216F_EX) || defined(CONFIG_MTK_CAM_PD2216JF_EX)
	{IMX355TXDPD2216MAIN_SENSOR_ID, 0xB0, Common_read_region},
	{IMX355SWNPD2216MAIN_SENSOR_ID, 0xB0, Common_read_region},
	{S5K4H7SUNPD2216MAIN_SENSOR_ID, 0xA4, Common_read_region},
	{HI556LCEPD2216FRONT_SENSOR_ID, 0x40, Common_read_region},
	{GC2375CXTPD2216MACRO_SENSOR_ID, 0xA0, Common_read_region},
	{OV13B10LCEPD2216MAIN_SENSOR_ID, 0xA2, Common_read_region},
	{OV13B10TXDPD2216MAIN_SENSOR_ID, 0xB0, Common_read_region},
	{IMX258QTEPD2216MAIN_SENSOR_ID, 0xB0, Common_read_region},
	{OV02B10LCEPD2216MACRO_SENSOR_ID, 0xA0, Common_read_region},
	{GC2375SUNPD2216MACRO_SENSOR_ID, 0xA0, Common_read_region},
	{GC5035TLYPD2216FRONT_SENSOR_ID, 0xA8, Common_read_region},
	{OV05A10TXDPD2216FRONT_SENSOR_ID, 0xA8, Common_read_region},
#elif defined(CONFIG_MTK_CAM_PD2236F_EX) || defined(CONFIG_MTK_CAM_PD2236DF_EX) || defined(CONFIG_MTK_CAM_PD2236IF_EX) || defined(CONFIG_MTK_CAM_PD2236)
	{OV05A10QTEPD2236FRONT_SENSOR_ID, 0xA8, Common_read_region},
	{HI556LCEPD2236FRONT_SENSOR_ID, 0x40, Common_read_region},
	{OV08D10SUNPD2236MAIN_SENSOR_ID, 0xA4, Common_read_region},
	{HI846HLTPD2236MAIN_SENSOR_ID, 0x44, Common_read_region},
	{GC08A3LCEPD2236MAIN_SENSOR_ID, 0x62, Common_read_region},
#else
	{OV13B10PD2036_SENSOR_ID, 0xA4, Common_read_region},
	{IMX258PD2036_SENSOR_ID, 0xA0, Common_read_region},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region},
	{S5K3L6XXPD2074_SENSOR_ID, 0xA0, Common_read_region},
    {OV13B10_SENSOR_ID, 0xA4, Common_read_region},
	{S5K4H7YX_SENSOR_ID, 0xA2, Common_read_region},
	{HI259_SENSOR_ID, 0xA4, Common_read_region},
	{S5K3L6XXPD2074V2_SENSOR_ID, 0xA0, Common_read_region},
	{HI846PD2036_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXPD2036_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXV1PD2036_SENSOR_ID, 0xA2, Common_read_region},
	{HI556PD2074_SENSOR_ID, 0xA2, Common_read_region},
	{HI556PD2074V1_SENSOR_ID, 0xA2, Common_read_region},
	{IMX230_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{OV13B10PD2139JF_SENSOR_ID, 0xA4, Common_read_region},
	{IMX355PD2139JF_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1PD2139JF_SENSOR_ID, 0xA4, Common_read_region},
	/*B+B*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856_SENSOR_ID, 0xA0, Common_read_region},
	/*61*/
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0xA2, Common_read_region},
	/*99*/
	{IMX258_SENSOR_ID, 0xA0, Common_read_region},
	{IMX258_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*97*/
	{OV23850_SENSOR_ID, 0xA0, Common_read_region},
	{OV23850_SENSOR_ID, 0xA8, Common_read_region},
	{S5K3M2_SENSOR_ID, 0xA0, Common_read_region},
	/*55*/
	{S5K2P8_SENSOR_ID, 0xA2, Common_read_region},
	{S5K2P8_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA2, Common_read_region},
	/* Others */
	{S5K2X8_SENSOR_ID, 0xA0, Common_read_region},
	{IMX377_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX486_SENSOR_ID, 0xA8, Common_read_region},
	{OV12A10_SENSOR_ID, 0xA8, Common_read_region},
	{OV13855_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{HI556_SENSOR_ID, 0x51, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0x5a, Common_read_region},
	{S5K5E8YXREAR2_SENSOR_ID, 0x5a, Common_read_region},
#endif
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


