/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "imgsensor_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi259v1mipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "HI259V1_camera_sensor  "
#define LOG_1 LOG_INF("HI259V1,MIPI 1LANE\n")
#define LOG_2 LOG_INF("preview 1600*1200@30fps,600Mbps/lane; video 1600*1200@30fps,600Mbps/lane; 1600*1200@30fps,600Mbps/lane\n")
/****************************   Modify end    *******************************************/
//#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(fmt, args...)  printk(KERN_INFO PFX fmt, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI259V1_SENSOR_ID, //HI259V1_SENSOR_ID,  /*sensor_id = 0x00E1*/ //record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0xcdcc26f2, //checksum value for Camera Auto Test

	.pre = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength = 1800,				//record different mode's linelength
		.framelength = 1236,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1228,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {/*normal capture*/
		.pclk = 72000000,
		.linelength = 1800,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1228,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.cap1 = {/*PIP capture*/ //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 72000000,
		.linelength = 1800,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1228,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150, //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
	},
	.normal_video = {
		.pclk = 72000000,
		.linelength = 1800,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 800,
		.grabwindow_height = 600,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 600,
	},
	.hs_video = {/*slow motion*/
		.pclk = 72000000,
		.linelength = 1800,
		.framelength = 662,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 600,

	},
	.slim_video = {/*VT Call*/
		.pclk = 72000000,
		.linelength = 1800,
		.framelength = 1292,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},

	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4, //1,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 1,		//enter capture delay frame num
	.pre_delay_frame = 2,		//enter preview delay frame num
	.video_delay_frame = 1, 	//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num

	.isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
 //Gionee: malp <2016-05-24> modify for CR01703897 bigin
#ifdef ORIGINAL_VERSION
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
#endif
	//Gionee: malp <2016-05-24> modify for CR01703897 end
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_1_LANE,//mipi lane num
	.i2c_addr_table = {0x70,0x60, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};



static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x14d,					//current shutter
	.gain = 0xe000,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x60,//record current sensor's i2c write id
};


/* Sensor output window information */
/*according toHI259V1 datasheet p53 image cropping*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =
{{ 1632, 1228,    0,  0, 1632, 1228, 1632, 1228, 0, 0, 1632, 1228,	  0,	0, 1632, 1228}, // Preview
 { 1632, 1228,    0,  0, 1632, 1228, 1632, 1228, 0, 0, 1632, 1228,	  0,	0, 1632, 1228}, // capture
 { 1632, 1228,    8,  8, 1600, 1200, 800, 600, 0, 0, 800, 600,	  0,	0, 800, 600}, // video
 { 1632, 1228,	  68,88, 1600,  1200, 640,  480, 0, 0, 640,  480,	  0,	0, 640,  480}, //hight speed video
 { 1632, 1228,	  256,176, 1280,  720, 1280,  720, 0, 0, 1280,  720,	  0,	0, 1280,  720}, // slim video
 { 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200}, //custom 1
 { 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 { 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 { 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 { 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
};
 //{ 1632, 1228,    8,  8, 1600, 1200, 800, 600, 0, 0, 800, 600,	  0,	0, 800, 600}, // video
 //{ 1632, 1228,	  68,88, 1600,  1200, 640,  480, 0, 0, 640,  480,	  0,	0, 640,  480}, //hight speed video
 //{ 1632, 1228,	  256,176, 1280,  720, 1280,  720, 0, 0, 1280,  720,	  0,	0, 1280,  720}, // slim video
 //{ 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200}, //custom 1
 //{ 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 //{ 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 //{ 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
 //{ 1632, 1228,	  0,  0, 1615, 1215, 1615, 1215, 8, 8, 1600, 1200,	  0,	0, 1600, 1200},
//};



extern int hi259v1_vivo_otp_read(void);
static int vivo_otp_read_when_power_on_hi259v1 = 0;
MUINT32  sn_inf_main_hi259v1 [13];  /*0 flag   1-12 data*/
extern otp_error_code_t HI259V1_OTP_ERROR_CODE;
extern unsigned int is_atboot;
extern unsigned  int hi259v1_flag;



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	 char pu_send_cmd[1] = {(char)(addr & 0xFF) };
	kal_uint16 get_byte=0;


	printk("read_cmos_sensor");
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor_hi259v1_gc2375(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 2, 0x2e);
}


static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
	//iWriteReg((u16)addr, (u32)para, 2, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */

	/*  Add dummy pixels: */
    /* 0x380c [0:4], 0x380d defines the PCLKs in one line of HI259V1  */
    /* Add dummy lines:*/
    /* 0x380e [0:1], 0x380f defines total lines in one frame of HI259V1 */
//rampart
	//write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	//write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    //write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
    //write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);

}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
   return read_cmos_sensor(0x04);
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
//	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution
	*  if shutter bigger than frame_length, should extend frame length first
	*/

	if(!shutter) shutter = 1; /*avoid 0*/

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	//if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
        else {
        // Extend frame length
        //rampart
        //write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        //write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        //write_cmos_sensor(0x03, 0x00);
        //write_cmos_sensor(0x4e, imgsensor.frame_length >> 8);
		//write_cmos_sensor(0x4f, imgsensor.frame_length & 0xFF);
        }
	} else {
		// Extend frame length
		//rampart
		//write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		//write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
       // write_cmos_sensor(0x03, 0x00);
       // write_cmos_sensor(0x4e, imgsensor.frame_length >> 8);
	//	write_cmos_sensor(0x4f, imgsensor.frame_length & 0xFF);

	}

	// Update Shutter
	//rampart
	//write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
	//write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	//write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	write_cmos_sensor(0x03, 0x00);
	write_cmos_sensor(0x1f, 0x01);
	write_cmos_sensor(0x03, 0x20);
	write_cmos_sensor(0x22, shutter >> 8);
	write_cmos_sensor(0x23, shutter & 0xFF);
	write_cmos_sensor(0x03, 0x00);
	write_cmos_sensor(0x1f, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	LOG_INF("frame_length = %d ", frame_length);

}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{

	kal_uint16 reg_gain = 0xe000;
//	kal_uint16 real_gain;
	//real_gain = gain / 64;

    //reg_gain =  (uint16_t)((512/gain)-(292/10));
    reg_gain =  (uint16_t)(((256 * 64 / gain)-17) * 2);
	//reg_gain = reg_gain & 0xFFFF;

	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/*
	* sensor gain 1x = 128
	* max gain = 0x7ff = 15.992x <16x
	* here we just use 0x3508 analog gain 1 bit[3:2].
	* 16x~32x should use 0x3508 analog gain 0 bit[1:0]
	*/

	if (gain < BASEGAIN || gain >= 8 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain >= 8 * BASEGAIN)
			gain = 8* BASEGAIN;
	}

	reg_gain = gain2reg(gain);

	//reg_gain = (uint16_t)(256/((gain/2)+16)); //sensor gain base 1x= 16, reg_gain = gain/(64*16);
        //reg_gain =  (uint16_t)((512/gain)-(292/10));
	//reg_gain &= 0x3ff;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x03, 0x20);
	//write_cmos_sensor(0x60, reg_gain&0xff);
	write_cmos_sensor(0x60, reg_gain>>1);
	//write_cmos_sensor(0x61, reg_gain>>8);
    write_cmos_sensor(0x61, reg_gain&0x01);

	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("Warining:Do not supportIHDR, Return. le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	//return;

	/*if (imgsensor.ihdr_en) {

		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
		*/
	//rampart
	/*
				// Extend frame length first
				write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
*/
		//set_gain(gain);
	}

//Gionee: malp modify for  CRXXXXXX begin
#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/

	switch (image_mirror) {
		case IMAGE_NORMAL:
		//	write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
		//	write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			write_cmos_sensor(0x03,0x00);
			write_cmos_sensor(0x11,0x80); // v flip  //mirror 0x80,0x81,0x82,0x83

			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x03,0x00);
			write_cmos_sensor(0x11,0x81); // v flip  //mirror 0x80,0x81,0x82,0x83

			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x03,0x00);
			write_cmos_sensor(0x11,0x82); // v flip  //mirror 0x80,0x81,0x82,0x83
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x03,0x00);
			write_cmos_sensor(0x11,0x83); // v flip  //mirror 0x80,0x81,0x82,0x83

			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}
#endif
//Gionee: malp modify for  CRXXXXXX end
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

	/*	preview_setting  */

static void sensor_init(void)
{
	LOG_INF("sensor init_E\n");

 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x01);		// sleep on
 write_cmos_sensor(0x01, 0x03);		// soft reset
 write_cmos_sensor(0x01, 0x01);		// disable Reset

//////////////////
// BGR enable
//////////////////
 write_cmos_sensor(0x03, 0x02);
 write_cmos_sensor(0x1f, 0x01);  //bgr_en

//////////////////
// PLL Setting
//////////////////
// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
 write_cmos_sensor(0x08, 0x62);  // pll main div : 4x ~ 127x
 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
 write_cmos_sensor(0x07, 0xc5);  // pll enable & pre div setting

//////////////////
// One Line = 1800 !!!
//////////////////

//////////////////
// Page 0x00
//////////////////
 write_cmos_sensor(0x03	,0x00);
 write_cmos_sensor(0x10	,0x00);
 //Gionee: malp <2016-05-24> modify for CR01703897 bigin
#ifdef ORIGINAL_VERSION
 write_cmos_sensor(0x11 ,0x83); // v flip
#else
 write_cmos_sensor(0x11	,0x80); // v flip  //mirror 0x80,0x81,0x82,0x83
#endif
 //Gionee: malp <2016-05-24> modify for CR01703897 end
 write_cmos_sensor(0x13	,0x00);
 write_cmos_sensor(0x14	,0x20);
 write_cmos_sensor(0x15	,0x80);
 write_cmos_sensor(0x17	,0x10);
 write_cmos_sensor(0x1a	,0x00);
 write_cmos_sensor(0x1c	,0x00);
 write_cmos_sensor(0x1f	,0x00);
 write_cmos_sensor(0x20	,0x00);
 write_cmos_sensor(0x21	,0x00);		//20150127
 write_cmos_sensor(0x22	,0x00);		//20150127
 write_cmos_sensor(0x23	,0x00);		//20150127
 write_cmos_sensor(0x24	,0x00);		//20150127
 write_cmos_sensor(0x25	,0x00);		//20150127
 write_cmos_sensor(0x26	,0x00);		//20150127
 write_cmos_sensor(0x27	,0x00);		//20150127
 write_cmos_sensor(0x28	,0x04);		//20150127
 write_cmos_sensor(0x29	,0xcc);		//20150127
 write_cmos_sensor(0x2a	,0x06);		//20150127
 write_cmos_sensor(0x2b	,0x60);		//20150127
 write_cmos_sensor(0x30	,0x00);
 write_cmos_sensor(0x31	,0x00);
 write_cmos_sensor(0x32	,0x00);
 write_cmos_sensor(0x33	,0x00);
 write_cmos_sensor(0x34	,0x00);
 write_cmos_sensor(0x35	,0x00);
 write_cmos_sensor(0x36	,0x00);
 write_cmos_sensor(0x37	,0x00);
 write_cmos_sensor(0x38	,0x02);
 write_cmos_sensor(0x39	,0x66);
 write_cmos_sensor(0x3a	,0x03);
 write_cmos_sensor(0x3b	,0x30);
 write_cmos_sensor(0x4c	,0x07);
 write_cmos_sensor(0x4d	,0x08);
 write_cmos_sensor(0x4e	,0x04); // 30fps set 0x50c -> 0x534 20150402
 write_cmos_sensor(0x4f	,0xd4);
 write_cmos_sensor(0x54	,0x02);
 write_cmos_sensor(0x55	,0x03);
 write_cmos_sensor(0x56	,0x04);
 write_cmos_sensor(0x57	,0x40);
 write_cmos_sensor(0x58	,0x03);
 write_cmos_sensor(0x5c	,0x0a);
 write_cmos_sensor(0x60	,0x00);
 write_cmos_sensor(0x61	,0x00);
 write_cmos_sensor(0x62	,0x80);
 write_cmos_sensor(0x68	,0x03);
 write_cmos_sensor(0x69	,0x42);
 write_cmos_sensor(0x80	,0x00);		//20150127
 write_cmos_sensor(0x81	,0x00);		//20150127
 write_cmos_sensor(0x82	,0x00);		//20150127
 write_cmos_sensor(0x83	,0x00);		//20150127
 write_cmos_sensor(0x84	,0x06);		//20150127
 write_cmos_sensor(0x85	,0x60);		//20150127
 write_cmos_sensor(0x86	,0x04);		//20150127
 write_cmos_sensor(0x87	,0xcc);		//20150127
 write_cmos_sensor(0x88	,0x00);
 write_cmos_sensor(0x89	,0x00);
 write_cmos_sensor(0x8a	,0x02);
 write_cmos_sensor(0x8b	,0x66);
 write_cmos_sensor(0x90	,0x00);
 write_cmos_sensor(0x91	,0x02);
 write_cmos_sensor(0xa0	,0x01);
 write_cmos_sensor(0xa1	,0x40);
 write_cmos_sensor(0xa2	,0x40);
 write_cmos_sensor(0xa3	,0x40);
 write_cmos_sensor(0xa4	,0x40);
 write_cmos_sensor(0xe4	,0x10);
 write_cmos_sensor(0xe5	,0x00);

//////////////////
// Page 0x01
//////////////////
 write_cmos_sensor(0x03	,0x01);
 write_cmos_sensor(0x10	,0x21); //2014.08.20.a
 write_cmos_sensor(0x11   ,0x00);	//2014.10.29.a  disable blc_ofs //2014.10.23.a  en_color_blc_ofs
 write_cmos_sensor(0x12   ,0x3f); //2014.10.23.a  adaptive region enable
 write_cmos_sensor(0x13	,0x08); //2014.08.25.a
 write_cmos_sensor(0x14	,0x04); //2014.08.20.a
 write_cmos_sensor(0x15	,0x01); //2014.08.25.a
 write_cmos_sensor(0x16	,0x00);
 write_cmos_sensor(0x17	,0x02);	//2015.02.17
 write_cmos_sensor(0x18	,0x00);	//2014.10.29.a disable adaptive blc_ofs //2014.10.23.a  float offset enable, adaptive offset enable
 write_cmos_sensor(0x19	,0x00);

 write_cmos_sensor(0x20	,0x60);
 write_cmos_sensor(0x21	,0x00);
 write_cmos_sensor(0x22	,0x20);
 write_cmos_sensor(0x23	,0x3c);

 write_cmos_sensor(0x24	,0x5c);
 write_cmos_sensor(0x25	,0x00);
 write_cmos_sensor(0x26	,0x60);
 write_cmos_sensor(0x27	,0x07);
 write_cmos_sensor(0x28	,0x80);
 write_cmos_sensor(0x29	,0x00);
 write_cmos_sensor(0x2a	,0xff);
 write_cmos_sensor(0x2b	,0x02);	//2015.02.17
 write_cmos_sensor(0x2c	,0x80);
 write_cmos_sensor(0x2d	,0x80);
 write_cmos_sensor(0x2e	,0x80);
 write_cmos_sensor(0x2f	,0x80);
///////////////////////////////////
// 2014.10.23 added
///////////////////////////////////
 write_cmos_sensor(0x30, 0x7b);		//blc adaptive time region setting
 write_cmos_sensor(0x31, 0x01);
 write_cmos_sensor(0x32, 0xfe);
 write_cmos_sensor(0x33, 0x00);
 write_cmos_sensor(0x34, 0x51);
 write_cmos_sensor(0x35, 0xb6);
 write_cmos_sensor(0x36, 0xfc);

 write_cmos_sensor(0x38, 0x66);		//man blc float
 write_cmos_sensor(0x39, 0x66);

 write_cmos_sensor(0x40, 0x00);		//man blc integer
 write_cmos_sensor(0x41, 0x00);
 write_cmos_sensor(0x42, 0x00);
 write_cmos_sensor(0x43, 0x00);
 write_cmos_sensor(0x44, 0x00);
 write_cmos_sensor(0x45, 0x00);

 write_cmos_sensor(0x48, 0x00);
 write_cmos_sensor(0x49, 0x00);
 write_cmos_sensor(0x4a, 0x00);
 write_cmos_sensor(0x4b, 0x00);
 write_cmos_sensor(0x4c, 0x00);
 write_cmos_sensor(0x4d, 0x00);

 write_cmos_sensor(0x50, 0x00);
 write_cmos_sensor(0x51, 0x00);
 write_cmos_sensor(0x52, 0x00);
 write_cmos_sensor(0x53, 0x00);
 write_cmos_sensor(0x54, 0x00);
 write_cmos_sensor(0x55, 0x00);

 write_cmos_sensor(0x58, 0x00);
 write_cmos_sensor(0x59, 0x00);
 write_cmos_sensor(0x5a, 0x00);
 write_cmos_sensor(0x5b, 0x00);
 write_cmos_sensor(0x5c, 0x00);
 write_cmos_sensor(0x5d, 0x00);
/////////////////////////////////////
 write_cmos_sensor(0x80	,0x00);
 write_cmos_sensor(0x81	,0x00);
 write_cmos_sensor(0x82	,0x00);
 write_cmos_sensor(0x83	,0x00);
 write_cmos_sensor(0x88	,0x20); //2014.08.19.b
 write_cmos_sensor(0x8a	,0x30); //2014.08.19.b
 write_cmos_sensor(0x8c	,0x00);
 write_cmos_sensor(0x90	,0x00);
 write_cmos_sensor(0x91	,0x60);
 write_cmos_sensor(0x92	,0x00);
 write_cmos_sensor(0x93	,0x60);

//////////////////
// Page 0x02
//////////////////
 write_cmos_sensor(0x03	,0x02);
 write_cmos_sensor(0x10	,0x00);
 write_cmos_sensor(0x11	,0x00);
 write_cmos_sensor(0x12	,0x40); //2015.02.03
 write_cmos_sensor(0x13	,0x00);
 write_cmos_sensor(0x16	,0x00);
 write_cmos_sensor(0x17	,0x00);
 write_cmos_sensor(0x19	,0x00);
 write_cmos_sensor(0x1a	,0x10); //2014.10.23.a adapt. NCP on
 write_cmos_sensor(0x1b	,0x00);
 write_cmos_sensor(0x1c	,0xc0);
 write_cmos_sensor(0x1d	,0x20);
 write_cmos_sensor(0x20	,0x04);
 write_cmos_sensor(0x21	,0x04);
 write_cmos_sensor(0x22	,0x06); //2015.02.03
 write_cmos_sensor(0x23	,0x10);
 write_cmos_sensor(0x24	,0x04);
 write_cmos_sensor(0x28	,0x00);
 write_cmos_sensor(0x29	,0x06); //2015.02.03
 write_cmos_sensor(0x2a	,0x00);
 write_cmos_sensor(0x2e	,0x00);
 write_cmos_sensor(0x2f	,0x2c);
 write_cmos_sensor(0x30	,0x00);
 write_cmos_sensor(0x31	,0x44);
 write_cmos_sensor(0x32	,0x02);
 write_cmos_sensor(0x33	,0x00);
 write_cmos_sensor(0x34	,0x00);
 write_cmos_sensor(0x35	,0x00);
 write_cmos_sensor(0x36	,0x06);
 write_cmos_sensor(0x37	,0xc0);
 write_cmos_sensor(0x38	,0x00);
 write_cmos_sensor(0x39	,0x32);
 write_cmos_sensor(0x3a	,0x02);
 write_cmos_sensor(0x3b	,0x04);
 write_cmos_sensor(0x3c	,0x04);
 write_cmos_sensor(0x3d	,0xfe);
 write_cmos_sensor(0x3e	,0x00);
 write_cmos_sensor(0x3f	,0x00);
 write_cmos_sensor(0x40	,0x00);
 write_cmos_sensor(0x41	,0x17); //2015.02.03
 write_cmos_sensor(0x42	,0x01);
 write_cmos_sensor(0x43	,0x25);
 write_cmos_sensor(0x47	,0x00);
 write_cmos_sensor(0x48	,0x9a);
 write_cmos_sensor(0x49	,0x24);
 write_cmos_sensor(0x4a	,0x0f);
 write_cmos_sensor(0x4b	,0x20);
 write_cmos_sensor(0x4c	,0x06);
 write_cmos_sensor(0x4d	,0xc0);
 write_cmos_sensor(0x50	,0xa9);
 write_cmos_sensor(0x51	,0x1c);
 write_cmos_sensor(0x52	,0x73);
 write_cmos_sensor(0x54	,0xc0);
 write_cmos_sensor(0x55	,0x40);
 write_cmos_sensor(0x56	,0x11);
 write_cmos_sensor(0x57	,0x00);
 write_cmos_sensor(0x58	,0x18);
 write_cmos_sensor(0x59	,0x16);
 write_cmos_sensor(0x5b	,0x00);
 write_cmos_sensor(0x62	,0x00);
 write_cmos_sensor(0x63	,0xc8);
 write_cmos_sensor(0x67	,0x3f);
 write_cmos_sensor(0x68	,0xc0);
 write_cmos_sensor(0x70	,0x03);
 write_cmos_sensor(0x71	,0xc7);
 write_cmos_sensor(0x72	,0x06); //linemem_pattern_test
 write_cmos_sensor(0x73	,0x75); //2014.10.23.a //2014.08.25.a
 write_cmos_sensor(0x74	,0x03);
 write_cmos_sensor(0x75	,0xc7);
 write_cmos_sensor(0x76	,0x05);
 write_cmos_sensor(0x77	,0x1d); //2014.10.23.a //2014.08.25.a
 write_cmos_sensor(0xa0	,0x01);
 write_cmos_sensor(0xa1	,0x48);
 write_cmos_sensor(0xa2	,0x02);
 write_cmos_sensor(0xa3	,0xde);
 write_cmos_sensor(0xa4	,0x02);
 write_cmos_sensor(0xa5	,0xde);
 write_cmos_sensor(0xa6	,0x06);
 write_cmos_sensor(0xa7	,0xf0);
 write_cmos_sensor(0xb0	,0x02);
 write_cmos_sensor(0xb1	,0x0f);
 write_cmos_sensor(0xb2	,0x02);
 write_cmos_sensor(0xb3	,0xdb); //2014.08.18 ramp_clk_msk_off1_1x_l
 write_cmos_sensor(0xb4	,0x03);
 write_cmos_sensor(0xb5	,0xc7);
 write_cmos_sensor(0xb6	,0x06);
 write_cmos_sensor(0xb7	,0xd7); //2014.08.18 ramp_clk_msk_off2_1x_l
 write_cmos_sensor(0xc0	,0x02);
 write_cmos_sensor(0xc1	,0x0f);
 write_cmos_sensor(0xc2	,0x02);
 write_cmos_sensor(0xc3	,0xdd);
 write_cmos_sensor(0xc4	,0x03);
 write_cmos_sensor(0xc5	,0xc7);
 write_cmos_sensor(0xc6	,0x06);
 write_cmos_sensor(0xc7	,0xdb); //2014.08.18 ramp_preset_off2_1x_l
 write_cmos_sensor(0xc8	,0x01);
 write_cmos_sensor(0xc9	,0x8e);
 write_cmos_sensor(0xca	,0x01);
 write_cmos_sensor(0xcb	,0x3e);
 write_cmos_sensor(0xcc	,0x03);
 write_cmos_sensor(0xcd	,0x1e);
 write_cmos_sensor(0xce	,0x02);
 write_cmos_sensor(0xcf	,0xe2);
 write_cmos_sensor(0xd0	,0x00); //2014.10.23.a
 write_cmos_sensor(0xd1	,0x00); //2014.10.23.a
 write_cmos_sensor(0xd2	,0x00); //2014.10.23.a
 write_cmos_sensor(0xd3	,0x00); //2014.10.23.a
 write_cmos_sensor(0xd4	,0x0c); //2014.10.23.a
 write_cmos_sensor(0xd5	,0x00); //2014.10.23.a
 write_cmos_sensor(0xe0	,0x1c); //2014.10.23.a  NCP setting
 write_cmos_sensor(0xe1	,0x1c); //2014.10.23.a
 write_cmos_sensor(0xe2	,0x1c); //2014.10.23.a
 write_cmos_sensor(0xe3	,0x04); //2014.10.23.a
 write_cmos_sensor(0xe4	,0x1c); //2014.10.23.a
 write_cmos_sensor(0xe5	,0x01);
 write_cmos_sensor(0xe8	,0x00);
 write_cmos_sensor(0xe9	,0x00);
 write_cmos_sensor(0xea	,0x00); //2014.10.23.a
 write_cmos_sensor(0xeb	,0x00); //2014.10.23.a
 write_cmos_sensor(0xec	,0x00); //2014.10.23.a
 write_cmos_sensor(0xed	,0x00); //2014.10.23.a

 write_cmos_sensor(0xf0	,0x70);
 write_cmos_sensor(0xf1	,0x00);
 write_cmos_sensor(0xf2	,0x82);
 write_cmos_sensor(0xf3	,0x00);

//////////////////
// Page 0x03
//////////////////
 write_cmos_sensor(0x03	,0x03);
 write_cmos_sensor(0x10	,0x00);
 write_cmos_sensor(0x11	,0x80);
 write_cmos_sensor(0x12	,0x00);
 write_cmos_sensor(0x13	,0x02);
 write_cmos_sensor(0x14	,0x06);
 write_cmos_sensor(0x15	,0xeb);
 write_cmos_sensor(0x16	,0x06);
 write_cmos_sensor(0x17	,0xf5);
 write_cmos_sensor(0x18	,0x01);
 write_cmos_sensor(0x19	,0x02);
 write_cmos_sensor(0x1a	,0x06);
 write_cmos_sensor(0x1b	,0xda);
 write_cmos_sensor(0x1c	,0x01);
 write_cmos_sensor(0x1d	,0x02);
 write_cmos_sensor(0x1e	,0x06);
 write_cmos_sensor(0x1f	,0xda);
 write_cmos_sensor(0x20	,0x01);
 write_cmos_sensor(0x21	,0x02);
 write_cmos_sensor(0x22	,0x01);
 write_cmos_sensor(0x23	,0x34);
 write_cmos_sensor(0x24	,0x01);
 write_cmos_sensor(0x25	,0x02);
 write_cmos_sensor(0x26	,0xff); //2014.10.23.a
 write_cmos_sensor(0x27	,0xff); //2014.10.23.a
 write_cmos_sensor(0x28	,0x01);
 write_cmos_sensor(0x29	,0x02);
 write_cmos_sensor(0x2a	,0x01);
 write_cmos_sensor(0x2b	,0x3e);
 write_cmos_sensor(0x2c	,0x01);
 write_cmos_sensor(0x2d	,0x02);
 write_cmos_sensor(0x2e	,0x01);
 write_cmos_sensor(0x2f	,0x3e);
 write_cmos_sensor(0x30	,0x01);
 write_cmos_sensor(0x31	,0x48);
 write_cmos_sensor(0x32	,0x06);
 write_cmos_sensor(0x33	,0xda);
 write_cmos_sensor(0x34	,0x01);
 write_cmos_sensor(0x35	,0x48);
 write_cmos_sensor(0x36	,0x06);
 write_cmos_sensor(0x37	,0xda);
 write_cmos_sensor(0x38	,0x06);
 write_cmos_sensor(0x39	,0xf1);
 write_cmos_sensor(0x3a	,0x06);
 write_cmos_sensor(0x3b	,0xfb);
 write_cmos_sensor(0x3c	,0x00);
 write_cmos_sensor(0x3d	,0x0c);
 write_cmos_sensor(0x3e	,0x00);
 write_cmos_sensor(0x3f	,0x16);
 write_cmos_sensor(0x40	,0x00);
 write_cmos_sensor(0x41	,0x04);
 write_cmos_sensor(0x42	,0x00);
 write_cmos_sensor(0x43	,0x45);
 write_cmos_sensor(0x44	,0x00);
 write_cmos_sensor(0x45	,0x02);
 write_cmos_sensor(0x46	,0x00);
 write_cmos_sensor(0x47	,0x74);
 write_cmos_sensor(0x48	,0x00);
 write_cmos_sensor(0x49	,0x06);
 write_cmos_sensor(0x4a	,0x00);
 write_cmos_sensor(0x4b	,0x42);
 write_cmos_sensor(0x4c	,0x00);
 write_cmos_sensor(0x4d	,0x06);
 write_cmos_sensor(0x4e	,0x00);
 write_cmos_sensor(0x4f	,0x42);
 write_cmos_sensor(0x50	,0x00);
 write_cmos_sensor(0x51	,0x0a);
 write_cmos_sensor(0x52	,0x00);
 write_cmos_sensor(0x53	,0x32);
 write_cmos_sensor(0x54	,0x00);
 write_cmos_sensor(0x55	,0x0a);
 write_cmos_sensor(0x56	,0x00);
 write_cmos_sensor(0x57	,0x32);
 write_cmos_sensor(0x58	,0x00);
 write_cmos_sensor(0x59	,0x0a);
 write_cmos_sensor(0x5a	,0x00);
 write_cmos_sensor(0x5b	,0x32);
 write_cmos_sensor(0x60	,0x00);
 write_cmos_sensor(0x61	,0x04);
 write_cmos_sensor(0x62	,0x00);
 write_cmos_sensor(0x63	,0x12);
 write_cmos_sensor(0x64	,0x00);
 write_cmos_sensor(0x65	,0x04);
 write_cmos_sensor(0x66	,0x00);
 write_cmos_sensor(0x67	,0x12);
 write_cmos_sensor(0x68	,0x00);
 write_cmos_sensor(0x69	,0x04);
 write_cmos_sensor(0x6a	,0x00);
 write_cmos_sensor(0x6b	,0x4a);
 write_cmos_sensor(0x70	,0x00);
 write_cmos_sensor(0x71	,0xda);
 write_cmos_sensor(0x72	,0x06);
 write_cmos_sensor(0x73	,0xe0);
 write_cmos_sensor(0x74	,0x00);
 write_cmos_sensor(0x75	,0xe0);
 write_cmos_sensor(0x76	,0x00);
 write_cmos_sensor(0x77	,0xfc);
 write_cmos_sensor(0x78	,0x00);
 write_cmos_sensor(0x79	,0xe0);
 write_cmos_sensor(0x7a	,0x00);
 write_cmos_sensor(0x7b	,0xfc);
 write_cmos_sensor(0x7c	,0x06);
 write_cmos_sensor(0x7d	,0xdc);
 write_cmos_sensor(0x7e	,0x06);
 write_cmos_sensor(0x7f	,0xe4);
 write_cmos_sensor(0x80	,0x02);
 write_cmos_sensor(0x81	,0xe2);
 write_cmos_sensor(0x82	,0x03);
 write_cmos_sensor(0x83	,0x28);
 write_cmos_sensor(0x84	,0x02);
 write_cmos_sensor(0x85	,0xe2);
 write_cmos_sensor(0x86	,0x03);
 write_cmos_sensor(0x87	,0x28);
 write_cmos_sensor(0x88	,0x06);
 write_cmos_sensor(0x89	,0xdc);
 write_cmos_sensor(0x8a	,0x06);
 write_cmos_sensor(0x8b	,0xe4);
 write_cmos_sensor(0x90	,0x00);
 write_cmos_sensor(0x91	,0xd9);
 write_cmos_sensor(0x92	,0x06);
 write_cmos_sensor(0x93	,0xd9);
 write_cmos_sensor(0x94	,0x00);
 write_cmos_sensor(0x95	,0xd9);
 write_cmos_sensor(0x96	,0x06);
 write_cmos_sensor(0x97	,0xd9);
 write_cmos_sensor(0x98	,0x06);
 write_cmos_sensor(0x99	,0xd9);
 write_cmos_sensor(0x9a	,0x00);
 write_cmos_sensor(0x9b	,0xd9);
 write_cmos_sensor(0x9c	,0x06);
 write_cmos_sensor(0x9d	,0xd9);
 write_cmos_sensor(0x9e	,0x00);
 write_cmos_sensor(0x9f	,0xd9);
 write_cmos_sensor(0xa0	,0x00);
 write_cmos_sensor(0xa1	,0x05);
 write_cmos_sensor(0xa2	,0x00);
 write_cmos_sensor(0xa3	,0x13);
 write_cmos_sensor(0xa4	,0x00);
 write_cmos_sensor(0xa5	,0x05);
 write_cmos_sensor(0xa6	,0x00);
 write_cmos_sensor(0xa7	,0x13);
 write_cmos_sensor(0xa8	,0x00);
 write_cmos_sensor(0xa9	,0xde);
 write_cmos_sensor(0xaa	,0x00);
 write_cmos_sensor(0xab	,0xee);
 write_cmos_sensor(0xac	,0x00);
 write_cmos_sensor(0xad	,0xdd);
 write_cmos_sensor(0xae	,0x00);
 write_cmos_sensor(0xaf	,0xed);
 write_cmos_sensor(0xb0	,0x06);
 write_cmos_sensor(0xb1	,0xe1);
 write_cmos_sensor(0xb2	,0x00);
 write_cmos_sensor(0xb3	,0xff);
 write_cmos_sensor(0xb4	,0x06);
 write_cmos_sensor(0xb5	,0xe1);
 write_cmos_sensor(0xb6	,0x00);
 write_cmos_sensor(0xb7	,0xff);
 write_cmos_sensor(0xe0	,0x00);
 write_cmos_sensor(0xe1	,0xda);
 write_cmos_sensor(0xe2	,0x02);
 write_cmos_sensor(0xe3	,0xde);
 write_cmos_sensor(0xe4	,0x03);
 write_cmos_sensor(0xe5	,0x05);
 write_cmos_sensor(0xe6	,0x06);
 write_cmos_sensor(0xe7	,0xda);
 write_cmos_sensor(0xe8	,0x01);
 write_cmos_sensor(0xe9	,0x00);
 write_cmos_sensor(0xea	,0x02);
 write_cmos_sensor(0xeb	,0xde);
 write_cmos_sensor(0xec	,0x06);
 write_cmos_sensor(0xed	,0xfc);
 write_cmos_sensor(0xee	,0x00);
 write_cmos_sensor(0xef	,0x00);
 write_cmos_sensor(0xf6	,0x00);
 write_cmos_sensor(0xf7	,0x12);
 write_cmos_sensor(0xf8	,0x00);
 write_cmos_sensor(0xf9	,0x1c);

//////////////////
// Page 0x04
//////////////////
 write_cmos_sensor(0x03	,0x04);
 write_cmos_sensor(0x10	,0x02); //2014.10.23.a ramp_dn manual
 write_cmos_sensor(0x11	,0x04); //2015.02.03
 write_cmos_sensor(0x12	,0x00);
 write_cmos_sensor(0x13	,0x00);
 write_cmos_sensor(0x14	,0x02);
 write_cmos_sensor(0x1a	,0x00);
 write_cmos_sensor(0x1b	,0x30);
 write_cmos_sensor(0x1c	,0x00);
 write_cmos_sensor(0x1d	,0xc0);
 write_cmos_sensor(0x1e	,0x44); //2014.08.18 ramp_hold_count
 write_cmos_sensor(0x20	,0x00);
 write_cmos_sensor(0x21	,0x38);
 write_cmos_sensor(0x22	,0x00);
 write_cmos_sensor(0x23	,0x70);
 write_cmos_sensor(0x24	,0x00);
 write_cmos_sensor(0x25	,0xa8);
 write_cmos_sensor(0x26	,0x00);
 write_cmos_sensor(0x27	,0xc5);
 write_cmos_sensor(0x28	,0x01);
 write_cmos_sensor(0x29	,0x8a);
 write_cmos_sensor(0x2a	,0x02);
 write_cmos_sensor(0x2b	,0x4f);
 write_cmos_sensor(0x30	,0x01); //2014.10.23.a cds_s2
 write_cmos_sensor(0x31	,0x3c); //
 write_cmos_sensor(0x32	,0x01); //
 write_cmos_sensor(0x33	,0x3c); //
 write_cmos_sensor(0x34	,0x01); //
 write_cmos_sensor(0x35	,0x34); //
 write_cmos_sensor(0x36	,0x01); //
 write_cmos_sensor(0x37	,0x02); //
 write_cmos_sensor(0x38	,0x01); //
 write_cmos_sensor(0x39	,0x02); //
 write_cmos_sensor(0x3a	,0x01); //
 write_cmos_sensor(0x3b	,0x02); //
 write_cmos_sensor(0x40	,0x01); //2014.10.23.a cds_s3
 write_cmos_sensor(0x41	,0x3e); //
 write_cmos_sensor(0x42	,0x01); //
 write_cmos_sensor(0x43	,0x3e); //
 write_cmos_sensor(0x44	,0x01); //
 write_cmos_sensor(0x45	,0x3e); //
 write_cmos_sensor(0x46	,0x01); //
 write_cmos_sensor(0x47	,0x02); //
 write_cmos_sensor(0x48	,0x01); //
 write_cmos_sensor(0x49	,0x02); //
 write_cmos_sensor(0x4a	,0x01); //
 write_cmos_sensor(0x4b	,0x02); //
 write_cmos_sensor(0x50	,0x00);
 write_cmos_sensor(0x58	,0x00);
 write_cmos_sensor(0x59	,0xdc);
 write_cmos_sensor(0x5a	,0x06);
 write_cmos_sensor(0x5b	,0xe0);
 write_cmos_sensor(0x5c	,0x00);
 write_cmos_sensor(0x5d	,0xdc);
 write_cmos_sensor(0x5e	,0x06);
 write_cmos_sensor(0x5f	,0xe0);
 write_cmos_sensor(0x60	,0x00);
 write_cmos_sensor(0x61	,0x60);
 write_cmos_sensor(0x62	,0x00);
 write_cmos_sensor(0x63	,0x40);
 write_cmos_sensor(0x64	,0x00);
 write_cmos_sensor(0x65	,0x60);
 write_cmos_sensor(0x66	,0x00);
 write_cmos_sensor(0x67	,0x40);
 write_cmos_sensor(0x68	,0x00);
 write_cmos_sensor(0x69	,0x60);
 write_cmos_sensor(0x6a	,0x00);
 write_cmos_sensor(0x6b	,0x40);
 write_cmos_sensor(0x70	,0x18); //2014.10.23.a analog_adapt_ctl
 write_cmos_sensor(0x71	,0x20); //2014.10.23.a clamp
 write_cmos_sensor(0x72	,0x20); //
 write_cmos_sensor(0x73	,0x00); //
 write_cmos_sensor(0x80	,0x70); //2015.02.03
 write_cmos_sensor(0x81	,0x00);
 write_cmos_sensor(0x82	,0x30); //2015.02.03
 write_cmos_sensor(0x83	,0x00);
 write_cmos_sensor(0x84	,0x10); //2015.02.03
 write_cmos_sensor(0x85	,0x00); //2015.02.03
 write_cmos_sensor(0x86	,0x00);
 write_cmos_sensor(0x87	,0x00);
 write_cmos_sensor(0x90	,0x03);
 write_cmos_sensor(0x91	,0x06);
 write_cmos_sensor(0x92	,0x06);
 write_cmos_sensor(0x93	,0x06);
 write_cmos_sensor(0x94	,0x06);
 write_cmos_sensor(0x95	,0x00); //2015.02.03
 write_cmos_sensor(0x96	,0x40); //2015.02.03
 write_cmos_sensor(0x97	,0x50); //2015.02.03
 write_cmos_sensor(0x98	,0x70); //2015.02.03
 write_cmos_sensor(0xa0	,0x06); //2014.10.23.a cds_s1
 write_cmos_sensor(0xa1	,0xda); //
 write_cmos_sensor(0xa2	,0x06); //
 write_cmos_sensor(0xa3	,0xda); //
 write_cmos_sensor(0xa4	,0x06); //
 write_cmos_sensor(0xa5	,0xda); //
 write_cmos_sensor(0xa6	,0x01); //
 write_cmos_sensor(0xa7	,0x02); //
 write_cmos_sensor(0xa8	,0x01); //
 write_cmos_sensor(0xa9	,0x02); //
 write_cmos_sensor(0xaa	,0x01); //
 write_cmos_sensor(0xab	,0x02); //

 write_cmos_sensor(0xb0	,0x04);
 write_cmos_sensor(0xb1	,0x04);
 write_cmos_sensor(0xb2	,0x00);
 write_cmos_sensor(0xb3	,0x04);
 write_cmos_sensor(0xb4	,0x00);
 write_cmos_sensor(0xc0	,0x00);
 write_cmos_sensor(0xc1	,0x48);
 write_cmos_sensor(0xc2	,0x00);
 write_cmos_sensor(0xc3	,0x6e);
 write_cmos_sensor(0xc4	,0x00);
 write_cmos_sensor(0xc5	,0x4d);
 write_cmos_sensor(0xc6	,0x00);
 write_cmos_sensor(0xc7	,0x6c);
 write_cmos_sensor(0xc8	,0x00);
 write_cmos_sensor(0xc9	,0x4f);
 write_cmos_sensor(0xca	,0x00);
 write_cmos_sensor(0xcb	,0x6a);
 write_cmos_sensor(0xcc	,0x00);
 write_cmos_sensor(0xcd	,0x50);
 write_cmos_sensor(0xce	,0x00);
 write_cmos_sensor(0xcf	,0x68);
 write_cmos_sensor(0xd0	,0x07);
 write_cmos_sensor(0xd1	,0x00);
 write_cmos_sensor(0xd2	,0x03);
 write_cmos_sensor(0xd3	,0x03);
 write_cmos_sensor(0xe0	,0x00);
 write_cmos_sensor(0xe1	,0x10);
 write_cmos_sensor(0xe2	,0x67);
 write_cmos_sensor(0xe3	,0x00);

//////////////////
// Page 0x08
//////////////////
 write_cmos_sensor(0x03	,0x08);
 write_cmos_sensor(0x10	,0x03);
 write_cmos_sensor(0x20	,0x01);
 write_cmos_sensor(0x21	,0x00);
 write_cmos_sensor(0x22	,0x01);
 write_cmos_sensor(0x23	,0x00);
 write_cmos_sensor(0x24	,0x01);
 write_cmos_sensor(0x25	,0x00);
 write_cmos_sensor(0x26	,0x01);
 write_cmos_sensor(0x27	,0x00);
 write_cmos_sensor(0x28	,0x01);
 write_cmos_sensor(0x29	,0x00);
 write_cmos_sensor(0x2a	,0x01);
 write_cmos_sensor(0x2b	,0x00);
 write_cmos_sensor(0x2c	,0x01);
 write_cmos_sensor(0x2d	,0x00);
 write_cmos_sensor(0x2e	,0x01);
 write_cmos_sensor(0x2f	,0x00);
 write_cmos_sensor(0x30	,0x03);
 write_cmos_sensor(0x31	,0xff);
 write_cmos_sensor(0x32	,0x03);
 write_cmos_sensor(0x33	,0xff);
 write_cmos_sensor(0x34	,0x03);
 write_cmos_sensor(0x35	,0xff);
 write_cmos_sensor(0x36	,0x03);
 write_cmos_sensor(0x37	,0xff);
 write_cmos_sensor(0x40	,0x03);
 write_cmos_sensor(0x50	,0x01);
 write_cmos_sensor(0x51	,0x00);
 write_cmos_sensor(0x52	,0x01);
 write_cmos_sensor(0x53	,0x00);
 write_cmos_sensor(0x54	,0x0f);
 write_cmos_sensor(0x55	,0xff);

//////////////////
// Page 0x10
//////////////////
 write_cmos_sensor(0x03	,0x10);
 write_cmos_sensor(0x10	,0x00);
 write_cmos_sensor(0x11	,0x00);

//////////////////
// Page 0x20
//////////////////
 write_cmos_sensor(0x03	,0x20);
 write_cmos_sensor(0x10	,0x00);
 write_cmos_sensor(0x11	,0x05);
 write_cmos_sensor(0x12	,0x00);
 write_cmos_sensor(0x22	,0x05); //2014.08.18 exptime_line_h
 write_cmos_sensor(0x23	,0x28); //2014.08.18 exptime_line_l
 write_cmos_sensor(0x26	,0xff);
 write_cmos_sensor(0x27	,0xff);
 write_cmos_sensor(0x29	,0x00);
 write_cmos_sensor(0x2a	,0x02);
 write_cmos_sensor(0x2b	,0x00);
 write_cmos_sensor(0x2c	,0x04);
 write_cmos_sensor(0x30	,0x00);
 write_cmos_sensor(0x31	,0x04);
/////////////////////////////
// 2014.10.23 added
/////////////////////////////
 write_cmos_sensor(0x40	,0x09); //bandtime pixel
 write_cmos_sensor(0x41	,0x1e);
 write_cmos_sensor(0x42	,0x60);
/////////////////////////////
 write_cmos_sensor(0x52	,0x0f);
 write_cmos_sensor(0x53	,0xf3);		//pga_max

 write_cmos_sensor(0x60, 0x0f);
 write_cmos_sensor(0x61, 0x00);

 write_cmos_sensor(0x64, 0x11);
 write_cmos_sensor(0x65, 0x01);		//ramp_gain_max

 //add 20150424
 //write_cmos_sensor(0x03, 0x08);
 //write_cmos_sensor(0x10, 0x07);
 //write_cmos_sensor(0x40, 0x07);

 //write_cmos_sensor(0x03, 0x20);
 //write_cmos_sensor(0x12, 0x03);
 //add 20150424





//////////////////
// MIPI Setting
//////////////////
 write_cmos_sensor(0x03, 0x05);  // mode_page
 //add 20150424
 write_cmos_sensor(0x39, 0x55);  // lvds_bias_ctl
 //write_cmos_sensor(0x39, 0x46);  // lvds_bias_ctl
 write_cmos_sensor(0x4c, 0x20);  // hs_wakeup_size_h
 write_cmos_sensor(0x4d, 0x00);  // hs_wakeup_size_l
 write_cmos_sensor(0x4e, 0x40);  // mipi_int_time_h
 write_cmos_sensor(0x4f, 0x00);  // mipi_int_time_l
 //add 20150424
 //write_cmos_sensor(0x14, 0x01);

 write_cmos_sensor(0x11, 0x00);  // lvds_ctl_2
 write_cmos_sensor(0x16, 0x02);  // lvds_inout_ctl1
 write_cmos_sensor(0x18, 0x80);  // lvds_inout_ctl3
 write_cmos_sensor(0x19, 0x00);  // lvds_inout_ctl4
 write_cmos_sensor(0x1a, 0xf0);  // lvds_time_ctl
 write_cmos_sensor(0x24, 0x2b);  // long_packet_id
// CLK Lane Timing Control //
 write_cmos_sensor(0x32, 0x1e);  // clk_zero_time
 write_cmos_sensor(0x33, 0x0f);  // clk_post_time
 write_cmos_sensor(0x34, 0x06);  // clk_prepare_time
 write_cmos_sensor(0x35, 0x05);  // clk_trail_time
 write_cmos_sensor(0x36, 0x01);  // clk_tlpx_time_dp
 write_cmos_sensor(0x37, 0x08);  // clk_tlpx_time_dn
// DATA Lane Timing Control //
 write_cmos_sensor(0x1c, 0x01);  // tlpx_time_l_dp
 write_cmos_sensor(0x1d, 0x09);  // tlpx_time_l_dn
 write_cmos_sensor(0x1e, 0x0f);  // hs_zero_time
 write_cmos_sensor(0x1f, 0x0b);  // hs_trail_time
 write_cmos_sensor(0x30, 0x07);  // l_pkt_wc_h   1600 * 5 / 4 = 2000  (MIPI Data Byte)
 write_cmos_sensor(0x31, 0xf8);  // l_pkt_wc_l   1600 * 5 / 4 = 2000  (MIPI Data Byte)

 write_cmos_sensor(0x10, 0x1d);  // lvds_ctl_1

/////////////////
// Sleep OFF
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off


}	/*	sensor_init  */

static void preview_setting(void)
{
	LOG_INF("preview_setting E\n");

  write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x01);		// sleep on

 //////////////////
// PLL Setting
//////////////////
// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
 write_cmos_sensor(0x08, 0x5a);  // pll main div : 4x ~ 127x
 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
 write_cmos_sensor(0x07, 0xC5);  // pll enable & pre div setting

 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x20	,0x00);
 write_cmos_sensor(0x21	,0x00);		//20150127
 write_cmos_sensor(0x22	,0x00);		//20150127
 write_cmos_sensor(0x23	,0x00);		//20150127
 write_cmos_sensor(0x24	,0x00);		//20150127
 write_cmos_sensor(0x25	,0x00);		//20150127
 write_cmos_sensor(0x26	,0x00);		//20150127
 write_cmos_sensor(0x27	,0x00);		//20150127
 write_cmos_sensor(0x28	,0x04);		//20150127
 write_cmos_sensor(0x29	,0xcc);		//20150127
 write_cmos_sensor(0x2a	,0x06);		//20150127
 write_cmos_sensor(0x2b	,0x60);		//20150127

 write_cmos_sensor(0x4c	,0x07);
 write_cmos_sensor(0x4d	,0x08);
 write_cmos_sensor(0x4e	,0x04); // 30fps set
 write_cmos_sensor(0x4f	,0xd4);

  write_cmos_sensor(0x80,0x00);		//20150127
 write_cmos_sensor(0x81	,0x00);		//20150127
 write_cmos_sensor(0x82	,0x00);		//20150127
 write_cmos_sensor(0x83	,0x00);		//20150127
 write_cmos_sensor(0x84	,0x06);		//20150127
 write_cmos_sensor(0x85	,0x60);		//20150127
 write_cmos_sensor(0x86	,0x04);		//20150127
 write_cmos_sensor(0x87	,0xcc);		//20150127

  write_cmos_sensor(0x03, 0x05);
  // CLK Lane Timing Control //
 write_cmos_sensor(0x32, 0x1e);  // clk_zero_time
 write_cmos_sensor(0x33, 0x0f);  // clk_post_time
 write_cmos_sensor(0x34, 0x06);  // clk_prepare_time
 write_cmos_sensor(0x35, 0x05);  // clk_trail_time
 write_cmos_sensor(0x36, 0x01);  // clk_tlpx_time_dp
 write_cmos_sensor(0x37, 0x08);  // clk_tlpx_time_dn
// DATA Lane Timing Control //
 write_cmos_sensor(0x1c, 0x01);  // tlpx_time_l_dp
 write_cmos_sensor(0x1d, 0x09);  // tlpx_time_l_dn
 write_cmos_sensor(0x1e, 0x0f);  // hs_zero_time
 write_cmos_sensor(0x1f, 0x0b);  // hs_trail_time
 write_cmos_sensor(0x30, 0x07);  // l_pkt_wc_h   1600 * 5 / 4 = 2000  (MIPI Data Byte)
 write_cmos_sensor(0x31, 0xf8);  // l_pkt_wc_l   1600 * 5 / 4 = 2000  (MIPI Data Byte)
 write_cmos_sensor(0x10, 0x1d);
  write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off

 mdelay(50);
}


static void capture_setting(kal_uint16 curretfps)
{
	LOG_INF("E! currefps:%d\n",curretfps);

  write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x01);		// sleep on

 //////////////////
// PLL Setting
//////////////////
// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
 write_cmos_sensor(0x08, 0x5a);  // pll main div : 4x ~ 127x
 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
 write_cmos_sensor(0x07, 0xC5);  // pll enable & pre div setting

 write_cmos_sensor(0x03, 0x00);
write_cmos_sensor(0x20	,0x00);
 write_cmos_sensor(0x21	,0x00);		//20150127
 write_cmos_sensor(0x22	,0x00);		//20150127
 write_cmos_sensor(0x23	,0x00);		//20150127
 write_cmos_sensor(0x24	,0x00);		//20150127
 write_cmos_sensor(0x25	,0x00);		//20150127
 write_cmos_sensor(0x26	,0x00);		//20150127
 write_cmos_sensor(0x27	,0x00);		//20150127
 write_cmos_sensor(0x28	,0x04);		//20150127
 write_cmos_sensor(0x29	,0xcc);		//20150127
 write_cmos_sensor(0x2a	,0x06);		//20150127
 write_cmos_sensor(0x2b	,0x60);		//20150127

 write_cmos_sensor(0x4c	,0x07);
 write_cmos_sensor(0x4d	,0x08);
 write_cmos_sensor(0x4e	,0x04); // 30fps set
 write_cmos_sensor(0x4f	,0xd4);

  write_cmos_sensor(0x80,0x00);		//20150127
 write_cmos_sensor(0x81	,0x00);		//20150127
 write_cmos_sensor(0x82	,0x00);		//20150127
 write_cmos_sensor(0x83	,0x00);		//20150127
 write_cmos_sensor(0x84	,0x06);		//20150127
 write_cmos_sensor(0x85	,0x60);		//20150127
 write_cmos_sensor(0x86	,0x04);		//20150127
 write_cmos_sensor(0x87	,0xcc);		//20150127

  write_cmos_sensor(0x03, 0x05);
  // CLK Lane Timing Control //
 write_cmos_sensor(0x32, 0x1e);  // clk_zero_time
 write_cmos_sensor(0x33, 0x0f);  // clk_post_time
 write_cmos_sensor(0x34, 0x06);  // clk_prepare_time
 write_cmos_sensor(0x35, 0x05);  // clk_trail_time
 write_cmos_sensor(0x36, 0x01);  // clk_tlpx_time_dp
 write_cmos_sensor(0x37, 0x08);  // clk_tlpx_time_dn
// DATA Lane Timing Control //
 write_cmos_sensor(0x1c, 0x01);  // tlpx_time_l_dp
 write_cmos_sensor(0x1d, 0x09);  // tlpx_time_l_dn
 write_cmos_sensor(0x1e, 0x0f);  // hs_zero_time //0f
 write_cmos_sensor(0x1f, 0x0b);  // hs_trail_time//0b
 write_cmos_sensor(0x30, 0x07);  // l_pkt_wc_h   1600 * 5 / 4 = 2000  (MIPI Data Byte)
 write_cmos_sensor(0x31, 0xf8);  // l_pkt_wc_l   1600 * 5 / 4 = 2000  (MIPI Data Byte)
 write_cmos_sensor(0x10, 0x1d);
  write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off
}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
/////////////////
// Sleep ON
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x01);		//sleep on
//////////////////
// PLL Setting
//////////////////
// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
 write_cmos_sensor(0x08, 0x5a);  // pll main div : 4x ~ 127x
 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
 write_cmos_sensor(0x07, 0xC5);  // pll enable & pre div setting

 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x30, 0x00);		//20150127
 write_cmos_sensor(0x31, 0x08);		//20150127
 write_cmos_sensor(0x32, 0x00);		//20150127
 write_cmos_sensor(0x33, 0x08);		//20150127
 write_cmos_sensor(0x34, 0x00);		//20150127
 write_cmos_sensor(0x35, 0x08);		//20150127
 write_cmos_sensor(0x36, 0x00);		//20150127
 write_cmos_sensor(0x37, 0x08);		//20150127
 write_cmos_sensor(0x38, 0x02);		//20150127
 write_cmos_sensor(0x39, 0x58);		//20150127
 write_cmos_sensor(0x3a, 0x03);		//20150127
 write_cmos_sensor(0x3b, 0x20);		//20150127

 write_cmos_sensor(0x80, 0x00);		//20150127
 write_cmos_sensor(0x81, 0x00);		//20150127
 write_cmos_sensor(0x82, 0x00);		//20150127
 write_cmos_sensor(0x83, 0x00);		//20150127
 write_cmos_sensor(0x84, 0x06);		//20150127
 write_cmos_sensor(0x85, 0x60);		//20150127
 write_cmos_sensor(0x86, 0x04);		//20150127
 write_cmos_sensor(0x87, 0xcc);		//20150127
 write_cmos_sensor(0x88, 0x00);		//20150127
 write_cmos_sensor(0x89, 0x00);		//20150127
 write_cmos_sensor(0x8a, 0x02);		//20150127
 write_cmos_sensor(0x8b, 0x66);		//20150127

//PREVIEW//
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x60, 0x00);
 write_cmos_sensor(0x10, 0x01);
 write_cmos_sensor(0x3a, 0x03);
 write_cmos_sensor(0x3b, 0x60);
 write_cmos_sensor(0x4e, 0x02);
 write_cmos_sensor(0x4f, 0x96);
 write_cmos_sensor(0x09, 0x17);

 write_cmos_sensor(0x03, 0x01);
 write_cmos_sensor(0x10, 0x21);
 write_cmos_sensor(0x13, 0x08);

// write_cmos_sensor(0x03, 0x20);
// write_cmos_sensor(0x60, 0xF0);
// write_cmos_sensor(0x61, 0x00);

 write_cmos_sensor(0x03, 0x02);
 write_cmos_sensor(0x40, 0x00);
 write_cmos_sensor(0x41, 0x17);
 write_cmos_sensor(0x47, 0x00);
 write_cmos_sensor(0x48, 0x9a);
 write_cmos_sensor(0x49, 0x24);

 write_cmos_sensor(0x03, 0x08);
 write_cmos_sensor(0x20, 0x01);
 write_cmos_sensor(0x21, 0x00);
 write_cmos_sensor(0x22, 0x01);
 write_cmos_sensor(0x23, 0x00);
 write_cmos_sensor(0x24, 0x01);
 write_cmos_sensor(0x25, 0x00);
 write_cmos_sensor(0x26, 0x01);
 write_cmos_sensor(0x27, 0x00);

 write_cmos_sensor(0x03, 0x10);
 write_cmos_sensor(0x10, 0x01);

 write_cmos_sensor(0x03, 0x05);
 write_cmos_sensor(0x1a, 0x50);
 write_cmos_sensor(0x1c, 0x05);
 write_cmos_sensor(0x1d, 0x0c);
 write_cmos_sensor(0x1e, 0x08);
 write_cmos_sensor(0x1f, 0x07);
 write_cmos_sensor(0x30, 0x03);
 write_cmos_sensor(0x31, 0xe8);
 write_cmos_sensor(0x32, 0x0f);
 write_cmos_sensor(0x33, 0x08);
 write_cmos_sensor(0x34, 0x03);
 write_cmos_sensor(0x35, 0x02);
 write_cmos_sensor(0x36, 0x01);
 write_cmos_sensor(0x37, 0x05);

/////////////////
// Sleep OFF
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off
}
static void hs_video_setting(void)
{
	LOG_INF("hs_video E\n");
	write_cmos_sensor(0x03, 0x00);
	 write_cmos_sensor(0x01, 0x01); 	// sleep on

	 //////////////////
	// PLL Setting
	//////////////////
	// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
	 write_cmos_sensor(0x03, 0x00);
	 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
	 write_cmos_sensor(0x08, 0x5a);  // pll main div : 4x ~ 127x
	 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
	 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
	 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
	 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
	 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
	 write_cmos_sensor(0x07, 0xC5);  // pll enable & pre div setting

	 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x30, 0x00);		//20150127
 write_cmos_sensor(0x31, 0x44);		//20150127
 write_cmos_sensor(0x32, 0x00);		//20150127
 write_cmos_sensor(0x33, 0x58);		//20150127
 write_cmos_sensor(0x34, 0x00);		//20150127
 write_cmos_sensor(0x35, 0x44);		//20150127
 write_cmos_sensor(0x36, 0x00);		//20150127
 write_cmos_sensor(0x37, 0x58);		//20150127
 write_cmos_sensor(0x38, 0x02);		//20150127
 write_cmos_sensor(0x39, 0x58);		//20150127
 write_cmos_sensor(0x3a, 0x03);		//20150127
 write_cmos_sensor(0x3b, 0x20);		//20150127

 write_cmos_sensor(0x80, 0x00);		//20150127
 write_cmos_sensor(0x81, 0x00);		//20150127
 write_cmos_sensor(0x82, 0x00);		//20150127
 write_cmos_sensor(0x83, 0x00);		//20150127
 write_cmos_sensor(0x84, 0x06);		//20150127
 write_cmos_sensor(0x85, 0x60);		//20150127
 write_cmos_sensor(0x86, 0x04);		//20150127
 write_cmos_sensor(0x87, 0xcc);		//20150127
 write_cmos_sensor(0x88, 0x00);		//20150127
 write_cmos_sensor(0x89, 0x00);		//20150127
 write_cmos_sensor(0x8a, 0x02);		//20150127
 write_cmos_sensor(0x8b, 0x66);		//20150127

//PREVIEW//
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x60, 0x00);
 write_cmos_sensor(0x10, 0x01);
 write_cmos_sensor(0x3a, 0x03);
 write_cmos_sensor(0x3b, 0x60);
 write_cmos_sensor(0x4e, 0x02);
 write_cmos_sensor(0x4f, 0x96);
 write_cmos_sensor(0x09, 0x17);

 write_cmos_sensor(0x03, 0x01);
 write_cmos_sensor(0x10, 0x21);
 write_cmos_sensor(0x13, 0x08);

// write_cmos_sensor(0x03, 0x20);
// write_cmos_sensor(0x60, 0xF0);
// write_cmos_sensor(0x61, 0x00);

 write_cmos_sensor(0x03, 0x02);
 write_cmos_sensor(0x40, 0x00);
 write_cmos_sensor(0x41, 0x17);
 write_cmos_sensor(0x47, 0x00);
 write_cmos_sensor(0x48, 0x9a);
 write_cmos_sensor(0x49, 0x24);

 write_cmos_sensor(0x03, 0x08);
 write_cmos_sensor(0x20, 0x01);
 write_cmos_sensor(0x21, 0x00);
 write_cmos_sensor(0x22, 0x01);
 write_cmos_sensor(0x23, 0x00);
 write_cmos_sensor(0x24, 0x01);
 write_cmos_sensor(0x25, 0x00);
 write_cmos_sensor(0x26, 0x01);
 write_cmos_sensor(0x27, 0x00);

 write_cmos_sensor(0x03, 0x10);
 write_cmos_sensor(0x10, 0x01);

 write_cmos_sensor(0x03, 0x05);
 write_cmos_sensor(0x1a, 0x50);
 write_cmos_sensor(0x1c, 0x05);
 write_cmos_sensor(0x1d, 0x0c);
 write_cmos_sensor(0x1e, 0x08);
 write_cmos_sensor(0x1f, 0x07);
 write_cmos_sensor(0x30, 0x03);
 write_cmos_sensor(0x31, 0x20);
 write_cmos_sensor(0x32, 0x0f);
 write_cmos_sensor(0x33, 0x08);
 write_cmos_sensor(0x34, 0x03);
 write_cmos_sensor(0x35, 0x02);
 write_cmos_sensor(0x36, 0x01);
 write_cmos_sensor(0x37, 0x05);

/////////////////
// Sleep OFF
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off


}

static void slim_video_setting(void)
{
	LOG_INF("slim_video E\n");
/////////////////
// Sleep ON
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x01);		//sleep on
//////////////////
// PLL Setting
//////////////////
// MIPI_4x = 360 , ISP_clk = 144Mhz , OPCLK = 72Mhz, PXL_clk = 72 Mhz
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x07, 0x05);  // pll enable & pre div setting
 write_cmos_sensor(0x08, 0x5a);  // pll main div : 4x ~ 127x
 write_cmos_sensor(0x09, 0x13);  // isp div : 2/5 mipi 4x div : 1/1 mipi 1x div : 1/5
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x07, 0x85);  // pll enable & pre div setting
 write_cmos_sensor(0x0a, 0x80);  // pll_clk_sw select & clk inv option
 write_cmos_sensor(0x07, 0xC5);  // pll enable & pre div setting

 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x20	,0x01);			//20150127
 write_cmos_sensor(0x21	,0x00);			//20150127
 write_cmos_sensor(0x22	,0x00);			//20150127
 write_cmos_sensor(0x23	,0xb0);			//20150127
 write_cmos_sensor(0x24	,0x00);			//20150127
 write_cmos_sensor(0x25	,0xfc);			//20150127
 write_cmos_sensor(0x26	,0x00);			//20150127
 write_cmos_sensor(0x27	,0xb0);			//20150127
 write_cmos_sensor(0x28	,0x02);			//20150127
 write_cmos_sensor(0x29	,0xd0);			//20150127
 write_cmos_sensor(0x2a	,0x05);			//20150127
 write_cmos_sensor(0x2b	,0x00);			//20150127

 write_cmos_sensor(0x4e, 0x05);
 write_cmos_sensor(0x4f, 0x0c);

 write_cmos_sensor(0x80, 0x00);		//20150127
 write_cmos_sensor(0x81, 0x00);		//20150127
 write_cmos_sensor(0x82, 0x00);		//20150127
 write_cmos_sensor(0x83, 0x00);		//20150127
 write_cmos_sensor(0x84, 0x06);		//20150127
 write_cmos_sensor(0x85, 0x60);		//20150127
 write_cmos_sensor(0x86, 0x04);		//20150127
 write_cmos_sensor(0x87, 0xcc);		//20150127
 write_cmos_sensor(0x88, 0x00);		//20150127
 write_cmos_sensor(0x89, 0x00);		//20150127
 write_cmos_sensor(0x8a, 0x02);		//20150127
 write_cmos_sensor(0x8b, 0x66);		//20150127

//////////////////
// Page 0x20
//////////////////
 write_cmos_sensor(0x03	,0x20);
 write_cmos_sensor(0x10	,0x00);
 write_cmos_sensor(0x11	,0x05);
 write_cmos_sensor(0x12	,0x00);
 write_cmos_sensor(0x22	,0x01); //2014.08.18 exptime_line_h
 write_cmos_sensor(0x23	,0x4d); //2014.08.18 exptime_line_l
 write_cmos_sensor(0x26	,0xff);
 write_cmos_sensor(0x27	,0xff);
 write_cmos_sensor(0x29	,0x00);
 write_cmos_sensor(0x2a	,0x02);
 write_cmos_sensor(0x2b	,0x00);
 write_cmos_sensor(0x2c	,0x04);
 write_cmos_sensor(0x30	,0x00);
 write_cmos_sensor(0x31	,0x04);
/////////////////////////////
// 2014.10.23 added
/////////////////////////////
 write_cmos_sensor(0x40	,0x09); //bandtime pixel
 write_cmos_sensor(0x41	,0x1e);
 write_cmos_sensor(0x42	,0x60);
/////////////////////////////
 write_cmos_sensor(0x52	,0x0f);
 write_cmos_sensor(0x53	,0xf3);		//pga_max

 write_cmos_sensor(0x60	,0xf0);
 write_cmos_sensor(0x61	,0x00);

 write_cmos_sensor(0x64	,0x11);
 write_cmos_sensor(0x65	,0x01);		//ramp_gain_max
//////////////////
// MIPI Setting
//////////////////
 write_cmos_sensor(0x03, 0x05);  // mode_page
 write_cmos_sensor(0x39, 0x57);  // lvds_bias_ctl
 write_cmos_sensor(0x4c, 0x20);  // hs_wakeup_size_h
 write_cmos_sensor(0x4d, 0x00);  // hs_wakeup_size_l
 write_cmos_sensor(0x4e, 0x40);  // mipi_int_time_h
 write_cmos_sensor(0x4f, 0x00);  // mipi_int_time_l

 write_cmos_sensor(0x11, 0x00);  // lvds_ctl_2
 write_cmos_sensor(0x16, 0x02);  // lvds_inout_ctl1
 write_cmos_sensor(0x18, 0x80);  // lvds_inout_ctl3
 write_cmos_sensor(0x19, 0x00);  // lvds_inout_ctl4
 write_cmos_sensor(0x1a, 0xf0);  // lvds_time_ctl
 write_cmos_sensor(0x24, 0x2b);  // long_packet_id
// CLK Lane Timing Control //
 write_cmos_sensor(0x32, 0x1e);  // clk_zero_time
 write_cmos_sensor(0x33, 0x0f);  // clk_post_time
 write_cmos_sensor(0x34, 0x06);  // clk_prepare_time
 write_cmos_sensor(0x35, 0x05);  // clk_trail_time
 write_cmos_sensor(0x36, 0x01);  // clk_tlpx_time_dp
 write_cmos_sensor(0x37, 0x08);  // clk_tlpx_time_dn
// DATA Lane Timing Control //
 write_cmos_sensor(0x1c, 0x01);  // tlpx_time_l_dp
 write_cmos_sensor(0x1d, 0x09);  // tlpx_time_l_dn
 write_cmos_sensor(0x1e, 0x0f);  // hs_zero_time
 write_cmos_sensor(0x1f, 0x0b);  // hs_trail_time
 write_cmos_sensor(0x30, 0x06);  // l_pkt_wc_h   1280 * 5 / 4 = 1600  (MIPI Data Byte)
 write_cmos_sensor(0x31, 0x40);  // l_pkt_wc_l   1280 * 5 / 4 = 1600  (MIPI Data Byte)

 write_cmos_sensor(0x10, 0x1d);  // lvds_ctl_1

/////////////////
// Sleep OFF
/////////////////
 write_cmos_sensor(0x03, 0x00);
 write_cmos_sensor(0x01, 0x00);		//sleep off
}


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//#define SLT_DEVINFO_CMM
#ifdef SLT_DEVINFO_CMM
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device
#endif
//extern int front_camera_find_success;
//extern bool camera_front_probe_ok;//bit1

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry_total = 1;
	kal_uint8 retry_cnt = retry_total;
#ifdef SLT_DEVINFO_CMM
 	s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	s_DEVINFO_ccm->device_type = "CCM";
	s_DEVINFO_ccm->device_module = "PC0FB0002B";//can change if got module id
	s_DEVINFO_ccm->device_vendor = "Sunrise";
	s_DEVINFO_ccm->device_ic = "HI259V1";
	s_DEVINFO_ccm->device_version = "HI";
	s_DEVINFO_ccm->device_info = "200W";
#endif

	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	LOG_INF(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 4){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
                          *sensor_id = return_sensor_id()+1;
                            LOG_INF("HI259V1  get_imgsensor_id sensor id: 0x%x\n",*sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				vivo_otp_read_when_power_on_hi259v1 = hi259v1_vivo_otp_read();
	#ifdef SLT_DEVINFO_CMM
		s_DEVINFO_ccm->device_used = DEVINFO_USED;
		devinfo_check_add_device(s_DEVINFO_ccm);
	#endif
    //                                front_camera_find_success=2;
  //                                  camera_front_probe_ok=1;

				return ERROR_NONE;
			}
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry_cnt--;
		} while(retry_cnt > 0);
		i++;
		retry_cnt = retry_total;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
	#ifdef SLT_DEVINFO_CMM
		s_DEVINFO_ccm->device_used = DEVINFO_UNUSED;
		devinfo_check_add_device(s_DEVINFO_ccm);
	#endif
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;

    printk("open camera sensor hi259v1\n");
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            sensor_id =  return_sensor_id()+1;
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	write_cmos_sensor_hi259v1_gc2375(0xef,0x00);
	write_cmos_sensor_hi259v1_gc2375(0xfc,0x8f);
	write_cmos_sensor_hi259v1_gc2375(0xf9,0x43);
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0xe000; //0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

    printk("open camera sensor hi259v1 done\n");
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
    }
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	mdelay(100);
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 600;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 1200;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5080[8]: 1 enable,  0 disable
		// 0x5080[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		//write_cmos_sensor(0x5080, 0x80);
		write_cmos_sensor(0x03, 0x00);
 		write_cmos_sensor(0x60, 0x04);		// sleep on
	} else {
		// 0x5080[8]: 1 enable,  0 disable
		// 0x5080[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x03, 0x00);
 		write_cmos_sensor(0x60, 0x00);		// sleep on

	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
//    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter((UINT16)*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n",  (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
			LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = (bool)*feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
			wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
    case SENSOR_FEATURE_GET_CUSTOM_INFO:
    	printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  HI259V1_OTP_ERROR_CODE:%d \n", *feature_data,HI259V1_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
				printk("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
				if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = HI259V1_OTP_ERROR_CODE;//otp_state
					memcpy( feature_data+2, sn_inf_main_hi259v1, sizeof(MUINT32)*13); 
					#if 0
						for (i = 0 ; i<13 ; i++ ){
						printk("sn_inf_main_hi259v1[%d]= 0x%x\n", i, sn_inf_main_hi259v1[i]);
						}
								
					#endif
					}
				break;
				}
			break;
		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI259V1_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
/*UINT32 HI259V1_MIPI_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)*/

{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI259V1_MIPI_RAW_SensorInit	*/
