/*
 * Copyright (C) 2015 MediaTek Inc.
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

#define LOG_TAG "LCM"
#define LOG_PANEL_TYPE "TRULY+ILI9982Q"
#define LOG_PROJ_NAME "PD2216"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "mtkfb.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)			(lcm_util.mdelay(n))
#define UDELAY(n)			(lcm_util.udelay(n))


#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define lcm_vddi_setting(cmd) \
			lcm_util.lcm_vddi_setting(cmd)
#define lcm_reset_setting(cmd) \
			lcm_util.lcm_reset_setting(cmd)
#define lcm_enp_setting(cmd) \
			lcm_util.lcm_enp_setting(cmd)
#define lcm_enn_setting(cmd) \
			lcm_util.lcm_enn_setting(cmd)
#define lcm_bkg_setting(cmd) \
			lcm_util.lcm_bkg_setting(cmd)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

extern long fts8016_fts_reset_lcm(unsigned int value);
#define tp_reset_setting(value) fts8016_fts_reset_lcm(value)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
//#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int phone_shutdown_state;
extern unsigned int is_atboot;
extern unsigned int pre_bklt_level;
#endif

/*****************************************************************************
 * Define
 *****************************************************************************/
/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;

#define LCM_DSI_CMD_MODE			(0)
#define FRAME_WIDTH					(720)
#define FRAME_HEIGHT				(1600)
#define LCM_PHYSICAL_WIDTH			(67932)
#define LCM_PHYSICAL_HEIGHT			(150960)


#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif


#define CONFIG_VIVO_LCM_PD1732DF_CONTROL

extern unsigned int rf_hw_id;
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
extern int vivo_lcm_cabc_backlight;
static unsigned int dimming_enable;
extern unsigned int lcm_id_version;
extern unsigned int dimming_for_camera;
extern void lcm_bias_set_avdd_n_avee(int value);
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY,120,{}},
};

#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
static struct LCM_setting_table init_setting_12bit[] = {
	{0xFF,3,{0x98,0x82,0x01}}, //3H
	{0x00,1,{0x47}},
	{0x01,1,{0x32}},
	{0x02,1,{0x00}},
	{0x03,1,{0x00}},
	{0x04,1,{0x04}},
	{0x05,1,{0x32}},
	{0x06,1,{0x00}},
	{0x07,1,{0x00}},
	{0x08,1,{0x85}},
	{0x09,1,{0x04}},
	{0x0a,1,{0x72}},
	{0x0b,1,{0x00}},
	{0x0c,1,{0x00}},
	{0x0d,1,{0x00}},
	{0x0e,1,{0x00}},
	{0x0f,1,{0x00}},
	{0x28,1,{0x48}},
	{0x29,1,{0x88}},
	{0x2A,1,{0x48}},
	{0x2B,1,{0x88}},

	//FW
	{0x31,1,{0x0C}}, //RST_L
	{0x32,1,{0x02}}, //VGL
	{0x33,1,{0x02}}, //VGL
	{0x34,1,{0x23}}, //GLV
	{0x35,1,{0x08}}, //STV1_L
	{0x36,1,{0x0A}}, //STV2_L
	{0x37,1,{0x06}}, //VDD
	{0x38,1,{0x06}}, //VDD
	{0x39,1,{0x10}}, //CLK1_L
	{0x3A,1,{0x10}}, //CLK1_L
	{0x3B,1,{0x12}}, //CLK2_L
	{0x3C,1,{0x12}}, //CLK2_L
	{0x3D,1,{0x14}}, //CLK1B_L
	{0x3E,1,{0x14}}, //CLK1B_L
	{0x3F,1,{0x16}}, //CLK2B_L
	{0x40,1,{0x16}}, //CLK2B_L
	{0x41,1,{0x02}}, //VGL
	{0x42,1,{0x07}}, //HIZ
	{0x43,1,{0x07}}, //HIZ
	{0x44,1,{0x07}}, //HIZ
	{0x45,1,{0x07}}, //HIZ
	{0x46,1,{0x07}}, //HIZ

	//FW_L
	{0x47,1,{0x0D}}, //RST_R
	{0x48,1,{0x02}}, //VGL
	{0x49,1,{0x02}}, //VGL
	{0x4A,1,{0x23}}, //GLV
	{0x4B,1,{0x09}}, //STV1_R
	{0x4C,1,{0x0B}}, //STV2_R
	{0x4D,1,{0x06}}, //VDD
	{0x4E,1,{0x06}}, //VDD
	{0x4F,1,{0x11}}, //CLK1_R
	{0x50,1,{0x11}}, //CLK1_R
	{0x51,1,{0x13}}, //CLK2_R
	{0x52,1,{0x13}}, //CLK2_R
	{0x53,1,{0x15}}, //CLK1B_R
	{0x54,1,{0x15}}, //CLK1B_R
	{0x55,1,{0x17}}, //CLK2B_R
	{0x56,1,{0x17}}, //CLK2B_R
	{0x57,1,{0x02}}, //VGL
	{0x58,1,{0x07}}, //HIZ
	{0x59,1,{0x07}}, //HIZ
	{0x5A,1,{0x07}}, //HIZ
	{0x5B,1,{0x07}}, //HIZ
	{0x5C,1,{0x07}}, //HIZ

	//BW_R
	{0x61,1,{0x0C}}, //RST_L
	{0x62,1,{0x02}}, //VGL
	{0x63,1,{0x02}}, //VGL
	{0x64,1,{0x23}}, //GLV
	{0x65,1,{0x08}}, //STV1_L
	{0x66,1,{0x0A}}, //STV2_L
	{0x67,1,{0x06}}, //VDD
	{0x68,1,{0x06}}, //VDD
	{0x69,1,{0x10}}, //CLK1_L
	{0x6A,1,{0x10}}, //CLK1_L
	{0x6B,1,{0x12}}, //CLK2_L
	{0x6C,1,{0x12}}, //CLK2_L
	{0x6D,1,{0x14}}, //CLK1B_L
	{0x6E,1,{0x14}}, //CLK1B_L
	{0x6F,1,{0x16}}, //CLK2B_L
	{0x70,1,{0x16}}, //CLK2B_L
	{0x71,1,{0x02}}, //VGL
	{0x72,1,{0x07}}, //HIZ
	{0x73,1,{0x07}}, //HIZ
	{0x74,1,{0x07}}, //HIZ
	{0x75,1,{0x07}}, //HIZ
	{0x76,1,{0x07}}, //HIZ

	//BW_L
	{0x77,1,{0x0D}}, //RST_R
	{0x78,1,{0x02}}, //VGL
	{0x79,1,{0x02}}, //VGL
	{0x7A,1,{0x23}}, //GLV
	{0x7B,1,{0x09}}, //STV1_R
	{0x7C,1,{0x0B}}, //STV2_R
	{0x7D,1,{0x06}}, //VDD
	{0x7E,1,{0x06}}, //VDD
	{0x7F,1,{0x11}}, //CLK1_R
	{0x80,1,{0x11}}, //CLK1_R
	{0x81,1,{0x13}}, //CLK2_R
	{0x82,1,{0x13}}, //CLK2_R
	{0x83,1,{0x15}}, //CLK1B_R
	{0x84,1,{0x15}}, //CLK1B_R
	{0x85,1,{0x17}}, //CLK2B_R
	{0x86,1,{0x17}}, //CLK2B_R
	{0x87,1,{0x02}}, //VGL
	{0x88,1,{0x07}}, //HIZ
	{0x89,1,{0x07}}, //HIZ
	{0x8A,1,{0x07}}, //HIZ
	{0x8B,1,{0x07}}, //HIZ
	{0x8C,1,{0x07}}, //HIZ

	{0xB0,1,{0x33}},
	{0xB1,1,{0x33}},
	{0xB2,1,{0x00}},
	{0xD0,1,{0x01}},
	{0xD1,1,{0x00}},
	{0xE2,1,{0x00}},
	{0xE6,1,{0x22}},
	{0xE7,1,{0x54}},

	//RTN. Internal VBP,0x Internal VFP
	{0xFF,3,{0x98,0x82,0x02}},
	{0xF1,1,{0x1C}}, //Tcon ESD option
	{0x4B,1,{0x5A}}, //line_chopper
	{0x50,1,{0xCA}}, //line_chopper
	{0x51,1,{0x00}}, //line_chopper
	{0x06,1,{0x8F}}, //Internal Line Time (RTN)
	{0x0B,1,{0xA0}}, //Internal VFP[9]
	{0x0C,1,{0x00}}, //Internal VFP[8]
	{0x0D,1,{0x14}}, //Internal VBP
	{0x0E,1,{0xE6}}, //Internal VFP
	{0x4E,1,{0x11}}, //SRC BIAS
	{0xF2,1,{0x4A}},

	{0x40,1,{0x45}}, //sdt
	{0x43,1,{0x02}}, //Tcon
	{0x47,1,{0x13}}, //Tcon
	{0x49,1,{0x03}}, //Tcon
	{0x4D,1,{0xCF}}, //PS enbale

	{0xFF,3,{0x98,0x82,0x03}},
	{0x83,1,{0x20}}, //
	{0x84,1,{0x00}},

	//Power Setting
	{0xFF,3,{0x98,0x82,0x05}},
	//{0x03,1,{0x01}}, //Vcom
	//{0x04,1,{0x06}}, //Vcom
	{0x58,1,{0x61}}, //VGL 2x
	{0x63,1,{0x97}}, //GVDDN = -5.5V
	{0x64,1,{0x97}}, //GVDDP = 5.5V
	{0x68,1,{0xA1}}, //VGHO = 15V
	{0x69,1,{0xA7}}, //VGH = 16V
	{0x6A,1,{0x79}}, //VGLO = -10V
	{0x6B,1,{0x6B}}, //VGL = -11V
	{0x85,1,{0x37}}, //HW RESET option

	//Resolution
	{0xFF,3,{0x98,0x82,0x06}},
	{0xD9,1,{0x10}}, //4Lane 1F ,0x 3Lane=10
	{0xC0,1,{0x40}}, //NL = 1600
	{0xC1,1,{0x16}}, //NL = 1600
	{0x80,1,{0x08}}, //

	{0xFF,3,{0x98,0x82,0x07}},
	{0x00,1,{0x08}}, //优化G252~255 曲线

	//OSC Auto Trim Setting
	{0xFF,3,{0x98,0x82,0x0B}},
	{0x9A,1,{0x89}},
	{0x9B,1,{0x02}},
	{0x9C,1,{0x06}},
	{0x9D,1,{0x06}},
	{0x9E,1,{0xE1}},
	{0x9F,1,{0xE1}},
	{0xAA,1,{0x22}},
	{0xAB,1,{0xE0}}, //AutoTrimType
	{0xAC,1,{0x7F}}, //trim_osc_max
	{0xAD,1,{0x3F}}, //trim_osc_min

	{0xFF,3,{0x98,0x82,0x0E}},
	{0x11,1,{0x10}}, //TSVD Rise Poisition
	{0x12,1,{0x08}}, //LV mode TSHD Rise position
	{0x13,1,{0x10}}, //LV mode TSHD Rise position
	{0x00,1,{0xA0}}, //LV mode

	{0xFF,3,{0x98,0x82,0x00}},
	{0x35,1,{0x00}},

	{0x11,0,{}},
	{REGFLAG_DELAY,120,{}},
	{0x29,0,{}},
	{REGFLAG_DELAY,100,{}},

	{0x51,2,{0x00,0x00}}, //CABC
	{0x53,1,{0x2C}}, //模式
	{0x55,1,{0x00}},
};

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51,2,{0x0F,0xFF} },
	{REGFLAG_END_OF_TABLE,0x0,{}},
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x2C}},
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_disable[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x24}},
};
#endif

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x2c}},
	{0x55,1,{0x01}},
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x2c}},
	{0x55,1,{0x02}},
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x2c}},
	{0x55,1,{0x02}},
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x00}},
};

static void push_table(struct LCM_setting_table *table,
				unsigned int count,
				unsigned char force_update)
{
	unsigned int i, cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void cabc_push_table(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i, cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 230;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 56;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 32;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;

	params->dsi.PLL_CLOCK = 394;
	params->dsi.data_rate = 788;

//#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL
	if (rf_hw_id == 1) {
		params->dsi.horizontal_backporch = 60;
		params->dsi.horizontal_frontporch = 60;
		params->dsi.PLL_CLOCK = 387;
		params->dsi.data_rate = 774;
		LCM_INFO("[%s/%s]rf_hw_id:%d, PLL_CLOCK:%d\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, rf_hw_id, params->dsi.PLL_CLOCK);
	}
//#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	if (is_atboot)
		params->dsi.esd_check_enable = 0;

	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.HS_PRPR = 0x8;
	params->dsi.HS_TRAIL = 0x9;

	/*add for mipi_dyn_clk*/
	params->dsi.PLL_CLOCK_dyn = 388;
	params->dsi.horizontal_sync_active_dyn = 43;
}

static void lcm_init_power(void)
{
	LCM_INFO("[%s/%s]%s:enter power on begin, panel_reset_state=%d\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__, panel_reset_state);

	tp_reset_setting(0); /*tp && lcm reset pin set to low at first*/
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	if (panel_reset_state == 0) { //panel power off now, and then panel power on
		panel_reset_state = 1;
		MDELAY(5);
		lcm_enp_setting(1);
		MDELAY(5);
		lcm_enn_setting(1);
		MDELAY(2);
		/************************************************************
		KTD2151 is 1th manufacture, SM3109 is 2nd default voltage is 5v5, so we
		need to set again It will lead to a raising on the waveform.
		*************************************************************/
		lcm_bias_set_avdd_n_avee(60);
		MDELAY(3);
	}
}

static void lcm_init(void)
{
	LCM_INFO("[%s/%s]%s:init begin\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);

	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	tp_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(1);
	/*****************************************************************
	lrz modify it to 15ms from 90ms.	90ms is used to ensure flash download initial code.
	But PD1987 use no-flash version nt36525, so 15ms is enough, NolenCheng confirmed 0304
	******************************************************************/
	MDELAY(35);
	MDELAY(55);

	push_table(init_setting_12bit, sizeof(init_setting_12bit) / sizeof(struct LCM_setting_table), 1);
	panel_reset_state = 1;
	panel_off_state = 0; //panel power on
}

static void lcm_suspend_power(void)
{
	LCM_INFO("[%s/%s]%s:suspend power off\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	if (phone_shutdown_state) {
		LCM_INFO("[%s/%s]reset keep low level\n", LOG_PROJ_NAME, LOG_PANEL_TYPE);
		MDELAY(5);
		lcm_reset_setting(0);
	}
	MDELAY(45);
	LCM_INFO("[%s/%s]vddi keep high level\n", LOG_PROJ_NAME, LOG_PANEL_TYPE);
	panel_reset_state = 0; //panel power off
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	panel_off_state = 1;
	LCM_INFO("[%s/%s]%s:suspend\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);
}

static void lcm_resume(void)
{
	LCM_INFO("[%s/%s]%s:resume\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);
	lcm_init();
}

static void lcm_reset_for_touch(void)
{
	LCM_INFO("[%s/%s]%s:reset for tp\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);
	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	tp_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(12);
	panel_reset_state = 1;
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
#if LCM_DSI_CMD_MODE
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}

/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);

	if (buffer[0] != 0x9c) {
		LCM_INFO("[%s/%s]lcm esd check error, [0x0a]=0x%02x\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, buffer[0]);
		return TRUE;
	}
	LCM_INFO("[%s/%s]lcm esd check normal, [0x0a]=0x%02x\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, buffer[0]);

	return FALSE;
#else
	return FALSE;
#endif
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_INFO("[%s/%s]%s:ATA check size=0x%x,0x%x,0x%x,0x%x\n", LOG_PROJ_NAME, LOG_PANEL_TYPE,
					__func__, x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	/* read id return two byte,version and id */
	data_array[0] = 0x00043700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	unsigned int level_original_12bit = level;

	LCM_INFO("[%s/%s]%s:set backlight by cmdq\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__);
	/* Due to the addition of low brightness screen, the backlight current is 24mA -> 23mA(96.64%) */
	level = (level * 9664 + 5920) / 10000;

	if ((level != 0) && (dimming_enable == 0) && (vivo_lcm_cabc_backlight != 0) && (dimming_for_camera == 1)) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		LCM_INFO("[%s/%s]dimming enable\n", LOG_PROJ_NAME, LOG_PANEL_TYPE);
	}
	if (level == 0) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 0;
		LCM_INFO("[%s/%s]dimming disable\n", LOG_PROJ_NAME, LOG_PANEL_TYPE);
	}

	bl_level_12bit[0].para_list[0] = (unsigned char)((level >> 8) & 0x0f); /*high 4 bit*/
	bl_level_12bit[0].para_list[1] = (unsigned char)(level & 0xff); /*low 8 bit*/
	cabc_push_table(handle, bl_level_12bit, sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);

	LCM_INFO("[%s/%s]LastBacklight=%d, level_original_12bit=%d, level=%d, high_bit=0x%x, low_bit=0x%x\n", LOG_PROJ_NAME, LOG_PANEL_TYPE,
						vivo_lcm_cabc_backlight, level_original_12bit, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	pre_bklt_level = level_original_12bit;
}

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_id_version == 0x28)
		return 0x06;
	else
		return 0xFF;
}

static void lcm_cabc_vivo_open(void *handle, unsigned char leveldimming, unsigned char levelsetting)
{
	/*add for lcm_diming*/
	if (leveldimming == 0x2d) {
		if ((levelsetting == 0) && (sizeof(lcm_cmd_backlight_dimming_disable) > 0)) {
			dimming_enable = 0;
			cabc_push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		} else if ((levelsetting == 1) && (sizeof(lcm_cmd_backlight_dimming_enable) > 0)) {
			dimming_enable = 1;
			cabc_push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		}
	/*add end*/
	} else if ((levelsetting == 0) && (sizeof(lcm_cmd_cabc_off)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_cabc_level1)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level1, sizeof(lcm_cmd_cabc_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_cabc_level2)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level2, sizeof(lcm_cmd_cabc_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_cabc_level3)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level3, sizeof(lcm_cmd_cabc_level3) / sizeof(struct LCM_setting_table), 1);
    LCM_INFO("[%s/%s]%s:vincent-dim:%d, level:%d\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__, leveldimming, levelsetting);
}

static void lcm_cabc_vivo_close(void *handle, unsigned char level)
{
	if (sizeof(lcm_cmd_cabc_off))
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	 LCM_INFO("[%s/%s]%s:vincent-level:%d\n", LOG_PROJ_NAME, LOG_PANEL_TYPE, __func__, level);
}

struct LCM_DRIVER pd2216_truly_ili9882q_hdplus_dsi_vdo_lcm_drv = {
	.name = "pd2216_truly_ili9882q_hdplus_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.get_id	    = lcm_get_id,
	.lcm_cabc_open  = lcm_cabc_vivo_open,
	.lcm_cabc_close = lcm_cabc_vivo_close,
	.lcm_reset = lcm_reset_for_touch,
};
