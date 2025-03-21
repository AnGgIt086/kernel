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
#define LOG_PANEL_TYPE "TXD+JD9365T"
#define LOG_PROJ_NAME "PD2236"

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
extern int jadard_tp_gpio_reset(unsigned int value);
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
	{REGFLAG_DELAY, 5, {} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
static struct LCM_setting_table init_setting_12bit[] = {
{0xdf,3,{0x93,0x83,0x16}},
{0xde,1,{0x00}},
{0xb2,5,{0x01,0x23,0x60,0x88,0x24}},
{0xb6,3,{0x0c,0x99,0xae}},
{0xb9,3,{0x3c,0x33,0x35}},
{0xbb,7,{0x0c,0x22,0x43,0x3c,0x5a,0x20,0x20}},
{0xbc,4,{0x2e,0x40,0x43,0x00}},
{0xbd,2,{0x00,0xcc}},
{0xc0,5,{0x00,0xdd,0x1a,0xdd,0x0a}},
{0xcb,42,{0x6c,0x64,0x5d,0x51,0x46,0x45,0x35,0x3a,0x23,0x3a,0x36,0x32,0x4a,0x35,0x3a,0x2b,0x26,0x17,0x08,0x02,0x00,0x6c,0x64,0x5d,0x51,0x46,0x45,0x35,0x3a,0x23,0x3a,0x36,0x32,0x4a,0x35,0x3a,0x2b,0x26,0x17,0x08,0x02,0x00}},
{0xcc,1,{0x32}},
{0xcd,2,{0x10,0x10}},
{0xb3,20,{0x00,0x80,0x85,0x00,0x64,0x1e,0x00,0x00,0x00,0x00,0x50,0x50,0x3c,0x3c,0x11,0x11,0x32,0x32,0x5a,0x5a}},
{0xbf,4,{0x32,0x5a,0x33,0xc3}},
{0xc3,13,{0x13,0x00,0x06,0x16,0x96,0x00,0x98,0x08,0x16,0x04,0x06,0x08,0x16}},
{0xc5,2,{0x30,0x20}},
{0xc6,4,{0x00,0xb7,0xc8,0x14}},
{0xce,23,{0x01,0x14,0x3c,0x3f,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x3c,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14}},
{0xcf,15,{0x00,0x00,0xc0,0xc3,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00}},
{0xd0,29,{0x00,0x1f,0xdf,0xd5,0xdf,0xc0,0xc2,0xd5,0xd5,0xc8,0xca,0xc4,0xc6,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x55,0x55,0x54}},
{0xd1,29,{0x00,0x1f,0xdf,0xd5,0xdf,0xc1,0xc3,0xd5,0xd5,0xc9,0xcb,0xc5,0xc7,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x55,0x55,0x54}},
{0xd2,29,{0x00,0x1f,0x15,0x15,0x1f,0x03,0x01,0x1f,0x15,0x07,0x05,0x0b,0x09,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xd3,29,{0x00,0x1f,0x15,0x15,0x1f,0x02,0x00,0x1f,0x15,0x06,0x04,0x0a,0x08,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xd4,62,{0x30,0x00,0x00,0x0a,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x10,0x60,0x01,0x00,0x04,0x01,0x01,0x00,0x01,0x00,0x04,0x01,0x01,0x11,0x60,0x03,0x00,0x05,0x01,0x01,0x00,0x00,0x00,0x04,0x0a,0x06,0x61,0x00,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x03,0xd4}},
{0xd5,27,{0x02,0x10,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x43,0x00,0x00,0x00,0x07,0x32,0x5a,0x00,0x48,0x3b,0x00,0x02,0x1e,0x02,0x73,0x00,0x0e,0x08}},
{0xd7,19,{0x00,0x10,0x6e,0x75,0x75,0x75,0x75,0x75,0x75,0x75,0x75,0x75,0x75,0x11,0x75,0x75,0x75,0x75,0x75}},
{0xde,1,{0x01}},
{0xb5,5,{0x6f,0x00,0x01,0x00,0x00}},//zly add PWM 18.7kHZ
{0xc5,2,{0x22,0x12}},
{0xde,1,{0x02}},
{0xb3,5,{0x20,0xa0,0xea,0x5f,0x4b}},
{0xb7,8,{0x44,0x00,0x88,0x00,0x03,0x00,0x6e,0x10}},
{0xbd,1,{0x14}},
{0xc1,8,{0x00,0x40,0x20,0x00,0x14,0x14,0x14,0x14}},
{0xc2,7,{0x42,0x70,0x01,0x01,0xe0,0x73,0xf8}},
{0xc3,6,{0x20,0xfb,0x00,0xa0,0x20,0x62}},
{0xe5,10,{0x00,0xe6,0xe5,0x02,0x27,0x42,0x27,0x42,0x09,0x00}},
{0xe6,3,{0x10,0x10,0xb5}},
{0xec,3,{0x01,0x7f,0x22}},
{0xde,1,{0x03}},
{0xd1,1,{0x00}},
{0xde,1,{0x00}},
{0x35,1,{0x00}},
};

static struct LCM_setting_table init_setting_bringup[] = {
{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,10,{}},
{0x51,2,{0x00,0x00}},
{0x53,1,{0x2c}},
{0x55,1,{0x00}},
{0xde,1,{0x01}},
{0xb9,1,{0x01}},
{0xc1,36,{0x3d,0x00,0xfd,0xb9,0xa9,0x99,0xce,0xee,0x14,0x35,0x56,0x75,0x77,0x55,0x62,0x32,0x43,0x02,0x32,0xf2,0x0f,0xec,0xf1,0x0e,0x0e,0xdb,0xcf,0xbf,0xdd,0xbd,0xcd,0xcd,0xdc,0xcc,0xaa,0x00}},
{0xc2,36,{0x40,0xc4,0x20,0xec,0xbb,0xbc,0xef,0xe0,0x24,0x44,0x45,0x64,0x43,0x1f,0x00,0xfe,0xde,0xdc,0xbc,0xbb,0xbc,0xdc,0xce,0x9b,0xdd,0xde,0xee,0xae,0x2e,0xe1,0x21,0x41,0x02,0x24,0x23,0x00}},
{0xc3,36,{0x3e,0xd3,0x21,0x0f,0xed,0xdc,0xde,0xee,0xf0,0x00,0x00,0x1f,0x0f,0xfe,0xec,0xcc,0xcc,0xba,0xba,0xaa,0xa9,0xa9,0xbb,0xab,0xaa,0xbb,0xcc,0xcd,0xdd,0xfe,0x01,0x23,0x33,0x34,0x54,0x05}},
{0xde,1,{0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_disable[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x24} },
};
#endif

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x00} },
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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 19;
	params->dsi.vertical_frontporch = 200;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 80;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 100;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;

	params->dsi.PLL_CLOCK = 430;
	params->dsi.data_rate = 860;

//#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL
	if (rf_hw_id == 1) {
		params->dsi.horizontal_backporch = 60;
		params->dsi.horizontal_frontporch = 60;
		params->dsi.PLL_CLOCK = 387;
		params->dsi.data_rate = 774;
		LCM_INFO("[%s/%s] rf_hw_id:%d PLL_CLOCK:%d\n",LOG_PROJ_NAME, LOG_PANEL_TYPE,rf_hw_id,params->dsi.PLL_CLOCK);
	}
//#endif

	params->dsi.clk_lp_per_line_enable = 0;
/*The ESD function is disabled by default*/
#ifdef CONFIG_VIVO_LCM_ESD_CHECK_DISABLE
	params->dsi.esd_check_enable = 0;
#else
	params->dsi.esd_check_enable = 1;
#endif

	if (is_atboot)
		params->dsi.esd_check_enable = 0;

	params->dsi.customization_esd_check_enable = 1;	//read from lcm
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
	params->dsi.HS_PRPR = 0x8;
	params->dsi.HS_TRAIL = 0x9;

	/*add for mipi_dyn_clk*/
	params->dsi.PLL_CLOCK_dyn = 424;
	params->dsi.horizontal_sync_active_dyn = 33;
}

static void lcm_init_power(void)
{
	LCM_INFO("[%s/%s]:enter power on begin, panel_reset_state=%d\n",LOG_PROJ_NAME,LOG_PANEL_TYPE,panel_reset_state);
	if (panel_reset_state == 0) {	//power on
		panel_reset_state = 1;
		lcm_enp_setting(1);
		MDELAY(5);
		lcm_enn_setting(1);
		MDELAY(5);
		/***********************************
		KTD2151 is 1th manufacture, SM3109 is 2nd
		default voltage is 5v5, so we need to set again
		It will lead to a raising on the waveform.
		************************************/
		lcm_bias_set_avdd_n_avee(60);
		LCM_INFO("[%s/%s]: enter power on end\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
	}
}

static void lcm_suspend_power(void)
{
	if (phone_shutdown_state) {
		LCM_INFO("[%s/%s]: lcm_suspend_power reset keep low level\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
		MDELAY(37);
		jadard_tp_gpio_reset(0);
		lcm_reset_setting(0);
	}
	MDELAY(10);
	lcm_enn_setting(0); //AVEE
	MDELAY(5);
	lcm_enp_setting(0); //AVDD
	MDELAY(5);
	LCM_INFO("[%s/%s]: lcm_suspend_power vddi keep high level\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
	panel_reset_state = 0;// clear reset state
}

static void lcm_init(void)
{
	LCM_INFO("[%s/%s]:init begin\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
	jadard_tp_gpio_reset(1);
	lcm_reset_setting(1);
	MDELAY(1);
	lcm_reset_setting(0);
	MDELAY(5);
	jadard_tp_gpio_reset(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(5);
	/*****************************************************************
	lrz modify it to 15ms from 90ms.	90ms is used to ensure flash download initial code.
	But PD1987 use no-flash version nt36525, so 15ms is enough, NolenCheng confirmed 0304
	******************************************************************/
	panel_reset_state = 1; // set reset state
	panel_off_state = 0;
	push_table(init_setting_12bit, sizeof(init_setting_12bit) / sizeof(struct LCM_setting_table), 1);
	MDELAY(5);
	jadard_tp_gpio_reset(1);
	MDELAY(5);
	push_table(init_setting_bringup, sizeof(init_setting_bringup) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("[%s/%s]:init end\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	panel_off_state = 1;
	LCM_INFO("[%s/%s]:lcm_suspend display off\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_reset_for_touch(void)
{
	lcm_reset_setting(0);
	MDELAY(2);
	lcm_reset_setting(1);
	MDELAY(12);
	panel_reset_state = 1;
	LCM_INFO("[%s/%s]:tp_control_suspend display off\n",LOG_PROJ_NAME, LOG_PANEL_TYPE);
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

	if (buffer[0] != 0x1c) {
		LCM_INFO("[LCM][LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_INFO("[LCM][LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
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

	LCM_INFO("[LCM]ATA check size = 0x%x, 0x%x, 0x%x, 0x%x\n",
		x0_MSB, x0_LSB, x1_MSB, x1_LSB);
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
	unsigned int level_original_12bit;
	level_original_12bit = level;
	//LCM_INFO("LastBacklight = %d, bl_level=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	level = (level*980+500)/1000;
	/*level =level*16;*/
	if ((level != 0) && (dimming_enable == 0) && (vivo_lcm_cabc_backlight != 0) && (dimming_for_camera == 1)) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);

	}
	if (level == 0) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 0;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	}

	if ((level > 0) && (level < 18))
		level = 18;

	bl_level_12bit[0].para_list[0] = (unsigned char)((level>>8)&0x0F);/*high 4 bit*/
	bl_level_12bit[0].para_list[1] = ((unsigned char)(level&0xFF));/*low 8 bit*/

	LCM_INFO("LastBacklight = %d, level_original_12bit=%d, level=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, level_original_12bit, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	cabc_push_table(handle, bl_level_12bit, sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);
	pre_bklt_level = level_original_12bit;
}

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_id_version == 0x93)
		return 0xA1;
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
    LCM_INFO("vincent-lcm_cabc_vivo_open dim:%d,level:%d\n", leveldimming, levelsetting);
}

static void lcm_cabc_vivo_close(void *handle, unsigned char level)
{
	if (sizeof(lcm_cmd_cabc_off))
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	 LCM_INFO("vincent-lcm_cabc_vivo_colose level:%d\n", level);
}

struct LCM_DRIVER pd2236_txd_jd9365t_hdplus_dsi_vdo_lcm_drv = {
	.name = "pd2236_txd_jd9365t_hdplus_dsi_vdo",
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
