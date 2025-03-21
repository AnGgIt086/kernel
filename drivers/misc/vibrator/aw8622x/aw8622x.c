/*
 * aw8622x.c
 *
 * Copyright (c) 2020 AWINIC Technology CO., LTD
 *
 * vun Author: Ray <yulei@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG
#define pr_fmt(fmt) "lra_haptic_aw8622x: " fmt
// #define dev_fmt(fmt) "awinic_haptics: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/vmalloc.h>
#include "aw8622x_reg.h"
#include "aw8622x.h"


#include "../vivo_haptic_core.h"
#include <linux/mman.h>

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8622x_DRIVER_VERSION			("v0.1.0")
#define AW8622x_I2C_NAME				("lra_haptic_aw8622x")
#define AW8622x_HAPTIC_NAME				"awinic_haptic" //miscdev name

#define RTP_SLAB_SIZE					512
#define RTP_BIN_MAX_SIZE				2000000 //用于加载RTP的固件
#define BASE_SCENE_COUNT_MAX			300

#define PLAYBACK_INFINITELY_RAM_ID		6 //用于长振的RAM_ID


#define SEQ_WAIT_UNIT					1280 //us
#define AW8622x_DUOBLE_CLICK_DELTA		30000 //us

#define AW_READ_CHIPID_RETRIES			(3)
#define AW_I2C_RETRIES					(2)
#define AW8622X_CHIP_ID					(0x00)
#define AW_REG_ID						(0x00)
#define AW8622X_REG_EFRD9				(0x64)

#define g_logDts 1


/******************************************************
 *
 * Value
 *
 ******************************************************/
static char *aw8622x_ram_name = "aw8622x_haptic.bin";
static struct kmem_cache *rtp_cachep;
static struct haptic_rtp_container *aw8622x_rtp;
static struct workqueue_struct *rtp_wq;
static int g_order;


struct pm_qos_request aw8622x_pm_qos_req_vb;

static struct haptic_wavefrom_info waveform_list_default[1] = {

	{1, 1, 8000, 12000, false, "default"},
};

static struct aw8622x *g_aw8622x;

static volatile int i2c_suspend;

static bool rtp_check_flag;

static bool at_test;



/***********************************************************
 *
 * prototype
 *
 ***********************************************************/
static int aw8622x_ram_work_init(struct aw8622x *aw8622x);
static int aw8622x_ram_update(struct aw8622x *aw8622x);

 /******************************************************
 *
 * aw8622x i2c write/read
 *
 ******************************************************/
static int aw8622x_i2c_write(struct aw8622x *aw8622x,
			unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	if (i2c_suspend) {
		dev_err(aw8622x->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&aw8622x->bus_lock);
	while (cnt < AW8622X_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw8622x->i2c, reg_addr, reg_data);
		if (ret < 0) {
			dev_err(aw8622x->dev, "%s: i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n",
				__func__, reg_addr, reg_data, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW8622X_I2C_RETRY_DELAY * 1000,
			AW8622X_I2C_RETRY_DELAY * 1000 + 500);
	}
	mutex_unlock(&aw8622x->bus_lock);
	return ret;
}

int aw8622x_i2c_read(struct aw8622x *aw8622x,
			unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	if (i2c_suspend) {
		dev_err(aw8622x->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8622x->bus_lock);
	while (cnt < AW8622X_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8622x->i2c, reg_addr);
		if (ret < 0) {
			dev_err(aw8622x->dev, "%s: i2c_read addr=0x%02X, cnt=%d error=%d\n",
				__func__, reg_addr, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW8622X_I2C_RETRY_DELAY * 1000,
				AW8622X_I2C_RETRY_DELAY * 1000 + 500);
	}
	mutex_unlock(&aw8622x->bus_lock);
	return ret;
}

static int aw8622x_i2c_writes(struct aw8622x *aw8622x,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = 0;
	unsigned char *data = NULL;

	if (i2c_suspend) {
		dev_err(aw8622x->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		data = kzalloc(len+1, GFP_KERNEL);
		if (data == NULL) {
			dev_err(aw8622x->dev, "%s: can not allocate memory\n", __func__);
			return  -ENOMEM;
		}
	} else {
		data = kmem_cache_alloc(rtp_cachep, GFP_KERNEL);
		if (!data) {
			dev_err(aw8622x->dev, "%s can not alloc cache memory\n", __func__);
			return -ENOMEM;
		}
	}
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	mutex_lock(&aw8622x->bus_lock);

	ret = i2c_master_send(aw8622x->i2c, data, len+1);
	if (ret < 0) {
		dev_err(aw8622x->dev, "%s: i2c master send error\n", __func__);
		//return rc;
	}

	mutex_unlock(&aw8622x->bus_lock);

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		if (data != NULL)
			kfree(data);
	} else {
		if (data != NULL)
			kmem_cache_free(rtp_cachep, data);
	}
	return ret;
}


static int aw8622x_i2c_write_bits(struct aw8622x *aw8622x,
			unsigned char reg_addr, unsigned int mask,
			unsigned char reg_data)
{
	int ret = -1;
	unsigned char reg_val = 0;
	if (i2c_suspend) {
		dev_err(aw8622x->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	ret = aw8622x_i2c_read(aw8622x, reg_addr, &reg_val);
	if (ret < 0) {
		dev_err(aw8622x->dev,
			"%s: i2c read error, ret=%d\n", __func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw8622x_i2c_write(aw8622x, reg_addr, reg_val);
	if (ret < 0) {
		dev_err(aw8622x->dev,
			"%s: i2c write error, ret=%d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static void aw8622x_haptic_auto_bst_enable(struct aw8622x *aw8622x, unsigned char flag) //
{

	if (flag) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_BRK_EN_MASK,
				AW8622X_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_BRK_EN_MASK,
				AW8622X_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static int aw8622x_haptic_vbat_mode_config(struct aw8622x *aw8622x, unsigned char flag) //
{

	if (flag == AW8622X_HAPTIC_CONT_VBAT_HW_ADJUST_MODE) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
				AW8622X_BIT_SYSCTRL1_VBAT_MODE_MASK,
				AW8622X_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
				AW8622X_BIT_SYSCTRL1_VBAT_MODE_MASK,
				AW8622X_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
	return 0;
}

static void aw8622x_haptic_raminit(struct aw8622x *aw8622x, bool flag) //
{

	if (flag) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
				AW8622X_BIT_SYSCTRL1_RAMINIT_MASK,
				AW8622X_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
				AW8622X_BIT_SYSCTRL1_RAMINIT_MASK,
				AW8622X_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static int aw8622x_haptic_set_gain(struct aw8622x *aw8622x, unsigned char gain)//
{

	aw8622x_i2c_write(aw8622x, AW8622X_REG_PLAYCFG2, gain);
	return 0;
}

static int aw8622x_haptic_softreset(struct aw8622x *aw8622x)
{
	aw8622x_i2c_write(aw8622x, AW_REG_ID, 0xAA);
	usleep_range(2000, 2500);
	return 0;
}

static unsigned char aw8622x_haptic_rtp_get_fifo_afs(struct aw8622x *aw8622x)//
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw8622x_i2c_read(aw8622x, AW8622X_REG_SYSST, &reg_val);
	reg_val &= AW8622X_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;
	return ret;
}

static void aw8622x_haptic_set_rtp_aei(struct aw8622x *aw8622x, bool flag)//
{
	if (flag) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
			AW8622X_BIT_SYSINTM_FF_AEM_MASK,
			AW8622X_BIT_SYSINTM_FF_AEM_ON);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
			AW8622X_BIT_SYSINTM_FF_AEM_MASK,
			AW8622X_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8622x_interrupt_clear(struct aw8622x *aw8622x) //
{
	unsigned char reg_val = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_SYSINT, &reg_val);
	dev_dbg(aw8622x->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);
}

static void aw8622x_haptic_upload_lra(struct aw8622x *aw8622x, unsigned int flag) //
{

	switch (flag) {
	case WRITE_ZERO:
		dev_info(aw8622x->dev, "%s write zero to trim_lra!\n", __func__);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_TRIMCFG3,
				AW8622X_BIT_TRIMCFG3_TRIM_LRA_MASK, 0x00);
		break;

	case F0_CALI:
		dev_info(aw8622x->dev, "%s write f0_cali_data to trim_lra = 0x%02X\n",
			__func__, aw8622x->f0_cali_data);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_TRIMCFG3,
				AW8622X_BIT_TRIMCFG3_TRIM_LRA_MASK,
				(unsigned char)aw8622x->f0_cali_data);
		break;

	case OSC_CALI:
		dev_info(aw8622x->dev, "%s write osc_cali_data to trim_lra = 0x%02X\n",
			__func__, aw8622x->osc_cali_data);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_TRIMCFG3,
				AW8622X_BIT_TRIMCFG3_TRIM_LRA_MASK,
				(unsigned char)aw8622x->osc_cali_data);
		break;

	default:
		break;
	}
}


/*****************************************************
 *
 * sram size, normally 3k(2k fifo, 1k ram)
 *
 *****************************************************/
static int aw8622x_sram_size(struct aw8622x *aw8622x, int size_flag)//
{

	if (size_flag == AW8622X_HAPTIC_SRAM_2K) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_DIS);
	} else if (size_flag == AW8622X_HAPTIC_SRAM_1K) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_DIS);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
	} else if (size_flag == AW8622X_HAPTIC_SRAM_3K) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
				AW8622X_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
	}
	return 0;
}


static void aw8622x_hw_reset(struct aw8622x *aw8622x)
{
	gpio_set_value_cansleep(aw8622x->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpio_set_value_cansleep(aw8622x->reset_gpio, 1);
	usleep_range(3500, 4000);
}


static void aw8622x_interrupt_setup(struct aw8622x *aw8622x) //
{
	unsigned char reg_val = 0;

	aw8622x_i2c_read(aw8622x, AW8622X_REG_SYSINT, &reg_val);

	dev_info(aw8622x->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);

	/* edge int mode */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
			AW8622X_BIT_SYSCTRL7_INT_MODE_MASK,
			AW8622X_BIT_SYSCTRL7_INT_MODE_EDGE);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
			AW8622X_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			AW8622X_BIT_SYSCTRL7_INT_EDGE_MODE_POS);
	/* int enable */
	/*
	*aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
	*			AW8622X_BIT_SYSINTM_BST_SCPM_MASK,
	*			AW8622X_BIT_SYSINTM_BST_SCPM_OFF);
	*aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
	*			AW8622X_BIT_SYSINTM_BST_OVPM_MASK,
	*		AW8622X_BIT_SYSINTM_BST_OVPM_ON);
	*/
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
			AW8622X_BIT_SYSINTM_UVLM_MASK,
			AW8622X_BIT_SYSINTM_UVLM_ON);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
			AW8622X_BIT_SYSINTM_OCDM_MASK,
			AW8622X_BIT_SYSINTM_OCDM_ON);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
		AW8622X_BIT_SYSINTM_OTM_MASK,
		AW8622X_BIT_SYSINTM_OTM_ON);

#if 0
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
	AW8622X_BIT_SYSINTM_FF_AEM_MASK,
	AW8622X_BIT_SYSINTM_FF_AEM_ON);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
	AW8622X_BIT_SYSINTM_FF_AFM_MASK,
	AW8622X_BIT_SYSINTM_FF_AFM_ON);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
	AW8622X_BIT_SYSINTM_DONEM_MASK,
	AW8622X_BIT_SYSINTM_DONEM_ON);
#endif
}

static int aw8622x_haptic_set_pwm(struct aw8622x *aw8622x, unsigned char mode) //
{

	switch (mode) {
	case AW8622X_PWM_48K:
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
				AW8622X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				AW8622X_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW8622X_PWM_24K:
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
				AW8622X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				AW8622X_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW8622X_PWM_12K:
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
				AW8622X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				AW8622X_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8622x_haptic_play_go(struct aw8622x *aw8622x, bool flag) //
{
	unsigned char reg_val = 0;


	if (flag == true) {
		aw8622x_i2c_write(aw8622x, AW8622X_REG_PLAYCFG4, 0x01);
		mdelay(2);
	} else {
		aw8622x_i2c_write(aw8622x, AW8622X_REG_PLAYCFG4, 0x02);
	}

	aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &reg_val);
	return 0;
}

static int aw8622x_haptic_juge_RTP_is_going_on(struct aw8622x *aw8622x) //
{
	unsigned char rtp_state = 0;
	unsigned char mode = 0;
	unsigned char glb_st = 0;

	aw8622x_i2c_read(aw8622x, AW8622X_REG_PLAYCFG3, &mode);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &glb_st);
	if ((mode & AW8622X_BIT_PLAYCFG3_PLAY_MODE_RTP) &&
		(glb_st == AW8622X_BIT_GLBRD5_STATE_RTP_GO)) {
		rtp_state = 1;
	}
	return rtp_state;
}


static int aw8622x_haptic_swicth_motor_protect_config(struct aw8622x *aw8622x, //
				unsigned char addr, unsigned char val)
{

	if (addr == 1) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG1,
				AW8622X_BIT_DETCFG1_PRCT_MODE_MASK,
				AW8622X_BIT_DETCFG1_PRCT_MODE_VALID);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG1,
				AW8622X_BIT_PWMCFG1_PRC_EN_MASK,
				AW8622X_BIT_PWMCFG1_PRC_ENABLE);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG3,
				AW8622X_BIT_PWMCFG3_PR_EN_MASK,
				AW8622X_BIT_PWMCFG3_PR_ENABLE);
	} else if (addr == 0) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG1,
				AW8622X_BIT_DETCFG1_PRCT_MODE_MASK,
				AW8622X_BIT_DETCFG1_PRCT_MODE_INVALID);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG1,
				AW8622X_BIT_PWMCFG1_PRC_EN_MASK,
				AW8622X_BIT_PWMCFG1_PRC_DISABLE);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG3,
				AW8622X_BIT_PWMCFG3_PR_EN_MASK,
				AW8622X_BIT_PWMCFG3_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG1,
				AW8622X_BIT_PWMCFG1_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PWMCFG3,
				AW8622X_BIT_PWMCFG3_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8622x_i2c_write(aw8622x, AW8622X_REG_PWMCFG4, val);
	}
	return 0;
}


static int aw8622x_read_chipid(struct aw8622x *aw8622x)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;
	unsigned char ef_id = 0xff;

	while (cnt < AW_READ_CHIPID_RETRIES) {

		ret = aw8622x_i2c_read(aw8622x, AW_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8622x->dev, "%s: failed to read register AW_REG_ID: %d\n", __func__, ret);
			aw8622x_hw_reset(aw8622x);
			cnt++;
			continue;
		}

		switch (reg) {

		case AW8622X_CHIP_ID:
			/* Distinguish products by AW8622X_REG_EFRD9. */
			aw8622x_i2c_read(aw8622x, AW8622X_REG_EFRD9, &ef_id);
			if ((ef_id & 0x41) == AW86224_5_EF_ID) {
				dev_info(aw8622x->dev, "%s aw86224_5 detected\n", __func__);
				aw8622x_haptic_softreset(aw8622x);
				return 0;
			} else if ((ef_id & 0x41) == AW86223_EF_ID) {
				dev_info(aw8622x->dev, "%s aw86223 detected\n", __func__);
				aw8622x_haptic_softreset(aw8622x);
				return 0;
			} else {
				dev_info(aw8622x->dev, "%s unsupported ef_id = (0x%02X)\n", __func__, ef_id);
				break;
			}
		default:
			dev_info(aw8622x->dev, "%s unsupported device revision (0x%x)\n", __func__, reg);
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return -EINVAL;
}


static int aw8622x_haptic_activate(struct aw8622x *aw8622x)//
{

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
			AW8622X_BIT_SYSCTRL2_STANDBY_MASK,
			AW8622X_BIT_SYSCTRL2_STANDBY_OFF);
	aw8622x_interrupt_clear(aw8622x);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSINTM,
			AW8622X_BIT_SYSINTM_UVLM_MASK,
			AW8622X_BIT_SYSINTM_UVLM_ON);
	return 0;
}

static int aw8622x_haptic_stop(struct aw8622x *aw8622x)//
{
	unsigned char cnt = 40;
	unsigned char reg_val = 0;
	bool force_flag = true;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x->play_mode = AW8622X_HAPTIC_STANDBY_MODE;
	aw8622x_i2c_write(aw8622x, AW8622X_REG_PLAYCFG4, 0x02);
	while (cnt) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00
				|| (reg_val & 0x0f) == 0x0A) {
			cnt = 0;
			force_flag = false;
			dev_info(aw8622x->dev, "%s entered standby! glb_state=0x%02X\n",
					__func__, reg_val);
		} else {
			cnt--;
			dev_dbg(aw8622x->dev, "%s wait for standby, glb_state=0x%02X\n",
					__func__, reg_val);
		}
		usleep_range(2000, 2500);
	}

	if (force_flag) {
		dev_err(aw8622x->dev, "%s force to enter standby mode!\n",
				__func__);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
				AW8622X_BIT_SYSCTRL2_STANDBY_MASK,
				AW8622X_BIT_SYSCTRL2_STANDBY_ON);
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
				AW8622X_BIT_SYSCTRL2_STANDBY_MASK,
				AW8622X_BIT_SYSCTRL2_STANDBY_OFF);
	}
	return 0;
}

static int aw8622x_haptic_play_mode(struct aw8622x *aw8622x, unsigned char play_mode) //
{

	switch (play_mode) {
	case AW8622X_HAPTIC_STANDBY_MODE:
		dev_info(aw8622x->dev, "%s: enter standby mode\n", __func__);
		aw8622x->play_mode = AW8622X_HAPTIC_STANDBY_MODE;
		aw8622x_haptic_stop(aw8622x);
		break;
	case AW8622X_HAPTIC_RAM_MODE:
		dev_info(aw8622x->dev, "%s: enter ram mode\n", __func__);
		aw8622x->play_mode = AW8622X_HAPTIC_RAM_MODE;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW8622X_HAPTIC_RAM_LOOP_MODE:
		dev_info(aw8622x->dev, "%s: enter ram loop mode\n",
				__func__);
		aw8622x->play_mode = AW8622X_HAPTIC_RAM_LOOP_MODE;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW8622X_HAPTIC_RTP_MODE:
		dev_info(aw8622x->dev, "%s: enter rtp mode\n", __func__);
		aw8622x->play_mode = AW8622X_HAPTIC_RTP_MODE;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_RTP);
		break;
	case AW8622X_HAPTIC_TRIG_MODE:
		dev_info(aw8622x->dev, "%s: enter trig mode\n", __func__);
		aw8622x->play_mode = AW8622X_HAPTIC_TRIG_MODE;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW8622X_HAPTIC_CONT_MODE:
		dev_info(aw8622x->dev, "%s: enter cont mode\n", __func__);
		aw8622x->play_mode = AW8622X_HAPTIC_CONT_MODE;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK,
				AW8622X_BIT_PLAYCFG3_PLAY_MODE_CONT);
		break;
	default:
		dev_err(aw8622x->dev, "%s: play mode %d error",
				__func__, play_mode);
		break;
	}
	return 0;
}

static void aw8622x_haptic_misc_para_init(struct aw8622x *aw8622x) //
{

	aw8622x->cont_drv1_lvl = aw8622x->dts_info.cont_drv1_lvl_dt;
	aw8622x->cont_drv2_lvl = aw8622x->dts_info.cont_drv2_lvl_dt;
	aw8622x->cont_drv1_time = aw8622x->dts_info.cont_drv1_time_dt;
	aw8622x->cont_drv2_time = aw8622x->dts_info.cont_drv2_time_dt;
	aw8622x->cont_brk_time = aw8622x->dts_info.cont_brk_time_dt;
	aw8622x->cont_wait_num = aw8622x->dts_info.cont_wait_num_dt;
	/* SIN_H */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_SYSCTRL3,
			aw8622x->dts_info.sine_array[0]);
	/* SIN_L */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_SYSCTRL4,
			aw8622x->dts_info.sine_array[1]);
	/* COS_H */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_SYSCTRL5,
				aw8622x->dts_info.sine_array[2]);
	/* COS_L */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_SYSCTRL6,
			aw8622x->dts_info.sine_array[3]);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_TRGCFG8,
				AW8622X_BIT_TRGCFG8_TRG_TRIG1_MODE_MASK,
				AW8622X_BIT_TRGCFG8_TRIG1);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_ANACFG8,
					AW8622X_BIT_ANACFG8_TRTF_CTRL_HDRV_MASK,
					AW8622X_BIT_ANACFG8_TRTF_CTRL_HDRV);

	/* d2s_gain */
	if (!aw8622x->dts_info.d2s_gain) {
		dev_err(aw8622x->dev, "%s aw8622x->dts_info.d2s_gain = 0!\n",
				__func__);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
				AW8622X_BIT_SYSCTRL7_D2S_GAIN_MASK,
				aw8622x->dts_info.d2s_gain);
	}

	/* cont_tset */
	if (!aw8622x->dts_info.cont_tset) {
		dev_err(aw8622x->dev,
			"%s aw8622x->dts_info.cont_tset = 0!\n", __func__);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG13,
				AW8622X_BIT_CONTCFG13_TSET_MASK,
				aw8622x->dts_info.cont_tset << 4);
	}

	/* cont_bemf_set */
	if (!aw8622x->dts_info.cont_bemf_set) {
		dev_err(aw8622x->dev, "%s aw8622x->dts_info.cont_bemf_set = 0!\n",
			__func__);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG13,
			AW8622X_BIT_CONTCFG13_BEME_SET_MASK,
			aw8622x->dts_info.cont_bemf_set);
	}

	/* cont_brk_time */
	if (!aw8622x->cont_brk_time) {
		dev_err(aw8622x->dev, "%s aw8622x->cont_brk_time = 0!\n",
			__func__);
	} else {
		aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG10,
			aw8622x->cont_brk_time);
	}

	/* cont_brk_gain */
	if (!aw8622x->dts_info.cont_brk_gain) {
		dev_err(aw8622x->dev, "%s aw8622x->dts_info.cont_brk_gain = 0!\n",
			__func__);
	} else {
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG5,
				AW8622X_BIT_CONTCFG5_BRK_GAIN_MASK,
				aw8622x->dts_info.cont_brk_gain);
	}
}

static int aw8622x_haptic_offset_calibration(struct aw8622x *aw8622x) //
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);

	aw8622x_haptic_raminit(aw8622x, true);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG2,
			AW8622X_BIT_DETCFG2_DIAG_GO_MASK,
			AW8622X_BIT_DETCFG2_DIAG_GO_ON);
	while (1) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_DETCFG2, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	if (cont == 0)
		dev_err(aw8622x->dev, "%s calibration offset failed!\n",
			__func__);
	aw8622x_haptic_raminit(aw8622x, false);
	return 0;
}

static int aw8622x_haptic_get_vbat(struct aw8622x *aw8622x)//
{
	unsigned char reg_val = 0;
	unsigned int vbat_code = 0;
	/*unsigned int cont = 2000;*/

	aw8622x_haptic_stop(aw8622x);
	aw8622x_haptic_raminit(aw8622x, true);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG2,
				AW8622X_BIT_DETCFG2_VBAT_GO_MASK,
				AW8622X_BIT_DETCFG2_VABT_GO_ON);
	usleep_range(20000, 25000);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_DET_VBAT, &reg_val);
	vbat_code = (vbat_code | reg_val) << 2;
	aw8622x_i2c_read(aw8622x, AW8622X_REG_DET_LO, &reg_val);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw8622x->vbat = 6100 * vbat_code / 1024;
	if (aw8622x->vbat > AW8622X_VBAT_MAX) {
		aw8622x->vbat = AW8622X_VBAT_MAX;
		dev_info(aw8622x->dev, "%s vbat max limit = %dmV\n",
			__func__, aw8622x->vbat);
	}
	if (aw8622x->vbat < AW8622X_VBAT_MIN) {
		aw8622x->vbat = AW8622X_VBAT_MIN;
		dev_info(aw8622x->dev, "%s vbat min limit = %dmV\n",
				__func__, aw8622x->vbat);
	}
	dev_info(aw8622x->dev, "%s aw8622x->vbat=%dmV, vbat_code=0x%02X\n",
				__func__, aw8622x->vbat, vbat_code);
	aw8622x_haptic_raminit(aw8622x, false);
	return 0;
}

/*****************************************************
 *
 * rtp brk
 *
 *****************************************************/
/*
*static int aw8622x_rtp_brake_set(struct aw8622x *aw8622x) {
*     aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG1,
*			AW8622X_BIT_CONTCFG1_MBRK_MASK,
*			AW8622X_BIT_CONTCFG1_MBRK_ENABLE);
*
*     aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
*			AW8622X_BIT_SYSCTRL7_D2S_GAIN_MASK,
*			0x05);
*     return 0;
*}
*/



static aw8622x_set_play_vol(struct aw8622x *aw8622x)
{
	int ret = 0;
	u8 gain_val;

	//如果aw8622x->play.vmax小于6000，gain_val根据aw8622x->play.vmax计算得出
	if (aw8622x->play.vmax < 6000) {
		gain_val = aw8622x->play.vmax * 128 / 6000;
	} else { //如果aw8622x->play.vmax大于等于6000，gain_val = 0x80
		gain_val = 0x80;
	}
	dev_info(aw8622x->dev, "%s<%d> aw8622x gain_val=%d\n", __func__, __LINE__, gain_val);
	//根据之前的变量值，对电压和增益进行设置
	aw8622x_haptic_set_gain(aw8622x, gain_val);

	return ret;
}


static int aw8622x_haptic_ram_vbat_compensate(struct aw8622x *aw8622x, //
					bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw8622x->ram_vbat_compensate ==
				AW8622X_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw8622x_haptic_get_vbat(aw8622x);
			temp_gain = aw8622x->gain * AW8622X_VBAT_REFER / aw8622x->vbat;
			if (temp_gain >
				(128 * AW8622X_VBAT_REFER / AW8622X_VBAT_MIN)) {
				temp_gain = 128 * AW8622X_VBAT_REFER / AW8622X_VBAT_MIN;
				dev_dbg(aw8622x->dev, "%s gain limit=%d\n",
						__func__, temp_gain);
			}
			aw8622x_haptic_set_gain(aw8622x, temp_gain);
		} else {
			aw8622x_haptic_set_gain(aw8622x, aw8622x->gain);
		}
	} else {
		aw8622x_haptic_set_gain(aw8622x, aw8622x->gain);
	}
	return 0;
}



static int aw8622x_haptic_start(struct aw8622x *aw8622x) //
{

	aw8622x_haptic_activate(aw8622x);
	aw8622x_haptic_play_go(aw8622x, true);
	return 0;
}


static int aw8622x_haptic_set_wav_seq(struct aw8622x *aw8622x, //
				unsigned char wav, unsigned char seq)
{

	aw8622x_i2c_write(aw8622x, AW8622X_REG_WAVCFG1 + wav, seq);
	return 0;
}

static int aw8622x_haptic_set_wav_loop(struct aw8622x *aw8622x, //
				unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_WAVCFG9 + (wav / 2),
				AW8622X_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_WAVCFG9 + (wav / 2),
				AW8622X_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
	return 0;
}


static int aw8622x_haptic_set_repeat_wav_seq(struct aw8622x *aw8622x, //
				unsigned char seq)
{

	aw8622x_haptic_set_wav_seq(aw8622x, 0x00, seq);
	aw8622x_haptic_set_wav_loop(aw8622x, 0x00,
				AW8622X_BIT_WAVLOOP_INIFINITELY);
	return 0;
}

static int aw8622x_haptic_play_repeat_seq(struct aw8622x *aw8622x, //
			unsigned char flag)
{

	if (flag) {
		aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_RAM_LOOP_MODE);
		aw8622x_haptic_play_go(aw8622x, true);
	}
	return 0;
}


static int aw8622x_haptic_play_wav_seq(struct aw8622x *aw8622x,
			unsigned char flag)
{
	if (flag) {
		aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_RAM_MODE);
		aw8622x_haptic_start(aw8622x);
	}
	return 0;
}

static void aw8622x_double_click_switch(struct aw8622x *aw8622x, bool sw)
{

	if (sw) {
		aw8622x_haptic_set_wav_seq(aw8622x, 0x00, 0x01);
		aw8622x_haptic_set_wav_seq(aw8622x, 0x01, ((AW8622x_DUOBLE_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);//第7位为1时，表示波形间的等待时间
		aw8622x_haptic_set_wav_seq(aw8622x, 0x02, 0x01);
	} else {
		aw8622x_haptic_set_wav_seq(aw8622x, 0x00, 0x00);
		aw8622x_haptic_set_wav_seq(aw8622x, 0x01, 0x00);
		aw8622x_haptic_set_wav_seq(aw8622x, 0x02, 0x00);
	}
}

/*****************************************************
 *
 * f0 calibration
 *
 *****************************************************/

static int aw8622x_haptic_read_lra_f0(struct aw8622x *aw8622x) //
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	/* F_LRA_F0_H */
	ret = aw8622x_i2c_read(aw8622x, AW8622X_REG_CONTRD14, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	/* F_LRA_F0_L */
	ret = aw8622x_i2c_read(aw8622x, AW8622X_REG_CONTRD15, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		dev_err(aw8622x->dev, "%s didn't get lra f0 because f0_reg value is 0!\n",
				__func__);
		aw8622x->f0 = aw8622x->dts_info.f0_ref;
		return -1;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw8622x->f0 = (unsigned int)f0_tmp;
		dev_info(aw8622x->dev, "%s lra_f0=%d\n", __func__,
				aw8622x->f0);
	}

	return 0;
}

static int aw8622x_haptic_read_cont_f0(struct aw8622x *aw8622x) //
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	ret = aw8622x_i2c_read(aw8622x, AW8622X_REG_CONTRD16, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	ret = aw8622x_i2c_read(aw8622x, AW8622X_REG_CONTRD17, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		dev_err(aw8622x->dev, "%s didn't get cont f0 because f0_reg value is 0!\n",
				__func__);
		aw8622x->cont_f0 = aw8622x->dts_info.f0_ref;
		return -1;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw8622x->cont_f0 = (unsigned int)f0_tmp;
		dev_info(aw8622x->dev, "%s cont_f0=%d\n", __func__,
				aw8622x->cont_f0);
	}
	return 0;
}


static int aw8622x_haptic_cont_get_f0(struct aw8622x *aw8622x) //
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;
	unsigned char brk_en_temp = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x->f0 = aw8622x->dts_info.f0_ref;
	/* enter standby mode */
	aw8622x_haptic_stop(aw8622x);
	/* f0 calibrate work mode */
	aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_CONT_MODE);
	/* enable f0 detect */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG1,
				AW8622X_BIT_CONTCFG1_EN_F0_DET_MASK,
				AW8622X_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG6,
				AW8622X_BIT_CONTCFG6_TRACK_EN_MASK,
				AW8622X_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto brake */
	aw8622x_i2c_read(aw8622x, AW8622X_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_BRK_EN_MASK,
				AW8622X_BIT_PLAYCFG3_BRK_ENABLE);
	/* f0 driver level */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG6,
			AW8622X_BIT_CONTCFG6_DRV1_LVL_MASK,
			aw8622x->dts_info.cont_drv1_lvl_dt);
	aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG7,
			aw8622x->dts_info.cont_drv2_lvl_dt);
	/* DRV1_TIME */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG8,
			aw8622x->dts_info.cont_drv1_time_dt);
	/* DRV2_TIME */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG9,
			aw8622x->dts_info.cont_drv2_time_dt);
	/* TRACK_MARGIN */
	if (!aw8622x->dts_info.cont_track_margin) {
		dev_err(aw8622x->dev, "%s aw8622x->dts_info.cont_track_margin = 0!\n",
			__func__);
	} else {
		aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG11,
				(unsigned char)aw8622x->dts_info.
				cont_track_margin);
	}
	/* DRV_WIDTH */
	/*
	 * aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG3,
	 *		aw8622x->dts_info.cont_drv_width);
	 */
	/* cont play go */
	aw8622x_haptic_play_go(aw8622x, true);
	/* 300ms */
	while (cnt) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			dev_info(aw8622x->dev, "%s entered standby mode! glb_state=0x%02X\n",
				__func__, reg_val);
		} else {
			cnt--;
			dev_info(aw8622x->dev, "%s waitting for standby, glb_state=0x%02X\n",
				__func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw8622x_haptic_read_lra_f0(aw8622x);
		aw8622x_haptic_read_cont_f0(aw8622x);
	} else {
		dev_err(aw8622x->dev, "%s enter standby mode failed, stop reading f0!\n",
			__func__);
	}
	/* restore default config */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG1,
				AW8622X_BIT_CONTCFG1_EN_F0_DET_MASK,
				AW8622X_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
				AW8622X_BIT_PLAYCFG3_BRK_EN_MASK,
				brk_en_temp);
	return ret;
}


static int aw8622x_haptic_get_lra_resistance(struct aw8622x *aw8622x) //
{
	unsigned char reg_val = 0;
	unsigned char d2s_gain_temp = 0;
	unsigned int lra_code = 0;
	unsigned int lra = 0;


	aw8622x_haptic_stop(aw8622x);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
			AW8622X_BIT_SYSCTRL7_D2S_GAIN_MASK,
			aw8622x->dts_info.d2s_gain);
	aw8622x_haptic_raminit(aw8622x, true);
	/* enter standby mode */
	aw8622x_haptic_stop(aw8622x);
	usleep_range(2000, 2500);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL2,
			AW8622X_BIT_SYSCTRL2_STANDBY_MASK,
			AW8622X_BIT_SYSCTRL2_STANDBY_OFF);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG1,
			AW8622X_BIT_DETCFG1_RL_OS_MASK,
			AW8622X_BIT_DETCFG1_RL);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_DETCFG2,
			AW8622X_BIT_DETCFG2_DIAG_GO_MASK,
			AW8622X_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_DET_RL, &reg_val);
	lra_code = (lra_code | reg_val) << 2;
	aw8622x_i2c_read(aw8622x, AW8622X_REG_DET_LO, &reg_val);
	lra_code = lra_code | (reg_val & 0x03);
	/* 2num */
	lra = (lra_code * 678 * 100) / (1024 * 10);
	/* Keep up with aw8622x driver */
	aw8622x->lra = lra * 10;
	aw8622x_haptic_raminit(aw8622x, false);
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL7,
			AW8622X_BIT_SYSCTRL7_D2S_GAIN_MASK,
			d2s_gain_temp);
	return 0;
}



static int aw8622x_haptic_f0_calibration(struct aw8622x *aw8622x) //
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	unsigned int f0_cali_min = aw8622x->dts_info.f0_ref *
				(100 - aw8622x->dts_info.f0_cali_percent) / 100;
	unsigned int f0_cali_max =  aw8622x->dts_info.f0_ref *
				(100 + aw8622x->dts_info.f0_cali_percent) / 100;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	/*
	 * aw8622x_haptic_upload_lra(aw8622x, WRITE_ZERO);
	 */
	if (aw8622x_haptic_cont_get_f0(aw8622x)) {
		dev_err(aw8622x->dev, "%s get f0 error, user defafult f0\n",
			__func__);
	} else {
		/* max and min limit */
		f0_limit = aw8622x->f0;
		dev_info(aw8622x->dev, "%s f0_ref = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d\n",
			__func__, aw8622x->dts_info.f0_ref,
				f0_cali_min, f0_cali_max, aw8622x->f0);

		if ((aw8622x->f0 < f0_cali_min) || aw8622x->f0 > f0_cali_max) {
			dev_err(aw8622x->dev, "%s f0 calibration out of range = %d!\n",
				__func__, aw8622x->f0);
			f0_limit = aw8622x->dts_info.f0_ref;
			return -ERANGE;
		}
		dev_info(aw8622x->dev, "%s f0_limit = %d\n", __func__,
				(int)f0_limit);
		/* calculate cali step */
		f0_cali_step = 100000 * ((int)f0_limit -
					 (int)aw8622x->dts_info.f0_ref) /
				((int)f0_limit * 24);
		dev_info(aw8622x->dev, "%s f0_cali_step = %d\n", __func__,
			f0_cali_step);
		if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = 32 + (f0_cali_step / 10 + 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		} else {	/* f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}
		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
		/* update cali step */
		aw8622x->f0_cali_data = f0_cali_lra;
		aw8622x_haptic_upload_lra(aw8622x, F0_CALI);

		aw8622x_i2c_read(aw8622x, AW8622X_REG_TRIMCFG3, &reg_val);

		dev_info(aw8622x->dev, "%s final trim_lra=0x%02x\n",
			__func__, reg_val);
	}
	/* restore standby work mode */
	aw8622x_haptic_stop(aw8622x);
	return ret;
}


/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw8622x_haptic_cont_config(struct aw8622x *aw8622x) //
{
	dev_info(aw8622x->dev, "%s enter\n", __func__);

	/* work mode */
	aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_CONT_MODE);
	/* cont config */
	/* aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG1,
	 ** AW8622X_BIT_CONTCFG1_EN_F0_DET_MASK,
	 ** AW8622X_BIT_CONTCFG1_F0_DET_ENABLE);
	 */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG6,
			AW8622X_BIT_CONTCFG6_TRACK_EN_MASK,
			AW8622X_BIT_CONTCFG6_TRACK_ENABLE);
	/* f0 driver level */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_CONTCFG6,
			AW8622X_BIT_CONTCFG6_DRV1_LVL_MASK,
			aw8622x->cont_drv1_lvl);
	aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG7,
		aw8622x->cont_drv2_lvl);
	/* DRV1_TIME */
	/* aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG8, 0xFF); */
	/* DRV2_TIME */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_CONTCFG9, 0xFF);
	/* cont play go */
	aw8622x_haptic_play_go(aw8622x, true);
	return 0;
}



static int aw8622x_haptic_rtp_init(struct aw8622x *aw8622x)
{
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;
	//int ret = 0, retval = 0;
	int retval = 0;

	dev_info(aw8622x->dev, "%s enter\n", __func__);

	pm_qos_add_request(&aw8622x_pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, AW8622X_PM_QOS_VALUE_VB);

	aw8622x->rtp_cnt = 0;
	mutex_lock(&aw8622x->rtp_lock);

	while ((!aw8622x_haptic_rtp_get_fifo_afs(aw8622x)) && (aw8622x->play_mode == AW8622X_HAPTIC_RTP_MODE)) {

		if (!aw8622x_rtp) {
			dev_info(aw8622x->dev, "%s:aw8622x_rtp is null, break!\n", __func__);
			break;
		}

		if (aw8622x->rtp_cnt < (aw8622x->ram.base_addr)) {
			if ((aw8622x_rtp->len - aw8622x->rtp_cnt) < (aw8622x->ram.base_addr)) {
				buf_len = aw8622x_rtp->len - aw8622x->rtp_cnt;
			} else {
				buf_len = aw8622x->ram.base_addr;
			}
		} else if ((aw8622x_rtp->len - aw8622x->rtp_cnt) < (aw8622x->ram.base_addr >> 2)) {
				buf_len = aw8622x_rtp->len - aw8622x->rtp_cnt;
		} else {
			buf_len = aw8622x->ram.base_addr >> 2;
		}


		dev_info(aw8622x->dev, "%s rtp cnt = %d, buf_len = %d\n", __func__, aw8622x->rtp_cnt, buf_len);

		aw8622x_i2c_writes(aw8622x, AW8622X_REG_RTPDATA, &aw8622x_rtp->data[aw8622x->rtp_cnt], buf_len);
		aw8622x->rtp_cnt += buf_len;

		aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &glb_state_val);
		if ((aw8622x->rtp_cnt == aw8622x_rtp->len) || ((glb_state_val & 0x0f) == 0x00)) {
			if (aw8622x->rtp_cnt == aw8622x_rtp->len)
				dev_info(aw8622x->dev, "%s: rtp load completely! glb_state_val=%02x aw8622x->rtp_cnt=%02x\n",
						__func__, glb_state_val, aw8622x->rtp_cnt);
			else
				dev_err(aw8622x->dev, "%s rtp load failed!! glb_state_val=%02x aw8622x->rtp_cnt=%02x\n",
						__func__, glb_state_val, aw8622x->rtp_cnt);
			aw8622x->rtp_cnt = 0;
			mutex_unlock(&aw8622x->rtp_lock);
			pm_qos_remove_request(&aw8622x_pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		dev_info(aw8622x->dev, "%s: i2c read error--->2: total length: %d, play length: %d\n",
			__func__, aw8622x_rtp->len, aw8622x->rtp_cnt);
		aw8622x->rtp_cnt = 0;
		mutex_unlock(&aw8622x->rtp_lock);
		pm_qos_remove_request(&aw8622x_pm_qos_req_vb);
		return 0;
	}


	if ((aw8622x->play_mode == AW8622X_HAPTIC_RTP_MODE) && (retval >= 0)) {
		dev_info(aw8622x->dev, "%s open rtp irq\n", __func__);
		aw8622x_haptic_set_rtp_aei(aw8622x, true);
	}


	dev_info(aw8622x->dev, "%s exit\n", __func__);
	mutex_unlock(&aw8622x->rtp_lock);
	pm_qos_remove_request(&aw8622x_pm_qos_req_vb);
	return 0;
}

static void aw8622x_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8622x *aw8622x = container_of(work, struct aw8622x, rtp_work);
	struct aw8622x_play_info *play = &aw8622x->play;
	struct haptic_wavefrom_info *effect_list = aw8622x->effect_list;
	int gain;
	char file_name[128] = {0};


	dev_info(aw8622x->dev, "%s enter\n", __func__);

	if (play->type != RTP_TYPE && play->type != RTP_MMAP_TYPE) {
		dev_err(aw8622x->dev, "new not rtp effect coming\n");
		return;
	}
	if (play->type == RTP_TYPE) {

		if (aw8622x->add_suffix) {
			snprintf(file_name, 128, "%s%s", "_12k_", effect_list[aw8622x->rtp_file_num].rtp_file_name);
		} else {
			strlcpy(file_name, effect_list[aw8622x->rtp_file_num].rtp_file_name, 128);
		}

		ret = request_firmware(&rtp_file, file_name, aw8622x->dev);
		if (ret < 0) {
			dev_info(aw8622x->dev, "%s: failed to read [%s]\n", __func__, file_name);
			return;
		}

		mutex_lock(&aw8622x->rtp_lock);
		aw8622x->rtp_init = 0;

		aw8622x_haptic_set_rtp_aei(aw8622x, false);

		if (aw8622x_rtp == NULL) {
			aw8622x_rtp = devm_kzalloc(aw8622x->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
			if (aw8622x_rtp == NULL) {
				dev_info(aw8622x->dev, "%s devm kzalloc failed\n", __func__);
				release_firmware(rtp_file);
				mutex_unlock(&aw8622x->rtp_lock);
				return;
			}
		}

		memset(aw8622x_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));
		dev_info(aw8622x->dev, "%s: rtp file [%s] size = %d\n", __func__,
					effect_list[aw8622x->rtp_file_num].rtp_file_name, rtp_file->size);

		if (rtp_file->size < RTP_BIN_MAX_SIZE)
			aw8622x_rtp->len = rtp_file->size;
		else
			aw8622x_rtp->len = RTP_BIN_MAX_SIZE;

		memcpy(aw8622x_rtp->data, rtp_file->data, aw8622x_rtp->len);
		release_firmware(rtp_file);
		aw8622x->rtp_init = 1;
		aw8622x->rtp_cnt = 0;
		mutex_unlock(&aw8622x->rtp_lock);

	} else {
		if (!aw8622x->rtp_mmap_page_alloc_flag) {
			dev_info(aw8622x->dev, "%s mmap rtp container invalid\n", __func__);
			return;
		}
		aw8622x_haptic_set_rtp_aei(aw8622x, false);
		aw8622x->rtp_init = 1;
		aw8622x->rtp_cnt = 0;
	}

	gain = play->vmax * 128 / HAPTIC_BATTERY_VOLTAGE;
	aw8622x_haptic_set_gain(aw8622x, gain);

	aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_RTP_MODE);

	/* haptic start */
	mutex_lock(&aw8622x->rtp_check_lock);
	if (rtp_check_flag) {
		aw8622x_haptic_upload_lra(aw8622x, OSC_CALI);
		aw8622x_haptic_start(aw8622x);
		dev_info(aw8622x->dev, "%s ------------------>\n", __func__);
	} else {
		dev_info(aw8622x->dev, "%s rtp work has cancel\n", __func__);
		mutex_unlock(&aw8622x->rtp_check_lock);
		return;
	}
	mutex_unlock(&aw8622x->rtp_check_lock);
	aw8622x_haptic_rtp_init(aw8622x);

}


static void aw8622x_long_vibrate_work_routine(struct work_struct *work) //
{
	struct aw8622x *aw8622x = container_of(work, struct aw8622x,
				long_vibrate_work);

	dev_info(aw8622x->dev, "%s enter\n", __func__);

	mutex_lock(&aw8622x->lock);
	/* Enter standby mode */
	aw8622x_haptic_stop(aw8622x);
	aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
	if (aw8622x->state) {
		if (aw8622x->activate_mode ==
			AW8622X_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8622x_haptic_ram_vbat_compensate(aw8622x, true);
			aw8622x_haptic_play_repeat_seq(aw8622x, true);
		} else if (aw8622x->activate_mode ==
			AW8622X_HAPTIC_ACTIVATE_CONT_MODE) {
			dev_info(aw8622x->dev, "%s mode:%s\n", __func__,
				"AW8622X_HAPTIC_ACTIVATE_CONT_MODE");
			aw8622x_haptic_cont_config(aw8622x);
		} else {
			dev_err(aw8622x->dev, "%s: activate_mode error\n",
					__func__);
		}
		/* run ms timer */
		hrtimer_start(&aw8622x->timer,
				ktime_set(aw8622x->duration / 1000,
					(aw8622x->duration % 1000) * 1000000),
					HRTIMER_MODE_REL);
	}
	mutex_unlock(&aw8622x->lock);

	if (lra_wake_lock_active(aw8622x->wklock)) {
		lra_wake_unlock(aw8622x->wklock);
		dev_info(aw8622x->dev, "%s, release wake lock\n", __func__);
	}

}


static enum hrtimer_restart aw8622x_vibrator_timer_func(struct hrtimer *timer) //
{
	struct aw8622x *aw8622x = container_of(timer, struct aw8622x, timer);

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x->state = 0;
	schedule_work(&aw8622x->long_vibrate_work);

	return HRTIMER_NORESTART;
}

static irqreturn_t aw8622x_irq(int irq, void *data)
{
	struct aw8622x *aw8622x = data;
	unsigned char reg_val = 0;
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;

	aw8622x_i2c_read(aw8622x, AW8622X_REG_SYSINT, &reg_val);
	dev_info(aw8622x->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);

	if (reg_val & AW8622X_BIT_SYSINT_UVLI)
		dev_err(aw8622x->dev, "%s chip uvlo int error\n", __func__);
	if (reg_val & AW8622X_BIT_SYSINT_OCDI)
		dev_err(aw8622x->dev, "%s chip over current int error\n", __func__);
	if (reg_val & AW8622X_BIT_SYSINT_OTI)
		dev_err(aw8622x->dev, "%s chip over temperature int error\n", __func__);
	if (reg_val & AW8622X_BIT_SYSINT_DONEI)
		dev_info(aw8622x->dev, "%s chip playback done\n", __func__);

	if (reg_val & AW8622X_BIT_SYSINT_FF_AEI) {
		dev_info(aw8622x->dev, "%s: aw8622x rtp fifo almost empty\n", __func__);
		if (aw8622x->rtp_init) {
			while ((!aw8622x_haptic_rtp_get_fifo_afs(aw8622x)) && (aw8622x->play_mode == AW8622X_HAPTIC_RTP_MODE)) {
				mutex_lock(&aw8622x->rtp_lock);

				dev_info(aw8622x->dev, "%s: aw8622x rtp mode fifo update, cnt=%d\n", __func__, aw8622x->rtp_cnt);
				if (!aw8622x_rtp) {
					dev_info(aw8622x->dev, "%s:aw8622x_rtp is null, break!\n", __func__);
					mutex_unlock(&aw8622x->rtp_lock);
					break;
				}
				if ((aw8622x_rtp->len - aw8622x->rtp_cnt) < (aw8622x->ram.base_addr >> 2)) {
					buf_len = aw8622x_rtp->len - aw8622x->rtp_cnt;
				} else {
					buf_len = (aw8622x->ram.base_addr >> 2);
				}
				aw8622x->rtp_update_flag = aw8622x_i2c_writes(aw8622x, AW8622X_REG_RTPDATA,
													&aw8622x_rtp->data[aw8622x->rtp_cnt], buf_len);
				aw8622x->rtp_cnt += buf_len;

				aw8622x_i2c_read(aw8622x, AW8622X_REG_GLBRD5, &glb_state_val);
				if ((aw8622x->rtp_cnt == aw8622x_rtp->len) || ((glb_state_val & 0x0f) == 0)) {
					if (aw8622x->rtp_cnt == aw8622x_rtp->len)
						dev_info(aw8622x->dev, "%s: rtp load completely! glb_state_val=%02x aw8622x->rtp_cnt=%02x\n",
								__func__, glb_state_val, aw8622x->rtp_cnt);
					else
						dev_err(aw8622x->dev, "%s rtp load failed!! glb_state_val=%02x aw8622x->rtp_cnt=%02x\n",
								__func__, glb_state_val, aw8622x->rtp_cnt);

					aw8622x_haptic_set_rtp_aei(aw8622x, false);
					aw8622x->rtp_cnt = 0;
					aw8622x->rtp_init = 0;
					dev_info(aw8622x->dev, "%s  aw8622x->rtp_init = 0\n", __func__);
					mutex_unlock(&aw8622x->rtp_lock);
					break;
				}
				mutex_unlock(&aw8622x->rtp_lock);
			}
		} else {
			dev_info(aw8622x->dev, "%s: aw8622x rtp init = %d, init error\n",
								__func__, aw8622x->rtp_init);
		}
	}

	if (reg_val & AW8622X_BIT_SYSINT_FF_AFI)
		dev_err(aw8622x->dev, "%s: aw8622x rtp mode fifo almost full!\n", __func__);

	if (aw8622x->play_mode != AW8622X_HAPTIC_RTP_MODE)
		aw8622x_haptic_set_rtp_aei(aw8622x, false);

	dev_info(aw8622x->dev, "%s exit\n", __func__);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * haptic core driver
 *
 *****************************************************************************/

static int aw8622x_load_effect(struct aw8622x *aw8622x, struct haptic_effect *p_effect)
{
	struct aw8622x_play_info *play = &aw8622x->play;
	s16 level, custom_data[CUSTOM_DATA_LEN] = {0, 0, 0};
	int real_vmax, i, j = 0, scene_effect_id = 0, scene_vmax = 0;
	struct haptic_wavefrom_info *effect_list = aw8622x->effect_list;
	int gain;
	int secne_idx = 0;

	memset(play, 0, sizeof(struct aw8622x_play_info));

	dev_info(aw8622x->dev, "%s: p_effect pointer (%#x)\n", __func__, p_effect); //ignore

	switch (p_effect->type) {

	case HAPTIC_CONSTANT:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8622x->dev, "%s  constant copy from user failed\n", __func__);
			return -EFAULT;
		}

		play->playLength = p_effect->length * USEC_PER_MSEC;
		level = p_effect->magnitude;
		play->vmax = level * aw8622x->default_vmax / 0x7fff;
		play->type = TIME_TYPE;
		//it is used in playback
		aw8622x->gain = level * 0x80 / 0x7fff;

		dev_info(aw8622x->dev, "%s: constant, length_us = %d, vmax_mv = %d, level = %d \n", __func__, play->playLength, play->vmax, level);

		if (!lra_wake_lock_active(aw8622x->wklock)) {
			lra_wake_lock(aw8622x->wklock);
			dev_info(aw8622x->dev, "%s, constant add wake lock\n", __func__);
		}

		/* Enter standby mode */
		aw8622x_haptic_stop(aw8622x);
		aw8622x_haptic_set_repeat_wav_seq(aw8622x, PLAYBACK_INFINITELY_RAM_ID);
		aw8622x_haptic_set_gain(aw8622x, aw8622x->gain);

		break;

	case HAPTIC_RTP_STREAM: //mmap
		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8622x->dev, "%s copy from user failed\n", __func__);
			return -EFAULT;
		}

		if (!aw8622x->rtp_mmap_page_alloc_flag) {
			dev_err(aw8622x->dev, "%s mmap rtp container invalid\n", __func__);
			return -ENOMEM;
		}

		real_vmax = HAPTIC_BATTERY_VOLTAGE;

		play->type = RTP_MMAP_TYPE;
		level = p_effect->magnitude;
		play->vmax = level * real_vmax / 0x7fff;
		play->times_ms = custom_data[0];
		play->times_ms = p_effect->data_count / 12; // 24 means sample rate
		gain = level * 128 / 0x7fff;

		if (aw8622x_rtp->len != p_effect->data_count) {
			dev_err(aw8622x->dev, "data count not eq, rtp len: %d, data count: %d\n", aw8622x_rtp->len, p_effect->data_count);
			return -EFAULT;
		}

		aw8622x_haptic_stop(aw8622x);
		aw8622x_haptic_set_rtp_aei(aw8622x, false);
		aw8622x_haptic_set_wav_loop(aw8622x, 0x00, 0x00);
		aw8622x_haptic_set_gain(aw8622x, gain);
		aw8622x_interrupt_clear(aw8622x);
		break;

	case HAPTIC_CUSTOM:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8622x->dev, "%s copy from user failed\n", __func__);
			return -EFAULT;
		}

		dev_err(aw8622x->dev, "scene id %d\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
		secne_idx = custom_data[CUSTOM_DATA_EFFECT_IDX];
		if (custom_data[CUSTOM_DATA_EFFECT_IDX] < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw8622x->base_scene_count; j++) {
				if (aw8622x->base_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8622x->base_scene_count) {
				dev_err(aw8622x->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8622x->base_scene_list[j].real_vmax; //real_vmax为场景设计的实际电压
			scene_effect_id = aw8622x->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw8622x->ext_scene_count; j++) {
				if (aw8622x->ext_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8622x->ext_scene_count) {
				dev_err(aw8622x->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8622x->ext_scene_list[j].real_vmax;
			scene_effect_id = aw8622x->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw8622x->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw8622x->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw8622x->effects_count) {
			dev_err(aw8622x->dev, "scene: %d effect: %d not supported!\n",
			custom_data[CUSTOM_DATA_EFFECT_IDX], scene_effect_id);
			return -EINVAL;
		}

		//更新real vmax值，dts配置和hidl配置，取较小值
		if (scene_vmax > 0 && scene_vmax < effect_list[i].vmax)
			real_vmax = scene_vmax;
		else
			real_vmax = effect_list[i].vmax;

		if (real_vmax > HAPTIC_BATTERY_VOLTAGE)
			real_vmax = HAPTIC_BATTERY_VOLTAGE;

		dev_err(aw8622x->dev, "real_vamx = %d, scene_vamx = %d, effect_vmax = %d\n", real_vmax, scene_vmax, effect_list[i].vmax);

		if (!effect_list[i].rtp_enable) {

			play->type = RAM_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			play->ram_id = effect_list[i].ram_id;
			gain = play->vmax * 128 / HAPTIC_BATTERY_VOLTAGE;

			dev_err(aw8622x->dev, "ram, effect_id = %d, ram_id = %d, vmax_mv = %d, length = %d, level = %d\n",
								scene_effect_id, play->ram_id, play->vmax, play->times_ms, level);
			aw8622x_haptic_stop(aw8622x);
			aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
			aw8622x_haptic_set_wav_loop(aw8622x, 0x00, 0x00);
			aw8622x_haptic_set_wav_seq(aw8622x, 0x00, play->ram_id);
			if (aw8622x->play.ram_id == 0) {
				aw8622x_double_click_switch(aw8622x, true);
			}
			aw8622x_haptic_set_gain(aw8622x, gain);
//			aw8622x_haptic_ram_vbat_compensate(aw8622x, false);

		} else {

			play->type = RTP_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			strlcpy(play->rtp_file, effect_list[i].rtp_file_name, 128);
			aw8622x->rtp_file_num = i;

			dev_err(aw8622x->dev, "%s: rtp, effect_id = %d, rtp_name: %s, vamx_mv = %d, length = %d, level = %d\n",
										__func__, scene_effect_id, play->rtp_file, play->vmax, play->times_ms, level);

			if ((secne_idx < 500) && (secne_idx > 400)) {
				if (!lra_wake_lock_active(aw8622x->wklock)) {
					lra_wake_lock(aw8622x->wklock);
					dev_info(aw8622x->dev, "%s, notification add wake lock\n", __func__);
				}
			}

			aw8622x_haptic_stop(aw8622x);
			aw8622x_haptic_set_rtp_aei(aw8622x, false);
			aw8622x_haptic_set_wav_loop(aw8622x, 0x00, 0x00);
			aw8622x_interrupt_clear(aw8622x);
		}


		custom_data[CUSTOM_DATA_TIMEOUT_SEC_IDX] = play->times_ms / MSEC_PER_SEC;
		custom_data[CUSTOM_DATA_TIMEOUT_MSEC_IDX] = play->times_ms % MSEC_PER_SEC;

		if (copy_to_user(p_effect->custom_data, custom_data, sizeof(s16) * CUSTOM_DATA_LEN))	{
			dev_err(aw8622x->dev, "%s copy to user failed\n", __func__);
			return -EFAULT;
		}

		break;

	default:
		dev_err(aw8622x->dev, "%s Unsupported effect type: %d\n", __func__, p_effect->type);
		return -ENODEV;
		break;
	}

	return 0;
}


static int aw8622x_playback_vib(struct aw8622x *aw8622x, int val)
{
	struct aw8622x_play_info *play = &aw8622x->play;
	int len;


	if (val) {

		/* Enter standby mode */
		aw8622x_haptic_stop(aw8622x);

		switch (play->type) {
		case RAM_TYPE:
			dev_info(aw8622x->dev, "%s: ---> start ram mode\n", __func__);
			aw8622x_haptic_play_wav_seq(aw8622x, true);
			break;

		case RTP_TYPE:
			dev_info(aw8622x->dev, "%s: ---> start rtp mode\n", __func__);
			queue_work(rtp_wq, &aw8622x->rtp_work);
			rtp_check_flag = true;
			break;
		case RTP_MMAP_TYPE:
			dev_info(aw8622x->dev, "%s: ---> start RTP_MMAP_TYPE mode\n", __func__);
			queue_work(rtp_wq, &aw8622x->rtp_work);
			rtp_check_flag = true;
			break;
		case TIME_TYPE:
			aw8622x->activate_mode = AW8622X_HAPTIC_ACTIVATE_RAM_MODE;
			len = play->playLength + 10000;
			dev_info(aw8622x->dev, "%s: ---> start time mode, length = %d\n", __func__, len);

			if (aw8622x->activate_mode == AW8622X_HAPTIC_ACTIVATE_RAM_MODE) {
				aw8622x_haptic_ram_vbat_compensate(aw8622x, false);
				aw8622x_haptic_play_repeat_seq(aw8622x, true);
			} else if (aw8622x->activate_mode == AW8622X_HAPTIC_ACTIVATE_CONT_MODE) {
				aw8622x_haptic_cont_config(aw8622x);
			} else {
				dev_info(aw8622x->dev, "%s: not suppoert activate mode\n", __func__);
			}
			/* run us timer */
			hrtimer_start(&aw8622x->timer,
				ktime_set(len / USEC_PER_SEC, (len % USEC_PER_SEC) * NSEC_PER_USEC),
				HRTIMER_MODE_REL);
			break;
		default:
			break;

		}

	} else {
		if (hrtimer_active(&aw8622x->timer)) {
			hrtimer_cancel(&aw8622x->timer);
			dev_info(aw8622x->dev, "%s playback cancel timer\n", __func__);
		}

		/* 取消rtp work */
		if (cancel_work_sync(&aw8622x->rtp_work)) {
			dev_info(aw8622x->dev, "%s palyback pending work cancle success\n", __func__);
		}
		//aw8622x->rtp_len = 0;
		mutex_lock(&aw8622x->rtp_check_lock);
		/* Enter standby mode */
		aw8622x_haptic_stop(aw8622x);
		if (aw8622x->play.ram_id == 0) {
			aw8622x_double_click_switch(aw8622x, false);
		}
		rtp_check_flag = false;
		mutex_unlock(&aw8622x->rtp_check_lock);
		aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
	}

	return 0;
}

static int aw8622x_upload_sync(struct haptic_handle *hp, struct haptic_effect *effect)
{
	int ret = 0;
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;

	ret = aw8622x_load_effect(aw8622x, effect);
	return ret;
}

static int aw8622x_erase_sync(struct haptic_handle *hp)
{
	return 0;
}

static int aw8622x_playback_sync(struct haptic_handle *hp, int value)
{
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;
	int ret = 0;

	ret = aw8622x_playback_vib(aw8622x, !!value);

	if (!value) {
		if (lra_wake_lock_active(aw8622x->wklock)) {
			lra_wake_unlock(aw8622x->wklock);
			dev_info(aw8622x->dev, "%s, release wake lock\n", __func__);
		}
	}


	return ret;
}

static void aw8622x_set_gain_sync(struct haptic_handle *hp, u16 gain)
{

	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;
	struct aw8622x_play_info *play = &aw8622x->play;

	dev_info(aw8622x->dev, "%s gain=%d\n", __func__, gain);

	if (play->type == TIME_TYPE) {
		if (gain > 0x7fff)
			gain = 0x7fff;

		aw8622x->gain = ((u32)(gain * 0x80)) / 0x7fff;
		aw8622x_haptic_ram_vbat_compensate(aw8622x, false);
	}

}
//get lra type
static void aw8622x_get_motor_type_sync(struct haptic_handle *hp, int *motorType)
{
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;
	//从dtsi中获取lra_info字段即可
	*motorType = (int)aw8622x->lra_information;
}

static void aw8622x_get_f0_sync(struct haptic_handle *hp, int *f0)
{
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;


	*f0 = (int)((aw8622x->cali_f0 + 5) / 10);

}

static void aw8622x_get_driver_ic_sync(struct haptic_handle *hp, enum ic_type *driver_ic)
{
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;


	*driver_ic = AW86224;

	dev_info(aw8622x->dev, "%s, ic type (%d)\n", __func__, *driver_ic);

}

static int aw8622x_judge_effect_support_sync(struct haptic_handle *hp, int16_t effectNo)
{
	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;
	int i, j, scene_effect_id = 0;

	if (effectNo < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw8622x->base_scene_count; j++) {
				if (aw8622x->base_scene_list[j].scene_id == effectNo)
					break;
			}

			if (j == aw8622x->base_scene_count) {
				dev_err(aw8622x->dev, "judge: scene:%d not support\n", effectNo);
				return -EINVAL;
			}

			scene_effect_id = aw8622x->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw8622x->ext_scene_count; j++) {
				if (aw8622x->ext_scene_list[j].scene_id == effectNo)
					break;
			}

			if (j == aw8622x->ext_scene_count) {
				dev_err(aw8622x->dev, "judge: scene:%d not support\n", effectNo);
				return -EINVAL;
			}

			scene_effect_id = aw8622x->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw8622x->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw8622x->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw8622x->effects_count) {
			dev_err(aw8622x->dev, "judge: scene: %d effect: %d not supported!\n",
			effectNo, scene_effect_id);
			return -EINVAL;
		}

		return 0;
}


static void aw8622x_set_cali_params_sync(struct haptic_handle *hp, struct haptic_cali_param *cali_params)
{

	struct aw8622x *aw8622x = (struct aw8622x *)hp->chip;


	if (cali_params->calibration_data_len == AWINIC_LEN) {
		aw8622x->f0_cali_data = cali_params->u.awinic.f0_offset;
		aw8622x->cali_f0 = cali_params->u.awinic.f0;
		dev_info(aw8622x->dev, "%s f0_offset = %d, cali_f0 = %d\n", __func__,
			cali_params->u.awinic.f0_offset, cali_params->u.awinic.f0);
	} else {
		dev_info(aw8622x->dev, "%s ic type not support\n", __func__);
	}

}


/*********************************************************************************************
 *
 * sysfs
 *
 *********************************************************************************************/
static ssize_t aw8622x_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);
	struct aw8622x_play_info *play = &aw8622x->play;
	unsigned int val = 0;
	int rc = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8622x->duration = val;
	play->playLength = val * USEC_PER_MSEC;
	play->ram_id = PLAYBACK_INFINITELY_RAM_ID;
	play->type = TIME_TYPE;

	return count;
}

static ssize_t aw8622x_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);
	struct aw8622x_play_info *play = &aw8622x->play;
	unsigned int val = 0;
	int rc = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	dev_info(aw8622x->dev, "%s: value=%d\n", __FUNCTION__, val);
	dev_info(aw8622x->dev, "%s: constant, length_us = %d, vmax_mv = %d\n", __func__, play->playLength, play->vmax);
	aw8622x_haptic_stop(aw8622x);
	aw8622x_haptic_set_repeat_wav_seq(aw8622x, PLAYBACK_INFINITELY_RAM_ID);
	play->vmax = aw8622x->default_vmax;
	aw8622x->gain = 0x80;
	aw8622x_playback_vib(aw8622x, val);
	return count;
}


static ssize_t aw8622x_iic_int_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);
	unsigned char reg = 0;
	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x_i2c_read(aw8622x, AW_REG_ID, &reg);


	if (reg != AW8622X_CHIP_ID) {
		dev_err(aw8622x->dev, "%s:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "+IIC:\"0\"\n+INT:\"1\"\n+RST:\"1\"\n");
	}
	return snprintf(buf, PAGE_SIZE, "+IIC:\"1\"\n+INT:\"1\"\n+RST:\"1\"\n");
}

static ssize_t aw8622x_at_trigger_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t len = 0;
	//unsigned char reg = 0;

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8622x no trig function");
	return len;
}

static ssize_t aw8622x_at_trigger_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	//int val = 0, ret = 0;
	//unsigned char reg = 0;

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	dev_info(aw8622x->dev, "aw8622x, at_trigger_state_store enter, %s\n", buf);

	return count;
}


/*use current f0,1040 lra no need to cali*/
static ssize_t aw8622x_f0_offset_10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);


	unsigned int databuf[1] = {0};
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	if (aw8622x->lra_information == AW8622x_LRA_1040) {
		dev_info(aw8622x->dev, "%s:enter;aw8622x->lra_information is 1040, no need cali\n", __func__);
	} else {

		dev_info(aw8622x->dev, "%s:enter;buf is %s\n", __func__, buf);
		if (1 == sscanf(buf, "%d", &databuf[0])) {
			at_test = true;

			aw8622x->f0_cali_flag = AW8622x_HAPTIC_CALI_F0;
			aw8622x_i2c_write(aw8622x, AW8622X_REG_TRIMCFG3, 0x00);
			dev_info(aw8622x->dev, "%s user current f0\n", __func__);

			switch (databuf[0]) {
			case 0:
				f0_limit = aw8622x->f0;
				break;
			case 10:
				f0_limit = aw8622x->f0 + 100;
				break;
			case -10:
				f0_limit = aw8622x->f0 - 100;
				break;
			default:
				f0_limit = aw8622x->f0;
				break;
			}
			dev_info(aw8622x->dev, "%s f0_pre = %d\n", __func__, aw8622x->dts_info.f0_ref);

		/* calculate cali step */
			f0_cali_step = 100000*((int)f0_limit-(int)aw8622x->dts_info.f0_ref) / ((int)f0_limit*25);
			dev_info(aw8622x->dev, "%s f0_cali_step=%d\n", __func__, f0_cali_step);
			if (f0_cali_step >= 0) {   /*f0_cali_step >= 0*/
				if (f0_cali_step % 10 >= 5)
					f0_cali_step = f0_cali_step/10 + 1 + 32;
				else
					f0_cali_step = f0_cali_step/10  + 32;
			} else {  /*f0_cali_step < 0*/
				if (f0_cali_step % 10 <= -5) {
					f0_cali_step = 32 + (f0_cali_step/10 - 1);
				} else {
					f0_cali_step = 32 + f0_cali_step/10;
				}
			}

			if (f0_cali_step > 31) {
				f0_cali_lra = (char)f0_cali_step - 32;
			} else {
				f0_cali_lra = (char)f0_cali_step + 32;
			}
			dev_info(aw8622x->dev, "%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);
			/* update cali step */
			aw8622x->f0_cali_data = f0_cali_lra;
			aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
			aw8622x_i2c_read(aw8622x, AW8622X_REG_TRIMCFG3, &reg_val);
			dev_info(aw8622x->dev, "%s final trim_lra=0x%02x\n", __func__, reg_val);
			/* restore default work mode */
			aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_STANDBY_MODE);
			aw8622x->play_mode = AW8622X_HAPTIC_RAM_MODE;
			aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_PLAYCFG3,
					AW8622X_BIT_PLAYCFG3_PLAY_MODE_MASK, AW8622X_BIT_PLAYCFG3_PLAY_MODE_RAM);
			aw8622x_haptic_stop(aw8622x);

			mdelay(100);
			dev_info(aw8622x->dev, "%s set freq to %dHZ\n", __func__, f0_limit);

			at_test = false;

		}
	}
	return count;
}

static ssize_t aw8622x_is_need_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	ssize_t len = 0;
	int need_cali = 1;
	char lra[] = "X-LRA 0619";
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	switch (aw8622x->lra_information) {

	case AW8622x_LRA_0815:
		strcpy(lra, "X-LRA 0815");
		need_cali = 1;
		break;
	case AW8622x_LRA_1040:
		strcpy(lra, "Z-LRA 1040");
		need_cali = 0;
		break;
	case AW8622x_LRA_0832:
		strcpy(lra, "Z-LRA 0832");
		need_cali = 1;
		break;
	default:
		strcpy(lra, "X-LRA 0619");
		need_cali = 1;
		break;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "+Type:\"%s\"\n", lra);
	len += snprintf(buf+len, PAGE_SIZE-len, "+Require:\"%d\"\n", need_cali);

	return len;
}


//先校准再读f0值，再读lra_resistance值
static ssize_t aw8622x_cali_f0_resis_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	//unsigned char reg_val = 0;
	ssize_t len = 0;

	at_test = true;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	//get resistance
	aw8622x_haptic_get_lra_resistance(aw8622x);

	if ((aw8622x->lra >= aw8622x->resistance_min) && (aw8622x->lra <= aw8622x->resistance_max)) {
		dev_info(aw8622x->dev, "%s lra resistent test ok, lra=%d\n", __func__, aw8622x->lra);
	} else {
		dev_info(aw8622x->dev, "%s lra resistent over range, lra=%d\n", __func__, aw8622x->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8622x->lra >= aw8622x->resistance_min && aw8622x->lra <= aw8622x->resistance_max) ? "ok" : "fail",
		aw8622x->lra/1000, aw8622x->lra%1000,
		aw8622x->resistance_min/1000, aw8622x->resistance_min%1000/100,
		aw8622x->resistance_max/1000, aw8622x->resistance_max%1000/100);


	if (aw8622x->lra_information == AW8622x_LRA_1040) {

		dev_info(aw8622x->dev, "%s:enter;aw8622x->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#ok#170.0#%d.%d-%d.%d#hz\n",
			aw8622x->freq_min/10, aw8622x->freq_min%10, aw8622x->freq_max/10, aw8622x->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#0\n",
			(aw8622x->f0 >= aw8622x->freq_min && aw8622x->f0 <= aw8622x->freq_max) ? "ok" : "fail",
			aw8622x->f0_cali_data);
		dev_info(aw8622x->dev, "%s  aw8622x->f0_cali_lra=%d\n", __func__, aw8622x->f0_cali_data);
	} else {

		mdelay(20);

		aw8622x_haptic_f0_calibration(aw8622x);

		mdelay(200);

		aw8622x->f0_cali_flag = AW8622x_HAPTIC_CALI_F0;
		aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
		aw8622x_haptic_cont_get_f0(aw8622x);

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
			(aw8622x->f0 >= aw8622x->freq_min && aw8622x->f0 <= aw8622x->freq_max) ? "ok" : "fail",
			aw8622x->f0/10, aw8622x->f0%10,
			aw8622x->freq_min/10, aw8622x->freq_min%10, aw8622x->freq_max/10, aw8622x->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#%d,%d\n", aw8622x->f0_cali_data, aw8622x->f0);
		aw8622x->cali_f0 = aw8622x->f0;
	}

	at_test = false;

	return len;
}



static ssize_t aw8622x_resis_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	ssize_t len = 0;


	at_test = true;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	aw8622x_haptic_get_lra_resistance(aw8622x);

	if ((aw8622x->lra >= aw8622x->resistance_min) && (aw8622x->lra <= aw8622x->resistance_max)) {
		dev_info(aw8622x->dev, "%s lra resistent test ok, lra=%d\n", __func__, aw8622x->lra);
	} else {
		dev_info(aw8622x->dev, "%s lra resistent over range, lra=%d\n", __func__, aw8622x->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8622x->lra >= aw8622x->resistance_min && aw8622x->lra <= aw8622x->resistance_max) ? "ok" : "fail",
		aw8622x->lra/1000, aw8622x->lra%1000,
		aw8622x->resistance_min/1000, aw8622x->resistance_min%1000/100,
		aw8622x->resistance_max/1000, aw8622x->resistance_max%1000/100);

	mdelay(20);

	aw8622x->f0_cali_flag = AW8622x_HAPTIC_CALI_F0;
	aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
	aw8622x_haptic_cont_get_f0(aw8622x);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw8622x->f0 >= aw8622x->freq_min && aw8622x->f0 <= aw8622x->freq_max) ? "ok" : "fail",
		aw8622x->f0/10, aw8622x->f0%10,
		aw8622x->freq_min/10, aw8622x->freq_min%10, aw8622x->freq_max/10, aw8622x->freq_max%10);

	at_test = false;



	return len;
}


/* 售后工具线性马达校准项
 * AT+BK_VBR_CAL=1  
 *【指令】：cat /sys/class/leds/vibrator/cali_f0
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 故offset值返回0，且不进行校准，只读f0
 */
static ssize_t aw8622x_cali_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	ssize_t len = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);


	at_test = true;
	dev_info(aw8622x->dev, "%s:aw8622x_cali_f0_show  enter\n", __func__);

	if (aw8622x->lra_information == AW8622x_LRA_1040) {

		dev_info(aw8622x->dev, "%s:enter;aw8622x->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "ok f0 170.0 (range:%d.%d-%d.%d)hz f0_offset=0\n",
			aw8622x->freq_min/10, aw8622x->freq_min%10, aw8622x->freq_max/10, aw8622x->freq_max%10);

	} else {

		dev_info(aw8622x->dev, "%s:enter;aw8622x->lra_information is not 1040\n", __func__);

		aw8622x_haptic_f0_calibration(aw8622x);

		mdelay(200);

		aw8622x->f0_cali_flag = AW8622x_HAPTIC_CALI_F0;
		aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
		aw8622x_haptic_cont_get_f0(aw8622x);

		len += snprintf(buf+len, PAGE_SIZE-len, "%s f0 %d.%d (range:%d.%d-%d.%d)hz f0_offset=%d,%d\n",
			(aw8622x->f0 >= aw8622x->freq_min && aw8622x->f0 <= aw8622x->freq_max) ? "ok" : "fail",
			aw8622x->f0/10, aw8622x->f0%10,
			aw8622x->freq_min/10, aw8622x->freq_min%10, aw8622x->freq_max/10, aw8622x->freq_max%10,
			aw8622x->f0_cali_data, aw8622x->f0);

		aw8622x->cali_f0 = aw8622x->f0;

	}

	at_test = false;


	return len;
}


static ssize_t aw8622x_i2c_reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);
	for (i = 0; i < AW8622X_REG_MAX; i++) {
		if (!(aw8622x_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw8622x_i2c_read(aw8622x, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}


static ssize_t aw8622x_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);


	unsigned int databuf[2] = {0, 0};
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8622x_i2c_write(aw8622x, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}


static ssize_t aw8622x_i2c_ram_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	aw8622x_haptic_stop(aw8622x);
	/* RAMINIT Enable */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
			AW8622X_BIT_SYSCTRL1_RAMINIT_MASK, AW8622X_BIT_SYSCTRL1_RAMINIT_ON);

	aw8622x_i2c_write(aw8622x, AW8622X_REG_RAMADDRH, (unsigned char)(aw8622x->ram.base_addr>>8));
	aw8622x_i2c_write(aw8622x, AW8622X_REG_RAMADDRL, (unsigned char)(aw8622x->ram.base_addr&0x00ff));
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8622x_haptic_ram:\n");
	for (i = 0; i < aw8622x->ram.len; i++) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_RAMDATA, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	/* RAMINIT Disable */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_SYSCTRL1,
			AW8622X_BIT_SYSCTRL1_RAMINIT_MASK, AW8622X_BIT_SYSCTRL1_RAMINIT_OFF);

	return len;
}

static ssize_t aw8622x_i2c_ram_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);


	unsigned int databuf[1] = {0};
	dev_info(aw8622x->dev, "%s  enter\n.", __func__);

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw8622x_ram_update(aw8622x);
		}
	}
	return count;
}


static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, NULL, aw8622x_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, NULL, aw8622x_activate_store);
static DEVICE_ATTR(iic_int_rst, S_IWUSR | S_IRUGO, aw8622x_iic_int_rst_show, NULL);
static DEVICE_ATTR(at_trigger_state, S_IWUSR | S_IRUGO, aw8622x_at_trigger_state_show, aw8622x_at_trigger_state_store);
static DEVICE_ATTR(f0_offset_10, S_IWUSR | S_IRUGO, NULL, aw8622x_f0_offset_10_store);
static DEVICE_ATTR(is_need_cali, S_IWUSR | S_IRUGO, aw8622x_is_need_cali_show, NULL);
static DEVICE_ATTR(cali_f0_resis, S_IWUSR | S_IRUGO, aw8622x_cali_f0_resis_show, NULL);
static DEVICE_ATTR(resis_f0, S_IWUSR | S_IRUGO, aw8622x_resis_f0_show, NULL);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO, aw8622x_cali_f0_show, NULL);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8622x_i2c_reg_show, aw8622x_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8622x_i2c_ram_show, aw8622x_i2c_ram_store);


static struct attribute *aw8622x_attributes[] = {
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_iic_int_rst.attr,
	&dev_attr_at_trigger_state.attr,
	&dev_attr_f0_offset_10.attr,
	&dev_attr_is_need_cali.attr,
	&dev_attr_cali_f0_resis.attr,
	&dev_attr_resis_f0.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};


static struct attribute_group aw8622x_attribute_group = {
	.attrs = aw8622x_attributes
};

static enum led_brightness aw8622x_haptic_brightness_get(struct led_classdev //
							 *cdev)
{
	struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	return aw8622x->amplitude;
}

static void aw8622x_haptic_brightness_set(struct led_classdev *cdev, //
					  enum led_brightness level)
{
	// struct aw8622x *aw8622x = container_of(cdev, struct aw8622x, cdev);

	// dev_info(aw8622x->dev, "%s enter\n", __func__);
	// if (!aw8622x->ram_init) {
		// dev_err(aw8622x->dev, "%s: ram init failed, not allow to play!\n",
			// __func__);
		// return;
	// }
	// if (aw8622x->ram_update_flag < 0)
		// return;
	// aw8622x->amplitude = level;
	// aw8622x_haptic_stop(aw8622x);
	// if (aw8622x->amplitude > 0) {
		// aw8622x_haptic_upload_lra(aw8622x, F0_CALI);
		// aw8622x_haptic_ram_vbat_compensate(aw8622x, false);
		// aw8622x_haptic_play_wav_seq(aw8622x, aw8622x->amplitude);
	// }

}

/********************************************************
 *
 * scene and effect dts parse
 *
 ********************************************************/
static haptic_parse_per_effect_dt(struct aw8622x *chip, struct device_node *child,
	struct haptic_wavefrom_info *effect_node)
{
	int rc;

	rc = of_property_read_u32(child, "awinic,effect-id", &effect_node->idx);
	if (rc < 0) {
		dev_info(chip->dev, "Read awinic effect-id failed, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_read_u32(child, "awinic,wf-vmax-mv", &effect_node->vmax);
	if (rc < 0) {
		dev_info(chip->dev, "Read awinic wf-vmax-mv failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-length", &effect_node->times_ms);
	if (rc < 0) {
		dev_info(chip->dev, "Read awinic wf-length failed, rc=%d\n", rc);
		return rc;
	}

	effect_node->rtp_enable = of_property_read_bool(child, "awinic,rtp-enable");

	if (effect_node->rtp_enable) {

		rc = of_property_read_string(child, "awinic,rtp-file", &effect_node->rtp_file_name);
		if (rc < 0) {
			dev_info(chip->dev, "Read awinic rtp-file failed, rc=%d\n", rc);
			return rc;
		}

		// rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		// if (rc < 0) {
			// dev_info(chip->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			// return rc;
		// }

		effect_node->ram_id = 1;

	} else {

		rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		if (rc < 0) {
			dev_info(chip->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			return rc;
		}
		effect_node->rtp_file_name = "default";
	}

	return 0;
}


static int haptic_effects_parse_dt(struct aw8622x *chip, struct device_node *effect_node)
{
	struct device_node *child = NULL;
	int i = 0, ret;


	for_each_available_child_of_node(effect_node, child) {
		if (!of_find_property(child, "awinic,effect-id", NULL))
			continue;

		ret = haptic_parse_per_effect_dt(chip, child, &chip->effect_list[i]);
		if (ret < 0) {
			dev_err(chip->dev, "parse effect %d failed, rc=%d\n", i, ret);
			of_node_put(child);
			return ret;
			}
		i++;
	}

	chip->effects_count = i;

	return 0;
}

static int haptic_scenes_parse_dt(struct aw8622x *chip, struct device_node *scene_node,
						int base_scene_element_count, int ext_scene_element_count)
{
	int ret = 0, i;

	#define SCENE_NU    3000
	u16 *temp_effect = NULL;

	temp_effect = devm_kzalloc(chip->dev, base_scene_element_count * sizeof(u16), GFP_KERNEL);// 1024/3=682
	if (!temp_effect) {
		dev_err(chip->dev, "Kzalloc for temp_effect failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(scene_node, "base_scene", temp_effect, base_scene_element_count);
	if (ret) {
		dev_err(chip->dev, "Get base effect failed, ret=%d\n", ret);
		return -ENODEV;
	} else {
		for (i = 0; i < base_scene_element_count / 3; i++) {
			chip->base_scene_list[i].scene_id = temp_effect[3 * i];
			chip->base_scene_list[i].effect_id = temp_effect[3 * i + 1];
			chip->base_scene_list[i].real_vmax = temp_effect[3 * i + 2];
		}
		chip->base_scene_count = base_scene_element_count / 3;
	}

	devm_kfree(chip->dev, temp_effect);
	temp_effect = NULL;


	temp_effect = devm_kzalloc(chip->dev, ext_scene_element_count * sizeof(u16), GFP_KERNEL);// 1024/3=341
	if (!temp_effect) {
		dev_err(chip->dev, "Kzalloc for temp_effect failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(scene_node, "ext_scene", temp_effect, ext_scene_element_count);
	if (ret) {
		dev_err(chip->dev, "Get ext effect failed, ret=%d\n", ret);
		return -ENODEV;
	} else {
		for (i = 0; i < ext_scene_element_count / 3; i++) {
			chip->ext_scene_list[i].scene_id = temp_effect[3 * i];
			chip->ext_scene_list[i].effect_id = temp_effect[3 * i + 1];
			chip->ext_scene_list[i].real_vmax = temp_effect[3 * i + 2];
		}
		chip->ext_scene_count = ext_scene_element_count / 3;
	}

	devm_kfree(chip->dev, temp_effect);


	return 0;
}


static int aw8622x_vibrator_init(struct aw8622x *aw8622x)
{
	int ret = 0;
	struct haptic_misc *hm = NULL;
	struct haptic_handle *handle = NULL;
	int order = 0;

	struct device_node *effect_node = NULL;
	struct device_node *child = NULL;
	int effects_count = 0;

	struct device_node *scene_node = NULL;
	int base_scene_element_count = 0;
	int ext_scene_element_count = 0;

	struct device_node *node = aw8622x->dev->of_node;


	hrtimer_init(&aw8622x->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8622x->timer.function = aw8622x_vibrator_timer_func;
	INIT_WORK(&aw8622x->long_vibrate_work, aw8622x_long_vibrate_work_routine);
	INIT_WORK(&aw8622x->rtp_work, aw8622x_rtp_work_routine);

	/* rtp模式要求数据写入及时，不能断流，故建立单独的工作队列，专门处理rtp数据传输，而不使用系统默认的工作队列 */
	rtp_wq = create_singlethread_workqueue("rtp_wq");
	if (!rtp_wq) {
		dev_err(aw8622x->dev, "Create rtp work queue failed\n");
		return -ENOENT;
	}

	mutex_init(&aw8622x->lock);
	mutex_init(&aw8622x->rtp_lock);
	mutex_init(&aw8622x->rtp_check_lock);

	aw8622x->wklock = lra_wake_lock_register(aw8622x->dev, "vivo-aw8622x-wakelock");


	aw8622x->cali_f0 = 2350;

	/* 由于每次写512字节，并且频繁申请和释放，所以为rtp模式开辟单独的高速缓存，提高内存分配速度 */

	rtp_cachep = kmem_cache_create("aw8622x-hap", RTP_SLAB_SIZE + 1, 0, SLAB_HWCACHE_ALIGN, NULL);
	if (rtp_cachep == NULL) {
		dev_err(aw8622x->dev, "%s alloc high cache failed\n", __func__);
	} else {
		dev_info(aw8622x->dev, "%s alloc high cache success\n", __func__);
	}

	// scene and effect dts parse begin
	effect_node = of_parse_phandle(node, "effect_list_aw862xx", 0);
	if (!effect_node) {
		dev_err(aw8622x->dev, "Get effect_list phandle failed\n");
		ret = -ENODEV;
		goto dts_err;
	}

	for_each_available_child_of_node(effect_node, child) {
		if (of_find_property(child, "awinic,effect-id", NULL))
			effects_count++;
	}

	aw8622x->effect_list = devm_kcalloc(aw8622x->dev, effects_count,
				sizeof(struct haptic_wavefrom_info), GFP_KERNEL);
	if (!aw8622x->effect_list) {
		dev_err(aw8622x->dev, "%s mem alloc failed\n", __func__);
		ret = -ENOMEM;
		goto dts_err;
	}

	ret = haptic_effects_parse_dt(aw8622x, effect_node);
	if (ret) {
		dev_err(aw8622x->dev, "%s parse effect failed\n", __func__);
		ret = -EFAULT;
		goto dts_err;
	}

	scene_node = of_parse_phandle(node, "scene_array_aw862xx", 0);
	if (!scene_node) {
		dev_err(aw8622x->dev, "Get effect array phandle failed\n");
		ret =  -ENODEV;
		goto dts_err;
	}

	base_scene_element_count = of_property_count_u16_elems(scene_node, "base_scene");
	if (base_scene_element_count < 0) {
		dev_err(aw8622x->dev, "Get base effect elements count failed\n");
		ret = -EINVAL;
		goto dts_err;
	}

	aw8622x->base_scene_list = devm_kcalloc(aw8622x->dev, base_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!aw8622x->base_scene_list) {
		dev_err(aw8622x->dev, "Kcalloc for base effect failed\n");
		ret = -ENOMEM;
		goto dts_err;

	}


	ext_scene_element_count = of_property_count_u16_elems(scene_node, "ext_scene");
	if (ext_scene_element_count < 0) {
		dev_err(aw8622x->dev, "Get ext effect elements count failed\n");
		ret = -EINVAL;
		goto dts_err;
	}

	aw8622x->ext_scene_list = devm_kcalloc(aw8622x->dev, ext_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!aw8622x->ext_scene_list) {
		dev_err(aw8622x->dev, "Kcalloc for ext effect failed\n");
		ret = -ENOMEM;
		goto dts_err;
	}

	ret = haptic_scenes_parse_dt(aw8622x, scene_node, base_scene_element_count, ext_scene_element_count);
	if (ret) {
		dev_err(aw8622x->dev, "%s parse scenes failed\n", __func__);
		ret = -EFAULT;
		goto dts_err;
	}

	if (g_logDts) {
		int j;
		dev_err(aw8622x->dev, "Effects Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8622x->effects_count; j++) {
			dev_err(aw8622x->dev, "effect: %d, idx: %u, ram_id: %u, vmax: %u, time_ms: %u, rtp_enable: %d, rtp_file_name: %s\n",
				j, aw8622x->effect_list[j].idx, aw8622x->effect_list[j].ram_id, aw8622x->effect_list[j].vmax, aw8622x->effect_list[j].times_ms,
				aw8622x->effect_list[j].rtp_enable, aw8622x->effect_list[j].rtp_file_name);
		}
		dev_err(aw8622x->dev, "Base Scenes Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8622x->base_scene_count; j++) {
			dev_err(aw8622x->dev, "base scene: %d, scene_id: %u, effect_id: %u, real_vmax: %u\n",
				j, aw8622x->base_scene_list[j].scene_id, aw8622x->base_scene_list[j].effect_id, aw8622x->base_scene_list[j].real_vmax);
		}
		dev_err(aw8622x->dev, "Ext Scenes Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8622x->ext_scene_count; j++) {
			dev_err(aw8622x->dev, "ext scene: %d, scene_id: %u, effect_id: %u, real_vmax: %u\n",
				j, aw8622x->ext_scene_list[j].scene_id, aw8622x->ext_scene_list[j].effect_id, aw8622x->ext_scene_list[j].real_vmax);
		}

	}


	/* register haptic miscdev */
	hm = devm_kzalloc(aw8622x->dev, sizeof(struct haptic_misc), GFP_KERNEL);
	if (!hm) {
		dev_err(aw8622x->dev,  "%s: kzalloc for haptic_misc failed\n", __func__);
		ret = -ENOMEM;
		goto dts_err;
	}

	/* register haptic miscdev */
	ret = haptic_handle_create(hm, "vivo_haptic");
	if (ret < 0) {
		dev_err(aw8622x->dev, "%s: handle create failed\n", __func__);
		ret = -ENOMEM;
		goto dts_err;
	}
	handle = hm->handle;
	handle->chip = aw8622x;
	handle->dev = &aw8622x->i2c->dev;
	handle->upload = aw8622x_upload_sync;
	handle->playback = aw8622x_playback_sync;
	handle->erase = aw8622x_erase_sync;
	handle->set_gain = aw8622x_set_gain_sync;
	handle->get_motor_type = aw8622x_get_motor_type_sync;
	handle->get_f0 = aw8622x_get_f0_sync;
	handle->set_cali_params = aw8622x_set_cali_params_sync;
	handle->get_driver_ic = aw8622x_get_driver_ic_sync;
	handle->judge_effect_support = aw8622x_judge_effect_support_sync;
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_SUPPORT_EFFECT);
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_SUPPORT_GAIN);
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_TRIGGER_INTENSITY);

	ret = haptic_miscdev_register(hm);
	if (ret) {
		dev_err(aw8622x->dev, "%s register misc register failed, ret=%d\n", __func__, ret);
		ret = -EFAULT;
		goto haptic_core_err;
	}
	aw8622x->hm = hm;

	order = get_order(RTP_BIN_MAX_SIZE + sizeof(int));
	aw8622x_rtp = (struct haptic_rtp_container *)__get_free_pages(GFP_KERNEL, order);
	if (aw8622x_rtp == NULL) {
		dev_err(aw8622x->dev, "Error __get_free_pages failed\n");
		aw8622x->rtp_mmap_page_alloc_flag = false;
	} else {
		aw8622x->rtp_mmap_page_alloc_flag = true;
		SetPageReserved(virt_to_page(aw8622x_rtp));
		aw8622x->hm->handle->haptic_container = aw8622x_rtp;
		g_order = order;

	}

	// led class sysfs

	aw8622x->cdev.name = "vibrator";
	aw8622x->cdev.brightness_get = aw8622x_haptic_brightness_get;
	aw8622x->cdev.brightness_set = aw8622x_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8622x->i2c->dev, &aw8622x->cdev);
	if (ret) {
		dev_err(aw8622x->dev, "%s: fail to create led dev\n", __func__);
		ret = -EFAULT;
		goto sysfs_err;
	}

	ret = sysfs_create_group(&aw8622x->cdev.dev->kobj, &aw8622x_attribute_group);
	if (ret) {
		dev_err(aw8622x->dev, "%s  error creating sysfs attr files\n", __func__);
		ret = -EFAULT;
		goto sysfs_err;
	}

	return 0;

sysfs_err:
	if (aw8622x_rtp)
		free_pages((unsigned long)aw8622x_rtp, order);
	haptic_miscdev_unregister(hm);
haptic_core_err:
	haptic_handle_destroy(hm->handle);
dts_err:
	if (rtp_cachep)
		kmem_cache_destroy(rtp_cachep);
	destroy_workqueue(rtp_wq);
	mutex_destroy(&aw8622x->lock);
	mutex_destroy(&aw8622x->rtp_lock);
	mutex_destroy(&aw8622x->rtp_check_lock);
	lra_wake_lock_unregister(aw8622x->wklock);


	return ret;
}


static int aw8622x_haptic_init(struct aw8622x *aw8622x) //
{
	unsigned char i = 0;
	unsigned char reg_val = 0;
	int retry = 3;


	aw8622x->activate_mode = aw8622x->dts_info.mode;

	aw8622x_haptic_play_mode(aw8622x, AW8622X_HAPTIC_STANDBY_MODE);
	aw8622x_haptic_set_pwm(aw8622x, AW8622X_PWM_12K);
	/* misc value init */
	aw8622x_haptic_misc_para_init(aw8622x);
	/* set motor protect */
	aw8622x_haptic_swicth_motor_protect_config(aw8622x, 0x00, 0x00);

	aw8622x_haptic_offset_calibration(aw8622x);
	/*config auto_brake*/
	aw8622x_haptic_auto_bst_enable(aw8622x, aw8622x->dts_info.is_enabled_auto_bst);
	/* vbat compensation */
	aw8622x_haptic_vbat_mode_config(aw8622x,
				AW8622X_HAPTIC_CONT_VBAT_HW_ADJUST_MODE);
	aw8622x->ram_vbat_compensate = AW8622X_HAPTIC_RAM_VBAT_COMP_ENABLE;

	/* f0 calibration */
	/*LRA trim source select register*/
	aw8622x_i2c_write_bits(aw8622x,
				AW8622X_REG_TRIMCFG1,
				AW8622X_BIT_TRIMCFG1_RL_TRIM_SRC_MASK,
				AW8622X_BIT_TRIMCFG1_RL_TRIM_SRC_REG);
	aw8622x_haptic_upload_lra(aw8622x, WRITE_ZERO);
//	aw8622x_haptic_f0_calibration(aw8622x);

	return 0;
}

/*****************************************************************
 *
 * ram init
 *
 ***************************************************************/

static int aw8622x_container_update(struct aw8622x *aw8622x, struct aw8622x_container *aw8622x_cont) //
{
	unsigned char reg_val = 0;
	unsigned int shift = 0;
	unsigned int temp = 0;
	int i = 0;
	int ret = 0;
#ifdef AW_CHECK_RAM_DATA
	unsigned short check_sum = 0;
#endif

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	mutex_lock(&aw8622x->lock);
	aw8622x->ram.baseaddr_shift = 2;
	aw8622x->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw8622x_haptic_raminit(aw8622x, true);
	/* Enter standby mode */
	aw8622x_haptic_stop(aw8622x);
	/* base addr */
	shift = aw8622x->ram.baseaddr_shift;
	aw8622x->ram.base_addr =
		(unsigned int)((aw8622x_cont->data[0 + shift] << 8) |
			(aw8622x_cont->data[1 + shift]));

	/* default 3k SRAM */
	aw8622x_sram_size(aw8622x, AW8622X_HAPTIC_SRAM_3K);

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG1, /*ADDRH*/
			AW8622X_BIT_RTPCFG1_ADDRH_MASK,
			aw8622x_cont->data[0 + shift]);

	aw8622x_i2c_write(aw8622x, AW8622X_REG_RTPCFG2, /*ADDRL*/
			aw8622x_cont->data[1 + shift]);

	/* FIFO_AEH */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG3,
			AW8622X_BIT_RTPCFG3_FIFO_AEH_MASK,
			(unsigned char)
				(((aw8622x->ram.base_addr >> 1) >> 4) & 0xF0));
	/* FIFO AEL */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_RTPCFG4,
			(unsigned char)
				(((aw8622x->ram.base_addr >> 1) & 0x00FF)));
	/* FIFO_AFH */
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RTPCFG3,
				AW8622X_BIT_RTPCFG3_FIFO_AFH_MASK,
				(unsigned char)(((aw8622x->ram.base_addr -
				(aw8622x->ram.base_addr >> 2)) >> 8) & 0x0F));
	/* FIFO_AFL */
	aw8622x_i2c_write(aw8622x, AW8622X_REG_RTPCFG5,
			(unsigned char)(((aw8622x->ram.base_addr -
				(aw8622x->ram.base_addr >> 2)) & 0x00FF)));
/*
*	unsigned int temp
*	HIGH<byte4 byte3 byte2 byte1>LOW
*	|_ _ _ _AF-12BIT_ _ _ _AE-12BIT|
*/
	aw8622x_i2c_read(aw8622x, AW8622X_REG_RTPCFG3, &reg_val);
	temp = ((reg_val & 0x0f) << 24) | ((reg_val & 0xf0) << 4);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_RTPCFG4, &reg_val);
	temp = temp | reg_val;
	dev_info(aw8622x->dev, "%s: almost_empty_threshold = %d\n", __func__,
		(unsigned short)temp);
	aw8622x_i2c_read(aw8622x, AW8622X_REG_RTPCFG5, &reg_val);
	temp = temp | (reg_val << 16);
	dev_info(aw8622x->dev, "%s: almost_full_threshold = %d\n", __func__,
		temp >> 16);
	/* ram */
	shift = aw8622x->ram.baseaddr_shift;

	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RAMADDRH,
			AW8622X_BIT_RAMADDRH_MASK,
			aw8622x_cont->data[0 + shift]);
	aw8622x_i2c_write(aw8622x, AW8622X_REG_RAMADDRL,
			aw8622x_cont->data[1 + shift]);
	shift = aw8622x->ram.ram_shift;
	dev_info(aw8622x->dev, "%s: ram_len = %d\n", __func__,
			aw8622x_cont->len - shift);
	for (i = shift; i < aw8622x_cont->len; i++) {
		aw8622x->ram_update_flag = aw8622x_i2c_write(aw8622x,
							AW8622X_REG_RAMDATA,
							aw8622x_cont->data
							[i]);
	}
#ifdef	AW_CHECK_RAM_DATA
	shift = aw8622x->ram.baseaddr_shift;
	aw8622x_i2c_write_bits(aw8622x, AW8622X_REG_RAMADDRH,
			AW8622X_BIT_RAMADDRH_MASK,
			aw8622x_cont->data[0 + shift]);
	aw8622x_i2c_write(aw8622x, AW8622X_REG_RAMADDRL,
			aw8622x_cont->data[1 + shift]);
	shift = aw8622x->ram.ram_shift;
	for (i = shift; i < aw8622x_cont->len; i++) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_RAMDATA, &reg_val);
		/*
		* dev_info(aw8622x->dev,
		*	"%s aw8622x_cont->data=0x%02X, ramdata=0x%02X\n",
		*	__func__,aw8622x_cont->data[i],reg_val);
		*/
		if (reg_val != aw8622x_cont->data[i]) {
			dev_err(aw8622x->dev,
				"%s: ram check error addr=0x%04x, file_data=0x%02X, ram_data=0x%02X\n",
				__func__, i, aw8622x_cont->data[i], reg_val);
			ret = -1;
			break;
		}
		check_sum += reg_val;
	}
	if (!ret) {
		aw8622x_i2c_read(aw8622x, AW8622X_REG_RTPCFG1, &reg_val);
		check_sum += reg_val & 0x0f;
		aw8622x_i2c_read(aw8622x, AW8622X_REG_RTPCFG2, &reg_val);
		check_sum += reg_val;

		if (check_sum != aw8622x->ram.check_sum) {
			dev_err(aw8622x->dev, "%s: ram data check sum error, check_sum=0x%04x\n",
				__func__, check_sum);
			ret = -1;
		} else {
			dev_info(aw8622x->dev, "%s: ram data check sum pass, check_sum=0x%04x\n",
				 __func__, check_sum);
		}
	}

#endif
	/* RAMINIT Disable */
	aw8622x_haptic_raminit(aw8622x, false);
	mutex_unlock(&aw8622x->lock);
	dev_info(aw8622x->dev, "%s exit\n", __func__);
	return ret;
}



static void aw8622x_ram_loaded(const struct firmware *cont, void *context) //
{
	struct aw8622x *aw8622x = context;
	struct aw8622x_container *aw8622x_fw;
	unsigned short check_sum = 0;
	int i = 0;
	int ret = 0;
#ifdef AW_READ_BIN_FLEXBALLY
	static unsigned char load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	dev_info(aw8622x->dev, "%s enter\n", __func__);
	if (!cont) {
		dev_err(aw8622x->dev, "%s: failed to read %s\n", __func__,
			   aw8622x_ram_name);
		release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw8622x->ram_work,
						msecs_to_jiffies(ram_timer_val));
			dev_info(aw8622x->dev, "%s:start hrtimer: load_cont=%d\n",
					__func__, load_cont);
		}
#endif
		return;
	}
	dev_info(aw8622x->dev, "%s: loaded %s - size: %zu bytes\n", __func__,
		aw8622x_ram_name, cont ? cont->size : 0);
/*
*	for(i=0; i < cont->size; i++) {
*		dev_info(aw8622x->dev, "%s: addr: 0x%04x, data: 0x%02X\n",
*			__func__, i, *(cont->data+i));
*	}
*/
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum !=
		(unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		dev_err(aw8622x->dev,
			"%s: check sum err: check_sum=0x%04x\n", __func__,
			check_sum);
		return;
	} else {
		dev_info(aw8622x->dev, "%s: check sum pass: 0x%04x\n",
			__func__, check_sum);
		aw8622x->ram.check_sum = check_sum;
	}

	/* aw8622x ram update less then 128kB */
	aw8622x_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8622x_fw) {
		release_firmware(cont);
		dev_err(aw8622x->dev, "%s: Error allocating memory\n",
			__func__);
		return;
	}
	aw8622x_fw->len = cont->size;
	memcpy(aw8622x_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw8622x_container_update(aw8622x, aw8622x_fw);
	if (ret) {
		kfree(aw8622x_fw);
		aw8622x->ram.len = 0;
		dev_err(aw8622x->dev, "%s: ram firmware update failed!\n",
			__func__);
	} else {
		aw8622x->ram_init = 1;
		aw8622x->ram.len = aw8622x_fw->len;
		kfree(aw8622x_fw);
		dev_info(aw8622x->dev, "%s: ram firmware update complete!\n",
			__func__);
	}

}

static int aw8622x_ram_update(struct aw8622x *aw8622x) //
{
	aw8622x->ram_init = 0;
	aw8622x->rtp_init = 0;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw8622x_ram_name, aw8622x->dev,
			GFP_KERNEL, aw8622x, aw8622x_ram_loaded);
}


static void aw8622x_ram_work_routine(struct work_struct *work) //
{
	struct aw8622x *aw8622x = container_of(work, struct aw8622x, ram_work.work);

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	aw8622x_ram_update(aw8622x);
}

static int aw8622x_ram_work_init(struct aw8622x *aw8622x) //
{
	int ram_timer_val = 8000;

	dev_info(aw8622x->dev, "%s enter\n", __func__);
	INIT_DELAYED_WORK(&aw8622x->ram_work, aw8622x_ram_work_routine);
	schedule_delayed_work(&aw8622x->ram_work, msecs_to_jiffies(ram_timer_val));
	return 0;
}

static int  aw8622x_lra_information_ctr(struct aw8622x *aw8622x, struct device_node *np)
{
	unsigned int val = 0;
	unsigned int prctmode_temp[3];
	unsigned int sine_array_temp[4];

	val = of_property_read_u32(np, "aw8622x_vib_mode", &aw8622x->dts_info.mode);
	if (val)
		dev_err(aw8622x->dev, "%s aw8622x_vib_mode not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_f0_pre", &aw8622x->dts_info.f0_ref);
	if (val)
		dev_info(aw8622x->dev, "%s vib_f0_ref not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_f0_cali_percen", &aw8622x->dts_info.f0_cali_percent);
	if (val)
		dev_info(aw8622x->dev, "%s vib_f0_cali_percent not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_drv1_lvl", &aw8622x->dts_info.cont_drv1_lvl_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_drv1_lvl not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_drv2_lvl", &aw8622x->dts_info.cont_drv2_lvl_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_drv2_lvl not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_drv1_time", &aw8622x->dts_info.cont_drv1_time_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_drv1_time not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_drv2_time", &aw8622x->dts_info.cont_drv2_time_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_drv2_time not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_drv_width", &aw8622x->dts_info.cont_drv_width);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_drv_width not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_wait_num", &aw8622x->dts_info.cont_wait_num_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_wait_num not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_brk_gain", &aw8622x->dts_info.cont_brk_gain);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_brk_gain not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_tset", &aw8622x->dts_info.cont_tset);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_tset not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_bemf_set", &aw8622x->dts_info.cont_bemf_set);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_bemf_set not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_d2s_gain", &aw8622x->dts_info.d2s_gain);
	if (val)
		dev_info(aw8622x->dev, "%s vib_d2s_gain not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_brk_time", &aw8622x->dts_info.cont_brk_time_dt);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_brk_time not found\n", __func__);

	val = of_property_read_u32(np, "aw8622x_vib_cont_track_margin", &aw8622x->dts_info.cont_track_margin);
	if (val)
		dev_info(aw8622x->dev, "%s vib_cont_track_margin not found\n", __func__);


	val = of_property_read_u32_array(np, "aw8622x_vib_prctmode", prctmode_temp, ARRAY_SIZE(prctmode_temp));
	if (val)
		dev_info(aw8622x->dev, "%s vib_prctmode not found\n", __func__);

	memcpy(aw8622x->dts_info.prctmode, prctmode_temp, sizeof(prctmode_temp));

	val = of_property_read_u32_array(np, "aw8622x_vib_sine_array", sine_array_temp, ARRAY_SIZE(sine_array_temp));
	if (val)
		dev_info(aw8622x->dev, "%s vib_sine_array not found\n", __func__);
	memcpy(aw8622x->dts_info.sine_array, sine_array_temp, sizeof(sine_array_temp));

	aw8622x->dts_info.is_enabled_auto_bst = of_property_read_bool(np, "aw8622x_vib_is_enabled_auto_bst");

	return 0;
}


static int aw8622x_parse_dt(struct device *dev, struct aw8622x *aw8622x,
			struct device_node *np)
{
	struct device_node *child;
	int rc, i = 0, effect_count = 0;

	aw8622x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(aw8622x->reset_gpio)) {
		aw8622x->reset_gpio = -1;
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		return -ENODEV;
	}

	aw8622x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(aw8622x->reset_gpio)) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "awinic,vmax", &aw8622x->default_vmax)) {
		dev_err(dev, "%s: no default vmax\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "resistance_min", &aw8622x->resistance_min)) {
		dev_err(dev, "%s: no resistance_min\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "resistance_max", &aw8622x->resistance_max)) {
		dev_err(dev, "%s: no resistance_max\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "freq_min", &aw8622x->freq_min)) {
		dev_err(dev, "%s: no freq_min\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "freq_max", &aw8622x->freq_max)) {
		dev_err(dev, "%s: no freq_max\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_bool(np, "disable-trigger")) {
		dev_info(dev, "not support trigger\n");
		aw8622x->no_trigger = true;
	}

	aw8622x->add_suffix = of_property_read_bool(np, "effect,add-suffix");


	if (of_property_read_u32(np, "lra_info", &aw8622x->lra_information)) {
		aw8622x->lra_information = 619;
		dev_err(dev, "get lra info failed, use default,lra_info:%d\n", aw8622x->lra_information);
	}

	rc = aw8622x_lra_information_ctr(aw8622x, np);
	if (rc < 0) {
		dev_err(dev, "%s: lra_information_ctr get failed\n", __func__);
	}

	return 0;
}

static int aw8622x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct  aw8622x *aw8622x;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	dev_err(&i2c->dev, "%s  enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	if (!np) {
		dev_err(&i2c->dev, "No dts device node\n");
		return -ENOENT;
	}

	aw8622x = devm_kzalloc(&i2c->dev, sizeof(struct aw8622x), GFP_KERNEL);
	if (aw8622x == NULL) {
		dev_err(&i2c->dev, "aw8622x kzalloc failed\n");
		return -ENOMEM;
	}

	aw8622x->dev = &i2c->dev;
	aw8622x->i2c = i2c;
	i2c_set_clientdata(i2c, aw8622x);
	mutex_init(&aw8622x->bus_lock);
	g_aw8622x = aw8622x;

	ret = aw8622x_parse_dt(&i2c->dev, aw8622x, np);
	if (ret) {
		dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
		return -ENODEV;
	}

	ret = devm_gpio_request_one(&i2c->dev, aw8622x->reset_gpio,
			GPIOF_OUT_INIT_LOW, "aw8622x_rst");
	if (ret) {
		dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
		return -ENODEV;
	}
	msleep(2);
	gpio_set_value_cansleep(aw8622x->reset_gpio, 1);
	msleep(5);

	ret = aw8622x_read_chipid(aw8622x);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8622x_read_chipid failed ret=%d\n", __func__, ret);
		return -ENODEV;
	}


	ret = devm_gpio_request_one(&i2c->dev, aw8622x->irq_gpio,
				GPIOF_DIR_IN, "aw8622x_int");
	if (ret) {
		dev_err(&i2c->dev, "%s: int request failed\n", __func__);
		return -ENODEV;
	} else {

		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev, gpio_to_irq(aw8622x->irq_gpio),
				NULL, aw8622x_irq, irq_flags, "aw8622x_irq", aw8622x);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
							__func__, gpio_to_irq(aw8622x->irq_gpio), ret);
			return -ENODEV;
		}
	}

	aw8622x_interrupt_setup(aw8622x);

	/* hw init */
	aw8622x_haptic_init(aw8622x);

	ret = aw8622x_vibrator_init(aw8622x);
	if (ret) {
		dev_err(&i2c->dev, "%s: vibrator init failed, ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	aw8622x_ram_work_init(aw8622x);

	dev_err(&i2c->dev, "%s  probe completed successfully!\n", __func__);

	return 0;
}

static int aw8622x_i2c_remove(struct i2c_client *i2c)
{
	struct aw8622x *aw8622x = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&aw8622x->ram_work);

	cancel_work_sync(&aw8622x->rtp_work);
	cancel_work_sync(&aw8622x->long_vibrate_work);

	hrtimer_cancel(&aw8622x->timer);

	sysfs_remove_group(&aw8622x->dev->kobj, &aw8622x_attribute_group);
	haptic_miscdev_unregister(aw8622x->hm);

	destroy_workqueue(rtp_wq);

	if (aw8622x_rtp && g_order)
		free_pages((unsigned long)aw8622x_rtp, g_order);

	if (rtp_cachep)
		kmem_cache_destroy(rtp_cachep);

	mutex_destroy(&aw8622x->lock);
	mutex_destroy(&aw8622x->rtp_lock);
	mutex_destroy(&aw8622x->bus_lock);
	mutex_destroy(&aw8622x->rtp_check_lock);
	lra_wake_lock_unregister(aw8622x->wklock);

	return 0;
}

static void aw8622x_pm_shutdown(struct i2c_client *i2c)
{

}

static int aw8622x_pm_suspend (struct device *dev)
{
	i2c_suspend = 1;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);

	return 0;
}


static int aw8622x_pm_resume (struct device *dev)
{

	i2c_suspend = 0;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);
	return 0;
}

static struct dev_pm_ops aw8622x_pm_ops = {
	.suspend = aw8622x_pm_suspend,
	.resume = aw8622x_pm_resume,
};


static const struct i2c_device_id aw8622x_i2c_id[] = {
	{ AW8622x_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8622x_i2c_id);

static struct of_device_id aw8622x_dt_match[] = {
	{ .compatible = "awinic,aw8622x_haptic" },
	{ },
};

static struct i2c_driver aw8622x_i2c_driver = {
	.driver = {
		.name = AW8622x_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8622x_dt_match),
		.pm = &aw8622x_pm_ops,
	},
	.shutdown = aw8622x_pm_shutdown,
	.probe = aw8622x_i2c_probe,
	.remove = aw8622x_i2c_remove,
	.id_table = aw8622x_i2c_id,
};

static int __init aw8622x_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8622x driver version %s\n", AW8622x_DRIVER_VERSION);
	ret = i2c_add_driver(&aw8622x_i2c_driver);
	if (ret) {
		pr_err("aw8622x fail to add device into i2c\n");
		return ret;
	}

	return 0;
}
module_init(aw8622x_i2c_init);


static void __exit aw8622x_i2c_exit(void)
{
	i2c_del_driver(&aw8622x_i2c_driver);
}
module_exit(aw8622x_i2c_exit);


MODULE_DESCRIPTION("AW8622x Haptic Driver");
MODULE_LICENSE("GPL v2");


