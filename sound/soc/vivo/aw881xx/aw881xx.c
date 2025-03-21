/*
 * aw881xx.c   aw881xx codec module
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include "aw881xx.h"
#include "aw881xx_reg.h"
#include "aw881xx_monitor.h"
#include "aw881xx_cali.h"
#include "../../mediatek/common/mtk-sp-spk-amp.h"
#include "../vivo_smartpa_cali.h"
#include "../vivo-codec-common.h"


/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW881XX_I2C_NAME "aw881xx_smartpa"

#define AW881XX_DRIVER_VERSION "V1.8.0.4"

#define AW881XX_RATES SNDRV_PCM_RATE_8000_48000
#define AW881XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5	/* 5ms */
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5	/* 5ms */

#define AW881XX_ACF_FILE		"aw881xx_acf.bin"

static DEFINE_MUTEX(g_aw881xx_lock);
static LIST_HEAD(g_dev_list);
static struct aw881xx_container *g_awinic_cfg;
static unsigned int g_aw881xx_dev_cnt;

static unsigned int g_fade_in_time = AW_1000_US / 10;
static unsigned int g_fade_out_time = AW_1000_US >> 1;
static unsigned int g_fade_step = AW_VOLUME_STEP_DB;

static char *profile_name[AW_PROFILE_MAX] = {"Music", "Voice", "Voip", "Ringtone", "Ringtone_hs", "Lowpower",
						"Bypass", "Mmi", "Fm", "Notification", "Receiver"};
static const char *const aw881xx_switch[] = {"Disable", "Enable"};

static int cali_fail_mute_en = 1;
static struct aw881xx *aw_pa_dev = NULL;

/******************************************************
 *
 * Value
 *
 ******************************************************/
static char aw881xx_cfg_name[][AW881XX_CFG_NAME_MAX] = {
	/* ui */
	{"aw881xx_pid_01_ui_fw_i2c"},
	{"aw881xx_pid_01_ui_cfg_i2c"},
};

/******************************************************
 *
 * aw881xx distinguish between codecs and components by version
 *
 ******************************************************/
#ifdef AW_KERNEL_VER_OVER_4_19_1
static struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_component,
	.aw_snd_soc_codec_get_drvdata = snd_soc_component_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_component_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_component,
	.aw_snd_soc_register_codec = snd_soc_register_component,
};
#else
static struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_codec,
	.aw_snd_soc_codec_get_drvdata = snd_soc_codec_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_codec_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_codec,
	.aw_snd_soc_register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef AW_KERNEL_VER_OVER_4_19_1
	return dai->component;
#else
	return dai->codec;
#endif
}

static void aw881xx_reg_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len);
static int aw881xx_load_chip_profile(struct aw881xx *aw881xx);
static int aw881xx_fw_update(struct aw881xx *aw881xx, bool up_dsp_fw_en);



/******************************************************
 *
 * aw881xx append suffix sound channel information
 *
 ******************************************************/
static void *aw881xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = NULL;

	str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);
	if (!str) {
		aw_dev_err(dev, "%s:devm_kzalloc %s failed\n",
			__func__, buf);
		return str;
	}
	memcpy(str, buf, strlen(buf));
	return str;
}

static int aw881xx_append_suffix(char *format, const char **change_name,
			struct aw881xx *aw881xx)
{
	char buf[64] = { 0 };
	int i2cbus = aw881xx->i2c->adapter->nr;
	int addr = aw881xx->i2c->addr;

	snprintf(buf, 64, format, *change_name, i2cbus, addr);
	*change_name = aw881xx_devm_kstrdup(aw881xx->dev, buf);
	if (!(*change_name))
		return -ENOMEM;

	aw_dev_info(aw881xx->dev, "%s:change name :%s\n",
		__func__, *change_name);
	return 0;
}

int aw881xx_get_version(char *buf, int size)
{
	if (size > strlen(AW881XX_DRIVER_VERSION)) {
		memcpy(buf, AW881XX_DRIVER_VERSION, strlen(AW881XX_DRIVER_VERSION));
		return strlen(AW881XX_DRIVER_VERSION);
	} else {
		return -ENOMEM;
	}
}


int aw881xx_get_dev_num(void)
{
	return g_aw881xx_dev_cnt;
}

/******************************************************
 *
 * aw881xx reg write/read
 *
 ******************************************************/
static int aw881xx_i2c_writes(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	int ret = -1;
	uint8_t *data = NULL;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		aw_dev_err(aw881xx->dev, "%s: can not allocate memory\n",
				__func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw881xx->i2c, data, len + 1);
	if (ret < 0)
		aw_dev_err(aw881xx->dev,
				"%s: i2c master send error\n", __func__);

	kfree(data);
	data = NULL;

	return ret;
}

static int aw881xx_i2c_reads(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *data_buf,
				uint16_t data_len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
				.addr = aw881xx->i2c->addr,
				.flags = 0,
				.len = sizeof(uint8_t),
				.buf = &reg_addr,
				},
		[1] = {
				.addr = aw881xx->i2c->addr,
				.flags = I2C_M_RD,
				.len = data_len,
				.buf = data_buf,
				},
	};

	ret = i2c_transfer(aw881xx->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != AW881XX_READ_MSG_NUM) {
		aw_dev_err(aw881xx->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	return 0;
}

static int aw881xx_i2c_write(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint16_t reg_data)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t buf[2] = { 0 };

	buf[0] = (reg_data & 0xff00) >> 8;
	buf[1] = (reg_data & 0x00ff) >> 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = aw881xx_i2c_writes(aw881xx, reg_addr, buf, 2);
		if (ret < 0)
			aw_dev_err(aw881xx->dev,
					"%s: i2c_write cnt=%d error=%d\n",
					__func__, cnt, ret);
		else
			break;
		cnt++;
	}

	if (aw881xx->i2c_log_en)
		aw_dev_info(aw881xx->dev, "%s: write: reg = 0x%02x, val = 0x%04x",
			__func__, reg_addr, reg_data);

	return ret;
}

static int aw881xx_i2c_read(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint16_t *reg_data)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t buf[2] = { 0 };

	while (cnt < AW_I2C_RETRIES) {
		ret = aw881xx_i2c_reads(aw881xx, reg_addr, buf, 2);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
					"%s: i2c_read cnt=%d error=%d\n",
					__func__, cnt, ret);
		} else {
			*reg_data = (buf[0] << 8) | (buf[1] << 0);
			break;
		}
		cnt++;
	}

	if (aw881xx->i2c_log_en)
		aw_dev_info(aw881xx->dev, "%s: read: reg = 0x%02x, val = 0x%04x",
			__func__, reg_addr, *reg_data);

	return ret;
}

static int aw881xx_i2c_write_bits(struct aw881xx *aw881xx,
					uint8_t reg_addr, uint16_t mask,
					uint16_t reg_data)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_i2c_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data & (~mask);
	ret = aw881xx_i2c_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

int aw881xx_reg_writes(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_writes(aw881xx, reg_addr, buf, len);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881x_i2c_writes fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_write(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, reg_addr, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881xx_i2c_write fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_read(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t *reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_read(aw881xx, reg_addr, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881xx_i2c_read fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_write_bits(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t mask, uint16_t reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write_bits(aw881xx, reg_addr, mask, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev,
				"%s: aw881xx_i2c_write_bits fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

void aw881xx_reg_dump(struct aw881xx *aw881xx)
{
	uint8_t i = 0;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_REG_MAX; i++) {
		if (aw881xx_reg_access[i] & REG_RD_ACCESS) {
			aw881xx_reg_read(aw881xx, i, &reg_val);
			aw_dev_info(aw881xx->dev,
				"%s: reg:0x%02x=0x%04x\n",
				__func__, i, reg_val);
		}
	}
}

/* vivo audio dxl add for dump reg start */
void aw881xx_reg_dump_vivo(struct aw881xx *aw881xx)
{
	uint8_t i = 0;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_REG_MAX; i++) {
		if (aw881xx_reg_access_vivo[i] & REG_RD_ACCESS) {
			aw881xx_reg_read(aw881xx, i, &reg_val);
			aw_dev_info(aw881xx->dev,
				"%s: reg:0x%02x=0x%04x\n",
				__func__, i, reg_val);
		}
	}
}
/* vivo audio dxl add for dump reg end */

static void aw881xx_clear_dsp_sel_st(struct aw881xx *aw881xx)
{
	uint16_t reg_value;
	uint8_t reg = AW881XX_REG_ID;

	aw881xx_i2c_read(aw881xx, reg, &reg_value);
}

int aw881xx_dsp_write(struct aw881xx *aw881xx,
			uint16_t dsp_addr, uint16_t dsp_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, dsp_addr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_write_err;
	}

	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMDAT, dsp_data);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_write_err;
	}

 dsp_write_err:
	aw881xx_clear_dsp_sel_st(aw881xx);
	mutex_unlock(&aw881xx->i2c_lock);
	return ret;

}

int aw881xx_dsp_read(struct aw881xx *aw881xx,
			uint16_t dsp_addr, uint16_t *dsp_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, dsp_addr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_read_err;
	}

	ret = aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, dsp_data);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		goto dsp_read_err;
	}

 dsp_read_err:
	aw881xx_clear_dsp_sel_st(aw881xx);
	mutex_unlock(&aw881xx->i2c_lock);
	return ret;
}

int aw881xx_get_list_head(struct list_head **head)
{
	if (list_empty(&g_dev_list))
		return -EINVAL;

	*head = &g_dev_list;

	return 0;
}

static int aw881xx_get_profile_count(struct aw881xx *aw881xx)
{
	return aw881xx->prof_info.count;
}

static char *aw881xx_get_profile_name(struct aw881xx *aw881xx, int index)
{
	int dev_profile_id;

	if ((index >= aw881xx->prof_info.count) || (index < 0)) {
		aw_dev_err(aw881xx->dev, "%s:index[%d] overflow dev prof num[%d]\n",
				__func__, index, aw881xx->prof_info.count);
		return NULL;
	}

	if (aw881xx->prof_info.prof_desc[index].id >= aw881xx->prof_info.prof_max) {
		aw_dev_err(aw881xx->dev, "%s:can not find match id[%d]\n", __func__,
			aw881xx->prof_info.prof_desc[index].id);
		return NULL;
	}

	dev_profile_id = aw881xx->prof_info.prof_desc[index].id;

	return aw881xx->prof_info.prof_name_list[dev_profile_id];
}

static int aw881xx_get_profile_index(struct aw881xx *aw881xx)
{
	return aw881xx->cur_prof;
}

static int aw881xx_check_profile_index(struct aw881xx *aw881xx, int index)
{
	if ((index >= aw881xx->prof_info.count) || (index < 0))
		return -EINVAL;
	else
		return 0;
}

static int aw881xx_set_profile_index(struct aw881xx *aw881xx, int index)
{
	int dev_profile_id = 0;

	if ((index >= aw881xx->prof_info.count) || (index < 0)) {
		return -EINVAL;
	} else {
		dev_profile_id = aw881xx->prof_info.prof_desc[index].id;
		aw881xx->set_prof = index;
		aw_dev_info(aw881xx->dev, "%s:set prof[%s]\n", __func__,
			aw881xx->prof_info.prof_name_list[dev_profile_id]);
	}

	return 0;
}

static int aw881xx_get_profile_data(struct aw881xx *aw881xx, int index,
			struct aw_prof_desc **prof_desc)
{
	if (index >= aw881xx->prof_info.count) {
		aw_dev_err(aw881xx->dev, "%s: index[%d] overflow count[%d]\n",
			__func__, index, aw881xx->prof_info.count);
		return -EINVAL;
	}

	*prof_desc = &aw881xx->prof_info.prof_desc[index];

	return 0;
}

/*
[7 : 4]: -6DB ; [3 : 0]: 0.5DB  real_value = value * 2 : 0.5db --> 1
*/
uint32_t aw881xx_reg_val_to_db(uint32_t value)
{
	return ((value >> 4) * AW881XX_VOL_6DB_STEP +
			((value & 0x0f) % AW881XX_VOL_6DB_STEP));
}

/*[7 : 4]: -6DB ;
 *[3 : 0]: -0.5DB reg_value = value / step << 4 + value % step ;
 *step = 6 * 2
 */
static uint32_t aw881xx_db_val_to_reg(uint32_t value)
{
	return (((value / AW881XX_VOL_6DB_STEP) << 4) + (value % AW881XX_VOL_6DB_STEP));
}

int aw881xx_set_volume(struct aw881xx *aw881xx, uint32_t value)
{
	uint16_t reg_value = 0;
	uint32_t real_value = aw881xx_db_val_to_reg(value);

	/*cal real value*/
	aw881xx_reg_read(aw881xx, AW881XX_REG_HAGCCFG6, &reg_value);

	aw_dev_dbg(aw881xx->dev, "%s: value %d , 0x%x\n",
			__func__, value, real_value);

	/*[15 : 8] volume*/
	real_value = (real_value << AW881XX_VOL_REG_SHIFT) | (reg_value & 0x00ff);

	/*write value*/
	aw881xx_reg_write(aw881xx, AW881XX_REG_HAGCCFG6, real_value);

	return 0;
}

int aw881xx_get_volume(struct aw881xx *aw881xx, uint32_t *value)
{
	uint16_t reg_value = 0;
	uint32_t real_value = 0;

	/* read value */
	aw881xx_reg_read(aw881xx, AW881XX_REG_HAGCCFG6, &reg_value);

	/*[15 : 8] volume*/
	real_value = reg_value >> AW881XX_VOL_REG_SHIFT;

	real_value = aw881xx_reg_val_to_db(real_value);
	*value = real_value;

	return 0;
}

static void aw881xx_fade_in(struct aw881xx *aw881xx)
{
	int i = 0;

	if (!aw881xx->fade_en)
		return;

	if (g_fade_step == 0 || g_fade_in_time == 0) {
		aw881xx_set_volume(aw881xx, aw881xx->db_offset);
		return;
	}

	/*volume up*/
	for (i = AW_FADE_OUT_TARGET_VOL; i >= (int32_t)aw881xx->db_offset; i -= g_fade_step) {
		aw881xx_set_volume(aw881xx, i);
		usleep_range(g_fade_in_time, g_fade_in_time + 10);
	}

	if (i != (int32_t)aw881xx->db_offset)
		aw881xx_set_volume(aw881xx, aw881xx->db_offset);

}

static void aw881xx_fade_out(struct aw881xx *aw881xx)
{
	int i = 0;
	unsigned start_volume = 0;

	if (!aw881xx->fade_en)
		return;

	if (g_fade_step == 0 || g_fade_out_time == 0) {
		aw881xx_set_volume(aw881xx, AW_FADE_OUT_TARGET_VOL);
		return;
	}

	aw881xx_set_volume(aw881xx, start_volume);
	i = start_volume;
	for (i = start_volume; i <= AW_FADE_OUT_TARGET_VOL; i += g_fade_step) {
		aw881xx_set_volume(aw881xx, i);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
	if (i != AW_FADE_OUT_TARGET_VOL) {
		aw881xx_set_volume(aw881xx, AW_FADE_OUT_TARGET_VOL);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
}



/******************************************************
 *
 * aw881xx control
 *
 ******************************************************/
static void aw881xx_i2s_tx_enable(struct aw881xx *aw881xx, bool is_enable)
{

	if (is_enable) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCFG1,
					AW881XX_BIT_I2SCFG1_TXEN_MASK,
					AW881XX_BIT_I2SCFG1_TXEN_ENABLE);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCFG1,
					AW881XX_BIT_I2SCFG1_TXEN_MASK,
					AW881XX_BIT_I2SCFG1_TXEN_DISABLE);
	}
}

static void aw881xx_run_mute(struct aw881xx *aw881xx, bool mute)
{
	aw_dev_info(aw881xx->dev, "%s: enter, cali_result=%d, mute_en=%d\n",
	           __func__, aw881xx->cali_desc.cali_result, cali_fail_mute_en);

	if (mute || ((aw881xx->cali_desc.cali_result == CALI_RESULT_ERROR) && cali_fail_mute_en)) {
		aw881xx_fade_out(aw881xx);
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PWMCTRL,
					AW881XX_BIT_PWMCTRL_HMUTE_MASK,
					AW881XX_BIT_PWMCTRL_HMUTE_ENABLE);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PWMCTRL,
					AW881XX_BIT_PWMCTRL_HMUTE_MASK,
					AW881XX_BIT_PWMCTRL_HMUTE_DISABLE);
		aw881xx_fade_in(aw881xx);
	}
}

static void aw881xx_run_pwd(struct aw881xx *aw881xx, bool pwd)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (pwd) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_PW_MASK,
					AW881XX_BIT_SYSCTRL_PW_PDN);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_PW_MASK,
					AW881XX_BIT_SYSCTRL_PW_ACTIVE);
	}
}

static void aw881xx_run_amppd(struct aw881xx *aw881xx, bool amppd)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (amppd) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_AMPPD_MASK,
					AW881XX_BIT_SYSCTRL_AMPPD_PDN);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_AMPPD_MASK,
					AW881XX_BIT_SYSCTRL_AMPPD_ACTIVE);
	}
}

static void aw881xx_dsp_enable(struct aw881xx *aw881xx, bool dsp)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (dsp) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_DSP_MASK,
					AW881XX_BIT_SYSCTRL_DSP_WORK);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_DSP_MASK,
					AW881XX_BIT_SYSCTRL_DSP_BYPASS);
	}
}

static void aw881xx_memclk_select(struct aw881xx *aw881xx, unsigned char flag)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (flag == AW881XX_MEMCLK_PLL) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL2,
					AW881XX_BIT_SYSCTRL2_MEMCLK_MASK,
					AW881XX_BIT_SYSCTRL2_MEMCLK_PLL);
	} else if (flag == AW881XX_MEMCLK_OSC) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL2,
					AW881XX_BIT_SYSCTRL2_MEMCLK_MASK,
					AW881XX_BIT_SYSCTRL2_MEMCLK_OSC);
	} else {
		aw_dev_err(aw881xx->dev,
				"%s: unknown memclk config, flag=0x%x\n",
				__func__, flag);
	}
}

static int aw881xx_sysst_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	unsigned char i;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_SYSST_CHECK_MAX; i++) {
		aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
		if ((reg_val & (~AW881XX_BIT_SYSST_CHECK_MASK)) ==
			AW881XX_BIT_SYSST_CHECK) {
			ret = 0;
			return ret;
		} else {
			aw_dev_dbg(aw881xx->dev,
					"%s: check fail, cnt=%d, reg_val=0x%04x\n",
					__func__, i, reg_val);
			usleep_range(AW_2000_US, AW_2000_US + 20);
		}
	}
	aw_dev_info(aw881xx->dev, "%s: check fail\n", __func__);

	return ret;
}

int aw881xx_get_sysint(struct aw881xx *aw881xx, uint16_t *int_status)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINT, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: read sysint fail, ret=%d\n",
				__func__, ret);
	} else {
		*int_status = reg_val;
		aw_dev_dbg(aw881xx->dev, "%s:read interrupt reg = 0x%04x",
				__func__, *int_status);
	}

	return ret;
}

void aw881xx_clear_int_status(struct aw881xx *aw881xx)
{
	uint16_t int_status = 0;

	/*read int status and clear*/
	aw881xx_get_sysint(aw881xx, &int_status);

	/*make suer int status is clear*/
	aw881xx_get_sysint(aw881xx, &int_status);
}

static int aw881xx_get_monitor_sysint_st(struct aw881xx *aw881xx)
{
	int ret = 0;

	if ((aw881xx->sysint_st) & (~AW881XX_BIT_SYSINT_CHECK_MASK)) {
		aw_dev_err(aw881xx->dev, "%s:check fail:reg = 0x%04x",
					__func__, aw881xx->sysint_st);
		ret = -EINVAL;
	}
	aw881xx->sysint_st = 0;

	return ret;
}

static int aw881xx_sysint_check(struct aw881xx *aw881xx)
{
	int ret = 0;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);
	ret = aw881xx_get_sysint(aw881xx, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & (~AW881XX_BIT_SYSINT_CHECK_MASK)) {
		ret = -1;
		aw_dev_dbg(aw881xx->dev, "%s: check fail, reg_val=0x%04x\n",
				__func__, reg_val);
	}

	return ret;
}

int aw881xx_get_iis_status(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
	if (reg_val & AW881XX_BIT_SYSST_PLLS)
		ret = 0;
	else
		aw_dev_err(aw881xx->dev, "%s:check pll lock fail,reg_val:0x%04x",
			__func__, reg_val);

	return ret;
}

static int aw881xx_mode1_pll_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t iis_check_max = 5;
	uint16_t i = 0;

	for (i = 0; i < iis_check_max; i++) {
		ret = aw881xx_get_iis_status(aw881xx);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
					"%s: mode1 iis signal check error\n", __func__);
			usleep_range(AW_2000_US, AW_2000_US + 20);
		} else {
			return 0;
		}
	}

	return ret;
}

static int aw881xx_mode2_pll_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t iis_check_max = 5;
	uint16_t i = 0;
	uint16_t reg_val = 0;

	aw881xx_reg_read(aw881xx, AW881XX_REG_PLLCTRL1, &reg_val);
	reg_val &= (~AW881XX_I2S_CCO_MUX_MASK);
	aw_dev_dbg(aw881xx->dev, "%s:REG_PLLCTRL1_bit14=0x%04x\n", __func__, reg_val);
	if (reg_val == AW881XX_I2S_CCO_MUX_8_16_32KHZ_VALUE) {
		aw_dev_dbg(aw881xx->dev, "%s: CCO_MUX is already 0\n", __func__);
		return ret;
	}

	/* change mode2 */
	aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PLLCTRL1,
		AW881XX_I2S_CCO_MUX_MASK, AW881XX_I2S_CCO_MUX_8_16_32KHZ_VALUE);

	for (i = 0; i < iis_check_max; i++) {
		ret = aw881xx_get_iis_status(aw881xx);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
					"%s: mode2 iis signal check error\n", __func__);
			usleep_range(AW_2000_US, AW_2000_US + 20);
		} else {
			break;
		}
	}

	/* change mode1*/
	aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PLLCTRL1,
		AW881XX_I2S_CCO_MUX_MASK, AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE);

	if (ret == 0) {
		usleep_range(AW_2000_US, AW_2000_US + 20);
		for (i = 0; i < iis_check_max; i++) {
			ret = aw881xx_get_iis_status(aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev,
						"%s: mode2 switch to mode1, iis signal check error\n", __func__);
				usleep_range(AW_2000_US, AW_2000_US + 20);
			} else {
				break;
			}
		}
	}

	return ret;
}

static int aw881xx_syspll_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	ret = aw881xx_mode1_pll_check(aw881xx);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s: mode1 check iis failed try switch to mode2 check\n", __func__);
		ret = aw881xx_mode2_pll_check(aw881xx);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
				"%s: mode2 check iis failed \n", __func__);
			aw881xx_run_mute(aw881xx, true);
			aw881xx_run_pwd(aw881xx, true);
		}
	}

	return ret;
}

int aw881xx_get_dsp_status(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_WDT, &reg_val);
	if (reg_val)
		ret = 0;

	return ret;
}

int aw881xx_get_dsp_config(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSCTRL, &reg_val);
	if (reg_val & AW881XX_BIT_SYSCTRL_DSP_BYPASS)
		aw881xx->dsp_cfg = AW881XX_DSP_BYPASS;
	else
		aw881xx->dsp_cfg = AW881XX_DSP_WORK;

	return ret;
}

int aw881xx_get_hmute(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_PWMCTRL, &reg_val);
	if (reg_val & AW881XX_BIT_PWMCTRL_HMUTE_ENABLE)
		ret = 1;
	else
		ret = 0;

	return ret;
}

static int aw881xx_get_icalk(struct aw881xx *aw881xx, int16_t *icalk)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t reg_icalk = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_EFRM, &reg_val);
	reg_icalk = (uint16_t)reg_val & AW881XX_EF_ISENSE_GAINERR_SLP_MASK;

	if (reg_icalk & AW881XX_EF_ISENSE_GAINERR_SLP_SIGN_MASK)
		reg_icalk = reg_icalk | AW881XX_EF_ISENSE_GAINERR_SLP_NEG;

	*icalk = (int16_t)reg_icalk;

	return ret;
}

static int aw881xx_get_vcalk(struct aw881xx *aw881xx, int16_t *vcalk)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t reg_vcalk = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_PRODUCT_ID, &reg_val);
	reg_val = reg_val >> AW881XX_EF_VSENSE_GAIN_SHIFT;

	reg_vcalk = (uint16_t)reg_val & AW881XX_EF_VSENSE_GAIN_MASK;

	if (reg_vcalk & AW881XX_EF_VSENSE_GAIN_SIGN_MASK)
		reg_vcalk = reg_vcalk | AW881XX_EF_VSENSE_GAIN_NEG;

	*vcalk = (int16_t)reg_vcalk;

	return ret;
}

static int aw881xx_dsp_set_vcalb(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;
	int vcalb;
	int icalk;
	int vcalk;
	int16_t icalk_val = 0;
	int16_t vcalk_val = 0;
	uint16_t vcalb_adj;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_VCALB, &vcalb_adj);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:get vcalb_adj failed\n", __func__);
		return ret;
	}

	if (vcalb_adj < AW881XX_VCALB_ADJ_MIN) {
		aw_dev_err(aw881xx->dev, "%s:vcalb_adj 0x%04x is less than the minimum 0x019a\n",
					__func__, vcalb_adj);
		return -EINVAL;
	} else {
		aw_dev_info(aw881xx->dev, "%s:vcalb_adj = 0x%04x\n", __func__, vcalb_adj);
	}

	ret = aw881xx_get_icalk(aw881xx, &icalk_val);
	if (ret < 0)
		return ret;
	ret = aw881xx_get_vcalk(aw881xx, &vcalk_val);
	if (ret < 0)
		return ret;

	icalk = AW881XX_CABL_BASE_VALUE + AW881XX_ICABLK_FACTOR * icalk_val;
	vcalk = AW881XX_CABL_BASE_VALUE + AW881XX_VCABLK_FACTOR * vcalk_val;

	if (icalk == 0 || vcalk == 0) {
		aw_dev_err(aw881xx->dev, "%s: icalk=%d, vcalk=%d is error\n",
			__func__, icalk, vcalk);
		return -EINVAL;
	}

	vcalb = (AW881XX_VCAL_FACTOR * AW881XX_VSCAL_FACTOR /
			AW881XX_ISCAL_FACTOR) * vcalk / icalk * vcalb_adj;

	vcalb = vcalb >> AW881XX_VCALB_ADJ_FACTOR;

	reg_val = (uint16_t) vcalb;
	aw_dev_info(aw881xx->dev, "%s: icalk=%d, vcalk=%d, vcalb=%d, reg_val=0x%04x\n",
			__func__, icalk, vcalk, vcalb, reg_val);

	ret = aw881xx_dsp_write(aw881xx, AW881XX_DSP_REG_VCALB, reg_val);

	return ret;
}

static int aw881xx_set_intmask(struct aw881xx *aw881xx, bool flag)
{
	int ret = -1;

	if (flag)
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM,
					aw881xx->intmask);
	else
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM,
					AW881XX_REG_SYSINTM_MASK);
	return ret;
}

/******************************************************
 *
 * aw881xx dsp
 *
 ******************************************************/
static int aw881xx_get_vmax(struct aw881xx *aw881xx, uint16_t *vmax_val)
{
	int ret;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_VMAX, vmax_val);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw881xx_dsp_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t dsp_check_max = 5;
	uint16_t i = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	for (i = 0; i < dsp_check_max; i++) {
		if (aw881xx->dsp_cfg == AW881XX_DSP_WORK) {
			aw881xx_dsp_enable(aw881xx, false);
			aw881xx_dsp_enable(aw881xx, true);
			usleep_range(AW_2000_US, AW_2000_US + 10);
			ret = aw881xx_get_dsp_status(aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev,
						"%s: dsp wdt status error=%d\n",
						__func__, ret);
			} else {
				return 0;
			}
		} else if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
			return 0;
		} else {
			aw_dev_err(aw881xx->dev, "%s: unknown dsp cfg=%d\n",
					__func__, aw881xx->dsp_cfg);
			return -EINVAL;
		}
	}
	aw881xx_reg_dump(aw881xx);
	return ret;
}

static int aw881xx_dsp_check_process(struct aw881xx *aw881xx)
{
	int i;
	int ret = -1;

	for (i = 0; i < AW881XX_DSP_CHECK_MAX; i++) {
		ret = aw881xx_dsp_check(aw881xx);
		if (ret < 0) {
			if (i == (AW881XX_DSP_CHECK_MAX - 1))
				break;

			ret = aw881xx_fw_update(aw881xx, true);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "%s:fw update failed\n",
					__func__);
				return ret;
			}
		} else {
			aw_dev_info(aw881xx->dev, "%s: dsp check pass\n",
				__func__);
			return 0;
		}
	}

	return -EINVAL;
}

static void aw881xx_dsp_update_cali_re(struct aw881xx *aw881xx)
{
	if (aw881xx->cali_desc.re != AW_ERRO_CALI_VALUE) {
		aw881xx_set_cali_re_to_dsp(&aw881xx->cali_desc);
	} else {
		aw_dev_info(aw881xx->dev, "%s: no set re=%d\n",
			__func__, aw881xx->cali_desc.re);
	}
}

/******************************************************
 *
 * aw881xx reg config
 *
 ******************************************************/
static void aw881xx_reg_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len)
{
	int i;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	uint16_t read_val;

	aw_dev_dbg(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < len; i += 4) {
		reg_addr = (data[i + 1] << 8) + data[i + 0];
		reg_val = (data[i + 3] << 8) + data[i + 2];

		if (reg_addr == AW881XX_REG_SYSINTM) {
			aw881xx->intmask = reg_val;
			reg_val = AW881XX_REG_SYSINTM_MASK;
		}
		if (reg_addr == AW881XX_REG_PWMCTRL) {
			aw881xx_reg_read(aw881xx, reg_addr, &read_val);
			read_val &= (~AW881XX_BIT_PWMCTRL_HMUTE_MASK);
			reg_val &= AW881XX_BIT_PWMCTRL_HMUTE_MASK;
			reg_val |= read_val;
		}

		aw_dev_dbg(aw881xx->dev, "%s:reg = 0x%02x, val = 0x%04x\n",
				__func__, reg_addr, reg_val);
		aw881xx_reg_write(aw881xx, (uint8_t)reg_addr, (uint16_t)reg_val);
	}

	aw881xx_hold_reg_spin_st(&aw881xx->spin_desc);
	aw881xx_get_volume(aw881xx, &aw881xx->db_offset);
	aw881xx_get_dsp_config(aw881xx);

	aw_dev_dbg(aw881xx->dev, "%s:exit\n", __func__);
}

static int aw881xx_reg_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len)
{

	aw_dev_dbg(aw881xx->dev, "%s:reg len:%d\n", __func__, len);

	if (len && (data != NULL)) {
		aw881xx_reg_container_update(aw881xx, data, len);
	} else {
		aw_dev_err(aw881xx->dev, "%s:reg data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_dsp_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len, uint16_t base)
{
	int i;
#ifdef AW881XX_DSP_I2C_WRITES
	uint32_t tmp_len = 0;
#else
	uint16_t reg_val = 0;
#endif

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);
	mutex_lock(&aw881xx->i2c_lock);
#ifdef AW881XX_DSP_I2C_WRITES
	/* i2c writes */
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, base);

	for (i = 0; i < len; i += AW881XX_MAX_RAM_WRITE_BYTE_SIZE) {
		if ((len - i) < AW881XX_MAX_RAM_WRITE_BYTE_SIZE)
			tmp_len = len - i;
		else
			tmp_len = AW881XX_MAX_RAM_WRITE_BYTE_SIZE;
		aw881xx_i2c_writes(aw881xx, AW881XX_REG_DSPMDAT,
					&data[i], tmp_len);
	}
#else
	/* i2c write */
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, base);
	for (i = 0; i < len; i += 2) {
		reg_val = (data[i] << 8) + data[i + 1];
		aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMDAT, reg_val);
	}
#endif
	mutex_unlock(&aw881xx->i2c_lock);
	aw_dev_dbg(aw881xx->dev, "%s: exit\n", __func__);

	return 0;
}

static int aw881xx_dsp_fw_update(struct aw881xx *aw881xx,
			uint8_t *data, uint32_t len)
{

	aw_dev_dbg(aw881xx->dev, "%s:dsp firmware len:%d\n", __func__, len);

	if (len && (data != NULL)) {
		aw881xx_dsp_container_update(aw881xx, data,
			len, AW881XX_DSP_FW_ADDR);
		aw881xx->dsp_fw_len = len;
	} else {
		aw_dev_err(aw881xx->dev, "%s: dsp firmware data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_dsp_cfg_update(struct aw881xx *aw881xx,
			uint8_t *data, uint32_t len)
{
	int ret;

	aw_dev_dbg(aw881xx->dev, "%s:dsp config len:%d\n",
		__func__, len);

	if (len && (data != NULL)) {
		aw881xx_dsp_container_update(aw881xx, data, len, AW881XX_DSP_CFG_ADDR);
		aw881xx->dsp_cfg_len = len;
		ret = aw881xx_get_vmax(aw881xx, &aw881xx->monitor.vmax_init_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:get vmax failed\n", __func__);
			return ret;
		} else {
			aw_dev_info(aw881xx->dev, "%s:get init vmax:0x%x\n",
					__func__, aw881xx->monitor.vmax_init_val);
		}

		aw881xx_dsp_set_vcalb(aw881xx);
		aw881xx_dsp_update_cali_re(aw881xx);

	} else {
		aw_dev_err(aw881xx->dev, "%s:dsp config data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_fw_update(struct aw881xx *aw881xx, bool up_dsp_fw_en)
{
	int ret = -1;
	struct aw_prof_desc *set_prof_desc = NULL;
	struct aw_sec_data_desc *sec_desc = NULL;
	char *prof_name = NULL;

	if (aw881xx->fw_status == AW881XX_FW_FAILED) {
		aw_dev_err(aw881xx->dev, "%s: fw status[%d] error\n",
			__func__, aw881xx->fw_status);
		return -EPERM;
	}

	prof_name = aw881xx_get_profile_name(aw881xx, aw881xx->set_prof);
	if (prof_name == NULL)
		return -ENOMEM;

	ret = aw881xx_get_profile_data(aw881xx, aw881xx->set_prof, &set_prof_desc);
	if (ret < 0)
		return ret;

	aw_dev_info(aw881xx->dev, "%s:start update %s\n", __func__, prof_name);

	sec_desc = set_prof_desc->sec_desc;
	ret = aw881xx_reg_update(aw881xx, sec_desc[AW_DATA_TYPE_REG].data,
					sec_desc[AW_DATA_TYPE_REG].len);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:update reg failed\n", __func__);
		return ret;
	}

	aw881xx_run_mute(aw881xx, true);
	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_OSC);

	if (up_dsp_fw_en) {
		ret = aw881xx_dsp_fw_update(aw881xx, sec_desc[AW_DATA_TYPE_DSP_FW].data,
					sec_desc[AW_DATA_TYPE_DSP_FW].len);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:update dsp fw failed\n", __func__);
			return ret;
		}
	}

	ret = aw881xx_dsp_cfg_update(aw881xx, sec_desc[AW_DATA_TYPE_DSP_CFG].data,
					sec_desc[AW_DATA_TYPE_DSP_CFG].len);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:update dsp cfg failed\n", __func__);
		return ret;
	}

	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_PLL);

	aw881xx->cur_prof = aw881xx->set_prof;

	aw_dev_info(aw881xx->dev, "%s:load %s done\n", __func__, prof_name);
	return 0;
}

static int aw881xx_device_start(struct aw881xx *aw881xx)
{
	int ret = -1;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	if (aw881xx->status == AW881XX_PW_ON) {
		aw_dev_info(aw881xx->dev, "%s: already power on\n", __func__);
		return 0;
	}

	aw881xx_run_pwd(aw881xx, false);
	usleep_range(AW_2000_US, AW_2000_US + 100);

	ret = aw881xx_syspll_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: pll check failed cannot start\n", __func__);
		aw881xx_reg_dump(aw881xx);
		goto pll_check_fail;
	}

	/*amppd on*/
	aw881xx_run_amppd(aw881xx, false);
	usleep_range(AW_1000_US, AW_1000_US + 50);

	/*check i2s status*/
	ret = aw881xx_sysst_check(aw881xx);
	if (ret < 0) {				/*check failed*/
		aw_dev_err(aw881xx->dev, "%s: sysst check fail\n", __func__);
		goto sysst_check_fail;
	}

	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_dsp_update_cali_re(aw881xx);

	ret = aw881xx_dsp_check_process(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp check fail\n", __func__);
		goto dsp_check_fail;
	}

	aw881xx_i2s_tx_enable(aw881xx, true);

	/*clear inturrupt*/
	aw881xx_clear_int_status(aw881xx);
	/*set inturrupt mask*/
	aw881xx_set_intmask(aw881xx, true);
	/*close mute*/
	aw881xx_run_mute(aw881xx, false);

	aw881xx_monitor_start(&aw881xx->monitor);

	aw881xx->status = AW881XX_PW_ON;

	aw_dev_info(aw881xx->dev, "%s: done\n", __func__);

	return 0;


dsp_check_fail:
sysst_check_fail:
	aw881xx_reg_dump(aw881xx);
	aw881xx_run_amppd(aw881xx, true);
pll_check_fail:
	aw881xx_run_pwd(aw881xx, true);
	/*clear inturrupt*/
	aw881xx_clear_int_status(aw881xx);
	aw881xx->status = AW881XX_PW_OFF;
	return ret;
}

static void aw881xx_software_reset(struct aw881xx *aw881xx)
{
	uint8_t reg_addr = AW881XX_REG_ID;
	uint16_t reg_val = AW881XX_SOFTWARE_RST_VALUE;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_write(aw881xx, reg_addr, reg_val);
}

static int aw881xx_phase_sync(struct aw881xx *aw881xx, bool sync_enable)
{
	struct aw_prof_desc *prof_desc = NULL;

	if (aw881xx->fw_status == AW881XX_FW_FAILED) {
		aw_dev_err(aw881xx->dev, "%s:dev acf load failed\n", __func__);
		return -EPERM;
	}

	prof_desc = &aw881xx->prof_info.prof_desc[aw881xx->cur_prof];
	if (sync_enable) {
		if ((prof_desc->sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
			(prof_desc->sec_desc[AW_DATA_TYPE_REG].len != 0)) {
			/*software reset PA*/
			aw881xx_software_reset(aw881xx);

			aw881xx_reg_container_update(aw881xx,
				prof_desc->sec_desc[AW_DATA_TYPE_REG].data,
				prof_desc->sec_desc[AW_DATA_TYPE_REG].len);
		}
	}

	return 0;
}

static void aw881xx_startup_work(struct work_struct *work)
{
	int ret;
	struct aw881xx *aw881xx = container_of(work, struct aw881xx, start_work.work);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	mutex_lock(&aw881xx->lock);
	if (aw881xx->fw_status == AW881XX_FW_OK) {
		if (aw881xx->allow_pw == false) {
			aw_dev_info(aw881xx->dev, "%s:dev can not allow power\n",
				__func__);
			mutex_unlock(&aw881xx->lock);
			return;
		}
		ret = aw881xx_device_start(aw881xx);
		if (ret) {
			aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
		} else {
			aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
		}
	} else {
		aw_dev_err(aw881xx->dev, "%s:dev acf load failed\n", __func__);
	}
	mutex_unlock(&aw881xx->lock);
}

static void aw881xx_start(struct aw881xx *aw881xx)
{
	int ret;

	if (aw881xx->cali_desc.re == AW_ERRO_CALI_VALUE)
		aw881xx_get_cali_re(&aw881xx->cali_desc);

	if (aw881xx->fw_status == AW881XX_FW_OK) {
		if (aw881xx->allow_pw == false) {
			aw_dev_info(aw881xx->dev, "%s:dev can not allow power\n",
				__func__);
			return;
		}

		if (aw881xx->status == AW881XX_PW_ON)
			return;

		if (aw881xx->cur_prof == aw881xx->set_prof) {
			aw881xx_phase_sync(aw881xx, aw881xx->pa_syn_en);
			aw_dev_info(aw881xx->dev, "%s: profile not change\n", __func__);
		} else {
			ret = aw881xx_fw_update(aw881xx, false);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
				return;
			}
		}

		queue_delayed_work(aw881xx->work_queue,
					&aw881xx->start_work, 0);
	} else {
		aw_dev_err(aw881xx->dev, "%s:dev acf load failed\n", __func__);
		return;
	}
}

static void aw881xx_stop(struct aw881xx *aw881xx)
{
	uint8_t reg_addr = AW881XX_REG_SYSCTRL;
	uint16_t reg_data;
	int int_st = 0;
	int monitor_int_st = 0;
	struct aw_sec_data_desc *dsp_cfg =
		&aw881xx->prof_info.prof_desc[aw881xx->cur_prof].sec_desc[AW_DATA_TYPE_DSP_CFG];
	struct aw_sec_data_desc *dsp_fw =
		&aw881xx->prof_info.prof_desc[aw881xx->cur_prof].sec_desc[AW_DATA_TYPE_DSP_FW];

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, reg_addr, &reg_data);

	if ((aw881xx->status == AW881XX_PW_OFF) &&
		(reg_data & AW881XX_BIT_SYSCTRL_PW_PDN)) {
		aw_dev_dbg(aw881xx->dev, "%s:already power off\n", __func__);
		return;
	}

	aw881xx->status = AW881XX_PW_OFF;

	aw881xx_monitor_stop(&aw881xx->monitor);

	aw881xx_run_mute(aw881xx, true);
	usleep_range(AW_4000_US, AW_4000_US + 100);

	aw881xx_i2s_tx_enable(aw881xx, false);
	usleep_range(AW_1000_US, AW_1000_US + 100);

	aw881xx_set_intmask(aw881xx, false);

	int_st = aw881xx_sysint_check(aw881xx);

	aw881xx_dsp_enable(aw881xx, false);

	/*enable amppd*/
	aw881xx_run_amppd(aw881xx, true);

	/*check monitor process sysint state*/
	monitor_int_st = aw881xx_get_monitor_sysint_st(aw881xx);

	/*system status anomaly*/
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_OSC);

	if (int_st < 0 || monitor_int_st < 0) {
		aw881xx_dsp_fw_update(aw881xx, dsp_fw->data, dsp_fw->len);
	}

	aw881xx_dsp_cfg_update(aw881xx, dsp_cfg->data, dsp_cfg->len);
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_PLL);

	aw881xx_run_pwd(aw881xx, true);
}

static int aw881xx_prof_update(struct aw881xx *aw881xx)
{
	int ret;

	if (aw881xx->allow_pw == false) {
		aw_dev_info(aw881xx->dev, "%s:dev can not allow power\n",
			__func__);
		return 0;
	}

	if (aw881xx->status == AW881XX_PW_ON)
		aw881xx_stop(aw881xx);

	ret = aw881xx_fw_update(aw881xx, false);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
		return ret;
	}

	ret = aw881xx_device_start(aw881xx);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
		return ret;
	} else {
		aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
	}

	aw_dev_info(aw881xx->dev, "%s:update done!\n", __func__);
	return 0;
}

/******************************************************
 *
 * kcontrol
 *
 ******************************************************/
static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 50, 0);

struct soc_mixer_control aw881xx_mixer = {
	.reg = AW881XX_REG_HAGCCFG6,
	.shift = AW881XX_VOL_REG_SHIFT,
	.min = AW881XX_VOLUME_MIN,
	.max = AW881XX_VOLUME_MAX,
};

static int aw881xx_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* set kcontrol info */
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mc->max - mc->min;

	return 0;
}

static int aw881xx_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	aw881xx_get_volume(aw881xx, &value);

	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int aw881xx_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint32_t value = 0;

	/* value is right */
	value = ucontrol->value.integer.value[0];
	if (value > (mc->max - mc->min)) {
		aw_dev_err(aw881xx->dev, "%s:value over range\n", __func__);
		return -EINVAL;
	}

	aw881xx_set_volume(aw881xx, value);

	return 0;
}

static struct snd_kcontrol_new aw881xx_volume = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "aw_dev_0_rx_volume",
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
		SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.tlv.p = (digital_gain),
	.info = aw881xx_volume_info,
	.get = aw881xx_volume_get,
	.put = aw881xx_volume_put,
	.private_value = (unsigned long)&aw881xx_mixer,
};

static int aw881xx_dynamic_create_volume_control(struct aw881xx *aw881xx)
{
	struct snd_kcontrol_new *dst_control = NULL;
	char temp_buf[64] = { 0 };

	dst_control = devm_kzalloc(aw881xx->dev, sizeof(struct snd_kcontrol_new),
				GFP_KERNEL);
	if (dst_control == NULL) {
		aw_dev_err(aw881xx->dev, "%s:devm_kzalloc faild\n", __func__);
		return -ENOMEM;
	}

	memcpy(dst_control, &aw881xx_volume, sizeof(struct snd_kcontrol_new));

	snprintf(temp_buf, sizeof(temp_buf),
		"aw_dev_%d_rx_volume", aw881xx->channel);
	dst_control->name = aw881xx_devm_kstrdup(aw881xx->dev, temp_buf);
	if (dst_control->name == NULL)
		return -ENOMEM;

	aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
							dst_control, 1);
	return 0;
}

static int aw881xx_profile_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count;
	char *name = NULL;
	const char *prof_name = NULL;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	count = aw881xx_get_profile_count(aw881xx);
	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		aw_dev_err(aw881xx->dev, "%s:get count[%d] failed\n", __func__, count);
		return 0;
	}

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count) {
		uinfo->value.enumerated.item = count - 1;
	}

	name = uinfo->value.enumerated.name;
	count = uinfo->value.enumerated.item;

	prof_name = aw881xx_get_profile_name(aw881xx, count);
	if (prof_name == NULL) {
		strlcpy(uinfo->value.enumerated.name, "null",
			sizeof(uinfo->value.enumerated.name));
		return 0;
	}

	strlcpy(name, prof_name, sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw881xx_profile_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw881xx_get_profile_index(aw881xx);
	aw_dev_dbg(codec->dev, "%s:profile index [%d]\n",
			__func__, aw881xx_get_profile_index(aw881xx));

	return 0;
}

static int aw881xx_profile_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	int ret;

	/* check value valid */
	ret = aw881xx_check_profile_index(aw881xx, ucontrol->value.integer.value[0]);
	if (ret) {
		aw_dev_info(codec->dev, "%s:unsupported index %d\n", __func__,
				(int)ucontrol->value.integer.value[0]);
		return 0;
	}

	if (aw881xx->set_prof == ucontrol->value.integer.value[0]) {
		aw_dev_info(codec->dev, "%s:index no change\n", __func__);
		return 0;
	}

	mutex_lock(&aw881xx->lock);
	aw881xx_set_profile_index(aw881xx, ucontrol->value.integer.value[0]);
	/*pstream = 1 no pcm just set status*/
	if (aw881xx->pstream)
		aw881xx_prof_update(aw881xx);

	mutex_unlock(&aw881xx->lock);
	aw_dev_info(codec->dev, "%s:prof id %d\n", __func__,
			(int)ucontrol->value.integer.value[0]);
	return 0;
}

static int aw881xx_switch_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = ARRAY_SIZE(aw881xx_switch);

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	strlcpy(uinfo->value.enumerated.name,
		aw881xx_switch[uinfo->value.enumerated.item],
		sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw881xx_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw881xx->allow_pw;
	aw_dev_dbg(codec->dev, "%s:allow_pw %d\n",
			__func__, aw881xx->allow_pw);
	return 0;

}

static int aw881xx_switch_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	int ret;

	aw_dev_dbg(codec->dev, "%s:set %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	if ((ucontrol->value.integer.value[0] == 0) &&
		(aw881xx->allow_pw == false)) {
		aw_dev_info(codec->dev, "%s:no change %ld\n",
			__func__, ucontrol->value.integer.value[0]);
		return 0;
	}

	if (ucontrol->value.integer.value[0] &&
		(aw881xx->allow_pw == true)) {
		aw_dev_info(codec->dev, "%s:no change %ld\n",
			__func__, ucontrol->value.integer.value[0]);
		return 0;
	}

	if (aw881xx->pstream) {
		if (ucontrol->value.integer.value[0] == 0) {
			/* vivo audio dxl add for dump reg start */
			aw881xx_reg_dump_vivo(aw881xx);
			/* vivo audio dxl add for dump reg end */
			cancel_delayed_work_sync(&aw881xx->start_work);
			mutex_lock(&aw881xx->lock);
			aw881xx_stop(aw881xx);
			aw881xx->allow_pw = false;
			mutex_unlock(&aw881xx->lock);
			aw_dev_info(aw881xx->dev, "%s:stop pa\n", __func__);
		} else {
			/*if stream have open ,PA can power on*/
			cancel_delayed_work_sync(&aw881xx->start_work);
			if (aw881xx->fw_status == AW881XX_FW_OK) {
				mutex_lock(&aw881xx->lock);
				aw881xx->allow_pw = true;

				if (aw881xx->cur_prof != aw881xx->set_prof) {
					ret = aw881xx_fw_update(aw881xx, false);
					if (ret < 0) {
						aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
						mutex_unlock(&aw881xx->lock);
						return ret;
					}
				}
				ret = aw881xx_device_start(aw881xx);
				if (ret < 0)
					aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
				else
					aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
				mutex_unlock(&aw881xx->lock);
			}
			/* vivo audio dxl add for dump reg start */
			aw881xx_reg_dump_vivo(aw881xx);
			/* vivo audio dxl add for dump reg end */
		}
	} else {
		mutex_lock(&aw881xx->lock);
		if (ucontrol->value.integer.value[0]) {
			aw881xx->allow_pw = true;
		} else {
			aw881xx->allow_pw = false;
		}
		mutex_unlock(&aw881xx->lock);
	}

	return 0;
}

static int aw881xx_monitor_switch_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int count;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = 2;

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	strlcpy(uinfo->value.enumerated.name,
		aw881xx_switch[uinfo->value.enumerated.item],
		strlen(aw881xx_switch[uinfo->value.enumerated.item]) + 1);

	return 0;
}

static int aw881xx_monitor_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	struct aw881xx_monitor *monitor_desc = &aw881xx->monitor;

	ucontrol->value.integer.value[0] = monitor_desc->monitor_cfg.monitor_switch;

	return 0;
}

static int aw881xx_monitor_switch_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	struct aw881xx_monitor *monitor_desc = &aw881xx->monitor;
	uint32_t enable = 0;

	aw_dev_info(codec->dev, "set monitor_switch:%ld", ucontrol->value.integer.value[0]);

	enable = ucontrol->value.integer.value[0];

	if (monitor_desc->monitor_cfg.monitor_switch == enable) {
		aw_dev_info(aw881xx->dev, "monitor_switch not change");
		return 0;
	} else {
		monitor_desc->monitor_cfg.monitor_switch = enable;
		if (enable)
			aw881xx_monitor_start(monitor_desc);
	}

	return 0;
}

static int aw881xx_dynamic_create_common_controls(struct aw881xx *aw881xx)
{
	struct snd_kcontrol_new *aw881xx_dev_control = NULL;
	char *kctl_name = NULL;

	aw881xx_dev_control = devm_kzalloc(aw881xx->codec->dev,
				sizeof(struct snd_kcontrol_new) * AW_CONTROL_NUM,
				GFP_KERNEL);
	if (aw881xx_dev_control == NULL) {
		aw_dev_err(aw881xx->dev, "%s: kcontrol malloc failed!\n", __func__);
		return -ENOMEM;
	}

	kctl_name = devm_kzalloc(aw881xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_prof", aw881xx->channel);

	aw881xx_dev_control[0].name = kctl_name;
	aw881xx_dev_control[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw881xx_dev_control[0].info = aw881xx_profile_info;
	aw881xx_dev_control[0].get = aw881xx_profile_get;
	aw881xx_dev_control[0].put = aw881xx_profile_set;

	kctl_name = devm_kzalloc(aw881xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_switch", aw881xx->channel);

	aw881xx_dev_control[1].name = kctl_name;
	aw881xx_dev_control[1].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw881xx_dev_control[1].info = aw881xx_switch_info;
	aw881xx_dev_control[1].get = aw881xx_switch_get;
	aw881xx_dev_control[1].put = aw881xx_switch_set;

	kctl_name = devm_kzalloc(aw881xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;
	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%u_monitor_switch", aw881xx->channel);

	aw881xx_dev_control[2].name = kctl_name;
	aw881xx_dev_control[2].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw881xx_dev_control[2].info = aw881xx_monitor_switch_info;
	aw881xx_dev_control[2].get = aw881xx_monitor_switch_get;
	aw881xx_dev_control[2].put = aw881xx_monitor_switch_set;

	aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
					aw881xx_dev_control, AW_CONTROL_NUM);

	return 0;
}

static int aw881xx_dynamic_create_controls(struct aw881xx *aw881xx)
{
	int ret;

	ret = aw881xx_dynamic_create_common_controls(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: create common control failed!\n", __func__);
		return ret;
	}

	ret = aw881xx_dynamic_create_volume_control(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: create volume control failed!\n", __func__);
		return ret;
	}

	return 0;
}

static int aw881xx_get_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_fade_in_time;

	pr_debug("%s:step time %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	return 0;

}

static int aw881xx_set_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0] > mc->max) {
		pr_debug("%s:set val %ld overflow %d", __func__,
			ucontrol->value.integer.value[0], mc->max);
		return 0;
	}

	g_fade_in_time = ucontrol->value.integer.value[0];

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int aw881xx_get_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_fade_out_time;

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

static int aw881xx_set_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0] > mc->max) {
		pr_debug("%s:set val %ld overflow %d\n", __func__,
			ucontrol->value.integer.value[0], mc->max);
		return 0;
	}

	g_fade_out_time = ucontrol->value.integer.value[0];

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}


static struct snd_kcontrol_new aw881xx_controls[] = {
	SOC_SINGLE_EXT("aw881xx_fadein_us", 0, 0, 1000000, 0,
		aw881xx_get_fade_in_time, aw881xx_set_fade_in_time),
	SOC_SINGLE_EXT("aw881xx_fadeout_us", 0, 0, 1000000, 0,
		aw881xx_get_fade_out_time, aw881xx_set_fade_out_time),
};

static void aw881xx_add_codec_controls(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	if (aw881xx->channel == 0) {
		aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
					aw881xx_controls, ARRAY_SIZE(aw881xx_controls));
		aw881xx_add_spin_controls((void *)aw881xx);
	}
}

/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static int aw881xx_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	return 0;
}

static int aw881xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);

	aw_dev_info(codec->dev, "%s: fmt=0x%x\n", __func__, fmt);

	return 0;
}

static int aw881xx_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: freq=%d\n", __func__, freq);

	aw881xx->sysclk = freq;
	return 0;
}

static int aw881xx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_dbg(aw881xx->dev, "%s:stream capture requested rate: %d, sample size: %d",
				__func__, params_rate(params), params_width(params));
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw_dev_dbg(aw881xx->dev, "%s:stream playback requested rate: %d, sample size: %d",
				__func__, params_rate(params), params_width(params));
	}

	return 0;
}

static int aw881xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: mute state=%d\n", __func__, mute);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK) {
		aw_dev_info(aw881xx->dev, "%s:capture\n", __func__);
		return 0;
	}

	if (!(aw881xx->flags & AW881XX_FLAG_START_ON_MUTE))
		return 0;

	if (mute) {
		aw881xx->pstream = AW881XX_AUDIO_STOP;
		cancel_delayed_work_sync(&aw881xx->start_work);
		mutex_lock(&aw881xx->lock);
		aw881xx_stop(aw881xx);
		mutex_unlock(&aw881xx->lock);
	} else {
		aw881xx->pstream = AW881XX_AUDIO_START;
		mutex_lock(&aw881xx->lock);
		aw881xx_start(aw881xx);
		aw881xx_hold_dsp_spin_st(&aw881xx->spin_desc);
		mutex_unlock(&aw881xx->lock);
	}

	return 0;
}

static void aw881xx_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);

	aw_dev_info(codec->dev, "%s:enter\n", __func__);
}

static const struct snd_soc_dai_ops aw881xx_dai_ops = {
	.startup = aw881xx_startup,
	.set_fmt = aw881xx_set_fmt,
	.set_sysclk = aw881xx_set_dai_sysclk,
	.hw_params = aw881xx_hw_params,
	.mute_stream = aw881xx_mute,
	.shutdown = aw881xx_shutdown,
};

static struct snd_soc_dai_driver aw881xx_dai[] = {
	{
	.name = "aw881xx-aif",
	.id = 1,
	.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW881XX_RATES,
			.formats = AW881XX_FORMATS,
			},
	 .capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW881XX_RATES,
			.formats = AW881XX_FORMATS,
			},
	.ops = &aw881xx_dai_ops,
	.symmetric_rates = 1,
	},
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static uint8_t aw881xx_crc8_check(unsigned char *data , uint32_t data_size)
{
	uint8_t crc_value = 0x00;
	uint8_t pdatabuf = 0;
	int i;

	while (data_size--) {
		pdatabuf = *data++;
		for (i = 0; i < 8; i++) {
			/*if the lowest bit is 1*/
			if ((crc_value ^ (pdatabuf)) & 0x01) {
				/*Xor multinomial*/
				crc_value ^= 0x18;
				crc_value >>= 1;
				crc_value |= 0x80;
			} else {
				crc_value >>= 1;
			}
			pdatabuf >>= 1;
		}
	}
	return crc_value;
}

static int aw881xx_check_cfg_by_hdr(struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;
	struct aw_cfg_dde *cfg_dde = NULL;
	unsigned int end_data_offset = 0;
	unsigned int act_data = 0;
	uint8_t act_crc8 = 0;
	uint32_t hdr_ddt_len = 0;
	int i;

	if (aw_cfg == NULL) {
		pr_err("%s:aw_prof is NULL\n", __func__);
		return -ENOMEM;
	}

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;

	/*check file type id is awinic acf file*/
	if (cfg_hdr->a_id != ACF_FILE_ID) {
		pr_err("%s:not acf type file\n", __func__);
		return -EINVAL;
	}

	hdr_ddt_len = cfg_hdr->a_hdr_offset + cfg_hdr->a_ddt_size;
	if (hdr_ddt_len > aw_cfg->len) {
		pr_err("%s:hdrlen with ddt_len [%d] overflow file size[%d]\n",
			__func__, cfg_hdr->a_hdr_offset, aw_cfg->len);
		return -EINVAL;
	}

	/*check data size*/
	cfg_dde = (struct aw_cfg_dde *)((char *)aw_cfg->data + cfg_hdr->a_hdr_offset);
	act_data += hdr_ddt_len;
	for (i = 0; i < cfg_hdr->a_ddt_num; i++)
		act_data += cfg_dde[i].data_size;

	if (act_data != aw_cfg->len) {
		pr_err("%s:act_data[%d] not equal to file size[%d]!\n",
			__func__, act_data, aw_cfg->len);
		return -EINVAL;
	}

	for (i = 0; i < cfg_hdr->a_ddt_num; i++) {
		/* data check */
		end_data_offset = cfg_dde[i].data_offset + cfg_dde[i].data_size;
		if (end_data_offset > aw_cfg->len) {
			pr_err("%s:a_ddt_num[%d] end_data_offset[%d] overflow file size[%d]\n",
				__func__, i, end_data_offset, aw_cfg->len);
			return -EINVAL;
		}

		/* crc check */
		act_crc8 = aw881xx_crc8_check(aw_cfg->data + cfg_dde[i].data_offset, cfg_dde[i].data_size);
		if (act_crc8 != cfg_dde[i].data_crc) {
			pr_err("%s:a_ddt_num[%d] crc8 check failed, act_crc8:0x%x != data_crc 0x%x\n",
				__func__, i, (uint32_t)act_crc8, cfg_dde[i].data_crc);
			return -EINVAL;
		}
	}

	pr_info("%s:project name [%s]\n", __func__, cfg_hdr->a_project);
	pr_info("%s:custom name [%s]\n", __func__, cfg_hdr->a_custom);
	pr_info("%s:version name [%d.%d.%d.%d]\n", __func__,
			cfg_hdr->a_version[3], cfg_hdr->a_version[2],
			cfg_hdr->a_version[1], cfg_hdr->a_version[0]);
	pr_info("%s:author id %d\n", __func__, cfg_hdr->a_author_id);

	return 0;
}

static int aw881xx_check_acf_by_hdr_v_1_0_0_0(struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde = NULL;
	unsigned int end_data_offset = 0;
	unsigned int act_data = 0;
	uint8_t act_crc8 = 0;
	uint32_t hdr_ddt_len = 0;
	int i;

	if (aw_cfg == NULL) {
		pr_err("%s:aw_prof is NULL\n", __func__);
		return -ENOMEM;
	}

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;

	/*check file type id is awinic acf file*/
	if (cfg_hdr->a_id != ACF_FILE_ID) {
		pr_err("%s:not acf type file\n", __func__);
		return -EINVAL;
	}

	hdr_ddt_len = cfg_hdr->a_hdr_offset + cfg_hdr->a_ddt_size;
	if (hdr_ddt_len > aw_cfg->len) {
		pr_err("%s:hdrlen with ddt_len [%d] overflow file size[%d]\n",
			__func__, cfg_hdr->a_hdr_offset, aw_cfg->len);
		return -EINVAL;
	}

	/*check data size*/
	cfg_dde = (struct aw_cfg_dde_v_1_0_0_0 *)((char *)aw_cfg->data + cfg_hdr->a_hdr_offset);
	act_data += hdr_ddt_len;
	for (i = 0; i < cfg_hdr->a_ddt_num; i++)
		act_data += cfg_dde[i].data_size;

	if (act_data != aw_cfg->len) {
		pr_err("%s:act_data[%d] not equal to file size[%d]!\n",
			__func__, act_data, aw_cfg->len);
		return -EINVAL;
	}

	for (i = 0; i < cfg_hdr->a_ddt_num; i++) {
		/* data check */
		end_data_offset = cfg_dde[i].data_offset + cfg_dde[i].data_size;
		if (end_data_offset > aw_cfg->len) {
			pr_err("%s:a_ddt_num[%d] end_data_offset[%d] overflow file size[%d]\n",
				__func__, i, end_data_offset, aw_cfg->len);
			return -EINVAL;
		}

		/* crc check */
		act_crc8 = aw881xx_crc8_check(aw_cfg->data + cfg_dde[i].data_offset, cfg_dde[i].data_size);
		if (act_crc8 != cfg_dde[i].data_crc) {
			pr_err("%s:a_ddt_num[%d] crc8 check failed, act_crc8:0x%x != data_crc 0x%x\n",
				__func__, i, (uint32_t)act_crc8, cfg_dde[i].data_crc);
			return -EINVAL;
		}
	}

	pr_info("%s:project name [%s]\n", __func__, cfg_hdr->a_project);
	pr_info("%s:custom name [%s]\n", __func__, cfg_hdr->a_custom);
	pr_info("%s:version name [%d.%d.%d.%d]\n", __func__,
			cfg_hdr->a_version[3], cfg_hdr->a_version[2],
			cfg_hdr->a_version[1], cfg_hdr->a_version[0]);
	pr_info("%s:author id %d\n", __func__, cfg_hdr->a_author_id);

	return 0;
}

int aw881xx_load_acf_check(struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;

	if (aw_cfg == NULL) {
		pr_err("%s: aw_prof is NULL\n", __func__);
		return -ENOMEM;
	}

	if (aw_cfg->len < sizeof(struct aw_cfg_hdr)) {
		pr_err("%s:cfg hdr size[%d] overflow file size[%d]",
			__func__, (int)sizeof(struct aw_cfg_hdr), aw_cfg->len);
		return -EINVAL;
	}

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	switch (cfg_hdr->a_hdr_version) {
	case AW_CFG_HDR_VER_0_0_0_1:
		return aw881xx_check_cfg_by_hdr(aw_cfg);
	case AW_CFG_HDR_VER_1_0_0_0:
		return aw881xx_check_acf_by_hdr_v_1_0_0_0(aw_cfg);
	default:
		pr_err("%s: unsupported hdr_version [0x%x]\n",
				__func__, cfg_hdr->a_hdr_version);
		return -EINVAL;
	}

	return 0;
}

static void aw881xx_update_dsp_data_order(struct aw881xx *aw881xx,
						uint8_t *data, uint32_t data_len)
{
	int i = 0;
	uint8_t tmp_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	for (i = 0; i < data_len; i += 2) {
		tmp_val = data[i];
		data[i] = data[i + 1];
		data[i + 1] = tmp_val;
	}
}

static int aw881xx_parse_raw_reg(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_REG].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_REG].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_raw_dsp_cfg(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	aw881xx_update_dsp_data_order(aw881xx, data, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_DSP_CFG].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_DSP_CFG].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_raw_dsp_fw(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	aw881xx_update_dsp_data_order(aw881xx, data, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_DSP_FW].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_DSP_FW].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_data_by_sec_type(struct aw881xx *aw881xx, struct aw_cfg_hdr *cfg_hdr,
			struct aw_cfg_dde *cfg_dde, struct aw_prof_desc *scene_prof_desc)
{
	switch (cfg_dde->data_type) {
	case ACF_SEC_TYPE_REG:
		return aw881xx_parse_raw_reg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_CFG:
		return aw881xx_parse_raw_dsp_cfg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_FW:
		return aw881xx_parse_raw_dsp_fw(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	}

	return 0;
}

static int aw881xx_parse_dev_type(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int i = 0;
	int ret;
	int sec_num = 0;
	struct aw_cfg_dde *cfg_dde =
		(struct aw_cfg_dde *)((char *)prof_hdr + prof_hdr->a_hdr_offset);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
			(aw881xx->i2c->addr == cfg_dde[i].dev_addr) &&
			(cfg_dde[i].type == AW_DEV_TYPE_ID)) {
			if (cfg_dde[i].data_type == ACF_SEC_TYPE_MONITOR) {
				ret = aw881xx_monitor_parse_fw(&aw881xx->monitor,
					(uint8_t *)prof_hdr + cfg_dde[i].data_offset,
					cfg_dde[i].data_size);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s: parse monitor failed\n", __func__);
					return ret;
				}
			} else {
				if (cfg_dde[i].dev_profile >= AW_PROFILE_MAX) {
					aw_dev_err(aw881xx->dev, "%s: dev_profile [%d] overflow\n",
						__func__, cfg_dde[i].dev_profile);
					return -EINVAL;
				}
				ret = aw881xx_parse_data_by_sec_type(aw881xx, prof_hdr, &cfg_dde[i],
					&all_prof_info->prof_desc[cfg_dde[i].dev_profile]);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:parse dev type failed\n", __func__);
					return ret;
				}
				sec_num++;
			}
		}
	}

	if (sec_num == 0) {
		aw_dev_info(aw881xx->dev, "%s:get dev type num is %d, please use default\n",
					__func__, sec_num);
		return AW_DEV_TYPE_NONE;
	}

	return AW_DEV_TYPE_OK;
}

static int aw881xx_parse_dev_default_type(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int i = 0;
	int ret;
	int sec_num = 0;
	struct aw_cfg_dde *cfg_dde =
		(struct aw_cfg_dde *)((char *)prof_hdr + prof_hdr->a_hdr_offset);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((aw881xx->channel == cfg_dde[i].dev_index) &&
			(cfg_dde[i].type == AW_DEV_DEFAULT_TYPE_ID)) {
			if (cfg_dde[i].data_type == ACF_SEC_TYPE_MONITOR) {
				ret = aw881xx_monitor_parse_fw(&aw881xx->monitor,
					(uint8_t *)prof_hdr + cfg_dde[i].data_offset,
					cfg_dde[i].data_size);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s: parse monitor failed\n", __func__);
					return ret;
				}
			} else {
				if (cfg_dde[i].dev_profile >= AW_PROFILE_MAX) {
					aw_dev_err(aw881xx->dev, "%s: dev_profile [%d] overflow\n",
						__func__, cfg_dde[i].dev_profile);
					return -EINVAL;
				}
				ret = aw881xx_parse_data_by_sec_type(aw881xx, prof_hdr, &cfg_dde[i],
						&all_prof_info->prof_desc[cfg_dde[i].dev_profile]);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s: parse default type failed\n", __func__);
					return ret;
				}
				sec_num++;
			}
		}
	}

	if (sec_num == 0) {
		aw_dev_err(aw881xx->dev, "%s: get dev default type failed, get num[%d]\n",
					__func__, sec_num);
		return -EINVAL;
	}

	return 0;
}

static int aw881xx_get_vaild_prof(struct aw881xx *aw881xx,
				struct aw_all_prof_info *all_prof_info)
{
	int i;
	int num = 0;
	struct aw_sec_data_desc *sec_desc = NULL;
	struct aw_prof_desc *prof_desc = all_prof_info->prof_desc;
	struct aw_prof_info *prof_info = &aw881xx->prof_info;

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (prof_desc[i].prof_st == AW_PROFILE_OK) {
			sec_desc = prof_desc[i].sec_desc;
			if ((sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_REG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].len != 0)) {
				prof_info->count++;
			}
		}
	}

	aw_dev_info(aw881xx->dev, "%s: get vaild profile:%d\n",
			__func__, aw881xx->prof_info.count);

	if (!prof_info->count) {
		aw_dev_err(aw881xx->dev, "%s:no profile data\n", __func__);
		return -EPERM;
	}

	prof_info->prof_desc = devm_kzalloc(aw881xx->dev,
				prof_info->count * sizeof(struct aw_prof_desc),
				GFP_KERNEL);
	if (prof_info->prof_desc == NULL) {
		aw_dev_err(aw881xx->dev, "%s:prof_desc kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (prof_desc[i].prof_st == AW_PROFILE_OK) {
			sec_desc = prof_desc[i].sec_desc;
			if ((sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_REG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].len != 0)) {
				if (num >= prof_info->count) {
					aw_dev_err(aw881xx->dev, "%s:get scene num[%d] overflow count[%d]\n",
						__func__, num, prof_info->count);
					return -ENOMEM;
				}
				prof_info->prof_desc[num] = prof_desc[i];
				prof_info->prof_desc[num].id = i;
				num++;
			}
		}
	}

	return 0;
}

static int aw881xx_load_cfg_by_hdr(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int ret;

	ret = aw881xx_parse_dev_type(aw881xx, prof_hdr, all_prof_info);
	if (ret < 0) {
		return ret;
	} else if (ret == AW_DEV_TYPE_NONE) {
		aw_dev_info(aw881xx->dev, "%s:get dev type num is 0, parse default dev type\n",
					__func__);
		ret = aw881xx_parse_dev_default_type(aw881xx, prof_hdr, all_prof_info);
		if (ret < 0)
			return ret;
	}

	ret = aw881xx_get_vaild_prof(aw881xx, all_prof_info);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:get vaild profile failed\n", __func__);
		return ret;
	}
	aw881xx->prof_info.prof_name_list = profile_name;
	aw881xx->prof_info.prof_max = AW_PROFILE_MAX;

	return 0;
}

static int aw881xx_get_dde_prof_id(struct aw881xx *aw881xx, int *scene_id,
					struct aw_cfg_dde_v_1_0_0_0 * cfg_dde)
{
	int i;
	int prof_count = aw881xx->prof_info.count;
	struct aw_prof_desc *prof_desc = aw881xx->prof_info.prof_desc;

	for (i = 0; i < prof_count; i++) {
		if (prof_desc[i].prof_id == cfg_dde->dev_profile)
			*scene_id = i;
	}

	return 0;
}

static int aw881xx_parse_data_by_sec_type_v_1_0_0_0(struct aw881xx *aw881xx, struct aw_cfg_hdr *cfg_hdr,
		struct aw_cfg_dde_v_1_0_0_0 *cfg_dde, struct aw_prof_desc *scene_prof_desc)
{
	switch (cfg_dde->data_type) {
	case ACF_SEC_TYPE_REG:
		scene_prof_desc->prof_str = cfg_dde->dev_profile_str;
		return aw881xx_parse_raw_reg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_CFG:
		scene_prof_desc->prof_str = cfg_dde->dev_profile_str;
		return aw881xx_parse_raw_dsp_cfg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_FW:
		scene_prof_desc->prof_str = cfg_dde->dev_profile_str;
		return aw881xx_parse_raw_dsp_fw(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	}

	return 0;
}

static int aw881xx_parse_dev_type_v_1_0_0_0(struct aw881xx *aw881xx, struct aw_cfg_hdr *prof_hdr)
{
	int ret, i = 0;
	int cur_scene_id = 0;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
		(struct aw_cfg_dde_v_1_0_0_0 *)((char *)prof_hdr + prof_hdr->a_hdr_offset);
	struct aw_prof_desc *prof_desc = aw881xx->prof_info.prof_desc;

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
			(aw881xx->i2c->addr == cfg_dde[i].dev_addr) &&
			(cfg_dde[i].type == AW_DEV_TYPE_ID)) {
			if (cfg_dde[i].data_type == ACF_SEC_TYPE_MONITOR) {
				ret = aw881xx_monitor_parse_fw(&aw881xx->monitor,
					(uint8_t *)prof_hdr + cfg_dde[i].data_offset,
					cfg_dde[i].data_size);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:parse monitor failed\n", __func__);
					return ret;
				}
			} else {
				aw881xx_get_dde_prof_id(aw881xx, &cur_scene_id, &cfg_dde[i]);
				ret = aw881xx_parse_data_by_sec_type_v_1_0_0_0(aw881xx, prof_hdr,
					&cfg_dde[i], &prof_desc[cur_scene_id]);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:parse dev type failed\n", __func__);
					return ret;
				}
			}
		}
	}

	aw_dev_info(aw881xx->dev, "%s:get dev type success, scene num[%d]\n",
		__func__, aw881xx->prof_info.count);

	return 0;
}

static int aw881xx_parse_dev_default_type_v_1_0_0_0(struct aw881xx *aw881xx, struct aw_cfg_hdr *prof_hdr)
{
	int i, ret;
	int cur_scene_id = 0;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
		(struct aw_cfg_dde_v_1_0_0_0 *)((char *)prof_hdr + prof_hdr->a_hdr_offset);
	struct aw_prof_desc *prof_desc = aw881xx->prof_info.prof_desc;

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((cfg_dde[i].type == AW_DEV_DEFAULT_TYPE_ID) &&
			(aw881xx->channel == cfg_dde[i].dev_index) &&
			(aw881xx->chipid == cfg_dde[i].chip_id)) {
			if (cfg_dde[i].data_type == ACF_SEC_TYPE_MONITOR) {
				ret = aw881xx_monitor_parse_fw(&aw881xx->monitor,
					(uint8_t *)prof_hdr + cfg_dde[i].data_offset,
					cfg_dde[i].data_size);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:parse monitor failed\n", __func__);
					return ret;
				}
			} else {
				aw881xx_get_dde_prof_id(aw881xx, &cur_scene_id, &cfg_dde[i]);
				ret = aw881xx_parse_data_by_sec_type_v_1_0_0_0(aw881xx, prof_hdr,
								&cfg_dde[i], &prof_desc[cur_scene_id]);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:parse default type failed\n", __func__);
					return ret;
				}
			}
		}
	}

	aw_dev_info(aw881xx->dev, "%s:get dev default type success, scene num[%d]\n",
		__func__, aw881xx->prof_info.count);

	return 0;
}

static bool aw881xx_find_dde_datatype(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg,
					uint16_t dev_profile, uint32_t data_type)
{
	int i, params_num = 0;
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
		(struct aw_cfg_dde_v_1_0_0_0 *)(aw_cfg->data + cfg_hdr->a_hdr_offset);
	int dde_type = aw881xx->prof_info.prof_type;

	if (dde_type == AW_DEV_TYPE_ID) {
		for (i = 0; i < cfg_hdr->a_ddt_num; i++) {
			if ((cfg_dde[i].data_type == data_type) &&
				(aw881xx->chipid == cfg_dde[i].chip_id) &&
				(aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
				(aw881xx->i2c->addr == cfg_dde[i].dev_addr) &&
				(dev_profile == cfg_dde[i].dev_profile))
				params_num++;
		}
	} else if (dde_type == AW_DEV_DEFAULT_TYPE_ID) {
		for (i = 0; i < cfg_hdr->a_ddt_num; i++) {
			if ((cfg_dde[i].data_type == data_type) &&
				(aw881xx->chipid == cfg_dde[i].chip_id) &&
				(aw881xx->channel == cfg_dde[i].dev_index) &&
				(dev_profile == cfg_dde[i].dev_profile))
				params_num++;
		}
	}

	if (params_num == 1)
		return true;
	else
		return false;
}

static bool aw881xx_find_dsp_fw_datatype(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg,
					uint16_t dev_profile)
{
	return aw881xx_find_dde_datatype(aw881xx, aw_cfg, dev_profile, ACF_SEC_TYPE_DSP_FW);
}

static bool aw881xx_find_dsp_cfg_datatype(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg,
					uint16_t dev_profile)
{
	return aw881xx_find_dde_datatype(aw881xx, aw_cfg, dev_profile, ACF_SEC_TYPE_DSP_CFG);
}

static bool aw881xx_find_reg_datatype(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg,
					uint16_t dev_profile)
{
	return aw881xx_find_dde_datatype(aw881xx, aw_cfg, dev_profile, ACF_SEC_TYPE_REG);
}

static int aw881xx_find_dde_datatype_info(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg,
						uint32_t dev_profile)
{
	bool ret = false;

	ret = aw881xx_find_reg_datatype(aw881xx, aw_cfg, dev_profile);
	if (!ret) {
		aw_dev_err(aw881xx->dev,
				"%s:find reg scene params failed\n", __func__);
		return -EINVAL;
	}
	ret = aw881xx_find_dsp_cfg_datatype(aw881xx, aw_cfg, dev_profile);
	if (!ret) {
		aw_dev_err(aw881xx->dev,
				"%s:find dsp_cfg scene params failed\n", __func__);
		return -EINVAL;
	}

	ret = aw881xx_find_dsp_fw_datatype(aw881xx, aw_cfg, dev_profile);
	if (!ret) {
		aw_dev_err(aw881xx->dev,
				"%s:find dsp_fw scene params failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int aw881xx_get_dde_type_info(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg)
{
	int i;
	int dev_num = 0;
	int default_num = 0;
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
		(struct aw_cfg_dde_v_1_0_0_0 *)(aw_cfg->data + cfg_hdr->a_hdr_offset);

	aw881xx->prof_info.prof_type = AW_DEV_NONE_TYPE_ID;
	for (i = 0;i < cfg_hdr->a_ddt_num; i++) {
		if (cfg_dde[i].type == AW_DEV_TYPE_ID)
			dev_num++;

		if (cfg_dde[i].type == AW_DEV_DEFAULT_TYPE_ID)
			default_num++;
	}

	if (!(dev_num || default_num)) {
		aw_dev_err(aw881xx->dev, "%s:can't find scene\n", __func__);
		return -EINVAL;
	}

	if (dev_num != 0) {
		aw881xx->prof_info.prof_type = AW_DEV_TYPE_ID;
	} else if (default_num != 0) {
		aw881xx->prof_info.prof_type = AW_DEV_DEFAULT_TYPE_ID;
	}

	return 0;
}

static int aw881xx_parse_dev_scene_count_v_1_0_0_0(struct aw881xx *aw881xx,
						struct aw881xx_container *aw_cfg, uint32_t *count)
{
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
		struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
			(struct aw_cfg_dde_v_1_0_0_0 *)(aw_cfg->data + cfg_hdr->a_hdr_offset);
	int i, ret = 0;

	for (i =0; i < cfg_hdr->a_ddt_num; ++i) {
		if ((cfg_dde[i].data_type == ACF_SEC_TYPE_REG) &&
			(aw881xx->chipid == cfg_dde[i].chip_id) &&
			(aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
			(aw881xx->i2c->addr == cfg_dde[i].dev_addr)) {
			ret = aw881xx_find_dde_datatype_info(aw881xx, aw_cfg,
							cfg_dde[i].dev_profile);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev,
					"%s:Incomplete or redundant scene params [%s]\n",
					__func__, cfg_dde[i].dev_profile_str);
				return -EINVAL;
			}
			(*count)++;
		}
	}

	return 0;
}

static int aw881xx_parse_default_scene_count_v_1_0_0_0(struct aw881xx *aw881xx,
						struct aw881xx_container *aw_cfg, uint32_t *count)
{
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
		struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
			(struct aw_cfg_dde_v_1_0_0_0 *)(aw_cfg->data + cfg_hdr->a_hdr_offset);
	int i, ret = 0;

	for (i =0; i < cfg_hdr->a_ddt_num; ++i) {
		if ((cfg_dde[i].data_type == ACF_SEC_TYPE_REG) &&
			(aw881xx->chipid == cfg_dde[i].chip_id) &&
			(aw881xx->channel == cfg_dde[i].dev_index)) {
			ret = aw881xx_find_dde_datatype_info(aw881xx, aw_cfg,
							cfg_dde[i].dev_profile);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev,
					"%s:Incomplete or redundant scene params [%s]\n",
					__func__, cfg_dde[i].dev_profile_str);
				return -EINVAL;
			}
			(*count)++;
		}
	}

	return 0;
}

static int aw881xx_parse_scene_count_v_1_0_0_0(struct aw881xx *aw881xx,
							struct aw881xx_container *aw_cfg, uint32_t *count)
{
	int ret = 0;

	ret = aw881xx_get_dde_type_info(aw881xx, aw_cfg);
	if (ret < 0)
		return ret;

	if (aw881xx->prof_info.prof_type == AW_DEV_TYPE_ID) {
		ret = aw881xx_parse_dev_scene_count_v_1_0_0_0(aw881xx, aw_cfg, count);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:scene params error\n", __func__);
			return ret;
		}
	} else if (aw881xx->prof_info.prof_type == AW_DEV_DEFAULT_TYPE_ID) {
		ret = aw881xx_parse_default_scene_count_v_1_0_0_0(aw881xx, aw_cfg, count);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:scene params error\n", __func__);
			return ret;
		}
	} else {
		aw_dev_err(aw881xx->dev, "%s:unsupported prof_type[%x]\n", __func__,
			aw881xx->prof_info.prof_type);
		return -EINVAL;
	}

	aw_dev_info(aw881xx->dev, "%s:scene count is %d\n", __func__, (*count));

	return 0;
}

static int aw881xx_set_dde_prof_id(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	struct aw_cfg_dde_v_1_0_0_0 *cfg_dde =
		(struct aw_cfg_dde_v_1_0_0_0 *)(aw_cfg->data + cfg_hdr->a_hdr_offset);
	struct aw_prof_desc *prof_desc = aw881xx->prof_info.prof_desc;
	int prof_num = aw881xx->prof_info.count;
	int prof_type = aw881xx->prof_info.prof_type;
	int i, j = 0;

	if (prof_type == AW_DEV_TYPE_ID) {
		for(i = 0; i < cfg_hdr->a_ddt_num; ++i) {
			if ((cfg_dde[i].data_type == ACF_SEC_TYPE_REG) &&
				(aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
				(aw881xx->i2c->addr == cfg_dde[i].dev_addr) &&
				(aw881xx->chipid == cfg_dde[i].chip_id)) {
				if (j > prof_num) {
					aw_dev_err(aw881xx->dev,
						"%s:Alrealdy set prof num [%d], redundant scene [%s]exist\n",
						__func__, prof_num, cfg_dde[i].dev_profile_str);
					return -EINVAL;
				} else {
					prof_desc[j].prof_id = cfg_dde[i].dev_profile;
					prof_desc[j].id = j;
					j++;
				}
			}
		}
	} else if (prof_type == AW_DEV_DEFAULT_TYPE_ID) {
		for(i = 0; i < cfg_hdr->a_ddt_num; ++i) {
			if ((cfg_dde[i].data_type == ACF_SEC_TYPE_REG) &&
				(aw881xx->channel == cfg_dde[i].dev_index) &&
				(aw881xx->chipid == cfg_dde[i].chip_id)) {
				if (j > prof_num) {
					aw_dev_err(aw881xx->dev,
						"%s:Alrealdy set prof num [%d], redundant scene [%s]exist\n",
						__func__, prof_num, cfg_dde[i].dev_profile_str);
					return -EINVAL;
				} else {
					prof_desc[j].prof_id = cfg_dde[i].dev_profile;
					prof_desc[j].id = j;
					j++;
				}
			}
		}
	}

	return 0;
}

static int aw881xx_parse_by_hdr_v_1_0_0_0(struct aw881xx *aw881xx, struct aw_cfg_hdr *cfg_hdr)
{
	int ret;

	if (aw881xx->prof_info.prof_type == AW_DEV_TYPE_ID) {
		ret = aw881xx_parse_dev_type_v_1_0_0_0(aw881xx, cfg_hdr);
		if (ret < 0)
			return ret;
	} else if (aw881xx->prof_info.prof_type == AW_DEV_DEFAULT_TYPE_ID) {
		ret = aw881xx_parse_dev_default_type_v_1_0_0_0(aw881xx, cfg_hdr);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int aw881xx_create_prof_name_list_v_1_0_0_0(struct aw881xx *aw881xx)
{
	struct aw_prof_info *prof_info = &aw881xx->prof_info;
	struct aw_prof_desc *prof_desc= prof_info->prof_desc;
	int i;

	if (prof_desc == NULL) {
		aw_dev_err(aw881xx->dev, "%s:prof_desc is NULL\n", __func__);
		return -EINVAL;
	}

	prof_info->prof_name_list = devm_kzalloc(aw881xx->dev,
					prof_info->count * PROFILE_STR_MAX,
					GFP_KERNEL);
	if (prof_info->prof_name_list == NULL) {
		aw_dev_err(aw881xx->dev, "%s:prof_name_list devm_kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < prof_info->count; i++) {
		prof_info->prof_name_list[i] = prof_desc[i].prof_str;
		aw_dev_info(aw881xx->dev, "%s:prof name is %s\n", __func__,
			prof_info->prof_name_list[i]);
	}
	aw881xx->prof_info.prof_max = prof_info->count;

	return 0;
}

static int aw881xx_load_cfg_by_hdr_v_1_0_0_0(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg)
{
	struct aw_prof_info *prof_info = &aw881xx->prof_info;
	struct aw_cfg_hdr *cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	int ret;

	ret = aw881xx_parse_scene_count_v_1_0_0_0(aw881xx, aw_cfg, &prof_info->count);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:get scene count failed\n", __func__);
		return ret;
	}

	prof_info->prof_desc = devm_kzalloc(aw881xx->dev,
					prof_info->count * sizeof(struct aw_prof_desc),
					GFP_KERNEL);
	if (prof_info->prof_desc == NULL) {
		aw_dev_err(aw881xx->dev, "%s:prof_desc devm_kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	ret = aw881xx_set_dde_prof_id(aw881xx, aw_cfg);
	if (ret < 0)
		return ret;

	ret = aw881xx_parse_by_hdr_v_1_0_0_0(aw881xx, cfg_hdr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed\n", __func__);
		return ret;
	}

	ret = aw881xx_create_prof_name_list_v_1_0_0_0(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:create prof name list failed\n", __func__);
		return ret;
	}

	return 0;
}

static int aw881xx_cfg_load(struct aw881xx *aw881xx,
				struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;
	struct aw_all_prof_info all_prof_info;
	int ret;

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	memset(&all_prof_info, 0, sizeof(struct aw_all_prof_info));

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	switch (cfg_hdr->a_hdr_version) {
	case AW_CFG_HDR_VER_0_0_0_1:
		ret = aw881xx_load_cfg_by_hdr(aw881xx, cfg_hdr, &all_prof_info);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:hdr_cersion[0x%x] parse failed\n",
					__func__, cfg_hdr->a_hdr_version);
			return ret;
		}
		break;
	case AW_CFG_HDR_VER_1_0_0_0:
		ret = aw881xx_load_cfg_by_hdr_v_1_0_0_0(aw881xx, aw_cfg);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:hdr_cersion[0x%x] parse failed\n",
					__func__, cfg_hdr->a_hdr_version);
			return ret;
		}
		break;
	default:
		aw_dev_err(aw881xx->dev, "%s:unsupported hdr_version [0x%x]\n",
				__func__, cfg_hdr->a_hdr_version);
		return -EINVAL;
	}

	aw881xx->fw_status = AW881XX_FW_OK;
	aw_dev_info(aw881xx->dev, "%s:parse cfg success\n", __func__);
	return 0;
}

void aw881xx_device_deinit(struct aw881xx *aw881xx)
{
	aw_dev_dbg(aw881xx->dev, "%s:enter\n", __func__);

	if (aw881xx->prof_info.prof_desc != NULL) {
		devm_kfree(aw881xx->dev, aw881xx->prof_info.prof_desc);
		aw881xx->prof_info.prof_desc = NULL;
	}

	aw881xx->prof_info.count = 0;
}

int aw881xx_device_init(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg)
{
	int ret;

	if (aw881xx == NULL || aw_cfg == NULL) {
		pr_err("%s:aw881xx is NULL or aw_cfg is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = aw881xx_cfg_load(aw881xx, aw_cfg);
	if (ret < 0) {
		aw881xx_device_deinit(aw881xx);
		aw_dev_err(aw881xx->dev, "%s:aw_dev acf parse failed\n", __func__);
		return -EINVAL;
	}

	aw881xx->cur_prof = aw881xx->prof_info.prof_desc[0].id;
	aw881xx->set_prof = aw881xx->prof_info.prof_desc[0].id;

	ret = aw881xx_fw_update(aw881xx, true);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: fw update failed\n", __func__);
		return ret;
	}

	aw881xx->status = AW881XX_PW_ON;
	aw881xx_clear_int_status(aw881xx);
	aw881xx_stop(aw881xx);

	mutex_lock(&g_aw881xx_lock);
	list_add(&aw881xx->list_node, &g_dev_list);
	mutex_unlock(&g_aw881xx_lock);

	aw_dev_info(aw881xx->dev, "%s:init done\n", __func__);

	return 0;
}

static void aw881xx_chip_profile_loaded(const struct firmware *cont, void *context)
{
	struct aw881xx *aw881xx = context;
	struct aw881xx_container *aw_cfg = NULL;
	int ret = -1;

	aw881xx->fw_status = AW881XX_FW_FAILED;
	if (!cont) {
		aw_dev_err(aw881xx->dev, "%s:failed to read %s\n",
			__func__, AW881XX_ACF_FILE);
		release_firmware(cont);
		if (aw881xx->fw_retry_cnt == AW_FW_LOAD_RETRIES) {
			aw881xx->fw_retry_cnt = 0;
		} else {
			aw881xx->fw_retry_cnt++;
			msleep(1000);
			aw_dev_info(aw881xx->dev, "load [%s] try [%d]!",
						AW881XX_ACF_FILE, aw881xx->fw_retry_cnt);
			aw881xx_load_chip_profile(aw881xx);
		}
		return;
	}

	aw_dev_info(aw881xx->dev, "%s:loaded %s - size: %zu\n",
			__func__, AW881XX_ACF_FILE, cont ? cont->size : 0);

	mutex_lock(&g_aw881xx_lock);
	if (g_awinic_cfg == NULL) {
		aw_cfg = vmalloc(cont->size + sizeof(uint32_t));
		memset(aw_cfg, 0, sizeof(struct aw881xx_container));
		if (aw_cfg == NULL) {
			aw_dev_err(aw881xx->dev, "%s:aw_cfg kzalloc failed\n",
						__func__);
			release_firmware(cont);
			mutex_unlock(&g_aw881xx_lock);
			return;
		}
		aw_cfg->len = cont->size;
		memcpy(aw_cfg->data, cont->data, cont->size);
		ret = aw881xx_load_acf_check(aw_cfg);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:Load [%s] failed ....!\n",
					__func__, AW881XX_ACF_FILE);
			vfree(aw_cfg);
			aw_cfg = NULL;
			release_firmware(cont);
			mutex_unlock(&g_aw881xx_lock);
			return;
		}
		g_awinic_cfg = aw_cfg;
	} else {
		aw_cfg = g_awinic_cfg;
		aw_dev_info(aw881xx->dev, "%s:[%s] already loaded...\n",
				__func__, AW881XX_ACF_FILE);
	}
	release_firmware(cont);
	mutex_unlock(&g_aw881xx_lock);

	mutex_lock(&aw881xx->lock);
	/*aw device init*/
	ret = aw881xx_device_init(aw881xx, aw_cfg);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s:dev init failed\n", __func__);
		mutex_unlock(&aw881xx->lock);
		return;
	}

	aw881xx_dynamic_create_controls(aw881xx);
	aw881xx->fw_retry_cnt = 0;
	aw881xx_check_spin_mode(&aw881xx->spin_desc);

	mutex_unlock(&aw881xx->lock);

	return;

}

static int aw881xx_load_chip_profile(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter, start load %s\n", __func__, AW881XX_ACF_FILE);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					AW881XX_ACF_FILE,
					aw881xx->dev, GFP_KERNEL, aw881xx,
					aw881xx_chip_profile_loaded);

}

static void aw881xx_request_firmware_file(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	aw881xx_load_chip_profile(aw881xx);

}

static void aw881xx_load_fw_work(struct work_struct *work)
{
	struct aw881xx *aw881xx = container_of(work, struct aw881xx, load_fw_work.work);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	aw881xx_request_firmware_file(aw881xx);
}

static const struct snd_soc_dapm_widget aw881xx_dapm_widgets[] = {
	 /* playback */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "Speaker_Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("audio_out"),
	/* capture */
	SND_SOC_DAPM_AIF_OUT("AIF_TX", "Speaker_Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("iv_in"),
};

static const struct snd_soc_dapm_route aw881xx_audio_map[] = {
	{"audio_out", NULL, "AIF_RX"},
	{"AIF_TX", NULL, "iv_in"},
};


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
static struct snd_soc_dapm_context *snd_soc_codec_get_dapm(struct snd_soc_codec *codec)
{
	return &codec->dapm;
}
#endif

static int aw881xx_add_widgets(struct aw881xx *aw881xx)
{
	int i = 0;
	int ret;
	struct snd_soc_dapm_widget *aw_widgets = NULL;
	struct snd_soc_dapm_route *aw_route = NULL;
#ifdef AW_KERNEL_VER_OVER_4_19_1
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(aw881xx->codec);
#else
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(aw881xx->codec);
#endif

	/*add widgets*/
	aw_widgets = devm_kzalloc(aw881xx->dev,
				sizeof(struct snd_soc_dapm_widget) * ARRAY_SIZE(aw881xx_dapm_widgets),
				GFP_KERNEL);
	if (!aw_widgets)
		return -ENOMEM;

	memcpy(aw_widgets, aw881xx_dapm_widgets,
			sizeof(struct snd_soc_dapm_widget) * ARRAY_SIZE(aw881xx_dapm_widgets));

	for (i = 0; i < ARRAY_SIZE(aw881xx_dapm_widgets); i++) {
		if (aw_widgets[i].name) {
			ret = aw881xx_append_suffix("%s_%d_%x", &aw_widgets[i].name, aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "aw_route source  append i2c suffix failed!\n");
				return ret;
			}
		}

		if (aw_widgets[i].sname) {
			ret = aw881xx_append_suffix("%s_%d_%x", &aw_widgets[i].sname, aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "aw_route source  append i2c suffix failed!\n");
				return ret;
			}
		}
	}

	snd_soc_dapm_new_controls(dapm, aw_widgets, ARRAY_SIZE(aw881xx_dapm_widgets));

	/*add route*/
	aw_route = devm_kzalloc(aw881xx->dev,
				sizeof(struct snd_soc_dapm_route) * ARRAY_SIZE(aw881xx_audio_map),
				GFP_KERNEL);
	if (!aw_route)
		return -ENOMEM;

	memcpy(aw_route, aw881xx_audio_map,
		sizeof(struct snd_soc_dapm_route) * ARRAY_SIZE(aw881xx_audio_map));

	for (i = 0; i < ARRAY_SIZE(aw881xx_audio_map); i++) {
		if (aw_route[i].sink) {
			ret = aw881xx_append_suffix("%s_%d_%x", &aw_route[i].sink, aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "aw_route source  append i2c suffix failed!\n");
				return ret;
			}
		}

		if (aw_route[i].source) {
			ret = aw881xx_append_suffix("%s_%d_%x", &aw_route[i].source, aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "aw_route source  append i2c suffix failed!\n");
				return ret;
			}
		}
	}
	snd_soc_dapm_add_routes(dapm, aw_route, ARRAY_SIZE(aw881xx_audio_map));

	return 0;
}

static int aw881xx_codec_probe(aw_snd_soc_codec_t *codec)
{
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx->codec = codec;

	aw881xx->work_queue = create_singlethread_workqueue("aw881xx");
	if (aw881xx->work_queue == NULL) {
		aw_dev_err(aw881xx->dev, "%s:create workqueue failed!\n",
			__func__);
		return -EINVAL;
	}
	INIT_DELAYED_WORK(&aw881xx->start_work, aw881xx_startup_work);
	INIT_DELAYED_WORK(&aw881xx->load_fw_work, aw881xx_load_fw_work);

	aw881xx_add_codec_controls(aw881xx);
	aw881xx_add_widgets(aw881xx);

	queue_delayed_work(aw881xx->work_queue,
			&aw881xx->load_fw_work,
			msecs_to_jiffies(AW_LOAD_BIN_TIME_MS));

	aw_dev_info(aw881xx->dev, "%s: exit\n", __func__);

	return 0;
}

#ifdef AW_KERNEL_VER_OVER_4_19_1
static void aw881xx_codec_remove(struct snd_soc_component *component)
{
	/*struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);*/

	aw_dev_info(component->dev, "%s: enter\n", __func__);

	return;
}
#else
static int aw881xx_codec_remove(struct snd_soc_codec *codec)
{
	/*struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);*/

	aw_dev_info(codec->dev, "%s: enter\n", __func__);

	return 0;
}
#endif

static unsigned int aw881xx_codec_read(aw_snd_soc_codec_t *codec,
					unsigned int reg)
{
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint16_t value = 0;
	int ret = -1;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (aw881xx_reg_access[reg] & REG_RD_ACCESS) {
		ret = aw881xx_reg_read(aw881xx, reg, &value);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: read register failed\n",
					__func__);
			return ret;
		}
	} else {
		aw_dev_dbg(aw881xx->dev, "%s: register 0x%x no read access\n",
				__func__, reg);
		return ret;
	}
	return ret;
}

static int aw881xx_codec_write(aw_snd_soc_codec_t *codec,
				unsigned int reg, unsigned int value)
{
	int ret = -1;
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_dbg(aw881xx->dev, "%s: enter, reg is 0x%x value is 0x%x\n",
			__func__, reg, value);

	if (aw881xx_reg_access[reg] & REG_WR_ACCESS)
		ret = aw881xx_reg_write(aw881xx, (uint8_t) reg,
					(uint16_t) value);
	else
		aw_dev_dbg(aw881xx->dev, "%s: register 0x%x no write access\n",
				__func__, reg);

	return ret;
}

#ifdef AW_KERNEL_VER_OVER_4_19_1
static struct snd_soc_component_driver soc_codec_dev_aw881xx = {
	.probe = aw881xx_codec_probe,
	.remove = aw881xx_codec_remove,
	.read = aw881xx_codec_read,
	.write = aw881xx_codec_write,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw881xx = {
	.probe = aw881xx_codec_probe,
	.remove = aw881xx_codec_remove,
	.read = aw881xx_codec_read,
	.write = aw881xx_codec_write,
	.reg_cache_size = AW881XX_REG_MAX,
	.reg_word_size = 2,
};
#endif

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw881xx_interrupt_setup(struct aw881xx *aw881xx)
{
	uint16_t reg_val = 0;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINTM, &reg_val);
	reg_val &= (~AW881XX_BIT_SYSINTM_PLLM);
	reg_val &= (~AW881XX_BIT_SYSINTM_OTHM);
	reg_val &= (~AW881XX_BIT_SYSINTM_OCDM);
	aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM, reg_val);
}

static void aw881xx_interrupt_handle(struct aw881xx *aw881xx)
{
	uint16_t reg_val = 0;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSST=0x%x\n", __func__, reg_val);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINT, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSINT=0x%x\n", __func__, reg_val);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINTM, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw881xx_irq(int irq, void *data)
{
	struct aw881xx *aw881xx = data;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_interrupt_handle(aw881xx);

	aw_dev_info(aw881xx->dev, "%s: exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw881xx_parse_gpio_dt(struct aw881xx *aw881xx,
				 struct device_node *np)
{
	aw881xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw881xx->reset_gpio < 0) {
		aw_dev_err(aw881xx->dev,
			"%s: no reset gpio provided, will not hw reset\n",
			__func__);
		return -EIO;
	} else {
		aw_dev_info(aw881xx->dev, "%s: reset gpio provided ok\n",
			__func__);
	}

	aw881xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw881xx->irq_gpio < 0)
		aw_dev_info(aw881xx->dev,
		"%s: no irq gpio provided.\n", __func__);
	else
		aw_dev_info(aw881xx->dev,
		"%s: irq gpio provided ok.\n", __func__);

	return 0;
}

static void aw881xx_parse_channel_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t channel_value;

	ret = of_property_read_u32(np, "sound-channel", &channel_value);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read sound-channel failed, use default: 0\n",
			__func__);
		channel_value = AW881XX_CHAN_VAL_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read sound-channel value is : %d\n",
			__func__, channel_value);
	}

	aw881xx->channel = channel_value;
}


static void aw881xx_parse_pa_sync_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t pa_syn_en;

	ret = of_property_read_u32(np, "pa-syn-enable", &pa_syn_en);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read pa-syn-enable failed, use default: 0\n",
			__func__);
		pa_syn_en = AW881XX_PA_SYNC_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read a-syn-enable value is: %d\n",
			__func__, pa_syn_en);
	}

	aw881xx->pa_syn_en = pa_syn_en;
}

static void aw881xx_parse_fade_enable_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t fade_en;

	ret = of_property_read_u32(np, "fade-enable", &fade_en);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read fade-enable failed, use default: 0\n",
			__func__);
		fade_en = AW881XX_FADE_IN_OUT_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read fade-enable value is: %d\n",
			__func__, fade_en);
	}

	aw881xx->fade_en = fade_en;
}

static int aw881xx_parse_dt(struct device *dev, struct aw881xx *aw881xx,
				struct device_node *np)
{
	int ret;

	aw881xx_parse_channel_dt(aw881xx, np);
	aw881xx_parse_pa_sync_dt(aw881xx, np);
	aw881xx_parse_fade_enable_dt(aw881xx, np);
	ret = aw881xx_parse_gpio_dt(aw881xx, np);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw881xx_hw_reset(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	if (gpio_is_valid(aw881xx->reset_gpio)) {
		gpio_set_value_cansleep(aw881xx->reset_gpio, 0);
		usleep_range(AW_1000_US, AW_1000_US + 10);
		gpio_set_value_cansleep(aw881xx->reset_gpio, 1);
		usleep_range(AW_1000_US, AW_1000_US + 10);
	} else {
		aw_dev_err(aw881xx->dev, "%s: failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static void aw881xx_ui_update_cfg_name(struct aw881xx *aw881xx)
{
	char aw881xx_head[] = { "aw881xx_pid_" };
	char buf[20] = { 0 };
	uint8_t i = 0;
	uint8_t head_index = 0;
	int i2cbus = aw881xx->i2c->adapter->nr;
	int addr = aw881xx->i2c->addr;

	memcpy(aw881xx->ui_cfg_name, aw881xx_cfg_name, sizeof(aw881xx_cfg_name));
	head_index = sizeof(aw881xx_head) - 1;

	/*add product information*/
	snprintf(buf, sizeof(buf), "%02x", aw881xx->pid);
	for (i = 0; i < AW_UI_MODE_MAX; i++)
		memcpy(aw881xx->ui_cfg_name[i] + head_index, buf, strlen(buf));

	/*add sound channel information*/
	snprintf(buf, sizeof(buf), "_%x_%x.bin", i2cbus, addr);
	for (i = 0; i < AW_UI_MODE_MAX; i++) {
		head_index = strlen(aw881xx->ui_cfg_name[i]);
		memcpy(aw881xx->ui_cfg_name[i] + head_index, buf, strlen(buf) + 1);
	}

	for (i = 0; i < AW_UI_MODE_MAX; i++)
		aw_dev_dbg(aw881xx->dev, "%s: id[%d], [%s]\n",
				__func__, i, aw881xx->ui_cfg_name[i]);
}

int aw881xx_read_dsp_pid(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_REG_DSP_PID, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read AW881XX_REG_DSP_PID: %d\n",
				__func__, ret);
		return -EIO;
	}

	switch (reg_val) {
	case AW881XX_DSP_PID_01:
		aw881xx->pid = AW881XX_PID_01;
		break;
	case AW881XX_DSP_PID_03:
		aw881xx->pid = AW881XX_PID_03;
		break;
	default:
		aw_dev_info(aw881xx->dev, "%s: unsupported dsp_pid=0x%04x\n",
				__func__, reg_val);
		return -EIO;
	}

	aw881xx_ui_update_cfg_name(aw881xx);

	return 0;
}

static int aw881xx_read_product_id(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t product_id = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_PRODUCT_ID, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read REG_EFRL: %d\n",
			__func__, ret);
		return -EIO;
	}

	product_id = reg_val & (~AW881XX_BIT_PRODUCT_ID_MASK);
	switch (product_id) {
	case AW881XX_PID_01:
		aw881xx->pid = AW881XX_PID_01;
		break;
	case AW881XX_PID_03:
		aw881xx->pid = AW881XX_PID_03;
		break;
	default:
		aw881xx->pid = AW881XX_PID_03;
	}

	aw881xx_ui_update_cfg_name(aw881xx);

	return 0;
}

static int aw881xx_read_chipid(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint16_t reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw881xx_reg_read(aw881xx, AW881XX_REG_ID, &reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
				"%s: failed to read chip id, ret=%d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_val) {
		case AW881XX_CHIPID:
			aw881xx->chipid = AW881XX_CHIPID;
			aw881xx->flags |= AW881XX_FLAG_START_ON_MUTE;
			aw881xx->flags |= AW881XX_FLAG_SKIP_INTERRUPTS;

			aw881xx_read_product_id(aw881xx);

			aw_dev_info(aw881xx->dev,
				"%s: chipid=0x%04x, product_id=0x%02x\n",
				__func__, aw881xx->chipid, aw881xx->pid);
			return 0;
		default:
			aw_dev_info(aw881xx->dev,
				"%s: unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;
		usleep_range(AW_5000_US, AW_5000_US + 100);
	}

	return -EINVAL;
}


static int aw881xx_ui_dsp_cfg_loaded(struct aw881xx *aw881xx)
{
	struct aw881xx_container *aw881xx_cfg = NULL;
	const struct firmware *cont = NULL;
	int ret = -1;

	ret = request_firmware(&cont, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE],
				aw881xx->dev);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read %s\n",
			__func__, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE]);
		release_firmware(cont);
		return ret;
	}

	aw_dev_info(aw881xx->dev, "%s: loaded %s - size: %zu\n",
		__func__, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE],
				cont ? cont->size : 0);

	aw881xx_cfg = devm_kzalloc(aw881xx->dev,
			cont->size + sizeof(uint32_t), GFP_KERNEL);
	if (aw881xx_cfg == NULL) {
		aw_dev_err(aw881xx->dev, "%s: alloc failed!\n", __func__);
		release_firmware(cont);
		return ret;
	}
	aw881xx_cfg->len = cont->size;
	memcpy(aw881xx_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	aw881xx_update_dsp_data_order(aw881xx,
			aw881xx_cfg->data, aw881xx_cfg->len);

	aw881xx_dsp_cfg_update(aw881xx,
		aw881xx_cfg->data, aw881xx_cfg->len);

	devm_kfree(aw881xx->dev, aw881xx_cfg);
	aw881xx_cfg = NULL;

	return 0;
}


static int aw881xx_ui_dsp_fw_loaded(struct aw881xx *aw881xx)
{
	struct aw881xx_container *aw881xx_cfg = NULL;
	const struct firmware *cont = NULL;
	int ret = -1;

	ret = request_firmware(&cont, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE], aw881xx->dev);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read %s\n",
				__func__, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE]);
		release_firmware(cont);
		return ret;
	}

	aw_dev_info(aw881xx->dev, "%s: loaded %s - size: %zu\n",
			__func__, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE],
			cont ? cont->size : 0);

	aw881xx_cfg = devm_kzalloc(aw881xx->dev,
			cont->size + sizeof(uint32_t), GFP_KERNEL);
	if (aw881xx_cfg == NULL) {
		aw_dev_err(aw881xx->dev, "%s: devm_kzalloc failed!\n", __func__);
		release_firmware(cont);
		return -ENOMEM;
	}
	aw881xx_cfg->len = cont->size;
	memcpy(aw881xx_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	aw881xx_update_dsp_data_order(aw881xx,
				aw881xx_cfg->data, aw881xx_cfg->len);
	aw881xx_dsp_fw_update(aw881xx, aw881xx_cfg->data, aw881xx_cfg->len);

	devm_kfree(aw881xx->dev, aw881xx_cfg);
	aw881xx_cfg = NULL;

	return 0;
}

static int aw881xx_ui_update_dsp(struct aw881xx *aw881xx)
{
	int ret = -1;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	ret = aw881xx_syspll_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: syspll check failed\n", __func__);
		return ret;
	}

	aw881xx_get_dsp_config(aw881xx);
	aw881xx_read_dsp_pid(aw881xx);

	aw881xx_run_mute(aw881xx, true);
	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_PLL);

	ret = aw881xx_ui_dsp_fw_loaded(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp fw loading failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = aw881xx_ui_dsp_cfg_loaded(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp cfg loading failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = aw881xx_sysst_check(aw881xx);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s: sysst check fail\n",
				__func__);
		return ret;
	}

	ret = aw881xx_dsp_check(aw881xx);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s: dsp cfg update error\n",
				__func__);
		return ret;
	}

	aw881xx->status = AW881XX_PW_OFF;
	ret = aw881xx_device_start(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: start fail, ret=%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

/******************************************************
 *
 * sys group attribute
 *
 ******************************************************/
static ssize_t aw881xx_reg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw881xx_reg_write(aw881xx, databuf[0], databuf[1]);

	return count;
}

static ssize_t aw881xx_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_REG_MAX; i++) {
		if (aw881xx_reg_access[i] & REG_RD_ACCESS) {
			aw881xx_reg_read(aw881xx, i, &reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}
	return len;
}

static ssize_t aw881xx_rw_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw881xx->reg_addr = (uint8_t) databuf[0];
		aw881xx_reg_write(aw881xx, databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw881xx->reg_addr = (uint8_t) databuf[0];
	}

	return count;
}

static ssize_t aw881xx_rw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t reg_val = 0;

	if (aw881xx_reg_access[aw881xx->reg_addr] & REG_RD_ACCESS) {
		aw881xx_reg_read(aw881xx, aw881xx->reg_addr, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%04x\n", aw881xx->reg_addr,
				reg_val);
	}
	return len;
}

static ssize_t aw881xx_dsp_rw_show( struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	ssize_t len = 0;
	uint16_t reg_val = 0;

	mutex_lock(&aw881xx->i2c_lock);
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, aw881xx->dsp_addr);
	aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"dsp:0x%04x=0x%04x\n", aw881xx->dsp_addr, reg_val);
	aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"dsp:0x%04x=0x%04x\n", aw881xx->dsp_addr + 1, reg_val);
	mutex_unlock(&aw881xx->i2c_lock);

	return len;
}

static ssize_t aw881xx_dsp_rw_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw881xx->dsp_addr = (unsigned int)databuf[0];
		aw881xx_dsp_write(aw881xx, databuf[0], databuf[1]);
		aw_dev_dbg(aw881xx->dev, "%s: get param: %x %x\n", __func__,
				databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw881xx->dsp_addr = (unsigned int)databuf[0];
		aw_dev_dbg(aw881xx->dev, "%s: get param: %x\n", __func__,
				databuf[0]);
	}

	return count;
}

static ssize_t aw881xx_dsp_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	unsigned int val = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	aw_dev_dbg(aw881xx->dev, "%s: value=%d\n", __func__, val);

	if (val) {
		cancel_delayed_work_sync(&aw881xx->start_work);

		mutex_lock(&aw881xx->lock);
		aw881xx_ui_update_dsp(aw881xx);
		mutex_unlock(&aw881xx->lock);
	}

	return count;
}

static ssize_t aw881xx_dsp_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	ssize_t len = 0;
	unsigned int i = 0;
	uint16_t reg_val = 0;
	int ret = -1;
	char *dsp_reg_info = NULL;
	ssize_t dsp_info_len = 0;

	if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
		len += snprintf((char *)(buf + len), PAGE_SIZE - len,
				"%s: aw881xx dsp bypass\n", __func__);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"aw881xx dsp working\n");
		ret = aw881xx_get_iis_status(aw881xx);
		if (ret < 0) {
			len += snprintf((char *)(buf + len),
					PAGE_SIZE - len,
					"%s: aw881xx no iis signal\n",
					__func__);
		} else {
			dsp_reg_info = devm_kzalloc(aw881xx->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
			aw_dev_info(aw881xx->dev, "%s: aw881xx_dsp_firmware:\n",
					__func__);
			mutex_lock(&aw881xx->i2c_lock);
			aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD,
					AW881XX_DSP_FW_ADDR);
			for (i = 0; i < aw881xx->dsp_fw_len; i += 2) {
				aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
				dsp_info_len += snprintf(dsp_reg_info + dsp_info_len, AW_NAME_BUF_MAX - dsp_info_len,
				"%02x,%02x,", (reg_val >> 0) & 0xff, (reg_val >> 8) & 0xff);

				if ((i / 2 + 1) % 8 == 0) {
					aw_dev_info(aw881xx->dev, "%s:dsp_fw: %s\n", __func__, dsp_reg_info);
					dsp_info_len = 0;
					memset(dsp_reg_info, 0, AW_NAME_BUF_MAX);
				}

				if (((aw881xx->dsp_fw_len) % 8 != 0) && (i == (aw881xx->dsp_fw_len - 2))) {
					aw_dev_info(aw881xx->dev, "%s:dsp_fw: %s\n", __func__, dsp_reg_info);
					dsp_info_len = 0;
					memset(dsp_reg_info, 0, AW_NAME_BUF_MAX);
				}
			}

			dsp_info_len = 0;
			memset(dsp_reg_info, 0, AW_NAME_BUF_MAX);

			aw_dev_info(aw881xx->dev, "%s: aw881xx_dsp_cfg:\n",
					__func__);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw881xx dsp config:\n");
			aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD,
					AW881XX_DSP_CFG_ADDR);
			for (i = 0; i < aw881xx->dsp_cfg_len; i += 2) {
				aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
				len += snprintf(buf + len, PAGE_SIZE - len,
					"%02x,%02x,", (reg_val >> 0) & 0xff,
					(reg_val >> 8) & 0xff);

				dsp_info_len += snprintf(dsp_reg_info + dsp_info_len, AW_NAME_BUF_MAX - dsp_info_len,
				"%02x,%02x,", (reg_val >> 0) & 0xff, (reg_val >> 8) & 0xff);

				if ((i / 2 + 1) % 8 == 0) {
					len += snprintf(buf + len,
							PAGE_SIZE - len, "\n");
					aw_dev_info(aw881xx->dev, "%s:dsp_cfg: %s\n", __func__, dsp_reg_info);
					dsp_info_len = 0;
					memset(dsp_reg_info, 0, AW_NAME_BUF_MAX);
				}

				if (((aw881xx->dsp_cfg_len) % 8 != 0) && (i == (aw881xx->dsp_cfg_len - 2))) {
					aw_dev_info(aw881xx->dev, "%s:dsp_cfg: %s\n", __func__, dsp_reg_info);
					dsp_info_len = 0;
					memset(dsp_reg_info, 0, AW_NAME_BUF_MAX);
				}
			}
			mutex_unlock(&aw881xx->i2c_lock);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			devm_kfree(aw881xx->dev, dsp_reg_info);
			dsp_reg_info = NULL;
		}
	}

	return len;
}

static ssize_t aw881xx_dsp_fw_ver_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	/*struct aw881xx *aw881xx = dev_get_drvdata(dev);*/

	return count;
}

static ssize_t aw881xx_dsp_fw_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t dsp_addr = 0x8c00;
	uint16_t temp_val = 0;
	uint32_t dsp_val = 0;

	for (dsp_addr = 0x8c00; dsp_addr < 0x9000; dsp_addr++) {
		dsp_val = 0;
		aw881xx_dsp_read(aw881xx, dsp_addr, &temp_val);
		dsp_val |= (temp_val << 0);
		aw881xx_dsp_read(aw881xx, dsp_addr + 1, &temp_val);
		dsp_val |= (temp_val << 16);
		if ((dsp_val & 0x7fffff00) == 0x7fffff00) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"dsp fw ver: v1.%u\n",
					0xff - (dsp_val & 0xff));
			break;
		}
	}

	return len;
}

static ssize_t aw881xx_spk_temp_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	ssize_t len = 0;
	int ret;
	int16_t reg_val;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_ASR2, (uint16_t *)&reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: read addr:0x%x failed\n",
					__func__, AW881XX_REG_ASR2);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len,
			"Temp:%d\n", reg_val);

	return len;
}

static ssize_t aw881xx_diver_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"driver version:%s\n", AW881XX_DRIVER_VERSION);

	return len;
}

static ssize_t aw881xx_fade_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t fade_en;

	if (1 == sscanf(buf, "%u", &fade_en))
		aw881xx->fade_en = fade_en;

	aw_dev_info(aw881xx->dev, "%s: set fade %d\n",
		__func__, aw881xx->fade_en);

	return count;
}

static ssize_t aw881xx_fade_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"fade_en: %u\n", aw881xx->fade_en);

	return len;
}

static ssize_t aw881xx_pa_sync_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t pa_syn_en;

	if (1 == sscanf(buf, "%u", &pa_syn_en))
		aw881xx->pa_syn_en = pa_syn_en;

	aw_dev_info(aw881xx->dev, "%s: set pa_syn_en %d\n",
		__func__, aw881xx->pa_syn_en);

	return count;
}

static ssize_t aw881xx_pa_sync_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"pa_syn_en: %u\n", aw881xx->pa_syn_en);

	return len;
}

static ssize_t aw881xx_log_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "i2c_log_en: %d\n",
		aw881xx->i2c_log_en);

	return len;
}

static ssize_t aw881xx_log_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t log_en = 0;

	if (1 == sscanf(buf, "%u", &log_en))
		aw881xx->i2c_log_en = log_en;

	aw_dev_info(aw881xx->dev, "set i2c_log_en: %d",
		aw881xx->i2c_log_en);

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
			aw881xx_reg_show, aw881xx_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO,
			aw881xx_rw_show, aw881xx_rw_store);
static DEVICE_ATTR(dsp_fw_ver, S_IWUSR | S_IRUGO,
			aw881xx_dsp_fw_ver_show, aw881xx_dsp_fw_ver_store);
static DEVICE_ATTR(driver_ver, S_IRUGO,
			aw881xx_diver_ver_show, NULL);
static DEVICE_ATTR(fade_en, S_IWUSR | S_IRUGO,
			aw881xx_fade_enable_show, aw881xx_fade_enable_store);
static DEVICE_ATTR(pa_sync_en, S_IWUSR | S_IRUGO,
			aw881xx_pa_sync_enable_show, aw881xx_pa_sync_enable_store);
static DEVICE_ATTR(i2c_log_en, S_IWUSR | S_IRUGO,
			aw881xx_log_en_show, aw881xx_log_en_store);

static struct attribute *aw881xx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rw.attr,
	&dev_attr_dsp_fw_ver.attr,
	&dev_attr_driver_ver.attr,
	&dev_attr_fade_en.attr,
	&dev_attr_pa_sync_en.attr,
	&dev_attr_i2c_log_en.attr,
	NULL
};

static struct attribute_group aw881xx_attribute_group = {
	.attrs = aw881xx_attributes
};

/* duanyitao add for vivo cali start */
extern void set_vivo_cali_function(struct vivo_cali_ops *fun);
static struct smartpa_param vivo_spk_param;
static struct smartpa_param vivo_rcv_param;
static struct vivo_cali_ops aw881xx_cali_ops;
static int cali_channels;

// parse cali data to smartpa_param
static void parse_calib_to_param(struct smartpa_param *sp, uint32_t calib_re_mo)
{
	if (calib_re_mo == 0 || ((calib_re_mo >= sp->Imped_Min_mo) && (calib_re_mo <= sp->Imped_Max_mo))) {
		sp->Calibration_Success = 1;
		sp->Re_mo = calib_re_mo;
	} else {
		sp->Calibration_Success = 0;
		sp->Re_mo = CALIB_FAILED;
	}
}

// parse Q_1000 data to smartpa_param
static bool parse_q_1000_to_param(struct smartpa_param *sp, uint32_t q_1000)
{
	sp->Qts_1000 = q_1000;
	if (q_1000 >= sp->Qt_Min_1000) {
		return true;
	} else {
		return false;
	}
}

// parse f0_hZ data to smartpa_param
static bool parse_f0_hz_to_param(struct smartpa_param *sp, uint32_t f0_hZ)
{
	sp->F0_hz = f0_hZ;

	if (f0_hZ >= sp->Freq_Min_hz && f0_hZ <= sp->Freq_Max_hz) {
		return true;
	} else {
		return false;
	}
}

// smartpa calibration
int aw881xx_smartpa_init_dbg(char *buffer, int size) {
	int n = 0;
	struct smartpa_param *privPointer;
	int done = 0;
	// uint32_t cali_to_file[4] = {0}; // max pa nums is four
	int ret = 0;
	int i = 0;
	int return_val = 0;

	/*============ 3rd code to cali ============*/
	int32_t temp_data_re[AW_DEV_CH_MAX << 1] = {CALIB_FAILED};

	if (cali_channels <=0 || cali_channels >4) {
		printk("[SmartPA]%s, cali_channels invalid (%d)\n", __func__, cali_channels);
		return -1;
	}

	ret = aw_cali_svc_cali_cmd(aw_pa_dev, AW_CALI_CMD_RE, false, CALI_OPS_HMUTE);
	if(ret < 0){
		printk("[SmartPA]%s, Re calibration failed\n", __func__);
	}else{
		printk("[SmartPA]%s, aw cali svc cali cmd success\n", __func__);
		//vivo add start by linjinping, solve calibration probability fail problem
		//aw_cali_svc_get_devs_cali_re(aw_pa_dev, temp_data_re, AW_DEV_CH_MAX);
		aw_cali_svc_get_dev_cali_val(aw_pa_dev, GET_RE_TYPE, temp_data_re);
		//vivo add end
	}

	/*============ print info to upper ============*/
	if (cali_channels == 1)
		n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", "Mono");
	else if (cali_channels == 2)
		n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", "Stereo");
	else {
		printk("[SmartPA]%s, used for future\n", __func__);
	}

	for (i = 0; i < cali_channels; i++) {
		if (i == 0) {
			privPointer = &vivo_spk_param;
		} else if (i == 1) {
			privPointer = &vivo_rcv_param;
		} else {
			printk("[SmartPA]%s, used for future\n", __func__);
			break;
		}

		parse_calib_to_param(privPointer, temp_data_re[i]); // copy data to smartpa_param
		// cali_to_file[i] = privPointer->Re_mo;
		printk("[SmartPA]%s, Re_mo read from AW interface: i = %d, Re_mo = %u\n",
			__func__, i, privPointer->Re_mo);

		if (privPointer->Calibration_Success == 1) {
			n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %01d.%03d ohm, ",
				i, privPointer->Re_mo/1000, privPointer->Re_mo%1000);

			printk("[SmartPA]%s, Re_mo show to upper: i = %d, impedance = %01d.%03d ohm\n",
				__func__, i, privPointer->Re_mo/1000, privPointer->Re_mo%1000);

			done++;
			return_val += (1<<i);
		} else {
			n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %#X ohm, ", i, CALIB_FAILED);
		}

		n += scnprintf(buffer + n, size - n, "valid range(%01d.%03d ~ %01d.%03d ohm).\n",
			privPointer->Imped_Min_mo/1000, privPointer->Imped_Min_mo%1000,
			privPointer->Imped_Max_mo/1000, privPointer->Imped_Max_mo%1000);
	}

	if (done == cali_channels) {
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "OKAY(impedance ok).\n");
		printk("[SmartPA]%s, Calibrate result show to upper: OK\n", __func__);
	} else {
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "ERROR(impedance error).\n");
		printk("[SmartPA]%s, Calibrate result show to upper: ERROR\n", __func__);
	}

	buffer[n] = 0;

	/*============ copy cali data to file ============*/
	/**
	 * use AW code, no need to save by vivo
	ret = vivo_smartpa_calib_save(&cali_to_file[0], cali_channels*4);
	if (ret < 0) {
		printk("[SmartPA]%s, copy to file failed\n", __func__);
	} else {
		printk("[SmartPA]%s, copy to file successful\n", __func__);
	}
	*/

	/*============ return info ============*/
	/* bit 0 is spk, bit 1 is rcv */
	printk("[SmartPA]%s, return_val show to upper: %d\n", __func__, return_val);
	return return_val;
}

// return cali result
/* bit 0 is spk, bit 1 is rcv */
int aw881xx_smartpa_check_calib_dbg(void) {
	int ret = 0;
	int i;
	uint32_t calib_mo_from_file[4] = {0};
	struct smartpa_param *privPointer;
	int return_val = 0;

	pr_info("[SmartPA]%s, enter\n", __func__);

	/*============ get Re_mo from file ============*/
	for (i = 0; i < cali_channels; i++) {
		if (i == 0) {
			privPointer = &vivo_spk_param;
		} else if (i == 1) {
			privPointer = &vivo_rcv_param;
		} else {
			printk("[SmartPA]%s, used for future\n", __func__);
			break;
		}

		 /* second param is from dts: sound-channel */
		 /* 0 is spk, 1 is rcv */
		ret = aw881xx_get_cali_re_from_file(&calib_mo_from_file[i], i);
		if (ret) {
			printk("[SmartPA]%s, get calibration from file error(i = %d)\n", __func__, i);
			calib_mo_from_file[i] = CALIB_FAILED;
		} else {
			calib_mo_from_file[i] = AW_DSP_RE_TO_SHOW_RE(calib_mo_from_file[i]);
			printk("[SmartPA]%s, get calibration from file success, i = %d, \n", __func__, i);
		}

		pr_info("[SmartPA]%s, Re_mo from file: channel[%d] = %u\n", __func__, i, calib_mo_from_file[i]);
		parse_calib_to_param(privPointer, calib_mo_from_file[i]); // copy file info to param

		if (privPointer->Calibration_Success == 1) {
			return_val += (1<<i);
		}
	}

	printk("[SmartPA]%s, return_val show to upper: %d\n", __func__, return_val);
	return return_val;
}

//vivo add start by linjinping, operating smartamp.bin file on user layer to solve gki unsymbol problem
int aw881xx_smartpa_init_calib_data(char *buffer, int size)
{
	pr_info("[SmartPA]%s, enter\n", __func__);
	return aw_cali_init_smartamp_bin_data(buffer, size);
}

int aw881xx_smartpa_get_calib_data(char *buffer)
{
	pr_info("[SmartPA]%s, enter\n", __func__);
	return aw_cali_get_smartamp_bin_data(buffer);
}
//vivo add end

//vivo linjinping add start for calib fail mute check
int aw_dev_set_cali_fail_mute_enable(int en)
{
	cali_fail_mute_en = en;
	aw_pr_info("enable(%d) success", en);
	return 0;
}
//vivo linjinping add end

//vivo linjinping add start for calib fail mute check
int aw881xx_smartpa_cali_fail_mute_enable(int en)
{
	pr_info("[SmartPA]%s, enable(%d) enter\n", __func__, en);
	return aw_dev_set_cali_fail_mute_enable(en);
}
//vivo linjinping add end

// f0 and Q
int aw881xx_smartpa_read_freq_dbg(char *buffer, int size) {
	int ret = 0;
	int i, j;
	uint32_t calib_mo_from_file[4] = {0};
	int n = 0;
	struct smartpa_param *privPointer;
	int return_val = 0; /* bit 0 is spk, bit 1 is rcv */

	uint32_t temp_F0_hz[2][4]; // [j][i]: j is pa index, i is times, need 3times data
	uint32_t temp_Q_1000[2][4]; // [j][0]: 3times are successful

	/*============ 3rd code to compute f0_HZ and Q_1000 ============*/
	int32_t temp_data_f0[AW_DEV_CH_MAX << 1] = {CALIB_FAILED};
	int32_t temp_data_q[AW_DEV_CH_MAX << 1] = {CALIB_FAILED};

	ret = aw_cali_svc_cali_cmd(aw_pa_dev, AW_CALI_CMD_F0_Q,
					false, CALI_OPS_HMUTE|CALI_OPS_NOISE);
	if (ret < 0){
		pr_err("[SmartPA]%s, f0 calibration failed\n", __func__);
	}else{
		//aw_cali_svc_get_devs_cali_f0_q(aw_pa_dev,temp_data_f0, temp_data_q, AW_DEV_CH_MAX);
		aw_cali_svc_get_dev_cali_val(aw_pa_dev, GET_F0_TYPE, temp_data_f0);
		aw_cali_svc_get_dev_cali_val(aw_pa_dev, GET_Q_TYPE, temp_data_q);
	}

	/*============ get Re_mo from file ============*/
	// ret = vivo_smartpa_calib_get(&calib_mo_from_file[0], cali_channels*4);
	for (j = 0; j < cali_channels; j++) {
		if (j == 0) {
			privPointer = &vivo_spk_param;
		} else if (j == 1) {
			privPointer = &vivo_rcv_param;
		} else {
			printk("[SmartPA]%s, used for future\n", __func__);
			break;
		}

		 /* second param is from dts: sound-channel */
		 /* 0 is spk, 1 is rcv */
		ret = aw881xx_get_cali_re_from_file(&calib_mo_from_file[j], j);
		if (ret) {
			pr_err("[SmartPA]%s, get calibration from file error(j = %d)\n", __func__, j);
			calib_mo_from_file[j] = CALIB_FAILED;
		} else {
			calib_mo_from_file[j] = AW_DSP_RE_TO_SHOW_RE(calib_mo_from_file[j]);
		}

		pr_info("[SmartPA]%s, Re_mo from file: channel[%d] = %u\n", __func__, j, calib_mo_from_file[j]);
		parse_calib_to_param(privPointer, calib_mo_from_file[j]);
	}

	/*============ copy data to upper layer ============*/
	// j is channel index, 0 is smartpa, 1 is rcv
	// i is times, total 3 times, i = 0 means whether all 3 times are successful
	for (j = 0; j < cali_channels; j++) {
		for (i = 1; i <= 3; i++) {
			temp_F0_hz[j][i] = (uint32_t)temp_data_f0[j];
			temp_Q_1000[j][i] = (uint32_t)temp_data_q[j];
		}

		if (j == 0)
			privPointer = &vivo_spk_param;
		else if (j == 1)
			privPointer = &vivo_rcv_param;
		else {
			printk("[SmartPA]%s, used for future\n", __func__);
			break;
		}

		temp_F0_hz[j][0] = parse_f0_hz_to_param(privPointer, temp_F0_hz[j][3]);
		temp_Q_1000[j][0] = parse_q_1000_to_param(privPointer, temp_Q_1000[j][3]);

		printk("[SmartPA]%s, j = %d, f0_hz (%u): %u, %u, %u\n",
			__func__, j, temp_F0_hz[j][0], temp_F0_hz[j][1], temp_F0_hz[j][2], temp_F0_hz[j][3]);
		printk("[SmartPA]%s, j = %d, q_1000 (%u): %u, %u, %u\n",
			__func__, j, temp_Q_1000[j][0], temp_Q_1000[j][1], temp_Q_1000[j][2], temp_Q_1000[j][3]);

		n += scnprintf(buffer+n, size-n, "Ch[%d] ", j);
		n += scnprintf(buffer+n, size-n, "Rdc = %01d.%03d\n",
				privPointer->Re_mo/1000, privPointer->Re_mo%1000);
		printk("[SmartPA]%s, Calibrate result show to upper: Ch[%d], Rdc = %01d.%03d\n",
			__func__, j, privPointer->Re_mo/1000, privPointer->Re_mo%1000);

		for (i = 1; i <= 3; i++) {
			n += scnprintf(buffer+n, size-n, "f0 = %01d Qt = %01d.%03d\n",
				temp_F0_hz[j][i], temp_Q_1000[j][i]/1000, temp_Q_1000[j][i]%1000);
			printk("[SmartPA]%s, f0 and Qt show to upper: j = %d, f0 = %01d, Qt = %01d.%03d\n",
				__func__, j, temp_F0_hz[j][i], temp_Q_1000[j][i]/1000, temp_Q_1000[j][i]%1000);
		}

		n += scnprintf(buffer+n, size-n, "F0 (%d~%d)\nQ_Min: %01d.%03d \n",
				privPointer->Freq_Min_hz, privPointer->Freq_Max_hz,
				privPointer->Qt_Min_1000/1000, privPointer->Qt_Min_1000%1000);

		if (temp_F0_hz[j][0] == 1 && temp_Q_1000[j][0] == 1) {
			n += scnprintf(buffer+n, size-n, "PASS\n");
			printk("[SmartPA]%s, f0 and Q result show to upper: PASS (j = %d)\n", __func__, j);
			return_val += (1<<j);
		} else {
			n += scnprintf(buffer+n, size-n, "FAIL\n");
			printk("[SmartPA]%s, f0 and Q result show to upper: FAIL (j = %d)\n", __func__, j);
		}
	}
	buffer[n] = 0;

	printk("[SmartPA]%s, return_val show to upper: %d\n", __func__, return_val);
	pr_info("[SmartPA]%s, leave<===\n", __func__);
	return return_val;
}

static void dump_param_info(struct smartpa_param *param) {
	printk("[SmartPa] %s : dump info==>", __func__);
	printk("[SmartPa] vivo,Impedance-Min-mo  :%u\n", param->Imped_Min_mo);
	printk("[SmartPa] vivo,Impedance-Max-mo  :%u\n", param->Imped_Max_mo);
	printk("[SmartPa] vivo,Frequency-Min-hz  :%u\n", param->Freq_Min_hz);
	printk("[SmartPa] vivo,Frequency-Max-hz  :%u\n", param->Freq_Max_hz);
	printk("[SmartPa] vivo,Qt-Min-1000       :%u\n", param->Qt_Min_1000);
	printk("[SmartPa] vivo,VendorID          :%u\n", param->PA_ID);
	printk("[SmartPa] vivo,V-Max             :%u\n", param->V_Max);
	printk("[SmartPa] vivo,I-Max             :%u\n", param->I_Max);
	printk("[SmartPa] vivo,Vout-Max          :%u\n", param->Vout_Max);
	printk("[SmartPa] Calibration_Success    :%u\n", param->Calibration_Success);
	printk("[SmartPa] Re_mo                  :%u\n", param->Re_mo);
	printk("[SmartPa] F0_hz                  :%u\n", param->F0_hz);
	printk("[SmartPa] Qts_1000               :%u\n", param->Qts_1000);
}

static int vivo_smartpa_parse_dt(struct i2c_client *i2c, struct smartpa_param *sp)
{
	unsigned int temp = 0;
	int ret = 0;
	pr_info("[SmartPA]%s, enter.\n", __func__);

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Impedance-Min-mo", &temp);
	sp->Imped_Min_mo = (!ret) ? temp:6000;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Impedance-Max-mo", &temp);
	sp->Imped_Max_mo = (!ret) ? temp:10000;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Frequency-Min-hz", &temp);
	sp->Freq_Min_hz = (!ret) ? temp:500;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Frequency-Max-hz", &temp);
	sp->Freq_Max_hz = (!ret) ? temp:1000;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-Min-1000", &temp);
	sp->Qt_Min_1000 = (!ret) ? temp:1000;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,VendorID", &temp);
	sp->PA_ID = (!ret) ? temp:0;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,V-Max", &temp);
	sp->V_Max = (!ret) ? temp:0;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,I-Max", &temp);
	sp->I_Max = (!ret) ? temp:0;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Vout-Max", &temp);
	sp->Vout_Max = (!ret) ? temp:0;

	sp->Calibration_Success = 1;
	sp->Re_mo = 0;
	sp->F0_hz = 0;
	sp->Qts_1000 = 0;

	return ret;
}
/* duanyitao add vivo cali end */

//ljp add begin, for AT+BK_AUD_HIFI=1 test
static ssize_t i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	const int size = 20;
	int n = 0;

	pr_info("[SmartPA]%s enter.\n", __func__);

	if (g_aw881xx_dev_cnt == 2) {
		n += scnprintf(buf, size, "SmartPA-stereo OK\n");
	} else if (g_aw881xx_dev_cnt == 1){
		n += scnprintf(buf, size, "SmartPA-mono OK\n");
	} else {
		n += scnprintf(buf, size, "SmartPA-mono ERROR\n");
	}

	return n;
}

static ssize_t test_re_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	const int size = 512;
	int n = 0;
	char test[512];

	pr_info("[SmartPA]%s enter.\n", __func__);
	aw881xx_smartpa_init_dbg(test, size);
	n += scnprintf(buf, size, test);

	return n;
}

static ssize_t test_freq_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	const int size = 512;
	int n = 0;
	char test[512];

	pr_info("[SmartPA]%s enter.\n", __func__);
	aw881xx_smartpa_read_freq_dbg(test, 512);
	n += scnprintf(buf, size, test);

	return n;
}

static ssize_t aw881xx_reg_store_common(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw881xx_reg_write(aw881xx, databuf[0], databuf[1]);

	return count;
}

static ssize_t aw881xx_reg_show_common(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = aw_pa_dev;
	ssize_t len = 0;
	uint8_t i = 0;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_REG_MAX; i++) {
		if (aw881xx_reg_access[i] & REG_RD_ACCESS) {
			aw881xx_reg_read(aw881xx, i, &reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}
	return len;
}

static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, i2c_show, NULL);

static struct kobj_attribute dev_attr_test_re =
	__ATTR(test_re, 0664, test_re_show, NULL);

static struct kobj_attribute dev_attr_test_freq =
	__ATTR(test_freq, 0664, test_freq_show, NULL);

static struct kobj_attribute dev_attr_dsp_rw =
	__ATTR(reg_dsp_rw, S_IWUSR | S_IRUGO, aw881xx_dsp_rw_show, aw881xx_dsp_rw_store);
static struct kobj_attribute dev_attr_dsp = 
	__ATTR(reg_dsp, S_IWUSR | S_IRUGO, aw881xx_dsp_show, aw881xx_dsp_store);
static struct kobj_attribute dev_attr_spk_temp = 
	__ATTR(reg_spk_temp, S_IRUGO, aw881xx_spk_temp_show, NULL);
static struct kobj_attribute dev_attr_reg_common =
	__ATTR(reg, 0664, aw881xx_reg_show_common, aw881xx_reg_store_common);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_i2c.attr,
	&dev_attr_test_re.attr,
	&dev_attr_test_freq.attr,
	&dev_attr_dsp_rw.attr,
	&dev_attr_dsp.attr,
	&dev_attr_spk_temp.attr,
	&dev_attr_reg_common.attr,
	NULL
};

static struct attribute_group aw881xx_node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};

static int class_attr_create(struct aw881xx *aw881xx)
{
	int ret = -1;

	if (aw881xx == NULL) {
		pr_info("kobject_create_and_add audio-smartpa file faild\n");
		return ret;
	}

	aw881xx->k_obj = kobject_create_and_add("audio-smartpa", kernel_kobj);
	if (!aw881xx->k_obj) {
		pr_info("%s, kobject_create_and_add faild\n", __func__);
		return 0;
	}

	ret = sysfs_create_group(aw881xx->k_obj, &aw881xx_node_attribute_group);
	if (ret) {
		pr_info("%s, sysfs_create_group audio-smartpa file faild\n", __func__);
	}

	return ret;
}
//ljp add end


/* duanyitao add for vivo cali end */

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw881xx_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai = NULL;
	struct aw881xx *aw881xx = NULL;
	struct device_node *np = i2c->dev.of_node;
	const char *aw881xx_rst = "aw881xx_rst";
	const char *aw881xx_int = "aw881xx_int";
	const char *aw881xx_irq_name = "aw881xx";
	int irq_flags = 0;
	int ret = -1;
	const char *vivo_type;

	aw_dev_info(&i2c->dev, "%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		aw_dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}
	/*add by duanyitao start*/
	ret = dev_set_name(&i2c->dev, "%s-%x", "aw81xx", i2c->addr);
	if (ret || ERR_PTR(ret)) {
		dev_err(&i2c->dev, "dev set name failed\n");
		return ret;
	} else {
		dev_info(&i2c->dev, "dev set name successful\n");
	}
	/*add by duanyitao end*/

	aw881xx = devm_kzalloc(&i2c->dev, sizeof(struct aw881xx), GFP_KERNEL);
	if (aw881xx == NULL)
		return -ENOMEM;

	aw881xx->dev = &i2c->dev;
	aw881xx->i2c = i2c;
	aw881xx->codec_ops = &aw_componet_codec_ops;

	i2c_set_clientdata(i2c, aw881xx);
	mutex_init(&aw881xx->lock);
	mutex_init(&aw881xx->i2c_lock);

	/* aw881xx rst & int */
	if (np) {
		ret = aw881xx_parse_dt(&i2c->dev, aw881xx, np);
		if (ret < 0) {
			aw_dev_err(&i2c->dev, "%s:failed to parse device tree node\n",
				__func__);
			return ret;
		}
	} else {
		aw881xx->reset_gpio = -1;
		aw881xx->irq_gpio = -1;
	}

	if (gpio_is_valid(aw881xx->reset_gpio)) {
		ret = aw881xx_append_suffix("%s_%d_%x", &aw881xx_rst, aw881xx);
		if (ret < 0)
			return ret;

		ret = devm_gpio_request_one(&i2c->dev, aw881xx->reset_gpio,
					GPIOF_OUT_INIT_LOW, aw881xx_rst);
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			return ret;
		}
	}

	if (gpio_is_valid(aw881xx->irq_gpio)) {
		ret = aw881xx_append_suffix("%s_%d_%x", &aw881xx_int, aw881xx);
		if (ret < 0)
			return ret;

		ret = devm_gpio_request_one(&i2c->dev, aw881xx->irq_gpio,
						GPIOF_DIR_IN, aw881xx_int);
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			return ret;
		}
	}

	/* hardware reset */
	aw881xx_hw_reset(aw881xx);

	/* aw881xx chip id */
	ret = aw881xx_read_chipid(aw881xx);
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s: aw881xx_read_chipid failed ret=%d\n",
			__func__, ret);
		return ret;
	}

	/* register codec */
	dai = devm_kzalloc(&i2c->dev, sizeof(aw881xx_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	memcpy(dai, aw881xx_dai, sizeof(aw881xx_dai));
	ret = aw881xx_append_suffix("%s-%d-%x", &dai->name, aw881xx);
	if (ret < 0)
		return ret;
	ret = aw881xx_append_suffix("%s_%d_%x",
		&dai->playback.stream_name, aw881xx);
	if (ret < 0)
		return ret;
	ret = aw881xx_append_suffix("%s_%d_%x",
		&dai->capture.stream_name, aw881xx);
	if (ret < 0)
		return ret;

	ret = aw_componet_codec_ops.aw_snd_soc_register_codec(&i2c->dev,
					&soc_codec_dev_aw881xx,
					dai, ARRAY_SIZE(aw881xx_dai));
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s failed to register aw881xx: %d\n",
				__func__, ret);
		return ret;
	}
	mtk_spk_set_type(MTK_SPK_AW_AWINIC_AW88xxx);
	vivo_set_codec_name(dev_name(&i2c->dev));

	/*add by duanyitao for vivo cali start*/
	if(aw_pa_dev == NULL){
		aw_pa_dev = aw881xx;
		aw881xx_cali_ops.vivo_smartpa_init_dbg = aw881xx_smartpa_init_dbg;
		aw881xx_cali_ops.vivo_smartpa_read_freq_dbg = aw881xx_smartpa_read_freq_dbg;
		aw881xx_cali_ops.vivo_smartpa_check_calib_dbg = aw881xx_smartpa_check_calib_dbg;
	
		//vivo add start by linjinping, operating smartamp.bin file on user layer to solve gki unsymbol problem
		aw881xx_cali_ops.vivo_smartpa_init_calib_data = aw881xx_smartpa_init_calib_data;
		aw881xx_cali_ops.vivo_smartpa_get_calib_data = aw881xx_smartpa_get_calib_data;
		//vivo add end
	
		//vivo linjinping add start for calib fail mute check
		aw881xx_cali_ops.vivo_smartpa_cali_fail_mute_enable = aw881xx_smartpa_cali_fail_mute_enable;
		//vivo linjinping add end
	
		//set_vivo_cali_function(&aw881xx_cali_ops);
		vivo_smartpa_debug_probe(&aw881xx_cali_ops);
		//ljp add begin, for AT+BK_AUD_HIFI=1 test
		class_attr_create(aw881xx);
		//ljp add end
	}
	ret = of_property_read_string(np, "vivo,Type", &vivo_type);
	if ((strcmp(vivo_type, "speaker")) == 0) {
		vivo_smartpa_parse_dt(i2c, &vivo_spk_param);
		dump_param_info(&vivo_spk_param);
		cali_channels++;
	} else if ((strcmp(vivo_type, "receiver")) == 0) {
		vivo_smartpa_parse_dt(i2c, &vivo_rcv_param);
		dump_param_info(&vivo_rcv_param);
		cali_channels++;
	} else {
		aw_dev_err(&i2c->dev, "vivo,Type is %s\n", vivo_type);
	}
	aw_dev_info(&i2c->dev, "cali_channels %d", cali_channels);
	/*add by duanyitao for vivo cali end*/

	/* aw881xx irq */
	if (gpio_is_valid(aw881xx->irq_gpio) &&
		!(aw881xx->flags & AW881XX_FLAG_SKIP_INTERRUPTS)) {
		ret = aw881xx_append_suffix("%s_%d_%x",
			&aw881xx_irq_name, aw881xx);
		if (ret < 0)
			goto err_irq_append_suffix;
		aw881xx_interrupt_setup(aw881xx);
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw881xx->irq_gpio),
						NULL, aw881xx_irq, irq_flags,
						aw881xx_irq_name, aw881xx);
		if (ret != 0) {
			aw_dev_err(&i2c->dev,
				"failed to request IRQ %d: %d\n",
				gpio_to_irq(aw881xx->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		aw_dev_info(&i2c->dev, "%s skipping IRQ registration\n",
				__func__);
		/* disable feature support if gpio was invalid */
		aw881xx->flags |= AW881XX_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw881xx);
	ret = sysfs_create_group(&i2c->dev.kobj, &aw881xx_attribute_group);
	if (ret < 0) {
		aw_dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
				__func__);
		goto err_sysfs;
	}
	aw881xx_cali_init(&aw881xx->cali_desc);
	aw881xx_monitor_init(&aw881xx->monitor);
	aw881xx_spin_init(&aw881xx->spin_desc);

	aw881xx->allow_pw = true;
	aw881xx->work_queue = NULL;
	aw881xx->fw_status = AW881XX_FW_FAILED;

	mutex_lock(&g_aw881xx_lock);
	g_aw881xx_dev_cnt++;
	mutex_unlock(&g_aw881xx_lock);
	aw_dev_info(&i2c->dev, "%s: dev_cnt %d completed successfully\n",
		__func__, g_aw881xx_dev_cnt);

	return 0;

err_sysfs:
err_irq:
err_irq_append_suffix:
	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);
	return ret;
}

static int aw881xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw881xx *aw881xx = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);

	aw881xx_cali_deinit(&aw881xx->cali_desc);
	aw881xx_monitor_deinit(&aw881xx->monitor);
	aw881xx_device_deinit(aw881xx);
	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);

	mutex_lock(&g_aw881xx_lock);
	g_aw881xx_dev_cnt--;
	if (g_aw881xx_dev_cnt == 0) {
		if (g_awinic_cfg != NULL) {
			vfree(g_awinic_cfg);
			g_awinic_cfg = NULL;
		}
	}
	mutex_unlock(&g_aw881xx_lock);

	return 0;
}

static const struct i2c_device_id aw881xx_i2c_id[] = {
	{AW881XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw881xx_i2c_id);

static struct of_device_id aw881xx_dt_match[] = {
	{.compatible = "awinic,aw881xx_smartpa"},
	{},
};

static struct i2c_driver aw881xx_i2c_driver = {
	.driver = {
		.name = AW881XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw881xx_dt_match),
		},
	.probe = aw881xx_i2c_probe,
	.remove = aw881xx_i2c_remove,
	.id_table = aw881xx_i2c_id,
};

static int __init aw881xx_i2c_init(void)
{
	int ret = -1;

	pr_info("%s: aw881xx driver version %s\n",
			__func__, AW881XX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw881xx_i2c_driver);
	if (ret)
		pr_err("%s: fail to add aw881xx device into i2c\n", __func__);

	return ret;
}

module_init(aw881xx_i2c_init);

static void __exit aw881xx_i2c_exit(void)
{
	i2c_del_driver(&aw881xx_i2c_driver);
}

module_exit(aw881xx_i2c_exit);

MODULE_DESCRIPTION("ASoC AW881XX Smart PA Driver");
MODULE_LICENSE("GPL v2");
