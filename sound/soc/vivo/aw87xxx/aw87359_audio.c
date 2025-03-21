/*
 * aw87359.c   aw87359 pa module
 *
 * Version: v1.0.4
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Joseph <zhangzetao@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include "aw87359_audio.h"
#include <linux/debugfs.h>
#include "vivo-codec-common.h"

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif

/*****************************************************************
* aw87359 marco
******************************************************************/
#define AW87359_I2C_NAME        "aw87359_pa"

#define AW87359_DRIVER_VERSION  "v1.0.4"


/*************************************************************************
 * aw87359 variable
 ************************************************************************/
static bool chipid_ok;
static struct i2c_driver aw87359_i2c_driver;

#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *aw87359_debugfs_root;
static struct dentry *aw87359_debugfs_reg;
static struct dentry *aw87359_debugfs_i2c;
#endif

struct aw87359 *aw87359;
struct aw87359_container *aw87359_dspk_cnt;
struct aw87359_container *aw87359_drcv_cnt;
struct aw87359_container *aw87359_abspk_cnt;
struct aw87359_container *aw87359_abrcv_cnt;

static char *aw87359_dspk_name = "aw87359_music.bin";
static char *aw87359_drcv_name = "aw87359_voice.bin";
static char *aw87359_abspk_name = "aw87359_abspk.bin";
static char *aw87359_abrcv_name = "aw87359_fm.bin";

unsigned int dspk_load_cont;
unsigned int drcv_load_cont;
unsigned int abspk_load_cont;
unsigned int abrcv_load_cont;
/**********************************************************
* i2c write and read
**********************************************************/
static int aw87359_i2c_write(struct aw87359 *aw87359,
	unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw87359->i2c_client,
			reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw87359_i2c_read(struct aw87359 *aw87359,
		 unsigned char reg_addr, unsigned char *reg_data)
{
	 int ret = -1;
	 unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw87359->i2c_client,
						reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/************************************************************************
* aw87359 hardware control
************************************************************************/
unsigned int aw87359_hw_on(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);

	aw87359->hwen_flag = 1;

	return 0;
}

unsigned int aw87359_hw_off(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);

	aw87359->hwen_flag = 0;

	return 0;
}


/*******************************************************************************
* aw87359 control interface
******************************************************************************/
unsigned char aw87359_audio_dspk(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87359 == NULL)
		return 2;

	if (!aw87359->hwen_flag)
		aw87359_hw_on(aw87359);

	length = sizeof(aw87359_dspk_cfg_default)/sizeof(char);
	if (aw87359->dspk_cfg_update_flag == 0) { /*update default data*/
		for (i = 0; i < length; i = i+2) {
			aw87359_i2c_write(aw87359,
			aw87359_dspk_cfg_default[i],
			aw87359_dspk_cfg_default[i+1]);
		}
	}

	if (aw87359->dspk_cfg_update_flag == 1) {  /*update firmware data*/
		for (i = 0; i < aw87359_dspk_cnt->len; i = i+2) {
			aw87359_i2c_write(aw87359,
					aw87359_dspk_cnt->data[i],
					aw87359_dspk_cnt->data[i+1]);

		}
	}

	return 0;
}

unsigned char aw87359_audio_drcv(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87359 == NULL)
		return 2;
	if (!aw87359->hwen_flag)
		aw87359_hw_on(aw87359);

	length = sizeof(aw87359_drcv_cfg_default)/sizeof(char);
	if (aw87359->drcv_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87359_i2c_write(aw87359,
				aw87359_drcv_cfg_default[i],
				aw87359_drcv_cfg_default[i+1]);
		}
	}

	if (aw87359->drcv_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87359_drcv_cnt->len; i = i+2) {
			aw87359_i2c_write(aw87359,
					aw87359_drcv_cnt->data[i],
					aw87359_drcv_cnt->data[i+1]);

		}
	}

	return 0;
}

unsigned char aw87359_audio_abspk(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87359 == NULL)
		return 2;

	if (!aw87359->hwen_flag)
		aw87359_hw_on(aw87359);


	length = sizeof(aw87359_abspk_cfg_default)/sizeof(char);
	if (aw87359->abspk_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87359_i2c_write(aw87359,
				aw87359_abspk_cfg_default[i],
				aw87359_abspk_cfg_default[i+1]);
		}
	}

	if (aw87359->abspk_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87359_abspk_cnt->len; i = i+2) {
			aw87359_i2c_write(aw87359,
					aw87359_abspk_cnt->data[i],
					aw87359_abspk_cnt->data[i+1]);

		}
	}

	return 0;
}

unsigned char aw87359_audio_abrcv(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87359 == NULL)
		return 2;

	if (!aw87359->hwen_flag)
		aw87359_hw_on(aw87359);


	length = sizeof(aw87359_abrcv_cfg_default)/sizeof(char);
	if (aw87359->abrcv_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87359_i2c_write(aw87359,
					aw87359_abrcv_cfg_default[i],
					aw87359_abrcv_cfg_default[i+1]);
		}
	}

	if (aw87359->abrcv_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87359_abrcv_cnt->len; i = i+2) {
			aw87359_i2c_write(aw87359,
					aw87359_abrcv_cnt->data[i],
					aw87359_abrcv_cnt->data[i+1]);

		}
	}

	return 0;
}

unsigned char aw87359_audio_off(void)
{
	if (aw87359 == NULL)
		return 2;

	if (aw87359->hwen_flag)
		aw87359_i2c_write(aw87359, 0x01, 0x00);   /*CHIP Disable*/

	aw87359_hw_off(aw87359);
	return 0;
}


/****************************************************************************
* aw87359 firmware cfg update
***************************************************************************/
static void aw87359_abrcv_cfg_loaded(const struct firmware *cont,
			void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	abrcv_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
		aw87359_abrcv_name);
		release_firmware(cont);
		if (abrcv_load_cont <= 2) {
			schedule_delayed_work(&aw87359->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87359_abrcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	/* aw87359 ram update */
	aw87359_abrcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87359_abrcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87359_abrcv_cnt->len = cont->size;
	memcpy(aw87359_abrcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87359->abrcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static int aw87359_abrcv_update(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87359_abrcv_name,
					&aw87359->i2c_client->dev,
					GFP_KERNEL,
					aw87359,
					aw87359_abrcv_cfg_loaded);
}

static void aw87359_abspk_cfg_loaded(const struct firmware *cont,
			void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	abspk_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
				aw87359_abspk_name);
		release_firmware(cont);
		if (abspk_load_cont <= 2) {
			schedule_delayed_work(&aw87359->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87359_abspk_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	/* aw87359 ram update */
	aw87359_abspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87359_abspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87359_abspk_cnt->len = cont->size;
	memcpy(aw87359_abspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87359->abspk_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static int aw87359_abspk_update(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87359_abspk_name,
					&aw87359->i2c_client->dev,
					GFP_KERNEL,
					aw87359,
					aw87359_abspk_cfg_loaded);
}

static void aw87359_drcv_cfg_loaded(const struct firmware *cont, void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	drcv_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87359_drcv_name);
		release_firmware(cont);
		if (drcv_load_cont <= 2) {
			schedule_delayed_work(&aw87359->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
				__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87359_drcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87359_drcv_cnt != NULL)
		aw87359_drcv_cnt = NULL;

	/* aw87359 ram update */
	aw87359_drcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87359_drcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87359_drcv_cnt->len = cont->size;
	memcpy(aw87359_drcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87359->drcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static int aw87359_drcv_update(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87359_drcv_name,
					&aw87359->i2c_client->dev,
					GFP_KERNEL,
					aw87359,
					aw87359_drcv_cfg_loaded);
}

static void aw87359_dspk_cfg_loaded(const struct firmware *cont, void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);
	dspk_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87359_dspk_name);
		release_firmware(cont);
		if (dspk_load_cont <= 2) {
			schedule_delayed_work(&aw87359->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
	return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87359_dspk_name,
				cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%02x, data:0x%02x\n",
				__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87359_dspk_cnt != NULL)
		aw87359_dspk_cnt = NULL;

	/* aw87359 ram update */
	aw87359_dspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87359_dspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87359_dspk_cnt->len = cont->size;
	memcpy(aw87359_dspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87359->dspk_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}



#ifdef AWINIC_CFG_UPDATE_DELAY
static int aw87359_dspk_update(struct aw87359 *aw87359)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87359_dspk_name,
					&aw87359->i2c_client->dev,
					GFP_KERNEL,
					aw87359,
					aw87359_dspk_cfg_loaded);
}

static void aw87359_cfg_work_routine(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);
	if (aw87359->dspk_cfg_update_flag == 0)
		aw87359_dspk_update(aw87359);
	if (aw87359->drcv_cfg_update_flag == 0)
		aw87359_drcv_update(aw87359);
	if (aw87359->abspk_cfg_update_flag == 0)
		aw87359_abspk_update(aw87359);
	if (aw87359->abrcv_cfg_update_flag == 0)
		aw87359_abrcv_update(aw87359);
}
#endif

static int aw87359_cfg_init(struct aw87359 *aw87359)
{
#ifdef AWINIC_CFG_UPDATE_DELAY
	int cfg_timer_val = 5000;

	INIT_DELAYED_WORK(&aw87359->ram_work, aw87359_cfg_work_routine);
	schedule_delayed_work(&aw87359->ram_work,
		msecs_to_jiffies(cfg_timer_val));
#else
	int cfg_timer_val = 0;

	INIT_DELAYED_WORK(&aw87359->ram_work, aw87359_cfg_work_routine);
	schedule_delayed_work(&aw87359->ram_work,
		msecs_to_jiffies(cfg_timer_val));

#endif
	return 0;
}
/****************************************************************************
* aw87359 attribute
*****************************************************************************/
static ssize_t aw87359_get_reg(struct device *dev,
		   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW87359_REG_MAX; i++) {
		if (!(aw87359_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw87359_i2c_read(aw87359, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t aw87359_set_reg(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw87359_i2c_write(aw87359, databuf[0], databuf[1]);

	return len;
}


static ssize_t aw87359_get_hwen(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "hwen: %d\n",
			aw87359->hwen_flag);

	return len;
}

static ssize_t aw87359_set_hwen(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0) {		/*OFF*/
		aw87359_hw_off(aw87359);
	} else {			/*ON*/
		aw87359_hw_on(aw87359);
	}

	if (ret < 0)
		goto out;

	return len;

out:
	dev_err(&aw87359->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87359->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static ssize_t aw87359_get_update(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}

static ssize_t aw87359_set_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;
	int cfg_timer_val = 10;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0) {
	} else {
		aw87359->dspk_cfg_update_flag = 1;
		aw87359->drcv_cfg_update_flag = 1;
		aw87359->abspk_cfg_update_flag = 1;
		aw87359->abrcv_cfg_update_flag = 1;
		schedule_delayed_work(&aw87359->ram_work,
				msecs_to_jiffies(cfg_timer_val));
	}

	if (ret < 0)
		goto out;

	return len;
out:
	dev_err(&aw87359->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87359->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static ssize_t aw87359_get_mode(struct device *cd,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "0: off mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "1: dspk mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "2: drcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "3: abspk mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "4: abrcv mode\n");

	return len;
}

static ssize_t aw87359_set_mode(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0)
		aw87359_audio_off();
	else if (state == 1)
		aw87359_audio_dspk();
	else if (state == 2)
		aw87359_audio_drcv();
	else if (state == 3)
		aw87359_audio_abspk();
	else if (state == 4)
		aw87359_audio_abrcv();
	else
		aw87359_audio_off();

	if (ret < 0)
		goto out;

	return len;
out:
	dev_err(&aw87359->i2c_client->dev, "%s: i2c access fail to register\n",
		__func__);
out_strtoint:
	dev_err(&aw87359->i2c_client->dev, "%s: fail to change str to int\n",
		__func__);
	return ret;
}

static DEVICE_ATTR(reg, 0660, aw87359_get_reg, aw87359_set_reg);
static DEVICE_ATTR(hwen, 0660, aw87359_get_hwen, aw87359_set_hwen);
static DEVICE_ATTR(update, 0660, aw87359_get_update, aw87359_set_update);
static DEVICE_ATTR(mode, 0660, aw87359_get_mode, aw87359_set_mode);

static struct attribute *aw87359_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_update.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group aw87359_attribute_group = {
	.attrs = aw87359_attributes
};

static int aw87359_pa_enable(int state)
{
	int ret = 0;

	pr_info("%s: enter, state: %d\n",
		__func__, state);

	switch (state) {
	case EXT_PA_SWICH_VOICE:
		ret = aw87359_audio_drcv();
		break;
	case EXT_PA_SWICH_MUSIC:
		ret = aw87359_audio_dspk();
		break;
	case EXT_PA_SWICH_FM:
		ret = aw87359_audio_abrcv();
		break;
	case EXT_PA_SWICH_NONE:
		ret = aw87359_audio_off();
		break;
	default:
		ret = aw87359_audio_off();
		break;
	}

	pr_info("%s: leave.\n", __func__);

	return ret;
}
/*****************************************************
* check chip id
*****************************************************/
int aw87359_read_chipid(struct aw87359 *aw87359)
{
	unsigned int cnt = 0;
	int ret = -1;
	unsigned char reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		aw87359_i2c_write(aw87359, 0x64, 0x2C);
		ret = aw87359_i2c_read(aw87359, REG_CHIPID, &reg_val);
		if (reg_val == AW87359_CHIPID) {
			pr_info("%s: This Chip is aw87359 chipid=0x%x\n",
					__func__, reg_val);
			chipid_ok = true;
			return 0;
		}
		cnt++;

		mdelay(AW_READ_CHIPID_RETRY_DELAY);
	}
	pr_info("%s: aw87359 chipid=0x%x error\n", __func__, reg_val);
	return -EINVAL;
}

#ifdef BBK_IQOO_AUDIO_DEBUG
static int aw87359_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t aw87359_debug_write(struct file *filp,
					const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)cnt);
	aw87359_i2c_write(aw87359,
			kbuf[0],
			kbuf[1]);

	kfree(temp);

	return cnt;
}
static ssize_t aw87359_debug_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	unsigned char reg_val;
	const int size = 512;
	char buffer[size];
	int len = 0;
	u8 i;

	for (i = 0; i < AW87359_REG_MAX; i++) {
		if (!(aw87359_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw87359_i2c_read(aw87359, i, &reg_val);
		len += snprintf(buf+len, size-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	buffer[len] = 0;
	pr_info("%s:============caught aw87359 reg end =============\n", __func__);

	return simple_read_from_buffer(buf, count, pos, buffer, len);
}

static struct file_operations aw87359_debugfs_fops = {
	.open = aw87359_debug_open,
	.read = aw87359_debug_read,
	.write = aw87359_debug_write,
};

static ssize_t aw87359_debug_i2c_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("%s enter.\n", __func__);

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n", aw87359->i2c_client->addr,
					chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87359_i2c_debugfs_fops = {
	.open = aw87359_debug_open,
	.read = aw87359_debug_i2c_read,
};

static void aw87359_debugfs_init(void)
{
	aw87359_debugfs_root = debugfs_create_dir("audio-aw87359", NULL);
	if (!aw87359_debugfs_root) {
		pr_err("%s debugfs create dir error\n", __func__);
	} else if (IS_ERR(aw87359_debugfs_root)) {
		pr_err("%s Kernel not support debugfs \n", __func__);
		aw87359_debugfs_root = NULL;
	}

	aw87359_debugfs_reg = debugfs_create_file("reg", 0644,
		aw87359_debugfs_root, NULL, &aw87359_debugfs_fops);
	if (!aw87359_debugfs_reg) {
		pr_err("aw87359 debugfs create fail \n");
	}
	aw87359_debugfs_i2c = debugfs_create_file("i2c", 0444,
		aw87359_debugfs_root, NULL, &aw87359_i2c_debugfs_fops);
	if (!aw87359_debugfs_i2c) {
		pr_err("aw87359 i2c create fail \n");
	}

	return ;
}

static void aw87359_debugfs_deinit(void)
{
	debugfs_remove(aw87359_debugfs_i2c);
	debugfs_remove(aw87359_debugfs_reg);
	debugfs_remove(aw87359_debugfs_root);
	return ;
}
#endif


/****************************************************************************
* aw87359 i2c driver
*****************************************************************************/
static int aw87359_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *vivo_codec_function = get_vivo_codec_function();
	int ret = -1;

	pr_info("%s Enter\n", __func__);

	if (!vivo_codec_function) {
		pr_err("%s:vivo_codec_function malloc failed\n", __func__);
		return -EPROBE_DEFER;
	}

	//client->addr = 0x58;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: check_functionality failed\n",
			__func__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw87359 = devm_kzalloc(&client->dev,
				sizeof(struct aw87359),
				GFP_KERNEL);
	if (aw87359 == NULL) {
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	aw87359->i2c_client = client;
	i2c_set_clientdata(client, aw87359);

	/* aw87359 chip id */
	ret = aw87359_read_chipid(aw87359);
	if (ret < 0) {
		dev_err(&client->dev, "%s: aw87359_read_chipid failed ret=%d\n",
			__func__, ret);
		goto exit_i2c_check_id_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &aw87359_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			__func__);
	}

	/* aw87359 cfg update */
	dspk_load_cont = 0;
	drcv_load_cont = 0;
	abspk_load_cont = 0;
	abrcv_load_cont = 0;
	aw87359->dspk_cfg_update_flag = 0;
	aw87359->drcv_cfg_update_flag = 0;
	aw87359->abspk_cfg_update_flag = 0;
	aw87359->abrcv_cfg_update_flag = 0;
	aw87359_cfg_init(aw87359);

	vivo_codec_function->ext_pa_enable = aw87359_pa_enable;
#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87359_debugfs_init();
#endif
	/* aw87359 hardware off */
	aw87359_hw_off(aw87359);

	return 0;

exit_i2c_check_id_failed:
	aw87359 = NULL;

exit_devm_kzalloc_failed:
exit_check_functionality_failed:
	i2c_del_driver(&aw87359_i2c_driver);

	return ret;
}

static int aw87359_i2c_remove(struct i2c_client *client)
{
#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87359_debugfs_deinit();
#endif
	return 0;
}

static const struct i2c_device_id aw87359_i2c_id[] = {
	{ AW87359_I2C_NAME, 0 },
	{ }
};


static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87359_pa"},
	{},
};


static struct i2c_driver aw87359_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AW87359_I2C_NAME,
		.of_match_table = extpa_of_match,
	},
	.probe = aw87359_i2c_probe,
	.remove = aw87359_i2c_remove,
	.id_table	= aw87359_i2c_id,
};

static int __init aw87359_pa_init(void)
{
	int ret;

	pr_info("%s enter\n", __func__);
	pr_info("%s: driver version: %s\n", __func__, AW87359_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87359_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n",
				__func__, ret);
		return ret;
	}
	return 0;
}

static void __exit aw87359_pa_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&aw87359_i2c_driver);
}

module_init(aw87359_pa_init);
module_exit(aw87359_pa_exit);

MODULE_AUTHOR("<zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("awinic aw87359 pa driver");
MODULE_LICENSE("GPL v2");
