/*
 * awinic_cali.c cali_module
 *
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
#include <linux/debugfs.h>
#include <asm/ioctls.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include "aw882xx.h"
#include "awinic_cali.h"
#include "awinic_monitor.h"
#include "awinic_dsp.h"
#include "aw882xx_reg.h"


uint8_t g_cali_status;

static int vivo_smartpa_calib_save(uint32_t *calib_value);
#ifdef CONFIG_VIVO_SMARTPA_NEW
#include "../vivo_smartpa_cali.h"

#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"

#define CHANNEL_NUMS 1
static struct aw882xx *smartpa_priv;
static struct smartpa_param_L15 param_priv;
static bool update_cali_flag;

static int vivo_smartpa_init_dbg(char *buffer, int size);
static int vivo_smartpa_read_freq_dbg(char *buffer, int size);
static int vivo_smartpa_check_calib_dbg(void);
static void vivo_smartpa_read_prars_dbg(int temp[5], unsigned char addr);

static struct vivo_cali_ops cali_ops = {
	.vivo_smartpa_init_dbg = vivo_smartpa_init_dbg,
	.vivo_smartpa_read_freq_dbg = vivo_smartpa_read_freq_dbg,
	.vivo_smartpa_check_calib_dbg = vivo_smartpa_check_calib_dbg,
	.vivo_smartpa_read_prars_dbg = vivo_smartpa_read_prars_dbg,
};

static void dump_param_info(struct smartpa_param_L15 *param) {
	printk("[SmartPa] %s : dump info==>", __func__);
	printk("[SmartPa] vivo,Impedance-Min-L15 :%u\n", param->Imped_Min_L15);
	printk("[SmartPa] vivo,Impedance-Max-L15 :%u\n", param->Imped_Max_L15);
	printk("[SmartPa] vivo,Frequency-Min-L15 :%u\n", param->Freq_Min_L15);
	printk("[SmartPa] vivo,Frequency-Max-L15 :%u\n", param->Freq_Max_L15);
	printk("[SmartPa] vivo,Qt-Min-L15        :%u\n", param->Qt_Min_L15);
	printk("[SmartPa] vivo,VendorID          :%u\n", param->PA_ID);
	printk("[SmartPa] vivo,V-Max-L15         :%u\n", param->V_Max_L15);
	printk("[SmartPa] vivo,I-Max-L15         :%u\n", param->I_Max_L15);
	printk("[SmartPa] vivo,Vout-Max-L15      :%u\n", param->Vout_Max_L15);
	printk("[SmartPa] Calibration_Success    :%u\n", param->Calibration_Success);
	printk("[SmartPa] Re_L15                 :%u\n", param->Re_L15);
	printk("[SmartPa] F0_L15                 :%u\n", param->F0_L15);
	printk("[SmartPa] Qts_L15                :%u\n", param->Qts_L15);
}

static int vivo_smartpa_calib_get(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	int i = 0;
	if (!calib_value) {
		pr_err("[SmartPA] there is no space to store file info\n");
		return -1;
	}
	
	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(pfile)) {
		pr_err("[SmartPA] calibrate: %s not found\n", CALIBRATE_FILE);
		found = 0;
	} else {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNEL_NUMS, &pos);
		for (i = 0; i < CHANNEL_NUMS; i++) {
			pr_info("[SmartPA] calibrate:get calib_value[%d] = %u  \n", i, calib_value[i]);
		}
		#if 1 //audio_v:  archer fix for replace aw protection algorithm with vivo algorithm 
		if(smartpa_priv->is_need_fix == true) {
			if(calib_value[0] <10000 && calib_value[0]>5000) {
				calib_value[0] = 219402;
				//pos=0;
				//ret=vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNEL_NUMS, &pos);
				pr_info("Warning!!! [SmartPA] calibrate: force set calib_value[0] = %u\n", calib_value[0]);
			}
		}
		#endif
		filp_close(pfile, NULL);
	}
	set_fs(old_fs);
	return found;
}

static void parse_calib_re(uint32_t calib_re)
{
	if (calib_re == 0 || ((calib_re >= param_priv.Imped_Min_L15) && (calib_re <= param_priv.Imped_Max_L15))) {
		param_priv.Calibration_Success = 1;
		param_priv.Re_L15 = calib_re;
	} else {
		param_priv.Calibration_Success = 0;
		param_priv.Re_L15 = CALIB_FAILED;
	}
}

void aw_vivo_adsp_send_params(void)
{ 
	uint32_t paramid = 0;
	struct CALIBRATION_RX_ calInfo;
	int ret = 0, iter = 0;
	uint32_t calib_re = 0;

	if (update_cali_flag == false) {
		vivo_smartpa_calib_get(&calib_re);
		parse_calib_re(calib_re);
	}

	dump_param_info(&param_priv);

	calInfo.Calibration_Success = param_priv.Calibration_Success;
	calInfo.PA_ID = param_priv.PA_ID;
	calInfo.Re_L15 = param_priv.Re_L15;
	calInfo.F0_L15 = param_priv.F0_L15;
	calInfo.Qts_L15 = param_priv.Qts_L15;
	calInfo.V_Max_L15 = param_priv.V_Max_L15;
	calInfo.I_Max_L15 = param_priv.I_Max_L15;
	calInfo.Vout_Max_L15 = param_priv.Vout_Max_L15;
	
	paramid = ((AFE_SA_SEND_ALL)|((iter+1)<<24)|((iter+1)<<16));
	ret = mtk_afe_smartamp_algo_ctrl((void *)&calInfo, paramid, AP_2_DSP_SEND_PARAM, sizeof(struct CALIBRATION_RX_));
	if (ret < 0) {
		pr_err("[smartPA-%s-%d]: mtk_afe_smartamp_algo_ctrl dsp param send error!\n", __func__, iter);
	} else {
		pr_info("[smartPA-%s-%d]: mtk_afe_smartamp_algo_ctrl dsp param send successful!\n", __func__, iter);
	}

	update_cali_flag == true;
}

static int vivo_smartpa_parse_dt(void)
{
	unsigned int temp = 0;
	int ret = 0;
	pr_info("[SmartPA]: %s enter.\n", __func__);

	if (smartpa_priv == NULL || smartpa_priv->dev == NULL || smartpa_priv->dev->of_node == NULL) {
		pr_err("[SmartPA]: %s smartpa private pointer is invalid\n", __func__);
		return -1;
	}
#if 1 //audio_v:  archer fix for replace aw protection algorithm with vivo algorithm 
	if ( of_find_property(smartpa_priv->dev->of_node, "vivo-replace-protect-algo", NULL)){
		smartpa_priv->is_need_fix = true;
		pr_info("%s: have define vivo-replace-protect-algo\n", __func__);
	}else{
		smartpa_priv->is_need_fix = false;
	}
#endif

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Impedance-Min-L15", &temp);
	param_priv.Imped_Min_L15 = (!ret) ? temp:163840;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Impedance-Max-L15", &temp);
	param_priv.Imped_Max_L15 = (!ret) ? temp:327680;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Frequency-Min-L15", &temp);
	param_priv.Freq_Min_L15 = (!ret) ? temp:16384000;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Frequency-Max-L15", &temp);
	param_priv.Freq_Max_L15 = (!ret) ? temp:32768000;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Qt-Min-L15", &temp);
	param_priv.Qt_Min_L15 = (!ret) ? temp:32768;;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,VendorID", &temp);
	param_priv.PA_ID = (!ret) ? temp:63;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,V-Max-L15", &temp);
	param_priv.V_Max_L15 = (!ret) ? temp:589824;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,I-Max-L15", &temp);
	param_priv.I_Max_L15 = (!ret) ? temp:120259;

	ret = of_property_read_u32(smartpa_priv->dev->of_node, "vivo,Vout-Max-L15", &temp);
	param_priv.Vout_Max_L15 = (!ret) ? temp:262144;

	param_priv.Calibration_Success = 1;
	param_priv.Re_L15 = 0;
	param_priv.F0_L15 = 0;
	param_priv.Qts_L15 = 0;

	// dump_param_info(&param_priv);

	return ret;
}

static int vivo_smartpa_calib_save(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	pr_info("[SmartPA] %s: enter ===>\n", __func__);
	if (!calib_value) {
		pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n", __LINE__);
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA] smartpa_calib_save: save calib_value[0]=%d \n", calib_value[0]);
		if (CHANNEL_NUMS == 2)
			pr_info("[SmartPA] smartpa_calib_save: save calib_value[1]=%d \n", calib_value[1]);
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNEL_NUMS, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA] smartpa_calib_save: %s open failed! \n", CALIBRATE_FILE);
		ret = -1;
	}
	set_fs(old_fs);
	pr_info("[SmartPA] %s: leave<===\n", __func__);
	return ret;
}

static int vivo_smartpa_freq_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d \n", __LINE__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __LINE__, FREQ_FILE);
		ret = -1;
	}
	set_fs(old_fs);
	return ret;
}


static int vivo_smartpa_init_dbg(char *buffer, int size)
{
	int  n = 0;
	struct smartpa_param_L15 *privPointer = NULL;
	int iter = 0;
	uint32_t paramid = 0;
	uint32_t data = 0;
	uint32_t flag = 0;
	bool done = false;
	int ret = 0;
	int nSize = sizeof(uint32_t);
	uint32_t calib_re = 0;
	
	pr_info("[SmartPA]: enter======> %s\n", __func__);

	privPointer = &param_priv;

	data = 1;
	paramid = ((AFE_SA_CALIB_INIT)|((iter+1)<<24)|((iter+1)<<16)); //(3814 | 1<<24 | 1<<16) = 16846566
	pr_info("[SmartPA] now startup cali for smartPA-%d\n", iter);
	ret = mtk_afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		done = false;
		pr_err("[SmartPA] startup cali for smartPA-%d error\n", iter);
	} else {
		pr_info("[SmartPA] startup cali for smartPA-%d successful\n", iter);
		done = true;
	}

	msleep(3000);
	
	if (done == false) {
		privPointer->Calibration_Success = 1;
		privPointer->Re_L15 = 0;
	} else {
		data = 0;
		paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|((iter+1)<<16)); //16846565
		pr_info("[SmartPA] now get calibrate result for smartPA-%d\n", iter);
		ret = mtk_afe_smartamp_algo_ctrl((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, 4);
		if (ret < 0) {
			pr_err("[SmartPA] get calibrate result for smartPA-%d error\n", iter);
			done = false;
			privPointer->Calibration_Success = 0;
			privPointer->Re_L15 = CALIB_FAILED;
		} else { 
			if (data < privPointer->Imped_Min_L15 || data > privPointer->Imped_Max_L15) {
				pr_err("[SmartPA] calibrate result for smartPA-%d is invalid\n", iter);
				done = false;
				privPointer->Calibration_Success = 0;
				privPointer->Re_L15 = CALIB_FAILED;
			} else {
				pr_info("[SmartPA] get calibrate result for smartPA-%d successful, value is %u\n", iter, data);
				privPointer->Calibration_Success = 1;
				privPointer->Re_L15 = data;
			}
		}
	}	

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (CHANNEL_NUMS == 1) ? "Mono" : "Stereo");
	
	if (privPointer->Re_L15 != CALIB_FAILED) {
		n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %d.%02d ohm, ", iter,
			VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Re_L15), VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Re_L15));
	} else {
		n += scnprintf(buffer + n, size - n, "Channel[%d]: impedance %#X ohm, ", iter, CALIB_FAILED);
	}

	n += scnprintf(buffer + n, size - n, " valid range(%d.%02d ~ %d.%02d ohm).\n",
		VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Imped_Min_L15), VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Imped_Min_L15),
		VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Imped_Max_L15), VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Imped_Max_L15));
	
	if (done == true)
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "OKAY(impedance ok).\n");
	else
		n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", "ERROR(impedance error).\n");

	buffer[n] = 0;

	
	data = privPointer->Re_L15;
	paramid = 17043176; //send Re
	pr_info("[SmartPA] now start send Re for smartPA-%d!\n", iter);
	ret = mtk_afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		pr_err("[SmartPA] send Re for smartPA-%d error!\n", iter);
	} else {
		pr_info("[SmartPA] send Re for smartPA-%d successful!\n", iter);
	}

	flag = privPointer->Calibration_Success;
	paramid = ((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|((iter+1)<<16)); //16846567
	pr_info("[SmartPA] now start send calibration_Success flag for smartPA-%d\n", iter);
	ret = mtk_afe_smartamp_algo_ctrl((uint8_t *)&flag, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		pr_err("[SmartPA] send calibration_Success flag for smartPA-%d error\n", iter);
	} else {
		pr_info("[SmartPA] send calibration_Success flag for smartPA-%d successful\n", iter);
	}	

	calib_re = privPointer->Re_L15;
	pr_err("[SmartPA] %s calib_re is %u\n", __func__, calib_re);
	ret = vivo_smartpa_calib_save(&calib_re);
	update_cali_flag = false;
	
	if (ret < 0) {
		pr_err("[SmartPA] store calibration result error\n");
	} else {
		pr_info("[SmartPA] store calibration result successful\n");
	}
	pr_info("[SmartPA]: leave<====== %s\n", __func__);

	return !done;
}

static int vivo_smartpa_read_freq_dbg(char *buffer, int size)
{
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	int nSize = sizeof(uint32_t);
	int iter = 0;
	bool done = false;
	struct smartpa_param_L15 *privPointer = NULL;
	int i = 0, j = 0, result = 1;
	uint32_t calib_re = 0;

	pr_info("[SmartPA]: enter======> %s\n", __func__);

	privPointer = &param_priv;

	if (privPointer->Re_L15 == 0) {
		ret = vivo_smartpa_calib_get(&calib_re);
		if (ret < 0) {
			pr_err("[SmartPA] get calibration result error\n");
		} else {
			pr_info("[SmartPA] get calibration result successful\n");
		}
		parse_calib_re(calib_re);
	}

	data = 1;
	paramid = ((AFE_SA_F0_TEST_INIT) | (iter << 16) | ((iter+1) << 24));
	pr_info("[SmartPA] now startup freq for smartPA-%d\n", iter);
	ret = mtk_afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, AP_2_DSP_SET_PARAM, nSize);
	if (ret < 0) {
		pr_err("[SmartPA] startup freq for smartPA-%d error\n", iter);
		done = false;
	} else {
		pr_info("[SmartPA] startup freq for smartPA-%d successful\n", iter);
		done = true;
	}

	msleep(5000);
	if (done == false) {
		pr_err("[SmartPA]: leave(done=false)<======%s\n", __func__);
		return -1;
	}

	n += scnprintf(buffer+n, size-n, "Ch[%d] ", iter);
	n += scnprintf(buffer+n, size-n, "Rdc = %d.%02d\n",
			VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Re_L15),
			VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Re_L15));

	while (i++ < 6 && j < 3) {
		result = 1;
		data = 0;

		paramid = (AFE_SA_GET_F0 | ((iter+1) << 16) | ((iter+1) << 24));
		pr_info("[SmartPA] now get F0 result for smartPA-%d\n", iter);
		ret = mtk_afe_smartamp_algo_ctrl((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, nSize);
		if (ret < 0) {
			pr_err("[SmartPA] get F0 result for smartPA-%d error\n", iter);
			result = 0;
		} else {
			pr_info("[SmartPA] get F0 result for smartPA-%d successful\n", iter);
			privPointer->F0_L15 = data;
			if ((privPointer->F0_L15 < privPointer->Freq_Min_L15) || (privPointer->F0_L15 > privPointer->Freq_Max_L15)) {
				pr_err("[SmartPA] F0 result for smartPA-%d is invalid\n", iter);
				result = 0;
			} else {
				pr_err("[SmartPA] F0 result for smartPA-%d is OK\n", iter);
			}
		}

		data = 0;
		paramid = (AFE_SA_GET_Q | ((iter+1) << 16) | ((iter+1) << 24));
		pr_info("[SmartPA] now get Q result for smartPA-%d\n", iter);
		ret = mtk_afe_smartamp_algo_ctrl((u8 *)&data, paramid, AP_2_DSP_GET_PARAM, nSize);
		if (ret < 0) {
			pr_err("[SmartPA] get Q result for smartPA-%d error\n", iter);
			result = 0;
		} else {
			pr_info("[SmartPA] get Q result for smartPA-%d successful\n", iter);
			privPointer->Qts_L15 = data;
			if (privPointer->Qts_L15 < privPointer->Qt_Min_L15) {
				pr_err("[SmartPA] Q result for smartPA-%d is invalid\n", iter);
				result = 0;
			} else {
				pr_info("[SmartPA] Q result for smartPA-%d is OK\n", iter);
			}
		}

		if (result == 1) {
			pr_info("[SmartPA] read freq dbg channel[%d]: F0 = %u Q = %u i = %d j = %d\n", 
					iter, privPointer->F0_L15, privPointer->Qts_L15, i, j);
			n += scnprintf(buffer+n, size-n, "F0: %d.%02d Qt = %d.%02d\n",
				VIVO_TRANSF_IMPED_TO_USER_I(privPointer->F0_L15), 
				VIVO_TRANSF_IMPED_TO_USER_M(privPointer->F0_L15), 
				VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Qts_L15), 
				VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Qts_L15));
			j++;

			if (j == 3)
				break;
		} else {
			pr_info("[SmartPA] read freq dbg channel[%d]: F0 = %u Q = %u i = %d j = %d\n",
					iter, privPointer->F0_L15, privPointer->Qts_L15, i, j);
			n += scnprintf(buffer+n, size-n, "F0: %d.%02d Qt = %d.%02d\n",
				VIVO_TRANSF_IMPED_TO_USER_I(privPointer->F0_L15),
				VIVO_TRANSF_IMPED_TO_USER_M(privPointer->F0_L15),
				VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Qts_L15),
				VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Qts_L15));
			pr_err("[SmartPA] the %dth time is error\n", i);
		}

		msleep(600);
	}

	n += scnprintf(buffer+n, size-n, "F0 (%d~%d)\nQ_Min: %d.%2d \n",
				   VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Freq_Min_L15), 
				   VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Freq_Max_L15),
				   VIVO_TRANSF_IMPED_TO_USER_I(privPointer->Qt_Min_L15), 
				   VIVO_TRANSF_IMPED_TO_USER_M(privPointer->Qt_Min_L15));
	if (j == 3)
		n += scnprintf(buffer+n, size-n, "PASS\n");
	else
		n += scnprintf(buffer+n, size-n, "FAIL\n");

	ret = vivo_smartpa_freq_save(buffer, n);
	buffer[n] = 0;

	pr_info("[SmartPA]: leave<======%s\n", __func__);
	return !result;
}

static int vivo_smartpa_check_calib_dbg(void)
{
	uint32_t impedance[CHANNEL_NUMS] = {0};
	uint8_t iter = 0;

	pr_info("[SmartPA]: %s enter.\n", __func__);
	vivo_smartpa_calib_get(impedance);
	for (iter = 0; iter < CHANNEL_NUMS; iter++) {
		if (impedance[iter] < param_priv.Imped_Min_L15 || impedance[iter] > param_priv.Imped_Max_L15) {
			return 0;
		}
	}

	return 1;
}

static void vivo_smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA]: %s enter.\n", __func__);
	return ;
}

#endif

#ifdef CONFIG_VIVO_PORT_SMARTPA
#include "smartpa-debug-common.h"

//static int aw882xx_get_dsp_msg_data(struct aw882xx *aw882xx,
//			char *data_ptr, int data_size, int inline_id);
static int aw_cali_get_re(struct aw882xx *aw882xx);
static struct aw882xx *aw882xx_vivo;
#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"
#define CHANNAL_NUMS (1)
#define RDC_MIN_L  (6000)
#define RDC_MAX_L  (8500)
#define RDC_MIN_R  (6000)
#define RDC_MAX_R  (8500)

#define TRANSF_IMPED_TO_USER_I(X) (((X * 100) >> 19) / 100)
#define TRANSF_IMPED_TO_USER_M(X) (((X * 100) >> 19) % 100)
struct aw_priv{
	uint32_t calibRe[CHANNAL_NUMS];
	uint32_t imped_min[CHANNAL_NUMS];
	uint32_t imped_max[CHANNAL_NUMS];
	uint32_t fres_min[CHANNAL_NUMS];
	uint32_t fres_max[CHANNAL_NUMS];
	uint32_t Qt[CHANNAL_NUMS];
	int mnChannels;
};
struct aw_priv *aw882xx_priv;
static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
    uint32_t imp_min, imp_max;
	if (!aw882xx_priv) {
		pr_err("[SmartPA-%d]: aw882xx_priv is NULL\n", __LINE__);
		return false;
	}

	if (iter >= CHANNAL_NUMS) {
		pr_err("[SmartPA-%d]: channel nums not expected\n", __LINE__);
		return false;
	}

    imp_min = ((aw882xx_priv->imped_min[iter]) >> 19) * 1000;
    imp_max = ((aw882xx_priv->imped_max[iter]) >> 19) * 1000;
	if (rdc > imp_min && rdc < imp_max) {
		return true;
	}
	pr_info("[SmartPA-%d]rdc check: rdc: %d invalid, [%d, %d] \n",
		__LINE__, rdc, imp_min, imp_max);
	return false;
}

static int smartpa_calib_save(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	if (!calib_value) {
		pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n", __LINE__);
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[0]=%d \n", __LINE__, calib_value[0]);
		if (CHANNAL_NUMS == 2)
			pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[1]=%d \n", __LINE__, calib_value[1]);
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNAL_NUMS, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]smartpa_calib_save: %s open failed! \n", __LINE__, CALIBRATE_FILE);
		ret = -1;
	}
	set_fs(old_fs);
	return ret;
}

static void smartpa_set_re(uint32_t *calibRe)
{
	int ret = 0;
	uint32_t cali_re = 0;
	if (!aw882xx_priv || !calibRe) {
		pr_err("[SmartPA-%d]: aw882xx_priv or calibRe is NULL\n", __LINE__);
		return;
	}
	//set re to adsp
	cali_re = *calibRe;
	cali_re = (cali_re * (1 << 12)) / 1000;
	ret = aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_RE, (void *)&cali_re,
					sizeof(uint32_t), aw882xx_vivo->chan_info.channel);
	if (ret < 0) {
		aw_dev_err(aw882xx_vivo->dev, "%s: dsp_msg_write error: %d\n",
			__func__, ret);
	}
	return;
}

static int smartpa_calib_get(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	
	if (!aw882xx_priv || !calib_value) {
		pr_err("[SmartPA-%d]: aw882xx_priv or calib_value is NULL\n", __LINE__);
		return false;
	}
	
	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*CHANNAL_NUMS, &pos);
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %d  \n", __LINE__, calib_value[0]);
		if (CHANNAL_NUMS == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %d  \n", __LINE__, calib_value[1]);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]calibrate: No found\n", __LINE__);
		found = 0;
	}
	set_fs(old_fs);
	return found;
}

static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[CHANNAL_NUMS] = {0};
	uint8_t iter = 0, channels = CHANNAL_NUMS;
	bool re_ok = true;

	pr_info("[SmartPA-%d] smartpa_check_re enter.\n", __LINE__);
	if (!aw882xx_priv) {
		pr_err("[SmartPA-%d]: aw882xx_priv is NULL\n", __LINE__);
		return false;
	}

	for (iter = 0; iter < channels; iter++) {
		if (rdc_check_valid(aw882xx_priv->calibRe[iter], iter) || (aw882xx_priv->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d ok.\n", __LINE__, iter, aw882xx_priv->calibRe[iter]);
			rc += 1;
		}
	}
	if (rc == channels)
		re_ok = true;
	else {
		smartpa_calib_get(impedance);
		rc = 0;
		for (iter = 0; iter < channels; iter++) {
			aw882xx_priv->calibRe[iter] = impedance[iter];
			if (rdc_check_valid(aw882xx_priv->calibRe[iter], iter)) {
				pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d success.\n", __LINE__, iter, aw882xx_priv->calibRe[iter]);
				rc += 1;
			} else {
				if (aw882xx_priv->calibRe[iter] == 0xCACACACA)
					rc += 1;
				pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d failed.\n", __LINE__, iter, aw882xx_priv->calibRe[iter]);
			}
		}
		if (rc == channels)
			re_ok = true;
		else
			re_ok = false;
	}

	return re_ok;
}

int smartpa_init_dbg(char *buffer, int size)
{
	int done[CHANNAL_NUMS] = {0};
	int ret = 0, n = 0;
	uint8_t iter = 0, channels = 1;
	uint32_t cali_temp = 0;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);
	if (aw882xx_vivo == NULL || aw882xx_priv == NULL) {
		pr_err("[SmartPA-%d]: enter %s PA was not inited \n", __LINE__, __func__);
		return -1;
	}
	aw882xx_run_mute_for_cali(aw882xx_vivo, false);
	channels = CHANNAL_NUMS;
	ret = aw_cali_get_re(aw882xx_vivo);
	if (ret < 0) {
		pr_info("%s:cali failed\n", __func__);
		memset(done, 0, CHANNAL_NUMS);
	}
	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (channels == 1) ? "Mono" : "Stereo"); 
	for (iter = 0; iter < channels; iter++) {
		cali_temp = (aw882xx_vivo->cali.re[iter] / 1000) << 19;
		if (aw882xx_priv->imped_max[iter] >= cali_temp && (aw882xx_priv->imped_min[iter]) <= cali_temp) {
			done[iter] = 1;
		}
		n += scnprintf(buffer + n, size - n,
				"Channel[%d]: impedance %2d.%03d ohm, valid range(%2d.%03d ~ %2d.%03d ohm). \n",
				iter, aw882xx_vivo->cali.re[iter]/1000, aw882xx_vivo->cali.re[iter]%1000, TRANSF_IMPED_TO_USER_I(aw882xx_priv->imped_min[iter]), TRANSF_IMPED_TO_USER_M(aw882xx_priv->imped_min[iter]), 
				TRANSF_IMPED_TO_USER_I(aw882xx_priv->imped_max[iter]), TRANSF_IMPED_TO_USER_M(aw882xx_priv->imped_max[iter]));
		n += scnprintf(buffer + n, size - n, "\n Calibrate result: %s\n", done[iter] ? "OKAY(impedance ok)." : "ERROR!");
	}
	aw882xx_vivo->cali.cali_result = done[iter]?1:0;
	aw882xx_run_mute_for_cali(aw882xx_vivo, !aw882xx_vivo->cali.cali_result);
	buffer[n] = 0;
	pr_info("[SmartPA]init_dbg: %s\n", buffer);
	for (iter = 0; iter < CHANNAL_NUMS; iter++) {
		aw882xx_priv->calibRe[iter] = aw882xx_vivo->cali.re[iter];
	}
	pr_info("[SmartPA-%d]init_dbg: write to file\n", __LINE__);
	smartpa_calib_save(aw882xx_priv->calibRe);
	pr_info("[SmartPA-%d]init_dbg: end\n", __LINE__);
	for (iter = 0; iter < CHANNAL_NUMS; iter++) {
		if (done[iter] != 1)
			return -1;
	}
	return 0;
}

static int smartpa_freq_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d \n", __LINE__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __LINE__, FREQ_FILE);
		ret = -1;
	}
	set_fs(old_fs);
	return ret;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	uint32_t calibRe[CHANNAL_NUMS] = {0};
	uint32_t F0[CHANNAL_NUMS] = {0}, Q[CHANNAL_NUMS] = {0};
	int ret = 0, n = 0;
	uint8_t iter = 0;
	//void *data_ptr;
	uint8_t retry = 3;
	uint32_t data_ptr[4] = {0};
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);
	if (aw882xx_vivo == NULL || aw882xx_priv == NULL) {
		pr_err("[SmartPA-%d]: enter %s PA was not inited \n", __LINE__, __func__);
		return -1;
	}
	//Load Calib
	if (smartpa_check_re()) {
		for (iter = 0; iter < CHANNAL_NUMS; iter++) {
			calibRe[iter] = aw882xx_priv->calibRe[iter];
		}
		smartpa_set_re(calibRe);
	}
	aw882xx_run_mute_for_cali(aw882xx_vivo, false);
	//wait 5s
	msleep(5000);
	iter = 0;
	//for(; iter < CHANNAL_NUMS; iter++)
	//{
	n += scnprintf(buffer+n, size-n, "Channel[%d]\n", iter);
	n += scnprintf(buffer+n, size-n, "impedance = %d.%03d\n",
			calibRe[iter]/1000, calibRe[iter]%1000);
	//read F0 Qt
	while (retry--) {
		ret = aw_read_msg_from_dsp(INLINE_PARAM_ID_F0_Q, (void *)data_ptr, sizeof(struct f0_q_data), aw882xx_vivo->chan_info.channel);
		//ret = aw882xx_get_dsp_msg_data(aw882xx_vivo, (char *)data_ptr, sizeof(struct f0_q_data), INLINE_PARAM_ID_F0_Q);//Jerry note
		F0[iter] = data_ptr[0];
		Q[iter] = data_ptr[1];
		pr_info("[SmartPA-%d]read freq dbg channel[%d]: f0 = %d Qt = %d\n", __LINE__, iter, F0[iter], Q[iter]);
		n += scnprintf(buffer+n, size-n, "f0 = %d Qt = %d.%03d\n", F0[iter], Q[iter]/1000, Q[iter]%1000);
	}
	n += scnprintf(buffer+n, size-n, "f0 (%d ~ %d)\nQt_Min: %d.%02d\n",
				   aw882xx_priv->fres_min[iter], aw882xx_priv->fres_max[iter],
				   aw882xx_priv->Qt[iter]/100, aw882xx_priv->Qt[iter]%100);
	if (ret == 0 && (aw882xx_priv->Qt[iter])*10 <= Q[iter]) {
		n += scnprintf(buffer+n, size-n, "PASS\n");
	} else {
		n += scnprintf(buffer+n, size-n, "FAIL\n");
	}
	aw882xx_run_mute_for_cali(aw882xx_vivo, !aw882xx_vivo->cali.cali_result);
	ret = smartpa_freq_save(buffer, n);
	buffer[n] = 0;
	return 0;
}
void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);
	return ;
}
void smartpa_get_client(struct i2c_client **client, unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);
	return ;
}
int smartpa_check_calib_dbg(void)
{
	uint32_t impedance[CHANNAL_NUMS] = {0};
	uint8_t iter = 0;
	int ret = 1;
	if (!aw882xx_priv)
		return 0;
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);
	smartpa_calib_get(impedance);
	for (iter = 0; iter < CHANNAL_NUMS; iter++) {
		ret &= rdc_check_valid(impedance[iter], iter);
	}
	return ret;
}
static int smartpa_parse_dt(struct aw882xx *aw882xx)
{
	int temp = 0, ret = 0, iter = 0;
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);
    aw882xx_priv = devm_kzalloc(aw882xx->dev, sizeof(struct aw_priv), GFP_KERNEL);
	if (aw882xx_priv == NULL)
		return -ENOMEM;
	aw882xx_priv->mnChannels = CHANNAL_NUMS;
	for (iter = 0; iter < aw882xx_priv->mnChannels; iter++) {
		ret = of_property_read_u32(aw882xx->dev->of_node, "vivo,impedance-min", &temp);
		aw882xx_priv->imped_min[iter] = (ret == 0) ? temp : RDC_MIN_L;
		ret = of_property_read_u32(aw882xx->dev->of_node, "vivo,impedance-max", &temp);
		aw882xx_priv->imped_max[iter] = (ret == 0) ? temp : RDC_MAX_L;
		ret = of_property_read_u32(aw882xx->dev->of_node, "vivo,frequency-min", &temp);
		aw882xx_priv->fres_min[iter] = (ret == 0) ? temp : 650;
		ret = of_property_read_u32(aw882xx->dev->of_node, "vivo,frequency-max", &temp);
		aw882xx_priv->fres_max[iter] = (ret == 0) ? temp : 1100;
		ret = of_property_read_u32(aw882xx->dev->of_node, "vivo,Qt-min", &temp);
		aw882xx_priv->Qt[iter] = (ret == 0) ? temp : 1100;
	}	
	pr_info("[SmartPA-%d]: %s ret=%d end.\n", __LINE__, __func__, ret);
	return ret;
}
#endif

#ifdef AW_CALI_STORE_EXAMPLE
/*write cali to persist file example*/
#define AWINIC_CALI_FILE  "/mnt/vendor/persist/factory/audio/aw_cali.bin"
#define AW_INT_DEC_DIGIT 10
static int aw882xx_write_cali_re_to_file(int32_t cali_re, int channel)
{
	struct file *fp = NULL;
	char buf[50] = {0};
	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}
	if (channel == AW882XX_CHANNLE_RIGHT)
		pos = AW_INT_DEC_DIGIT;

	cali_re = FIXED_RE_TO_MOHM(cali_re);
	snprintf(buf, sizeof(buf), "%10d", cali_re);

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: channel:%d buf:%s cali_re:%d\n",
		__func__, channel, buf, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int aw882xx_get_cali_re_from_file(int32_t *cali_re, int channel)
{
	struct file *fp = NULL;
	/*struct inode *node;*/
	int f_size;
	char *buf = NULL;
	int32_t int_cali_re = 0;
	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AWINIC_CALI_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}

	if (channel == AW882XX_CHANNLE_RIGHT)
		pos = AW_INT_DEC_DIGIT;

	/*node = fp->f_dentry->d_inode;*/
	/*f_size = node->i_size;*/
	f_size = AW_INT_DEC_DIGIT;

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: channel:%d malloc mem %d failed!\n",
			__func__, channel, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, f_size, &pos);

	set_fs(fs);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = MOHM_TO_FIXED_RE(int_cali_re);
	else
		*cali_re = AW_ERRO_CALI_VALUE;

	pr_info("%s: channel:%d buf:%s int_cali_re: %d\n",
		__func__, channel, buf, int_cali_re);

	kfree(buf);
	buf = NULL;
	filp_close(fp, NULL);

	return  0;

}
#endif

 /*custom need add to set/get cali_re form/to nv*/
int aw882xx_set_cali_re_to_nvram(int32_t cali_re, int32_t channel)
{
	/*custom add, if success return value is 0, else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw882xx_write_cali_re_to_file(cali_re, channel);
#else
	return -EBUSY;
#endif
}
int aw882xx_get_cali_re_from_nvram(int32_t *cali_re, int32_t channel)
{
	/*custom add, if success return value is 0 , else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw882xx_get_cali_re_from_file(cali_re, channel);
#endif
#ifdef CONFIG_VIVO_PORT_SMARTPA
	uint32_t impedance[CHANNAL_NUMS] = {0};
	int found = 0;
	if (channel > CHANNAL_NUMS) {
		return -EINVAL;
	}
	found = smartpa_calib_get(impedance);
    if (found <= 0) {
		pr_info("cali_re not found!!\n");
		return -EINVAL;	
	}
	*cali_re = (int32_t)impedance[channel];
	*cali_re = MOHM_TO_FIXED_RE(*cali_re);
    pr_info("%s: channel:%d, *cali_re: %d\n", __func__, channel, *cali_re);
	return 0;
#endif
#ifdef CONFIG_VIVO_SMARTPA_NEW
	uint32_t impedance = 0;
	int found = 0;
	found = vivo_smartpa_calib_get(&impedance);
	if (found <= 0) {
		pr_info("[smartPA] %s cali_re not found!!\n", __func__);
		return -EINVAL; 
	}
	*cali_re = VIVO_TRANSF_IMPED_TO_USER_I(impedance); // *cali_re is such as 7.5
	pr_info("%s: channel:%d(before), *cali_re: %d\n", __func__, 0, *cali_re);
	*cali_re = MOHM_TO_FIXED_RE(*cali_re);
	pr_info("%s: channel:%d(after), *cali_re: %d\n", __func__, 0, *cali_re);
	return 0;
#else
	return -EINVAL;
#endif
}

static int aw882xx_store_cali_re(struct aw882xx *aw882xx, int32_t cali_re)
{
	struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

	if (aw882xx == NULL)
		return -EINVAL;
	aw882xx->cali.cali_re = cali_re;
	return aw882xx_set_cali_re_to_nvram(cali_re, chan_info->channel);
}

void aw882xx_load_cali_re(struct aw_cali *cali)
{
	int32_t cali_re = 0;
	int ret = 0;
	struct aw882xx *aw882xx =
			container_of(cali, struct aw882xx, cali);
	struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

	ret = aw882xx_get_cali_re_from_nvram(&cali_re, chan_info->channel);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: get cali re from nv failed: %d\n",
			 __func__, ret);
		cali_re = AW_ERRO_CALI_VALUE;
#ifdef CONFIG_VIVO_PORT_SMARTPA
		aw882xx->cali.cali_result = 1;//no cali flie, let PA working
		aw882xx->cali.cali_re = cali_re;
		return;
#endif
	}
#ifdef CONFIG_VIVO_PORT_SMARTPA
	aw882xx->cali.cali_re = cali_re;
	aw882xx->cali.cali_result = (rdc_check_valid(FIXED_RE_TO_MOHM(cali_re), chan_info->channel) ? 1 : 0);
	aw_dev_info(aw882xx->dev, "%s: cali_result:%d \n", __func__, aw882xx->cali.cali_result);
#endif

#ifdef CONFIG_VIVO_SMARTPA_NEW
	aw882xx->cali.cali_result = 1;
	aw882xx->cali.cali_re = cali_re;
	return;
#endif

}

static int aw_run_dsp_hmute(struct aw882xx *aw882xx, uint32_t hmute_st)
{
#ifdef AWINIC_DSP_HMUTE
	aw_dev_dbg(aw882xx->dev, "%s: hmute_st:%d\n",
			__func__, hmute_st);

	return aw_write_msg_to_dsp(INLINE_PARAM_ID_ENABLE_HMUTE,
			(void *)&hmute_st, sizeof(hmute_st),
			aw882xx->chan_info.channel);
#endif
	return 0;
}

static int aw_run_cali_cfg_to_dsp(struct aw882xx *aw882xx, int cali_flag)
{
	int ret;
	struct aw_cali *cali = &aw882xx->cali;

#ifdef AWINIC_DSP_MSG
	aw_dev_dbg(aw882xx->dev, "%s: cali_flag:%d\n",
			__func__, cali_flag);

	ret = aw_write_msg_to_dsp(INLINE_PARAM_ID_ENABLE_CALI,
			(void *)&cali_flag, sizeof(cali_flag),
			aw882xx->chan_info.channel);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:start cali failed!\n",
			__func__);
		return ret;
	}

	if (cali_flag)
		msleep(cali->cali_re_time);

	return 0;
#else
	struct cali_cfg set_cfg = { {0, 0, -1} };
	int i;

	aw_dev_dbg(aw882xx->dev, "%s: cali_flag:%d\n",
			__func__, cali_flag);

	if (cali_flag) {
		for (i = cali->first_cali_num; i <= cali->end_cali_num; i++) {
			ret = aw_read_data_from_dsp(INDEX_PARAMS_ID_RX_CALI_CFG,
					(void *)&aw882xx->cali.store_cfg[i],
					sizeof(struct cali_cfg), i);
			if (ret < 0) {
				aw_dev_err(aw882xx->dev, "%s:dev[%d] read cali cfg data failed!\n",
					__func__, i);
				goto back_cfg;
			}

			ret = aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_CALI_CFG,
						(void *)&set_cfg,
						sizeof(struct cali_cfg), i);
			if (ret < 0) {
				aw_dev_err(aw882xx->dev, "%s: dev[%d]start cali failed !\n",
					__func__, i);
				goto back_cfg;
			}
		}
		msleep(10 * 1000);
	} else {
		for (i = cali->first_cali_num; i <= cali->end_cali_num; i++)
			aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_CALI_CFG,
				(void *)&aw882xx->cali.store_cfg[i],
				sizeof(struct cali_cfg), i);
	}

	return 0;

back_cfg:
	i--;
	while (i >= cali->first_cali_num) {
		aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_CALI_CFG,
				(void *)&aw882xx->cali.store_cfg[i],
				sizeof(struct cali_cfg), i);
		i--;
	}
	return ret;

#endif
}

static int aw_run_noise_to_dsp(struct aw882xx *aw882xx,
				int32_t noise_enable)
{
	int i, ret;
	struct aw_cali *cali = &aw882xx->cali;

	aw_dev_dbg(aw882xx->dev, "%s: noise_enable:%d\n",
			 __func__, noise_enable);

	for (i = cali->first_cali_num; i <= cali->end_cali_num; i++) {
		ret =  aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_NOISE,
			(void *)&noise_enable, sizeof(noise_enable), i);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s:dev[%d] set noise:%d failed!\n",
					__func__, i, noise_enable);
			return ret;
		}
	}
	return 0;
}

static int aw_cali_get_re_from_dsp(struct aw882xx *aw882xx)
{
	struct cali_data cali_data;
	int ret, i;
	struct aw_cali *cali = &aw882xx->cali;

	/*get cali data*/
	for (i = cali->first_cali_num; i <= cali->end_cali_num; i++) {
		ret = aw_read_data_from_dsp(INDEX_PARAMS_ID_RX_REAL_DATA,
						(void *)&cali_data,
						sizeof(struct cali_data), i);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s:dev[%d] read cali data failed!\n",
				__func__, i);
			return ret;
		}

		aw882xx->cali.re[i] = FIXED_RE_TO_MOHM(cali_data.data[0]);
		aw_dev_dbg(aw882xx->dev, "%s:dev[%d]: re %d\n",
			__func__, i, aw882xx->cali.re[i]);
	}

	return 0;
}

static int aw_cali_get_f0_from_dsp(struct aw882xx *aw882xx)
{
	int32_t read_f0;
	int ret, i;
	struct aw_cali *cali = &aw882xx->cali;

	for (i = cali->first_cali_num; i <= cali->end_cali_num; i++) {
		ret = aw_read_data_from_dsp(INDEX_PARAMS_ID_RX_F0,
				(void *)&read_f0, sizeof(int32_t), i);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s:dev[%d] read f0 failed!\n",
				__func__, i);
			return -EBUSY;
		}
		aw882xx->cali.f0[i] = read_f0;
		aw_dev_dbg(aw882xx->dev, "%s:dev[%d]: f0 %d\n",
			__func__, i, aw882xx->cali.f0[i]);
	}

	return 0;
}

static int aw_cali_get_re(struct aw882xx *aw882xx)
{
	int ret;

	g_cali_status = true;

	ret = aw_run_dsp_hmute(aw882xx, true);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:set dsp hmute failed!\n",
			__func__);
		goto mute_failed;
	}

	ret = aw_run_cali_cfg_to_dsp(aw882xx, true);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:set cali cfg failed !\n",
			__func__);
		goto set_cali_cfg_failed;
	}

	/*get cali data*/
	ret = aw_cali_get_re_from_dsp(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read re failed!\n",
			__func__);
		goto cali_data_failed;
	}

cali_data_failed:
	aw_run_cali_cfg_to_dsp(aw882xx, false);
set_cali_cfg_failed:
	aw_run_dsp_hmute(aw882xx, false);
mute_failed:
	g_cali_status = false;
	return ret;
}

static int aw_cali_get_f0(struct aw882xx *aw882xx, bool noise_en)
{
	int ret;

	g_cali_status = true;
	if (noise_en) {
		ret = aw_run_cali_cfg_to_dsp(aw882xx, true);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s:set cali cfg failed !\n",
				__func__);
			goto set_cali_cfg_failed;
		}

		ret = aw_run_noise_to_dsp(aw882xx, true);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s: set noise enable failed\n",
				__func__);
			goto set_noise_failed;
		}

		/*keep 5 s, wait data stable*/
		msleep(5 * 1000);
	}

	/*get cali data*/
	ret = aw_cali_get_f0_from_dsp(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:read f0 failed!\n",
			__func__);
	}

set_noise_failed:
	if (noise_en) {
		aw_run_noise_to_dsp(aw882xx, false);
		aw_run_cali_cfg_to_dsp(aw882xx, false);
	}
set_cali_cfg_failed:
	g_cali_status = false;
	return ret;
}

static int aw882xx_cali_start_up(struct aw882xx *aw882xx)
{
	int ret = -1;
	if (aw882xx->init != AW882XX_INIT_OK) {
		aw_dev_err(aw882xx->dev, "%s:need startup play\n", __func__);
		return -1;
	}

	g_cali_status = true;
	ret = aw_run_dsp_hmute(aw882xx, true);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:set dsp hmute failed!\n", __func__);
		goto mute_failed;
	}

	ret = aw_run_cali_cfg_to_dsp(aw882xx, true);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:set cali cfg failed !\n",
			__func__);
		aw_run_dsp_hmute(aw882xx, false);
		goto set_cali_cfg_failed;
	}

	/*get cali data*/
	ret = aw_cali_get_re_from_dsp(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read re failed!\n",
			__func__);
		goto re_cali_failed;
	}

	ret = aw_run_dsp_hmute(aw882xx, false);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:set dsp unhmute failed!\n",
			__func__);
		goto unhmute_failed;
	}

	/*start white noise*/
	ret = aw_run_noise_to_dsp(aw882xx, true);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: set noise enable failed\n",
			__func__);
		goto set_noise_failed;
	}

	/*keep 5 s, wait data stable*/
	msleep(5 * 1000);

	/*get f0 value*/
	ret = aw_cali_get_f0_from_dsp(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read f0 failed !\n", __func__);
		goto f0_cali_failed;
	}

f0_cali_failed:
	aw_run_noise_to_dsp(aw882xx, false);
set_noise_failed:
unhmute_failed:
re_cali_failed:
	aw_run_cali_cfg_to_dsp(aw882xx, false);
set_cali_cfg_failed:
mute_failed:
	g_cali_status = false;
	return ret;
}

#ifdef CONFIG_DEBUG_FS
/***************cali debug fs***************/
/*unit mOhms*/
static int R0_MAX = 15000;
static int R0_MIN = 5000;

static ssize_t smartpa_dbgfs_i2c_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;
	const int size = 512;
	char buffer[size];
	int n = 0;
	if (!aw882xx)
		return -ENOMEM;
	pr_info("[SmartPA-%d]%s enter.\n", __LINE__, __func__);
	n += scnprintf(buffer+n, size-n, "SmartPA-mono %s\n",
				aw882xx->smartpa_i2c_check ? "OK" : "ERROR");
	buffer[n] = 0;
	return simple_read_from_buffer(user_buf, count, ppos, buffer, n);
}
const struct file_operations smartpa_dbgfs_i2c_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_i2c_read,
	.llseek = default_llseek,
};
int  aw_cali_range_open(struct inode *inode, struct file *file)
{
	struct aw882xx *aw882xx = (struct aw882xx *)inode->i_private;

	file->private_data = (void *)aw882xx;
	aw_dev_info(aw882xx->dev, "%s: open success", __func__);
	return 0;
}

ssize_t aw_cali_range_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char local_buf[50] = { 0 };
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;

	if (*ppos)
		return 0;

	memset(local_buf, 0, sizeof(local_buf));
	if (len < sizeof(local_buf)) {
		aw_dev_err(aw882xx->dev, "%s: buf len not enough\n", __func__);
		return -ENOSPC;
	}

	ret = snprintf(local_buf, sizeof(local_buf) - 1,
		" Min:%d mOhms, Max:%d mOhms\n", R0_MIN, R0_MAX);

	ret = simple_read_from_buffer(buf, len, ppos, local_buf, ret);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: copy failed!\n", __func__);
		return -ENOMEM;
	}
	return ret;
}

ssize_t aw_cali_range_write(struct file *file,
	const char __user *buf, size_t len, loff_t *ppos)
{
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;
	uint32_t time;
	int ret;

	if (*ppos)
		return 0;

	ret = kstrtouint_from_user(buf, len, 0, &time);
	if (ret)
		return len;

	if (time < AW_CALI_RE_MIN_TIMER) {
		aw_dev_err(aw882xx->dev, "%s:time:%d is too short, no set\n",
			__func__, time);
		return -EINVAL;
	}

	aw882xx->cali.cali_re_time = time;

	return len;
}

static const struct file_operations aw_cali_range_fops = {
	.open = aw_cali_range_open,
	.read = aw_cali_range_read,
	.write = aw_cali_range_write,
};

int  aw_cali_open(struct inode *inode, struct file *file)
{
	struct aw882xx *aw882xx = (struct aw882xx *)inode->i_private;

	file->private_data = (void *)aw882xx;
	aw_dev_dbg(aw882xx->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_cali_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char ret_value[64] = { 0 };
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;

	if (*ppos)
		return 0;

	memset(ret_value, 0, sizeof(ret_value));
	if (len < sizeof(ret_value)) {
		aw_dev_err(aw882xx->dev, "%s:buf len no enough\n", __func__);
		memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
		return -ENOMEM;
	}

	ret = aw_cali_get_re(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:cali failed\n", __func__);
		memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
		return ret;
	}

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		ret = snprintf(ret_value, sizeof(ret_value) - 1,
				"left:%d mOhms right:%d mOhms\n",
			aw882xx->cali.re[0], aw882xx->cali.re[1]);
	else
		ret = snprintf(ret_value, sizeof(ret_value) - 1,
			"%d\n", aw882xx->cali.re[aw882xx->chan_info.channel]);

	return simple_read_from_buffer(buf, len, ppos, ret_value, ret);
}

ssize_t aw_cali_write(struct file *file,
	const char __user *buf, size_t len, loff_t *ppos)
{

	return 0;
}

static const struct file_operations aw_cali_fops = {
	.open = aw_cali_open,
	.read = aw_cali_read,
	.write = aw_cali_write,
};

int  aw_f0_open(struct inode *inode, struct file *file)
{
	struct aw882xx *aw882xx = (struct aw882xx *)inode->i_private;

	file->private_data = (void *)aw882xx;
	aw_dev_dbg(aw882xx->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_f0_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char ret_value[64] = { 0 };
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;

	if (*ppos)
		return 0;

	memset(ret_value, 0, sizeof(ret_value));
	if (len < sizeof(ret_value)) {
		aw_dev_err(aw882xx->dev, "%s:buf len no enough\n", __func__);
		memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
		return -ENOMEM;
	}

	ret = aw_cali_get_f0(aw882xx, false);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:cali failed\n", __func__);
		memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
		return ret;
	}

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		ret = snprintf(ret_value, sizeof(ret_value) - 1,
				"left:%d right:%d\n", aw882xx->cali.f0[0],
				aw882xx->cali.f0[1]);
	else
		ret = snprintf(ret_value, sizeof(ret_value) - 1, "%d\n",
				aw882xx->cali.f0[aw882xx->chan_info.channel]);

	ret = simple_read_from_buffer(buf, len, ppos, ret_value, ret);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:copy failed!\n", __func__);
		return -ENOMEM;
	}
	return ret;
}

static const struct file_operations aw_f0_fops = {
	.open = aw_f0_open,
	.read = aw_f0_read,
};
int  aw_cali_status_open(struct inode *inode, struct file *file)
{
	struct aw882xx *aw882xx = (struct aw882xx *)inode->i_private;

	file->private_data = (void *)aw882xx;
	aw_dev_dbg(aw882xx->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_cali_status_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char status_value[20] = { 0 };
	struct cali_data cali_data;
	int32_t real_r0;
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;
	struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

	if (*ppos)
		return 0;

	if (len < sizeof(status_value)) {
		aw_dev_err(aw882xx->dev, "%s:buf len no enough\n", __func__);
		return -ENOSPC;
	}

	/*get cali data*/
	ret = aw_read_data_from_dsp(INDEX_PARAMS_ID_RX_REAL_DATA,
				(void *)&cali_data, sizeof(struct cali_data),
				chan_info->channel);
	if (ret) {
		aw_dev_err(aw882xx->dev, "%s:read speaker status failed!\n",
			__func__);
		return -EBUSY;
	}
	/*R0 factor form 4096 to 1000*/
	real_r0 = FIXED_RE_TO_MOHM(cali_data.data[0]);
	ret = snprintf(status_value, sizeof(status_value) - 1,
			"%d : %d\n", real_r0, cali_data.data[1]);

	ret = simple_read_from_buffer(buf, len, ppos, status_value, ret);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s:copy failed!", __func__);
		return -ENOMEM;
	}
	return ret;
}

static const struct file_operations aw_cali_status_fops = {
	.open = aw_cali_status_open,
	.read = aw_cali_status_read,
};

int  aw_reg_open(struct inode *inode, struct file *file)
{
	struct aw882xx *aw882xx = (struct aw882xx *)inode->i_private;
	file->private_data = (void *)aw882xx;
	aw_dev_dbg(aw882xx->dev, "%s: open success\n", __func__);
	return 0;
}
ssize_t aw_reg_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	char *buffer = NULL;
	ssize_t n = 0;
	unsigned char i = 0;
	size_t remainLen = len;
	size_t needLen = 16 * AW882XX_REG_MAX;
	unsigned int reg_val = 0;
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;
	if (*ppos) {
		return -EINVAL;
	}
	if (remainLen < needLen) {
		aw_dev_err(aw882xx->dev, "%s:buf len not enough\n", __func__);
		return -ENOSPC;
	}
	buffer = kzalloc(needLen, GFP_KERNEL);
	if (buffer == NULL) {
		aw_dev_err(aw882xx->dev, "%s: failed to kmalloc!\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < AW882XX_REG_MAX; i++) {
		if (aw882xx_reg_access[i]&REG_RD_ACCESS) {
			aw882xx_i2c_read(aw882xx, i, &reg_val);
			n += scnprintf(buffer+n, remainLen-n, "reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}
	buffer[n] = 0;
	n = simple_read_from_buffer(buf, len, ppos, buffer, n);
	if (n < 0) {
		aw_dev_err(aw882xx->dev, "%s:copy failed!", __func__);
		kfree(buffer);
		return -ENOMEM;
	}
	kfree(buffer);
	return n;
}
ssize_t aw_reg_write(struct file *file,
		const char __user *buf, size_t len, loff_t *ppos)
{
	struct aw882xx *aw882xx = (struct aw882xx *)file->private_data;
	unsigned int databuf[2] = {0};
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);
	return len;
}
static const struct file_operations aw_reg_fops = {
	.open = aw_reg_open,
	.read = aw_reg_read,
	.write = aw_reg_write,
};
static void aw_cali_debugfs_init(struct aw882xx *aw882xx)
{
	const char *debugfs_dir = "audio-aw882xx";
	struct aw_dbg_cali *dbg_fs = &aw882xx->cali.dbg_fs;
	int ret;

	ret = aw882xx_append_suffix("%s_%s", &debugfs_dir, aw882xx);
	if (ret < 0)
		return;

	dbg_fs->dbg_dir = debugfs_create_dir(debugfs_dir, NULL);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_dir)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs failed !\n");
		return;
	}
	debugfs_create_file("i2c", S_IRUGO|S_IWUGO, dbg_fs->dbg_dir,
			aw882xx, &smartpa_dbgfs_i2c_fops);
	dbg_fs->dbg_range = debugfs_create_file("range", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw882xx, &aw_cali_range_fops);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_range)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs range failed !\n");
		return;
	}
	dbg_fs->dbg_cali = debugfs_create_file("cali", S_IFREG|S_IRUGO|S_IWUGO,
			dbg_fs->dbg_dir, aw882xx, &aw_cali_fops);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_cali)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs cali failed !\n");
		return;
	}
	dbg_fs->dbg_f0 = debugfs_create_file("f0", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw882xx, &aw_f0_fops);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_f0)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs cali failed !\n");
		return;
	}
	dbg_fs->dbg_status = debugfs_create_file("status", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw882xx, &aw_cali_status_fops);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_status)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs status failed !\n");
		return;
	}
	dbg_fs->dbg_reg = debugfs_create_file("reg", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw882xx, &aw_reg_fops);
	if (IS_ERR_OR_NULL(dbg_fs->dbg_reg)) {
		aw_dev_err(aw882xx->dev, "create cali debugfs range failed !\n");
		return;
	}
}

void aw_cali_debugfs_deinit(struct aw882xx *aw882xx)
{
	struct aw_dbg_cali *dbg_fs = &aw882xx->cali.dbg_fs;

	debugfs_remove(dbg_fs->dbg_range);
	debugfs_remove(dbg_fs->dbg_cali);
	debugfs_remove(dbg_fs->dbg_f0);
	debugfs_remove(dbg_fs->dbg_status);
	debugfs_remove(dbg_fs->dbg_reg);
	debugfs_remove(dbg_fs->dbg_dir);
}
#endif

/***********************cali misc device*********************/
static int aw882xx_file_open(struct inode *inode, struct file *file)
{
	struct miscdevice *device = NULL;
	struct aw_misc_cali *misc_ptr = NULL;
	struct aw_cali *cali_ptr = NULL;
	struct aw882xx *aw882xx = NULL;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	device = (struct miscdevice *)file->private_data;

	misc_ptr = container_of(device, struct aw_misc_cali, misc_device);
	cali_ptr = container_of(misc_ptr, struct aw_cali, misc);
	aw882xx = container_of(cali_ptr, struct aw882xx, cali);

	file->private_data = (void *)aw882xx;

	aw_dev_dbg(aw882xx->dev, "%s: misc open success\n", __func__);
	return 0;
}

static int aw882xx_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	pr_debug("misc release successi\n");
	return 0;
}

static int aw882xx_file_get_index(unsigned int cmd, int32_t *index)
{
	switch (cmd) {
	case AW882XX_IOCTL_GET_CALI_CFG:
	case AW882XX_IOCTL_SET_CALI_CFG:
		*index = INDEX_PARAMS_ID_RX_CALI_CFG;
		break;
	case AW882XX_IOCTL_GET_CALI_DATA:
		*index = INDEX_PARAMS_ID_RX_REAL_DATA;
		break;
	case AW882XX_IOCTL_SET_NOISE:
		*index = INDEX_PARAMS_ID_RX_NOISE;
		break;
	case AW882XX_IOCTL_GET_F0:
		*index = INDEX_PARAMS_ID_RX_F0;
		break;
	case AW882XX_IOCTL_GET_CALI_RE:
	case AW882XX_IOCTL_SET_CALI_RE:
		*index = INDEX_PARAMS_ID_RX_RE;
		break;
	case AW882XX_IOCTL_GET_VMAX:
	case AW882XX_IOCTL_SET_VMAX:
		*index = INDEX_PARAMS_ID_RX_VMAX;
		break;
	case AW882XX_IOCTL_SET_PARAM:
	case AW882XX_IOCTL_SET_PTR_PARAM_NUM:
		*index = INDEX_PARAMS_ID_RX_PARAMS;
		break;
	case AW882XX_IOCTL_ENABLE_CALI:
		break;
	case AW882XX_IOCTL_GET_F0_Q:
	case AW882XX_IOCTL_SET_DSP_HMUTE:
	case AW882XX_IOCTL_SET_CALI_CFG_FLAG:
		*index = INDEX_PARAMS_ID_AWDSP_RX_MSG;
		break;
	default:
		pr_err("%s: unsupported cmd %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static int aw882xx_cali_operation(struct aw882xx *aw882xx,
			unsigned int cmd, unsigned long arg)
{
	int16_t data_len = _IOC_SIZE(cmd);
	int ret = 0;
	char *data_ptr = NULL;
	uint32_t index = 0;
	struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;
	struct ptr_params_data *p_params = NULL;
	int32_t *p_data = NULL;

	aw_dev_info(aw882xx->dev, "cmd : %d, data_len%d\n", cmd, data_len);

	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (!data_ptr) {
		aw_dev_err(aw882xx->dev, "%s : malloc failed !\n", __func__);
		return -ENOMEM;
	}

	ret = aw882xx_file_get_index(cmd, &index);
	if (ret < 0)
		goto exit;

	switch (cmd) {
	case AW882XX_IOCTL_ENABLE_CALI:
		if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		g_cali_status = (int8_t)data_ptr[0];
		aw_dev_info(aw882xx->dev, "%s:set cali %s", __func__,
			(g_cali_status == 0) ? ("disable") : ("enable"));
		break;
	case AW882XX_IOCTL_SET_CALI_CFG:
	case AW882XX_IOCTL_SET_NOISE:
	case AW882XX_IOCTL_SET_VMAX:
	case AW882XX_IOCTL_SET_PARAM:
		if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		ret = aw_write_data_to_dsp(index, data_ptr,
					data_len, chan_info->channel);
		if (ret) {
			aw_dev_err(aw882xx->dev, "%s: dsp_msg_write error: %d\n",
				__func__, index);
			goto exit;
		}
		break;
	case AW882XX_IOCTL_SET_PTR_PARAM_NUM:
		if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		p_params = (struct ptr_params_data *)data_ptr;
		if (p_params->data == NULL || (!p_params->len)) {
			aw_dev_err(aw882xx->dev, "%s: p_params error\n",
				__func__);
			ret = -EFAULT;
			goto exit;
		}
		p_data = kzalloc(p_params->len, GFP_KERNEL);
		if (!p_data) {
			aw_dev_err(aw882xx->dev,
				"%s: error allocating memory\n", __func__);
			ret = -ENOMEM;
			goto exit;
		}

		if (copy_from_user(p_data,
				(void __user *)p_params->data,
				p_params->len)) {
			kfree(p_data);
			p_data = NULL;
			ret = -EFAULT;
			goto exit;
		}

		ret = aw_write_data_to_dsp(index, p_data,
					p_params->len, chan_info->channel);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s: dsp_msg_write error: %d\n",
				__func__, index);
			kfree(p_data);
			p_data = NULL;
			goto exit;
		}
		kfree(p_data);
		p_data = NULL;
		break;
	case AW882XX_IOCTL_SET_CALI_RE:
		if (copy_from_user(data_ptr,
			(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}

		ret = aw882xx_store_cali_re(aw882xx, *((int32_t *)data_ptr));
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s: store cali re error\n",
				__func__);
			goto exit;
		}

		ret = aw_write_data_to_dsp(index, data_ptr,
					data_len, chan_info->channel);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s: dsp_msg_write error: %d\n",
				__func__, index);
			ret = 0;
		}
		break;
	case AW882XX_IOCTL_SET_DSP_HMUTE:
		if (copy_from_user(data_ptr,
			(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		ret = aw_write_msg_to_dsp(INLINE_PARAM_ID_ENABLE_HMUTE,
			data_ptr, data_len, aw882xx->chan_info.channel);
		if (ret < 0)
			goto exit;
		break;
	case AW882XX_IOCTL_SET_CALI_CFG_FLAG:
		if (copy_from_user(data_ptr,
			(void __user *)arg, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		ret = aw_write_msg_to_dsp(INLINE_PARAM_ID_ENABLE_CALI,
			data_ptr, data_len, aw882xx->chan_info.channel);
		if (ret < 0)
			goto exit;
		break;
	case AW882XX_IOCTL_GET_CALI_CFG:
	case AW882XX_IOCTL_GET_CALI_DATA:
	case AW882XX_IOCTL_GET_F0:
	case AW882XX_IOCTL_GET_CALI_RE:
	case AW882XX_IOCTL_GET_VMAX:
		ret = aw_read_data_from_dsp(index, data_ptr,
					data_len, chan_info->channel);
		if (ret) {
			aw_dev_err(aw882xx->dev, "%s: dsp_msg_read error: %d\n",
				__func__, index);
			ret = -EFAULT;
			goto exit;
		}
		if (copy_to_user((void __user *)arg,
			data_ptr, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		break;
	case AW882XX_IOCTL_GET_F0_Q:
		ret = aw_read_msg_from_dsp(INLINE_PARAM_ID_F0_Q,
			data_ptr, data_len, chan_info->channel);
		if (ret < 0)
			goto exit;
		if (copy_to_user((void __user *)arg,
			data_ptr, data_len)) {
			ret = -EFAULT;
			goto exit;
		}
		break;
	default:
		aw_dev_err(aw882xx->dev, "%s : cmd %d\n",
			__func__, cmd);
		break;
	}
exit:
	kfree(data_ptr);
	data_ptr = NULL;
	return ret;
}

static long aw882xx_file_unlocked_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aw882xx *aw882xx = NULL;

	if (((_IOC_TYPE(cmd)) != (AW882XX_IOCTL_MAGIC))) {
		pr_err("%s: cmd magic err\n", __func__);
		return -EINVAL;
	}
	aw882xx = (struct aw882xx *)file->private_data;
	ret = aw882xx_cali_operation(aw882xx, cmd, arg);
	if (ret)
		return -EINVAL;

	return 0;
}

#ifdef CONFIG_COMPAT
static long aw882xx_file_compat_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aw882xx *aw882xx = NULL;

	if (((_IOC_TYPE(cmd)) != (AW882XX_IOCTL_MAGIC))) {
		pr_err("%s: cmd magic err\n", __func__);
		return -EINVAL;
	}
	aw882xx = (struct aw882xx *)file->private_data;
	ret = aw882xx_cali_operation(aw882xx, cmd, arg);
	if (ret)
		return -EINVAL;

	return 0;
}
#endif

static const struct file_operations aw882xx_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aw882xx_file_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = aw882xx_file_compat_ioctl,
#endif
	.open = aw882xx_file_open,
	.release = aw882xx_file_release,
};

static void aw_cali_misc_init(struct aw882xx *aw882xx)
{
	int ret;
	struct miscdevice *device = &aw882xx->cali.misc.misc_device;
	const char *aw_misc_name = "aw882xx_smartpa";

	ret = aw882xx_append_suffix("%s_%s", &aw_misc_name, aw882xx);
	if (ret < 0)
		return;

	device->minor = MISC_DYNAMIC_MINOR;
	device->name  = aw_misc_name;
	device->fops  = &aw882xx_fops;

	ret = misc_register(device);
	if (ret) {
		aw_dev_err(aw882xx->dev, "%s: misc register fail: %d\n",
			__func__, ret);
		return;
	}
	aw_dev_dbg(aw882xx->dev, "%s: misc register success\n", __func__);
}

static void aw_cali_misc_deinit(struct aw882xx *aw882xx)
{
	misc_deregister(&aw882xx->cali.misc.misc_device);
	aw_dev_dbg(aw882xx->dev, "%s: misc unregister done\n", __func__);
}

/*****************ATTR FOR Calibration**********************************/
static ssize_t aw882xx_cali_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d ms\n",
				aw882xx->cali.cali_re_time);

	return len;
}

static ssize_t aw882xx_cali_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	uint32_t time;

	ret = kstrtoint(buf, 0, &time);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s, read buf %s failed\n",
			__func__, buf);
		return ret;
	}

	if (time < AW_CALI_RE_MIN_TIMER) {
		aw_dev_err(aw882xx->dev, "%s:time:%d is too short, no set\n",
			__func__, time);
		return -EINVAL;
	}

	aw882xx->cali.cali_re_time = time;
	aw_dev_info(aw882xx->dev, "%s:time:%d\n",
			__func__, aw882xx->cali.cali_re_time);

	return count;
}

static ssize_t aw882xx_cali_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret;

	ret = aw882xx_cali_start_up(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
		memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
		return ret;
	}

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num) {
		len += snprintf(buf+len,
			PAGE_SIZE - len, "Re = left:%d mOhms right:%d mOhms\n",
			aw882xx->cali.re[0], aw882xx->cali.re[1]);
		len += snprintf(buf+len,
			PAGE_SIZE - len, "F0 = left:%d right:%d\n",
			aw882xx->cali.f0[0], aw882xx->cali.f0[1]);
	} else {
		len += snprintf(buf+len,
			PAGE_SIZE - len, "Re = %d\n",
			aw882xx->cali.re[aw882xx->chan_info.channel]);
		len += snprintf(buf+len,
			PAGE_SIZE - len, "F0 = %d\n",
			aw882xx->cali.f0[aw882xx->chan_info.channel]);
	}

	return len;
}

static ssize_t aw882xx_cali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	if (strncmp("start_cali", buf, strlen("start_cali"))) {
		aw_dev_err(aw882xx->dev, "%s: not define cmd %s\n",
			__func__, buf);
		ret = -EINVAL;
		goto cali_failed;
	}

	ret = aw882xx_cali_start_up(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		goto cali_failed;
	}

	return count;

cali_failed:
	memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
	memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
	return ret;
}

static ssize_t aw882xx_cali_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret;

	ret = aw_cali_get_re(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
		return ret;
	}

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		len += snprintf(buf+len,
			PAGE_SIZE - len, "left:%d mOhms right:%d mOhms\n",
			aw882xx->cali.re[0], aw882xx->cali.re[1]);
	else
		len += snprintf(buf+len,
			PAGE_SIZE - len, "%d\n",
			aw882xx->cali.re[aw882xx->chan_info.channel]);

	return len;
}

static ssize_t aw882xx_cali_re_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	if (strncmp("start_cali_re", buf, strlen("start_cali_re"))) {
		aw_dev_err(aw882xx->dev, "%s: not define cmd %s\n",
			__func__, buf);
		ret = -EINVAL;
		goto cali_re_failed;
	}

	ret = aw_cali_get_re(aw882xx);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		goto cali_re_failed;
	}

	return count;

cali_re_failed:
	memset(aw882xx->cali.re, 0, sizeof(aw882xx->cali.re));
	return ret;
}

static ssize_t aw882xx_cali_f0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret;

	ret = aw_cali_get_f0(aw882xx, false);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
		return ret;
	}

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		len += snprintf(buf+len,
			PAGE_SIZE - len, "left:%d right:%d\n",
			aw882xx->cali.f0[0], aw882xx->cali.f0[1]);
	else
		len += snprintf(buf+len,
			PAGE_SIZE - len, "%d\n",
			aw882xx->cali.f0[aw882xx->chan_info.channel]);

	return len;
}


static ssize_t aw882xx_cali_f0_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	if (strncmp("start_cali_f0", buf, strlen("start_cali_f0"))) {
		aw_dev_err(aw882xx->dev, "%s: not define cmd %s\n",
			__func__, buf);
		ret = -EINVAL;
		goto cali_f0_failed;
	}

	ret = aw_cali_get_f0(aw882xx, false);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: cali failed\n", __func__);
		goto cali_f0_failed;
	}

	return count;

cali_f0_failed:
	memset(aw882xx->cali.f0, 0, sizeof(aw882xx->cali.f0));
	return ret;
}

static ssize_t aw882xx_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		len += snprintf(buf+len,
			PAGE_SIZE - len, "left:%d mOhms right:%d mOhms\n",
			aw882xx->cali.re[0], aw882xx->cali.re[1]);
	else
		len += snprintf(buf+len,
			PAGE_SIZE - len, "%d\n",
			aw882xx->cali.re[aw882xx->chan_info.channel]);

	return len;
}

static ssize_t aw882xx_f0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	if (aw882xx->cali.end_cali_num - aw882xx->cali.first_cali_num)
		len += snprintf(buf+len,
			PAGE_SIZE - len, "left:%d right:%d\n",
			aw882xx->cali.f0[0], aw882xx->cali.f0[1]);
	else
		len += snprintf(buf+len,
			PAGE_SIZE - len, "%d\n",
			aw882xx->cali.f0[aw882xx->chan_info.channel]);

	return len;
}

static ssize_t aw882xx_f0_q_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct f0_q_data data;
	int ret = -1;
	ssize_t len = 0;

	ret = aw_get_f0_q(&data, sizeof(struct f0_q_data), aw882xx->chan_info.channel);
	if (ret < 0)
		return ret;

	if (aw882xx->cali.first_cali_num == aw882xx->cali.end_cali_num) {
		if (aw882xx->chan_info.channel == AW882XX_CHANNLE_LEFT_MONO) {
			len += snprintf(buf+len, PAGE_SIZE - len,
				"f0:%d q:%d\n", data.left_f0, data.left_q);
		} else if (aw882xx->chan_info.channel == AW882XX_CHANNLE_RIGHT) {
			len += snprintf(buf+len, PAGE_SIZE - len,
				"f0:%d q:%d\n", data.right_f0, data.right_q);
		} else {
			aw_dev_err(aw882xx->dev, "%s: channel:%d unsupported\n",
					__func__, aw882xx->chan_info.channel);
			return -EINVAL;
		}
	} else {
		len += snprintf(buf+len, PAGE_SIZE - len, "[left]f0:%d q:%d [right]f0:%d q:%d\n",
			data.left_f0, data.left_q, data.right_f0, data.right_q);
	}

	return len;
}


static ssize_t aw882xx_dsp_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	int ret = -1;
	ssize_t len = 0;
	uint32_t re = 0;

	ret = aw_read_data_from_dsp(INDEX_PARAMS_ID_RX_RE,
				&re, sizeof(uint32_t),
				aw882xx->chan_info.channel);
	if (ret < 0)
		aw_dev_err(aw882xx->dev, "%s : get dsp re failed\n",
			__func__);

	re = FIXED_RE_TO_MOHM(re);
	len += snprintf(buf+len, PAGE_SIZE-len, "dsp_re:%d\n", re);

	return len;
}

static ssize_t aw882xx_dsp_re_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	int32_t data;
	int32_t cali_re;

	ret = kstrtoint(buf, 0, &data);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s, read buf %s failed\n",
			__func__, buf);
		return ret;
	}

	cali_re = MOHM_TO_FIXED_RE(data);

	ret = aw882xx_store_cali_re(aw882xx, cali_re);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: store cali re error\n",
			__func__);
		return -EPERM;
	}

	ret = aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_RE, (void *)&cali_re,
				sizeof(uint32_t), aw882xx->chan_info.channel);
	if (ret) {
		aw_dev_err(aw882xx->dev, "%s: write cali_re to dsp failed\n",
			__func__);
	}

	aw_dev_dbg(aw882xx->dev, "%s: re:0x%x",
			__func__, aw882xx->cali.cali_re);

	return count;
}

static DEVICE_ATTR(cali_time, S_IRUGO | S_IWUSR,
	aw882xx_cali_time_show, aw882xx_cali_time_store);
static DEVICE_ATTR(cali, S_IRUGO | S_IWUSR,
	aw882xx_cali_show, aw882xx_cali_store);
static DEVICE_ATTR(cali_re, S_IRUGO | S_IWUSR,
	aw882xx_cali_re_show, aw882xx_cali_re_store);
static DEVICE_ATTR(cali_f0, S_IRUGO | S_IWUSR,
	aw882xx_cali_f0_show, aw882xx_cali_f0_store);
static DEVICE_ATTR(re_show, S_IRUGO,
	aw882xx_re_show, NULL);
static DEVICE_ATTR(f0_show, S_IRUGO,
	aw882xx_f0_show, NULL);
static DEVICE_ATTR(f0_q, S_IRUGO,
	aw882xx_f0_q_show, NULL);
static DEVICE_ATTR(dsp_re, S_IWUSR | S_IRUGO,
	aw882xx_dsp_re_show, aw882xx_dsp_re_store);


static struct attribute *aw882xx_cali_attr[] = {
	&dev_attr_cali_time.attr,
	&dev_attr_cali.attr,
	&dev_attr_cali_re.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_re_show.attr,
	&dev_attr_f0_show.attr,
	&dev_attr_f0_q.attr,
	&dev_attr_dsp_re.attr,
	NULL
};

static struct attribute_group aw882xx_cali_attr_group = {
	.attrs = aw882xx_cali_attr
};

static void aw_cali_attr_init(struct aw882xx *aw882xx)
{
	int ret;

	ret = sysfs_create_group(&aw882xx->dev->kobj, &aw882xx_cali_attr_group);
	if (ret < 0) {
		aw_dev_info(aw882xx->dev, "%s error creating sysfs cali attr files\n",
			__func__);
	}
}

static void aw_cali_attr_deinit(struct aw882xx *aw882xx)
{
	aw_dev_info(aw882xx->dev, "%s attr files deinit\n", __func__);
}

void aw_cali_init(struct aw_cali *cali)
{
	struct aw882xx *aw882xx =
			container_of(cali, struct aw882xx, cali);

	aw_dev_info(aw882xx->dev, "%s enter\n", __func__);

	aw882xx->cali.cali_re_time = AW_CALI_RE_DEFAULT_TIMER;
#ifdef CONFIG_DEBUG_FS
	if (cali->cali_mode == AW_CALI_MODE_DBGFS)
		aw_cali_debugfs_init(aw882xx);
	else
#endif
	if (cali->cali_mode == AW_CALI_MODE_MISC)
		aw_cali_misc_init(aw882xx);

	aw_cali_attr_init(aw882xx);

#ifdef CONFIG_VIVO_PORT_SMARTPA
	cali->cali_result = 0;
	if (aw882xx_vivo == NULL) {
	aw882xx_vivo = aw882xx;
	smartpa_debug_probe(aw882xx->i2c);
	smartpa_parse_dt(aw882xx);
		aw_dev_info(aw882xx->dev, "vivo cali func regsiter OK\n");
	}
#endif

#ifdef CONFIG_VIVO_SMARTPA_NEW
	printk("[smartpa] : %s, probe begin==>\n", __func__);
	smartpa_priv = aw882xx;
	vivo_smartpa_debug_probe(&cali_ops);
	printk("[smartpa] : %s, probe end<===\n", __func__);
	vivo_smartpa_parse_dt();
#endif
}

void aw_cali_deinit(struct aw_cali *cali)
{
	struct aw882xx *aw882xx =
			container_of(cali, struct aw882xx, cali);

	aw_dev_info(aw882xx->dev, "%s enter\n", __func__);
#ifdef CONFIG_DEBUG_FS
	if (cali->cali_mode == AW_CALI_MODE_DBGFS)
		aw_cali_debugfs_deinit(aw882xx);
	else 
#endif
	if (cali->cali_mode == AW_CALI_MODE_MISC)
		aw_cali_misc_deinit(aw882xx);

	aw_cali_attr_deinit(aw882xx);
}

/*****************************************************
 *
 * device tree parse cali mode
 *
 *****************************************************/
void aw882xx_parse_cali_mode_dt(struct aw_cali *cali)
{
	int ret = -1;
	const char *cali_mode_str = NULL;
	struct aw882xx *aw882xx =
			container_of(cali, struct aw882xx, cali);
	struct device_node *np = aw882xx->dev->of_node;

	ret = of_property_read_string(np, "aw-cali-mode", &cali_mode_str);
	if (ret < 0) {
		dev_info(aw882xx->dev, "%s: aw-cali-mode get failed ,user default attr way\n",
				__func__);
		cali->cali_mode = AW_CALI_MODE_NONE;
		return;
	}
#ifdef CONFIG_DEBUG_FS
	if (!strcmp(cali_mode_str, "aw_debugfs"))
		cali->cali_mode = AW_CALI_MODE_DBGFS;
	else
#endif
	if (!strcmp(cali_mode_str, "aw_misc"))
		cali->cali_mode = AW_CALI_MODE_MISC;
	else
		cali->cali_mode = AW_CALI_MODE_NONE;

	aw_dev_info(aw882xx->dev, "%s:cali mode str:%s num:%d\n",
			__func__, cali_mode_str, aw882xx->cali.cali_mode);
}

static void aw882xx_dev_cali_init(struct aw882xx *aw882xx, bool stereo_cali_en)
{
	struct aw_cali *cali = &aw882xx->cali;

	if (stereo_cali_en && aw882xx->chan_info.name_suffix) {
		cali->first_cali_num = AW882XX_CHANNLE_LEFT_MONO;
		cali->end_cali_num = AW882XX_CHANNLE_RIGHT;
		aw_dev_info(aw882xx->dev, "%s:use stereo channel cali\n",
			__func__);
	} else {
		cali->first_cali_num =
			cali->end_cali_num = aw882xx->chan_info.channel;
		aw_dev_info(aw882xx->dev, "%s:use mono channel cali\n",
			__func__);
	}
}

void aw882xx_parse_cali_way_dt(struct aw_cali *cali)
{
	bool aw_stereo_en;
	struct aw882xx *aw882xx =
			container_of(cali, struct aw882xx, cali);
	struct device_node *np = aw882xx->dev->of_node;

	aw_stereo_en = of_property_read_bool(np, "aw-stereo-cali");

	aw882xx_dev_cali_init(aw882xx, aw_stereo_en);
}
