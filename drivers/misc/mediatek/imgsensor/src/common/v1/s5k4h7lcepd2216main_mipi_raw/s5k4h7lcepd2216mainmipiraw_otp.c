#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "s5k4h7lcepd2216mainmipiraw_Sensor.h"

#define PFX "walf S5K4H7LCEPD2216MAIN_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_info(PFX "[%s] " format,  __func__,  ##args)

/**************/

#define VIVO_SUB_EEPROM_WRITE_ID (0x20)

#define S5K4H7LCEPD2216MAIN_PAGE_BASE_ADDR	(0x0A04)
#define S5K4H7LCEPD2216MAIN_PAGE_SIZE		(0x40)
#define S5K4H7LCEPD2216MAIN_PAGE_COUNT		(116 - 21 + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_START		(21)

#define S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_ADDR	(0x00)
#define S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_SIZE	(0xA16 - 0xA04 + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_ADDR	(0xA17 - 0x0A04)
#define S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE	(0xA28 - 0xA17 + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_ADDR	(0x00)
#define S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE	(1868 + 2)
#define S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_ADDR	(0x00)
#define S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE	(0x0A0C - 0x0A04 + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_ADDR	(0x0A0D - 0x0A04)
#define S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE	(0x0A1D - 0x0A0D + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_ADDR	(0x0A1E - 0x0A04)
#define S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE	(0x0A26 - 0x0A1E + 1)
#define S5K4H7LCEPD2216MAIN_PAGE_VALID_INFO_SIZE	(S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_SIZE + \
													S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE + \
													S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE + \
													S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE + \
													S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE + \
													S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE)

#define S5K4H7LCEPD2216MAIN_FLAG_VALID		(0x01)

#define VIVO_MAIN_OTP_DATA_SIZE (S5K4H7LCEPD2216MAIN_PAGE_SIZE*S5K4H7LCEPD2216MAIN_PAGE_COUNT)
static unsigned char vivo_main_otp_all_data[VIVO_MAIN_OTP_DATA_SIZE];
unsigned char otp_data_vendor_read_s5k4h7lcepd2216main[S5K4H7LCEPD2216MAIN_PAGE_VALID_INFO_SIZE];
kal_uint8 module_info_pages[] = {21, 53, 85};
kal_uint8 awb_info_pages[] = {21, 53, 85};
kal_uint8 lsc_info_pages[] = {22, 54, 86};
kal_uint8 af_info_pages[] = {52, 84, 116};
kal_uint8 sn_info_pages[] = {52, 84, 116};
kal_uint8 matrial_info_pages[] = {52, 84, 116};


typedef enum{
	TYPE_MODULE,
	TYPE_AWB,
	TYPE_LSC,
	TYPE_AF,
	TYPE_SN,
	TYPE_MATERIAL,
}S5k4h7_OTP_Info_Type;


static void write_cmos_sensor_16_8(kal_uint32 addr, kal_uint32 para)
{
  char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
  //	kdSetI2CSpeed(400); 
  iWriteRegI2C(pu_send_cmd, 3, VIVO_SUB_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
  kal_uint16 get_byte = 0;
  char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

  iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, VIVO_SUB_EEPROM_WRITE_ID);
  return get_byte;
}

static int s5k4h7lce_main_otp_read_setting_init_continuous_read()
{
	int i = 0, j = 0;
	write_cmos_sensor_16_8(0x0100, 0x01);
	mdelay(50);
	for(j = 0; j < S5K4H7LCEPD2216MAIN_PAGE_COUNT; j++){
		write_cmos_sensor_16_8(0x0A02, (S5K4H7LCEPD2216MAIN_PAGE_START + j));
		write_cmos_sensor_16_8(0x0A00, 0x01);
		mdelay(3);//page == 64 bytes
		for (i = 0; i < S5K4H7LCEPD2216MAIN_PAGE_SIZE; i++) {
			vivo_main_otp_all_data[i +S5K4H7LCEPD2216MAIN_PAGE_SIZE*j] = read_cmos_sensor_16_8(S5K4H7LCEPD2216MAIN_PAGE_BASE_ADDR +i);
		}
	}
	write_cmos_sensor_16_8(0x0A00, 0x04);	
	write_cmos_sensor_16_8(0x0A00, 0x00);
	return 0;
}

static int s5k4h7lce_read_group_otp_data(S5k4h7_OTP_Info_Type info_type, unsigned char *out_info_addr, kal_uint16 out_info_size){
	kal_uint8 *info_pages = NULL;
	kal_uint16 info_size = 0;
	kal_uint8 info_addr_offset = 0;
	kal_uint8 i = 0;

	if (out_info_addr == NULL || out_info_size < 0) {
		return 0;
	}
	switch (info_type) {
	case TYPE_MODULE:
		info_pages = module_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_ADDR;
		break;
	case TYPE_AWB:
		info_pages = awb_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_ADDR;
		break;
	case TYPE_LSC:
		info_pages = lsc_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_ADDR;
		break;
	case TYPE_AF:
		info_pages = af_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_ADDR;
		break;
	case TYPE_SN:
		info_pages = sn_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_ADDR;
		break;
	case TYPE_MATERIAL:
		info_pages = matrial_info_pages;
		info_size = S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE;
		info_addr_offset = S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_ADDR;
		break;
	default:
		break;
	};
	if (info_pages == NULL ||  info_size == 0 || info_size > out_info_size) {
		LOG_INF("pameter error\n");
		return 0;
	}

	for (i = 0; i < 3; i ++) {
		kal_uint16 addr = (info_pages[i] - S5K4H7LCEPD2216MAIN_PAGE_START)*S5K4H7LCEPD2216MAIN_PAGE_SIZE + info_addr_offset;  
		if (vivo_main_otp_all_data[addr] == S5K4H7LCEPD2216MAIN_FLAG_VALID) {
			memcpy(out_info_addr, vivo_main_otp_all_data + addr, info_size);
			return info_size;
		}
	}
	return 0;
}

int S5K4H7LCEPD2216MAIN_otp_read(void)
{
	kal_uint16 offset = 0;
	kal_uint16 read_size = 0;
#if 0
	int i, j;
#endif

	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	/*if (is_atboot == 1) {
		LOG_INF("AT mode skip vivo_sub_otp_read\n");
		return 1;
	}
	*/
	/* guojunzheng add end */

	s5k4h7lce_main_otp_read_setting_init_continuous_read();
	LOG_INF("read_s5k4h7yx_data\n");

	#if 0
	for(j = 0; j < S5K4H7LCEPD2216MAIN_PAGE_COUNT; j++){
		for (i = 0 ; i < 0x40; i++) {
			LOG_INF("read_s5k4h7yx_data[%d][0x%x][%d] = 0x%x\n", (j + 21), (i + 0x0A04), (i +0x40*j), vivo_main_otp_all_data[i +0x40*j]);
		}
	}
	#endif

	/*moduleinfo  */
	read_size = s5k4h7lce_read_group_otp_data(TYPE_MODULE, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_MOD_INFO_SIZE) {
		LOG_INF("read mod info failed,size = %d", read_size);
		goto error;
	} else {
		otp_data_vendor_read_s5k4h7lcepd2216main[0] = otp_data_vendor_read_s5k4h7lcepd2216main[1];
		otp_data_vendor_read_s5k4h7lcepd2216main[1] = otp_data_vendor_read_s5k4h7lcepd2216main[2];
		otp_data_vendor_read_s5k4h7lcepd2216main[2] = otp_data_vendor_read_s5k4h7lcepd2216main[0x0A0F - 0x0A04];
		otp_data_vendor_read_s5k4h7lcepd2216main[3] = otp_data_vendor_read_s5k4h7lcepd2216main[0x0A10 - 0x0A04];
		offset += 4;
	}
	read_size = s5k4h7lce_read_group_otp_data(TYPE_AWB, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE) {
		LOG_INF("read awb info failed,size = %d", read_size);
		goto error;
	}else {
		offset += S5K4H7LCEPD2216MAIN_PAGE_AWB_INFO_SIZE;
	}

	read_size = s5k4h7lce_read_group_otp_data(TYPE_LSC, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE) {
		LOG_INF("read lsc info failed,size = %d", read_size);
		goto error;
	}else {
		offset += S5K4H7LCEPD2216MAIN_PAGE_LCS_INFO_SIZE;
	}

	read_size = s5k4h7lce_read_group_otp_data(TYPE_AF, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE) {
		LOG_INF("read af info failed,size = %d", read_size);
		goto error;
	}else {
		offset += S5K4H7LCEPD2216MAIN_PAGE_AF_INFO_SIZE;
	}
	read_size = s5k4h7lce_read_group_otp_data(TYPE_SN, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE) {
		LOG_INF("read sn info failed,size = %d", read_size);
		goto error;
	}else {
		offset += S5K4H7LCEPD2216MAIN_PAGE_SN_INFO_SIZE;
	}
	read_size = s5k4h7lce_read_group_otp_data(TYPE_MATERIAL, otp_data_vendor_read_s5k4h7lcepd2216main + offset, S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE);
	if (read_size != S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE) {
		LOG_INF("read material info failed,size = %d", read_size);
		goto error;
	}else {
		offset += S5K4H7LCEPD2216MAIN_PAGE_MATERIAL_INFO_SIZE;
	}


	return 1;
error:
	LOG_INF("read_s5k4h7yx_data failed\n");
	return 0;
}
/*vivo cfx add end*/
