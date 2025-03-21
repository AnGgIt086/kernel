#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>


#include "../imgsensor_i2c.h"
#include "gc08a3lcepd2236mainmipiraw_Sensor.h"

#define PFX "gc08a3lcepd2236main_EEPROM_OTP"

#define LOG_INF(format,  args...)        pr_info(PFX "[%s] " format,  __func__,  ##args)
#define LOG_ERR(format,  args...)        pr_err(PFX "[%s] " format,  __func__,  ##args)

/**************/
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define VIVO_MAIN_EEPROM_WRITE_ID 0x62
#define VIVO_MAIN_I2C_SPEED 400
#define GC08A3_OTP_DEBUG  0

static u16 read_cmos_sensor(u16 addr)
{
	u16 get_byte=0;
	char pu_send_cmd[2] = { (char)((addr >> 8) & 0xff), (char)(addr & 0xff) };
    //kdSetI2CSpeed(VIVO_MAIN_I2C_SPEED);
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, VIVO_MAIN_EEPROM_WRITE_ID);

	return get_byte;
}

static void write_cmos_sensor(u16 addr, u8 para)
{
	char pu_send_cmd[3] = { (char)((addr >> 8) & 0xff), (char)(addr & 0xff), (char)(para & 0xff) };
    //kdSetI2CSpeed(VIVO_MAIN_I2C_SPEED);
	iWriteRegI2C(pu_send_cmd, 3, VIVO_MAIN_EEPROM_WRITE_ID);
}

typedef enum{
    OTP_TYPE_INFO = 0,
    OTP_TYPE_FUSEID = 1,
    OTP_TYPE_SN = 2,
    OTP_TYPE_AWB = 3,
    OTP_TYPE_LSC = 4,
    OTP_TYPE_AF = 5,
    OTP_TYPE_MAX = 6,
} GC08A3_OTP_DATA_TYPE;

#define GC08A3_OTP_GROUP_NUMBER            3
#define GC08A3_OTP_DATA_SIZE               2017 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)+sizeof(sn)+sizeof(fuseid)*/
#define GC08A3_OTP_ALL_DATA_SIZE           GC08A3_OTP_DATA_SIZE * GC08A3_OTP_GROUP_NUMBER

const kal_uint32 GC08A3_OTP_MAP_HEAD_ADDR = 0X15A0;
const kal_uint32 GC08A3_OTP_GROUP_FLAG_VALUE = 0x40;
const kal_uint32 GC08A3_OTP_GROUP_DATA_LENGTH[OTP_TYPE_MAX] = {32, 36, 33, 24, 1870, 22}; //GC08A3_OTP_DATA_SIZE
const kal_uint32 GC08A3_OTP_GROUP_FLAG_INDEX[OTP_TYPE_MAX * GC08A3_OTP_GROUP_NUMBER] =
        //info_group_index, fuseid_group_index, sn_group_index, awb_group_index, lsc_group_index, af_group_index
        {0, 2017, 4034, 32, 2049, 4066, 68, 2085, 4102, 101, 2118, 4135, 125, 2142, 4159, 1995, 4012, 6029};

static unsigned char otp_data_kernel_read_gc08a3lcepd2236main[GC08A3_OTP_ALL_DATA_SIZE];
unsigned char otp_data_vendor_read_gc08a3lcepd2236main[GC08A3_OTP_DATA_SIZE];

static int otp_read_gc08a3lcepd2236main(kal_uint16 addr)
{
    kal_uint32 i;

    //init
	write_cmos_sensor(0x0324, 0x42);
	write_cmos_sensor(0x0316, 0x09);
	write_cmos_sensor(0x0a67, 0x80);
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a53, 0x0e);
	write_cmos_sensor(0x0a65, 0x17);
	write_cmos_sensor(0x0a68, 0xa1);
	write_cmos_sensor(0x0a47, 0x00);
	write_cmos_sensor(0x0a58, 0x00);
	write_cmos_sensor(0x0ace, 0x0c);
    mdelay(1);
    //
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a69, (addr >> 8) & 0xff);
	write_cmos_sensor(0x0a6a, addr & 0xff);
	write_cmos_sensor(0x0313, 0x20);
	write_cmos_sensor(0x0313, 0x12);

    for (i = 0; i < GC08A3_OTP_ALL_DATA_SIZE ; i++)
    {
        otp_data_kernel_read_gc08a3lcepd2236main[i] = read_cmos_sensor(0x0a6c);
#if GC08A3_OTP_DEBUG
        LOG_INF("otp_kernel i:%d addr:0x%x, data:0x%x\n", i, addr + i * 8, otp_data_kernel_read_gc08a3lcepd2236main[i]);
#endif
    }

    mdelay(10);
    //close
	write_cmos_sensor(0x0316, 0x01);
	write_cmos_sensor(0x0a67, 0x00);
    return 0;
}

kal_int16 copy_valid_otp_data_gc08a3()
{
    kal_uint32 i = 0;
    kal_uint32 j = 0;
    kal_uint32 k = 0;
    kal_uint32 l = 0;

    for(i = 0; i < OTP_TYPE_MAX;i++)
    {
        //get valid group
        for(j = 0; j < GC08A3_OTP_GROUP_NUMBER; j++)
        {
            LOG_INF("current data type[%d] group[%d] data[0x%x]",i, j, otp_data_kernel_read_gc08a3lcepd2236main[GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j]]);
            if(GC08A3_OTP_GROUP_FLAG_VALUE == otp_data_kernel_read_gc08a3lcepd2236main[GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j]])
            {
                LOG_INF("current data type[%d] group[%d] index[%d] length[%d]",i, j, GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j], GC08A3_OTP_GROUP_DATA_LENGTH[i]);
                memcpy(&otp_data_vendor_read_gc08a3lcepd2236main[GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER]],
                    &otp_data_kernel_read_gc08a3lcepd2236main[GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j]], GC08A3_OTP_GROUP_DATA_LENGTH[i]);
				k = 0;
				l = 0;
#if GC08A3_OTP_DEBUG
				for(k = GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER];
					k < GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER] + GC08A3_OTP_GROUP_DATA_LENGTH[i];
					k++){
					LOG_INF("memcpy otp_kernel_index:%d otp_kernel[%d]:0x%x otp_vendor[%d]:0x%x",
					GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j] + l,
					k, otp_data_kernel_read_gc08a3lcepd2236main[GC08A3_OTP_GROUP_FLAG_INDEX[i * GC08A3_OTP_GROUP_NUMBER + j] + l],
					k, otp_data_vendor_read_gc08a3lcepd2236main[k]);
					l++;
				}
#endif
				break;
            }
        }
    }
	for (i = 0; i < GC08A3_OTP_DATA_SIZE ; i++){
#if GC08A3_OTP_DEBUG
		LOG_INF("otp_vendor addr = %d, data = 0x%x\n", i, otp_data_vendor_read_gc08a3lcepd2236main[i]);
#endif
	}
	return 0;
}

int vivo_main_otp_read_gc08a3lcepd2236main(void)
{
    otp_read_gc08a3lcepd2236main(GC08A3_OTP_MAP_HEAD_ADDR);
    copy_valid_otp_data_gc08a3();
    return true;
}
