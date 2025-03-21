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
#include "hi846hltpd2236mainmipiraw_Sensor.h"

#define PFX "hi846hltpd2236main_EEPROM_OTP"

#define LOG_INF(format,  args...)        pr_info(PFX "[%s] " format,  __func__,  ##args)
#define LOG_ERR(format,  args...)        pr_err(PFX "[%s] " format,  __func__,  ##args)

/**************/
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define VIVO_MAIN_EEPROM_WRITE_ID 0x44
#define VIVO_MAIN_I2C_SPEED 400


static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    kdSetI2CSpeed(VIVO_MAIN_I2C_SPEED);
    iWriteRegI2C(pu_send_cmd, 3, VIVO_MAIN_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    kdSetI2CSpeed(VIVO_MAIN_I2C_SPEED);

    iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, VIVO_MAIN_EEPROM_WRITE_ID);

    return get_byte;
}

typedef enum{
    OTP_TYPE_INFO = 0,
    OTP_TYPE_AWB = 1,
    OTP_TYPE_LSC = 2,
    OTP_TYPE_AF = 3,
    OTP_TYPE_SN = 4,
    OTP_TYPE_MATERIAL = 5,
    OTP_TYPE_MAX = 6,
} OTP_DATA_TYPE;

#define OTP_GROUP_NUMBER               0x3
#define OTP_DATATYPE_NUMBER               0x6  //module_info  awb lsc af sn material
#define OTP_ALL_DATA_SIZE 0x189b - 0x0201 + 2  //addr 0x200
#define OTP_DATA_SIZE 0x78d /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)+sizeof(sn)+sizeof(material)*/
const kal_uint32 OTP_GROUP_FLAG_VALUE[OTP_GROUP_NUMBER] = {0x01, 0x13, 0x37};
const kal_uint32 OTP_GROUP_FLAG_SEG[OTP_DATATYPE_NUMBER] = {0x0201, 0x022f, 0x0263,0x184b, 0x185b, 0x1883};
const kal_uint32 OTP_GROUP_CHECKSUM_SEG[OTP_DATATYPE_NUMBER] = {0x0210, 0x0240, 0x09b0, 0x1850, 0x1868, 0x188b};
const kal_uint32 OTP_GROUP_ALL_DATA_LENGTH[OTP_DATATYPE_NUMBER] = {16, 18, 1870, 6, 14, 9};
const kal_uint32 OTP_MAP_HEAD_SEG = 0x0200;

static unsigned char otp_data_kernel_read_hi846hltpd2236main[OTP_ALL_DATA_SIZE];
unsigned char otp_data_vendor_read_hi846hltpd2236main[OTP_DATA_SIZE];


static int otp_read_hi846hltpd2236main(kal_uint16 addr)
{
    kal_uint32 i;

    write_cmos_sensor_8(0x0A02, 0x01);
    write_cmos_sensor_8(0x0A00, 0x00);
    mdelay(10);
    write_cmos_sensor_8(0x0F02, 0x00);
    write_cmos_sensor_8(0x071A, 0x01);
    write_cmos_sensor_8(0x071B, 0x09);
    write_cmos_sensor_8(0x0D04, 0x01);
    write_cmos_sensor_8(0x0D00, 0x07);
    write_cmos_sensor_8(0x003E, 0x10);
    write_cmos_sensor_8(0x0A00, 0x01);
    mdelay(1);

    write_cmos_sensor_8(0x70A, ((addr)>>8)&0xff);/*start address H*/
    write_cmos_sensor_8(0x70B, (addr)&0xff);/*start address L*/
    write_cmos_sensor_8(0x702, 0x01);/*single read*/

    for (i = 0; i < OTP_ALL_DATA_SIZE ; i++)
    {
	    otp_data_kernel_read_hi846hltpd2236main[i] = read_cmos_sensor(0x708);
    }
    write_cmos_sensor_8(0x0A00, 0x00);      // stand by on
    mdelay(10);
    write_cmos_sensor_8(0x003E, 0x00);      // display mode
    write_cmos_sensor_8(0x0A00, 0x01);      // stand by off
    return 0;
}

kal_int16 copy_valid_otp_data()
{
    kal_uint32 current_group_number = 0 ;
    kal_uint32 current_data_head = 0 ;
    kal_uint32 data_head = 0 ;
    kal_uint32 data_length = 0 ;
    kal_uint32  i = 0;

    for(i = 0; i < OTP_TYPE_MAX;i++)
    {
        //get group number
        for(current_group_number = 0; current_group_number < OTP_GROUP_NUMBER; current_group_number++)
        {
            if(OTP_GROUP_FLAG_VALUE[current_group_number] == otp_data_kernel_read_hi846hltpd2236main[OTP_GROUP_FLAG_SEG[i]-OTP_MAP_HEAD_SEG])
            {
                otp_data_vendor_read_hi846hltpd2236main[current_data_head] = 1; //flag
                LOG_INF("current data type [%d]  group [%d]",i,current_group_number);
                break;
            }
        }
        if(current_group_number < OTP_GROUP_NUMBER)
        {
            data_head = OTP_GROUP_FLAG_SEG[i] - OTP_MAP_HEAD_SEG + current_group_number * (OTP_GROUP_ALL_DATA_LENGTH[i] -1 ) + 1; //skip flag
            data_length = OTP_GROUP_ALL_DATA_LENGTH[i] -1 ; //skip flag
            //read data
            memcpy(&otp_data_vendor_read_hi846hltpd2236main[current_data_head+1],&otp_data_kernel_read_hi846hltpd2236main[data_head],data_length);
        }
        else
        {
            LOG_ERR("current data type [%d]  group [%d] is invalid data",i,current_group_number);
        }
        current_data_head += OTP_GROUP_ALL_DATA_LENGTH[i];
    }
	return 0;
}

int vivo_main_otp_read_hi846hltpd2236main(void)
{
    otp_read_hi846hltpd2236main(OTP_MAP_HEAD_SEG);
    copy_valid_otp_data();
    return true;
}