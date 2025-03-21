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

/*
 * DW9800VAF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "DW9800VAF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18
#define _SLV_OIS_ 0xE4

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C write failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;

	i2c_read(addr, &get_byte);

	return get_byte;
}

static int s4DW9800WAF_ReadReg(unsigned short *a_pu2Result)
{
	*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

/* DW9800VAF OIS CODE START*/
int s4AF_WriteReg_DW9800VAF(unsigned short i2c_id, unsigned char *a_pSendData,
			    unsigned short a_sizeSendData)
{
	int i4RetValue = 0;

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	i4RetValue =
		i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);

	if (i4RetValue != a_sizeSendData) {
		printk("[DW9781OISAF] I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
			a_pSendData[0], a_pSendData[1]);
		return -1;
	}

	return 0;
}

int s4AF_ReadReg_DW9800VAF(unsigned short i2c_id, unsigned char *a_pSendData,
			   unsigned short a_sizeSendData,
			   unsigned char *a_pRecvData,
			   unsigned short a_sizeRecvData)
{
	int i4RetValue;
	struct i2c_msg msg[2];

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue =
		i2c_transfer(g_pstAF_I2Cclient->adapter, msg, ARRAY_SIZE(msg));

	if (i4RetValue != 2) {
		printk("[DW9781OISAF] I2C Read failed!!\n");
		return -1;
	}
	return 0;
}


void DW9800VAF_WR_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat)
{
	s4AF_WriteReg_DW9800VAF(slvadr, dat, size);
}

unsigned short int DW9800VAF_RD_I2C(unsigned char slvadr, unsigned char size,unsigned char *dat)
{
	unsigned short int read_data = 0;
	unsigned short int read_data_h = 0;

	s4AF_ReadReg_DW9800VAF(slvadr, dat, 2,(unsigned char *)&read_data, 2);

	read_data_h = read_data >> 8;
	read_data = read_data << 8;
	read_data = read_data | read_data_h;

	return read_data;
}

/*void DW9800VAF_I2C_OIS_per_write(unsigned short int u16_adr, unsigned short int u16_dat)
{
	unsigned char out[4];

	out[0] = (char)(u16_adr >> 8);
	out[1] = (char)(u16_adr & 0xFF);
	out[2] = (char)(u16_dat >> 8);
	out[3] = (char)(u16_dat & 0xFF);

	DW9800VAF_WR_I2C(_SLV_OIS_, 4, out);

}
*/
unsigned short int DW9800VAF_I2C_OIS_per_read(unsigned short int u16_adr)
{

	unsigned char u08_dat[2];

	u08_dat[0] = (char)(u16_adr >> 8);; /*  */
	u08_dat[1] = (char)(u16_adr & 0xFF);	/* target address */

	return DW9800VAF_RD_I2C(_SLV_OIS_, 2, u08_dat);
}


void DW9800VAF_setOISMode(int Disable)
{
	//DW9800VAF_I2C_OIS_per_write(0x7015, Disable); /*ois OFF servo on ,*/
	printk("[DW9781OISAF] ois setOISMode\n");
}


void DW9800VAF_OIS_Standby(void)
{
	printk("[DW9781OISAF] ois standby\n");
}

/* MAIN OIS */
void DW9800VAF_Main_OIS(void)
{
	printk("[DW9781OISAF] ois reset start\n");
}

static inline int setAFPara(__user struct stAF_MotorCmd *pstMotorCmd)
{
	struct stAF_MotorCmd stMotorCmd;

	if (copy_from_user(&stMotorCmd, pstMotorCmd, sizeof(stMotorCmd)))
		printk("[DW9781OISAF] copy to user failed when getting motor command\n");

	printk("[DW9781OISAF] Motor CmdID : %x\n", stMotorCmd.u4CmdID);

	printk("[DW9781OISAF] Motor Param : %x\n", stMotorCmd.u4Param);

	switch (stMotorCmd.u4CmdID) {
	case 1:
		DW9800VAF_setOISMode(stMotorCmd.u4Param); /* 1 : disable */
		break;
	case 2:

		break;
	}

	return 0;
}

/* DW9800VAF OIS CODE END*/


static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition >> 1;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		printk("[DW9781OISAF] copy to user failed when getting motor information\n");

	return 0;
}

static int initdrv(void)
{
	int i4RetValue = 0;
	char puSendCmd0[2] = { 0x02, 0x01 };
	char puSendCmd1[2] = { 0x02, 0x00 };
	char puSendCmd2[2] = { 0x02, 0x02 };
	char puSendCmd3[2] = { 0x06, 0x80 };
	char puSendCmd4[2] = { 0x07, 0x65 };
#if defined(CONFIG_MTK_CAM_PD2282)
	char puSendCmd5[2] = { 0x10, 0x00 };
#else
	char puSendCmd5[2] = { 0x10, 0x01 };
#endif
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	LOG_INF("+\n");
	mdelay(10);
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd0, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd0 send failed!!\n");
		return -1;
		}
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd1 send failed!!\n");
		return -1;
		}
		mdelay(1);
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd5, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd5 send failed!!\n");
		return -1;
		}
		mdelay(1);
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd2 send failed!!\n");
		return -1;
		}
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd3 send failed!!\n");
		return -1;
		}
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
		if (i4RetValue < 0) {
		LOG_INF("I2C puSendCmd4 send failed!!\n");
		return -1;
		}

		/*udelay(1000);*/
		/*LOG_INF("regiestor 0x2: 0x%x", read_data(0x02));*/
		/*LOG_INF("regiestor 0x6: 0x%x", read_data(0x06));*/
		/*LOG_INF("regiestor 0x7: 0x%x", read_data(0x07));*/

	LOG_INF("-\n");

	DW9800VAF_Main_OIS();
	return i4RetValue;

}



static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4DW9800WAF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}


	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);


	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9800VAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		printk("[DW9781OISAF] AFIOC_G_MOTORINFO\n");
		i4RetValue = getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		printk("[DW9781OISAF] AFIOC_T_MOVETO\n");
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		printk("[DW9781OISAF] AFIOC_T_SETINFPOS\n");
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		printk("[DW9781OISAF] AFIOC_T_SETMACROPOS\n");
		i4RetValue = setAFMacro(a_u4Param);
		break;
	case AFIOC_S_SETPARA:
		printk("[DW9781OISAF] AFIOC_T_SETMACROPOS\n");
		i4RetValue =
			setAFPara((__user struct stAF_MotorCmd *)(a_u4Param));
		break;
	default:
		//printk("[DW9781OISAF] No CMD\n");
		/*chenhan add for ois per frame operation check*/
#if 0//def CONFIG_MTK_CAM_PD2083F_EX
		i4RetValue = ois_interface_dispatcher(OIS_DW9781C, a_u4Command, (__user void *)(a_u4Param));
#else
		i4RetValue = -EPERM;
#endif
		/*add end*/

		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9800VAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2){

		LOG_INF("Wait\n");
		s4AF_WriteReg(512);
		msleep(20);
		DW9800VAF_OIS_Standby();
}
	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");
	return 0;
}

int DW9800VAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initdrv();

	return 1;
}

int DW9800VAF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	printk("[DW9781OISAF] FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
