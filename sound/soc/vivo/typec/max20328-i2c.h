
/*
 * Copyright (C) 2020 Samsung System LSI, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __MAX_20328_H__
#define __MAX_20328_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/version.h>
#include <linux/power_supply.h>
#include "tcpm.h"
#include <fsa4480-i2c.h>

// the pack pragma is required to make that the size in memory
// matches the actual variable lenghts
// This is to assure that the binary files can be transported between
// different platforms.
#pragma pack (push, 1)


enum max20328b_mode {
	MAX20328B_UART,             /* 0x06 MODE[2:0] = 100 */
	MAX20328B_AUDIO_ACCESSORY,	/* 0x06 MODE[2:0] = 110 */
	MAX20328B_AUDIO_MANUAL_1,
	MAX20328B_AUDIO_MANUAL_2,
	MAX20328B_AUDIO_MANUAL_SETMODE,
	MAX20328B_AUDIO_MANUAL_MG,
	MAX20328B_AUDIO_MANUAL_LRMG,
	MAX20328B_AUDIO_MANUAL_GM,
	MAX20328B_AUDIO_MANUAL_LRGM,
	MAX20328B_AUDIO_MANUAL_DATACONNCET,
};

typedef struct{
	struct delayed_work work;
	uint8_t mode;
} max20328b_work ;


enum max_function {
#if 1
    MAX_MIC_GND_SWAP,
    MAX_USBC_AUIDO_HP_ON,
    MAX_USBC_AUIDO_HP_OFF,
    MAX_USBC_ORIENTATION_CC1,
    MAX_USBC_ORIENTATION_CC2,
    MAX_USBC_DISPLAYPORT_DISCONNECTED,
    MAX_USBC_FAST_CHARGE_SELECT,
    MAX_USBC_FAST_CHARGE_EXIT,
    MAX_USBC_SWITCH_ENABLE,
    MAX_USBC_SWITCH_DISABLE,
    MAX_USBC_SWITCH_SBU_DIRECT_CONNECT,    //SBU1_H=SBU1?갨BU2_H=SBU2
    MAX_USBC_SWITCH_SBU_FLIP_CONNECT,    //SBU1_H=SBU2?갨BU2_H=SBU1
    MAX_USBC_SWITCH_SBU_HIZ,
	MAX_USBC_AUDIO_REPORT_IN,
	MAX_USBC_AUDIO_REPORT_REMOVE,
    MAX_EVENT_MAX,
#else
	FSA_MIC_GND_SWAP,
	FSA_USBC_AUIDO_HP_ON,
	FSA_USBC_AUIDO_HP_OFF,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	FSA_USBC_FAST_CHARGE_SELECT,
	FSA_USBC_FAST_CHARGE_EXIT,
	FSA_USBC_SWITCH_ENABLE,
	FSA_USBC_SWITCH_DISABLE,
	FSA_USBC_SWITCH_SBU_DIRECT_CONNECT,	//SBU1_H=SBU1¡¢SBU2_H=SBU2
	FSA_USBC_SWITCH_SBU_FLIP_CONNECT,	//SBU1_H=SBU2¡¢SBU2_H=SBU1
	FSA_USBC_SWITCH_SBU_HIZ,
	FSA_USBC_AUDIO_REPORT_IN,
	FSA_USBC_AUDIO_REPORT_REMOVE,
	FSA_EVENT_MAX,
#endif
};

enum max20328_regs_def {
	MAX20328_DEVICE_ID       = 0x00,
	MAX20328_ADC_VAL         = 0x01,
	MAX20328_STATUS1         = 0x02,
	MAX20328_STATUS2         = 0x03,
	MAX20328_INTERRUPT       = 0x04,
	MAX20328_MASK            = 0x05,
	MAX20328_CONTROL1        = 0x06,
	MAX20328_CONTROL2        = 0x07,
	MAX20328_CONTROL3        = 0x08,
	MAX20328_ADC_CONTROL1    = 0x09,
	MAX20328_ADC_CONTROL2    = 0x0A,
	MAX20328_HIHS_VAL        = 0x0B,
	MAX20328_OMTP_VAL        = 0x0C,
	MAX20328_SW_DEFLT1       = 0x0D,
	MAX20328_SW_DEFLT2       = 0x0E,
	MAX20328_REG_MAX,
};

static u8 max20328_regs[] = {
	MAX20328_DEVICE_ID,
	MAX20328_ADC_VAL,
	MAX20328_STATUS1,
	MAX20328_STATUS2,
	MAX20328_INTERRUPT,
	MAX20328_MASK,
	MAX20328_CONTROL1,
	MAX20328_CONTROL2,
	MAX20328_CONTROL3,
	MAX20328_ADC_CONTROL1,
	MAX20328_ADC_CONTROL2,
	MAX20328_HIHS_VAL,
	MAX20328_OMTP_VAL,
	MAX20328_SW_DEFLT1,
	MAX20328_SW_DEFLT2,
};


/* Indicates USB Type-C CC connection status */
//enum typec_cc_mode {
//	POWER_SUPPLY_TYPEC_NONE,
//	POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER,	/* Ra/Ra */
//};

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
#define ACCDET_TCPC_VOTER		"ACCDET_TCPC_VOTER"
extern void vote_typec_drp(const char *voter, bool drp);
extern int get_typec_drp(void);
extern void vote_drp_enable(bool enable);
#endif

int get_usbc_mg_status(void);
extern int max20328_switch_mode_event(enum max_function event);
int max20328_notify_mos(int state); /*audio_v add*/
int max20328_switch_status_restore(void);


#pragma pack (pop)
#endif /* __MAX_20328_H__*/
