/*
 * =====================================================================================
 *
 *       Filename:  vivo_haptic_core_user.h
 *
 *       Description:  Consistent with the vivo_haptic_core.h file of the hal layer
 *
 *       Created:  03/06/2023 11:01:47 AM
 *
 * =====================================================================================
 */

#ifndef __HAPTIC_CORE_USER_H__
#define __HAPTIC_CORE_USER_H__

enum ic_type {
	AW8624 = 0,
	AW86224,
	AW8697,
	AW86917,
	AW86927,
	AWINIC_MAX = 10,
	DRV2625,
	TI_MAX = 20,
	PM8350BH,
	PM8550BH,
	QCOM_MAX = 30,
	VIB_NONE = 0xFF,
};

enum motor_type {
	Z_1040 = 1040,
	Z_0832 = 832,
	X_0619 = 619,
	X_0815 = 815,
	X_9595 = 9595,
	X_0809 = 809,
	X_81540 = 81540,
};

enum ic_calibration_data_len {
	AWINIC_LEN = 2,
	TI_LEN = 4,
	QCOM_LEN = 3
};

enum haptic_effect_type {
	HAPTIC_CONSTANT,
	HAPTIC_CUSTOM,
	HAPTIC_RTP_STREAM,
};

enum haptic_write_event_type {
	HAPTIC_PLAY_EVENT,
	HAPTIC_GAIN_EVENT,
};

struct haptic_write_event {
	__s16           type;
	__s16           code;
	__s32           value;
};

struct custom_fifo_data {
	uint32_t        effect_id;
	uint32_t        length;
	uint32_t        play_rate_hz;
	const int8_t    *data;
};

struct haptic_effect {
	enum haptic_effect_type      type;
	__s16                 id; // caller identifier
	__s16                 magnitude;

	__s16                 length; // only for constant mode
	__u32                 data_count; // only for rtp stream mode
	__u32                 custom_len;
	__s16 __user         *custom_data;

};

struct haptic_cali_awinic {
	int f0;
	int f0_offset;
};

struct haptic_cali_ti {
	int f0;
	int CalComp;
	int CalBemf;
	int CalGain;
};

struct haptic_cali_qcom {
	uint32_t f0;
	uint32_t cl_t_lra_us;
	uint32_t rc_clk_cal_count;
};

struct haptic_cali_param {
	enum ic_calibration_data_len calibration_data_len;
	union {
		struct haptic_cali_awinic awinic;
		struct haptic_cali_ti ti;
		struct haptic_cali_qcom qcom;
	} u;
};

struct haptic_rtp_container{
	unsigned int len;
	unsigned char data[];
};

/* only used for vivo haptic core */
#define HAPTIC_CORE_IOCTL_MAGIC           'v'

#define HAPTIC_CORE_GET_VIB_COUNT         _IOR(HAPTIC_CORE_IOCTL_MAGIC, 11, int*)
/*****************************************************/

#define HAPTIC_IOCTL_MAGIC                'h'

#define HAPTIC_UPLOAD                     _IOWR(HAPTIC_IOCTL_MAGIC, 11, struct haptic_effect*)
#define HAPTIC_PLAYBACK                   _IOWR(HAPTIC_IOCTL_MAGIC, 12, int)
#define HAPTIC_STOP                       _IOWR(HAPTIC_IOCTL_MAGIC, 13, int)
#define HAPTIC_GAIN                       _IOWR(HAPTIC_IOCTL_MAGIC, 14, int)
#define HAPTIC_TRIGGER_INTENSITY          _IOW(HAPTIC_IOCTL_MAGIC, 15, int)
#define HAPTIC_JUDGE_EFFECT_SUPPORT       _IOWR(HAPTIC_IOCTL_MAGIC, 17, int*)
#define HAPTIC_GET_MOTOR_TYPE             _IOWR(HAPTIC_IOCTL_MAGIC, 18, int*)
#define HAPTIC_GET_F0                     _IOWR(HAPTIC_IOCTL_MAGIC, 19, int*) // 线性马达的实际f0，而非理论值
#define HAPTIC_DISPATCH_CALI_PARAM        _IOWR(HAPTIC_IOCTL_MAGIC, 20, struct haptic_cali_param *)
#define HAPTIC_GET_DRIVER_IC_TYPE         _IOWR(HAPTIC_IOCTL_MAGIC, 21, enum ic_type *)

#define HAPTIC_SUPPORT_BITMASK            _IOWR(HAPTIC_IOCTL_MAGIC, 16, unsigned char *)

/* only used for aac richtap begin */

#define RICHTAP_IOCTL_GROUP 0x52
#define RICHTAP_GET_HWINFO          _IO(RICHTAP_IOCTL_GROUP, 0x03)
#define RICHTAP_SET_FREQ            _IO(RICHTAP_IOCTL_GROUP, 0x04)
#define RICHTAP_SETTING_GAIN        _IO(RICHTAP_IOCTL_GROUP, 0x05)
#define RICHTAP_OFF_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x06)
#define RICHTAP_TIMEOUT_MODE        _IO(RICHTAP_IOCTL_GROUP, 0x07)
#define RICHTAP_RAM_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x08)
#define RICHTAP_RTP_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x09)
#define RICHTAP_STREAM_MODE         _IO(RICHTAP_IOCTL_GROUP, 0x0A)
#define RICHTAP_UPDATE_RAM          _IO(RICHTAP_IOCTL_GROUP, 0x10)
#define RICHTAP_GET_F0              _IO(RICHTAP_IOCTL_GROUP, 0x11)
#define RICHTAP_STOP_MODE           _IO(RICHTAP_IOCTL_GROUP, 0x12)
#define RICHTAP_F0_UPDATE           _IO(RICHTAP_IOCTL_GROUP, 0x13)

/* only used for aac richtap end */


/* support bit mask*/
#define HAPTIC_MASK_BIT_SUPPORT_EFFECT    0x00
#define HAPTIC_MASK_BIT_SUPPORT_GAIN      0x01
#define HAPTIC_MASK_BIT_TRIGGER_INTENSITY 0x07
#define HAPTIC_MASK_BIT_AAC_RICHTAP       0x06

#define HAPTIC_CNT                        0x08

#endif

