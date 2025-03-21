#ifndef __HAPTIC_CORE_H__
#define __HAPTIC_CORE_H__

/* ============================================================================ */
/* ============= below only kernel defination ======================== */
/* ============================================================================ */

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include "vivo_haptic_core_user.h"

enum effect_channel {
	UPLOAD = 0,
	AAC_RTP_MODE,
	AAC_STREAM_MODE,
};


struct reg_debug {
	u16 start_addr;
	int count;
	bool set_flag;
};

/**
 * struct user_status - represents current calling status for userspace
 * @user_id: allocted for current calling
 * @current_owner: current calling open the dev node
 */
struct user_status {
	int user_id;
	struct file *current_owner;
	enum effect_channel channel_flag; // only used for aac richtap

};


/**
 * struct haptic_handle - represents a Operating collection for IC
 * @name: name of miscdev handle
 * @dev: driver model view of IC driver device(eg:form IC probe)
 * @upload: complete the preparation work of vibration playback，(Must be implemented)
 * @playback: start or stop playing，(Must be implemented)
 * @erase: the work of cleaning up (maybe not use)
 * @set_gain: set the gain of vibration
 * @set_trigger_intensity: set the gain of trigger vibration
 * @judge_effect_support: Determine whether the current effect is supported in DTS
 * @get_f0:get motor actual f0
 * @get_motor_type: get motor type information
 * @set_cali_params: set calibration data to ic driver
 * @get_driver_ic: get driver ic type information
 * @init_dev: when the device is opened, initialize the device
 * @hap_bit: bitmap of functions that ic has
 * @chip: point to IC information structure
 * @private: reserve
 */

struct haptic_handle {
	const char     *name;
	struct device  *dev;

	int (*upload)(struct haptic_handle *hh, struct haptic_effect *effect);
	int (*playback)(struct haptic_handle *hh, int value);
	int (*erase)(struct haptic_handle *hh);
	void (*set_gain)(struct haptic_handle *hh, u16 gain);
	void (*set_trigger_intensity)(struct haptic_handle *hh, int gain);
	int (*judge_effect_support)(struct haptic_handle *hp, int16_t effectNo);
	void (*get_motor_type)(struct haptic_handle *hh, int *motorType);
	void (*get_f0)(struct haptic_handle *hh, int *f0);
	void (*set_cali_params)(struct haptic_handle *hh, struct haptic_cali_param *cali_params);
	void (*get_driver_ic)(struct haptic_handle *hh, enum ic_type *driver_ic);

	void (*init_dev)(struct haptic_handle *hh);
	void (*destroy)(struct haptic_handle *hh);

	// AT and Debug
	int (*f0_calibration)(struct haptic_handle *hh, char *buf);
	int (*f0_test)(struct haptic_handle *hh, char *buf);
	int (*impedance_test)(struct haptic_handle *hh, char *buf);

	int (*dump_reg)(struct haptic_handle *hh, char *buf, u16 start_addr, int count);
	int (*set_reg)(struct haptic_handle *hh, u16 addr, u8 val);

	// extend for AAC RICHTAP begin
	void (*get_ic_identity_NO)(struct haptic_handle *hh, uint8_t *identity_NO);
	void (*get_motor_f0)(struct haptic_handle *hh, int *f0);
	void (*set_gain_reg_direct)(struct haptic_handle *hh, int gain_reg_val);
	int (*play_rtp_less_than_FIFO)(struct haptic_handle *hh, void __user *p);
	int (*play_rtp_from_cycle_buffer)(struct haptic_handle *hh);
	int (*rtp_stop)(struct haptic_handle *hh);
	// extend for AAC RICHTAP end

	unsigned long  hap_bit[BITS_TO_LONGS(HAPTIC_CNT)];
	void           *chip;
	void           *private;
	struct haptic_rtp_container *haptic_container;
	struct mmap_buf_format      *start_buf; // only for AAC RICHTAP

};


/**
 * struct haptic_misc - represents a haptic device
 * @misc: miscdev structure
 * @handle: Operator functions that drive the IC,
 *          and relevant information
 * @lock: mutex lock for fops
 * @devt: device number of the miscdev
 * @open: times of opening the miscdev
 * @list: used to mount it to haptic_list
 * @user_status: record the current state of the call
 * @usr_bit: bitmap that records ID assigned to the caller
 * @private_data: reserve
 */

#define DYNAMIC_USER_IDS (0xFF + 1)
struct haptic_misc {
	struct miscdevice     misc;
	const char            *name;
	struct haptic_handle  *handle;
	struct mutex          lock;
	dev_t                 devt;
	int                   open;
	struct list_head      list;
	struct user_status    user_status;
	//unsigned long         usr_bit[BITS_TO_LONGS(DYNAMIC_USER_IDS)];
	struct file *effect_owners[DYNAMIC_USER_IDS];
	void                  *private_data;
	struct reg_debug      reg_debug;
	char                  *m_buf;
};


extern int haptic_miscdev_register(struct haptic_misc *hm);
extern int haptic_miscdev_unregister(struct haptic_misc *hm);
extern int haptic_handle_create(struct haptic_misc *hap_misc_dev, const char *name);
extern void haptic_device_set_capcity(struct haptic_handle *hh, int bit);
extern void haptic_handle_destroy(struct haptic_handle *hh);




/* for vivo VCODE begin */

#define VIVO_HAPTIC_VCODE

// must less than or equal to ENTRY_LEN_MAX in fs/proc/vivo_fmea.c
#define ENTRY_LEN_MAX 384

enum haptic_vcode_type {
	HBOOST_TIMEOUT = 0,
	BOOT_IMPEDANCE_CHECK_ERROR,
	AUTO_STANDBY_ERROR,
	SCENE_NOT_EXIST,
	EFFECT_NOT_EXIST,
	BIN_FILE_LOAD_ERROR,
	RTP_MEM_ALLOC_ERROR,
	CONSTANT_MODE_VOL_ERROR,
	HAPTIC_VCODE_ALL,
};

enum haptic_vcode_report_strategy { // 10 minutes upload
	EVERY_TIMES_COUNT = -1,
	ONLY_ONCE_FIRST_OCCUR = 1,
	ONLY_THREE_TIMES_OCCUR = 3,
};

enum log_collect_type {
	KMSG_SAVE_EVERYTIME = 0,
	KMSG_SAVE_ONCE, // only catch log first upload
	SELF_DEFINE,
};

struct vcode_infomation_statistic {
	ktime_t upload_time_line;
	struct delayed_work vcode_work;
	DECLARE_BITMAP(first_upload_bitmask, HAPTIC_VCODE_ALL);
	int times_statistic[HAPTIC_VCODE_ALL];
	enum haptic_vcode_report_strategy count_strategy[HAPTIC_VCODE_ALL];
	enum log_collect_type log_strategy[HAPTIC_VCODE_ALL];
	char exinfo[ENTRY_LEN_MAX];
	bool exception_occur;
	bool delay_work_on;
};

#ifdef VIVO_HAPTIC_VCODE

extern int vcode_haptic_statistic(enum haptic_vcode_type exception_type, const char *ex_info);

#define VCODE_HAPTIC_UPLOAD_EXCEPTION(exception_type, ex_info) \
	vcode_haptic_statistic(exception_type, ex_info)

#else

#define VCODE_HAPTIC_UPLOAD_EXCEPTION(exception_type, ex_info)

#endif // end of VIVO_HAPTIC_VCODE

// vivo haptic core device struct

struct haptic_core_dev {
	int vibrator_count;
	struct mutex          lock;
	#ifdef VIVO_HAPTIC_VCODE
	struct vcode_infomation_statistic vcode;
	#endif
};

/* for vivo VCODE end */

#endif

