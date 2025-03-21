/*********************************************************
 *
 * aw8622x.h
 *
 ********************************************************/
#ifndef _AW8622X_H_
#define _AW8622X_H_

#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/leds.h>
#include "../lra-wakeup.h"
#include "../vivo_haptic_core.h"



/*********************************************************
 *
 * Marco
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 1)
#define AW_KERNEL_VER_OVER_4_19
#endif



#define AW8622X_I2C_RETRIES			(5)
#define AW8622X_RTP_NAME_MAX		(64)
#define AW8622X_SEQUENCER_SIZE		(8)
#define AW8622X_SEQUENCER_LOOP_SIZE	(4)
#define AW8622X_OSC_CALI_MAX_LENGTH	(11000000)
#define AW8622X_PM_QOS_VALUE_VB		(400)
#define AW8622X_VBAT_REFER			(4200)
#define AW8622X_VBAT_MIN			(3000)
#define AW8622X_VBAT_MAX			(5500)
#define AW8622X_TRIG_NUM			(3)
#define AW8622X_I2C_RETRY_DELAY		(2)


#define AW8622x_LRA_0619                    619
#define AW8622x_LRA_0832                    832
#define AW8622x_LRA_1040                    1040
#define AW8622x_LRA_0815                    815


#define HAPTIC_BATTERY_VOLTAGE              4000

enum aw8622x_flags {
	AW8622X_FLAG_NONR = 0,
	AW8622X_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8622x_haptic_work_mode {
	AW8622X_HAPTIC_STANDBY_MODE = 0,
	AW8622X_HAPTIC_RAM_MODE = 1,
	AW8622X_HAPTIC_RTP_MODE = 2,
	AW8622X_HAPTIC_TRIG_MODE = 3,
	AW8622X_HAPTIC_CONT_MODE = 4,
	AW8622X_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8622x_haptic_activate_mode {
	AW8622X_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW8622X_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw8622x_haptic_cont_vbat_comp_mode {
	AW8622X_HAPTIC_CONT_VBAT_SW_ADJUST_MODE = 0,
	AW8622X_HAPTIC_CONT_VBAT_HW_ADJUST_MODE = 1,
};

enum aw8622x_haptic_ram_vbat_compensate_mode {
	AW8622X_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8622X_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};


enum aw8622x_sram_size_flag {
	AW8622X_HAPTIC_SRAM_1K = 0,
	AW8622X_HAPTIC_SRAM_2K = 1,
	AW8622X_HAPTIC_SRAM_3K = 2,
};

enum aw8622x_haptic_pwm_mode {
	AW8622X_PWM_48K = 0,
	AW8622X_PWM_24K = 1,
	AW8622X_PWM_12K = 2,
};

enum aw8622x_haptic_play {
	AW8622X_HAPTIC_PLAY_NULL = 0,
	AW8622X_HAPTIC_PLAY_ENABLE = 1,
	AW8622X_HAPTIC_PLAY_STOP = 2,
	AW8622X_HAPTIC_PLAY_GAIN = 8,
};

enum aw8622x_haptic_cmd {
	AW8622X_HAPTIC_CMD_NULL = 0,
	AW8622X_HAPTIC_CMD_ENABLE = 1,
	AW8622X_HAPTIC_CMD_HAPTIC = 0x0f,
	AW8622X_HAPTIC_CMD_TP = 0x10,
	AW8622X_HAPTIC_CMD_SYS = 0xf0,
	AW8622X_HAPTIC_CMD_STOP = 255,
};

enum aw8622x_haptic_cali_lra {
	WRITE_ZERO = 0,
	F0_CALI = 1,
	OSC_CALI = 2,
};

enum aw8622x_haptic_rtp_mode {
	AW8622X_RTP_SHORT = 4,
	AW8622X_RTP_LONG = 5,
	AW8622X_RTP_SEGMENT = 6,
};

enum aw8622x_ef_id {
	AW86223_EF_ID = 0x01,
	AW86224_5_EF_ID = 0x00,
};

enum play_type {
	RAM_TYPE = 0,
	RTP_TYPE = 1,
	TIME_TYPE = 2,
	CONT_TYPE = 3,
	RTP_MMAP_TYPE = 4,
};

enum haptics_custom_effect_param {
	CUSTOM_DATA_EFFECT_IDX,
	CUSTOM_DATA_TIMEOUT_SEC_IDX,
	CUSTOM_DATA_TIMEOUT_MSEC_IDX,
	CUSTOM_DATA_LEN,
};
enum aw8622x_haptic_f0_flag {
	AW8622x_HAPTIC_LRA_F0 = 0,
	AW8622x_HAPTIC_CALI_F0 = 1,
};

/*********************************************************
 *
 * Struct Define
 *
 ********************************************************/


/* trig_config
 * trig default high level
 * ___________           ___________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |__________
 *        first edge
 *                   second edge
 ******************** vib_trig_config *********************
 *     level polar pos_en pos_seq neg_en neg_seq brk bst
 trig1*  1     0     1       1       1      2     0   0
 trig2*  1     0     0       1       0      2     0   0
 trig3*  1     0     0       1       0      2     0   0
*/
struct trig {
	unsigned char trig_level;
	unsigned char trig_polar;
	unsigned char pos_enable;
	unsigned char pos_sequence;
	unsigned char neg_enable;
	unsigned char neg_sequence;
	unsigned char trig_brk;
};

struct aw8622x_dts_info {
	unsigned int mode;
	unsigned int f0_ref;
	unsigned int f0_cali_percent;
	unsigned int cont_drv1_lvl_dt;
	unsigned int cont_drv2_lvl_dt;
	unsigned int cont_drv1_time_dt;
	unsigned int cont_drv2_time_dt;
	unsigned int cont_wait_num_dt;
	unsigned int cont_brk_time_dt;
	unsigned int cont_track_margin;
	unsigned int cont_tset;
	unsigned int cont_drv_width;
	unsigned int cont_bemf_set;
	unsigned int cont_brk_gain;
	unsigned int d2s_gain;
	unsigned int prctmode[3];
	unsigned int sine_array[4];
	unsigned int trig_config[24];
	bool is_enabled_powerup_f0_cali;
	bool is_enabled_auto_bst;
};

//数据类型用u32，与dts的数据类型保持一致
struct haptic_wavefrom_info {
	u32 idx;
	u32 ram_id;
	u32 vmax;
	u32 times_ms;
	bool rtp_enable;
	const char *rtp_file_name;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct scene_effect_info {
	u16		scene_id;
	u16		effect_id;
	u16		real_vmax;
};

struct aw8622x_play_info {
	enum play_type type;
	u32 ram_id;//用于ram模式
	u32 vmax;
	u32 times_ms;//用于返回ram和rtp模式波形时长
	u32 playLength;//用于时长播放
	char rtp_file[128];//用于rtp模式
};


struct aw8622x {
	struct i2c_client *i2c;
	struct device *dev;

	struct hrtimer timer;
	struct work_struct long_vibrate_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	struct aw8622x_dts_info dts_info;
	struct ram ram;

	struct mutex lock;
	struct mutex rtp_lock;
	struct mutex rtp_check_lock;
	struct mutex bus_lock;

	unsigned char rtp_init;
	unsigned char ram_init;

	unsigned char f0_cali_flag;
	unsigned char ram_vbat_compensate;

	unsigned char chipid;
	unsigned char play_mode;
	unsigned char activate_mode;

	int state;
	int duration;
	int amplitude;
	int gain;


	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned int f0;
	unsigned int cont_f0;
	unsigned int cont_drv1_lvl;
	unsigned int cont_drv2_lvl;
	unsigned int cont_brk_time;
	unsigned int cont_wait_num;
	unsigned int cont_drv1_time;
	unsigned int cont_drv2_time;

	unsigned int vbat;
	unsigned int lra;
	unsigned int ram_update_flag;
	unsigned int rtp_update_flag;

	// common dts info begin
	int irq_gpio;
	int reset_gpio;
	u32 resistance_min;
	u32 resistance_max;
	u32 freq_min;
	u32 freq_max;
	int default_vmax;
	bool no_trigger;
	bool add_suffix;
	bool is_major;
	u32 lra_information;  //板子马达型号

	struct haptic_wavefrom_info *effect_list;
	int effects_count;
	struct scene_effect_info *base_scene_list;
	int base_scene_count;
	struct scene_effect_info *ext_scene_list;
	int ext_scene_count;
	// common dts info end

	struct haptic_misc *hm;
	struct led_classdev cdev;
	struct aw8622x_play_info play;

	volatile unsigned int osc_cali_data; //标准的osc时钟频率偏移，osc校准获取
	volatile unsigned int f0_cali_data; //校准f0时获取的时钟频率偏移
	int cali_f0;
	bool rtp_mmap_page_alloc_flag; // true : allco sucess, false: alloc failed

	struct wakeup_source *wklock;
};

struct aw8622x_container {
	int len;
	unsigned char data[];
};
#endif
