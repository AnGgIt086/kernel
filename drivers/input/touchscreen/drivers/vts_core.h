#ifndef __VTS_CORE_H__
#define __VTS_CORE_H__

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/sensors.h>
#include <linux/pm_wakeup.h>
#include "vts_op.h"
#include "vts_incell.h"
#include "vts_spi_drv.h"
#include <linux/vts_state.h>
#include <linux/moduleparam.h>
#include <asm/div64.h>
#include <linux/miscdevice.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <uapi/linux/sched/types.h>
#endif

#define VTS_CODE_VERSION 0x02180710

enum vts_type {
	VTS_TYPE_MAIN,
	VTS_TYPE_SECOND
};

#define	VIVO_TS_MAX_TOUCH_NUM 10

#define	FUN_NOT_REG	500
#define VTS_SENSOR_TEST_CALIBRATION	165
#define VTS_SENSOR_TEST_CHECK		166
#define VTS_PALM_DELAY	300

/* ic number */
/* st */
#define VTS_IC_FTS2		11
#define VTS_IC_FTS3		12
#define VTS_IC_FTM4		13
#define VTS_IC_FTM5		14
#define VTS_IC_ST80Y	15
/* syna */
#define VTS_IC_S3202	21
#define VTS_IC_S3203	22
#define VTS_IC_S3508	23
#define VTS_IC_S3501	24
#define VTS_IC_S3528	25
#define VTS_IC_S3310	26
#define VTS_IC_S3320	124
#define VTS_IC_S3718	28
#define VTS_IC_S3502	29
#define VTS_IC_S3606	61
#define VTS_IC_S1302	62
#define VTS_IC_S3706	63
#define VTS_IC_TD4322	123
#define VTS_IC_TD3320	124
#define VTS_IC_TD4330	125
#define VTS_IC_S3908	64
/* ft */
#define VTS_IC_FT5316	31
#define VTS_IC_FT5336	32
#define VTS_IC_FT5446	33
#define VTS_IC_FT8006	34
#define	VTS_IC_FT8736	35
#define	VTS_IC_FT8719	36
#define	VTS_IC_FT8756	37
#define VTS_IC_FT3519T_I2C  131
#define VTS_IC_FT3519T_SPI  132

/* goodix */
#define VTS_IC_GT970	41
#define VTS_IC_GT9159	42
#define VTS_IC_GT1151	43
#define VTS_IC_GT9286	44
#define VTS_IC_GT5688	45
#define VTS_IC_GT9886	46
#define VTS_IC_GT9885TS 47
#define VTS_IC_GT9897		48	//0x5d spi
#define VTS_IC_GT9897_I2C	49	//0x14 i2c
#define VTS_IC_GT9916		50	//0x5d i2c
#define VTS_IC_GT9916_SPI	54	//0x5d spi
#define VTS_IC_GT9895		55	//0x5d i2c
#define VTS_IC_GT9916_THP	56  //0x5d spi thp
#define VTS_IC_GT9966_SPI	57  //0X5d spi

/* cypress */
#define VTS_IC_CYTMA568	51
#define VTS_IC_CYTMA545	52
#define VTS_IC_CP3155	53
/* nt */
#define VTS_IC_NT36672	81
#define VTS_IC_NT36525	82
#define VTS_IC_NT36670	83
#define VTS_IC_NT36675  84
/* samsung */
#define VTS_IC_SEC_Y661 70
#define VTS_IC_SEC_TDDI 71
#define VTS_IC_SEC_Y761 72
/*ilitek*/
#define VTS_IC_ILI_9881H 90
#define VTS_IC_ILI_9882N 91
#define VTS_IC_ILI_9883C 92

/* himax */
#define VTS_IC_HIMAX83113	100
#define VTS_IC_HIMAX83112   101

/*chipone*/
#define VTS_IC_CHIPONE_NL9911C 110
#define VTS_IC_CHIPONE_NL9916 111
/*ovt*/
#define VTS_IC_OVT_TD4375	120


/* module vendor code:MVC*/
#define VTS_MVC_LG	0xB0
#define VTS_MVC_BIE	0x3B
#define VTS_MVC_TRY	0x70
#define VTS_MVC_ECH	0x80
#define VTS_MVC_LNS	0x10
#define VTS_MVC_BYD	0x59
#define VTS_MVC_SAM	0x90
#define VTS_MVC_JDI	0xC0
#define VTS_MVC_BOE	0xD0
#define VTS_MVC_SAP	0xE9
#define VTS_MVC_TMA	0xF0
#define VTS_MVC_AUO	0x30

#ifndef KEY_TS_SWIPE
#define KEY_TS_SWIPE	746
#endif

#define KEY_VIRTUAL_WAKEUP		0x2f2
#define KEY_LONGPRESS_DOWN		0x2f3
#define KEY_LONGPRESS_UP		0x2f4

#define KEY_INSIDE_SLIDE_LEFT   0x2f5
#define KEY_INSIDE_SLIDE_RIGHT  0x2f6
#define KEY_QUIT_ACTIVE_MODE    0x2f7

#define VTS_KEY_FINGER_GESTURE 254
#define VTS_KEY_FACE_GESTURE 605
#define VTS_KEY_V_GESTURE 606
#define VTS_KEY_HEART_GESTURE 607

#define KEY_SCREEN_CLOCK_WAKE_UP    0x279 //633

#define VTS_GESTURE_ARRAY_LEN 40
#define VTS_GESTURE_POINT 0
#define VTS_GESTURE_O_DIR 1

#define VTS_EDGE_AREA_ABS_TYPE BIT(0)
#define VTS_EDGE_AREA_LONG_PRESS_TYPE BIT(1)
#define VTS_EDGE_AREA_CHARACTER_TYPE BIT(2)
#define VTS_ACTIVE_AREA_GESTURE_TYPE BIT(3)

// 数据帧异常，例如汇顶框信息checksum error
#define FRAME_CHECKSUM_ERROR BIT(0)

enum vts_ic_param_size {
	PARAM_SIZE_INVALID = 0,
	PARAM_SIZE_GOODIX = 20,
	PARAM_SIZE_SYNA = 16,
	PARAM_SIZE_ST = 8,
	PARAM_SIZE_MAX = 50
};

enum vts_event {
       VTS_EVENT_GESTURE_FINGERPRINT_DETECT = 0,
       VTS_EVENT_GESTURE_FACE_DETECT,
       VTS_EVENT_GESTURE_LARGE_AREA_PRESS,
       VTS_EVENT_GESTURE_DOUBLE_CLICK,
       VTS_EVENT_GESTURE_TAKE_CAMERA,
       VTS_EVENT_GESTURE_PATTERN_C,
       VTS_EVENT_GESTURE_PATTERN_E,
       VTS_EVENT_GESTURE_PATTERN_M,
       VTS_EVENT_GESTURE_PATTERN_W,
       VTS_EVENT_GESTURE_PATTERN_A,
       VTS_EVENT_GESTURE_PATTERN_F,
       VTS_EVENT_GESTURE_PATTERN_O,
       VTS_EVENT_GESTURE_PATTERN_V,
       VTS_EVENT_GESTURE_PATTERN_HEART,
       VTS_EVENT_GESTURE_PATTERN_LEFT,
       VTS_EVENT_GESTURE_PATTERN_RIGHT,
       VTS_EVENT_GESTURE_PATTERN_UP,
       VTS_EVENT_GESTURE_PATTERN_DOWN,
       VTS_EVENT_GESTURE_PATTERN_SWAP,
       VTS_EVENT_GESTURE_VK_LONG_PRESS,
       VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE,
       VTS_EVENT_GESTURE_VK_DC,
       VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK,
       VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT,
       VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT,
       VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE,
       VTS_EVENT_GESTURE_PATTERN_H,
       VTS_EVENT_GESTURE_FINGER3_MODE,
	   VTS_EVENT_GESTURE_SINGLE_CLICK,
  	   VTS_EVENT_GESTURE_PEN_SINGLE_CLICK
};


enum vts_frame_type {
	VTS_FRAME_MUTUAL_RAW,
	VTS_FRAME_MUTUAL_DELTA,
	VTS_FRAME_SELF_RAW,
	VTS_FRAME_SELF_DELTA,
	VTS_FRAME_AMBIENT_BASELINE
};

enum vts_test_apk_type {
	VTS_TEST_APK_TYPE_SENSOR_TEST,
	VTS_TEST_APK_TYPE_LCM_NOISE_TEST,
	VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST,
	VTS_TEST_APK_TYPE_RAWDATA_TEST,
	VTS_TEST_APK_TYPE_RF_TEST,
	VTS_TEST_APK_TYPE_MAX
};

enum vts_property {
	VTS_PROPERTY_PANEL_TYPE,
	VTS_PROPERTY_SENSOR_TX_NUM,
	VTS_PROPERTY_SENSOR_RX_NUM,
	VTS_PROPERTY_DIMENTION_X,
	VTS_PROPERTY_DIMENTION_Y,
	VTS_PROPERTY_DISPLAY_X,
	VTS_PROPERTY_DISPLAY_Y,
	VTS_PROPERTY_RESOLUTION_ADJUST,
	VTS_PROPERTY_IC_NUMBER,
	VTS_PROPERTY_TDDI,
	VTS_PROPERTY_VENDOR,
	VTS_PROPERTY_POLICY,
	VTS_PROPERTY_NO_FLASH,
	VTS_PROPERTY_LONG_PRESS,
	VTS_PROPERTY_VIRTUAL_PROXIMINITY,
	VTS_PROPERTY_NEED_CALI,
	VTS_PROPERTY_GAME_MODE,
	VTS_PROPERTY_BAND,
	VTS_PROPERTY_ROTATION,
	VTS_PROPERTY_CHARGE,
	VTS_PROPERTY_GESTURE,
	VTS_PROPERTY_VIRTUAL_KEY,
	VTS_PROPERTY_LANDSCAPE_GAMEMODE,
	VTS_PROPERTY_REPORT_TIMESTAMP,
	VTS_PROPERYT_VIRTUAL_GAMEKEY,
	VTS_PROPERTY_SCREEN_CLOCK,
	VTS_PROPERTY_BROKEN_DISABLE,
	VTS_PROPERTY_GAME_HIGH_RATE,
	VTS_PROPERTY_GAME_IDLE_TIME,
	VTS_PROPERTY_FINGER_MODE,
	VTS_PROPERTY_FD_FEED_BACK,
	VTS_PROPERTY_INPUT_METHOD,
	VTS_PROPERTY_TP_CHANNEL_COMP,
	VTS_PROPERTY_SET_BUS_STATE,
	VTS_PROPERTY_I2C_EVENT,
	VTS_PROPERTY_BOOST,
	VTS_PROPERTY_FINGER_CENTER,
	VTS_PROPERTY_FINGER_CENTER_X,
	VTS_PROPERTY_FINGER_CENTER_Y,
	VTS_PROPERTY_CALIBRATION_TWICE,
	VTS_PROPERTY_SCREEN_CLOCK_REPORT_ABS,
	VTS_PROPERTY_VIRTUAL_LCMID,
	VTS_PROPERTY_SHARE_LCMID,
	VTS_PROPERTY_DEF_LCMID,
	VTS_PROPERTY_SLEEP_TIME,
	VTS_PROPERTY_RESET_DISCONECT,
	VTS_PROPERTY_DEBUG_CUST, //支持debug开关打印rawdata
	VTS_PROPERTY_SINGLETAP_SUPPORT,
	VTS_PROPERTY_MAX
};

enum point_down_or_up {
	POINT_UP = 0,
	POINT_DOWN,
	POINT_RELEASE,
};

struct vts_edge_cmd {
	int x;
	int y;
	int width;
	int height;
	int area_type;
	bool enable;
	int index;
};
struct vts_screen_clock_point{
     struct mutex  scrclMutex;
	 unsigned int  pointX;
	 unsigned int  pointY;
	 unsigned int  realX;
	 unsigned int  realY;
};

struct vts_screen_clock_cmd {
	int x;
	int y;
	int width;
	int height;
};

struct vts_screen_clock_zone{
	 struct vts_screen_clock_point startPoint;
	 int width;
	 int height;
	 int enable;
	 struct mutex c_Mutex;
	};

struct vts_frame {
	enum vts_frame_type type;
	short *data;
	int size;
	struct list_head node;
};

struct vts_device;
struct vts_incell;

struct vts_incell_ops {
	int (*early_suspend)(struct vts_incell *incell, int display_id);
	int (*suspend)(struct vts_incell *incell, int display_id);
	int (*early_resume)(struct vts_incell *incell, int display_id);
	int (*resume)(struct vts_incell *incell, int display_id);
};

enum vts_report_flags_e {
	FLAGS_REPORT_TIMESTAMP = BIT(0),
};

struct vts_point {
	int x, y, wx,wy;
	int touch_id;
	int nr_touches;
	bool large_press;
	u8 state;
	int physical_x, physical_y;
};

struct vts_report {
	unsigned long slotbit;
	unsigned long touchbit;
	struct mutex lock;
	char km_name[32];
	struct kmem_cache *km;
	struct list_head down_points;
	struct list_head up_points;
	struct list_head keys;
	int nr_coordinates;
	unsigned long flags;
	u16 coordinates_x[32];
	u16 coordinates_y[32];
};

struct vts_virtual_proxminity {
	struct sensors_classdev cdev;
	struct input_dev *idev;
	struct delayed_work dwork ;
	int values[3];
	int nr_remain_report_times;
	spinlock_t lock;
	bool inited;
};

enum vts_fw_type {
	VTS_FW_TYPE_FW,
	VTS_FW_TYPE_IHEX,
	VTS_FW_TYPE_CONFIG,
	VTS_FW_TYPE_LIMIT,
	VTS_FW_TYPE_MP,
	VTS_FW_TYPE_COUNT
};

enum vts_zone {
	VTS_ROM_ZONE_IMEI,
	VTS_ROM_ZONE_LCM
};

struct vts_firmware {
	char path[128];
	struct firmware *firmware;
};

struct vts_grip_area {
	int area_edge;
	int area_center;
};

enum vts_sensor_test_result {
	VTS_SENSOR_TEST_SUCCESS = 0,
	VTS_SENSOR_TEST_DELTA_FAILED = BIT(0),
	VTS_SENSOR_TEST_ITO_FAILED = BIT(1),
	VTS_SENSOR_TEST_DATA_FAILED = BIT(2),
	VTS_SENSOR_TEST_INIT_FAILED = BIT(3),
	VTS_SENSOR_TEST_SHORT_FAILED = BIT(4),
	VTS_SENSOR_TEST_OPEN_FAILED = BIT(5),
	VTS_SENSOR_TEST_NOISE_FAILED = BIT(6),
	VTS_SENSOR_TEST_DOZECC_FAILED = BIT(7),
	VTS_SENSOR_TEST_DIGITAL_FAILED = BIT(8),
	VTS_SENSOR_TEST_SLP_FAILED = BIT(9),
};

enum vts_sensor_cali_result {
	VTS_SENSOR_CALIBERATE_SUCCESS = 0,
	VTS_SENSOR_CALIBERATE_FAILED = BIT(0),
};

// 充电器状态类型
enum vts_charge_state {
	VTS_CHARGE_NULL_STATE = 0, 
	VTS_CHARGE_USB_STATE = 1,
	VTS_CHARGE_WIRLESS_STATE = 4,
};

enum vts_report_rate_proc{
	RATE_NORMAL = 0,
	RATE_HIGH
};
enum vts_idle_times_proc{
	IDLE_TIME_1S = 1,
	IDLE_TIME_2S, 
	IDLE_TIME_10S
};

enum vts_game_proc_types{
	PROC_OUT_GAME = 0,
	PROC_IN_GAME
};

struct vts_operations {
	int (*init)(struct vts_device *vtsdev);
	int (*exit)(struct vts_device *vtsdev);
	int (*get_frame)(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size);
	int (*get_fw_version)(struct vts_device *vtsdev, u64 *version);
	int (*get_ic_mode)(struct vts_device *vtsdev);
	int (*set_rotation)(struct vts_device *vtsdev, int on);;
	int (*set_charging)(struct vts_device *vtsdev, int state);
	int (*set_auto_idle)(struct vts_device *vtsdev, int state);
	int (*set_bandstate)(struct vts_device *vtsdev, int state);
	int (*process_package)(struct vts_device *vtsdev, unsigned char *package_name);
	int (*early_suspend)(struct vts_device *vtsdev);
	int (*early_resume)(struct vts_device *vtsdev);
	int (*set_virtual_prox)(struct vts_device *vtsdev, int enable);
	int (*set_long_press)(struct vts_device *vtsdev, int enable);
	int (*set_finger_mode)(struct vts_device *vtsdev, int mode);
	int (*set_gesture)(struct vts_device *vtsdev, int enable);
	int (*otherInfo)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	int (*get_grip_status)(struct vts_device *vtsdev, struct vts_grip_area *grip_area);
	int (*change_mode)(struct vts_device *vtsdev, int which);
	int (*update_firmware)(struct vts_device *vtsdev, const struct firmware *firmware);
	int (*reset)(struct vts_device *vtsdev);
	int (*broken_disable)(struct vts_device *vtsdev, int state);
	int (*sensor_test)(struct vts_device *vtsdev, enum vts_sensor_test_result *result);
	int (*sensor_caliberate)(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result);
	int (*get_ic_status)(struct vts_device *vtsdev, int *status);
	int (*set_edge_reject_area)(struct vts_device *vtsdev,  struct vts_edge_cmd *cmd);
	int (*set_virtual_key)(struct vts_device *vtsdev, int state);
	int (*set_vk_longpress)(struct vts_device *vtsdev, int state);
	int (*set_vk_activemode)(struct vts_device *vtsdev, int state);
	int (*set_landscape_gamemode)(struct vts_device *vtsdev, int state);
	int (*set_virtual_gamekey)(struct vts_device *vtsdev, int state);
	int (*rom_size)(struct vts_device *vtsdev, u32 *size);
	ssize_t (*rom_read)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	ssize_t (*rom_write)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	int (*set_screen_clock_report_abs)(struct vts_device *vtsdev,int report_enable);
	int (*set_screen_clock_area)(struct vts_device *vtsdev ,int state);
	int (*update_gesture_bit)(struct vts_device *vtsdev,int state);
	int (*set_input_method)(struct vts_device *vtsdev,int state);
	int (*get_tp_channel_comp_data)(struct vts_device *vtsdev ,bool true_false);
	int (*set_bus_state)(struct vts_device *vtsdev,int state);
	int (*set_finger_center)(struct vts_device *vtsdev,int state);
	int (*get_calibration_status)(struct vts_device *vtsdev);
	int (*get_fw_resolution)(struct vts_device *vtsdev, int enable);
	int (*set_debug_print)(struct vts_device *vtsdev, int state);
};

#define VTS_MODULE_LCMID_NR_MAX 32
#define VTS_MODULE_CALI_MAX 5

/* <softid, moduleid> in tp_module */
struct vts_vendor_module_id {
	struct list_head entry;
	u32 module_id;
	u32 softid;
};

struct vts_board_version_id {
	struct list_head entry;
	u32 board_id_bit;
	u32 board_id_val;
	u32 module_id;  //ƥ��board version��module id, dtsi���õ�ֵ������lcmid�ظ�
};
struct vts_panel_module {
	struct list_head list;
	u32 properties[VTS_PROPERTY_MAX];
	u32 lcmid[VTS_MODULE_LCMID_NR_MAX];
	u32 softid_cmdline;
	int lcmid_cnt;
	int softid_cnt;
	u32 highest_calibration[VTS_MODULE_CALI_MAX];
	u32 other_calibration[VTS_MODULE_CALI_MAX];
	u32 board_version_id[VTS_MODULE_LCMID_NR_MAX];
	int highestCali_cnt;
	int otherCali_cnt;
	int board_version_id_cnt;
	#if defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	struct drm_panel *active_panel_v2;
	#endif
	bool vts_lcm_module_active;			/* dts ����vts-lcm-moduleʱʹ�� */
	struct list_head module_list;
	struct list_head vendor_module_list;		/* dts ����vts-softid-module_idʱ����<softid, module_id> */
	struct list_head board_version_id_list;     /* dts ����vts-board-version-idʱ����<board_bit, board_bit_val, ָ����lcmid> */
	int module_match_bit;    /* softid��Ҫƥ�伸λ, Ĭ��0xff, ������Ŀ��������0xFFFF/0xFFFFFF */
	const char *policy_name;
};

#ifdef VTS_TEMP_ENABLE
struct vts_fw_collection {
	int lcm_id;
	enum vts_fw_type fw_type;
	const unsigned char *fw_data;
	unsigned int fw_size;
};
#endif

struct vts_policy {
	enum vts_run_mode (*expected_mode)(struct vts_device *vtsdev);
	bool (*modechange_allow)(struct vts_device *vtsdev, enum vts_run_mode old, enum vts_run_mode new, enum vts_state reason);
	bool (*on_event_down)(struct vts_device *vtsdev, enum vts_event event, int *keycode);
	bool (*on_event_up)(struct vts_device *vtsdev, enum vts_event event, int *keycode);
	bool (*on_point_down)(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	bool (*on_point_up)(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	void (*on_state_changed)(struct vts_device *vtsdev, enum vts_state state, int val);
	void (*incell_proc)(struct vts_device *vtsdev, unsigned long event, int blank, int display_id);
	int  (*on_post_key)(struct vts_report *report, int keycode ,int value);
	const char *name;
};

enum vts_ic_exception {
	VTS_EXCEPTION_ESD_ERR,
	VTS_EXCEPTION_WATCHDOG_ERR,
	VTS_EXCEPTION_CHECKSUM_ERR,
	VTS_EXCEPTION_I2C_ERR,
	VTS_EXCEPTION_COUNT,
};

struct vts_long_press_report {
	/*long press report timer*/
	struct hrtimer long_press_timer;
	struct work_struct long_press_work;
	int slot;
	int major;
	int index;
	int work_status;
};

#define vts_boost_enable(ev, val1) do {} while(0)
static inline bool vts_is_debug(void)
{
	return false;
}

static inline void vts_proxminity_report(struct vts_device *vtsdev, int x, int y, int z) {}


typedef irqreturn_t (*vts_irq_handler_t)(int, void *, ktime_t);

/* @ input rejection function vts_rejection_data_handle, and use in vts_rejection_insert_point_get */
struct vts_coordinate_data {
	ktime_t ktime;
	int id;
	int action;// Press(仅第一帧)-move（第二帧和之后）-release-none
	int x;
	int y;
	int major; // 长边长度，而非固定的某边
	int minor; // 短边长度，而非固定的某边
	int node_num; // 框内大于阈值的节点数
	int max_value; // 框内最大差值
	int edge_ratio; // 最边缘差值累加和/次边缘差值累加和,放大了16倍
	int report_flag; // 是否满足报点：1-报点，0-不报
};

struct vts_device {
	struct device *dev;
	struct device_node *ic_node;
	struct list_head entry;
	struct list_head tp_list;
	struct input_dev *idev;
	const struct vts_operations *ops;
	void *private;
	struct list_head frames;

	char msg_km_name[32];
	struct kmem_cache *msg_km;
	struct list_head messages;
	spinlock_t msg_lock;
	struct semaphore msg_sem;

	struct task_struct *t;
	struct semaphore sem;

	int irq;
	void *irqdata;
	unsigned long flags;
	vts_irq_handler_t irq_thread_fn;

	struct wakeup_source *irq_wakelock;
	struct wakeup_source *k_wakelock;

	int (*exception_handler)(struct vts_device *vtsdev, enum vts_ic_exception e, int arg);

	struct vts_firmware firmwares[VTS_FW_TYPE_COUNT];
	atomic_t firmware_cache;
	struct vts_panel_module *module;
	u32 ic_number;
	enum vts_type type;
	struct vts_policy *policy;

	atomic_t run_mode;
	spinlock_t mslock;
	atomic_t mobile_state[VTS_STA_MAX];
	unsigned char busType;

	const char *activity_path[VTS_TEST_APK_TYPE_MAX];
	
	struct vts_incell incell;
	struct vts_report report;
	unsigned long large_touches;
	bool finger3_mode;
	struct vts_virtual_proxminity virtual_prox;
	struct vts_screen_clock_zone screen_clock_zone;
	struct vts_screen_clock_point  screen_clock_point;
	unsigned int has_screen_clock;
	atomic_t idle_state;
	struct vts_long_press_report long_press_report;
	int frame_error;	// ic checksum error等信息
	int reject_algorithm_support;

	u8 *rom_data;
	u32 rom_size;
	u32 exceptions[VTS_EXCEPTION_COUNT];
#if defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	struct drm_panel *active_panel_v2;
#endif
#ifdef VTS_TEMP_ENABLE
	struct delayed_work fw_update_work;
#endif
	char wakelock_name[32];

	int fw_x;
	int fw_y;
	bool find_board_id;
	unsigned long int cmdline_board_version;


	/*card slide region begin*/
	unsigned int y0;
	unsigned int y1;
	unsigned int width;
	/*card slide region end*/
	int rawdata_print_flag; // 0,表示raw或者diff 1， 表示baseline
};
#define VTS_SELF_OTHERDATA_MAX_LEN	64

// 与点相关的数据
// 禁止改变大小或者调整成员位置
struct vts_sample_point {
  	unsigned char available;
  	unsigned char id;
  	unsigned char action;		// 0:up, 1:down, 2:move, other:invalid
  	unsigned short x; 			// 报点坐标 x
  	unsigned short y;			// 报点坐标 y
  	unsigned short wx;		    // 长轴
  	unsigned short wy;		    // 短轴
  	unsigned short node_num;	// 接触面积大于阈值的节点个数
  	unsigned short diff_max;	// 接触面积的最大diff值
  
  	unsigned char drv_left;		//Tx左边界
  	unsigned char drv_right;	//Tx右边界
  	unsigned char sen_top;		//Rx上边界
  	unsigned char sen_buttom;	//Rx下边界
  	unsigned short pressure;	// 压力，diff_sum
  	unsigned short rect_ratio;      //边缘通道与次边缘通道比值
  	long long kt; //只有数据缓存时才赋值使用，其他未赋值
  	char reserve[16]; // 默认填充0xFF, 扩展使用时请判非法值
}__attribute((packed));

static inline void *vts_get_drvdata(struct vts_device *vtsdev)
{
	return vtsdev->private;
}

static inline void vts_set_drvdata(struct vts_device *vtsdev, void *data)
{
	vtsdev->private = data;
}

static inline const char *vts_name(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vivo_ts";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vivo_ts_second";
	else
		return "invalid type";
}

static inline const char *touscreen_name(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "touchscreen";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "touchscreen_second";
	else
		return "invalid type";
}


static inline const char *vts_name_fp(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vivo_ts_fp";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vivo_ts_second_fp";
	else
		return "invalid type";
}

static inline const char *vts_name_pen(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vivo_ts_pen";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vivo_ts_second_pen";
	else
		return "invalid type";
}

#define incell_to_vts(x) container_of(x, struct vts_device, incell)

#define incell_to_vts(x) container_of(x, struct vts_device, incell)

/* gesture switch bit mask, sync this bit with touchscreen service*/
#define VTS_GESTURE_C		(0x01 << 6)
#define VTS_GESTURE_E		(0x01 << 5)
#define VTS_GESTURE_UP		(0x01 << 1)
#define VTS_GESTURE_DOWN	(0x01 << 7)
#define VTS_GESTURE_M		(0x01 << 4)
#define VTS_GESTURE_LR		(0x01 << 0)
#define VTS_GESTURE_W		(0x01 << 3)
#define VTS_GESTURE_DCLICK	(0x01 << 8)
#define VTS_GESTURE_F		(0x01 << 10)
#define VTS_GESTURE_A		(0x01 << 9)
#define VTS_GESTURE_FG		(0x01 << 12)
#define VTS_GESTURE_O		(0x01 << 2)
#define VTS_GESTURE_SHORT_DOWN_SWIPE	(0x01 << 11)
#define VTS_GESTURE_FACE_GESTURE		(0x01 << 13)
#define VTS_GESTURE_V		(0x01 << 14)
#define VTS_GESTURE_HEART	(0x01 << 15)
#define VTS_GESTURE_COVER_MUTE			(0x01 << 16)

#define VTS_GESTURE_ENABLE(bitmap, gesture) 	((unsigned int)(bitmap | (unsigned int)gesture))
#define VTS_GESTURE_DISABLE(bitmap, gesture) 	((unsigned int)(bitmap & (~(unsigned int)gesture))

enum vts_run_mode {
	VTS_ST_NORMAL = 0,
	VTS_ST_SLEEP,
	VTS_ST_GESTURE,
	VTS_ST_UNKNOWN
};

#define vts_dev_log(vtsdev, force, flag, fmt, param...) do { \
		if(likely(force)) \
			printk(KERN_ERR "VIVO_TS_"flag"[%s]%s:%d:"fmt, vtsdev ? vts_name(vtsdev) : "common", __func__, __LINE__, ##param); \
	} while (0)
#define vts_dev_err(vtsdev, fmt, param...) do { \
		vts_dev_log(vtsdev, 1, "ERR", fmt, ##param); \
	} while(0)
#define vts_debug_code() if(vts_is_debug())

#define vts_dev_info(vtsdev, fmt, param...) vts_dev_log(vtsdev, 1, "INF", fmt, ##param)
#define vts_dev_dbg(vtsdev, fmt, param...) vts_dev_log(vtsdev, vts_is_debug(), "DBG", fmt, ##param)

#define VTI(fmt, param...) vts_dev_info(NULL, fmt, ##param)
#define VTE(fmt, param...) vts_dev_err(NULL, fmt, ##param)
#define VTD(fmt, param...) vts_dev_dbg(NULL, fmt, ##param)

#define VTS_GESTURE_ARRAY_LEN 40
#define VTS_GESTURE_POINT 0
#define VTS_GESTURE_O_DIR 1

extern struct vts_device *vts_device_alloc(void);
extern int vts_device_free(struct vts_device *vtsdev);
extern int vts_parse_dt_property(struct vts_device *vtsdev, struct device_node *np);
extern int vts_register_driver(struct vts_device *vtsdev);
extern int vts_unregister_driver(struct vts_device *vtsdev);
extern void vts_device_lock(struct vts_device *vtsdev);
extern int vts_device_lock_timeout(struct vts_device *vtsdev, long jiffies);
extern void vts_device_unlock(struct vts_device *vtsdev);
extern u8 *vts_fw_data_get(struct vts_device *vtsdev, enum vts_fw_type type, int *size);
extern void vts_fw_data_put(struct vts_device *vtsdev,  enum vts_fw_type type);
extern int vts_fw_path_set(struct vts_device *vtsdev, enum vts_fw_type type, const char *path, bool update);
extern int vts_fw_path_get(struct vts_device *vtsdev, enum vts_fw_type type, char *path, size_t size);
extern struct vts_frame *vts_frame_data_get(struct vts_device *vtsdev, enum vts_frame_type type);
extern int vts_frame_data_put(struct vts_device *vtsdev, struct vts_frame *frame);
extern int vts_get_lcmid(struct vts_device *vtsdev, u32 *lcmid);
extern int vts_get_lcmid_compatible(struct vts_device *vtsdev, u32 *lcmid, size_t *size);
int vts_get_matched_mod_id(struct vts_device *vtsdev, u32 *mod_id);
int vts_get_module_id_compatible(struct vts_device *vtsdev, u32 *module_id, size_t *size);
extern int vts_property_get(struct vts_device *vtsdev, enum vts_property prop, u32 *val);
extern int vts_state_set(struct vts_device *vtsdev, enum vts_state id, int val);
extern int vts_state_set_sync(struct vts_device *vtsdev, enum vts_state id, int val);
extern int vts_state_broadcast(struct vts_device *vtsdev, enum vts_state id, int val);
extern bool vts_state_all_equals(enum vts_state state, int val);
extern enum vts_run_mode vts_get_run_mode(struct vts_device *vtsdev);
extern int vts_state_get(struct vts_device *vtsdev, enum vts_state state);
extern void vts_state_dump(struct vts_device *vtsdev);
int vts_notify_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg);
extern bool vts_is_module_match(void);
extern bool vts_is_debug(void);
int vts_get_tp_count(void);
extern void vts_debug_enable(bool enable);
extern int vts_interrupt_register(struct vts_device *vtsdev, unsigned int irq, vts_irq_handler_t irq_thread_fn, unsigned long flags, void *data);
extern void vts_interrupt_unregister(struct vts_device *vtsdev);
extern int vts_incell_init(struct vts_device *vtsdev);
extern int vts_incell_deinit(struct vts_device *vtsdev);
extern int vts_report_coordinates_set(struct vts_device *vtsdev, u16 *x, u16 *y, int nr_points);
extern int vts_report_coordinates_get(struct vts_device *vtsdev, u16 *x, u16 *y, size_t size, int *nr_points);
extern int vts_report_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, bool large_press, u8 *data, size_t size, ktime_t kt);
extern int vts_report_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, int x, int y, int wx, int wy, bool large_press, ktime_t kt);
extern int vts_report_release(struct vts_device *vtsdev);
extern int vts_report_set_flags(struct vts_device *vtsdev, unsigned long flags);
extern int vts_report_inject_points(struct vts_device *vtsdev, char *filepath);
extern unsigned long vts_report_flags(struct vts_device *vtsdev);
extern int vts_report_event_down(struct vts_device *vtsdev, enum vts_event event);
extern int vts_report_event_up(struct vts_device *vtsdev, enum vts_event event);
extern int vts_report_point_sync(struct vts_device *vtsdev);
extern int vts_report_ic_status(struct vts_device *vtsdev, int status);
extern int vts_report_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg);
extern int vts_firmware_version_get(struct vts_device *vtsdev, u64 *version);
extern int vts_touch_ic_mode_get(struct vts_device *vtsdev);
extern int vts_get_calibration_status(struct vts_device *vtsdev);
extern int vts_classdev_register(struct device *parent, struct vts_device *vtsdev);
extern void vts_classdev_unregister(struct vts_device *vtsdev);
extern int vts_update_dclick_point(struct vts_device *vtsdev, int x, int y);
extern unsigned int vts_get_msdd_report_lcm_id(enum vts_type type);
extern struct wakeup_source *vts_wakelock_register(struct vts_device *vtsdev, const char *wakelock_name);
extern void vts_wakelock_unregister(struct wakeup_source *wakelock);
extern void vts_long_press_timer_init(struct vts_device *vtsdev);
extern void vts_long_press_timer_deinit(struct vts_device *vtsdev);


int vts_report_init(struct vts_device *vtsdev);
extern int vts_report_deinit(struct vts_device *vtsdev);
extern int vts_node_sysfs_add(struct vts_device *vtsdev);
extern void vts_node_sysfs_remove(struct vts_device *vtsdev);
extern ssize_t vts_rom_zone_read(struct vts_device *vtsdev, enum vts_zone zone, u8 *buf, size_t size);
extern ssize_t vts_rom_zone_write(struct vts_device *vtsdev, enum vts_zone zone, const u8 *buf, size_t size);
extern ssize_t vts_rom_zone_size(struct vts_device *vtsdev, enum vts_zone zone);
extern bool vts_is_load_driver(char *driver_name, const int *ic_numbers, size_t size);
extern int vts_reset(struct vts_device *vtsdev);
extern int vts_device_count_get(void);
extern int vts_get_screen_clock_zone(struct vts_screen_clock_cmd *dis, struct vts_screen_clock_zone *src);

void vts_kthread_stop(struct semaphore *sem, struct task_struct *task);
int vts_debug_interface(struct vts_device *vtsdev);

#define vts_call_func_sync(vts, return_type, func, arg...) ({ \
		return_type ret; \
		vts_device_lock(vts); \
		ret = func(vts, ##arg); \
		vts_device_unlock(vts); \
		ret; \
	})

#define __vts_call_ic_ops(vts, func, sync, arg...) ({ \
		int ret = 0; \
		s64 ktime_start; \
		s64 ktime_end; \
		u64 ktime_diff; \
		u64 ktime_mod; \
		\
		if (vts && vts->ops && vts->ops->func) { \
			if (sync) \
				vts_device_lock(vts); \
			vts_dev_info(vts, "call ic ops "#func" start\n"); \
			ktime_start = ktime_to_us(ktime_get()); \
			ret = vts->ops->func(vts, ##arg); \
			ktime_end = ktime_to_us(ktime_get()); \
			ktime_diff = (u64)(ktime_end - ktime_start); \
			ktime_mod = do_div(ktime_diff, 1000); \
			vts_dev_info(vts, "call ic ops "#func" end, took %lld.%lldms, ret = %d\n", ktime_diff, ktime_mod, ret); \
			if (sync) \
				vts_device_unlock(vts); \
		} else { \
			vts_dev_info(vts, "ic didn't implement "#func", no need call\n"); \
		} \
		\
		ret; \
	})

#define vts_call_ic_ops(vts, func, arg...) __vts_call_ic_ops(vts, func, false, ##arg)
#define vts_call_ic_ops_sync(vts, func, arg...) __vts_call_ic_ops(vts, func, true, ##arg)

#define module_vts_driver(name, ic_numbers, init, exit) \
	int vts_driver_##name##_init(int *load) \
	{ \
		bool ret; \
		ret = vts_is_load_driver(#name, ic_numbers, ARRAY_SIZE(ic_numbers)); \
		*load = ret; \
		if (!ret) \
			return -ENODEV; \
	\
		return init; \
	} \
	void vts_driver_##name##_exit(void) \
	{ \
		exit; \
	} \

#endif
