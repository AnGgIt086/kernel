/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

struct mtk_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct usb_role_switch *role_sw;
	unsigned int c_role; /* current data role */
	struct workqueue_struct *extcon_wq;
	struct regulator *vbus;
	struct gpio_desc *id_gpiod;
	unsigned int vbus_vol;
	unsigned int vbus_cur;
	unsigned int id_irq;
	bool vbus_on;
	struct device_connection dev_conn;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct delayed_work wq_detcable;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcpc_device *tcpc_dev;
	struct notifier_block tcpc_nb;
#endif
	bool bypss_typec_sink;
	unsigned int dr;
	bool host_mode_enabled;
	bool typec_hw_det;
	struct delayed_work host_disabled_work;
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
	struct alarm host_disabled_alarm;
#endif
	struct mutex switch_mutex;
	struct wakeup_source *switch_lock;
};

struct usb_role_info {
	struct mtk_extcon_info *extcon;
	struct delayed_work dwork;
	unsigned int d_role; /* desire data role */
};

enum {
	DUAL_PROP_MODE_UFP = 0,
	DUAL_PROP_MODE_DFP,
	DUAL_PROP_MODE_NONE,
};

enum {
	DUAL_PROP_PR_SRC = 0,
	DUAL_PROP_PR_SNK,
	DUAL_PROP_PR_NONE,
};

enum host_mode {
	HOST_MODE_DISABLE = 0,
	HOST_MODE_ENABLE,
	HOST_MODE_DISABLE_HW_DET,
	HOST_MODE_ENABLE_HW_DET,
};

enum {
	DUAL_PROP_DR_HOST = 0,
	DUAL_PROP_DR_DEVICE,
	DUAL_PROP_DR_NONE,
};

extern void mt_usbaudio_connect(void);
extern void mt_usbaudio_disconnect(void);
#if IS_ENABLED(CONFIG_USB_MTU3)
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#endif
