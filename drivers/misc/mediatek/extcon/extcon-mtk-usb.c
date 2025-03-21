// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/pm_wakeup.h>
#include <linux/of_platform.h>
#include <linux/alarmtimer.h>
#include <linux/time.h>

#include "extcon-mtk-usb.h"
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
extern void vivo_headset_accdet(bool plugin);
extern void config_otg_mode(bool enable);
#endif
#include <linux/usb/vivo_debug.h>
#undef dev_info
#define dev_info dev_err

#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
#define ACCDET_TCPC_VOTER "ACCDET_TCPC_VOTER"
#define USBID_TCPC_VOTER  "USBID_TCPC_VOTER"
extern bool upmu_is_chr_det(void);
extern int get_typec_drp_voter_effective(const char *voter);
#define USB_TCPC_VOTER "USB_TCPC_VOTER"
extern void vote_typec_drp(const char *voter, bool drp);
#endif

#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
struct timespec64 host_disabled_delay_time = {300, 0};
#endif

#if IS_ENABLED(CONFIG_SND_SOC_MAX20328)
extern int max20328_read_reg_val(unsigned int reg, unsigned int *val);
#endif

#define HOST_DISABLED_DELAY_TIME  (300*1000)
static BLOCKING_NOTIFIER_HEAD(mt_usb_notifier_list);

static int usb_cable_status;
static struct mtk_extcon_info *g_extcon_info;

#if IS_ENABLED(CONFIG_TCPC_CLASS)
#include "tcpm.h"
#endif

#ifdef CONFIG_MTK_USB_TYPEC_U3_MUX
#include "mux_switch.h"
#endif

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void mtk_usb_extcon_update_role(struct work_struct *work)
{
	struct usb_role_info *role = container_of(to_delayed_work(work),
					struct usb_role_info, dwork);
	struct mtk_extcon_info *extcon = role->extcon;
	unsigned int cur_dr, new_dr;

	if (!g_extcon_info) {
		pr_info("g_extcon_info = NULL\n");
		return;
	}
	cur_dr = extcon->c_role;
	new_dr = role->d_role;

	dev_info(extcon->dev, "%s, extcon->c_role:%d, g_extcon_info->dr:%d\n", \
		__func__, extcon->c_role, g_extcon_info->dr);
	dev_info(extcon->dev, "%s, cur_dr(%d) new_dr(%d), 0:none, 1:host, 2:device\n", \
		__func__, cur_dr, new_dr);
	dev_info(extcon->dev, "%s, g_extcon_info->typec_hw_det:%d\n", \
		__func__, g_extcon_info->typec_hw_det);

	g_extcon_info->dr = new_dr;
	/* none -> device */
	if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	/* none -> host */
	} else if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_HOST) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled || g_extcon_info->typec_hw_det)
			extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
		else
			pr_info("%s: %d: host_mode_enabled = %d, typec_hw_det = %d, ignore set host mode\n",
					__func__, __LINE__, g_extcon_info->host_mode_enabled, g_extcon_info->typec_hw_det);
		mutex_unlock(&g_extcon_info->switch_mutex);
	/* device -> none */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
	/* host -> none */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
	/* device -> host */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_HOST) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
	/* host -> device */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	}

	/* usb role switch */
	if (extcon->role_sw) {
		if (new_dr == DUAL_PROP_DR_DEVICE)
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_DEVICE);
		else if (new_dr == DUAL_PROP_DR_HOST)
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_HOST);
		else
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_NONE);
	}
	if (new_dr == DUAL_PROP_DR_HOST)
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
		alarm_try_to_cancel(&g_extcon_info->host_disabled_alarm);
#else
		cancel_delayed_work(&g_extcon_info->host_disabled_work);
#endif
	else {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled) {
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
			ktime_t kt = timespec64_to_ktime(host_disabled_delay_time);
			alarm_start_relative(&g_extcon_info->host_disabled_alarm, kt);
#else
			queue_delayed_work(g_extcon_info->extcon_wq,
					   &g_extcon_info->host_disabled_work,
					   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
#endif
		}
		mutex_unlock(&g_extcon_info->switch_mutex);
	}

	extcon->c_role = new_dr;
	kfree(role);
}

static int mtk_usb_extcon_set_role(struct mtk_extcon_info *extcon,
						unsigned int role)
{
	struct usb_role_info *role_info;

	/* create and prepare worker */
	role_info = kzalloc(sizeof(*role_info), GFP_ATOMIC);
	if (!role_info)
		return -ENOMEM;

	INIT_DELAYED_WORK(&role_info->dwork, mtk_usb_extcon_update_role);

	role_info->extcon = extcon;
	role_info->d_role = role;
	/* issue connection work */
	queue_delayed_work(extcon->extcon_wq, &role_info->dwork, 0);

	return 0;
}

static int mtk_usb_extcon_psy_notifier(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct mtk_extcon_info *extcon = container_of(nb,
					struct mtk_extcon_info, psy_nb);
	union power_supply_propval pval;
	union power_supply_propval ival;
	union power_supply_propval tval;
	int ret;

	if (event != PSY_EVENT_PROP_CHANGED || psy != extcon->usb_psy)
		return NOTIFY_DONE;

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get online prop\n");
		return NOTIFY_DONE;
	}

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_AUTHENTIC, &ival);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get authentic prop\n");
		ival.intval = 0;
	}

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_TYPE, &tval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get usb type\n");
		return NOTIFY_DONE;
	}

	dev_info(extcon->dev, "online=%d, ignore_usb=%d, type=%d\n",
				pval.intval, ival.intval, tval.intval);

	if (ival.intval)
		return NOTIFY_DONE;

#ifdef CONFIG_TCPC_CLASS
	if (extcon->c_role == DUAL_PROP_DR_NONE && pval.intval &&
			(tval.intval == POWER_SUPPLY_TYPE_USB ||
			tval.intval == POWER_SUPPLY_TYPE_USB_CDP))
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
#else
	if (pval.intval && (tval.intval == POWER_SUPPLY_TYPE_USB ||
			tval.intval == POWER_SUPPLY_TYPE_USB_CDP))
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
	else
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
#endif
	return NOTIFY_DONE;
}

static int mtk_usb_extcon_psy_init(struct mtk_extcon_info *extcon)
{
	int ret = 0;
	struct device *dev = extcon->dev;
	union power_supply_propval pval;
	union power_supply_propval ival;
	union power_supply_propval tval;

	extcon->usb_psy = devm_power_supply_get_by_phandle(dev, "charger");
	if (IS_ERR_OR_NULL(extcon->usb_psy)) {
		dev_err(dev, "fail to get usb_psy\n");
		extcon->usb_psy = NULL;
		return -EINVAL;
	}

	extcon->psy_nb.notifier_call = mtk_usb_extcon_psy_notifier;
	ret = power_supply_reg_notifier(&extcon->psy_nb);
	if (ret) {
		dev_err(dev, "fail to register notifer\n");
		return ret;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get online prop\n");
		return 0;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_AUTHENTIC, &ival);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get authentic prop\n");
		ival.intval = 0;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_USB_TYPE, &tval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get usb type\n");
		return 0;
	}

	dev_info(extcon->dev, "online=%d, ignore_usb=%d, type=%d\n",
				pval.intval, ival.intval, tval.intval);

	if (ival.intval)
		return 0;

	if (pval.intval && (tval.intval == POWER_SUPPLY_USB_TYPE_SDP ||
			tval.intval == POWER_SUPPLY_USB_TYPE_CDP))
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);

	return 0;
}


static int mtk_usb_extcon_set_vbus(struct mtk_extcon_info *extcon,
							bool is_on)
{
	struct regulator *vbus = extcon->vbus;
	struct device *dev = extcon->dev;

	/* vbus is optional */
	if (!vbus || extcon->vbus_on == is_on)
		return 0;

	dev_info(dev, "vbus turn %s\n", is_on ? "on" : "off");

	if (is_on) {
		pr_err("%s, set_vbus on\n", __func__);
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		config_otg_mode(true);
#endif
	} else {
		pr_err("%s, set_vbus off\n", __func__);
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		config_otg_mode(false);
#endif
	}

	extcon->vbus_on = is_on;

	return 0;
}

static int tcpc_audio_enable(void)
{
	mt_usbaudio_connect();
	return 0;
}

static int tcpc_audio_disable(void)
{
	mt_usbaudio_disconnect();
	return 0;
}

#if IS_ENABLED(CONFIG_TCPC_CLASS)
static int mtk_extcon_tcpc_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_extcon_info *extcon =
			container_of(nb, struct mtk_extcon_info, tcpc_nb);
	struct device *dev = extcon->dev;
	bool is_device = (extcon->c_role == DUAL_PROP_DR_DEVICE);
	bool vbus_on;

	pr_err("%s, %d otg_notifier callback, current event: %d\n", __func__, __LINE__, event);
	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		dev_info(dev, "tcpc_notifier source vbus = %dmv\n",
				 noti->vbus_state.mv);
		vbus_on = (noti->vbus_state.mv) ? true : false;
		mtk_usb_extcon_set_vbus(extcon, vbus_on);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		dev_info(dev, "tcpc_notifier old_state=%d, new_state=%d, 0:none, 1:sink, 2:source\n",
				noti->typec_state.old_state,
				noti->typec_state.new_state);

#ifdef CONFIG_MTK_USB_TYPEC_U3_MUX
		if ((noti->typec_state.new_state == TYPEC_ATTACHED_SRC ||
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
			noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC ||
			noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC)) {
			if (noti->typec_state.polarity == 0)
				usb3_switch_set(TYPEC_ORIENTATION_REVERSE);
			else
				usb3_switch_set(TYPEC_ORIENTATION_NORMAL);
		} else if (noti->typec_state.new_state == TYPEC_UNATTACHED) {
			usb3_switch_set(TYPEC_ORIENTATION_NONE);
		}
#endif
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			dev_info(dev, "tcpc_notifier Type-C SRC plug in\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
		} else if (!(extcon->bypss_typec_sink) &&
			noti->typec_state.old_state == TYPEC_UNATTACHED &&
			(noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
			noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC ||
			noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC)) {
			dev_info(dev, "tcpc_notifier Type-C SINK plug in\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
			noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
				dev_info(dev, "tcpc_notifier Type-C plug out\n");
				mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
				is_device = false;
#else
				if (is_device && extcon->bypss_typec_sink) {
					/*
					 * if bypss_typec_sink
					 * when in device mode, we should bypass typec plug out
					 */
					dev_info(dev, "tcpc_notifier Type-C plug out ignore in device mode\n");
				} else {
					dev_info(dev, "tcpc_notifier Type-C plug out\n");
					mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
				}
#endif
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("tcpc_notifier %s Audio Plug in\n", __func__);
			tcpc_audio_enable();
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO &&
					noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_info("tcpc_notifier %s Audio Plug out\n", __func__);
			tcpc_audio_disable();
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		dev_info(dev, "tcpc_notifier %s dr_swap, new role=%d\n",
				__func__, noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_UFP &&
				extcon->c_role == DUAL_PROP_DR_HOST) {
			dev_info(dev, "tcpc_notifier switch role to device\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
		} else if (noti->swap_state.new_role == PD_ROLE_DFP &&
				extcon->c_role == DUAL_PROP_DR_DEVICE) {
			dev_info(dev, "tcpc_notifier switch role to host\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
		}
		break;
	}

	return NOTIFY_OK;
}

static int mtk_usb_extcon_tcpc_init(struct mtk_extcon_info *extcon)
{
	struct tcpc_device *tcpc_dev;
	int ret;

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		dev_err(extcon->dev, "get tcpc device fail\n");
		return -ENODEV;
	}

	extcon->tcpc_nb.notifier_call = mtk_extcon_tcpc_notifier;
	ret = register_tcp_dev_notifier(tcpc_dev, &extcon->tcpc_nb,
		TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_VBUS |
		TCP_NOTIFY_TYPE_MISC);
	if (ret < 0) {
		dev_err(extcon->dev, "register notifer fail\n");
		return -EINVAL;
	}

	extcon->tcpc_dev = tcpc_dev;

	return 0;
}
#endif

static void mtk_usb_extcon_detect_cable(struct work_struct *work)
{
	struct mtk_extcon_info *extcon = container_of(to_delayed_work(work),
						    struct mtk_extcon_info,
						    wq_detcable);
	int id;

	/* check ID and update cable state */
	id = extcon->id_gpiod ?
		gpiod_get_value_cansleep(extcon->id_gpiod) : 1;

	/* at first we clean states which are no longer active */
	if (id) {
		mtk_usb_extcon_set_vbus(extcon, false);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
		pr_err("charger_notify_usb_to_set_role %s, %d, USB_ROLE_NONE \n", __func__, __LINE__);
	} else {
		mtk_usb_extcon_set_vbus(extcon, true);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
		pr_err("charger_notify_usb_to_set_role %s, %d, USB_ROLE_HOST \n", __func__, __LINE__);
	}
}

static irqreturn_t mtk_usb_idpin_handle(int irq, void *dev_id)
{
	struct mtk_extcon_info *extcon = dev_id;

	/* issue detection work */
	queue_delayed_work(system_power_efficient_wq, &extcon->wq_detcable, 0);

	return IRQ_HANDLED;
}

static int mtk_usb_extcon_id_pin_init(struct mtk_extcon_info *extcon)
{
	int ret = 0;
	int id;

	extcon->id_gpiod = devm_gpiod_get(extcon->dev, "id", GPIOD_IN);

	if (!extcon->id_gpiod || IS_ERR(extcon->id_gpiod)) {
		dev_err(extcon->dev, "failed to get id gpio\n");
		extcon->id_gpiod = NULL;
		return -EINVAL;
	}

	extcon->id_irq = gpiod_to_irq(extcon->id_gpiod);
	if (extcon->id_irq < 0) {
		dev_err(extcon->dev, "failed to get ID IRQ\n");
		extcon->id_gpiod = NULL;
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&extcon->wq_detcable, mtk_usb_extcon_detect_cable);

	ret = devm_request_threaded_irq(extcon->dev, extcon->id_irq, NULL,
			mtk_usb_idpin_handle, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(extcon->dev), extcon);

	if (ret < 0) {
		dev_err(extcon->dev, "failed to request handler for ID IRQ\n");
		extcon->id_gpiod = NULL;
		return ret;
	}
	enable_irq_wake(extcon->id_irq);

	// get id pin value when boot on
	id = extcon->id_gpiod ?	gpiod_get_value_cansleep(extcon->id_gpiod) : 1;
	dev_info(extcon->dev, "id value : %d\n", id);
	if (!id) {
		mtk_usb_extcon_set_vbus(extcon, true);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_USB_MTU3)
static void issue_connection_work(unsigned int dr)
{
	int retry = 20;

	while (retry && !g_extcon_info) {
		pr_info("usb extcon driver is not ready, waiting 100ms, retry: %d\n", retry);
		mdelay(100);
		retry--;
	}

	if (retry == 0 && !g_extcon_info)
		return;

	/* issue connection work */
	mtk_usb_extcon_set_role(g_extcon_info, dr);
}

void mt_usb_connect(void)
{
	pr_info("%s in mtk extcon\n", __func__);

	issue_connection_work(DUAL_PROP_DR_DEVICE);
}
EXPORT_SYMBOL_GPL(mt_usb_connect);

void mt_usb_disconnect(void)
{
	pr_info("%s  in mtk extcon\n", __func__);
	issue_connection_work(DUAL_PROP_DR_NONE);
}
EXPORT_SYMBOL_GPL(mt_usb_disconnect);
#endif


void mt_usbaudio_connect(void)
{
	if (g_extcon_info->typec_hw_det) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled)
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
			alarm_try_to_cancel(&g_extcon_info->host_disabled_alarm);
#else
			cancel_delayed_work(&g_extcon_info->host_disabled_work);
#endif
		mutex_unlock(&g_extcon_info->switch_mutex);
		pr_info("%s\n", __func__);
	}
}
EXPORT_SYMBOL_GPL(mt_usbaudio_connect);

void mt_usbaudio_disconnect(void)
{
	if (g_extcon_info->typec_hw_det) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled) {
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
			ktime_t kt = timespec64_to_ktime(host_disabled_delay_time);
			alarm_start_relative(&g_extcon_info->host_disabled_alarm, kt);
#else
			queue_delayed_work(g_extcon_info->extcon_wq,
					   &g_extcon_info->host_disabled_work,
					   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
#endif
		}
		mutex_unlock(&g_extcon_info->switch_mutex);
		pr_info("%s\n", __func__);
	}
}
EXPORT_SYMBOL_GPL(mt_usbaudio_disconnect);

bool mt_usb_is_host(void)
{
	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return 0;
	}

	return g_extcon_info->dr == DUAL_PROP_DR_HOST;
}
EXPORT_SYMBOL_GPL(mt_usb_is_host);

int mt_usb_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mt_usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mt_usb_register_notify);

void mt_usb_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&mt_usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mt_usb_unregister_notify);

static void mt_usb_notify_host_mode(enum host_mode mode)
{
	blocking_notifier_call_chain(&mt_usb_notifier_list, mode, NULL);
}

static int mt_usb_notify(struct notifier_block *self,
			 unsigned long is_host, void *data)
{
	//static uint8_t typec_role = TYPEC_ROLE_SNK;

	switch (is_host) {
	case HOST_MODE_DISABLE:
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		//vivo_set_typec_power_role(typec_role);
		vote_typec_drp(USB_TCPC_VOTER, false);
#endif
		pr_info("%s: set drp mode false vote\n", __func__);
		break;
	case HOST_MODE_ENABLE:
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		//typec_role = vivo_get_typec_power_role();
		//vivo_set_typec_power_role(TYPEC_ROLE_TRY_SRC);
		vote_typec_drp(USB_TCPC_VOTER, true);
#endif
		pr_info("%s: set drp mode true vote\n", __func__);
		break;
	case HOST_MODE_DISABLE_HW_DET:
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		vote_typec_drp(USB_TCPC_VOTER, false);
#endif
		pr_info("%s: set drp mode false vote\n", __func__);
		break;
	case HOST_MODE_ENABLE_HW_DET:
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
		vote_typec_drp(USB_TCPC_VOTER, true);
#endif
		pr_info("%s: set drp mode true vote\n", __func__);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block mt_usb_nb = {
	.notifier_call = mt_usb_notify,
};

static ssize_t otg_mode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	ssize_t size;

	pr_err("%s, usb_cable_status:%d, g_extcon_info->typec_hw_det:%d\n", \
		__func__, usb_cable_status, g_extcon_info->typec_hw_det);

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
	pr_err("%s, %d, upmu_is_chr_det():%u\n", __func__, __LINE__, upmu_is_chr_det());
#endif

	if (mt_usb_is_host()) {
		pr_info("%s: ----otg enter host mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "host\n");
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
	} else if (upmu_is_chr_det() || usb_cable_status) {
#else
	} else if (usb_cable_status) {
#endif
		pr_info("%s: ----otg enter peripheral mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "peripheral\n");
	} else {
		pr_info("%s: ----otg enter none mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "none\n");
	}

	if (g_extcon_info->typec_hw_det) {
		if (!strncmp(buf, "peripheral", 10) || !strncmp(buf, "none", 4)) {
#if IS_ENABLED(CONFIG_VIVO_CHARGING_NEW_ARCH)
			if (get_typec_drp_voter_effective(USBID_TCPC_VOTER)) {
				pr_info("%s: ----otg enter ccopened mode----\n", __func__);
				size = snprintf(buf, PAGE_SIZE, "%s", "ccopened\n");
			}
#endif
		}
	}

	pr_err("%s,%d buf:%s\n", __func__, __LINE__, buf);

	return size;
}

static ssize_t otg_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	return -EPERM;
}



static ssize_t host_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	ssize_t size;

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&g_extcon_info->switch_mutex);
	if (g_extcon_info->host_mode_enabled)
		size = snprintf(buf, PAGE_SIZE, "%s", "enabled\n");
	else
		size = snprintf(buf, PAGE_SIZE, "%s", "disabled\n");
	mutex_unlock(&g_extcon_info->switch_mutex);

	return size;
}

static ssize_t host_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	char value[32];
	bool is_enable;

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

	if (sscanf(buf, "%s", value) != 1)
		return -EINVAL;

	if (!strncmp(value, "enabled", 7))
		is_enable = true;
	else if (!strncmp(value, "disabled", 8))
		is_enable = false;
	else
		return -EINVAL;

	mutex_lock(&g_extcon_info->switch_mutex);
	if (is_enable == g_extcon_info->host_mode_enabled) {
		mutex_unlock(&g_extcon_info->switch_mutex);
		return size;
	}

    if (is_enable) {
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
		ktime_t kt = timespec64_to_ktime(host_disabled_delay_time);
		alarm_start_relative(&g_extcon_info->host_disabled_alarm, kt);
#else
		queue_delayed_work(g_extcon_info->extcon_wq,
				   &g_extcon_info->host_disabled_work,
				   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
		__pm_stay_awake(g_extcon_info->switch_lock);
#endif
		pr_info("%s: ===vivo otg enabled ===\n", __func__);
	} else {
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
		alarm_cancel(&g_extcon_info->host_disabled_alarm);
#else
		cancel_delayed_work(&g_extcon_info->host_disabled_work);
		if (!g_extcon_info->host_mode_enabled) {
			pr_info("%s: ===vivo otg already disabled ===\n", __func__);
			mutex_unlock(&g_extcon_info->switch_mutex);
			return size;
		}
		__pm_relax(g_extcon_info->switch_lock);
#endif
		pr_info("%s: ===vivo otg disabled ===\n", __func__);
	}
	g_extcon_info->host_mode_enabled = is_enable;
	if (!g_extcon_info->typec_hw_det)
		mt_usb_notify_host_mode(is_enable ? HOST_MODE_ENABLE : HOST_MODE_DISABLE);
	else
		mt_usb_notify_host_mode(is_enable ? HOST_MODE_ENABLE_HW_DET : HOST_MODE_DISABLE_HW_DET);
	mutex_unlock(&g_extcon_info->switch_mutex);

	return size;
}

DEVICE_ATTR(otg_mode,  0664, otg_mode_show, otg_mode_store);
DEVICE_ATTR(host_mode,	0664, host_mode_show, host_mode_store);

static struct attribute *otg_attributes[] = {
	&dev_attr_otg_mode.attr,
	&dev_attr_host_mode.attr,
	NULL
};

static const struct attribute_group otg_attr_group = {
	.attrs = otg_attributes,
};

static void host_disabled_work(struct work_struct *w)
{
	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return;
	}

	mutex_lock(&g_extcon_info->switch_mutex);
	if (!g_extcon_info->host_mode_enabled) {
		pr_info("%s: ===vivo otg already disabled ===\n", __func__);
		mutex_unlock(&g_extcon_info->switch_mutex);
		return;
	}
	g_extcon_info->host_mode_enabled = false;
	pr_info("%s: ===vivo otg disabled ===\n", __func__);
	if (!g_extcon_info->typec_hw_det)
		mt_usb_notify_host_mode(HOST_MODE_DISABLE);
	else
		mt_usb_notify_host_mode(HOST_MODE_DISABLE_HW_DET);
	if (g_extcon_info->switch_lock->active)
		__pm_relax(g_extcon_info->switch_lock);
	mutex_unlock(&g_extcon_info->switch_mutex);
}

#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
static enum alarmtimer_restart host_disabled_alarm(struct alarm *alarm, ktime_t now)
{
	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		goto end;
	}

	pr_info("%s: ===vivo otg disabled alarm ===\n", __func__);
	__pm_stay_awake(g_extcon_info->switch_lock);
	queue_delayed_work(g_extcon_info->extcon_wq,
			   &g_extcon_info->host_disabled_work, 0);
end:
	return ALARMTIMER_NORESTART;
}
#endif

static int mtk_usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_extcon_info *extcon;
	struct platform_device *conn_pdev;
	struct device_node *conn_np;
	int ret;

	extcon = devm_kzalloc(&pdev->dev, sizeof(*extcon), GFP_KERNEL);
	if (!extcon)
		return -ENOMEM;

	extcon->dev = dev;

	/* extcon */
	extcon->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(extcon->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, extcon->edev);
	if (ret < 0) {
		dev_info(dev, "failed to register extcon device\n");
		return ret;
	}

	/* usb role switch */
	conn_np = of_parse_phandle(dev->of_node, "dev-conn", 0);
	if (!conn_np) {
		dev_info(dev, "failed to get dev-conn node\n");
		return -EINVAL;
	}

	conn_pdev = of_find_device_by_node(conn_np);
	if (!conn_pdev) {
		dev_info(dev, "failed to get dev-conn pdev\n");
		return -EINVAL;
	}

	extcon->dev_conn.endpoint[0] = kasprintf(GFP_KERNEL,
				"%s-role-switch", dev_name(&conn_pdev->dev));
	extcon->dev_conn.endpoint[1] = dev_name(extcon->dev);
	extcon->dev_conn.id = "usb-role-switch";
	device_connection_add(&extcon->dev_conn);

	extcon->role_sw = usb_role_switch_get(extcon->dev);
	if (IS_ERR(extcon->role_sw)) {
		dev_err(dev, "failed to get usb role\n");
		return PTR_ERR(extcon->role_sw);
	}

	/* vbus */
	extcon->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(extcon->vbus)) {
		dev_err(dev, "failed to get vbus\n");
		return PTR_ERR(extcon->vbus);
	}

	if (!of_property_read_u32(dev->of_node, "vbus-voltage",
					&extcon->vbus_vol))
		dev_info(dev, "vbus-voltage=%d", extcon->vbus_vol);

	if (!of_property_read_u32(dev->of_node, "vbus-current",
					&extcon->vbus_cur))
		dev_info(dev, "vbus-current=%d", extcon->vbus_cur);

	extcon->bypss_typec_sink =
		of_property_read_bool(dev->of_node,
			"mediatek,bypss-typec-sink");

	extcon->extcon_wq = create_singlethread_workqueue("extcon_usb");
	if (!extcon->extcon_wq)
		return -ENOMEM;

	/* default turn off vbus */
	mtk_usb_extcon_set_vbus(extcon, false);

	/*get id resources*/
	ret = mtk_usb_extcon_id_pin_init(extcon);
	if (ret < 0)
		dev_info(dev, "failed to init id pin\n");

	/* power psy */
	ret = mtk_usb_extcon_psy_init(extcon);
	if (ret < 0)
		dev_err(dev, "failed to init psy\n");

#if IS_ENABLED(CONFIG_TCPC_CLASS)
	/* tcpc */
	ret = mtk_usb_extcon_tcpc_init(extcon);
	if (ret < 0)
		dev_err(dev, "failed to init tcpc\n");
#endif

	platform_set_drvdata(pdev, extcon);

	g_extcon_info = extcon;
	extcon->typec_hw_det = of_property_read_bool(dev->of_node, "vivo,typec-hw-det");
	dev_info(&pdev->dev, "typec-hw-det = %d\n", extcon->typec_hw_det);
	mutex_init(&extcon->switch_mutex);
	g_extcon_info->switch_lock = wakeup_source_register(dev, "switch_wakelock");
	INIT_DELAYED_WORK(&extcon->host_disabled_work, host_disabled_work);
#if IS_ENABLED(CONFIG_VIVO_USB_OTG_ALARM)
	alarm_init(&extcon->host_disabled_alarm, ALARM_REALTIME, host_disabled_alarm);
#endif
	ret = sysfs_create_group(&pdev->dev.kobj, &otg_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "falied to register sysfs: %d\n", ret);
		return ret;
	}

	mt_usb_register_notify(&mt_usb_nb);

	/* initial role should after g_extcon_info inited*/
	extcon->c_role = DUAL_PROP_DR_DEVICE;
	/* default initial role */
	mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);

#if IS_ENABLED(CONFIG_VIVO_FORCE_USB_DEVICE)
	extcon->c_role = DUAL_PROP_DR_NONE;

	/* default initial role */
	mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
	dev_info(dev, "force usb to device\n");
#endif
	pr_err("%s, %d done \n", __func__, __LINE__);
	return 0;
}

static int mtk_usb_extcon_remove(struct platform_device *pdev)
{
	struct mtk_extcon_info *extcon = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &otg_attr_group);
	cancel_delayed_work_sync(&g_extcon_info->host_disabled_work);
	if (extcon->dev_conn.id)
		device_connection_remove(&extcon->dev_conn);

	return 0;
}

static void mtk_usb_extcon_shutdown(struct platform_device *pdev)
{
	struct mtk_extcon_info *extcon = platform_get_drvdata(pdev);

	if (extcon->c_role == DUAL_PROP_DR_HOST) {
		dev_info(extcon->dev, "set host vbus off when shutdown\n");
		mtk_usb_extcon_set_vbus(extcon, false);
	}
}

static const struct of_device_id mtk_usb_extcon_of_match[] = {
	{ .compatible = "mediatek,extcon-usb", },
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_usb_extcon_of_match);

static struct platform_driver mtk_usb_extcon_driver = {
	.probe		= mtk_usb_extcon_probe,
	.remove		= mtk_usb_extcon_remove,
	.shutdown	= mtk_usb_extcon_shutdown,
	.driver		= {
		.name	= "mtk-extcon-usb",
		.of_match_table = mtk_usb_extcon_of_match,
	},
};

static int __init mtk_usb_extcon_init(void)
{
	return platform_driver_register(&mtk_usb_extcon_driver);
}
late_initcall(mtk_usb_extcon_init);

static void __exit mtk_usb_extcon_exit(void)
{
	platform_driver_unregister(&mtk_usb_extcon_driver);
}
module_exit(mtk_usb_extcon_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Extcon USB Driver");
