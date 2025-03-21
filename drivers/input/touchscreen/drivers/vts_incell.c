#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/version.h>

#include "vts_core.h"

#define callback(incell, func, display_id) do { \
		int ret = 0; \
		struct vts_device *vtsdev = incell_to_vts(incell); \
		vts_dev_info(vtsdev, "%s\n", #func); \
		if (incell && incell->ops && incell->ops->func) \
			ret = incell->ops->func(incell, display_id); \
		if (ret) { \
			vts_dev_err(vtsdev, "incell callback %s error, display_id = %d, ret = %d", #func, display_id, ret); \
		} \
	} while (0)

static int vts_lcm_state_process(struct vts_incell *incell, unsigned long event, int blank, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);

	vts_dev_info(vtsdev, "event:%ld, blank:%d, display_id:%d\n", event, blank, display_id);
	if (display_id != 0) {
		vts_dev_info(vtsdev, "display id is non-zero, not phone display, do nothing\n");
		return 0;
	}

	if (event == VTS_EVENT_BLANK && blank == VTS_BLANK_UNBLANK) {
		callback(incell, resume, display_id);
		vts_device_unlock(vtsdev);
		__pm_stay_awake(incell->lcd_wakelock);
	} else if (event == VTS_EARLY_EVENT_BLANK && blank == VTS_BLANK_UNBLANK) {
		vts_device_lock(vtsdev);
		callback(incell, early_resume, display_id);
	} else if (event == VTS_EVENT_BLANK && blank == VTS_BLANK_POWERDOWN) {
		callback(incell, suspend, display_id);
		vts_device_unlock(vtsdev);
	} else if (event == VTS_EARLY_EVENT_BLANK && blank == VTS_BLANK_POWERDOWN){
		__pm_relax(incell->lcd_wakelock);
		vts_device_lock(vtsdev);
		callback(incell, early_suspend, display_id);
	}
	return 0;
}

/*高通平台并且大于kernel-5.10使用drm event notify*/
#if (defined(QCOM_FOR_VIVO_TS) && !defined(MTK_DRM_NOTIFY) && LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static void vts_drm_event_notif_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *notification, void *incell)
{
	if (!notification) {
		VTE("Invalid notification\n");
		return;
	}
	VTD("Notification type:%d, early_trigger:%d", notification->notif_type, notification->notif_data.early_trigger);
	
	vts_lcm_state_process(incell, notification->notif_data.early_trigger, notification->notif_type, 0);
}
#endif

static int vts_lcm_state_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct vts_incell *incell = container_of(self, struct vts_incell, nb);
	int blank;
#if (!defined(MTK_DRM_NOTIFY) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)) 
	display_notifier *evdata = data;
	if(evdata == NULL){
		VTE("evdata is NULL");
		return 0;
	}
	if(evdata->data == NULL){
		VTE("evdata->data is NULL");
		return 0;	
	}
	if(IS_ERR(evdata) || IS_ERR(evdata->data)){
		VTE("evdata or evdata->data is ERR point");
		return 0;	
	}
	blank = *(int *)evdata->data;
	return vts_lcm_state_process(incell, event, blank, vts_get_display_id(evdata));
#else 
	if(data == NULL){
		VTE("data is ERR point");
		return 0;	
	}
	blank = *(int *)data;
	return vts_lcm_state_process(incell, event, blank, 0);
#endif
}

static int incell_early_suspend(struct vts_incell *incell, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);
	int ret = 0;

	ret = vts_call_ic_ops(vtsdev, early_suspend);
	if (ret) {
		vts_dev_err(vtsdev, "early_suspend run error!");
		return ret;
	}

	return 0;
}

static void incell_suspend_work(struct work_struct *work)
{
	struct vts_incell *incell = container_of(work, struct vts_incell, suspend_work);
	struct vts_device *vtsdev = incell_to_vts(incell);

	vts_dev_dbg(vtsdev, "step3:suspend work execute start\n");
	vts_state_set_sync(vtsdev, VTS_STA_TDDI_LCD, 0);
	up(&incell->sem);
	vts_dev_dbg(vtsdev, "step4:suspend work execute finished\n");
}

static int incell_suspend(struct vts_incell *incell, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);
	int ret;

	vts_dev_dbg(vtsdev, "step1:enqueue suspend work\n");
	queue_work(incell->wq, &incell->suspend_work);
	vts_dev_dbg(vtsdev, "step2:wait suspend work start\n");
	ret = down_timeout(&incell->sem, 2*HZ);
	vts_dev_dbg(vtsdev, "step5:wait suspend work finished, ret = %d\n", ret);
	return ret;
}

static int incell_early_resume(struct vts_incell *incell, int display_id)
{
	 struct vts_device *vtsdev = incell_to_vts(incell);
	 int ret = 0;

	 ret = vts_call_ic_ops(vtsdev, early_resume);
	 if (ret) {
		vts_dev_err(vtsdev, "early_resume run error!");
		return ret;
	 }

	 return 0;
}

static int incell_resume(struct vts_incell *incell, int display_id)
{
	 struct vts_device *vtsdev = incell_to_vts(incell);
	 return vts_state_set(vtsdev, VTS_STA_TDDI_LCD, 1);
}

static const struct vts_incell_ops incell_ops = {
	.early_suspend = incell_early_suspend,
	.suspend = incell_suspend,
	.early_resume = incell_early_resume,
	.resume = incell_resume,
};

static inline const char *vts_lcd_name(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vts_lcd";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vts_lcd_second";
	else
		return "invalid type";
}

int vts_incell_init(struct vts_device *vtsdev)
{
	struct vts_incell *incell = &vtsdev->incell;
	u32 tddi;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);
	if (!tddi){
		VTI("not incell!!!!!!!!!!!");
		return 0;
		}
	incell->wq = create_singlethread_workqueue(vts_name(vtsdev));
	if (!incell->wq) {
		vts_dev_err(vtsdev, "create workqueue error!\n");
		return -ENOMEM;
	}

	incell->lcd_wakelock = vts_wakelock_register(vtsdev, vts_lcd_name(vtsdev));
	if (!incell->lcd_wakelock) {
		vts_dev_err(vtsdev, "lcd wakelock init fail!\n");
		return -ENOMEM;
	}

	INIT_WORK(&incell->suspend_work, incell_suspend_work);
	sema_init(&incell->sem, 0);
	incell->ops = &incell_ops;
	incell->nb.notifier_call = vts_lcm_state_callback;
#if defined(MTK_DRM_NOTIFY)
	return vts_lcm_register_client("vts_incell", &incell->nb);
#elif (defined(CONFIG_ARCH_QCOM) && !defined(MTK_DRM_NOTIFY) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
	if (vtsdev->module->vts_lcm_module_active) {
		if(vtsdev->module->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			return vts_lcm_register_client(vtsdev->module->active_panel_v2, &incell->nb);
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	} else {
		if(vtsdev->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			return vts_lcm_register_client(vtsdev->active_panel_v2, &incell->nb);
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	}
#elif (defined(QCOM_FOR_VIVO_TS) && LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) //高通平台大于kernel-5.10使用高通notify
	if (vtsdev->module->vts_lcm_module_active) {
		if(vtsdev->module->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			incell->notifier_entry = vts_lcm_register_client(PANEL_EVENT_NOTIFICATION_PRIMARY, PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, 
								vtsdev->module->active_panel_v2, &vts_drm_event_notif_callback, incell);
			return 0;
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	} else {
		if(vtsdev->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			incell->notifier_entry = vts_lcm_register_client(PANEL_EVENT_NOTIFICATION_PRIMARY, PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, 
								vtsdev->active_panel_v2, &vts_drm_event_notif_callback, incell);
			return 0;
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	}
 
#else
 
 return vts_lcm_register_client(&incell->nb);
#endif
}

int vts_incell_deinit(struct vts_device *vtsdev)
{
	struct vts_incell *incell = &vtsdev->incell;
	int ret = 0;
	u32 tddi;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);

	if (!tddi)
		return 0;

	vts_wakelock_unregister(incell->lcd_wakelock);
	incell->lcd_wakelock = NULL;

#if (!defined(MTK_DRM_NOTIFY) && defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
	VTI("active_panel_v2  deinit ");
	if(vtsdev->module->active_panel_v2)
	 ret = vts_lcm_unregister_client(vtsdev->module->active_panel_v2, &incell->nb);
#elif (defined(QCOM_FOR_VIVO_TS) && LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	if(vtsdev->module->active_panel_v2)
		vts_lcm_unregister_client(incell->notifier_entry);
#else
	 ret = vts_lcm_unregister_client(&incell->nb);
#endif
	incell->nb.notifier_call = NULL;
	incell->ops = NULL;
	destroy_workqueue(incell->wq);
	return ret;
}
