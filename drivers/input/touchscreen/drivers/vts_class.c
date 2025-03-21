#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/sensors.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "vts_core.h"

static struct class *vts_class = NULL;
static atomic_t nr_devices = ATOMIC_INIT(0);

#define VTS_FW_ATTR(name, type) \
	static ssize_t vts_##name##_show(struct device *dev, \
				struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_get, type, buf, PAGE_SIZE); \
		if (ret) \
			return ret; \
	\
		sprintf(buf + strlen(buf), "\n"); \
		return strlen(buf); \
	} \
	\
	static ssize_t vts_##name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
		char *tmp = (char *)buf; \
		char *path; \
		char *update_s; \
		int update; \
	\
		path = strsep(&tmp, ","); \
		if (!path) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		update_s = strsep(&tmp, ","); \
		if (!update_s) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		if(kstrtoint(update_s, 10, &update)) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_set, type, buf, update != 0); \
		if (ret) \
			return ret; \
	\
		return size; \
	} \
	static DEVICE_ATTR(name, 0644, vts_##name##_show, vts_##name##_store)

static ssize_t vts_lcmid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid;
	int ret;

	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid, &lcmid);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", lcmid);
}

static ssize_t vts_lcmid_compatible_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid[VTS_MODULE_LCMID_NR_MAX];
	int ret;
	ssize_t count = 0;
	size_t sz = ARRAY_SIZE(lcmid);
	int i;
	struct vts_board_version_id *board_version_id;

	memset(lcmid, 0, sizeof(lcmid));
	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid_compatible, lcmid, &sz);
	if (ret)
		return ret;

	if (vtsdev->find_board_id) { //board_id_bit每个位的值为16进制需*4后再右移
		list_for_each_entry(board_version_id, &vtsdev->module->board_version_id_list, entry) {
			if (((vtsdev->cmdline_board_version >> (board_version_id->board_id_bit * 4)) & 0x0001) == board_version_id->board_id_val) {
				lcmid[0] = board_version_id->module_id;
				count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d \n", lcmid[0]);
				return count;
			}
		}
	} else {
		if (sz == 0)
			return -ENODEV;

		for (i = 0; i < sz; i++) {
			if (i == (sz -1))
				count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d\n", lcmid[i]);
			else
				count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d ", lcmid[i]);
		}

		return count;
	}

	return count;
}

/* 获取dts中与cmdline 匹配的 softid */
static ssize_t vts_softid_cmdline_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	struct vts_vendor_module_id *vendor_module_id;
	u32 softid = vtsdev->module->softid_cmdline;
	u32 matched_softid = 0;

	if(softid <= 0)
		return -ENODEV;

	list_for_each_entry(vendor_module_id, &vtsdev->module->vendor_module_list, entry) {
		if (vendor_module_id->softid == softid) {
			matched_softid = vendor_module_id->softid;
			break;
		}
	}
	return snprintf(buf, PAGE_SIZE, "0x%x\n", matched_softid);
}

/* 获取dts中softid对应的module_id */
static ssize_t vts_softid_module_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 module_id[VTS_MODULE_LCMID_NR_MAX];
	int ret;
	ssize_t count = 0;
	size_t sz = ARRAY_SIZE(module_id);
	int i;

	memset(module_id, 0, sizeof(module_id));
	ret = vts_call_func_sync(vtsdev, int, vts_get_module_id_compatible, module_id, &sz);
	if (ret)
		return ret;

	for (i = 0; i < sz; i++)
		if (i == (sz - 1))
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "0x%x\n", module_id[i]);
		else
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "0x%x ", module_id[i]);

	return count;
}

static ssize_t vts_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u64 version;
	int ret;
	ret = vts_call_func_sync(vtsdev, int, vts_firmware_version_get, &version);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%llx\n", version);
}

VTS_FW_ATTR(firmware, VTS_FW_TYPE_FW);
VTS_FW_ATTR(firmware_config, VTS_FW_TYPE_CONFIG);
VTS_FW_ATTR(threshold, VTS_FW_TYPE_LIMIT);
VTS_FW_ATTR(firmware_mp, VTS_FW_TYPE_MP);
static DEVICE_ATTR(lcmid, 0644, vts_lcmid_show, NULL);
static DEVICE_ATTR(lcmid_compatible, 0644, vts_lcmid_compatible_show, NULL);
static DEVICE_ATTR(softid_cmdline, 0644, vts_softid_cmdline_show, NULL);
static DEVICE_ATTR(softid_module, 0644, vts_softid_module_show, NULL);
static DEVICE_ATTR(version, 0644, vts_version_show, NULL);

static struct attribute *vts_dev_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_firmware.attr,
	&dev_attr_firmware_mp.attr,
	&dev_attr_firmware_config.attr,
	&dev_attr_threshold.attr,
	NULL,
};

static struct attribute_group vts_attr_group = {
	.name	= NULL,
	.attrs	= vts_dev_attrs,
};

static struct attribute *vts_property_attrs[] = {
	&dev_attr_lcmid.attr,
	&dev_attr_lcmid_compatible.attr,
	&dev_attr_softid_cmdline.attr,
	&dev_attr_softid_module.attr,
	NULL,
};

static struct attribute_group property_attr_group = {
	.name	= "properties",
	.attrs	= vts_property_attrs,
};

const struct attribute_group *vts_class_groups[] = {
	&property_attr_group,
	&vts_attr_group,
	NULL,
};

static int vts_dev_suspend(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "suspended\n");
	vts_device_lock(vtsdev);
	return 0;
}

static int vts_dev_resume(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "resumed\n");
	vts_device_unlock(vtsdev);
	return 0;
}

struct dev_pm_ops vts_dev_pm = {
	.suspend = vts_dev_suspend,
	.resume = vts_dev_resume,
};

static int vts_class_init(void)
{
	int ret = 0;

	if(vts_class)
		return 0;

	vts_class = class_create(THIS_MODULE, "vts");
	if (IS_ERR_OR_NULL(vts_class)) {
		VTE("create class vts failed ret = %ld\n", PTR_ERR(vts_class));
		ret = PTR_ERR(vts_class);
		vts_class = NULL;
		return ret;
	}
	vts_class->dev_groups = vts_class_groups;
	vts_class->pm = &vts_dev_pm;
	return 0;
}

static void vts_class_exit(void)
{
	if (!vts_class)
		return ;

	class_destroy(vts_class);
	vts_class = NULL;
}

int vts_classdev_register(struct device *parent,
				struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ret;
	}

	if (atomic_inc_return(&nr_devices) == 1) {
		ret = vts_class_init();
		if (ret)
			return ret;
	}

	vtsdev->dev = device_create(vts_class, parent, val,
				      vtsdev, "%s", vts_name(vtsdev));
	if (IS_ERR(vtsdev->dev)) {
		vts_dev_err(vtsdev, "device create failed! ret = %ld\n", PTR_ERR(vtsdev->dev));
		if (atomic_dec_return(&nr_devices) == 0)
			vts_class_exit();
		return PTR_ERR(vtsdev->dev);
	}

	vts_dev_info(vtsdev, "class device registered\n");
	return 0;
}

void vts_classdev_unregister(struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ;
	}

	device_destroy(vts_class, val);
	vts_dev_info(vtsdev, "class device unregistered\n");
	if (atomic_dec_return(&nr_devices) == 0)
		vts_class_exit();
}

