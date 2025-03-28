/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */
#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <linux/input.h>
#include <linux/workqueue.h>

//#ifdef CONFIG_BBK_FP_ID
#include "../fp_id.h"
//#endif

#define FPC_TTW_HOLD_TIME		1000
#define RESET_LOW_SLEEP_MIN_US 5000
#define RESET_LOW_SLEEP_MAX_US		(RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US	5000
#define RESET_HIGH_SLEEP1_MAX_US	(RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US	5000
#define RESET_HIGH_SLEEP2_MAX_US	(RESET_HIGH_SLEEP2_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US 2000
#define PWR_ON_SLEEP_MAX_US		(PWR_ON_SLEEP_MIN_US + 900)
#define NUM_PARAMS_REG_ENABLE_SET	2

#define RELEASE_WAKELOCK_W_V		"release_wakelock_with_verification"
#define RELEASE_WAKELOCK		"release_wakelock"
#define START_IRQS_RECEIVED_CNT		"start_irqs_received_counter"


static const char * const pctl_names[] = {
	"fingerprint_irq",
	"miso_spi",
	"miso_pullhigh",
	"miso_pulllow",
	"reset_high",
	"reset_low",
	"mosi_spi",
	"mosi_pullhigh",
	"mosi_pulllow",
	"cs_spi",
	"cs_pullhigh",
	"cs_pulllow",
	"clk_spi",
	"clk_pullhigh",
	"clk_pulllow",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config vreg_conf[] = {
	{ "vdd_ana_1p8", 1800000UL, 1800000UL, 8000, },
	{ "vdd_ana_3p0", 3000000UL, 3000000UL, 8000, },
	{ "vdd_ana_3p3", 3300000UL, 3300000UL, 8000, },
};

struct fpc1020_data {
	struct device *dev;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
#ifdef CONFIG_PM_WAKELOCKS
	struct wakeup_source *fp_wakelock;
	struct wakeup_source f_wakelock;
#else
	struct wake_lock *fp_wakelock;
	struct wake_lock f_wakelock;
#endif
	struct mutex lock; /* To set/get exported values in sysfs */
	int irq_gpio;
	int rst_gpio;

	int vdd_en_gpio;
	bool vdd_use_pmic;
	bool vdd_use_gpio;

	bool prepared;
	bool resource_requested;
	const char *regulator_name;
	atomic_t wakeup_enabled; /* Used both in ISR and non-ISR */
};

static uint8_t support_soft_fpid;

static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio);
static int hw_reset(struct  fpc1020_data *fpc1020);
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name);

static void fpc1020_set_spi_status(struct fpc1020_data *fpc1020, int enable)
{
	if (enable) {
		dev_info(fpc1020->dev, "fpc1020_set_spi_status enable\n");
		select_pin_ctl(fpc1020, "miso_spi");
		select_pin_ctl(fpc1020, "mosi_spi");
		select_pin_ctl(fpc1020, "cs_spi");
		select_pin_ctl(fpc1020, "clk_spi");
	} else {
		dev_info(fpc1020->dev, "fpc1020_set_spi_status disbale\n");
		select_pin_ctl(fpc1020, "miso_pulllow");
		select_pin_ctl(fpc1020, "mosi_pulllow");
		select_pin_ctl(fpc1020, "cs_pulllow");
		select_pin_ctl(fpc1020, "clk_pulllow");
	}
}


static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(vreg_conf); i++) {
		const char *n = vreg_conf[i].name;

		if (!memcmp(n, fpc1020->regulator_name, strlen(n)))
			goto found;
	}

	dev_err(dev, "Regulator %s not found\n", name);

	return -EINVAL;

found:
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = devm_regulator_get(dev, name);
			if (IS_ERR_OR_NULL(vreg)) {
				dev_err(dev, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			dev_info(dev, "voltage min:%d; max:%d\n", vreg_conf[i].vmin, vreg_conf[i].vmax);
			if (rc) {
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
				return rc;
			}
		}

		rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
		if (rc < 0) {
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
			return rc;
		}
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			vreg = NULL;
			fpc1020->vreg[i] = vreg;
			return rc;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				regulator_put(vreg);
				dev_info(dev, "disabled %s\n", name);
			}
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}

	return rc;
}


extern void vfp_spi_clk_enable(uint8_t bonoff);
static int set_clks(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
	printk("set_clks enable=%d\n", enable);

	if (enable) {
		vfp_spi_clk_enable(1);
		rc = 1;
	} else {
		vfp_spi_clk_enable(0);
		rc = 0;
	}

	return rc;
}

/*
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility. Only prints a debug print that it is
 * disabled.
 */
static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	return set_clks(fpc1020, (*buf == '1')) ? : count;
}
static DEVICE_ATTR(clk_enable, 0200, NULL, clk_enable_set);

/*
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */

static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;
	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		const char *n = pctl_names[i];

		if (!memcmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}

	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);

exit:
	return rc;
}

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);
	return rc ? rc : count;
}

static DEVICE_ATTR(pinctl_set, 0200, NULL, pinctl_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	int rc;

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(fpc1020->dev, "before %s: irq_gpio: %d\n", __func__, irq_gpio);

	rc = select_pin_ctl(fpc1020, "reset_high");
	if (rc)
		goto exit;

	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = select_pin_ctl(fpc1020, "reset_low");
	if (rc)
		goto exit;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "reset_high");
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(fpc1020->dev, "after %s: irq_gpio: %d\n", __func__, irq_gpio);

exit:

	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = -EINVAL;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!memcmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		rc = hw_reset(fpc1020);
		mutex_unlock(&fpc1020->lock);
	} else {
		return rc;
	}

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, 0200, NULL, hw_reset_set);

/*
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "reset_low");

		if (fpc1020->vdd_use_pmic) {
			dev_info(fpc1020->dev, "fpc1540 vreg_setup.\n");
			rc = vreg_setup(fpc1020, "vfp", true);
			if (rc)
				goto exit;
		}

		if (fpc1020->vdd_use_gpio) {
			rc = gpio_direction_output(fpc1020->vdd_en_gpio, 1);
			if (rc) {
				dev_err(fpc1020->dev, "fpc1540 power on fail.\n");
				goto exit;
			}
		}
		fpc1020_set_spi_status(fpc1020, 1);
		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);
		(void)select_pin_ctl(fpc1020, "reset_high");

	} else if (!enable && fpc1020->prepared) {
		rc = 0;

		fpc1020_set_spi_status(fpc1020, 0);

		(void)select_pin_ctl(fpc1020, "reset_low");

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

		if (fpc1020->vdd_use_pmic) {
			rc = vreg_setup(fpc1020, "vfp", false);
		}

		if (fpc1020->vdd_use_gpio) {
			(void)gpio_direction_output(fpc1020->vdd_en_gpio, 0);
		}
exit:
		fpc1020->prepared = false;
	}

	mutex_unlock(&fpc1020->lock);

	return rc;
}

/*
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!memcmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!memcmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
		return -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, 0200, NULL, device_prepare_set);

static int fpc_hw_get_power_state(struct fpc1020_data *fpc1020)
{
	/* TODO: LDO configure */
	static int retval;
	retval = fpc1020->prepared;
	return retval;
}

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;
	if (!strncmp(buf, "enable", strlen("enable"))) {
		rc = device_prepare(fpc1020, true);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		rc = device_prepare(fpc1020, false);
	} else {
		return -EINVAL;
	}
	dev_info(dev, "%s rc=%d\n", __func__, rc);
	return rc ? rc : count;
}
static ssize_t regulator_enable_get(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc;
    rc = fpc_hw_get_power_state(fpc1020);
	dev_info(dev, "%s rc=%d\n", __func__, rc);
    return scnprintf(buf, PAGE_SIZE, "%d\n", rc);

}
static DEVICE_ATTR(regulator_enable, S_IRUSR | S_IWUSR, regulator_enable_get, regulator_enable_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!memcmp(buf, "enable", strlen("enable")))
		atomic_set(&fpc1020->wakeup_enabled, 1);
	else if (!memcmp(buf, "disable", strlen("disable")))
		atomic_set(&fpc1020->wakeup_enabled, 0);
	else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(wakeup_enable, 0200, NULL, wakeup_enable_set);

static ssize_t request_resource_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = 0;
	size_t i;
	int irqf;

	mutex_lock(&fpc1020->lock);
	if (!memcmp(buf, "request", strlen("request"))) {
		dev_info(dev, "%s request\n", __func__);
		if (fpc1020->resource_requested) {
			dev_info(dev, "%s already requested!\n", __func__);
			rc = 0;
			goto exit;
		}

		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
		if (rc)
			goto exit;

		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_reset",
			&fpc1020->rst_gpio);
		if (rc)
			goto exit;

		fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
			if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
				dev_info(dev, "pinctrl not ready\n");
				rc = -EPROBE_DEFER;
				goto exit;
			}
			dev_err(dev, "Target does not use pinctrl\n");
			fpc1020->fingerprint_pinctrl = NULL;
			rc = -EINVAL;
			goto exit;
		}
		for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
			const char *n = pctl_names[i];
			struct pinctrl_state *state =
				pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
			if (IS_ERR(state)) {
				dev_err(dev, "cannot find '%s'\n", n);
				rc = -EINVAL;
				goto exit;
			}
			dev_info(dev, "found pin control %s\n", n);
			fpc1020->pinctrl_state[i] = state;
		}
		rc = select_pin_ctl(fpc1020, "reset_low");
		if (rc)
			goto exit;
		rc = select_pin_ctl(fpc1020, "fingerprint_irq");
		if (rc)
			goto exit;

		atomic_set(&fpc1020->wakeup_enabled, 0);

		irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
		if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
			//irqf |= IRQF_NO_SUSPEND;
			device_init_wakeup(dev, 1);
		}
		rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
		if (rc) {
			dev_err(dev, "could not request irq %d\n",
			gpio_to_irq(fpc1020->irq_gpio));
			goto exit;
		}
		dev_info(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));
		/* Request that the interrupt should be wakeable */
		enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
		fpc1020->resource_requested = true;
	} else if (!memcmp(buf, "release", strlen("release"))) {
		dev_info(dev, "%s release\n", __func__);
		if (fpc1020->resource_requested == false) {
			dev_info(dev, "%s already released!\n", __func__);
			rc = 0;
			goto exit;
		}
		devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
		if (gpio_is_valid(fpc1020->irq_gpio)) {
			devm_gpio_free(dev, fpc1020->irq_gpio);
			dev_info(dev, "remove irq_gpio success\n");
		}
		if (gpio_is_valid(fpc1020->rst_gpio)) {
			devm_gpio_free(dev, fpc1020->rst_gpio);
			dev_info(dev, "remove rst_gpio success\n");
		}
		devm_pinctrl_put(fpc1020->fingerprint_pinctrl);
		fpc1020->resource_requested = false;
	} else {
		rc = -EINVAL;
	}
exit:
	mutex_unlock(&fpc1020->lock);
	return rc ? rc : count;
}
static DEVICE_ATTR(request_resource, 0200, NULL, request_resource_set);


/*
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
}

/*
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(irq, 0600 | 0200, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_request_resource.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	pr_info("fpc1020 irq handler: %s\n", __func__);
	//mutex_lock(&fpc1020->lock);
#ifdef CONFIG_PM_WAKELOCKS
	__pm_wakeup_event(fpc1020->fp_wakelock, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
#else
	wake_lock_timeout(fpc1020->fp_wakelock, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
#endif
	//mutex_unlock(&fpc1020->lock);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	fpc1020->vdd_use_gpio = of_property_read_bool(fpc1020->dev->of_node, "fp,vdd_use_gpio");
	fpc1020->vdd_use_pmic = of_property_read_bool(fpc1020->dev->of_node, "fp,vdd_use_pmic");
	rc = of_property_read_string(fpc1020->dev->of_node, "fp,regulator_name", &fpc1020->regulator_name);
	if (rc) {
		printk("%s:fp,regulator_name property do not find\n", __func__);
		fpc1020->regulator_name = "vdd_ana_1p8";
	}
	printk("%s:fp,regulator_name %s\n", __func__, fpc1020->regulator_name);

	if (0 == support_soft_fpid) {
		size_t i;
		int irqf;
		if (fpc1020->vdd_use_gpio) {
		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_vdd_en",
				&fpc1020->vdd_en_gpio);
		if (rc)
			goto exit;
	}
		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
				&fpc1020->irq_gpio);
		if (rc)
			goto exit;

		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_reset",
				&fpc1020->rst_gpio);
		if (rc)
			goto exit;

		fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
			if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
				dev_info(dev, "pinctrl not ready\n");
				rc = -EPROBE_DEFER;
					goto exit;
			}
			dev_err(dev, "Target does not use pinctrl\n");
			fpc1020->fingerprint_pinctrl = NULL;
			rc = -EINVAL;
				goto exit;
		}

		for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
			const char *n = pctl_names[i];
			struct pinctrl_state *state =
				pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
			if (IS_ERR(state)) {
				dev_err(dev, "cannot find '%s'\n", n);
				rc = -EINVAL;
					goto exit;
			}
			dev_info(dev, "found pin control %s\n", n);
			fpc1020->pinctrl_state[i] = state;
		}
		rc = select_pin_ctl(fpc1020, "reset_low");
		if (rc)
			goto exit;
		rc = select_pin_ctl(fpc1020, "fingerprint_irq");
		if (rc)
			goto exit;

		atomic_set(&fpc1020->wakeup_enabled, 0);

		irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
		if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
			irqf |= IRQF_NO_SUSPEND;
			device_init_wakeup(dev, 1);
		}

		mutex_init(&fpc1020->lock);
		rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
				NULL, fpc1020_irq_handler, irqf,
				dev_name(dev), fpc1020);
		if (rc) {
			dev_err(dev, "could not request irq %d\n",
					gpio_to_irq(fpc1020->irq_gpio));
				goto exit;
		}

		dev_info(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
		#ifdef CONFIG_PM_WAKELOCKS

		#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
			fpc1020->fp_wakelock = wakeup_source_register(dev, "fpc_ttw_wl");
		#else
			fpc1020->fp_wakelock = &fpc1020->f_wakelock;
			wakeup_source_init(fpc1020->fp_wakelock, "fpc_ttw_wl");
		#endif

		#else
			fpc1020->fp_wakelock = &fpc1020->f_wakelock;
			wake_lock_init(fpc1020->fp_wakelock, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");
		#endif

		rc = sysfs_create_group(&dev->kobj, &attribute_group);
		if (rc) {
			dev_err(dev, "could not create sysfs\n");
				goto exit;
		}

	} else {
		fpc1020->resource_requested = false;
		mutex_init(&fpc1020->lock);
		#ifdef CONFIG_PM_WAKELOCKS

		#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
			fpc1020->fp_wakelock = wakeup_source_register(dev, "fpc_ttw_wl");
		#else
			fpc1020->fp_wakelock = &fpc1020->f_wakelock;
			wakeup_source_init(fpc1020->fp_wakelock, "fpc_ttw_wl");
		#endif

		#else
			fpc1020->fp_wakelock = &fpc1020->f_wakelock;
			wake_lock_init(fpc1020->fp_wakelock, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");
		#endif

		rc = sysfs_create_group(&dev->kobj, &attribute_group);
		if (rc) {
			dev_err(dev, "could not create sysfs\n");
				goto exit;
		}
	}
	dev_info(dev, "%s: ok\n", __func__);

exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
#ifdef CONFIG_PM_WAKELOCKS

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	wakeup_source_unregister(fpc1020->fp_wakelock);
#else
	wakeup_source_trash(fpc1020->fp_wakelock);
#endif

#else
	wake_lock_destroy(fpc1020->fp_wakelock);
#endif

	if (fpc1020->vdd_use_pmic) {
		(void)vreg_setup(fpc1020, "vfp", false);
	}

	if (fpc1020->vdd_use_gpio) {
		(void)gpio_direction_output(fpc1020->vdd_en_gpio, 0);
	}

	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "mediatek,fpc-sidefp", },
	{},
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);
static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1552",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};
extern unsigned int is_atboot;
static int __init fpc1020_init(void)
{
	if (is_atboot == 1) {
		printk("%s:in AT mode, not load gf driver!\n", __func__);
		return 0;
	}

	support_soft_fpid = is_support_soft_fpid();
	if (0 == support_soft_fpid) {
		if (get_fp_id() != FPC_FPC1540 && get_fp_id() != FPC_FPC1511) {
			printk("%s:wrong fp id, not fpc15xx driver!\n", __func__);
			return 0;
		}
	}

	return platform_driver_register(&fpc1020_driver);
}
late_initcall(fpc1020_init);

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}
module_exit(fpc1020_exit);


MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
MODULE_LICENSE("GPL v2");
