// SPDX-License-Identifier: GPL-2.0+
/*
 * Platform driver for OneXPlayer, AOKZOE, AYANEO, and OrangePi Handhelds
 * that expose fan reading and control via hwmon sysfs.
 *
 * Old OXP boards have the same DMI strings and they are told apart by
 * the boot cpu vendor (Intel/AMD). Of these older models only AMD is
 * supported.
 *
 * Fan control is provided via pwm interface in the range [0-255].
 * Old AMD boards use [0-100] as range in the EC, the written value is
 * scaled to accommodate for that. Newer boards like the mini PRO and
 * AOKZOE are not scaled but have the same EC layout. Newer models
 * like the 2 and X1 are [0-184] and are scaled to 0-255. OrangePi
 * are [1-244] and scaled to 0-255.
 *
 * Copyright (C) 2022 Joaquín I. Aramendía <samsagax@gmail.com>
 * Copyright (C) 2024 Derek J. Clark <derekjohn.clark@gmail.com>
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/processor.h>

/* Handle ACPI lock mechanism */
static u32 oxp_mutex;

#define ACPI_LOCK_DELAY_MS	500

static bool lock_global_acpi_lock(void)
{
	return ACPI_SUCCESS(acpi_acquire_global_lock(ACPI_LOCK_DELAY_MS, &oxp_mutex));
}

static void unlock_global_acpi_lock(void)
{
	/*
	 * Ignore any errors when releasing the lock again. Rational is that we
	 * can't do anything about the probleme here anyway.
	 */
	acpi_release_global_lock(oxp_mutex);
}

/* Fan reading and PWM */
#define OXP_SENSOR_FAN_REG             0x76 /* Fan reading is 2 registers long */
#define OXP_2_SENSOR_FAN_REG           0x58 /* Fan reading is 2 registers long */
#define OXP_SENSOR_PWM_ENABLE_REG      0x4A /* PWM enable is 1 register long */
#define OXP_SENSOR_PWM_REG             0x4B /* PWM control is 1 register long */
#define PWM_MODE_AUTO                  0x00
#define PWM_MODE_MANUAL                0x01

/* OrangePi fan reading and PWM */
#define ORANGEPI_SENSOR_FAN_REG        0x78 /* Fan reading is 2 registers long */
#define ORANGEPI_SENSOR_PWM_ENABLE_REG 0x40 /* PWM enable is 1 register long */
#define ORANGEPI_SENSOR_PWM_REG        0x38 /* PWM control is 1 register long */

/* Turbo button takeover function
 * Different boards have different values and EC registers
 * for the same function
 */
#define OXP_TURBO_SWITCH_REG           0xF1 /* Mini Pro, OneXFly, AOKZOE */
#define OXP_2_TURBO_SWITCH_REG         0xEB /* OXP2 and X1 */
#define OXP_MINI_TURBO_SWITCH_REG      0x1E /* Mini AO7 */

#define OXP_MINI_TURBO_TAKE_VAL        0x01 /* Mini AO7 */
#define OXP_TURBO_TAKE_VAL             0x40 /* All other models */

#define OXP_TURBO_RETURN_VAL           0x00 /* Common return val */

enum oxp_feature_bits {
	OXP_FEATURE_SENSOR_FAN,
	OXP_FEATURE_PWM,
	OXP_FEATURE_TURBO,
	OXP_FEATURE_ONLY_AMD,
};

struct oxp_config {
	unsigned long features;

	u8 sensor_pwm_enable_reg;
	u8 sensor_pwm_reg;
	u8 sensor_pwm_scale[2];
	u8 sensor_fan_reg;

	u8 turbo_switch_reg;
	u8 turbo_enable_val;
};

struct oxp_data {
	struct device *hwmon_dev;
	const struct oxp_config *config;

	bool pwm_auto; /* Is the EC controlling the PWM automatically? */
};

static const struct oxp_config config_oxp = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM),

	.sensor_pwm_enable_reg = OXP_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = OXP_SENSOR_PWM_REG,
	.sensor_pwm_scale = {0, 100},
	.sensor_fan_reg = OXP_SENSOR_FAN_REG,
};

static const struct oxp_config config_mini_amd = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM) | BIT(OXP_FEATURE_ONLY_AMD),

	.sensor_pwm_enable_reg = OXP_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = OXP_SENSOR_PWM_REG,
	.sensor_pwm_scale = {0, 100},
	.sensor_fan_reg = OXP_SENSOR_FAN_REG,
};

static const struct oxp_config config_mini_amd_a07 = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM) | BIT(OXP_FEATURE_TURBO),

	.sensor_pwm_enable_reg = OXP_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = OXP_SENSOR_PWM_REG,
	.sensor_pwm_scale = {0, 100},
	.sensor_fan_reg = OXP_SENSOR_FAN_REG,

	.turbo_switch_reg = OXP_MINI_TURBO_SWITCH_REG,
	.turbo_enable_val = OXP_MINI_TURBO_TAKE_VAL,
};

static const struct oxp_config config_oxp2_turbo = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM) | BIT(OXP_FEATURE_TURBO),

	.sensor_pwm_enable_reg = OXP_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = OXP_SENSOR_PWM_REG,
	.sensor_pwm_scale = {0, 184},
	.sensor_fan_reg = OXP_2_SENSOR_FAN_REG,

	.turbo_switch_reg = OXP_2_TURBO_SWITCH_REG,
	.turbo_enable_val = OXP_TURBO_TAKE_VAL,
};

static const struct oxp_config config_orange = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM),

	.sensor_pwm_enable_reg = ORANGEPI_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = ORANGEPI_SENSOR_PWM_REG,
	.sensor_pwm_scale = {1, 244},
	.sensor_fan_reg = ORANGEPI_SENSOR_FAN_REG,
};

static const struct oxp_config config_aok_zoe = {
	.features = BIT(OXP_FEATURE_SENSOR_FAN) | BIT(OXP_FEATURE_PWM) | BIT(OXP_FEATURE_TURBO),

	.sensor_pwm_enable_reg = OXP_SENSOR_PWM_ENABLE_REG,
	.sensor_pwm_reg = OXP_SENSOR_PWM_REG,
	.sensor_pwm_scale = {0, 255},
	.sensor_fan_reg = OXP_SENSOR_FAN_REG,

	.turbo_switch_reg = OXP_TURBO_SWITCH_REG,
	.turbo_enable_val = OXP_TURBO_TAKE_VAL,
};

static const struct dmi_system_id dmi_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AOKZOE"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AOKZOE A1 AR07"),
		},
		.driver_data = (void *)&config_aok_zoe,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AOKZOE"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AOKZOE A1 Pro"),
		},
		.driver_data = (void *)&config_aok_zoe,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_MATCH(DMI_BOARD_NAME, "AYANEO 2"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AIR"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AIR 1S"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AB05-Mendocino"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "AIR Pro"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_MATCH(DMI_BOARD_NAME, "FLIP"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_MATCH(DMI_BOARD_NAME, "GEEK"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "KUN"),
		},
		.driver_data = (void *)&config_oxp,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "OrangePi"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "NEO-01"),
		},
		.driver_data = (void *)&config_orange,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "ONE XPLAYER"),
		},
		.driver_data = (void *)&config_mini_amd,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_MATCH(DMI_BOARD_NAME, "ONEXPLAYER 2"),
		},
		.driver_data = (void *)&config_oxp2_turbo,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "ONEXPLAYER F1"),
		},
		.driver_data = (void *)&config_aok_zoe,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "ONEXPLAYER mini A07"),
		},
		.driver_data = (void *)&config_mini_amd_a07,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "ONEXPLAYER Mini Pro"),
		},
		.driver_data = (void *)&config_aok_zoe,
	},
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "ONE-NETBOOK"),
			DMI_MATCH(DMI_BOARD_NAME, "ONEXPLAYER X1"),
		},
		.driver_data = (void *)&config_oxp2_turbo,
	},
	{},
};

/* Helper functions to handle EC read/write */
static int read_u8_from_ec(u8 reg, u8 *val)
{
	int ret;

	if (!lock_global_acpi_lock())
		return -EBUSY;

	ret = ec_read(reg, val);

	unlock_global_acpi_lock();

	return ret;
}

static int read_u16_from_ec(u8 reg, u16 *val)
{
	int ret;

	/* Swap the two bytes already while reading. */
	const u8 addr_buf[2] = {reg + 1, reg};

	if (!lock_global_acpi_lock())
		return -EBUSY;

	ret = ec_read_buffer(addr_buf, (u8 *)val, 2);

	unlock_global_acpi_lock();

	return ret;
}

static int write_to_ec(u8 reg, u8 value)
{
	int ret;

	if (!lock_global_acpi_lock())
		return -EBUSY;

	ret = ec_write(reg, value);

	unlock_global_acpi_lock();

	return ret;
}

static int pwm_auto_from_hw(struct oxp_data *data)
{
	const struct oxp_config *config = data->config;

	u8 tmp;
	int ret;

	ret = read_u8_from_ec(config->sensor_pwm_enable_reg, &tmp);
	if (ret < 0)
		return ret;

	data->pwm_auto = tmp == PWM_MODE_AUTO;

	return ret;
}

/* Rescale a (HW) sensor PWM value to userspace range. */
static long rescale_sensor_pwm_to_user(const struct oxp_config *config, long val)
{
	const long range = config->sensor_pwm_scale[1] - config->sensor_pwm_scale[0];

	if (range == 255)
		return val;

	return ((val - config->sensor_pwm_scale[0]) * 255) / range;
}

/* Rescale a (userspace) sensor PWM value to hw range. */
static long rescale_sensor_pwm_to_hw(const struct oxp_config *config, long val)
{
	const long range = config->sensor_pwm_scale[1] - config->sensor_pwm_scale[0];

	if (range == 255)
		return val;

	return config->sensor_pwm_scale[0] + (val * range) / 255;
}

/* Turbo button toggle functions */
static int tt_toggle_enable(const struct oxp_config *config)
{
	if (!test_bit(OXP_FEATURE_TURBO, &config->features))
		return -EINVAL;

	return write_to_ec(config->turbo_switch_reg, config->turbo_enable_val);
}

static int tt_toggle_disable(const struct oxp_config *config)
{
	if (!test_bit(OXP_FEATURE_TURBO, &config->features))
		return -EINVAL;

	return write_to_ec(config->turbo_switch_reg, OXP_TURBO_RETURN_VAL);
}

/* Callbacks for turbo toggle attribute */
static umode_t tt_toggle_is_visible(struct kobject *kobj,
				    struct attribute *attr, int n)
{
	return attr->mode;
}

static ssize_t tt_toggle_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct oxp_data *data = dev_get_drvdata(dev);
	const struct oxp_config *config = data->config;

	int rval;
	bool value;

	rval = kstrtobool(buf, &value);
	if (rval)
		return rval;

	if (value) {
		rval = tt_toggle_enable(config);
	} else {
		rval = tt_toggle_disable(config);
	}
	if (rval)
		return rval;

	return count;
}

static ssize_t tt_toggle_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct oxp_data *data = dev_get_drvdata(dev);
	const struct oxp_config *config = data->config;
	int retval;
	u8 val;

	if (!test_bit(OXP_FEATURE_TURBO, &config->features))
		return -EINVAL;

	retval = read_u8_from_ec(config->turbo_switch_reg, &val);
	if (retval)
		return retval;

	return sysfs_emit(buf, "%d\n", !!val);
}

static DEVICE_ATTR_RW(tt_toggle);

/* PWM enable/disable functions */
static int oxp_pwm_enable(struct oxp_data *data)
{
	const struct oxp_config *config;
	int ret;

	if (!data->pwm_auto)
		return 0;

	config = data->config;

	if (test_bit(OXP_FEATURE_PWM, &config->features)) {
		ret = write_to_ec(config->sensor_pwm_enable_reg, PWM_MODE_MANUAL);
		if (ret < 0)
			return ret;

		data->pwm_auto = false;

		return 0;
	}

	return -EINVAL;
}

static int oxp_pwm_disable(struct oxp_data *data)
{
	const struct oxp_config *config;
	int ret;

	if (data->pwm_auto)
		return 0;

	config = data->config;

	if (test_bit(OXP_FEATURE_PWM, &config->features)) {
		ret = write_to_ec(config->sensor_pwm_enable_reg, PWM_MODE_AUTO);
		if (ret < 0)
			return ret;

		data->pwm_auto = true;

		return 0;
	}

	return -EINVAL;
}

/* Callbacks for hwmon interface */
static umode_t oxp_ec_hwmon_is_visible(const void *drvdata,
				       enum hwmon_sensor_types type, u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return 0444;
	case hwmon_pwm:
		return 0644;
	default:
		return 0;
	}
}

static int oxp_platform_read(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long *val)
{
	struct oxp_data *data = dev_get_drvdata(dev);
	const struct oxp_config *config = data->config;

	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input && test_bit(OXP_FEATURE_SENSOR_FAN, &config->features)) {
			u16 tmp;
			int ret;

			ret = read_u16_from_ec(config->sensor_fan_reg, &tmp);
			*val = tmp;

			return ret;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			if (test_bit(OXP_FEATURE_PWM, &config->features)) {
				u8 tmp;
				int ret;

				ret = read_u8_from_ec(config->sensor_pwm_reg, &tmp);
				if (ret)
					return ret;

				*val = rescale_sensor_pwm_to_user(config, tmp);

				return 0;
			}
			break;
		case hwmon_pwm_enable:
			if (test_bit(OXP_FEATURE_PWM, &config->features)) {
				*val = data->pwm_auto ? PWM_MODE_AUTO : PWM_MODE_MANUAL;

				return 0;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int oxp_platform_write(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long val)
{
	struct oxp_data *data = dev_get_drvdata(dev);
	const struct oxp_config *config = data->config;

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_enable:
			if (val == 1)
				return oxp_pwm_enable(data);
			else if (val == 0)
				return oxp_pwm_disable(data);
			return -EINVAL;
		case hwmon_pwm_input:
			if (val < 0 || val > 255 || data->pwm_auto)
				return -EINVAL;
			if (test_bit(OXP_FEATURE_PWM, &config->features)) {
				const long hw_val = data->pwm_auto ? val : rescale_sensor_pwm_to_hw(config, val);

				return write_to_ec(config->sensor_pwm_reg, hw_val);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

/* Known sensors in the OXP EC controllers */
static const struct hwmon_channel_info * const oxp_platform_sensors[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL,
};

static struct attribute *oxp_ec_attrs[] = {
	&dev_attr_tt_toggle.attr,
	NULL
};

static struct attribute_group oxp_ec_attribute_group = {
	.is_visible = tt_toggle_is_visible,
	.attrs = oxp_ec_attrs,
};

static const struct attribute_group *oxp_ec_groups[] = {
	&oxp_ec_attribute_group,
	NULL
};

static const struct hwmon_ops oxp_ec_hwmon_ops = {
	.is_visible = oxp_ec_hwmon_is_visible,
	.read = oxp_platform_read,
	.write = oxp_platform_write,
};

static const struct hwmon_chip_info oxp_ec_chip_info = {
	.ops = &oxp_ec_hwmon_ops,
	.info = oxp_platform_sensors,
};

/* Initialization logic */
static int oxp_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oxp_data *data;

	const struct dmi_system_id *dmi_entry;
	const struct oxp_config *config;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dmi_entry = dmi_first_match(dmi_table);
	if (!dmi_entry)
		return -ENODEV;

	config = dmi_entry->driver_data;

	/*
	 * Have to check for AMD processor here because DMI strings are the same
	 * between Intel and AMD boards on older OneXPlayer devices, the only way
	 * to tell them apart is the CPU. Old Intel boards have an unsupported EC.
	 */
	if (test_bit(OXP_FEATURE_ONLY_AMD, &config->features) &&
	    boot_cpu_data.x86_vendor != X86_VENDOR_AMD)
		return -ENODEV;

	/* If we lack the turbo feature, don't expose the turbo toggle attribute. */
	if (!test_bit(OXP_FEATURE_TURBO, &config->features))
		dev_attr_tt_toggle.attr.mode = 0;

	data->config = config;
	data->hwmon_dev = devm_hwmon_device_register_with_info(dev, "oxpec", data,
							       &oxp_ec_chip_info, NULL);
	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	platform_set_drvdata(pdev, data);

	if (test_bit(OXP_FEATURE_PWM, &config->features)) {
		int ret;

		ret = pwm_auto_from_hw(data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct platform_driver oxp_platform_driver = {
	.driver = {
		.name = "oxp-platform",
		.dev_groups = oxp_ec_groups,
	},
	.probe = oxp_platform_probe,
};

static struct platform_device *oxp_platform_device;

static int __init oxp_platform_init(void)
{
	oxp_platform_device =
		platform_create_bundle(&oxp_platform_driver,
				       oxp_platform_probe, NULL, 0, NULL, 0);

	return PTR_ERR_OR_ZERO(oxp_platform_device);
}

static void __exit oxp_platform_exit(void)
{
	platform_device_unregister(oxp_platform_device);
	platform_driver_unregister(&oxp_platform_driver);
}

MODULE_DEVICE_TABLE(dmi, dmi_table);

module_init(oxp_platform_init);
module_exit(oxp_platform_exit);

MODULE_AUTHOR("Joaquín Ignacio Aramendía <samsagax@gmail.com>");
MODULE_DESCRIPTION("Platform driver that handles EC sensors of OneXPlayer devices");
MODULE_LICENSE("GPL");
