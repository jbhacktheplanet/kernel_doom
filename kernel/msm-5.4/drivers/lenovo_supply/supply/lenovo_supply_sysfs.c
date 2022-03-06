// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Sysfs interface for the universal power supply monitor class
 *
 *  Copyright © 2007  David Woodhouse <dwmw2@infradead.org>
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/lenovo_supply.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include "lenovo_supply.h"

#define MAX_PROP_NAME_LEN 30

struct lenovo_supply_attr {
	const char *prop_name;
	char attr_name[MAX_PROP_NAME_LEN + 1];
	struct device_attribute dev_attr;
	const char * const *text_values;
	int text_values_len;
};

#define _LENOVO_SUPPLY_ATTR(_name, _text, _len)	\
[LENOVO_SUPPLY_PROP_ ## _name] =			\
{						\
	.prop_name = #_name,			\
	.attr_name = #_name "\0",		\
	.text_values = _text,			\
	.text_values_len = _len,		\
}

#define LENOVO_SUPPLY_ATTR(_name) _LENOVO_SUPPLY_ATTR(_name, NULL, 0)
#define _LENOVO_SUPPLY_ENUM_ATTR(_name, _text)	\
	_LENOVO_SUPPLY_ATTR(_name, _text, ARRAY_SIZE(_text))
#define LENOVO_SUPPLY_ENUM_ATTR(_name)	\
	_LENOVO_SUPPLY_ENUM_ATTR(_name, LENOVO_SUPPLY_ ## _name ## _TEXT)

static const char * const LENOVO_SUPPLY_TYPE_TEXT[] = {
	[LENOVO_SUPPLY_TYPE_UNKNOWN]		= "Unknown",
	[LENOVO_SUPPLY_TYPE_BATTERY]		= "Battery",
	[LENOVO_SUPPLY_TYPE_UPS]			= "UPS",
	[LENOVO_SUPPLY_TYPE_MAINS]		= "Mains",
	[LENOVO_SUPPLY_TYPE_SC]		= "SC",
	[LENOVO_SUPPLY_TYPE_USB]			= "USB",
	[LENOVO_SUPPLY_TYPE_USB_DCP]		= "USB_DCP",
	[LENOVO_SUPPLY_TYPE_USB_CDP]		= "USB_CDP",
	[LENOVO_SUPPLY_TYPE_USB_ACA]		= "USB_ACA",
	[LENOVO_SUPPLY_TYPE_USB_TYPE_C]		= "USB_C",
	[LENOVO_SUPPLY_TYPE_USB_PD]		= "USB_PD",
	[LENOVO_SUPPLY_TYPE_USB_PD_DRP]		= "USB_PD_DRP",
	[LENOVO_SUPPLY_TYPE_APPLE_BRICK_ID]	= "BrickID",
	[LENOVO_SUPPLY_TYPE_WIRELESS]		= "Wireless",
};

static const char * const LENOVO_SUPPLY_USB_TYPE_TEXT[] = {
	[LENOVO_SUPPLY_USB_TYPE_UNKNOWN]		= "Unknown",
	[LENOVO_SUPPLY_USB_TYPE_SDP]		= "SDP",
	[LENOVO_SUPPLY_USB_TYPE_DCP]		= "DCP",
	[LENOVO_SUPPLY_USB_TYPE_CDP]		= "CDP",
	[LENOVO_SUPPLY_USB_TYPE_ACA]		= "ACA",
	[LENOVO_SUPPLY_USB_TYPE_C]		= "C",
	[LENOVO_SUPPLY_USB_TYPE_PD]		= "PD",
	[LENOVO_SUPPLY_USB_TYPE_PD_DRP]		= "PD_DRP",
	[LENOVO_SUPPLY_USB_TYPE_PD_PPS]		= "PD_PPS",
	[LENOVO_SUPPLY_USB_TYPE_APPLE_BRICK_ID]	= "BrickID",
};

static const char * const LENOVO_SUPPLY_STATUS_TEXT[] = {
	[LENOVO_SUPPLY_STATUS_UNKNOWN]		= "Unknown",
	[LENOVO_SUPPLY_STATUS_CHARGING]		= "Charging",
	[LENOVO_SUPPLY_STATUS_DISCHARGING]	= "Discharging",
	[LENOVO_SUPPLY_STATUS_NOT_CHARGING]	= "Not charging",
	[LENOVO_SUPPLY_STATUS_FULL]		= "Full",
};

static const char * const LENOVO_SUPPLY_CHARGE_TYPE_TEXT[] = {
	[LENOVO_SUPPLY_CHARGE_TYPE_UNKNOWN]	= "Unknown",
	[LENOVO_SUPPLY_CHARGE_TYPE_NONE]		= "N/A",
	[LENOVO_SUPPLY_CHARGE_TYPE_TRICKLE]	= "Trickle",
	[LENOVO_SUPPLY_CHARGE_TYPE_FAST]		= "Fast",
	[LENOVO_SUPPLY_CHARGE_TYPE_STANDARD]	= "Standard",
	[LENOVO_SUPPLY_CHARGE_TYPE_ADAPTIVE]	= "Adaptive",
	[LENOVO_SUPPLY_CHARGE_TYPE_CUSTOM]	= "Custom",
};

static const char * const LENOVO_SUPPLY_HEALTH_TEXT[] = {
	[LENOVO_SUPPLY_HEALTH_UNKNOWN]		    = "Unknown",
	[LENOVO_SUPPLY_HEALTH_GOOD]		    = "Good",
	[LENOVO_SUPPLY_HEALTH_OVERHEAT]		    = "Overheat",
	[LENOVO_SUPPLY_HEALTH_DEAD]		    = "Dead",
	[LENOVO_SUPPLY_HEALTH_OVERVOLTAGE]	    = "Over voltage",
	[LENOVO_SUPPLY_HEALTH_UNSPEC_FAILURE]	    = "Unspecified failure",
	[LENOVO_SUPPLY_HEALTH_COLD]		    = "Cold",
	[LENOVO_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE] = "Watchdog timer expire",
	[LENOVO_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE]   = "Safety timer expire",
	[LENOVO_SUPPLY_HEALTH_OVERCURRENT]	    = "Over current",
	[LENOVO_SUPPLY_HEALTH_CALIBRATION_REQUIRED]  = "Calibration required",
	[LENOVO_SUPPLY_HEALTH_WARM]		    = "Warm",
	[LENOVO_SUPPLY_HEALTH_COOL]		    = "Cool",
	[LENOVO_SUPPLY_HEALTH_HOT]		    = "Hot",
};

static const char * const LENOVO_SUPPLY_TECHNOLOGY_TEXT[] = {
	[LENOVO_SUPPLY_TECHNOLOGY_UNKNOWN]	= "Unknown",
	[LENOVO_SUPPLY_TECHNOLOGY_NiMH]		= "NiMH",
	[LENOVO_SUPPLY_TECHNOLOGY_LION]		= "Li-ion",
	[LENOVO_SUPPLY_TECHNOLOGY_LIPO]		= "Li-poly",
	[LENOVO_SUPPLY_TECHNOLOGY_LiFe]		= "LiFe",
	[LENOVO_SUPPLY_TECHNOLOGY_NiCd]		= "NiCd",
	[LENOVO_SUPPLY_TECHNOLOGY_LiMn]		= "LiMn",
};

static const char * const LENOVO_SUPPLY_CAPACITY_LEVEL_TEXT[] = {
	[LENOVO_SUPPLY_CAPACITY_LEVEL_UNKNOWN]	= "Unknown",
	[LENOVO_SUPPLY_CAPACITY_LEVEL_CRITICAL]	= "Critical",
	[LENOVO_SUPPLY_CAPACITY_LEVEL_LOW]	= "Low",
	[LENOVO_SUPPLY_CAPACITY_LEVEL_NORMAL]	= "Normal",
	[LENOVO_SUPPLY_CAPACITY_LEVEL_HIGH]	= "High",
	[LENOVO_SUPPLY_CAPACITY_LEVEL_FULL]	= "Full",
};

static const char * const LENOVO_SUPPLY_SCOPE_TEXT[] = {
	[LENOVO_SUPPLY_SCOPE_UNKNOWN]	= "Unknown",
	[LENOVO_SUPPLY_SCOPE_SYSTEM]	= "System",
	[LENOVO_SUPPLY_SCOPE_DEVICE]	= "Device",
};

static struct lenovo_supply_attr lenovo_supply_attrs[] = {
	/* Properties of type `int' */
	LENOVO_SUPPLY_ENUM_ATTR(STATUS),
	LENOVO_SUPPLY_ENUM_ATTR(CHARGE_TYPE),
	LENOVO_SUPPLY_ENUM_ATTR(HEALTH),
	LENOVO_SUPPLY_ATTR(PRESENT),
	LENOVO_SUPPLY_ATTR(ONLINE),
	LENOVO_SUPPLY_ATTR(AUTHENTIC),
	LENOVO_SUPPLY_ENUM_ATTR(TECHNOLOGY),
	LENOVO_SUPPLY_ATTR(CYCLE_COUNT),
	LENOVO_SUPPLY_ATTR(VOLTAGE_MAX),
	LENOVO_SUPPLY_ATTR(VOLTAGE_MIN),
	LENOVO_SUPPLY_ATTR(VOLTAGE_MAX_DESIGN),
	LENOVO_SUPPLY_ATTR(VOLTAGE_MIN_DESIGN),
	LENOVO_SUPPLY_ATTR(VOLTAGE_NOW),
	LENOVO_SUPPLY_ATTR(VOLTAGE_AVG),
	LENOVO_SUPPLY_ATTR(VOLTAGE_OCV),
	LENOVO_SUPPLY_ATTR(VOLTAGE_BOOT),
	LENOVO_SUPPLY_ATTR(CURRENT_MAX),
	LENOVO_SUPPLY_ATTR(CURRENT_NOW),
	LENOVO_SUPPLY_ATTR(CURRENT_AVG),
	LENOVO_SUPPLY_ATTR(CURRENT_BOOT),
	LENOVO_SUPPLY_ATTR(POWER_NOW),
	LENOVO_SUPPLY_ATTR(POWER_AVG),
	LENOVO_SUPPLY_ATTR(CHARGE_FULL_DESIGN),
	LENOVO_SUPPLY_ATTR(CHARGE_EMPTY_DESIGN),
	LENOVO_SUPPLY_ATTR(CHARGE_FULL),
	LENOVO_SUPPLY_ATTR(CHARGE_EMPTY),
	LENOVO_SUPPLY_ATTR(CHARGE_NOW),
	LENOVO_SUPPLY_ATTR(CHARGE_AVG),
	LENOVO_SUPPLY_ATTR(CHARGE_COUNTER),
	LENOVO_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT),
	LENOVO_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT_MAX),
	LENOVO_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE),
	LENOVO_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE_MAX),
	LENOVO_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT),
	LENOVO_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT_MAX),
	LENOVO_SUPPLY_ATTR(CHARGE_CONTROL_START_THRESHOLD),
	LENOVO_SUPPLY_ATTR(CHARGE_CONTROL_END_THRESHOLD),
	LENOVO_SUPPLY_ATTR(INPUT_CURRENT_LIMIT),
	LENOVO_SUPPLY_ATTR(INPUT_VOLTAGE_LIMIT),
	LENOVO_SUPPLY_ATTR(INPUT_POWER_LIMIT),
	LENOVO_SUPPLY_ATTR(ENERGY_FULL_DESIGN),
	LENOVO_SUPPLY_ATTR(ENERGY_EMPTY_DESIGN),
	LENOVO_SUPPLY_ATTR(ENERGY_FULL),
	LENOVO_SUPPLY_ATTR(ENERGY_EMPTY),
	LENOVO_SUPPLY_ATTR(ENERGY_NOW),
	LENOVO_SUPPLY_ATTR(ENERGY_AVG),
	LENOVO_SUPPLY_ATTR(CAPACITY),
	LENOVO_SUPPLY_ATTR(CAPACITY_ALERT_MIN),
	LENOVO_SUPPLY_ATTR(CAPACITY_ALERT_MAX),
	LENOVO_SUPPLY_ATTR(CAPACITY_ERROR_MARGIN),
	LENOVO_SUPPLY_ENUM_ATTR(CAPACITY_LEVEL),
	LENOVO_SUPPLY_ATTR(TEMP),
	LENOVO_SUPPLY_ATTR(TEMP_MAX),
	LENOVO_SUPPLY_ATTR(TEMP_MIN),
	LENOVO_SUPPLY_ATTR(TEMP_ALERT_MIN),
	LENOVO_SUPPLY_ATTR(TEMP_ALERT_MAX),
	LENOVO_SUPPLY_ATTR(TEMP_AMBIENT),
	LENOVO_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MIN),
	LENOVO_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MAX),
	LENOVO_SUPPLY_ATTR(TIME_TO_EMPTY_NOW),
	LENOVO_SUPPLY_ATTR(TIME_TO_EMPTY_AVG),
	LENOVO_SUPPLY_ATTR(TIME_TO_FULL_NOW),
	LENOVO_SUPPLY_ATTR(TIME_TO_FULL_AVG),
	LENOVO_SUPPLY_ENUM_ATTR(TYPE),
	LENOVO_SUPPLY_ATTR(USB_TYPE),
	LENOVO_SUPPLY_ENUM_ATTR(SCOPE),
	LENOVO_SUPPLY_ATTR(PRECHARGE_CURRENT),
	LENOVO_SUPPLY_ATTR(CHARGE_TERM_CURRENT),
	LENOVO_SUPPLY_ATTR(CALIBRATE),
	LENOVO_SUPPLY_ATTR(MANUFACTURE_YEAR),
	LENOVO_SUPPLY_ATTR(MANUFACTURE_MONTH),
	LENOVO_SUPPLY_ATTR(MANUFACTURE_DAY),
	/* Properties of type `const char *' */
	LENOVO_SUPPLY_ATTR(MODEL_NAME),
	LENOVO_SUPPLY_ATTR(MANUFACTURER),
	LENOVO_SUPPLY_ATTR(SERIAL_NUMBER),
#ifdef CONFIG_PRODUCT_DOOM
	LENOVO_SUPPLY_ATTR(INPUT_CURRENT_NOW),
	LENOVO_SUPPLY_ATTR(BATTERY_CHARGING_ENABLED),
	LENOVO_SUPPLY_ATTR(CHARGING_ENABLED),
	LENOVO_SUPPLY_ATTR(INPUT_SUSPEND),
	LENOVO_SUPPLY_ATTR(SC_BATTERY_PRESENT),
	LENOVO_SUPPLY_ATTR(SC_VBUS_PRESENT),
	LENOVO_SUPPLY_ATTR(SC_BATTERY_VOLTAGE),
	LENOVO_SUPPLY_ATTR(SC_BATTERY_CURRENT),
	LENOVO_SUPPLY_ATTR(SC_BATTERY_TEMPERATURE),
	LENOVO_SUPPLY_ATTR(SC_BUS_VOLTAGE),
	LENOVO_SUPPLY_ATTR(SC_BUS_CURRENT),
	LENOVO_SUPPLY_ATTR(SC_BUS_TEMPERATURE),
	LENOVO_SUPPLY_ATTR(SC_DIE_TEMPERATURE),
	LENOVO_SUPPLY_ATTR(SC_ALARM_STATUS),
	LENOVO_SUPPLY_ATTR(SC_VBUS_ERROR_STATUS),
	LENOVO_SUPPLY_ATTR(SC_FAULT_STATUS),
	LENOVO_SUPPLY_ATTR(TI_DIE_TEMPERATURE),
	LENOVO_SUPPLY_ATTR(SOH),
	LENOVO_SUPPLY_ATTR(CAPACITY_DECIMAL),
	LENOVO_SUPPLY_ATTR(CHIP_FW),
	LENOVO_SUPPLY_ATTR(FG1_VOLTAGE_NOW),
	LENOVO_SUPPLY_ATTR(FG1_CURRENT_NOW),
	LENOVO_SUPPLY_ATTR(FG1_SOH),
	LENOVO_SUPPLY_ATTR(FG1_TI_DIE_TEMPERATURE),
	LENOVO_SUPPLY_ATTR(FG1_CHARGE_FULL),
	LENOVO_SUPPLY_ATTR(FG1_FULL_DESIGN),
	LENOVO_SUPPLY_ATTR(FG1_CHARGE_COUNTER),
	LENOVO_SUPPLY_ATTR(FG1_CYCLE_COUNT),
	LENOVO_SUPPLY_ATTR(FG1_TIME_TO_EMPTY_NOW),
	LENOVO_SUPPLY_ATTR(FG1_TIME_TO_FULL_NOW),
	LENOVO_SUPPLY_ATTR(FG1_CHIP_FW),
	LENOVO_SUPPLY_ATTR(ADSP_CHG_ENABLE),
	LENOVO_SUPPLY_ATTR(TYPEC_CC_ORIENTATION),
	LENOVO_SUPPLY_ATTR(ADSP_CHG_PPS_ST),
	LENOVO_SUPPLY_ATTR(HLOS_CHG_ONLINE),
	LENOVO_SUPPLY_ATTR(HLOS_CHG_PPS_ST),
	LENOVO_SUPPLY_ATTR(REAL_TYPE),
	LENOVO_SUPPLY_ATTR(SEC_BATTERY_VOLTAGE),
	LENOVO_SUPPLY_ATTR(SEC_SYSTEM_VOLTAGE),
	LENOVO_SUPPLY_ATTR(SEC_BUS_VOLTAGE),
	LENOVO_SUPPLY_ATTR(SEC_BATTERY_CURRENT),
	LENOVO_SUPPLY_ATTR(SEC_ICL_LIMIT),
	LENOVO_SUPPLY_ATTR(SEC_FCC_LIMIT),
	LENOVO_SUPPLY_ATTR(SEC_ICL_NOW),
	LENOVO_SUPPLY_ATTR(ADSP_USB_SUSPEND_ENABLE),
	LENOVO_SUPPLY_ATTR(HLOS_GET_CP_STEP),
	LENOVO_SUPPLY_ATTR(PROTECT_DATA),
#ifdef SUPPORT_USER_THERMAL_CASE
	LENOVO_SUPPLY_ATTR(THERM_DISPLAY_RATE_LEVEL),
	LENOVO_SUPPLY_ATTR(THERM_DISPLAY_RATE_LEVEL_MAX),
	LENOVO_SUPPLY_ATTR(THERM_DISPLAY_RATE),
	LENOVO_SUPPLY_ATTR(THERM_DISPLAY_RATE_LIMIT),
	LENOVO_SUPPLY_ATTR(THERM_SPEAKER_LEVEL),
	LENOVO_SUPPLY_ATTR(THERM_SPEAKER_LEVEL_MAX),
	LENOVO_SUPPLY_ATTR(THERM_SPEAKER),
	LENOVO_SUPPLY_ATTR(THERM_SPEAKER_LIMIT),
	LENOVO_SUPPLY_ATTR(THERM_MODEM_5G_LEVEL),
	LENOVO_SUPPLY_ATTR(THERM_MODEM_5G_LEVEL_MAX),
	LENOVO_SUPPLY_ATTR(THERM_MODEM_5G),
	LENOVO_SUPPLY_ATTR(THERM_MODEM_5G_LIMIT),
	LENOVO_SUPPLY_ATTR(THERM_CAMERA_LEVEL),
	LENOVO_SUPPLY_ATTR(THERM_CAMERA_LEVEL_MAX),
	LENOVO_SUPPLY_ATTR(THERM_CAMERA),
	LENOVO_SUPPLY_ATTR(THERM_CAMERA_LIMIT),
#endif
	LENOVO_SUPPLY_ATTR(ADSP_FCC_VALUE),
	LENOVO_SUPPLY_ATTR(USER_HEALTH_CHARGE),
	LENOVO_SUPPLY_ATTR(USER_CHARGING_ENABLED),
	LENOVO_SUPPLY_ATTR(USER_INPUT_SUSPEND),
	LENOVO_SUPPLY_ATTR(ADSP_ICL_VALUE),
	LENOVO_SUPPLY_ATTR(ADSP_REAL_TYPE),
	LENOVO_SUPPLY_ATTR(ADSP_CP_THERMAL_CTR),
	LENOVO_SUPPLY_ATTR(FAC_TEST_ENABLE),
	LENOVO_SUPPLY_ATTR(FIXPDO1_CURR),
	LENOVO_SUPPLY_ATTR(FIXPDO2_CURR),
	LENOVO_SUPPLY_ATTR(FIXPDO2_MAX_POWER),
	LENOVO_SUPPLY_ATTR(TYPEC_ORIENT),
	LENOVO_SUPPLY_ATTR(SET_FV),
	LENOVO_SUPPLY_ATTR(CHARGE_THERMAL_STATUS),
	LENOVO_SUPPLY_ATTR(THERM_EVENT_LEVEL),
	LENOVO_SUPPLY_ATTR(THERM_EVENT_LEVEL_MAX),
	LENOVO_SUPPLY_ATTR(THERM_EVENT),
	LENOVO_SUPPLY_ATTR(THERM_EVENT_LIMIT),
	LENOVO_SUPPLY_ATTR(TYPEC_CONNECT_ST),
#endif
};

static struct attribute *
__lenovo_supply_attrs[ARRAY_SIZE(lenovo_supply_attrs) + 1];

static struct lenovo_supply_attr *to_ps_attr(struct device_attribute *attr)
{
	return container_of(attr, struct lenovo_supply_attr, dev_attr);
}

static enum lenovo_supply_property dev_attr_psp(struct device_attribute *attr)
{
	return  to_ps_attr(attr) - lenovo_supply_attrs;
}

static ssize_t lenovo_supply_show_usb_type(struct device *dev,
					  const struct lenovo_supply_desc *desc,
					  union lenovo_supply_propval *value,
					  char *buf)
{
	enum lenovo_supply_usb_type usb_type;
	ssize_t count = 0;
	bool match = false;
	int i;

	for (i = 0; i < desc->num_usb_types; ++i) {
		usb_type = desc->usb_types[i];

		if (value->intval == usb_type) {
			count += sprintf(buf + count, "[%s] ",
					 LENOVO_SUPPLY_USB_TYPE_TEXT[usb_type]);
			match = true;
		} else {
			count += sprintf(buf + count, "%s ",
					 LENOVO_SUPPLY_USB_TYPE_TEXT[usb_type]);
		}
	}

	if (!match) {
		dev_warn(dev, "driver reporting unsupported connected type\n");
		return -EINVAL;
	}

	if (count)
		buf[count - 1] = '\n';

	return count;
}

#ifdef CONFIG_PRODUCT_DOOM
static ssize_t lenovo_supply_show_real_type(struct device *dev,
					  const struct lenovo_supply_desc *desc,
					  union lenovo_supply_propval *value,
					  char *buf)
{
	ssize_t count = 0;

	count = sprintf(buf, "%s", LENOVO_SUPPLY_USB_TYPE_TEXT[value->intval]);

	return count;
}
#endif

static ssize_t lenovo_supply_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf) {
	ssize_t ret;
	struct lenovo_supply *psy = dev_get_drvdata(dev);
	struct lenovo_supply_attr *ps_attr = to_ps_attr(attr);
	enum lenovo_supply_property psp = dev_attr_psp(attr);
	union lenovo_supply_propval value;

	if (psp == LENOVO_SUPPLY_PROP_TYPE) {
		value.intval = psy->desc->type;
	} else {
		ret = lenovo_supply_get_property(psy, psp, &value);

		if (ret < 0) {
			if (ret == -ENODATA)
				dev_dbg(dev, "driver has no data for `%s' property\n",
					attr->attr.name);
			else if (ret != -ENODEV && ret != -EAGAIN)
				dev_err_ratelimited(dev,
					"driver failed to report `%s' property: %zd\n",
					attr->attr.name, ret);
			return ret;
		}
	}

	if (ps_attr->text_values_len > 0 &&
	    value.intval < ps_attr->text_values_len && value.intval >= 0) {
		return sprintf(buf, "%s\n", ps_attr->text_values[value.intval]);
	}

	switch (psp) {
	case LENOVO_SUPPLY_PROP_USB_TYPE:
		ret = lenovo_supply_show_usb_type(dev, psy->desc,
						&value, buf);
		break;
	case LENOVO_SUPPLY_PROP_MODEL_NAME ... LENOVO_SUPPLY_PROP_SERIAL_NUMBER:
	case LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE:
		ret = sprintf(buf, "%s\n", value.strval);
		break;
#ifdef CONFIG_PRODUCT_DOOM
	case LENOVO_SUPPLY_PROP_REAL_TYPE:
		ret = lenovo_supply_show_real_type(dev, psy->desc,
						&value, buf);
		break;
#endif
	default:
		ret = sprintf(buf, "%d\n", value.intval);
	}

	return ret;
}

static ssize_t lenovo_supply_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count) {
	ssize_t ret;
	struct lenovo_supply *psy = dev_get_drvdata(dev);
	struct lenovo_supply_attr *ps_attr = to_ps_attr(attr);
	enum lenovo_supply_property psp = dev_attr_psp(attr);
	union lenovo_supply_propval value;

	ret = -EINVAL;
	if (ps_attr->text_values_len > 0) {
		ret = __sysfs_match_string(ps_attr->text_values,
					   ps_attr->text_values_len, buf);
	}

	/*
	 * If no match was found, then check to see if it is an integer.
	 * Integer values are valid for enums in addition to the text value.
	 */
	if (ret < 0) {
		long long_val;

		ret = kstrtol(buf, 10, &long_val);
		if (ret < 0)
			return ret;

		ret = long_val;
	}

	value.intval = ret;

	ret = lenovo_supply_set_property(psy, psp, &value);
	if (ret < 0)
		return ret;

	return count;
}

static umode_t lenovo_supply_attr_is_visible(struct kobject *kobj,
					   struct attribute *attr,
					   int attrno)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lenovo_supply *psy = dev_get_drvdata(dev);
	umode_t mode = S_IRUSR | S_IRGRP | S_IROTH;
	int i;

	if (!lenovo_supply_attrs[attrno].prop_name)
		return 0;

	if (attrno == LENOVO_SUPPLY_PROP_TYPE)
		return mode;

	for (i = 0; i < psy->desc->num_properties; i++) {
		int property = psy->desc->properties[i];

		if (property == attrno) {
			if (psy->desc->property_is_writeable &&
			    psy->desc->property_is_writeable(psy, property) > 0)
				mode |= S_IWUSR;

			return mode;
		}
	}

	return 0;
}

static struct attribute_group lenovo_supply_attr_group = {
	.attrs = __lenovo_supply_attrs,
	.is_visible = lenovo_supply_attr_is_visible,
};

static const struct attribute_group *lenovo_supply_attr_groups[] = {
	&lenovo_supply_attr_group,
	NULL,
};

static void str_to_lower(char *str)
{
	while (*str) {
		*str = tolower(*str);
		str++;
	}
}

void lenovo_supply_init_attrs(struct device_type *dev_type)
{
	int i;

	dev_type->groups = lenovo_supply_attr_groups;

	for (i = 0; i < ARRAY_SIZE(lenovo_supply_attrs); i++) {
		struct device_attribute *attr;

		if (!lenovo_supply_attrs[i].prop_name) {
			pr_warn("%s: Property %d skipped because is is missing from lenovo_supply_attrs\n",
				__func__, i);
			sprintf(lenovo_supply_attrs[i].attr_name, "_err_%d", i);
		} else {
			str_to_lower(lenovo_supply_attrs[i].attr_name);
		}

		attr = &lenovo_supply_attrs[i].dev_attr;

		attr->attr.name = lenovo_supply_attrs[i].attr_name;
		attr->show = lenovo_supply_show_property;
		attr->store = lenovo_supply_store_property;
		__lenovo_supply_attrs[i] = &attr->attr;
	}
}

static int add_prop_uevent(struct device *dev, struct kobj_uevent_env *env,
			   enum lenovo_supply_property prop, char *prop_buf)
{
	int ret = 0;
	struct lenovo_supply_attr *pwr_attr;
	struct device_attribute *dev_attr;
	char *line;

	pwr_attr = &lenovo_supply_attrs[prop];
	dev_attr = &pwr_attr->dev_attr;

	ret = lenovo_supply_show_property(dev, dev_attr, prop_buf);
	if (ret == -ENODEV || ret == -ENODATA) {
		/*
		 * When a battery is absent, we expect -ENODEV. Don't abort;
		 * send the uevent with at least the the PRESENT=0 property
		 */
		return 0;
	}

	if (ret < 0)
		return ret;

	line = strchr(prop_buf, '\n');
	if (line)
		*line = 0;

	return add_uevent_var(env, "LENOVO_SUPPLY_%s=%s",
			      pwr_attr->prop_name, prop_buf);
}

int lenovo_supply_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct lenovo_supply *psy = dev_get_drvdata(dev);
	int ret = 0, j;
	char *prop_buf;

	if (!psy || !psy->desc) {
		dev_dbg(dev, "No power supply yet\n");
		return ret;
	}

	ret = add_uevent_var(env, "LENOVO_SUPPLY_NAME=%s", psy->desc->name);
	if (ret)
		return ret;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return -ENOMEM;

	ret = add_prop_uevent(dev, env, LENOVO_SUPPLY_PROP_TYPE, prop_buf);
	if (ret)
		goto out;

	for (j = 0; j < psy->desc->num_properties; j++) {
		ret = add_prop_uevent(dev, env, psy->desc->properties[j],
				      prop_buf);
		if (ret)
			goto out;
	}

out:
	free_page((unsigned long)prop_buf);

	return ret;
}
