/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _BATTERY_CHARGER_H
#define _BATTERY_CHARGER_H
#ifdef CONFIG_PRODUCT_DOOM
#include <linux/lenovo_supply.h>
#endif

enum battery_charger_prop {
	BATTERY_RESISTANCE,
	BATTERY_CHARGER_PROP_MAX,
};

#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val);
#else
static inline int
qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	return -EINVAL;
}
#endif

#if IS_ENABLED(CONFIG_LENOVO_SUPPLY)
int qti_battery_charger_get_ext_prop(const char *name,
                                enum lenovo_supply_property prop, int *val);
int qti_battery_charger_get_qcom_prop(const char *name,
				enum power_supply_property prop, int *val);

int qti_battery_charger_set_ext_prop(const char *name,
                                enum lenovo_supply_property prop, int val);
const char *qti_battery_charger_get_real_type(const char *name);
int qti_power_supply_changed(const char *name);
#else
static inline int
qti_battery_charger_get_ext_prop(const char *name,
				enum lenovo_supply_property prop, int *val)
{
	return -EINVAL;
}
static inline int
qti_battery_charger_get_qcom_prop(const char *name,
				enum power_supply_property prop, int *val)
{
	return -EINVAL;
}
static const char
*qti_battery_charger_get_real_type(const char *name)
{
	return "Unknown";
}
static inline int
qti_power_supply_changed(const char *name)
{
	return -EINVAL;
}
#endif

#endif
