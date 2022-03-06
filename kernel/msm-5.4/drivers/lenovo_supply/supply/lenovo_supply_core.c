// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Universal power supply monitor class
 *
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/lenovo_supply.h>
#include <linux/property.h>
#include <linux/thermal.h>
#include "lenovo_supply.h"
#include <linux/power_supply.h>
#include <linux/soc/qcom/battery_charger.h>

/* exported for the APM Power driver, APM emulation */
struct class *lenovo_supply_class;
EXPORT_SYMBOL_GPL(lenovo_supply_class);

ATOMIC_NOTIFIER_HEAD(lenovo_supply_notifier);
EXPORT_SYMBOL_GPL(lenovo_supply_notifier);

static struct device_type lenovo_supply_dev_type;

#define LENOVO_SUPPLY_DEFERRED_REGISTER_TIME	msecs_to_jiffies(10)

#ifdef CONFIG_LENOVO_SUPPLY
static bool __lenovo_supply_is_supplied_by(struct lenovo_supply *supplier,
					 struct lenovo_supply *supply)
{
	int i;

	if (!supply->supplied_from && !supplier->supplied_to)
		return false;

	/* Support both supplied_to and supplied_from modes */
	if (supply->supplied_from) {
		if (!supplier->desc->name)
			return false;
		for (i = 0; i < supply->num_supplies; i++)
			if (!strcmp(supplier->desc->name, supply->supplied_from[i]))
				return true;
	} else {
		if (!supply->desc->name)
			return false;
		for (i = 0; i < supplier->num_supplicants; i++)
			if (!strcmp(supplier->supplied_to[i], supply->desc->name))
				return true;
	}

	return false;
}

static int __lenovo_supply_changed_work(struct device *dev, void *data)
{
	struct lenovo_supply *psy = data;
	struct lenovo_supply *pst = dev_get_drvdata(dev);

	if (__lenovo_supply_is_supplied_by(psy, pst)) {
		if (pst->desc->external_power_changed)
			pst->desc->external_power_changed(pst);
	}

	return 0;
}
#endif

static void lenovo_supply_changed_work(struct work_struct *work)
{
	unsigned long flags;
	struct lenovo_supply *psy = container_of(work, struct lenovo_supply,
						changed_work);

	dev_dbg(&psy->dev, "%s\n", __func__);

	spin_lock_irqsave(&psy->changed_lock, flags);
	/*
	 * Check 'changed' here to avoid issues due to race between
	 * lenovo_supply_changed() and this routine. In worst case
	 * lenovo_supply_changed() can be called again just before we take above
	 * lock. During the first call of this routine we will mark 'changed' as
	 * false and it will stay false for the next call as well.
	 */
	if (likely(psy->changed)) {
		psy->changed = false;
		spin_unlock_irqrestore(&psy->changed_lock, flags);
#ifdef CONFIG_LENOVO_SUPPLY
		class_for_each_device(lenovo_supply_class, NULL, psy,
				      __lenovo_supply_changed_work);
#endif
		atomic_notifier_call_chain(&lenovo_supply_notifier,
				LENOVO_PSY_EVENT_PROP_CHANGED, psy);
		kobject_uevent(&psy->dev.kobj, KOBJ_CHANGE);
		spin_lock_irqsave(&psy->changed_lock, flags);
	}

	/*
	 * Hold the wakeup_source until all events are processed.
	 * lenovo_supply_changed() might have called again and have set 'changed'
	 * to true.
	 */
	if (likely(!psy->changed))
		pm_relax(&psy->dev);
	spin_unlock_irqrestore(&psy->changed_lock, flags);
}

void lenovo_supply_changed(struct lenovo_supply *psy)
{
	unsigned long flags;

	dev_dbg(&psy->dev, "%s\n", __func__);
#ifdef CONFIG_LENOVO_SUPPLY
	qti_power_supply_changed("battery");
#endif

	spin_lock_irqsave(&psy->changed_lock, flags);
	psy->changed = true;
	pm_stay_awake(&psy->dev);
	spin_unlock_irqrestore(&psy->changed_lock, flags);
	schedule_work(&psy->changed_work);
}
EXPORT_SYMBOL_GPL(lenovo_supply_changed);

static int psy_register_cooler(struct device *dev, struct lenovo_supply *psy);
/*
 * Notify that power supply was registered after parent finished the probing.
 *
 * Often power supply is registered from driver's probe function. However
 * calling lenovo_supply_changed() directly from lenovo_supply_register()
 * would lead to execution of get_property() function provided by the driver
 * too early - before the probe ends.
 * Also, registering cooling device from the probe will execute the
 * get_property() function. So register the cooling device after the probe.
 *
 * Avoid that by waiting on parent's mutex.
 */
static void lenovo_supply_deferred_register_work(struct work_struct *work)
{
	struct lenovo_supply *psy = container_of(work, struct lenovo_supply,
						deferred_register_work.work);

	if (psy->dev.parent) {
		while (!mutex_trylock(&psy->dev.parent->mutex)) {
			if (psy->removing)
				return;
			msleep(10);
		}
	}

	psy_register_cooler(psy->dev.parent, psy);
	lenovo_supply_changed(psy);

	if (psy->dev.parent)
		mutex_unlock(&psy->dev.parent->mutex);
}

#ifdef CONFIG_OF
#ifdef CONFIG_LENOVO_SUPPLY
static int __lenovo_supply_populate_supplied_from(struct device *dev,
						 void *data)
{
	struct lenovo_supply *psy = data;
	struct lenovo_supply *epsy = dev_get_drvdata(dev);
	struct device_node *np;
	int i = 0;

	do {
		np = of_parse_phandle(psy->of_node, "power-supplies", i++);
		if (!np)
			break;

		if (np == epsy->of_node) {
			dev_info(&psy->dev, "%s: Found supply : %s\n",
				psy->desc->name, epsy->desc->name);
			psy->supplied_from[i-1] = (char *)epsy->desc->name;
			psy->num_supplies++;
			of_node_put(np);
			break;
		}
		of_node_put(np);
	} while (np);

	return 0;
}
#endif

static int lenovo_supply_populate_supplied_from(struct lenovo_supply *psy)
{
	int error=0;

#ifdef CONFIG_LENOVO_SUPPLY
	error = class_for_each_device(lenovo_supply_class, NULL, psy,
				      __lenovo_supply_populate_supplied_from);
#endif

	dev_dbg(&psy->dev, "%s %d\n", __func__, error);

	return error;
}

#ifdef CONFIG_LENOVO_SUPPLY
static int  __lenovo_supply_find_supply_from_node(struct device *dev,
						 void *data)
{
	struct device_node *np = data;
	struct lenovo_supply *epsy = dev_get_drvdata(dev);

	/* returning non-zero breaks out of class_for_each_device loop */
	if (epsy->of_node == np)
		return 1;

	return 0;
}
#endif

static int lenovo_supply_find_supply_from_node(struct device_node *supply_node)
{
	int error=0;

#ifdef CONFIG_LENOVO_SUPPLY
	/*
	 * class_for_each_device() either returns its own errors or values
	 * returned by __lenovo_supply_find_supply_from_node().
	 *
	 * __lenovo_supply_find_supply_from_node() will return 0 (no match)
	 * or 1 (match).
	 *
	 * We return 0 if class_for_each_device() returned 1, -EPROBE_DEFER if
	 * it returned 0, or error as returned by it.
	 */
	error = class_for_each_device(lenovo_supply_class, NULL, supply_node,
				       __lenovo_supply_find_supply_from_node);
#endif

	return error ? (error == 1 ? 0 : error) : -EPROBE_DEFER;
}

static int lenovo_supply_check_supplies(struct lenovo_supply *psy)
{
	struct device_node *np;
	int cnt = 0;

	/* If there is already a list honor it */
	if (psy->supplied_from && psy->num_supplies > 0)
		return 0;

	/* No device node found, nothing to do */
	if (!psy->of_node)
		return 0;

	do {
		int ret;

		np = of_parse_phandle(psy->of_node, "power-supplies", cnt++);
		if (!np)
			break;

		ret = lenovo_supply_find_supply_from_node(np);
		of_node_put(np);

		if (ret) {
			dev_dbg(&psy->dev, "Failed to find supply!\n");
			return ret;
		}
	} while (np);

	/* Missing valid "power-supplies" entries */
	if (cnt == 1)
		return 0;

	/* All supplies found, allocate char ** array for filling */
	psy->supplied_from = devm_kzalloc(&psy->dev, sizeof(psy->supplied_from),
					  GFP_KERNEL);
	if (!psy->supplied_from)
		return -ENOMEM;

	*psy->supplied_from = devm_kcalloc(&psy->dev,
					   cnt - 1, sizeof(char *),
					   GFP_KERNEL);
	if (!*psy->supplied_from)
		return -ENOMEM;

	return lenovo_supply_populate_supplied_from(psy);
}
#else
static int lenovo_supply_check_supplies(struct lenovo_supply *psy)
{
	int nval, ret;

	if (!psy->dev.parent)
		return 0;

	nval = device_property_read_string_array(psy->dev.parent,
						 "supplied-from", NULL, 0);
	if (nval <= 0)
		return 0;

	psy->supplied_from = devm_kmalloc_array(&psy->dev, nval,
						sizeof(char *), GFP_KERNEL);
	if (!psy->supplied_from)
		return -ENOMEM;

	ret = device_property_read_string_array(psy->dev.parent,
		"supplied-from", (const char **)psy->supplied_from, nval);
	if (ret < 0)
		return ret;

	psy->num_supplies = nval;

	return 0;
}
#endif

struct psy_am_i_supplied_data {
	struct lenovo_supply *psy;
	unsigned int count;
};

#ifdef CONFIG_LENOVO_SUPPLY
static int __lenovo_supply_am_i_supplied(struct device *dev, void *_data)
{
	union lenovo_supply_propval ret = {0,};
	struct lenovo_supply *epsy = dev_get_drvdata(dev);
	struct psy_am_i_supplied_data *data = _data;

	if (__lenovo_supply_is_supplied_by(epsy, data->psy)) {
		data->count++;
		if (!epsy->desc->get_property(epsy, LENOVO_SUPPLY_PROP_ONLINE,
					&ret))
			return ret.intval;
	}

	return 0;
}
#endif

int lenovo_supply_am_i_supplied(struct lenovo_supply *psy)
{
	struct psy_am_i_supplied_data data = { psy, 0 };
	int error=0;

#ifdef CONFIG_LENOVO_SUPPLY
	error = class_for_each_device(lenovo_supply_class, NULL, &data,
				      __lenovo_supply_am_i_supplied);
#endif

	dev_dbg(&psy->dev, "%s count %u err %d\n", __func__, data.count, error);

	if (data.count == 0)
		return -ENODEV;

	return error;
}
EXPORT_SYMBOL_GPL(lenovo_supply_am_i_supplied);

#ifdef CONFIG_LENOVO_SUPPLY
static int __lenovo_supply_is_system_supplied(struct device *dev, void *data)
{
	union lenovo_supply_propval ret = {0,};
	struct lenovo_supply *psy = dev_get_drvdata(dev);
	unsigned int *count = data;

	(*count)++;
	if (psy->desc->type != LENOVO_SUPPLY_TYPE_BATTERY)
		if (!psy->desc->get_property(psy, LENOVO_SUPPLY_PROP_ONLINE,
					&ret))
			return ret.intval;

	return 0;
}
#endif

int lenovo_supply_is_system_supplied(void)
{
	int error=0;
	unsigned int count = 0;

#ifdef CONFIG_LENOVO_SUPPLY
	error = class_for_each_device(lenovo_supply_class, NULL, &count,
				      __lenovo_supply_is_system_supplied);
#endif

	/*
	 * If no power class device was found at all, most probably we are
	 * running on a desktop system, so assume we are on mains power.
	 */
	if (count == 0)
		return 1;

	return error;
}
EXPORT_SYMBOL_GPL(lenovo_supply_is_system_supplied);

#ifdef CONFIG_LENOVO_SUPPLY
static int __lenovo_supply_get_supplier_max_current(struct device *dev,
						   void *data)
{
	union lenovo_supply_propval ret = {0,};
	struct lenovo_supply *epsy = dev_get_drvdata(dev);
	struct lenovo_supply *psy = data;

	if (__lenovo_supply_is_supplied_by(epsy, psy))
		if (!epsy->desc->get_property(epsy,
					      LENOVO_SUPPLY_PROP_CURRENT_MAX,
					      &ret))
			return ret.intval;

	return 0;
}
#endif

int lenovo_supply_set_input_current_limit_from_supplier(struct lenovo_supply *psy)
{
	union lenovo_supply_propval val = {0,};
	int curr=0;

	if (!psy->desc->set_property)
		return -EINVAL;

	/*
	 * This function is not intended for use with a supply with multiple
	 * suppliers, we simply pick the first supply to report a non 0
	 * max-current.
	 */
#ifdef CONFIG_LENOVO_SUPPLY
	curr = class_for_each_device(lenovo_supply_class, NULL, psy,
				      __lenovo_supply_get_supplier_max_current);
#endif
	if (curr <= 0)
		return (curr == 0) ? -ENODEV : curr;

	val.intval = curr;

	return psy->desc->set_property(psy,
				LENOVO_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
}
EXPORT_SYMBOL_GPL(lenovo_supply_set_input_current_limit_from_supplier);

int lenovo_supply_set_battery_charged(struct lenovo_supply *psy)
{
	if (atomic_read(&psy->use_cnt) >= 0 &&
			psy->desc->type == LENOVO_SUPPLY_TYPE_BATTERY &&
			psy->desc->set_charged) {
		psy->desc->set_charged(psy);
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(lenovo_supply_set_battery_charged);

static int lenovo_supply_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct lenovo_supply *psy = dev_get_drvdata(dev);

	return strcmp(psy->desc->name, name) == 0;
}

/**
 * lenovo_supply_get_by_name() - Search for a power supply and returns its ref
 * @name: Power supply name to fetch
 *
 * If power supply was found, it increases reference count for the
 * internal power supply's device. The user should lenovo_supply_put()
 * after usage.
 *
 * Return: On success returns a reference to a power supply with
 * matching name equals to @name, a NULL otherwise.
 */
struct lenovo_supply *lenovo_supply_get_by_name(const char *name)
{
	struct lenovo_supply *psy = NULL;
	struct device *dev = class_find_device(lenovo_supply_class, NULL, name,
					lenovo_supply_match_device_by_name);

	if (dev) {
		psy = dev_get_drvdata(dev);
		atomic_inc(&psy->use_cnt);
	}

	return psy;
}
EXPORT_SYMBOL_GPL(lenovo_supply_get_by_name);

/**
 * lenovo_supply_put() - Drop reference obtained with lenovo_supply_get_by_name
 * @psy: Reference to put
 *
 * The reference to power supply should be put before unregistering
 * the power supply.
 */
void lenovo_supply_put(struct lenovo_supply *psy)
{
	might_sleep();

	atomic_dec(&psy->use_cnt);
	put_device(&psy->dev);
}
EXPORT_SYMBOL_GPL(lenovo_supply_put);

#ifdef CONFIG_OF
static int lenovo_supply_match_device_node(struct device *dev, const void *data)
{
	return dev->parent && dev->parent->of_node == data;
}

/**
 * lenovo_supply_get_by_phandle() - Search for a power supply and returns its ref
 * @np: Pointer to device node holding phandle property
 * @property: Name of property holding a power supply name
 *
 * If power supply was found, it increases reference count for the
 * internal power supply's device. The user should lenovo_supply_put()
 * after usage.
 *
 * Return: On success returns a reference to a power supply with
 * matching name equals to value under @property, NULL or ERR_PTR otherwise.
 */
struct lenovo_supply *lenovo_supply_get_by_phandle(struct device_node *np,
							const char *property)
{
	struct device_node *lenovo_supply_np;
	struct lenovo_supply *psy = NULL;
	struct device *dev;

	lenovo_supply_np = of_parse_phandle(np, property, 0);
	if (!lenovo_supply_np)
		return ERR_PTR(-ENODEV);

	dev = class_find_device(lenovo_supply_class, NULL, lenovo_supply_np,
						lenovo_supply_match_device_node);

	of_node_put(lenovo_supply_np);

	if (dev) {
		psy = dev_get_drvdata(dev);
		atomic_inc(&psy->use_cnt);
	}

	return psy;
}
EXPORT_SYMBOL_GPL(lenovo_supply_get_by_phandle);

static void devm_lenovo_supply_put(struct device *dev, void *res)
{
	struct lenovo_supply **psy = res;

	lenovo_supply_put(*psy);
}

/**
 * devm_lenovo_supply_get_by_phandle() - Resource managed version of
 *  lenovo_supply_get_by_phandle()
 * @dev: Pointer to device holding phandle property
 * @property: Name of property holding a power supply phandle
 *
 * Return: On success returns a reference to a power supply with
 * matching name equals to value under @property, NULL or ERR_PTR otherwise.
 */
struct lenovo_supply *devm_lenovo_supply_get_by_phandle(struct device *dev,
						      const char *property)
{
	struct lenovo_supply **ptr, *psy;

	if (!dev->of_node)
		return ERR_PTR(-ENODEV);

	ptr = devres_alloc(devm_lenovo_supply_put, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	psy = lenovo_supply_get_by_phandle(dev->of_node, property);
	if (IS_ERR_OR_NULL(psy)) {
		devres_free(ptr);
	} else {
		*ptr = psy;
		devres_add(dev, ptr);
	}
	return psy;
}
EXPORT_SYMBOL_GPL(devm_lenovo_supply_get_by_phandle);
#endif /* CONFIG_OF */

int lenovo_supply_get_battery_info(struct lenovo_supply *psy,
				  struct lenovo_supply_battery_info *info)
{
	struct lenovo_supply_resistance_temp_table *resist_table;
	struct device_node *battery_np;
	const char *value;
	int err, len, index;
	const __be32 *list;

	info->energy_full_design_uwh         = -EINVAL;
	info->charge_full_design_uah         = -EINVAL;
	info->voltage_min_design_uv          = -EINVAL;
	info->voltage_max_design_uv          = -EINVAL;
	info->precharge_current_ua           = -EINVAL;
	info->charge_term_current_ua         = -EINVAL;
	info->constant_charge_current_max_ua = -EINVAL;
	info->constant_charge_voltage_max_uv = -EINVAL;
	info->factory_internal_resistance_uohm  = -EINVAL;
	info->resist_table = NULL;

	for (index = 0; index < LENOVO_SUPPLY_OCV_TEMP_MAX; index++) {
		info->ocv_table[index]       = NULL;
		info->ocv_temp[index]        = -EINVAL;
		info->ocv_table_size[index]  = -EINVAL;
	}

	if (!psy->of_node) {
		dev_warn(&psy->dev, "%s currently only supports devicetree\n",
			 __func__);
		return -ENXIO;
	}

	battery_np = of_parse_phandle(psy->of_node, "monitored-battery", 0);
	if (!battery_np)
		return -ENODEV;

	err = of_property_read_string(battery_np, "compatible", &value);
	if (err)
		goto out_put_node;

	if (strcmp("simple-battery", value)) {
		err = -ENODEV;
		goto out_put_node;
	}

	/* The property and field names below must correspond to elements
	 * in enum lenovo_supply_property. For reasoning, see
	 * Documentation/power/lenovo_supply_class.rst.
	 */

	of_property_read_u32(battery_np, "energy-full-design-microwatt-hours",
			     &info->energy_full_design_uwh);
	of_property_read_u32(battery_np, "charge-full-design-microamp-hours",
			     &info->charge_full_design_uah);
	of_property_read_u32(battery_np, "voltage-min-design-microvolt",
			     &info->voltage_min_design_uv);
	of_property_read_u32(battery_np, "voltage-max-design-microvolt",
			     &info->voltage_max_design_uv);
	of_property_read_u32(battery_np, "precharge-current-microamp",
			     &info->precharge_current_ua);
	of_property_read_u32(battery_np, "charge-term-current-microamp",
			     &info->charge_term_current_ua);
	of_property_read_u32(battery_np, "constant-charge-current-max-microamp",
			     &info->constant_charge_current_max_ua);
	of_property_read_u32(battery_np, "constant-charge-voltage-max-microvolt",
			     &info->constant_charge_voltage_max_uv);
	of_property_read_u32(battery_np, "factory-internal-resistance-micro-ohms",
			     &info->factory_internal_resistance_uohm);

	len = of_property_count_u32_elems(battery_np, "ocv-capacity-celsius");
	if (len < 0 && len != -EINVAL) {
		err = len;
		goto out_put_node;
	} else if (len > LENOVO_SUPPLY_OCV_TEMP_MAX) {
		dev_err(&psy->dev, "Too many temperature values\n");
		err = -EINVAL;
		goto out_put_node;
	} else if (len > 0) {
		of_property_read_u32_array(battery_np, "ocv-capacity-celsius",
					   info->ocv_temp, len);
	}

	for (index = 0; index < len; index++) {
		struct lenovo_supply_battery_ocv_table *table;
		char *propname;
		int i, tab_len, size;

		propname = kasprintf(GFP_KERNEL, "ocv-capacity-table-%d", index);
		list = of_get_property(battery_np, propname, &size);
		if (!list || !size) {
			dev_err(&psy->dev, "failed to get %s\n", propname);
			kfree(propname);
			lenovo_supply_put_battery_info(psy, info);
			err = -EINVAL;
			goto out_put_node;
		}

		kfree(propname);
		tab_len = size / (2 * sizeof(__be32));
		info->ocv_table_size[index] = tab_len;

		table = info->ocv_table[index] =
			devm_kcalloc(&psy->dev, tab_len, sizeof(*table), GFP_KERNEL);
		if (!info->ocv_table[index]) {
			lenovo_supply_put_battery_info(psy, info);
			err = -ENOMEM;
			goto out_put_node;
		}

		for (i = 0; i < tab_len; i++) {
			table[i].ocv = be32_to_cpu(*list);
			list++;
			table[i].capacity = be32_to_cpu(*list);
			list++;
		}
	}

	list = of_get_property(battery_np, "resistance-temp-table", &len);
	if (!list || !len)
		goto out_put_node;

	info->resist_table_size = len / (2 * sizeof(__be32));
	resist_table = info->resist_table = devm_kcalloc(&psy->dev,
							 info->resist_table_size,
							 sizeof(*resist_table),
							 GFP_KERNEL);
	if (!info->resist_table) {
		lenovo_supply_put_battery_info(psy, info);
		err = -ENOMEM;
		goto out_put_node;
	}

	for (index = 0; index < info->resist_table_size; index++) {
		resist_table[index].temp = be32_to_cpu(*list++);
		resist_table[index].resistance = be32_to_cpu(*list++);
	}

out_put_node:
	of_node_put(battery_np);
	return err;
}
EXPORT_SYMBOL_GPL(lenovo_supply_get_battery_info);

void lenovo_supply_put_battery_info(struct lenovo_supply *psy,
				   struct lenovo_supply_battery_info *info)
{
	int i;

	for (i = 0; i < LENOVO_SUPPLY_OCV_TEMP_MAX; i++) {
		if (info->ocv_table[i])
			devm_kfree(&psy->dev, info->ocv_table[i]);
	}

	if (info->resist_table)
		devm_kfree(&psy->dev, info->resist_table);
}
EXPORT_SYMBOL_GPL(lenovo_supply_put_battery_info);

/**
 * lenovo_supply_temp2resist_simple() - find the battery internal resistance
 * percent
 * @table: Pointer to battery resistance temperature table
 * @table_len: The table length
 * @ocv: Current temperature
 *
 * This helper function is used to look up battery internal resistance percent
 * according to current temperature value from the resistance temperature table,
 * and the table must be ordered descending. Then the actual battery internal
 * resistance = the ideal battery internal resistance * percent / 100.
 *
 * Return: the battery internal resistance percent
 */
int lenovo_supply_temp2resist_simple(struct lenovo_supply_resistance_temp_table *table,
				    int table_len, int temp)
{
	int i, resist;

	for (i = 0; i < table_len; i++)
		if (temp > table[i].temp)
			break;

	if (i > 0 && i < table_len) {
		int tmp;

		tmp = (table[i - 1].resistance - table[i].resistance) *
			(temp - table[i].temp);
		tmp /= table[i - 1].temp - table[i].temp;
		resist = tmp + table[i].resistance;
	} else if (i == 0) {
		resist = table[0].resistance;
	} else {
		resist = table[table_len - 1].resistance;
	}

	return resist;
}
EXPORT_SYMBOL_GPL(lenovo_supply_temp2resist_simple);

/**
 * lenovo_supply_ocv2cap_simple() - find the battery capacity
 * @table: Pointer to battery OCV lookup table
 * @table_len: OCV table length
 * @ocv: Current OCV value
 *
 * This helper function is used to look up battery capacity according to
 * current OCV value from one OCV table, and the OCV table must be ordered
 * descending.
 *
 * Return: the battery capacity.
 */
int lenovo_supply_ocv2cap_simple(struct lenovo_supply_battery_ocv_table *table,
				int table_len, int ocv)
{
	int i, cap, tmp;

	for (i = 0; i < table_len; i++)
		if (ocv > table[i].ocv)
			break;

	if (i > 0 && i < table_len) {
		tmp = (table[i - 1].capacity - table[i].capacity) *
			(ocv - table[i].ocv);
		tmp /= table[i - 1].ocv - table[i].ocv;
		cap = tmp + table[i].capacity;
	} else if (i == 0) {
		cap = table[0].capacity;
	} else {
		cap = table[table_len - 1].capacity;
	}

	return cap;
}
EXPORT_SYMBOL_GPL(lenovo_supply_ocv2cap_simple);

struct lenovo_supply_battery_ocv_table *
lenovo_supply_find_ocv2cap_table(struct lenovo_supply_battery_info *info,
				int temp, int *table_len)
{
	int best_temp_diff = INT_MAX, temp_diff;
	u8 i, best_index = 0;

	if (!info->ocv_table[0])
		return NULL;

	for (i = 0; i < LENOVO_SUPPLY_OCV_TEMP_MAX; i++) {
		temp_diff = abs(info->ocv_temp[i] - temp);

		if (temp_diff < best_temp_diff) {
			best_temp_diff = temp_diff;
			best_index = i;
		}
	}

	*table_len = info->ocv_table_size[best_index];
	return info->ocv_table[best_index];
}
EXPORT_SYMBOL_GPL(lenovo_supply_find_ocv2cap_table);

int lenovo_supply_batinfo_ocv2cap(struct lenovo_supply_battery_info *info,
				 int ocv, int temp)
{
	struct lenovo_supply_battery_ocv_table *table;
	int table_len;

	table = lenovo_supply_find_ocv2cap_table(info, temp, &table_len);
	if (!table)
		return -EINVAL;

	return lenovo_supply_ocv2cap_simple(table, table_len, ocv);
}
EXPORT_SYMBOL_GPL(lenovo_supply_batinfo_ocv2cap);

int lenovo_supply_get_property(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    union lenovo_supply_propval *val)
{
	if (atomic_read(&psy->use_cnt) <= 0) {
		if (!psy->initialized)
			return -EAGAIN;
		return -ENODEV;
	}

	return psy->desc->get_property(psy, psp, val);
}
EXPORT_SYMBOL_GPL(lenovo_supply_get_property);

int lenovo_supply_set_property(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    const union lenovo_supply_propval *val)
{
	if (atomic_read(&psy->use_cnt) <= 0 || !psy->desc->set_property)
		return -ENODEV;

	return psy->desc->set_property(psy, psp, val);
}
EXPORT_SYMBOL_GPL(lenovo_supply_set_property);

int lenovo_supply_property_is_writeable(struct lenovo_supply *psy,
					enum lenovo_supply_property psp)
{
	if (atomic_read(&psy->use_cnt) <= 0 ||
			!psy->desc->property_is_writeable)
		return -ENODEV;

	return psy->desc->property_is_writeable(psy, psp);
}
EXPORT_SYMBOL_GPL(lenovo_supply_property_is_writeable);

void lenovo_supply_external_power_changed(struct lenovo_supply *psy)
{
	if (atomic_read(&psy->use_cnt) <= 0 ||
			!psy->desc->external_power_changed)
		return;

	psy->desc->external_power_changed(psy);
}
EXPORT_SYMBOL_GPL(lenovo_supply_external_power_changed);

int lenovo_supply_powers(struct lenovo_supply *psy, struct device *dev)
{
	return sysfs_create_link(&psy->dev.kobj, &dev->kobj, "powers");
}
EXPORT_SYMBOL_GPL(lenovo_supply_powers);

static void lenovo_supply_dev_release(struct device *dev)
{
	struct lenovo_supply *psy = to_lenovo_supply(dev);
	dev_dbg(dev, "%s\n", __func__);
	kfree(psy);
}

int lenovo_supply_reg_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&lenovo_supply_notifier, nb);
}
EXPORT_SYMBOL_GPL(lenovo_supply_reg_notifier);

void lenovo_supply_unreg_notifier(struct notifier_block *nb)
{
	atomic_notifier_chain_unregister(&lenovo_supply_notifier, nb);
}
EXPORT_SYMBOL_GPL(lenovo_supply_unreg_notifier);

#ifdef CONFIG_THERMAL
static int lenovo_supply_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return ret;

	/* Convert tenths of degree Celsius to milli degree Celsius. */
	*temp = val.intval * 100;

	return ret;
}

static struct thermal_zone_device_ops psy_tzd_ops = {
	.get_temp = lenovo_supply_read_temp,
};

#ifdef SUPPORT_USER_THERMAL_CASE
static int lenovo_supply_read_display_rate(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE, &val);
	if (ret)
		return ret;

	*temp = val.intval;

	return ret;
}

static struct thermal_zone_device_ops psy_display_rate_tzd_ops = {
	.get_temp = lenovo_supply_read_display_rate,
};

static int lenovo_supply_read_speaker(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_THERM_SPEAKER, &val);
	if (ret)
		return ret;

	*temp = val.intval;

	return ret;
}

static struct thermal_zone_device_ops psy_speaker_tzd_ops = {
	.get_temp = lenovo_supply_read_speaker,
};

static int lenovo_supply_read_camera(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_THERM_CAMERA, &val);
	if (ret)
		return ret;

	*temp = val.intval;

	return ret;
}

static struct thermal_zone_device_ops psy_camera_tzd_ops = {
	.get_temp = lenovo_supply_read_camera,
};

static int lenovo_supply_read_therm_event(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_THERM_EVENT, &val);
	if (ret)
		return ret;

	*temp = val.intval;

	return ret;
}

static struct thermal_zone_device_ops psy_therm_event_tzd_ops = {
	.get_temp = lenovo_supply_read_therm_event,
};

static int lenovo_supply_read_modem_5g(struct thermal_zone_device *tzd,
		int *temp)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = lenovo_supply_get_property(psy, LENOVO_SUPPLY_PROP_THERM_MODEM_5G, &val);
	if (ret)
		return ret;

	*temp = val.intval;

	return ret;
}

static struct thermal_zone_device_ops psy_modem_5g_tzd_ops = {
	.get_temp = lenovo_supply_read_modem_5g,
};
#endif

static int psy_register_thermal(struct lenovo_supply *psy)
{
#ifdef SUPPORT_USER_THERMAL_CASE
	int i;
	int ret = 0;

	if (psy->desc->no_thermal)
		return 0;

	/* Register battery zone device psy reports temperature */
	for (i = 0; i < psy->desc->num_properties; i++) {
		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_TEMP) {
			psy->tzd = thermal_zone_device_register(psy->desc->name,
					0, 0, psy, &psy_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd);
		}

		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE) {
			psy->tzd_display_rate = thermal_zone_device_register("display-rate",
					0, 0, psy, &psy_display_rate_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd_display_rate);
		}

		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_THERM_SPEAKER) {
			psy->tzd_speaker = thermal_zone_device_register("speaker2",
					0, 0, psy, &psy_speaker_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd_speaker);
		}

		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_THERM_MODEM_5G) {
			psy->tzd_modem_5g = thermal_zone_device_register("modem-5g",
					0, 0, psy, &psy_modem_5g_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd_modem_5g);
		}

		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_THERM_CAMERA) {
			psy->tzd_camera = thermal_zone_device_register("camera2",
					0, 0, psy, &psy_camera_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd_camera);
		}

		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_THERM_EVENT) {
			psy->tzd_therm_event = thermal_zone_device_register("therm_event",
					0, 0, psy, &psy_therm_event_tzd_ops, NULL, 0, 0);
			ret |= PTR_ERR_OR_ZERO(psy->tzd_therm_event);
		}
	}
	return ret;
#else
	int i;

	if (psy->desc->no_thermal)
		return 0;

	/* Register battery zone device psy reports temperature */
	for (i = 0; i < psy->desc->num_properties; i++) {
		if (psy->desc->properties[i] == LENOVO_SUPPLY_PROP_TEMP) {
			psy->tzd = thermal_zone_device_register(psy->desc->name,
					0, 0, psy, &psy_tzd_ops, NULL, 0, 0);
			return PTR_ERR_OR_ZERO(psy->tzd);
		}
	}
	return 0;
#endif
}

static void psy_unregister_thermal(struct lenovo_supply *psy)
{
#ifdef CONFIG_LENOVO_SUPPLY
#ifdef SUPPORT_USER_THERMAL_CASE
	if (!IS_ERR_OR_NULL(psy->tzd))
		thermal_zone_device_unregister(psy->tzd);

	if (!IS_ERR_OR_NULL(psy->tzd_display_rate))
		thermal_zone_device_unregister(psy->tzd_display_rate);

	if (!IS_ERR_OR_NULL(psy->tzd_speaker))
		thermal_zone_device_unregister(psy->tzd_speaker);

	if (!IS_ERR_OR_NULL(psy->tzd_modem_5g))
		thermal_zone_device_unregister(psy->tzd_modem_5g);

	if (!IS_ERR_OR_NULL(psy->tzd_camera))
		thermal_zone_device_unregister(psy->tzd_camera);

	if (!IS_ERR_OR_NULL(psy->tzd_therm_event))
		thermal_zone_device_unregister(psy->tzd_therm_event);
#else
	if (IS_ERR_OR_NULL(psy->tzd))
		return;
	thermal_zone_device_unregister(psy->tzd);
#endif
#endif
}

/* thermal cooling device callbacks */
static int ps_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_tcd_ops = {
	.get_max_state = ps_get_max_charge_cntl_limit,
	.get_cur_state = ps_get_cur_charge_cntl_limit,
	.set_cur_state = ps_set_cur_charge_cntl_limit,
};

#ifdef SUPPORT_USER_THERMAL_CASE
static int ps_get_max_display_rate_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_display_rate_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_display_rate_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_display_rate_tcd_ops = {
	.get_max_state = ps_get_max_display_rate_limit,
	.get_cur_state = ps_get_cur_display_rate_limit,
	.set_cur_state = ps_set_cur_display_rate_limit,
};

static int ps_get_max_speaker_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_speaker_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_speaker_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_speaker_tcd_ops = {
	.get_max_state = ps_get_max_speaker_limit,
	.get_cur_state = ps_get_cur_speaker_limit,
	.set_cur_state = ps_set_cur_speaker_limit,
};

static int ps_get_max_modem_5g_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_modem_5g_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_modem_5g_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_modem_5g_tcd_ops = {
	.get_max_state = ps_get_max_modem_5g_limit,
	.get_cur_state = ps_get_cur_modem_5g_limit,
	.set_cur_state = ps_set_cur_modem_5g_limit,
};

static int ps_get_max_camera_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_camera_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_camera_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_camera_tcd_ops = {
	.get_max_state = ps_get_max_camera_limit,
	.get_cur_state = ps_get_cur_camera_limit,
	.set_cur_state = ps_set_cur_camera_limit,
};

static int ps_get_max_therm_event_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL_MAX, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_get_cur_therm_event_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	ret = lenovo_supply_get_property(psy,
			LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_therm_event_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct lenovo_supply *psy;
	union lenovo_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_therm_event_tcd_ops = {
	.get_max_state = ps_get_max_therm_event_limit,
	.get_cur_state = ps_get_cur_therm_event_limit,
	.set_cur_state = ps_set_cur_therm_event_limit,
};
#endif

static int psy_register_cooler(struct device *dev, struct lenovo_supply *psy)
{
#ifdef CONFIG_LENOVO_SUPPLY
#ifdef SUPPORT_USER_THERMAL_CASE
	int i;
	int ret = 0;

	/* Register for cooling device if psy can control charging */
	for (i = 0; i < psy->desc->num_properties; i++) {
		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT) {
			if (dev)
				psy->tcd = thermal_of_cooling_device_register(
							dev_of_node(dev),
							(char *)psy->desc->name,
							psy, &psy_tcd_ops);
			else
				psy->tcd = thermal_cooling_device_register(
							(char *)psy->desc->name,
							psy, &psy_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd);
		}

		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL_MAX) {
			if (dev)
				psy->tcd_display_rate = thermal_of_cooling_device_register(
							dev_of_node(dev),
							"display-rate",
							psy, &psy_display_rate_tcd_ops);
			else
				psy->tcd_display_rate = thermal_cooling_device_register(
							"display-rate",
							psy, &psy_display_rate_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd_display_rate);
		}

		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL_MAX) {
			if (dev)
				psy->tcd_speaker = thermal_of_cooling_device_register(
							dev_of_node(dev),
							"speaker2",
							psy, &psy_speaker_tcd_ops);
			else
				psy->tcd_speaker = thermal_cooling_device_register(
							"speaker2",
							psy, &psy_speaker_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd_speaker);
		}

		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL_MAX) {
			if (dev)
				psy->tcd_modem_5g = thermal_of_cooling_device_register(
							dev_of_node(dev),
							"modem-5g",
							psy, &psy_modem_5g_tcd_ops);
			else
				psy->tcd_modem_5g = thermal_cooling_device_register(
							"modem-5g",
							psy, &psy_modem_5g_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd_modem_5g);
		}

		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL_MAX) {
			if (dev)
				psy->tcd_camera = thermal_of_cooling_device_register(
							dev_of_node(dev),
							"camera2",
							psy, &psy_camera_tcd_ops);
			else
				psy->tcd_camera = thermal_cooling_device_register(
							"camera2",
							psy, &psy_camera_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd_camera);
		}

		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL_MAX) {
			if (dev)
				psy->tcd_therm_event = thermal_of_cooling_device_register(
							dev_of_node(dev),
							"therm_event",
							psy, &psy_therm_event_tcd_ops);
			else
				psy->tcd_therm_event = thermal_cooling_device_register(
							"therm_event",
							psy, &psy_therm_event_tcd_ops);
			ret |= PTR_ERR_OR_ZERO(psy->tcd_therm_event);
		}

	}
	return ret;
#else
	int i;

	/* Register for cooling device if psy can control charging */
	for (i = 0; i < psy->desc->num_properties; i++) {
		if (psy->desc->properties[i] ==
				LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT) {
			if (dev)
				psy->tcd = thermal_of_cooling_device_register(
							dev_of_node(dev),
							(char *)psy->desc->name,
							psy, &psy_tcd_ops);
			else
				psy->tcd = thermal_cooling_device_register(
							(char *)psy->desc->name,
							psy, &psy_tcd_ops);
			return PTR_ERR_OR_ZERO(psy->tcd);
		}
	}
	return 0;
#endif
#else
	return 0;
#endif
}

static void psy_unregister_cooler(struct lenovo_supply *psy)
{
#ifdef SUPPORT_USER_THERMAL_CASE
	if (!IS_ERR_OR_NULL(psy->tcd))
		thermal_cooling_device_unregister(psy->tcd);

	if (!IS_ERR_OR_NULL(psy->tcd_display_rate))
		thermal_cooling_device_unregister(psy->tcd_display_rate);

	if (!IS_ERR_OR_NULL(psy->tcd_speaker))
		thermal_cooling_device_unregister(psy->tcd_speaker);

	if (!IS_ERR_OR_NULL(psy->tcd_modem_5g))
		thermal_cooling_device_unregister(psy->tcd_modem_5g);

	if (!IS_ERR_OR_NULL(psy->tcd_camera))
		thermal_cooling_device_unregister(psy->tcd_camera);

	if (!IS_ERR_OR_NULL(psy->tcd_therm_event))
		thermal_cooling_device_unregister(psy->tcd_therm_event);
#else
	if (IS_ERR_OR_NULL(psy->tcd))
		return;
	thermal_cooling_device_unregister(psy->tcd);
#endif
}
#else
static int psy_register_thermal(struct lenovo_supply *psy)
{
	return 0;
}

static void psy_unregister_thermal(struct lenovo_supply *psy)
{
}

static int psy_register_cooler(struct device *dev, struct lenovo_supply *psy)
{
	return 0;
}

static void psy_unregister_cooler(struct lenovo_supply *psy)
{
}
#endif

static struct lenovo_supply *__must_check
__lenovo_supply_register(struct device *parent,
				   const struct lenovo_supply_desc *desc,
				   const struct lenovo_supply_config *cfg,
				   bool ws)
{
	struct device *dev;
	struct lenovo_supply *psy;
	int i, rc;

	if (!parent)
		pr_warn("%s: Expected proper parent device for '%s'\n",
			__func__, desc->name);

	if (!desc || !desc->name || !desc->properties || !desc->num_properties)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < desc->num_properties; ++i) {
		if ((desc->properties[i] == LENOVO_SUPPLY_PROP_USB_TYPE) &&
		    (!desc->usb_types || !desc->num_usb_types))
			return ERR_PTR(-EINVAL);
	}

	psy = kzalloc(sizeof(*psy), GFP_KERNEL);
	if (!psy)
		return ERR_PTR(-ENOMEM);

	dev = &psy->dev;

	device_initialize(dev);

	dev->class = lenovo_supply_class;
	dev->type = &lenovo_supply_dev_type;
	dev->parent = parent;
	dev->release = lenovo_supply_dev_release;
	dev_set_drvdata(dev, psy);
	psy->desc = desc;
	if (cfg) {
		dev->groups = cfg->attr_grp;
		psy->drv_data = cfg->drv_data;
		psy->of_node =
			cfg->fwnode ? to_of_node(cfg->fwnode) : cfg->of_node;
		psy->supplied_to = cfg->supplied_to;
		psy->num_supplicants = cfg->num_supplicants;
	}

	rc = dev_set_name(dev, "%s", desc->name);
	if (rc)
		goto dev_set_name_failed;

	INIT_WORK(&psy->changed_work, lenovo_supply_changed_work);
	INIT_DELAYED_WORK(&psy->deferred_register_work,
			  lenovo_supply_deferred_register_work);

	rc = lenovo_supply_check_supplies(psy);
	if (rc) {
		dev_info(dev, "Not all required supplies found, defer probe\n");
		goto check_supplies_failed;
	}

	spin_lock_init(&psy->changed_lock);
	rc = device_add(dev);
	if (rc)
		goto device_add_failed;

	rc = device_init_wakeup(dev, ws);
	if (rc)
		goto wakeup_init_failed;

	rc = psy_register_thermal(psy);
	if (rc)
		goto register_thermal_failed;

	rc = lenovo_supply_add_hwmon_sysfs(psy);
	if (rc)
		goto add_hwmon_sysfs_failed;

	/*
	 * Update use_cnt after any uevents (most notably from device_add()).
	 * We are here still during driver's probe but
	 * the lenovo_supply_uevent() calls back driver's get_property
	 * method so:
	 * 1. Driver did not assigned the returned struct lenovo_supply,
	 * 2. Driver could not finish initialization (anything in its probe
	 *    after calling lenovo_supply_register()).
	 */
	atomic_inc(&psy->use_cnt);
	psy->initialized = true;

	queue_delayed_work(system_power_efficient_wq,
			   &psy->deferred_register_work,
			   LENOVO_SUPPLY_DEFERRED_REGISTER_TIME);

	return psy;

add_hwmon_sysfs_failed:
	psy_unregister_thermal(psy);
register_thermal_failed:
	device_del(dev);
wakeup_init_failed:
device_add_failed:
check_supplies_failed:
dev_set_name_failed:
	put_device(dev);
	return ERR_PTR(rc);
}

/**
 * lenovo_supply_register() - Register new power supply
 * @parent:	Device to be a parent of power supply's device, usually
 *		the device which probe function calls this
 * @desc:	Description of power supply, must be valid through whole
 *		lifetime of this power supply
 * @cfg:	Run-time specific configuration accessed during registering,
 *		may be NULL
 *
 * Return: A pointer to newly allocated lenovo_supply on success
 * or ERR_PTR otherwise.
 * Use lenovo_supply_unregister() on returned lenovo_supply pointer to release
 * resources.
 */
struct lenovo_supply *__must_check lenovo_supply_register(struct device *parent,
		const struct lenovo_supply_desc *desc,
		const struct lenovo_supply_config *cfg)
{
	return __lenovo_supply_register(parent, desc, cfg, true);
}
EXPORT_SYMBOL_GPL(lenovo_supply_register);

/**
 * lenovo_supply_register_no_ws() - Register new non-waking-source power supply
 * @parent:	Device to be a parent of power supply's device, usually
 *		the device which probe function calls this
 * @desc:	Description of power supply, must be valid through whole
 *		lifetime of this power supply
 * @cfg:	Run-time specific configuration accessed during registering,
 *		may be NULL
 *
 * Return: A pointer to newly allocated lenovo_supply on success
 * or ERR_PTR otherwise.
 * Use lenovo_supply_unregister() on returned lenovo_supply pointer to release
 * resources.
 */
struct lenovo_supply *__must_check
lenovo_supply_register_no_ws(struct device *parent,
		const struct lenovo_supply_desc *desc,
		const struct lenovo_supply_config *cfg)
{
	return __lenovo_supply_register(parent, desc, cfg, false);
}
EXPORT_SYMBOL_GPL(lenovo_supply_register_no_ws);

static void devm_lenovo_supply_release(struct device *dev, void *res)
{
	struct lenovo_supply **psy = res;

	lenovo_supply_unregister(*psy);
}

/**
 * devm_lenovo_supply_register() - Register managed power supply
 * @parent:	Device to be a parent of power supply's device, usually
 *		the device which probe function calls this
 * @desc:	Description of power supply, must be valid through whole
 *		lifetime of this power supply
 * @cfg:	Run-time specific configuration accessed during registering,
 *		may be NULL
 *
 * Return: A pointer to newly allocated lenovo_supply on success
 * or ERR_PTR otherwise.
 * The returned lenovo_supply pointer will be automatically unregistered
 * on driver detach.
 */
struct lenovo_supply *__must_check
devm_lenovo_supply_register(struct device *parent,
		const struct lenovo_supply_desc *desc,
		const struct lenovo_supply_config *cfg)
{
	struct lenovo_supply **ptr, *psy;

	ptr = devres_alloc(devm_lenovo_supply_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	psy = __lenovo_supply_register(parent, desc, cfg, true);
	if (IS_ERR(psy)) {
		devres_free(ptr);
	} else {
		*ptr = psy;
		devres_add(parent, ptr);
	}
	return psy;
}
EXPORT_SYMBOL_GPL(devm_lenovo_supply_register);

/**
 * devm_lenovo_supply_register_no_ws() - Register managed non-waking-source power supply
 * @parent:	Device to be a parent of power supply's device, usually
 *		the device which probe function calls this
 * @desc:	Description of power supply, must be valid through whole
 *		lifetime of this power supply
 * @cfg:	Run-time specific configuration accessed during registering,
 *		may be NULL
 *
 * Return: A pointer to newly allocated lenovo_supply on success
 * or ERR_PTR otherwise.
 * The returned lenovo_supply pointer will be automatically unregistered
 * on driver detach.
 */
struct lenovo_supply *__must_check
devm_lenovo_supply_register_no_ws(struct device *parent,
		const struct lenovo_supply_desc *desc,
		const struct lenovo_supply_config *cfg)
{
	struct lenovo_supply **ptr, *psy;

	ptr = devres_alloc(devm_lenovo_supply_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	psy = __lenovo_supply_register(parent, desc, cfg, false);
	if (IS_ERR(psy)) {
		devres_free(ptr);
	} else {
		*ptr = psy;
		devres_add(parent, ptr);
	}
	return psy;
}
EXPORT_SYMBOL_GPL(devm_lenovo_supply_register_no_ws);

/**
 * lenovo_supply_unregister() - Remove this power supply from system
 * @psy:	Pointer to power supply to unregister
 *
 * Remove this power supply from the system. The resources of power supply
 * will be freed here or on last lenovo_supply_put() call.
 */
void lenovo_supply_unregister(struct lenovo_supply *psy)
{
	WARN_ON(atomic_dec_return(&psy->use_cnt));
	psy->removing = true;
	cancel_work_sync(&psy->changed_work);
	cancel_delayed_work_sync(&psy->deferred_register_work);
	sysfs_remove_link(&psy->dev.kobj, "powers");
	lenovo_supply_remove_hwmon_sysfs(psy);
	psy_unregister_cooler(psy);
	psy_unregister_thermal(psy);
	device_init_wakeup(&psy->dev, false);
	device_unregister(&psy->dev);
}
EXPORT_SYMBOL_GPL(lenovo_supply_unregister);

void *lenovo_supply_get_drvdata(struct lenovo_supply *psy)
{
	return psy->drv_data;
}
EXPORT_SYMBOL_GPL(lenovo_supply_get_drvdata);

static int __init lenovo_supply_class_init(void)
{
	lenovo_supply_class = class_create(THIS_MODULE, "lenovo_supply");

	if (IS_ERR(lenovo_supply_class))
		return PTR_ERR(lenovo_supply_class);

	lenovo_supply_class->dev_uevent = lenovo_supply_uevent;
	lenovo_supply_init_attrs(&lenovo_supply_dev_type);

	return 0;
}

static void __exit lenovo_supply_class_exit(void)
{
	class_destroy(lenovo_supply_class);
}

subsys_initcall(lenovo_supply_class_init);
module_exit(lenovo_supply_class_exit);

MODULE_DESCRIPTION("Universal power supply monitor class");
MODULE_AUTHOR("Ian Molton <spyro@f2s.com>, "
	      "Szabolcs Gyurko, "
	      "Anton Vorontsov <cbou@mail.ru>");
MODULE_LICENSE("GPL");
