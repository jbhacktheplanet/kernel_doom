/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Universal power supply monitor class
 *
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 */

#ifndef __LINUX_LENOVO_SUPPLY_H__
#define __LINUX_LENOVO_SUPPLY_H__

#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>

#ifdef CONFIG_LENOVO_SUPPLY
#define SUPPORT_USER_THERMAL_CASE
#endif

/*
 * All voltages, currents, charges, energies, time and temperatures in uV,
 * µA, µAh, µWh, seconds and tenths of degree Celsius unless otherwise
 * stated. It's driver's job to convert its raw values to units in which
 * this class operates.
 */

/*
 * For systems where the charger determines the maximum battery capacity
 * the min and max fields should be used to present these values to user
 * space. Unused/unknown fields will not appear in sysfs.
 */

enum {
	LENOVO_SUPPLY_STATUS_UNKNOWN = 0,
	LENOVO_SUPPLY_STATUS_CHARGING,
	LENOVO_SUPPLY_STATUS_DISCHARGING,
	LENOVO_SUPPLY_STATUS_NOT_CHARGING,
	LENOVO_SUPPLY_STATUS_FULL,
};

/* What algorithm is the charger using? */
enum {
	LENOVO_SUPPLY_CHARGE_TYPE_UNKNOWN = 0,
	LENOVO_SUPPLY_CHARGE_TYPE_NONE,
	LENOVO_SUPPLY_CHARGE_TYPE_TRICKLE,	/* slow speed */
	LENOVO_SUPPLY_CHARGE_TYPE_FAST,		/* fast speed */
	LENOVO_SUPPLY_CHARGE_TYPE_STANDARD,	/* normal speed */
	LENOVO_SUPPLY_CHARGE_TYPE_ADAPTIVE,	/* dynamically adjusted speed */
	LENOVO_SUPPLY_CHARGE_TYPE_CUSTOM,	/* use CHARGE_CONTROL_* props */
};

enum {
	LENOVO_SUPPLY_HEALTH_UNKNOWN = 0,
	LENOVO_SUPPLY_HEALTH_GOOD,
	LENOVO_SUPPLY_HEALTH_OVERHEAT,
	LENOVO_SUPPLY_HEALTH_DEAD,
	LENOVO_SUPPLY_HEALTH_OVERVOLTAGE,
	LENOVO_SUPPLY_HEALTH_UNSPEC_FAILURE,
	LENOVO_SUPPLY_HEALTH_COLD,
	LENOVO_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE,
	LENOVO_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE,
	LENOVO_SUPPLY_HEALTH_OVERCURRENT,
	LENOVO_SUPPLY_HEALTH_CALIBRATION_REQUIRED,
	LENOVO_SUPPLY_HEALTH_WARM,
	LENOVO_SUPPLY_HEALTH_COOL,
	LENOVO_SUPPLY_HEALTH_HOT,
};

enum {
	LENOVO_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
	LENOVO_SUPPLY_TECHNOLOGY_NiMH,
	LENOVO_SUPPLY_TECHNOLOGY_LION,
	LENOVO_SUPPLY_TECHNOLOGY_LIPO,
	LENOVO_SUPPLY_TECHNOLOGY_LiFe,
	LENOVO_SUPPLY_TECHNOLOGY_NiCd,
	LENOVO_SUPPLY_TECHNOLOGY_LiMn,
};

enum {
	LENOVO_SUPPLY_CAPACITY_LEVEL_UNKNOWN = 0,
	LENOVO_SUPPLY_CAPACITY_LEVEL_CRITICAL,
	LENOVO_SUPPLY_CAPACITY_LEVEL_LOW,
	LENOVO_SUPPLY_CAPACITY_LEVEL_NORMAL,
	LENOVO_SUPPLY_CAPACITY_LEVEL_HIGH,
	LENOVO_SUPPLY_CAPACITY_LEVEL_FULL,
};

enum {
	LENOVO_SUPPLY_SCOPE_UNKNOWN = 0,
	LENOVO_SUPPLY_SCOPE_SYSTEM,
	LENOVO_SUPPLY_SCOPE_DEVICE,
};

enum lenovo_supply_property {
	/* Properties of type `int' */
	LENOVO_SUPPLY_PROP_STATUS = 0,
	LENOVO_SUPPLY_PROP_CHARGE_TYPE,
	LENOVO_SUPPLY_PROP_HEALTH,
	LENOVO_SUPPLY_PROP_PRESENT,
	LENOVO_SUPPLY_PROP_ONLINE,
	LENOVO_SUPPLY_PROP_AUTHENTIC,
	LENOVO_SUPPLY_PROP_TECHNOLOGY,
	LENOVO_SUPPLY_PROP_CYCLE_COUNT,
	LENOVO_SUPPLY_PROP_VOLTAGE_MAX,
	LENOVO_SUPPLY_PROP_VOLTAGE_MIN,
	LENOVO_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	LENOVO_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_VOLTAGE_AVG,
	LENOVO_SUPPLY_PROP_VOLTAGE_OCV,
	LENOVO_SUPPLY_PROP_VOLTAGE_BOOT,
	LENOVO_SUPPLY_PROP_CURRENT_MAX,
	LENOVO_SUPPLY_PROP_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_CURRENT_AVG,
	LENOVO_SUPPLY_PROP_CURRENT_BOOT,
	LENOVO_SUPPLY_PROP_POWER_NOW,
	LENOVO_SUPPLY_PROP_POWER_AVG,
	LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	LENOVO_SUPPLY_PROP_CHARGE_FULL,
	LENOVO_SUPPLY_PROP_CHARGE_EMPTY,
	LENOVO_SUPPLY_PROP_CHARGE_NOW,
	LENOVO_SUPPLY_PROP_CHARGE_AVG,
	LENOVO_SUPPLY_PROP_CHARGE_COUNTER,
	LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	LENOVO_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	LENOVO_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, /* in percents! */
	LENOVO_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, /* in percents! */
	LENOVO_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	LENOVO_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	LENOVO_SUPPLY_PROP_INPUT_POWER_LIMIT,
	LENOVO_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
	LENOVO_SUPPLY_PROP_ENERGY_FULL,
	LENOVO_SUPPLY_PROP_ENERGY_EMPTY,
	LENOVO_SUPPLY_PROP_ENERGY_NOW,
	LENOVO_SUPPLY_PROP_ENERGY_AVG,
	LENOVO_SUPPLY_PROP_CAPACITY, /* in percents! */
	LENOVO_SUPPLY_PROP_CAPACITY_ALERT_MIN, /* in percents! */
	LENOVO_SUPPLY_PROP_CAPACITY_ALERT_MAX, /* in percents! */
	LENOVO_SUPPLY_PROP_CAPACITY_ERROR_MARGIN, /* in percents! */
	LENOVO_SUPPLY_PROP_CAPACITY_LEVEL,
	LENOVO_SUPPLY_PROP_TEMP,
	LENOVO_SUPPLY_PROP_TEMP_MAX,
	LENOVO_SUPPLY_PROP_TEMP_MIN,
	LENOVO_SUPPLY_PROP_TEMP_ALERT_MIN,
	LENOVO_SUPPLY_PROP_TEMP_ALERT_MAX,
	LENOVO_SUPPLY_PROP_TEMP_AMBIENT,
	LENOVO_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
	LENOVO_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
	LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW,
	LENOVO_SUPPLY_PROP_TIME_TO_FULL_AVG,
	LENOVO_SUPPLY_PROP_TYPE, /* use lenovo_supply.type instead */
	LENOVO_SUPPLY_PROP_USB_TYPE,
	LENOVO_SUPPLY_PROP_SCOPE,
	LENOVO_SUPPLY_PROP_PRECHARGE_CURRENT,
	LENOVO_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	LENOVO_SUPPLY_PROP_CALIBRATE,
	LENOVO_SUPPLY_PROP_MANUFACTURE_YEAR,
	LENOVO_SUPPLY_PROP_MANUFACTURE_MONTH,
	LENOVO_SUPPLY_PROP_MANUFACTURE_DAY,
	/* Properties of type `const char *' */
	LENOVO_SUPPLY_PROP_MODEL_NAME,
	LENOVO_SUPPLY_PROP_MANUFACTURER,
	LENOVO_SUPPLY_PROP_SERIAL_NUMBER,
#ifdef CONFIG_PRODUCT_DOOM
	LENOVO_SUPPLY_PROP_INPUT_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	LENOVO_SUPPLY_PROP_CHARGING_ENABLED,
	LENOVO_SUPPLY_PROP_INPUT_SUSPEND,
	LENOVO_SUPPLY_PROP_SC_BATTERY_PRESENT,
	LENOVO_SUPPLY_PROP_SC_VBUS_PRESENT,
	LENOVO_SUPPLY_PROP_SC_BATTERY_VOLTAGE,
	LENOVO_SUPPLY_PROP_SC_BATTERY_CURRENT,
	LENOVO_SUPPLY_PROP_SC_BATTERY_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SC_BUS_VOLTAGE,
	LENOVO_SUPPLY_PROP_SC_BUS_CURRENT,
	LENOVO_SUPPLY_PROP_SC_BUS_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SC_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SC_ALARM_STATUS,
	LENOVO_SUPPLY_PROP_SC_VBUS_ERROR_STATUS,
	LENOVO_SUPPLY_PROP_SC_FAULT_STATUS,
	LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SOH,
	LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL,
	LENOVO_SUPPLY_PROP_CHIP_FW,
	LENOVO_SUPPLY_PROP_FG1_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_FG1_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_FG1_SOH,
	LENOVO_SUPPLY_PROP_FG1_TI_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_FG1_CHARGE_FULL,
	LENOVO_SUPPLY_PROP_FG1_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_FG1_CHARGE_COUNTER,
	LENOVO_SUPPLY_PROP_FG1_CYCLE_COUNT,
	LENOVO_SUPPLY_PROP_FG1_TIME_TO_EMPTY_NOW,
	LENOVO_SUPPLY_PROP_FG1_TIME_TO_FULL_NOW,
	LENOVO_SUPPLY_PROP_FG1_CHIP_FW,
	LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE,
	LENOVO_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	LENOVO_SUPPLY_PROP_ADSP_CHG_PPS_ST,
	LENOVO_SUPPLY_PROP_HLOS_CHG_ONLINE,
	LENOVO_SUPPLY_PROP_HLOS_CHG_PPS_ST,
	LENOVO_SUPPLY_PROP_REAL_TYPE,
	LENOVO_SUPPLY_PROP_SEC_BATTERY_VOLTAGE,
	LENOVO_SUPPLY_PROP_SEC_SYSTEM_VOLTAGE,
	LENOVO_SUPPLY_PROP_SEC_BUS_VOLTAGE,
	LENOVO_SUPPLY_PROP_SEC_BATTERY_CURRENT,
	LENOVO_SUPPLY_PROP_SEC_ICL_LIMIT,
	LENOVO_SUPPLY_PROP_SEC_FCC_LIMIT,
	LENOVO_SUPPLY_PROP_SEC_ICL_NOW,
	LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE,
	LENOVO_SUPPLY_PROP_HLOS_GET_CP_STEP,
	LENOVO_SUPPLY_PROP_PROTECT_DATA,
#ifdef SUPPORT_USER_THERMAL_CASE
	LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE,
	LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LIMIT,
	LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_SPEAKER,
	LENOVO_SUPPLY_PROP_THERM_SPEAKER_LIMIT,
	LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_MODEM_5G,
	LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LIMIT,
	LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_CAMERA,
	LENOVO_SUPPLY_PROP_THERM_CAMERA_LIMIT,
#endif
	LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE,
	LENOVO_SUPPLY_PROP_USER_HEALTH_CHARGE,
	LENOVO_SUPPLY_PROP_USER_CHARGING_ENABLED,
	LENOVO_SUPPLY_PROP_USER_INPUT_SUSPEND,
	LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE,
	LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE,
	LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR,
	LENOVO_SUPPLY_PROP_FAC_TEST_ENABLE,
	LENOVO_SUPPLY_PROP_FIXPDO1_CURR,
	LENOVO_SUPPLY_PROP_FIXPDO2_CURR,
	LENOVO_SUPPLY_PROP_FIXPDO2_MAX_POWER,
	LENOVO_SUPPLY_PROP_TYPEC_ORIENT,
	LENOVO_SUPPLY_PROP_SET_FV,
	LENOVO_SUPPLY_PROP_FG1_READ_ENABLE,
	LENOVO_SUPPLY_PROP_CHARGE_THERMAL_STATUS,
	LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_EVENT,
	LENOVO_SUPPLY_PROP_THERM_EVENT_LIMIT,
	LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST,
	LENOVO_SUPPLY_PROP_TOTLA_MAX, /* max */
#endif
};

enum lenovo_supply_type {
	LENOVO_SUPPLY_TYPE_UNKNOWN = 0,
	LENOVO_SUPPLY_TYPE_BATTERY,
	LENOVO_SUPPLY_TYPE_UPS,
	LENOVO_SUPPLY_TYPE_MAINS,
	LENOVO_SUPPLY_TYPE_USB,			/* Standard Downstream Port */
	LENOVO_SUPPLY_TYPE_USB_DCP,		/* Dedicated Charging Port */
	LENOVO_SUPPLY_TYPE_USB_CDP,		/* Charging Downstream Port */
	LENOVO_SUPPLY_TYPE_USB_ACA,		/* Accessory Charger Adapters */
	LENOVO_SUPPLY_TYPE_USB_TYPE_C,		/* Type C Port */
	LENOVO_SUPPLY_TYPE_USB_PD,		/* Power Delivery Port */
	LENOVO_SUPPLY_TYPE_USB_PD_DRP,		/* PD Dual Role Port */
	LENOVO_SUPPLY_TYPE_APPLE_BRICK_ID,	/* Apple Charging Method */
#ifdef CONFIG_PRODUCT_DOOM
	LENOVO_SUPPLY_TYPE_USB_HVDCP,            /* High Voltage DCP */
	LENOVO_SUPPLY_TYPE_SC,
#endif
	LENOVO_SUPPLY_TYPE_WIRELESS,		/* Wireless */
};

enum lenovo_supply_usb_type {
	LENOVO_SUPPLY_USB_TYPE_UNKNOWN = 0,
	LENOVO_SUPPLY_USB_TYPE_SDP,		/* Standard Downstream Port */
	LENOVO_SUPPLY_USB_TYPE_DCP,		/* Dedicated Charging Port */
	LENOVO_SUPPLY_USB_TYPE_CDP,		/* Charging Downstream Port */
	LENOVO_SUPPLY_USB_TYPE_ACA,		/* Accessory Charger Adapters */
	LENOVO_SUPPLY_USB_TYPE_C,		/* Type C Port */
	LENOVO_SUPPLY_USB_TYPE_PD,		/* Power Delivery Port */
	LENOVO_SUPPLY_USB_TYPE_PD_DRP,		/* PD Dual Role Port */
	LENOVO_SUPPLY_USB_TYPE_PD_PPS,		/* PD Programmable Power Supply */
	LENOVO_SUPPLY_USB_TYPE_APPLE_BRICK_ID,	/* Apple Charging Method */
};

enum lenovo_supply_notifier_events {
	LENOVO_PSY_EVENT_PROP_CHANGED,
};

union lenovo_supply_propval {
	int intval;
	const char *strval;
};

struct device_node;
struct lenovo_supply;

/* Run-time specific power supply configuration */
struct lenovo_supply_config {
	struct device_node *of_node;
	struct fwnode_handle *fwnode;

	/* Driver private data */
	void *drv_data;

	/* Device specific sysfs attributes */
	const struct attribute_group **attr_grp;

	char **supplied_to;
	size_t num_supplicants;
};

/* Description of power supply */
struct lenovo_supply_desc {
	const char *name;
	enum lenovo_supply_type type;
	enum lenovo_supply_usb_type *usb_types;
	size_t num_usb_types;
	enum lenovo_supply_property *properties;
	size_t num_properties;

	/*
	 * Functions for drivers implementing power supply class.
	 * These shouldn't be called directly by other drivers for accessing
	 * this power supply. Instead use lenovo_supply_*() functions (for
	 * example lenovo_supply_get_property()).
	 */
	int (*get_property)(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    union lenovo_supply_propval *val);
	int (*set_property)(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    const union lenovo_supply_propval *val);
	/*
	 * property_is_writeable() will be called during registration
	 * of power supply. If this happens during device probe then it must
	 * not access internal data of device (because probe did not end).
	 */
	int (*property_is_writeable)(struct lenovo_supply *psy,
				     enum lenovo_supply_property psp);
	void (*external_power_changed)(struct lenovo_supply *psy);
	void (*set_charged)(struct lenovo_supply *psy);

	/*
	 * Set if thermal zone should not be created for this power supply.
	 * For example for virtual supplies forwarding calls to actual
	 * sensors or other supplies.
	 */
	bool no_thermal;
	/* For APM emulation, think legacy userspace. */
	int use_for_apm;
};

struct lenovo_supply {
	const struct lenovo_supply_desc *desc;

	char **supplied_to;
	size_t num_supplicants;

	char **supplied_from;
	size_t num_supplies;
	struct device_node *of_node;

	/* Driver private data */
	void *drv_data;

	/* private */
	struct device dev;
	struct work_struct changed_work;
	struct delayed_work deferred_register_work;
	spinlock_t changed_lock;
	bool changed;
	bool initialized;
	bool removing;
	atomic_t use_cnt;
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *tcd;
#ifdef SUPPORT_USER_THERMAL_CASE
	struct thermal_zone_device *tzd_display_rate;
	struct thermal_cooling_device *tcd_display_rate;
	struct thermal_zone_device *tzd_speaker;
	struct thermal_cooling_device *tcd_speaker;
	struct thermal_zone_device *tzd_modem_5g;
	struct thermal_cooling_device *tcd_modem_5g;
	struct thermal_zone_device *tzd_camera;
	struct thermal_cooling_device *tcd_camera;
	struct thermal_zone_device *tzd_therm_event;
	struct thermal_cooling_device *tcd_therm_event;
#endif
#endif

#ifdef CONFIG_LEDS_TRIGGERS
	struct led_trigger *charging_full_trig;
	char *charging_full_trig_name;
	struct led_trigger *charging_trig;
	char *charging_trig_name;
	struct led_trigger *full_trig;
	char *full_trig_name;
	struct led_trigger *online_trig;
	char *online_trig_name;
	struct led_trigger *charging_blink_full_solid_trig;
	char *charging_blink_full_solid_trig_name;
#endif
};

/*
 * This is recommended structure to specify static power supply parameters.
 * Generic one, parametrizable for different power supplies. Power supply
 * class itself does not use it, but that's what implementing most platform
 * drivers, should try reuse for consistency.
 */

struct lenovo_supply_info {
	const char *name;
	int technology;
	int voltage_max_design;
	int voltage_min_design;
	int charge_full_design;
	int charge_empty_design;
	int energy_full_design;
	int energy_empty_design;
	int use_for_apm;
};

struct lenovo_supply_battery_ocv_table {
	int ocv;	/* microVolts */
	int capacity;	/* percent */
};

struct lenovo_supply_resistance_temp_table {
	int temp;	/* celsius */
	int resistance;	/* internal resistance percent */
};

#define LENOVO_SUPPLY_OCV_TEMP_MAX 20

/*
 * This is the recommended struct to manage static battery parameters,
 * populated by lenovo_supply_get_battery_info(). Most platform drivers should
 * use these for consistency.
 * Its field names must correspond to elements in enum lenovo_supply_property.
 * The default field value is -EINVAL.
 * Power supply class itself doesn't use this.
 */

struct lenovo_supply_battery_info {
	int energy_full_design_uwh;	    /* microWatt-hours */
	int charge_full_design_uah;	    /* microAmp-hours */
	int voltage_min_design_uv;	    /* microVolts */
	int voltage_max_design_uv;	    /* microVolts */
	int precharge_current_ua;	    /* microAmps */
	int charge_term_current_ua;	    /* microAmps */
	int constant_charge_current_max_ua; /* microAmps */
	int constant_charge_voltage_max_uv; /* microVolts */
	int factory_internal_resistance_uohm;   /* microOhms */
	int ocv_temp[LENOVO_SUPPLY_OCV_TEMP_MAX];/* celsius */
	struct lenovo_supply_battery_ocv_table *ocv_table[LENOVO_SUPPLY_OCV_TEMP_MAX];
	int ocv_table_size[LENOVO_SUPPLY_OCV_TEMP_MAX];
	struct lenovo_supply_resistance_temp_table *resist_table;
	int resist_table_size;
};

extern struct atomic_notifier_head lenovo_supply_notifier;
extern int lenovo_supply_reg_notifier(struct notifier_block *nb);
extern void lenovo_supply_unreg_notifier(struct notifier_block *nb);
extern struct lenovo_supply *lenovo_supply_get_by_name(const char *name);
extern void lenovo_supply_put(struct lenovo_supply *psy);
#ifdef CONFIG_OF
extern struct lenovo_supply *lenovo_supply_get_by_phandle(struct device_node *np,
							const char *property);
extern struct lenovo_supply *devm_lenovo_supply_get_by_phandle(
				    struct device *dev, const char *property);
#else /* !CONFIG_OF */
static inline struct lenovo_supply *
lenovo_supply_get_by_phandle(struct device_node *np, const char *property)
{ return NULL; }
static inline struct lenovo_supply *
devm_lenovo_supply_get_by_phandle(struct device *dev, const char *property)
{ return NULL; }
#endif /* CONFIG_OF */

extern int lenovo_supply_get_battery_info(struct lenovo_supply *psy,
					 struct lenovo_supply_battery_info *info);
extern void lenovo_supply_put_battery_info(struct lenovo_supply *psy,
					  struct lenovo_supply_battery_info *info);
extern int lenovo_supply_ocv2cap_simple(struct lenovo_supply_battery_ocv_table *table,
				       int table_len, int ocv);
extern struct lenovo_supply_battery_ocv_table *
lenovo_supply_find_ocv2cap_table(struct lenovo_supply_battery_info *info,
				int temp, int *table_len);
extern int lenovo_supply_batinfo_ocv2cap(struct lenovo_supply_battery_info *info,
					int ocv, int temp);
extern int
lenovo_supply_temp2resist_simple(struct lenovo_supply_resistance_temp_table *table,
				int table_len, int temp);
extern void lenovo_supply_changed(struct lenovo_supply *psy);
extern int lenovo_supply_am_i_supplied(struct lenovo_supply *psy);
extern int lenovo_supply_set_input_current_limit_from_supplier(
					 struct lenovo_supply *psy);
extern int lenovo_supply_set_battery_charged(struct lenovo_supply *psy);

//#ifdef CONFIG_LENOVO_SUPPLY
#if 1
extern int lenovo_supply_is_system_supplied(void);
#else
static inline int lenovo_supply_is_system_supplied(void) { return -ENOSYS; }
#endif

extern int lenovo_supply_get_property(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    union lenovo_supply_propval *val);
extern int lenovo_supply_set_property(struct lenovo_supply *psy,
			    enum lenovo_supply_property psp,
			    const union lenovo_supply_propval *val);
extern int lenovo_supply_property_is_writeable(struct lenovo_supply *psy,
					enum lenovo_supply_property psp);
extern void lenovo_supply_external_power_changed(struct lenovo_supply *psy);

extern struct lenovo_supply *__must_check
lenovo_supply_register(struct device *parent,
				 const struct lenovo_supply_desc *desc,
				 const struct lenovo_supply_config *cfg);
extern struct lenovo_supply *__must_check
lenovo_supply_register_no_ws(struct device *parent,
				 const struct lenovo_supply_desc *desc,
				 const struct lenovo_supply_config *cfg);
extern struct lenovo_supply *__must_check
devm_lenovo_supply_register(struct device *parent,
				 const struct lenovo_supply_desc *desc,
				 const struct lenovo_supply_config *cfg);
extern struct lenovo_supply *__must_check
devm_lenovo_supply_register_no_ws(struct device *parent,
				 const struct lenovo_supply_desc *desc,
				 const struct lenovo_supply_config *cfg);
extern void lenovo_supply_unregister(struct lenovo_supply *psy);
extern int lenovo_supply_powers(struct lenovo_supply *psy, struct device *dev);

#define to_lenovo_supply(device) container_of(device, struct lenovo_supply, dev)

extern void *lenovo_supply_get_drvdata(struct lenovo_supply *psy);
/* For APM emulation, think legacy userspace. */
extern struct class *lenovo_supply_class;

static inline bool lenovo_supply_is_amp_property(enum lenovo_supply_property psp)
{
	switch (psp) {
	case LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case LENOVO_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
	case LENOVO_SUPPLY_PROP_CHARGE_FULL:
	case LENOVO_SUPPLY_PROP_CHARGE_EMPTY:
	case LENOVO_SUPPLY_PROP_CHARGE_NOW:
	case LENOVO_SUPPLY_PROP_CHARGE_AVG:
	case LENOVO_SUPPLY_PROP_CHARGE_COUNTER:
	case LENOVO_SUPPLY_PROP_PRECHARGE_CURRENT:
	case LENOVO_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case LENOVO_SUPPLY_PROP_CURRENT_MAX:
	case LENOVO_SUPPLY_PROP_CURRENT_NOW:
	case LENOVO_SUPPLY_PROP_CURRENT_AVG:
	case LENOVO_SUPPLY_PROP_CURRENT_BOOT:
		return 1;
	default:
		break;
	}

	return 0;
}

static inline bool lenovo_supply_is_watt_property(enum lenovo_supply_property psp)
{
	switch (psp) {
	case LENOVO_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case LENOVO_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
	case LENOVO_SUPPLY_PROP_ENERGY_FULL:
	case LENOVO_SUPPLY_PROP_ENERGY_EMPTY:
	case LENOVO_SUPPLY_PROP_ENERGY_NOW:
	case LENOVO_SUPPLY_PROP_ENERGY_AVG:
	case LENOVO_SUPPLY_PROP_VOLTAGE_MAX:
	case LENOVO_SUPPLY_PROP_VOLTAGE_MIN:
	case LENOVO_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case LENOVO_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
	case LENOVO_SUPPLY_PROP_VOLTAGE_NOW:
	case LENOVO_SUPPLY_PROP_VOLTAGE_AVG:
	case LENOVO_SUPPLY_PROP_VOLTAGE_OCV:
	case LENOVO_SUPPLY_PROP_VOLTAGE_BOOT:
	case LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case LENOVO_SUPPLY_PROP_POWER_NOW:
		return 1;
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_LENOVO_SUPPLY_HWMON
int lenovo_supply_add_hwmon_sysfs(struct lenovo_supply *psy);
void lenovo_supply_remove_hwmon_sysfs(struct lenovo_supply *psy);
#else
static inline int lenovo_supply_add_hwmon_sysfs(struct lenovo_supply *psy)
{
	return 0;
}

static inline
void lenovo_supply_remove_hwmon_sysfs(struct lenovo_supply *psy) {}
#endif

#endif /* __LINUX_LENOVO_SUPPLY_H__ */
