/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This package demos the architecture of bq2589x driver used for linux
 * based system. It also contains the code to tune the output of an adjustable
 * adjust high voltage adapter(AHVDCP) dynamically to achieve better charging efficiency.
 *									[DISCLAIMER]
 * The code is designed for DEMO purpose only, and may be subject to change
 * for any bug fix or improvement without proir notice.
 * TI could offer help to port and debug customer code derived from the demo code,
 * but it is customer's responsibility to assure the code quality and reliability
 * to meet their application requirement.
 */

//#define pr_fmt(fmt)	"bq2589x: %s: " fmt, __func__
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/lenovo_supply.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#include "bq2589x_reg.h"
#include "bq2589x_temp_map.h"

#ifdef CONFIG_PRODUCT_DOOM
#include <linux/soc/qcom/battery_charger.h>
#endif

#define DBG_FS
//#undef	DBG_FS

#ifdef CONFIG_PRODUCT_DOOM
#include "../sc/pd_policy_manager.h"
extern struct usbpd_pm *bq_usbpd_pm;
#define LENOVE_WORK_MS 1000
#define BATT_DEFAULT_TEMP				250
#define BATT_HOT_TEMP					650
#define BATT_NTC_TIMEOUT				10000  // 30s
static int last_temp = BATT_DEFAULT_TEMP;
unsigned long ntc_jiffies = -1;
static int g_online = 0;
int boot_flag;

int first_port_online;
extern bool fusb_orient_en;
extern int usb1_otg_en;
extern int usb2_otg_en;
extern int gpio_otg1_status;
extern int gpio_otg2_status;
extern bool fusb_pd_enable;
bool is_protect_data;
extern int audio_switch_state;
extern int usb1_audio_en;
bool is_batt2_thermal_stop;
extern int g_charge_thermal_level;
extern int g_charge_thermal_fcc;

enum typec_connect_value {
	TYPEC_PORT_VALUE_UNKNOWN = 0,
	TYPEC_PORT_VALUE_FIR,
	TYPEC_PORT_VALUE_SEC,
	TYPEC_PORT_VALUE_BOTH,
};

#define FFC_THREAD_0C               0
#define FFC_THREAD_10C              100
#define FFC_THREAD_15C              150
#define FFC_THREAD_30C              300
#define FFC_THREAD_45C              450
#define FFC_THREAD_60C              600

enum temp_level {
	TH_COLD_THRESHOLD_BL0C,
	TH_COLD_THRESHOLD_0T10C,
	TH_COOL_THRESHOLD_10T15C,
	TH_COOL_THRESHOLD_15T30C,
	TH_GOOD_THRESHOLD_30T45C,
	TH_WARM_THRESHOLD_45T60C,
	TH_HOT_THRESHOLD_UP60C,
};

#define FFC_CP_STEP_VOLT  4250000

#define FFC_PD_MAX_CURRENT_2P5A		2500
#define FFC_PD_MAX_CURRENT_3A			3000
#define FFC_PD_MAX_CURRENT_3P5A		3500
#define FFC_PD_MAX_CURRENT_4P5A		4500
#define FFC_PD_MAX_CURRENT_5A			5000
#define FFC_PD_MAX_CURRENT_5P3A		5300
#define FFC_PD_MAX_CURRENT_6A			6000
#define FFC_PD_MAX_CURRENT_7A			7000
#define FFC_PD_MAX_CURRENT_7P5A		7500
#define FFC_PD_MAX_CURRENT_8A			8000
#define FFC_PD_MAX_CURRENT_8P5A		8500

#define LENOVO_FFC_CTRL_WORK_MS 500
int cp_max_curr;
#endif

/*USBIN CURRENT */
#define USB_BQ2589X_USBIN_CURRENT		250
#define DCP_BQ2589X_USBIN_CURRENT		100
#define CDP_BQ2589X_USBIN_CURRENT		250
#define HVDCP_BQ2589X_USBIN_CURRENT		100

/*BATT FCC CURRENT*/
#define FCC_BQ2589X_TYPE_USB_CURRENT		250
#define FCC_BQ2589X_TYPE_DCP_CURRENT		250
#define FCC_BQ2589X_TYPE_CDP_CURRENT		250
#define FCC_BQ2589X_TYPE_HVDCP_CURRENT		500

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no {
	BQ25890 = 0x03,
};

/* availabe on bq25898D bq25890H
enum bq2589x_dp_volt {
	BQ2589X_DP_HIZ,
	BQ2589X_DP_0MV,
	BQ2589X_DP_600MV,
	BQ2589X_DP_1200MV,
	BQ2589X_DP_2000MV,
	BQ2589X_DP_2700MV,
	BQ2589X_DP_3300MV,
	BQ2589X_DP_SHORT
};
enum bq2589x_dm_volt {
	BQ2589X_DM_HIZ,
	BQ2589X_DM_0MV,
	BQ2589X_DM_600MV,
	BQ2589X_DM_1200MV,
	BQ2589X_DM_2000MV,
	BQ2589X_DM_2700MV,
	BQ2589X_DM_3300MV,
	BQ2589X_DM_3300MV2,
};
*/
enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
	BATT_TUNE	= BIT(4),
	SYS_OFF         	= BIT(5),
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

enum bq2589x_charge_state {
	CHARGE_STATE_IDLE = BQ2589X_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = BQ2589X_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = BQ2589X_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = BQ2589X_CHRG_STAT_CHGDONE,
};

#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2589x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

struct bq2589x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq2589x_config {
	bool enable_auto_dpdm;
	bool enable_12v;
	bool enable_hvdcp;
	bool enable_maxc;

	int	charge_voltage;
	int	charge_current;
	int boost_voltage;
	int boost_current;

	bool enable_term;
	int	term_current;

	bool enable_ico;
	bool use_absolute_vindpm;

	int dpdm_sw_gpio;
	int usb_id_gpio;
};


struct bq2589x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2589x_part_no part_no;
	int revision;

	struct bq2589x_config cfg;

	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex profile_change_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	struct bq2589x_wakeup_source bq2589x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool usb_present;
	bool charge_enabled;
	bool otg_enabled;

	bool power_good;
	bool vbus_good;

	bool batt_full;
	bool batt_present;

	int	vbus_type;
	int charge_state;
	int charging_disabled_status;

	enum lenovo_supply_type usb_type;
	enum lenovo_supply_usb_type psy_usb_type;

	int chg_ma;
	int chg_mv;
	int icl_ma;
	int ivl_mv;

/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int chg_curr;

	int fault_status;

	int skip_writes;
	int skip_reads;
#ifdef CONFIG_PRODUCT_DOOM
/* charge switch power gpio*/
	int charge_1t1_gpio;
	struct pinctrl *charge_1t1_gpio_pinctrl;
	struct pinctrl_state *charge_1t1_gpio_pinctrl_state_active;
	struct pinctrl_state *charge_1t1_gpio_pinctrl_state_suspend;
	int charge_1t2_gpio;
	struct pinctrl *charge_1t2_gpio_pinctrl;
	struct pinctrl_state *charge_1t2_gpio_pinctrl_state_active;
	struct pinctrl_state *charge_1t2_gpio_pinctrl_state_suspend;

	int charge_2t1_gpio;
	struct pinctrl *charge_2t1_gpio_pinctrl;
	struct pinctrl_state *charge_2t1_gpio_pinctrl_state_active;
	struct pinctrl_state *charge_2t1_gpio_pinctrl_state_suspend;

	int charge_2t2_gpio;
	struct pinctrl *charge_2t2_gpio_pinctrl;
	struct pinctrl_state *charge_2t2_gpio_pinctrl_state_active;
	struct pinctrl_state *charge_2t2_gpio_pinctrl_state_suspend;
/*monitor two ports connect status use work*/
	struct delayed_work	lenovo_monitor_ports_status_work;
	enum typec_connect_value	ports_insert_value;

	int user_health_charge;
	int user_charging_enabled;
	int user_input_suspend;
	struct delayed_work	lenovo_battery_monitor_work;
	struct delayed_work	lenovo_thermal_monitor_work;
	struct delayed_work	lenovo_charge_monitor_work;
	struct delayed_work	lenovo_insert_delay_work;
	struct delayed_work	lenovo_bootup_delay_work;
	struct delayed_work	lenovo_wr_pdo_bootup_delay_work;
	struct delayed_work	lenovo_icl_settled_work;
	struct delayed_work	lenovo_restart_pd_work;
	struct delayed_work	 lenovo_ffc_ctrl_work;
	struct lenovo_supply	*master_fg_psy;
	struct lenovo_supply	*slave_fg_psy;
	struct lenovo_supply	*bq2589x_psy;
	int			first_main_icl_ua;
	int			first_main_fcc_ua;
	int			first_batt_max_fcc_ua;
	int			first_batt_max_fv_uv;
	int			first_request_fcc_ua;
	int			first_request_fv_uv;
	int			first_request_icl_ua;
	int			sec_main_icl_ua;
	int			sec_main_fcc_ua;
	int			sec_main_fv_uv;
	int			sec_batt_max_fcc_ua;
	int			sec_batt_max_fv_uv;
	int			sec_request_fcc_ua;
	int			sec_request_fv_uv;
	int			sec_request_icl_ua;
	int			max_icl_settled_ua;
	int			sin_port_max_power;
	int			icl_settled_ready;
	int			usb_vbus_uv;
	int			restart_pd_status;
	int			charge_therm_fcc_ua;
	int			total_main_fcc_ua;
	int			total_main_icl_ua;
	int			port1_bus_5v_icl_ua;
	int			port1_bus_9v_icl_ua;
	int			port2_bus_5v_icl_ua;
	int			port2_bus_9v_icl_ua;
	int			batt1_cap_full;
	int			batt2_cap_full;
	int			fac_test_enable;
	bool			wake_lock;
	int			charge_thermal_status;
	int			adsp_icl;
	int			adsp_fcc;
	int			adsp_fv;
	int			adsp_cp_thermal;
	int			typec_connect_value;
#ifdef CONFIG_BATTERY_FAC
	int			port1_fac_pd_en;
	int			port2_fac_pd_en;
#endif
#endif
	struct delayed_work ico_work;
	struct delayed_work pe_work;

	struct delayed_work discharge_jeita_work;
	struct delayed_work charge_jeita_work;

	struct alarm jeita_alarm;

	struct dentry *debug_root;

	struct bq2589x_otg_regulator otg_vreg;

	//struct lenovo_supply batt_psy;
	struct lenovo_supply_desc batt_psy;
	struct lenovo_supply_config batt_cfg;
	struct lenovo_supply *fc_main_psy;
	struct power_supply *usb_psy;
	struct power_supply *qcom_batt_psy;
	struct lenovo_supply     *ffc_25890h_batt_psy;
	const struct regulator_desc *batt_init_desc;

};

struct pe_ctrl {
	bool enable;
	bool tune_up;
	bool tune_down;
	bool tune_done;
	bool tune_fail;
	int  tune_count;
	int  target_volt;
	int	 high_volt_level;/* vbus volt > this threshold means tune up successfully */
	int  low_volt_level; /* vbus volt < this threshold means tune down successfully */
	int  vbat_min_volt;  /* to tune up voltage only when vbat > this threshold */
};


#ifdef CONFIG_PRODUCT_DOOM
static void bq2589x_convert_vbus_type(struct bq2589x *bq);
static int smblib_get_first_online(struct bq2589x *chg,
				    union lenovo_supply_propval *val);
static int smblib_get_second_online(struct bq2589x *chg,
				    union lenovo_supply_propval *val);
#endif

static int get_adsp_vbus_en(struct bq2589x *bq)
{
	int rc = 0;
	union power_supply_propval val = {0, };

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy)
			return 0;
	}

	rc = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_ORIENT,
				&val);
	if (rc < 0) {
		pr_err("Couldn't get qcom vbus connect, rc=%d\n", rc);
		return 0;
	}

	if ((val.intval) && (!usb1_otg_en) && (!usb1_audio_en))
		val.intval = 1;
	else
		val.intval = 0;

	return val.intval;
}

static int get_adsp_typec_cc_st(struct bq2589x *bq)
{
	int rc = 0;
	union power_supply_propval val = {0, };

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy)
			return 0;
	}

	rc = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_ORIENT,
				&val);
	if (rc < 0) {
		pr_err("faild get adsp typec orient, rc=%d\n", rc);
		return 0;
	}

	return val.intval;
}

#ifdef CONFIG_PRODUCT_DOOM
static int lenovo_glink_read(enum lenovo_supply_property prop, int *val)
{
	int rc;
	int data;

        rc = qti_battery_charger_get_ext_prop("battery", prop, &data);
        if (rc < 0) {
                pr_err("Failed to get adsp battery data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
	*val = data;

	return 0;
}

static int lenovo_glink_write(enum lenovo_supply_property prop, int val)
{
	int rc;
	int data;

	data = val;
        rc = qti_battery_charger_set_ext_prop("battery", prop, data);
        if (rc < 0) {
                pr_err("Failed to set adsp battery data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }

	return 0;
}

static int lenovo_glink_usb_read(enum lenovo_supply_property prop, int *val)
{
	int rc;
	int data;

        rc = qti_battery_charger_get_ext_prop("usb", prop, &data);
        if (rc < 0) {
                pr_err("Failed to get adsp usb data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
	*val = data;

	return 0;
}

static int lenovo_glink_usb_write(enum lenovo_supply_property prop, int val)
{
	int rc;
	int data;

	data = val;
        rc = qti_battery_charger_set_ext_prop("usb", prop, data);
        if (rc < 0) {
                pr_err("Failed to set adsp usb data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }

	return 0;
}

static int get_adsp_chg_enable(struct bq2589x *bq)
{
	int ret;
	int chg_en = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_read(LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE, &chg_en);
        if (ret < 0) {
                pr_err("Failed to get adsp charger status, ret=%d\n", ret);
                return -EINVAL;
	}
	return chg_en;
}

static int set_adsp_chg_enable(struct bq2589x *bq, int val)
{
	int ret;
	int chg_en = 0;

	if (bq == NULL)
                return -EINVAL;
	chg_en = val;
        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE, chg_en);
        if (ret < 0) {
                pr_err("Failed to set adsp charger status, ret=%d\n", ret);
                return -EINVAL;
	}
	return 0;
}

static int get_adsp_chg_pps_st(struct bq2589x *bq)
{
	int ret;
	int pps_en = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_read(LENOVO_SUPPLY_PROP_ADSP_CHG_PPS_ST, &pps_en);
        if (ret < 0) {
                pr_err("Failed to get adsp charger pps status, ret=%d\n", ret);
                return -EINVAL;
	}
	pr_info("successful get adsp_pps_en is %d\n", pps_en);
	return pps_en;
}

static int get_hlos_chg_online(struct bq2589x *bq)
{
	int sec_chg_en = 0;

	if (bq == NULL)
                return -EINVAL;
	sec_chg_en = fusb_orient_en;

        if (sec_chg_en < 0) {
                pr_err("Failed to get sec port charger online status\n");
                return -EINVAL;
	}

	if (sec_chg_en && (usb2_otg_en == 0) && ((audio_switch_state == 255) || (audio_switch_state != 0)))
		sec_chg_en = 1;
	else
		sec_chg_en = 0;
	return sec_chg_en;
}

static int get_hlos_chg_pps_st(struct bq2589x *bq)
{
	int sec_pps_en = 0;

	if (bq == NULL)
                return -EINVAL;
	sec_pps_en = fusb_pd_enable;

        if (sec_pps_en < 0) {
                pr_err("Failed to get sec port charger pps status\n");
                return -EINVAL;
	}
	pr_info("successful get sec_pps_en is %d\n", sec_pps_en);
	return sec_pps_en;
}

static int get_adsp_usb_suspend(struct bq2589x *bq)
{
	int ret;
	int usb_suspend_en = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_usb_read(LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE, &usb_suspend_en);
        if (ret < 0) {
                pr_err("Failed to get adsp usb suspend status, ret=%d\n", ret);
                return -EINVAL;
	}
	return usb_suspend_en;
}

static int set_adsp_usb_suspend(struct bq2589x *bq, int val)
{
	int ret;
	int usb_suspend_en = 0;

	if (bq == NULL)
                return -EINVAL;
	usb_suspend_en = val;
        ret = lenovo_glink_usb_write(LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE, usb_suspend_en);
        if (ret < 0) {
                pr_err("Failed to set adsp usb suspend status, ret=%d\n", ret);
                return -EINVAL;
	}
	return 0;
}

static int get_adsp_cp_step(struct bq2589x *bq)
{
	int ret;
	int cp_step = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_usb_read(LENOVO_SUPPLY_PROP_HLOS_GET_CP_STEP, &cp_step);
        if (ret < 0) {
                pr_err("Failed to get adsp cp step, ret=%d\n", ret);
                return -EINVAL;
	}
	return cp_step;
}

static int get_adsp_real_type(struct bq2589x *bq,
			union lenovo_supply_propval *val)
{
	if (bq == NULL)
                return -EINVAL;

        val->strval = qti_battery_charger_get_real_type("usb");

	return 0;
}

static int set_adsp_fcc_value(struct bq2589x *bq, int val)
{
	int ret;
	int fcc_value = 0;

	if (bq == NULL)
                return -EINVAL;
	fcc_value = val;	/*fcc_value in uA*/
	bq->adsp_fcc = fcc_value;
        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE, fcc_value);
        if (ret < 0) {
                pr_err("HLOS failed to set adsp fcc value, ret=%d\n", ret);
                return -EINVAL;
	}else{
		pr_info("HLOS set adsp fcc value successful val = %d uA\n", fcc_value);
	}
	return 0;
}

static int set_adsp_usb_icl_value(struct bq2589x *bq, int val)
{
	int ret;
	int icl_value = 0;

	if (bq == NULL)
                return -EINVAL;
	icl_value = val;	/*usb_icl_value in mA*/
	bq->adsp_icl = icl_value;
        ret = lenovo_glink_usb_write(LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE, icl_value);
        if (ret < 0) {
                pr_err("HLOS failed to set adsp icl value, ret=%d\n", ret);
                return -EINVAL;
	}else{
		pr_info("HLOS set adsp icl value successful val = %d uA\n", icl_value);
	}
	return 0;
}

static int control_adsp_cp(struct bq2589x *bq, int val)
{
	int ret;
	int cp_st = 0;

	if (bq == NULL)
                return -EINVAL;
	cp_st = val;
	bq->adsp_cp_thermal = cp_st;
        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR, cp_st);
        if (ret < 0) {
                pr_err("HLOS failed to control adsp charge pump, ret=%d\n", ret);
                return -EINVAL;
	}else{
		pr_info("HLOS control adsp charge pump successful cp_st = %d\n", cp_st);
	}
	return 0;
}

static int get_adsp_fixpdo1_current(struct bq2589x *bq)
{
	int ret;
	int fixpdo1_curr = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_usb_read(LENOVO_SUPPLY_PROP_FIXPDO1_CURR, &fixpdo1_curr);
        if (ret < 0) {
                pr_err("Failed to get adsp fixpdo1 current, ret=%d\n", ret);
                return -EINVAL;
	}
	return fixpdo1_curr;
}

static int get_adsp_fixpdo2_current(struct bq2589x *bq)
{
	int ret;
	int fixpdo2_curr = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_usb_read(LENOVO_SUPPLY_PROP_FIXPDO2_CURR, &fixpdo2_curr);
        if (ret < 0) {
                pr_err("Failed to get adsp fixpdo2 current, ret=%d\n", ret);
                return -EINVAL;
	}
	return fixpdo2_curr;
}

static int get_adsp_fixpdo2_max_power(struct bq2589x *bq)
{
	int ret;
	int max_power = 0;

	if (bq == NULL)
                return -EINVAL;

        ret = lenovo_glink_usb_read(LENOVO_SUPPLY_PROP_FIXPDO2_MAX_POWER, &max_power);
        if (ret < 0) {
                pr_err("Failed to get adsp fixpdo2 max power, ret=%d\n", ret);
                return -EINVAL;
	}
	return max_power;
}

static int set_adsp_batt_fv(struct bq2589x *bq, int val)
{
	int ret;
	int fv_mv;

	if (bq == NULL)
                return -EINVAL;
	fv_mv = val;	/*fv value in mV*/
	if (fv_mv == 0)
		fv_mv = 4370;
	bq->adsp_fv = fv_mv;
        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_SET_FV, fv_mv);
        if (ret < 0) {
                pr_err("HLOS failed to set adsp batt fv value, ret=%d\n", ret);
                return -EINVAL;
	}else{
		pr_info("HLOS set adsp batt fv value successful val = %d mV\n", fv_mv);
	}
	return 0;
}

static int send_port2_typec_connect_to_adsp(struct bq2589x *bq, int val)
{
	int ret;
	int port2_connect_val;

	if (bq == NULL)
                return -EINVAL;
	port2_connect_val = val;

	bq->typec_connect_value = port2_connect_val;
        ret = lenovo_glink_usb_write(LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST, port2_connect_val);
        if (ret < 0) {
                pr_err("HLOS failed to send adsp typec connect value, ret=%d\n", ret);
                return -EINVAL;
	}else{
		pr_info("HLOS send port2 typec connect status to adsp, value is %d\n", port2_connect_val);
	}
	return 0;
}
#endif

static struct pe_ctrl pe;

static int bq2589x_update_charging_profile(struct bq2589x *bq);

static int __bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;

}

static int __bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				data, reg, ret);
		return ret;
	}
	return 0;

}

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);

	ret = __bq2589x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2589x_write_byte(bq, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
	}

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

#if 0
static void bq2589x_stay_awake(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2589x_relax(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}
#endif
static void bq2589x_wakeup_src_init(struct bq2589x *bq)
{
	spin_lock_init(&bq->bq2589x_ws.ws_lock);
	//wakeup_source_init(&bq->bq2589x_ws.source, "bq2589x");
}

#if 0
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}
#endif

static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK, val);

}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt)
{
	u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB)
			<< BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
				BQ2589X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 700)
		temp = BQ2589X_BOOST_LIM_700MA;
	else if (curr == 1100)
		temp = BQ2589X_BOOST_LIM_1100MA;
	else if (curr == 1600)
		temp = BQ2589X_BOOST_LIM_1600MA;
	else if (curr == 1800)
		temp = BQ2589X_BOOST_LIM_1800MA;
	else if (curr == 2100)
		temp = BQ2589X_BOOST_LIM_2100MA;
	else if (curr == 2400)
		temp = BQ2589X_BOOST_LIM_2400MA;
	else
		temp = BQ2589X_BOOST_LIM_1300MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
				BQ2589X_BOOST_LIM_MASK,
				temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);


static int bq2589x_enable_hvdcp(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_ENABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_hvdcp);

static int bq2589x_disable_hvdcp(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_DISABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_hvdcp);

static int bq2589x_enable_maxc(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_ENABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_maxc);

static int bq2589x_disable_maxc(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_DISABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_maxc);


static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);

static int bq2589x_get_charge_enable(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_03);
	if (!ret)
		bq->charge_enabled = !!(val & BQ2589X_CHG_CONFIG_MASK);
	return ret;
}

int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK,
					BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
					BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
				BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);


int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);


int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

static void bq2589x_dump_reg2(struct bq2589x *bq)
{
	int ret;
	int addr;
	char data[128];
	int len = 0;
	u8 val;

	memset(data, 0x00, sizeof(data));
	for (addr = 0x0; addr <= 0x14; addr++) {
		msleep(5);
		ret = bq2589x_read_byte(bq, &val, addr);

			sprintf(data + len, "%02X ", val);
			len += 3;
	}
	pr_err("%s\n", data);
}

static int bq_vadc_map_voltage_temp(const struct bq_temp_map *pts,
				      u32 tablesize, s32 input, s64 *output)
{
	bool descending = 1;
	u32 i = 0;

	if (!pts)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending) && (pts[i].x < input)) {
			/* table entry is less than measured*/
			 /* value and table is descending, stop */
			break;
		} else if ((!descending) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured*/
			/*value and table is ascending, stop */
			break;
		}
		i++;
	}

	if (i == 0) {
		*output = pts[0].y;
	} else if (i == tablesize) {
		*output = pts[tablesize - 1].y;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((s32)((pts[i].y - pts[i - 1].y) *
			(input - pts[i - 1].x)) /
			(pts[i].x - pts[i - 1].x)) +
			pts[i - 1].y);
	}

	return 0;
}

int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
	uint8_t val;
	int temp;
	int ret;
	s64 value = 0, result = 0;
	union lenovo_supply_propval pval = {0,};
	int on1, on2, online;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return BATT_DEFAULT_TEMP;
	} else{
		//temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		value = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		value = 21 * 432 * value  * 10000 / (210000000 - 2532 * value);

	        ret = bq_vadc_map_voltage_temp(adcmap_100k_104ef_104fb_1875_vref,
				ARRAY_SIZE(adcmap_100k_104ef_104fb_1875_vref),
				value, &result);
		if (ret) {
			dev_err(bq->dev, "error: vadc_map_temp, reg_10 :0x%02X, value :%lld, ret :%d, temp :%d\n", val, value, ret, result);
			return BATT_DEFAULT_TEMP;
		}

		ret = smblib_get_first_online(bq, &pval);
		if (ret < 0)
			on1 = 0;
		else
			on1 = pval.intval;
		ret = smblib_get_second_online(bq, &pval);
		if (ret < 0)
			on2 = 0;
		else
			on2 = pval.intval;
		online = on1 | on2;
		if (online != g_online) {
			ntc_jiffies = -1;
			g_online= online;
			pr_info("on1=%d, on2=%d, online=%d, g_online=%d\n", on1, on2, online, g_online);
		}

		temp = result;
		if (temp >= BATT_HOT_TEMP) {
			if (ntc_jiffies == -1) {
				temp = last_temp;
				ntc_jiffies = jiffies;
			} else if (time_before(jiffies, ntc_jiffies + msecs_to_jiffies(BATT_NTC_TIMEOUT))) {
				temp = last_temp;
			}
		} else {
			last_temp = temp;
			ntc_jiffies = -1;
		}

		if (result >= BATT_HOT_TEMP) {
			dev_err(bq->dev, "reg_10 :0x%02X, value :%lld, ret :%d, result: %d, temp :%d, last :%d\n",
					val, value, ret, result, temp, last_temp);
			bq2589x_dump_reg2(bq);
		}

		return temp;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

#ifdef CONFIG_PRODUCT_DOOM
int bq2589x_read_config_icl(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret < 0) {
		dev_err(bq->dev, "read config icl failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_IINLIM_BASE + ((val & BQ2589X_IINLIM_MASK) >> BQ2589X_IINLIM_SHIFT) * BQ2589X_IINLIM_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_config_icl);

int bq2589x_read_config_fcc(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_04);
	if (ret < 0) {
		dev_err(bq->dev, "read config fcc failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHG_BASE + ((val & BQ2589X_ICHG_MASK) >> BQ2589X_ICHG_SHIFT) * BQ2589X_ICHG_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_config_fcc);
#endif

int bq2589x_set_chargecurrent(struct bq2589x *bq, int curr)
{
	u8 ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);


int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06,
						BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);


int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;
	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D,
						BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK,
						val << BQ2589X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

/*
int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;
	if (offset == 400)
		val = BQ2589X_VINDPMOS_400MV;
	else
		val = BQ2589X_VINDPMOS_600MV;
	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK,
						val << BQ2589X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);
*/

int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		pr_err("Failed to read register 0x0b:%d\n",ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

void bq2589x_set_otg(struct bq2589x *bq, bool enable)
{
	int ret;

	if (enable)
		ret = bq2589x_enable_otg(bq);
	else
		ret = bq2589x_disable_otg(bq);

	if (!ret)
		bq->otg_enabled = enable;
	else
		dev_err(bq->dev, "%s:Failed to %s otg:%d\n", __func__,
						enable ? "enable" : "disable", ret);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);

int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	u8 val;

	val = (timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB;
	val <<= BQ2589X_WDT_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
						BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_FORCE_DPDM_MASK, val);

	pr_info("Force DPDM %s\n", !ret ? "successfully" : "failed");

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);

int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14,
						BQ2589X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_ship_mode(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_BATFET_DIS_MASK, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);

int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

int bq2589x_pumpx_enable(struct bq2589x *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(bq2589x_check_ico_done);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

static int bq2589x_enable_12v_handshake(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ENABLE_12V << BQ2589X_EN12V_SHIFT;
	else
		val = BQ2589X_DISABLE_12V << BQ2589X_EN12V_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_01,
						BQ2589X_EN12V_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_12v_handshake);

static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_use_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_use_absolute_vindpm);

static int bq2589x_enable_ico(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ico);

static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

#if 0
static bool bq2589x_is_charge_done(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);
#endif

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	bq2589x_disable_watchdog_timer(bq);

	if (bq->cfg.enable_hvdcp)
		bq2589x_enable_hvdcp(bq);
	else
		bq2589x_disable_hvdcp(bq);

	if (bq->cfg.enable_maxc)
		bq2589x_enable_maxc(bq);
	else
		bq2589x_disable_maxc(bq);

	bq2589x_enable_12v_handshake(bq, bq->cfg.enable_12v);
	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	bq2589x_enable_ico(bq, bq->cfg.enable_ico);

	/*force use absolute vindpm if auto_dpdm not enabled*/
	if (!bq->cfg.enable_auto_dpdm)
		bq->cfg.use_absolute_vindpm = true;
	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);

	bq->chg_ma = bq->cfg.charge_current;
	bq->chg_mv = bq->cfg.charge_voltage;

/*
 * ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		pr_err("Failed to set vindpm offset:%d\n",  ret);
	}
*/
	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		pr_err("Failed to set termination current:%d\n",  ret);
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n",  ret);
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n",  ret);
	}

	ret = bq2589x_set_otg_volt(bq, bq->cfg.boost_voltage);
	if (ret < 0) {
		pr_err("Failed to set boost voltage:%d\n", ret);
	}

	ret = bq2589x_set_otg_current(bq, bq->cfg.boost_current);
	if (ret < 0) {
		pr_err("Failed to set boost current:%d\n", ret);
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		pr_err("Failed to enable charger:%d\n",  ret);
		return ret;
	} else {
		bq->charge_enabled = true;
	}

	if (pe.enable) {
		ret = bq2589x_pumpx_enable(bq, true);
		if (ret < 0)
			pr_err("Failed to enable pumpx\n");
	}

	bq2589x_adc_start(bq, false);


	return 0;
}

#if 0
static int bq2589x_charge_status(struct bq2589x *bq)
{
	u8 val = 0;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return LENOVO_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return LENOVO_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return LENOVO_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return LENOVO_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}
#endif
static int bq2589x_charging_disable(struct bq2589x *bq, int reason,
						int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2589x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2589x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static void bq2589x_dump_status(struct bq2589x* bq);
static inline bool is_device_suspended(struct bq2589x *bq);

static int bq2589x_get_prop_charge_type(struct bq2589x *bq)
{
	u8 val = 0;
	if (is_device_suspended(bq))
		return LENOVO_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case CHARGE_STATE_FASTCHG:
		return LENOVO_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHG:
		return LENOVO_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_CHGDONE:
	case CHARGE_STATE_IDLE:
		return LENOVO_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return LENOVO_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2589x_get_prop_charge_status(struct bq2589x *bq)
{
	int ret;
	u8 status;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		return 	LENOVO_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state =
		(status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	switch(bq->charge_state) {
		case CHARGE_STATE_FASTCHG:
		case CHARGE_STATE_PRECHG:
			return LENOVO_SUPPLY_STATUS_CHARGING;
		case CHARGE_STATE_CHGDONE:
			return LENOVO_SUPPLY_STATUS_NOT_CHARGING;
		case CHARGE_STATE_IDLE:
			return LENOVO_SUPPLY_STATUS_DISCHARGING;
		default:
			return 	LENOVO_SUPPLY_STATUS_UNKNOWN;
	}

}
#if 0
static int bq2589x_get_prop_health(struct bq2589x *bq)
{
	int ret;

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot)
				ret = LENOVO_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = LENOVO_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = LENOVO_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = LENOVO_SUPPLY_HEALTH_COLD;
		} else {
			ret = LENOVO_SUPPLY_HEALTH_GOOD;
		}
	}
	ret = LENOVO_SUPPLY_HEALTH_GOOD;
	return ret;
}
#endif

static enum lenovo_supply_property bq2589x_charger_props[] = {
		LENOVO_SUPPLY_PROP_CHARGE_TYPE,
		LENOVO_SUPPLY_PROP_PRESENT,
		LENOVO_SUPPLY_PROP_CHARGING_ENABLED,
		LENOVO_SUPPLY_PROP_STATUS,
		LENOVO_SUPPLY_PROP_INPUT_SUSPEND,
		LENOVO_SUPPLY_PROP_TEMP,
#ifdef CONFIG_PRODUCT_DOOM
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
		LENOVO_SUPPLY_PROP_CHARGE_THERMAL_STATUS,
		LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST,
#endif
};


static int bq2589x_charger_get_property(struct lenovo_supply *psy,
			enum lenovo_supply_property psp,
			union lenovo_supply_propval *val)
{

	//struct bq2589x *bq = container_of(psy, struct bq2589x, batt_psy);
	struct bq2589x *bq = lenovo_supply_get_drvdata(psy);

	int ret;
	u8 hiz;
		if (bq == NULL){
			pr_err("tianye:2589x is null\n");
		return 0;
		}
	switch (psp) {
	case LENOVO_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_get_prop_charge_type(bq);
		pr_debug("LENOVO_SUPPLY_PROP_BQ_CHARGE_TYPE:%d\n", val->intval);
		break;
	case LENOVO_SUPPLY_PROP_CHARGING_ENABLED:
		bq2589x_get_charge_enable(bq);
		val->intval = bq->charge_enabled;
		break;
	case LENOVO_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_prop_charge_status(bq);
		break;
	case LENOVO_SUPPLY_PROP_PRESENT:
		break;
	case LENOVO_SUPPLY_PROP_INPUT_SUSPEND:
		ret = bq2589x_get_hiz_mode(bq, &hiz);
		if (!ret)
			val->intval = hiz;
		break;
	case LENOVO_SUPPLY_PROP_TEMP:
		ret = bq2589x_adc_read_temperature(bq);
		val->intval = ret;
		break;
#ifdef CONFIG_PRODUCT_DOOM
	case LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE:
		ret = get_adsp_chg_enable(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
		val->intval = get_adsp_vbus_en(bq);
		break;
	case LENOVO_SUPPLY_PROP_ADSP_CHG_PPS_ST:
		ret = get_adsp_chg_pps_st(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_HLOS_CHG_ONLINE:
		ret = get_hlos_chg_online(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_HLOS_CHG_PPS_ST:
		ret = get_hlos_chg_pps_st(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_REAL_TYPE:
		bq2589x_convert_vbus_type(bq);
		val->intval = bq->psy_usb_type;
		break;
	case LENOVO_SUPPLY_PROP_SEC_BATTERY_VOLTAGE:
		val->intval = bq2589x_adc_read_battery_volt(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_BUS_VOLTAGE:
		val->intval = bq2589x_adc_read_vbus_volt(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_BATTERY_CURRENT:
		val->intval = bq2589x_adc_read_charge_current(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_SYSTEM_VOLTAGE:
		val->intval = bq2589x_adc_read_sys_volt(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_ICL_LIMIT:
		val->intval = bq2589x_read_config_icl(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_FCC_LIMIT:
		val->intval = bq2589x_read_config_fcc(bq);
		break;
	case LENOVO_SUPPLY_PROP_SEC_ICL_NOW:
		val->intval = bq2589x_read_idpm_limit(bq);
		break;
	case LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE:
		ret = get_adsp_usb_suspend(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_HLOS_GET_CP_STEP:
		ret = get_adsp_cp_step(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_PROTECT_DATA:
		val->intval = is_protect_data;
		break;
	case LENOVO_SUPPLY_PROP_USER_HEALTH_CHARGE:
		val->intval = bq->user_health_charge;
		pr_info("get user health charge %d\n", bq->user_health_charge);
		break;
	case LENOVO_SUPPLY_PROP_USER_CHARGING_ENABLED:
		val->intval = bq->user_charging_enabled;
		pr_info("get user charging enabled %d\n", bq->user_charging_enabled);
		break;
	case LENOVO_SUPPLY_PROP_USER_INPUT_SUSPEND:
		val->intval = bq->user_input_suspend;
		pr_info("get user input suspend %d\n", bq->user_input_suspend);
		break;
	case LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE:
		ret = get_adsp_real_type(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_FAC_TEST_ENABLE:
		val->intval = bq->fac_test_enable;
		break;
	case LENOVO_SUPPLY_PROP_FIXPDO1_CURR:
		ret = get_adsp_fixpdo1_current(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_FIXPDO2_CURR:
		ret = get_adsp_fixpdo2_current(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_FIXPDO2_MAX_POWER:
		ret = get_adsp_fixpdo2_max_power(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_TYPEC_ORIENT:
		ret = get_adsp_typec_cc_st(bq);
		val->intval = ret;
		break;
	case LENOVO_SUPPLY_PROP_CHARGE_THERMAL_STATUS:
		val->intval = bq->charge_thermal_status;
		break;
#if 0
	case LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE:
		val->intval = bq->adsp_icl;
		break;
	case LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE:
		val->intval = bq->adsp_fcc;
		break;
	case LENOVO_SUPPLY_PROP_SET_FV:
		val->intval = bq->adsp_fv;
		break;
	case LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR:
		val->intval = bq->adsp_cp_thermal;
		break;
#endif
#endif
	default:
		return -EINVAL;

	}
	return 0;
}

static int bq2589x_charger_set_property(struct lenovo_supply *psy,
					enum lenovo_supply_property psp,
					const union lenovo_supply_propval *val)
{
	//struct bq2589x *bq = container_of(psy, struct bq2589x, batt_psy);
#ifdef CONFIG_PRODUCT_DOOM
	int ret;
#endif
	struct bq2589x *bq = lenovo_supply_get_drvdata(psy);

	switch (psp) {
	case LENOVO_SUPPLY_PROP_CHARGING_ENABLED:
		pr_err(" LENOVO_SUPPLY_PROP_BQ_CHARGING_ENABLED: %d\n", val->intval);
		bq2589x_charging_disable(bq, USER, !val->intval);
		lenovo_supply_changed(bq->fc_main_psy);
		break;
	case LENOVO_SUPPLY_PROP_INPUT_SUSPEND:
		pr_err(" LENOVO_SUPPLY_PROP_INPUT_SUSPEND: %d\n", val->intval);
		if (val->intval)
			bq2589x_enter_hiz_mode(bq);
		else
			bq2589x_exit_hiz_mode(bq);
		break;
#ifdef CONFIG_PRODUCT_DOOM
	case LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE:
		pr_err(" LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE: %d\n", val->intval);
		ret = set_adsp_chg_enable(bq, val->intval);
		if (ret)
			pr_info("set adsp fail\n");
		break;
	case LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE:
		pr_err(" LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE: %d\n", val->intval);
		ret = set_adsp_usb_suspend(bq, val->intval);
		if (ret)
			pr_info("set adsp usb suspend fail\n");
		break;
	case LENOVO_SUPPLY_PROP_PROTECT_DATA:
		is_protect_data = val->intval;
		pr_info("protect,is_protect_data is %d\n", is_protect_data);
		break;
	case LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE:
		ret = set_adsp_fcc_value(bq, val->intval);
		if (ret)
			pr_info("set adsp usb suspend fail\n");
		break;
	case LENOVO_SUPPLY_PROP_USER_HEALTH_CHARGE:
		bq->user_health_charge = val->intval;
		pr_info("set user health charge to %d\n", bq->user_health_charge);
		break;
	case LENOVO_SUPPLY_PROP_USER_CHARGING_ENABLED:
		bq->user_charging_enabled = val->intval;
		pr_info("set user charging enabled to %d\n", bq->user_charging_enabled);
		cancel_delayed_work_sync(&bq->lenovo_charge_monitor_work);
		schedule_delayed_work(&bq->lenovo_charge_monitor_work,
							msecs_to_jiffies(0));
		break;
	case LENOVO_SUPPLY_PROP_USER_INPUT_SUSPEND:
		bq->user_input_suspend = val->intval;
		pr_info("set user input suspend to %d\n", bq->user_input_suspend);
		break;
	case	LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE:
		ret = set_adsp_usb_icl_value(bq, val->intval);
		if (ret)
			pr_info("set adsp usb icl fail\n");
		break;
	case	LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR:
		ret = control_adsp_cp(bq, val->intval);
		if (ret)
			pr_info("ctrl adsp cp fail\n");
		break;
	case LENOVO_SUPPLY_PROP_FAC_TEST_ENABLE:
		bq->fac_test_enable = val->intval;
		pr_info("set factory test enable %d\n", bq->fac_test_enable);
		break;
	case LENOVO_SUPPLY_PROP_SET_FV:
		ret = set_adsp_batt_fv(bq, val->intval);
		if (ret)
			pr_err("set adsp batt fv fail %d\n", val->intval);
		else
			pr_info("set battery float voltage %d\n", val->intval);
		break;
	case LENOVO_SUPPLY_PROP_CHARGE_THERMAL_STATUS:
		bq->charge_thermal_status = val->intval;
		break;
	case LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST:
		ret = send_port2_typec_connect_to_adsp(bq, val->intval);
		if (ret)
			pr_info("send port2 typec connect fail\n");
		else
			pr_err("send port2 typec connect  value is %d\n", val->intval);
		break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2589x_charger_is_writeable(struct lenovo_supply *psy,
					enum lenovo_supply_property prop)
{
	int ret;

	switch(prop) {
	case LENOVO_SUPPLY_PROP_CHARGING_ENABLED:
	case LENOVO_SUPPLY_PROP_INPUT_SUSPEND:
#ifdef CONFIG_PRODUCT_DOOM
	case	LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE:
	case	LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE:
	case	LENOVO_SUPPLY_PROP_PROTECT_DATA:
	case	LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE:
	case	LENOVO_SUPPLY_PROP_USER_HEALTH_CHARGE:
	case	LENOVO_SUPPLY_PROP_USER_CHARGING_ENABLED:
	case	LENOVO_SUPPLY_PROP_USER_INPUT_SUSPEND:
	case	LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE:
	case LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR:
	case LENOVO_SUPPLY_PROP_SET_FV:
	case LENOVO_SUPPLY_PROP_CHARGE_THERMAL_STATUS:
	case LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST:
#endif
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int bq2589x_psy_register(struct bq2589x *bq)
{
	int ret;
	bq->batt_cfg.drv_data = bq;
	bq->batt_cfg.of_node = bq->dev->of_node;
	ret = 0;
	bq->batt_psy.name = "bq2589h-charger";
	bq->batt_psy.type = LENOVO_SUPPLY_TYPE_MAINS;
	bq->batt_psy.properties = bq2589x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->batt_psy.get_property = bq2589x_charger_get_property;
	bq->batt_psy.set_property = bq2589x_charger_set_property;
	//bq->batt_psy.external_power_changed = NULL;//bq2589x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2589x_charger_is_writeable;

	bq->fc_main_psy = devm_lenovo_supply_register(bq->dev,
			&bq->batt_psy, &bq->batt_cfg);

	if (IS_ERR(bq->fc_main_psy)) {
		pr_err("failed to register fc_main_psy:%d\n", ret);
		return PTR_ERR(bq->fc_main_psy);
	}
#if 0
	ret = lenovo_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}
#endif
	return 0;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
	lenovo_supply_unregister(bq->fc_main_psy);
}

static int bq2589x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_enable_otg(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2589x OTG mode Enabled!\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_disable_otg(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2589x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_03);
	if (ret)
		return ret;
	enabled = ((status & BQ2589X_OTG_CONFIG_MASK) >> BQ2589X_OTG_CONFIG_SHIFT);

	return (enabled == BQ2589X_OTG_ENABLE) ? 1 : 0;

}


struct regulator_ops bq2589x_otg_reg_ops = {
	.enable		= bq2589x_otg_regulator_enable,
	.disable	= bq2589x_otg_regulator_disable,
	.is_enabled = bq2589x_otg_regulator_is_enable,
};

static int bq2589x_regulator_init(struct bq2589x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node,bq->batt_init_desc);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2589x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25890");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2589x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf,"%x %x",&reg, &val);
	if (ret == 2 && reg <= 0x14) {
		bq2589x_write_byte(bq,(unsigned char)reg,(unsigned char)val);
	}

	return count;
}

static DEVICE_ATTR(registers, 0660, bq2589x_show_registers, bq2589x_store_register);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "ti,bq2589x,dpdm-switch-gpio",
								&bq->cfg.dpdm_sw_gpio);
    if(ret)
		pr_err("Failed to read node of ti,bq2589x,dpdm-switch-gpio\n");

	bq->cfg.usb_id_gpio = of_get_named_gpio(np, "ti,usbid-gpio",0);
	if (bq->cfg.usb_id_gpio < 0) {
		pr_err("usb_id_gpio is not avaiable\n");
	}

	if (pe.enable) {
		ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-high-level",
									&pe.high_volt_level);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbus-volt-high-level\n");


		ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-low-level",
									&pe.low_volt_level);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbus-volt-low-level\n");

		ret = of_property_read_u32(np, "ti,bq2589x,vbat-min-volt-to-tuneup",
									&pe.vbat_min_volt);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbat-min-volt-to-tuneup\n");
	}

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np,
								"ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np,
								"ti,bq2589x,enable-termination");
	bq->cfg.enable_ico = of_property_read_bool(np,
								"ti,bq2589x,enable-ico");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np,
								"ti,bq2589x,use-absolute-vindpm");
	bq->cfg.enable_12v = of_property_read_bool(np,
								"ti,bq2589x,enable-12v");
	bq->cfg.enable_hvdcp = of_property_read_bool(np,
								"ti,bq2589x,enable-hvdcp");
	bq->cfg.enable_maxc = of_property_read_bool(np,
								"ti,bq2589x,enable-maxc");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",
								&bq->cfg.charge_voltage);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-voltage\n");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",
								&bq->cfg.charge_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-current\n");

	ret = of_property_read_u32(np, "ti,bq2589x,termination-current",
								&bq->cfg.term_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,termination-current\n");

	return 0;
}

#if  0
static int bq2589x_parse_jeita_dt(struct device *dev, struct bq2589x* bq)
{
    struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-hot-degc",
						&bq->batt_hot_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-degc",
						&bq->batt_warm_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-degc",
						&bq->batt_cool_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-degc\n");
		return ret;
	}
	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cold-degc",
						&bq->batt_cold_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-hot-hysteresis",
						&bq->hot_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cold-hysteresis",
						&bq->cold_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-ma",
						&bq->batt_cool_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-mv",
						&bq->batt_cool_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-ma",
						&bq->batt_warm_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-mv",
						&bq->batt_warm_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported =
		of_property_read_bool(np,"ti,bq2589x,software-jeita-supported");

	return 0;
}
#endif
static void bq2589x_init_jeita(struct bq2589x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	//bq2589x_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}

static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	int ret;

	/* wait for new adc data */
	msleep(1000);
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	if (vbus_volt < 6000)
		bq->ivl_mv = vbus_volt - 600;
	else
		bq->ivl_mv = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);

	pr_err("Set absolute vindpm threshold %s\n",
			!ret ? "Successfully" : "Failed");

}


static int bq2589x_update_charging_profile(struct bq2589x *bq)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;

#ifdef CONFIG_PRODUCT_DOOM
return 0;
#endif
	if (!bq->usb_present)
		return 0;

	pr_info("chg_mv:%d,chg_ma:%d,icl:%d,ivl:%d\n",
					bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);


	mutex_lock(&bq->profile_change_lock);

	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		chg_ma = bq->chg_ma;
		chg_mv = bq->chg_mv;
	}

	icl = bq->icl_ma;

	/*TODO: add therm_lvl_sel*/

	pr_info("chg_mv:%d, chg_ma:%d, icl:%d, ivl:%d\n",
				chg_mv, chg_ma, icl, bq->ivl_mv);

	ret = bq2589x_set_chargevoltage(bq, bq->chg_mv);
	if (ret < 0)
		pr_err("Failed to set charge current:%d\n", ret);

	ret = bq2589x_set_chargecurrent(bq, bq->chg_ma);
	if (ret < 0)
		pr_err("Failed to set charge current:%d\n", ret);

	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("failed to set input volt limit:%d\n", ret);

	ret = bq2589x_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0)
		pr_err("failed to set input current limit:%d\n", ret);

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}

#ifdef CONFIG_PRODUCT_DOOM
int bq2589x_dynamic_update_charging_profile(struct bq2589x *chg, int chg_uv, int chg_ua, int icl_ua, int ivl_uv)
{
	int ret;

	pr_debug("[CHARGE] profile chg_uv:%d,chg_ua:%d,icl:%d,ivl:%d\n",
					chg_uv, chg_ua, icl_ua, ivl_uv);

	if (chg_uv > 0) {
		ret = bq2589x_set_chargevoltage(chg, chg_uv/1000);
		if (ret < 0)
			pr_err("[CHARGE] Failed to set charge current:%d\n", ret);
	}

	ret = bq2589x_set_chargecurrent(chg, chg_ua/1000);
	if (ret < 0)
		pr_err("[CHARGE] Failed to set charge current:%d\n", ret);

	ret = bq2589x_set_input_volt_limit(chg, ivl_uv/1000);
	if (ret < 0)
		pr_err("[CHARGE] failed to set input volt limit:%d\n", ret);

	ret = bq2589x_set_input_current_limit(chg, icl_ua/1000);
	if (ret < 0)
		pr_err("[CHARGE] failed to set input current limit:%d\n", ret);

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_dynamic_update_charging_profile);
#endif

static void bq2589x_start_pe_handshake(struct bq2589x *bq, bool up)
{

	if (!pe.enable)
		return;

	if (up) {
		pe.target_volt = pe.high_volt_level;
		pe.tune_up = true;
		pe.tune_down = false;
	} else {
		pe.target_volt = pe.low_volt_level;
		pe.tune_up = false;
		pe.tune_down = true;
	}
	pe.tune_done = false;
	pe.tune_count = 0;
	pe.tune_fail = false;

	//disable charger, enable it again after voltage tune done
//	bq2589x_charging_disable(bq, BATT_TUNE, true);
	schedule_delayed_work(&bq->pe_work, 0);
}



static void bq2589x_convert_vbus_type(struct bq2589x *bq)
{
#ifdef CONFIG_PRODUCT_DOOM
	int ret;
	u8 status = 0;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		pr_err("read vbus type error : %d\n", ret);
		bq->vbus_type = BQ2589X_VBUS_UNKNOWN;
	} else {
		bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	}
#endif

	switch (bq->vbus_type) {
	case BQ2589X_VBUS_USB_SDP:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB;
		break;
	case BQ2589X_VBUS_USB_CDP:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB_CDP;
		break;
	case BQ2589X_VBUS_USB_DCP:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB_DCP;
		break;
	case BQ2589X_VBUS_MAXC:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB_HVDCP;
		break;
	case BQ2589X_VBUS_NONSTAND:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB_ACA;
		break;
	case BQ2589X_VBUS_UNKNOWN:
		bq->usb_type = LENOVO_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		bq->usb_type = LENOVO_SUPPLY_TYPE_UNKNOWN;
		break;
	}

#ifdef CONFIG_PRODUCT_DOOM
	switch (bq->usb_type) {
	case LENOVO_SUPPLY_TYPE_USB:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_SDP;
		break;
	case LENOVO_SUPPLY_TYPE_USB_CDP:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_CDP;
		break;
	case LENOVO_SUPPLY_TYPE_USB_DCP:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_DCP;
		break;
	case LENOVO_SUPPLY_TYPE_USB_HVDCP:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_DCP;
		break;
	case LENOVO_SUPPLY_TYPE_USB_ACA:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_ACA;
		break;
	case LENOVO_SUPPLY_TYPE_UNKNOWN:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	default:
		bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}
#endif
}

static void bq2589x_get_hvdcp_profile(struct bq2589x *bq)
{
	u16 vbus_volt;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt <= 5300) {
		bq->icl_ma = 500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 500;
	} else if (vbus_volt <= 9500) {
		bq->icl_ma = 2000;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	} else {
		bq->icl_ma = 1500;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	}
}

static void bq2589x_adapter_in_handler(struct bq2589x *bq)
{
	int ret;
	bool update_profile = true;

	bq2589x_convert_vbus_type(bq);
#ifdef CONFIG_PRODUCT_DOOM
	pr_err("vbus_type:%d, usb_type:%d, psy_usb_type:%d\n", bq->vbus_type, bq->usb_type, bq->psy_usb_type);
#else
	pr_err("vbus_type:%d, usb_type:%d\n", bq->vbus_type, bq->usb_type);
#endif

	switch (bq->usb_type) {
	case LENOVO_SUPPLY_TYPE_USB:
		bq->icl_ma = USB_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_USB_CURRENT;
		break;
	case LENOVO_SUPPLY_TYPE_USB_CDP:
		bq->icl_ma = CDP_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_CDP_CURRENT;
		break;
	case LENOVO_SUPPLY_TYPE_USB_DCP:
		bq->icl_ma = 2000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 2000;
		bq2589x_start_pe_handshake(bq, true);
		break;
	case LENOVO_SUPPLY_TYPE_USB_HVDCP:
		bq2589x_get_hvdcp_profile(bq);
		break;
	case LENOVO_SUPPLY_TYPE_USB_ACA:
		bq->usb_type = LENOVO_SUPPLY_TYPE_USB_ACA;
		bq->icl_ma = 1000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 1000;
		break;
	case LENOVO_SUPPLY_TYPE_UNKNOWN:
		bq->icl_ma = USB_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_USB_CURRENT;
		break;
	default:
		break;
	}

	if (update_profile)
		bq2589x_update_charging_profile(bq);

	if (bq->usb_type == LENOVO_SUPPLY_TYPE_USB_DCP ||
		bq->usb_type == LENOVO_SUPPLY_TYPE_USB_HVDCP) {
		ret = bq2589x_force_ico(bq);
		if (!ret) {
			schedule_delayed_work(&bq->ico_work, 2 * HZ);
			pr_info("Force ICO successfully\n");
		} else {
			pr_err("Force ICO failed\n");
		}
	}

	cancel_delayed_work(&bq->discharge_jeita_work);

//	bq2589x_set_watchdog_timer(bq, 80);
}

static void bq2589x_adapter_out_handler(struct bq2589x *bq)
{
	//int ret;

	bq2589x_disable_watchdog_timer(bq);

	pr_err("usb removed, set usb present = %d\n", bq->usb_present);
}

static const unsigned char* charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2589x_dump_reg(struct bq2589x *bq)
{
	int ret;
	int addr;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		msleep(5);
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}
#if 1
static void bq2589x_dump_status(struct bq2589x *bq)
{
	int ret;
	u8 status,fault;
	int chg_current;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	chg_current = bq2589x_adc_read_charge_current(bq);

	pr_info("vbus:%d,vbat:%d,ibat:%d\n", bq->vbus_volt,
					bq->vbat_volt, chg_current);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
	if (!ret){
		if (status & BQ2589X_VDPM_STAT_MASK)
			dev_info(bq->dev, "%s:VINDPM occurred\n", __func__);
		if (status & BQ2589X_IDPM_STAT_MASK)
			dev_info(bq->dev, "%s:IINDPM occurred\n", __func__);
	}


	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (!ret) {
		mutex_lock(&bq->data_lock);
		bq->fault_status = fault;
		mutex_unlock(&bq->data_lock);
	}

	if (bq->fault_status & BQ2589X_FAULT_WDT_MASK)
		pr_err("Watchdog timer expired!\n");
	if (bq->fault_status & BQ2589X_FAULT_BOOST_MASK)
		pr_err("Boost fault occurred!\n");

	status = (bq->fault_status & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
	if (status == BQ2589X_FAULT_CHRG_INPUT)
		pr_err("input fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_THERMAL)
		pr_err("charge thermal shutdown fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_TIMER)
		pr_err("charge timer expired fault!\n");

	if (bq->fault_status & BQ2589X_FAULT_BAT_MASK)
		pr_err("battery ovp fault!\n");

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (!ret) {
		bq->charge_state = status & BQ2589X_CHRG_STAT_MASK;
		bq->charge_state >>= BQ2589X_CHRG_STAT_SHIFT;
		pr_err("%s\n", charge_stat_str[bq->charge_state]);
	}

//	bq2589x_dump_reg(bq);
}
#endif

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	int ret;

	ret = bq2589x_check_ico_done(bq);

	if (ret == 1) {
		ret = bq2589x_read_idpm_limit(bq);
		if (ret < 0)
			pr_info("ICO done, but failed to read idmp limit:%d\n", ret);
		else
			pr_info("ICO done, idpm limit = %dmA\n", ret);
	} else {
		schedule_delayed_work(&bq->ico_work, 2 * HZ);
	}

}

static void bq2589x_pe_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pe_work.work);
	int ret;
	static bool pumpx_cmd_issued;
	ret = 0;
	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	pr_info("VBus:%d, Target:%d\n", bq->vbus_volt, pe.target_volt);

	if ((pe.tune_up && bq->vbus_volt > pe.target_volt) ||
	    (pe.tune_down && bq->vbus_volt < pe.target_volt)) {
		pr_err("voltage tune successfully\n");
		pe.tune_done = true;
		bq2589x_adjust_absolute_vindpm(bq);
		if (pe.tune_up) {
			ret = bq2589x_force_ico(bq);
			if (!ret) {
				schedule_delayed_work(&bq->ico_work, 2 * HZ);
				pr_info("Force ICO successfully\n");
			} else {
				pr_err("Force ICO failed\n");
			}
		}
		return;
	}

	if (pe.tune_count > 5) {
		pr_info("voltage tune failed,reach max retry count\n");
		pe.tune_fail = true;
		bq2589x_adjust_absolute_vindpm(bq);

		if (pe.tune_up) {
			bq2589x_update_charging_profile(bq);
			ret = bq2589x_force_ico(bq);
			if (!ret) {
				schedule_delayed_work(&bq->ico_work, 2 * HZ);
				pr_info("Force ICO successfully\n");
			} else {
				pr_err("Force ICO failed\n");
			}
		}
		return;
	}

	if (!pumpx_cmd_issued) {
		if (pe.tune_up)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if (pe.tune_down)
			ret =  bq2589x_pumpx_decrease_volt(bq);
		if (ret) {
			schedule_delayed_work(&bq->pe_work, HZ);
		} else {
			dev_info(bq->dev, "%s:pumpx command issued.\n", __func__);
			pumpx_cmd_issued = true;
			pe.tune_count++;
			schedule_delayed_work(&bq->pe_work, 3*HZ);
		}
	} else {
		if (pe.tune_up)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if (pe.tune_down)
			ret = bq2589x_pumpx_decrease_volt_done(bq);
		if (ret == 0) {
			dev_info(bq->dev, "%s:pumpx command finishedd!\n", __func__);
			pumpx_cmd_issued = 0;
		}
		schedule_delayed_work(&bq->pe_work, HZ);
	}


}

static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	int ret;
	struct bq2589x *bq = data;
	u8 status = 0;
	u8 fault = 0;

	msleep(5);

	//bq2589x_dump_reg(bq);
	//bq2589x_dump_status(bq);

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	} else {
		mutex_lock(&bq->data_lock);
		bq->fault_status = fault;
		mutex_unlock(&bq->data_lock);
	}

	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;

	if (((bq->vbus_type == BQ2589X_VBUS_NONE) || (bq->vbus_type == BQ2589X_VBUS_OTG))
						&& bq->usb_present) {
		pr_err("adapter removed\n");
		bq->usb_present = false;
		bq2589x_adapter_out_handler(bq);
#ifdef CONFIG_PRODUCT_DOOM
		schedule_work(&bq_usbpd_pm->usb_psy_change_work);
#endif
	} else if (bq->vbus_type != BQ2589X_VBUS_NONE && (bq->vbus_type != BQ2589X_VBUS_OTG)
						&& !bq->usb_present) {
		pr_err("adapter plugged in\n");
		bq->usb_present = true;
		bq2589x_adapter_in_handler(bq);
#ifdef CONFIG_PRODUCT_DOOM
		schedule_work(&bq_usbpd_pm->usb_psy_change_work);
#endif
	}

	mutex_unlock(&bq->irq_complete);
	//lenovo_supply_changed(bq->fc_main_psy);

	return IRQ_HANDLED;
}
#ifdef CONFIG_PRODUCT_DOOM
#define CHARGER_1T1_EN_ACTIVE    "port1_charge_switch_1_to_1_active"
#define CHARGER_1T1_EN_SUSPEND   "port1_charge_switch_1_to_1_suspend"

#define CHARGER_1T2_EN_ACTIVE    "port1_charge_switch_1_to_2_active"
#define CHARGER_1T2_EN_SUSPEND   "port1_charge_switch_1_to_2_suspend"

#define CHARGER_2T1_EN_ACTIVE    "port2_charge_switch_2_to_1_active"
#define CHARGER_2T1_EN_SUSPEND   "port2_charge_switch_2_to_1_suspend"

#define CHARGER_2T2_EN_ACTIVE    "port2_charge_switch_2_to_2_active"
#define CHARGER_2T2_EN_SUSPEND   "port2_charge_switch_2_to_2_suspend"

static int lenovo_charge_path_pinctrl_init(struct bq2589x *chg)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chg->charge_1t1_gpio_pinctrl = devm_pinctrl_get(chg->dev);
	chg->charge_1t2_gpio_pinctrl = devm_pinctrl_get(chg->dev);
	chg->charge_2t1_gpio_pinctrl = devm_pinctrl_get(chg->dev);
	chg->charge_2t2_gpio_pinctrl = devm_pinctrl_get(chg->dev);

	if (IS_ERR_OR_NULL(chg->charge_1t1_gpio_pinctrl)) {
		retval = PTR_ERR(chg->charge_1t1_gpio_pinctrl);
		pr_err("charge_1t1_gpio_pinctrl enable does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	if (IS_ERR_OR_NULL(chg->charge_1t2_gpio_pinctrl)) {
		retval = PTR_ERR(chg->charge_1t2_gpio_pinctrl);
		pr_err("charge_1t2_gpio_pinctrl enable does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	if (IS_ERR_OR_NULL(chg->charge_2t1_gpio_pinctrl)) {
		retval = PTR_ERR(chg->charge_2t1_gpio_pinctrl);
		pr_err("charge_2t1_gpio_pinctrl enable does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	if (IS_ERR_OR_NULL(chg->charge_2t2_gpio_pinctrl)) {
		retval = PTR_ERR(chg->charge_1t1_gpio_pinctrl);
		pr_err("charge_2t2_gpio_pinctrl enable does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}
	/*PORT_1 TO FIRST BATT*/
	chg->charge_1t1_gpio_pinctrl_state_active
			= pinctrl_lookup_state(chg->charge_1t1_gpio_pinctrl, CHARGER_1T1_EN_ACTIVE);

	if (IS_ERR_OR_NULL(chg->charge_1t1_gpio_pinctrl_state_active)) {
		retval = PTR_ERR(chg->charge_1t1_gpio_pinctrl_state_active);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_1T1_EN_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	chg->charge_1t1_gpio_pinctrl_state_suspend
			= pinctrl_lookup_state(chg->charge_1t1_gpio_pinctrl, CHARGER_1T1_EN_SUSPEND);

	if (IS_ERR_OR_NULL(chg->charge_1t1_gpio_pinctrl_state_suspend)) {
		retval = PTR_ERR(chg->charge_1t1_gpio_pinctrl_state_suspend);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_1T1_EN_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	/*PORT_1 TO SECOND BATT*/
	chg->charge_1t2_gpio_pinctrl_state_active
			= pinctrl_lookup_state(chg->charge_1t2_gpio_pinctrl, CHARGER_1T2_EN_ACTIVE);

	if (IS_ERR_OR_NULL(chg->charge_1t2_gpio_pinctrl_state_active)) {
		retval = PTR_ERR(chg->charge_1t2_gpio_pinctrl_state_active);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_1T2_EN_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	chg->charge_1t2_gpio_pinctrl_state_suspend
			= pinctrl_lookup_state(chg->charge_1t2_gpio_pinctrl, CHARGER_1T2_EN_SUSPEND);

	if (IS_ERR_OR_NULL(chg->charge_1t2_gpio_pinctrl_state_suspend)) {
		retval = PTR_ERR(chg->charge_1t2_gpio_pinctrl_state_suspend);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_1T2_EN_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	/*PORT_2 TO FIRST BATT*/
	chg->charge_2t1_gpio_pinctrl_state_active
			= pinctrl_lookup_state(chg->charge_2t1_gpio_pinctrl, CHARGER_2T1_EN_ACTIVE);

	if (IS_ERR_OR_NULL(chg->charge_2t1_gpio_pinctrl_state_active)) {
		retval = PTR_ERR(chg->charge_2t1_gpio_pinctrl_state_active);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_2T1_EN_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	chg->charge_2t1_gpio_pinctrl_state_suspend
			= pinctrl_lookup_state(chg->charge_2t1_gpio_pinctrl, CHARGER_2T1_EN_SUSPEND);

	if (IS_ERR_OR_NULL(chg->charge_2t1_gpio_pinctrl_state_suspend)) {
		retval = PTR_ERR(chg->charge_2t1_gpio_pinctrl_state_suspend);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_2T1_EN_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	/*PORT_2 TO SECOND BATT*/
	chg->charge_2t2_gpio_pinctrl_state_active
			= pinctrl_lookup_state(chg->charge_1t2_gpio_pinctrl, CHARGER_2T2_EN_ACTIVE);

	if (IS_ERR_OR_NULL(chg->charge_2t2_gpio_pinctrl_state_active)) {
		retval = PTR_ERR(chg->charge_2t2_gpio_pinctrl_state_active);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_2T2_EN_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	chg->charge_2t2_gpio_pinctrl_state_suspend
			= pinctrl_lookup_state(chg->charge_2t2_gpio_pinctrl, CHARGER_2T2_EN_SUSPEND);

	if (IS_ERR_OR_NULL(chg->charge_2t2_gpio_pinctrl_state_suspend)) {
		retval = PTR_ERR(chg->charge_2t2_gpio_pinctrl_state_suspend);
		pr_err("charger can not lookup %s pinstate %d\n",
				CHARGER_2T2_EN_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(chg->charge_1t1_gpio_pinctrl);
	devm_pinctrl_put(chg->charge_1t2_gpio_pinctrl);
	devm_pinctrl_put(chg->charge_2t1_gpio_pinctrl);
	devm_pinctrl_put(chg->charge_2t2_gpio_pinctrl);

err_pinctrl_get:
	chg->charge_1t1_gpio_pinctrl = NULL;
	chg->charge_1t2_gpio_pinctrl = NULL;
	chg->charge_2t1_gpio_pinctrl = NULL;
	chg->charge_2t2_gpio_pinctrl = NULL;
	return retval;
}

static void lenovo_init_charger_path_switch_gpio(struct bq2589x *chg)
{
	int ret;

	ret = lenovo_charge_path_pinctrl_init(chg);
	if (!ret) {
/*
* Pinctrl handle is optional. If pinctrl handle is found
* let pins to be configured in active state. If not
* found continue further without error.
*/
		/*gpio 1t1 pinctrl*/
		if (chg->charge_1t1_gpio_pinctrl){
			ret = pinctrl_select_state(chg->charge_1t1_gpio_pinctrl,
					chg->charge_1t1_gpio_pinctrl_state_active);
			if (ret < 0) {
				pr_err("%s: charge failed to select %s pinstate %d\n",
					__func__, CHARGER_1T1_EN_ACTIVE, ret);
			}
		}
		/*gpio 1t2 pinctrl*/
		if (chg->charge_1t2_gpio_pinctrl){
			ret = pinctrl_select_state(chg->charge_1t2_gpio_pinctrl,
					chg->charge_1t2_gpio_pinctrl_state_active);
			if (ret < 0) {
				pr_err("%s: charge failed to select %s pinstate %d\n",
					__func__, CHARGER_1T2_EN_ACTIVE, ret);
			}
		}
		/*gpio 2t1 pinctrl*/
		if (chg->charge_2t1_gpio_pinctrl){
			ret = pinctrl_select_state(chg->charge_2t1_gpio_pinctrl,
					chg->charge_2t1_gpio_pinctrl_state_active);
			if (ret < 0) {
				pr_err("%s: charge failed to select %s pinstate %d\n",
					__func__, CHARGER_2T1_EN_ACTIVE, ret);
			}
		}
		/*gpio 2t2 pinctrl*/
		if (chg->charge_2t2_gpio_pinctrl){
			ret = pinctrl_select_state(chg->charge_2t2_gpio_pinctrl,
					chg->charge_2t2_gpio_pinctrl_state_active);
			if (ret < 0) {
				pr_err("%s: charge failed to select %s pinstate %d\n",
					__func__, CHARGER_2T2_EN_ACTIVE, ret);
			}
		}

		/*gpio 1t1 request*/
		chg->charge_1t1_gpio = of_get_named_gpio(chg->dev->of_node,
				"qcom,port1_1_to_1_en", 0);

		ret = gpio_request(chg->charge_1t1_gpio, "charger_switch_1t1_en");
		if (ret) {
			pr_err("Can't request charge_1t1_gpio en gpio\n", ret);
		return;
		}
		/*gpio 1t2 request*/
		chg->charge_1t2_gpio = of_get_named_gpio(chg->dev->of_node,
				"qcom,port1_1_to_2_en", 0);

		ret = gpio_request(chg->charge_1t2_gpio, "charger_switch_1t2_en");
		if (ret) {
			pr_err("Can't request charge_1t2_gpio en gpio\n", ret);
		return;
		}
		/*gpio 2t1 request*/
		chg->charge_2t1_gpio = of_get_named_gpio(chg->dev->of_node,
				"qcom,port2_2_to_1_en", 0);

		ret = gpio_request(chg->charge_2t1_gpio, "charger_switch_2t1_en");
		if (ret) {
			pr_err("Can't request charge_2t1_gpio en gpio\n", ret);
		return;
		}
		/*gpio 2t2 request*/
		chg->charge_2t2_gpio = of_get_named_gpio(chg->dev->of_node,
				"qcom,port2_2_to_2_en", 0);

		ret = gpio_request(chg->charge_2t2_gpio, "charger_switch_2t2_en");
		if (ret) {
			pr_err("Can't request charge_2t2_gpio en gpio\n", ret);
		return;
		}

		gpio_direction_output(chg->charge_1t1_gpio, 0);
		gpio_direction_output(chg->charge_1t2_gpio, 0);
		gpio_direction_output(chg->charge_2t1_gpio, 0);
		gpio_direction_output(chg->charge_2t2_gpio, 0);

	}
}

int last_st_val = 0;
int new_st_val = 0;
bool port_status_changed_enable(int status_value)
{


	last_st_val = status_value;
	if (last_st_val != new_st_val)
	{
		new_st_val = last_st_val;
		pr_info("port online status changed\n");
		return 1;
	}else{
		pr_info("port online status not changed\n");
		return 0;
	}
}

void lenovo_set_gpios_ctrl(struct bq2589x *chg, bool st_changed)
{

	bool reset_gpio_val;
	reset_gpio_val = st_changed;
	reset_gpio_val = 0;
	pr_info("enter lenovo_set_gpios_ctrl,reset_gpio_val =%d, is_protect_data = %d, boot_flag=%d\n",
		reset_gpio_val, is_protect_data, boot_flag);
	switch(chg->ports_insert_value)
	{
	case TYPEC_PORT_VALUE_FIR:
		 if (reset_gpio_val){
                        if (!is_protect_data && !boot_flag ){
                                gpio_set_value(chg->charge_1t1_gpio, 1);
                                gpio_set_value(chg->charge_1t2_gpio, 1);
                                gpio_set_value(chg->charge_2t1_gpio, 1);
                                gpio_set_value(chg->charge_2t2_gpio, 1);
                                msleep(300);
                        }
			boot_flag =0;
		}
		if (is_protect_data){
                        gpio_set_value(chg->charge_1t1_gpio, 0);
                        gpio_set_value(chg->charge_1t2_gpio, 1);
                        gpio_set_value(chg->charge_2t1_gpio, 1);
                        gpio_set_value(chg->charge_2t2_gpio, 0);
                }else{
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		pr_info("main port charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_SEC:
              if (reset_gpio_val){
                        if (!is_protect_data){
                                gpio_set_value(chg->charge_1t1_gpio, 1);
                                gpio_set_value(chg->charge_1t2_gpio, 1);
                                gpio_set_value(chg->charge_2t1_gpio, 1);
                                gpio_set_value(chg->charge_2t2_gpio, 1);
                                msleep(300);
                        }
                }
                if (is_protect_data){
                        gpio_set_value(chg->charge_1t1_gpio, 0);
                        gpio_set_value(chg->charge_1t2_gpio, 1);
                        gpio_set_value(chg->charge_2t1_gpio, 1);
                        gpio_set_value(chg->charge_2t2_gpio, 0);
                }else{
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		pr_info("sec port charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_BOTH:
		if (reset_gpio_val){
                        if (!is_protect_data){
                                gpio_set_value(chg->charge_1t1_gpio, 1);
                                gpio_set_value(chg->charge_1t2_gpio, 1);
                                gpio_set_value(chg->charge_2t1_gpio, 1);
                                gpio_set_value(chg->charge_2t2_gpio, 1);
                                msleep(450);
                        }
		}
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 0);
		pr_info("two ports charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_UNKNOWN:
		if (reset_gpio_val){
                        gpio_set_value(chg->charge_1t1_gpio, 1);
                        gpio_set_value(chg->charge_1t2_gpio, 1);
                        gpio_set_value(chg->charge_2t1_gpio, 1);
                        gpio_set_value(chg->charge_2t2_gpio, 1);
                        msleep(450);
                }
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 0);
		pr_info("two ports cable disconnect\n");
		break;
	}
}

void enable_otg_charge(struct bq2589x *chg)
{
          if(usb1_otg_en==1&&usb2_otg_en==0){
		pr_info("enable_usb2_charge\n");
		gpio_set_value(chg->charge_1t1_gpio, 1);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->charge_2t1_gpio, 0);
		gpio_set_value(chg->charge_2t2_gpio, 0);
          }
	   else if(usb1_otg_en==0&&usb2_otg_en==1){
	   	pr_info("enable_usb1_charge\n");
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 0);
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 1);
          }
}

static int ffc_get_batt2_vol(struct bq2589x *chg)
{
       int rc;
       union lenovo_supply_propval pval = {0, };

       if (!chg->ffc_25890h_batt_psy) {
               chg->ffc_25890h_batt_psy= lenovo_supply_get_by_name("bq27z561-slave");
               if (!chg->ffc_25890h_batt_psy)
                       return 0;
       }

       rc = lenovo_supply_get_property(chg->ffc_25890h_batt_psy,
                       LENOVO_SUPPLY_PROP_VOLTAGE_NOW, &pval);

       return pval.intval;
}


int ffc_get_batt2_temp_level(struct bq2589x *chg)
{

	int rc;
	union lenovo_supply_propval pval = {0, };

	if (!chg->ffc_25890h_batt_psy) {
		chg->ffc_25890h_batt_psy= lenovo_supply_get_by_name("bq27z561-slave");
		if (!chg->ffc_25890h_batt_psy)
			return 0;
	}

	rc = lenovo_supply_get_property(chg->ffc_25890h_batt_psy,
				LENOVO_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_err("Couldn't get bq2589h battery for ffc, rc=%d\n", rc);
		return 0;
	}
	/*250*/
	pr_info("batt 2 temp is %d\n", pval.intval);

	/*TEMP BELOW 0*/
	if (pval.intval < FFC_THREAD_0C)
		return TH_COLD_THRESHOLD_BL0C;
	/*TEMP 0-10*/
	else if((pval.intval < FFC_THREAD_10C) && (pval.intval >= FFC_THREAD_0C))
		return TH_COLD_THRESHOLD_0T10C;
	/*TEMP 10-15*/
	else if((pval.intval < FFC_THREAD_15C) && (pval.intval >= FFC_THREAD_10C))
		return TH_COOL_THRESHOLD_10T15C;
	/*TEMP 15-30*/
	else if((pval.intval < FFC_THREAD_30C) && (pval.intval >= FFC_THREAD_15C))
		return TH_COOL_THRESHOLD_15T30C;
	/*TEMP 35-45*/
	else if((pval.intval < FFC_THREAD_45C) && (pval.intval >= FFC_THREAD_30C))
		return TH_GOOD_THRESHOLD_30T45C;
	/*TEMP 45-60*/
	else if((pval.intval < FFC_THREAD_60C) && (pval.intval >= FFC_THREAD_45C))
		return TH_WARM_THRESHOLD_45T60C;
	/*TEMP UP 60*/
	else
		return TH_HOT_THRESHOLD_UP60C;
}

int lenovo_get_adapters_connect_status(void)
{
       if ((!first_port_online) && (!fusb_orient_en))
               return TYPEC_PORT_VALUE_UNKNOWN;
       else if ((first_port_online) && (!fusb_orient_en))
               return TYPEC_PORT_VALUE_FIR;
       else if ((!first_port_online) && (fusb_orient_en))
               return TYPEC_PORT_VALUE_SEC;
       else
               return TYPEC_PORT_VALUE_BOTH;
}

static int ffc_get_cp_charge_max_current(struct bq2589x *bq)
{
	int adapter_status;
	int qcom_pd_enable;
	int batt_vol;
	int temp_lv;
	int bq_current = FFC_PD_MAX_CURRENT_7P5A;

	adapter_status = lenovo_get_adapters_connect_status();
	batt_vol = ffc_get_batt2_vol(bq);
	temp_lv = ffc_get_batt2_temp_level(bq);
	qcom_pd_enable = get_adsp_chg_pps_st(bq);
	pr_info("charge_max_current, adapter_status is %d, batt_vol is %d, temp_lv is %d\n",
		adapter_status, batt_vol, temp_lv);

	if (!fusb_pd_enable)
               goto out;

       switch(temp_lv){
               case TH_COOL_THRESHOLD_15T30C:
               case TH_GOOD_THRESHOLD_30T45C:
                       if (batt_vol <= FFC_CP_STEP_VOLT){
                               if (adapter_status == TYPEC_PORT_VALUE_SEC)
                                       bq_current = FFC_PD_MAX_CURRENT_5A;
                               else if (adapter_status == TYPEC_PORT_VALUE_BOTH)
                                       bq_current = FFC_PD_MAX_CURRENT_7P5A;
                       }
                       else{
                               if (adapter_status == TYPEC_PORT_VALUE_SEC)
                                       bq_current = FFC_PD_MAX_CURRENT_5A;
                               else if (adapter_status == TYPEC_PORT_VALUE_BOTH)
                                       bq_current = FFC_PD_MAX_CURRENT_5P3A;
                       }
                       break;
               default:
                       break;
       }
out:
       return bq_current;
}

static void lenovo_ffc_ctrl_workfunc(struct work_struct *work)
{
       struct bq2589x *chg = container_of(work, struct bq2589x,
                                               lenovo_ffc_ctrl_work.work);

       if ((lenovo_get_adapters_connect_status() == TYPEC_PORT_VALUE_SEC)
		|| (lenovo_get_adapters_connect_status() == TYPEC_PORT_VALUE_BOTH)){
              cp_max_curr =  (ffc_get_cp_charge_max_current(chg)/2);
               pr_info("run ffc algorithm, cp_max_curr = %d\n", cp_max_curr);
               }
       schedule_delayed_work(&chg->lenovo_ffc_ctrl_work,
                       msecs_to_jiffies(LENOVO_FFC_CTRL_WORK_MS));

}

static void lenovo_monitor_ports_status_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
				lenovo_monitor_ports_status_work.work);
	bool insert_val_changed;
	int rc = 0;
	union lenovo_supply_propval val = {0, };

	first_port_online = get_adsp_vbus_en(chg);

	pr_info("first_port_online = %d, fusb_orient_en = %d,gpio_otg1_status = %d,gpio_otg2_status = %d\n",
                                first_port_online, fusb_orient_en, gpio_get_value(gpio_otg1_status),gpio_get_value(gpio_otg2_status));
	if ((first_port_online == 0) && (fusb_orient_en == 0)){
		chg->ports_insert_value = TYPEC_PORT_VALUE_UNKNOWN;
		}
	else if ((first_port_online > 0) && (fusb_orient_en == 0)){
		chg->ports_insert_value = TYPEC_PORT_VALUE_FIR;
		}
	else if ((first_port_online == 0) && (fusb_orient_en == 1)){
		chg->ports_insert_value = TYPEC_PORT_VALUE_SEC;
		}
	else{
		chg->ports_insert_value = TYPEC_PORT_VALUE_BOTH;
		}

	insert_val_changed = port_status_changed_enable(chg->ports_insert_value);

	if(usb1_otg_en==0&&usb2_otg_en==0){
	lenovo_set_gpios_ctrl(chg, insert_val_changed);
	}else {
	    pr_info("enable_otg_charge\n");
           enable_otg_charge(chg);
	}

	val.intval = fusb_orient_en;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_TYPEC_CONNECT_ST,
				&val);
	pr_info("lenovo: send adsp port2 typec orient is %d\n", val.intval);
	schedule_delayed_work(&chg->lenovo_monitor_ports_status_work,
				msecs_to_jiffies(LENOVE_WORK_MS));
}


#define DEFAULT_BATTERY_TEMP					255
static int skin_temp = 0;
bool is_batt1_thermal_stop;
u8 bootup_delay_done = 0;
u8 pdo_wr_boot_done = 0;
u8 insert_delay_done = 0;
static int therm_check_total = 0;
static int therm_level[4] = {0, 0, 0, 0};

extern int last_batt1_soc;
extern int last_batt2_soc;
extern int last_batt1_real_soc;
extern int last_batt2_real_soc;
extern int last_user_soc;
extern int is_pd2_done;

static int charger1_online = -1;
static int last_charger1_online = -1;
static int charger2_online = -1;
static int last_charger2_online = -1;
static int batt1_status = -1;
static int last_batt1_status = -1;
static int batt2_status = -1;
static int last_batt2_status = -1;
static int charger1_type = -1;
static int last_charger1_type = -1;
static int charger2_type = -1;
static int last_charger2_type = -1;
static int battery_soc = -1;
static int last_battery_soc = -1;

void charger_insert_remove_process(struct bq2589x *chg, int online);
extern int lenovo_get_sensor_temp(char *name, int *temp);
#if 1 //TODO wr for debug
static int get_adsp_chg_pps_st(struct bq2589x *bq);
int get_charger_max_power(struct bq2589x *chg)
{
	if (get_adsp_chg_pps_st(chg))
		return 30;
	else
		return 0;
}
#endif
//extern int get_charger_max_power(void);
//extern int get_charger1_5v_obj_current(void);
//extern int get_charger1_9v_obj_current(void);
extern int get_charger2_max_power(void);
extern int get_charger2_5v_obj_current(void);
extern int get_charger2_9v_obj_current(void);
extern int get_charger2a_max_power(void);
//extern int get_charger2a_5v_obj_current(void);
//extern int get_charger2a_9v_obj_current(void);
extern void fts_is_charger_enabled(int enable);

#define LENOVO_BATTERY_EVENT_DEALY_MS				500
#define LENOVO_CHARGER_MONITOR_DEALY_MS				10000
#define LENOVO_THERMAL_MONITOR_DEALY_MS				4000
#define CHARGER_IN_DELAY_MS					6000
#define BATTERY_FAKE_TEMP					-400
#define SEC_CHARGE_VBUS						4600000
#define	THERMAL_CHECK_MAX_NUM					30

enum pd_therm_level {
        PD_THERM_UNKNOWN,
        PD_THERM_NORMAL,
        PD_THERM_WARM,
        PD_THERM_HOT,
        PD_THERM_PERF,
#ifdef CONFIG_BATTERY_FAC
        PD_THERM_FAC,
#endif
        PD_THERM_BYPASS,
        PD_THERM_HEALTH_CHARGE,
        PD_THERM_BATT_HOT,
};

#define SINGLE_PD_THERM_LOW_TEMP		46000
#define SINGLE_PD_THERM_HIGH_TEMP		49000
#define DUAL_PD_THERM_LOW_TEMP			53000
#define DUAL_PD_THERM_HIGH_TEMP			56000
#define SINGLE_PD_THERM_USER_HIGH_TEMP		46000
#define DUAL_PD_THERM_USER_HIGH_TEMP		53000
#define	PD_THERM_WARM_FCC			3000

static bool bq_master_psy_initialized(struct bq2589x *chg)
{
	if (chg->master_fg_psy)
		return true;

	chg->master_fg_psy = lenovo_supply_get_by_name("bq27z561-master");
	if (!chg->master_fg_psy) {
		pr_err("Couldn't find master_fg_psy.\n");
		return false;
	}

	return true;
}

static bool bq_slave_psy_initialized(struct bq2589x *chg)
{
	if (chg->slave_fg_psy)
		return true;

	chg->slave_fg_psy = lenovo_supply_get_by_name("bq27z561-slave");
	if (!chg->slave_fg_psy) {
		pr_err("Couldn't find slave_fg_psy.\n");
		return false;
	}

	return true;
}

static bool bq2589x_psy_initialized(struct bq2589x *chg)
{
	if (chg->bq2589x_psy)
		return true;

	chg->bq2589x_psy = lenovo_supply_get_by_name("bq2589h-charger");
	if (!chg->bq2589x_psy) {
		pr_err("Couldn't find bq2589x_psy.\n");
		return false;
	}

	return true;
}

static bool qcom_usb_psy_initialized(struct bq2589x *chg)
{
	if (chg->usb_psy)
		return true;

	chg->usb_psy = power_supply_get_by_name("usb");
	if (!chg->usb_psy) {
		pr_err("Couldn't find usb_psy.\n");
		return false;
	}

	return true;
}

static bool qcom_battery_psy_initialized(struct bq2589x *chg)
{
	if (chg->qcom_batt_psy)
		return true;

	chg->qcom_batt_psy = power_supply_get_by_name("battery");
	if (!chg->qcom_batt_psy) {
		pr_err("Couldn't find qcom_batt_psy.\n");
		return false;
	}

	return true;
}

static int smblib_get_qcom_battery_soc(struct bq2589x *chg,
				    union power_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!qcom_battery_psy_initialized(chg))
		return -ENODEV;

	rc = power_supply_get_property(chg->qcom_batt_psy,
				POWER_SUPPLY_PROP_CAPACITY,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get qcom battery sco POWER_SUPPLY_PROP_CAPACITY, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_master_battery_status(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = LENOVO_SUPPLY_STATUS_DISCHARGING;

	if (!bq_master_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->master_fg_psy,
				LENOVO_SUPPLY_PROP_STATUS,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery LENOVO_SUPPLY_PROP_STATUS, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_slave_battery_status(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = LENOVO_SUPPLY_STATUS_DISCHARGING;

	if (!bq_slave_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->slave_fg_psy,
				LENOVO_SUPPLY_PROP_STATUS,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery LEONVO_SUPPLY_PROP_STATUS, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_master_battery_temp(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = DEFAULT_BATTERY_TEMP;

	if (!bq_master_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->master_fg_psy,
				LENOVO_SUPPLY_PROP_TEMP,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery LENOVO_SUPPLY_PROP_TEMP, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_slave_battery_temp(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = DEFAULT_BATTERY_TEMP;

	if (!bq_slave_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->slave_fg_psy,
				LENOVO_SUPPLY_PROP_TEMP,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery LENOVO_SUPPLY_PROP_TEMP, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_master_battery_current_now(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq_master_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->master_fg_psy,
				LENOVO_SUPPLY_PROP_CURRENT_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery LENOVO_SUPPLY_PROP_CURRENT_NOW, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_slave_battery_current_now(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq_slave_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->slave_fg_psy,
				LENOVO_SUPPLY_PROP_CURRENT_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery LENOVO_SUPPLY_PROP_CURRENT_NOW, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_master_battery_voltage_now(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq_master_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->master_fg_psy,
				LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery LENOVO_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_slave_battery_voltage_now(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq_slave_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->slave_fg_psy,
				LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery LENOVO_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_first_online(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get first online LENOVO_SUPPLY_PROP_TYPEC_CC_ORIENTATION, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_second_online(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_HLOS_CHG_ONLINE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get first online LENOVO_SUPPLY_PROP_HLOS_CHG_ONLINE, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_first_real_type(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = LENOVO_SUPPLY_USB_TYPE_UNKNOWN;

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get first charger LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_second_real_type(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = LENOVO_SUPPLY_USB_TYPE_UNKNOWN;

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_REAL_TYPE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charger LENOVO_SUPPLY_PROP_REAL_TYPE, rc=%d\n",
				rc);

	return rc;
}

static int is_usb_first_port(struct bq2589x *chg)
{
	int rc;
	int is_usb;
	union lenovo_supply_propval val;

	if (!bq2589x_psy_initialized(chg))
		return 0;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get first charger LENOVO_SUPPLY_PROP_ADSP_REAL_TYPE, rc=%d\n",
				rc);
		return 0;
	}

	//if ((val.intval == POWER_SUPPLY_TYPE_USB) || (val.intval == POWER_SUPPLY_TYPE_USB_CDP))
	if ((!strcmp(val.strval, "SDP")) || (!strcmp(val.strval, "CDP")))
		is_usb = 1;
	else
		is_usb = 0;

	pr_info("batt_sys: first usb device : %s [%s]\n", is_usb ? "true" : "false", val.strval);
	return is_usb;
}

static int is_usb_sec_port(struct bq2589x *chg)
{
	int rc;
	int is_usb;
	union lenovo_supply_propval val;

	if (!bq2589x_psy_initialized(chg))
		return 0;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_REAL_TYPE,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get second charger LENOVO_SUPPLY_PROP_REAL_TYPE, rc=%d\n",
				rc);
		return 0;
	}

	if ((val.intval == LENOVO_SUPPLY_USB_TYPE_SDP) || (val.intval == LENOVO_SUPPLY_USB_TYPE_CDP))
		is_usb = 1;
	else
		is_usb = 0;

	pr_info("batt_sys: second usb device : %s [%d]\n", is_usb ? "true" : "false", val.intval);
	return is_usb;
}

static int smblib_get_first_usb_voltage(struct bq2589x *chg,
				    union power_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!qcom_usb_psy_initialized(chg))
		return -ENODEV;

	rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get first usb voltage POWER_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);

	return rc;
}

static int smblib_get_second_usb_voltage(struct bq2589x *chg,
				    union lenovo_supply_propval *val)
{
	int rc;
	val->intval = 0;

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	rc = lenovo_supply_get_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_SEC_BUS_VOLTAGE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second usb voltage LENOVO_SUPPLY_PROP_SEC_BUS_VOLTAGE, rc=%d\n",
				rc);

	return rc;
}

static void lenovo_battery_monitor_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_battery_monitor_work.work);
	int rc = 0;
	u8 event_ready = 1;
	union lenovo_supply_propval val = {0,};
	union power_supply_propval ps_val = {0,};

	rc = smblib_get_master_battery_status(chg, &val);
	if (rc < 0)
		batt1_status = last_batt1_status;
	else
		batt1_status = val.intval;

	rc = smblib_get_slave_battery_status(chg, &val);
	if (rc < 0)
		batt2_status = last_batt2_status;
	else
		batt2_status = val.intval;

	rc = smblib_get_first_online(chg, &val);
	if (rc < 0)
		charger1_online = last_charger1_online;
	else
		charger1_online = val.intval;

	rc = smblib_get_second_online(chg, &val);
	if (rc < 0)
		charger2_online = last_charger2_online;
	else
		charger2_online = val.intval;

	rc = smblib_get_first_real_type(chg, &val);
	if (rc < 0)
		charger1_type = last_charger1_type;
	else
		charger1_type = val.intval;

	rc = smblib_get_second_real_type(chg, &val);
	if (rc < 0)
		charger2_type = last_charger2_type;
	else
		charger2_type = val.intval;

	rc = smblib_get_qcom_battery_soc(chg, &ps_val);
	if (rc < 0)
		battery_soc = last_battery_soc;
	else
		battery_soc = ps_val.intval;

	if (last_charger1_online != charger1_online) {
		last_charger1_online = charger1_online;
		charger_insert_remove_process(chg, charger1_online);
		fts_is_charger_enabled(charger1_online);
	} else if (last_charger2_online != charger2_online) {
		last_charger2_online = charger2_online;
		charger_insert_remove_process(chg, charger1_online);
		fts_is_charger_enabled(charger2_online);
	} else if (last_batt1_status != batt1_status)
		last_batt1_status = batt1_status;
	else if (last_batt2_status != batt2_status)
		last_batt2_status = batt2_status;
	else if (last_charger1_type != charger1_type)
		last_charger1_type = charger1_type;
	else if (last_charger2_type != charger2_type)
		last_charger2_type = charger2_type;
	else if (last_battery_soc != battery_soc)
		last_battery_soc = battery_soc;
	else
		event_ready = 0;

	//TODO: add wake lock if online
	if (last_charger1_online || last_charger2_online) {
		if (!chg->wake_lock) {
			pr_info("batt_sys: add charger wake lock\n");
			pm_stay_awake(chg->dev);
			chg->wake_lock = true;
		}
	} else {
		if (chg->wake_lock) {
			pr_info("batt_sys: release charger wake lock\n");
			pm_relax(chg->dev);
			chg->wake_lock = false;
		}
	}

	if (event_ready) {
		pr_info("batt_sys: bat-monitor on1 %d, on2 %d, type1 %d, type2 %d\n",
				charger1_online, charger2_online, charger1_type, charger2_type);

		if (chg->bq2589x_psy)
			lenovo_supply_changed(chg->bq2589x_psy);
	}

	schedule_delayed_work(&chg->lenovo_battery_monitor_work,
				msecs_to_jiffies(LENOVO_BATTERY_EVENT_DEALY_MS));
}

static int set_first_main_charge_prop(struct bq2589x *chg, int chg_icl, int chg_fcc, int chg_volt)
{
	int rc = 0;
	union lenovo_supply_propval val = {0, };

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_first_main_charge_prop break \n");
		return 0;
	}

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = chg_icl / 1000;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger LENOVO_SUPPLY_PROP_ADSP_ICL_VALUE, rc=%d\n",
			rc);

	val.intval = chg_fcc;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger LENOVO_SUPPLY_PROP_ADSP_FCC_VALUE, rc=%d\n",
			rc);


	val.intval = chg_volt / 1000;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_SET_FV,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger LENOVO_SUPPLY_PROP_SET_FV, rc=%d\n",
			rc);

	return rc;
}

static int set_sec_main_charge_prop(struct bq2589x *chg, int chg_icl, int chg_fcc, int chg_volt)
{
	int rc;

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_sec_main_charge_prop break \n");
		return 0;
	}

	rc = bq2589x_dynamic_update_charging_profile(chg, chg_volt, chg_fcc, chg_icl, SEC_CHARGE_VBUS);

	return rc;
}

static int set_first_main_charge_enable(struct bq2589x *chg, int enable)
{
	int rc;
	union lenovo_supply_propval val = {0, };

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_first_main_charge_enable break \n");
		return 0;
	}

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = !!enable;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charge LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE, rc=%d\n",
			rc);

	return rc;
}

static int set_first_cp_enable(struct bq2589x *chg, int enable)
{
	int rc;
	union lenovo_supply_propval val = {0, };

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_first_main_usb_input_suspend break \n");
		return 0;
	}

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = !enable;
	is_batt1_thermal_stop = val.intval;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first cp LENOVO_SUPPLY_PROP_ADSP_CP_THERMAL_CTR, rc=%d\n",
			rc);

	return rc;
}

static int set_first_main_usb_input_suspend(struct bq2589x *chg, int suspend)
{
	int rc;
	union lenovo_supply_propval val = {0, };

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_sec_main_charge_enable break \n");
		return 0;
	}

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = !!suspend;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charge LENOVO_SUPPLY_PROP_ADSP_USB_SUSPEND_ENABLE, rc=%d\n",
			rc);

	return rc;
}

static int set_sec_main_charge_enable(struct bq2589x *chg, int enable)
{
	int rc;
	union lenovo_supply_propval val = {0, };

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] set_sec_main_usb_input_suspend break \n");
		return 0;
	}

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = !!enable;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charge LENOVO_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);

	return rc;
}

static int set_sec_main_usb_input_suspend(struct bq2589x *chg, int suspend)
{
	int rc;
	union lenovo_supply_propval val = {0, };

	if (!bq2589x_psy_initialized(chg))
		return -ENODEV;

	val.intval = !!suspend;
	rc = lenovo_supply_set_property(chg->bq2589x_psy,
				LENOVO_SUPPLY_PROP_INPUT_SUSPEND,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charge LENOVO_SUPPLY_PROP_INPUT_SUSPEND, rc=%d\n",
				rc);

	return rc;
}

static int get_pd1_running(struct bq2589x *chg)
{
	int ret;

	ret = get_adsp_cp_step(chg);
	if ((0 == ret) || (7 == ret)) {
		pr_info("batt_sys: PD1 [STOP] [%d]\n", ret);
		return 0;
	} else {
		pr_info("batt_sys: PD1 [RUNNING] [%d]\n", ret);
		return 1;
	}
}

static void set_pd_enable(struct bq2589x *chg, int en)
{
	set_first_cp_enable(chg, en);
	is_batt2_thermal_stop = !en;
	pr_info("batt_sys: set pd enable [%d]\n", en);
}

static int get_pd2_running(void)
{
	if (1 == is_pd2_done) {
		pr_info("batt_sys: PD2 [RUNNING] [%d]\n", is_pd2_done);
		return 1;
	} else {
		pr_info("batt_sys: PD2 [STOP] [%d]\n", is_pd2_done);
		return 0;
	}
}

enum {
	C_UNKNOWN = 0,
	C_NONE,
	C_USB,
	C_DCP,
	C_PD,
	C_WEAK,
};

#define PORT_1_DCP_ICL_MAX				3000000
#define PORT_1_DCP_FCC_MAX				2700000

#define PORT_2_DCP_ICL_MAX				3000000
#define PORT_2_DCP_FCC_MAX            			2700000

#define PORT_2_PD_FIR_MAIN_ICL_MAX			1500000
#define PORT_2_PD_FIR_MAIN_FCC_MAX			1500000

#define PORT_1_PD_SEC_MAIN_ICL_MAX			1500000
#define PORT_1_PD_SEC_MAIN_FCC_MAX			1500000

#define DUAL_DCP_ICL_MAX				2800000
#define DUAL_DCP_FCC_MAX				5000000

#define PORT_DCP_ICL_DEFAULT				2000000
#define PORT_DCP_FCC_DEFAULT				2000000

#define USB_CURRENT_MAX					500000
#define PORT_CURRENT_100MA				100000
#define PORT_CURRENT_250MA				250000

#define WEAK_CURRENT_MAX				1000000
#define WEAK_DUAL_PORT_MAX				500000

#define ICL_DISABLE					-1
#define ICL_UNKNOWN					-2
#define FCC_DISABLE					-1
#define FCC_UNKNOWN					-2

#define DEFAULT_USB_VOLTAGE				5000000
#define DEFAULT_PD_VOLTAGE				8000000

#define HVDCP_VBUS_LEVEL_0				DEFAULT_USB_VOLTAGE
#define HVDCP_VBUS_LEVEL_1				6000000
#define HVDCP_VBUS_LEVEL_2				7000000
#define HVDCP_VBUS_LEVEL_3				8000000
#define HVDCP_VBUS_LEVEL_4				9000000

struct dual_charge_map {
	int port1_status;
	int port2_status;
	int port1_def_icl;
	int port1_def_fcc;
	int port2_def_icl;
	int port2_def_fcc;
};

static const struct dual_charge_map dual_charge_map[] = {
	{ C_UNKNOWN , C_UNKNOWN , ICL_DISABLE        , FCC_DISABLE        , PORT_CURRENT_100MA , PORT_CURRENT_100MA },
	{ C_NONE    , C_NONE    , ICL_DISABLE        , FCC_DISABLE        , PORT_CURRENT_100MA , PORT_CURRENT_100MA },

	{ C_USB     , C_USB     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_USB     , C_DCP     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_USB     , C_PD      , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_USB     , C_WEAK    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , WEAK_CURRENT_MAX   , WEAK_CURRENT_MAX   },

	{ C_DCP     , C_USB     , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_DCP     , C_DCP     , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_DCP     , C_PD      , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_DCP     , C_WEAK    , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , WEAK_CURRENT_MAX   , WEAK_CURRENT_MAX   },

	{ C_PD      , C_USB     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_PD      , C_DCP     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_PD      , C_PD      , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_PD      , C_WEAK    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , WEAK_CURRENT_MAX   , WEAK_CURRENT_MAX   },

	{ C_USB     , C_NONE    , PORT_CURRENT_250MA , PORT_CURRENT_250MA , PORT_CURRENT_250MA , PORT_CURRENT_250MA },
	{ C_DCP     , C_NONE    , ICL_UNKNOWN        , FCC_UNKNOWN        , ICL_UNKNOWN        , FCC_UNKNOWN        },
	{ C_PD      , C_NONE    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_1_PD_SEC_MAIN_ICL_MAX , PORT_1_PD_SEC_MAIN_FCC_MAX },
	{ C_WEAK    , C_NONE    , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX },

	{ C_NONE    , C_USB     , PORT_CURRENT_250MA , PORT_CURRENT_250MA , PORT_CURRENT_250MA , PORT_CURRENT_250MA },
	{ C_NONE    , C_DCP     , ICL_UNKNOWN        , FCC_UNKNOWN        , ICL_UNKNOWN        , FCC_UNKNOWN        },
	{ C_NONE    , C_PD      , PORT_2_PD_FIR_MAIN_ICL_MAX , PORT_2_PD_FIR_MAIN_FCC_MAX , USB_CURRENT_MAX , USB_CURRENT_MAX },
	{ C_NONE    , C_WEAK    , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX , WEAK_DUAL_PORT_MAX },
};

struct chg_ffc_table {
	int temp_low;
	int temp_high;
	int chg_volt1;
	int chg_volt2;
	int cutoff_volt;
	int chg_curr;
	int cutoff_curr;
};

static const struct  chg_ffc_table ffc1[] = {
	{    0 , 100 , 0, 0, 4400000, 1800000, -133000  },
	{  100 , 150 , 0, 0, 4400000, 3000000, -133000  },
	{  150 , 300 , 0, 0, 4470000, 3000000, -1000000 },
	{  300 , 450 , 0, 0, 4470000, 3000000, -1300000 },
	{  450 , 500 , 0, 0, 4200000, 2700000, -133000  },
};
static const struct  chg_ffc_table ffc2[] = {
	{    0 , 100 , 0, 0, 4400000, 1800000, -133000  },
	{  100 , 150 , 0, 0, 4400000, 3000000, -133000  },
	{  150 , 300 , 0, 0, 4470000, 3000000, -1000000 },
	{  300 , 450 , 0, 0, 4470000, 3000000, -1300000 },
	{  450 , 500 , 0, 0, 4200000, 2700000, -133000  },
};

static int set_main_charge_prop(struct bq2589x *chg, int fcc1, int icl1, int volt1, int fcc2, int icl2, int volt2)
{
	int rc = 0;

	if (chg->fac_test_enable) {
		pr_info("[FACTORY TEST MODE] set_main_charge_prop break \n");
		return 0;
	}

	pr_info("batt_sys: charge config: [first] fcc=%d icl=%d volt=%d pd=%d, [sec] curr=%d icl=%d volt=%d pd=%d;\n",
					fcc1, icl1, volt1, is_batt1_thermal_stop, fcc2, icl2, volt2, is_batt2_thermal_stop);

	if (-1 == icl1) {
		pr_info("batt_sys: first main process [IBUS] [STOP]\n");
		set_first_main_charge_prop(chg, 0, 0, volt1);
		set_first_main_charge_enable(chg, 0);
		set_first_main_usb_input_suspend(chg, 1);
	} else if (-1 == fcc1) {
		pr_info("batt_sys: first main process [CHARGE] [STOP]\n");
		set_first_main_charge_prop(chg, icl1, 0, volt1);
		set_first_main_charge_enable(chg, 0);
		set_first_main_usb_input_suspend(chg, 0);
	} else {
		pr_info("batt_sys: first main process [CHARGE] [RUNNING]\n");
		set_first_main_charge_prop(chg, icl1, fcc1, volt1);
		set_first_main_charge_enable(chg, 1);
		set_first_main_usb_input_suspend(chg, 0);
	}

	if (-1 == icl2) {
		pr_info("batt_sys: second main process [IBUS] [STOP]\n");
		set_sec_main_charge_prop(chg, 0, 100000, volt2);
		set_sec_main_charge_enable(chg, 0);
		set_sec_main_usb_input_suspend(chg, 1);
	} else if (-1 == fcc2) {
		pr_info("batt_sys: second main process [CHARGE] [STOP]\n");
		set_sec_main_charge_prop(chg, icl2, 100000, volt2);
		set_sec_main_charge_enable(chg, 0);
		set_sec_main_usb_input_suspend(chg, 0);
	} else {
		pr_info("batt_sys: second main process [CHARGE] [RUNNING]\n");
		set_sec_main_charge_prop(chg, icl2, fcc2, volt2);
		set_sec_main_charge_enable(chg, 1);
		set_sec_main_usb_input_suspend(chg, 0);
	}

	return rc;
}

static void set_thermal_engine_battery_policy(struct bq2589x *chg, int val)
{
	if (val != chg->charge_thermal_status) {
		pr_info("batt_sys: Thermal-engine battery policy %d\n", val);
		chg->charge_thermal_status = val;
	}
}

static int get_battery_soc_delta_level(int soc1, int soc2, int *d1, int *d2)
{
	int data1 = 50, data2 = 50;

	if ((abs(soc1 - soc2)) <= 20) {
		data1 = 50;
		data2 = 50;
	} else if ((soc1 - soc2) > 20) {
		data1 = 30;
		data2 = 70;
	} else if ((soc2 - soc1) > 20) {
		data1 = 70;
		data2 = 30;
	}

	*d1 = data1;
	*d2 = data2;

	return 0;
}

static get_port_vbus(int volt)
{
	int vbus_uv;

	if (volt <= HVDCP_VBUS_LEVEL_1)
		vbus_uv= HVDCP_VBUS_LEVEL_0;
	else if (volt <= HVDCP_VBUS_LEVEL_2)
		vbus_uv = HVDCP_VBUS_LEVEL_1;
	else if (volt <= HVDCP_VBUS_LEVEL_3)
		vbus_uv = HVDCP_VBUS_LEVEL_2;
	else if (volt <= HVDCP_VBUS_LEVEL_4)
		vbus_uv = HVDCP_VBUS_LEVEL_3;
	else
		vbus_uv = HVDCP_VBUS_LEVEL_4;

	return vbus_uv;
}

static int set_dual_charge_profile(struct bq2589x *chg, int status1, int status2, int pd_enabled)
{
	int rc = 0;
	u8 ready = 1;
	union lenovo_supply_propval val = {0,};
	union power_supply_propval psy_val = {0,};
	int i, j;
	int d1, d2;
	int batt_temp, batt1_temp, batt2_temp;
	int batt_curr;
	int batt_volt;
	int tmp_volt;
	int tmp_curr;
	int tmp_cutcurr;
	//int max_power1 = get_charger_max_power(chg); // TODO get_charger_max_power();
	//int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());
	int port1_pps = get_adsp_chg_pps_st(chg);
	int port2_pps = get_hlos_chg_pps_st(chg);
	int therm_status = PD_THERM_UNKNOWN;
	int pd1_running = get_pd1_running(chg);
	int pd2_running = get_pd2_running();
	u8 max_jeita_num1 = ARRAY_SIZE(ffc1) - 1;
	u8 max_jeita_num2 = ARRAY_SIZE(ffc2) - 1;
	int vbus1, vbus2, vbus1_uv, vbus2_uv;
	int total_icl = -1, total_fcc = -1;
	int icl1 = 0, icl2 = 0, fcc1 = 0, fcc2 = 0;
	int port1_icl, port2_icl, port1_fcc, port2_fcc;
	int dual_icl;

	if (chg->fac_test_enable) {
		pr_info("batt_sys: [FACTORY TEST MODE] charge monitor work break \n");
		return 0;
	}

	rc = smblib_get_master_battery_temp(chg, &val);
	batt1_temp = (rc < 0) ? -1000 : val.intval;
	rc = smblib_get_slave_battery_temp(chg, &val);
	batt2_temp = (rc < 0) ? -1000 : val.intval;

	get_battery_soc_delta_level(last_batt1_soc, last_batt2_soc, &d1, &d2);

	rc = smblib_get_first_usb_voltage(chg, &psy_val);
	if ((rc < 0) || (psy_val.intval < 4200000))
		vbus1 = DEFAULT_USB_VOLTAGE;
	else
		vbus1 = psy_val.intval;

	rc = smblib_get_second_usb_voltage(chg, &val);
	if ((rc < 0) || (val.intval < 4200))
		vbus2 = DEFAULT_USB_VOLTAGE;
	else
		vbus2 = val.intval * 1000;

	vbus1_uv = get_port_vbus(vbus1);
	vbus2_uv = get_port_vbus(vbus2);
	pr_info("batt_sys: vbus1 %d vbus2 %d vbus1_uv %d vbus2_uv %d\n",
			vbus1, vbus2, vbus1_uv, vbus2_uv);

	/*************************************************************
	* BUS power
	*************************************************************/
	if (port1_pps) {
		//chg->port1_bus_5v_icl_ua = 2000000; // TODO get_charger1_5v_obj_current() * 1000;
		//chg->port1_bus_9v_icl_ua = 2000000; // TODO get_charger1_9v_obj_current() * 1000;
		chg->port1_bus_5v_icl_ua = get_adsp_fixpdo1_current(chg) * 1000;
		chg->port1_bus_9v_icl_ua = get_adsp_fixpdo2_current(chg) * 1000;
	} else {
		chg->port1_bus_5v_icl_ua = PORT_DCP_ICL_DEFAULT;
		chg->port1_bus_9v_icl_ua = PORT_DCP_ICL_DEFAULT;
	}

	if (port2_pps) {
		chg->port2_bus_5v_icl_ua = get_charger2_5v_obj_current() * 1000;
		chg->port2_bus_9v_icl_ua = get_charger2_9v_obj_current() * 1000;
	} else {
		chg->port2_bus_5v_icl_ua = PORT_DCP_ICL_DEFAULT;
		chg->port2_bus_9v_icl_ua = PORT_DCP_ICL_DEFAULT;
	}

	pr_info("batt_sys: port-1 pps %d 5v %d 9v %d; port-2 pps %d 5v %d 9v %d\n",
			port1_pps, chg->port1_bus_5v_icl_ua, chg->port1_bus_9v_icl_ua,
			port2_pps, chg->port2_bus_5v_icl_ua, chg->port2_bus_9v_icl_ua);

	/*************************************************************
	* FCC Default config
	*************************************************************/
	for (i = 0; i < ARRAY_SIZE(dual_charge_map); i++) {
		if ((status1 == dual_charge_map[i].port1_status) &&
				(status2 == dual_charge_map[i].port2_status)) {
			icl1 = dual_charge_map[i].port1_def_icl;
			fcc1 = dual_charge_map[i].port1_def_fcc;
			icl2 = dual_charge_map[i].port2_def_icl;
			fcc2 = dual_charge_map[i].port2_def_fcc;
			break;
		}
	}

	chg->first_main_icl_ua = icl1;
	chg->first_main_fcc_ua = fcc1;
	chg->sec_main_icl_ua = icl2;
	chg->sec_main_fcc_ua = fcc2;
	pr_info("batt_sys: charge [config] icl:fcc  (1) %d:%d  (2) %d:%d\n",
			icl1, fcc1, icl2, fcc2);

	if ((ICL_UNKNOWN == icl1) && (ICL_UNKNOWN == icl2)) {
		if (DEFAULT_USB_VOLTAGE == vbus1_uv)
			dual_icl = max(chg->port1_bus_5v_icl_ua, chg->port2_bus_5v_icl_ua);
		else
			dual_icl = max(chg->port1_bus_9v_icl_ua, chg->port2_bus_9v_icl_ua);
		chg->total_main_icl_ua = min(DUAL_DCP_ICL_MAX, dual_icl);
		chg->total_main_fcc_ua = DUAL_DCP_FCC_MAX;
	} else {
		chg->total_main_icl_ua = ICL_UNKNOWN;
		chg->total_main_fcc_ua = FCC_UNKNOWN;
		if (DEFAULT_USB_VOLTAGE == vbus1_uv)
			chg->first_main_icl_ua = min(chg->port1_bus_5v_icl_ua, chg->first_main_icl_ua);
		else
			chg->first_main_icl_ua = min(chg->port1_bus_9v_icl_ua, chg->first_main_icl_ua);
		if (DEFAULT_USB_VOLTAGE == vbus2_uv)
			chg->sec_main_icl_ua = min(chg->port2_bus_5v_icl_ua, chg->sec_main_icl_ua);
		else
			chg->sec_main_icl_ua = min(chg->port2_bus_9v_icl_ua, chg->sec_main_icl_ua);
	}

	pr_info("batt_sys: charge [default] icl:fcc  (1) %d:%d  (2) %d:%d  (total) %d:%d\n",
			chg->first_main_icl_ua, chg->first_main_fcc_ua,
			chg->sec_main_icl_ua, chg->sec_main_fcc_ua,
			chg->total_main_icl_ua, chg->total_main_fcc_ua);

	/*************************************************************
	* Get Max ICL
	*************************************************************/
#if 0
	//if ((status1 != C_PD) && (status2 != C_PD)) {
	//if ((!pd1_running) && (!pd2_running)) {
	if ((-1 == curr1) && (-1 == curr2)) {
		if (0 == chg->icl_settled_ready) {
			pr_info("batt_sys: BEGIN ICL SETTLED\n");
			chg->icl_settled_ready = 1;
			curr1 = DUAL_DCP_ICL_MAX;
			chg->first_request_icl_ua = curr1 * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
			curr2 = 100000;

			schedule_delayed_work(&chg->lenovo_icl_settled_work,
					msecs_to_jiffies(50000));
		} else if (1 == chg->icl_settled_ready) {
			pr_info("batt_sys: WAIT FOR ICL SETTLED\n");
			return 0;
		} else if (2 == chg->icl_settled_ready) {
/*
			curr1 = chg->sin_port_max_power * d1 / (DEFAULT_USB_VOLTAGE / 1000000) / 100;
			curr2 = chg->sin_port_max_power * d2 / (DEFAULT_USB_VOLTAGE / 1000000) / 100;
			chg->first_request_icl_ua = chg->sin_port_max_power * d1 / (vbus_uv / 1000000) / 100;
			chg->sec_request_icl_ua = chg->sin_port_max_power * d2 / (vbus_uv / 1000000) / 100;
*/
/*
			curr1 = PORT_1_DCP_FCC_MAX * / 100;
			curr2 = PORT_2_DCP_FCC_MAX * / 100;
			chg->first_request_icl_ua = PORT_1_DCP_FCC_MAX * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
			chg->sec_request_icl_ua = PORT_2_DCP_FCC_MAX * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
*/
			curr1 = DUAL_DCP_ICL_MAX / 2;
			curr2 = DUAL_DCP_ICL_MAX / 2;
			chg->first_request_icl_ua = curr1;
			chg->sec_request_icl_ua = curr2;
		}
	} else {
		if (1 == chg->icl_settled_ready)
			cancel_delayed_work_sync(&chg->lenovo_icl_settled_work);
		chg->icl_settled_ready = 0;
		chg->max_icl_settled_ua = 0;
		chg->usb_vbus_uv = 0;
		chg->sin_port_max_power = 0;
	}
#endif
	chg->icl_settled_ready = 0;

	/*************************************************************
	* PD Thermal
	*************************************************************/
	if (lenovo_get_sensor_temp("skin-msm-therm-usr", &skin_temp) < 0) {
		skin_temp = 0;
		pr_info("batt_sys: can not get skin_temp %d\n", skin_temp);
	}
	pr_info("batt_sys: skin_temp %d, d1 %d, d2 %d\n",
			skin_temp, d1, d2);

	if (port1_pps || port2_pps) {
		if (port1_pps && port2_pps) {
			if ((skin_temp >= DUAL_PD_THERM_LOW_TEMP) && (skin_temp <= DUAL_PD_THERM_HIGH_TEMP)) {
				therm_status = PD_THERM_WARM;
				pr_info("batt_sys: pd event [Dual-PD WARM]\n");
			} else if (skin_temp > DUAL_PD_THERM_HIGH_TEMP) {
				therm_status = PD_THERM_HOT;
				pr_info("batt_sys: pd event [Dual-PD HOT]\n");
			} else {
				therm_status = PD_THERM_NORMAL;
			}
		} else {
			if ((skin_temp >= SINGLE_PD_THERM_LOW_TEMP) && (skin_temp <= SINGLE_PD_THERM_HIGH_TEMP)) {
				therm_status = PD_THERM_WARM;
				pr_info("batt_sys: pd event [Single-PD WARM]\n");
			} else if (skin_temp > SINGLE_PD_THERM_HIGH_TEMP) {
				therm_status = PD_THERM_HOT;
				pr_info("batt_sys: pd event [Single-PD HOT]\n");
			} else {
				therm_status = PD_THERM_NORMAL;
			}
		}

		if ((batt1_temp >= 450) || (batt2_temp >= 450)) {
			therm_status = PD_THERM_BATT_HOT;
			pr_info("batt_sys: pd event [BATT HOT]\n");
		} else if (chg->charge_therm_fcc_ua != -1) {
			therm_status = PD_THERM_PERF;
			pr_info("batt_sys: pd event [THERMAL]\n");
#ifdef CONFIG_BATTERY_FAC
		} else if ((!chg->port1_fac_pd_en) || (!chg->port2_fac_pd_en)) {
			therm_status = PD_THERM_FAC;
			pr_info("batt_sys: pd event [FACTORY]\n");
#endif
		} else if (!chg->user_charging_enabled) {
			therm_status = PD_THERM_BYPASS;
			pr_info("batt_sys: pd event [BYPASS]\n");
		} else if (chg->user_health_charge) {
			therm_status = PD_THERM_HEALTH_CHARGE;
			pr_info("batt_sys: pd event [HEALTH CHARGE]\n");
		}

		switch (therm_status) {
		case PD_THERM_WARM:
		case PD_THERM_HOT:
		case PD_THERM_PERF:
#ifdef CONFIG_BATTERY_FAC
		case PD_THERM_FAC:
#endif
		case PD_THERM_BYPASS:
		case PD_THERM_HEALTH_CHARGE:
		case PD_THERM_BATT_HOT:
			if (1 == chg->restart_pd_status)
				cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
			set_pd_enable(chg, 0);
			chg->restart_pd_status = 0;
			break;

		case PD_THERM_NORMAL:
			if (port1_pps && port2_pps)
				pr_info("batt_sys: pd event [Dual-PD NORMAL]\n");
			else
				pr_info("batt_sys: pd event [Single-PD NORMAL]\n");
			set_pd_enable(chg, 1);
			if (0 == chg->restart_pd_status) {
				pr_info("batt_sys: thermal normal, restart pd work\n");
				chg->restart_pd_status = 1;
				schedule_delayed_work(&chg->lenovo_restart_pd_work,
						msecs_to_jiffies(30000));
				schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
				return 0;
			}
			break;

		default:
			break;
		}

		pr_info("batt_sys: therm_status %d, pd restart %d, pd stop %d:%d pd_en %d\n",
			therm_status, chg->restart_pd_status,
			is_batt1_thermal_stop, is_batt2_thermal_stop, pd_enabled);
	} else {
		if (1 == chg->restart_pd_status)
			cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
		pr_info("batt_sys: pd event [NONE]\n");
		set_pd_enable(chg, 1);
		chg->restart_pd_status = 0;
	}

	if ((pd1_running) || (pd2_running) || (1 == chg->restart_pd_status)) {
		pr_info("batt_sys: thermal-engine disable [PD running]\n");
		set_thermal_engine_battery_policy(chg, 0);
	} else {
		pr_info("batt_sys: thermal-engine enable [default NOT PD]\n");
		set_thermal_engine_battery_policy(chg, 1);
	}

	/*************************************************************
	* System Thermal
	*************************************************************/

	if (1 != chg->icl_settled_ready) {
		if (chg->charge_therm_fcc_ua != -1)
			pr_info("batt_sys: charge [therm enable] total fcc %d\n", chg->charge_therm_fcc_ua);
		else
			pr_info("batt_sys: charge [therm disable] total fcc %d\n", chg->charge_therm_fcc_ua);
	} else
		pr_info("batt_sys: charge [therm unknown] total fcc %d\n", chg->charge_therm_fcc_ua);

	/*************************************************************
	* Battery Jeita
	*************************************************************/
	for (j = 0; j < 2; j++) {
		if (0 == j) {
			batt_temp = batt1_temp;
			rc = smblib_get_master_battery_current_now(chg, &val);
			batt_curr = (rc < 0) ? 0 : val.intval;
			rc = smblib_get_master_battery_voltage_now(chg, &val);
			batt_volt = (rc < 0) ? 0 : val.intval;
			pr_info("batt_sys: batt1 temp %d, volt %d, curr %d, soc %d, real_soc %d, full %d\n",
					batt_temp, batt_volt, batt_curr, last_batt1_soc, last_batt1_real_soc, chg->batt1_cap_full);

			if (batt_temp <= ffc1[0].temp_low) {
				pr_info("batt_sys: BATT 1 STATUS [COOL]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc1[0].cutoff_volt;
			} else if (batt_temp >= ffc1[max_jeita_num1].temp_high) {
				pr_info("batt_sys: BATT 1 STATUS [HOT]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc1[max_jeita_num1].cutoff_volt;
			} else {
				for (i = 0; i <= max_jeita_num1; i++) {
					if ((batt_temp > ffc1[i].temp_low) && (batt_temp <= ffc1[i].temp_high)) {
						tmp_curr = ffc1[i].chg_curr;
						tmp_volt = ffc1[i].cutoff_volt;
						tmp_cutcurr = ffc1[i].cutoff_curr;
						break;
					}
				}
			}

			chg->first_batt_max_fv_uv = tmp_volt;

#ifdef CONFIG_BATTERY_FAC
			if (FCC_DISABLE == chg->first_main_fcc_ua) {
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				chg->port1_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] charger1 [Unknown or None Status] soc [%d]\n", last_batt1_soc);
			} else if (chg->batt1_cap_full) {
				if (last_batt1_soc <= 68) {
					chg->batt1_cap_full = 0;
					chg->first_batt_max_fcc_ua = tmp_curr;
					chg->port1_fac_pd_en = 1;
					pr_info("batt_sys: [FACTORY] batt1 [%d] [Re-Charge]\n", last_batt1_soc);
				} else {
					chg->first_batt_max_fcc_ua = FCC_DISABLE;
					chg->port1_fac_pd_en = 0;
					pr_info("batt_sys: [FACTORY] batt1 [%d] [FULL -> Not Charge]\n", last_batt1_soc);
				}
			} else if (last_batt1_soc == 70) {
				chg->batt1_cap_full = 1;
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				chg->port1_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] batt1 [%d] [FULL]\n", last_batt1_soc);
			} else if (last_batt1_soc > 70) {
				chg->batt1_cap_full = 1;
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				chg->first_main_icl_ua = ICL_DISABLE;
				chg->port1_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] batt1 [%d] [Discharge]\n", last_batt1_soc);
			} else {
				chg->first_batt_max_fcc_ua = tmp_curr;
				chg->port1_fac_pd_en = 1;
				pr_info("batt_sys: [FACTORY] batt1 [%d] [Charge]\n", last_batt1_soc);
			}
#else
			if (FCC_DISABLE == chg->first_main_fcc_ua) {
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: charger1 [Unknown or None Status]\n");
			} else if (chg->batt1_cap_full) {
				if ((chg->user_health_charge) && (last_user_soc <= 85)) {
					chg->batt1_cap_full = 0;
					chg->first_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt1 [90] [Health-Charge] [Re-Charge]\n");
				} else if ((!chg->user_health_charge) && (last_batt1_real_soc <= 98)) {
					chg->batt1_cap_full = 0;
					chg->first_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt1 [100] [Re-Charge]\n");
				} else {
					chg->first_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt1 [100] [Not Charge]\n");
				}
			} else if ((chg->user_health_charge) && (last_user_soc >= 90)) {
				chg->batt1_cap_full = 1;
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: batt1 [90] [Health-Charge] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
			} else if (100 == last_batt1_real_soc) {
				if (batt_curr >= tmp_cutcurr) {
					chg->batt1_cap_full = 1;
					chg->first_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt1 [100] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				} else {
					chg->first_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt1 [100] [Charge] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				}
			} else {
				chg->first_batt_max_fcc_ua = tmp_curr;
			}
#endif
		} else {
			batt_temp = batt2_temp;
			rc = smblib_get_slave_battery_current_now(chg, &val);
			batt_curr = (rc < 0) ? 0 : val.intval;
			rc = smblib_get_slave_battery_voltage_now(chg, &val);
			batt_volt = (rc < 0) ? 0 : val.intval;
			pr_info("batt_sys: batt2 temp %d, volt %d, curr %d, soc %d, real_soc %d, full %d\n",
					batt_temp, batt_volt, batt_curr, last_batt2_soc, last_batt2_real_soc, chg->batt2_cap_full);

			if (batt_temp <= ffc2[0].temp_low) {
				pr_info("batt_sys: BATT 2 STATUS [COOL]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc2[0].cutoff_volt;
			} else if (batt_temp >= ffc2[max_jeita_num2].temp_high) {
				pr_info("batt_sys: BATT 2 STATUS [HOT]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc2[max_jeita_num2].cutoff_volt;
			} else {
				for (i = 0; i <= max_jeita_num2; i++) {
					if ((batt_temp > ffc2[i].temp_low) && (batt_temp <= ffc2[i].temp_high)) {
						tmp_curr = ffc2[i].chg_curr;
						tmp_volt = ffc2[i].cutoff_volt;
						tmp_cutcurr = ffc2[i].cutoff_curr;
						break;
					}
				}
			}

			chg->sec_batt_max_fv_uv = tmp_volt;

#ifdef CONFIG_BATTERY_FAC
			if (FCC_DISABLE == chg->sec_main_fcc_ua) {
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				chg->port2_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] charger2 [Unknown or None Status] soc [%d]\n", last_batt2_soc);
			} else if (chg->batt2_cap_full) {
				if (last_batt2_soc <= 68) {
					chg->batt2_cap_full = 0;
					chg->sec_batt_max_fcc_ua = tmp_curr;
					chg->port2_fac_pd_en = 1;
					pr_info("batt_sys: [FACTORY] batt2 [%d] [Re-Charge]\n", last_batt2_soc);
				} else {
					chg->sec_batt_max_fcc_ua = FCC_DISABLE;
					chg->port2_fac_pd_en = 0;
					pr_info("batt_sys: [FACTORY] batt2 [%d] [FULL -> Not Charge]\n", last_batt2_soc);
				}
			} else if (last_batt2_soc == 70) {
				chg->batt2_cap_full = 1;
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				chg->port2_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] batt2 [%d] [FULL]\n", last_batt2_soc);
			} else if (last_batt2_soc > 70) {
				chg->batt2_cap_full = 1;
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				chg->sec_main_icl_ua = ICL_DISABLE;
				chg->port2_fac_pd_en = 0;
				pr_info("batt_sys: [FACTORY] batt2 [%d] [Discharge]\n", last_batt2_soc);
			} else {
				chg->sec_batt_max_fcc_ua = tmp_curr;
				chg->port2_fac_pd_en = 1;
				pr_info("batt_sys: [FACTORY] batt2 [%d] [Charge]\n", last_batt2_soc);
			}
#else
			if (FCC_DISABLE == chg->sec_main_fcc_ua) {
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: charger2 [Unknown or None Status]\n");
			} else if (chg->batt2_cap_full) {
				if ((chg->user_health_charge) && (last_user_soc <= 85)) {
					chg->batt2_cap_full = 0;
					chg->sec_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt2 [90] [Health-Charge] [Re-Charge]\n");
				} else if ((!chg->user_health_charge) && (last_batt2_real_soc <= 98)) {
					chg->batt2_cap_full = 0;
					chg->sec_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt2 [100] [Re-Charge]\n");
				} else {
					chg->sec_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt2 [100] [Not Charge]\n");
				}
			} else if ((chg->user_health_charge) && (last_user_soc >= 90)) {
				chg->batt2_cap_full = 1;
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: batt2 [90] [Health-Charge] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
			} else if (100 == last_batt2_real_soc) {
				if (batt_curr >= tmp_cutcurr) {
					chg->batt2_cap_full = 1;
					chg->sec_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt2 [100] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				} else {
					chg->sec_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt2 [100] [Charge] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				}
			} else {
				chg->sec_batt_max_fcc_ua = tmp_curr;
			}
#endif
		}
	}

	/*************************************************************
	* User Bypass Charge
	*************************************************************/
	if (!chg->user_charging_enabled) {
		chg->first_batt_max_fcc_ua = FCC_DISABLE;
		chg->sec_batt_max_fcc_ua = FCC_DISABLE;
		pr_info("batt_sys: user bypass charge [Disable Charge]\n");
	}

	/*************************************************************
	* Request ICL and FCC
	*************************************************************/
#if 0
	if (1 == chg->icl_settled_ready) {
		chg->first_request_fcc_ua = chg->first_main_fcc_ua;
		chg->sec_request_fcc_ua = chg->sec_main_fcc_ua;;
		pr_info("batt_sys: aicl setting [1] fcc %d, [2] fcc %d\n",
				chg->first_request_fcc_ua, chg->sec_request_fcc_ua);
	}
#endif

	if ((ICL_UNKNOWN != chg->first_main_icl_ua) && (ICL_UNKNOWN != chg->sec_main_icl_ua)) {
		port1_icl = chg->first_main_icl_ua;
		port1_fcc = min(chg->first_main_fcc_ua, chg->first_batt_max_fcc_ua);
		port2_icl = chg->sec_main_icl_ua;
		port2_fcc = min(chg->sec_main_fcc_ua, chg->sec_batt_max_fcc_ua);
		if (chg->charge_therm_fcc_ua != -1) {
			port1_fcc = min(port1_fcc, chg->charge_therm_fcc_ua / 2);
			port2_fcc = min(port2_fcc, chg->charge_therm_fcc_ua / 2);
		}
	} else {
		total_icl = chg->total_main_icl_ua;
		if (-1 == chg->charge_therm_fcc_ua)
			total_fcc = chg->total_main_fcc_ua;
		else
			total_fcc = min(chg->total_main_fcc_ua, chg->charge_therm_fcc_ua);
		port1_icl = total_icl / 2;
		port1_fcc = min(chg->first_batt_max_fcc_ua, total_fcc / 2);
		port2_icl = total_icl / 2;
		port2_fcc = min(chg->sec_batt_max_fcc_ua, total_fcc / 2);
	}
	pr_info("batt_sys: charge [port] icl:fcc  (1) %d:%d  (2) %d:%d  (total) %d:%d\n",
			port1_icl, port1_fcc, port2_icl, port2_fcc, total_icl, total_fcc);

	//chg->first_request_icl_ua = port1_icl * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus1_uv / 1000000);
	//chg->sec_request_icl_ua = port2_icl * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus2_uv / 1000000);
	chg->first_request_icl_ua = port1_icl;
	chg->sec_request_icl_ua = port2_icl;
	chg->first_request_fcc_ua = port1_fcc;
	chg->sec_request_fcc_ua = port2_fcc;
	chg->first_request_fv_uv = chg->first_batt_max_fv_uv;
	chg->sec_request_fv_uv = chg->sec_batt_max_fv_uv;

	pr_info("batt_sys: charge [request] icl:fcc:fv  (1) %d:%d:%d  (2) %d:%d:%d\n",
		chg->first_request_icl_ua, chg->first_request_fcc_ua, chg->first_request_fv_uv,
		chg->sec_request_icl_ua, chg->sec_request_fcc_ua, chg->sec_request_fv_uv);

	pr_info("batt_sys: profile 1 ready %d, stat %d, def %d jeita %d req %d icl %d fv %d\n",
			ready, status1,
			chg->first_main_fcc_ua, chg->first_batt_max_fcc_ua,
			chg->first_request_fcc_ua, chg->first_request_icl_ua, chg->first_request_fv_uv);
	pr_info("batt_sys: profile 2 ready %d, stat %d, def %d jeita %d req %d icl %d fv %d\n",
			ready, status2,
			chg->sec_main_fcc_ua, chg->sec_batt_max_fcc_ua,
			chg->sec_request_fcc_ua, chg->sec_request_icl_ua, chg->sec_request_fv_uv);
	if (ready) {
//TODO 
#if 0
		rc = smblib_get_prop_input_current_settled(chg, &val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		}

		if (val.intval == 100000) {
			if ((C_PD == status1) ||
				(C_DCP == status1) ||
				(C_PD == status2) ||
				(C_DCP == status2))
			pr_info("batt_sys: profile qcom recovery process %d\n", val.intval);

			set_main_charge_prop(chg, 150000, 150000, chg->first_request_fv_uv, 350000, 350000, chg->sec_request_fv_uv);
			msleep(1000);
		}
#endif
		set_main_charge_prop(chg, chg->first_request_fcc_ua, chg->first_request_icl_ua, chg->first_request_fv_uv,
					chg->sec_request_fcc_ua, chg->sec_request_icl_ua, chg->sec_request_fv_uv);
	}

	return rc;
}

static void lenovo_charge_monitor_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_charge_monitor_work.work);
	int port1_is_charger;
	int port2_is_charger;
	int port1_is_usb;
	int port2_is_usb;
	int status1;
	int status2;
	int pd1_running = get_pd1_running(chg);
	int pd2_running = get_pd2_running();
	int ready = 1;
	int rc;
	union lenovo_supply_propval val = {0,};
	//int max_power1 = get_charger_max_power(chg); //TODO get_charger_max_power();
	//int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());
	//int port1_pps = get_adsp_chg_pps_st(chg);
	//int port2_pps = get_hlos_chg_pps_st(chg);
	int pd_en = 1;

	//mutex_lock(&chg->charge_work_lock);
	//port1_is_charger = (qcom_orient_en | first_typec_attached ) & (!usb1_otg_en);
	port1_is_charger = last_charger1_online;
	port2_is_charger = last_charger2_online;
	port1_is_usb = is_usb_first_port(chg);
	port2_is_usb = is_usb_sec_port(chg);
	status1 = C_NONE;
	status2 = C_NONE;

	//if ((!g_chg_en_dev) ||
	//		((!port1_is_charger) && (!port2_is_charger)))
	if ((!port1_is_charger) && (!port2_is_charger))
		goto out;

	if (!insert_delay_done) {
		status1 = C_UNKNOWN;
		status2 = C_UNKNOWN;
		goto out;
	}

	if (port1_is_charger) {
		if (port1_is_usb) {
			status1 = C_USB;
			pr_info("batt_sys: port-1 type : [ USB ] Device\n");
#if 0
		} else if ((max_power1 > 0) && (max_power1 <= 15)) {
			status1 = C_USB;
			pr_info("batt_sys: port-1 type : [ USB ] WA for weak pd charger [%d]\n", max_power1);
#endif
		} else if (is_batt1_thermal_stop) {
			status1 = C_DCP;
			pr_info("batt_sys: port-1 type : [ DCP ] Request pd stop for thermal\n");
		} else if (pd1_running) {
			status1 = C_PD;
			pr_info("batt_sys: port-1 type : [ PD ] pd running\n");
		} else if (1 == chg->restart_pd_status) {
			status1 = C_PD;
			pr_info("batt_sys: port-1 type : [ PD ] Request pd restart\n");
		} else {
			status1 = C_DCP;
			pr_info("batt_sys: port-1 type : [ DCP ] Default device\n");
		}
	}

	if (port2_is_charger) {
		rc = smblib_get_second_real_type(chg, &val);
		if (rc < 0) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] Can not get charger real type\n");
		} else if (port2_is_usb) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] Device\n");
#if 0
		} else if ((max_power2 > 0) && (max_power2 <= 15)) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] WA for weak pd charger [%d]\n", max_power2);
#endif
		} else if (is_batt2_thermal_stop) {
			status2 = C_DCP;
			pr_info("batt_sys: port-2 type : [ DCP ] Request pd stop for thermal\n");
		} else if (pd2_running) {
			status2 = C_PD;
			pr_info("batt_sys: port-2 type : [ PD ] pd running\n");
		} else if (1 == chg->restart_pd_status) {
			status2 = C_PD;
			pr_info("batt_sys: port-2 type : [ PD ] Request pd restart\n");
		} else if (POWER_SUPPLY_TYPE_UNKNOWN == val.intval) {
			//status2 = C_WEAK;
			//pr_info("batt_sys: port-2 type : [ WEAK ] Charger real type Unknown\n");
			status2 = C_DCP;
			pr_info("batt_sys: port-2 type : [ DCP ] Charger real type Unknown\n");
		} else {
			status2 = C_DCP;
			pr_info("batt_sys: port-2 type : [ DCP ] Default device\n");
		}
	}

	if ((!bootup_delay_done) && ((C_USB == status1) || (C_USB == status2))) {
		pr_info("batt_sys: system bootup ...");
		status1 = C_UNKNOWN;
		status2 = C_UNKNOWN;
	}

out:
	pr_info("batt_sys: chg-monitor ready %d, in_delay %d, charger %d:%d, usb %d:%d, pd %d:%d, status %d:%d\n",
			ready, insert_delay_done,
			port1_is_charger, port2_is_charger,
			port1_is_usb, port2_is_usb,
			pd1_running, pd2_running,
			status1, status2);

	if (ready)
		set_dual_charge_profile(chg, status1, status2, pd_en);
	//mutex_unlock(&chg->charge_work_lock);

	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(LENOVO_CHARGER_MONITOR_DEALY_MS));
}

static void lenovo_icl_settled_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_icl_settled_work.work);

	pr_info("batt_sys: icl settled done [%d] [%d] [%d]\n",
			chg->max_icl_settled_ua, chg->usb_vbus_uv, chg->sin_port_max_power);
	if (!chg->max_icl_settled_ua) {
		chg->max_icl_settled_ua = 2000000;
		chg->usb_vbus_uv = DEFAULT_USB_VOLTAGE;
		chg->sin_port_max_power = 10000000;
		pr_info("batt_sys: icl settled default [%d] [%d] [%d]\n",
				 chg->max_icl_settled_ua, chg->usb_vbus_uv, chg->sin_port_max_power);
	}
	chg->icl_settled_ready = 2;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

static void lenovo_restart_pd_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_restart_pd_work.work);

	pr_info("batt_sys: restart pd work done\n");
	chg->restart_pd_status = 2;
}

static void lenovo_insert_delay_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_insert_delay_work.work);

	pr_info("batt_sys: charger insert/remove/recheck delay done\n");
	insert_delay_done = 1;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

static void lenovo_bootup_delay_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_bootup_delay_work.work);

	pr_info("batt_sys: charger bootup delay done\n");
	bootup_delay_done = 1;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

static void lenovo_wr_pdo_bootup_delay_work(struct work_struct *work)
{
	pr_info("batt_sys: pdo wr bootup delay done\n");
	pdo_wr_boot_done = 1;
}

void charger_insert_remove_process(struct bq2589x *chg, int online)
{
	int i;

	pr_info("batt_sys: charger insert/remove/recheck process %d\n", online);

	cancel_delayed_work_sync(&chg->lenovo_insert_delay_work);
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	cancel_delayed_work_sync(&chg->lenovo_icl_settled_work);
	cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
	set_dual_charge_profile(chg, C_UNKNOWN, C_UNKNOWN, 1);
	insert_delay_done = 0;
	chg->icl_settled_ready = 0;
	chg->max_icl_settled_ua = 0;
	chg->usb_vbus_uv = 0;
	chg->sin_port_max_power = 0;
	chg->restart_pd_status = 0;
	chg->batt1_cap_full = 0;
	chg->batt2_cap_full = 0;
	chg->charge_therm_fcc_ua = -1;
	therm_check_total = 0;
	for (i = 3; i >= 0; i--)
		therm_level[i] = 0;

	schedule_delayed_work(&chg->lenovo_insert_delay_work,
						msecs_to_jiffies(CHARGER_IN_DELAY_MS));
}

static void lenovo_thermal_monitor_work(struct work_struct *work)
{
	struct bq2589x *chg = container_of(work, struct bq2589x,
			lenovo_thermal_monitor_work.work);
	int max = 0;
	int i;
	int index;
	int user_therm_enable = 0;
	//int max_power1 = get_charger_max_power(chg); //TODO get_charger_max_power();
	//int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());
	int port1_pps = get_adsp_chg_pps_st(chg);
	int port2_pps = get_hlos_chg_pps_st(chg);

	therm_level[g_charge_thermal_level]++;
	therm_check_total++;

	if (therm_check_total > THERMAL_CHECK_MAX_NUM) {
		for (i = 3; i >= 0; i--) {
			if (max < therm_level[i]) {
				max = therm_level[i];
				index = i;
			}
		}

		if (port1_pps && port2_pps) {
			if (skin_temp >= DUAL_PD_THERM_USER_HIGH_TEMP)
				user_therm_enable = 1;
		} else {
			if (skin_temp >= SINGLE_PD_THERM_USER_HIGH_TEMP)
				user_therm_enable = 1;
		}

		if (user_therm_enable) {
			if (0 == index)
				chg->charge_therm_fcc_ua = -1;
			else
				chg->charge_therm_fcc_ua = g_charge_thermal_fcc;
		} else
			chg->charge_therm_fcc_ua = -1;

		pr_info("batt_sys: thermal user %d, skin temp %d, level %d, count %d %d %d %d, fcc %d\n",
				user_therm_enable, skin_temp, index,
				therm_level[0], therm_level[1], therm_level[2], therm_level[3], chg->charge_therm_fcc_ua);

		therm_check_total = 0;
		for (i = 3; i >= 0; i--)
			therm_level[i] = 0;

	} else
		pr_info("batt_sys: thermal check num %d level %d %d %d %d\n",
				therm_check_total, therm_level[0], therm_level[1], therm_level[2], therm_level[3]);

	schedule_delayed_work(&chg->lenovo_thermal_monitor_work,
				msecs_to_jiffies(LENOVO_THERMAL_MONITOR_DEALY_MS));
}
#endif
static void determine_initial_status(struct bq2589x *bq)
{
	int ret;
	u8 status;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (!ret && (status & BQ2589X_PG_STAT_MASK)){
		msleep(2);
		bq2589x_force_dpdm(bq);
	}
}

#ifdef DBG_FS
static int show_registers(struct seq_file *m, void *data)
{
	struct bq2589x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2589x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2589x *bq)
{
	bq->debug_root = debugfs_create_dir("bq25890h-main-charger", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charging_disabled_status));

		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->vbus_type));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}
}
#endif

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int ret;
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}
	boot_flag = 1;
	pr_info("bq2589x_charger_probe:set boot_flag is %d\n", boot_flag);
	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;
	bq->usb_type = LENOVO_SUPPLY_TYPE_UNKNOWN;
	bq->psy_usb_type = LENOVO_SUPPLY_USB_TYPE_UNKNOWN;
#ifdef CONFIG_PRODUCT_DOOM
	bq->user_health_charge = 0;
	bq->user_charging_enabled = 1;
	bq->user_input_suspend = 0;
	bq->fac_test_enable = 0;
	bq->charge_thermal_status = 0;
	bq->adsp_icl = -1;
	bq->adsp_fcc = -1;
	bq->adsp_fv = -1;
	bq->adsp_cp_thermal = -1;
#ifdef CONFIG_BATTERY_FAC
	bq->port1_fac_pd_en = 1;
	bq->port2_fac_pd_en = 1;
#endif
#endif

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		pr_info("charger device bq25890 detected, revision:%d\n",
									bq->revision);
	} else {
		pr_info("no bq25890H charger device found:%d\n",ret);
		return -ENODEV;
	}


	pe.enable = false;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = bq2589x_psy_register(bq);
	if (ret)
		goto err_0;


	ret = bq2589x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2589x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->pe_work, bq2589x_pe_workfunc);
#ifdef CONFIG_PRODUCT_DOOM
	INIT_DELAYED_WORK(&bq->lenovo_monitor_ports_status_work,
					lenovo_monitor_ports_status_work);
	schedule_delayed_work(&bq->lenovo_monitor_ports_status_work,
				msecs_to_jiffies(LENOVE_WORK_MS));

	INIT_DELAYED_WORK(&bq->lenovo_battery_monitor_work,
					lenovo_battery_monitor_work);
	INIT_DELAYED_WORK(&bq->lenovo_thermal_monitor_work,
					lenovo_thermal_monitor_work);
	INIT_DELAYED_WORK(&bq->lenovo_charge_monitor_work,
					lenovo_charge_monitor_work);
	INIT_DELAYED_WORK(&bq->lenovo_insert_delay_work,
					lenovo_insert_delay_work);
	INIT_DELAYED_WORK(&bq->lenovo_bootup_delay_work,
					lenovo_bootup_delay_work);
	INIT_DELAYED_WORK(&bq->lenovo_wr_pdo_bootup_delay_work,
					lenovo_wr_pdo_bootup_delay_work);
	INIT_DELAYED_WORK(&bq->lenovo_icl_settled_work,
					lenovo_icl_settled_work);
	INIT_DELAYED_WORK(&bq->lenovo_restart_pd_work,
					lenovo_restart_pd_work);

	INIT_DELAYED_WORK(&bq->lenovo_ffc_ctrl_work,
					lenovo_ffc_ctrl_workfunc);

	schedule_delayed_work(&bq->lenovo_ffc_ctrl_work,
			msecs_to_jiffies(5000));
#endif
	bq2589x_init_jeita(bq);


	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2589x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2589x_charger1_irq", bq);
		if (ret) {
			pr_err("Request IRQ %d failed: %d\n", client->irq, ret);
			goto err_irq;
		} else {
			pr_info("irq = %d\n", client->irq);
		}
		enable_irq_wake(client->irq);
	}

	bq2589x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, true);



#ifdef DBG_FS
	create_debugfs_entry(bq);
#endif
	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}


	determine_initial_status(bq);
#ifdef CONFIG_PRODUCT_DOOM
	lenovo_init_charger_path_switch_gpio(bq);
#endif
	bq2589x_dump_reg(bq);
	bq2589x_dump_status(bq);
	pr_err(" probe successfully\n");
#ifdef CONFIG_PRODUCT_DOOM
	schedule_delayed_work(&bq->lenovo_battery_monitor_work,
				msecs_to_jiffies(1500));
	schedule_delayed_work(&bq->lenovo_thermal_monitor_work,
				msecs_to_jiffies(2000));
	schedule_delayed_work(&bq->lenovo_charge_monitor_work,
				msecs_to_jiffies(6000));
	schedule_delayed_work(&bq->lenovo_bootup_delay_work,
				msecs_to_jiffies(40000));
	schedule_delayed_work(&bq->lenovo_wr_pdo_bootup_delay_work,
				msecs_to_jiffies(25000));
#endif

	return 0;

err_irq:

err_0:
	bq2589x_psy_unregister(bq);
	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

	return ret;
}

static inline bool is_device_suspended(struct bq2589x *bq)
{
	return !bq->resume_completed;
}

static int bq2589x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	pr_err("Suspend successfully!");

	return 0;
}

static int bq2589x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2589x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	lenovo_supply_changed(bq->fc_main_psy);
	pr_err("Resume successfully!");

	return 0;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2589x_psy_unregister(bq);

	cancel_delayed_work(&bq->ico_work);
	cancel_delayed_work(&bq->pe_work);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

#ifdef DBG_FS
	debugfs_remove_recursive(bq->debug_root);
#endif

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	return 0;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);
	pr_info("shutdown..");
	bq2589x_charging_disable(bq, SYS_OFF, 1);
        bq2589x_adc_stop(bq);
}

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq2589x-charger",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x-charger", BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

const struct dev_pm_ops bq2589x_pm = {
	.suspend = bq2589x_suspend,
	.resume = bq2589x_resume,
};

static const struct dev_pm_ops bq2589x_pm_ops = {
	.resume		= bq2589x_resume,
	.suspend_noirq = bq2589x_suspend_noirq,
	.suspend	= bq2589x_suspend,
};

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq25890h",
		.of_match_table = bq2589x_charger_match_table,
		.pm	= &bq2589x_pm_ops,
	},
	.id_table	= bq2589x_charger_id,

	.probe		= bq2589x_charger_probe,
	.shutdown  	= bq2589x_charger_shutdown,
	.remove		= bq2589x_charger_remove,
};

module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

