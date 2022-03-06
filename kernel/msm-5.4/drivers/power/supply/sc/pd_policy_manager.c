
#define pr_fmt(fmt)	"[USBPD-PM]: %s: " fmt, __func__

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/lenovo_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
//#include <linux/usb/usbpd.h>
#include "pd_policy_manager.h"

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define BATT_MAX_CHG_VOLT		4400
#define BATT_FAST_CHG_CURR		8000
#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#define BUS_VOLT_INIT_UP		600

#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD

#define PM_WORK_RUN_INTERVAL		500

#ifdef CONFIG_PRODUCT_DOOM
extern bool fusb_pd_enable;
struct usbpd_pm *bq_usbpd_pm = NULL;
extern PPSStatus_t *fusb_pps_st;
extern struct charger_object* fusb_fetch_pdo_chip;
extern struct fusb30x_chip* fusb_pps_chip;
int is_pd2_done = 0;
extern int cp_max_curr;
extern bool is_batt2_thermal_stop;
#endif

enum {
	PM_ALGO_RET_OK,
	PM_ALGO_RET_THERM_FAULT,
	PM_ALGO_RET_OTHER_FAULT,
	PM_ALGO_RET_CHG_DISABLED,
	PM_ALGO_RET_TAPER_DONE,
};

static const struct pdpm_config pm_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT,
	.bus_volt_lp_lmt		= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt		= (BAT_CURR_LOOP_LMT >> 1),

	.fc2_taper_current		= 2000,
	.fc2_steps				= 1,

	.min_adapter_volt_required	= 11000,
	.min_adapter_curr_required	= 2000,

	.min_vbat_for_cp		= 3500,

	.cp_sec_enable         = true,
	.fc2_disable_sw			= true,
};

static struct usbpd_pm *sc_pdpm;

static int fc2_taper_timer;
static int ibus_lmt_change_timer;


static void usbpd_check_usb_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->usb_psy) {
		pdpm->usb_psy = lenovo_supply_get_by_name("usb");
		if (!pdpm->usb_psy)
			pr_err("usb psy not found!\n");
	}
}
static void usbpd_check_cp_psy(struct usbpd_pm *pdpm)
{
    if (!pdpm->cp_psy) {
        if (pm_config.cp_sec_enable)
			pdpm->cp_psy = lenovo_supply_get_by_name("sc8551-master");
        else
            pdpm->cp_psy = lenovo_supply_get_by_name("sc8551-standalone");
        if (!pdpm->cp_psy)
            pr_err("cp_psy not found\n");
    }
}

static void usbpd_check_cp_sec_psy(struct usbpd_pm *pdpm)
{
    if (!pdpm->cp_sec_psy) {
        pdpm->cp_sec_psy = lenovo_supply_get_by_name("sc8551-slave");
        if (!pdpm->cp_sec_psy)
            pr_err("cp_sec_psy not found\n");
    }
}

static void usbpd_pm_update_cp_status(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);
	if (!pdpm->cp_psy)
		return;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BATTERY_VOLTAGE, &val);
	if (!ret)
		pdpm->cp.vbat_volt = val.intval; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BATTERY_CURRENT, &val);
	if (!ret)
		pdpm->cp.ibat_curr_cp = val.intval;

	pdpm->cp.ibat_curr = pdpm->cp.ibat_curr_cp ;//+ pdpm->cp.ibat_curr_sw; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BUS_VOLTAGE, &val);
	if (!ret)
		pdpm->cp.vbus_volt = val.intval; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BUS_CURRENT, &val);
	if (!ret)
		pdpm->cp.ibus_curr_cp = val.intval;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_VBUS_ERROR_STATUS, &val);
	if (!ret)
	{
		pr_err("vbus error state : %02x\n", val.intval);
		pdpm->cp.vbus_error_low = (val.intval >> 5) & 0x01;
		pdpm->cp.vbus_error_high = (val.intval >> 4) & 0x01;
	}
		
	pdpm->cp.ibus_curr = pdpm->cp.ibus_curr_cp + pdpm->cp.ibus_curr_sw; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BUS_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.bus_temp = val.intval; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BATTERY_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.bat_temp = val.intval; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_DIE_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.die_temp = val.intval; 

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_BATTERY_PRESENT, &val);
	if (!ret)
		pdpm->cp.batt_pres = val.intval;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_VBUS_PRESENT, &val);
	if (!ret)
		pdpm->cp.vbus_pres = val.intval;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = val.intval;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_ALARM_STATUS, &val);
	if (!ret) {
		pdpm->cp.bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK);
		pdpm->cp.bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK);
		pdpm->cp.bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK);
		pdpm->cp.bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK);
		pdpm->cp.bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK);
		pdpm->cp.bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		pdpm->cp.bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		pdpm->cp.die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_SC_FAULT_STATUS, &val);
	if (!ret) {
		pdpm->cp.bat_ovp_fault = !!(val.intval & BAT_OVP_FAULT_MASK);
		pdpm->cp.bat_ocp_fault = !!(val.intval & BAT_OCP_FAULT_MASK);
		pdpm->cp.bus_ovp_fault = !!(val.intval & BUS_OVP_FAULT_MASK);
		pdpm->cp.bus_ocp_fault = !!(val.intval & BUS_OCP_FAULT_MASK);
		pdpm->cp.bat_therm_fault = !!(val.intval & BAT_THERM_FAULT_MASK);
		pdpm->cp.bus_therm_fault = !!(val.intval & BUS_THERM_FAULT_MASK);
		pdpm->cp.die_therm_fault = !!(val.intval & DIE_THERM_FAULT_MASK);
	}
}

static void usbpd_pm_update_cp_sec_status(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pm_config.cp_sec_enable)
		return;

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return;

	ret = lenovo_supply_get_property(pdpm->cp_sec_psy,
			LENOVO_SUPPLY_PROP_SC_BUS_CURRENT, &val);
	if (!ret)
		pdpm->cp_sec.ibus_curr = val.intval;

	ret = lenovo_supply_get_property(pdpm->cp_sec_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = val.intval;
}


static int usbpd_pm_enable_cp(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = lenovo_supply_set_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	pr_info("usbpd_pm_enable_cp enable is %d\n", enable);
	return ret;
}

static int usbpd_pm_enable_cp_sec(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	val.intval = enable;
	ret = lenovo_supply_set_property(pdpm->cp_sec_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	pr_info("usbpd_pm_enable_cp_sec enable is %d\n", enable);

	return ret;
}

static int usbpd_pm_check_cp_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	ret = lenovo_supply_get_property(pdpm->cp_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	pr_info("usbpd_pm_check_enable_cp enable is %d\n", pdpm->cp.charge_enabled);

	return ret;
}

static int usbpd_pm_check_cp_sec_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	ret = lenovo_supply_get_property(pdpm->cp_sec_psy,
			LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = !!val.intval;

	pr_info("usbpd_pm_check_enable_cp_sec enable is %d\n", pdpm->cp_sec.charge_enabled);

	return ret;
}

#ifdef CONFIG_PRODUCT_DOOM
static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = lenovo_supply_get_by_name("bq2589h-charger");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	if(enable)   val.intval = 3000000;
	else         val.intval = 100000;

	ret = lenovo_supply_set_property(pdpm->sw_psy,
			LENOVO_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &val);
	pr_info("call usbpd_pm_enable_sw enable is %d\n", enable);
	return ret;
}
#else
static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = lenovo_supply_get_by_name("bq2589h-charger");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	if(enable)   val.intval = 3000000;
	else         val.intval = 100000;

	ret = lenovo_supply_set_property(pdpm->sw_psy, 
			LENOVO_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);
	
	return ret;
}
#endif
static int usbpd_pm_check_sw_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = lenovo_supply_get_by_name("bq2589h-charger");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	ret = lenovo_supply_get_property(pdpm->sw_psy, 
			LENOVO_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,&val);
	if (!ret)
	{
		if(val.intval == 100000)  pdpm->sw.charge_enabled = false;
		else  pdpm->sw.charge_enabled = true;
	}

	return ret;
}

static void usbpd_pm_update_sw_status(struct usbpd_pm *pdpm)
{
	usbpd_pm_check_sw_enabled(pdpm);
}
#ifdef CONFIG_PRODUCT_DOOM
static int charger2a_pd_max_power = 0;
extern void reset_charger2_pd_power_data(void);

int get_charger2a_max_power(void) {
	return charger2a_pd_max_power;
}
EXPORT_SYMBOL_GPL(get_charger2a_max_power);

void reset_charger2a_pd_power_data(void) {
	charger2a_pd_max_power = 0;
}
EXPORT_SYMBOL_GPL(reset_charger2a_pd_power_data);

static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{

	if (fusb_pd_enable){
		pdpm->apdo_selected_pdo =fusb_fetch_pdo_chip->requested_pdo;
		pdpm->apdo_max_volt = fusb_fetch_pdo_chip->req_voltage*20;
		pdpm->apdo_max_curr = fusb_fetch_pdo_chip->req_current*50;
		pdpm->pps_supported = true;
		pr_err("sc8551 src cap apdo_obj = %d, apdo_max_vol = %d, apdo_max_curr = %d\n",
			pdpm->apdo_selected_pdo, pdpm->apdo_max_volt, pdpm->apdo_max_curr);
	}
	if (pdpm->pps_supported) {
		pr_notice("PPS supported, preferred APDO pos:%d, max volt:%d, current:%d\n",
				pdpm->apdo_selected_pdo,
				pdpm->apdo_max_volt,
				pdpm->apdo_max_curr);
		charger2a_pd_max_power = pdpm->apdo_max_volt * pdpm->apdo_max_curr / 1000000;
	} else
		pr_notice("Not qualified PPS adapter\n");
}
#else
static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{
	int ret;
	int i;

	if (!pdpm->pd) {
		pdpm->pd = smb_get_usbpd();
		if (!pdpm->pd) {
			pr_err("couldn't get usbpd device\n");
			return;
		}
	}

	ret = usbpd_fetch_pdo(pdpm->pd, pdpm->pdo);
	if (ret) {
		pr_err("Failed to fetch pdo info\n");
		return;
	}

	pdpm->apdo_max_volt = pm_config.min_adapter_volt_required;
	pdpm->apdo_max_curr = pm_config.min_adapter_curr_required;

	for (i = 0; i < 7; i++) {
		pr_err("[SC manager] %d type %d\n", i, pdpm->pdo[i].type);

		if (pdpm->pdo[i].type == PD_SRC_PDO_TYPE_AUGMENTED 
			&& pdpm->pdo[i].pps && pdpm->pdo[i].pos) {
			if (pdpm->pdo[i].max_volt_mv >= pdpm->apdo_max_volt
					&& pdpm->pdo[i].curr_ma > pdpm->apdo_max_curr) {
				pdpm->apdo_max_volt = pdpm->pdo[i].max_volt_mv;
				pdpm->apdo_max_curr = pdpm->pdo[i].curr_ma;
				pdpm->apdo_selected_pdo = pdpm->pdo[i].pos;
				pdpm->pps_supported = true;
				pr_err("[SC manager] vola %d  curr %d\n", 
						pdpm->apdo_max_volt, pdpm->apdo_max_curr);
			}		
		}
	}
	
	if (pdpm->pps_supported)
		pr_notice("PPS supported, preferred APDO pos:%d, max volt:%d, current:%d\n",
				pdpm->apdo_selected_pdo,
				pdpm->apdo_max_volt,
				pdpm->apdo_max_curr);
	else
		pr_notice("Not qualified PPS adapter\n");
}
#endif
#if 0
static void usbpd_update_pps_status(struct usbpd_pm *pdpm)
{
	int ret;
	u32 status;

	ret = usbpd_get_pps_status(pdpm->pd, &status);

	if (!ret) {
		/*TODO: check byte order to insure data integrity*/
		pdpm->adapter_voltage = ((status >> 16) & 0xFFFF )* 20;
		pdpm->adapter_current = ((status >> 8) & 0xFF ) * 50;
		pdpm->adapter_ptf = (status & 0x06) >> 1;
		pdpm->adapter_omf = !!(status & 0x08);
		pr_err("adapter_volt:%d, adapter_current:%d\n",
				pdpm->adapter_voltage, pdpm->adapter_current);
	}
}
#endif
static int usbpd_update_ibat_curr(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pdpm->bms_psy) {
		pdpm->bms_psy = lenovo_supply_get_by_name("bms");
		if (!pdpm->bms_psy) {
			return -ENODEV;
		}
	}

	ret = lenovo_supply_get_property(pdpm->bms_psy, 
			LENOVO_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		pdpm->cp.ibat_curr_sw= -(int)(val.intval/1000);

	ret = lenovo_supply_get_property(pdpm->bms_psy, 
			LENOVO_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		pdpm->cp.vbat_volt = (int)(val.intval/1000);

	return ret;
}


static int usbpd_update_ibus_curr(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = lenovo_supply_get_by_name("usb");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	ret = lenovo_supply_get_property(pdpm->sw_psy, 
			LENOVO_SUPPLY_PROP_INPUT_CURRENT_NOW, &val);
	if (!ret)
		pdpm->cp.ibus_curr_sw = (int)(val.intval/1000);

	return ret;
}

/*static void usbpd_pm_disconnect(struct usbpd_pm *pdpm);
static void usb_psy_pd_active_update(struct usbpd_pm *pdpm)
{
	int ret;
	union lenovo_supply_propval val = {0,};

	ret = lenovo_supply_get_property(pdpm->usb_psy,
			LENOVO_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		pr_err("Failed to get usb pd active state\n");
		return;
	}

	if(!val.intval)
	{
		pdpm->pd_active = 0;
	}
	else{
		pdpm->pd_active = 1;
	}
}*/


#define TAPER_TIMEOUT	(50)
#define IBUS_CHANGE_TIMEOUT  (5)
static int usbpd_pm_fc2_charge_algo(struct usbpd_pm *pdpm)
{
	int steps;
	int sw_ctrl_steps = 0;
	int hw_ctrl_steps = 0;
	int step_vbat = 0;
	int step_ibus = 0;
	int step_ibat = 0;
	int step_bat_reg = 0;
	int ibus_total = 0;

	static int ibus_limit;

	if (ibus_limit == 0)
		ibus_limit = pm_config.bus_curr_lp_lmt;// + 400;

	/* reduce bus current in cv loop */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50) {
		if (ibus_lmt_change_timer++ > IBUS_CHANGE_TIMEOUT) {
			ibus_lmt_change_timer = 0;
			ibus_limit = pm_config.bus_curr_lp_lmt;// - 400;
		}
	} else if (pdpm->cp.vbat_volt < pm_config.bat_volt_lp_lmt - 250) {
		ibus_limit = pm_config.bus_curr_lp_lmt;// + 400;
		ibus_lmt_change_timer = 0;
	} else {
		ibus_lmt_change_timer = 0;
	}

	/* battery voltage loop*/
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt)
		step_vbat = -pm_config.fc2_steps;
	else if (pdpm->cp.vbat_volt < pm_config.bat_volt_lp_lmt - 7)
		step_vbat = pm_config.fc2_steps;;


	/* battery charge current loop*/
	if (pdpm->cp.ibat_curr < pm_config.bat_curr_lp_lmt )
		step_ibat = pm_config.fc2_steps;
	else if (pdpm->cp.ibat_curr > pm_config.bat_curr_lp_lmt + 100)
		step_ibat = -pm_config.fc2_steps;


    /* bus current loop*/
    ibus_total = pdpm->cp.ibus_curr;

    if (pm_config.cp_sec_enable)
		ibus_total += pdpm->cp_sec.ibus_curr;

	if (ibus_total < ibus_limit - 50)
		step_ibus = pm_config.fc2_steps;
	else if (ibus_total > ibus_limit)
		step_ibus = -pm_config.fc2_steps;

	/* hardware regulation loop*/
	/*if (pdpm->cp.vbat_reg || pdpm->cp.ibat_reg)
		step_bat_reg = 5 * (-pm_config.fc2_steps);
	else
		step_bat_reg = pm_config.fc2_steps;*/
	step_bat_reg = pm_config.fc2_steps;

	sw_ctrl_steps = min(min(step_vbat, step_ibus), step_ibat);
	sw_ctrl_steps = min(sw_ctrl_steps, step_bat_reg);

	/* hardware alarm loop */
	if (pdpm->cp.bat_ocp_alarm /*|| pdpm->cp.bat_ovp_alarm */
		|| pdpm->cp.bus_ocp_alarm || pdpm->cp.bus_ovp_alarm
		/*|| pdpm->cp.tbat_temp > 60
		  || pdpm->cp.tbus_temp > 50*/)
		hw_ctrl_steps = -pm_config.fc2_steps;
	else
		hw_ctrl_steps = pm_config.fc2_steps;

    /* check if cp disabled due to other reason*/
    usbpd_pm_check_cp_enabled(pdpm);
    pr_err("cp enable bit %d\n", pdpm->cp.charge_enabled);
    if (pm_config.cp_sec_enable) {
        usbpd_pm_check_cp_sec_enabled(pdpm);
        pr_err("cp sec enable bit %d\n", pdpm->cp_sec.charge_enabled);
    }

    if (pdpm->cp.bat_therm_fault ) { /* battery overheat, stop charge*/
        pr_notice("bat_therm_fault:%d\n", pdpm->cp.bat_therm_fault);
        return PM_ALGO_RET_THERM_FAULT;
    } else if (pdpm->cp.bat_ocp_fault || pdpm->cp.bus_ocp_fault
            || pdpm->cp.bat_ovp_fault || pdpm->cp.bus_ovp_fault) {
        pr_notice("bat_ocp_fault:%d, bus_ocp_fault:%d, bat_ovp_fault:%d, \
                bus_ovp_fault:%d\n", pdpm->cp.bat_ocp_fault,
                pdpm->cp.bus_ocp_fault, pdpm->cp.bat_ovp_fault,
                pdpm->cp.bus_ovp_fault);
            return PM_ALGO_RET_OTHER_FAULT; /* go to switch, and try to ramp up*/
    } else if ((!pdpm->cp.charge_enabled && (pdpm->cp.vbus_error_low
                || pdpm->cp.vbus_error_high)) || (pm_config.cp_sec_enable && !pdpm->cp_sec.charge_enabled && !pdpm->cp_sec_stopped)) {
        pr_notice("cp.charge_enabled:%d  %d  %d\n",
                pdpm->cp.charge_enabled, pdpm->cp.vbus_error_low, pdpm->cp.vbus_error_high);
        return PM_ALGO_RET_CHG_DISABLED;
    }

	/* charge pump taper charge */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
			&& pdpm->cp.ibat_curr < pm_config.fc2_taper_current) {
		if (fc2_taper_timer++ > TAPER_TIMEOUT) {
			pr_notice("charge pump taper charging done\n");
			fc2_taper_timer = 0;
			return PM_ALGO_RET_TAPER_DONE;
		}
	} else {
		fc2_taper_timer = 0;
	}

	/*TODO: customer can add hook here to check system level
	 * thermal mitigation*/

	steps = min(sw_ctrl_steps, hw_ctrl_steps);

	pr_err("%d %d %d sw %d hw %d all %d\n",
			step_vbat, step_ibat, step_ibus, sw_ctrl_steps, hw_ctrl_steps, steps);

	pdpm->request_voltage += steps * 20;

	if (pdpm->request_voltage > pdpm->apdo_max_volt - 1000)
		pdpm->request_voltage = pdpm->apdo_max_volt - 1000;

	/*if (pdpm->adapter_voltage > 0
			&& pdpm->request_voltage > pdpm->adapter_voltage + 500)
		pdpm->request_voltage = pdpm->adapter_voltage + 500;*/

	return PM_ALGO_RET_OK;
}

static const unsigned char *pm_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_FC2_ENTRY",
	"PD_PM_STATE_FC2_ENTRY_1",
	"PD_PM_STATE_FC2_ENTRY_2",
	"PD_PM_STATE_FC2_ENTRY_3",
	"PD_PM_STATE_FC2_TUNE",
	"PD_PM_STATE_FC2_EXIT",
};

static void usbpd_pm_move_state(struct usbpd_pm *pdpm, enum pm_state state)
{
#if 1
	pr_err("state change:%s -> %s\n", 
		pm_str[pdpm->state], pm_str[state]);
#endif
	pdpm->state = state;
}
#ifdef CONFIG_PRODUCT_DOOM
static int usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	int ret;
	int rc = 0;
	static int tune_vbus_retry;
	static bool stop_sw;
	static bool recover;


        FSC_U16 report_vol = 0;
        FSC_U8 report_curr=0;
        FSC_U8 obj = 0;
        pdpm->port = fusb_fetch_pdo_chip->port;

	pr_info("state phase :%d\n", pdpm->state);
	pr_info("vbus_vol = %d, vbat_vol = %d\n", pdpm->cp.vbus_volt, pdpm->cp.vbat_volt);
	pr_info("ibus_curr = %d, ibat_curr = %d\n", pdpm->cp.ibus_curr, pdpm->cp.ibat_curr);
	if (is_batt2_thermal_stop){
		recover = true;
		pr_info("stop port2 charge pump for thermal\n");
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		}
	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		stop_sw = false;
		recover = false;

		if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp) {
			pr_notice("batt_volt-%d, waiting...\n", pdpm->cp.vbat_volt);
		} else if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 100) {
			pr_notice("batt_volt-%d is too high for cp,\
					charging with switch charger\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		} else {
			pr_notice("batt_volt-%d is ok, start flash charging\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY);
			is_pd2_done = 1;
		}
		break;

	case PD_PM_STATE_FC2_ENTRY:
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		break;
	case PD_PM_STATE_FC2_ENTRY_1:
		pdpm->request_voltage = pdpm->cp.vbat_volt * 210/100;

		pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);

		if (pdpm->request_current > cp_max_curr)
			pdpm->request_current = cp_max_curr;

                report_vol = (FSC_U16) (pdpm->request_voltage/20);
                report_curr = (FSC_U8)(pdpm->request_current/50);
                obj = (FSC_U8)pdpm->apdo_selected_pdo;

                DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
                core_state_machine(pdpm->port);

                pr_info("FC2_ENTRY_1:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
                        pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);


		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_2);

		tune_vbus_retry = 0;
		break;
	case PD_PM_STATE_FC2_ENTRY_2:
		pr_err("tune_vbus_retry %d\n", tune_vbus_retry);

		pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);
		if (pdpm->request_current > cp_max_curr)
			pdpm->request_current = cp_max_curr;

		if (pdpm->cp.vbus_error_low || pdpm->cp.vbus_volt < pdpm->cp.vbat_volt * 207/100) {
			tune_vbus_retry++;
			pdpm->request_voltage += 20;

                report_vol = (FSC_U16)(pdpm->request_voltage/20);
                report_curr = (FSC_U8)(pdpm->request_current/50);
                obj = (FSC_U8)pdpm->apdo_selected_pdo;
                DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
                core_state_machine(pdpm->port);

                pr_info("FC2_ENTRY_2_1:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
                        pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);


		} else if (pdpm->cp.vbus_error_high || pdpm->cp.vbus_volt > pdpm->cp.vbat_volt * 210/100) {
			tune_vbus_retry++;
			pdpm->request_voltage -= 20;

                report_vol = (FSC_U16)(pdpm->request_voltage/20);
                report_curr = (FSC_U8)(pdpm->request_current/50);
                obj = (FSC_U8)pdpm->apdo_selected_pdo;

                DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
                core_state_machine(pdpm->port);

                pr_info("FC2_ENTRY_2_2:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
                        pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);

		} else {
			pr_notice("adapter volt tune ok, retry %d times\n", tune_vbus_retry);
		    usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_3);
			break;
		}

		if (tune_vbus_retry > 30) {
			pr_notice("Failed to tune adapter volt into valid range, \
					charge with switching charger\n");
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		}
		break;
	case PD_PM_STATE_FC2_ENTRY_3:
		usbpd_pm_check_cp_enabled(pdpm);
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		usbpd_pm_check_cp_sec_enabled(pdpm);
		if(!pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm, true);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}

		if (pdpm->cp.charge_enabled) {
		      if ((pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled)
                    || !pm_config.cp_sec_enable) {
			}
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
			ibus_lmt_change_timer = 0;
			fc2_taper_timer = 0;
		}
		break;

	case PD_PM_STATE_FC2_TUNE:
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
		}
		pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);
		if (pdpm->request_current > cp_max_curr)
			pdpm->request_current = cp_max_curr;

		ret = usbpd_pm_fc2_charge_algo(pdpm);
		if (ret == PM_ALGO_RET_THERM_FAULT) {
			pr_notice("Move to stop charging:%d\n", ret);
			stop_sw = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_OTHER_FAULT || ret == PM_ALGO_RET_TAPER_DONE) {
			pr_notice("Move to switch charging:%d\n", ret);
			stop_sw = false;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_CHG_DISABLED) {
			pr_notice("Move to switch charging, will try to recover \
					flash charging:%d\n", ret);
			recover = true;
			stop_sw = false;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else {
			pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);
			if (pdpm->request_current > cp_max_curr)
				pdpm->request_current = cp_max_curr;

			report_vol = (FSC_U16)(pdpm->request_voltage/20);
			report_curr = (FSC_U8)(pdpm->request_current/50);
			obj = (FSC_U8)pdpm->apdo_selected_pdo;

	                DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
	                core_state_machine(pdpm->port);

	                pr_info("FC2_TUNE:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
	                        pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);
		}
		/*stop second charge pump if either of ibus is lower than 750ma during CV*/
		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled
				&& pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
				&& (pdpm->cp.ibus_curr < 750 || pdpm->cp_sec.ibus_curr < 750)) {
			pr_notice("second cp is disabled due to ibus < 750mA\n");
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
			pdpm->cp_sec_stopped = true;
		}
		break;
	case PD_PM_STATE_FC2_EXIT:
		if (pdpm->cp.charge_enabled) {
			pr_info("enter FC2_EXIT,  disable cp\n");
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_check_cp_enabled(pdpm);
		}
		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}

		pr_info("sw state %d   %d\n", stop_sw, pdpm->sw.charge_enabled);

		if (stop_sw && pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm, false);
		else if (!stop_sw && !pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm, true);

		usbpd_pm_check_sw_enabled(pdpm);

		if (recover)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		else{
			rc = 1;
			is_pd2_done = 2;
		}

	        break;
	}
	return rc;
}
#else
static int usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	int ret;
	int rc = 0;
	static int tune_vbus_retry;
	static bool stop_sw;
	static bool recover;

	pr_err(">>>>>>>>>>>state phase :%d\n", pdpm->state);
	pr_err(">>>>>vbus_vol %d    vbat_vol %d   vout %d\n", pdpm->cp.vbus_volt, pdpm->cp.vbat_volt, pdpm->cp.vout_volt);
	pr_err(">>>>>ibus_curr %d    ibat_curr %d\n", pdpm->cp.ibus_curr, pdpm->cp.ibat_curr);
	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		stop_sw = false;
		recover = false;

		if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp) {
			pr_notice("batt_volt-%d, waiting...\n", pdpm->cp.vbat_volt);
		} else if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 100) {
			pr_notice("batt_volt-%d is too high for cp,\
					charging with switch charger\n", 
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		} else {
			pr_notice("batt_volt-%d is ok, start flash charging\n", 
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY);
		}
		break;

	case PD_PM_STATE_FC2_ENTRY:
		if (pm_config.fc2_disable_sw) {
			if (pdpm->sw.charge_enabled) {
				usbpd_pm_enable_sw(pdpm, false);
				usbpd_pm_check_sw_enabled(pdpm);
			}
			if (!pdpm->sw.charge_enabled)
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		} else {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		}
		break;

	case PD_PM_STATE_FC2_ENTRY_1:
		pdpm->request_voltage = pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP;
			
		pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);

		usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
				pdpm->request_voltage * 1000, pdpm->request_current * 1000);
		pr_err("request_voltage:%d, request_current:%d\n",
				pdpm->request_voltage, pdpm->request_current);
	
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_2);

		tune_vbus_retry = 0;
		break;

	case PD_PM_STATE_FC2_ENTRY_2:
		pr_err("tune_vbus_retry %d\n", tune_vbus_retry);
		if (pdpm->cp.vbus_error_low && pdpm->cp.vbus_volt > pdpm->cp.vbat_volt * 207/100) {//pdpm->cp.vbus_volt < (pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP - 50)
			tune_vbus_retry++;
			pdpm->request_voltage += 20;
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_err("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		} else if (pdpm->cp.vbus_error_high && pdpm->cp.vbus_volt < pdpm->cp.vbat_volt * 219/100) {//pdpm->cp.vbus_volt > (pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP + 50)
			tune_vbus_retry++;
			pdpm->request_voltage -= 20;
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_err("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		} else {
			pr_notice("adapter volt tune ok, retry %d times\n", tune_vbus_retry);
		    usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_3);
			break;
		}
		
		if (tune_vbus_retry > 25) {
			pr_notice("Failed to tune adapter volt into valid range, \
					charge with switching charger\n");
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		}	
		break;
	case PD_PM_STATE_FC2_ENTRY_3:
		usbpd_pm_check_cp_enabled(pdpm);
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pdpm->cp.charge_enabled) {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
			ibus_lmt_change_timer = 0;
			fc2_taper_timer = 0;
		}
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
		break;

	case PD_PM_STATE_FC2_TUNE:

		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
		}

		ret = usbpd_pm_fc2_charge_algo(pdpm);
		if (ret == PM_ALGO_RET_THERM_FAULT) {
			pr_notice("Move to stop charging:%d\n", ret);
			stop_sw = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_OTHER_FAULT || ret == PM_ALGO_RET_TAPER_DONE) {
			pr_notice("Move to switch charging:%d\n", ret);
			stop_sw = false;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_CHG_DISABLED) {
			pr_notice("Move to switch charging, will try to recover \
					flash charging:%d\n", ret);
			recover = true;
			stop_sw = false;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else {
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_err("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		}
	
		
		/*stop second charge pump if either of ibus is lower than 750ma during CV*/
		/*if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
				&& (pdpm->cp.ibus_curr < 750)) {
			pr_notice("second cp is disabled due to ibus < 750mA\n");
		}*/
		break;

	case PD_PM_STATE_FC2_EXIT:
		/* select default 5V*/
		usbpd_select_pdo(pdpm->pd, 1, 0, 0);

		if (pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		pr_err("sw state %d   %d\n", stop_sw, pdpm->sw.charge_enabled);
		if (stop_sw && pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm, false);
		else if (!stop_sw && !pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm, true);
			
		usbpd_pm_check_sw_enabled(pdpm);

		if (recover)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		else
			rc = 1;
		
	        break;
	}

	return rc;
}
#endif
static void usbpd_pm_workfunc(struct work_struct *work)
{
	//struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
	//				pm_work);
#ifdef CONFIG_PRODUCT_DOOM
	if (fusb_fetch_pdo_chip == NULL)
		return;
#endif
	usbpd_pm_update_sw_status(sc_pdpm);
	usbpd_update_ibus_curr(sc_pdpm);
	usbpd_pm_update_cp_status(sc_pdpm);
	usbpd_pm_update_cp_sec_status(sc_pdpm);
	usbpd_update_ibat_curr(sc_pdpm);

	if ((!usbpd_pm_sm(sc_pdpm)) &&fusb_fetch_pdo_chip->fusb302_pd_active)
		schedule_delayed_work(&sc_pdpm->pm_work,
				msecs_to_jiffies(PM_WORK_RUN_INTERVAL));
}

static void usbpd_pm_disconnect(struct usbpd_pm *pdpm)
{
	usbpd_pm_enable_cp(pdpm, false);
	usbpd_pm_enable_cp_sec(pdpm, false);

	is_pd2_done = 0;
	pr_info("disable sc8551 master and slave, is_pd2_done = %d\n", is_pd2_done);

	cancel_delayed_work(&pdpm->pm_work);
	pr_info("charge pump cancle_work\n");

	if (!pdpm->sw.charge_enabled) {
		usbpd_pm_enable_sw(pdpm, true);
		usbpd_pm_check_sw_enabled(pdpm);
	}
	
	pdpm->pps_supported = false;
	pdpm->apdo_selected_pdo = 0;

	usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
}

static void usbpd_pd_contact(struct usbpd_pm *pdpm, bool connected)
{
	pdpm->pd_active = connected;
	pr_info("SC manager:pd_active %d\n",
			pdpm->pd_active);
	if (connected) {
		msleep(10);
		usbpd_pm_evaluate_src_caps(pdpm);
		pr_err("SC manager:start cp charging pps support %d\n", pdpm->pps_supported);
		if (pdpm->pps_supported)
			schedule_delayed_work(&pdpm->pm_work, 0);
	} else {
		usbpd_pm_disconnect(pdpm);
	}
}

static void cp_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					cp_psy_change_work);
#if 0
	union lenovo_supply_propval val = {0,};
	bool ac_pres = pdpm->cp.vbus_pres;
	int ret;

	
	if (!pdpm->cp_psy)
		return;

	ret = lenovo_supply_get_property(pdpm->cp_psy, LENOVO_SUPPLY_PROP_TI_VBUS_PRESENT, &val);
	if (!ret)
		pdpm->cp.vbus_pres = val.intval;

	if (!ac_pres && pdpm->cp.vbus_pres)
		schedule_delayed_work(&pdpm->pm_work, 0);
#endif
	pdpm->psy_change_running = false;
}
#ifdef CONFIG_PRODUCT_DOOM
static void usb_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
		usb_psy_change_work);
	if (fusb_fetch_pdo_chip != NULL){
		pr_info("usb_psy_change_work,fusb302_pdactive is %d\n",
			fusb_fetch_pdo_chip->fusb302_pd_active);
	}

	if ((fusb_fetch_pdo_chip != NULL)  && (fusb_fetch_pdo_chip->fusb302_pd_active)){
		usbpd_pd_contact(pdpm,true);
		pr_info("start charge pump pd work\n");
	}
	else{
		usbpd_pd_contact(pdpm,false);
		pr_info("charge pump work can not start\n");
	}

	pdpm->psy_change_running = false;
}
#else
static void usb_psy_change_work(struct work_struct *work)
{
	int ret;
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					usb_psy_change_work);
	union lenovo_supply_propval val = {0,};

	pr_err("SC manager:usb change work\n");

	ret = lenovo_supply_get_property(pdpm->usb_psy,
			LENOVO_SUPPLY_PROP_TYPEC_POWER_ROLE,&val);
	if (ret) {
		pr_err("Failed to read typec power role\n");
		goto out;
	}

	if (val.intval != LENOVO_SUPPLY_TYPEC_PR_SINK && 
			val.intval != LENOVO_SUPPLY_TYPEC_PR_DUAL)
		goto out;

	ret = lenovo_supply_get_property(pdpm->usb_psy,
			LENOVO_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		pr_err("Failed to get usb pd active state\n");
		goto out;
	}
	pr_err("SC manager: pd_active %d,  val.intval %d\n",
			pdpm->pd_active, val.intval);

	if (!pdpm->pd_active && val.intval)
		usbpd_pd_contact(pdpm, true);
	else if (pdpm->pd_active && !val.intval)
		usbpd_pd_contact(pdpm, false);
out:
	pdpm->psy_change_running = false;
}	
#endif

#ifdef CONFIG_PRODUCT_DOOM
static int usbpd_psy_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	return NOTIFY_OK;
}
#else
static int usbpd_psy_notifier_cb(struct notifier_block *nb, 
			unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, nb);
	struct lenovo_supply *psy = data;
	unsigned long flags;

	//if (event != PSY_EVENT_PROP_CHANGED)
	//	return NOTIFY_OK;

	usbpd_check_cp_psy(pdpm);
	usbpd_check_cp_sec_psy(pdpm);
	usbpd_check_usb_psy(pdpm);

	if (!pdpm->cp_psy || !pdpm->usb_psy)
		return NOTIFY_OK;

	if (psy == pdpm->cp_psy || psy == pdpm->usb_psy) {
		spin_lock_irqsave(&pdpm->psy_change_lock, flags);
		pr_err("SC manager:pdpm->psy_change_running : %d\n", pdpm->psy_change_running);
		if (!pdpm->psy_change_running) {
			pdpm->psy_change_running = true;
			if (psy == pdpm->cp_psy)
				schedule_work(&pdpm->cp_psy_change_work);
			else
				schedule_work(&pdpm->usb_psy_change_work);
		}
		spin_unlock_irqrestore(&pdpm->psy_change_lock, flags);
	}

	return NOTIFY_OK;
}
#endif

static int __init usbpd_pm_init(void)
{
	struct usbpd_pm *pdpm;

	pdpm = kzalloc(sizeof(*pdpm), GFP_KERNEL);
	if (!pdpm)
		return -ENOMEM;

	sc_pdpm = pdpm;

#ifdef CONFIG_PRODUCT_DOOM
	bq_usbpd_pm = pdpm;
#endif

	INIT_WORK(&pdpm->cp_psy_change_work, cp_psy_change_work);
	INIT_WORK(&pdpm->usb_psy_change_work, usb_psy_change_work);

	spin_lock_init(&pdpm->psy_change_lock);

	usbpd_check_cp_psy(pdpm);
	usbpd_check_usb_psy(pdpm);

	INIT_DELAYED_WORK(&pdpm->pm_work, usbpd_pm_workfunc);

	pdpm->nb.notifier_call = usbpd_psy_notifier_cb;
	lenovo_supply_reg_notifier(&pdpm->nb);

	return 0;
}

static void __exit usbpd_pm_exit(void)
{
	lenovo_supply_unreg_notifier(&sc_pdpm->nb);
	cancel_delayed_work(&sc_pdpm->pm_work);
	cancel_work_sync(&sc_pdpm->cp_psy_change_work);
	cancel_work_sync(&sc_pdpm->usb_psy_change_work);

}

module_init(usbpd_pm_init);
module_exit(usbpd_pm_exit);

