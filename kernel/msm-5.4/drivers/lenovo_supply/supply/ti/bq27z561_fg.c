/*
 * bq27z561 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[bq27z561] %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/lenovo_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/battery_charger.h>


#define SYS_ERROR_BATTERY_SOC				15
#define USER_BOTH_BATTERY_ERROR_SOC			16
#define USER_SLAVE_BATTERY_ERROR_SOC			17
//#define USER_MASTER_BATTERY_ERROR_SOC			18
#define USER_MASTER_BATTERY_ERROR_SOC			0
#define USER_BAD_BATTERY_SOC				0
#define USER_BAD_BATTERY_VOLTAGE			0
#define USER_BAD_BATTERY_CURRENT			0
#define USER_BAD_BATTERY_TEMP				-300

int last_batt1_soc = -1;
int last_batt2_soc = -1;
int last_batt1_real_soc = -1;
int last_batt2_real_soc = -1;
int last_user_soc = -1;
int last_cc1 = -1;
int last_cc2 = -1;
int last_charge_full1 = -1;
int last_charge_full2 = -1;
int last_batt1_soc_decimal = -1;
int last_batt2_soc_decimal = -1;
int last_user_soc_decimal = -1;

enum bq_fg_device {
	BQ27Z561_MASTER,
	BQ27Z561_SLAVE,
};

static int bq27z561_mode_data[] = {
	[BQ27Z561_MASTER] = BQ27Z561_MASTER,
	[BQ27Z561_SLAVE] = BQ27Z561_SLAVE,
};

static const unsigned char *device2str[] = {
	"bq27z561-master",
	"bq27z561-slave",
};

#define DEFAULT_CHIP_FW					0
#define DEFAULT_VOLTAGE					0
#define DEFAULT_CURRENT					0
#define DEFAULT_SOH					50
#define DEFAULT_DIE_TEMP				250
#define DEFAULT_FCC					1000
#define DEFAULT_DC					1000
#define DEFAULT_RM					1000
#define DEFAULT_CC					0
#define DEFAULT_TTE					0
#define DEFAULT_TTF					0

#define FG_READ_DELAY_SOH			60000
#define FG_READ_DELAY_DIE_TEMP			1000
#define FG_READ_DELAY_VOLTAGE			1000
#define FG_READ_DELAY_CURRENT			1000
#define FG_READ_DELAY_FCC			1000
#define FG_READ_DELAY_RM			1000
#define FG_READ_DELAY_DC			60000

static u16 last_soh = DEFAULT_SOH;
static int last_die_temp = DEFAULT_DIE_TEMP;
static int last_volt = DEFAULT_VOLTAGE;
static int last_curr = DEFAULT_CURRENT;
static int last_dc = DEFAULT_DC;
static unsigned long soh_jiffies = -1;
static unsigned long die_temp_jiffies = -1;
static unsigned long volt_jiffies = -1;
static unsigned long curr_jiffies = -1;
static unsigned long fcc_jiffies = -1;
static unsigned long dc_jiffies = -1;
static unsigned long rm_jiffies = -1;

u8 is_fake_master_battery = 0;
u8 is_fake_slave_battery = 0;
int debug_temp_enable = 0;
int debug_temp1 = 250;
int debug_temp2 = 250;
static int bq_wake_lock = -1;

#if 0
#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err
#else
#define bq_info(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_dbg(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_err(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_log(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#endif


#define	INVALID_REG_ADDR	0xFF

#define FG_FLAGS_FD				BIT(4)
#define	FG_FLAGS_FC				BIT(5)
#define	FG_FLAGS_DSG				BIT(6)
#define FG_FLAGS_RCA				BIT(9)

int get_batt2_vol;

enum bq_fg_reg_idx {
	BQ_FG_REG_CTRL = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_BATT_STATUS,	/* BatteryStatus */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_RM,		/* Remaining Capacity */
	BQ_FG_REG_CC,		/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DC,		/* Design Capacity */
	BQ_FG_REG_ALT_MAC,	/* AltManufactureAccess*/
	BQ_FG_REG_MAC_CHKSUM,	/* MACChecksum */
	BQ_FG_REG_MI,		/* Measured Current */
	NUM_REGS,
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_CTRL_STATUS		= 0x0000,
	FG_MAC_CMD_DEV_TYPE		= 0x0001,
	FG_MAC_CMD_FW_VER		= 0x0002,
	FG_MAC_CMD_HW_VER		= 0x0003,
	FG_MAC_CMD_IF_SIG		= 0x0004,
	FG_MAC_CMD_CHEM_ID		= 0x0006,
	FG_MAC_CMD_GAUGING		= 0x0021,
	FG_MAC_CMD_SEAL			= 0x0030,
	FG_MAC_CMD_DEV_RESET		= 0x0041,
	FG_MAC_CMD_DEV_NAME		= 0x004A,
	FG_MAC_CMD_DEV_CHEM		= 0x004B,
	FG_MAC_CMD_MANU_NAME		= 0x004C,
	FG_MAC_CMD_MANU_DATE		= 0x004D,
	FG_MAC_CMD_SERIAL_NUMBER	= 0x004E,
	FG_MAC_CMD_ITSTATUS1		= 0x0073,
};


enum {
	SEAL_STATE_RSVED,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
	SEAL_STATE_FA,
};

static u8 bq27z561_regs[NUM_REGS] = {
	0x00,	/* CONTROL */
	0x06,	/* TEMP */
	0x08,	/* VOLT */
	0x14,	/* AVG CURRENT */
	0x0A,	/* FLAGS */
	0x16,	/* Time to empty */
	0x18,	/* Time to full */
	0x12,	/* Full charge capacity */
	0x10,	/* Remaining Capacity */
	0x2A,	/* CycleCount */
	0x2C,	/* State of Charge */
	0x2E,	/* State of Health */
	0x3C,	/* Design Capacity */
	0x3E,	/* AltManufacturerAccess*/
	0x60,	/* MACChecksum */
	0x0C,	/* Measured Current*/
};

static const u8 fg_dump_regs[] = {
	0x00, 0x02, 0x04, 0x06,
	0x08, 0x0A, 0x0C, 0x0E,
	0x10, 0x16, 0x18, 0x1A,
	0x1C, 0x1E, 0x20, 0x28,
	0x2A, 0x2C, 0x2E, 0x30,
	0x66, 0x68, 0x6C, 0x6E,
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;


	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

	/* status tracking */

	bool batt_fc;
	bool batt_fd;	/* full depleted */

	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */

	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_ttf;
	int batt_soc;
	int batt_soh;
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr1;
	int batt_curr2;

	int batt_cyclecnt;	/* cycle count */

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct lenovo_supply *fg_ap_psy;
	struct lenovo_supply *fg_adsp_psy;
	struct lenovo_supply *fg_user_psy;
	struct lenovo_supply *fg_master_psy;
	struct lenovo_supply *fg_slave_psy;
	struct lenovo_supply *qcom_batt_bak_psy;
	struct lenovo_supply *qcom_usb_bak_psy;
	struct lenovo_supply *bq2589x_psy;
	struct lenovo_supply *cp_1_psy;
	struct lenovo_supply *cp_2_psy;
	struct lenovo_supply *cp_3_psy;
	struct lenovo_supply *cp_4_psy;
	struct power_supply *qcom_battery_psy;
	struct power_supply *qcom_batt_psy;
#ifdef SUPPORT_USER_THERMAL_CASE
	struct lenovo_supply *thermal_psy;
#endif
	struct lenovo_supply_desc fg_ap_psy_d;
	struct lenovo_supply_desc fg_adsp_psy_d;
	struct lenovo_supply_desc fg_user_psy_d;
	struct lenovo_supply_desc qcom_batt_bak_psy_d;
	struct lenovo_supply_desc qcom_usb_bak_psy_d;
#ifdef SUPPORT_USER_THERMAL_CASE
	struct lenovo_supply_desc thermal_psy_d;
	int thermal_display_rate_level;
	int thermal_levels_display_rate;
	int *thermal_mitigation_display_rate;
	int thermal_current_display_rate;
	int thermal_speaker_level;
	int thermal_levels_speaker;
	int *thermal_mitigation_speaker;
	int thermal_current_speaker;
	int thermal_modem_5g_level;
	int thermal_levels_modem_5g;
	int *thermal_mitigation_modem_5g;
	int thermal_current_modem_5g;
	int thermal_camera_level;
	int thermal_levels_camera;
	int *thermal_mitigation_camera;
	int thermal_current_camera;
	int thermal_event_level;
	int thermal_levels_event;
	int *thermal_mitigation_event;
	int thermal_current_event;
#endif

	int last_batt_soc;
	int last_batt_soh;
	int last_batt_status;
	int last_batt_volt;
	int last_batt_temp;
	int last_batt_present;
	int last_batt_current;
	int last_batt_charge_full;
	int last_batt_design_full;
	int last_batt_cc;

	struct delayed_work fake_batt_work;
	int fw1_check_done;
	int fw2_check_done;
	int master_fw;
	int check_fake_count;
	int mode;
	int fg_read_enable;
};

static int fg_ap_read_temperature(struct bq_fg_chip *bq);
static int fg_ap_read_rm(struct bq_fg_chip *bq);
static int fg_ap_read_fcc(struct bq_fg_chip *bq);
static int fg_adsp_read_rm(struct bq_fg_chip *bq);
static int fg_adsp_read_fcc(struct bq_fg_chip *bq);

#if 0
static int __fg_write_byte(struct bq_fg_chip *bq, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int fg_write_byte(struct bq_fg_chip *bq, u8 reg, u8 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_byte(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int __fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_word(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

#if 0
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif

static int __fg_read_byte(struct bq_fg_chip *bq, u8 reg, u8 *val)
{
	s32 ret;

	if (is_fake_master_battery && (bq->mode == BQ27Z561_MASTER))
		return 0;
	else if (is_fake_slave_battery && (bq->mode == BQ27Z561_SLAVE))
		return 0;
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8)ret;

	return 0;
}

static int __fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	s32 ret;

	if (is_fake_master_battery && (bq->mode == BQ27Z561_MASTER))
		return 0;
	else if (is_fake_slave_battery && (bq->mode == BQ27Z561_SLAVE))
		return 0;
	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}


static int __fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{

	int ret;

	if (is_fake_master_battery && (bq->mode == BQ27Z561_MASTER))
		return 0;
	else if (is_fake_slave_battery && (bq->mode == BQ27Z561_SLAVE))
		return 0;

	ret = i2c_smbus_read_i2c_block_data(bq->client, reg, len, buf);

	return ret;
}

static int __fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (is_fake_master_battery && (bq->mode == BQ27Z561_MASTER))
		return 0;
	else if (is_fake_slave_battery && (bq->mode == BQ27Z561_SLAVE))
		return 0;
	ret = i2c_smbus_write_i2c_block_data(bq->client, reg, len, buf);

	return ret;
}


static int fg_read_byte(struct bq_fg_chip *bq, u8 reg, u8 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_byte(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

static void fg_dump_registers(struct bq_fg_chip *bq)
{
	int i;
	int ret;
	u16 val;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			bq_err("Reg[%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}
}


static int lenovo_glink_write(enum lenovo_supply_property prop, int val)
{
#ifdef CONFIG_LENOVO_SUPPLY
	int rc;
	int data;

	data = val;
        rc = qti_battery_charger_set_ext_prop("battery", prop, data);
        if (rc < 0) {
                pr_err("Failed to set adsp battery data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
#endif

	return 0;
}

static int qcom_glink_read(enum lenovo_supply_property prop, int *val)
{
#ifdef CONFIG_LENOVO_SUPPLY
	int rc;
	int data;

        rc = qti_battery_charger_get_ext_prop("battery", prop, &data);
        if (rc < 0) {
                pr_err("Failed to get adsp battery data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
	*val = data;
#endif

	return 0;
}

static int qcom_battery_read(enum power_supply_property prop, int *val)
{
#ifdef CONFIG_LENOVO_SUPPLY
	int rc;
	int data;

        rc = qti_battery_charger_get_qcom_prop("battery", prop, &data);
        if (rc < 0) {
                pr_err("Failed to get qcom battery data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
	*val = data;
#endif

	return 0;
}

static int qcom_usb_read(enum power_supply_property prop, int *val)
{
#ifdef CONFIG_LENOVO_SUPPLY
	int rc;
	int data;

        rc = qti_battery_charger_get_qcom_prop("usb", prop, &data);
        if (rc < 0) {
                pr_err("Failed to get qcom usb data, rc=%d, prop=%d\n", rc, prop);
                return rc;
        }
	*val = data;
#endif
	return 0;
}
#if 0
#define TIMEOUT_INIT_COMPLETED	100
static int fg_check_init_completed(struct bq_fg_chip *bq)
{
	int ret;
	int i = 0;
	u16 status;

	while (i++ < TIMEOUT_INIT_COMPLETED) {
		ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &status);
		if (ret >= 0 && (status & 0x0080))
			return 0;
		msleep(100);
	}
	bq_err("wait for FG INITCOMP timeout\n");
	return ret;
}

static int fg_get_seal_state(struct bq_fg_chip *bq)
{
	int ret;
	u16 status;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &status);
	if (ret < 0) {
		bq_err("Failed to read control status, ret = %d\n", ret);
		return ret;
	}
	status &= 0x6000;
	status >>= 13;

	if (status == 1)
		bq->seal_state = SEAL_STATE_FA;
	else if (status == 2)
		bq->seal_state = SEAL_STATE_UNSEALED;
	else if (status == 3)
		bq->seal_state = SEAL_STATE_SEALED;

	return 0;
}

static int fg_unseal_send_key(struct bq_fg_chip *bq, int key)
{
	int ret;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], key & 0xFFFF);

	if (ret < 0) {
		bq_err("unable to write unseal key step 1, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], (key >> 16) & 0xFFFF);
	if (ret < 0) {
		bq_err("unable to write unseal key step 2, ret = %d\n", ret);
		return ret;
	}

	msleep(100);

	return 0;
}

#define FG_DEFAULT_UNSEAL_KEY	0x80008000
static int fg_unseal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_KEY);
	if (!ret) {
		while (retry++ < 100) {
			ret = fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_UNSEALED ||
			    bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is unsealed");
				return 0;
			}
		}
	}

	return -1;
}

#define FG_DEFAULT_UNSEAL_FA_KEY	0x36724614
static int fg_unseal_full_access(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_FA_KEY);
	if (!ret) {
		while (retry++ < 100) {
			fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is in full access.");
				return 0;
			}
			msleep(200);
		}
	}

	return -1;
}


static int fg_seal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], FG_MAC_CMD_SEAL);

	if (ret < 0) {
		bq_err("Failed to send seal command\n");
		return ret;
	}

	while (retry++ < 100) {
		fg_get_seal_state(bq);
		if (bq->seal_state == SEAL_STATE_SEALED) {
			bq_log("FG is sealed successfully");
			return 0;
		}
		msleep(200);
	}

	return -1;
}
#endif

static int fg_mac_read_block(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 cksum_calc, cksum;
	u8 t_buf[36];
	u8 t_len;
	int i;

	memset(t_buf, 0x00, sizeof(t_buf));
	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(100);

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 32);
	if (ret < 0)
		return ret;

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC] + 32, t_buf + 32, 4);
	if (ret < 0)
		return ret;

	fg_print_buf("mac_read_block", t_buf, 36);

	cksum = t_buf[34];
	t_len = t_buf[35];

	cksum_calc = checksum(t_buf, t_len - 2);
	if (cksum_calc != cksum)
		return 1;

	for (i = 0; i < len; i++)
		buf[i] = t_buf[i+2];

	return 0;
}

#if 0
static int fg_mac_write_block(struct bq_fg_chip *bq, u16 cmd, u8 *data, u8 len)
{
	int ret;
	u8 cksum;
	u8 t_buf[40];
	int i;

	if (len > 32)
		return -1;

	t_buf[0] = (u8)(cmd >> 8);
	t_buf[1] = (u8)cmd;
	for (i = 0; i < len; i++)
		t_buf[i+2] = data[i];

	cksum = checksum(data, len + 2);
	/*write command/addr, data*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, len + 2);
	if (ret < 0)
		return ret;
	t_buf[0] = cksum;
	t_buf[1] = len + 4; /*buf length, cmd, CRC and len byte itself*/
	/*write checksum and length*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MAC_CHKSUM], t_buf, 2);

	return ret;
}
#endif

static bool bq_master_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->fg_master_psy)
		return true;

	bq->fg_master_psy = lenovo_supply_get_by_name("bq27z561-master");
	if (!bq->fg_master_psy) {
		bq_err("Couldn't find fg_master_psy.\n");
		return false;
	}

	return true;
}

static bool bq_slave_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->fg_slave_psy)
		return true;

	bq->fg_slave_psy = lenovo_supply_get_by_name("bq27z561-slave");
	if (!bq->fg_slave_psy) {
		bq_err("Couldn't find fg_slave_psy.\n");
		return false;
	}

	return true;
}

static bool bq_battery_psy_initialized(struct bq_fg_chip *bq)
{
	if (!bq_master_psy_initialized(bq))
		return false;

	return bq_slave_psy_initialized(bq);
}

static bool bq2589x_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->bq2589x_psy)
		return true;

	bq->bq2589x_psy = lenovo_supply_get_by_name("bq2589h-charger");
	if (!bq->bq2589x_psy) {
		bq_err("Couldn't find bq2589x_psy.\n");
		return false;
	}

	return true;
}

#if 0
static bool qcom_batt_bak_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->qcom_batt_bak_psy)
		return true;

	bq->qcom_batt_bak_psy = lenovo_supply_get_by_name("qcom-battery");
	if (!bq->qcom_batt_bak_psy) {
		bq_err("Couldn't find qcom_batt_bak_psy.\n");
		return false;
	}

	return true;
}

static bool qcom_usb_bak_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->qcom_usb_bak_psy)
		return true;

	bq->qcom_usb_bak_psy = lenovo_supply_get_by_name("qcom-usb");
	if (!bq->qcom_usb_bak_psy) {
		bq_err("Couldn't find qcom_usb_bak_psy.\n");
		return false;
	}

	return true;
}
#endif

static int get_first_charger_online(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union lenovo_supply_propval val = {0, };

	if (!bq2589x_psy_initialized(bq))
		return 0;

	rc = lenovo_supply_get_property(bq->bq2589x_psy,
				LENOVO_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get first usb online status, rc=%d\n", rc);
		return 0;
	}

	return val.intval;
}

static int get_sec_charger_online(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union lenovo_supply_propval val = {0, };

	if (!bq2589x_psy_initialized(bq))
		return 0;

	rc = lenovo_supply_get_property(bq->bq2589x_psy,
				LENOVO_SUPPLY_PROP_HLOS_CHG_ONLINE,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get second usb online status, rc=%d\n", rc);
		return 0;
	}

	return val.intval;
}

static int calibrate_slave_battery_temp(int value)
{
	int temp;

	if (value >= 580)
		temp = value - 8;
	else if (value >= 520)
		temp = value - 5;
	else if (value >= 500)
		temp = value - 11;
	else if (value >= 490)
		temp = value - 14;
	else if (value >= 480)
		temp = value - 17;
	else if (value >= 470)
		temp = value - 19;
	else if (value >= 450)
		temp = value - 20;
	else if (value >= 400)
		temp = value - 22;
	else if (value >= 370)
		temp = value - 20;
	else if (value >= 350)
		temp = value - 17;
	else if (value >= 300)
		temp = value - 18;
	else if (value >= 270)
		temp = value - 15;
	else if (value >= 250)
		temp = value - 13;
	else if (value >= 200)
		temp = value - 16;
	else if (value >= 170)
		temp = value - 12;
	else if (value >= 160)
		temp = value - 7;
	else if (value >= 150)
		temp = value - 2;
	else if (value >= 130)
		temp = value - 4;
	else if (value >= 100)
		temp = value - 3;
	else if (value >= 50)
		temp = value - 1;
	else if (value >= 20)
		temp = value + 1;
	else
		temp = value - 7;

	return temp;
}

static int get_slave_battery_temp(struct bq_fg_chip *bq)
{
	int rc;
	int temp;
	union lenovo_supply_propval pval = {0, };
	int batt2_volt = 0;

	if (!bq2589x_psy_initialized(bq))
		return USER_BAD_BATTERY_TEMP;

	if ((!get_first_charger_online(bq)) && (!get_sec_charger_online(bq))) {
		rc = lenovo_supply_get_property(bq->fg_slave_psy,
				LENOVO_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (rc < 0) {
			bq_err("Failed to read slave battery voltage.");
		} else {
			batt2_volt = pval.intval;
		}

		if (batt2_volt < 3650000) {
			temp = fg_ap_read_temperature(bq);
		} else {
			rc = lenovo_supply_get_property(bq->bq2589x_psy,
					LENOVO_SUPPLY_PROP_TEMP, &pval);
			if (rc < 0) {
				bq_err("Failed to read bq2589x battery temp.");
				return USER_BAD_BATTERY_TEMP;
			}
			if (915 == pval.intval)
				temp = fg_ap_read_temperature(bq);
			else
				temp = calibrate_slave_battery_temp(pval.intval);
		}
	} else {
		rc = lenovo_supply_get_property(bq->bq2589x_psy,
				LENOVO_SUPPLY_PROP_TEMP, &pval);
		if (rc < 0) {
			bq_err("Failed to read bq2589x battery temp. rc=%d", rc);
			return USER_BAD_BATTERY_TEMP;
		}
		temp = calibrate_slave_battery_temp(pval.intval);
	}

	return temp;
}

static int get_master_battery_temp(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union lenovo_supply_propval val = {0, };
	int temp;

	if (!bq->qcom_batt_bak_psy) {
		bq->qcom_batt_bak_psy = lenovo_supply_get_by_name("qcom-battery");
		if (!bq->qcom_batt_bak_psy)
			return USER_BAD_BATTERY_TEMP;
	}

	rc = lenovo_supply_get_property(bq->qcom_batt_bak_psy,
				LENOVO_SUPPLY_PROP_TEMP,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get qcom backup battery LENOVO_SUPPLY_PROP_TEMP, rc=%d\n", rc);
		return USER_BAD_BATTERY_TEMP;
	}

	if (val.intval > 480)
		temp = val.intval;
	else if (val.intval >= 130)
		temp = val.intval - 10;
	else if (val.intval >= 120)
		temp = val.intval - 13;
	else if (val.intval >= 110)
		temp = val.intval - 16;
	else
		temp = val.intval - 20;

	return temp;
}

static void fg_fw_init(struct bq_fg_chip *bq)
{
	int ret;
	u8 buf[36];

	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, buf, 11);
	if (ret != 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
#if 0
		if (bq->mode == BQ27Z561_MASTER) {
			is_fake_master_battery = 1;
		} else if (bq->mode == BQ27Z561_SLAVE) {
			is_fake_slave_battery = 1;
		}
#endif
		return;
	}

	bq_log("FW Ver:%04X, Build:%04X\n",
		buf[3] << 8 | buf[2], buf[5] << 8 | buf[4]);
	bq_log("Ztrack Ver:%04X\n", buf[8] << 8 | buf[7]);

	ret = fg_mac_read_block(bq, 0x44C2, buf, 2);
	if (ret != 0) {
		bq_err("Failed to read it gauging configuration:%d\n", ret);
		return;
	}
	buf[2] = '\0';
	bq_log("it gauging configuration:0x%02X%02X\n", buf[1], buf[0]);
}

/********************************************************************
***  For slave fg
********************************************************************/
static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->data_lock);
	bq->batt_fc		= !!(flags & FG_FLAGS_FC);
	bq->batt_fd		= !!(flags & FG_FLAGS_FD);
	bq->batt_rca		= !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&bq->data_lock);

	return 0;
}

static int fg_ap_read_rsoc(struct bq_fg_chip *bq)
{
#if 0
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	return soc;
#endif
	int cc = 0;
	int soc;
	int real_soc;
	int charge_full = 0;

	if ((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) {
		soc = USER_MASTER_BATTERY_ERROR_SOC;
		real_soc = soc;
		goto out;
	} else if ((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) {
		soc = USER_SLAVE_BATTERY_ERROR_SOC;
		real_soc = soc;
		goto out;
	}

	if (bq->mode == BQ27Z561_MASTER) {
		//cc = last_cc1;
		//charge_full = last_charge_full1;
		cc = fg_adsp_read_rm(bq);
		charge_full = fg_adsp_read_fcc(bq);
	} else if (bq->mode == BQ27Z561_SLAVE) {
		//cc = last_cc2;
		//charge_full = last_charge_full2;
		cc = fg_ap_read_rm(bq);
		charge_full = fg_ap_read_fcc(bq);
	}

	if (charge_full == 0) {
		soc = SYS_ERROR_BATTERY_SOC;
		real_soc = SYS_ERROR_BATTERY_SOC;
		goto out;
	}

	soc = (cc * 100 * 100) / (charge_full * 95);
	soc = (soc > 100) ? 100 : soc;
	real_soc = cc * 100 / charge_full;
	real_soc = (real_soc > 100) ? 100 : real_soc;

out:
	if (bq->mode == BQ27Z561_MASTER) {
		last_batt1_soc = soc;
		last_batt1_real_soc = real_soc;
	} else if (bq->mode == BQ27Z561_SLAVE) {
		last_batt2_soc = soc;
		last_batt2_real_soc = real_soc;
	}

	return soc;
}

static int fg_ap_read_rsoc_decimal(struct bq_fg_chip *bq)
{
	unsigned int cc = 0;
	int soc;
	unsigned int lsoc;
	unsigned int charge_full = 1000;

	if ((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) {
		soc = USER_MASTER_BATTERY_ERROR_SOC * 100;
		goto out;
	} else if ((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) {
		soc = USER_SLAVE_BATTERY_ERROR_SOC * 100;
		goto out;
	}

	if (bq->mode == BQ27Z561_MASTER) {
		//cc = last_cc1;
		//charge_full = last_charge_full1;
		cc = fg_adsp_read_rm(bq);
		charge_full = fg_adsp_read_fcc(bq);
	} else if (bq->mode == BQ27Z561_SLAVE) {
		//cc = last_cc2;
		//charge_full = last_charge_full2;
		cc = fg_ap_read_rm(bq);
		charge_full = fg_ap_read_fcc(bq);
	}

	if (charge_full == 0) {
		soc = SYS_ERROR_BATTERY_SOC;
		goto out;
	}

	lsoc = (cc * 100 * 100 * 100) / (charge_full * 95);
	soc = (lsoc > 10000) ? 10000 : lsoc;

out:
	return soc;
}

static int fg_ap_read_soh(struct bq_fg_chip *bq)
{
	int ret;
	u16 soh = 0;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_SOH;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOH], &soh);
	if (ret < 0) {
		bq_err("could not read SOH, ret = %d\n", ret);
		return DEFAULT_SOH;
	}

	return soh;
}

static int fg_ap_read_chip_fw(struct bq_fg_chip *bq)
{
	int ret;
	u8 buf[36];
	int fw;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)))
		return DEFAULT_CHIP_FW;

	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, buf, 11);
	if (ret != 0) {
		bq_err("Failed to read chip firmware:%d\n", ret);
		return DEFAULT_CHIP_FW;
	}

	fw = buf[3] << 8 | buf[2];
	return fw;
}

static int fg_ap_get_batt_status(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int batt_status;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)))
		return LENOVO_SUPPLY_STATUS_DISCHARGING;

#if 0
	fg_read_status(bq);

	if (bq->batt_fc) {
		batt_status = LENOVO_SUPPLY_STATUS_FULL;
	} else if (bq->batt_dsg)
		batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
	else
		batt_status = LENOVO_SUPPLY_STATUS_CHARGING;
#endif

	if (bq->mode == BQ27Z561_MASTER) {
		if (!bq_master_psy_initialized(bq))
			return LENOVO_SUPPLY_STATUS_DISCHARGING;

		rc = lenovo_supply_get_property(bq->fg_master_psy,
				LENOVO_SUPPLY_PROP_CAPACITY, &pval);
		if (rc < 0) {
			bq_err("Failed to read master battery capacity. (2)");
			return LENOVO_SUPPLY_STATUS_DISCHARGING;
		}
	} else if (bq->mode == BQ27Z561_SLAVE) {
		if (!bq_slave_psy_initialized(bq))
			return LENOVO_SUPPLY_STATUS_DISCHARGING;

		rc = lenovo_supply_get_property(bq->fg_slave_psy,
				LENOVO_SUPPLY_PROP_CAPACITY, &pval);
		if (rc < 0) {
			bq_err("Failed to read slave battery capacity. (2)");
			return LENOVO_SUPPLY_STATUS_DISCHARGING;
		}
	}

	if ((!get_first_charger_online(bq)) && (!get_sec_charger_online(bq))) {
		batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
	} else if (100 == pval.intval) {
		batt_status = LENOVO_SUPPLY_STATUS_FULL;
	} else {
		if (!bq2589x_psy_initialized(bq))
			return LENOVO_SUPPLY_STATUS_DISCHARGING;

		rc = lenovo_supply_get_property(bq->bq2589x_psy,
				LENOVO_SUPPLY_PROP_CHARGING_ENABLED, &pval);
		if (rc < 0) {
			dev_err(bq->dev, "Couldn't get second charge enable LENOVO_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n", rc);
			return LENOVO_SUPPLY_STATUS_DISCHARGING;
		}

		if (0 == pval.intval)
			batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
		else
			batt_status = LENOVO_SUPPLY_STATUS_CHARGING;
	}

	if (get_first_charger_online(bq) || get_sec_charger_online(bq)) {
		if (bq_wake_lock != 1) {
			pm_stay_awake(bq->dev);
			bq_wake_lock = 1;
			pr_info("batt_sys: bq_fg pm_stay_awake\n");
		}
	} else {
		if (bq_wake_lock != 0) {
			pm_relax(bq->dev);
			bq_wake_lock = 0;
			pr_info("batt_sys: bq_fg pm_relax\n");
		}
	}
	return batt_status;
}


static int fg_ap_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	u16 temp = 0;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_DIE_TEMP;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_err("could not read temperature, ret = %d\n", ret);
		return DEFAULT_DIE_TEMP;
	}

	return temp - 2730;
}

static int fg_ap_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	u16 volt = 0;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_VOLTAGE;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("could not read voltage, ret = %d\n", ret);
		return DEFAULT_VOLTAGE;
	}

	return volt * 1000;
}

static int fg_ap_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret = -EINVAL;
	u16 avg_curr = 0;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done))) {
		*curr = DEFAULT_CURRENT;
		return ret;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_MI], &avg_curr);
	if (ret < 0) {
		bq_err("could not read current, ret = %d\n", ret);
		return DEFAULT_CURRENT;
	}
	*curr = (int)((s16)avg_curr) * -1000;

	return ret;
}

static int fg_ap_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	u16 fcc;
	int value;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done))) {
		value = DEFAULT_FCC;
		goto out;
	}

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_err("FCC command not supported!\n");
		value = DEFAULT_FCC;
		goto out;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);
	if (ret < 0) {
		bq_err("could not read FCC, ret=%d\n", ret);
		value = DEFAULT_FCC;
	} else
		value = fcc;

out:
	if (bq->mode == BQ27Z561_MASTER)
		last_charge_full1 = value;
	else if (bq->mode == BQ27Z561_SLAVE)
		last_charge_full2 = value;

	return value;
}

static int fg_ap_read_dc(struct bq_fg_chip *bq)
{
	int ret;
	u16 dc;
	int value;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done))) {
		value = DEFAULT_DC;
		goto out;
	}

	if (bq->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		bq_err("DesignCapacity command not supported!\n");
		value = DEFAULT_DC;
		goto out;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_DC], &dc);
	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		value = DEFAULT_DC;
		goto out;
	} else
		value = dc;

out:
	return value;
}


static int fg_ap_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	u16 rm;
	int value;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done))) {
		value = DEFAULT_RM;
		goto out;
	}

	if (bq->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		bq_err("RemainingCapacity command not supported!\n");
		value = DEFAULT_RM;
		goto out;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_RM], &rm);
	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		value = DEFAULT_RM;
	} else
		value = rm;

out:
	if (bq->mode == BQ27Z561_MASTER)
		last_cc1 = value;
	else if (bq->mode == BQ27Z561_SLAVE)
		last_cc2 = value;

	return value;
}

static int fg_ap_read_cyclecount(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_CC;

	if (bq->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		bq_err("Cycle Count not supported!\n");
		return DEFAULT_CC;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		bq_err("could not read Cycle Count, ret=%d\n", ret);
		return DEFAULT_CC;
	}

	return cc;
}

static int fg_ap_read_tte(struct bq_fg_chip *bq)
{
	int ret;
	u16 tte;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_TTE;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_err("Time To Empty not supported!\n");
		return DEFAULT_TTE;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		bq_err("could not read Time To Empty, ret=%d\n", ret);
		return DEFAULT_TTE;
	}

	if (ret == 0xFFFF)
		return DEFAULT_TTE;

	return tte;
}

static int fg_ap_read_ttf(struct bq_fg_chip *bq)
{
	int ret;
	u16 ttf;

	if (((bq->mode == BQ27Z561_MASTER) && (is_fake_master_battery)) || 
			((bq->mode == BQ27Z561_SLAVE) && (is_fake_slave_battery)) ||
			((bq->mode == BQ27Z561_SLAVE) && (!bq->fw2_check_done)))
		return DEFAULT_TTF;

	if (bq->regs[BQ_FG_REG_TTF] == INVALID_REG_ADDR) {
		bq_err("Time To Full not supported!\n");
		return DEFAULT_TTF;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTF], &ttf);

	if (ret < 0) {
		bq_err("could not read Time To Full, ret=%d\n", ret);
		return DEFAULT_TTF;
	}

	if (ret == 0xFFFF)
		return DEFAULT_TTF;

	return ttf;
}

static enum lenovo_supply_property fg_ap_props[] = {
	LENOVO_SUPPLY_PROP_STATUS,
	LENOVO_SUPPLY_PROP_PRESENT,
	LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_CAPACITY,
	LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL,
	LENOVO_SUPPLY_PROP_TEMP,
	LENOVO_SUPPLY_PROP_HEALTH,
	LENOVO_SUPPLY_PROP_CHARGE_FULL,
	LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_CYCLE_COUNT,
	LENOVO_SUPPLY_PROP_TECHNOLOGY,
	LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW,
	LENOVO_SUPPLY_PROP_CHARGE_COUNTER,
	LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SOH,
	LENOVO_SUPPLY_PROP_CHIP_FW,
};

static int fg_ap_get_property(struct lenovo_supply *psy, enum lenovo_supply_property psp,
					union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_STATUS:
		val->intval = fg_ap_get_batt_status(bq);
		//mutex_lock(&bq->data_lock);
		//mutex_unlock(&bq->data_lock);
		break;
	case LENOVO_SUPPLY_PROP_VOLTAGE_NOW:
		bq->batt_volt = fg_ap_read_volt(bq);
		val->intval = bq->batt_volt;
		break;
	case LENOVO_SUPPLY_PROP_PRESENT:
		val->intval = !is_fake_slave_battery;
		break;
	case LENOVO_SUPPLY_PROP_HEALTH:
		val->intval = LENOVO_SUPPLY_HEALTH_GOOD;
		break;
	case LENOVO_SUPPLY_PROP_CURRENT_NOW:
		fg_ap_read_current(bq, &bq->batt_curr2);
		val->intval = bq->batt_curr2;
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY:
		bq->batt_soc = fg_ap_read_rsoc(bq);
		val->intval = bq->batt_soc;
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL:
		val->intval = fg_ap_read_rsoc_decimal(bq);
		break;

	case LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		bq->batt_temp = fg_ap_read_temperature(bq);
		val->intval = bq->batt_temp;
		break;

	case LENOVO_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}

		if (bq->mode == BQ27Z561_MASTER) {
			ret = get_master_battery_temp(bq);
		} else if (bq->mode == BQ27Z561_SLAVE) {
			ret = get_slave_battery_temp(bq);
		} else
			ret = USER_BAD_BATTERY_TEMP;

		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_temp = ret;
		val->intval = bq->batt_temp;
		mutex_unlock(&bq->data_lock);

		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				val->intval = debug_temp1;
			else if (bq->mode == BQ27Z561_SLAVE)
				val->intval = debug_temp2;
		}
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		bq->batt_tte = fg_ap_read_tte(bq);
		val->intval = bq->batt_tte;
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW:
		bq->batt_ttf = fg_ap_read_ttf(bq);
		val->intval = bq->batt_ttf;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL:
		bq->batt_fcc = fg_ap_read_fcc(bq);
		val->intval = bq->batt_fcc * 1000;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		bq->batt_dc = fg_ap_read_dc(bq);
		val->intval = bq->batt_dc * 1000;
		break;

	case LENOVO_SUPPLY_PROP_CYCLE_COUNT:
		bq->batt_cyclecnt = fg_ap_read_cyclecount(bq);
		val->intval = bq->batt_cyclecnt;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_COUNTER:
		bq->batt_rm = fg_ap_read_rm(bq);
		val->intval = bq->batt_rm * 1000;
		break;

	case LENOVO_SUPPLY_PROP_TECHNOLOGY:
		val->intval = LENOVO_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case LENOVO_SUPPLY_PROP_SOH:
		bq->batt_soh = fg_ap_read_soh(bq);
		val->intval = bq->batt_soh;
		break;

	case LENOVO_SUPPLY_PROP_CHIP_FW:
		val->intval = fg_ap_read_chip_fw(bq);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fg_ap_set_property(struct lenovo_supply *psy,
			       enum lenovo_supply_property prop,
			       const union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);

	switch (prop) {
#if 0
	case LENOVO_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case LENOVO_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		lenovo_supply_changed(bq->fg_ap_psy);
		break;
#endif
	case LENOVO_SUPPLY_PROP_TEMP:
		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				debug_temp1 = val->intval;
			else if (bq->mode == BQ27Z561_SLAVE)
				debug_temp2 = val->intval;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int fg_ap_prop_is_writeable(struct lenovo_supply *psy,
				       enum lenovo_supply_property prop)
{
	int ret;

	switch (prop) {
#if 0
	case LENOVO_SUPPLY_PROP_CAPACITY:
#endif
	case LENOVO_SUPPLY_PROP_TEMP:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

/********************************************************************
***  For master fg
********************************************************************/
static int fg_adsp_read_rsoc(struct bq_fg_chip *bq)
{
	int cc = 0;
	int soc;
	int real_soc;
	int charge_full = 0;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
		soc = USER_MASTER_BATTERY_ERROR_SOC;
		real_soc = soc;
		goto out;
	}

	//cc = last_cc1;
	//charge_full = last_charge_full1;
	cc = fg_adsp_read_rm(bq);
	charge_full = fg_adsp_read_fcc(bq);

	if (charge_full == 0) {
		soc = SYS_ERROR_BATTERY_SOC;
		real_soc = SYS_ERROR_BATTERY_SOC;
		goto out;
	}

	soc = (cc * 100 * 100) / (charge_full * 95);
	soc = (soc > 100) ? 100 : soc;
	real_soc = cc * 100 / charge_full;
	real_soc = (real_soc > 100) ? 100 : real_soc;

out:
	last_batt1_soc = soc;
	last_batt1_real_soc = real_soc;

	return soc;
}

static int fg_adsp_read_rsoc_decimal(struct bq_fg_chip *bq)
{
	unsigned int cc = 0;
	int soc;
	unsigned int lsoc;
	unsigned int charge_full = 0;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
		soc = USER_MASTER_BATTERY_ERROR_SOC * 100;
		goto out;
	}

	//cc = last_cc1;
	//charge_full = last_charge_full1;
	cc = fg_adsp_read_rm(bq);
	charge_full = fg_adsp_read_fcc(bq);

	if (charge_full == 0) {
		soc = SYS_ERROR_BATTERY_SOC;
		goto out;
	}

	lsoc = (cc * 100 * 100 * 100) / (charge_full * 95);
	soc = (lsoc > 10000) ? 10000 : lsoc;

out:
	return soc;
}

static int fg_adsp_get_batt_status(struct bq_fg_chip *bq)
{

	union lenovo_supply_propval pval = {0, };
	int rc;
	int batt_status;

	if (is_fake_master_battery || !bq->fw1_check_done)
		return LENOVO_SUPPLY_STATUS_DISCHARGING;

	if (!bq_master_psy_initialized(bq))
		return LENOVO_SUPPLY_STATUS_DISCHARGING;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery capacity. (2)");
		return LENOVO_SUPPLY_STATUS_DISCHARGING;
	}

	if ((!get_first_charger_online(bq)) && (!get_sec_charger_online(bq))) {
		batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
	} else if (100 == pval.intval) {
		batt_status = LENOVO_SUPPLY_STATUS_FULL;
	} else {
		if (!bq2589x_psy_initialized(bq))
			return LENOVO_SUPPLY_STATUS_DISCHARGING;

		rc = lenovo_supply_get_property(bq->bq2589x_psy,
				LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE, &pval);
		if (rc < 0) {
			dev_err(bq->dev, "Couldn't get first charge enable LENOVO_SUPPLY_PROP_ADSP_CHG_ENABLE, rc=%d\n", rc);
			return LENOVO_SUPPLY_STATUS_DISCHARGING;
		}

		if (0 == pval.intval)
			batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
		else
			batt_status = LENOVO_SUPPLY_STATUS_CHARGING;
	}

	if (get_first_charger_online(bq) || get_sec_charger_online(bq)) {
		if (bq_wake_lock != 1) {
			pm_stay_awake(bq->dev);
			bq_wake_lock = 1;
			pr_info("batt_sys: bq_fg pm_stay_awake\n");
		}
	} else {
		if (bq_wake_lock != 0) {
			pm_relax(bq->dev);
			bq_wake_lock = 0;
			pr_info("batt_sys: bq_fg pm_relax\n");
		}
	}
	return batt_status;
}

static int fg_adsp_read_soh(struct bq_fg_chip *bq)
{
	int ret;
	int soh = 0;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable)
                return DEFAULT_SOH;

	if (soh_jiffies != -1) {
		if (time_before(jiffies, soh_jiffies + msecs_to_jiffies(FG_READ_DELAY_SOH)))
			return last_soh;
		else
			soh_jiffies = jiffies;
	} else
		soh_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_SOH, &soh);
        if (ret < 0) {
                pr_err("Failed to get adsp battery soh, ret=%d\n", ret);
                return DEFAULT_SOH;
	}

	last_soh = soh;
	return last_soh;
}

static int fg_adsp_read_chip_fw(struct bq_fg_chip *bq)
{
	int ret;
	int fw = 0;

	if ((bq->master_fw != 0x00) && (bq->master_fw != 0xFF))
		return bq->master_fw;

	if (is_fake_master_battery)
                return DEFAULT_CHIP_FW;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_CHIP_FW, &fw);
        if (ret < 0) {
                pr_err("Failed to get adsp battery chip fw, ret=%d\n", ret);
                return DEFAULT_CHIP_FW;
	}

	return fw;
}

static int fg_adsp_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	int temp = 0;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable)
                return DEFAULT_DIE_TEMP;

	if (die_temp_jiffies != -1) {
		if (time_before(jiffies, die_temp_jiffies + msecs_to_jiffies(FG_READ_DELAY_DIE_TEMP)))
			return last_die_temp;
		else
			die_temp_jiffies = jiffies;
	} else
		die_temp_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_TI_DIE_TEMPERATURE, &temp);
        if (ret < 0) {
                pr_err("Failed to get adsp battery die temp, ret=%d\n", ret);
                return DEFAULT_DIE_TEMP;
	}

	last_die_temp =  temp - 2730;
	return last_die_temp;
}

static int fg_adsp_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	int volt;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable)
                return DEFAULT_VOLTAGE;

	if (volt_jiffies != -1) {
		if (time_before(jiffies, volt_jiffies + msecs_to_jiffies(FG_READ_DELAY_VOLTAGE)))
			return last_volt;
		else
			volt_jiffies = jiffies;
	} else
		volt_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_VOLTAGE_NOW, &volt);
        if (ret < 0) {
                pr_err("Failed to get adsp battery voltage, ret=%d\n", ret);
                volt = DEFAULT_VOLTAGE;
	}

	last_volt = volt;
	return last_volt;
}

static int fg_adsp_read_current(struct bq_fg_chip *bq, int *curr)
{
	int rc = -EINVAL;
	int avg_curr = 0;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
		*curr = DEFAULT_CURRENT;
                return rc;
	}

	if (curr_jiffies != -1) {
		if (time_before(jiffies, curr_jiffies + msecs_to_jiffies(FG_READ_DELAY_CURRENT)))
			return last_curr;
		else
			curr_jiffies = jiffies;
	} else
		curr_jiffies = jiffies;

        rc = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_CURRENT_NOW, &avg_curr);
        if (rc < 0) {
                pr_err("Failed to get adsp battery current, rc=%d\n", rc);
                *curr = DEFAULT_CURRENT;
	} else 
		*curr = avg_curr * -1000;

	last_curr = *curr;
	return rc;
}

static int fg_adsp_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	int fcc;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
                fcc = DEFAULT_FCC;
		goto out;
	}

	if (fcc_jiffies != -1) {
		if (time_before(jiffies, fcc_jiffies + msecs_to_jiffies(FG_READ_DELAY_FCC)))
			return last_charge_full1;
		else
			fcc_jiffies = jiffies;
	} else
		fcc_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_CHARGE_FULL, &fcc);
        if (ret < 0) {
                pr_err("Failed to get adsp battery fcc, ret=%d\n", ret);
                fcc = DEFAULT_FCC;
	}

out:
	last_charge_full1 = fcc;
	return fcc;
}

static int fg_adsp_read_dc(struct bq_fg_chip *bq)
{
	int ret;
	int dc;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
                dc = DEFAULT_DC;
		goto out;
	}

	if (dc_jiffies != -1) {
		if (time_before(jiffies, dc_jiffies + msecs_to_jiffies(FG_READ_DELAY_DC)))
			return last_charge_full1;
		else
			dc_jiffies = jiffies;
	} else
		dc_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_FULL_DESIGN, &dc);
        if (ret < 0) {
                pr_err("Failed to get adsp battery dc, ret=%d\n", ret);
                dc = DEFAULT_DC;
		goto out;
	}

out:
	last_dc = dc;
	return last_dc;
}

static int fg_adsp_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	int rm;

	if (is_fake_master_battery || !bq->fw1_check_done || !bq->fg_read_enable) {
                rm = DEFAULT_RM;
		goto out;
	}

	if (rm_jiffies != -1) {
		if (time_before(jiffies, rm_jiffies + msecs_to_jiffies(FG_READ_DELAY_RM)))
			return last_cc1;
		else
			rm_jiffies = jiffies;
	} else
		rm_jiffies = jiffies;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_CHARGE_COUNTER, &rm);
        if (ret < 0) {
                pr_err("Failed to get adsp battery rm, ret=%d\n", ret);
                rm = DEFAULT_RM;
		goto out;
	}

out:
	last_cc1 = rm;
	return rm;
}

static int fg_adsp_read_cyclecount(struct bq_fg_chip *bq)
{
#if 1
	//TODO donot access adsp for glink performance issue
	return DEFAULT_CC;
#else
	int ret;
	int cc;

	if (is_fake_master_battery || !bq->fw1_check_done)
                return DEFAULT_CC;
		
        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_CYCLE_COUNT, &cc);
        if (ret < 0) {
                pr_err("Failed to get adsp battery cc, ret=%d\n", ret);
                return DEFAULT_CC;
	}

	return cc;
#endif
}

static int fg_adsp_read_tte(struct bq_fg_chip *bq)
{
#if 1
	//TODO donot access adsp for glink performance issue
	return DEFAULT_TTE;
#else
	int ret;
	int tte;

	if (is_fake_master_battery || !bq->fw1_check_done)
                return DEFAULT_TTE;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_TIME_TO_EMPTY_NOW, &tte);
        if (ret < 0) {
                pr_err("Failed to get adsp battery tte, ret=%d\n", ret);
                return DEFAULT_TTE;
	}

	if (ret == 0xFFFF)
                return DEFAULT_TTE;

	return tte;
#endif
}

static int fg_adsp_read_ttf(struct bq_fg_chip *bq)
{
#if 1
	//TODO donot access adsp for glink performance issue
	return DEFAULT_TTF;
#else
	int ret;
	int ttf;

	if (is_fake_master_battery || !bq->fw1_check_done)
                return DEFAULT_TTF;

        ret = qcom_glink_read(LENOVO_SUPPLY_PROP_FG1_TIME_TO_FULL_NOW, &ttf);
        if (ret < 0) {
                pr_err("Failed to get adsp battery ttf, ret=%d\n", ret);
                return DEFAULT_TTF;
	}

	if (ret == 0xFFFF)
                return DEFAULT_TTF;

	return ttf;
#endif
}


static enum lenovo_supply_property fg_adsp_props[] = {
	LENOVO_SUPPLY_PROP_STATUS,
	LENOVO_SUPPLY_PROP_PRESENT,
	LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_CAPACITY,
	LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL,
	LENOVO_SUPPLY_PROP_TEMP,
	LENOVO_SUPPLY_PROP_HEALTH,
	LENOVO_SUPPLY_PROP_CHARGE_FULL,
	LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_CYCLE_COUNT,
	LENOVO_SUPPLY_PROP_TECHNOLOGY,
	LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW,
	LENOVO_SUPPLY_PROP_CHARGE_COUNTER,
	LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SOH,
	LENOVO_SUPPLY_PROP_CHIP_FW,
};

static int fg_adsp_get_property(struct lenovo_supply *psy, enum lenovo_supply_property psp,
					union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_STATUS:
		val->intval = fg_adsp_get_batt_status(bq);
		break;
	case LENOVO_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fg_adsp_read_volt(bq);
		break;
	case LENOVO_SUPPLY_PROP_PRESENT:
		val->intval = !is_fake_master_battery;
		break;
	case LENOVO_SUPPLY_PROP_HEALTH:
		val->intval = LENOVO_SUPPLY_HEALTH_GOOD;
		break;
	case LENOVO_SUPPLY_PROP_CURRENT_NOW:
		fg_adsp_read_current(bq, &bq->batt_curr1);
		val->intval = bq->batt_curr1;
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY:
		bq->batt_soc = fg_adsp_read_rsoc(bq);
		val->intval = bq->batt_soc;
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL:
		val->intval = fg_adsp_read_rsoc_decimal(bq);
		break;

	case LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		bq->batt_temp = fg_adsp_read_temperature(bq);
		val->intval = bq->batt_temp;
		break;

	case LENOVO_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}

		ret = get_master_battery_temp(bq);

		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_temp = ret;
		val->intval = bq->batt_temp;
		mutex_unlock(&bq->data_lock);

		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				val->intval = debug_temp1;
			else if (bq->mode == BQ27Z561_SLAVE)
				val->intval = debug_temp2;
		}
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		bq->batt_tte = fg_adsp_read_tte(bq);
		val->intval = bq->batt_tte;
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW:
		bq->batt_ttf = fg_adsp_read_ttf(bq);
		val->intval = bq->batt_ttf;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL:
		bq->batt_fcc = fg_adsp_read_fcc(bq);
		val->intval = bq->batt_fcc * 1000;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		bq->batt_dc = fg_adsp_read_dc(bq);
		val->intval = bq->batt_dc * 1000;
		break;

	case LENOVO_SUPPLY_PROP_CYCLE_COUNT:
		bq->batt_cyclecnt = fg_adsp_read_cyclecount(bq);
		val->intval = bq->batt_cyclecnt;
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_COUNTER:
		bq->batt_rm = fg_adsp_read_rm(bq);
		val->intval = bq->batt_rm * 1000;
		break;

	case LENOVO_SUPPLY_PROP_TECHNOLOGY:
		val->intval = LENOVO_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case LENOVO_SUPPLY_PROP_SOH:
		bq->batt_soh = fg_adsp_read_soh(bq);
		val->intval = bq->batt_soh;
		break;

	case LENOVO_SUPPLY_PROP_CHIP_FW:
		val->intval = fg_adsp_read_chip_fw(bq);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fg_adsp_set_property(struct lenovo_supply *psy,
			       enum lenovo_supply_property prop,
			       const union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);

	switch (prop) {
#if 0
	case LENOVO_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case LENOVO_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		lenovo_supply_changed(bq->fg_ap_psy);
		break;
#endif
	case LENOVO_SUPPLY_PROP_TEMP:
		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				debug_temp1 = val->intval;
			else if (bq->mode == BQ27Z561_SLAVE)
				debug_temp2 = val->intval;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int fg_adsp_prop_is_writeable(struct lenovo_supply *psy,
				       enum lenovo_supply_property prop)
{
	int ret;

	switch (prop) {
#if 0
	case LENOVO_SUPPLY_PROP_CAPACITY:
#endif
	case LENOVO_SUPPLY_PROP_TEMP:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}


/********************************************************************
***  For UI
********************************************************************/
static int bq_user_get_batt_capacity(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int soc, soc2, soc_tmp;

	if (is_fake_master_battery && is_fake_slave_battery) {
		bq_err("fake both battery soc is %d\n", USER_BOTH_BATTERY_ERROR_SOC);
		soc_tmp = USER_BOTH_BATTERY_ERROR_SOC;
		goto out;
	} else if (is_fake_master_battery) {
		bq_err("fake master battery soc is %d\n", USER_MASTER_BATTERY_ERROR_SOC);
		//soc_tmp = USER_MASTER_BATTERY_ERROR_SOC;
		//goto out;
	} else if (is_fake_slave_battery) {
		bq_err("fake slave battery soc is %d\n", USER_SLAVE_BATTERY_ERROR_SOC);
		soc_tmp = USER_SLAVE_BATTERY_ERROR_SOC;
		goto out;
	}

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_SOC;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery capacity.");
		soc_tmp = USER_MASTER_BATTERY_ERROR_SOC;
		goto out;
	}
	soc = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery capacity.");
		soc_tmp = USER_SLAVE_BATTERY_ERROR_SOC;
		goto out;
	}
	soc2 = pval.intval;

	if ((soc < 0) && (soc2 < 0))
		return bq->last_batt_soc;
	else if (soc < 0)
		return bq->last_batt_soc;
	else if (soc2 < 0)
		return bq->last_batt_soc;
	else {
		if (!bq_battery_psy_initialized(bq))
			return bq->last_batt_soc;

		rc = lenovo_supply_get_property(bq->fg_master_psy,
				LENOVO_SUPPLY_PROP_CURRENT_NOW, &pval);
		if (rc < 0) {
			bq_err("Failed to read bq battery current_now. (3)");
			return bq->last_batt_soc;
		}

		soc_tmp = (soc + soc2) / 2 + (soc + soc2) % 2;
#if 0
		if ((pval.intval < 0) && (soc_tmp < bq->last_batt_soc)) {
			bq_err("batt_sys: WA for soc %d, %d, %d", bq->last_batt_soc, soc_tmp, pval.intval);
			return bq->last_batt_soc;
		}
#endif
	}
out:
	bq->last_batt_soc = soc_tmp;
	return bq->last_batt_soc;
}

static int bq_user_get_batt_capacity_decimal(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int soc, soc2, soc_tmp;

	if (is_fake_master_battery && is_fake_slave_battery) {
		bq_err("fake both battery soc is %d\n", USER_BOTH_BATTERY_ERROR_SOC);
		soc_tmp = USER_BOTH_BATTERY_ERROR_SOC * 100;
		goto out;
	} else if (is_fake_master_battery) {
		bq_err("fake master battery soc is %d\n", USER_MASTER_BATTERY_ERROR_SOC);
		soc_tmp = USER_MASTER_BATTERY_ERROR_SOC * 100;
		goto out;
	} else if (is_fake_slave_battery) {
		bq_err("fake slave battery soc is %d\n", USER_SLAVE_BATTERY_ERROR_SOC);
		soc_tmp = USER_SLAVE_BATTERY_ERROR_SOC * 100;
		goto out;
	}

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery capacity.");
		soc_tmp = SYS_ERROR_BATTERY_SOC * 100;
		goto out;
	}
	soc = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery capacity.");
		soc_tmp = SYS_ERROR_BATTERY_SOC * 100;
		goto out;
	}
	soc2 = pval.intval;

	if ((soc < 0) && (soc2 < 0))
		return last_user_soc_decimal;
	else if (soc < 0)
		return last_user_soc_decimal;
	else if (soc2 < 0)
		return last_user_soc_decimal;
	else {
		if (!bq_battery_psy_initialized(bq))
			return last_user_soc_decimal;

		rc = lenovo_supply_get_property(bq->fg_master_psy,
				LENOVO_SUPPLY_PROP_CURRENT_NOW, &pval);
		if (rc < 0) {
			bq_err("Failed to read bq battery current_now. (3)");
			return last_user_soc_decimal;
		}

		soc_tmp= (soc + soc2) / 2 + (soc + soc2) % 2;
		if ((pval.intval < 0) && (soc_tmp < last_user_soc_decimal)) {
			bq_err("batt_sys: WA for soc_decimal %d, %d, %d", last_user_soc_decimal, soc_tmp, pval.intval);
			return last_user_soc_decimal;
		}
	}

out:
	last_user_soc_decimal = soc_tmp;
	return last_user_soc_decimal;
}

static int bq_user_get_batt_status(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int status;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return LENOVO_SUPPLY_STATUS_DISCHARGING;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery status.");
		return bq->last_batt_status;
	}
	status = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery status.");
		return bq->last_batt_status;
	}


	if ((status == LENOVO_SUPPLY_STATUS_FULL) &&
			(pval.intval == LENOVO_SUPPLY_STATUS_FULL))
		return LENOVO_SUPPLY_STATUS_FULL;
	else if ((status == LENOVO_SUPPLY_STATUS_CHARGING) ||
			(pval.intval == LENOVO_SUPPLY_STATUS_CHARGING))
		return LENOVO_SUPPLY_STATUS_CHARGING;
	else if ((status == LENOVO_SUPPLY_STATUS_NOT_CHARGING) &&
			(pval.intval == LENOVO_SUPPLY_STATUS_NOT_CHARGING))
		return LENOVO_SUPPLY_STATUS_NOT_CHARGING;
	else
		return LENOVO_SUPPLY_STATUS_DISCHARGING;
}

static int bq_user_get_batt_soh(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int soh;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_SOH, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery soh.");
		//return bq->last_batt_soh;
		return 0;
	}
	soh = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_SOH, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery soh.");
		//return bq->last_batt_soh;
		return 0;
	}
	bq->last_batt_soh = (soh + pval.intval) / 2;
	return bq->last_batt_soh;
}

static int bq_user_get_batt_volt(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int volt = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_VOLTAGE;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery voltage.");
		return bq->last_batt_volt;
	}
	volt = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	get_batt2_vol = pval.intval;

	if (rc < 0) {
		bq_err("Failed to read slave battery voltage.");
		return bq->last_batt_volt;
	}

	bq->last_batt_volt = (volt + pval.intval) / 2;
	return bq->last_batt_volt;
}

static int bq_user_get_batt_persent(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int present = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_PRESENT, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery present.");
		return bq->last_batt_present;
	}
	present = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_PRESENT, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery present.");
		return bq->last_batt_present;
	}

	return (present & pval.intval);
}

static int bq_user_get_batt_current(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int curr = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_CURRENT;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery current.");
		return bq->last_batt_current;
	}
	curr = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery voltage.");
		return bq->last_batt_current;
	}

	bq->last_batt_current = curr + pval.intval;
	return bq->last_batt_current;
}

static int bq_user_get_batt_temp(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int temp = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_TEMP;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery temp.");
		return bq->last_batt_temp;
	}
	temp = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery temp.");
		return bq->last_batt_temp;
	}

	bq->last_batt_temp = (temp > pval.intval) ? temp : pval.intval;
	if (is_fake_master_battery || is_fake_slave_battery)
		bq->last_batt_temp = USER_BAD_BATTERY_TEMP;
	pr_debug("user temp: batt=%d batt-1=%d batt-2=%d fake-1=%d fake-2=%d\n",
			bq->last_batt_temp, temp, pval.intval, is_fake_master_battery, is_fake_slave_battery);

	return bq->last_batt_temp;
}

static int bq_user_get_die_temp(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int temp = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_TEMP;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery temp.");
		return USER_BAD_BATTERY_TEMP;
		//return bq->last_batt_temp;
	}
	temp = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery temp.");
		return USER_BAD_BATTERY_TEMP;
		//return bq->last_batt_temp;
	}

	bq->last_batt_temp = (temp > pval.intval) ? temp : pval.intval;
	return bq->last_batt_temp;
}

static int bq_user_get_batt_charge_full(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int charge_full = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery charge full.");
		return bq->last_batt_charge_full;
	}
	charge_full = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery charge full.");
		return bq->last_batt_charge_full;
	}

	bq->last_batt_charge_full = charge_full + pval.intval;
	return bq->last_batt_charge_full;
}

static int bq_user_get_batt_desgin_full(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int design_full = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery charge full desgin.");
		return bq->last_batt_design_full;
	}
	design_full = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery charge full desgin.");
		return bq->last_batt_design_full;
	}

	bq->last_batt_design_full = design_full + pval.intval;
	return bq->last_batt_design_full;
}

static int bq_user_get_batt_charge_counter(struct bq_fg_chip *bq)
{
	union lenovo_supply_propval pval = {0, };
	int rc;
	int cc = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = lenovo_supply_get_property(bq->fg_master_psy,
			LENOVO_SUPPLY_PROP_CHARGE_COUNTER, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery cc.");
		return bq->last_batt_cc;
	}
	cc = pval.intval;

	rc = lenovo_supply_get_property(bq->fg_slave_psy,
			LENOVO_SUPPLY_PROP_CHARGE_COUNTER, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery cc.");
		return bq->last_batt_cc;
	}

	bq->last_batt_cc = cc + pval.intval;
	return bq->last_batt_cc;
}

static enum lenovo_supply_property fg_user_props[] = {
	LENOVO_SUPPLY_PROP_STATUS,
	LENOVO_SUPPLY_PROP_PRESENT,
	LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_CAPACITY,
	LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL,
	LENOVO_SUPPLY_PROP_TEMP,
	LENOVO_SUPPLY_PROP_HEALTH,
	LENOVO_SUPPLY_PROP_CHARGE_FULL,
	LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	LENOVO_SUPPLY_PROP_CYCLE_COUNT,
	LENOVO_SUPPLY_PROP_TECHNOLOGY,
	LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW,
	LENOVO_SUPPLY_PROP_CHARGE_COUNTER,
	LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	LENOVO_SUPPLY_PROP_SOH,
	LENOVO_SUPPLY_PROP_CHIP_FW,
};

static int fg_user_get_property(struct lenovo_supply *psy, enum lenovo_supply_property psp,
					union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_STATUS:
		val->intval = bq_user_get_batt_status(bq);
		break;

	case LENOVO_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq_user_get_batt_volt(bq);
		break;

	case LENOVO_SUPPLY_PROP_PRESENT:
		val->intval = bq_user_get_batt_persent(bq);
		break;

	case LENOVO_SUPPLY_PROP_HEALTH:
		val->intval = LENOVO_SUPPLY_HEALTH_GOOD;
		break;

	case LENOVO_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq_user_get_batt_current(bq);
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY:
		val->intval = bq_user_get_batt_capacity(bq);
		last_user_soc = val->intval;
		break;

	case LENOVO_SUPPLY_PROP_CAPACITY_DECIMAL:
		val->intval = bq_user_get_batt_capacity_decimal(bq);
		break;

	case LENOVO_SUPPLY_PROP_SOH:
		val->intval = bq_user_get_batt_soh(bq);
		break;

	case LENOVO_SUPPLY_PROP_CHIP_FW:
		val->intval = 0;
		break;

	case LENOVO_SUPPLY_PROP_TEMP:
		val->intval = bq_user_get_batt_temp(bq);
		break;

	case LENOVO_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		val->intval = bq_user_get_die_temp(bq);
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_ap_read_tte(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_tte = ret;

		val->intval = bq->batt_tte;
		mutex_unlock(&bq->data_lock);
		break;

	case LENOVO_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = fg_ap_read_ttf(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_ttf = ret;

		val->intval = bq->batt_ttf;
		mutex_unlock(&bq->data_lock);
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq_user_get_batt_charge_full(bq);
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq_user_get_batt_desgin_full(bq);
		break;

	case LENOVO_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_ap_read_cyclecount(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_cyclecnt = ret;
		val->intval = bq->batt_cyclecnt;
		mutex_unlock(&bq->data_lock);
		break;

	case LENOVO_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq_user_get_batt_charge_counter(bq);
		break;

	case LENOVO_SUPPLY_PROP_TECHNOLOGY:
		val->intval = LENOVO_SUPPLY_TECHNOLOGY_LIPO;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int fg_user_set_property(struct lenovo_supply *psy,
			       enum lenovo_supply_property prop,
			       const union lenovo_supply_propval *val)
{
	return 0;
}


static int fg_user_prop_is_writeable(struct lenovo_supply *psy,
				       enum lenovo_supply_property prop)
{
	int ret = 0;
	return ret;
}

static enum lenovo_supply_property qcom_batt_bak_props[] = {
	LENOVO_SUPPLY_PROP_STATUS,
	LENOVO_SUPPLY_PROP_HEALTH,
	LENOVO_SUPPLY_PROP_CAPACITY,
	LENOVO_SUPPLY_PROP_VOLTAGE_NOW,
	LENOVO_SUPPLY_PROP_CURRENT_NOW,
	LENOVO_SUPPLY_PROP_TEMP,
};

static int qcom_batt_bak_get_property(struct lenovo_supply *psy, enum lenovo_supply_property psp,
					union lenovo_supply_propval *val)
{
	int ret;
	int data;
	enum power_supply_property qcom_psp;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_STATUS:
		qcom_psp = POWER_SUPPLY_PROP_STATUS;
		break;
	case LENOVO_SUPPLY_PROP_HEALTH:
		qcom_psp = POWER_SUPPLY_PROP_HEALTH;
		break;
	case LENOVO_SUPPLY_PROP_CAPACITY:
		qcom_psp = POWER_SUPPLY_PROP_CAPACITY;
		break;
	case LENOVO_SUPPLY_PROP_VOLTAGE_NOW:
		qcom_psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;
		break;
	case LENOVO_SUPPLY_PROP_CURRENT_NOW:
		qcom_psp = POWER_SUPPLY_PROP_CURRENT_NOW;
		break;
	case LENOVO_SUPPLY_PROP_TEMP:
		qcom_psp = POWER_SUPPLY_PROP_TEMP;
		break;
	default:
		return -EINVAL;
	}

        ret = qcom_battery_read(qcom_psp, &data);
        if (ret < 0) {
                pr_err("Failed to get battery data, ret=%d\n", ret);
		return ret;
	}

	switch (psp) {
	case LENOVO_SUPPLY_PROP_CAPACITY:
		val->intval = data / 100;
		break;
	case LENOVO_SUPPLY_PROP_TEMP:
		val->intval = data / 10;
		break;
	default:
		val->intval = data;
		break;
	}

	return 0;
}

static int qcom_batt_bak_set_property(struct lenovo_supply *psy,
			       enum lenovo_supply_property prop,
			       const union lenovo_supply_propval *val)
{
	return 0;
}


static int qcom_batt_bak_prop_is_writeable(struct lenovo_supply *psy,
				       enum lenovo_supply_property prop)
{
	int ret = 0;
	return ret;
}

static enum lenovo_supply_property qcom_usb_bak_props[] = {
	LENOVO_SUPPLY_PROP_ONLINE,
	LENOVO_SUPPLY_PROP_REAL_TYPE,
};

static int qcom_usb_bak_get_property(struct lenovo_supply *psy, enum lenovo_supply_property psp,
					union lenovo_supply_propval *val)
{

	int ret;
	int data;
	enum power_supply_property qcom_psp;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_ONLINE:
		qcom_psp = POWER_SUPPLY_PROP_ONLINE;
		break;
	case LENOVO_SUPPLY_PROP_REAL_TYPE:
		qcom_psp = POWER_SUPPLY_PROP_USB_TYPE;
		break;
	default:
		return -EINVAL;
	}

        ret = qcom_usb_read(qcom_psp, &data);
        if (ret < 0) {
                pr_err("Failed to get usb data, ret=%d\n", ret);
		return ret;
	}

	switch (psp) {
	default:
		val->intval = data;
		break;
	}

	return 0;
}

static int qcom_usb_bak_set_property(struct lenovo_supply *psy,
			       enum lenovo_supply_property prop,
			       const union lenovo_supply_propval *val)
{
	return 0;
}


static int qcom_usb_bak_prop_is_writeable(struct lenovo_supply *psy,
				       enum lenovo_supply_property prop)
{
	int ret = 0;
	return ret;
}

#ifdef SUPPORT_USER_THERMAL_CASE
int lenovo_get_prop_therm_display_rate_level_max(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_levels_display_rate;
	return 0;
}

int lenovo_get_prop_therm_display_rate_level(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_display_rate_level;
	return 0;
}

int lenovo_set_prop_therm_display_rate_level(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_display_rate <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_display_rate)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_display_rate)
		chg->thermal_display_rate_level = chg->thermal_levels_display_rate - 1;
	else
		chg->thermal_display_rate_level = val->intval;

	return 0;
}

int lenovo_get_prop_therm_display_rate_limit(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	if (!chg->thermal_mitigation_display_rate)
		return -1;

	val->intval = chg->thermal_mitigation_display_rate[chg->thermal_display_rate_level];
	return 0;
}

int lenovo_get_prop_therm_display_rate(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_current_display_rate;
	return 0;
}

int lenovo_set_prop_therm_display_rate(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	chg->thermal_current_display_rate = val->intval;
	return 0;
}

int lenovo_get_prop_therm_speaker_level_max(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_levels_speaker;
	return 0;
}

int lenovo_get_prop_therm_speaker_level(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_speaker_level;
	return 0;
}

int lenovo_set_prop_therm_speaker_level(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_speaker <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_speaker)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_speaker)
		chg->thermal_speaker_level = chg->thermal_levels_speaker - 1;
	else
		chg->thermal_speaker_level = val->intval;

	return 0;
}

int lenovo_get_prop_therm_speaker_limit(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	if (!chg->thermal_mitigation_speaker)
		return -1;

	val->intval = chg->thermal_mitigation_speaker[chg->thermal_speaker_level];
	return 0;
}

int lenovo_get_prop_therm_speaker(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_current_speaker;
	return 0;
}

int lenovo_set_prop_therm_speaker(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	chg->thermal_current_speaker = val->intval;
	return 0;
}

int lenovo_get_prop_therm_modem_5g_level_max(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_levels_modem_5g;
	return 0;
}

int lenovo_get_prop_therm_modem_5g_level(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_modem_5g_level;
	return 0;
}

int lenovo_set_prop_therm_modem_5g_level(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_modem_5g <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_modem_5g)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_modem_5g)
		chg->thermal_modem_5g_level = chg->thermal_levels_modem_5g - 1;
	else
		chg->thermal_modem_5g_level = val->intval;

	return 0;
}

int lenovo_get_prop_therm_modem_5g_limit(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	if (!chg->thermal_mitigation_modem_5g)
		return -1;

	val->intval = chg->thermal_mitigation_modem_5g[chg->thermal_modem_5g_level];
	return 0;
}

int lenovo_get_prop_therm_modem_5g(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_current_modem_5g;
	return 0;
}

int lenovo_set_prop_therm_modem_5g(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	chg->thermal_current_modem_5g = val->intval;
	return 0;
}

int lenovo_get_prop_therm_camera_level_max(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_levels_camera;
	return 0;
}

int lenovo_get_prop_therm_camera_level(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_camera_level;
	return 0;
}

int lenovo_set_prop_therm_camera_level(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_camera <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_camera)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_camera)
		chg->thermal_camera_level = chg->thermal_levels_camera - 1;
	else
		chg->thermal_camera_level = val->intval;

	return 0;
}

int lenovo_get_prop_therm_camera_limit(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	if (!chg->thermal_mitigation_camera)
		return -1;

	val->intval = chg->thermal_mitigation_camera[chg->thermal_camera_level];
	return 0;
}

int lenovo_get_prop_therm_camera(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_current_camera;
	return 0;
}

int lenovo_set_prop_therm_camera(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	chg->thermal_current_camera = val->intval;
	return 0;
}

int lenovo_get_prop_therm_event_level_max(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_levels_event;
	return 0;
}

int lenovo_get_prop_therm_event_level(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_event_level;
	return 0;
}

int lenovo_set_prop_therm_event_level(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_event <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_event)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_event)
		chg->thermal_event_level = chg->thermal_levels_event - 1;
	else
		chg->thermal_event_level = val->intval;

	return 0;
}

int lenovo_get_prop_therm_event_limit(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	if (!chg->thermal_mitigation_event)
		return -1;

	val->intval = chg->thermal_mitigation_event[chg->thermal_event_level];
	return 0;
}

int lenovo_get_prop_therm_event(struct bq_fg_chip *chg,
				union lenovo_supply_propval *val)
{
	val->intval = chg->thermal_current_event;
	return 0;
}

int lenovo_set_prop_therm_event(struct bq_fg_chip *chg,
				const union lenovo_supply_propval *val)
{
	chg->thermal_current_event = val->intval;
	return 0;
}

static enum lenovo_supply_property thermal_props[] = {
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
	LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL,
	LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL_MAX,
	LENOVO_SUPPLY_PROP_THERM_EVENT,
	LENOVO_SUPPLY_PROP_THERM_EVENT_LIMIT,
};

static int thermal_get_property(struct lenovo_supply *psy,
		enum lenovo_supply_property psp,
		union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL:
		ret = lenovo_get_prop_therm_display_rate_level(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL_MAX:
		ret = lenovo_get_prop_therm_display_rate_level_max(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE:
		ret = lenovo_get_prop_therm_display_rate(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LIMIT:
		ret = lenovo_get_prop_therm_display_rate_limit(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL:
		ret = lenovo_get_prop_therm_speaker_level(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL_MAX:
		ret = lenovo_get_prop_therm_speaker_level_max(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER:
		ret = lenovo_get_prop_therm_speaker(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER_LIMIT:
		ret = lenovo_get_prop_therm_speaker_limit(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL:
		ret = lenovo_get_prop_therm_modem_5g_level(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL_MAX:
		ret = lenovo_get_prop_therm_modem_5g_level_max(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G:
		ret = lenovo_get_prop_therm_modem_5g(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LIMIT:
		ret = lenovo_get_prop_therm_modem_5g_limit(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL:
		ret = lenovo_get_prop_therm_camera_level(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL_MAX:
		ret = lenovo_get_prop_therm_camera_level_max(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA:
		ret = lenovo_get_prop_therm_camera(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA_LIMIT:
		ret = lenovo_get_prop_therm_camera_limit(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL:
		ret = lenovo_get_prop_therm_event_level(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL_MAX:
		ret = lenovo_get_prop_therm_event_level_max(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT:
		ret = lenovo_get_prop_therm_event(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT_LIMIT:
		ret = lenovo_get_prop_therm_event_limit(bq, val);
		break;
	default:
		return -EINVAL;
	}
	if (ret < 0) {
		pr_debug("Couldn't get prop %d ret = %d\n", psp, ret);
		return -ENODATA;
	}
	return 0;
}

static int thermal_set_property(struct lenovo_supply *psy,
		enum lenovo_supply_property psp,
		const union lenovo_supply_propval *val)
{
	struct bq_fg_chip *bq = lenovo_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL:
		pr_info("batt_sys: write display rate level %d\n", val->intval);
		ret = lenovo_set_prop_therm_display_rate_level(bq, val);
		lenovo_supply_changed(bq->thermal_psy);
		break;
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE:
		pr_info("batt_sys: write current display rate value %d\n", val->intval);
		ret = lenovo_set_prop_therm_display_rate(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL:
		pr_info("batt_sys: write speaker level %d\n", val->intval);
		ret = lenovo_set_prop_therm_speaker_level(bq, val);
		lenovo_supply_changed(bq->thermal_psy);
		break;
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER:
		pr_info("batt_sys: write current speaker value %d\n", val->intval);
		ret = lenovo_set_prop_therm_speaker(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL:
		pr_info("batt_sys: write modem 5g level %d\n", val->intval);
		ret = lenovo_set_prop_therm_modem_5g_level(bq, val);
		lenovo_supply_changed(bq->thermal_psy);
		break;
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G:
		pr_info("batt_sys: write current modem 5g value %d\n", val->intval);
		ret = lenovo_set_prop_therm_modem_5g(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL:
		pr_info("batt_sys: write camera level %d\n", val->intval);
		ret = lenovo_set_prop_therm_camera_level(bq, val);
		lenovo_supply_changed(bq->thermal_psy);
		break;
	case LENOVO_SUPPLY_PROP_THERM_CAMERA:
		pr_info("batt_sys: write current camera value %d\n", val->intval);
		ret = lenovo_set_prop_therm_camera(bq, val);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL:
		pr_info("batt_sys: write event level %d\n", val->intval);
		ret = lenovo_set_prop_therm_event_level(bq, val);
		lenovo_supply_changed(bq->thermal_psy);
		break;
	case LENOVO_SUPPLY_PROP_THERM_EVENT:
		pr_info("batt_sys: write current event value %d\n", val->intval);
		ret = lenovo_set_prop_therm_event(bq, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int thermal_prop_is_writeable(struct lenovo_supply *psy,
		enum lenovo_supply_property psp)
{
	switch (psp) {
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE_LEVEL:
	case LENOVO_SUPPLY_PROP_THERM_DISPLAY_RATE:
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER_LEVEL:
	case LENOVO_SUPPLY_PROP_THERM_SPEAKER:
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G_LEVEL:
	case LENOVO_SUPPLY_PROP_THERM_MODEM_5G:
	case LENOVO_SUPPLY_PROP_THERM_CAMERA_LEVEL:
	case LENOVO_SUPPLY_PROP_THERM_CAMERA:
	case LENOVO_SUPPLY_PROP_THERM_EVENT_LEVEL:
	case LENOVO_SUPPLY_PROP_THERM_EVENT:
		return 1;
	default:
		break;
	}

	return 0;
}
#endif

static int fg_ap_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config fg_ap_psy_cfg = {};

	if (bq->mode == BQ27Z561_MASTER)
		bq->fg_ap_psy_d.name = "bq27z561-master";
	else
		bq->fg_ap_psy_d.name = "bq27z561-slave";
	bq->fg_ap_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->fg_ap_psy_d.properties = fg_ap_props;
	bq->fg_ap_psy_d.num_properties = ARRAY_SIZE(fg_ap_props);
	bq->fg_ap_psy_d.get_property = fg_ap_get_property;
	bq->fg_ap_psy_d.set_property = fg_ap_set_property;
	bq->fg_ap_psy_d.property_is_writeable = fg_ap_prop_is_writeable;

	fg_ap_psy_cfg.drv_data = bq;
	fg_ap_psy_cfg.num_supplicants = 0;
	bq->fg_ap_psy = devm_lenovo_supply_register(bq->dev,
						&bq->fg_ap_psy_d,
						&fg_ap_psy_cfg);
	if (IS_ERR(bq->fg_ap_psy)) {
		bq_err("Failed to register fg_ap_psy");
		return PTR_ERR(bq->fg_ap_psy);
	}

	return 0;
}

static int fg_adsp_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config fg_adsp_psy_cfg = {};

	bq->fg_adsp_psy_d.name = "bq27z561-master";
		
	bq->fg_adsp_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->fg_adsp_psy_d.properties = fg_adsp_props;
	bq->fg_adsp_psy_d.num_properties = ARRAY_SIZE(fg_adsp_props);
	bq->fg_adsp_psy_d.get_property = fg_adsp_get_property;
	bq->fg_adsp_psy_d.set_property = fg_adsp_set_property;
	bq->fg_adsp_psy_d.property_is_writeable = fg_adsp_prop_is_writeable;

	fg_adsp_psy_cfg.drv_data = bq;
	fg_adsp_psy_cfg.num_supplicants = 0;
	bq->fg_adsp_psy = devm_lenovo_supply_register(bq->dev,
						&bq->fg_adsp_psy_d,
						&fg_adsp_psy_cfg);
	if (IS_ERR(bq->fg_adsp_psy)) {
		bq_err("Failed to register fg_adsp_psy");
		return PTR_ERR(bq->fg_adsp_psy);
	}

	return 0;
}

static int fg_user_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config fg_user_psy_cfg = {};

	bq->fg_user_psy_d.name = "bq-battery";

	bq->fg_user_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->fg_user_psy_d.properties = fg_user_props;
	bq->fg_user_psy_d.num_properties = ARRAY_SIZE(fg_user_props);
	bq->fg_user_psy_d.get_property = fg_user_get_property;
	bq->fg_user_psy_d.set_property = fg_user_set_property;
	bq->fg_user_psy_d.property_is_writeable = fg_user_prop_is_writeable;

	fg_user_psy_cfg.drv_data = bq;
	fg_user_psy_cfg.num_supplicants = 0;
	bq->fg_user_psy = devm_lenovo_supply_register(bq->dev,
						&bq->fg_user_psy_d,
						&fg_user_psy_cfg);
	if (IS_ERR(bq->fg_user_psy)) {
		bq_err("Failed to register fg_user_psy");
		return PTR_ERR(bq->fg_user_psy);
	}

	return 0;
}

#ifdef SUPPORT_USER_THERMAL_CASE
static int thermal_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config thermal_psy_cfg = {};

	bq->thermal_psy_d.name = "thermal";

	bq->thermal_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->thermal_psy_d.properties = thermal_props;
	bq->thermal_psy_d.num_properties = ARRAY_SIZE(thermal_props);
	bq->thermal_psy_d.get_property = thermal_get_property;
	bq->thermal_psy_d.set_property = thermal_set_property;
	bq->thermal_psy_d.property_is_writeable = thermal_prop_is_writeable;

	thermal_psy_cfg.drv_data = bq;
	thermal_psy_cfg.num_supplicants = 0;
	bq->thermal_psy = devm_lenovo_supply_register(bq->dev,
						&bq->thermal_psy_d,
						&thermal_psy_cfg);
	if (IS_ERR(bq->thermal_psy)) {
		bq_err("Failed to register thermal_psy");
		return PTR_ERR(bq->thermal_psy);
	}

	return 0;
}
#endif

static int qcom_batt_bak_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config qcom_batt_bak_psy_cfg = {};

	bq->qcom_batt_bak_psy_d.name = "qcom-battery";

	bq->qcom_batt_bak_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->qcom_batt_bak_psy_d.properties = qcom_batt_bak_props;
	bq->qcom_batt_bak_psy_d.num_properties = ARRAY_SIZE(qcom_batt_bak_props);
	bq->qcom_batt_bak_psy_d.get_property = qcom_batt_bak_get_property;
	bq->qcom_batt_bak_psy_d.set_property = qcom_batt_bak_set_property;
	bq->qcom_batt_bak_psy_d.property_is_writeable = qcom_batt_bak_prop_is_writeable;

	qcom_batt_bak_psy_cfg.drv_data = bq;
	qcom_batt_bak_psy_cfg.num_supplicants = 0;
	bq->qcom_batt_bak_psy = devm_lenovo_supply_register(bq->dev,
						&bq->qcom_batt_bak_psy_d,
						&qcom_batt_bak_psy_cfg);
	if (IS_ERR(bq->qcom_batt_bak_psy)) {
		bq_err("Failed to register qcom_batt_bak_psy");
		return PTR_ERR(bq->qcom_batt_bak_psy);
	}

	return 0;
}

static int qcom_usb_bak_psy_register(struct bq_fg_chip *bq)
{
	struct lenovo_supply_config qcom_usb_bak_psy_cfg = {};

	bq->qcom_usb_bak_psy_d.name = "qcom-usb";

	bq->qcom_usb_bak_psy_d.type = LENOVO_SUPPLY_TYPE_BATTERY;
	bq->qcom_usb_bak_psy_d.properties = qcom_usb_bak_props;
	bq->qcom_usb_bak_psy_d.num_properties = ARRAY_SIZE(qcom_usb_bak_props);
	bq->qcom_usb_bak_psy_d.get_property = qcom_usb_bak_get_property;
	bq->qcom_usb_bak_psy_d.set_property = qcom_usb_bak_set_property;
	bq->qcom_usb_bak_psy_d.property_is_writeable = qcom_usb_bak_prop_is_writeable;

	qcom_usb_bak_psy_cfg.drv_data = bq;
	qcom_usb_bak_psy_cfg.num_supplicants = 0;
	bq->qcom_usb_bak_psy = devm_lenovo_supply_register(bq->dev,
						&bq->qcom_usb_bak_psy_d,
						&qcom_usb_bak_psy_cfg);
	if (IS_ERR(bq->qcom_usb_bak_psy)) {
		bq_err("Failed to register qcom_usb_bak_psy");
		return PTR_ERR(bq->qcom_usb_bak_psy);
	}

	return 0;
}

static void fg_psy_unregister(struct bq_fg_chip *bq)
{
	if (bq->mode == BQ27Z561_MASTER) {
		lenovo_supply_unregister(bq->fg_user_psy);
		lenovo_supply_unregister(bq->fg_adsp_psy);
		lenovo_supply_unregister(bq->qcom_batt_bak_psy);
		lenovo_supply_unregister(bq->qcom_usb_bak_psy);
#ifdef SUPPORT_USER_THERMAL_CASE
		lenovo_supply_unregister(bq->thermal_psy);
#endif
	}
	lenovo_supply_unregister(bq->fg_ap_psy);
}

static ssize_t fg_attr_show_Ra_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 t_buf[40];
	u8 temp_buf[40];
	int ret;
	int i, idx, len;

	ret = fg_mac_read_block(bq, 0x40C0, t_buf, 32);
	if (ret != 0) {
		bq_err("Failed to read Ra table:%d\n", ret);
		return 0;
	}

	idx = 0;
	len = sprintf(temp_buf, "Ra Flag:0x%02X\n", t_buf[1] << 8 | t_buf[0]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "RaTable:\n");
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	for (i = 1; i < 16; i++) {
		len = sprintf(temp_buf, "%d ", t_buf[i*2 + 1] << 8 | t_buf[i*2]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}

	return idx;
}

static ssize_t fg_attr_show_Qmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[64];
	int len;

	memset(t_buf, 0, 64);

	ret = fg_mac_read_block(bq, 0x4146, t_buf, 2);
	if (ret < 0) {
		bq_err("Failed to read Qmax:%d\n", ret);
		return 0;
	}

	//len = sprintf(buf, "Qmax Cell 0 = %d\n", (t_buf[1] << 8) | t_buf[0]);
	len = sprintf(buf, "%d\n", (t_buf[1] << 8) | t_buf[0]);

	return len;
}

static ssize_t fg_attr_show_Deivce_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 t_buf[40];
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 temp_buf[40];
	int ret;
	int idx, len;

	idx = 0;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_TYPE, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read device type:%d\n", ret);
		return 0;
	} 
	len = sprintf(temp_buf, "Device Type:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_HW_VER, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read hardware version:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "HW Ver:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_CHEM_ID, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read chemical ID:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Chemical ID:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_NAME, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read devie name:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Dev Name:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_CHEM, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read devie chem:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Dev Chem:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_MANU_NAME, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read manufacturer name:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Manufacturer:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_MANU_DATE, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read manufacture date:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Manu Date:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_SERIAL_NUMBER, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read serial number:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Serial:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	return idx;
}


static ssize_t fg_attr_show_fw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[64];
	int len;

	memset(t_buf, 0, 64);

	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, t_buf, 11);
	if (ret != 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		len = sprintf(buf, "%04X\n", 0x0);
	} else
		len = sprintf(buf, "%04X\n", t_buf[3] << 8 | t_buf[2]);

	return len;
}

static int fg_read_itstatus1(struct bq_fg_chip *bq, int of)
{
	int ret;
	u8 buf[36];

	ret = fg_mac_read_block(bq, FG_MAC_CMD_ITSTATUS1, buf, 20);
	if (ret != 0) {
		bq_err("Failed to read itstatus1, offset=%d, err=%d\n", of, ret);
		return 0;
	}

	return (buf[of + 1] << 8 | buf[of]);
}

static ssize_t fg_attr_show_TrueRemQ(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 0));
}

static ssize_t fg_attr_show_TrueFullChgQ(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 8));
}

static ssize_t fg_attr_show_T_sim(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 12));
}

static ssize_t fg_attr_show_T_ambient(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 14));
}

static DEVICE_ATTR(RaTable, S_IRUGO, fg_attr_show_Ra_table, NULL);
static DEVICE_ATTR(Qmax, S_IRUGO, fg_attr_show_Qmax, NULL);
static DEVICE_ATTR(DeviceInfo, S_IRUGO, fg_attr_show_Deivce_info, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, fg_attr_show_fw, NULL);
static DEVICE_ATTR(TrueRemQ, S_IRUGO, fg_attr_show_TrueRemQ, NULL);
static DEVICE_ATTR(TrueFullChgQ, S_IRUGO, fg_attr_show_TrueFullChgQ, NULL);
static DEVICE_ATTR(Tsim, S_IRUGO, fg_attr_show_T_sim, NULL);
static DEVICE_ATTR(Tambient, S_IRUGO, fg_attr_show_T_ambient, NULL);

static struct attribute *fg_attributes[] = {
	&dev_attr_RaTable.attr,
	&dev_attr_Qmax.attr,
	&dev_attr_DeviceInfo.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_TrueRemQ.attr,
	&dev_attr_TrueFullChgQ.attr,
	&dev_attr_Tsim.attr,
	&dev_attr_Tambient.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};


static irqreturn_t fg_irq_thread(int irq, void *dev_id)
{
	struct bq_fg_chip *bq = dev_id;
	u8 status;
	int ret;

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		bq_err("IRQ triggered before device resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	fg_read_status(bq);

	fg_dump_registers(bq);

	mutex_lock(&bq->data_lock);

	bq->batt_soc = fg_ap_read_rsoc(bq);
	bq->batt_soh = fg_ap_read_soh(bq);
	bq->batt_volt = fg_ap_read_volt(bq);
	fg_ap_read_current(bq, &bq->batt_curr2);
	bq->batt_temp = fg_ap_read_temperature(bq);
	bq->batt_rm = fg_ap_read_rm(bq);

	mutex_unlock(&bq->data_lock);

	bq_log("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		bq->batt_soc, bq->batt_volt, bq->batt_curr2, bq->batt_temp);

	ret = fg_read_byte(bq, 0x6E, &status); /*InterruptStatus Reg*/
	if (!ret) {
		bq_log("VOLT_HI %s\n", status & 0x01 ? "set" : "clear");
		bq_log("TEMP_HI %s\n", status & 0x04 ? "set" : "clear");
		bq_log("VOLT_LOW %s\n", status & 0x02 ? "set" : "clear");
		bq_log("TEMP_LOW %s\n", status & 0x08 ? "set" : "clear");
	}
	return IRQ_HANDLED;
}


static void determine_initial_status(struct bq_fg_chip *bq)
{
	fg_irq_thread(bq->client->irq, bq);
}

static struct of_device_id bq_fg_match_table[] = {
	{
		.compatible = "ti,bq27z561-master",
		.data = &bq27z561_mode_data[BQ27Z561_MASTER],
	},
	{
		.compatible = "ti,bq27z561-slave",
		.data = &bq27z561_mode_data[BQ27Z561_SLAVE],
	},
	{},
};
//MODULE_DEVICE_TABLE(of, bq_fg_match_table);

#define CHECK_FAKE_BATTERY_NUM			20
static void fake_battery_check_work(struct work_struct *work)
{
	struct bq_fg_chip *bq = container_of(work, struct bq_fg_chip, fake_batt_work.work);
	int fw;

	if (!bq->fw1_check_done) {
		fw = fg_adsp_read_chip_fw(bq);
		bq_log("Fake battery check fw: master 0x%02x\n", fw);
		if ((fw != 0x00) && (fw != 0xFF))
			bq->fw1_check_done = 1;
			bq->master_fw = fw;
	}

	if (!bq->fw2_check_done) {
		fw = fg_ap_read_chip_fw(bq);
		bq_log("Fake battery check fw: slave 0x%02x\n", fw);
		if ((fw != 0x00) && (fw != 0xFF))
			bq->fw2_check_done = 1;
	}

	if (++bq->check_fake_count >= CHECK_FAKE_BATTERY_NUM) {
		is_fake_master_battery = bq->fw1_check_done ? 0 : 1;
		is_fake_slave_battery = bq->fw2_check_done ? 0 : 1;
		bq_log("Fake battery check timeout: count %d fake-master %d fake-slave %d\n",
			bq->check_fake_count, is_fake_master_battery, is_fake_slave_battery);
		return;
	} else {
		if (bq->fw1_check_done && bq->fw2_check_done) {
			bq_log("Fake battery check done: count %d fake-master %d fake-slave %d\n",
				bq->check_fake_count, is_fake_master_battery, is_fake_slave_battery);
			return;
		}
	}

	bq_log("Fake battery check: count %d fake-master %d/%d fake-slave %d/%d \n",
		bq->check_fake_count, is_fake_master_battery, bq->fw1_check_done,
		 is_fake_slave_battery, bq->fw2_check_done);
	schedule_delayed_work(&bq->fake_batt_work, 2 * HZ);
}

#ifdef SUPPORT_USER_THERMAL_CASE
static int bq_parse_dt(struct device *dev, struct bq_fg_chip *bq)
{
	struct device_node *node = dev->of_node;
	int rc;

	/* get Camera thermal level */
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation-camera",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	bq->thermal_levels_camera = rc;
	bq->thermal_mitigation_camera = devm_kcalloc(bq->dev,
					bq->thermal_levels_camera,
					sizeof(*bq->thermal_mitigation_camera),
					GFP_KERNEL);
	if (!bq->thermal_mitigation_camera)
		return -ENOMEM;

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation-camera",
					bq->thermal_mitigation_camera,
					bq->thermal_levels_camera);

	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation-camera, rc=%d\n", rc);
		return rc;
	}

	/* get Display Rate thermal level */
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation-display-rate",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	bq->thermal_levels_display_rate = rc;
	bq->thermal_mitigation_display_rate = devm_kcalloc(bq->dev,
					bq->thermal_levels_display_rate,
					sizeof(*bq->thermal_mitigation_display_rate),
					GFP_KERNEL);
	if (!bq->thermal_mitigation_display_rate)
		return -ENOMEM;

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation-display-rate",
					bq->thermal_mitigation_display_rate,
					bq->thermal_levels_display_rate);

	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation-display-rate, rc=%d\n", rc);
		return rc;
	}

	/* get Speaker thermal level */
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation-speaker",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	bq->thermal_levels_speaker = rc;
	bq->thermal_mitigation_speaker = devm_kcalloc(bq->dev,
					bq->thermal_levels_speaker,
					sizeof(*bq->thermal_mitigation_speaker),
					GFP_KERNEL);
	if (!bq->thermal_mitigation_speaker)
		return -ENOMEM;

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation-speaker",
					bq->thermal_mitigation_speaker,
					bq->thermal_levels_speaker);

	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation-speaker, rc=%d\n", rc);
		return rc;
	}

	/* get Modem 5G thermal level */
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation-modem-5g",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	bq->thermal_levels_modem_5g = rc;
	bq->thermal_mitigation_modem_5g = devm_kcalloc(bq->dev,
					bq->thermal_levels_modem_5g,
					sizeof(*bq->thermal_mitigation_modem_5g),
					GFP_KERNEL);
	if (!bq->thermal_mitigation_modem_5g)
		return -ENOMEM;

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation-modem-5g",
					bq->thermal_mitigation_modem_5g,
					bq->thermal_levels_modem_5g);

	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation-modem-5g, rc=%d\n", rc);
		return rc;
	}

	/* get event thermal level */
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation-event",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	bq->thermal_levels_event = rc;
	bq->thermal_mitigation_event = devm_kcalloc(bq->dev,
					bq->thermal_levels_event,
					sizeof(*bq->thermal_mitigation_event),
					GFP_KERNEL);
	if (!bq->thermal_mitigation_event)
		return -ENOMEM;

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation-event",
					bq->thermal_mitigation_event,
					bq->thermal_levels_event);

	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation-event, rc=%d\n", rc);
		return rc;
	}

	return 0;
}
#endif

static int bq_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	int ret;
	struct bq_fg_chip *bq;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	u8 *regs;

	pr_err("enter\n");
	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->batt_soc	= -ENODATA;
	bq->batt_soh	= -ENODATA;
	bq->batt_fcc	= -ENODATA;
	bq->batt_rm	= -ENODATA;
	bq->batt_dc	= -ENODATA;
	bq->batt_volt	= -ENODATA;
	bq->batt_temp	= -ENODATA;
	bq->batt_curr1	= -ENODATA;
	bq->batt_curr2	= -ENODATA;
	bq->batt_cyclecnt = -ENODATA;

	bq->last_batt_soc = 0;
	bq->last_batt_soh = 0;
	bq->last_batt_volt = 0;
	bq->last_batt_temp = 0;
	bq->last_batt_status = LENOVO_SUPPLY_STATUS_DISCHARGING;
	bq->last_batt_present = 1;
	bq->last_batt_current = 0;
	bq->last_batt_charge_full = 0;
	bq->last_batt_design_full = 0;
	bq->last_batt_cc = 0;
	bq->fw1_check_done = 0;
	bq->fw2_check_done = 0;
	bq->master_fw = 0;
	bq->check_fake_count = 0;
	bq->fg_read_enable = 1;

	bq->fake_soc	= -EINVAL;
	bq->fake_temp	= -EINVAL;

	match = of_match_node(bq_fg_match_table, node);
	if (match == NULL) {
		pr_err("device tree match not found!\n");
		return -ENODEV;
	}
	bq->mode	= *(int *)match->data;
	pr_err("check %s\n", device2str[bq->mode]);

	if (bq->chip == BQ27Z561_MASTER) {
		regs = bq27z561_regs;
	} else if (bq->chip == BQ27Z561_SLAVE) {
		regs = bq27z561_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq27z561_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			fg_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq fuel gauge irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	fg_fw_init(bq);

#ifdef SUPPORT_USER_THERMAL_CASE
	bq->thermal_display_rate_level = 0;
	bq->thermal_levels_display_rate = 0;
	bq->thermal_current_display_rate = -1;
	bq->thermal_speaker_level = 0;
	bq->thermal_levels_speaker = 0;
	bq->thermal_current_speaker = -1;
	bq->thermal_modem_5g_level = 0;
	bq->thermal_levels_modem_5g = 0;
	bq->thermal_current_modem_5g = -1;
	bq->thermal_camera_level = 0;
	bq->thermal_levels_camera = 0;
	bq->thermal_current_camera = -1;
	bq->thermal_event_level = 0;
	bq->thermal_levels_event = 0;
	bq->thermal_current_event = -1;

	if (client->dev.of_node)
		bq_parse_dt(&client->dev, bq);
#endif

	fg_ap_psy_register(bq);
	if (bq->mode == BQ27Z561_SLAVE) {
		qcom_usb_bak_psy_register(bq);
		qcom_batt_bak_psy_register(bq);
		fg_adsp_psy_register(bq);
		fg_user_psy_register(bq);
#ifdef SUPPORT_USER_THERMAL_CASE
		thermal_psy_register(bq);
#endif
	}

	ret = sysfs_create_group(&bq->dev->kobj, &fg_attr_group);
	if (ret)
		bq_err("Failed to register sysfs, err:%d\n", ret);

	determine_initial_status(bq);

	INIT_DELAYED_WORK(&bq->fake_batt_work, fake_battery_check_work);
	schedule_delayed_work(&bq->fake_batt_work, 2 * HZ);

	bq_log("bq fuel gauge probe successfully, %s\n",
			device2str[bq->chip]);

	return 0;

err_1:
	fg_psy_unregister(bq);
	return ret;
}


static inline bool is_device_suspended(struct bq_fg_chip *bq)
{
	return !bq->resume_completed;
}


static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

#if 0
	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
#endif
	int ret;

        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_FG1_READ_ENABLE, 0);
        if (ret < 0)
                pr_err("Failed to disable adsp fg read, ret=%d\n", ret);
	else
                pr_err("disable adsp fg read, ret=%d\n", ret);
	bq->fg_read_enable = 0;

	return 0;
}

static int bq_fg_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

#if 0
	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending, %d\n", bq->mode);
		return -EBUSY;
	}
#endif
	int ret;

        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_FG1_READ_ENABLE, 0);
        if (ret < 0)
                pr_err("Failed to disable adsp fg read, ret=%d\n", ret);
	else
                pr_err("disable adsp fg read, ret=%d\n", ret);
	bq->fg_read_enable = 0;

	return 0;
}


static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

#if 0
	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		fg_irq_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	lenovo_supply_changed(bq->fg_ap_psy);
#endif
	int ret;

        ret = lenovo_glink_write(LENOVO_SUPPLY_PROP_FG1_READ_ENABLE, 1);
        if (ret < 0)
                pr_err("Failed to enable adsp fg read, ret=%d\n", ret);
	else
                pr_err("enable adsp fg read, ret=%d\n", ret);

	bq->fg_read_enable = 1;
	soh_jiffies = -1;
	die_temp_jiffies = -1;
	volt_jiffies = -1;
	curr_jiffies = -1;
	fcc_jiffies = -1;
	dc_jiffies = -1;
	rm_jiffies = -1;

	return 0;
}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	sysfs_remove_group(&bq->dev->kobj, &fg_attr_group);

	return 0;

}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_err("bq fuel gauge driver shutdown!\n");
}


static const struct i2c_device_id bq_fg_id[] = {
	{ "bq27z561-master", BQ27Z561_MASTER },
	{ "bq27z561-slave", BQ27Z561_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume		= bq_fg_resume,
	.suspend_noirq = bq_fg_suspend_noirq,
	.suspend	= bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver	= {
		.name   = "bq_fg",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
	},
	.id_table       = bq_fg_id,

	.probe          = bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ27Z561 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

