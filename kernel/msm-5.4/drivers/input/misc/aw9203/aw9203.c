/*
 * aw9203.c   aw9203 touch key module
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include "aw9203.h"
#include "aw9203_reg.h"
#include "aw9203_para.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW9203_I2C_NAME "aw9203_ts"
#define AW9203_I2C_NAME_L "aw9203_ts_l"

#define AW9203_DRIVER_VERSION "V1.1.4"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2
static unsigned long slide_time_max = (800L); //1s
static unsigned long slide_time_min = (150L); //70ms
static bool is_reset = true;
static bool aw9203_is_suspend = false;
static void aw9203_slide_report(struct aw9203 *aw9203);
/******************************************************
 *
 * variable
 *
 ******************************************************/

 /******************************************************
 *
 * aw9203 i2c write/read
 *
 ******************************************************/
static int i2c_write(struct aw9203 *aw9203,
		     unsigned char addr, unsigned int reg_data)
{
	int ret;
	u8 wbuf[512] = { 0 };

	struct i2c_msg msgs[] = {
		{
		 .addr = aw9203->i2c->addr,
		 .flags = 0,
		 .len = 3,
		 .buf = wbuf,
		 },
	};

	wbuf[0] = addr;
	wbuf[1] = (unsigned char)((reg_data & 0xff00) >> 8);
	wbuf[2] = (unsigned char)(reg_data & 0x00ff);

	ret = i2c_transfer(aw9203->i2c->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s: i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

static int i2c_read(struct aw9203 *aw9203,
		    unsigned char addr, unsigned int *reg_data)
{
	int ret;
	unsigned char rbuf[512] = { 0 };
	unsigned int get_data;

	struct i2c_msg msgs[] = {
		{
		 .addr = aw9203->i2c->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = aw9203->i2c->addr,
		 .flags = I2C_M_RD,
		 .len = 2,
		 .buf = rbuf,
		 },
	};

	ret = i2c_transfer(aw9203->i2c->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	get_data = (unsigned int)(rbuf[0] & 0x00ff);
	get_data <<= 8;
	get_data |= (unsigned int)rbuf[1];

	*reg_data = get_data;

	return ret;
}

static int aw9203_i2c_write(struct aw9203 *aw9203,
			    unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw9203, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw9203_i2c_read(struct aw9203 *aw9203,
			   unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw9203, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}
	return ret;
}

#if 0
static int aw9203_i2c_write_bits(struct aw9203 *aw9203,
				 unsigned char reg_addr, unsigned int mask,
				 unsigned int reg_data)
{
	unsigned int reg_val;

	aw9203_i2c_read(aw9203, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw9203_i2c_write(aw9203, reg_addr, reg_val);

	return 0;
}

static int aw9203_i2c_writes(struct aw9203 *aw9203,
			     unsigned char reg_addr, unsigned char *buf,
			     unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw9203->i2c, data, len + 1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	kfree(data);

	return ret;
}
#endif

/******************************************************
 *
 * auto cali
 *
 ******************************************************/
#if 0
static int aw9203_auto_cali(struct aw9203 *aw9203)
{
	unsigned char i;
	unsigned char cali_dir[6];

	unsigned int buf[6];
	unsigned int ofr_cfg[6];
	unsigned int sen_num;
	unsigned int reg_val;

	if (aw9203->cali_num == 0) {
		aw9203_i2c_read(aw9203, AW9203_REG_OFR1, &reg_val);
		ofr_cfg[0] = reg_val;
		aw9203_i2c_read(aw9203, AW9203_REG_OFR2, &reg_val);
		ofr_cfg[1] = reg_val;
		aw9203_i2c_read(aw9203, AW9203_REG_OFR3, &reg_val);
		ofr_cfg[2] = reg_val;
	} else {
		for (i = 0; i < 3; i++) {
			ofr_cfg[i] = aw9203->old_ofr_cfg[i];
		}
	}

	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x3);
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		buf[i] = reg_val;
	}
	aw9203_i2c_read(aw9203, AW9203_REG_SLPR, &sen_num);

	for (i = 0; i < 6; i++)
		aw9203->rawdata_sum[i] = (aw9203->cali_cnt == 0) ?
		    (0) : (aw9203->rawdata_sum[i] + buf[i]);

	if (aw9203->cali_cnt == 4) {
		for (i = 0; i < 6; i++) {
			/* sensor used */
			if ((sen_num & (1 << i)) == 0) {
				if ((aw9203->rawdata_sum[i] >> 2) <
				    CALI_RAW_MIN) {
					if ((i % 2)
					    && ((ofr_cfg[i >> 1] & 0xFF00) ==
						0x1000)) {
						/* 0x10** -> 0x00** */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] & 0x00FF;
						cali_dir[i] = 2;
					} else if ((i % 2)
						   &&
						   ((ofr_cfg[i >> 1] & 0xFF00)
						    == 0x0000)) {
						/* 0x00**    no calibration */
						cali_dir[i] = 0;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0010)) {
						/* 0x**10 -> 0x**00 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] & 0xFF00;
						cali_dir[i] = 2;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0000)) {
						/* 0x**00 no calibration */
						cali_dir[i] = 0;
					} else {
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] -
						    ((i % 2) ? (1 << 8) : 1);
						cali_dir[i] = 2;
					}
				} else if ((aw9203->rawdata_sum[i] >> 2) >
					   CALI_RAW_MAX) {
					if ((i % 2)
					    && ((ofr_cfg[i >> 1] & 0xFF00) ==
						0x1F00)) {
						/* 0x1F** no calibration */
						cali_dir[i] = 0;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x001F)) {
						/* 0x**1F no calibration */
						cali_dir[i] = 0;
					} else if ((i % 2)
						   &&
						   ((ofr_cfg[i >> 1] & 0xFF00)
						    == 0x0000)) {
						/* 0x00** -> 0x1000 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] | 0x1000;
						cali_dir[i] = 1;
					} else if (((i % 2) == 0)
						   &&
						   ((ofr_cfg[i >> 1] & 0x00FF)
						    == 0x0000)) {
						/* 0x**00 -> 0x**10 */
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] | 0x0010;
						cali_dir[i] = 1;
					} else {
						ofr_cfg[i >> 1] =
						    ofr_cfg[i >> 1] +
						    ((i % 2) ? (1 << 8) : 1);
						cali_dir[i] = 1;
					}
				} else {
					cali_dir[i] = 0;
				}

				if (aw9203->cali_num > 0) {
					if (cali_dir[i] !=
					    aw9203->old_cali_dir[i]) {
						cali_dir[i] = 0;
						ofr_cfg[i >> 1] =
						    aw9203->old_ofr_cfg[i >> 1];
					}
				}
			}
		}

		aw9203->cali_flag = 0;
		for (i = 0; i < 6; i++) {
			/* sensor used */
			if ((sen_num & (1 << i)) == 0) {
				if (cali_dir[i] != 0) {
					aw9203->cali_flag = 1;
				}
			}
		}
		if ((aw9203->cali_flag == 0) && (aw9203->cali_num == 0)) {
			aw9203->cali_used = 0;
		} else {
			aw9203->cali_used = 1;
		}

		if (aw9203->cali_flag == 0) {
			aw9203->cali_num = 0;
			aw9203->cali_cnt = 0;
			return 0;
		}
		/* touch disbale */
		aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
		reg_val &= 0xFFFD;
		aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);
		for (i = 0; i < 3; i++) {
			aw9203_i2c_write(aw9203, AW9203_REG_OFR1 + i,
					 ofr_cfg[i]);
		}
		aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
		reg_val |= 0x0002;
		aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);

		/* no calibration */
		if (aw9203->cali_num == (CALI_NUM - 1)) {
			aw9203->cali_flag = 0;
			aw9203->cali_num = 0;
			aw9203->cali_cnt = 0;
			return 0;
		}

		for (i = 0; i < 6; i++) {
			aw9203->old_cali_dir[i] = cali_dir[i];
		}

		for (i = 0; i < 3; i++) {
			aw9203->old_ofr_cfg[i] = ofr_cfg[i];
		}

		aw9203->cali_num++;
	}

	if (aw9203->cali_cnt < 4) {
		aw9203->cali_cnt++;
	} else {
		aw9203->cali_cnt = 0;
	}

	return 0;
}
#endif

/******************************************************
 *
 * aw9203 cfg
 *
 ******************************************************/
static void aw9203_cali_init(struct aw9203 *aw9203)
{
	aw9203->cali_flag = 1;
	aw9203->cali_num = 0;
	aw9203->cali_cnt = 0;
}

static void aw9203_sleep_mode(struct aw9203 *aw9203, bool enable)
{
	pr_info("aw9203: mode = %d\n", enable);
	if (enable)
		/* touch key disable */
		aw9203_i2c_write(aw9203, AW9203_REG_SLPR, 0x1C);
	else
		/* touch key enable */
		aw9203_i2c_write(aw9203, AW9203_REG_SLPR, AW9203_NORMAL_SLPR);
}

static void aw9203_normal_mode_init(struct aw9203 *aw9203)
{
	unsigned int reg_val;
	int ret = 0;

	aw9203_cali_init(aw9203);

	/* touch disbale */
	ret = aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
	if (ret < 0) {
		pr_info("%s: read error!\n", __func__);
		return;
	}
	reg_val &= 0xFFFD;
	/* disable chip */
	aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);

	/* cap-touch config */
	/* touch key enable */
	aw9203_i2c_write(aw9203, AW9203_REG_SLPR, AW9203_NORMAL_SLPR);
	/* scan time setting */
	aw9203_i2c_write(aw9203, AW9203_REG_SCFG1, AW9203_NORMAL_SCFG1);
	/* bit0~3 is sense seting */
	aw9203_i2c_write(aw9203, AW9203_REG_SCFG2, AW9203_NORMAL_SCFG2);

	/* offset */
	aw9203_i2c_write(aw9203, AW9203_REG_OFR1, AW9203_NORMAL_OFR1);
	/* offset */
	aw9203_i2c_write(aw9203, AW9203_REG_OFR2, AW9203_NORMAL_OFR2);
	/* offset */
	aw9203_i2c_write(aw9203, AW9203_REG_OFR3, AW9203_NORMAL_OFR3);

	/* s1 press thred setting */
	aw9203_i2c_write(aw9203, AW9203_REG_THR2, AW9203_NORMAL_THR2);
	/* s2 press thred setting */
	aw9203_i2c_write(aw9203, AW9203_REG_THR3, AW9203_NORMAL_THR3);
	/* s3 press thred setting */
	aw9203_i2c_write(aw9203, AW9203_REG_THR4, AW9203_NORMAL_THR4);

	/* debounce */
	aw9203_i2c_write(aw9203, AW9203_REG_SETCNT, AW9203_NORMAL_SETCNT);
	/* base trace rate */
	aw9203_i2c_write(aw9203, AW9203_REG_BLCTH, AW9203_NORMAL_BLCTH);

	/* aks */
	aw9203_i2c_write(aw9203, AW9203_REG_AKSR, AW9203_NORMAL_AKSR);
#ifndef AW_AUTO_CALI
	/* signel click interrupt */
	aw9203_i2c_write(aw9203, AW9203_REG_INTER, AW9203_NORMAL_INTER);
#else
	/* frame interrupt */
	aw9203_i2c_write(aw9203, AW9203_REG_INTER, 0x0280);
#endif

#if 0
	/* long press time */
	aw9203_i2c_write(aw9203, AW9203_REG_MPTR, AW9203_NORMAL_MPTR);
	/* gesture time setting */
	aw9203_i2c_write(aw9203, AW9203_REG_GDTR, AW9203_NORMAL_GDTR);
	/* gesture key select */
	aw9203_i2c_write(aw9203, AW9203_REG_GDCFGR, AW9203_NORMAL_GDCFGR);
	/* double click 1 */
	aw9203_i2c_write(aw9203, AW9203_REG_TAPR1, AW9203_NORMAL_TAPR1);
	/* double click 2 */
	aw9203_i2c_write(aw9203, AW9203_REG_TAPR2, AW9203_NORMAL_TAPR2);
	/* double click time */
	aw9203_i2c_write(aw9203, AW9203_REG_TDTR, AW9203_NORMAL_TDTR);
	/* IDLE time setting */
	aw9203_i2c_write(aw9203, AW9203_REG_IDLECR, AW9203_NORMAL_IDLECR);

	aw9203_i2c_write(aw9203, AW9203_REG_GESTR1, AW9203_NORMAL_0X23);
	aw9203_i2c_write(aw9203, AW9203_REG_GESTR2, AW9203_NORMAL_0X24);
#endif

#ifndef AW_AUTO_CALI
	/* gesture and double click enable */
	aw9203_i2c_write(aw9203, AW9203_REG_GIER, AW9203_NORMAL_GIER);
#else
	/* gesture and double click disable */
	aw9203_i2c_write(aw9203, AW9203_REG_GIER, 0x0000);
#endif

	/* chip enable */
	aw9203_i2c_read(aw9203, AW9203_REG_GCR, &reg_val);
	reg_val |= 0x0002;
	/* disable chip */
	aw9203_i2c_write(aw9203, AW9203_REG_GCR, reg_val);
}

/*****************************************************
*
* touch info store if touch info is valid.
* realize by a simple FIFO.
*
*****************************************************/
static void aw9203_touch_store(struct aw9203 *aw9203,
				struct aw_touch_info this_touch)
{

	memmove(&aw9203->touch_info[1], aw9203->touch_info,
		(AW_TOUCH_MAX - 1) * sizeof(struct aw_touch_info));
	memcpy(aw9203->touch_info, &this_touch,
				sizeof(struct aw_touch_info));
}
/*****************************************************
*
* slide init
*
*****************************************************/
static void aw9203_slide_reset(struct aw9203 *aw9203)
{
	memset(aw9203->key_status_last, 0, 3 * sizeof(uint8_t));
	memset(aw9203->key_status_now, 0, 3 * sizeof(uint8_t));
}
/******************************************************
 *
 * slide
 *
 ******************************************************/
 static void aw9203_report_event(struct aw9203 *aw9203, int key, int value)
{
	input_report_abs(aw9203->input, ABS_MT_TOOL_X, 0);
	input_report_abs(aw9203->input, ABS_MT_TOOL_Y, 0);
	input_report_abs(aw9203->input, ABS_MT_TRACKING_ID, 0);
	//key
	input_report_abs(aw9203->input, ABS_MT_WIDTH_MAJOR, key);
	//value
	input_report_abs(aw9203->input, ABS_MT_WIDTH_MINOR, value);

	input_report_abs(aw9203->input, ABS_MT_BLOB_ID, 0);
	input_report_abs(aw9203->input, ABS_MT_TOOL_TYPE, 0x146);
	input_report_abs(aw9203->input,  ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(aw9203->input, ABS_MT_TOUCH_MINOR, 0);

	input_sync(aw9203->input);
}

static void aw9203_slide_report_abs(struct aw9203 *aw9203, bool left)
{

	if (left == true) {
		if (aw9203->pos == 1) { //left
			aw9203_report_event(aw9203, KEY_CAP_L_SLIDE, 1);
			aw9203_report_event(aw9203, KEY_CAP_L_SLIDE, 0);
		} else {
			aw9203_report_event(aw9203, KEY_CAP_R_SLIDE_U, 1);
			aw9203_report_event(aw9203, KEY_CAP_R_SLIDE_U, 0);
		}
		pr_info("%s: left slide\n", __func__);
	} else {
		if (aw9203->pos == 1) { //left
			aw9203_report_event(aw9203, KEY_CAP_L_SLIDE_U, 1);
			aw9203_report_event(aw9203, KEY_CAP_L_SLIDE_U, 0);
		} else {
			aw9203_report_event(aw9203, KEY_CAP_R_SLIDE, 1);
			aw9203_report_event(aw9203, KEY_CAP_R_SLIDE, 0);
		}
		pr_info("%s: right slide\n", __func__);
	}
	aw9203_slide_reset(aw9203);
	return;
}

static void aw9203_slide(struct aw9203 *aw9203,unsigned int reg_kisr)
{
	uint8_t i = 0;
	uint32_t temp = 0;
	unsigned int status = 0;
	struct aw_touch_info this_touch;

	aw9203_i2c_read(aw9203, AW9203_REG_AKSST, &status);
	pr_debug("%s: key status=0x%02x\n",__func__, status);
	for (i = 0; i < 3; i++) {
		temp = (status >> (i+2));
		/* pr_info("%s: i = %d, temp = 0x%x\n", __func__, i, temp); */
		if ((temp & 0x01) == 0x01) {
			aw9203->key_status_now[i] = 1;
		}else{
			aw9203->key_status_now[i] = 0;
		}
		if (aw9203->key_status_now[i] == 1) {
				this_touch.time_pressed = ktime_get();
				switch (i) {
				case 0:
					if(reg_kisr & 0x04){
						pr_debug("%s: btn 1 press\n", __func__);
						this_touch.key = aw9203_key1;
						aw9203_touch_store(aw9203, this_touch);
						aw9203_slide_report(aw9203);
					}
					break;
				case 1:
					if(reg_kisr & 0x08){
						pr_debug("%s: btn 2 press\n", __func__);
						this_touch.key = aw9203_key2;
						aw9203_touch_store(aw9203, this_touch);
						aw9203_slide_report(aw9203);
					}
					break;
				case 2:
					if(reg_kisr & 0x10){
						pr_debug("%s: btn 3 press\n", __func__);
						this_touch.key = aw9203_key3;
						aw9203_touch_store(aw9203, this_touch);
						aw9203_slide_report(aw9203);
					}
					break;
				}
			}
			aw9203->key_status_last[i] =
				aw9203->key_status_now[i];
	}
}
/*****************************************************
*
* report if slide info is valid
*
*****************************************************/
static void aw9203_slide_report(struct aw9203 *aw9203)
{
	unsigned long int mir_sec = 0;
	int8_t i = 0;
	int32_t press_status = 0;

	mir_sec = ktime_to_ms(ktime_sub(aw9203->touch_info[0].time_pressed,
		aw9203->touch_info[AW_TOUCH_MAX - 1].time_pressed));

	pr_debug("%s: press_status = 0x%04x\n", __func__, press_status);
		if ((mir_sec < slide_time_max) && (mir_sec > slide_time_min)) {
		for (i = 0; i < AW_TOUCH_MAX; i++) {
			press_status |= aw9203->touch_info[i].key << i;
		}
		switch (press_status) {
		case ((aw9203_key3<<2) | (aw9203_key2<<1) | (aw9203_key1<<0)):
			/* 321 right_slide */
			aw9203_slide_report_abs(aw9203, false);
			break;
		case ((aw9203_key1<<2) | (aw9203_key2<<1) | (aw9203_key3<<0)):
			/* 123 left_slide */
			aw9203_slide_report_abs(aw9203, true);
			break;
		default:
			pr_debug("%s: not a slide, exit.\n", __func__);
			break;
		}
	}else {
		pr_debug("%s: time = %ld, over time max = %ld",
			__func__, mir_sec, slide_time_max);

		if(aw9203_key1 | aw9203_key2 | aw9203_key3){
			if (aw9203->pos == 1) { //left
				aw9203_report_event(aw9203, KEY_CAP_L1, 1);
				aw9203_report_event(aw9203, KEY_CAP_L1, 0);
			} else {
				aw9203_report_event(aw9203, KEY_CAP_R1, 1);
				aw9203_report_event(aw9203, KEY_CAP_R1, 0);
			}
			pr_debug("%s: single click\n", __func__);
		}else{
			pr_debug("%s: no single click\n", __func__);
		}
	}
}
/*
static void  aw9203_leave_detection(struct aw9203 *aw9203)
{
	uint32_t ret = 0;
	uint32_t status = 0;
	int8_t i = 0;

	pr_info("%s: enter\n", __func__);

	aw9203_i2c_read(aw9203, AW9203_REG_AKSST, &ret);
	status = (uint8_t)(ret&0x1c >> 2);
	for (i = 0; i < 3; i++) {
		if ((((status >> i) & 0x01) != 0x01) &&
					(aw9203->key_status_now[i] == 1)) {
			aw9203->key_status_last[i] = 0;
			pr_info("%s:aw9203->key_status_now[%d] is reset\n",
								__func__, i);
		}
	}
}
*/
/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw9203_interrupt_clear(struct aw9203 *aw9203)
{
	unsigned int reg_val;
	pr_debug("%s enter\n", __func__);
	aw9203_i2c_read(aw9203, AW9203_REG_ISR, &reg_val);
	pr_debug("%s: reg ISR=0x%x\n", __func__, reg_val);
}

static void aw9203_interrupt_setup(struct aw9203 *aw9203)
{
	pr_debug("%s enter\n", __func__);
	aw9203_interrupt_clear(aw9203);
}

static irqreturn_t aw9203_irq(int irq, void *data)
{
	struct aw9203 *aw9203 = data;
	unsigned int reg_kisr;
	unsigned int reg_slid;

	pr_debug("%s enter\n", __func__);
	if (aw9203_is_suspend == true) {
		pr_info("%s : suspend, do noting.\n", __func__);
		return IRQ_HANDLED;
	}
#ifdef AW_AUTO_CALI
	if (aw9203->cali_flag) {
		aw9203_auto_cali(aw9203);
		if (aw9203->cali_flag == 0) {
			if (aw9203->cali_used) {
				aw9203_i2c_read(aw9203, AW9203_REG_GCR,
						&reg_val);
				reg_val &= 0xFFFD;
				aw9203_i2c_write(aw9203, AW9203_REG_GCR,
						 reg_val);
			}
			aw9203_i2c_write(aw9203, AW9203_REG_INTER,
					 AW9203_NORMAL_INTER);
			aw9203_i2c_write(aw9203, AW9203_REG_GIER,
					 AW9203_NORMAL_GIER);
			if (aw9203->cali_used) {
				aw9203_i2c_read(aw9203, AW9203_REG_GCR,
						&reg_val);
				reg_val |= 0x0002;
				aw9203_i2c_write(aw9203, AW9203_REG_GCR,
						 reg_val);
			}
		}
		aw9203_interrupt_clear(aw9203);
		return IRQ_HANDLED;
	}
#endif

	aw9203_i2c_read(aw9203, AW9203_REG_ISR, &reg_kisr);
	aw9203_i2c_read(aw9203, AW9203_REG_GISR, &reg_slid);
	pr_debug("%s: kisr=0x%04x, reg_slid = 0x%04x\n", __func__, reg_kisr, reg_slid);

	if(reg_kisr & 0x1c){
		aw9203_slide(aw9203,reg_kisr);
	}
	aw9203_interrupt_clear(aw9203);
	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw9203_parse_dt(struct device *dev, struct aw9203 *aw9203,
			   struct device_node *np)
{
	aw9203->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw9203->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: reset gpio provided ok.\n", __func__);
	}

	aw9203->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw9203->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

	return 0;
}

int aw9203_hw_reset(struct aw9203 *aw9203)
{
	pr_info("%s enter, pos = %d\n", __func__, aw9203->pos);
	if ( is_reset == false ) {
		pr_info("aw9203 hw reset once.\n");
		return 0;
	}

	if (aw9203 && gpio_is_valid(aw9203->reset_gpio)) {
		gpio_set_value_cansleep(aw9203->reset_gpio, 0);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(aw9203->reset_gpio, 1);
		usleep_range(5000, 5500);
	} else {
		dev_err(aw9203->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

int aw9203_hw_off(struct aw9203 *aw9203)
{
	pr_info("%s enter, pos = %d\n", __func__, aw9203->pos);

	if (aw9203 && gpio_is_valid(aw9203->reset_gpio)) {
		gpio_set_value_cansleep(aw9203->reset_gpio, 0);
		usleep_range(2000, 2500);
	} else {
		dev_err(aw9203->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
int aw9203_read_chipid(struct aw9203 *aw9203)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned int reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw9203_i2c_read(aw9203, AW9203_REG_RSTR, &reg_val);
		if (ret < 0) {
			dev_err(aw9203->dev,
				"%s: failed to read register AW9203_REG_RSTR: %d\n",
				__func__, ret);
		}
		switch (reg_val) {
		case 0xb223:
			pr_info("%s aw9203 detected\n", __func__);
			/* aw9203->flags |= AW9203_FLAG_SKIP_INTERRUPTS; */
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

static int aw9203_i2c_pinctrl_init(struct device *dev, struct aw9203 *aw9203)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	aw9203->aw9203_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(aw9203->aw9203_pinctrl)) {
		retval = PTR_ERR(aw9203->aw9203_pinctrl);
		pr_info("aw9203 Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	aw9203->pinctrl_state_active
		= pinctrl_lookup_state(aw9203->aw9203_pinctrl, "pmx_cap_touch_active");
	if (IS_ERR_OR_NULL(aw9203->pinctrl_state_active)) {
		retval = PTR_ERR(aw9203->pinctrl_state_active);
		pr_info("aw9203 Can not lookup pmx_cap_touch_active pinstate %d\n",
			retval);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(aw9203->aw9203_pinctrl);
err_pinctrl_get:
	aw9203->aw9203_pinctrl = NULL;
	return retval;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw9203_i2c_reg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);

	unsigned int databuf[2] = { 0, 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw9203_i2c_write(aw9203, (unsigned char)databuf[0],
				 (unsigned int)databuf[1]);
	}

	return count;
}

static ssize_t aw9203_i2c_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned int reg_val = 0;
	for (i = 0; i < AW9203_REG_MAX; i++) {
		if (!(aw9203_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw9203_i2c_read(aw9203, i, &reg_val);
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%04x \n",
			     i, reg_val);
	}
	return len;
}

static ssize_t aw9203_i2c_rawdata_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/* struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_rawdata_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0003);
	len += snprintf(buf + len, PAGE_SIZE - len, "rawdata:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_base_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	/* struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_base_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0002);
	len += snprintf(buf + len, PAGE_SIZE - len, "baseline:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_delta_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	/* struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_delta_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned char i;

	mutex_lock(&aw9203->lock);
	aw9203_i2c_write(aw9203, AW9203_REG_MCR, 0x0001);
	len += snprintf(buf + len, PAGE_SIZE - len, "delta:\n");
	for (i = 0; i < 6; i++) {
		aw9203_i2c_read(aw9203, AW9203_REG_KDATA0 + i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	mutex_unlock(&aw9203->lock);

	return len;
}

static ssize_t aw9203_i2c_irqstate_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	/* struct aw9203 *aw9203 = dev_get_drvdata(dev); */
	return count;
}

static ssize_t aw9203_i2c_irqstate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw9203 *aw9203 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;
	unsigned int tmp_val = 0;
	unsigned char i;

	len += snprintf(buf + len, PAGE_SIZE - len, "touch:\n");
	aw9203_i2c_read(aw9203, AW9203_REG_AKSST, &reg_val);
	for (i = 0; i < 6; i++) {
		tmp_val = (reg_val >> i) & 0x01;
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", tmp_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "gesture:\n");
	aw9203_i2c_read(aw9203, AW9203_REG_GISR, &reg_val);
	for (i = 0; i < 5; i++) {
		tmp_val = (reg_val >> i) & 0x01;
		len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", tmp_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t aw9203_i2c_slide_time_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int databuf[2] = { 0, 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		slide_time_min = databuf[0];
		slide_time_max = databuf[1];
	}

	return count;
}

static ssize_t aw9203_i2c_slide_time_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
				"min_slide_interval_time = %d, max_slide_interval_time = %d\n",
			     slide_time_min, slide_time_max);
	return len;
}

static ssize_t aw9203_cap_l_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int databuf = 0;
	struct aw9203 *aw9203 = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &databuf) == 1) {
		if (databuf == 1) {
			aw9203->cap_enable=true;
			aw9203_sleep_mode(aw9203, false);
		} else if (databuf == 0) {
			aw9203->cap_enable=false;
			aw9203_sleep_mode(aw9203, true);
		} else {
			pr_info("aw9203: The currently entered mode is not supported!\n");
		}
	}

	return count;
}

static ssize_t aw9203_cap_r_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int databuf = 0;
	struct aw9203 *aw9203 = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &databuf) == 1) {
		if (databuf == 1) {
			aw9203->cap_enable=true;
			aw9203_sleep_mode(aw9203, false);
		} else if (databuf == 0) {
			aw9203->cap_enable=false;
			aw9203_sleep_mode(aw9203, true);
		} else {
			pr_info("aw9203: The currently entered mode is not supported!\n");
		}
	}

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw9203_i2c_reg_show,
		   aw9203_i2c_reg_store);
static DEVICE_ATTR(rawdata, S_IWUSR | S_IRUGO, aw9203_i2c_rawdata_show,
		   aw9203_i2c_rawdata_store);
static DEVICE_ATTR(base, S_IWUSR | S_IRUGO, aw9203_i2c_base_show,
		   aw9203_i2c_base_store);
static DEVICE_ATTR(delta, S_IWUSR | S_IRUGO, aw9203_i2c_delta_show,
		   aw9203_i2c_delta_store);
static DEVICE_ATTR(irqstate, S_IWUSR | S_IRUGO, aw9203_i2c_irqstate_show,
		   aw9203_i2c_irqstate_store);
static DEVICE_ATTR(slide_time, S_IWUSR | S_IRUGO, aw9203_i2c_slide_time_show,
		   aw9203_i2c_slide_time_store);
static DEVICE_ATTR(cap_l_enable, S_IWUSR | S_IRUGO, NULL,
		   aw9203_cap_l_enable_store);
static DEVICE_ATTR(cap_r_enable, S_IWUSR | S_IRUGO, NULL,
		   aw9203_cap_r_enable_store);

static struct attribute *aw9203_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_base.attr,
	&dev_attr_delta.attr,
	&dev_attr_irqstate.attr,
	&dev_attr_slide_time.attr,
	&dev_attr_cap_r_enable.attr,
	&dev_attr_cap_l_enable.attr,
	NULL
};

static struct attribute_group aw9203_attribute_group = {
	.attrs = aw9203_attributes
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw9203_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aw9203 *aw9203;
	struct device_node *np = i2c->dev.of_node;
	struct input_dev *input_dev;
	int irq_flags;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw9203 = devm_kzalloc(&i2c->dev, sizeof(struct aw9203), GFP_KERNEL);
	if (aw9203 == NULL)
		return -ENOMEM;

	aw9203->dev = &i2c->dev;
	aw9203->i2c = i2c;
	aw9203->pos = 0;
	if (i2c->addr == 0x2c)
		aw9203->pos = 1; /* left */
	pr_info("%s positon: %d\n", __func__, aw9203->pos);

	i2c_set_clientdata(i2c, aw9203);

	mutex_init(&aw9203->lock);

	/* aw9203 int */
	if (np) {
		ret = aw9203_parse_dt(&i2c->dev, aw9203, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err;
		}
	} else {
		aw9203->reset_gpio = -1;
		aw9203->irq_gpio = -1;
	}

    ret = aw9203_i2c_pinctrl_init(&i2c->dev, aw9203);
    if (!ret && aw9203->aw9203_pinctrl) {
	/*
	* Pinctrl handle is optional. If pinctrl handle is found
	* let pins to be configured in active state. If not
	* found continue further without error.
	*/
	ret = pinctrl_select_state(aw9203->aw9203_pinctrl,
			aw9203->pinctrl_state_active);
	if (ret < 0) {
		pr_info("aw9203: Failed to select %s pinstate %d\n",
					__func__, ret);
	}
    }

	if (gpio_is_valid(aw9203->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9203->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw9203_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			//goto err;
		}
	}

	if (gpio_is_valid(aw9203->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9203->irq_gpio,
					    GPIOF_DIR_IN, "aw9203_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err;
		}
	}

	/* aw9203 hardware reset */
	aw9203_hw_reset(aw9203);
	is_reset = false;
	/* aw9203 chip id */
	ret = aw9203_read_chipid(aw9203);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw9203_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw9203 irq */
	if (gpio_is_valid(aw9203->irq_gpio) &&
	    !(aw9203->flags & AW9203_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw9203_interrupt_setup(aw9203);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw9203->irq_gpio),
						NULL, aw9203_irq, irq_flags,
						"aw9203", aw9203);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw9203->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw9203->flags |= AW9203_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw9203);

	/* input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(KEY_CAP_L1, input_dev->keybit);
	__set_bit(KEY_CAP_L2, input_dev->keybit);
	__set_bit(KEY_CAP_L3, input_dev->keybit);
	__set_bit(KEY_CAP_R1, input_dev->keybit);
	__set_bit(KEY_CAP_R2, input_dev->keybit);
	__set_bit(KEY_CAP_R3, input_dev->keybit);
	__set_bit(KEY_CAP_L_SLIDE, input_dev->keybit);
	__set_bit(KEY_CAP_R_SLIDE, input_dev->keybit);
	__set_bit(KEY_CAP_L_SLIDE_U, input_dev->keybit);
	__set_bit(KEY_CAP_R_SLIDE_U, input_dev->keybit);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 0xFF, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, 1023, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 1023, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 1023, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_BLOB_ID, 0, 65535, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_X, 0, 1080, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_Y, 0, 2460, 0, 0);

    if ( i2c->addr == 0x2c )
        input_dev->name = AW9203_I2C_NAME;
    else
        input_dev->name = AW9203_I2C_NAME_L;
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&i2c->dev,
			"%s: failed to register input device: %s\n",
			__func__, dev_name(&i2c->dev));
		goto exit_input_register_device_failed;
	}
	aw9203->input = input_dev;

	/* attribute */
	ret = sysfs_create_group(&i2c->dev.kobj, &aw9203_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	aw9203_normal_mode_init(aw9203);
	aw9203_sleep_mode(aw9203, true);
	aw9203->cap_enable=false;

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

 err_sysfs:
	input_unregister_device(input_dev);
 exit_input_register_device_failed:
	input_free_device(input_dev);
 exit_input_dev_alloc_failed:
 err_irq:
 err_id:
	if (gpio_is_valid(aw9203->irq_gpio))
		devm_gpio_free(&i2c->dev, aw9203->irq_gpio);
 err:
	return ret;
}

static int aw9203_i2c_remove(struct i2c_client *i2c)
{
	struct aw9203 *aw9203 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	mutex_destroy(&aw9203->lock);

	sysfs_remove_group(&i2c->dev.kobj, &aw9203_attribute_group);

	if (gpio_is_valid(aw9203->irq_gpio))
		devm_gpio_free(&i2c->dev, aw9203->irq_gpio);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int aw9203_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw9203 *aw9203 = i2c_get_clientdata(client);

	aw9203_is_suspend = true;
	aw9203_hw_off(aw9203);
	is_reset = true;
	return 0;
}

static int aw9203_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw9203 *aw9203 = i2c_get_clientdata(client);

	aw9203_hw_reset(aw9203);
	is_reset = false;
	aw9203_normal_mode_init(aw9203);
	if (aw9203->cap_enable==false)
		aw9203_sleep_mode(aw9203, true);
	aw9203_is_suspend = false;

	return 0;
}

static SIMPLE_DEV_PM_OPS(aw9203_pm, aw9203_suspend, aw9203_resume);
#endif

static const struct i2c_device_id aw9203_i2c_id[] = {
    { AW9203_I2C_NAME, 0 },
    { AW9203_I2C_NAME_L, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, aw9203_i2c_id);

static struct of_device_id aw9203_dt_match[] = {
    { .compatible = "awinic,aw9203_ts"},
    { .compatible = "awinic,aw9203_ts_1"},
   { },
};

static struct i2c_driver aw9203_i2c_driver = {
	.driver = {
		   .name = AW9203_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw9203_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw9203_pm,
#endif
		   },
	.probe = aw9203_i2c_probe,
	.remove = aw9203_i2c_remove,
	.id_table = aw9203_i2c_id,
};

static int __init aw9203_i2c_init(void)
{
	int ret = 0;

	pr_info("aw9203 driver version %s\n", AW9203_DRIVER_VERSION);

	ret = i2c_add_driver(&aw9203_i2c_driver);
	if (ret) {
		pr_err("fail to add aw9203 device into i2c\n");
		return ret;
	}

	return 0;
}

module_init(aw9203_i2c_init);

static void __exit aw9203_i2c_exit(void)
{
	i2c_del_driver(&aw9203_i2c_driver);
}

module_exit(aw9203_i2c_exit);

MODULE_DESCRIPTION("AW9203 Touch Key Driver");
MODULE_LICENSE("GPL v2");
