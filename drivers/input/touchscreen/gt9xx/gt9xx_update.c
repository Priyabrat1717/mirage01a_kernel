/* drivers/input/touchscreen/gt9xx_update.c
 *
 * 2010 - 2012 Goodix Technology.
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Latest Version:1.6
 * Author: andrew@goodix.com
 * Revision Record:
 *      V1.0:
 *          first release. By Andrew, 2012/08/31
 *      V1.2:
 *          add force update,GT9110P pid map. By Andrew, 2012/10/15
 *      V1.4:
 *          1. add config auto update function;
 *          2. modify enter_update_mode;
 *          3. add update file cal checksum.
 *                          By Andrew, 2012/12/12
 *      V1.6:
 *          1. replace guitar_client with i2c_connect_client;
 *          2. support firmware header array update.
 *                          By Meta, 2013/03/11
 */
#include "gt9xx.h"
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include "gt9xx_firmware.h"
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/kthread.h>
#define FIRMWARE_NAME_LEN_MAX		256

#define GUP_REG_HW_INFO             0x4220
#define GUP_REG_FW_MSG              0x41E4
#define GUP_REG_PID_VID             0x8140

#define GOODIX_FIRMWARE_FILE_NAME	"_goodix_update_.bin"
#define GOODIX_CONFIG_FILE_NAME		"_goodix_config_.cfg"

#define FW_HEAD_LENGTH               14
#define FW_SECTION_LENGTH            0x2000
#define FW_DSP_ISP_LENGTH            0x1000
#define FW_DSP_LENGTH                0x1000
#define FW_BOOT_LENGTH               0x800
#define FW_SS51_LENGTH               (4 * FW_SECTION_LENGTH)	/* 32K */
#define FW_BOOT_ISP_LENGTH           0x800	/* 2k */
#define FW_GLINK_LENGTH              0x3000	/* 12k */
#define FW_GWAKE_LENGTH              (4 * FW_SECTION_LENGTH)	/* 32k */

#define PACK_SIZE                    256
#define MAX_FRAME_CHECK_TIME         5

#define _bRW_MISCTL__SRAM_BANK       0x4048
#define _bRW_MISCTL__MEM_CD_EN       0x4049
#define _bRW_MISCTL__CACHE_EN        0x404B
#define _bRW_MISCTL__TMR0_EN         0x40B0
#define _rRW_MISCTL__SWRST_B0_       0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE 0x4184
#define _rRW_MISCTL__BOOTCTL_B0_     0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_    0x4218
#define _rRW_MISCTL__BOOT_CTL_       0x5094

#define AUTO_SEARCH_BIN              0x01
#define AUTO_SEARCH_CFG              0x02
#define BIN_FILE_READY               0x80
#define CFG_FILE_READY               0x08
#define HEADER_FW_READY              0x00

#define FAIL    0
#define SUCCESS 1

struct st_fw_head {
	u8  hw_info[4];		/* hardware info */
	u8  pid[8];		/* product id   */
	u16 vid;		/* version id   */
} __packed;

struct st_update_msg {
	u8 force_update;
	u8 fw_flag;
	struct file *file;
	struct file *cfg_file;
	bool need_free;
	u8 *fw_data;
	u32 fw_len;
	u32 fw_total_len;
	u32 fw_burned_len;
	struct st_fw_head  ic_fw_msg;
	mm_segment_t old_fs;
};

static struct st_update_msg update_msg;
u16 show_len;
u16 total_len;
u8 got_file_flag;
u8 searching_file;

static u8 gup_burn_fw_gwake_section(struct i2c_client *client, u8 *fw_section,
				    u16 start_addr, u32 len, u8 bank_cmd);
/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
static s32 gup_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u8 retries = 0;
	struct i2c_msg msgs[2] = {
		{
			.flags = !I2C_M_RD,
			.addr  = client->addr,
			.len   = GTP_ADDR_LENGTH,
			.buf   = &buf[0],
		},
		{
			.flags = I2C_M_RD,
			.addr  = client->addr,
			.len   = len - GTP_ADDR_LENGTH,
			.buf   = &buf[GTP_ADDR_LENGTH],
		},
	};

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (retries == 5) {
		dev_err(&client->dev, "I2C read retry limit over.\n");
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
	1: succeed, otherwise: failed
*********************************************************/
s32 gup_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u8 retries = 0;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.addr  = client->addr,
		.len   = len,
		.buf   = buf,
	};

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (retries == 5) {
		dev_err(&client->dev, "I2C write retry limit over.\n");
		ret = -EIO;
	}

	return ret;
}

static s32 gup_init_panel(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	u8 *config_data;
	s32 ret = 0;
	s32 i = 0;
	u8 check_sum = 0;
	u8 opr_buf[16];
	u8 sensor_id = 0;

	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++)
		if (ts->pdata->config_data_len[i])
			break;

	if (i == GOODIX_MAX_CFG_GROUP) {
		sensor_id = 0;
	} else {
		ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID,
							&sensor_id, 1);
		if (SUCCESS == ret) {
			if (sensor_id >= GOODIX_MAX_CFG_GROUP) {
				pr_err("Invalid sensor_id(0x%02X), No Config Sent!",
					sensor_id);
				return -EINVAL;
			}
		} else {
			pr_err("Failed to get sensor_id, No config sent!");
			return -EINVAL;
		}
	}

	pr_info("Sensor ID selected: %d", sensor_id);

	if (ts->pdata->config_data_len[sensor_id] < GTP_CONFIG_MIN_LENGTH ||
		!ts->pdata->config_data_len[sensor_id]) {
		pr_err("Sensor_ID(%d) matches with NULL or INVALID CONFIG GROUP!",
				sensor_id);
		return -EINVAL;
	}

	ret = gtp_i2c_read_dbl_check(client, GTP_REG_CONFIG_DATA,
					&opr_buf[0], 1);
	if (ret == SUCCESS) {
		pr_info("CFG_GROUP%d Config Version: %d, IC Config Version: %d",
			sensor_id + 1,
			ts->pdata->config_data[sensor_id][0],
			opr_buf[0]);

		ts->pdata->config_data[sensor_id][0] = opr_buf[0];
		ts->fixed_cfg = 0;
	} else {
		pr_err("Failed to get ic config version!No config sent!");
		return -EINVAL;
	}

	config_data = ts->pdata->config_data[sensor_id];
	ts->config_data = ts->pdata->config_data[sensor_id];
	ts->gtp_cfg_len = ts->pdata->config_data_len[sensor_id];

	pr_info("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
	ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);

	config_data[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
	config_data[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
	config_data[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
	config_data[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

	if (GTP_INT_TRIGGER == 0)  /* RISING */
		config_data[TRIGGER_LOC] &= 0xfe;
	else if (GTP_INT_TRIGGER == 1)  /* FALLING */
		config_data[TRIGGER_LOC] |= 0x01;

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
		check_sum += config_data[i];

	config_data[ts->gtp_cfg_len] = (~check_sum) + 1;

	ret = gtp_send_cfg(ts);
	if (ret < 0)
		pr_err("Send config error.");

	ts->config_data = NULL;
	ts->gtp_cfg_len = 0;
	msleep(20);
	return 0;
}

static u8 gup_get_ic_msg(struct i2c_client *client, u16 addr, u8 *msg, s32 len)
{
	u8 i = 0;

	msg[0] = (addr >> 8) & 0xff;
	msg[1] = addr & 0xff;

	for (i = 0; i < 5; i++)
		if (gup_i2c_read(client, msg, GTP_ADDR_LENGTH + len) > 0)
			break;

	if (i >= 5) {
		pr_err("Read data from 0x%02x%02x failed!", msg[0], msg[1]);
		return FAIL;
	}

	return SUCCESS;
}

static u8 gup_set_ic_msg(struct i2c_client *client, u16 addr, u8 val)
{
	u8 i = 0;
	u8 msg[3] = {
		(addr >> 8) & 0xff,
		addr & 0xff,
		val,
	};

	for (i = 0; i < 5; i++)
		if (gup_i2c_write(client, msg, GTP_ADDR_LENGTH + 1) > 0)
			break;

	if (i >= 5) {
		pr_err("Set data to 0x%02x%02x failed!", msg[0], msg[1]);
		return FAIL;
	}

	return SUCCESS;
}

static u8 gup_get_ic_fw_msg(struct i2c_client *client)
{
	s32 ret = -1;
	u8  retry = 0;
	u8  buf[16];
	u8  i;

	/* step1:get hardware info */
	ret = gtp_i2c_read_dbl_check(client, GUP_REG_HW_INFO,
					&buf[GTP_ADDR_LENGTH], 4);
	if (ret == FAIL) {
		pr_err("get hw_info failed,exit");
		return FAIL;
	}

	/*  buf[2~5]: 00 06 90 00 */
	/* hw_info: 00 90 06 00 */
	for (i = 0; i < 4; i++)
		update_msg.ic_fw_msg.hw_info[i] = buf[GTP_ADDR_LENGTH + 3 - i];

	pr_info("IC Hardware info:%02x%02x%02x%02x",
		update_msg.ic_fw_msg.hw_info[0],
		update_msg.ic_fw_msg.hw_info[1],
		update_msg.ic_fw_msg.hw_info[2],
		update_msg.ic_fw_msg.hw_info[3]);

	/* step2:get firmware message */
	for (retry = 0; retry < 2; retry++) {
		ret = gup_get_ic_msg(client, GUP_REG_FW_MSG, buf, 1);
		if (ret == FAIL) {
			pr_err("Read firmware message fail.");
			return ret;
		}

		update_msg.force_update = buf[GTP_ADDR_LENGTH];
		if ((0xBE != update_msg.force_update) && (!retry)) {
			pr_info("The check sum in ic is error.");
			pr_info("The IC will be updated by force.");
			continue;
		}
		break;
	}
	pr_info("IC force update flag:0x%x", update_msg.force_update);

	/*  step3:get pid & vid */
	ret = gtp_i2c_read_dbl_check(client, GUP_REG_PID_VID,
						&buf[GTP_ADDR_LENGTH], 6);
	if (ret == FAIL) {
		pr_err("get pid & vid failed,exit");
		return FAIL;
	}

	memset(update_msg.ic_fw_msg.pid, 0, sizeof(update_msg.ic_fw_msg.pid));
	memcpy(update_msg.ic_fw_msg.pid, &buf[GTP_ADDR_LENGTH], 4);
	pr_info("IC Product id:%s", update_msg.ic_fw_msg.pid);

	/* GT9XX PID MAPPING
	|-----FLASH-----RAM-----|
	|------918------918-----|
	|------968------968-----|
	|------913------913-----|
	|------913P-----913P----|
	|------927------927-----|
	|------927P-----927P----|
	|------9110-----9110----|
	|------9110P----9111----|*/
	if (update_msg.ic_fw_msg.pid[0] != 0) {
		if (!memcmp(update_msg.ic_fw_msg.pid, "9111", 4)) {
			pr_info("IC Mapping Product id:%s",
					update_msg.ic_fw_msg.pid);
			memcpy(update_msg.ic_fw_msg.pid, "9110P", 5);
		}
	}

	update_msg.ic_fw_msg.vid = buf[GTP_ADDR_LENGTH + 4] +
				(buf[GTP_ADDR_LENGTH + 5] << 8);
	pr_info("IC version id:%04x", update_msg.ic_fw_msg.vid);

	return SUCCESS;
}

s32 gup_enter_update_mode(struct i2c_client *client)
{
	s32 ret = -1;
	u8 retry = 0;
	u8 rd_buf[3];
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	/* step1:RST output low last at least 2ms */
	gpio_direction_output(ts->pdata->reset_gpio, 0);
	msleep(2);

	/* step2:select I2C slave addr,INT:0--0xBA;1--0x28. */
	gpio_direction_output(ts->pdata->irq_gpio,
			(client->addr == GTP_I2C_ADDRESS_HIGH));
	msleep(2);

	/* step3:RST output high reset guitar */
	gpio_direction_output(ts->pdata->reset_gpio, 1);

	/* 20121211 modify start */
	msleep(5);
	while (retry++ < 200) {
		/* step4:Hold ss51 & dsp */
		ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
		if (ret <= 0) {
			pr_info("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}

		/* step5:Confirm hold */
		ret = gup_get_ic_msg(client, _rRW_MISCTL__SWRST_B0_, rd_buf, 1);
		if (ret <= 0) {
			pr_info("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}
		if (rd_buf[GTP_ADDR_LENGTH] == 0x0C) {
			pr_info("Hold ss51 & dsp confirm SUCCESS");
			break;
		}
		pr_info("Hold ss51 & dsp confirm 0x4180 failed,value:%d",
					rd_buf[GTP_ADDR_LENGTH]);
	}
	if (retry >= 200) {
		pr_err("Enter update Hold ss51 failed.");
		return FAIL;
	}

	/* step6:DSP_CK and DSP_ALU_CK PowerOn */
	ret = gup_set_ic_msg(client, 0x4010, 0x00);

	/* 20121211 modify end */
	return ret;
}

void gup_leave_update_mode(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	gpio_direction_input(ts->pdata->irq_gpio);
	pr_info("reset chip.");
	gtp_reset_guitar(ts, 20);
}

/*	Get the correct nvram data
	The correct conditions:
	1. the hardware info is the same
	2. the product id is the same
	3. the firmware version in update file is greater than the firmware
	version in ic or the check sum in ic is wrong

	Update Conditions:
	1. Same hardware info
	2. Same PID
	3. File PID > IC PID

	Force Update Conditions:
	1. Wrong ic firmware checksum
	2. INVALID IC PID or VID
	3. IC PID == 91XX || File PID == 91XX
*/

static u8 gup_enter_update_judge(struct st_fw_head *fw_head)
{
	u16 u16_tmp;
	s32 i = 0;
        u32 fw_len = 0;
	u16_tmp = fw_head->vid;
	fw_head->vid = (u16)(u16_tmp>>8) + (u16)(u16_tmp<<8);

	pr_info("FILE HARDWARE INFO:%02x%02x%02x%02x", fw_head->hw_info[0],
		fw_head->hw_info[1], fw_head->hw_info[2], fw_head->hw_info[3]);
	pr_info("FILE PID:%s", fw_head->pid);
	pr_info("FILE VID:%04x", fw_head->vid);

	pr_info("IC HARDWARE INFO:%02x%02x%02x%02x",
		update_msg.ic_fw_msg.hw_info[0],
		update_msg.ic_fw_msg.hw_info[1],
		update_msg.ic_fw_msg.hw_info[2],
		update_msg.ic_fw_msg.hw_info[3]);
	pr_info("IC PID:%s", update_msg.ic_fw_msg.pid);
	pr_info("IC VID:%04x", update_msg.ic_fw_msg.vid);

	if (!memcmp(fw_head->pid, "9158", 4)
	    && !memcmp(update_msg.ic_fw_msg.pid, "915S", 4)) {
		pr_info("Update GT915S to GT9158 directly!");
		return SUCCESS;
	}

	/* First two conditions */
	if (!memcmp(fw_head->hw_info, update_msg.ic_fw_msg.hw_info,sizeof(update_msg.ic_fw_msg.hw_info))){
		fw_len = 42 * 1024;}
	else {
		fw_len = fw_head->hw_info[3];
		fw_len += (((u32)fw_head->hw_info[2]) << 8);
		fw_len += (((u32)fw_head->hw_info[1]) << 16);
		fw_len += (((u32)fw_head->hw_info[0]) << 24);
	}
	if (update_msg.fw_total_len != fw_len) {
		pr_err
		    ("size error, Default size: %d(%dK), actual size: %d(%dK)",
		     fw_len, fw_len / 1024, update_msg.fw_total_len,
		     update_msg.fw_total_len / 1024);
		return FAIL;
	}
	pr_info("Firmware length:%d(%dK)", update_msg.fw_total_len,
		 update_msg.fw_total_len / 1024);

		if (update_msg.force_update != 0xBE) {
			pr_info("FW chksum error,need enter update.");
			return SUCCESS;
		}

		/* 20130523 start */
		if (strlen(update_msg.ic_fw_msg.pid) < 3) {
			pr_info("Illegal IC pid, need enter update");
			return SUCCESS;
		} else {
			for (i = 0; i < 3; i++) {
				if ((update_msg.ic_fw_msg.pid[i] < 0x30) ||
					(update_msg.ic_fw_msg.pid[i] > 0x39)) {
					pr_info("Illegal IC pid, out of bound, need enter update");
					return SUCCESS;
				}
			}
		}
		/* 20130523 end */

		if ((!memcmp(fw_head->pid, update_msg.ic_fw_msg.pid,
		(strlen(fw_head->pid) < 3 ? 3 : strlen(fw_head->pid)))) ||
		(!memcmp(update_msg.ic_fw_msg.pid, "91XX", 4)) ||
		(!memcmp(fw_head->pid, "91XX", 4))) {
			if (!memcmp(fw_head->pid, "91XX", 4))
				pr_info("Force none same pid update mode.");
			else
				pr_info("Get the same pid.");

			/* The third condition */
			if (fw_head->vid > update_msg.ic_fw_msg.vid) {
				pr_info("Need enter update.");
				return SUCCESS;
			}
			pr_err("Don't meet the third condition.");
			pr_err("File VID <= Ic VID, update aborted!");
		} else {
			pr_err("File PID != Ic PID, update aborted!");
		}

	return FAIL;
}

/*static s8 gup_update_config(struct i2c_client *client,
					const struct firmware *cfg)
{
	s32 ret = 0;
	s32 i = 0;
	s32 file_cfg_len = 0;
	u32 chip_cfg_len = 0;
	s32 count = 0;
	u8 *buf;
	u8 *file_config;
	u8 pid[8];
	u8 high, low;

	if (!cfg || !cfg->data) {
		pr_err("No need to upgrade config!");
		return FAIL;
	}

	ret = gup_get_ic_msg(client, GUP_REG_PID_VID, pid, 6);
	if (ret == FAIL) {
		pr_err("Read product id & version id fail.");
		return FAIL;
	}
	pid[5] = '\0';
	pr_info("update cfg get pid:%s", &pid[GTP_ADDR_LENGTH]);

	chip_cfg_len = 186;
	if (!memcmp(&pid[GTP_ADDR_LENGTH], "968", 3) ||
		!memcmp(&pid[GTP_ADDR_LENGTH], "910", 3) ||
		!memcmp(&pid[GTP_ADDR_LENGTH], "960", 3)) {
		chip_cfg_len = 228;
	}
	pr_info("config file ASCII len:%ld", cfg->size);
	pr_info("need config binary len:%d", chip_cfg_len);
	if ((cfg->size + 5) < chip_cfg_len * 5) {
		pr_err("Config length error");
		return -EINVAL;
	}

	buf = devm_kzalloc(&client->dev, cfg->size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Memory allocation failed for buf.");
		return -ENOMEM;
	}

	file_config = devm_kzalloc(&client->dev, chip_cfg_len + GTP_ADDR_LENGTH,
								GFP_KERNEL);
	if (!file_config) {
		dev_err(&client->dev, "Memory allocation failed.");
		return -ENOMEM;
	}

	pr_info("Delete illgal charactor.");
	for (i = 0, count = 0; i < cfg->size; i++) {
		if (cfg->data[i] == ' ' || cfg->data[i] == '\r'
					|| cfg->data[i] == '\n')
			continue;
		buf[count++] = cfg->data[i];
	}

	pr_info("Ascii to hex.");
	file_config[0] = GTP_REG_CONFIG_DATA >> 8;
	file_config[1] = GTP_REG_CONFIG_DATA & 0xff;
	for (i = 0, file_cfg_len = GTP_ADDR_LENGTH; i < count; i = i + 5) {
		if ((buf[i] == '0') && ((buf[i + 1] == 'x') ||
						(buf[i + 1] == 'X'))) {
			ret = hex2bin(&high, &buf[i + 2], 1);
			if (ret) {
				pr_err("Failed to convert high address from hex2bin");
				return ret;
			}
			ret = hex2bin(&low, &buf[i + 3], 1);
			if (ret) {
				pr_err("Failed to convert low address from hex2bin");
				return ret;
			}

			if ((high == 0xFF) || (low == 0xFF)) {
				ret = 0;
				pr_err("Illegal config file.");
				return ret;
			}
			file_config[file_cfg_len++] = (high<<4) + low;
		} else {
			ret = 0;
			pr_err("Illegal config file.");
			return ret;
		}
	}

	i = 0;
	while (i++ < 5) {
		ret = gup_i2c_write(client, file_config, file_cfg_len);
		if (ret > 0) {
			pr_info("Send config SUCCESS.");
			break;
		}
		pr_err("Send config i2c error.");
	}

	return ret;
}

static s32 gup_get_firmware_file(struct i2c_client *client,
		struct st_update_msg *msg, u8 *path)
{
	s32 ret;
	const struct firmware *fw = NULL;

	ret = request_firmware(&fw, path, &client->dev);
	if (ret < 0) {
		dev_info(&client->dev, "Cannot get firmware - %s (%d)\n",
					path, ret);
		return -EEXIST;
	}

	dev_dbg(&client->dev, "Config File: %s size=%ld", path, fw->size);
	msg->fw_data =
		devm_kzalloc(&client->dev, fw->size, GFP_KERNEL);
	if (!msg->fw_data) {
		dev_err(&client->dev,
			"Not enough memory for firmware data.");
		release_firmware(fw);
		return -ENOMEM;
	}

	memcpy(msg->fw_data, fw->data, fw->size);
	msg->fw_len = fw->size;
	msg->need_free = true;
	release_firmware(fw);
	return 0;
}

static u8 gup_check_firmware_name(struct i2c_client *client,
					u8 **path_p)
{
	u8 len;
	u8 *fname;

	if (!(*path_p)) {
		*path_p = GOODIX_FIRMWARE_FILE_NAME;
		return 0;
	}

	len = strnlen(*path_p, FIRMWARE_NAME_LEN_MAX);
	if (len >= FIRMWARE_NAME_LEN_MAX) {
		dev_err(&client->dev, "firmware name too long!");
		return -EINVAL;
	}

	fname = strrchr(*path_p, '/');
	if (fname) {
		fname = fname + 1;
		*path_p = fname;
	}
	return 0;
}*/

static u8 gup_check_update_file(struct i2c_client *client,
			struct st_fw_head *fw_head, u8 *path)
{
	//s32 ret = 0;
	s32 i = 0;
	s32 fw_checksum = 0;
	//u16 temp;
	//const struct firmware *fw = NULL;


        pr_info("Update by default firmware array");

	/*ret = request_firmware(&fw, GOODIX_CONFIG_FILE_NAME, &client->dev);
	if (ret < 0) {
		dev_info(&client->dev, "Cannot get config file - %s (%d)\n",
						GOODIX_CONFIG_FILE_NAME, ret);
	} else {
		dev_dbg(&client->dev,
			"Update config File: %s", GOODIX_CONFIG_FILE_NAME);
		ret = gup_update_config(client, fw);
		if (ret <= 0)
			dev_err(&client->dev, "Update config failed.");
		release_firmware(fw);
	}

	update_msg.need_free = false;
	update_msg.fw_len = 0; */
		update_msg.fw_total_len =
		    sizeof(gtp_default_FW) - FW_HEAD_LENGTH;
		if (sizeof(gtp_default_FW) <
		    (FW_HEAD_LENGTH + FW_SECTION_LENGTH * 4 +
		     FW_DSP_ISP_LENGTH + FW_DSP_LENGTH + FW_BOOT_LENGTH)) {
			pr_err
			    ("INVALID gtp_default_FW, check gt9xx_firmware.h!");
			return FAIL;
		}
		pr_info("Firmware actual size: %d(%dK)",
			  update_msg.fw_total_len,
			  update_msg.fw_total_len / 1024);
		memcpy(fw_head, &gtp_default_FW[0], FW_HEAD_LENGTH);

		/* check firmware legality */
		fw_checksum = 0;
		for (i = 0; i < update_msg.fw_total_len; i += 2)
			fw_checksum +=
			    (gtp_default_FW[FW_HEAD_LENGTH + i] << 8) +
			    gtp_default_FW[FW_HEAD_LENGTH + i + 1];

//		pr_info("firmware checksum:%x", fw_checksum & 0xFFFF);
		if (fw_checksum & 0xFFFF) {
			pr_err("Illegal firmware file.");
			return FAIL;
		}
		got_file_flag = HEADER_FW_READY;
		return SUCCESS;



	/*if (gup_check_firmware_name(client, &path))
		goto load_failed;

	if (gup_get_firmware_file(client, &update_msg, path))
		goto load_failed;

	memcpy(fw_head, update_msg.fw_data, FW_HEAD_LENGTH);*/

	/* check firmware legality */
	/*fw_checksum = 0;
	for (i = 0; i < FW_SECTION_LENGTH * 4 + FW_DSP_ISP_LENGTH +
			FW_DSP_LENGTH + FW_BOOT_LENGTH; i += 2) {
		temp = (update_msg.fw_data[FW_HEAD_LENGTH + i] << 8) +
			update_msg.fw_data[FW_HEAD_LENGTH + i + 1];
		fw_checksum += temp;
	}

	pr_info("firmware checksum:%x", fw_checksum & 0xFFFF);
	if (fw_checksum & 0xFFFF) {
		dev_err(&client->dev, "Illegal firmware file.");
		goto load_failed;
	}

	return SUCCESS;

load_failed:
	if (update_msg.need_free) {
		devm_kfree(&client->dev, update_msg.fw_data);
		update_msg.need_free = false;
	}
	return FAIL;*/
}

static u8 gup_burn_proc(struct i2c_client *client, u8 *burn_buf, u16 start_addr,
							u16 total_length)
{
	s32 ret = 0;
	u16 burn_addr = start_addr;
	u16 frame_length = 0;
	u16 burn_length = 0;
	u8  wr_buf[PACK_SIZE + GTP_ADDR_LENGTH];
	u8  rd_buf[PACK_SIZE + GTP_ADDR_LENGTH];
	u8  retry = 0;

	pr_info("Begin burn %dk data to addr 0x%x", (total_length / 1024),
								start_addr);
	while (burn_length < total_length) {
		pr_info("B/T:%04d/%04d", burn_length, total_length);
		frame_length = ((total_length - burn_length) > PACK_SIZE)
				? PACK_SIZE : (total_length - burn_length);
		wr_buf[0] = (u8)(burn_addr>>8);
		rd_buf[0] = wr_buf[0];
		wr_buf[1] = (u8)burn_addr;
		rd_buf[1] = wr_buf[1];
		memcpy(&wr_buf[GTP_ADDR_LENGTH], &burn_buf[burn_length],
								frame_length);

		for (retry = 0; retry < MAX_FRAME_CHECK_TIME; retry++) {
			ret = gup_i2c_write(client, wr_buf,
					GTP_ADDR_LENGTH + frame_length);
			if (ret <= 0) {
				pr_err("Write frame data i2c error.");
				continue;
			}
			ret = gup_i2c_read(client, rd_buf, GTP_ADDR_LENGTH +
							frame_length);
			if (ret <= 0) {
				pr_err("Read back frame data i2c error.");
				continue;
			}

			if (memcmp(&wr_buf[GTP_ADDR_LENGTH],
				&rd_buf[GTP_ADDR_LENGTH], frame_length)) {
				pr_err("Check frame data fail,not equal.");
				continue;
			} else {
				break;
			}
		}
		if (retry >= MAX_FRAME_CHECK_TIME) {
			pr_err("Burn frame data time out,exit.");
			return FAIL;
		}
		burn_length += frame_length;
		burn_addr += frame_length;
	}
	return SUCCESS;
}

static u8 gup_load_section_file(u8 *buf, u32 offset, u16 length, u8 set_or_end)
{
	if (got_file_flag == HEADER_FW_READY) {
		if (SEEK_SET == set_or_end)
			memcpy(buf, &gtp_default_FW[FW_HEAD_LENGTH + offset],
			       length);
		else
			memcpy(buf,
			       &gtp_default_FW[update_msg.fw_total_len +
					       FW_HEAD_LENGTH - offset],
			       length);
		return SUCCESS;
	}
	{
		s32 ret = 0;

		if ((update_msg.file == NULL) || IS_ERR(update_msg.file)) {
			pr_err
			    ("cannot find update file,load section file fail.");
			return FAIL;
		}

		if (SEEK_SET == set_or_end)
			update_msg.file->f_pos = FW_HEAD_LENGTH + offset;
		else
			update_msg.file->f_pos =
			    update_msg.fw_total_len + FW_HEAD_LENGTH - offset;

		ret =
		    update_msg.file->f_op->read(update_msg.file, (char *)buf,
						length,
						&update_msg.file->f_pos);
		if (ret < 0) {
			pr_err("Read update file fail.");
			return FAIL;
		}

		return SUCCESS;
	}
}

static u8 gup_recall_check(struct i2c_client *client, u8 *chk_src,
					u16 start_rd_addr, u16 chk_length)
{
	u8  rd_buf[PACK_SIZE + GTP_ADDR_LENGTH];
	s32 ret = 0;
	u16 recall_addr = start_rd_addr;
	u16 recall_length = 0;
	u16 frame_length = 0;

	while (recall_length < chk_length) {
		frame_length = ((chk_length - recall_length) > PACK_SIZE)
				? PACK_SIZE : (chk_length - recall_length);
		ret = gup_get_ic_msg(client, recall_addr, rd_buf, frame_length);
		if (ret <= 0) {
			pr_err("recall i2c error,exit");
			return FAIL;
		}

		if (memcmp(&rd_buf[GTP_ADDR_LENGTH], &chk_src[recall_length],
			frame_length)) {
			pr_err("Recall frame data fail,not equal.");
			return FAIL;
		}

		recall_length += frame_length;
		recall_addr += frame_length;
	}
	pr_info("Recall check %dk firmware success.", (chk_length/1024));

	return SUCCESS;
}

static u8 gup_burn_fw_section(struct i2c_client *client, u8 *fw_section,
					u16 start_addr, u8 bank_cmd)
{
	s32 ret = 0;
	u8  rd_buf[5];

	/* step1:hold ss51 & dsp */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("hold ss51 & dsp fail.");
		return FAIL;
	}

	 /* step2:set scramble */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("set scramble fail.");
		return FAIL;
	}

	/* step3:select bank */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK,
						(bank_cmd >> 4)&0x0F);
	if (ret <= 0) {
		pr_err("select bank %d fail.",
					(bank_cmd >> 4)&0x0F);
		return FAIL;
	}

	/* step4:enable accessing code */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
	if (ret <= 0) {
		pr_err("enable accessing code fail.");
		return FAIL;
	}

	/* step5:burn 8k fw section */
	ret = gup_burn_proc(client, fw_section, start_addr, FW_SECTION_LENGTH);
	if (ret == FAIL)  {
		pr_err("burn fw_section fail.");
		return FAIL;
	}

	/* step6:hold ss51 & release dsp */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
	if (ret <= 0) {
		pr_err("hold ss51 & release dsp fail.");
		return FAIL;
	}
	/* must delay */
	msleep(1);

	/* step7:send burn cmd to move data to flash from sram */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, bank_cmd&0x0f);
	if (ret <= 0) {
		pr_err("send burn cmd fail.");
		return FAIL;
	}
	pr_info("Wait for the burn is complete.");
	do {
		ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
		if (ret <= 0) {
			pr_err("Get burn state fail");
			return FAIL;
		}
		msleep(10);
	} while (rd_buf[GTP_ADDR_LENGTH]);

	/* step8:select bank */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK,
							(bank_cmd >> 4)&0x0F);
	if (ret <= 0) {
		pr_err("select bank %d fail.",
							(bank_cmd >> 4)&0x0F);
		return FAIL;
	}

	/* step9:enable accessing code */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
	if (ret <= 0) {
		pr_err("enable accessing code fail.");
		return FAIL;
	}

	/* step10:recall 8k fw section */
	ret = gup_recall_check(client, fw_section, start_addr,
							FW_SECTION_LENGTH);
	if (ret == FAIL) {
		pr_err("recall check 8k firmware fail.");
		return FAIL;
	}

	/* step11:disable accessing code */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x00);
	if (ret <= 0) {
		pr_err("disable accessing code fail.");
		return FAIL;
	}

	return SUCCESS;
}

static u8 gup_burn_dsp_isp(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *fw_dsp_isp = NULL;
	u8  retry = 0;

	pr_info("Begin burn dsp isp.");

	/* step1:alloc memory */
	pr_info("step1:alloc memory");
	while (retry++ < 5) {
		fw_dsp_isp = devm_kzalloc(&client->dev, FW_DSP_ISP_LENGTH,
								GFP_KERNEL);
		if (fw_dsp_isp == NULL) {
			continue;
		} else {
			pr_info("Alloc %dk byte memory success.",
					(FW_DSP_ISP_LENGTH/1024));
			break;
		}
	}
	if (retry == 5) {
		pr_err("Alloc memory fail,exit.");
		return FAIL;
	}

	/* step2:load dsp isp file data */
	pr_info("step2:load dsp isp file data");
	ret = gup_load_section_file(fw_dsp_isp, FW_DSP_ISP_LENGTH, FW_DSP_ISP_LENGTH, SEEK_END);
	if (ret == FAIL) {
		pr_err("load firmware dsp_isp fail.");
		return FAIL;
	}

	/* step3:disable wdt,clear cache enable */
	pr_info("step3:disable wdt,clear cache enable");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__TMR0_EN, 0x00);
	if (ret <= 0) {
		pr_err("disable wdt fail.");
		return FAIL;
	}
	ret = gup_set_ic_msg(client, _bRW_MISCTL__CACHE_EN, 0x00);
	if (ret <= 0) {
		pr_err("clear cache enable fail.");
		return FAIL;
	}

	/* step4:hold ss51 & dsp */
	pr_info("step4:hold ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("hold ss51 & dsp fail.");
		return FAIL;
	}

	/* step5:set boot from sram */
	pr_info("step5:set boot from sram");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOTCTL_B0_, 0x02);
	if (ret <= 0) {
		pr_err("set boot from sram fail.");
		return FAIL;
	}

	/* step6:software reboot */
	pr_info("step6:software reboot");
	ret = gup_set_ic_msg(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01);
	if (ret <= 0) {
		pr_err("software reboot fail.");
		return FAIL;
	}

	/* step7:select bank2 */
	pr_info("step7:select bank2");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x02);
	if (ret <= 0) {
		pr_err("select bank2 fail.");
		return FAIL;
	}

	/* step8:enable accessing code */
	pr_info("step8:enable accessing code");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
	if (ret <= 0) {
		pr_err("enable accessing code fail.");
		return FAIL;
	}

	/* step9:burn 4k dsp_isp */
	pr_info("step9:burn 4k dsp_isp");
	ret = gup_burn_proc(client, fw_dsp_isp, 0xC000, FW_DSP_ISP_LENGTH);
	if (ret == FAIL) {
		pr_err("burn dsp_isp fail.");
		return FAIL;
	}

	/* step10:set scramble */
	pr_info("step10:set scramble");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("set scramble fail.");
		return FAIL;
	}
        update_msg.fw_burned_len += FW_DSP_ISP_LENGTH;
	return SUCCESS;
}

static u8 gup_burn_fw_ss51(struct i2c_client *client)
{
	u8 *fw_ss51 = NULL;
	u8  retry = 0;
	s32 ret = 0;

	pr_info("Begin burn ss51 firmware.");

	/* step1:alloc memory */
	pr_info("step1:alloc memory");
	while (retry++ < 5) {
		fw_ss51 = devm_kzalloc(&client->dev, FW_SECTION_LENGTH,
							GFP_KERNEL);
		if (fw_ss51 == NULL) {
			continue;
		} else {
			pr_info("Alloc %dk byte memory success.",
						(FW_SECTION_LENGTH/1024));
			break;
		}
	}
	if (retry == 5) {
		pr_err("Alloc memory fail,exit.");
		return FAIL;
	}

	/* step2:load ss51 firmware section 1 file data */
	/*pr_info("step2:load ss51 firmware section 1 file data");
	ret = gup_load_section_file(fw_ss51, 0, FW_SECTION_LENGTH);
	if (ret == FAIL) {
		pr_err("load ss51 firmware section 1 fail.");
		return FAIL;
	}*/
	pr_info("[burn_fw_ss51]Reset first 8K of ss51 to 0xFF.");
	pr_info("[burn_fw_ss51]step2: reset bank0 0xC000~0xD000");
	memset(fw_ss51, 0xFF, FW_SECTION_LENGTH);

	/* step3:clear control flag */
	pr_info("step3:clear control flag");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x00);
	if (ret <= 0) {
		pr_err("clear control flag fail.");
		return FAIL;
	}

	/* step4:burn ss51 firmware section 1 */
	pr_info("step4:burn ss51 firmware section 1");
	ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x01);
	if (ret == FAIL) {
		pr_err("burn ss51 firmware section 1 fail.");
		return FAIL;
	}

	/* step5:load ss51 firmware section 2 file data */
	pr_info("step5:load ss51 firmware section 2 file data");
	ret = gup_load_section_file(fw_ss51, FW_SECTION_LENGTH,FW_SECTION_LENGTH, SEEK_SET);
	if (ret == FAIL) {
		pr_err("[burn_fw_ss51]load ss51 firmware section 2 fail.");
		return FAIL;
	}

	/* step6:burn ss51 firmware section 2 */
	pr_info("step6:burn ss51 firmware section 2");
	ret = gup_burn_fw_section(client, fw_ss51, 0xE000, 0x02);
	if (ret == FAIL) {
		pr_err("burn ss51 firmware section 2 fail.");
		return FAIL;
	}

	/* step7:load ss51 firmware section 3 file data */
	pr_info("step7:load ss51 firmware section 3 file data");
	ret = gup_load_section_file(fw_ss51, 2*FW_SECTION_LENGTH,FW_SECTION_LENGTH, SEEK_SET);
	if (ret == FAIL) {
		pr_err("load ss51 firmware section 3 fail.");
		return FAIL;
	}

	/* step8:burn ss51 firmware section 3 */
	pr_info("step8:burn ss51 firmware section 3");
	ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x13);
	if (ret == FAIL) {
		pr_err("burn ss51 firmware section 3 fail.");
		return FAIL;
	}

	/* step9:load ss51 firmware section 4 file data */
	pr_info("step9:load ss51 firmware section 4 file data");
	ret = gup_load_section_file(fw_ss51, 3*FW_SECTION_LENGTH,FW_SECTION_LENGTH, SEEK_SET);
	if (ret == FAIL) {
		pr_err("load ss51 firmware section 4 fail.");
		return FAIL;
	}

	/* step10:burn ss51 firmware section 4 */
	pr_info("step10:burn ss51 firmware section 4");
	ret = gup_burn_fw_section(client, fw_ss51, 0xE000, 0x14);
	if (ret == FAIL) {
		pr_err("burn ss51 firmware section 4 fail.");
		return FAIL;
	}
	update_msg.fw_burned_len += (FW_SECTION_LENGTH * 4);
	pr_info("[burn_fw_ss51]Burned length:%d", update_msg.fw_burned_len);
	return SUCCESS;
}

static u8 gup_burn_fw_dsp(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *fw_dsp = NULL;
	u8  retry = 0;
	u8  rd_buf[5];

	pr_info("Begin burn dsp firmware.");
	/* step1:alloc memory */
	pr_info("step1:alloc memory");
	while (retry++ < 5) {
		fw_dsp = devm_kzalloc(&client->dev, FW_DSP_LENGTH,
							GFP_KERNEL);
		if (fw_dsp == NULL) {
			continue;
		} else  {
			pr_info("Alloc %dk byte memory success.",
					(FW_SECTION_LENGTH/1024));
			break;
		}
	}
	if (retry == 5) {
		pr_err("Alloc memory fail,exit.");
		return FAIL;
	}

	/* step2:load firmware dsp */
	pr_info("step2:load firmware dsp");
	ret = gup_load_section_file(fw_dsp, 4*FW_SECTION_LENGTH, FW_DSP_LENGTH, SEEK_SET);
	if (ret == FAIL) {
		pr_err("load firmware dsp fail.");
		return ret;
	}

	/* step3:select bank3 */
	pr_info("step3:select bank3");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
	if (ret <= 0) {
		pr_err("select bank3 fail.");
		return FAIL;
	}

	/* Step4:hold ss51 & dsp */
	pr_info("step4:hold ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("hold ss51 & dsp fail.");
		return FAIL;
	}

	/* step5:set scramble */
	pr_info("step5:set scramble");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("set scramble fail.");
		return FAIL;
	}

	/* step6:release ss51 & dsp */
	pr_info("step6:release ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
	if (ret <= 0) {
		pr_err("release ss51 & dsp fail.");
		return FAIL;
	}
	/* must delay */
	msleep(1);

	/* step7:burn 4k dsp firmware */
	pr_info("step7:burn 4k dsp firmware");
	ret = gup_burn_proc(client, fw_dsp, 0x9000, FW_DSP_LENGTH);
	if (ret == FAIL) {
		pr_err("[burn_fw_dsp]burn fw_section fail.");
		return ret;
	}

	/* step8:send burn cmd to move data to flash from sram */
	pr_info("step8:send burn cmd to move data to flash from sram");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x05);
	if (ret <= 0) {
		pr_err("send burn cmd fail.");
		return ret;
	}
	pr_info("Wait for the burn is complete.");
	do {
		ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
		if (ret <= 0) {
			pr_err("Get burn state fail");
			return ret;
		}
		msleep(10);
	} while (rd_buf[GTP_ADDR_LENGTH]);

	/* step9:recall check 4k dsp firmware */
	pr_info("step9:recall check 4k dsp firmware");
	ret = gup_recall_check(client, fw_dsp, 0x9000, FW_DSP_LENGTH);
	if (ret == FAIL) {
		pr_err("recall check 4k dsp firmware fail.");
		return ret;
	}
	update_msg.fw_burned_len += FW_DSP_LENGTH;
	pr_info("[burn_fw_dsp]Burned length:%d", update_msg.fw_burned_len);
	return SUCCESS;
}

static u8 gup_burn_fw_boot(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *fw_boot = NULL;
	u8  retry = 0;
	u8  rd_buf[5];

	pr_info("Begin burn bootloader firmware.");

	/* step1:Alloc memory */
	pr_info("step1:Alloc memory");
	while (retry++ < 5) {
		fw_boot = devm_kzalloc(&client->dev, FW_BOOT_LENGTH,
							GFP_KERNEL);
		if (fw_boot == NULL) {
			continue;
		} else {
			pr_info("Alloc %dk byte memory success.",
						(FW_BOOT_LENGTH/1024));
			break;
		}
	}
	if (retry == 5) {
		pr_err("Alloc memory fail,exit.");
		return FAIL;
	}

	/* step2:load firmware bootloader */
	pr_info("step2:load firmware bootloader");
	ret = gup_load_section_file(fw_boot, (4 * FW_SECTION_LENGTH +
				FW_DSP_LENGTH), FW_BOOT_LENGTH, SEEK_SET);
	if (ret == FAIL) {
		pr_err("load firmware dsp fail.");
		return ret;
	}

	/* step3:hold ss51 & dsp */
	pr_info("step3:hold ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("hold ss51 & dsp fail.");
		return FAIL;
	}

	/* step4:set scramble */
	pr_info("step4:set scramble");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("set scramble fail.");
		return FAIL;
	}

	/* step5:release ss51 & dsp */
	pr_info("step5:release ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
	if (ret <= 0) {
		pr_err("release ss51 & dsp fail.");
		return FAIL;
	}
	/* must delay */
	msleep(1);

	/* step6:select bank3 */
	pr_info("step6:select bank3");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
	if (ret <= 0) {
		pr_err("select bank3 fail.");
		return FAIL;
	}

	/* step7:burn 2k bootloader firmware */
	pr_info("step7:burn 2k bootloader firmware");
	ret = gup_burn_proc(client, fw_boot, 0x9000, FW_BOOT_LENGTH);
	if (ret == FAIL) {
		pr_err("burn fw_section fail.");
		return ret;
	}

	/* step7:send burn cmd to move data to flash from sram */
	pr_info("step7:send burn cmd to flash data from sram");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x06);
	if (ret <= 0) {
		pr_err("send burn cmd fail.");
		return ret;
	}
	pr_info("Wait for the burn is complete.");
	do {
		ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
		if (ret <= 0) {
			pr_err("Get burn state fail");
			return ret;
		}
		msleep(10);
	} while (rd_buf[GTP_ADDR_LENGTH]);

	/* step8:recall check 2k bootloader firmware */
	pr_info("step8:recall check 2k bootloader firmware");
	ret = gup_recall_check(client, fw_boot, 0x9000, FW_BOOT_LENGTH);
	if (ret == FAIL) {
		pr_err("recall check 4k dsp firmware fail.");
		return ret;
	}
	update_msg.fw_burned_len += FW_BOOT_LENGTH;
	pr_info("[burn_fw_boot]Burned length:%d", update_msg.fw_burned_len);

	/* step9:enable download DSP code  */
	/*pr_info("step9:enable download DSP code ");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
	if (ret <= 0) {
		pr_err("enable download DSP code fail.");
		return FAIL;
	}*/

	/* step10:release ss51 & hold dsp */
	/*pr_info("step10:release ss51 & hold dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x08);
	if (ret <= 0) {
		pr_err("release ss51 & hold dsp fail.");
		return FAIL;
	}*/

	return SUCCESS;
}
static u8 gup_burn_fw_boot_isp(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *fw_boot_isp = NULL;
	u8 retry = 0;
	u8 rd_buf[5];

	if (update_msg.fw_burned_len >= update_msg.fw_total_len) {
		pr_info("No need to upgrade the boot_isp code!");
		return SUCCESS;
	}
	pr_info("[burn_fw_boot_isp]Begin burn boot_isp firmware---->>");

	/* step1:Alloc memory */
	pr_info("[burn_fw_boot_isp]step1:Alloc memory");
	while (retry++ < 5) {
		fw_boot_isp = kzalloc(FW_BOOT_ISP_LENGTH, GFP_KERNEL);
		if (fw_boot_isp == NULL)
			continue;
		else {
			pr_info
			    ("[burn_fw_boot_isp]Alloc %dk byte memory success.",
			     (FW_BOOT_ISP_LENGTH / 1024));
			break;
		}
	}
	if (retry >= 5) {
		pr_err("[burn_fw_boot_isp]Alloc memory fail,exit.");
		return FAIL;
	}
	/* step2:load firmware bootloader */
	pr_info("[burn_fw_boot_isp]step2:load firmware bootloader isp");
	ret = gup_load_section_file(fw_boot_isp,
			(update_msg.fw_burned_len - FW_DSP_ISP_LENGTH),
			FW_BOOT_ISP_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err("[burn_fw_boot_isp]load firmware boot_isp fail.");
		goto exit_burn_fw_boot_isp;
	}
	/* step3:hold ss51 & dsp */
	pr_info("[burn_fw_boot_isp]step3:hold ss51 & dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("[burn_fw_boot_isp]hold ss51 & dsp fail.");
		ret = FAIL;
		goto exit_burn_fw_boot_isp;
	}
	/* step4:set scramble */
	pr_info("[burn_fw_boot_isp]step4:set scramble");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("[burn_fw_boot_isp]set scramble fail.");
		ret = FAIL;
		goto exit_burn_fw_boot_isp;
	}
	/* step5:hold ss51 & release dsp */
	pr_info("[burn_fw_boot_isp]step5:hold ss51 & release dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
	if (ret <= 0) {
		pr_err("[burn_fw_boot_isp]release ss51 & dsp fail.");
		ret = FAIL;
		goto exit_burn_fw_boot_isp;
	}
	/* must delay */
	msleep(1);

	/* step6:select bank3 */
	pr_info("[burn_fw_boot_isp]step6:select bank3");
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
	if (ret <= 0) {
		pr_err("[burn_fw_boot_isp]select bank3 fail.");
		ret = FAIL;
		goto exit_burn_fw_boot_isp;
	}
	/* step7:burn 2k bootload_isp firmware */
	pr_info("[burn_fw_boot_isp]step7:burn 2k bootloader firmware");
	ret = gup_burn_proc(client, fw_boot_isp, 0x9000, FW_BOOT_ISP_LENGTH);
	if (FAIL == ret) {
		pr_err("[burn_fw_boot_isp]burn fw_section fail.");
		goto exit_burn_fw_boot_isp;
	}
	/* step7:send burn cmd to move data to flash from sram */
	pr_info("[burn_fw_boot_isp]step8: move data to flash from sram");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x07);
	if (ret <= 0) {
		pr_err("[burn_fw_boot_isp]send burn cmd fail.");
		goto exit_burn_fw_boot_isp;
	}
	pr_info("[burn_fw_boot_isp]Wait for the burn is complete......");
	do {
		ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
		if (ret <= 0) {
			pr_err("[burn_fw_boot_isp]Get burn state fail");
			goto exit_burn_fw_boot_isp;
		}
		msleep(10);
	} while (rd_buf[GTP_ADDR_LENGTH]);

	/* step8:recall check 2k bootload_isp firmware */
	pr_info
	    ("[burn_fw_boot_isp]step9:recall check 2k bootloader firmware");
	ret = gup_recall_check(client, fw_boot_isp, 0x9000, FW_BOOT_ISP_LENGTH);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_boot_isp]check bootcode_isp firmware fail.");
		goto exit_burn_fw_boot_isp;
	}

	update_msg.fw_burned_len += FW_BOOT_ISP_LENGTH;
	pr_info("[burn_fw_boot_isp]Burned length:%d",
		  update_msg.fw_burned_len);
	ret = SUCCESS;

exit_burn_fw_boot_isp:
	kfree(fw_boot_isp);
	return ret;
}

static u8 gup_burn_fw_link(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *fw_link = NULL;
	u8 retry = 0;
	u32 offset;

	if (update_msg.fw_burned_len >= update_msg.fw_total_len) {
		pr_info("No need to upgrade the link code!");
		return SUCCESS;
	}
	pr_info("[burn_fw_link]Begin burn link firmware---->>");

	/* step1:Alloc memory */
	pr_info("[burn_fw_link]step1:Alloc memory");
	while (retry++ < 5) {
		fw_link = kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
		if (fw_link == NULL)
			continue;
		else {
			pr_info
			    ("[burn_fw_link]Alloc %dk byte memory success.",
			    (FW_SECTION_LENGTH / 1024));
			break;
		}
	}
	if (retry >= 5) {
		pr_err("[burn_fw_link]Alloc memory fail,exit.");
		return FAIL;
	}
	/* step2:load firmware link section 1 */
	pr_info("[burn_fw_link]step2:load firmware link section 1");
	offset = update_msg.fw_burned_len - FW_DSP_ISP_LENGTH;
	ret = gup_load_section_file(fw_link, offset,
					FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err("[burn_fw_link]load firmware link section 1 fail.");
		goto exit_burn_fw_link;
	}
	/* step3:burn link firmware section 1 */
	pr_info("[burn_fw_link]step3:burn link firmware section 1");
	ret = gup_burn_fw_gwake_section(client, fw_link, 0x9000,
					FW_SECTION_LENGTH, 0x38);
	if (FAIL == ret) {
		pr_err("[burn_fw_link]burn link firmware section 1 fail.");
		goto exit_burn_fw_link;
	}
	/* step4:load link firmware section 2 file data */
	pr_info("[burn_fw_link]step4:load link firmware section 2 file data");
	offset += FW_SECTION_LENGTH;
	ret = gup_load_section_file(fw_link, offset,
				FW_GLINK_LENGTH - FW_SECTION_LENGTH,
				SEEK_SET);
	if (FAIL == ret) {
		pr_err("[burn_fw_link]load link firmware section 2 fail.");
		goto exit_burn_fw_link;
	}
	/* step5:burn link firmware section 2 */
	pr_info("[burn_fw_link]step4:burn link firmware section 2");
	ret = gup_burn_fw_gwake_section(client, fw_link, 0x9000,
					FW_GLINK_LENGTH - FW_SECTION_LENGTH,
					0x39);
	if (FAIL == ret) {
		pr_err("[burn_fw_link]burn link firmware section 2 fail.");
		goto exit_burn_fw_link;
	}

	update_msg.fw_burned_len += FW_GLINK_LENGTH;
	pr_info("[burn_fw_link]Burned length:%d", update_msg.fw_burned_len);
	ret = SUCCESS;

exit_burn_fw_link:
	kfree(fw_link);
	return ret;
}

static u8 gup_burn_fw_gwake_section(struct i2c_client *client, u8 *fw_section,
				    u16 start_addr, u32 len, u8 bank_cmd)
{
	s32 ret = 0;
	u8 rd_buf[5];

	/* step1:hold ss51 & dsp */
        pr_info("[burn_fw_app_section] step1!!");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
	if (ret <= 0) {
		pr_err("[burn_fw_app_section]hold ss51 & dsp fail.");
		return FAIL;
	}
        pr_info("[burn_fw_app_section] step2!!");
	/* step2:set scramble */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	if (ret <= 0) {
		pr_err("[burn_fw_app_section]set scramble fail.");
		return FAIL;
	}
         pr_info("[burn_fw_app_section] step3!!");
	/* step3:hold ss51 & release dsp */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
	if (ret <= 0) {
		pr_err("[burn_fw_app_section]hold ss51 & release dsp fail.");
		return FAIL;
	}
	/* must delay */
	msleep(1);
         pr_info("[burn_fw_app_section] step4!!");
	/* step4:select bank */
	ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK,
				(bank_cmd >> 4) & 0x0F);
	if (ret <= 0) {
		pr_err("[burn_fw_section]select bank %d fail.",
			  (bank_cmd >> 4) & 0x0F);
		return FAIL;
	}
        pr_info("[burn_fw_app_section] step5!!");
	/* step5:burn fw section */
	ret = gup_burn_proc(client, fw_section, start_addr, len);
	if (FAIL == ret) {
		pr_err("[burn_fw_app_section]burn fw_section fail.");
		return FAIL;
	}
        pr_info("[burn_fw_app_section] step6!!");
	/* step6:send burn cmd to move data to flash from sram */
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, bank_cmd & 0x0F);
	if (ret <= 0) {
		pr_err("[burn_fw_app_section]send burn cmd fail.");
		return FAIL;
	}
	pr_info("[burn_fw_section]Wait for the burn is complete......");
	do {
		ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
		if (ret <= 0) {
			pr_err("[burn_fw_app_section]Get burn state fail");
			return FAIL;
		}
		msleep(10);
	} while (rd_buf[GTP_ADDR_LENGTH]);
       pr_info("[burn_fw_app_section] step7!!");
	/* step7:recall fw section */
	ret = gup_recall_check(client, fw_section, start_addr, len);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_app_section]recall check %dk firmware fail.",
		     len / 1024);
		return FAIL;
	}

	return SUCCESS;
}

static u8 gup_burn_fw_gwake(struct i2c_client *client)
{
	u8 *fw_gwake = NULL;
	u8 retry = 0;
	s32 ret = 0;
	//u16 start_index;
        u16 start_index = 4*FW_SECTION_LENGTH+FW_DSP_LENGTH+FW_BOOT_LENGTH + FW_BOOT_ISP_LENGTH + FW_GLINK_LENGTH;
	if (update_msg.fw_burned_len >= update_msg.fw_total_len) {
		pr_info("No need to upgrade the gwake code!");
		return SUCCESS;
	}
	//start_index = update_msg.fw_burned_len - FW_DSP_ISP_LENGTH;
	pr_info("[burn_fw_gwake]Begin burn gwake firmware---->>");

	/* step1:alloc memory */
	pr_info("[burn_fw_gwake]step1:alloc memory");
	while (retry++ < 5) {
		fw_gwake = kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
		if (fw_gwake == NULL)
			continue;
		else {
			pr_info
			    ("[burn_fw_gwake]Alloc %dk byte memory success.",
			     (FW_SECTION_LENGTH / 1024));
			break;
		}
	}
	if (retry >= 5) {
		pr_err("[burn_fw_gwake]Alloc memory fail,exit.");
		return FAIL;
	}

	ret = gup_set_ic_msg(client,_rRW_MISCTL__BOOT_CTL_, 0x00);
	if(ret <= 0)
       {
        pr_info("[burn_fw_finish]clear control flag fail.");
        goto exit_burn_fw_gwake;
       }

	/* step2:load app_code firmware section 1 file data */
	pr_info
	    ("[burn_fw_gwake]step2:load app_code firmware section 1 file data");
	ret = gup_load_section_file(fw_gwake, start_index,
					FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]load app_code firmware section 1 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step3:burn app_code firmware section 1 */
	pr_info("[burn_fw_gwake]step3:burn app_code firmware section 1");
	ret = gup_burn_fw_gwake_section(client, fw_gwake, 0x9000,
					FW_SECTION_LENGTH, 0x3A);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]burn app_code firmware section 1 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step5:load app_code firmware section 2 file data */
	pr_info
	    ("[burn_fw_gwake]step5:load app_code firmware section 2 file data");
	ret = gup_load_section_file(fw_gwake, start_index + FW_SECTION_LENGTH,
						FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]load app_code firmware section 2 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step6:burn app_code firmware section 2 */
	pr_info("[burn_fw_gwake]step6:burn app_code firmware section 2");
	ret = gup_burn_fw_gwake_section(client, fw_gwake, 0x9000,
					FW_SECTION_LENGTH, 0x3B);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]burn app_code firmware section 2 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step7:load app_code firmware section 3 file data */
	pr_info
	    ("[burn_fw_gwake]step7:load app_code firmware section 3 file data");
	ret = gup_load_section_file(fw_gwake, start_index
					+ 2 * FW_SECTION_LENGTH,
					FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]load app_code firmware section 3 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step8:burn app_code firmware section 3 */
	pr_info("[burn_fw_gwake]step8:burn app_code firmware section 3");
	ret = gup_burn_fw_gwake_section(client, fw_gwake, 0x9000,
					FW_SECTION_LENGTH, 0x3C);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]burn app_code firmware section 3 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step9:load app_code firmware section 4 file data */
	pr_info
	    ("[burn_fw_gwake]step9:load app_code firmware section 4 file data");
	ret = gup_load_section_file(fw_gwake, start_index
					+ 3 * FW_SECTION_LENGTH,
					FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]load app_code firmware section 4 fail.");
		goto exit_burn_fw_gwake;
	}
	/* step10:burn app_code firmware section 4 */
	pr_info("[burn_fw_gwake]step10:burn app_code firmware section 4");
	ret = gup_burn_fw_gwake_section(client, fw_gwake, 0x9000,
					FW_SECTION_LENGTH, 0x3D);
	if (FAIL == ret) {
		pr_err
		    ("[burn_fw_gwake]burn app_code firmware section 4 fail.");
		goto exit_burn_fw_gwake;
	}

	//update_msg.fw_burned_len += FW_GWAKE_LENGTH;
	pr_info("[burn_fw_gwake]Burned length:%d", update_msg.fw_burned_len);
	ret = SUCCESS;

exit_burn_fw_gwake:
	kfree(fw_gwake);
	return ret;
}

static u8 gup_burn_fw_finish(struct i2c_client *client)
{
	u8 *fw_ss51 = NULL;
	u8 retry = 0;
	s32 ret = 0;

	pr_info("[burn_fw_finish]burn first 8K of ss51 and finish update.");
	/* step1:alloc memory */
	pr_info("[burn_fw_finish]step1:alloc memory");
	while (retry++ < 5) {
		fw_ss51 = kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
		if (fw_ss51 == NULL)
			continue;
		else {
			pr_info
			    ("[burn_fw_finish]Alloc %dk byte memory success.",
			     (FW_SECTION_LENGTH / 1024));
			break;
		}
	}
	if (retry >= 5) {
		pr_err("[burn_fw_finish]Alloc memory fail,exit.");
		return FAIL;
	}

	pr_info("[burn_fw_finish]step2: burn ss51 first 8K.");
	ret = gup_load_section_file(fw_ss51, 0, FW_SECTION_LENGTH, SEEK_SET);
	if (FAIL == ret) {
		pr_err("[burn_fw_finish]load ss51 firmware section 1 fail.");
		goto exit_burn_fw_finish;
	}

	pr_info("[burn_fw_finish]step3:clear control flag");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x00);
	if (ret <= 0) {
		pr_err("[burn_fw_finish]clear control flag fail.");
		goto exit_burn_fw_finish;
	}

	pr_info("[burn_fw_finish]step4:burn ss51 firmware section 1");
	ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x01);
	if (FAIL == ret) {
		pr_err("[burn_fw_finish]burn ss51 firmware section 1 fail.");
		goto exit_burn_fw_finish;
	}
	/* step11:enable download DSP code */
	pr_info("[burn_fw_finish]step5:enable download DSP code ");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
	if (ret <= 0) {
		pr_err("[burn_fw_finish]enable download DSP code fail.");
		goto exit_burn_fw_finish;
	}
	/* step12:release ss51 & hold dsp */
	pr_info("[burn_fw_finish]step6:release ss51 & hold dsp");
	ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x08);
	if (ret <= 0) {
		pr_err("[burn_fw_finish]release ss51 & hold dsp fail.");
		goto exit_burn_fw_finish;
	}

	kfree(fw_ss51);
	return SUCCESS;

exit_burn_fw_finish:
	kfree(fw_ss51);
	return FAIL;
}

s32 gup_update_proc(void *dir)
{
	s32 ret = 0;
	u8 retry = 0;
	struct st_fw_head fw_head;
	struct goodix_ts_data *ts = NULL;

	pr_info("Begin update.");

	if (!i2c_connect_client) {
		pr_err("No i2c connect client for %s\n", __func__);
		return -EIO;
	}

	show_len = 1;
	total_len = 100;

	ts = i2c_get_clientdata(i2c_connect_client);

	if (searching_file) {
		/* exit .bin update file searching  */
		searching_file = 0;
		pr_info("Exiting searching .bin update file.");
		/* wait for auto update quitted completely */
		while ((show_len != 200) && (show_len != 100))
			msleep(100);
	}
        update_msg.file = NULL;
	ret = gup_check_update_file(i2c_connect_client, &fw_head, (u8 *)dir);
	if (ret == FAIL) {
		pr_err("check update file fail.");
		goto file_fail;
	}

	/* gtp_reset_guitar(i2c_connect_client, 20); */
	ret = gup_get_ic_fw_msg(i2c_connect_client);
	if (ret == FAIL) {
		pr_err("get ic message fail.");
		goto file_fail;
	}

	//if (ts->force_update) {
		//dev_dbg(&ts->client->dev, "Enter force update.");
	//} else {
		ret = gup_enter_update_judge(&fw_head);
		if (ret == FAIL) {
			pr_err("[update_proc]Check *.bin file fail.");
			goto file_fail;
		}
	//}

	ts->enter_update = 1;
	gtp_irq_disable(ts);
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ret = gup_enter_update_mode(i2c_connect_client);
	if (ret == FAIL) {
		pr_err("enter update mode fail.");
		goto update_fail;
	}

	while (retry++ < 5) {
		show_len = 10;
		total_len = 100;
                update_msg.fw_burned_len = 0;
		ret = gup_burn_dsp_isp(i2c_connect_client);
		if (ret == FAIL) {
			pr_err("burn dsp isp fail.");
			continue;
		}

		show_len = 20;
		ret = gup_burn_fw_ss51(i2c_connect_client);
		if (ret == FAIL) {
			pr_err("burn ss51 firmware fail.");
			continue;
		}

		show_len = 30;
		ret = gup_burn_fw_dsp(i2c_connect_client);
		if (ret == FAIL) {
			pr_err("burn dsp firmware fail.");
			continue;
		}

		show_len = 40;
		ret = gup_burn_fw_boot(i2c_connect_client);
		if (ret == FAIL) {
			pr_err("burn bootloader fw fail.");
			continue;
		}
		show_len = 50;
		ret = gup_burn_fw_boot_isp(i2c_connect_client);
		if (FAIL == ret) {
			pr_err("[update_proc]burn boot_isp firmware fail.");
			continue;
		}

		show_len = 60;
		ret = gup_burn_fw_link(i2c_connect_client);
		if (FAIL == ret) {
			pr_err("[update_proc]burn link firmware fail.");
			continue;
		}

		show_len = 70;
		ret = gup_burn_fw_gwake(i2c_connect_client);
		if (FAIL == ret) {
			pr_err("[update_proc]burn app_code firmware fail.");
			continue;
		}
		show_len = 80;

		ret = gup_burn_fw_finish(i2c_connect_client);
		if (FAIL == ret) {
			pr_err("[update_proc]burn finish fail.");
			continue;
		}
		show_len = 90;
		pr_info("UPDATE SUCCESS.");
		break;
	}
	if (retry >= 5) {
		pr_err("retry timeout,UPDATE FAIL.");
		goto update_fail;
	}

	pr_info("leave update mode.");
	gup_leave_update_mode(i2c_connect_client);

	msleep(100);

	if (ts->fw_error) {
		pr_info("firmware error auto update, resent config!");
		gup_init_panel(ts);
	}
	show_len = 100;
	total_len = 100;
	ts->enter_update = 0;
	gtp_irq_enable(ts);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
	if (update_msg.need_free) {
		devm_kfree(&ts->client->dev, update_msg.fw_data);
		update_msg.need_free = false;
	}

	return SUCCESS;

update_fail:
	ts->enter_update = 0;
	gtp_irq_enable(ts);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

file_fail:
	show_len = 200;
	total_len = 100;
	if (update_msg.need_free) {
		devm_kfree(&ts->client->dev, update_msg.fw_data);
		update_msg.need_free = false;
	}
	return FAIL;
}

static void gup_update_work(struct work_struct *work)
{
	if (gup_update_proc(NULL) == FAIL)
		pr_err("Goodix update work fail!\n");
}

u8 gup_init_update_proc(struct goodix_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "Ready to run update work.");

	INIT_DELAYED_WORK(&ts->goodix_update_work, gup_update_work);
	schedule_delayed_work(&ts->goodix_update_work,
		msecs_to_jiffies(3000));

	return 0;
}
