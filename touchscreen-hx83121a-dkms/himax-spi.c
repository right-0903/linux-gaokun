// SPDX-License-Identifier: GPL-2.0
/*
 * Himax SPI Driver
 * based on Himax hx83102j
 *
 * Only tested it on the 2023 model without screen protector
 *
 * Copyright (C) 2019,2024 Himax Corporation.
 * Copyright (C) 2026 Pengyu Luo <mitltlatltl@gmail.com>
 */

/* TODO: DT parse: vdd_dig & avdd_analog */

#define DEBUG
#include<linux/sort.h>
#include<linux/dev_printk.h>

#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/math.h>

#include <drm/drm_panel.h>

#define HIMAX_BUS_RETRY					3
/* SPI bus read header length */
#define HIMAX_BUS_R_HLEN				3U
/* SPI bus write header length */
#define HIMAX_BUS_W_HLEN				2U
/* TP SRAM address size and data size */
#define HIMAX_REG_SZ					4U
#define HIMAX_MAX_RX					60U
#define HIMAX_MAX_TX					40U
#define HIMAX_MAX_TOUCH					10
#define HIMAX_HX83121A_SAFE_MODE_PASSWORD		0x9527
/* FIXME: this is for hx83120j, ~4928 for hx83121a 40 * 60 * 2 + 128 */
#define HIMAX_HX83121A_STACK_SIZE			128U
#define HIMAX_HX83121A_FULL_STACK_SZ \
(HIMAX_HX83121A_STACK_SIZE + \
(2 + HIMAX_MAX_RX * HIMAX_MAX_TX + \
HIMAX_MAX_TX + HIMAX_MAX_RX) * 2)

/* Clear 4 bytes data */
#define HIMAX_DATA_CLEAR				0x00000000
/* AHB register addresses */
#define HIMAX_AHB_ADDR_BYTE_0				0x00
#define HIMAX_AHB_ADDR_RDATA_BYTE_0			0x08
#define HIMAX_AHB_ADDR_ACCESS_DIRECTION			0x0c
#define HIMAX_AHB_ADDR_INCR4				0x0d
#define HIMAX_AHB_ADDR_CONTI				0x13
#define HIMAX_AHB_ADDR_EVENT_STACK			0x30
#define HIMAX_AHB_ADDR_PSW_LB				0x31

/* AHB register values/commands */
#define HIMAX_AHB_CMD_ACCESS_DIRECTION_READ		0x00
#define HIMAX_AHB_CMD_CONTI				0x31
#define HIMAX_AHB_CMD_INCR4				0x10
#define HIMAX_AHB_CMD_INCR4_ADD_4_BYTE			0x01
#define HIMAX_AHB_CMD_LEAVE_SAFE_MODE			0x0000

/* DSRAM flag addresses */
#define HIMAX_DSRAM_ADDR_2ND_FLASH_RELOAD		0x100072c0
#define HIMAX_DSRAM_ADDR_FLASH_RELOAD			0x10007f00
#define HIMAX_DSRAM_ADDR_SORTING_MODE_EN		0x10007f04
#define HIMAX_DSRAM_ADDR_SET_NFRAME			0x10007294

/* dsram flag data */
#define HIMAX_DSRAM_DATA_FW_RELOAD_DONE			0x000072c0
/* hx83121a-specific register/dsram flags/data */
#define HIMAX_HX83121A_DSRAM_ADDR_RAW_OUT_SEL		0x100072ec
#define HIMAX_HX83121A_FLASH_SIZE			(255 * 1024)

/* hardware register addresses */
#define HIMAX_REG_ADDR_RELOAD_STATUS			0x80050000
#define HIMAX_REG_ADDR_RELOAD_CRC32_RESULT		0x80050018
#define HIMAX_REG_ADDR_RELOAD_ADDR_FROM			0x80050020
#define HIMAX_REG_ADDR_RELOAD_ADDR_CMD_BEAT		0x80050028
#define HIMAX_REG_ADDR_CTRL_FW				0x9000005c
#define HIMAX_REG_ADDR_FW_STATUS			0x900000a8
#define HIMAX_REG_ADDR_ICID				0x900000d0

/* hardware reg data/flags */
#define HIMAX_REG_DATA_FW_STATE_RUNNING			0x05
#define HIMAX_REG_DATA_FW_STATE_SAFE_MODE		0x0c
#define HIMAX_REG_DATA_FW_RE_INIT			0x00
#define HIMAX_REG_DATA_FW_GO_SAFEMODE			0xa5
#define HIMAX_REG_DATA_FW_IN_SAFEMODE			0x87
#define HIMAX_REG_DATA_RELOAD_DONE			0x01
#define HIMAX_REG_DATA_RELOAD_PASSWORD			0x99

/* SPI CS setup time */
#define HIMAX_SPI_CS_SETUP_TIME				300
/* HIMAX SPI function select, 1st byte of any SPI command sequence */
#define HIMAX_SPI_FUNCTION_READ				0xf3
#define HIMAX_SPI_FUNCTION_WRITE			0xf2
/* TODO: f4 for huawei? */

union himax_dword_data {
	u32 dword;
	u16 word[2];
	u8 byte[4];
};

struct himax_ts_data {
	u8 *xfer_buf;
	u32 spi_xfer_max_sz;
	u32 xfer_buf_sz;
	/* lock for irq_save */
	spinlock_t irq_lock;
	bool irq_enabled;
	struct gpio_desc *gpiod_rst;
	struct device *dev;
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct touchscreen_properties props;
	struct drm_panel_follower panel_follower;
};

/*
 * 1st byte is the spi function select, 2nd byte is the command belong to the
 * spi function and 3rd byte is the dummy byte for IC to process the command.
 */
static int himax_spi_read(struct himax_ts_data *ts, u8 cmd, u8 *buf, u32 len)
{
	int ret;
	int retry_cnt;
	struct spi_message msg;

	memset(ts->xfer_buf, 0, HIMAX_BUS_R_HLEN + len);
	ts->xfer_buf[0] = HIMAX_SPI_FUNCTION_READ;
	ts->xfer_buf[1] = cmd;
	ts->xfer_buf[2] = 0x00;

	struct spi_transfer xfer = {
		.len = HIMAX_BUS_R_HLEN + len,
		.tx_buf = ts->xfer_buf,
		.rx_buf = ts->xfer_buf,
		.cs_change = 0,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	for (retry_cnt = 0; retry_cnt < HIMAX_BUS_RETRY; retry_cnt++) {
		ret = spi_sync(ts->spi, &msg);
		if (ret < 0)
			// TODO: 确定一般会不会出现重试的情况
			dev_err(&ts->spi->dev, "spi transfer error, %d, retry: %d", ret, retry_cnt);
		else
			break;
	}

	if (ret < 0)
		return ret;

	memcpy(buf, ts->xfer_buf + HIMAX_BUS_R_HLEN, len);
	return 0;
}

/*
 * 1st byte is the spi function select and 2nd byte is the command belong to the
 * spi function. Else is the data to write.
 */
static int himax_spi_write(struct himax_ts_data *ts, u8 *tx_buf, u32 len)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.len = len,
		.cs_change = 0,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(ts->spi, &msg);
	if (ret < 0) {
		dev_err(&ts->spi->dev, "spi transfer error, %d", ret);
		return ret;
	}

	return 0;
}

/* TODO: merge into spi_write, fix len (=len(data)) */
static int himax_write(struct himax_ts_data *ts, u8 cmd, u8 *addr, const u8 *data, u32 len)
{
	/** @len: len(data) + len(addr) */
	u8 *ptr = ts->xfer_buf;

	memset(ts->xfer_buf, 0, len + HIMAX_BUS_W_HLEN);
	ts->xfer_buf[0] = HIMAX_SPI_FUNCTION_WRITE;
	ts->xfer_buf[1] = cmd;
	ptr += HIMAX_BUS_W_HLEN;
	len += HIMAX_BUS_W_HLEN;

	if (addr) {
		memcpy(ptr, addr, 4);
		ptr += 4;
	}

	if (data)
		memcpy(ptr, data, len - (ptr - ts->xfer_buf));

	return himax_spi_write(ts, ts->xfer_buf, len);
}

/**
 * himax_mcu_set_burst_mode() - Set burst mode for MCU
 * @ts: Himax touch screen data
 * @auto_add_4_byte: Enable auto add 4 byte mode
 *
 * Set burst mode for MCU, which is used for read/write data from/to MCU.
 * HIMAX_AHB_ADDR_CONTI config the IC to take data continuously,
 * HIMAX_AHB_ADDR_INCR4 config the IC to auto increment the address by 4 byte when
 * each 4 bytes read/write.
 *
 * Return: 0 on success, negative error code on failure
 */
static int himax_mcu_set_burst_mode(struct himax_ts_data *ts, bool auto_add_4_byte)
{
	int ret;
	u8 tmp;

	tmp = HIMAX_AHB_CMD_CONTI;
	ret = himax_write(ts, HIMAX_AHB_ADDR_CONTI, NULL, &tmp, 1);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write ahb_addr_conti failed\n", __func__);
		return ret;
	}

	tmp = HIMAX_AHB_CMD_INCR4;
	if (auto_add_4_byte)
		tmp |= HIMAX_AHB_CMD_INCR4_ADD_4_BYTE;

	ret = himax_write(ts, HIMAX_AHB_ADDR_INCR4, NULL, &tmp, 1);
	if (ret < 0)
		dev_err(ts->dev, "%s: write ahb_addr_incr4 failed\n", __func__);

	return ret;
}

/*
 * Read data from himax internal registers and SRAM
 *
 * We write the (SRAM)address to AHB register to tell where to read. Then set
 * the access direction to read, and read the data from AHB register.
 */
static int himax_mcu_register_read(struct himax_ts_data *ts, u32 addr, u8 *buf, u32 len)
{
	int ret;
	u8 direction_switch = HIMAX_AHB_CMD_ACCESS_DIRECTION_READ;
	union himax_dword_data target_addr;

	ret = himax_mcu_set_burst_mode(ts, len > HIMAX_REG_SZ);
	if (ret)
		return ret;

	target_addr.dword = cpu_to_le32(addr);
	ret = himax_write(ts, HIMAX_AHB_ADDR_BYTE_0, target_addr.byte, NULL, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write ahb_addr_byte_0 failed\n", __func__);
		return ret;
	}

	ret = himax_write(ts, HIMAX_AHB_ADDR_ACCESS_DIRECTION, NULL,
				  &direction_switch, 1);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write ahb_addr_access_direction failed\n", __func__);
		return ret;
	}

	ret = himax_spi_read(ts, HIMAX_AHB_ADDR_RDATA_BYTE_0, buf, len);
	if (ret < 0) {
		dev_err(ts->dev, "%s: read ahb_addr_rdata_byte_0 failed\n", __func__);
		return ret;
	}

	return himax_mcu_set_burst_mode(ts, !(len > HIMAX_REG_SZ));
}

/* Write the internal (SRAM)address and data to AHB register */
static int himax_mcu_register_write(struct himax_ts_data *ts, u32 addr, const u8 *buf, u32 len)
{
	int ret;

	union himax_dword_data target_addr;

	ret = himax_mcu_set_burst_mode(ts, len > HIMAX_REG_SZ);
	if (ret)
		return ret;

	target_addr.dword = cpu_to_le32(addr);
	ret = himax_write(ts, HIMAX_AHB_ADDR_BYTE_0,
				target_addr.byte, buf, len + HIMAX_REG_SZ);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write ahb_addr_byte_0 failed\n", __func__);
	}

	return himax_mcu_set_burst_mode(ts, !(len > HIMAX_REG_SZ));
}

/*
 * Wakeup IC bus interface. The IC may enter sleep mode and need to wakeup
 * before any operation.
 */
static int himax_mcu_interface_on(struct himax_ts_data *ts)
{
	int ret;
	u8 buf[HIMAX_REG_SZ];
	u8 buf2[HIMAX_REG_SZ];
	u32 retry_cnt;
	const u32 burst_retry_limit = 10;

	/* Read a dummy register to wake up BUS. */
	ret = himax_spi_read(ts, HIMAX_AHB_ADDR_RDATA_BYTE_0, buf, 4);
	if (ret < 0)
		goto err;

	for (retry_cnt = 0; retry_cnt < burst_retry_limit; retry_cnt++) {
		ret = himax_mcu_set_burst_mode(ts, false);
		if (ret < 0)
			goto err;

		/* Check cmd */
		ret = himax_spi_read(ts, HIMAX_AHB_ADDR_CONTI, buf, 1);
		if (ret < 0)
			goto err;

		ret = himax_spi_read(ts, HIMAX_AHB_ADDR_INCR4, buf2, 1);
		if (ret < 0)
			goto err;

		if (buf[0] == HIMAX_AHB_CMD_CONTI && buf2[0] == HIMAX_AHB_CMD_INCR4)
			return 0;

		usleep_range(1000, 1100);
	}

err:
	dev_err(ts->dev, "%s: failed!\n", __func__);

	return -EIO;
}

static void himax_pin_reset(struct himax_ts_data *ts)
{
	/* TODO: reduce to 10ms, 20ms? */
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);
	usleep_range(20000, 20100);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);
	usleep_range(50000, 50100);
}

static void himax_int_enable(struct himax_ts_data *ts, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (enable && ts->irq_enabled == false)
		enable_irq(ts->spi->irq);
	else if (!enable && ts->irq_enabled == true)
		disable_irq_nosync(ts->spi->irq);
	ts->irq_enabled = enable;
	spin_unlock_irqrestore(&ts->irq_lock, flags);
}

static void himax_mcu_ic_reset(struct himax_ts_data *ts, bool int_off)
{
	if (int_off)
		himax_int_enable(ts, false);

	himax_pin_reset(ts);

	if (int_off)
		himax_int_enable(ts, true);
}

/**
 * sense_off: stop MCU
 * 1. request FW to stop
 * 2. enter safe mode (and reset TCON for some ICs).
 *
 * @check_en: confirm if the FW is stopped
 */
static int himax_sense_off(struct himax_ts_data *ts, bool check_en)
{
	int ret;
	u32 retry_cnt;
	const u32 stop_fw_retry_limit = 35;
	const u32 enter_safe_mode_retry_limit = 5;
	const union himax_dword_data safe_mode = {
		.dword = cpu_to_le32(HIMAX_REG_DATA_FW_GO_SAFEMODE)
	};
	union himax_dword_data data;

	dev_info(ts->dev, "%s: check %s\n", __func__, check_en ? "True" : "False");
	if (!check_en)
		goto without_check;

	for (retry_cnt = 0; retry_cnt < stop_fw_retry_limit; retry_cnt++) {
		if (retry_cnt == 0 ||
		    (data.byte[0] != HIMAX_REG_DATA_FW_GO_SAFEMODE &&
		    data.byte[0] != HIMAX_REG_DATA_FW_RE_INIT &&
		    data.byte[0] != HIMAX_REG_DATA_FW_IN_SAFEMODE)) {
			ret = himax_mcu_register_write(ts, HIMAX_REG_ADDR_CTRL_FW,
						       safe_mode.byte, 4);
			if (ret < 0) {
				dev_err(ts->dev, "%s: stop FW failed\n", __func__);
				return ret;
			}
		}
		usleep_range(10000, 11000);

		ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_FW_STATUS, data.byte, 4);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read central state failed\n", __func__);
			return ret;
		}
		if (data.byte[0] != HIMAX_REG_DATA_FW_STATE_RUNNING) {
			dev_info(ts->dev, "%s: Do not need wait FW, Status = 0x%02X!\n", __func__,
			  data.byte[0]);
			break;
		}

		ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_CTRL_FW, data.byte, 4);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read ctrl FW failed\n", __func__);
			return ret;
		}
		if (data.byte[0] == HIMAX_REG_DATA_FW_IN_SAFEMODE)
			break;
	}

	if (data.byte[0] != HIMAX_REG_DATA_FW_IN_SAFEMODE)
		dev_warn(ts->dev, "%s: Failed to stop FW!\n", __func__);

without_check:
	for (retry_cnt = 0; retry_cnt < enter_safe_mode_retry_limit; retry_cnt++) {
		/* set Enter safe mode : 0x31 ==> 0x9527 */
		data.word[0] = cpu_to_le16(HIMAX_HX83121A_SAFE_MODE_PASSWORD);
		ret = himax_write(ts, HIMAX_AHB_ADDR_PSW_LB, NULL, data.byte, 2);
		if (ret < 0) {
			dev_err(ts->dev, "%s: enter safe mode failed\n", __func__);
			return ret;
		}

		/* Check enter_save_mode */
		ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_FW_STATUS, data.byte, 4);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read central state failed\n", __func__);
			return ret;
		}

		if (data.byte[0] == HIMAX_REG_DATA_FW_STATE_SAFE_MODE) {
			dev_info(ts->dev, "%s: Safe mode entered\n", __func__);
			return 0;
		}

		usleep_range(10000, 10100); // TODO: 5ms for HX83121?
		himax_pin_reset(ts);
	}
	dev_err(ts->dev, "%s: failed!\n", __func__);

	return -EIO;
}

static int hx83102j_sense_off(struct himax_ts_data *ts, bool check_en)
{
	return himax_sense_off(ts, check_en);
}

/**
 * @sw_reset: true for software reset, false for hardware reset
 *     true: write IC to leave safe mode
 *     false: pin reset
 *
 * make MCU restart running the FW
 */
static int himax_sense_on(struct himax_ts_data *ts, bool sw_reset)
{
	int ret;
	const union himax_dword_data re_init = {
		.dword = cpu_to_le32(HIMAX_REG_DATA_FW_RE_INIT)
	};
	union himax_dword_data data;

	ret = himax_mcu_interface_on(ts);
	if (ret < 0)
		return ret;

	ret = himax_mcu_register_write(ts, HIMAX_REG_ADDR_CTRL_FW, re_init.byte, 4);
	if (ret < 0)
		return ret;
	usleep_range(10000, 11000);
	if (!sw_reset) {
		himax_mcu_ic_reset(ts, false);
	} else {
		data.word[0] = cpu_to_le16(HIMAX_AHB_CMD_LEAVE_SAFE_MODE);
		ret = himax_write(ts, HIMAX_AHB_ADDR_PSW_LB, NULL, data.byte, 2);
		if (ret < 0)
			return ret;
	}
	/* TODO: more operations for other himax ICs */

	return 0;
}

static int hx83121a_chip_detect(struct himax_ts_data *ts)
{
	int ret;
	u32 retry_cnt;
	const u32 read_icid_retry_limit = 5;
	union himax_dword_data data;

	himax_pin_reset(ts);

	ret = himax_mcu_interface_on(ts);
	if (ret < 0) {
		dev_err(ts->dev, "%s: read ahb_addr_conti failed\n", __func__);
		return ret;
	}

	ret = hx83102j_sense_off(ts, false);
	if (ret)
		return ret;

	for (retry_cnt = 0; retry_cnt < read_icid_retry_limit; retry_cnt++) {
		ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_ICID, data.byte, 4);
		if (ret) {
			dev_err(ts->dev, "%s: Read IC ID Fail\n", __func__);
			return ret;
		}

		/*
		 * For suffix > F, it should be (data.byte[1] & 0xF) + 'A'; // or 'a'
		 * e.g. hx83102j: 0x83102900
		 */
		data.dword = le32_to_cpu(data.dword) >> 8;
		dev_info(ts->dev, "Detected IC HX%06X\n", data.dword);

		if (data.dword == 0x83121a)
			return 0;
	}
	return -ENODEV;
}

/* -------------------------------------------------------------------------- */
/* input 子系统 */

static int himax_input_dev_config(struct himax_ts_data *ts)
{
	struct input_dev *input_dev;
	int ret;

	input_dev = devm_input_allocate_device(ts->dev);
	if (!input_dev)
		return -ENOMEM;

	ts->input_dev = input_dev;
	input_set_drvdata(input_dev, ts);

	input_dev->name = "Himax Capacitive TouchScreen";
	input_dev->phys = "input/ts";
	input_dev->id.bustype = BUS_SPI;

	/* Standard capacitive touchscreen fuzz (8) to absorb static finger deformation
	 * during clicks, preventing libinput from treating 10px drifts as swipes. */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, SZ_64K - 1, 8, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, SZ_64K - 1, 8, 0);
	// TODO: 根据触点附近数据集建模得到
	// input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 4095, 0, 0);
	touchscreen_parse_properties(ts->input_dev, true, &ts->props);

	ret = input_mt_init_slots(ts->input_dev, HIMAX_MAX_TOUCH,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED|
				  INPUT_MT_TRACK);
	if (ret)
		return ret;

	ret = input_register_device(ts->input_dev);
	if (ret)
		return ret;

	return 0;
}

static void himax_report_state(struct himax_ts_data *ts,
			       struct input_mt_pos *pos,
			       u16 *strengths,
			       int touch_num)
{
	int i, slots[HIMAX_MAX_TOUCH];

	/*
	 * Although it is continuous, without transformation first, in-kernel
	 * MT track won't be happy.
	 */
	for (i = 0; i < touch_num; i++) {
		touchscreen_set_mt_pos(&pos[i], &ts->props, pos[i].x, pos[i].y);
	}

	/* Set dmax=400 to stably track fingers up to 4.5cm per frame */
	int ret = input_mt_assign_slots(ts->input_dev, slots, pos, touch_num, 400);
	if (ret < 0) {
		dev_warn_ratelimited(ts->dev, "input_mt_assign_slots failed: %d\n", ret);
		return;
	}

	for (i = 0; i < touch_num; i++) {
		if (slots[i] >= HIMAX_MAX_TOUCH)
			continue;
		input_mt_slot(ts->input_dev, slots[i]);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, pos[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pos[i].y);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, strengths[i]);
		// input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, );
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/* -------------------------------------------------------------------------- */
/* 中断处理 */

static int hx83121a_gaokun_read_event_stack(struct himax_ts_data *ts)
{
	u32 i;
	int ret;
	const u32 max_trunk_sz = ts->spi_xfer_max_sz - HIMAX_BUS_R_HLEN;
	u8 *buf = ts->xfer_buf;

	memset(ts->xfer_buf, 0x00, ts->xfer_buf_sz);
	size_t length = HIMAX_HX83121A_FULL_STACK_SZ; /* FIXME: use actual size. */

	for (i = 0; i < length; i += max_trunk_sz) {
		ret = himax_spi_read(ts, HIMAX_AHB_ADDR_EVENT_STACK, buf + i,
				 min(length - i, max_trunk_sz));
		if (ret) {
			dev_err(ts->dev, "%s: read event stack error!\n", __func__);
			return ret;
		}
	}

	return 0;
}

#define EQUILIBRIUM	0x8000
#define DOWN_THRESHOLD	0x100
#define UP_THRESHOLD	0x1000
#define VALID(val) ((EQUILIBRIUM + DOWN_THRESHOLD < val ) & (val < EQUILIBRIUM + UP_THRESHOLD))

static u16 simple_filter(int val)
{
	/* 触控区貌似只会大于 EQUILIBRIUM (0x8000) */
	return VALID(val) ? val - EQUILIBRIUM : 0;
}

/* TODO: 可以创建一个 sysfs 用于和用户空间 IO, 分析, 处理, 数据 等等 */
static void dump_frame(u16 *ptr, bool raw)
{
	char buf[1024];

	pr_warn("Frame start\n");
	for (int i = 0, offset; i < 2400 + 60; i++) { // 60 x 40 + 60
		if ( i % 60 == 0) {
			if (i)
				pr_info("%s\n", buf);

			offset = sprintf(buf, "%04x:", i);
		}
		offset += sprintf(buf + offset, " %04x", raw ? ptr[i] : simple_filter(ptr[i]));
	}
}

/*
 * array 2D to 1D 且扩展处理边界外一圈
 * 即不在 tx_max * rx_max 范围内返回 0
 */
#define ARRACC(arr, tx, rx) le16_to_cpup(arr + tx * HIMAX_MAX_RX + rx)
static u16 dd2d(u16 *arr, int tx, int rx)
{
	return (tx >= 0) & (tx < HIMAX_MAX_TX) & (rx >= 0) & (rx < HIMAX_MAX_RX)
	       ? simple_filter(ARRACC(arr, tx, rx)) : 0;
}

#define OFST 4

#define HIMAX_MAX_PEAK_CANDIDATES 30

struct himax_peak {
	s16 x, y;
	u16 strength;
};

static int himax_peak_cmp_desc(const void *a, const void *b)
{
	return (int)((const struct himax_peak *)b)->strength -
	       (int)((const struct himax_peak *)a)->strength;
}

#define DIST(pos0, pos1) \
( abs((pos0)->x - (pos1)->x) + abs((pos0)->y - (pos1)->y) )

static void raw2rxtx(u16 *arr, struct input_mt_pos *pos, int *cnt)
{
	struct himax_peak candidates[HIMAX_MAX_PEAK_CANDIDATES];
	int ncand = 0;
	int i, j, k;
	u16 vc, vl, vr, vu, vd, vlu, vld, vru, vrd;
	*cnt = 0;

	/* 收集所有局部极大值（最多30个候选） 30 = 10 指单聚点三峰 */
	for (i = 0; i < HIMAX_MAX_TX; ++i) {
		for (j = 0; j < HIMAX_MAX_RX; ++j) {
			if (ncand >= HIMAX_MAX_PEAK_CANDIDATES)
				goto sort;

			vc = dd2d(arr, i, j);
			if (!vc)
				continue;

			vl = dd2d(arr, i, j - 1);
			vr = dd2d(arr, i, j + 1);
			vu = dd2d(arr, i - 1, j);
			vd = dd2d(arr, i + 1, j);

			vlu = dd2d(arr, i - 1, j - 1);
			vld = dd2d(arr, i + 1, j - 1);
			vru = dd2d(arr, i - 1, j + 1);
			vrd = dd2d(arr, i + 1, j + 1);

			if (vc >= vl && vc >= vr && vc >= vu && vc >= vd &&
			    vc >= vlu && vc > vru && vc >= vld && vc >= vrd) {
				/* horizontal rx: j-idx, vertical tx: i-idx */
				candidates[ncand].y = i;
				candidates[ncand].x = j;
				candidates[ncand].strength = vc;
				ncand++;
				j++; /* the right one must not be a peak */
			}
		}
	}

sort:
	if (ncand == 0)
		return;

	/* 按强度降序排列：最强的峰优先 */
	sort(candidates, ncand, sizeof(*candidates), himax_peak_cmp_desc, NULL);

	/*
	 * 从最强开始取，跳过与已选点距离 < 3 的候选（近邻去重）。
	 * 简单起见, 这里用 L1 距离
	 */
	for (i = 0; i < ncand && *cnt < HIMAX_MAX_TOUCH; i++) {
		bool too_close = false;

		for (k = 0; k < *cnt; k++) {
			if (DIST(candidates + i, pos + k) < 3) {
				too_close = true;
				break;
			}
		}

		if (!too_close) {
			pos[*cnt].x = candidates[i].x;
			pos[*cnt].y = candidates[i].y;
			(*cnt)++;
		}
	}
}

/* generate numerator items */
#define GEN_N(v1, v2, v3, n)	\
do {				\
	n = 3 * v3 + v2 - v1;	\
} while (0)

static int rxtx2xy(u16 *arr, struct input_mt_pos *pos, u16 *strengths_out, int cnt)
{
	int i, out = 0, temp;
	int tx, rx;
	u32 vl, vr, vc, vu, vd, vlu, vld, vru, vrd;
	int n1, n2, n3, m;
	int dx, dy, scale_x, scale_y;

	/*
	 * Weighted average, tweaked to reduce multiplication operations
	 * and round error
	 * FIXME: since zero weight for outer boundary, active windows
	 * is only central (x, y) =
	 * (2560/60/2, 2560 - 2560/60/2) x (1600/40/2, 1660 - 1600/40/2)
	 */
	for (i = 0; i < cnt; ++i) {
		tx = pos[i].y;
		rx = pos[i].x;
		vc = dd2d(arr, tx, rx);

		/*
		 * 弱触点跳过，不写入输出，有效计数不增
		 * 如果不过滤这些点, 会导致指甲也能触发, 且滑动轨迹测试时会剧烈抖动
		 * (画直线, 看点散落的轨迹图, 两个触控通道间来回拉扯, 尤其是加大接触面)
		 */
		if (vc < 800)
			continue;

		scale_x = 3;
		scale_y = 1;
		dx = scale_x * 2560 / HIMAX_MAX_RX;
		dy = scale_y * 1600 / HIMAX_MAX_TX;

		vl = dd2d(arr, tx, rx - 1);
		vr = dd2d(arr, tx, rx + 1);
		vu = dd2d(arr, tx - 1, rx);
		vd = dd2d(arr, tx + 1, rx);

#ifdef FIVE_POINT
		GEN_N(vl, vc, vr, n2);
		m = vl + vr + vc;
		temp = dx * rx * m + dx / 2 * n2;
		pos[out].x = temp / (scale_x * m);

		GEN_N(vu, vc, vd, n2);
		m = vu + vc + vd;
		pos[out].y = dy * tx + dy / 2 * n2 / (scale_y * m);
#endif

		vlu = dd2d(arr, tx - 1, rx - 1);
		vld = dd2d(arr, tx + 1, rx - 1);
		vru = dd2d(arr, tx - 1, rx + 1);
		vrd = dd2d(arr, tx + 1, rx + 1);

		m = vl + vr + vc + vu + vd + vlu + vld + vru + vrd;

		GEN_N(vlu, vu, vru, n1);
		GEN_N(vl, vc, vr, n2);
		GEN_N(vld, vd, vrd, n3);
		temp = dx * rx * m + dx / 2 * (n1 + n2 + n3);
		pos[out].x = temp / (scale_x * m);

		GEN_N(vlu, vl, vld, n1);
		GEN_N(vu, vc, vd, n2);
		GEN_N(vru, vr, vrd, n3);
		pos[out].y = dy * tx + dy / 2 * (n1 + n2 + n3) / (scale_y * m);

		strengths_out[out] = vc;

		pr_debug("calculate x=%d, y=%d (rx=%d, tx=%d, vc=%u(vl=%u, vr=%u, vu=%u, vd=%u))\n",
			 pos[out].x, pos[out].y, rx, tx, vc, vl, vr, vu, vd);
		out++;
	}
	return out;
}

static void get_m(u16 *arr, u32 *val, u32 *tx, u32 *rx, bool max)
{
	u16 tmp = EQUILIBRIUM, ti, tj, walk, i, j;

	for (i = 0; i < HIMAX_MAX_TX; ++i)
		for (j = 0; j < HIMAX_MAX_RX; ++j) {
			walk = ARRACC(arr, i, j);
			if ( (walk > tmp) * max + (walk < tmp) * !max ) {
				tmp = walk;
				ti = i;
				tj = j;
			}
		}

	*val = tmp;
	*tx = ti;
	*rx = tj;
}

static bool fast_check(u16 *arr)
{
	u32 tx, rx, val;
	/* 屏幕右下角的最后一个触控通道 永远是 0x0000, 重置成默认值, 方便后续的处理 */
	arr[0] = EQUILIBRIUM;

	get_m(arr, &val, &tx, &rx, 1);
	/* Drop the frame if the max is an isolated point or out of bound */
	return VALID(val) && (dd2d(arr, tx - 1, rx) || dd2d(arr, tx + 1, rx) ||
	       dd2d(arr, tx, rx - 1) || dd2d(arr, tx, rx + 1));
}

static irqreturn_t himax_ts_thread(int irq, void *data)
{
	struct himax_ts_data *ts = data;
	u16 *ptr = (void *)ts->xfer_buf + OFST;
	struct input_mt_pos pos[HIMAX_MAX_TOUCH];
	u16 final_strengths[HIMAX_MAX_TOUCH];
	int cnt = 0;

	if (hx83121a_gaokun_read_event_stack(ts)) {
		dev_err(ts->dev, "failed to get touch data!\n");
		himax_mcu_ic_reset(ts, true);
		return IRQ_NONE;
	}

	/*
	 * TODO: 3 in and 3 out: for the stability, report when continuous
	 * 3 touch down events, release when 3 touch up events.
	 */
	if (fast_check((void *)ptr)) {
		raw2rxtx(ptr, pos, &cnt);
		cnt = rxtx2xy(ptr, pos, final_strengths, cnt);
	}

	himax_report_state(ts, pos, final_strengths, cnt);
	return IRQ_HANDLED;
}

static int himax_mcu_assign_sorting_mode(struct himax_ts_data *ts, u8 *tmp_data_in)
{
	int ret;
	u8 rdata[4];
	u32 retry_cnt;
	const u32 retry_limit = 3;

	for (retry_cnt = 0; retry_cnt < retry_limit; retry_cnt++) {
		ret = himax_mcu_register_write(ts, HIMAX_DSRAM_ADDR_SORTING_MODE_EN,
					       tmp_data_in, HIMAX_REG_SZ);
		if (ret < 0) {
			dev_err(ts->dev, "%s: write sorting mode fail\n", __func__);
			return ret;
		}
		usleep_range(1000, 1100);
		ret = himax_mcu_register_read(ts, HIMAX_DSRAM_ADDR_SORTING_MODE_EN,
					      rdata, HIMAX_REG_SZ);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read sorting mode fail\n", __func__);
			return ret;
		}

		if (!memcmp(tmp_data_in, rdata, HIMAX_REG_SZ))
			return 0;
	}
	dev_err(ts->dev, "%s: fail to write sorting mode\n", __func__);

	return -EINVAL;
}

/*
 * Tell FW not to reload data from flash. It needs to be
 * set before FW start running.
 */
static int himax_disable_fw_reload(struct himax_ts_data *ts, bool disable)
{
	union himax_dword_data data = {
		/*
		 * HIMAX_DSRAM_ADDR_FLASH_RELOAD: 0x10007f00
		 * 0x10007f00 <= 0x9aa9, let FW know there's no flash
		 *            <= 0x5aa5, there has flash, but not reload
		 *            <= 0x0000, there has flash, and reload
		 */
		.dword = cpu_to_le32(disable ? 0x5aa5 : 0x0000) // TODO: handle it for zf
	};

	return himax_mcu_register_write(ts, HIMAX_DSRAM_ADDR_FLASH_RELOAD, data.byte, 4);
}

static int himax_mcu_power_on_init(struct himax_ts_data *ts)
{
	int ret;
	u32 retry_cnt;
	const u32 retry_limit = 30;
	union himax_dword_data data;

	data.dword = cpu_to_le32(HIMAX_DATA_CLEAR);
	/* Initial sorting mode password to normal mode */
	ret = himax_mcu_assign_sorting_mode(ts, data.byte); // 不必要的步骤对于 gaokun3
	if (ret < 0) {
		dev_err(ts->dev, "%s: assign sorting mode fail\n", __func__);
		return ret;
	}

	/* N frame initial */ // 不必要的步骤对于 gaokun3
	/* reset N frame back to default value 1 for normal mode */
	data.dword = cpu_to_le32(1);
	ret = himax_mcu_register_write(ts, HIMAX_DSRAM_ADDR_SET_NFRAME, data.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: set N frame fail\n", __func__);
		return ret;
	}

	/* Initial FW reload status */ // 必要的步骤
	data.dword = cpu_to_le32(HIMAX_DATA_CLEAR);
	ret = himax_mcu_register_write(ts, HIMAX_DSRAM_ADDR_2ND_FLASH_RELOAD, data.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: initial FW reload status fail\n", __func__);
		return ret;
	}

	ret = himax_sense_on(ts, false); // 必要的步骤
	if (ret < 0) {
		dev_err(ts->dev, "%s: sense on fail\n", __func__);
		return ret;
	}

	dev_info(ts->dev, "%s: waiting for FW reload data\n", __func__);
	for (retry_cnt = 0; retry_cnt < retry_limit; retry_cnt++) {
		ret = himax_mcu_register_read(ts, HIMAX_DSRAM_ADDR_2ND_FLASH_RELOAD, data.byte, 4);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read FW reload status fail\n", __func__);
			return ret;
		}

		// use all 4 bytes to compare
		if (le32_to_cpu(data.dword) == HIMAX_DSRAM_DATA_FW_RELOAD_DONE) {
			dev_info(ts->dev, "%s: FW reload done\n", __func__);
			break;
		}
		dev_info(ts->dev, "%s: wait FW reload %u times\n", __func__, retry_cnt + 1);

		usleep_range(10000, 11000);
	}

	if (retry_cnt == retry_limit) {
		dev_err(ts->dev, "%s: FW reload fail!\n", __func__);
		return -EINVAL;
	}

	/* RawOut select set, must be 0xf6, fuck you huawei */
	data.dword = cpu_to_le32(0xf6);
	ret = himax_mcu_register_write(ts, HIMAX_HX83121A_DSRAM_ADDR_RAW_OUT_SEL, data.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: set RawOut select fail\n", __func__);
		return ret;
	}

	return 0;
}

static int himax_mcu_check_crc(struct himax_ts_data *ts, u32 start_addr,
			       int reload_length, u32 *crc_result)
{
	int ret;
	int length = reload_length / HIMAX_REG_SZ;
	u32 retry_cnt;
	const u32 retry_limit = 100;
	union himax_dword_data data, addr;

	addr.dword = cpu_to_le32(start_addr);
	ret = himax_mcu_register_write(ts, HIMAX_REG_ADDR_RELOAD_ADDR_FROM, addr.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write reload start address fail\n", __func__);
		return ret;
	}

	data.word[1] = cpu_to_le16(HIMAX_REG_DATA_RELOAD_PASSWORD);
	data.word[0] = cpu_to_le16(length);
	ret = himax_mcu_register_write(ts, HIMAX_REG_ADDR_RELOAD_ADDR_CMD_BEAT, data.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: write reload length and password fail!\n", __func__);
		return ret;
	}

	ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_RELOAD_ADDR_CMD_BEAT, data.byte, 4);
	if (ret < 0) {
		dev_err(ts->dev, "%s: read reload length and password fail!\n", __func__);
		return ret;
	}

	if (le16_to_cpu(data.word[0]) != length) {
		dev_err(ts->dev, "%s: length verify failed!\n", __func__);
		return -EINVAL;
	}

	for (retry_cnt = 0; retry_cnt < retry_limit; retry_cnt++) {
		ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_RELOAD_STATUS, data.byte, 4);
		if (ret < 0) {
			dev_err(ts->dev, "%s: read reload status fail!\n", __func__);
			return ret;
		}

		data.dword = le32_to_cpu(data.dword);
		if ((data.byte[0] & HIMAX_REG_DATA_RELOAD_DONE) != HIMAX_REG_DATA_RELOAD_DONE) {
			ret = himax_mcu_register_read(ts, HIMAX_REG_ADDR_RELOAD_CRC32_RESULT,
						      data.byte, HIMAX_REG_SZ);
			if (ret < 0) {
				dev_err(ts->dev, "%s: read crc32 result fail!\n", __func__);
				return ret;
			}
			*crc_result = le32_to_cpu(data.dword);
			return 0;
		}

		dev_info(ts->dev, "%s: Waiting for HW ready!\n", __func__);
		usleep_range(1000, 1100);
	}

	dev_err(ts->dev, "%s: read FW status fail\n", __func__);
	return -EINVAL;
}

static int himax_spi_panel_follower_resume(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);
	msleep(50);
	himax_disable_fw_reload(ts, 0); /* 0: follow huawei driver */
	himax_mcu_power_on_init(ts);
	himax_int_enable(ts, true);
	return 0;
}

/* Swipe would not work then, must rebind to recover totally? */
static int himax_spi_panel_follower_suspend(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);
	himax_int_enable(ts, false);
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);
	return 0;
}

static const struct
drm_panel_follower_funcs himax_spi_panel_follower_prepare_funcs = {
	.panel_prepared = himax_spi_panel_follower_resume,
	.panel_unpreparing = himax_spi_panel_follower_suspend,
};

/* -------------------------------------------------------------------------- */
static int himax_spi_probe(struct spi_device *spi)
{
	int ret;
	struct himax_ts_data *ts;
	u32 crc_hw;

	ts = devm_kzalloc(&spi->dev, sizeof(struct himax_ts_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->dev = &spi->dev;

	ts->gpiod_rst = devm_gpiod_get_optional(ts->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->gpiod_rst)) {
		dev_err(ts->dev, "%s: gpio-rst value is not valid\n", __func__);
		return -EIO;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->cs_setup.value = HIMAX_SPI_CS_SETUP_TIME;

	ts->spi = spi;
	ts->spi_xfer_max_sz = HIMAX_HX83121A_FULL_STACK_SZ;
	ts->xfer_buf_sz = ts->spi_xfer_max_sz;
	ts->xfer_buf = devm_kzalloc(ts->dev, ts->xfer_buf_sz, GFP_KERNEL);
	if (!ts->xfer_buf)
		return -ENOMEM;

	spin_lock_init(&ts->irq_lock);
	dev_set_drvdata(&spi->dev, ts);
	spi_set_drvdata(spi, ts);

	ret = hx83121a_chip_detect(ts);
	if (ret) {
		dev_err(ts->dev, "%s: IC detect failed\n", __func__);
		return ret;
	}

	ret = himax_mcu_check_crc(ts, 0, HIMAX_HX83121A_FLASH_SIZE, &crc_hw);
	if (ret || crc_hw) {
		dev_err(ts->dev, "hw crc failed, fw broken, fix it on windows\n");
		return ret;
	}

	ret = himax_input_dev_config(ts);
	if (ret) {
		dev_err(ts->dev, "input device set failed\n");
		return ret;
	}

	ret = devm_request_threaded_irq(ts->dev, ts->spi->irq, NULL,
					himax_ts_thread, IRQF_ONESHOT |
					IRQF_NO_AUTOEN, "himax-spi-ts", ts);
	if (ret) {
		dev_err(ts->dev, "request irq failed. ret=%d\n", ret);
		return ret;
	}

	ts->panel_follower.funcs = &himax_spi_panel_follower_prepare_funcs;
	return devm_drm_panel_add_follower(ts->dev, &ts->panel_follower);
}

static const struct spi_device_id himax_spi_ids[] = {
	{ .name = "hx83121a-ts" },
	{ },
};
MODULE_DEVICE_TABLE(spi, himax_spi_ids);

static const struct of_device_id himax_spi_of_match[] = {
	{ .compatible = "himax,hx83121a-ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, himax_spi_of_match);

static struct spi_driver himax_spi_driver = {
	.driver = {
		.name = "himax-spi",
		.of_match_table = himax_spi_of_match,
	},
	.probe = himax_spi_probe,
	.id_table = himax_spi_ids,
};
module_spi_driver(himax_spi_driver);

MODULE_LICENSE("GPL");
