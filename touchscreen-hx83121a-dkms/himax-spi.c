// SPDX-License-Identifier: GPL-2.0
/*
 * Himax SPI Driver
 * based on Himax hx83102j
 *
 * Copyright (C) 2019,2024 Himax Corporation.
 * Copyright (C) 2026 Pengyu Luo <mitltlatltl@gmail.com>
 */

/* TODO: DT parse: vdd_dig & avdd_analog */

#define DEBUG
#include<linux/dev_printk.h>

#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/math.h>

#define HIMAX_BUS_RETRY					3
/* SPI bus read header length */
#define HIMAX_BUS_R_HLEN				3U
/* SPI bus write header length */
#define HIMAX_BUS_W_HLEN				2U
/* TP SRAM address size and data size */
#define HIMAX_REG_SZ					4U
#define HIMAX_MAX_RX					60U
#define HIMAX_MAX_TX					40U
#define HIMAX_HX83121A_SAFE_MODE_PASSWORD		0x9527
/* FIXME: this is for hx83120j */
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
	unsigned long update_time;
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
	gpiod_set_value(ts->gpiod_rst, 1);
	usleep_range(20000, 20100);
	gpiod_set_value(ts->gpiod_rst, 0);
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
	const u32 ic_id_mask = GENMASK(31, 8);
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

		data.dword = le32_to_cpu(data.dword);
		/*
		 * For suffix > F, it should be (data.byte[1] & 0xF) + 'A'; // or 'a'
		 * e.g. hx83102j: 0x83102900
		 */
		dev_info(ts->dev, "Detected IC HX%X%X%X\n", data.byte[3], data.byte[2], data.byte[1]);

		if ((data.dword & ic_id_mask) == 0x83121a00)
			return 0;
	}
	return -ENODEV;
}

/* -------------------------------------------------------------------------- */
/* input 子系统 */
#define HIMAX_MAX_TOUCH 10
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

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, SZ_64K - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, SZ_64K - 1, 0, 0);
	// TODO: 根据触点附近数据集建模得到
	// input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input_dev, true, &ts->props);

	ret = input_mt_init_slots(ts->input_dev, HIMAX_MAX_TOUCH,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(ts->input_dev);
	if (ret)
		return ret;

	return 0;
}

static void himax_report_state(struct himax_ts_data *ts,
			       struct input_mt_pos *pos,
			       int touch_num)
{
	int i, slots[HIMAX_MAX_TOUCH];

	input_mt_assign_slots(ts->input_dev, slots, pos, touch_num, 0);
	for (i = 0; i < touch_num; i++) {
		input_mt_slot(ts->input_dev, slots[i]);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

		touchscreen_report_pos(ts->input_dev, &ts->props,
				       pos[i].x, pos[i].y, true);
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
#define THRESHOLD	0xa0 /* casual, 拉高有助于防止乱跳, 代价是灵敏度降低 */
static u16 simple_filter(int val)
{
	/* 触控区貌似只会大于 EQUILIBRIUM, 如果是稳定的数据, 只会出现 0x8000 和 0x8000+ */
	int out = abs(val - EQUILIBRIUM);
	return out < THRESHOLD ? 0 : out;
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
static inline u16 dd2d(u16 *arr, int tx, int rx)
{
	return (tx >= 0) & (tx < HIMAX_MAX_TX) & (rx >= 0) & (rx < HIMAX_MAX_RX)
	       ? simple_filter(le16_to_cpup(arr + tx * HIMAX_MAX_RX + rx)) : 0;
}

#define DIS(pos0, pos1) \
( abs(pos0->x - pos1->x) + abs(pos0->y - pos1->y) )

#define REPLACE(dst, src) \
*dst = *src;

static bool is_new_point(u16 *arr, struct input_mt_pos *pos, int cnt)
{
	struct input_mt_pos *pos0 = pos + cnt;
	u16 val = dd2d(arr, pos0->y, pos0->x);
	struct input_mt_pos *posi;
	int i;

	if (!val)
		return false;

	/* 检查 pos0 周围有没有点, 有的话保留 rawdata 更大那一个 */
	for (i = cnt - 1; i >= 0; --i) {
		posi = pos + i;

		/* 存在之前记录的点与 '新点' 过于接近, 二选一, 保留 rawdata 更大的那个 */
		if (DIS(pos0, posi) < 3) {
			if (val > dd2d(arr, posi->y, posi->x))
				REPLACE(posi, pos0); // TODO: 处理替换后与其他点距离小于3的情况?
			return false;
		}
	}

	/*
	 * 与所有点都距离大于 2, 是新点, 该点早已被插入, 只要标记 true 返回即可
	 * cnt == 0 时也会直接 true
	 */
	return true;
}

#define OFST 4
static void raw2rxtx(u16 *arr, int tx_max, int rx_max,
		     struct input_mt_pos *pos, int *cnt)
{
	int i, j;

	*cnt = 0;
	for (i = 0; i < tx_max; ++i)
		for (j = 0; j < rx_max; ++j){
			if (*cnt >= HIMAX_MAX_TOUCH) {
				dump_frame((void *)arr - OFST, true);
				pr_warn("%s: more then 10 contact points\n", __func__);
				break;
			}

			/* 水平方向为 rx, 垂直为 tx */
			pos[*cnt].x = j;
			pos[*cnt].y = i;
			*cnt += is_new_point(arr, pos, *cnt);
		}
}

static void rxtx2xy(u16 *arr, struct input_mt_pos *pos, int cnt)
{
	int i, tx, rx;
	u16 vl, vr, vc, vu, vd;
	unsigned long tmp;

	/*
	 * 四点权重拟合. 这里以 左 中 右 三点为例, 更进一步我们再以 中 右 为例,
	 * 假设当前在中间, 则向右偏移 val_右/(val_中 + val_右) * (两邻点水平间距)
	 * 同理处理左移, 上移, 下移.
	 * TODO: 参考更多点的权值.
	 */
	for (i = 0; i < cnt; ++i) {
		tx = pos[i].y;
		rx = pos[i].x;
		vc = dd2d(arr, tx, rx);
		vl = dd2d(arr, tx, rx - 1);
		vr = dd2d(arr, tx, rx + 1);
		vu = dd2d(arr, tx - 1, rx);
		vd = dd2d(arr, tx + 1, rx);
		tmp = (vc + vr) * (vc + vl);
		pos[i].x = 64 * ( (2 * rx + 1) * tmp + 2 * vc * (vr - vl) ) / (3 * tmp);
		tmp = (vc + vu) * (vc + vd);
		pos[i].y = 20 * ( (2 * tx + 1) * tmp + 2 * vc * (vd - vu) ) / tmp;
		pr_debug("calculate x: 64 * ( (2 * %d + 1) * %lu + 2 * %d * (%d - %d) ) / (3 * %lu) = %d\n",
			 rx, tmp, vc, vr, vl, tmp, pos[i].x);
		pr_debug("calculate y: 20 * ( (2 * %d + 1) * %lu + 2 * %d * (%d - %d) ) / %lu = %d\n",
			 tx, tmp, vc, vd, vu, tmp, pos[i].y);
	}
}

static irqreturn_t himax_ts_thread(int irq, void *data)
{
	struct himax_ts_data *ts = data;
	u16 *ptr = (void *)ts->xfer_buf + OFST;
	struct input_mt_pos pos[HIMAX_MAX_TOUCH];
	int cnt;

	/*
	 * 指定 若干毫秒 最多触发1次, 调试用可设置为 500, 1000 减少日志刷屏, 但必要时保持
	 * 手指触摸以避免漏掉数据. 至少请设置为 10, 100hz 较为接近实际刷新率, 不设置会发生
	 * 数据错误.
	 */
	if (time_before(jiffies, ts->update_time + msecs_to_jiffies(20))) {
		drop_cnt++;
		return IRQ_HANDLED;
	}

	if (hx83121a_gaokun_read_event_stack(ts)) {
		dev_err(ts->dev, "failed to get touch data!\n");
		himax_mcu_ic_reset(ts, true);
		return IRQ_NONE;
	}

	// dump_frame((void *)ptr - OFST, true);

	/* 右下角的最后一个触控通道 永远是 0x0000, 重置成默认值, 方便后续的处理 */
	ptr[0] = EQUILIBRIUM;
	raw2rxtx(ptr, HIMAX_MAX_TX, HIMAX_MAX_RX, pos, &cnt);
	for (int i = 0; i < cnt; ++i) {
		dev_dbg(ts->dev, "rx-tx %d: %d, %d\n", i, pos[i].x, pos[i].y);
	}

	rxtx2xy(ptr, pos, cnt);
	/* Not final results, touchscreen_report_pos will handle this (x-y swap, y invert) */
	for (int i = 0; i < cnt; ++i) {
		dev_dbg(ts->dev, "x-y %d: %d, %d\n", i, pos[i].x, pos[i].y);
	}

	/* 这里报告给系统的坐标数据可以通过 evtest 查看 */
	himax_report_state(ts, pos, cnt);

	ts->update_time = jiffies;

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
static int himax_disable_fw_reload(struct himax_ts_data *ts)
{
	union himax_dword_data data = {
		/*
		 * HIMAX_DSRAM_ADDR_FLASH_RELOAD: 0x10007f00
		 * 0x10007f00 <= 0x9aa9, let FW know there's no flash
		 *            <= 0x5aa5, there has flash, but not reload
		 *            <= 0x0000, there has flash, and reload
		 */
		.dword = cpu_to_le32(0x5aa5) // TODO: handle it for zf
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
					himax_ts_thread, IRQF_ONESHOT,
					"himax-spi-ts", ts);
	if (ret) {
		dev_err(ts->dev, "request irq failed. ret=%d\n", ret);
		return ret;
	}
	ts->irq_enabled = true;
	himax_int_enable(ts, false);
	himax_disable_fw_reload(ts);
	himax_mcu_power_on_init(ts);
	himax_int_enable(ts, true);
	return ret;
}

static const struct spi_device_id himax_spi_ids[] = {
	{ .name = "hx83121a" },
	{ },
};
MODULE_DEVICE_TABLE(spi, himax_spi_ids);

static const struct of_device_id himax_spi_of_match[] = {
	{ .compatible = "himax,hx83121a" },
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
