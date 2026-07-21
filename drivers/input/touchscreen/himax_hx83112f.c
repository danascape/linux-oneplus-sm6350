// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Himax HX83112F "no-flash" touchscreens (SPI)
 *
 * Copyright (C) 2026 Saalim Quadri <danascape@gmail.com>
 *
 * Minimal bring-up: Himax SPI framing, the AHB register model, controller
 * reset and safe-mode entry, IC-ID detection, and a type-B multitouch input
 * device driven by a threaded IRQ. The HX83112F has no on-chip flash, so it
 * does not report touch until its firmware is downloaded into SRAM; that is
 * added in later changes.
 *
 * The register/event handling is derived from the mainline himax_hx83112b
 * driver by Job Noorman.
 *
 * Back-ported to the 4.19 downstream OnePlus/SM6350 tree (no cleanup.h scope
 * guards, devm_mutex_init() or modern PM_OPS helpers); binds the existing
 * "himax,hxcommon" device node in place of the vendor framework driver.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#define HIMAX_MAX_POINTS		10

/* Himax SPI framing: a 2-byte command header precedes every AHB access. */
#define HIMAX_SPI_WRITE			0xf2
#define HIMAX_SPI_READ			0xf3
/* header(2) + AHB address(4) + max payload per transfer */
#define HIMAX_SRAM_CHUNK		240
#define HIMAX_XFER_MAX			(2 + 4 + HIMAX_SRAM_CHUNK)

/* AHB (register bus) sub-addresses carried in the SPI command byte */
#define HIMAX_AHB_ADDR_BYTE_0		0x00
#define HIMAX_AHB_ADDR_RDATA_BYTE_0	0x08
#define HIMAX_AHB_ADDR_ACCESS_DIR	0x0c
#define HIMAX_AHB_ADDR_INCR4		0x0d
#define HIMAX_AHB_ADDR_BURST_READ	0x11
#define HIMAX_AHB_ADDR_CONTI		0x13
#define HIMAX_AHB_ADDR_EVENT_STACK	0x30
#define HIMAX_AHB_ADDR_RST_0		0x31
#define HIMAX_AHB_ADDR_RST_1		0x32

#define HIMAX_AHB_CMD_ACCESS_DIR_READ	0x00
#define HIMAX_AHB_CMD_INCR4		0x10
#define HIMAX_AHB_CMD_CONTI		0x31
#define HIMAX_AHB_CMD_BURST_READ_OFF	0x00
#define HIMAX_AHB_CMD_BURST_READ_ON	0x01

/* 32-bit register addresses accessed over the AHB bus */
#define HIMAX_REG_ICID			0x900000d0
#define HIMAX_ICID_HX83112F		0x83112f

#define HIMAX_REG_SAFE_MODE		0x9000005c
#define HIMAX_REG_SAFE_MODE_STATUS	0x900000a8
#define HIMAX_REG_TCON_RST		0x80020020
#define HIMAX_REG_ADC_RST		0x80020094

#define HIMAX_INVALID_COORD		0xffff

#define HIMAX_RESET_MS			5

struct himax_event_point {
	__be16 x;
	__be16 y;
} __packed;

/* 56-byte coordinate event stack layout reported by the FW */
struct himax_event {
	struct himax_event_point points[HIMAX_MAX_POINTS];
	u8 majors[HIMAX_MAX_POINTS];
	u8 pad0[2];
	u8 num_points;
	u8 pad1[2];
	u8 checksum_fix;
} __packed;

static_assert(sizeof(struct himax_event) == 56);

struct himax_ts_data {
	struct spi_device *spi;
	struct device *dev;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct gpio_desc *reset_gpio;
	struct mutex bus_lock;

	/* DMA-safe SPI bounce buffers, protected by bus_lock. */
	u8 *tx_buf;
	u8 *rx_buf;
};

static void himax_addr_to_bytes(u32 addr, u8 *b)
{
	b[0] = addr & 0xff;
	b[1] = (addr >> 8) & 0xff;
	b[2] = (addr >> 16) & 0xff;
	b[3] = (addr >> 24) & 0xff;
}

/* Raw AHB write: [0xf2][cmd][payload...] */
static int himax_ahb_write(struct himax_ts_data *ts, u8 cmd,
			   const u8 *data, size_t len)
{
	int ret;

	if (len > HIMAX_XFER_MAX - 2)
		return -EINVAL;

	mutex_lock(&ts->bus_lock);

	ts->tx_buf[0] = HIMAX_SPI_WRITE;
	ts->tx_buf[1] = cmd;
	memcpy(&ts->tx_buf[2], data, len);

	ret = spi_write(ts->spi, ts->tx_buf, len + 2);

	mutex_unlock(&ts->bus_lock);
	return ret;
}

/* Raw AHB read: TX [0xf3][cmd][0x00], then RX len bytes. */
static int himax_ahb_read(struct himax_ts_data *ts, u8 cmd,
			  u8 *data, size_t len)
{
	struct spi_transfer xfers[2] = { };
	int ret;

	if (len > HIMAX_XFER_MAX)
		return -EINVAL;

	mutex_lock(&ts->bus_lock);

	ts->tx_buf[0] = HIMAX_SPI_READ;
	ts->tx_buf[1] = cmd;
	ts->tx_buf[2] = 0x00;

	xfers[0].tx_buf = ts->tx_buf;
	xfers[0].len = 3;
	xfers[1].rx_buf = ts->rx_buf;
	xfers[1].len = len;

	ret = spi_sync_transfer(ts->spi, xfers, ARRAY_SIZE(xfers));
	if (!ret)
		memcpy(data, ts->rx_buf, len);

	mutex_unlock(&ts->bus_lock);
	return ret;
}

static int himax_ahb_write_byte(struct himax_ts_data *ts, u8 cmd, u8 val)
{
	return himax_ahb_write(ts, cmd, &val, 1);
}

static int himax_burst_enable(struct himax_ts_data *ts, bool auto_incr4)
{
	int ret;

	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_CONTI, HIMAX_AHB_CMD_CONTI);
	if (ret)
		return ret;

	return himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_INCR4,
				    HIMAX_AHB_CMD_INCR4 | (auto_incr4 ? 1 : 0));
}

/* Read a register window (len <= 256) at a 32-bit AHB address. */
static int himax_reg_read(struct himax_ts_data *ts, u32 addr,
			  u8 *data, size_t len)
{
	u8 addr_bytes[4];
	int ret;

	if (len > 256)
		return -EINVAL;

	ret = himax_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	himax_addr_to_bytes(addr, addr_bytes);
	ret = himax_ahb_write(ts, HIMAX_AHB_ADDR_BYTE_0, addr_bytes, 4);
	if (ret)
		return ret;

	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_ACCESS_DIR,
				   HIMAX_AHB_CMD_ACCESS_DIR_READ);
	if (ret)
		return ret;

	return himax_ahb_read(ts, HIMAX_AHB_ADDR_RDATA_BYTE_0, data, len);
}

static int himax_reg_read32(struct himax_ts_data *ts, u32 addr, u32 *val)
{
	u8 d[4];
	int ret;

	ret = himax_reg_read(ts, addr, d, 4);
	if (ret)
		return ret;

	*val = d[0] | d[1] << 8 | d[2] << 16 | d[3] << 24;
	return 0;
}

/* Single burst write: [addr(4)][data(len)] to AHB address 0. */
static int himax_reg_write(struct himax_ts_data *ts, u32 addr,
			   const u8 *data, size_t len)
{
	u8 buf[8];

	if (len != 4)
		return -EINVAL;

	himax_addr_to_bytes(addr, buf);
	memcpy(&buf[4], data, len);

	return himax_ahb_write(ts, HIMAX_AHB_ADDR_BYTE_0, buf, len + 4);
}

static int himax_reg_write32(struct himax_ts_data *ts, u32 addr, u32 val)
{
	u8 d[4];

	himax_addr_to_bytes(val, d);
	return himax_reg_write(ts, addr, d, 4);
}

static void himax_reset(struct himax_ts_data *ts)
{
	/* reset-gpios is active-low: logical 1 == asserted == chip in reset. */
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(HIMAX_RESET_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(HIMAX_RESET_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(4 * HIMAX_RESET_MS);
}

/* Put the analog front-end into safe mode so SRAM can be rewritten. */
static int himax_sense_off(struct himax_ts_data *ts)
{
	u32 status;
	int ret;

	ret = himax_reg_write32(ts, HIMAX_REG_SAFE_MODE, 0x000000a5);
	if (ret)
		return ret;

	ret = himax_reg_read32(ts, HIMAX_REG_SAFE_MODE_STATUS, &status);
	if (ret)
		return ret;

	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_RST_0, 0x27);
	if (ret)
		return ret;
	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_RST_1, 0x95);
	if (ret)
		return ret;

	ret = himax_reg_read32(ts, HIMAX_REG_SAFE_MODE_STATUS, &status);
	if (ret)
		return ret;

	if ((status & 0xff) == 0x0c) {
		/* Reset TCON */
		himax_reg_write32(ts, HIMAX_REG_TCON_RST, 0x00000000);
		msleep(20);
		himax_reg_write32(ts, HIMAX_REG_TCON_RST, 0x00000001);
		/* Reset ADC */
		himax_reg_write32(ts, HIMAX_REG_ADC_RST, 0x00000000);
		msleep(20);
		himax_reg_write32(ts, HIMAX_REG_ADC_RST, 0x00000001);
	}

	return 0;
}

static int himax_check_product_id(struct himax_ts_data *ts)
{
	u32 id;
	int ret;

	ret = himax_reg_read32(ts, HIMAX_REG_ICID, &id);
	if (ret)
		return ret;

	id >>= 8;
	dev_dbg(ts->dev, "product id: %#x\n", id);

	if (id != HIMAX_ICID_HX83112F) {
		dev_err(ts->dev, "unknown product id: %#x\n", id);
		return -ENODEV;
	}

	return 0;
}

static bool himax_verify_checksum(struct himax_ts_data *ts,
				  const struct himax_event *event)
{
	const u8 *data = (const u8 *)event;
	u16 checksum = 0;
	int i;

	for (i = 0; i < sizeof(*event); i++)
		checksum += data[i];

	if (checksum & 0xff) {
		dev_dbg(ts->dev, "bad event checksum: %#04x\n", checksum);
		return false;
	}

	return true;
}

static u8 himax_event_num_points(const struct himax_event *event)
{
	if (event->num_points == 0xff)
		return 0;

	return event->num_points & 0x0f;
}

static bool himax_process_point(struct himax_ts_data *ts,
				const struct himax_event *event, int i)
{
	u16 x = be16_to_cpu(event->points[i].x);
	u16 y = be16_to_cpu(event->points[i].y);
	u8 w = event->majors[i];

	if (x == HIMAX_INVALID_COORD || y == HIMAX_INVALID_COORD)
		return false;

	input_mt_slot(ts->input, i);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input, &ts->props, x, y, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, w);
	return true;
}

/* Read the 56-byte coordinate event stack (burst-read must be off around it). */
static int himax_read_event(struct himax_ts_data *ts, struct himax_event *event)
{
	int ret;

	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_BURST_READ,
				   HIMAX_AHB_CMD_BURST_READ_OFF);
	if (ret)
		return ret;

	ret = himax_ahb_read(ts, HIMAX_AHB_ADDR_EVENT_STACK,
			     (u8 *)event, sizeof(*event));
	if (ret)
		return ret;

	return himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_BURST_READ,
				    HIMAX_AHB_CMD_BURST_READ_ON);
}

static irqreturn_t himax_irq(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;
	struct himax_event event;
	int i, left;

	if (himax_read_event(ts, &event))
		return IRQ_NONE;

	if (!himax_verify_checksum(ts, &event))
		return IRQ_HANDLED;

	left = himax_event_num_points(&event);
	for (i = 0; i < HIMAX_MAX_POINTS && left > 0; i++) {
		if (himax_process_point(ts, &event, i))
			left--;
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	return IRQ_HANDLED;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	u32 coords[2];
	u32 max_x = 1080, max_y = 2400;
	int ret;

	ts->input = devm_input_allocate_device(ts->dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "Himax HX83112F Touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_drvdata(ts->input, ts);

	/*
	 * Coordinate range: standard "touchscreen-size-x/y" (via
	 * touchscreen_parse_properties() below) takes precedence; otherwise fall
	 * back to the OnePlus vendor "touchpanel,panel-coords" so this works
	 * unchanged on the existing device node.
	 */
	if (!device_property_read_u32_array(ts->dev, "touchpanel,panel-coords",
					    coords, 2)) {
		max_x = coords[0];
		max_y = coords[1];
	}

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(ts->input, true, &ts->props);

	ret = input_mt_init_slots(ts->input, HIMAX_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(ts->dev, ret, "failed to init MT slots\n");

	return input_register_device(ts->input);
}

/* Bring the controller up: reset, safe mode, IC-ID check, input + IRQ. */
static int himax_start_hw(struct himax_ts_data *ts)
{
	int ret;

	himax_reset(ts);

	ret = himax_sense_off(ts);
	if (ret)
		return dev_err_probe(ts->dev, ret, "failed to enter safe mode\n");

	ret = himax_check_product_id(ts);
	if (ret)
		return ret;

	ret = himax_input_register(ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(ts->dev, ts->spi->irq, NULL, himax_irq,
					IRQF_ONESHOT, "himax-hx83112f", ts);
	if (ret)
		return dev_err_probe(ts->dev, ret, "failed to request IRQ\n");

	return 0;
}

static int himax_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct himax_ts_data *ts;
	int ret;

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->dev = dev;
	spi_set_drvdata(spi, ts);

	mutex_init(&ts->bus_lock);

	ts->tx_buf = devm_kzalloc(dev, HIMAX_XFER_MAX, GFP_KERNEL);
	ts->rx_buf = devm_kzalloc(dev, HIMAX_XFER_MAX, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf)
		return -ENOMEM;

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	return himax_start_hw(ts);
}

static int __maybe_unused himax_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	disable_irq(ts->spi->irq);
	return 0;
}

static int __maybe_unused himax_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	enable_irq(ts->spi->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(himax_pm_ops, himax_suspend, himax_resume);

static const struct spi_device_id himax_spi_id[] = {
	{ "hx83112f" },
	{ "hxcommon" },
	{ }
};
MODULE_DEVICE_TABLE(spi, himax_spi_id);

static const struct of_device_id himax_of_match[] = {
	{ .compatible = "himax,hx83112f" },
	{ .compatible = "himax,hxcommon" },
	{ }
};
MODULE_DEVICE_TABLE(of, himax_of_match);

static struct spi_driver himax_spi_driver = {
	.probe = himax_probe,
	.id_table = himax_spi_id,
	.driver = {
		.name = "himax-hx83112f",
		.of_match_table = himax_of_match,
		.pm = &himax_pm_ops,
	},
};
module_spi_driver(himax_spi_driver);

MODULE_AUTHOR("Saalim Quadri <danascape@gmail.com>");
MODULE_DESCRIPTION("Himax HX83112F no-flash touchscreen driver");
MODULE_LICENSE("GPL");
