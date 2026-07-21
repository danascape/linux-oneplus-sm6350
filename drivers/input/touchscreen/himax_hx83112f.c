// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Himax HX83112F "no-flash" touchscreens (SPI)
 *
 * Copyright (C) 2026 Saalim Quadri <danascape@gmail.com>
 *
 * The HX83112F has no on-chip flash: its firmware lives in the rootfs and
 * must be downloaded into the controller's SRAM after every reset (probe and
 * resume) before touch reporting works. This driver implements the Himax SPI
 * framing, the AHB register model, the zero-flash SRAM download, and touch
 * event parsing.
 *
 * The register/event handling is derived from the mainline himax_hx83112b
 * driver by Job Noorman, and the zero-flash sequences from the OnePlus/Himax
 * "Android Driver Sample Code for HX83112 chipset".
 *
 * This copy is back-ported to the 4.19 downstream OnePlus/SM6350 tree: it
 * avoids cleanup.h scope guards, devm_mutex_init() and the modern PM_OPS
 * helpers, and it also matches the existing "himax,hxcommon" device node so
 * it can be swapped in for the vendor framework driver without DT changes
 * beyond a mainline-style "reset-gpios" property (see the driver commit msg).
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
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
#include <linux/workqueue.h>

#define HIMAX_MAX_POINTS		10

/* Himax SPI framing: a 2-byte command header precedes every AHB access. */
#define HIMAX_SPI_WRITE			0xf2
#define HIMAX_SPI_READ			0xf3
#define HIMAX_SPI_RETRIES		10
/* header(2) + AHB address(4) + max SRAM payload per transfer */
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
#define HIMAX_REG_RELOAD_ACTIVE		0x90000048
#define HIMAX_RELOAD_ACTIVE_REQ		0x000000ec
#define HIMAX_RELOAD_ACTIVE_DONE	0x000001ec

/* Hardware CRC engine */
#define HIMAX_REG_CRC_ADDR		0x80050020
#define HIMAX_REG_CRC_LEN		0x80050028
#define HIMAX_REG_CRC_STATUS		0x80050000
#define HIMAX_REG_CRC_RESULT		0x80050018
#define HIMAX_CRC_LEN_MAGIC		0x00990000
#define HIMAX_CRC_BUSY			BIT(0)

/* Zero-flash download */
#define HIMAX_SRAM_FW_ADDR		0x20000000
#define HIMAX_REG_DIS_FLASH_RELOAD	0x10007f00
#define HIMAX_DIS_FLASH_RELOAD_VAL	0x00009aa9
#define HIMAX_REG_FLASH_RELOAD_CLR	0x100072c0
#define HIMAX_REG_MODE_SWITCH		0x10007294

#define HIMAX_FW_1K			0x400		/* leading header */
#define HIMAX_FW_64K			0x10000		/* main FW body */
#define HIMAX_PART_TABLE_OFF		(HIMAX_FW_64K + HIMAX_FW_1K)
#define HIMAX_PART_DESC_SZ		0x10
#define HIMAX_PART_NUM_OFF		12
#define HIMAX_CFG_ALIGN			16

#define HIMAX_INVALID_COORD		0xffff

#define HIMAX_RESET_MS			5

#define HIMAX_DEFAULT_FW_NAME		"Himax_firmware.bin"

/*
 * On Android the touch firmware is supplied by ueventd through the firmware
 * usermode-helper fallback (it searches /vendor/firmware, /vendor/etc/firmware,
 * etc.), which the kernel's direct filesystem search path does not cover. So we
 * must use request_firmware() (which engages that fallback), exactly like the
 * vendor driver -- request_firmware_direct() bypasses it and never finds the
 * file. request_firmware() can block until userspace is ready, so it is run
 * from a retrying delayed work (not probe) to keep boot non-blocking.
 */
#define HIMAX_FW_FIRST_DELAY_MS		4000	/* let ueventd/vendor come up first */
#define HIMAX_FW_RETRY_MS		2000
#define HIMAX_FW_MAX_RETRIES		20

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

	/* Cached firmware image, re-downloaded to SRAM on every reset. */
	const u8 *fw_data;
	size_t fw_size;

	/* DMA-safe SPI bounce buffers, protected by bus_lock. */
	u8 *tx_buf;
	u8 *rx_buf;

	/* Deferred firmware load (Android: fw partition mounts after probe). */
	struct delayed_work fw_work;
	int fw_retries;
};

/* One config partition merged into a contiguous SRAM window. */
struct himax_cfg_part {
	u32 sram_addr;
	u32 fw_addr;
	u16 write_size;
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

/* Chunked SRAM write with auto address increment. */
static int himax_sram_write(struct himax_ts_data *ts, u32 addr,
			    const u8 *data, size_t len)
{
	u8 hdr[4];
	size_t off = 0;
	int ret;

	ret = himax_burst_enable(ts, true);
	if (ret)
		return ret;

	while (off < len) {
		size_t chunk = min_t(size_t, len - off, HIMAX_SRAM_CHUNK);

		himax_addr_to_bytes(addr + off, hdr);

		mutex_lock(&ts->bus_lock);
		ts->tx_buf[0] = HIMAX_SPI_WRITE;
		ts->tx_buf[1] = HIMAX_AHB_ADDR_BYTE_0;
		memcpy(ts->tx_buf + 2, hdr, 4);
		memcpy(ts->tx_buf + 2 + 4, data + off, chunk);
		ret = spi_write(ts->spi, ts->tx_buf, chunk + 4 + 2);
		mutex_unlock(&ts->bus_lock);
		if (ret)
			return ret;

		off += chunk;
		udelay(100);
	}

	return 0;
}

static int himax_sys_reset(struct himax_ts_data *ts)
{
	int ret;

	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_RST_0, 0x27);
	if (ret)
		return ret;
	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_RST_1, 0x95);
	if (ret)
		return ret;
	ret = himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_RST_0, 0x00);
	if (ret)
		return ret;

	usleep_range(1000, 1100);
	return 0;
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

static int himax_reload_to_active(struct himax_ts_data *ts)
{
	u32 val;
	int retry;

	for (retry = 0; retry < HIMAX_SPI_RETRIES; retry++) {
		himax_reg_write32(ts, HIMAX_REG_RELOAD_ACTIVE,
				  HIMAX_RELOAD_ACTIVE_REQ);
		usleep_range(1000, 1100);
		if (himax_reg_read32(ts, HIMAX_REG_RELOAD_ACTIVE, &val))
			continue;
		if (val == HIMAX_RELOAD_ACTIVE_DONE)
			return 0;
	}

	dev_warn(ts->dev, "reload-to-active did not settle\n");
	return 0;
}

/* Wake the AHB interface and lock it into continuous burst mode. */
static int himax_interface_on(struct himax_ts_data *ts)
{
	u8 dummy[4];
	u8 c13, c0d;
	int cnt, ret;

	/* Dummy read to knock the bus awake. */
	ret = himax_ahb_read(ts, HIMAX_AHB_ADDR_RDATA_BYTE_0, dummy, sizeof(dummy));
	if (ret)
		return ret;

	for (cnt = 0; cnt < HIMAX_SPI_RETRIES; cnt++) {
		himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_CONTI, HIMAX_AHB_CMD_CONTI);
		himax_ahb_write_byte(ts, HIMAX_AHB_ADDR_INCR4, HIMAX_AHB_CMD_INCR4);

		if (himax_ahb_read(ts, HIMAX_AHB_ADDR_CONTI, &c13, 1))
			continue;
		if (himax_ahb_read(ts, HIMAX_AHB_ADDR_INCR4, &c0d, 1))
			continue;
		if (c13 == HIMAX_AHB_CMD_CONTI && c0d == HIMAX_AHB_CMD_INCR4)
			return 0;
		msleep(20);
	}

	dev_warn(ts->dev, "failed to enable burst mode\n");
	return 0;
}

static int himax_sense_on(struct himax_ts_data *ts)
{
	int ret;

	ret = himax_interface_on(ts);
	if (ret)
		return ret;

	ret = himax_reg_write32(ts, HIMAX_REG_SAFE_MODE, 0x00000000);
	if (ret)
		return ret;

	ret = himax_sys_reset(ts);
	if (ret)
		return ret;

	return himax_reload_to_active(ts);
}

/* Kick the hardware CRC engine over [addr, addr+len) and read the result. */
static u32 himax_hw_crc(struct himax_ts_data *ts, u32 addr, u32 len)
{
	u8 addr_bytes[4];
	u32 status, result = 0;
	int words = len / 4;
	int retry;

	himax_addr_to_bytes(addr, addr_bytes);
	himax_reg_write(ts, HIMAX_REG_CRC_ADDR, addr_bytes, 4);
	himax_reg_write32(ts, HIMAX_REG_CRC_LEN, HIMAX_CRC_LEN_MAGIC | words);

	for (retry = 0; retry < 100; retry++) {
		if (himax_reg_read32(ts, HIMAX_REG_CRC_STATUS, &status))
			break;
		if (!(status & HIMAX_CRC_BUSY)) {
			himax_reg_read32(ts, HIMAX_REG_CRC_RESULT, &result);
			return result;
		}
		usleep_range(1000, 1100);
	}

	dev_warn(ts->dev, "CRC engine stayed busy\n");
	return ~0;
}

/* CRC-32C (Castagnoli) over the config image, matching the FW's algorithm. */
static u32 himax_sw_crc(const u8 *data, u32 len)
{
	u32 crc = 0xffffffff;
	int i, j;

	for (i = 0; i < len / 4; i++) {
		u32 word = data[i * 4] | data[i * 4 + 1] << 8 |
			   data[i * 4 + 2] << 16 | data[i * 4 + 3] << 24;

		crc ^= word;
		for (j = 0; j < 32; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0x82f63b78;
			else
				crc >>= 1;
		}
	}

	return crc;
}

/*
 * Parse the partition table that follows the 64K FW body, merge the config
 * partitions into one contiguous SRAM window, write it and verify its CRC.
 */
static int himax_download_config(struct himax_ts_data *ts)
{
	const u8 *fw = ts->fw_data;
	u32 base = U32_MAX, max = 0, cfg_sz;
	int part_num, i, i_max = 0;
	struct himax_cfg_part *parts;
	u8 *cfg_buf;
	u32 crc;
	int ret = 0;

	if (ts->fw_size < HIMAX_PART_TABLE_OFF + HIMAX_PART_DESC_SZ)
		return -EINVAL;

	part_num = fw[HIMAX_PART_TABLE_OFF + HIMAX_PART_NUM_OFF];
	if (part_num <= 1)
		return -EINVAL;

	if (ts->fw_size < HIMAX_PART_TABLE_OFF + part_num * HIMAX_PART_DESC_SZ)
		return -EINVAL;

	parts = kcalloc(part_num, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;

	/* Partition 0 is the main FW body (already written); 1.. are config. */
	for (i = 1; i < part_num; i++) {
		const u8 *d = &fw[HIMAX_PART_TABLE_OFF + i * HIMAX_PART_DESC_SZ];

		parts[i].sram_addr = d[0] | d[1] << 8 | d[2] << 16 | d[3] << 24;
		parts[i].write_size = d[4] | d[5] << 8;
		parts[i].fw_addr = d[8] | d[9] << 8 | d[10] << 16;

		if (parts[i].fw_addr + parts[i].write_size > ts->fw_size) {
			ret = -EINVAL;
			goto out_parts;
		}

		if (parts[i].sram_addr < base)
			base = parts[i].sram_addr;
		if (parts[i].sram_addr > max) {
			max = parts[i].sram_addr;
			i_max = i;
		}
	}

	cfg_sz = (max - base) + parts[i_max].write_size;
	cfg_sz = ALIGN(cfg_sz, HIMAX_CFG_ALIGN);

	cfg_buf = kzalloc(cfg_sz, GFP_KERNEL);
	if (!cfg_buf) {
		ret = -ENOMEM;
		goto out_parts;
	}

	for (i = 1; i < part_num; i++) {
		if (parts[i].sram_addr - base + parts[i].write_size > cfg_sz) {
			ret = -EINVAL;
			goto out_cfg;
		}
		memcpy(cfg_buf + (parts[i].sram_addr - base),
		       &fw[parts[i].fw_addr], parts[i].write_size);
	}

	ret = himax_sram_write(ts, base, cfg_buf, cfg_sz);
	if (ret)
		goto out_cfg;

	crc = himax_hw_crc(ts, base, cfg_sz);
	if (crc != himax_sw_crc(cfg_buf, cfg_sz))
		dev_warn(ts->dev, "config CRC mismatch (hw %#x)\n", crc);

out_cfg:
	kfree(cfg_buf);
out_parts:
	kfree(parts);
	return ret;
}

/* Full zero-flash download: main FW body + config into SRAM, then activate. */
static int himax_download_firmware(struct himax_ts_data *ts)
{
	u32 crc;
	int ret;

	if (ts->fw_size < HIMAX_FW_1K + HIMAX_FW_64K)
		return -EINVAL;

	himax_reset(ts);

	ret = himax_sys_reset(ts);
	if (ret)
		return ret;

	ret = himax_sense_off(ts);
	if (ret)
		return ret;

	/* Main FW body: skip the 1K header, write the next 64K to SRAM. */
	ret = himax_sram_write(ts, HIMAX_SRAM_FW_ADDR,
			       ts->fw_data + HIMAX_FW_1K, HIMAX_FW_64K);
	if (ret)
		return ret;

	crc = himax_hw_crc(ts, HIMAX_SRAM_FW_ADDR, HIMAX_FW_64K);
	if (crc)
		dev_warn(ts->dev, "firmware CRC mismatch (%#x)\n", crc);

	ret = himax_download_config(ts);
	if (ret) {
		dev_err(ts->dev, "config download failed: %d\n", ret);
		return ret;
	}

	ret = himax_reg_write32(ts, HIMAX_REG_MODE_SWITCH, 0x00000000);
	if (ret)
		return ret;

	/* Disable flash reload so the FW runs from SRAM. */
	ret = himax_reg_write32(ts, HIMAX_REG_DIS_FLASH_RELOAD,
				HIMAX_DIS_FLASH_RELOAD_VAL);
	if (ret)
		return ret;
	himax_reg_write32(ts, HIMAX_REG_FLASH_RELOAD_CLR, 0x00000000);

	msleep(20);
	ret = himax_sense_on(ts);
	if (ret)
		return ret;
	msleep(20);

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

/* Bring the controller up once the firmware image is in hand. */
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

	ret = himax_download_firmware(ts);
	if (ret)
		return dev_err_probe(ts->dev, ret, "firmware download failed\n");

	ret = himax_input_register(ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(ts->dev, ts->spi->irq, NULL, himax_irq,
					IRQF_ONESHOT, "himax-hx83112f", ts);
	if (ret)
		return dev_err_probe(ts->dev, ret, "failed to request IRQ\n");

	return 0;
}

/*
 * Retrying firmware loader, mirroring the vendor driver: request_firmware()
 * (usermode-helper fallback enabled) retried until userspace/ueventd can serve
 * the file. Runs from a delayed work so probe -- and therefore boot -- never
 * blocks on it.
 */
static void himax_fw_work(struct work_struct *work)
{
	struct himax_ts_data *ts =
		container_of(to_delayed_work(work), struct himax_ts_data, fw_work);
	const struct firmware *fw;
	const char *fw_name;
	u8 *copy;
	int ret;

	if (device_property_read_string(ts->dev, "firmware-name", &fw_name))
		fw_name = HIMAX_DEFAULT_FW_NAME;

	ret = request_firmware(&fw, fw_name, ts->dev);
	if (ret) {
		if (ts->fw_retries-- > 0) {
			dev_info(ts->dev, "firmware \"%s\" not ready (%d), retrying\n",
				 fw_name, ret);
			schedule_delayed_work(&ts->fw_work,
					      msecs_to_jiffies(HIMAX_FW_RETRY_MS));
			return;
		}
		dev_err(ts->dev, "giving up loading firmware \"%s\": %d\n",
			fw_name, ret);
		return;
	}

	copy = devm_kmemdup(ts->dev, fw->data, fw->size, GFP_KERNEL);
	ts->fw_size = fw->size;
	release_firmware(fw);
	if (!copy)
		return;
	ts->fw_data = copy;

	ret = himax_start_hw(ts);
	if (ret)
		dev_err(ts->dev, "hardware init failed: %d\n", ret);
	else
		dev_info(ts->dev, "touchscreen ready (fw %zu bytes)\n",
			 ts->fw_size);
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

	/*
	 * Defer firmware load + controller bring-up: probe returns immediately
	 * so boot is never blocked waiting for the firmware partition.
	 */
	ts->fw_retries = HIMAX_FW_MAX_RETRIES;
	INIT_DELAYED_WORK(&ts->fw_work, himax_fw_work);
	schedule_delayed_work(&ts->fw_work,
			      msecs_to_jiffies(HIMAX_FW_FIRST_DELAY_MS));

	return 0;
}

static int himax_remove(struct spi_device *spi)
{
	struct himax_ts_data *ts = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&ts->fw_work);
	return 0;
}

static int __maybe_unused himax_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	/* Nothing to do if the controller never came up (no firmware yet). */
	if (!ts->fw_data)
		return 0;

	disable_irq(ts->spi->irq);
	return 0;
}

static int __maybe_unused himax_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	if (!ts->fw_data)
		return 0;

	/* SRAM is volatile: re-download the firmware after every suspend. */
	ret = himax_download_firmware(ts);
	if (ret)
		dev_err(dev, "firmware re-download failed on resume: %d\n", ret);

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
	.remove = himax_remove,
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
