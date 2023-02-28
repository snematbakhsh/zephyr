/*
 * Copyright (c) 2023 Rivos Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT lowrisc_opentitan_i2c

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_opentitan);

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <soc.h>
#include <zephyr/sys/sys_io.h>

#include "i2c-priv.h"

#define I2C_INTR_STATE_REG_OFFSET 0x00
#define I2C_CTRL_REG_OFFSET 0x10
#define I2C_STATUS_REG_OFFSET 0x14
#define I2C_RDATA_REG_OFFSET 0x1C
#define I2C_FDATA_REG_OFFSET 0x1C
#define I2C_FIFO_CTRL_REG_OFFSET 0x20
#define I2C_TIMING0_REG_OFFSET 0x30
#define I2C_TIMING1_REG_OFFSET 0x34
#define I2C_TIMING2_REG_OFFSET 0x38
#define I2C_TIMING3_REG_OFFSET 0x3C
#define I2C_TIMING4_REG_OFFSET 0x40

#define I2C_INTR_STATE_FMT_OVERFLOW_BIT BIT(2)
#define I2C_INTR_STATE_RX_OVERFLOW_BIT BIT(3)
#define I2C_INTR_STATE_NAK_BIT BIT(4)
#define I2C_INTR_STATE_SCL_INTERFERENCE_BIT BIT(5)
#define I2C_INTR_STATE_SDA_INTERFERENCE_BIT BIT(6)
#define I2C_INTR_STATE_STRETCH_TIMEOUT_BIT BIT(7)
#define I2C_INTR_STATE_SDA_UNSTABLE_BIT BIT(8)
#define I2C_INTR_STATE_ERROR_MASK (BIT_MASK(7) << 2)

#define I2C_CTRL_ENABLEHOST_BIT BIT(0)

#define I2C_STATUS_FMTFULL_BIT BIT(0)
#define I2C_STATUS_RXFULL_BIT BIT(1)
#define I2C_STATUS_FMTEMPTY_BIT BIT(2)
#define I2C_STATUS_RXEMPTY_BIT BIT(5)

#define I2C_FDATA_START_BIT BIT(8)
#define I2C_FDATA_STOP_BIT BIT(9)
#define I2C_FDATA_READ_BIT BIT(10)
#define I2C_FDATA_RCONT_BIT BIT(11)
#define I2C_FDATA_NAKOK_BIT BIT(12)

#define I2C_FIFO_CTRL_RXRST_BIT BIT(0)
#define I2C_FIFO_CTRL_FMTRST_BIT BIT(1)
#define I2C_FIFO_CTRL_ACQRST_BIT BIT(7)
#define I2C_FIFO_CTRL_TXRST_BIT BIT(8)

struct i2c_opentitan_cfg {
	uint32_t base;
	uint32_t f_sys;
	uint32_t f_bus;
};

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

static int i2c_opentitan_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_opentitan_cfg *cfg = NULL;

	uint16_t i2c_thigh_cycles, i2c_tlow_cycles;
	uint16_t i2c_tf_cycles, i2c_tr_cycles;
	uint16_t i2c_thd_sta_cycles, i2c_tsu_sta_cycles;
	uint16_t i2c_thd_dat_cycles, i2c_tsu_dat_cycles;
	uint16_t i2c_tbuf_cycles, i2c_tsto_cycles;
	uint16_t i2c_min_clk_period_cycles;

	int sys_mhz;
	int extra_cycles;

	if (dev == NULL) {
		LOG_ERR("Device handle is NULL");
		return -EINVAL;
	}
	cfg = dev->config;
	if (cfg == NULL) {
		LOG_ERR("Device config is NULL");
		return -EINVAL;
	}

	/* Support I2C Master mode only */
	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		LOG_ERR("I2C only supports operation as master");
		return -ENOTSUP;
	}

	/* Driver does not support 10-bit addressing. This can be added
	 * in the future when needed.
	 */
	if (dev_config & I2C_ADDR_10_BITS) {
		LOG_ERR("I2C driver does not support 10-bit addresses");
		return -ENOTSUP;
	}

	sys_mhz = cfg->f_sys / MHZ(1);

	/* Disable the I2C peripheral */
	sys_write32(0, cfg->base + I2C_CTRL_REG_OFFSET);

	/* Configure bus frequency */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		i2c_thigh_cycles = DIV_ROUND_UP(4000, sys_mhz);
		i2c_tlow_cycles = DIV_ROUND_UP(4700, sys_mhz);
		i2c_tf_cycles = DIV_ROUND_UP(300, sys_mhz);
		i2c_tr_cycles = DIV_ROUND_UP(1000, sys_mhz);
		i2c_thd_sta_cycles = DIV_ROUND_UP(4000, sys_mhz);
		i2c_tsu_sta_cycles = DIV_ROUND_UP(4700, sys_mhz);
		i2c_thd_dat_cycles = 0;
		i2c_tsu_dat_cycles = DIV_ROUND_UP(250, sys_mhz);
		i2c_tbuf_cycles = DIV_ROUND_UP(4700, sys_mhz);
		i2c_tsto_cycles = DIV_ROUND_UP(4000, sys_mhz);
		i2c_min_clk_period_cycles = DIV_ROUND_UP(10000, sys_mhz);
		break;

	case I2C_SPEED_FAST:
		i2c_thigh_cycles = DIV_ROUND_UP(600, sys_mhz);
		i2c_tlow_cycles = DIV_ROUND_UP(1300, sys_mhz);
		i2c_tf_cycles = DIV_ROUND_UP(20, sys_mhz);
		i2c_tr_cycles = DIV_ROUND_UP(20, sys_mhz);
		i2c_thd_sta_cycles = DIV_ROUND_UP(600, sys_mhz);
		i2c_tsu_sta_cycles = DIV_ROUND_UP(600, sys_mhz);
		i2c_thd_dat_cycles = 0;
		i2c_tsu_dat_cycles = DIV_ROUND_UP(100, sys_mhz);
		i2c_tbuf_cycles = DIV_ROUND_UP(1300, sys_mhz);
		i2c_tsto_cycles = DIV_ROUND_UP(600, sys_mhz);
		i2c_min_clk_period_cycles = DIV_ROUND_UP(2500, sys_mhz);
		break;

	case I2C_SPEED_FAST_PLUS:
		i2c_thigh_cycles = DIV_ROUND_UP(260, sys_mhz);
		i2c_tlow_cycles = DIV_ROUND_UP(500, sys_mhz);
		i2c_tf_cycles = DIV_ROUND_UP(20, sys_mhz);
		i2c_tr_cycles = DIV_ROUND_UP(20, sys_mhz);
		i2c_thd_sta_cycles = DIV_ROUND_UP(260, sys_mhz);
		i2c_tsu_sta_cycles = DIV_ROUND_UP(260, sys_mhz);
		i2c_thd_dat_cycles = 0;
		i2c_tsu_dat_cycles = DIV_ROUND_UP(50, sys_mhz);
		i2c_tbuf_cycles = DIV_ROUND_UP(500, sys_mhz);
		i2c_tsto_cycles = DIV_ROUND_UP(260, sys_mhz);
		i2c_min_clk_period_cycles = DIV_ROUND_UP(1000, sys_mhz);
		break;

	case I2C_SPEED_HIGH:
	case I2C_SPEED_ULTRA:
	default:
		LOG_ERR("Unsupported I2C speed requested");
		return -ENOTSUP;
	}

    /* Pad high period to meet desired clock period */
	extra_cycles = i2c_min_clk_period_cycles -
		i2c_thigh_cycles - i2c_tlow_cycles - i2c_tf_cycles - i2c_tr_cycles;
	if (extra_cycles > 0) {
		i2c_thigh_cycles +=  extra_cycles;
	}

    /* Write timing registers */
	sys_write32((i2c_tlow_cycles << 16) | i2c_thigh_cycles,
		cfg->base + I2C_TIMING0_REG_OFFSET);
	sys_write32((i2c_tf_cycles << 16) | i2c_tr_cycles,
		cfg->base + I2C_TIMING1_REG_OFFSET);
	sys_write32((i2c_thd_sta_cycles << 16) | i2c_tsu_sta_cycles,
		cfg->base + I2C_TIMING2_REG_OFFSET);
	sys_write32((i2c_thd_dat_cycles << 16) | i2c_tsu_dat_cycles,
		cfg->base + I2C_TIMING3_REG_OFFSET);
	sys_write32((i2c_tbuf_cycles << 16) | i2c_tsto_cycles,
		cfg->base + I2C_TIMING4_REG_OFFSET);

	/* Reset FIFOs */
	sys_write32(I2C_FIFO_CTRL_RXRST_BIT | I2C_FIFO_CTRL_FMTRST_BIT |
		I2C_FIFO_CTRL_ACQRST_BIT | I2C_FIFO_CTRL_TXRST_BIT,
		cfg->base + I2C_FIFO_CTRL_REG_OFFSET);

	/* Enable the I2C peripheral */
	sys_write32(I2C_CTRL_ENABLEHOST_BIT, cfg->base + I2C_CTRL_REG_OFFSET);

	return 0;
}

static bool i2c_opentitan_fmt_fifo_full(const struct device *dev)
{
	const struct i2c_opentitan_cfg *cfg = dev->config;

	return sys_read32(cfg->base + I2C_STATUS_REG_OFFSET) & I2C_STATUS_FMTFULL_BIT;
}

static bool i2c_opentitan_rx_fifo_empty(const struct device *dev)
{
	const struct i2c_opentitan_cfg *cfg = dev->config;

	return sys_read32(cfg->base + I2C_STATUS_REG_OFFSET) & I2C_STATUS_RXEMPTY_BIT;
}

static void i2c_opentitan_write_byte(const struct device *dev, uint8_t byte,
		bool start, bool stop)
{
	const struct i2c_opentitan_cfg *cfg = dev->config;
	uint32_t reg = byte;

	if (start) {
		reg |= I2C_FDATA_START_BIT;
	}
	if (stop) {
		reg |= I2C_FDATA_STOP_BIT;
	}
	while (i2c_opentitan_fmt_fifo_full(dev)) {
		;
	}
	sys_write32(reg, cfg->base + I2C_FDATA_REG_OFFSET);
}

static bool i2c_opentitan_error(const struct device *dev)
{
	const struct i2c_opentitan_cfg *cfg = dev->config;
	uint32_t intr = sys_read32(cfg->base + I2C_INTR_STATE_REG_OFFSET);

	sys_write32(intr, cfg->base + I2C_INTR_STATE_REG_OFFSET);
	return intr & I2C_INTR_STATE_ERROR_MASK;
}

static int i2c_opentitan_read_bytes(const struct device *dev,
		uint8_t *read_bytes, uint32_t len, bool stop)
{
	const struct i2c_opentitan_cfg *cfg = dev->config;
	uint32_t reg = len | I2C_FDATA_READ_BIT;

    /* Wait for space in FMT FIFO to write read command */
	while (i2c_opentitan_fmt_fifo_full(dev)) {
		;
	}
	sys_write32(reg, cfg->base + I2C_FDATA_REG_OFFSET);

	for (int i = 0; i < len; ++i) {
		/* Wait for FIFO data */
		while (i2c_opentitan_rx_fifo_empty(dev)) {
			if (i2c_opentitan_error(dev)) {
				LOG_ERR("NAK on read");
				return -EIO;
			}
		}
		read_bytes[i] = sys_read32(cfg->base + I2C_RDATA_REG_OFFSET);
	}
	return 0;
}

static int i2c_opentitan_transfer(const struct device *dev,
				   struct i2c_msg *msgs,
				   uint8_t num_msgs,
				   uint16_t addr)
{
	int rc = 0;
	bool read, stop;
	uint8_t byte;
	struct i2c_msg *msg;

	/* Check for NULL pointers */
	if (dev == NULL) {
		LOG_ERR("Device handle is NULL");
		return -EINVAL;
	}
	if (dev->config == NULL) {
		LOG_ERR("Device config is NULL");
		return -EINVAL;
	}
	if (msgs == NULL) {
		return -EINVAL;
	}

	for (int i = 0; i < num_msgs; i++) {
		msg = &msgs[i];
		read = msg->flags & I2C_MSG_READ;
		stop = msg->flags & I2C_MSG_STOP;

		if (i == 0 || (msg->flags & I2C_MSG_RESTART)) {
			byte = (addr << 1)  | read;
			i2c_opentitan_write_byte(dev, byte, true, false);
		}

		if (read) {
			rc = i2c_opentitan_read_bytes(dev, msg->buf, msg->len, false);
		} else {
			for (int j = 0; j < msg->len; ++j) {
				i2c_opentitan_write_byte(dev, msg->buf[j], false,
						stop && (j == msg->len - 1));
			}
		}

		if (rc != 0) {
			LOG_ERR("I2C failed to transfer messages");
			return rc;
		}
	}

	return 0;
};

static int i2c_opentitan_init(const struct device *dev)
{
	const struct i2c_opentitan_cfg *config = dev->config;
	uint32_t dev_config = 0U;
	int rc = 0;

	dev_config = (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(config->f_bus));

	rc = i2c_opentitan_configure(dev, dev_config);
	if (rc != 0) {
		LOG_ERR("Failed to configure I2C on init");
		return rc;
	}

	return 0;
}


static struct i2c_driver_api i2c_opentitan_api = {
	.configure = i2c_opentitan_configure,
	.transfer = i2c_opentitan_transfer,
};

#define I2C_OPENTITAN_INIT(n) \
	static struct i2c_opentitan_cfg i2c_opentitan_cfg_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.f_sys = DT_PROP_BY_PHANDLE(DT_NODELABEL(i2c##n), clocks, clock_frequency), \
		.f_bus = DT_INST_PROP(n, clock_frequency), \
	}; \
	I2C_DEVICE_DT_INST_DEFINE(n, \
				i2c_opentitan_init, \
				NULL, \
				NULL, \
				&i2c_opentitan_cfg_##n, \
				POST_KERNEL, \
				CONFIG_I2C_INIT_PRIORITY, \
				&i2c_opentitan_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_OPENTITAN_INIT)
