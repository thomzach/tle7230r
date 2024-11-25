/*
* Copyright (c) 2024 Zach Thomas
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT infineon_tle7230r

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#define SPI_OP  SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_MSB

#define CMD_DIAGNOSIS 0b00 << 6
#define CMD_READ      0b01 << 6
#define CMD_RESET     0b10 << 6
#define CMD_WRITE     0b11 << 6
#define ADDR_MAP      0b001
#define ADDR_BOL      0b010
#define ADDR_OVL      0b011
#define ADDR_OVT      0b100
#define ADDR_SLE      0b101
#define ADDR_STA      0b110

#define ADDR_CTL      0b111

LOG_MODULE_REGISTER(gpio_tle7230r, CONFIG_GPIO_LOG_LEVEL);

enum tle7230r_cmd {
  TLE7230R_CMD_DIAGNOSIS = 0b00000000,
  TLE7230R_CMD_READ =      0b01000000,
  TLE7230R_CMD_RESET =     0b10000000,
  TLE7230R_CMD_WRITE =     0b11000000,
};

enum tle7230r_addr {
  TLE7230R_ADDR_MAP = 0b001,
  TLE7230R_ADDR_BOL = 0b010,
  TLE7230R_ADDR_OVL = 0b011,
  TLE7230R_ADDR_OVT = 0b100,
  TLE7230R_ADDR_SLE = 0b101,
  TLE7230R_ADDR_STA = 0b110,
  TLE7230R_ADDR_CTL = 0b111,
};


struct tle7230r_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_reset;
};

struct tle7230r_data {
	struct gpio_driver_data common;
	uint8_t state;
	uint8_t previous_state;
	uint8_t configured;
	struct k_mutex lock;
};



static int tle7230_transceive_frame(const struct device *dev,
                                    enum tle7230r_cmd cmd_header,
                                    uint8_t write_data, uint16_t *read_data)
{
  int result;

	const struct tle7230r_config *config = dev->config;

  uint16_t write_frame;
  uint16_t read_frame;
  uint8_t buffer_tx[2];

  uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
  const struct spi_buf tx_buf[] = {{
      .buf = buffer_tx,
      .len = ARRAY_SIZE(buffer_tx),
  }};
  const struct spi_buf rx_buf[] = {{
      .buf = buffer_rx,
      .len = ARRAY_SIZE(buffer_rx),
  }};
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };
  const struct spi_buf_set rx = {
      .buffers = rx_buf,
      .count = ARRAY_SIZE(rx_buf),
  };

  write_frame = write_data << 0;
  write_frame |= cmd_header << 8;

  sys_put_be16(write_frame, buffer_tx);

  result = spi_transceive_dt(&config->bus, &tx, &rx);
  if (result != 0) {
    LOG_ERR("spi_write failed with error %i", result);
    return result;
  }

  read_frame = sys_get_be16(buffer_rx);

  *read_data = FIELD_GET(GENMASK(16, 0), read_frame);

  return 0;
}

static int tle7230r_write_register(const struct device *dev, enum tle7230r_cmd cmd, enum tle7230r_addr addr, uint8_t data)
{
  int ret;
  uint16_t rx;

  ret = tle7230_transceive_frame(dev, (cmd | addr), data, &rx);
  LOG_DBG("diag 0x%04X", rx);

  return 0;
}

static int tle7230r_write_state(const struct device *dev)
{
	struct tle7230r_data *data = dev->data;
    int ret;
  
    ret = tle7230r_write_register(dev, TLE7230R_CMD_WRITE, TLE7230R_ADDR_CTL, data->state);

    data->previous_state = data->state;
    return ret;
} 

static int tle7230r_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	struct tle7230r_data *data = dev->data;
	int result;

	/* cannot execute a bus operation in an ISR context */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->state = (data->state & ~mask) | (mask & value);
	result = tle7230r_write_state(dev);
    
	k_mutex_unlock(&data->lock);

	return result;
}

static int tle7230r_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return tle7230r_port_set_masked_raw(dev, mask, mask);
}

static int tle7230r_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	return tle7230r_port_set_masked_raw(dev, mask, 0);
}

static int tle7230r_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	struct tle7230r_data *data = dev->data;

	/* cannot execute a bus operation in an ISR context */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->state ^= mask;

	int result = tle7230r_write_state(dev);

	k_mutex_unlock(&data->lock);

	return result;
}

static int tle7230r_port_get_raw(const struct device *dev, uint32_t *value)
{
	LOG_ERR("input pins are not available");
	return -ENOTSUP;
}

static int tle7230r_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
  
	struct tle7230r_data *data = dev->data;
	int result;

	/* cannot execute a bus operation in an ISR context */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if ((flags & GPIO_INPUT) != 0) {
		LOG_ERR("cannot configure pin as input");
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) == 0) {
		LOG_ERR("pin must be configured as an output");
		return -ENOTSUP;
	}

	if ((flags & GPIO_SINGLE_ENDED) == 0) {
		LOG_ERR("pin must be configured as single ended");
		return -ENOTSUP;
	}

	if ((flags & GPIO_LINE_OPEN_DRAIN) == 0) {
		LOG_ERR("pin must be configured as open drain");
		return -ENOTSUP;
	}

	if ((flags & GPIO_PULL_UP) != 0) {
		LOG_ERR("pin cannot have a pull up configured");
		return -ENOTSUP;
	}

	if ((flags & GPIO_PULL_DOWN) != 0) {
		LOG_ERR("pin cannot have a pull down configured");
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
		WRITE_BIT(data->state, pin, 0);
	} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
		WRITE_BIT(data->state, pin, 1);
	}

	WRITE_BIT(data->configured, pin, 1);
	result = tle7230r_write_state(dev);
	k_mutex_unlock(&data->lock);

	return result;
}

static const struct gpio_driver_api api_table = {
	.pin_configure = tle7230r_pin_configure,
	.port_get_raw = tle7230r_port_get_raw,
	.port_set_masked_raw = tle7230r_port_set_masked_raw,
	.port_set_bits_raw = tle7230r_port_set_bits_raw,
	.port_clear_bits_raw = tle7230r_port_clear_bits_raw,
	.port_toggle_bits = tle7230r_port_toggle_bits,
};

static int tle7230r_init(const struct device *dev)
{
	const struct tle7230r_config *config = dev->config;
	struct tle7230r_data *data = dev->data;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	int result = k_mutex_init(&data->lock);

	if (result != 0) {
		LOG_ERR("unable to initialize mutex");
		return result;
	}


	if (result != 0) {
		LOG_ERR("failed to initialize GPIO for reset");
		return result;
	}

    if (config->gpio_reset.port != NULL) {
        if (!gpio_is_ready_dt(&config->gpio_reset)) {
            LOG_ERR("%s: reset GPIO is not ready", dev->name);
            return -ENODEV;
        }

        result = gpio_pin_configure_dt(&config->gpio_reset, GPIO_OUTPUT_ACTIVE);
		if (result != 0) {
			LOG_ERR("failed to initialize GPIO for reset");
			return result;
		}

        /* k_busy_wait(WAIT_TIME_RESET_ACTIVE_IN_US); */
        gpio_pin_set_dt(&config->gpio_reset, 1);
        /* k_busy_wait(WAIT_TIME_RESET_INACTIVE_TO_CS_IN_US); */
    }

	return 0;
}

BUILD_ASSERT(CONFIG_GPIO_TLE7230R_INIT_PRIORITY > CONFIG_SPI_INIT_PRIORITY,
         "TLE7230R must be initialized after SPI");

#define TLE7230R_INIT_GPIO_FIELDS(inst, gpio)                                      \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, gpio),                                 \
            (GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), gpio, 0)), ({0}))

#define TLE7230R_INIT(inst)                                                        \
    static const struct tle7230r_config tle7230r_##inst##_config = {               \
        .common = {                                                                \
                .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
        },                                                                         \
        .bus = SPI_DT_SPEC_INST_GET(                                               \
            inst, SPI_OP, 0),                                                      \
        .gpio_reset = TLE7230R_INIT_GPIO_FIELDS(inst, resn_gpios),                 \
    };                                                                             \
                                                                                   \
    static struct tle7230r_data tle7230r_##inst##_drvdata;                         \
                                                                                   \
    /* This has to be initialized after the SPI peripheral. */                     \
    DEVICE_DT_INST_DEFINE(inst, tle7230r_init, NULL, &tle7230r_##inst##_drvdata,   \
                  &tle7230r_##inst##_config, POST_KERNEL,                          \
                  CONFIG_GPIO_TLE7230R_INIT_PRIORITY, &api_table);

DT_INST_FOREACH_STATUS_OKAY(TLE7230R_INIT)
