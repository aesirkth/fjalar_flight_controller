/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(io_led), gpios);
static const struct device *const gps_dev = DEVICE_DT_GET(DT_ALIAS(gps_uart));
static const struct device *const imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso32));
static const struct device *const baro_dev = DEVICE_DT_GET(DT_NODELABEL(ms5607));
static const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(w25q128));

static inline float sensor_to_float(struct sensor_value val) {
	return (val.val1 + (float)val.val2 / 1000000);
}

void gps_uart_cb(const struct device *dev, void *user_data) {
	uint8_t buf[256];
	int len = 0;

	if (!uart_irq_update(gps_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(gps_dev)) {
		return;
	}

	/* read until FIFO empty */
	len = uart_fifo_read(gps_dev, buf, sizeof(buf));
	printk("%.*s", len, buf);
}


int main(void) {
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	if (!device_is_ready(gps_dev)) {
		LOG_ERR("%s: is not ready", gps_dev->name);
		return 0;
	}

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("%s: is not ready", gps_dev->name);
		return 0;
	}

	if (!device_is_ready(baro_dev)) {
		LOG_ERR("barometer not ready");
		return 0;
	}

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	struct uart_config uart_config = {
		.baudrate = 9600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};
	ret = uart_configure(gps_dev, &uart_config);
	uart_irq_callback_set(gps_dev, gps_uart_cb);
	uart_irq_rx_enable(gps_dev);

	if (ret < 0) {
		return 0;
	}

	struct sensor_value odr_attr;

	/* set accel/gyro sampling frequency to 12.5 Hz */
	odr_attr.val1 = 1;
	odr_attr.val2 = 0;
	// ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	// if (ret) {
	// 	LOG_ERR("Could not set imu accel XYZ odr");
	// }
	// ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	// if (ret) {
	// 	LOG_ERR("Could not set imu gyro XYZ odr");
	// }

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(1000);
		struct sensor_value press;
		struct sensor_value temp;
		struct sensor_value ax;
		struct sensor_value ay;
		struct sensor_value az;
		struct sensor_value gx;
		struct sensor_value gy;
		struct sensor_value gz;

		sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &ax);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &ay);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &az);

		sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gx);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gy);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gz);

		sensor_sample_fetch_chan(baro_dev, SENSOR_CHAN_ALL);
		sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

		// sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
		// sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		// sensor_channel_get(baro_dev, SENSOR)

		printf("accel: %f %f %f\n", sensor_to_float(ax), sensor_to_float(ay), sensor_to_float(az));
		printf("gyro: %f %f %f\n", sensor_to_float(gx), sensor_to_float(gy), sensor_to_float(gz));
		printf("temp: %f\n", sensor_to_float(temp));
		printf("pressure: %f\n", sensor_to_float(press));
	}
	return 0;
}