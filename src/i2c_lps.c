/* i2c_lps.c
   Copyright (C) 2025 Timo Kokkonen <tjko@iki.fi>

   SPDX-License-Identifier: GPL-3.0-or-later

   This file is part of pico-sensor-lib.

   pico-sensor-lib is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   pico-sensor-lib is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with pico-sensor-lib. If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "pico_sensor_lib/i2c.h"

/* LPS Common Registers */
#define WHO_AM_I         0x0f
#define STATUS_REG       0x27
#define PRES_OUT_XL      0x28
#define PRES_OUT_L       0x29
#define PRES_OUT_H       0x2a
#define TEMP_OUT_L       0x2b
#define TEMP_OUT_H       0x2c

/* LPS28/LPS33/LPS35 */
#define CTRL_REG1        0x10
#define CTRL_REG2        0x11
#define CTRL_REG3        0x12
#define CTRL_REG4        0x13
#define FIFO_CTRL        0x14

/* LPS22 */
#define LPS22_FIFO_CTRL  0x13

/* LPS25 */
#define LPS25_RES_CONF   0x10
#define LPS25_CTRL_REG1  0x20
#define LPS25_CTRL_REG2  0x21
#define LPS25_CTRL_REG3  0x22
#define LPS25_CTRL_REG4  0x23
#define LPS25_FIFO_CTRL  0x2e

/* Device IDs */
#define LPS22_DEVICE_ID      0xb3
#define LPS25_DEVICE_ID      0xbd
#define LPS28_DEVICE_ID      0xb4
#define LPS33_DEVICE_ID      0xb1
#define LPS35_DEVICE_ID      0xb1


typedef struct lps_sensor_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	uint8_t id; // device ID
	uint8_t model;  // index to models[] table
} lps_sensor_context_t;


struct sensor_models {
	const char *name;
	uint8_t device_id;
	const uint8_t *init_cmds;
	uint16_t meas_duration;
};


static const uint8_t lps22_init_cmds[] = {
	CTRL_REG1, 0x3c, // Set ODR 25Hz, EN_LPFP, LPFP_CFG (ODR/20)
	CTRL_REG2, 0x12, // Set IF_ADD_INC & LOW_NOISE_EN
	0,
};

static const uint8_t lps25_init_cmds[] = {
	LPS25_RES_CONF, 0x0f, // Set Pressure and Temperature resolution
	LPS25_CTRL_REG1, 0xb0, // Set active mode and ODR 12.5Hz
	LPS25_FIFO_CTRL, 0xdf, // Set FIFO to Mean mode (32 sample moving average)
	0,
};

static const uint8_t lps28_init_cmds[] = {
	CTRL_REG1, 0x24, // Set ODR 25Hz, Averaging 64 samples
	CTRL_REG2, 0x30, // Set EN_LPFP, LPFP_CFG (ODR/9)
	CTRL_REG3, 0x01, // Set IF_ADD_INC
	FIFO_CTRL, 0x00, // Set FIFO to Bypass mode
	0,
};

static const uint8_t lps33_init_cmds[] = {
	CTRL_REG1, 0x3c, // Set ODR 25Hz, EN_LPFP, LPFP_CFG (ODR/20)
	CTRL_REG2, 0x10, // Set IF_ADD_INC
	0,
};

static const struct sensor_models models[] = {
	{ "LPS22", LPS22_DEVICE_ID, lps22_init_cmds, 800 },
	{ "LPS25", LPS25_DEVICE_ID, lps25_init_cmds, 1000 },
	{ "LPS28", LPS28_DEVICE_ID, lps28_init_cmds, 400 },
	{ "LPS33/LPS35", LPS33_DEVICE_ID, lps33_init_cmds, 800 },
	{ NULL, 0 },
};


void* lps_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	int idx;
	const uint8_t *cmds;
	uint8_t reg;
	uint8_t val = 0;
	lps_sensor_context_t *ctx = calloc(1, sizeof(lps_sensor_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read and verify device ID */
	if (i2c_read_register_u8(i2c, addr, WHO_AM_I, &val)) {
		*result = -1;
		goto panic;
	}

	/* Check if this is a sensor supported by this driver... */
	idx = 0;
	while (models[idx].name) {
		if (models[idx].device_id == val) {
			DEBUG_PRINT("Sensor model: %s\n", models[idx].name);
			ctx->id = val;
			ctx->model = idx;
			break;
		}
		idx++;
	}
	if (!ctx->id) {
		*result = -2;
		goto panic;
	}

	/* Reset Sensor */
	reg = (ctx->id == LPS25_DEVICE_ID ? LPS25_CTRL_REG2 : CTRL_REG2);

	if (i2c_write_register_u8(i2c, addr, reg, 0x04)) { // SWRESET
		*result = -3;
		goto panic;
	}
	sleep_us(5);
	if (i2c_write_register_u8(i2c, addr, reg, 0x80)) { // BOOT
		*result = -4;
		goto panic;
	}
	sleep_us(2300);

	/* Read configuration register */
	if (i2c_read_register_u8(i2c, addr, reg, &val)) {
		*result = -5;
		goto panic;
	}
	/* Check that boot is complete */
	if (val & 0x80) {
		*result = -6;
		goto panic;
	}

	/* Configure Sensor */
	cmds = models[idx].init_cmds;
	while (*cmds) {
		reg = *cmds++;
		val = *cmds++;
		DEBUG_PRINT("Init: reg%02x = %02x\n", reg, val);
		if (i2c_write_register_u8(i2c, addr, reg, val)) {
			*result = -7;
			goto panic;
		}
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int lps_start_measurement(void *ctx)
{
	lps_sensor_context_t *c = (lps_sensor_context_t*)ctx;


        /* Nothing to do, sensor is in continuous measurement mode... */

	return models[c->model].meas_duration;
}


int lps_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	lps_sensor_context_t *c = (lps_sensor_context_t*)ctx;
	int res;
	uint8_t reg, val;
	uint8_t buf[5];
	int32_t t_raw, p_raw;


	/* Read status register */
	res = i2c_read_register_u8(c->i2c, c->addr, STATUS_REG, &val);
	if (res)
		return -1;

	/* Check P_DA and T_DA (data available) bits */
	if ((val & 0x03) != 0x03)
		return 1;

	/* Get Measurement */
	reg = PRES_OUT_XL;
	if (c->id == LPS25_DEVICE_ID)
		reg |= 0x80;
	res = i2c_read_register_block(c->i2c, c->addr, reg, buf, sizeof(buf), 0);
	if (res)
		return -2;

	p_raw = twos_complement((uint32_t)((buf[2] << 16) | (buf[1] << 8) | buf[0]), 24);
	t_raw = twos_complement((uint32_t)(((buf[4] << 8) | buf[3])), 16);

	DEBUG_PRINT("t_raw = %ld, p_raw = %ld\n", t_raw, p_raw);

	if (c->id == LPS25_DEVICE_ID) {
		*temp = 42.5 + t_raw /480.0;
	} else {
		*temp = t_raw / 100.0;
	}
	*pressure = p_raw / 4096.0;
	*humidity = -1.0;

	DEBUG_PRINT("temp = %0.1f C, pressure = %0.1f hPa\n", *temp, *pressure);

	return 0;
}


