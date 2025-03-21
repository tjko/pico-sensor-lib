/* i2c_hts221.c
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

/* HTS221 Registers */
#define WHO_AM_I         0x0f
#define AV_CONF          0x10
#define CTRL_REG1        0x20
#define CTRL_REG2        0x21
#define CTRL_REG3        0x22
#define STATUS           0x27
#define HUM_OUT_L        0x28
#define HUM_OUT_H        0x29
#define TEMP_OUT_L       0x2a
#define TEMP_OUT_H       0x2b
#define CALIB_DATA       0x30

#define HTS221_DEVICE_ID      0xbc


typedef struct hts221_sensor_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	uint8_t H0_x2;
	uint16_t T0_x8;
	int16_t T0_raw;
	int16_t H0_raw;
	float H_f;
	float T_f;
} hts221_sensor_context_t;



void* hts221_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	int res, retries;
	uint8_t val = 0;
	hts221_sensor_context_t *ctx = calloc(1, sizeof(hts221_sensor_context_t));
	uint8_t buf[16];

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read and verify device ID */
	retries = 3;
	do {
		res = i2c_read_register_u8(i2c, addr, WHO_AM_I, &val);
	} while (res && retries-- > 0);
	if (res) {
		*result = -1;
		goto panic;
	}
	if (val != HTS221_DEVICE_ID) {
		*result = -2;
		goto panic;
	}

	/* Reset Sensor */
	if (i2c_write_register_u8(i2c, addr, CTRL_REG2, 0x80)) { // BOOT
		*result = -3;
		goto panic;
	}
	sleep_ms(5);

	/* Read configuration register */
	if (i2c_read_register_u8(i2c, addr, CTRL_REG2, &val)) {
		*result = -4;
		goto panic;
	}
	/* Check that boot is complete */
	if (val & 0x80) {
		*result = -5;
		goto panic;
	}

	/* Read Calibration Data */
	if (i2c_read_register_block(i2c, addr, CALIB_DATA | 0x80, buf , sizeof(buf), false)) {
		*result = -6;
		goto panic;
	}

	uint8_t H0_rH_x2 = buf[0];
	uint8_t H1_rH_x2 = buf[1];
	uint16_t T0_degC_x8 = (uint16_t)(buf[5] & 0x03) << 8 | buf[2];
	uint16_t T1_degC_x8 = (uint16_t)(buf[5] & 0x0c) << 6 | buf[3];
	int16_t H0_T0_OUT = (int16_t)((buf[7] << 8) | buf[6]);
	int16_t H1_T0_OUT = (int16_t)((buf[11] << 8) | buf[10]);
	int16_t T0_OUT = (int16_t)((buf[13] << 8) | buf[12]);
	int16_t T1_OUT = (int16_t)((buf[15] << 8) | buf[14]);

	ctx->H0_x2 = H0_rH_x2;
	ctx->T0_x8 = T0_degC_x8;
	ctx->T0_raw = T0_OUT;
	ctx->H0_raw = H0_T0_OUT;
	ctx->H_f = (float)(H1_rH_x2 - H0_rH_x2) / (H1_T0_OUT - H0_T0_OUT);
	ctx->T_f = (float)(T1_degC_x8 - T0_degC_x8) / (T1_OUT - T0_OUT);

	DEBUG_PRINT("H0_rH_x2 = %u (%f)\n", H0_rH_x2, H0_rH_x2 / 2.0);
	DEBUG_PRINT("H1_rH_x2 = %u (%f)\n", H1_rH_x2, H1_rH_x2 / 2.0);
	DEBUG_PRINT("T0_degC_x8 = %u (%f)\n", T0_degC_x8, T0_degC_x8 / 8.0);
	DEBUG_PRINT("T1_degC_x8 = %u (%f)\n", T1_degC_x8, T1_degC_x8 / 8.0);
	DEBUG_PRINT("H0_T0_OUT / H1_T0_OUT = %d / %d\n", H0_T0_OUT, H1_T0_OUT);
	DEBUG_PRINT("T0_OUT / T1_OUT = %d / %d\n", T0_OUT, T1_OUT);

	/* Set resolution: AVGT=64, AVGH=64 */
	if (i2c_write_register_u8(i2c, addr, AV_CONF, 0x2c)) {
		*result = -7;
		goto panic;
	}

	/* Set active mode and ODR 12.5Hz */
	if (i2c_write_register_u8(i2c, addr, CTRL_REG1, 0x83)) {
		*result = -8;
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int hts221_start_measurement(void *ctx)
{
	/* Nothing to do, sensor is in continuous measurement mode... */

	return 80;  /* measurement should be available after 80ms */
}


int hts221_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	hts221_sensor_context_t *c = (hts221_sensor_context_t*)ctx;
	int res;
	uint8_t val;
	uint8_t buf[4];
	int32_t t_raw, h_raw;


	/* Read status register */
	if ((res = i2c_read_register_u8(c->i2c, c->addr, STATUS, &val)))
		return -1;

	/* Check P_DA and T_DA (data available) bits */
	if ((val & 0x03) != 0x03)
		return 1;

	/* Get Measurement */
	if ((res = i2c_read_register_block(c->i2c, c->addr, HUM_OUT_L | 0x80, buf, sizeof(buf), 0)))
		return -2;

	h_raw = (int16_t)((buf[1] << 8) | buf[0]);
	t_raw = (int16_t)((buf[3] << 8) | buf[2]);
	DEBUG_PRINT("t_raw = %ld, h_raw = %ld\n", t_raw, h_raw);

	*temp = (c->T0_x8 + (t_raw - c->T0_raw) * c->T_f) / 8.0;
	*humidity = (c->H0_x2 + (h_raw - c->H0_raw) * c->H_f) / 2.0;
	*pressure = -1.0;

	DEBUG_PRINT("temp = %0.1f C, humidity = %0.1f %%\n", *temp, *humidity);

	return 0;
}


