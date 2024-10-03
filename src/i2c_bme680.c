/* i2c_bme680.c
   Copyright (C) 2024 Timo Kokkonen <tjko@iki.fi>

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
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "i2c.h"

/* BME680 Registers */
#define REG_EAS_STATUS_0  0x1d
#define REG_PRESS_MSB     0x1f
#define REG_PRESS_LSB     0x20
#define REG_PRESS_XLSB    0x21
#define REG_TEMP_MSB      0x22
#define REG_TEMP_LSB      0x23
#define REG_TEMP_XLSB     0x24
#define REG_HUM_MSB       0x25
#define REG_HUM_LSB       0x26
#define REG_GAS_R_MSB     0x2a
#define REG_GAS_R_LSB     0x2b
#define REG_IDAC_HEAT_X   0x50 // 0x59...0x50
#define REG_HEAT_X        0x5a // 0x63...0x5a
#define REG_GAS_WAIT_X    0x64 // 0x6d...0x64
#define REG_CTRL_GAS_0    0x70
#define REG_CTRL_GAS_1    0x71
#define REG_CTRL_HUM      0x72
#define REG_STATUS        0x73
#define REG_CTRL_MEAS     0x74
#define REG_CONFIG        0x75
#define REG_ID            0xd0
#define REG_RESET         0xe0 // write 0xb6 to reset

#define REG_CALIB         0xeb
#define REG_HEAT_RANGE    0x02
#define REG_HEAT_VAL      0x00



#define BME680_DEVICE_ID  0x61


typedef struct bme680_context_t {
	i2c_inst_t *i2c;
	uint8_t addr;
	/* Calibration values */
	uint8_t par_g1;
	uint16_t par_g2;
	uint8_t par_g3;
	uint8_t res_heat_range;
	int8_t res_heat_val;
	/* Saved Register values */
	uint8_t ctrl_meas;
} bme680_context_t;



static uint8_t calculate_res_heat_x(bme680_context_t *ctx, float target_temp, float amb_temp)
{
	double var1 = ((double)ctx->par_g1 / 16.0) + 49.0;
	double var2 = (((double)ctx->par_g2 / 32768.0) * 0.0005) + 0.00235;
	double var3 = (double)ctx->par_g3 / 1024.0;
	double var4 = var1 * (1.0 + (var2 * (double)target_temp));
	double var5 = var4 + (var3 * (double)amb_temp);

	uint8_t x = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)ctx->res_heat_range)) * (1.0 / (1.0 + ((double)ctx->res_heat_val * 0.002)))) - 25));

	DEBUG_PRINT("calculate_res_heax_x(%p,%f,%f): %u\n", ctx, target_temp, amb_temp, x);

	return x;
}


void* bme680_init(i2c_inst_t *i2c, uint8_t addr)
{
	bme680_context_t *ctx = calloc(1, sizeof(bme680_context_t));
	uint8_t buf[6];
	uint8_t val = 0;
	int res;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;


	/* Read and verify device ID */
	res  = i2c_read_register_u8(i2c, addr, REG_ID, &val);
	if (res || (val  != BME680_DEVICE_ID))
		goto panic;

	/* Reset Sensor */
	res = i2c_write_register_u8(i2c, addr, REG_RESET, 0xb6);
	if (res)
		goto panic;

	/* Wait for sensor to soft reset (reset should take 2ms per datasheet)  */
	sleep_ms(10);

	/* Read Calibration parameters */
	res = i2c_read_register_block(i2c, addr, REG_CALIB, buf, 4, 0);
	if (res)
		goto panic;
	ctx->par_g1 = buf[2];
	ctx->par_g2 = (buf[1] << 8) | buf[0];
	ctx->par_g3 = buf[3];
	DEBUG_PRINT("par_g1 = %u\n", ctx->par_g1);
	DEBUG_PRINT("par_g1 = %u\n", ctx->par_g2);
	DEBUG_PRINT("par_g1 = %d\n", ctx->par_g3);

	res = i2c_read_register_u8(i2c, addr, REG_HEAT_RANGE, &val);
	if (res)
		goto panic;
	ctx->res_heat_range = (val >> 4) & 0x03;
	DEBUG_PRINT("res_heat_range = %u (%u)\n", ctx->res_heat_range, val);

	res = i2c_read_register_u8(i2c, addr, REG_HEAT_VAL, &val);
	if (res)
		goto panic;
	ctx->res_heat_val = val;
	DEBUG_PRINT("res_heat_val = %u\n", ctx->res_heat_val);


	/* Initialize sensor */
	buf[0] = 0x00; // CTRL_GAS_0:
	buf[1] = 0x10; // CTRL_GAS_1: run_gas
	buf[2] = 0x05; // CTRL_HUM: 16x oversampling
	buf[3] = 0x00; // STATUS:
	buf[4] = 0xb4; // CTRL_MEAS: temperature: 16x oversampling, pressure: 16x oversampling
	buf[5] = 0x10; // CONFIG: filter coefficient 15

	ctx->ctrl_meas = buf[4] & 0xfc;

	res = i2c_write_register_block(i2c, addr, REG_CTRL_GAS_0, buf, 6, 0);
	if (res)
		goto panic;

	/* Set heat up duration to 100ms */
	res = i2c_write_register_u8(i2c, addr, REG_GAS_WAIT_X, 0x59);
	if (res)
		goto panic;

	/* Set heater temperature to 300C */
	res = i2c_write_register_u8(i2c, addr, REG_HEAT_X, calculate_res_heat_x(ctx, 300.0, 25.0));
	if (res)
		goto panic;

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int bme680_start_measurement(void *ctx)
{
	bme680_context_t *c = (bme680_context_t*)ctx;
	int res;

	res = i2c_write_register_u8(c->i2c, c->addr, REG_CTRL_MEAS, c->ctrl_meas | 0x01);
	if (res)
		return -1;

	return 3000;  /* measurement should be available after 3s */
}


int bme680_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	bme680_context_t *c = (bme680_context_t*)ctx;
	int res;
	uint8_t buf[10];
	int32_t p_raw, t_raw, h_raw, gas;


	/* Read Pressure & Temperature registers */
	res = i2c_read_register_block(c->i2c, c->addr, REG_PRESS_MSB, buf, sizeof(buf), 0);
	if (res)
		return -2;


	p_raw = twos_complement((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4), 20);
	t_raw = twos_complement((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4), 20);
	h_raw = twos_complement((buf[6] << 8) | buf[7], 16);
	gas = twos_complement((buf[8] << 2) | (buf[9] >> 6), 10);


	DEBUG_PRINT("p_raw = %ld, t_raw = %ld, h_raw = %ld, gas = %ld\n",
		p_raw, t_raw, h_raw, gas);

	*temp = -1.0;
	*pressure = -1.0;
	*humidity = -1.0;

	return 0;
}
