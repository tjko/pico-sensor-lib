/* i2c_mpl3115a2.c
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


/* MPL3115A2 Registers */
#define STATUS         0x00
#define OUT_P_MSB      0x01
#define OUT_P_CSB      0x02
#define OUT_P_LSB      0x03
#define OUT_T_MSB      0x04
#define OUT_T_LSB      0x05
#define WHO_AM_I       0x0c
#define F_STATUS       0x0d
#define F_SETUP        0x0f
#define SYSMOD         0x11
#define PT_DATA_CFG    0x13
#define CTRL_REG1      0x26
#define CTRL_REG2      0x27
#define CTRL_REG3      0x28
#define CTRL_REG4      0x29
#define CTRL_REG5      0x2a

#define MPL3115A2_DEVICE_ID 0xc4


void* mpl3115a2_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	int res;
	int count = 0;
	uint8_t val;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read Device ID */
	if (i2c_read_register_u8(i2c, addr, WHO_AM_I, &val)) {
		*result = -1;
		goto panic;
	}
	if (val != MPL3115A2_DEVICE_ID) {
		*result = -2;
		goto panic;
	}

	/* Reset Sensor */
	if (i2c_write_register_u8(i2c, addr, CTRL_REG1, 0x04)) {
		*result = -3;
		goto panic;
	}
	sleep_ms(1);
	while (1) {
		if ((res = i2c_read_register_u8(i2c, addr, CTRL_REG1, &val))) {
			DEBUG_PRINT("Read failed: %d\n", res);
		} else {
			DEBUG_PRINT("CTRL_REG1=%02x\n", val);
			if ((val & 0x04) == 0)
				break;
		}
		if (count++ >= 10) {
			*result = -4;
			goto panic;
		}
		sleep_ms(5);
	}

	/* Disable FIFO */
	if (i2c_write_register_u8(i2c, addr, F_SETUP, 0x00)) {
		*result = -5;
		goto panic;
	}

	/* Set Barometer mode with oversampling ratio 128 */
	if (i2c_write_register_u8(i2c, addr, CTRL_REG1, 0x38)) {
		*result = -6;
		goto panic;
	}

	/* Enable event flags */
	if (i2c_write_register_u8(i2c, addr, PT_DATA_CFG, 0x07)) {
		*result = -7;
		goto panic;
	}

	/* Activate sensor */
	if (i2c_write_register_u8(i2c, addr, CTRL_REG1, 0x39)) {
		*result = -8;
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int mpl3115a2_start_measurement(void *ctx)
{
	/* Nothing to do, sensor in continuous measurement mode. */

	return 512;  /* measurement should be available after 512ms */
}


int mpl3115a2_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint8_t buf[6];
	uint32_t p_raw;
	int32_t t_raw;

	/* Read measurement data (P&T) */
	if ((res = i2c_read_register_block(c->i2c, c->addr, STATUS, buf, sizeof(buf), 0)))
		return -1;

	if (!(buf[0] & 0x08))
		return -2;

	p_raw = ((buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4));
	t_raw = twos_complement((buf[4] << 4) | (buf[5] >> 4), 12);

	DEBUG_PRINT("p_raw = %lu (%lx), t_raw = %ld (%lx)\n", p_raw, p_raw, t_raw, t_raw);

	*temp = (float)t_raw / (1 << 4);
	*pressure = ((float)p_raw / (1 << 2)) / 100;
	*humidity = -1.0;

	DEBUG_PRINT("temp = %0.1f C, pressure = %0.1f hPa\n", *temp, *pressure);

	return 0;
}

