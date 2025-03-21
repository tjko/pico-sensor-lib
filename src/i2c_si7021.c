/* i2c_si7021.c
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
#include "pico_sensor_lib/crc.h"

/* Si7021 Registers */
#define CMD_MEASURE_RH_HOLD         0xe5
#define CMD_MEASURE_RH_NO_HOLD      0xf5
#define CMD_MEASURE_T_HOLD          0xe3
#define CMD_MEASURE_T_NO_HOLD       0xf3
#define CMD_READ_PREV_TEMP          0xe0
#define CMD_RESET                   0xfe
#define CMD_WRITE_USER_REG          0xe6
#define CMD_READ_USER_REG           0xe7
#define CMD_WRITE_CTRL_REG          0x51
#define CMD_READ_CTRL_REG           0x11

#define CMD_READ_ID_1               0xfa0f
#define CMD_READ_ID_2               0xfcc9
#define CMD_READ_FW_VER             0x84b8

#define SI7021_DEVICE_ID 0x15



void* si7021_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	uint8_t buf[8], c, user_reg, ctrl_reg;
	uint64_t serial;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;


	/* Read Serial Number (Part 1) */
	if (i2c_write_raw_u16(i2c, addr, CMD_READ_ID_1, true)) {
		*result = -1;
		goto panic;
	}
	if (i2c_read_raw(i2c, addr, buf, 8, false)) {
		*result = -2;
		goto panic;
	}

	/* Validate checksums */
	c = 0;
	for (int i = 0; i < 4; i++) {
		c = crc8_byte(c, buf[i * 2], 0x31);
		DEBUG_PRINT("SNA_%d=%02x, CRC=%02x (%02x)\n",
			3 - i, buf[i * 2], buf[i * 2 + 1], c);
		if (c != buf[i * 2 + 1]) {
			*result = -3;
			goto panic;
		}
	}
	serial = ((uint64_t)buf[0] << 56) | ((uint64_t)buf[2] << 48)
		| ((uint64_t)buf[4] << 40) | ((uint64_t)buf[6] << 32);


	/* Read Serial Number (Part 2) */
	if (i2c_write_raw_u16(i2c, addr, CMD_READ_ID_2, true)) {
		*result = -4;
		goto panic;
	}
	if (i2c_read_raw(i2c, addr, buf, 6, false)) {
		*result = -5;
		goto panic;
	}

	/* Validate checksums */
	c = crc8_generic(&buf[0], 2, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("SNB_3=%02x, SNB_2=%02x CRC=%02x (%02x)\n", buf[0], buf[1], buf[2], c);
	if (c != buf[2]) {
		*result = -6;
		goto panic;
	}
	c = crc8_generic(&buf[3], 2, 0x31, c, 0x00, false, false);
	DEBUG_PRINT("SNB_1=%02x, SNB_0=%02x CRC=%02x (%02x)\n", buf[3], buf[4], buf[5], c);
	if (c != buf[5]) {
		*result = -7;
		goto panic;
	}

	serial |= (buf[0] << 24) | (buf[1] << 16) | (buf[3] << 8) | buf[4];
	DEBUG_PRINT("Serial number: %16llx\n", serial);


	/* Check Model Identifier */
	c = (serial >> 24) & 0xff;
	if (c != SI7021_DEVICE_ID) {
		DEBUG_PRINT("Unknown device ID: %02x\n", c);
		*result = -8;
		goto panic;
	}
	DEBUG_PRINT("Device Model: 70%02d\n", c);


	/* Reset Sensor */
	if (i2c_write_raw_u8(i2c, addr, CMD_RESET, false)) {
		*result = -9;
		goto panic;
	}
	sleep_ms(15);

	/* Read User Register */
	if (i2c_read_register_u8(i2c, addr, CMD_READ_USER_REG, &user_reg)) {
		*result = -10;
		goto panic;
	}

	/* Read Control Register */
	if (i2c_read_register_u8(i2c, addr, CMD_READ_CTRL_REG, &ctrl_reg)) {
		*result = -11;
		goto panic;
	}

	DEBUG_PRINT("User register: %02x\n", user_reg);
	DEBUG_PRINT("Control register: %02x\n", ctrl_reg);


	/* Configure Sensor */
	user_reg &= ~0x81; // Clear bits RES1, RES0 (RH 12bit, Temp 14bit)
	user_reg &= ~0x04; // Clear HTRE bit (disable header)
	ctrl_reg &= ~0x0f; // Clear HEATER[3:0] (Set heater to lowest setting).

	if (i2c_write_register_u8(i2c, addr, CMD_WRITE_CTRL_REG, ctrl_reg)) {
		*result = -12;
		goto panic;
	}
	DEBUG_PRINT("Set User register: %02x\n", user_reg);
	if (i2c_write_register_u8(i2c, addr, CMD_WRITE_USER_REG, user_reg)) {
		*result = -13;
		goto panic;
	}
	DEBUG_PRINT("Set Control register: %02x\n", ctrl_reg);

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int si7021_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;

	/* Start RH Measurement */
	if ((res = i2c_write_raw_u8(c->i2c, c->addr, CMD_MEASURE_RH_NO_HOLD, false)))
		return -1;

	return 23; /* Results should be available after 22.8ms */
}


int si7021_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint16_t val;
	int32_t rh;
	uint8_t buf[3], crc;


	/* Get RH Measurement */
	if ((res = i2c_read_raw(c->i2c, c->addr, buf, 3, false)))
		return -1;
	val = buf[0] << 8 | buf[1];
	crc = crc8_generic(buf, 2, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("RAW: %02x %02x %02x (crc=%02x)\n", buf[0], buf[1], buf[2], crc);
	if (buf[2] != crc)
		return -2;

	rh = (125 * val) / 65536 - 6;
	if (rh < 0)
		rh = 0;
	if (rh > 100)
		rh = 100;
	DEBUG_PRINT("RH: %ld (raw=%u)\n", rh, val);

	/* Get Last Temp Measurement */
	if ((res = i2c_read_register_u16(c->i2c, c->addr, CMD_READ_PREV_TEMP, &val)))
		return -3;

	*temp = (175.72 * val) / 65536 - 46.85;
	DEBUG_PRINT("Temp: %f (raw=%u)\n", *temp, val);

	*humidity = rh;
	*pressure = -1.0;

	return 0;
}


