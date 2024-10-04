/* i2c_sht3x.c
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

#include "pico_sensor_lib/i2c.h"
#include "pico_sensor_lib/crc.h"

/* SHT3X Commands */
#define CMD_MEASURE            0x2400 // high repeatability, clock stretching disabled
#define CMD_FETCH              0xe000
#define CMD_ART                0x2b32
#define CMD_BREAK              0x3093
#define CMD_RESET              0x30a2
#define CMD_HEATER_ON          0x306d
#define CMD_HEATER_OFF         0x3066
#define CMD_STATUS             0xf32d
#define CMD_CLEAR_STATUS       0x3041



static inline uint8_t crc8(uint8_t *buf, size_t len)
{
	return crc8_generic(buf, len, 0x31, 0xff, 0x00, false, false);
}


static int sht3x_read_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t *val, bool nostop)
{
	uint8_t buf[3], crc;
	int res;

	DEBUG_PRINT("args=%p,%02x,%p\n", i2c, addr, val);

	res = i2c_read_raw(i2c, addr, buf, 3, nostop);
	if (res) {
		DEBUG_PRINT("read failed (%d)\n", res);
		return -1;
	}

	*val = (buf[0] << 8) | buf[1];
	crc = crc8(buf, 2);
	DEBUG_PRINT("read ok: [%02x %02x (%02x)] %04x (%u), crc=%02x\n", buf[0], buf[1], buf[2], *val, *val, crc);

	if (crc != buf[2]) {
		DEBUG_PRINT("checksum mismatch!\n");
		return -2;
	}

	return 0;
}


void* sht3x_init(i2c_inst_t *i2c, uint8_t addr)
{
	int res;
	uint16_t val = 0;
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;


	/* Read status register to verify correct device... */
	res = i2c_write_raw_u16(i2c, addr, CMD_STATUS, false);
	if (res)
		goto panic;
	sleep_us(10);
	res = sht3x_read_u16(i2c, addr, &val, false);
	if (res)
		goto panic;
	if ((val & 0x500c) != 0)
		goto panic;


	/* Reset sensor */
	res = i2c_write_raw_u16(i2c, addr, CMD_RESET, false);
	if (res)
		goto panic;
	sleep_us(1500);


	/* Read status register again... */
	res = i2c_write_raw_u16(i2c, addr, CMD_STATUS, false);
	if (res)
		goto panic;
	sleep_us(10);
	res = sht3x_read_u16(i2c, addr, &val, false);
	if (res)
		goto panic;


	return ctx;

panic:
	free(ctx);
	return NULL;
}


int sht3x_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;

	/* Initiate measurement */
	res = i2c_write_raw_u16(c->i2c, c->addr, CMD_MEASURE, false);
	if (res)
		return -1;

	return 16;  /* measurement should be available after 15.5ms */
}


int sht3x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint8_t buf[6];
	uint16_t t_raw = 0;
	uint16_t h_raw = 0;

	/* Get Measurement */
	res = i2c_read_raw(c->i2c, c->addr, buf, 6, false);
	if (res)
		return -1;

	/* Check CRC of received values */
	if ((crc8(&buf[0], 2) != buf[2]) || (crc8(&buf[3], 2) != buf[5]))
		return -2;

	t_raw = (buf[0] << 8) | buf[1];
	h_raw = (buf[3] << 8) | buf[4];
	DEBUG_PRINT("h_raw = %u, t_raw = %u\n", h_raw, t_raw);

	*temp = -45 + 175 * (double) t_raw / 65535;
	*pressure = -1.0;
	*humidity = 100 * (double) h_raw / 65535;
	DEBUG_PRINT("temp: %0.1f, humidity: %0.1f\n", *temp, *humidity);

	return 0;
}


