/* i2c_sht4x.c
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

/* SHT4X Commands */
#define CMD_MEASURE_HP         0xfd
#define CMD_MEASURE_MP         0xf6
#define CMD_MEASURE_LR         0xe0
#define CMD_READ_SERIAL        0x89
#define CMD_SOFT_RESET         0x94



static inline uint8_t crc8(uint8_t *buf, size_t len)
{
	return crc8_generic(buf, len, 0x31, 0xff, 0x00, false, false);
}


void* sht4x_init(i2c_inst_t *i2c, uint8_t addr)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	uint8_t buf[6];
	int res;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	memset(buf, 0, sizeof(buf));

	/* Read and verify device serial */
	res = i2c_read_register_block(i2c, addr, CMD_READ_SERIAL, buf, sizeof(buf), 1000);
	if (res)
		goto panic;

	/* Check CRC of received values */
	if ((crc8(&buf[0], 2) != buf[2]) || (crc8(&buf[3], 2) != buf[5])) {
		DEBUG_PRINT("invalid serial (CRC mismatch)\n");
		goto panic;
	}
#if I2C_DEBUG > 0
	uint32_t serial = (buf[0] << 24 | buf[1] << 16 | buf[3] << 8 | buf[4]);
	DEBUG_PRINT("sensor serial: %08lx\n", serial);
#endif

	/* Reset sensor */
	res = i2c_write_raw_u8(i2c, addr, CMD_SOFT_RESET, false);
	if (res)
		goto panic;
	sleep_us(1000);


	/* Read and verify device serial again */
	res = i2c_read_register_block(i2c, addr, CMD_READ_SERIAL, buf, sizeof(buf), 1000);
	if (res)
		goto panic;

	/* Check CRC of received values */
	if ((crc8(&buf[0], 2) != buf[2]) || (crc8(&buf[3], 2) != buf[5])) {
		DEBUG_PRINT("invalid serial (CRC mismatch)\n");
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int sht4x_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;

	/* Initiate measurement */
	res = i2c_write_raw_u8(c->i2c, c->addr, CMD_MEASURE_HP, false);
	if (res)
		return -1;

	return 9;  /* measurement should be available after 8.3ms */
}


int sht4x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
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
	if (crc8(&buf[0], 2) != buf[2])
		return -2;
	if (crc8(&buf[3], 2) != buf[5])
		return -3;

	t_raw = (buf[0] << 8) | buf[1];
	h_raw = (buf[3] << 8) | buf[4];
	DEBUG_PRINT("h_raw = %u, t_raw = %u\n", h_raw, t_raw);

	*temp = -45 + 175 * (double) t_raw / 65535;
	*pressure = -1.0;
	*humidity = -6 + 125 * (double) h_raw / 65535;
	if (*humidity > 100.0)
		*humidity = 100.0;
	if (*humidity < 0.0)
		*humidity = 0.0;

	DEBUG_PRINT("temp: %0.1f, humidity: %0.1f\n", *temp, *humidity);

	return 0;
}


