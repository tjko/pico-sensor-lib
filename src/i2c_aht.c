/* i2c_aht.c
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


/* AHT1x / AHT2x Registers */

#define AHT_REG_STATUS       0x71
#define AHT_REG_START_MEAS   0xac
#define AHT_REG_RESET        0xba
#define AHT2_REG_INIT        0xbe
#define AHT1_REG_INIT        0xe1


typedef struct aht_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	uint8_t type;  // 1 = AHT1x, 2 = AHX2x
} aht_context_t;



static inline uint8_t aht_crc8(uint8_t *buf, uint len)
{
	return crc8_generic(buf, len, 0x31, 0xff, 0x00, false, false);
}


static void* aht_init(i2c_inst_t *i2c, uint8_t addr, uint8_t type)
{
	int res;
	uint8_t buf[2];
	aht_context_t *ctx = calloc(1, sizeof(aht_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;
	ctx->type = type;

	/* Reset sensor*/
	res = i2c_write_raw_u8(i2c, addr, AHT_REG_RESET, false);
	if (res)
		goto panic;

	/* Wait for sensor to soft reset (should be done in 20ms?)  */
	sleep_us(25000);

	/* Write initialization register: set normal mode */
	res = i2c_write_register_u16(i2c, addr,
				(type == 1 ? AHT1_REG_INIT : AHT2_REG_INIT),
				0x0800);
	if (res)
		goto panic;

	/* Read status register */
	res = i2c_read_register_u8(i2c, addr, AHT_REG_STATUS, buf);
	if (res)
		goto panic;

	/* Check that calibration bit is on */
	if ((buf[0] & 0x08) == 0)
		goto panic;

	return ctx;

panic:
	free(ctx);
	return NULL;
}


void* aht1x_init(i2c_inst_t *i2c, uint8_t addr, uint8_t type)
{
	return aht_init(i2c, addr, 1);
}


void* aht2x_init(i2c_inst_t *i2c, uint8_t addr, uint8_t type)
{
	return aht_init(i2c, addr, 2);
}


int aht_start_measurement(void *ctx)
{
	aht_context_t *c = (aht_context_t*)ctx;
	int res;

	/* Send start measurement command */
	res = i2c_write_register_u16(c->i2c, c->addr, AHT_REG_START_MEAS, 0x3300);
	if (res) {
		DEBUG_PRINT("aht: start measure failed: %d\n", res);
	}

	return 100;  /* measurement should be available after 100ms */
}


int aht_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	aht_context_t *c = (aht_context_t*)ctx;
	int res;
	uint8_t buf[7];
	uint32_t raw_t, raw_h;
	int len = c->type == 1 ? 6 : 7;

	/* Read data from sensor... */
	res = i2c_read_raw(c->i2c, c->addr, buf, len, false);
	if (res)
		return -1;

	DEBUG_PRINT("aht: len=%d, status=%02x\n", len, buf[0]);

	/* Check busy bit */
	if ((buf[0] & 0x80))
		return -2;

	/* Check CRC */
	if (c->type == 2) {
		if (aht_crc8(buf, len - 1) != buf[len - 1]) {
			DEBUG_PRINT("aht2x: CRC mismatch\n");
			return -3;
		}
	}

	raw_h = (buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4);
	if (raw_h > 0x100000)
		raw_h = 0x100000;
	raw_t = ((buf[3] & 0x0f) << 16) | (buf[4] << 8) | buf[5];


	*temp = ((double)raw_t / 0x100000) * 200 - 50;
	*humidity = ((double)raw_h / 0x100000) * 100;
	*pressure = -1.0;

	return 0;
}

