/* i2c_htu21d.c
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


/* HTU21D Commands */

#define TEMP_MEAS        0xf3 // No Hold Master
#define HUM_MEAS         0xf5 // No Hold Master
#define WRITE_USER       0xe6
#define READ_USER        0xe7
#define SOFT_RESET       0xfe


typedef struct htu21d_sensor_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	uint8_t state;
	float temp;
	float humidity;
} htu21d_sensor_context_t;


void* htu21d_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	uint8_t val = 0;
	uint8_t new_val;
	htu21d_sensor_context_t *ctx = calloc(1, sizeof(htu21d_sensor_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;
	ctx->state = 0;
	ctx->temp = 0.0;
	ctx->humidity = -1.0;

	/* Soft Reset */
	if (i2c_write_raw_u8(i2c, addr, SOFT_RESET, false)) {
		*result = -1;
		goto panic;
	}
	sleep_ms(15);

	/* Read Status Register */
	if (i2c_read_register_u8(i2c, addr, READ_USER, &val)) {
		*result = -2;
		goto panic;
	}
	DEBUG_PRINT("Status register: %02x\n", val);

	/* Set resolution 12bits (RH) / 14bits (Temp) */
	new_val = val & ~(0x81);
	if (val != new_val) {
		DEBUG_PRINT("Set status register to: %02x\n", new_val);
		if (i2c_write_register_u8(i2c, addr, WRITE_USER, new_val)) {
			*result = -3;
			goto panic;
		}
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int htu21d_start_measurement(void *ctx)
{
	htu21d_sensor_context_t *c = (htu21d_sensor_context_t*)ctx;
	uint8_t cmd = (c->state == 0 ? TEMP_MEAS : HUM_MEAS);
	int res;

	if ((res = i2c_write_raw_u8(c->i2c, c->addr, cmd, false))) {
		DEBUG_PRINT("start_measurement failed: %d\n", res);
		return -1;
	}

	return 50;  /* measurement should be available after 50ms */
}


int htu21d_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	htu21d_sensor_context_t *c = (htu21d_sensor_context_t*)ctx;
	int res;
	uint8_t buf[3], crc;
	uint16_t raw;


	/* Get Measurement */
	if ((res = i2c_read_raw(c->i2c, c->addr, buf, sizeof(buf), false)))
		return -1;
	crc = crc8_generic(buf, 2, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("Raw %s measurement: %02x %02x %02x [crc=%02x]\n",
		(c->state == 0 ? "Temp" : "RH"), buf[0], buf[1], buf[2], crc);
	if (buf[2] != crc) {
		DEBUG_PRINT("CRC error\n");
		return -2;
	}
	if ((buf[0] == 0 && buf[1] == 0) | (buf[0] == 0xff && buf[1] == 0xff)) {
		DEBUG_PRINT("Diagnostic state: %s\n",
			((buf[1] & 0x03) == 0x03 ? "Short Circuit" : "Open Circuit"));
		return -3;
	}

	raw = (buf[0] << 8) | (buf[1] & 0xfc);

	if (c->state == 0) {
		/* Process Temperature Measurement */
		c->temp = -46.85 + 175.72 * raw / 65536;
		DEBUG_PRINT("Temp = %f\n", c->temp);
		c->state = 1;
	} else {
		/* Process Humidity Measurement */
		c->humidity = -6.0 + 125.0 * raw / 65536;
		if (c->humidity < 0.0)
			c->humidity = 0.0;
		if (c->humidity > 100.0)
			c->humidity = 100.0;
		DEBUG_PRINT("rH = %f\n", c->humidity);
		c->state = 0;
	}

	*temp = c->temp;
	*humidity = c->humidity;
	*pressure = -1.0;

	return 0;
}


