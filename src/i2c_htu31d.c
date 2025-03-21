/* i2c_htu31d.c
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


/* HTU31D Commands */

#define CONVERSION       0x5e // OSR_RH=3, OSR_Temp=3
#define READ_T_RH        0x00
#define READ_RH          0x10
#define SOFT_RESET       0x1e
#define HEATER_ON        0x04
#define HEATER_OFF       0x01
#define READ_SERIAL      0x0a
#define READ_DIAG        0x08




static int read_diagnostics(i2c_inst_t *i2c, uint8_t addr, uint8_t *diag)
{
	int res;
	uint8_t buf[2], crc;

	if ((res = i2c_read_register_block(i2c, addr, READ_DIAG, buf, sizeof(buf),0)))
		return -1;

	crc = crc8_generic(buf, 1, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("Diag register: %02x %02x (crc=%02x)\n",
		buf[0], buf[1], crc);
	if (buf[1] != crc)
		return -2;
	*diag = buf[0];

	return 0;
}


void* htu31d_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	uint8_t buf[4], crc, diag;
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read Serial Number */
	if (i2c_read_register_block(i2c, addr, READ_SERIAL, buf, sizeof(buf), 0)) {
		*result = -1;
		goto panic;
	}
	crc = crc8_generic(buf, 3, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("Serial: %02x%02x%02x %02x (crc=%02x)\n",
		buf[0], buf[1], buf[2], buf[3], crc);
	if (buf[3] != crc) {
		*result = -2;
		goto panic;
	}

	/* Soft Reset */
	if (i2c_write_raw_u8(i2c, addr, SOFT_RESET, false)) {
		*result = -3;
		goto panic;
	}
	sleep_ms(15);

	/* Read Diagnostics Register */
	if (read_diagnostics(i2c, addr, &diag)) {
		*result = -4;
		goto panic;
	}
	if (diag & 0x80) {
		/* NVM failure */
		DEBUG_PRINT("NVM error\n");
		*result = -5;
		goto panic;
	}
	if ((diag & 0x01)) {
		/* Turn off heater */
		DEBUG_PRINT("Turn OFF heater\n");
		if (i2c_write_raw_u8(i2c, addr, HEATER_OFF, false)) {
			*result = -6;
			goto panic;
		}
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int htu31d_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;

	if ((res = i2c_write_raw_u8(c->i2c, c->addr, CONVERSION, false))) {
		DEBUG_PRINT("start_measurement failed: %d\n", res);
		return -1;
	}

	return 13;  /* measurement should be available after 12.1ms */
}


int htu31d_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint8_t buf[6], crc[2], diag;
	uint16_t t_raw, rh_raw;

	/* Read Diagnostics */
	if (read_diagnostics(c->i2c, c->addr, &diag))
		return -1;
	if (diag & 0x81)
		return -2;

	/* Get Measurement */
	if ((res = i2c_read_register_block(c->i2c, c->addr, READ_T_RH, buf, sizeof(buf), 0)))
		return -3;
	t_raw = buf[0] << 8 | buf[1];
	rh_raw = buf[3] << 8 | buf[4];
	crc[0] = crc8_generic(&buf[0], 2, 0x31, 0x00, 0x00, false, false);
	crc[1] = crc8_generic(&buf[3], 2, 0x31, 0x00, 0x00, false, false);
	DEBUG_PRINT("Raw  T: %02x %02x %02x [crc=%02x] t_raw=%u\n",
		buf[0], buf[1], buf[2], crc[0], t_raw);
	DEBUG_PRINT("Raw rH: %02x %02x %02x [crc=%02x] rh_raw=%u\n",
		buf[3], buf[4], buf[5], crc[1], rh_raw);
	if (buf[2] != crc[0] || buf[5] != crc[1]) {
		DEBUG_PRINT("CRC error\n");
		return -4;
	}

	*temp = -40.0 + 165.0 * t_raw / 65535;
	*humidity = 100.0 * rh_raw / 65535;
	if (*humidity < 0.0)
		*humidity = 0.0;
	if (*humidity > 100.0)
		*humidity = 100.0;
	*pressure = -1.0;

	DEBUG_PRINT("temp = %0.2f C, humidity = %0.2f %%\n", *temp, *humidity);

	return 0;
}


