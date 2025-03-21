/* i2c_tc74.c
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


/* TC74 Registers */
#define TEMP          0x00
#define CONFIG        0x01


void* tc74_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	uint8_t cfg = 0;


	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read config register */
	if (i2c_read_register_u8(i2c, addr, CONFIG, &cfg)) {
		*result = -1;
		goto panic;
	}

	/* Low 6 bits should always be zero */
	if ((cfg & 0x3f) != 0) {
		*result = -2;
		goto panic;
	}


	/* Set sensor to Normal mode */
	if (i2c_write_register_u8(i2c, addr, CONFIG, 0x00)) {
		*result = -3;
		goto panic;
	}


	return ctx;

panic:
	free(ctx);
	return NULL;
}


int tc74_start_measurement(void *ctx)
{
	/* Nothing to do, sensor is in continuous measurement mode... */

	return 125;  /* Measurement should be available after 125ms (8 measurements/sec) */
}


int tc74_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int8_t meas = 0;
	uint8_t cfg;


	/* Check Data Ready flag */
	if (i2c_read_register_u8(c->i2c, c->addr, CONFIG, &cfg))
		return -1;
	if (!(cfg & 0x40))
		return -2;

	/* Get Temperature Measurement */
	if (i2c_read_register_u8(c->i2c, c->addr, TEMP, (uint8_t*)&meas))
		return -3;

	*temp = (float)meas;
	*pressure = -1.0;
	*humidity = -1.0;

	return 0;
}
