/* i2c_tmp11x.c
   Copyright (C) 2024-2026 Timo Kokkonen <tjko@iki.fi>

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

/* TMP117/TMP119 Registers */
#define REG_TEMP_RESULT  0x00
#define REG_CONFIG       0x01
#define REG_THIGH_LIMIT  0x02
#define REG_TLOW_LIMIT   0x03
#define REG_EEPROM_UL    0x04
#define REG_EEPROM1      0x05
#define REG_EEPROM2      0x06
#define REG_TEMP_OFFSET  0x07
#define REG_EEPROM3      0x08
#define REG_DEVICE_ID    0x0f

#define TMP116_DEVICE_ID 0x0116
#define TMP117_DEVICE_ID 0x0117 // TMP119 uses same ID



#define DEFAULT_CONFIG 0x0220 // Continuous Conversion, 1s cycle, 8 averaged conversions


void* tmp11x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	uint16_t cfg = DEFAULT_CONFIG;
	uint16_t dev_id = 0;
	uint16_t val = 0;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read and verify device ID */
	if (i2c_read_register_u16(i2c, addr, REG_DEVICE_ID, &val)) {
		*result = -1;
		goto panic;
	}
	dev_id = val & 0x0fff;
	if (!(dev_id == TMP116_DEVICE_ID || dev_id == TMP117_DEVICE_ID)) {
		*result = -2;
		goto panic;
	}

	/* Reset Sensor */
	if (dev_id == TMP117_DEVICE_ID) {
		cfg |= 0x0002; /* trigger soft-reset */
	}
	if (i2c_write_register_u16(i2c, addr, REG_CONFIG, cfg)) {
		*result = -3;
		goto panic;
	}

	/* Wait for sensor to soft reset (reset should take 2ms per datasheet)  */
	sleep_us(2500);

	/* Read configuration register */
	if (i2c_read_register_u16(i2c, addr, REG_CONFIG, &val)) {
		*result = -4;
		goto panic;
	}

	/* Check that confuration is now as expected... */
	if ((val & 0x0FFC) != DEFAULT_CONFIG) {
		*result = -5;
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int tmp11x_start_measurement(void *ctx)
{
	/* Nothing to do, sensor is in continuous measurement mode... */

	return 1000;  /* measurement should be available after 1s */
}


int tmp11x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint16_t val;


	/* Read configuration register */
	res = i2c_read_register_u16(c->i2c, c->addr, REG_CONFIG, &val);
	if (res)
		return -1;

	/* Check Data_Ready bit */
	if ((val & 0x2000) == 0)
		return 1;

	/* Get Measurement */
	res = i2c_read_register_u16(c->i2c, c->addr, REG_TEMP_RESULT, &val);
	if (res)
		return -2;

	*temp = ((int16_t)val) / 128.0;
	*pressure = -1.0;
	*humidity = -1.0;

	return 0;
}


