/* i2c_hdc302x.c
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

/* HDC302X Registers */
#define START_MEASUREMENT        0x2400 // Trigger-On Demand Mode, Single T and RH measurement
#define HEATER_ON                0x306d
#define HEATER_OFF               0x3066
#define HEATER_CONFIG            0x306e
#define READ_STATUS              0xf32d
#define CLEAR_STATUS             0x3041
#define SOFT_RESET               0x30a2
#define READ_NIST_ID_5_4         0x3683
#define READ_NIST_ID_3_2         0x3684
#define READ_NIST_ID_1_0         0x3685
#define READ_MANUF_ID            0x3781

#define MANUFACTURER_ID          0x3000



static int hdc302x_read_register_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t reg, uint16_t *val)
{
	uint8_t buf[3], crc;
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p\n", i2c, addr, reg, val);

	if ((res = i2c_write_raw_u16(i2c, addr, reg, true))) {
		DEBUG_PRINT("failed to write command\n");
		return -1;
	}
	if ((res = i2c_read_raw(i2c, addr, buf, sizeof(buf), false))) {
		DEBUG_PRINT("failed to read register\n");
		return -2;
	}

	*val = (buf[0] << 8) | buf[1];
	crc = crc8_generic(buf, 2, 0x31, 0xff, 0x00, false, false);

	DEBUG_PRINT("read: [%02x %02x %02x] %04x (%u) [crc=%2x]\n",
		buf[0], buf[1], buf[2], *val, *val, crc);
	if (crc != buf[2]) {
		DEBUG_PRINT("CRC error\n");
		return -3;
	}

	return 0;
}



void* hdc302x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	uint16_t val = 0;
	uint64_t serial = 0;
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read and verify device ID */
	if (hdc302x_read_register_u16(i2c, addr, READ_MANUF_ID, &val)) {
		*result = -1;
		goto panic;
	}
	if (val != MANUFACTURER_ID) {
		*result = -2;
		goto panic;
	}

	/* Reset Sensor */
	if (i2c_write_raw_u16(i2c, addr, SOFT_RESET, false)) {
		*result = -3;
		goto panic;
	}

	/* Wait for sensor to soft reset (reset should take max 3ms per datasheet)  */
	sleep_ms(3);


	/* Read NIST ID */
	for (int i = 0; i < 3; i++) {
		if (hdc302x_read_register_u16(i2c, addr, READ_NIST_ID_5_4 + i, &val)) {
			*result = -4;
			goto panic;
		}
		serial = (serial << 16) | val;
	}
	DEBUG_PRINT("Serial (NIST ID): %12llx\n", serial);

#if 0
	/* Clear Status Register */
	if (i2c_write_raw_u16(i2c, addr, CLEAR_STATUS, false)) {
		*result = -4;
		goto panic;
	}
#endif

	/* Read Status Register */
	if (hdc302x_read_register_u16(i2c, addr, READ_STATUS, &val)) {
		*result = -5;
		goto panic;
	}
	DEBUG_PRINT("Status Register: %04x\n", val);

	/* Disable Heater (if needed) */
	if (val & 0x2000) {
		DEBUG_PRINT("Disable heater\n");
		if (i2c_write_raw_u16(i2c, addr, HEATER_OFF, false)) {
			*result = -6;
			goto panic;
		}
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int hdc302x_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;

	/* Start Measurement */
	if ((res = i2c_write_raw_u16(c->i2c, c->addr, START_MEASUREMENT, false)))
		return -1;

	return 15;  /* measurement should be available after 14.1ms */
}


int hdc302x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	int res;
	uint16_t t, rh;
	uint8_t buf[6], crc;

	/* Read Measurement */
	if ((res = i2c_read_raw(c->i2c, c->addr, buf, sizeof(buf), false)))
		return -1;

	t = buf[0] << 8 | buf[1];
	DEBUG_PRINT("Raw Temp=%u\n", t);
	crc = crc8_generic(&buf[0], 2, 0x31, 0xff, 0x00, false, false);
	if (crc != buf[2]) {
		DEBUG_PRINT("CRC error (Temp)\n");
		return -2;
	}

	rh = buf[3] << 8 | buf[4];
	DEBUG_PRINT("Raw RH=%u\n", rh);
	crc = crc8_generic(&buf[3], 2, 0x31, 0xff, 0x00, false, false);
	if (crc != buf[5]) {
		DEBUG_PRINT("CRC error (RH)\n");
		return -3;
	}

	*temp = -45.0 + (175.0 * ((float)t / 65535));
	*humidity = 100.0 * ((float)rh / 65535);
	*pressure = -1.0;

	return 0;
}


