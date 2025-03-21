/* i2c_am2320.c
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


/* AM2320 Registers */
#define HUM_MSB          0x00
#define HUM_LSB          0x01
#define TEMP_MSB         0x02
#define TEMP_LSB         0x03
#define MODEL_MSB        0x08
#define MODEL_LSB        0x09
#define VERSION          0x0a
#define DEVICE_ID        0x0b
#define STATUS           0x0f
#define USER_MSB         0x10
#define USER_LSB         0x11
#define USER2_MSB        0x12
#define USER2_LSB        0x13

/* AM2320 Function Codes */
#define READ_REG         0x03
#define WRITE_REG        0x10



static uint16_t crc16(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xffff;

	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc >>= 1;
				crc ^= 0xa001;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}


/* Read one or more registers from the sensor.
   Note, buf must be count + 4 bytes long. */

static int am2320_read_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t count, uint8_t *buf, bool wakeup)
{
	int res;
	uint8_t cmd[3];
	uint16_t crc;


	if (wakeup) {
		/* Try to wakeup the sensor. */
		cmd[0] = 0;
		res = i2c_write_raw(i2c, addr, cmd, 1, false);
		sleep_us(800);
	}

	/* Send Read Register(s) command. */
	cmd[0] = READ_REG;
	cmd[1] = reg;
	cmd[2] = count;
	DEBUG_PRINT("Send command: %02x %02x %02x\n", cmd[0], cmd[1], cmd[2]);
	if ((res = i2c_write_raw(i2c, addr, cmd, sizeof(cmd), false))) {
		DEBUG_PRINT("Failed to send command: %d\n", res);
		return -1;
	}
	sleep_us(1500);

	/* Read response. */
	if ((res = i2c_read_raw(i2c, addr, buf, count + 4, false))) {
		DEBUG_PRINT("Failed to read response: %d\n", res);
		return -2;
	}
#if I2C_DEBUG > 0
	DEBUG_PRINT("Response: [%02x %02x]", buf[0], buf[1]);
	for (int i = 0; i < count; i++) {
		printf(" %02x", buf[i + 2]);
	}
	printf(" [%02x %02x]\n", buf[count + 2], buf[count + 3]);
#endif
	crc = crc16(buf, count + 2);
	DEBUG_PRINT("CRC=%04x\n", crc);
	if (crc != ((buf[count + 3] << 8) | buf[count + 2])) {
		DEBUG_PRINT("CRC error\n");
		return -3;
	}
	if (buf[0] != cmd[0] || buf[1] != cmd[2]) {
		DEBUG_PRINT("Invalid response: %02x %02x\n", buf[0], buf[1]);
		return -4;
	}

	return 0;
}


void* am2320_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	i2c_sensor_context_t *ctx = calloc(1, sizeof(i2c_sensor_context_t));
	uint8_t buf[4 + 4];
	int res;

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Try reading a humidity and temperature measurement. */
	if ((res = am2320_read_registers(i2c, addr, HUM_MSB, 4, buf, true))) {
		*result = res;
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int am2320_start_measurement(void *ctx)
{
	/* Nothing to do. */

	return 2000;  /* measurement should be available after 2s */
}


int am2320_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = (i2c_sensor_context_t*)ctx;
	uint8_t buf[4 + 4];
	int16_t t_raw;
	uint16_t h_raw;

	if (am2320_read_registers(c->i2c, c->addr, HUM_MSB, 4, buf, true))
		return -1;

	h_raw = (buf[2] << 8) | buf[3];
	t_raw = (buf[4] << 8) | buf[5];

	DEBUG_PRINT("h_raw = %u (%04x)\n", h_raw, h_raw);
	DEBUG_PRINT("t_raw = %d (%04x)\n", t_raw, t_raw);

	*temp = t_raw / 10.0;
	*humidity = h_raw / 10.0;
	*pressure = -1.0;

	DEBUG_PRINT("temp = %0.2f C, humidity = %0.2f %%\n", *temp, *humidity);

	return 0;
}


