/* i2c_ms5611.c
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

/* MS5611 Registers */

#define RESET              0x1e
#define CONVERT_D1         0x48 // OSR=4096
#define CONVERT_D2         0x58 // OSR=4096
#define READ_ADC           0x00
#define PROM_READ          0xa0


typedef struct ms5611_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	uint8_t addr2;
	uint8_t state;
	uint16_t prom_pt[8];
	uint32_t d1;
	uint32_t d2;
	float temp;
	float pressure;
} ms5611_context_t;



static uint8_t crc4_pt(const uint16_t prom[])
{
	uint16_t n_rem = 0;

	for (uint8_t cnt = 0; cnt < 16; cnt++) {
		uint8_t prom_idx = cnt >> 1;
		uint16_t n_prom = prom[prom_idx];

		if (prom_idx == 7)
			n_prom &= 0xff00;

		if (cnt % 2 == 1)
			n_rem ^= n_prom & 0x00ff;
		else
			n_rem ^= n_prom >> 8;

		for (int n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000)
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem = (n_rem >> 12) & 0x000f;
	return n_rem ^ 0x00;
}


void* ms5611_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	ms5611_context_t *ctx = calloc(1, sizeof(ms5611_context_t));
	uint8_t crc;


	if (!ctx)
		return NULL;

	ctx->i2c = i2c;
	ctx->addr = addr;
	ctx->state = 0;
	ctx->temp = 0.0;
	ctx->pressure = -1.0;


	/* Reset Sensor */
	if (i2c_write_raw_u8(ctx->i2c, ctx->addr, RESET, false)) {
		*result = -1;
		goto panic;
	}
	sleep_ms(10);


	/* Read PROM */
	for (uint8_t i = 0; i < 8; i++) {
		if (i2c_read_register_u16(ctx->i2c, ctx->addr, PROM_READ + (i << 1),
						&ctx->prom_pt[i])) {
			*result = -2;
			goto panic;
		}
		sleep_us(100);
	}
	DEBUG_PRINT("C1=%u\n", ctx->prom_pt[1]);
	DEBUG_PRINT("C2=%u\n", ctx->prom_pt[2]);
	DEBUG_PRINT("C3=%u\n", ctx->prom_pt[3]);
	DEBUG_PRINT("C4=%u\n", ctx->prom_pt[4]);
	DEBUG_PRINT("C5=%u\n", ctx->prom_pt[5]);
	DEBUG_PRINT("C6=%u\n", ctx->prom_pt[6]);
	crc = crc4_pt(ctx->prom_pt);
	DEBUG_PRINT("PROM CRC-4: %02x (%02x)\n", crc, ctx->prom_pt[7] & 0x000f);
	if (crc != (ctx->prom_pt[7] & 0x000f)) {
		*result = -3;
		goto panic;
	}

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int ms5611_start_measurement(void *ctx)
{
	ms5611_context_t *c = (ms5611_context_t*)ctx;
	int res;
	uint8_t cmd = (c->state == 0 ? CONVERT_D1 : CONVERT_D2);

	res = i2c_write_raw_u8(c->i2c, c->addr, cmd, false);
	if (res)
		return -1;

	return 10;
}


int ms5611_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	ms5611_context_t *c = (ms5611_context_t*)ctx;
	int res;
	uint32_t val;
	int32_t dt, t, p, t2;
	int64_t off, sens, off2, sens2, tmp;


	res = i2c_read_register_u24(c->i2c, c->addr, READ_ADC, &val);
	if (res)
		return -1;

	DEBUG_PRINT("state=%d, val=%lu\n", c->state, val);

	if (c->state == 0) {
		c->d1 = val;
		DEBUG_PRINT("D1 = %lu\n", c->d1);
	} else {
		c->d2 = val;
		DEBUG_PRINT("D2 = %lu\n", c->d2);

		dt = (int32_t)c->d2 - ((int32_t)c->prom_pt[5] << 8);
		DEBUG_PRINT("dT = %ld\n", dt);
		t = 2000 + (((int64_t)dt * (int64_t)(c->prom_pt[6])) >> 23);

		/* Second order compensation */
		if (t < 2000) {
			t2 = ((int64_t)dt * (int64_t)dt) >> 31;
			tmp = ((int64_t)t - 2000) * ((int64_t)t - 2000);
			off2 = (5 * tmp) >> 1;
			sens2 = (5 * tmp) >> 2;
			if (t < -1500) {
				tmp = ((int64_t)t + 1500) * ((int64_t)t + 1500);
				off2 += 7 * tmp;
				sens2 += (11 * tmp) >> 1;
			}
		} else {
			t2 = 0;
			off2 = 0;
			sens2 = 0;
		}

		DEBUG_PRINT("TEMP = %ld (TEMP2 = %ld)\n", t, t2);
		off = ((int64_t)c->prom_pt[2] << 16) + (((int64_t)c->prom_pt[4] * dt) >> 7);
		off -= off2;
		DEBUG_PRINT("OFF = %lld (OFF2 = %lld)\n", off, off2);
		sens = ((int64_t)c->prom_pt[1] << 15) + (((int64_t)c->prom_pt[3] * dt) >>  8);
		sens -= sens2;
		DEBUG_PRINT("SENS = %lld (SENS2 = %lld)\n", sens, sens2);
		p = ((((int64_t)c->d1 * sens) >> 21) - off) >> 15;
		DEBUG_PRINT("P = %ld\n", p);

		c->temp = (t - t2) / 100.0;
		c->pressure = p / 100.0;
	}

	c->state++;
	if (c->state > 1)
		c->state = 0;

	*temp = c->temp;
	*pressure = c->pressure;
	*humidity = -1.0;

	DEBUG_PRINT("temp = %0.1f C, pressure = %0.1f hPa\n", *temp, *pressure);

	return 0;
}


