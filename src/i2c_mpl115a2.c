/* i2c_mpl115a2.c
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

/* MPL115A2 Registers */
#define PADC_MSB       0x00
#define PADC_LSB       0x01
#define TADC_MSB       0x02
#define TADC_LSB       0x03
#define READ_COEF      0x04
#define CONVERT        0x12

#define T_REF          25.0   // Reference temperature (C)
#define T_REF_ADC      515    // ADC reading at T_REF


typedef struct mpl115a2_sensor_context_t {
	struct { I2C_SENSOR_CONTEXT_MEMBERS };
	int16_t a0;
	int16_t b1;
	int16_t b2;
	int16_t c12;
} mpl115a2_sensor_context_t;



void* mpl115a2_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result)
{
	mpl115a2_sensor_context_t *ctx = calloc(1, sizeof(mpl115a2_sensor_context_t));
	uint8_t buf[8];

	if (!ctx)
		return NULL;
	ctx->i2c = i2c;
	ctx->addr = addr;

	/* Read Calibration Data */
	if (i2c_read_register_block(i2c, addr, READ_COEF, buf , sizeof(buf), false)) {
		*result = -1;
		goto panic;
	}

	ctx->a0 = (buf[0] << 8) | buf[1];
	ctx->b1 = (buf[2] << 8) | buf[3];
	ctx->b2 = (buf[4] << 8) | buf[5];
	ctx->c12 = (buf[6] << 8) | buf[7];

	DEBUG_PRINT("A0 = %04x (%f)\n", ctx->a0, (float)ctx->a0 / (1 << 3));
	DEBUG_PRINT("B1 = %04x (%f)\n", ctx->b1, (float)ctx->b1 / (1 << 13));
	DEBUG_PRINT("B2 = %04x (%f)\n", ctx->b2, (float)ctx->b2 / (1 << 14));
	DEBUG_PRINT("C12 = %04x (%f)\n", ctx->c12, (float)ctx->c12 / (1 << 22));

	return ctx;

panic:
	free(ctx);
	return NULL;
}


int mpl115a2_start_measurement(void *ctx)
{
	mpl115a2_sensor_context_t *c = (mpl115a2_sensor_context_t*)ctx;
	int res;

	if ((res = i2c_write_register_u8(c->i2c, c->addr, CONVERT, 0x00)))
		return -1;

	return 3;  /* measurement should be available after 3ms */
}


int mpl115a2_get_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	mpl115a2_sensor_context_t *c = (mpl115a2_sensor_context_t*)ctx;
	int res;
	uint8_t buf[4];
	uint16_t t_raw, p_raw;
	int32_t c12x2, a1, a1x1, y1, a2x2, p_comp;


	/* Get Measurement (read both pressure and temp measurements at once) */
	if ((res = i2c_read_register_block(c->i2c, c->addr, PADC_MSB, buf, sizeof(buf), 3)))
		return -1;

	p_raw = (buf[0] << 2) | (buf[1] >> 6); // 10bit unsigned
	t_raw = (buf[2] << 2) | (buf[3] >> 6); // 10bit unsigned
	DEBUG_PRINT("t_raw = %d, p_raw = %d\n", t_raw, p_raw);

	/* Calculate compensated pressure per Application Note (AN3785) */
	c12x2 = (((int32_t)c->c12) * t_raw) >> 11;
	a1 = c->b1 + c12x2;
	a1x1 = a1 * p_raw;
	y1 = (((int32_t)c->a0) << 10) + a1x1;
	a2x2 = (((int32_t)c->b2) * t_raw) >> 1;
	p_comp = (y1 + a2x2) >> 9; // Result has 4bit fractional part
	DEBUG_PRINT("PComp = %ld (%f)\n", p_comp, (float)p_comp / (1 << 4));

	*temp = T_REF + (t_raw - T_REF_ADC) / -5.35;
	*pressure = (50.0 + ((float)p_comp / (1 << 4)) * ((115.0 - 50.0) / 1023.0)) * 10;
	*humidity = -1.0;

	DEBUG_PRINT("temp = %0.1f C, pressure = %0.1f hPa\n", *temp, *pressure);

	return 0;
}

