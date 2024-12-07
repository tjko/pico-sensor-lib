/* i2c.c
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
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "pico_sensor_lib.h"
#include "pico_sensor_lib/i2c.h"


#define I2C_TIMEOUT_SCALE_FACTOR (10000 / i2c_current_baudrate)

// timeouts in us (at 1000kHz)
#define I2C_READ_BASE_TIMEOUT 10000
#define I2C_WRITE_BASE_TIMEOUT 10000

#define I2C_READ_TIMEOUT(x) ((I2C_READ_BASE_TIMEOUT + (x * 250)) * I2C_TIMEOUT_SCALE_FACTOR / 10)
#define I2C_WRITE_TIMEOUT(x) ((I2C_WRITE_BASE_TIMEOUT + (x * 250)) * I2C_TIMEOUT_SCALE_FACTOR / 10)



/* i2c_adt7410.c */
void* adt7410_init(i2c_inst_t *i2c, uint8_t addr);
int adt7410_start_measurement(void *ctx);
int adt7410_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_aht.c */
void* aht1x_init(i2c_inst_t *i2c, uint8_t addr);
void* aht2x_init(i2c_inst_t *i2c, uint8_t addr);
int aht_start_measurement(void *ctx);
int aht_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_as621x.c */
void* as621x_init(i2c_inst_t *i2c, uint8_t addr);
int as621x_start_measurement(void *ctx);
int as621x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_bmp180.c */
void* bmp180_init(i2c_inst_t *i2c, uint8_t addr);
int bmp180_start_measurement(void *ctx);
int bmp180_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_bmp280.c */
void* bmp280_init(i2c_inst_t *i2c, uint8_t addr);
int bmp280_start_measurement(void *ctx);
int bmp280_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_dps310.c */
void* dps310_init(i2c_inst_t *i2c, uint8_t addr);
int dps310_start_measurement(void *ctx);
int dps310_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_lps22.c */
void* lps22_init(i2c_inst_t *i2c, uint8_t addr);
int lps22_start_measurement(void *ctx);
int lps22_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_lps25.c */
void* lps25_init(i2c_inst_t *i2c, uint8_t addr);
int lps25_start_measurement(void *ctx);
int lps25_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_mcp9808.c */
void* mcp9808_init(i2c_inst_t *i2c, uint8_t addr);
int mcp9808_start_measurement(void *ctx);
int mcp9808_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_ms8607.c */
void* ms8607_init(i2c_inst_t *i2c, uint8_t addr);
int ms8607_start_measurement(void *ctx);
int ms8607_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_pct2075.c */
void* pct2075_init(i2c_inst_t *i2c, uint8_t addr);
int pct2075_start_measurement(void *ctx);
int pct2075_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_shtc3.c */
void* shtc3_init(i2c_inst_t *i2c, uint8_t addr);
int shtc3_start_measurement(void *ctx);
int shtc3_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_sht3x.c */
void* sht3x_init(i2c_inst_t *i2c, uint8_t addr);
int sht3x_start_measurement(void *ctx);
int sht3x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_sht4x.c */
void* sht4x_init(i2c_inst_t *i2c, uint8_t addr);
int sht4x_start_measurement(void *ctx);
int sht4x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_stts22h.c */
void* stts22h_init(i2c_inst_t *i2c, uint8_t addr);
int stts22h_start_measurement(void *ctx);
int stts22h_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_tmp102.c */
void* tmp102_init(i2c_inst_t *i2c, uint8_t addr);
int tmp102_start_measurement(void *ctx);
int tmp102_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_tmp117.c */
void* tmp117_init(i2c_inst_t *i2c, uint8_t addr);
int tmp117_start_measurement(void *ctx);
int tmp117_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

static const i2c_sensor_entry_t i2c_sensor_types[] = {
	{ "NONE", NULL, NULL, NULL, NULL, false, 0 }, /* this needs to be first so that valid sensors have index > 0 */
	{ "ADT7410", adt7410_init, adt7410_start_measurement, adt7410_get_measurement, NULL, false, 1 },
	{ "AHT1x", aht1x_init, aht_start_measurement, aht_get_measurement, NULL, false, 1 },
	{ "AHT2x", aht2x_init, aht_start_measurement, aht_get_measurement, NULL, false, 1 },
	{ "AS621x", as621x_init, as621x_start_measurement, as621x_get_measurement, NULL, false, 1 },
	{ "BMP180", bmp180_init, bmp180_start_measurement, bmp180_get_measurement, NULL, false, 2 },
	{ "BMP280", bmp280_init, bmp280_start_measurement, bmp280_get_measurement, NULL, false, 1 },
	{ "DPS310", dps310_init, dps310_start_measurement, dps310_get_measurement, NULL, false, 1 },
	{ "LPS22", lps22_init, lps22_start_measurement, lps22_get_measurement, NULL, false, 1 },
	{ "LPS25", lps25_init, lps25_start_measurement, lps25_get_measurement, NULL, false, 1 },
	{ "MCP9808", mcp9808_init, mcp9808_start_measurement, mcp9808_get_measurement, NULL, false, 1 },
	{ "MS8607", ms8607_init, ms8607_start_measurement, ms8607_get_measurement, NULL, true, 3 },
	{ "PCT2075", pct2075_init, pct2075_start_measurement, pct2075_get_measurement, NULL, false, 1 },
	{ "SHTC3", shtc3_init, shtc3_start_measurement, shtc3_get_measurement, NULL, false, 1 },
	{ "SHT3x", sht3x_init, sht3x_start_measurement, sht3x_get_measurement, NULL, true, 1 },
	{ "SHT4x", sht4x_init, sht4x_start_measurement, sht4x_get_measurement, NULL, true, 1 },
	{ "STTS22H", stts22h_init, stts22h_start_measurement, stts22h_get_measurement, NULL, false, 1 },
	{ "TMP102", tmp102_init, tmp102_start_measurement, tmp102_get_measurement, NULL, false, 1 },
	{ "TMP117", tmp117_init, tmp117_start_measurement, tmp117_get_measurement, NULL, false, 1 },
	{ NULL, NULL, NULL, NULL, NULL, false }
};

#define SENSOR_TYPES_COUNT ((sizeof(i2c_sensor_types) / sizeof(i2c_sensor_entry_t)) - 1)


static uint i2c_current_baudrate = 1000;  // kHz

void i2c_sensor_baudrate(uint baudrate)
{
	if (baudrate > 0)
		i2c_current_baudrate = baudrate;
}

int i2c_init_sensor(uint8_t sensor_type, i2c_inst_t *i2c_bus, uint8_t addr, void **ctx)
{
	uint8_t buf[2];

	if (sensor_type < 1 || sensor_type >= SENSOR_TYPES_COUNT ||
		!ctx || i2c_reserved_address(addr))
		return -1;

	/* Check for a device on given address... */
	if (!i2c_sensor_types[sensor_type].no_scan) {
		if (i2c_read_timeout_us(i2c_bus, addr, buf, 1, false,
						I2C_READ_TIMEOUT(1)) < 0)
			return -2;
	}

	/* Initialize sensor */
	if (!(*ctx = i2c_sensor_types[sensor_type].init(i2c_bus, addr)))
		return -3;

	((i2c_sensor_context_t*)(*ctx))->sensor_type = sensor_type;

	return 0;
}


int i2c_shutdown_sensor(void *ctx)
{
	i2c_sensor_context_t *c = ctx;

	if (!ctx)
		return -1;

	if (i2c_sensor_types[c->sensor_type].shutdown) {
		i2c_sensor_types[c->sensor_type].shutdown(ctx);
	}

	free(ctx);

	return 0;
}


int i2c_start_measurement(void *ctx)
{
	i2c_sensor_context_t *c = ctx;

	if (!ctx)
		return -1;

	if (c->sensor_type < 1 || c->sensor_type >= SENSOR_TYPES_COUNT)
		return -2;

	return i2c_sensor_types[c->sensor_type].start_measurement(ctx);
}

int i2c_read_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = ctx;

	if (!ctx)
		return -1;

	if (c->sensor_type < 1 || c->sensor_type >= SENSOR_TYPES_COUNT)
		return -2;

	return i2c_sensor_types[c->sensor_type].get_measurement(ctx, temp, pressure, humidity);
}


int i2c_run_measurement(void *ctx, float *temp, float *pressure, float *humidity)
{
	i2c_sensor_context_t *c = ctx;
	const i2c_sensor_entry_t *sensor = NULL;
	int res = -1;

	if (!ctx)
		return -1;
	if (c->sensor_type < 1 || c->sensor_type >= SENSOR_TYPES_COUNT)
		return -2;

	sensor = &i2c_sensor_types[c->sensor_type];

	for (int i = 0; i < sensor->cycle_len; i++) {
		res = sensor->start_measurement(ctx);
		if (res < 0)
			return res;
		sleep_ms(res);
		res = sensor->get_measurement(ctx, temp, pressure, humidity);
		if (res < 0)
			return res;
	}

	return res;
}


inline int32_t twos_complement(uint32_t value, uint8_t bits)
{
	uint32_t mask = ((uint32_t)0xffffffff >> (32 - bits));

	if (value & ((uint32_t)1 << (bits - 1))) {
		/* negative value, set high bits */
		value |= ~mask;
	} else {
		/* positive value, clear high bits */
		value &= mask;
	}

	return (int32_t)value;
}


inline bool i2c_reserved_address(uint8_t addr)
{
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


int i2c_read_register_block(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len,
			uint32_t read_delay_us)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p,%u\n", i2c, addr, reg, buf, len);
	res = i2c_write_timeout_us(i2c, addr, &reg, 1, true,
				I2C_WRITE_TIMEOUT(1));
	if (res < 1) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	if (read_delay_us > 0)
		sleep_us(read_delay_us);

	res = i2c_read_timeout_us(i2c, addr, buf, len, false,
				I2C_READ_TIMEOUT(len));
	if (res < len) {
		DEBUG_PRINT("read failed (%d)\n", res);
		return -2;
	} else {
#if I2C_DEBUG > 0
		DEBUG_PRINT("read ok: ");
		for(int i = 0; i < len; i++) {
			printf(" %02x", buf[i]);
		}
		printf("\n");
#endif
	}

	return 0;
}


int i2c_read_register_u24(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint32_t *val)
{
	uint8_t buf[3];
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p\n", i2c, addr, reg, val);
	res = i2c_read_register_block(i2c, addr, reg, buf, sizeof(buf), 0);
	if (res) {
		DEBUG_PRINT("failed to read register\n");
		return -1;
	}

	*val = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	DEBUG_PRINT("read ok: [%02x %02x %02x] %08lx (%lu)\n", buf[0], buf[1], buf[2], *val, *val);

	return 0;
}


int i2c_read_register_u16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint16_t *val)
{
	uint8_t buf[2];
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p\n", i2c, addr, reg, val);
	res = i2c_read_register_block(i2c, addr, reg, buf, sizeof(buf), 0);
	if (res) {
		DEBUG_PRINT("failed to read register\n");
		return -1;
	}

	*val = (buf[0] << 8) | buf[1];
	DEBUG_PRINT("read ok: [%02x %02x] %04x (%u)\n", buf[0], buf[1], *val, *val);

	return 0;
}


int i2c_read_register_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *val)
{
	uint8_t buf[1];
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p\n", i2c, addr, reg, val);
	res = i2c_read_register_block(i2c, addr, reg, buf, sizeof(buf), 0);
	if (res) {
		DEBUG_PRINT("failed to read register\n");
		return -1;
	}

	*val = buf[0];
	DEBUG_PRINT("read ok: %02x (%u)\n", *val, *val);

	return 0;
}


int i2c_write_register_block(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len)
{
	uint8_t tmp[128];
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p,%u\n", i2c, addr, reg, buf, len);
	tmp[0] = reg;
	if (len >= sizeof(tmp)) {
		DEBUG_PRINT("too large buffer: %d\n", len);
		return -1;
	}
	memcpy(&tmp[1], buf, len);
	res = i2c_write_timeout_us(i2c, addr, buf, len + 1, false,
				I2C_WRITE_TIMEOUT(len + 1));
	if (res < len + 1) {
		DEBUG_PRINT("write register values failed (%d)\n", res);
		return -2;
	} else {
#if I2C_DEBUG > 0
		DEBUG_PRINT("write ok: %d [", res);
		for(int i = 0; i <= len; i++) {
			printf(" %02x", tmp[i]);
		}
		printf(" ]\n");
#endif
	}

	return 0;
}


int i2c_write_register_u16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint16_t val)
{
	uint8_t buf[3];
	int res;

	buf[0] = reg;
	buf[1] = val >> 8;
	buf[2] = val & 0xff;

	DEBUG_PRINT("args=%p,%02x,%02x,%04x (%u)\n", i2c, addr, reg, val, val);

	res = i2c_write_timeout_us(i2c, addr, buf, 3, false,
				I2C_WRITE_TIMEOUT(3));
	if (res < 3) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	return 0;
}


int i2c_write_register_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t buf[2];
	int res;

	buf[0] = reg;
	buf[1] = val;

	DEBUG_PRINT("args=%p,%02x,%02x,%02x (%u)\n", i2c, addr, reg, val, val);

	res = i2c_write_timeout_us(i2c, addr, buf, 2, false,
				I2C_WRITE_TIMEOUT(2));
	if (res < 2) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	return 0;
}


int i2c_read_raw(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf, size_t len, bool nostop)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%p,%u\n", i2c, addr, buf, len);

	res = i2c_read_timeout_us(i2c, addr, buf, len, nostop,
				I2C_READ_TIMEOUT(len));
	if (res < len) {
		DEBUG_PRINT("read failed (%d)\n", res);
		return -2;
	}

	DEBUG_PRINT("read ok: %u\n", len);

	return 0;
}


int i2c_write_raw_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t cmd, bool nostop)
{
	uint8_t buf[2];
	int res;

	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xff;

	DEBUG_PRINT("args=%p,%02x,%04x\n", i2c, addr, cmd);

	res = i2c_write_timeout_us(i2c, addr, buf, 2, nostop,
				I2C_WRITE_TIMEOUT(2));
	if (res < 2) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	return 0;
}


int i2c_write_raw_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t cmd, bool nostop)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%04x\n", i2c, addr, cmd);

	res = i2c_write_timeout_us(i2c, addr, &cmd, 1, nostop,
				I2C_WRITE_TIMEOUT(1));
	if (res < 1) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	return 0;
}


uint get_i2c_sensor_type(const char *name)
{
	int type = -1;
	int len;

	if (!name)
		return 0;

	len = strlen(name);

	for (int i = 0; i2c_sensor_types[i].name; i++) {
		if (!strncasecmp(i2c_sensor_types[i].name, name, len)) {
			type = i;
			break;
		}
	}

	return (type > 0 ? type : 0);
}


const char *i2c_sensor_type_str(uint sensor_type)
{
	if (sensor_type >=0 && sensor_type < SENSOR_TYPES_COUNT) {
		return i2c_sensor_types[sensor_type].name;
	}

	return "NONE";
}




