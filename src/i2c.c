/* i2c.c
   Copyright (C) 2024-2025 Timo Kokkonen <tjko@iki.fi>

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
void* adt7410_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int adt7410_start_measurement(void *ctx);
int adt7410_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_aht.c */
void* aht1x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
void* aht2x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int aht_start_measurement(void *ctx);
int aht_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_am2320.c */
void* am2320_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int am2320_start_measurement(void *ctx);
int am2320_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_as621x.c */
void* as621x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int as621x_start_measurement(void *ctx);
int as621x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_bmp180.c */
void* bmp180_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int bmp180_start_measurement(void *ctx);
int bmp180_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_bmp280.c */
void* bmp280_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int bmp280_start_measurement(void *ctx);
int bmp280_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_dps310.c */
void* dps310_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int dps310_start_measurement(void *ctx);
int dps310_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_hdc302x.c */
void* hdc302x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int hdc302x_start_measurement(void *ctx);
int hdc302x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_hts221.c */
void* hts221_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int hts221_start_measurement(void *ctx);
int hts221_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_htu21d.c */
void* htu21d_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int htu21d_start_measurement(void *ctx);
int htu21d_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_htu31d.c */
void* htu31d_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int htu31d_start_measurement(void *ctx);
int htu31d_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_lps.c */
void* lps_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int lps_start_measurement(void *ctx);
int lps_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_mcp9808.c */
void* mcp9808_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int mcp9808_start_measurement(void *ctx);
int mcp9808_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_mpl115a2.c */
void* mpl115a2_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int mpl115a2_start_measurement(void *ctx);
int mpl115a2_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_mpl3115a2.c */
void* mpl3115a2_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int mpl3115a2_start_measurement(void *ctx);
int mpl3115a2_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_ms5611.c */
void* ms5611_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int ms5611_start_measurement(void *ctx);
int ms5611_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_ms8607.c */
void* ms8607_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int ms8607_start_measurement(void *ctx);
int ms8607_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_pct2075.c */
void* pct2075_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int pct2075_start_measurement(void *ctx);
int pct2075_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_shtc3.c */
void* shtc3_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int shtc3_start_measurement(void *ctx);
int shtc3_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_sht3x.c */
void* sht3x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int sht3x_start_measurement(void *ctx);
int sht3x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_sht4x.c */
void* sht4x_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int sht4x_start_measurement(void *ctx);
int sht4x_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_si7021.c */
void* si7021_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int si7021_start_measurement(void *ctx);
int si7021_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_stts22h.c */
void* stts22h_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int stts22h_start_measurement(void *ctx);
int stts22h_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_tc74.c */
void* tc74_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int tc74_start_measurement(void *ctx);
int tc74_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_tmp102.c */
void* tmp102_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int tmp102_start_measurement(void *ctx);
int tmp102_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);

/* i2c_tmp117.c */
void* tmp117_init(i2c_inst_t *i2c, uint8_t addr, int16_t *result);
int tmp117_start_measurement(void *ctx);
int tmp117_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);




static const char* as621x_aliases[] = {
	"AS6212",
	"AS6214",
	"AS6218",
	NULL
};

static const char* hdc302x_aliases[] = {
	"HDC3020",
	"HDC3021",
	"HDC3022",
	NULL
};

static const char* lps_aliases[] = {
	"LPS22",
	"LPS25",
	"LPS28",
	"LPS33",
	"LPS35",
	NULL
};

static const char* sht3x_aliases[] = {
	"SHT30",
	"SHT31",
	"SHT35",
	NULL
};

static const char* sht4x_aliases[] = {
	"SHT40",
	"SHT41",
	"SHT43",
	"SHT45",
	NULL
};

static const char* tc74_aliases[] = {
	"TC74A0",
	NULL
};

static const i2c_sensor_entry_t i2c_sensor_types[] = {
	{ "NONE", NULL, NULL, NULL, NULL, false, 0, NULL  }, /* this needs to be first so that valid sensors have index > 0 */
	{ "ADT7410", adt7410_init, adt7410_start_measurement, adt7410_get_measurement, NULL, false, 1, NULL },
	{ "AHT1x", aht1x_init, aht_start_measurement, aht_get_measurement, NULL, false, 1, NULL },
	{ "AHT2x", aht2x_init, aht_start_measurement, aht_get_measurement, NULL, false, 1, NULL },
	{ "AM2320", am2320_init, am2320_start_measurement, am2320_get_measurement, NULL, true, 1, NULL },
	{ "AS621x", as621x_init, as621x_start_measurement, as621x_get_measurement, NULL, false, 1, as621x_aliases },
	{ "BMP180", bmp180_init, bmp180_start_measurement, bmp180_get_measurement, NULL, false, 2, NULL },
	{ "BMP280", bmp280_init, bmp280_start_measurement, bmp280_get_measurement, NULL, false, 1, NULL },
	{ "DPS310", dps310_init, dps310_start_measurement, dps310_get_measurement, NULL, false, 1, NULL },
	{ "HDC302x", hdc302x_init, hdc302x_start_measurement, hdc302x_get_measurement, NULL, false, 1, hdc302x_aliases },
	{ "HTS221", hts221_init, hts221_start_measurement, hts221_get_measurement, NULL, false, 1, NULL },
	{ "HTU21D", htu21d_init, htu21d_start_measurement, htu21d_get_measurement, NULL, true, 2, NULL },
	{ "HTU31D", htu31d_init, htu31d_start_measurement, htu31d_get_measurement, NULL, false, 1, NULL },
	{ "LPSxx", lps_init, lps_start_measurement, lps_get_measurement, NULL, false, 1, lps_aliases },
	{ "MCP9808", mcp9808_init, mcp9808_start_measurement, mcp9808_get_measurement, NULL, false, 1, NULL },
	{ "MPL115A2", mpl115a2_init, mpl115a2_start_measurement, mpl115a2_get_measurement, NULL, false, 1, NULL },
	{ "MPL3115A2", mpl3115a2_init, mpl3115a2_start_measurement, mpl3115a2_get_measurement, NULL, false, 1, NULL },
	{ "MS5611", ms5611_init, ms5611_start_measurement, ms5611_get_measurement, NULL, true, 2, NULL },
	{ "MS8607", ms8607_init, ms8607_start_measurement, ms8607_get_measurement, NULL, true, 3, NULL },
	{ "PCT2075", pct2075_init, pct2075_start_measurement, pct2075_get_measurement, NULL, false, 1, NULL },
	{ "SHTC3", shtc3_init, shtc3_start_measurement, shtc3_get_measurement, NULL, false, 1, NULL },
	{ "SHT3x", sht3x_init, sht3x_start_measurement, sht3x_get_measurement, NULL, true, 1, sht3x_aliases },
	{ "SHT4x", sht4x_init, sht4x_start_measurement, sht4x_get_measurement, NULL, true, 1, sht4x_aliases },
	{ "SI7021", si7021_init, si7021_start_measurement, si7021_get_measurement, NULL, false, 1, NULL },
	{ "STTS22H", stts22h_init, stts22h_start_measurement, stts22h_get_measurement, NULL, false, 1, NULL },
	{ "TC74", tc74_init, tc74_start_measurement, tc74_get_measurement, NULL, false, 1, tc74_aliases },
	{ "TMP102", tmp102_init, tmp102_start_measurement, tmp102_get_measurement, NULL, false, 1, NULL },
	{ "TMP117", tmp117_init, tmp117_start_measurement, tmp117_get_measurement, NULL, false, 1, NULL },
	{ NULL, NULL, NULL, NULL, NULL, false, 0, NULL }
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
	int16_t res = 0;

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
	if (!(*ctx = i2c_sensor_types[sensor_type].init(i2c_bus, addr, &res)))
		return (res - 100);

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
		return res;
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
		return res;
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
		return res;
	}

	*val = buf[0];
	DEBUG_PRINT("read ok: %02x (%u)\n", *val, *val);

	return 0;
}


int i2c_write_register_block(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x,%p,%u\n", i2c, addr, reg, buf, len);

	res = i2c_write_timeout_us(i2c, addr, &reg, 1, true, I2C_WRITE_TIMEOUT(1));
	if (res < 1) {
		DEBUG_PRINT("write register failed (%d)\n", res);
		return -1;
	}
	res = i2c_write_timeout_us(i2c, addr, buf, len, false,
				I2C_WRITE_TIMEOUT(len));
	if (res < len) {
		DEBUG_PRINT("write register values failed (%d)\n", res);
		return -2;
	} else {
#if I2C_DEBUG > 0
		DEBUG_PRINT("write ok: %d [ %02x ", res + 1, reg);
		for(int i = 0; i <= len; i++) {
			printf(" %02x", buf[i]);
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


int i2c_read_raw_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t *val, bool nostop)
{
	int res;
	uint8_t buf[2];

	DEBUG_PRINT("args=%p,%02x,%p\n", i2c, addr, val);

	res = i2c_read_timeout_us(i2c, addr, buf, 2, nostop,
				I2C_READ_TIMEOUT(2));
	if (res < 2) {
		DEBUG_PRINT("read failed (%d)\n", res);
		return -2;
	}

	*val = (buf[0] << 8) | buf[1];

	DEBUG_PRINT("read ok: %04x\n", *val);

	return 0;
}


int i2c_write_raw(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf, size_t len, bool nostop)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%p,%u\n", i2c, addr, buf, len);

	res = i2c_write_timeout_us(i2c, addr, buf, len, nostop,
				I2C_WRITE_TIMEOUT(len));
	if (res != len) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	DEBUG_PRINT("write ok: %d\n", len);

	return 0;
}


int i2c_write_raw_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t val, bool nostop)
{
	uint8_t buf[2];
	int res;

	buf[0] = val >> 8;
	buf[1] = val & 0xff;

	DEBUG_PRINT("args=%p,%02x,%04x\n", i2c, addr, val);

	res = i2c_write_timeout_us(i2c, addr, buf, 2, nostop,
				I2C_WRITE_TIMEOUT(2));
	if (res < 2) {
		DEBUG_PRINT("write failed (%d)\n", res);
		return -1;
	}

	return 0;
}


int i2c_write_raw_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t val, bool nostop)
{
	int res;

	DEBUG_PRINT("args=%p,%02x,%02x\n", i2c, addr, val);

	res = i2c_write_timeout_us(i2c, addr, &val, 1, nostop,
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
		if (i2c_sensor_types[i].aliases) {
			int j = 0;
			while (i2c_sensor_types[i].aliases[j]) {
				if (!strncasecmp(i2c_sensor_types[i].aliases[j], name, len)) {
					type = i;
					break;
				}
				j++;
			}
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




