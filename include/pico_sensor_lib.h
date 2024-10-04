/* pico_sensor_lib.h
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

#ifndef PICO_SENSOR_LIB_H
#define PICO_SENSOR_LIB_H 1

#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif



uint get_i2c_sensor_type(const char *name);
const char *i2c_sensor_type_str(uint type);
bool i2c_reserved_address(uint8_t addr);

void i2c_sensor_baudrate(uint baudrate);
int i2c_init_sensor(uint8_t sensor_type, i2c_inst_t *i2c_bus, uint8_t addr, void **ctx);
int i2c_start_measurement(void *ctx);
int i2c_get_measurement(void *ctx, float *temp, float *pressure, float *humidity);
int i2c_shutdown_sensor(void *ctx);


#ifdef __cplusplus
}
#endif


#endif /* PICO_SENSOR_LIB_H */
