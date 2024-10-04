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


/**
 * Lookup sensor type code using sensor name (string).
 *
 * @param name Sensor model name.
 *
 * @return sensor_type Sensor type code (0 means no sensor found).
 */
uint get_i2c_sensor_type(const char *name);


/**
 * Get sensor name from sensor type code.
 *
 * @param sensor_type Sensor type code.
 *
 * @return Sensor name (string). Returns "NONE" if invalid type code supplied.
 */
const char *i2c_sensor_type_str(uint sensor_type);


/**
 * Check if I2C Bus Address is a reserved address.
 *
 * @param addr I2C Address
 *
 * @return True is address is valid (device) address.
 */
bool i2c_reserved_address(uint8_t addr);


/**
 * Set I2C Bus speed (baudrate).
 * This is used to calculate delay how should I2C reads/writes should
 * wait before returning error (if no response from sesnsor).
 *
 * @param baudrate Bus speed (in kHz).
 */
void i2c_sensor_baudrate(uint baudrate);


/**
 * Initialize sensor and return context that can be used to initiate measurements
 * and collect results from the sensor.
 *
 * @param sensor_type Sensor type.
 * @param i2c_bus I2C Bus instance.
 * @param addr I2C Address.
 * @param ctx Reference to sensor context.
 *
 * @return Status code,
 *          -  0, success
 *          - -1, invalid parameters
 *          - -2, device not found (no reponse to 'read')
 *          - -3, device failed to initialize (wrong device?)
 *
 * After successful call ctx will opint to allocated sensor context.
 */
int i2c_init_sensor(uint8_t sensor_type, i2c_inst_t *i2c_bus, uint8_t addr, void **ctx);


/**
 * Start measurement cycle.
 *
 * @param ctx Sensor context.
 *
 * @return Time in milliseconds until measurement is done. Negative value indicates error.
 */
int i2c_start_measurement(void *ctx);


/**
 * Get measurement results. Measurement must have been started
 * with call to i2c_start_measurement() before this should be called.
 *
 * @param ctx Sensor context
 * @param temp reference to temperature variable to populate
 * @param pressure reference to pressure variable to populate
 * @param humidity reference to humidity variable to populate
 *
 * @return Status code,
 *          -  0, success
 *          - <0, failure
 */
int i2c_read_measurement(void *ctx, float *temp, float *pressure, float *humidity);


/**
 * Run measurement and return results. This call is will block
 * to wait measurement(s) to complete.
 *
 * @param ctx Sensor context
 * @param temp reference to temperature variable to populate
 * @param pressure reference to pressure variable to populate
 * @param humidity reference to humidity variable to populate
 *
 * @return Status code,
 *          -  0, success
 *          - <0, failure
 */
int i2c_run_measurement(void *ctx, float *temp, float *pressure, float *humidity);


/**
 * Shutdown sensor and free context memory.
 *
 * @param ctx Sensor context
 *
 * @return Status code,
 *          -  0, success
 *          - <0, failure
 */
int i2c_shutdown_sensor(void *ctx);


#ifdef __cplusplus
}
#endif


#endif /* PICO_SENSOR_LIB_H */
