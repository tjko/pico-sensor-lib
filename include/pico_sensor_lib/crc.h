/* crc.h
   Copyright (C) 2024 Timo Kokkonen <tjko@iki.fi>

   SPDX-License-Identifier: GPL-3.0-or-later

   This file is part of Pico-Sensor-Lib.

   Pico-Sensor-Lib is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Pico-Sensor-Lib is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Pico-Sensor-Lib. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef PICO_SENSOR_LIB_CRC_H
#define PICO_SENSOR_LIB_CRC_H 1

#ifdef __cplusplus
extern "C"
{
#endif


/* crc.c */
uint8_t crc8_generic(uint8_t *buf, size_t len, uint8_t polynomial, uint8_t initial, uint8_t final, bool in_reversed, bool out_reversed);
uint8_t crc8_byte(uint8_t crc, uint8_t byte, uint8_t polynomial);


#ifdef __cplusplus
}
#endif

#endif /* PICO_SENSOR_LIB_CRC_H */
