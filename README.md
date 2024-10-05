# pico-sensor-lib
Lightweight I2C Sensor C Library for Raspberry Pi Pico SDK

This library contains I2C Sensor (drivers) that were initially created for
[FanPico](https://github.com/tjko/fanpico) project.

## Supported Sensors

List of currently supported sensors:

Sensor Type (string)|Possible Addresses|Description|Notes
--------------------|------------------|-----------|-----
ADT7410|0x48, 0x49, 0x4a, 0x4b|16bit, 0.5C accuracy
AHT1x||AHT1x (AHT10, AHT11 ,...)
AHT2x||AHT2x (AHT20, AHT21 ,...)
AS621x||AS621x series: AS6212 (0.2C), AC6214 (0.4C), AC6218 (0.8C)
BMP180||16bit, 0.5C accuracy
BMP280|0x76, 0x77|20bit, 0.5C accuracy
DPS310|0x77, 0x76|24bit, 0.5C accuracy
LPS22|0x5d, 0x5c|Temperature and Pressure sensor
LPS25|0x5d, 0x5c|Temperature and Pressure sensor, 2C accuracy
MCP9808||13bit, 0.25C accuracy
MS8607|0x77|Temperature, Humidity and Pressure Sensor
PCT2075||11bit, 1C accuracy
SHT3x|0x44, 0x34|SHT3x Series Temperature and Humidity sensors (SHT30, SHT31, SHT35)|Not always found when scanning bus (SYS:I2C:SCAN?)
SHT4x|0x44|SHT4x Series Temperature and Humidity sensors (SHT40, SHT41, SHT43, SHT45)|Not always found when scanning bus (SYS:I2C:SCAN?)
SHTC3|0x70|Temperature and Humidity sensor, 0.2C accuracy
STTS22H|0x38, 0x3c, 0x3e, 0x3f|16bit, 0.5C accuracy
TMP102|0x48, 0x49, 0x4a, 0x4b|12bit, 2C accuracy
TMP117|0x48, 0x49, 0x4a, 0x4b|16bit, 0.1C accuracy


## Usage

### Including _pico-sensor-lib_ in a project
First, get the library (this example adds it as a submodule to existing repository):

```
$ mkdir libs
$ cd libs
$ git submodule add https://github.com/tjko/pico-sensor-lib.git
```

Then to use this library, include it in your CMakeLists.txt file:
```
# Include pico-sensor-lib library.
add_subdirectory(libs/pico-sensor-lib)
```

Also add ```pico_sensor_lib``` in _target_link_libraries_ statement:
```
target_link_libraries(myprogram PRIVATE
  ...
  pico_sensor_lib
  )
```


## Examples

See [FanPico source code](https://github.com/tjko/fanpico/blob/main/src/i2c.c)

