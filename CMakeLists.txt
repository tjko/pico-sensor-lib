add_library(pico_sensor_lib INTERFACE)

target_include_directories(pico_sensor_lib INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico_sensor_lib INTERFACE
  hardware_i2c
)

target_sources(pico_sensor_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c.c
  ${CMAKE_CURRENT_LIST_DIR}/src/crc.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_adt7410.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_aht.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_as621x.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_bmp180.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_bmp280.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_dps310.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_lps22.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_lps25.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_mcp9808.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_ms8607.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_pct2075.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_shtc3.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_sht3x.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_sht4x.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_stts22h.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_tmp102.c
  ${CMAKE_CURRENT_LIST_DIR}/src/i2c_tmp117.c
)
