cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(My_start_ble_dm_central)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})
# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/pwm_led.c
  src/peer.c
  src/service.c
  )
# NORDIC SDK APP END

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
