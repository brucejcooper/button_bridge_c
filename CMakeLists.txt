cmake_minimum_required(VERSION 4.0.0)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(WIZNET_CHIP W5100S)
# add_definitions(-D_WIZCHIP_=W5100S)
# add_definitions(-D_DNS_DEBUG_)
# add_definitions(-DDEVICE_BOARD_NAME=W5100S_EVB_PICO)
# set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
# set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)

# set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
# set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
# add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
# add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)




include(libraries/pico-sdk/pico_sdk_init.cmake)

project(button_handler)


include(wiznet_pico_c_sdk_version.cmake)
# include(libraries/ioLibrary_Driver.cmake)
include(secrets.cmake)


add_executable(button_handler
   src/main.c
   src/dali.c
   src/modbus.c
   src/buttons.c
   src/modbus_receiver.c
   src/crcbuf.c
   src/regs.c
)


target_include_directories(button_handler PRIVATE 
   /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/arm-none-eabi/include
   ${CMAKE_CURRENT_LIST_DIR}/src 
   ${CMAKE_CURRENT_LIST_DIR}/libraries/pico-sdk/src/rp2_common/hardware_pio/include
   # ${WIZNET_DIR}/Ethernet
   # ${WIZNET_DIR}/Internet/DHCP
   # ${WIZNET_DIR}/Internet/DNS
   # ${WIZNET_DIR}/Internet/MQTT
   # libraries/cjson

)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()
pico_generate_pio_header(button_handler ${CMAKE_CURRENT_LIST_DIR}/src/dali.pio)
pico_generate_pio_header(button_handler ${CMAKE_CURRENT_LIST_DIR}/src/modbus.pio)
pico_enable_stdio_usb(button_handler 1)
pico_enable_stdio_uart(button_handler 0)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(button_handler pico_stdlib hardware_pio hardware_dma hardware_spi pico_multicore) #ETHERNET_FILES W5100S_FILES DHCP_FILES DNS_FILES MQTT_FILES)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(button_handler)