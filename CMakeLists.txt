cmake_minimum_required(VERSION 3.5)

include(libraries/pico-sdk/pico_sdk_init.cmake)
project(button_handler)


add_executable(button_handler
   src/main.c
   src/dali.c
   src/modbus.c
   src/queue.c
   src/buttons.c
   src/cli.c
)


target_include_directories(button_handler PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src ${CMAKE_CURRENT_LIST_DIR}/libraries/pico-sdk/src/rp2_common/hardware_pio/include)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()
pico_generate_pio_header(button_handler ${CMAKE_CURRENT_LIST_DIR}/src/dali.pio)
pico_generate_pio_header(button_handler ${CMAKE_CURRENT_LIST_DIR}/src/modbus.pio)
pico_enable_stdio_usb(button_handler 1)
pico_enable_stdio_uart(button_handler 0)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(button_handler pico_stdlib hardware_pio)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(button_handler)