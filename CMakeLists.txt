cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME lora-sensor-pico) # <-- Name your project/executable here!

set(PICO_SDK_PATH /opt/share/playing/lora/pico/pico-sdk)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)
# include(pico_lorawan.cmake)

add_subdirectory("lib/pico-lorawan" EXCLUDE_FROM_ALL)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
   src/main.cpp # <-- Add source files here!
)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
# include(libraries/pico_explorer/pico_explorer)
# include(libraries/pico_display/pico_display)
include(drivers/bme280/bme280)
include(common/pimoroni_i2c)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    # pico_explorer # <-- List libraries here!
    pico_stdlib
    bme280
    pico_lorawan
)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
