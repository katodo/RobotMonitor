cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)


project(robotmonitor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


link_directories(libmicroros)

add_executable(robotmonitor
    robotmonitor.c
    pico_uart_transport.c
)


# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(robotmonitor ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_link_libraries(robotmonitor
    pico_stdlib
    microros
    hardware_pio
	hardware_dma 
	hardware_irq
)

target_include_directories(robotmonitor PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(robotmonitor 1)
pico_enable_stdio_uart(robotmonitor 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(robotmonitor)
