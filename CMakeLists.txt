cmake_minimum_required(VERSION 3.5)

# Enable USB CDC for Serial output (makes Serial use USB instead of UART0)
# Must be set before project() so Arduino component sees these flags
add_compile_definitions(ARDUINO_USB_MODE=1)
add_compile_definitions(ARDUINO_USB_CDC_ON_BOOT=1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(avionics-software-2025-2026)
