idf_component_register(SRC_DIRS "MPU9255-Arduino-Library/src"
                    INCLUDE_DIRS "MPU9255-Arduino-Library/src"
                    REQUIRES "arduino-esp32")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-switch -Wno-char-subscripts)
