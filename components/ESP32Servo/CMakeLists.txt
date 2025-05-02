idf_component_register(SRC_DIRS "ESP32Servo/src"
                    INCLUDE_DIRS "ESP32Servo/src"
                    REQUIRES "arduino-esp32")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format= -Wno-error=format-extra-args)
