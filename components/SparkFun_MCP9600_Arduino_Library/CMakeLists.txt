idf_component_register(SRC_DIRS "SparkFun_MCP9600_Arduino_Library/src"
                    INCLUDE_DIRS "SparkFun_MCP9600_Arduino_Library/src"
                    REQUIRES "arduino-esp32")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-switch -Wno-char-subscripts -Wno-overflow)
