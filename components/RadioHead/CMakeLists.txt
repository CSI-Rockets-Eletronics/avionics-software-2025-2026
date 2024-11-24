idf_component_register(SRC_DIRS "RadioHead"
                    INCLUDE_DIRS "RadioHead"
                    REQUIRES "arduino-esp32")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-register)
