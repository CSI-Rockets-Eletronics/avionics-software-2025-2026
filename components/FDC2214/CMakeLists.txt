idf_component_register(SRC_DIRS "FDC2214/src"
                    INCLUDE_DIRS "FDC2214/src"
                    REQUIRES "arduino-esp32")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-switch -Wno-char-subscripts -Wno-overflow)
