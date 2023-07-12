set(SOURCES "ble_module.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES wifi_module bt)