idf_component_register(SRCS "ble_connection.c" "ble_stack_manager.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash tinyusb)