#ifndef BLE_CONNECTION_H
#define BLE_CONNECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"


// Enum for IR buttons
typedef enum {
    IR_BUTTON_NONE = 0,
    IR_BUTTON_VOL_UP,
    IR_BUTTON_VOL_DOWN,
    IR_BUTTON_POWER,
    IR_BUTTON_MUTE,
    IR_BUTTON_INPUT,
    IR_BUTTON_FINISH_WRITING
} ir_button_t;

// Public API
esp_gap_ble_cb_t get_ble_gap_callback();
esp_gattc_cb_t get_ble_gattc_callback();

esp_err_t set_target_device_addr(esp_bd_addr_t target_addr);
esp_err_t handle_connection();

esp_err_t set_vol_up_ir_code(const uint8_t *value, uint8_t size);
esp_err_t set_vol_down_ir_code(const uint8_t *value, uint8_t size);
esp_err_t set_pow_ir_code(const uint8_t *value, uint8_t size);
esp_err_t set_mute_ir_code(const uint8_t *value, uint8_t size);
esp_err_t set_input_ir_code(const uint8_t *value, uint8_t size);

esp_err_t enable_ir_buttons();
esp_err_t disable_ir_buttons();

void register_ble_button_callback(void (*callback)(uint8_t));


#endif // BLE_CONNECTION_H
