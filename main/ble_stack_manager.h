#ifndef BLE_STACK_MANAGER_H
#define BLE_STACK_MANAGER_H

#include "esp_gap_ble_api.h"   // esp_gap_ble_cb_t
#include "esp_gattc_api.h"     // esp_gattc_cb_t


// Initializes the BLE stack.
void ble_stack_init();

// Registers GAP and GATTC callbacks.
void register_ble_callbacks(esp_gap_ble_cb_t gap_cb, esp_gattc_cb_t gattc_cb);

// Deinitializes the BLE stack.
void ble_stack_deinit();


#endif // BLE_STACK_MANAGER_H
