#include "esp_bt.h"                 // esp_bt_controller_mem_release, esp_bt_controller_enable, etc.
#include "esp_bt_main.h"           // esp_bluedroid_init, esp_bluedroid_enable
#include "esp_bt_device.h"         // (Optional: for device name functions, etc.)
#include "esp_err.h"               // esp_err_t and ESP_ERROR_CHECK
#include "esp_mac.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"   // For esp_ble_gap_set_security_param(), esp_ble_auth_req_t, and related constants



#define PROFILE_A_APP_ID 0


static const char *TAG = "BLE_INIT";


void ble_stack_init()
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    ESP_ERROR_CHECK(ret);
}


void register_ble_callbacks(esp_gap_ble_cb_t gap_cb, esp_gattc_cb_t gattc_cb)
{
    if (gap_cb == NULL || gattc_cb == NULL) {
        ESP_LOGE(TAG, "GAP and GATTC callbacks must not be NULL.");
        abort();
    }

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_A_APP_ID));
}


void ble_stack_deinit()
{
    // Unregister the application (optional, but good practice)
    esp_ble_gattc_app_unregister(PROFILE_A_APP_ID);

    // Disable and deinit the Bluetooth stack
    esp_bluedroid_disable();
    esp_bluedroid_deinit();

    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    // Optionally release memory (depending on if you plan to reinitialize later)
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
}
