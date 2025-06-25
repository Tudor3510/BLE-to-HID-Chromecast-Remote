#include "ble_stack_manager.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_gatt_common_api.h"

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
    esp_ble_gattc_app_unregister(PROFILE_A_APP_ID);

    esp_bluedroid_disable();
    esp_bluedroid_deinit();

    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
}
