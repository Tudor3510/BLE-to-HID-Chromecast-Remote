#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_device.h"
#include "esp_bt_defs.h"

static const char *TAG = "BLE_BONDED_CONNECT";

// This will hold the GATT client interface once we get it from the event handler.
static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;

/**
 * Example GATTC event handler
 */
static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t ifx,
                                esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        ESP_LOGI(TAG, "GATTC_REG_EVT, gatt_if %d", ifx);
        gattc_if = ifx;
        break;
    }

    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT: connected to remote device");
        ESP_LOGI(TAG, "Remote BDA: %02x:%02x:%02x:%02x:%02x:%02x",
                param->connect.remote_bda[0],
                param->connect.remote_bda[1],
                param->connect.remote_bda[2],
                param->connect.remote_bda[3],
                param->connect.remote_bda[4],
                param->connect.remote_bda[5]);

        // Try to enforce encryption using the previously bonded keys
        esp_err_t ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set encryption, err = 0x%x", ret);
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK){
            ESP_LOGI(TAG, "ESP_GATTC_OPEN_EVT: connection opened!");
        } else {
            ESP_LOGE(TAG, "ESP_GATTC_OPEN_EVT: connection failed, status=%d",
                    param->open.status);
        }
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        ESP_LOGW(TAG, "ESP_GATTC_DISCONNECT_EVT, reason=0x%x",
                param->disconnect.reason);
        break;
    }

    default:
        break;
    }
}

/**
 * Initialize BLE in GATT Client mode and register the client callback
 */
static void ble_init_gattc(void)
{
    esp_err_t ret;

    // 1. Release memory for the Classic BT controller, if you only need BLE:
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    // 2. Initialize the BT Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    // 3. Enable BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed\n", __func__);
        return;
    }

    // 4. Initialize Bluedroid library
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    // 5. Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluedroid failed\n", __func__);
        return;
    }

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));

    // 6. Register the GATT Client callback
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gattc register error, error code = %x", ret);
        return;
    }

    // 7. Register one GATT client application (it can be arbitrary ID)
    ret = esp_ble_gattc_app_register(0);
    if (ret){
        ESP_LOGE(TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    // Optional: set preferred PHY, etc. as needed. For basic use, skip.
}

/**
 * Task that forces a connection to the first bonded device in our NVS storage
 */
static void connect_to_first_bonded(void *pvParam)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS); // A short delay so that BLE is fully initialized

    // 1. Check how many bonded devices we have
    int dev_num = esp_ble_get_bond_device_num();
    ESP_LOGI(TAG, "Number of bonded devices: %d", dev_num);

    if (dev_num == 0) {
        ESP_LOGW(TAG, "No bonded devices found; cannot connect.");
        vTaskDelete(NULL);
        return;
    }

    // 2. Retrieve the bond list
    esp_ble_bond_dev_t *bd_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!bd_list) {
        ESP_LOGE(TAG, "Malloc failed for bd_list");
        vTaskDelete(NULL);
        return;
    }
    memset(bd_list, 0, sizeof(esp_ble_bond_dev_t) * dev_num);

    esp_err_t ret = esp_ble_get_bond_device_list(&dev_num, bd_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_get_bond_device_list failed, err code = %x", ret);
        free(bd_list);
        vTaskDelete(NULL);
        return;
    }

    // 3. Choose the first device in the bond list
    esp_bd_addr_t remote_bda;
    memcpy(remote_bda, bd_list[0].bd_addr, sizeof(esp_bd_addr_t));
    free(bd_list);

    ESP_LOGI(TAG, "Attempting to connect to device[0]: %02x:%02x:%02x:%02x:%02x:%02x",
             remote_bda[0], remote_bda[1], remote_bda[2],
             remote_bda[3], remote_bda[4], remote_bda[5]);

    // 4. Use the GATT Client interface to open a direct connection
    if (gattc_if != ESP_GATT_IF_NONE) {
        ret = esp_ble_gattc_open(gattc_if, remote_bda, BLE_ADDR_TYPE_PUBLIC, true);
        if (ret) {
            ESP_LOGE(TAG, "Failed to open GATT connection, error code = %x", ret);
        }
    } else {
        ESP_LOGW(TAG, "GATT Client interface not ready");
    }

    // The task can be deleted or block forever. For demo, we just delete it:
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize NVS, required for storing bonding keys
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ble_init_gattc();  // Initialize the BLE Stack in GATT Client mode

    // Start a FreeRTOS task that attempts to connect to the first bonded device
    xTaskCreate(&connect_to_first_bonded, "connect_task", 4096, NULL, 5, NULL);
}
