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

// For convenience, define the numeric code for connectable directed adv:
#define ADV_DIRECT_IND 0x01

static const char *TAG = "BLE_BONDED_CONNECT";

static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;

// We'll store the first bonded device's address here
static esp_bd_addr_t s_bonded_addr;  
static bool s_has_bonded_addr = false;

// Scan parameters (feel free to adjust)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t ifx,
                                esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATTC_REG_EVT, gatt_if = %d", ifx);
        gattc_if = ifx;
        break;

    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT: connected to remote device");
        ESP_LOGI(TAG, "Remote BDA: %02x:%02x:%02x:%02x:%02x:%02x",
                 param->connect.remote_bda[0],
                 param->connect.remote_bda[1],
                 param->connect.remote_bda[2],
                 param->connect.remote_bda[3],
                 param->connect.remote_bda[4],
                 param->connect.remote_bda[5]);

        // Optionally enforce encryption using previously bonded keys
        esp_err_t ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set encryption, err = 0x%x", ret);
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "ESP_GATTC_OPEN_EVT: connection opened!");
        } else {
            ESP_LOGE(TAG, "ESP_GATTC_OPEN_EVT: connection failed, status = %d",
                     param->open.status);
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "ESP_GATTC_DISCONNECT_EVT, reason=0x%x",
                 param->disconnect.reason);
        break;

    default:
        break;
    }
}

/** GAP callback to handle scan events, etc. */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Start scanning after we set scan params
        esp_ble_gap_start_scanning(30); // Scan for 30 seconds
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Scan started successfully");
        } else {
            ESP_LOGE(TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // Check if it is connectable directed adv
            uint8_t adv_type = scan_result->scan_rst.ble_evt_type;
            // Compare with our known bonded device?
            if (adv_type == ADV_DIRECT_IND) {
                // The 'bda' might be the advertiser's address. For a true directed adv,
                // the "target" address is typically in scan_rst->scan_rst.direct_rda.
                // But some stacks put the advertiser's address in bda. Let's do a quick example:
                esp_bd_addr_t *adv_bda = (esp_bd_addr_t *)&scan_result->scan_rst.bda;
                if (memcmp(adv_bda, s_bonded_addr, sizeof(esp_bd_addr_t)) == 0) {
                    ESP_LOGI(TAG, "Detected connectable directed adv from bonded device!");
                    // Stop scanning and connect
                    esp_ble_gap_stop_scanning();
                    if (gattc_if != ESP_GATT_IF_NONE) {
                        esp_err_t err = esp_ble_gattc_open(gattc_if, *adv_bda, BLE_ADDR_TYPE_PUBLIC, true);
                        if (err) {
                            ESP_LOGE(TAG, "esp_ble_gattc_open failed, err = 0x%x", err);
                        }
                    }
                }
            }
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan stopped");
        break;

    default:
        break;
    }
}

/** Initialize BLE in GATT Client mode, register client + GAP callbacks */
static void ble_init_gattc(void)
{
    esp_err_t ret;

    // 1. Release memory for Classic BT, if not needed
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    // 2. Initialize the BT Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %d", ret);
        return;
    }

    // 3. Enable the BT controller in BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable BLE failed: %d", ret);
        return;
    }

    // 4. Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %d", ret);
        return;
    }

    // 5. Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %d", ret);
        return;
    }

    // (Optional) set bonding or no-bond parameters here:
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  // or ESP_LE_AUTH_NO_BOND
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));

    // 6. Register the GATT Client callback
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gattc register error, code = 0x%x", ret);
        return;
    }

    // Register the GAP callback
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, code = 0x%x", ret);
        return;
    }

    // 7. Register a GATT client application (app_id=0)
    ret = esp_ble_gattc_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "gattc app register error, code = 0x%x", ret);
        return;
    }
}

/**
 * Task that checks for any bonded devices; if found,
 * it saves the address of the first one and sets scan params.
 */
static void connect_to_first_bonded(void *pvParam)
{
    // Wait a bit for BLE stack to be ready
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // 1. Check how many bonded devices we have
    int dev_num = esp_ble_get_bond_device_num();
    ESP_LOGI(TAG, "Number of bonded devices: %d", dev_num);

    if (dev_num == 0) {
        ESP_LOGW(TAG, "No bonded devices found; nothing to do.");
        vTaskDelete(NULL);
        return;
    }

    // 2. Retrieve the bond list
    esp_ble_bond_dev_t *bd_list = calloc(dev_num, sizeof(esp_ble_bond_dev_t));
    if (!bd_list) {
        ESP_LOGE(TAG, "Failed to allocate memory for bond dev list");
        vTaskDelete(NULL);
        return;
    }

    esp_err_t ret = esp_ble_get_bond_device_list(&dev_num, bd_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_get_bond_device_list failed, code=0x%x", ret);
        free(bd_list);
        vTaskDelete(NULL);
        return;
    }

    // 3. Choose the first device in the bond list
    memcpy(s_bonded_addr, bd_list[0].bd_addr, sizeof(esp_bd_addr_t));
    s_has_bonded_addr = true;

    ESP_LOGI(TAG, "We will connect only if we see connectable directed adv from: "
             "%02x:%02x:%02x:%02x:%02x:%02x",
             s_bonded_addr[0], s_bonded_addr[1], s_bonded_addr[2],
             s_bonded_addr[3], s_bonded_addr[4], s_bonded_addr[5]);

    free(bd_list);

    // 4. Set the scan parameters
    esp_ble_gap_set_scan_params(&ble_scan_params);

    // The task can exit
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize NVS for storing bond keys
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BLE
    ble_init_gattc();

    // Start the task that checks for the first bonded device and triggers scanning
    xTaskCreate(&connect_to_first_bonded, "connect_task", 4096, NULL, 5, NULL);
}
