#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_device.h"
#include "esp_mac.h"

#define GATTC_TAG "GATTC_CLIENT"
#define REMOTE_DEVICE_NAME "MyBLEDevice"
#define SCAN_DURATION_SECONDS 30
#define MAX_RETRY_COUNT 3
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define MAIN_RUNTIME_SECONDS 15 // Run time for main function before exiting

static esp_bd_addr_t target_device_addr = {0xE4, 0xE1, 0x12, 0xDB, 0x65, 0x5F};
static bool device_found = false;
static int retry_count = 0;
static uint16_t gl_gattc_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint32_t connection_timestamp;
static bool is_scanning = false;
static bool is_paired = false;
static esp_bd_addr_t current_remote_bda = {0};

static uint8_t target_ir_uuid[ESP_UUID_LEN_128] = {
        0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc,
        0x05, 0x4f, 0x21, 0x5a, 0xc0, 0xbf, 0x43, 0xd3
    };


static esp_gattc_char_elem_t charact;
static esp_gattc_char_elem_t enable_ir_writing;
static esp_gattc_char_elem_t button_to_write;
static esp_gattc_char_elem_t code_to_write;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static void start_scan(void);

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if (!is_scanning)
        {
            start_scan();
        }
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
            is_scanning = false;
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
            start_scan();
        }
        else
        {
            is_scanning = true;
            ESP_LOGI(GATTC_TAG, "Scan started successfully");
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            if (memcmp(param->scan_rst.bda, target_device_addr, sizeof(esp_bd_addr_t)) == 0)
            {
                ESP_LOGI(GATTC_TAG, "Found target device. Address: %02x:%02x:%02x:%02x:%02x:%02x",
                        param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                        param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
                device_found = true;
                if (is_scanning)
                {
                    esp_ble_gap_stop_scanning();
                }
                esp_ble_gattc_open(gl_gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Scan stopped successfully");
            is_scanning = false;
            if (!device_found)
            {
                ESP_LOGI(GATTC_TAG, "Target device not found. Retrying scan...");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
                start_scan();
            }
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(GATTC_TAG, "Pairing successful");
            is_paired = true;

            // Print the values of gl_gattc_if and conn_id before calling service discovery
            ESP_LOGI(GATTC_TAG, "gl_gattc_if: %d, conn_id: %d", gl_gattc_if, conn_id);

            esp_ble_gattc_search_service(gl_gattc_if, conn_id, NULL);

            // if (is_paired)
            // {
            //     ESP_LOGI(GATTC_TAG, "We can start again service discovery...");
            //     ESP_LOGI(GATTC_TAG, "Starting service discovery...");
            //     esp_ble_gattc_search_service(gl_gattc_if, conn_id, NULL);
            // }
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Pairing failed, reason: %d", param->ble_security.auth_cmpl.fail_reason);
            is_paired = false;
        }
        break;
    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_KEY_EVT");
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT, passkey:%d", (int)param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        // If your device requires a specific passkey, you can set it here
        // esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 123456);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // Numeric comparison request event (not used for Just Works)
        ESP_LOGI(GATTC_TAG, "Numeric Comparison");
        // esp_ble_confirm_reply(param->ble_security_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: {
        esp_ble_gap_conn_params_t *params = &param->update_conn_params;

        ESP_LOGI(GATTC_TAG, "Connection parameters updated:");
        ESP_LOGI(GATTC_TAG, "Interval: %d", params->interval_min);  // or int_max
        ESP_LOGI(GATTC_TAG, "Latency: %d", params->latency);
        ESP_LOGI(GATTC_TAG, "Timeout: %d (%.1f seconds)", params->supervision_timeout, params->supervision_timeout * 0.01);

        break;
}
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Am intraaaat aici !!!!");
        conn_id = p_data->connect.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", conn_id, gattc_if);
        esp_ble_gattc_send_mtu_req(gattc_if, conn_id);

        
        
        // Tested for now
        esp_err_t ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
        if (ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Failed to set encryption, err = 0x%x", ret);
        }



        memcpy(current_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));


        connection_timestamp = esp_log_timestamp();


        esp_ble_gattc_cb_param_t *conn = param;

        uint16_t interval = conn->connect.conn_params.interval;   // in units of 1.25ms
        uint16_t latency  = conn->connect.conn_params.latency;
        uint16_t max_timeout = ((1 + latency) * interval * 2) + 1;  // minimum valid timeout in 10ms units

        ESP_LOGI(GATTC_TAG, "Connected to peripheral");
        ESP_LOGI(GATTC_TAG, "Interval: %d (%.2f ms)", interval, interval * 1.25);
        ESP_LOGI(GATTC_TAG, "Latency: %d", latency);
        ESP_LOGI(GATTC_TAG, "Max safe supervision timeout: %d (%.1f seconds)",
                max_timeout, max_timeout * 0.01);

        esp_ble_conn_update_params_t update_params = {
            .min_int = 0x10,  // 20ms
            .max_int = 0x20,  // 40ms
            .latency = 0,
            .timeout = 3000,  // 30 seconds (in 10ms units)
        };
        memcpy(update_params.bda, conn->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&update_params);


        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status == ESP_GATT_OK)
        {
            ESP_LOGI(GATTC_TAG, "Open success");
            retry_count = 0;                                                   // Reset retry count on successful connection
            //vTaskDelay(pdMS_TO_TICKS(3000));
            //esp_ble_gattc_search_service(gattc_if, param->open.conn_id, NULL); // Discover all services
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", param->open.status);
            if (retry_count < MAX_RETRY_COUNT)
            {
                retry_count++;
                ESP_LOGI(GATTC_TAG, "Retrying connection (attempt %d of %d)...", retry_count, MAX_RETRY_COUNT);
                esp_ble_gattc_close(gattc_if, param->open.conn_id);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
                start_scan();
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "Max retry count reached. Unable to connect.");
            }
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Config MTU failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
        conn_id = 0;
        is_paired = false;
        if (retry_count < MAX_RETRY_COUNT)
        {
            retry_count++;
            ESP_LOGI(GATTC_TAG, "Retrying connection (attempt %d of %d)...", retry_count, MAX_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
            start_scan();
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Max retry count reached. Unable to maintain connection.");
        }
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        // Look for specific services, like a known UUID
        ESP_LOGI(GATTC_TAG, "Service found: %lx", (unsigned long)param->search_res.srvc_id.uuid.uuid.uuid32);


        
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
            memcmp(param->search_res.srvc_id.uuid.uuid.uuid128, target_ir_uuid, ESP_UUID_LEN_128) == 0) {
            ESP_LOGI(GATTC_TAG, "Target ir service found!");
            // Save service handle or take action


            uint16_t count = 0;

            esp_ble_gattc_get_attr_count(gattc_if, p_data->connect.conn_id, ESP_GATT_DB_CHARACTERISTIC, param->search_res.start_handle, param->search_res.end_handle, 0, &count);
            ESP_LOGI(GATTC_TAG, "Found %d characteristics for ir service", count);

            esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            esp_gatt_status_t status = esp_ble_gattc_get_all_char(gattc_if, p_data->connect.conn_id, param->search_res.start_handle, param->search_res.end_handle, char_elem_result, &count, 0);

            enable_ir_writing = char_elem_result[0];
            button_to_write = char_elem_result[1];
            code_to_write = char_elem_result[2];
        }

        if (param->search_res.srvc_id.uuid.uuid.uuid32 == 0x1812)
        {

            uint16_t count = 0;

            esp_ble_gattc_get_attr_count(gattc_if, p_data->connect.conn_id, ESP_GATT_DB_CHARACTERISTIC, param->search_res.start_handle, param->search_res.end_handle, 0, &count);
            ESP_LOGI(GATTC_TAG, "Found %d characteristics", count);

            esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            esp_gatt_status_t status = esp_ble_gattc_get_all_char(gattc_if, p_data->connect.conn_id, param->search_res.start_handle, param->search_res.end_handle, char_elem_result, &count, 0);

            if (status == ESP_GATT_OK)
            {
                ESP_LOGI(GATTC_TAG, "Successfully retrieved all characteristics.");
                // Continue with processing the characteristics
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "Failed to retrieve characteristics, error code: %d", status);
            }

            for (int i = 0; i < count; i++)
            {
                ESP_LOGI(GATTC_TAG, "Characteristic UUID: %x, properties: %x", char_elem_result[i].uuid.uuid.uuid16, char_elem_result[i].properties);
                ESP_LOGI(GATTC_TAG, "Characteristic HANDLE: %x", char_elem_result[i].char_handle);
            }

            if (char_elem_result[4].uuid.uuid.uuid32 == 0x2A4D)
            {
                ESP_LOGI(GATTC_TAG, "-----Found the correct UUID for HID: 0x00%x", char_elem_result[4].char_handle);
                charact = char_elem_result[4];
            }
        }

        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "Service search completed");
        // After services are discovered, search for the characteristic you're interested in
        // esp_ble_gattc_get_characteristic(gattc_if, param->search_cmpl.conn_id, NULL, NULL);


        esp_err_t notify_enabled_status = esp_ble_gattc_register_for_notify(gattc_if, current_remote_bda, charact.char_handle);
        if (notify_enabled_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Registered for notifications successfully");

            uint16_t count = 2;
            esp_gattc_descr_elem_t *descr_elem_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            esp_err_t status = esp_ble_gattc_get_all_descr(gattc_if, p_data->connect.conn_id, charact.char_handle, descr_elem_result, &count, 0);

            if (status == ESP_OK)
            {
                ESP_LOGI(GATTC_TAG, "Number of descriptors found: %d", count);

                // Loop through the result and display the UUID and handle of each descriptor
                for (int i = 0; i < count; i++)
                {
                    ESP_LOGI(GATTC_TAG, "Descriptor %d: Handle = 0x%04x, UUID = 0x%04x", i, descr_elem_result[i].handle, descr_elem_result[i].uuid.uuid.uuid16);

                    // If the descriptor is CCCD (UUID 0x2902), log it
                    if (descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ESP_LOGI(GATTC_TAG, "Found Client Characteristic Configuration Descriptor (CCCD) at handle 0x%04x, with i = %d", descr_elem_result[i].handle, i);
                    }
                }
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "Failed to get descriptors, error: %s", esp_err_to_name(status));
            }

            uint8_t notify_en[2] = {0x01, 0x00};  // Enable notifications
            // esp_err_t write_status = esp_ble_gattc_write_char_descr(gattc_if, conn_id, descr_elem_result[0].handle, sizeof(notify_en), notify_en, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);


            // if (write_status == ESP_OK)
            // {
            //     ESP_LOGI(GATTC_TAG, "Notification enabled successfully on the server");
            // }
            // else
            // {
            //     ESP_LOGE(GATTC_TAG, "Failed to write to CCCD to enable notifications, error code: %d", write_status);
            // }
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Failed to register for notifications, error code: %d", notify_enabled_status);
        }



        
        
        break;
    case ESP_GATTC_NOTIFY_EVT:
    {
        ESP_LOGI(GATTC_TAG, "Notification received for handle: 0x%04x", param->notify.handle);
        if (param->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "Notification received for handle: %d", param->notify.handle);
            ESP_LOGI(GATTC_TAG, "Notification data: ");
            for (int i = 0; i < param->notify.value_len; i++)
            {
                ESP_LOGI(GATTC_TAG, "0x%02x ", param->notify.value[i]);
            }
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Indication received for handle: %d", param->notify.handle);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        if (p_data->reg_for_notify.status == ESP_GATT_OK)
            ESP_LOGE(GATTC_TAG, "---Verification in separate event: Notification registered successfully");



        uint8_t en_ir_writing_value[1] = {0x01}; // Enable notification
        uint8_t ds_ir_writing_value[1] = {0x00};

        uint8_t vol_up_button[2] = {0x00, 0x18};
        uint8_t vol_up_val[] = {
            0x03, 0x21, 0x01, 0x7c, 0x00, 0x22, 0x00, 0x02, 0x01, 0x57, 0x00, 0xab, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x05, 0xf5, 0x01, 0x57, 0x00, 0x56, 0x00, 0x16, 0x0e, 0x60
        };


        uint8_t vol_down_button[2] = {0x00, 0x19};
        uint8_t vol_down_val[] = {
            0x03, 0x21, 0x01, 0x7c, 0x00, 0x22, 0x00, 0x02, 0x01, 0x57, 0x00, 0xab, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x05, 0xf5, 0x01, 0x57, 0x00, 0x56, 0x00, 0x16, 0x0e, 0x60
        };

        uint8_t pow_button[2] = {0x00, 0x1a};  // Hex value to write
        uint8_t pow_val[] = {
            0x03, 0x21, 0x01, 0x7c, 0x00, 0x22, 0x00, 0x02, 0x01, 0x57, 0x00, 0xab, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x05, 0xf5, 0x01, 0x57, 0x00, 0x56, 0x00, 0x16, 0x0e, 0x60
        };

        uint8_t mute_button[2] = {0x00, 0xa4};
        uint8_t mute_val[] = {
            0x03, 0x21, 0x01, 0x7c, 0x00, 0x22, 0x00, 0x02, 0x01, 0x57, 0x00, 0xab, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x05, 0xf5, 0x01, 0x57, 0x00, 0x56, 0x00, 0x16, 0x0e, 0x60
        };

        uint8_t input_button[2] = {0x00, 0xb2};
        uint8_t input_val[] = {
            0x03, 0x21, 0x01, 0x7c, 0x00, 0x22, 0x00, 0x02, 0x01, 0x57, 0x00, 0xab, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x05, 0xf5, 0x01, 0x57, 0x00, 0x56, 0x00, 0x16, 0x0e, 0x60
        };

        // Step 1: Register for notifications on the client side
        ESP_LOGI(GATTC_TAG, "Trying to activate IR service on device: %02x:%02x:%02x:%02x:%02x:%02x",
            current_remote_bda[0], current_remote_bda[1], current_remote_bda[2],
            current_remote_bda[3], current_remote_bda[4], current_remote_bda[5]);


        esp_err_t write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, enable_ir_writing.char_handle, sizeof(en_ir_writing_value), en_ir_writing_value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Serviciul IR se asteapta sa scriem");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Serviciul IR NU se asteapta sa scriem din cauze unei erori");
        }


        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(vol_up_button), vol_up_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Volume up button se asteapta sa primeasca valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Volume up button NU se asteapta sa primeasca valoarea din cauze unei erori");
        }

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(vol_up_val), vol_up_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Volume up button a primit valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Volume up button NU a primit valoarea din cauze unei erori");
        }


        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(vol_down_button), vol_down_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Volume down button se asteapta sa primeasca valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Volume down button NU se asteapta sa primeasca valoarea din cauze unei erori");
        }

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(vol_down_val), vol_down_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Volume down button a primit valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Volume down button NU a primit valoarea din cauze unei erori");
        }
        

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(pow_button), pow_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Power button se asteapta sa primeasca valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Power button NU se asteapta sa primeasca valoarea din cauze unei erori");
        }

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(pow_val), pow_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Power button a primit valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Power button NU a primit valoarea din cauze unei erori");
        }


        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(mute_button), mute_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Mute button se asteapta sa primeasca valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Mute button NU se asteapta sa primeasca valoarea din cauze unei erori");
        }

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(mute_val), mute_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Mute button a primit valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Mute button NU a primit valoarea din cauze unei erori");
        }


        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(input_button), input_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Input button se asteapta sa primeasca valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Input button NU se asteapta sa primeasca valoarea din cauze unei erori");
        }

        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(input_val), input_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Input button a primit valoarea");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Input button NU a primit valoarea din cauze unei erori");
        }


        write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, enable_ir_writing.char_handle, sizeof(ds_ir_writing_value), ds_ir_writing_value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (write_success_status == ESP_OK)
        {
            ESP_LOGI(GATTC_TAG, "Serviciul IR a scris tot");
        } else
        {
            ESP_LOGI(GATTC_TAG, "Serviciul IR NU a scris tot din cauze unei erori");
        }

        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        ESP_LOGE(GATTC_TAG, "-----Am intrat aici");
        if (param->write.status == ESP_GATT_OK)
        {
            ESP_LOGI(GATTC_TAG, "---Descriptor write successful. Handle: 0x%04x", param->write.handle);
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Descriptor write failed. Status: %d, Handle: 0x%04x", param->write.status, param->write.handle);
        }
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
    {
        ESP_LOGE(GATTC_TAG, "------------------Am intrat aici !!!!!");

        if (p_data->write.status == ESP_GATT_OK)
            ESP_LOGE(GATTC_TAG, "---Write was succesful.");
        else
            ESP_LOGE(GATTC_TAG, "---Write failed with status 0x%X", p_data->write.status);

        break;
    }

    default:
        break;
    }
}

static void esp_gattc_cb_wrapper(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_gattc_if = gattc_if;
            esp_gattc_cb(event, gattc_if, param);
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_gattc_if)
    {
        esp_gattc_cb(event, gattc_if, param);
    }
}

static void start_scan(void)
{
    if (!is_scanning)
    {
        esp_err_t ret = esp_ble_gap_start_scanning(SCAN_DURATION_SECONDS);
        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTC_TAG, "Failed to start scan, error code = %x", ret);
            is_scanning = false;
        }
        else
        {
            is_scanning = true;
        }
    }
    else
    {
        ESP_LOGW(GATTC_TAG, "Scan already in progress, ignoring start_scan request");
    }
}

static void set_security_parameters()
{
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_IO; // Set to ESP_IO_CAP_OUT if you want to display passkey
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    //esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    //esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    //esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    //esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    //esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
}




void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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




    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t dev_list[dev_num];
    esp_ble_get_bond_device_list(&dev_num, dev_list);

    for (int i = 0; i < dev_num; i++) {
        ESP_LOGI("BOND", "Bonded device %d: %02x:%02x:%02x:%02x:%02x:%02x",
            i,
            dev_list[i].bd_addr[0], dev_list[i].bd_addr[1], dev_list[i].bd_addr[2],
            dev_list[i].bd_addr[3], dev_list[i].bd_addr[4], dev_list[i].bd_addr[5]);
    }
    



    // Set security parameters
    set_security_parameters();

    // Register BLE callbacks
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gattc_register_callback(esp_gattc_cb_wrapper);
    esp_ble_gattc_app_register(PROFILE_A_APP_ID);

    esp_ble_gatt_set_local_mtu(500);

    // Start scanning
    start_scan();

    // ✅ Main Loop: Check when to start service discovery
    ESP_LOGI(GATTC_TAG, "Starting main loop for %d seconds...", MAIN_RUNTIME_SECONDS);
    uint32_t start_time = esp_log_timestamp();
    
    bool service_discovery_started = false;

    while ((esp_log_timestamp() - start_time) < (MAIN_RUNTIME_SECONDS * 1000))
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // ✅ Avoids blocking the system

        if (is_paired)
        {
            ESP_LOGI(GATTC_TAG, "We are connected...");

            // if (is_paired && !service_discovery_started)
            // {
            //     uint32_t elapsed_time = esp_log_timestamp() - connection_timestamp;
            //     if (elapsed_time >= 1000) // ✅ Wait 1 second after connection
            //     {
            //         ESP_LOGI(GATTC_TAG, "Calling esp_ble_gattc_search_service...");
            //         service_discovery_started = true; // ✅ Ensure it runs only once
            //     }
            // }
        }
    }

    // Cleanup
    ESP_LOGI(GATTC_TAG, "Main loop completed. Cleaning up...");
    if (conn_id != 0)
    {
        esp_ble_gattc_close(gl_gattc_if, conn_id);
    }

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    ESP_LOGI(GATTC_TAG, "BLE client terminated");
}
