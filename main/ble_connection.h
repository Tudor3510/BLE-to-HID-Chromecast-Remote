/*******************  Standard C *******************/
#include <stdint.h>     // uint8_t, uint16_t, etc.
#include <stdbool.h>    // bool
#include <string.h>     // memset, memcpy, memcmp
#include <stdlib.h>     // malloc, free

/*******************  FreeRTOS ********************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"     // vTaskDelay, pdMS_TO_TICKS
#include "freertos/queue.h"    // xQueueSend, xQueueReceive

/*******************  ESP-IDF core ****************/
#include "esp_system.h"
#include "esp_err.h"           // esp_err_t, ESP_OK
#include "esp_log.h"           // ESP_LOGI, ESP_LOGE
#include "esp_timer.h"         // esp_timer_get_time() (if you later need it)

/*******************  ESP-IDF Bluetooth / BLE ****/
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#include "esp_gap_ble_api.h"      // esp_gap_ble_cb_event_t, esp_ble_gap_*
#include "esp_gatt_defs.h"        // GATT UUID/handle defs
#include "esp_gatt_common_api.h"  // common helpers (ESP-IDF ≥4.0)
#include "esp_gattc_api.h"        // esp_gattc_* client API

/*******************  TinyUSB (HID) ***************/
#include "tusb.h"                 // tud_hid_report()





#define GATTC_TAG "GATTC_CLIENT"


// For convenience, define the numeric code for connectable directed adv:
#define ADV_DIRECT_IND 0x01

static esp_bd_addr_t target_device_addr = {0xE4, 0xE1, 0x12, 0xDB, 0x65, 0x5F};
static uint16_t gl_gattc_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;

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
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ONLY_WLST,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};



static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Failed to set scan parameters, status = 0x%02x", param->scan_param_cmpl.status);
        }
    
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
            start_scan();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            if (memcmp(param->scan_rst.bda, target_device_addr, sizeof(esp_bd_addr_t)) == 0) //&& param->scan_rst.ble_evt_type == ADV_DIRECT_IND)
            {
                ESP_LOGI(GATTC_TAG, "Found target device. Address: %02x:%02x:%02x:%02x:%02x:%02x",
                        param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                        param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);

                esp_ble_gap_stop_scanning();

                esp_ble_gattc_open(gl_gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
        }
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(GATTC_TAG, "Pairing successful");

            // Print the values of gl_gattc_if and conn_id before calling service discovery
            ESP_LOGI(GATTC_TAG, "gl_gattc_if: %d, conn_id: %d", gl_gattc_if, conn_id);

            esp_ble_gattc_search_service(gl_gattc_if, conn_id, NULL);
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Pairing failed, reason: %d", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
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
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_gattc_if = gattc_if;
            ESP_LOGI(GATTC_TAG, "REG_EVT successful");
            esp_ble_gap_set_scan_params(&ble_scan_params);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Am intraaaat aici !!!!");
        conn_id = p_data->connect.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", conn_id, gattc_if);
        esp_ble_gattc_send_mtu_req(gattc_if, conn_id);

        
        // Tested for now
        esp_err_t ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
        if (ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Failed to set encryption on ESP_GATTC_CONNECT_EVT, err = 0x%x", ret);
        }

        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Failed to open connection, status = 0x%02X", param->open.status);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Config MTU failed, error status = %x", param->cfg_mtu.status);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Start scanning because of disconnect reason 0x%02X", param->disconnect.reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        start_scan();

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
        // After services are discovered, search for the characteristic you're interested in
        // esp_ble_gattc_get_characteristic(gattc_if, param->search_cmpl.conn_id, NULL, NULL);
        esp_err_t notify_enabled_status = esp_ble_gattc_register_for_notify(gattc_if, target_device_addr, charact.char_handle);
        if (notify_enabled_status != ESP_OK)
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
            ESP_LOGI(GATTC_TAG, "0x%02x ", param->notify.value[0]);

            ESP_LOGI(TINY_USB_TAG, "Sending Keyboard report");
            
            if (param->notify.value[0] == REMOTE_RELEASE_KEY)
            {
                ESP_LOGI(TINY_USB_TAG, "--Release Key");

                hid_report_mapper hid_report_mapper_copy;
                xQueueReceive(release_button_queue, &hid_report_mapper_copy, 0);
                tud_hid_report(hid_report_mapper_copy.report_id, hid_report_mapper_copy.keycode, hid_report_mapper_copy.length);
            }
            else
            {
                hid_report_mapper hid_report_mapper_copy = remote_map_windows_hid[param->notify.value[0]];
                tud_hid_report(hid_report_mapper_copy.report_id, hid_report_mapper_copy.keycode, hid_report_mapper_copy.length);

                hid_report_mapper_copy.keycode[0] = 0, hid_report_mapper_copy.keycode[2] = 0;
                xQueueSend(release_button_queue, &hid_report_mapper_copy, 0);
            }

            // xQueueSend(hid_queue, &remote_map_windows_hid[param->notify.value[0]], 0); // non-blocking send
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Indication received for handle: %d", param->notify.handle);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Failed to register for notification, status = 0x%x", p_data->reg_for_notify.status);
        }

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
        ESP_LOGI(GATTC_TAG, "Trying to activate IR service on device");


        // esp_err_t write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, enable_ir_writing.char_handle, sizeof(en_ir_writing_value), en_ir_writing_value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Serviciul IR se asteapta sa scriem");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Serviciul IR NU se asteapta sa scriem din cauze unei erori");
        // }


        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(vol_up_button), vol_up_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume up button se asteapta sa primeasca valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume up button NU se asteapta sa primeasca valoarea din cauze unei erori");
        // }

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(vol_up_val), vol_up_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume up button a primit valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume up button NU a primit valoarea din cauze unei erori");
        // }


        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(vol_down_button), vol_down_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume down button se asteapta sa primeasca valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume down button NU se asteapta sa primeasca valoarea din cauze unei erori");
        // }

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(vol_down_val), vol_down_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume down button a primit valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Volume down button NU a primit valoarea din cauze unei erori");
        // }
        

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(pow_button), pow_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Power button se asteapta sa primeasca valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Power button NU se asteapta sa primeasca valoarea din cauze unei erori");
        // }

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(pow_val), pow_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Power button a primit valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Power button NU a primit valoarea din cauze unei erori");
        // }


        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(mute_button), mute_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Mute button se asteapta sa primeasca valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Mute button NU se asteapta sa primeasca valoarea din cauze unei erori");
        // }

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(mute_val), mute_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Mute button a primit valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Mute button NU a primit valoarea din cauze unei erori");
        // }


        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, button_to_write.char_handle, sizeof(input_button), input_button, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Input button se asteapta sa primeasca valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Input button NU se asteapta sa primeasca valoarea din cauze unei erori");
        // }

        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, code_to_write.char_handle, sizeof(input_val), input_val, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Input button a primit valoarea");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Input button NU a primit valoarea din cauze unei erori");
        // }


        // write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, enable_ir_writing.char_handle, sizeof(ds_ir_writing_value), ds_ir_writing_value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        // if (write_success_status == ESP_OK)
        // {
        //     ESP_LOGI(GATTC_TAG, "Serviciul IR a scris tot");
        // } else
        // {
        //     ESP_LOGI(GATTC_TAG, "Serviciul IR NU a scris tot din cauze unei erori");
        // }

        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (param->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Descriptor write failed. Status: %d, Handle: 0x%04x", param->write.status, param->write.handle);
        }
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
    {

        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "---Write failed with status 0x%X", p_data->write.status);
        }
        break;
    }

    default:
        break;
    }
}
