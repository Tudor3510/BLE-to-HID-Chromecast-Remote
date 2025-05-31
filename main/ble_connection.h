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
#define IR_CODE_SET_TAG "IR_CODE"
#define TASK_SYNC_TAG "TASK_SYNC"

// For convenience, define the numeric code for connectable directed adv:
#define ADV_DIRECT_IND 0x01

#define HID_SERV_UUID32 0x1812
#define HID_REPORT_CHAR 0x2A4D

static const uint8_t TARGET_IR_SERV_UUID[ESP_UUID_LEN_128] = {
    0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc,
    0x05, 0x4f, 0x21, 0x5a, 0xc0, 0xbf, 0x43, 0xd3
};
#define IR_CONFIG_CHAR_INDEX 0
#define IR_BUTTON_CHAR_INDEX 1
#define IR_CODE_CHAR_INDEX 2


static const uint8_t IR_CONFIG_ENABLE_VAL = 0x01;
static const uint8_t IR_CONFIG_DISABLE_VAL = 0x00;
static const uint8_t VOL_UP_BUTTON[2] = {0x00, 0x18};
static const uint8_t VOL_DOWN_BUTTON[2] = {0x00, 0x19};
static const uint8_t POW_BUTTON[2] = {0x00, 0x1a};  // Hex value to write
static const uint8_t MUTE_BUTTON[2] = {0x00, 0xa4};
static const uint8_t INPUT_BUTTON[2] = {0x00, 0xb2};


static esp_bd_addr_t target_device_addr;
static uint16_t gl_gattc_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;

static esp_gattc_char_elem_t hid_report_char;
static esp_gattc_char_elem_t ir_config_char;
static esp_gattc_char_elem_t ir_button_char;
static esp_gattc_char_elem_t ir_code_char;


static uint8_t vol_up_val_sz;
static uint8_t *vol_up_val;

static uint8_t vol_down_val_sz;
static uint8_t *vol_down_val;

static uint8_t pow_val_sz;
static uint8_t *pow_val;

static uint8_t mute_val_sz;
static uint8_t *mute_val;

static uint8_t input_val_sz;
static uint8_t *input_val;


static SemaphoreHandle_t ir_write_in_progress_mtx = NULL;
static bool ir_write_in_progress = false;

typedef enum {
    IR_BUTTON_NONE = 0,
    IR_BUTTON_VOL_UP,
    IR_BUTTON_VOL_DOWN,
    IR_BUTTON_POWER,
    IR_BUTTON_MUTE,
    IR_BUTTON_INPUT,
    IR_BUTTON_FINISH_WRITING
    // Add more buttons as needed
} ir_button_t;

static volatile ir_button_t ir_btn_to_write_next = IR_BUTTON_NONE;
static volatile esp_err_t ir_write_result = ESP_OK;


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ONLY_WLST,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};


// Step 1: Declare a static variable to store the callback
static void (*ble_button_cb)(uint8_t) = NULL;

// Step 2: Register function
void register_ble_button_callback(void (*callback)(uint8_t)) {
    ble_button_cb = callback;
}


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
            esp_ble_gap_start_scanning(0);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            if (memcmp(param->scan_rst.bda, target_device_addr, sizeof(esp_bd_addr_t)) == 0) //&& param->scan_rst.ble_evt_type == ADV_DIRECT_IND)
            {
                

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
            esp_ble_gap_set_scan_params(&ble_scan_params);
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "REG_EVT failed, status: 0x%X", param->reg.status);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
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
        esp_ble_gap_start_scanning(0);

        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        // Look for specific services, like a known UUID        
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
            memcmp(param->search_res.srvc_id.uuid.uuid.uuid128, TARGET_IR_SERV_UUID, ESP_UUID_LEN_128) == 0) {
            ESP_LOGI(GATTC_TAG, "Target ir service found!");
            // Save service handle or take action


            uint16_t count = 0;

            esp_ble_gattc_get_attr_count(gattc_if, p_data->connect.conn_id, ESP_GATT_DB_CHARACTERISTIC, param->search_res.start_handle, param->search_res.end_handle, 0, &count);
            ESP_LOGI(GATTC_TAG, "Found %d characteristics for ir service", count);

            esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            esp_gatt_status_t status = esp_ble_gattc_get_all_char(gattc_if, p_data->connect.conn_id, param->search_res.start_handle, param->search_res.end_handle, char_elem_result, &count, 0);

            ir_config_char = char_elem_result[IR_CONFIG_CHAR_INDEX];
            ir_button_char = char_elem_result[IR_BUTTON_CHAR_INDEX];
            ir_code_char = char_elem_result[IR_CODE_CHAR_INDEX];
        }

        if (param->search_res.srvc_id.uuid.uuid.uuid32 == HID_SERV_UUID32)
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
                if (char_elem_result[i].uuid.uuid.uuid32 == HID_REPORT_CHAR)
                {
                    ESP_LOGI(GATTC_TAG, "-----Found the correct UUID for HID: 0x00%x", char_elem_result[i].char_handle);
                    hid_report_char = char_elem_result[i];
                }
            }

        }

        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        // After services are discovered, search for the characteristic you're interested in
        // esp_ble_gattc_get_characteristic(gattc_if, param->search_cmpl.conn_id, NULL, NULL);
        esp_err_t notify_enabled_status = esp_ble_gattc_register_for_notify(gattc_if, target_device_addr, hid_report_char.char_handle);
        if (notify_enabled_status != ESP_OK)
        {
            ESP_LOGE(GATTC_TAG, "Failed to register for notifications, error code: %d", notify_enabled_status);
        }

        
        break;
    case ESP_GATTC_NOTIFY_EVT:
    {
        if (param->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "Notification received for handle: %d", param->notify.handle);

            ble_button_cb(param->notify.value[0]);
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
            ir_write_result = ESP_FAIL;
            if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY))
            {
                ir_write_in_progress = false;
                xSemaphoreGive(ir_write_in_progress_mtx);
            }
            else
            {
                ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex in enable_ir_buttons()");
                ir_write_in_progress = false;
            }
            break;
        }

        if (p_data->write.handle == ir_config_char.char_handle && ir_btn_to_write_next == IR_BUTTON_NONE)
        {
            if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY))
            {
                ir_write_in_progress = false;
                xSemaphoreGive(ir_write_in_progress_mtx);
            }
            else
            {
                ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex in enable_ir_buttons()");
                ir_write_result = ESP_FAIL;
                ir_write_in_progress = false;
            }
            break;
        }
        
        if (ir_btn_to_write_next == IR_BUTTON_FINISH_WRITING)
        {
            ir_btn_to_write_next = IR_BUTTON_NONE;

            esp_err_t write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, ir_config_char.char_handle, sizeof(IR_CONFIG_DISABLE_VAL), &IR_CONFIG_DISABLE_VAL, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (write_success_status != ESP_OK)
            {
                ESP_LOGE(GATTC_TAG,
                        "Failed to initiate write to IR config characteristic (handle: 0x%04x). "
                        "esp_ble_gattc_write_char() returned 0x%x (%s). Possible causes: invalid connection, "
                        "write not permitted, or GATT busy.",
                        ir_config_char.char_handle,
                        write_success_status,
                        esp_err_to_name(write_success_status));

                ir_write_result = ESP_FAIL;
                // Reset the in-progress flag before returning
                if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY))
                {
                    ir_write_in_progress = false;
                    xSemaphoreGive(ir_write_in_progress_mtx);
                }
                else
                {
                    ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex while resetting ir_write_in_progress");
                    ir_write_in_progress = false;
                }

                break;
            }

            break;
        }

        if (p_data->write.handle == ir_config_char.char_handle || p_data->write.handle == ir_code_char.char_handle)
        {
            uint8_t ir_btn_len;
            uint8_t *ir_btn_buf;

            if (ir_btn_to_write_next == IR_BUTTON_INPUT)
            {
                ir_btn_len = sizeof(INPUT_BUTTON);
                ir_btn_buf = INPUT_BUTTON;
            }
            if (ir_btn_to_write_next == IR_BUTTON_MUTE)
            {
                ir_btn_len = sizeof(MUTE_BUTTON);
                ir_btn_buf = MUTE_BUTTON;
            }
            if (ir_btn_to_write_next == IR_BUTTON_POWER)
            {
                ir_btn_len = sizeof(POW_BUTTON);
                ir_btn_buf = POW_BUTTON;
            }
            if (ir_btn_to_write_next == IR_BUTTON_VOL_DOWN)
            {
                ir_btn_len = sizeof(VOL_DOWN_BUTTON);
                ir_btn_buf = VOL_DOWN_BUTTON;
            }
            if (ir_btn_to_write_next == IR_BUTTON_VOL_UP)
            {
                ir_btn_len = sizeof(VOL_UP_BUTTON);
                ir_btn_buf = VOL_UP_BUTTON;
            }

            esp_err_t write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, ir_button_char.char_handle, ir_btn_len, ir_btn_buf, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (write_success_status != ESP_OK)
            {
                ESP_LOGE(GATTC_TAG,
                        "Failed to initiate write to IR config characteristic (handle: 0x%04x). "
                        "esp_ble_gattc_write_char() returned 0x%x (%s). Possible causes: invalid connection, "
                        "write not permitted, or GATT busy.",
                        ir_config_char.char_handle,
                        write_success_status,
                        esp_err_to_name(write_success_status));

                ir_write_result = ESP_FAIL;
                // Reset the in-progress flag before returning
                if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY))
                {
                    ir_write_in_progress = false;
                    xSemaphoreGive(ir_write_in_progress_mtx);
                }
                else
                {
                    ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex while resetting ir_write_in_progress");
                    ir_write_in_progress = false;
                }
            }

            break;
        }

        if (p_data->write.handle == ir_button_char.char_handle)
        {
            uint8_t ir_btn_len = 0;
            uint8_t *ir_btn_buf = NULL;

            if (ir_btn_to_write_next == IR_BUTTON_INPUT)
            {
                ir_btn_len = input_val_sz;
                ir_btn_buf = input_val;

                ir_btn_to_write_next = IR_BUTTON_FINISH_WRITING;
            }
            if (ir_btn_to_write_next == IR_BUTTON_MUTE)
            {
                ir_btn_len = mute_val_sz;
                ir_btn_buf = mute_val;

                ir_btn_to_write_next = IR_BUTTON_INPUT;
            }
            if (ir_btn_to_write_next == IR_BUTTON_POWER)
            {
                ir_btn_len = pow_val_sz;
                ir_btn_buf = pow_val;

                ir_btn_to_write_next = IR_BUTTON_MUTE;
            }
            if (ir_btn_to_write_next == IR_BUTTON_VOL_DOWN)
            {
                ir_btn_len = vol_down_val_sz;
                ir_btn_buf = vol_down_val;

                ir_btn_to_write_next = IR_BUTTON_POWER;
            }
            if (ir_btn_to_write_next == IR_BUTTON_VOL_UP)
            {
                ir_btn_len = vol_up_val_sz;
                ir_btn_buf = vol_up_val;

                ir_btn_to_write_next = IR_BUTTON_VOL_DOWN;
            }

            esp_err_t write_success_status = esp_ble_gattc_write_char(gattc_if, conn_id, ir_code_char.char_handle, ir_btn_len, ir_btn_buf, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (write_success_status != ESP_OK)
            {
                ESP_LOGE(GATTC_TAG,
                        "Failed to initiate write to IR config characteristic (handle: 0x%04x). "
                        "esp_ble_gattc_write_char() returned 0x%x (%s). Possible causes: invalid connection, "
                        "write not permitted, or GATT busy.",
                        ir_config_char.char_handle,
                        write_success_status,
                        esp_err_to_name(write_success_status));

                ir_write_result = ESP_FAIL;
                // Reset the in-progress flag before returning
                if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY))
                {
                    ir_write_in_progress = false;
                    xSemaphoreGive(ir_write_in_progress_mtx);
                }
                else
                {
                    ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex while resetting ir_write_in_progress");
                    ir_write_in_progress = false;
                }
            }

            break;
        }

        break;
    }

    default:
        break;
    }
}




/* Return a pointer to the file-local static esp_gap_cb() */
esp_gap_ble_cb_t get_ble_gap_callback()
{
    return esp_gap_cb;        /* '&' is optional for function names */
}

/* Return a pointer to the file-local static esp_gattc_cb() */
esp_gattc_cb_t get_ble_gattc_callback()
{
    return esp_gattc_cb;
}

esp_err_t set_target_device_addr(esp_bd_addr_t target_addr)
{
    memcpy(target_device_addr, target_addr, sizeof(esp_bd_addr_t));

    esp_err_t err = esp_ble_gap_update_whitelist(true, target_device_addr, BLE_ADDR_TYPE_PUBLIC);
    if (err != ESP_OK) {
        ESP_LOGE("BLE", "Failed to add device to allow list: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t handle_connection()
{
    if (ir_write_in_progress_mtx == NULL) {
        ir_write_in_progress_mtx = xSemaphoreCreateMutex();
        if (ir_write_in_progress_mtx == NULL) {
            ESP_LOGE(TASK_SYNC_TAG, "Failed to create mutex");
        }
    }

    esp_err_t ret = esp_ble_gap_start_scanning(0);
    if (ret != ESP_OK) {
        ESP_LOGE("BLE_SCAN", "Failed to start scanning: %s", esp_err_to_name(ret));
    }

    return ret;
}


static esp_err_t set_ir_code(uint8_t **buffer, uint8_t *size_out, const uint8_t *value, uint8_t size)
{
    if (value == NULL || size == 0) {
        ESP_LOGE(IR_CODE_SET_TAG, "set_ir_code: Invalid argument (value=NULL or size=0)");
        return ESP_ERR_INVALID_ARG;
    }

    if (*buffer != NULL) {
        free(*buffer);
        *buffer = NULL;
        *size_out = 0;
    }

    *buffer = malloc(size);
    if (*buffer == NULL) {
        ESP_LOGE(IR_CODE_SET_TAG, "set_ir_code: Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    memcpy(*buffer, value, size);
    *size_out = size;

    return ESP_OK;
}

esp_err_t set_vol_up_ir_code(const uint8_t *value, uint8_t size)
{
    return set_ir_code(&vol_up_val, &vol_up_val_sz, value, size);
}

esp_err_t set_vol_down_ir_code(const uint8_t *value, uint8_t size)
{
    return set_ir_code(&vol_down_val, &vol_down_val_sz, value, size);
}

esp_err_t set_pow_ir_code(const uint8_t *value, uint8_t size)
{
    return set_ir_code(&pow_val, &pow_val_sz, value, size);
}

esp_err_t set_mute_ir_code(const uint8_t *value, uint8_t size)
{
    return set_ir_code(&mute_val, &mute_val_sz, value, size);
}

esp_err_t set_input_ir_code(const uint8_t *value, uint8_t size)
{
    return set_ir_code(&input_val, &input_val_sz, value, size);
}


// This is a blocking call
esp_err_t enable_ir_buttons()
{
    if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
        if (ir_write_in_progress) {
            ESP_LOGE(TASK_SYNC_TAG, "IR write already in progress");
            xSemaphoreGive(ir_write_in_progress_mtx);
            return ESP_ERR_INVALID_STATE;  // or a custom error if desired
        }

        ir_write_in_progress = true;
        xSemaphoreGive(ir_write_in_progress_mtx);
    } else {
        ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex in enable_ir_buttons()");
        return ESP_FAIL;
    }

    ir_btn_to_write_next = IR_BUTTON_VOL_UP;
    ir_write_result = ESP_OK;
    esp_err_t write_success_status = esp_ble_gattc_write_char(gl_gattc_if, conn_id, ir_config_char.char_handle, sizeof(IR_CONFIG_ENABLE_VAL), &IR_CONFIG_ENABLE_VAL, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (write_success_status != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG,
            "Failed to initiate write to IR config characteristic (handle: 0x%04x). "
            "esp_ble_gattc_write_char() returned 0x%x (%s). Possible causes: invalid connection, "
            "write not permitted, or GATT busy.",
            ir_config_char.char_handle,
            write_success_status,
            esp_err_to_name(write_success_status));

        // Reset the in-progress flag before returning
        if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
            ir_write_in_progress = false;
            xSemaphoreGive(ir_write_in_progress_mtx);
        } else {
            ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex while resetting ir_write_in_progress");
            ir_write_in_progress = false;
        }

        return write_success_status;
    }

    while (true) {
        if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
            if (!ir_write_in_progress) {
                xSemaphoreGive(ir_write_in_progress_mtx);
                break;  // Done waiting
            }
            xSemaphoreGive(ir_write_in_progress_mtx);
        }

        // Wait a bit before trying again to avoid hogging the CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ir_write_result;
}




// This is a blocking call
esp_err_t disable_ir_buttons()
{
    if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
        if (ir_write_in_progress) {
            ESP_LOGE(TASK_SYNC_TAG, "IR write already in progress");
            xSemaphoreGive(ir_write_in_progress_mtx);
            return ESP_ERR_INVALID_STATE;  // or a custom error if desired
        }

        ir_write_in_progress = true;
        xSemaphoreGive(ir_write_in_progress_mtx);
    } else {
        ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex in enable_ir_buttons()");
        return ESP_FAIL;
    }

    ir_btn_to_write_next = IR_BUTTON_FINISH_WRITING;
    ir_write_result = ESP_OK;
    esp_err_t write_success_status = esp_ble_gattc_write_char(gl_gattc_if, conn_id, ir_config_char.char_handle, sizeof(IR_CONFIG_ENABLE_VAL), &IR_CONFIG_ENABLE_VAL, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (write_success_status != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG,
            "Failed to initiate write to IR config characteristic (handle: 0x%04x). "
            "esp_ble_gattc_write_char() returned 0x%x (%s). Possible causes: invalid connection, "
            "write not permitted, or GATT busy.",
            ir_config_char.char_handle,
            write_success_status,
            esp_err_to_name(write_success_status));

        // Reset the in-progress flag before returning
        if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
            ir_write_in_progress = false;
            xSemaphoreGive(ir_write_in_progress_mtx);
        } else {
            ESP_LOGE(TASK_SYNC_TAG, "Failed to acquire mutex while resetting ir_write_in_progress");
            ir_write_in_progress = false;
        }

        return write_success_status;
    }

    while (true) {
        if (xSemaphoreTake(ir_write_in_progress_mtx, portMAX_DELAY)) {
            if (!ir_write_in_progress) {
                xSemaphoreGive(ir_write_in_progress_mtx);
                break;  // Done waiting
            }
            xSemaphoreGive(ir_write_in_progress_mtx);
        }

        // Wait a bit before trying again to avoid hogging the CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ir_write_result;
}