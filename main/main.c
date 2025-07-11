#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"

#include "ble_stack_manager.h"
#include "ble_connection.h"
#include "usb_hid_keyboard.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "esp_timer.h"


#define REMOTE_RELEASE_KEY 0x00

#define BOOT_BUTTON_GPIO GPIO_NUM_0
#define NVS_NAMESPACE_NAME "app_storage"
#define NVS_KEY "switch"
#define NVS_TAG "NVS"

#define BUTTON_TAG "BOOT_BUTTON"
#define BUTTON_PRESS_INTERVAL_US 2000000

#define HID_KEY_PAUSE_BREAK 0x48




static const esp_bd_addr_t target_device_addr = {0xE4, 0xE1, 0x12, 0xDB, 0x65, 0x5F};

static const uint8_t VOL_UP_VAL[] = {
    0x04, 0x21, 0x01, 0x68, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x0c, 0xcf, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x0c, 0xcf
};

static const uint8_t VOL_DOWN_VAL[] = {
    0x04, 0x21, 0x01, 0x68, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x21, 0x0c, 0xb0, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x21, 0x0c, 0xb0
};

static const uint8_t POW_VAL[] = {
    0x04, 0x21, 0x01, 0x68, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x21, 0x00, 0x20, 0x00, 0x41, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x3f, 0x00, 0x21, 0x00, 0x20, 0x00, 0x41, 0x00, 0x20, 0x00, 0x21, 0x0c, 0xc8, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x41, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x20, 0x00, 0x21, 0x00, 0x3f, 0x00, 0x21, 0x00, 0x20, 0x00, 0x41, 0x00, 0x20, 0x00, 0x21, 0x0c, 0xc8
};

static const uint8_t MUTE_VAL[] = {
    0x04, 0x21, 0x01, 0x68, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x3e, 0x00, 0x21, 0x0c, 0xb0, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x21, 0x00, 0x1f, 0x00, 0x41, 0x00, 0x3e, 0x00, 0x21, 0x0c, 0xb0
};

static const uint8_t INPUT_VAL[] = {
    0x04, 0x21, 0x01, 0x68, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x40, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x3e, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x40, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x0c, 0xed, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x40, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x3e, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x40, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1f, 0x00, 0x20, 0x0c, 0xed
};

static const hid_report_payload_t remote_map_windows_hid[18] = {
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD & HID_ITF_PROTOCOL_CONSUMER, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},         // pos 00: release for keyboard
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_A, 0, 0, 0, 0, 0}, .length = 8},                             // pos 01: power
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},                                     // pos 02: release for consumer
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ARROW_UP, 0, 0, 0, 0, 0}, .length = 8},                      // pos 03: up
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ARROW_DOWN, 0, 0, 0, 0, 0}, .length = 8},                    // pos 04: down
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ARROW_LEFT, 0, 0, 0, 0, 0}, .length = 8},                    // pos 05: left
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ARROW_RIGHT, 0, 0, 0, 0, 0}, .length = 8},                   // pos 06: right
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ENTER, 0, 0, 0, 0, 0}, .length = 8},                         // pos 07: enter
    {.report_id = HID_ITF_PROTOCOL_CONSUMER, .keycode = {HID_USAGE_CONSUMER_MUTE, 0, 0, 0, 0, 0, 0, 0}, .length = 2},               // pos 08: mute
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},                                     // pos 09: 
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_GUI_LEFT, 0, 0, 0, 0, 0}, .length = 8},                      // pos 10: home
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_ESCAPE, 0, 0, 0, 0, 0}, .length = 8},                        // pos 11: back
    {.report_id = HID_ITF_PROTOCOL_CONSUMER, .keycode = {HID_USAGE_CONSUMER_VOLUME_INCREMENT, 0, 0, 0, 0, 0, 0, 0}, .length = 2},   // pos 12: vol_up
    {.report_id = HID_ITF_PROTOCOL_CONSUMER, .keycode = {HID_USAGE_CONSUMER_VOLUME_DECREMENT, 0, 0, 0, 0, 0, 0, 0}, .length = 2},   // pos 13: vol_down
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_PAUSE_BREAK, 0, 0, 0, 0, 0}, .length = 8},                   // pos 14: YouTube
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_F3, 0, 0, 0, 0, 0}, .length = 8},                            // pos 15: Netflix
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},                                     // pos 16: 
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_F, 0, 0, 0, 0, 0}, .length = 8},                             // pos 17: input
};

static QueueHandle_t release_button_queue = NULL;
static TaskHandle_t button_task_handle = NULL;


void remote_button_cb(uint8_t value)
{
    if (value == REMOTE_RELEASE_KEY)
    {
        hid_report_payload_t btn_report_copy;
        xQueueReceive(release_button_queue, &btn_report_copy, 0);
        send_hid_report(&btn_report_copy);
    }
    else
    {
        hid_report_payload_t btn_report_copy = remote_map_windows_hid[value];
        send_hid_report(&btn_report_copy);

        btn_report_copy.keycode[0] = 0, btn_report_copy.keycode[2] = 0;
        xQueueSend(release_button_queue, &btn_report_copy, 0);
    }
}


static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


// Monitoring task that only creates new tasks on press
void button_monitor_task(void* arg) {
    button_task_handle = xTaskGetCurrentTaskHandle();
    if (button_task_handle == NULL) {
        ESP_LOGE(BUTTON_TAG, "Failed to get current task handle!");
        return;
    }

    nvs_handle_t hndl;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &hndl);
    if (ret != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle!");
    }

    int8_t stored_value = 0;
    ret = nvs_get_i8(hndl, NVS_KEY, &stored_value);
    switch (ret) {
        case ESP_OK:
            // ESP_LOGI(NVS_TAG, "Stored value: %d", stored_value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(NVS_TAG, "Value not set yet");
            break;
        default:
            ESP_LOGE(NVS_TAG, "Error reading: %s", esp_err_to_name(ret));
    }

    int64_t last_processed_time = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

        int64_t now = esp_timer_get_time();  // Get current time in microseconds
        if (now - last_processed_time < BUTTON_PRESS_INTERVAL_US) {
            ESP_LOGI(BUTTON_TAG, "Signal ignored: too soon after previous.");
            continue;
        }

        if (stored_value) {
            if (enable_ir_buttons() != ESP_OK)
                ESP_LOGE(BUTTON_TAG, "Failed to enable IR buttons: %s", esp_err_to_name(ret));
        } else {
            if (disable_ir_buttons() != ESP_OK)
                ESP_LOGE(BUTTON_TAG, "Failed to disable IR buttons: %s", esp_err_to_name(ret));
        }
            
        stored_value = !stored_value;
        ret = nvs_set_i8(hndl, NVS_KEY, stored_value);
        if (ret != ESP_OK) 
            ESP_LOGE(NVS_TAG, "Failed to set value in NVS! Error: %s", esp_err_to_name(ret));

        last_processed_time = now;
    }
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

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    
    nvs_handle_t nvs_switch_handle;

    ret = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &nvs_switch_handle);
    ESP_ERROR_CHECK(ret);

    release_button_queue = xQueueCreate(10, sizeof(hid_report_payload_t));
    if (release_button_queue == NULL) {
        ESP_LOGE(TINY_USB_TAG, "Release button queue was not created successfully");
        return;
    }

    ble_stack_init();
    usb_hid_kbd_init();

    register_ble_button_callback(remote_button_cb);
    set_target_device_addr(target_device_addr);

    set_vol_up_ir_code(VOL_UP_VAL, sizeof(VOL_UP_VAL));
    set_vol_down_ir_code(VOL_DOWN_VAL, sizeof(VOL_DOWN_VAL));
    set_pow_ir_code(POW_VAL, sizeof(POW_VAL));
    set_mute_ir_code(MUTE_VAL, sizeof(MUTE_VAL));
    set_input_ir_code(INPUT_VAL, sizeof(INPUT_VAL));

    register_ble_callbacks(get_ble_gap_callback(), get_ble_gattc_callback());

    // Start scanning
    handle_connection();


    xTaskCreate(button_monitor_task, "button_monitor_task", 6144, NULL, 5, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BOOT_BUTTON_GPIO, boot_button_isr_handler, (void*) BOOT_BUTTON_GPIO));
}