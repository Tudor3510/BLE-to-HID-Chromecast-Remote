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
#include "esp_bt_defs.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"


#include "ble_stack_manager.h"
#include "ble_connection.h"


#define TINY_USB_TAG "TINY_USB"

#define REMOTE_RELEASE_KEY 0x00


/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)
#define HID_ITF_PROTOCOL_CONSUMER 2



typedef struct {
    uint8_t report_id;
    uint8_t keycode[8];
    uint8_t length;
} hid_report_mapper;

static hid_report_mapper remote_map_windows_hid[18] = {
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD & HID_ITF_PROTOCOL_CONSUMER, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},         // pos 00: 
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_A, 0, 0, 0, 0, 0}, .length = 8},                             // pos 01: power
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},                                     // pos 02: 
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
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_S, 0, 0, 0, 0, 0}, .length = 8},                             // pos 14: YouTube
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_D, 0, 0, 0, 0, 0}, .length = 8},                             // pos 15: Netflix
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, 0, 0, 0, 0, 0, 0}, .length = 8},                                     // pos 16: 
    {.report_id = HID_ITF_PROTOCOL_KEYBOARD, .keycode = {0, 0, HID_KEY_F, 0, 0, 0, 0, 0}, .length = 8},                             // pos 17: input
};


const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_ITF_PROTOCOL_CONSUMER))
};

static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};


static QueueHandle_t release_button_queue;




uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
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


    ESP_LOGI(TINY_USB_TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TINY_USB_TAG, "USB initialization DONE");

    release_button_queue = xQueueCreate(10, sizeof(hid_report_mapper));
    if (release_button_queue == NULL) {
        ESP_LOGE(TINY_USB_TAG, "Release button queue was not created successfully");
        return;
    }

    ble_stack_init();

    ret = esp_ble_gap_update_whitelist(true, target_device_addr, BLE_ADDR_TYPE_PUBLIC);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device added to whitelist successfully");
    } else {
        ESP_LOGE(TAG, "Failed to add device to whitelist: 0x%x", ret);
    }

    ret = esp_ble_gap_update_whitelist(true, target_device_addr, BLE_ADDR_TYPE_PUBLIC);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device added to whitelist successfully");
    } else {
        ESP_LOGE(TAG, "Failed to add device to whitelist: 0x%x", ret);
    }

    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t dev_list[dev_num];
    esp_ble_get_bond_device_list(&dev_num, dev_list);

    for (int i = 0; i < dev_num; i++) {
        ESP_LOGI("BOND", "Bonded device %d: %02x:%02x:%02x:%02x:%02x:%02x",
            i,
            dev_list[i].bd_addr[0], dev_list[i].bd_addr[1], dev_list[i].bd_addr[2],
            dev_list[i].bd_addr[3], dev_list[i].bd_addr[4], dev_list[i].bd_addr[5]);
    }
    
    register_ble_callbacks(esp_gap_cb, esp_gattc_cb);

    // Start scanning
    esp_ble_gap_start_scanning(0);

}
