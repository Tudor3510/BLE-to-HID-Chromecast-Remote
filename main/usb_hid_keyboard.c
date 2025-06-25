#include "usb_hid_keyboard.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

static const char* hid_string_descriptor[] = {
    (char[]){0x09, 0x04},                       // 0: English (0x0409)
    "TinyUSB",                                  // 1: Manufacturer
    "TinyUSB Device",                           // 2: Product
    "DCDA0C29C9FC",                             // 3: Serial Number
    "Chromecast Remote HID interface",          // 4: HID
};

static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_ITF_PROTOCOL_CONSUMER))
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

void usb_hid_kbd_init(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
}

bool send_hid_report(hid_report_payload_t* btn_report)
{
    while (!tud_hid_ready()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (!tud_hid_report(btn_report->report_id, btn_report->keycode, btn_report->length)) {
        printf("Failed to send HID report: report_id=%u, length=%u, keycodes=[",
            btn_report->report_id, btn_report->length);
        for (int i = 0; i < 8; ++i) {
            printf("%02X", btn_report->keycode[i]);
            if (i < 7) printf(" ");
        }
        printf("]\n");
        return false;
    }

    return true;
}
