#include <stdint.h>         // for uint8_t, uint16_t
#include <stdbool.h>        // for bool
#include "tinyusb.h"           // main TinyUSB header
#include "class/hid/hid.h"  // HID-specific definitions (e.g., HID usage codes)
#include "freertos/FreeRTOS.h"  // for QueueHandle_t
#include "freertos/queue.h"     // for xQueue-related functions
#include "esp_log.h"       // for logging (ESP-IDF specific)



#define TINY_USB_TAG "TINY_USB"


/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)
#define HID_ITF_PROTOCOL_CONSUMER 2




typedef struct {
    uint8_t report_id;
    uint8_t keycode[8];
    uint8_t length;
} hid_report_payload_t;


const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},                       // 0: is supported language is English (0x0409)
    "TinyUSB",                                  // 1: Manufacturer
    "TinyUSB Device",                           // 2: Product
    "DCDA0C29C9FC",                             // 3: Serials, should use chip ID
    "Chromecast Remote HID interface",          // 4: HID
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

void usb_hid_kbd_init()
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
    if (!tud_hid_report(btn_report->report_id, btn_report->keycode, btn_report->length)) {
        printf("Failed to send HID report: report_id=%u, length=%u\n",
            btn_report->report_id, btn_report->length);

        return false;
    }

    return true;
}
