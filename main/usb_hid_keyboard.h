#ifndef USB_HID_KEYBOARD_H
#define USB_HID_KEYBOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "tinyusb.h"
#include "class/hid/hid.h"


#define TUSB_DESC_TOTAL_LEN            (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)
#define HID_ITF_PROTOCOL_CONSUMER      2
#define TINY_USB_TAG                   "TINY_USB"

// Payload for sending HID reports
typedef struct {
    uint8_t report_id;
    uint8_t keycode[8];
    uint8_t length;
} hid_report_payload_t;

// Initialize the HID USB driver
void usb_hid_kbd_init(void);

// Send a HID report to the host
bool send_hid_report(hid_report_payload_t* btn_report);


#endif // USB_HID_KEYBOARD_H
