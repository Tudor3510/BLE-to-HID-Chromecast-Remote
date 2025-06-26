# Firmware for an ESP32-S3-based dongle that bridges BLE input from the Chromecast Remote to USB HID output, enabling control of HID-compatible devices.

### ✅ Firmware Version

This firmware is compiled using **ESP-IDF v5.3.1**. Ensure you have this version (or a compatible one) installed when compiling.

---

### 🧲 Pairing the Chromecast Remote

To pair the Chromecast Remote with the dongle:

1. **Set the remote's BLE MAC address** in `main.c`.  
   Locate the line where the MAC address is defined and replace it with your remote's actual address.

2. **Plug the dongle into the host device** (e.g., PC or Raspberry Pi) and wait for it to boot.

3. **Put the remote into pairing mode** by holding **Home + Back** simultaneously until the LED on the remote becomes white.

> ⚠️ **Important:** The MAC address **must** be correctly set in `main.c` before attempting to pair.

---

### 📺 Using the IR Buttons with Your TV

To make the IR buttons on the Chromecast Remote control your TV:

1. Extract the IR payloads for your specific TV model.
2. Insert the payloads into the appropriate section in `main.c`.
3. Once added, the firmware will allow you to switch the IR buttons between:
   - Controlling the host device via **USB HID**
   - Controlling the TV via **infrared (IR)**

To toggle between IR and USB HID mode, **press the "BOOT" button** on the dongle.

---

### 🎮 Customizing Button Mappings (USB HID)

To change how the Chromecast Remote buttons behave on the host device:

- Edit the `remote_map_windows_hid` array in `main.c`.

This array maps Chromecast Remote buttons to standard USB HID keycodes. Modify it to customize what each button does.