Payload:
cd ~/payload/code/payload_code/main_code/PlatformIO/Projects/PAYLOAD
~/.platformio/penv/bin/pio device monitor -e Stack_with_LoRa --dtr 1 --rts 1 \
    -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_B8:F8:62:4C:5C:64-if00

Ground Station:
cd ~/payload/code/payload_code/main_code/PlatformIO/Projects/PAYLOAD
~/.platformio/penv/bin/python tools/gs_capture.py \
    /dev/serial/by-id/usb-Arduino_LLC_Arduino_NANO_33_IoT_5177072C50304D484D2E3120FF131B20-if00 \
    3600 capture.bin --decode /tmp/gs_decode_sim

