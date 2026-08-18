# test_serial_companion.py
import serial
import time
import sys

def test_serial_communication():
    try:
        # Open the serial port your MCU is connected to
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # Wait for connection

        # Listen for test commands
        while True:
            if ser.in_waiting:
                data = ser.readline().decode().strip()
                print(f"Received: {data}")

                # Echo back or send expected response
                if data == "PING":
                    ser.write(b"PONG\n")
                elif data == "GET_STATUS":
                    ser.write(b"OK\n")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    test_serial_communication()