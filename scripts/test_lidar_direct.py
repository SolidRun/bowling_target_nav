#!/usr/bin/env python3
"""Direct RPLidar communication test."""

import serial
import time
import sys

port = '/dev/ttyUSB0'
if len(sys.argv) > 1:
    port = sys.argv[1]

print(f"Testing RPLidar on {port}")
print("=" * 40)

for baud in [115200, 256000]:
    print(f"\nTesting at {baud} baud...")
    try:
        s = serial.Serial(port, baud, timeout=2)
        s.reset_input_buffer()
        s.reset_output_buffer()

        # Send stop command first
        s.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        s.reset_input_buffer()

        # Send get_info command (0xA5 0x50)
        s.write(bytes([0xA5, 0x50]))
        time.sleep(0.5)

        response = s.read(100)
        if response:
            print(f"  Response ({len(response)} bytes): {response.hex()}")
            if len(response) >= 7:
                print(f"  SUCCESS! LiDAR responds at {baud} baud")
        else:
            print("  No response - LiDAR not responding")

        # Check health
        s.reset_input_buffer()
        s.write(bytes([0xA5, 0x52]))  # Get Health
        time.sleep(0.3)
        health = s.read(50)
        if health:
            print(f"  Health response: {health.hex()}")

        s.close()
    except Exception as e:
        print(f"  Error: {e}")

print("\n" + "=" * 40)
print("If no response at any baud rate:")
print("  1. Check if LiDAR motor is spinning")
print("  2. Check if LiDAR LED is on")
print("  3. Try unplugging and replugging the USB cable")
print("  4. The LiDAR may be faulty")
