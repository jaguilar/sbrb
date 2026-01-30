#!/usr/bin/env python3

import asyncio
from bleak import BleakScanner, BleakClient

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# These must match the UUIDs in your ESP32 NimBLE setup
# Using standard Nordic UART Service (NUS) UUIDs from your earlier example
UUID_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UUID_RX      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" # We write to this
DEVICE_NAME  = "SBRB" # The name you gave in NimBLEDevice::init()

async def run():
    print(f"Scanning for device named '{DEVICE_NAME}'...")
    
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME or (ad.local_name == DEVICE_NAME)
    )

    if not device:
        print(f"❌ Device '{DEVICE_NAME}' not found. Is it powered on?")
        return

    print(f"✅ Found {device.name} ({device.address}). Connecting...")

    async with BleakClient(device) as client:
        print(f"🔗 Connected!")
        print("-" * 50)
        print("COMMAND GUIDE:")
        print("  P<val>  -> Set Kp (e.g., 'P1.5')")
        print("  I<val>  -> Set Ki (e.g., 'I0.05')")
        print("  D<val>  -> Set Kd (e.g., 'D0.02')")
        print("  M<val>  -> Set Min PWM/Deadband (e.g., 'M35')")
        print("  X       -> EMERGENCY STOP (Sets P=0, M=0)")
        print("  Q       -> Quit")
        print("-" * 50)

        while True:
            # 1. Get input from user
            cmd = await asyncio.to_thread(input, "Enter Command > ")
            cmd = cmd.strip()

            if not cmd:
                continue

            if cmd == "D":
                val = await client.read_gatt_char(UUID_RX)
                print(f"   Current values: {val}")
                continue

            if cmd == "Q":
                print("Disconnecting...")
                break

            # 2. Safety Check for Emergency Stop
            if cmd == "X":
                print("🚨 SENDING EMERGENCY STOP!")
            
            # 3. Send data
            try:
                # Encode string to bytes (UTF-8)
                data = cmd.encode('utf-8')
                
                # Write to the characteristic
                # response=True ensures the ESP32 received it before we move on
                await client.write_gatt_char(UUID_RX, data, response=True)
                print(f"   Refreshed: {cmd}")
                
            except Exception as e:
                print(f"❌ Failed to send: {e}")
                break

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nExiting...")
