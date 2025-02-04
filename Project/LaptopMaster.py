import asyncio
import tkinter
from bleak import BleakClient, BleakScanner

# Define the BLE UUIDs
laptop_master_service_uuid = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_uuid = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_send_characteristic_uuid = "00000002-5EC4-4083-81CD-A10B8D5CF6EC"

# The name of the Arduino device (same as what you set in the Arduino sketch)
arduino_device_name = "C5-BLE"  # Replace with the name you gave to your Arduino deviceclccc

# Function to interact with the Arduino
async def run():
    # Discover BLE devices
    devices = await BleakScanner.discover()

    # Search for the Arduino by name
    arduino_device = None
    for device in devices:
        if device.name == arduino_device_name:
            arduino_device = device
            break

    if not arduino_device:
        print(f"Could not find device with name: {arduino_device_name}")
        return

    print(f"Connecting to {arduino_device_name} ({arduino_device.address})")
    
    # Connect to the Arduino once
    async with BleakClient(arduino_device.address) as client:

        print(f"Connected to {arduino_device_name}\n")

        # Keep the connection open and listen for notifications
        while True:
                valueReceived = await client.read_gatt_char(laptop_master_characteristic_uuid)
                try:
                    valueReceived = valueReceived.decode('utf-8')
                    print(f"{valueReceived} Degrees")
                    test_str = str(8)
                    test_str_bytes = bytearray(test_str, encoding = "utf-8")
                    await client.write_gatt_char(laptop_master_send_characteristic_uuid, test_str_bytes, response=True)
                    await asyncio.sleep(0.001)  # Adjust the sleep time if needed
                except KeyboardInterrupt:
                    print("Disconnected from the Arduino")

# Run the asynchronous function
asyncio.run(run())
