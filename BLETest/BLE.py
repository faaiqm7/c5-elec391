import asyncio
from bleak import BleakScanner, BleakClient

# Replace with your Arduino's BLE name and characteristic UUIDs
TARGET_DEVICE_NAME = "Nano 33 BLE (Central)"
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHARACTERISTIC_UUID = "abcdef12-3456-789a-bcde-f0123456789a"

async def connect_to_arduino():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    arduino_device = None
    for device in devices:
        print(f"Found device: {device.name} - {device.address}")
        if device.name == TARGET_DEVICE_NAME:
            arduino_device = device
            break

    if not arduino_device:
        print(f"Device with name '{TARGET_DEVICE_NAME}' not found. Exiting.")
        return

    print(f"Found target device: {arduino_device.name} - {arduino_device.address}")

    async with BleakClient(arduino_device.address) as client:
        print(f"Connected to {arduino_device.name}!")

        # Check if the service and characteristic exist
        services = await client.get_services()
        print("Services and characteristics:")
        for service in services:
            print(f"[Service] {service.uuid}")
            for char in service.characteristics:
                print(f"  [Characteristic] {char.uuid} (Properties: {char.properties})")

        if CHARACTERISTIC_UUID in [char.uuid for char in services.get_service(SERVICE_UUID).characteristics]:
            print(f"Reading from characteristic {CHARACTERISTIC_UUID}...")
            value = await client.read_gatt_char(CHARACTERISTIC_UUID)
            print(f"Value read: {value}")

            print(f"Writing to characteristic {CHARACTERISTIC_UUID}...")
            await client.write_gatt_char(CHARACTERISTIC_UUID, b"Hello from Python!")
            print("Write successful.")
        else:
            print(f"Characteristic {CHARACTERISTIC_UUID} not found.")

if __name__ == "__main__":
    asyncio.run(connect_to_arduino())
