import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner
import threading

# Define the BLE UUIDs
laptop_master_service_uuid = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_uuid = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_send_characteristic_uuid = "00000002-5EC4-4083-81CD-A10B8D5CF6EC"

# The name of the Arduino device
arduino_device_name = "C5-BLE"  # Replace with your Arduino device name

global client, ble_loop
client = None
ble_loop = None

def on_button_click():
    if client and ble_loop:
        test_str_bytes = bytearray(str(5), encoding="utf-8")
        asyncio.run_coroutine_threadsafe(client.write_gatt_char(laptop_master_send_characteristic_uuid, test_str_bytes, response=True), ble_loop)
        print("Sent 5")

async def run_ble():
    global client, ble_loop
    ble_loop = asyncio.get_running_loop()
    devices = await BleakScanner.discover()
    arduino_device = next((device for device in devices if device.name == arduino_device_name), None)
    
    if not arduino_device:
        print(f"Could not find device with name: {arduino_device_name}")
        return

    print(f"Connecting to {arduino_device_name} ({arduino_device.address})")
    async with BleakClient(arduino_device.address) as client:
        print(f"Connected to {arduino_device_name}\n")
        while True:
            try:
                valueReceived = await client.read_gatt_char(laptop_master_characteristic_uuid)
                valueReceived = valueReceived.decode('utf-8')
                print(f"{valueReceived} Degrees")
                await asyncio.sleep(0.001)
            except Exception as e:
                print(f"Error: {e}")
                break

# Run BLE communication in a separate thread
def start_ble_loop():
    asyncio.run(run_ble())

ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
ble_thread.start()

# Create the Tkinter GUI
root = tk.Tk()
root.title("BLE GUI")
button = tk.Button(root, text="Click Me", command=on_button_click)
button.pack(pady=20)
root.mainloop()
