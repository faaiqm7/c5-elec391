import asyncio
import customtkinter as ctk
from bleak import BleakClient, BleakScanner
import threading
from pynput import keyboard
import time

# Define the BLE UUIDs
laptop_master_service_uuid = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_uuid = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_send_characteristic_uuid = "00000002-5EC4-4083-81CD-A10B8D5CF6EC"

# The name of the Arduino device
arduino_device_name = "C5-BLE"  # Replace with your Arduino device name

global client, ble_loop
client = None
ble_loop = None

# Speed variables
Forward_Speed = 0
Back_Speed = 0
Left_Speed = 0
Right_Speed = 0

# Throttle BLE messages
last_sent_time = 0
send_interval = 0.1  # Minimum time between BLE sends (100ms)

def send_ble_command():
    global last_sent_time
    current_time = time.time()
    if current_time - last_sent_time >= send_interval:
        last_sent_time = current_time
        if client and ble_loop:
            command = f"L:{Left_Speed} R:{Right_Speed} F:{Forward_Speed} B:{Back_Speed}"
            test_str_bytes = bytearray(command, encoding="utf-8")
            asyncio.run_coroutine_threadsafe(
                client.write_gatt_char(laptop_master_send_characteristic_uuid, test_str_bytes, response=True),
                ble_loop
            )
            print(f"{command}")

def on_press(key):
    global Forward_Speed, Back_Speed, Left_Speed, Right_Speed
    try:
        if key.char == 'w':
            Forward_Speed += 1
        elif key.char == 's':
            Back_Speed += 1
        elif key.char == 'a':
            Left_Speed += 1
        elif key.char == 'd':
            Right_Speed += 1
        send_ble_command()
    except AttributeError:
        pass

def on_release(key):
    global Forward_Speed, Back_Speed, Left_Speed, Right_Speed
    try:
        if key.char == 'w':
            Forward_Speed = 0
        elif key.char == 's':
            Back_Speed = 0
        elif key.char == 'a':
            Left_Speed = 0
        elif key.char == 'd':
            Right_Speed = 0
        send_ble_command()
    except AttributeError:
        pass

def on_button_click(direction):
    global Forward_Speed, Back_Speed, Left_Speed, Right_Speed
    if direction == "Forward":
        Forward_Speed += 1
    elif direction == "Back":
        Back_Speed += 1
    elif direction == "Left":
        Left_Speed += 1
    elif direction == "Right":
        Right_Speed += 1
    send_ble_command()

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
                await asyncio.sleep(0.01)
            except Exception as e:
                print(f"Error: {e}")
                break

# Run BLE communication in a separate thread
def start_ble_loop():
    asyncio.run(run_ble())

ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
ble_thread.start()

# Start listening to keyboard inputs
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Create the CustomTkinter GUI
ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")

root = ctk.CTk()
root.geometry("1366x768")
root.title("C5 Balancing Robot Controller")

button_forward = ctk.CTkButton(root, 
                               text="Forward", 
                               width=60, 
                               fg_color="#CD0000", 
                               hover_color="#9C0000", 
                               text_color="white", 
                               command=lambda: on_button_click("Forward"))
button_forward.place(relx=0.1, rely=0.575, anchor="center")

button_back = ctk.CTkButton(root, 
                            text="Back", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("Back"))
button_back.place(relx=0.1, rely=0.625, anchor="center")

button_left = ctk.CTkButton(root, 
                            text="Left", 
                            width=60, 
                            fg_color="#CD0000", 
                            hover_color="#9C0000", 
                            text_color="white", 
                            command=lambda: on_button_click("Left"))
button_left.place(relx=0.05, rely=0.625, anchor="center")

button_right = ctk.CTkButton(root, 
                             text="Right", 
                             width=60, 
                             fg_color="#CD0000", 
                             hover_color="#9C0000", 
                             text_color="white", 
                             command=lambda: on_button_click("Right"))
button_right.place(relx=0.15, rely=0.625, anchor="center")

label = ctk.CTkLabel(root, text="C5-Robot", font=("Roboto", 40))
label.place(relx=0.1, rely=0.1, anchor="center")

root.mainloop()
