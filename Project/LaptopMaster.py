import asyncio
import customtkinter as ctk
from bleak import BleakClient, BleakScanner
import threading
from pynput import keyboard
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys
import io

# Define the BLE UUIDs
laptop_master_service_uuid = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_angle_uuid = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_send_characteristic_uuid = "00000002-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_coordinates_uuid = "00000003-5EC4-4083-81CD-A10B8D5CF6EC"
laptop_master_characteristic_distance_sensor_uuid = "00000004-5EC4-4083-81CD-A10B8D5CF6EC"

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

# Angle Data for Live Plotting
angles = []  # List to store received angles
time_stamps = []  # List to store time of each angle reading

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
            if(Forward_Speed >= 100):
                Forward_Speed = 100
        elif key.char == 's':
            Back_Speed += 1
            if(Back_Speed >= 100):
                Back_Speed = 100
        elif key.char == 'a':
            Left_Speed += 1
            if(Left_Speed >= 100):
                Left_Speed = 100
        elif key.char == 'd':
            Right_Speed += 1
            if(Right_Speed >= 100):
                Right_Speed = 100
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
    global client, ble_loop, angles, time_stamps
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
                coordinatesReceived = await client.read_gatt_char(laptop_master_characteristic_coordinates_uuid)
                angleReceived = await client.read_gatt_char(laptop_master_characteristic_angle_uuid)
                distanceSensed = await client.read_gatt_char(laptop_master_characteristic_distance_sensor_uuid)
                coordinatesReceived = coordinatesReceived.decode('utf-8')
                angleReceived = angleReceived.decode('utf-8')
                distanceSensed = distanceSensed.decode('utf-8')
                print(f"{angleReceived} Degrees at {coordinatesReceived} and distance sensed: {distanceSensed}")

                # Update angle data for plotting
                current_time = time.time()
                angles.append(float(angleReceived))
                time_stamps.append(current_time)

                # Limit the number of points in the plot to avoid memory issues
                if len(angles) > 100:
                    angles.pop(0)
                    time_stamps.pop(0)

                update_plot()

                # Update the angle label
                update_angle_label()

                await asyncio.sleep(0.01)
            except Exception as e:
                print(f"Error: {e}")
                break

# Create and update plot
def update_plot():
    global angles, time_stamps
    ax.clear()
    
    # Plot the angle with a red line
    ax.plot(time_stamps, angles, label="Angle (Degrees)", color="#DE0000")
    
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (°)")

    # Remove the scientific notation for the x-axis
    ax.get_xaxis().get_major_formatter().set_scientific(False)

    canvas.draw()

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

# Set the root window background to white
root.configure(bg="white")

# Add text above the angle plot
angleplot_label = ctk.CTkLabel(root, text="Robot Angle", font=("Roboto", 24))
angleplot_label.place(relx=0.64, rely=0.153, anchor="center")


# Create a frame for plotting the angle
plot_frame = ctk.CTkFrame(root, width=500, height=100)
plot_frame.place(relx=0.77, rely=0.4, anchor="center")

# Create a Matplotlib figure with a dark-red border
fig, ax = plt.subplots(figsize=(6.25, 4.5))

# Set the figure's border color to dark red
fig.patch.set_edgecolor('#9A0000')  # Dark red color
fig.patch.set_linewidth(3)  # Set border thickness

ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (°)")

# Embed Matplotlib plot into CustomTkinter window
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

# Add control buttons
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

# Create a terminal display with custom border color and width
terminal_frame = ctk.CTkFrame(root, width=500, height=200, border_color="#9A0000", border_width=2)
terminal_frame.place(relx=0.74, rely=0.8, anchor="center")

# Remove default border from the terminal text box by setting border width to 0
terminal = ctk.CTkTextbox(terminal_frame, width=480, height=180, wrap="word", state="disabled", font=("Courier", 12), border_width=0)
terminal.pack(padx=10, pady=10)

# Add text above the terminal
terminal_label = ctk.CTkLabel(root, text="Terminal", font=("Roboto", 16))
terminal_label.place(relx=0.628, rely=0.685, anchor="center")

# Redirect print to the terminal
class TerminalRedirector(io.StringIO):
    def __init__(self, textbox):
        super().__init__()
        self.textbox = textbox
    
    def write(self, text):
        self.textbox.configure(state="normal")
        self.textbox.insert("end", text)
        self.textbox.yview("end")  # Auto-scroll
        self.textbox.configure(state="disabled")

# Set up terminal redirection
sys.stdout = TerminalRedirector(terminal)

# Create a white background frame for the angle label
angle_frame = ctk.CTkFrame(root, width=160, height=50, fg_color="white", border_width=2, border_color="#9A0000")
angle_frame.place(relx=0.1, rely=0.45, anchor="center")

# Create the angle label inside the frame
angle_label = ctk.CTkLabel(angle_frame, text="Angle: 0°", font=("Roboto", 24))
angle_label.place(relx=0.5, rely=0.5, anchor="center")  # Center the label inside the frame

# Update the angle label with the latest value
def update_angle_label():
    global angles
    if angles:
        latest_angle = angles[-1]
        angle_label.configure(text=f"Angle: {latest_angle:.2f}°")

root.mainloop()
