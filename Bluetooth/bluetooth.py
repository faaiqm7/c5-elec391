import asyncio
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from bleak import BleakScanner, BleakClient

class BLEApp:
    def __init__(self, root):
        self.root = root
        self.root.title("BLE Client")
        self.client = None
        self.device_dict = {}  # Store device name-address mappings
        self.selected_device_address = None
        self.characteristic_uuid = None  # Set your writable characteristic UUID here

        self.create_widgets()
        self.scan_devices()

    def create_widgets(self):
        ttk.Label(self.root, text="Select a BLE Device:").pack(pady=5)
        
        self.device_combobox = ttk.Combobox(self.root, state="readonly")
        self.device_combobox.pack(pady=5)
        
        self.scan_button = ttk.Button(self.root, text="Scan", command=self.scan_devices)
        self.scan_button.pack(pady=5)
        
        self.connect_button = ttk.Button(self.root, text="Connect", command=self.connect_device)
        self.connect_button.pack(pady=5)
        
        ttk.Label(self.root, text="Send Data:").pack(pady=5)
        
        self.data_entry = ttk.Entry(self.root)
        self.data_entry.pack(pady=5)
        
        self.send_button = ttk.Button(self.root, text="Send", command=self.send_data, state="disabled")
        self.send_button.pack(pady=5)

    def scan_devices(self):
        def scan():
            devices = asyncio.run(self.async_scan())
            self.device_dict = {device.name: device.address for device in devices if device.name}  # Store name-address mapping
            
            if self.device_dict:
                self.device_combobox["values"] = list(self.device_dict.keys())  # Display only device names
                messagebox.showinfo("Scan Complete", "BLE devices found. Select one from the dropdown.")
            else:
                messagebox.showerror("Error", "No BLE devices found. Try again.")

        threading.Thread(target=scan, daemon=True).start()

    async def async_scan(self):
        devices = await BleakScanner.discover()
        return devices

    def connect_device(self):
        selected_name = self.device_combobox.get()
        
        if not selected_name or selected_name not in self.device_dict:
            messagebox.showerror("Error", "Please select a valid BLE device.")
            return

        # Retrieve correct address
        self.selected_device_address = self.device_dict[selected_name]

        print(f"Attempting to connect to {selected_name} at {self.selected_device_address}")
        
        # Use asyncio.create_task() to avoid blocking the Tkinter loop
        asyncio.create_task(self.async_connect(self.selected_device_address))

    async def async_connect(self, address):
        try:
            print(f"Connecting to BLE device at {address}...")
            self.client = BleakClient(address, timeout=10.0)
            await self.client.connect()
            
            if not self.client.is_connected:
                messagebox.showerror("Error", "Failed to connect to the BLE device.")
                return
            
            print(f"Successfully connected to {address}")
            
            services = await self.client.get_services()
            writable_chars = [char.uuid for service in services for char in service.characteristics if "write" in char.properties]

            if writable_chars:
                self.characteristic_uuid = writable_chars[0]
                self.send_button["state"] = "normal"
                messagebox.showinfo("Success", f"Connected to {address}.\nWritable Characteristic: {self.characteristic_uuid}")
            else:
                messagebox.showerror("Error", "No writable characteristic found.")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Error connecting to {address}: {str(e)}")

    def send_data(self):
        data = self.data_entry.get()
        if not data or not self.client or not self.characteristic_uuid:
            messagebox.showerror("Error", "Invalid data or device not connected.")
            return
        
        asyncio.create_task(self.async_send(data))

    async def async_send(self, data):
        try:
            await self.client.write_gatt_char(self.characteristic_uuid, data.encode(), response=True)
            messagebox.showinfo("Success", "Data sent successfully!")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

def main():
    root = tk.Tk()
    app = BLEApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
