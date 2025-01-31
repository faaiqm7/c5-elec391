import customtkinter
import asyncio

customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("dark-blue")

root = customtkinter.CTk()
root.geometry("1366x768")

# Global speed variables
forward_speed = 0 
left_speed = 0
right_speed = 0
backward_speed = 0
speed_increment = 2  # Increase by 9 every 50ms (to reach 100 in ~500ms)
speed_decrement = 2  # Decrease by 10 every 50ms
key_pressed = set()  # Set to track which keys are currently pressed

async def moveRobot(dir, forward_speed, left_speed, right_speed, backward_speed):
    """Simulate moving the robot, printing the current speeds."""
    print(f"Moving {dir} at F:{forward_speed} B:{backward_speed} L:{left_speed} R:{right_speed}")
    await asyncio.sleep(0.05)  # Simulate a small delay for smooth updates

async def increase_speed(button, dir, key):
    """Gradually increases speed up to 100 while the button is held."""
    global forward_speed, left_speed, right_speed, backward_speed
    while key in key_pressed:
        if dir == "Forward" and forward_speed < 100:
            forward_speed += speed_increment
        elif dir == "Backward" and backward_speed < 100:
            backward_speed += speed_increment
        elif dir == "Left" and left_speed < 100:
            left_speed += speed_increment
        elif dir == "Right" and right_speed < 100:
            right_speed += speed_increment
        
        # Call moveRobot coroutine to update movement
        await moveRobot(dir, forward_speed, left_speed, right_speed, backward_speed)
        await asyncio.sleep(0.1)  # Wait before updating again

async def decrease_speed(button, key):
    """Gradually decreases speed to 0 when no movement keys are pressed."""
    global forward_speed, left_speed, right_speed, backward_speed
    while key not in key_pressed:
        if key == "w" and forward_speed > 0:
            forward_speed -= speed_increment
        elif key == "s" and backward_speed > 0:
            backward_speed -= speed_increment
        elif key == "a" and left_speed > 0:
            left_speed -= speed_increment
        elif key == "d" and right_speed > 0:
            right_speed -= speed_increment
        
        # Call moveRobot coroutine to update movement
        await moveRobot("Stopping", forward_speed, left_speed, right_speed, backward_speed)
        await asyncio.sleep(0.05)  # Wait before updating again

async def handle_key_press(event, button, dir, key):
    """Handles the key press event, starting an asyncio task for speed increase."""
    key_pressed.add(key)  # Mark key as pressed
    button.configure(fg_color="#9C0000")  # Change button color immediately
    await increase_speed(button, dir, key)

async def handle_key_release(event, button, key):
    """Handles the key release event, starting an asyncio task for speed decrease."""
    key_pressed.remove(key)  # Remove key from pressed set
    button.configure(fg_color="#CD0000")  # Reset color immediately when key is released
    await decrease_speed(button, key)

frame = customtkinter.CTkFrame(master=root)
frame.pack(pady=20, padx=20, fill="both", expand=True)

label = customtkinter.CTkLabel(master=frame, text="C5-Robot", font=("Roboto", 40))
label.place(relx=0.1, rely=0.1, anchor="center")

# Create Forward Button
forwardbutton = customtkinter.CTkButton(
    master=frame, 
    text="Forward",  
    width=60,
    fg_color="#CD0000",
    hover_color="#9C0000",
    text_color="white"
)
forwardbutton.place(relx=0.1, rely=0.475, anchor="center")

# Create Backward Button
backwardbutton = customtkinter.CTkButton(
    master=frame, 
    text="Back", 
    width=60,
    fg_color="#CD0000",
    hover_color="#9C0000",
    text_color="white"
)
backwardbutton.place(relx=0.1, rely=0.525, anchor="center")

# Create Left Button
leftbutton = customtkinter.CTkButton(
    master=frame, 
    text="Left", 
    width=60,
    fg_color="#CD0000",
    hover_color="#9C0000",
    text_color="white"
)
leftbutton.place(relx=0.05, rely=0.525, anchor="center")

# Create Right Button
rightbutton = customtkinter.CTkButton(
    master=frame, 
    text="Right", 
    width=60,
    fg_color="#CD0000",
    hover_color="#9C0000",
    text_color="white"
)
rightbutton.place(relx=0.15, rely=0.525, anchor="center")

# Bind movement keys to increase speed
root.bind("<w>", lambda event: asyncio.create_task(handle_key_press(event, forwardbutton, "Forward", "w")))
root.bind("<a>", lambda event: asyncio.create_task(handle_key_press(event, leftbutton, "Left", "a")))
root.bind("<s>", lambda event: asyncio.create_task(handle_key_press(event, backwardbutton, "Backward", "s")))
root.bind("<d>", lambda event: asyncio.create_task(handle_key_press(event, rightbutton, "Right", "d")))

# Bind key release to check when keys are released
root.bind("<KeyRelease-w>", lambda event: asyncio.create_task(handle_key_release(event, forwardbutton, "w")))
root.bind("<KeyRelease-a>", lambda event: asyncio.create_task(handle_key_release(event, leftbutton, "a")))
root.bind("<KeyRelease-s>", lambda event: asyncio.create_task(handle_key_release(event, backwardbutton, "s")))
root.bind("<KeyRelease-d>", lambda event: asyncio.create_task(handle_key_release(event, rightbutton, "d")))

async def run_tkinter():
    """Run the tkinter main loop in the asyncio event loop."""
    while True:
        root.update_idletasks()  # Process all idle tasks
        root.update()  # Process all pending events
        await asyncio.sleep(0.01)  # Yield to the asyncio event loop

# Start asyncio event loop
async def start():
    asyncio.create_task(run_tkinter())  # Run tkinter main loop within asyncio
    await asyncio.sleep(100000)  # Keep running indefinitely

asyncio.run(start())
