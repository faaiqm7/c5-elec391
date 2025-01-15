import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Set up the serial connection
serial_port = "COM3"
baud_rate = 9600  # Match the Arduino baud rate
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Initialize lists to store x, y, z data
x_data, y_data, z_data = [], [], []
time_data = []

# Create the plot figure
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

# Create line objects for each axis
line1, = ax1.plot([], [], label="X-axis", color="blue")
line2, = ax2.plot([], [], label="Y-axis", color="green")
line3, = ax3.plot([], [], label="Z-axis", color="red")

# Add live reading text objects
x_text = ax1.text(0.02, 0.95, "", transform=ax1.transAxes, fontsize=10, verticalalignment='top')
y_text = ax2.text(0.02, 0.95, "", transform=ax2.transAxes, fontsize=10, verticalalignment='top')
z_text = ax3.text(0.02, 0.95, "", transform=ax3.transAxes, fontsize=10, verticalalignment='top')

# Initialize axes
ax1.set_title("X-axis Gyroscope Data")
ax1.set_xlabel("Time")
ax1.set_ylabel("Value (deg/s)")
ax1.legend()

ax2.set_title("Y-axis Gyroscope Data")
ax2.set_xlabel("Time")
ax2.set_ylabel("Value (deg/s)")
ax2.legend()

ax3.set_title("Z-axis Gyroscope Data")
ax3.set_xlabel("Time")
ax3.set_ylabel("Value (deg/s)")
ax3.legend()

def init():
    """Initialize the plot lines."""
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    return line1, line2, line3

def read_serial_data():
    """Read and parse data from the serial port."""
    try:
        if ser.in_waiting > 0:
            # Flush old data to avoid buffer delay
            ser.flushInput()
            line = ser.readline().decode('utf-8').strip()
            values = line.split(',')
            if len(values) == 3:
                return [float(v) for v in values]
    except Exception as e:
        print(f"Error reading serial data: {e}")
    return None

def update(frame):
    """Update the plot with new data."""
    data = read_serial_data()
    if data:
        time_data.append(len(time_data))  # Increment time index
        x_data.append(data[0])
        y_data.append(data[1])
        z_data.append(data[2])

        # Update the data for each line
        line1.set_data(time_data, x_data)
        line2.set_data(time_data, y_data)
        line3.set_data(time_data, z_data)

        # Adjust x-axis limits dynamically
        ax1.set_xlim(max(0, len(time_data) - 100), len(time_data))
        ax2.set_xlim(max(0, len(time_data) - 100), len(time_data))
        ax3.set_xlim(max(0, len(time_data) - 100), len(time_data))

        # Update the live data text
        x_text.set_text(f"X: {data[0]:.2f} deg/s")
        y_text.set_text(f"Y: {data[1]:.2f} deg/s")
        z_text.set_text(f"Z: {data[2]:.2f} deg/s")

        # Adjust y-axis limits dynamically
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax3.relim()
        ax3.autoscale_view()

    return line1, line2, line3, x_text, y_text, z_text

# Set up the animation
ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=10)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    print("Exiting...")

# Close the serial connection when done
ser.close()
