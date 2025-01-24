import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

"""
Name: Task 3 Python Script
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis

"""

class GyroAccumulatedAnglePlotter:
    def __init__(self, port="COM7", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.qData, self.tData = [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.qLine, = self.ax.plot([], [], label="Angle", color="purple")
        self.qText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.axesSetup()

    def axesSetup(self):
        self.ax.set_title("Gyro Angle vs Time")
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Angle (deg)")
        self.ax.set_ylim(-100, 100)
        self.ax.legend()

    def serialRead(self):
        try:
            while self.ser.in_waiting: # while helps clear the buffer out compared to if
                line = self.ser.readline().decode('ascii').strip()
            if line:
                return float(line)
        except Exception: # ignore any errors in reading data
            pass
        return None

    def update(self, frame):
        qValue = self.serialRead()
        if qValue :
            self.tData.append(len(self.tData))
            self.qData.append(qValue)
            
            self.qLine.set_data(self.tData, self.qData)
            self.qText.set_text(f"Q: {qValue:.4f}")

            self.ax.set_xlim(max(0, len(self.tData) - 50), len(self.tData))
            self.ax.relim()
            self.ax.autoscale_view()

        return self.qLine, self.qText

    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=50)
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == "__main__":
    plotter = GyroAccumulatedAnglePlotter()
    plotter.run()
