import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ComplimentaryPlotter:
    def __init__(self, port="COM3", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.compData, self.accData, self.gyroData, self.tData = [], [], [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.compLine, = self.ax.plot([], [], label="Complementary", color="red")
        self.accLine, = self.ax.plot([], [], label="Accelerometer", color="green")
        self.gyroLine, = self.ax.plot([], [], label="Gyroscope", color="blue")
        self.compText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.accText = self.ax.text(0.02, 0.90, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.gyroText = self.ax.text(0.02, 0.85, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self._setup_axes()

    def _setup_axes(self):
        self.ax.set_title("Complementary filter Angle with Gyro and Acceleromter readings vs Time")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Angle (deg)")
        self.ax.set_ylim(-100, 100)
        self.ax.legend()

    def read_serial_data(self):
        try:
            while self.ser.in_waiting:
                self.ser.reset_input_buffer()
                line = self.ser.readline().decode('ascii', errors='ignore').strip() 
            values = line.split(',')
            if len(values) == 3 and len(values[0]) >= 3:
                return [float(v) for v in values]
        except Exception:
            pass
        return None

    def update(self, frame):
        axData = self.read_serial_data()
        if axData:
            self.tData.append(len(self.tData))
            self.compData.append(axData[0])
            self.accData.append(axData[1])
            self.gyroData.append(axData[2])
            
            self.compLine.set_data(self.tData, self.compData)
            self.accLine.set_data(self.tData, self.accData)
            self.gyroLine.set_data(self.tData, self.gyroData)

            self.compText.set_text(f"Complementary Angle: {axData[0]:.2f} deg")
            self.accText.set_text(f"Accelerometer Angle: {axData[1]:.2f} deg")
            self.gyroText.set_text(f"Gyroscope Angle: {axData[2]:.2f} deg")

            self.ax.set_xlim(max(0, len(self.tData) - 50), len(self.tData))
            self.ax.relim()
            self.ax.autoscale_view()

        return self.compLine, self.accLine, self.gyroLine, self.compText, self.accText, self.gyroText

    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=10)
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == "__main__":
    plotter = ComplimentaryPlotter()
    plotter.run()
