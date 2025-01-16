import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ComplimentaryPlotter:
    def __init__(self, port="COM3", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.xData, self.yData, self.zData, self.tData = [], [], [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.xLine, = self.ax.plot([], [], label="Complementary", color="red")
        self.yLine, = self.ax.plot([], [], label="Accelerometer", color="green")
        self.zLine, = self.ax.plot([], [], label="Gyroscope", color="blue")
        self.xText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.yText = self.ax.text(0.02, 0.90, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self.zText = self.ax.text(0.02, 0.85, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
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
                line = self.ser.readline().decode('ascii').strip() 
            values = line.split(',')
            if len(values) == 3:
                return [float(v) for v in values]
        except Exception:
            pass
        return None

    def update(self, frame):
        axData = self.read_serial_data()
        if axData:
            self.tData.append(len(self.tData))
            self.xData.append(axData[0])
            self.yData.append(axData[1])
            self.zData.append(axData[2])
            
            self.xLine.set_data(self.tData, self.xData)
            self.yLine.set_data(self.tData, self.yData)
            self.zLine.set_data(self.tData, self.zData)

            self.xText.set_text(f"Complementary Angle: {axData[0]:.2f} deg")
            self.yText.set_text(f"Accelerometer Angle: {axData[1]:.2f} deg")
            self.zText.set_text(f"Gyroscope Angle: {axData[2]:.2f} deg")

            self.ax.set_xlim(max(0, len(self.tData) - 50), len(self.tData))
            self.ax.relim()
            self.ax.autoscale_view()

        return self.xLine, self.yLine, self.zLine, self.xText, self.yText, self.zText

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
