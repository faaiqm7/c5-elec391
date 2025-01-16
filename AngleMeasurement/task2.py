import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class AcceloremeterAnglePlotter:
    def __init__(self, port="COM3", baud_rate=9600):
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.qData, self.tData = [], []
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.qLine, = self.ax.plot([], [], label="Tilt Angle (Q)", color="purple")
        self.qText = self.ax.text(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=10, verticalalignment='top')
        self._setup_axes()

    def _setup_axes(self):
        self.ax.set_title("Tilt Angle (Q) over Time")
        self.ax.set_xlabel("Time (frames)")
        self.ax.set_ylabel("Angle (radians)")
        self.ax.legend()

    def read_serial_data(self):
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii').strip()
            if line:
                return float(line)
        except Exception:
            pass
        return None

    def update(self, frame):
        qValue = self.read_serial_data()
        if qValue is not None:
            self.tData.append(len(self.tData))
            self.qData.append(qValue)
            
            self.qLine.set_data(self.tData, self.qData)
            self.qText.set_text(f"Q: {qValue:.4f} rad")

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
    plotter = AcceloremeterAnglePlotter()
    plotter.run()
