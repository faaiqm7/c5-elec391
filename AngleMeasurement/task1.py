import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class SerialPlot:
    def __init__(self, COMPORTnum: int, baud: int, numDataPoints: int,
                 Yauto: bool = True, Ymax: float = 100, Ymin: float = 0):
        self.comport = COMPORTnum
        self.baud = baud
        self.numDataPoints = numDataPoints
        self.Yauto = Yauto
        self.Ymax = Ymax
        self.Ymin = Ymin

        self.datalist = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ser = serial.Serial(f"COM{self.comport}", self.baud) 
        time.sleep(2)
    
    def _readSerial(self) -> float:
        while True:
            rawline = self.ser.readline().decode('ascii').strip() # Read and decode
            if rawline:
                try:
                    # Parse the x, y, z values from comma-separated data
                    data = rawline.split(',')
                    x = float(data[0])  # Extract the x value
                    print(f"x: {x}")  # Debug x value
                    return x
                except (ValueError, IndexError):
                    print(f"Invalid data: {rawline}")  # Debug invalid data
                    pass

    def _animate(self, *args):
        self.ser.write(b'g')  # Optional: Trigger Arduino (not necessary here)

        # Read x value
        readdata = self._readSerial()
        self.datalist.append(readdata)

        # Fix the list size to maintain the window of numDataPoints
        self.datalist = self.datalist[-self.numDataPoints:]
        
        # Update the plot
        self.ax.clear()  # Clear the previous frame
        self._formatPlot()
        self.ax.plot(self.datalist, label="x (deg/s)", color="r")
        self.ax.legend()

    def _formatPlot(self):
        MARGIN = 1.1
        if self.Yauto:
            ymin = min(0, int(min(self.datalist)) * MARGIN)
            ymax = int(max(self.datalist)) * MARGIN
        else:
            ymin = self.Ymin
            ymax = self.Ymax
        self.ax.set_ylim([ymin, ymax])  
        self.ax.set_title("Arduino Gyroscope Data (x-axis)")  # Title
        self.ax.set_ylabel("Angular Velocity (deg/s)")  # Y-axis label

    def run(self):
        ani = animation.FuncAnimation(self.fig, self._animate, frames=100, fargs=None, interval=100) 
        plt.show()

    def __del__(self):
        self.ser.close()


if __name__ == "__main__":
    liveplot = SerialPlot(
        COMPORTnum=3,
        baud=9600,
        numDataPoints=70,
        Yauto=True,
        Ymax=30  # Adjust as needed for your data range
    )

    liveplot.run()
