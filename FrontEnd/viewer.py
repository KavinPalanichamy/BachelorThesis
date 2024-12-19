import serial
import time

import matplotlib.pyplot as plt

def read_serial_plot():
    # Configure the serial port
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)  # Wait for the serial connection to initialize

    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'bo')  # Blue dot for the ball position

    ax.set_xlim(0, 100)  # Set x-axis limits
    ax.set_ylim(0, 100)  # Set y-axis limits

    while True:
        try:
            data = ser.readline().decode('utf-8').strip()
            if data:
                x, y = map(float, data.split(','))
                line.set_xdata(x)
                line.set_ydata(y)
                ax.draw_artist(ax.patch)
                ax.draw_artist(line)
                fig.canvas.flush_events()
                fig.canvas.draw()
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")

    ser.close()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    read_serial_plot()