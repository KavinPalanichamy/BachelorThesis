import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    # Configure serial port
    ser = serial.Serial('COM3', 115200, timeout=1)
    time.sleep(2)  # Wait for serial connection to initialize

    # Set up live plotting
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'bo')
    ax.set_xlim(0, 1023)  # Match touchscreen range
    ax.set_ylim(0, 1023)

    x_data, y_data = [], []

    try:
        while True:
            data = ser.readline().decode('utf-8').strip()
            if data:
                try:
                    x, y = map(float, data.split(','))
                    x_data.append(x)
                    y_data.append(y)

                    line.set_data(x_data, y_data)
                    ax.relim()
                    ax.autoscale_view()
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                except ValueError:
                    print(f"Invalid data: {data}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    read_serial_plot()
