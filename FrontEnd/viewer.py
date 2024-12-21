import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    # Configure serial port with shorter timeout
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)  # Wait for serial connection to initialize

    # Initialize plot
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-')
    x_data, y_data = [], []

    # Add titles and gridlines
    ax.set_title('Path tracked by the ball (Kp = 0.1160, Ki = 0.00130, Kd = 0.1430)')
    ax.set_xlabel('X Value')
    ax.set_ylabel('Y Value')
    ax.grid(True)
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)

    plt.ion()
    plt.show()

    try:
        last_update = time.time()
        while True:
            # Read a line from serial
            data = ser.readline().decode('utf-8').strip()

            current_time = time.time()
            if current_time - last_update >= 1.0 and data:
                print(f"Received data: {data}")
                try:
                    x, y = map(float, data.split(','))
                    print(f"Parsed values - x: {x}, y: {y}")

                    x_data.append(x)
                    y_data.append(y)
                    
                    # Limit data points to prevent memory issues
                    if len(x_data) > 100:
                        x_data.pop(0)
                        y_data.pop(0)
                    
                    line.set_data(x_data, y_data)
                    
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    
                    last_update = current_time
                except ValueError as e:
                    print(f"Invalid data: {data}, Error: {e}")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    read_serial_plot()
