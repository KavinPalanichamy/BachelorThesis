import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Create line objects for error plots and reference lines
    line_x_error, = ax1.plot([], [], 'b-', label='X Error')
    line_y_error, = ax2.plot([], [], 'r-', label='Y Error')
    ax1.axhline(y=0, color='k', linestyle='--')
    ax2.axhline(y=0, color='k', linestyle='--')

    # Data storage
    time_data = []
    x_error_data = []
    y_error_data = []

    # Configure plots
    ax1.set_title('X Error over Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('X Error')
    ax1.grid(True)
    ax1.legend()

    ax2.set_title('Y Error over Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y Error')
    ax2.grid(True)
    ax2.legend()

    plt.suptitle('Error Tracking')
    plt.tight_layout()
    plt.ion()
    plt.show()

    try:
        last_update = time.time()
        while True:
            current_time = time.time()
            if current_time - last_update >= 0.025:
                ser.flushInput()
                data = ser.readline().decode('utf-8').strip()

                if data:
                    try:
                        # Parse rawErrorX, rawErrorY, and timestamp
                        x_error, y_error, timestamp = map(float, data.split(','))
                        
                        # Convert milliseconds to seconds
                        time_sec = timestamp / 1000.0
                        
                        # Append data
                        time_data.append(time_sec)
                        x_error_data.append(x_error)
                        y_error_data.append(y_error)

                        # Keep last 200 points
                        if len(time_data) > 200:
                            time_data.pop(0)
                            x_error_data.pop(0)
                            y_error_data.pop(0)

                        # Update plot data
                        line_x_error.set_data(time_data, x_error_data)
                        line_y_error.set_data(time_data, y_error_data)

                        # Update axes - show complete time range
                        for ax in [ax1, ax2]:
                            ax.relim()
                            ax.autoscale_view()
                            ax.set_xlim(min(time_data), max(time_data))

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
