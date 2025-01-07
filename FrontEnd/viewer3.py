import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Create line objects for both actual positions and setpoints
    line_x_actual, = ax1.plot([], [], 'b-', label='Actual X Position')
    line_x_setpoint, = ax1.plot([], [], 'r--', label='X Setpoint')
    line_y_actual, = ax2.plot([], [], 'b-', label='Actual Y Position')
    line_y_setpoint, = ax2.plot([], [], 'r--', label='Y Setpoint')

    # Data storage
    time_data = []
    x_actual_data = []
    y_actual_data = []
    x_setpoint_data = []
    y_setpoint_data = []

    # Configure plots
    ax1.set_title('X Position and Setpoint')
    ax1.set_xlabel('Time (s)')  # Changed to seconds
    ax1.set_ylabel('Position')
    ax1.grid(True)
    ax1.legend()

    ax2.set_title('Y Position and Setpoint')
    ax2.set_xlabel('Time (s)')  # Changed to seconds
    ax2.set_ylabel('Position')
    ax2.grid(True)
    ax2.legend()

    plt.suptitle('Position Tracking - Circle Trajectory')
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
                        # Parse actual X, actual Y, setpoint X, setpoint Y, timestamp
                        x_actual, y_actual, x_setpoint, y_setpoint, timestamp = map(float, data.split(','))
                        # Subtract 500 to get real positions
                        x_actual -= 500
                        y_actual -= 500
                        
                        # Convert milliseconds to seconds
                        time_sec = timestamp / 1000.0
                        
                        # Append data
                        time_data.append(time_sec)
                        x_actual_data.append(x_actual)
                        y_actual_data.append(y_actual)
                        x_setpoint_data.append(x_setpoint)
                        y_setpoint_data.append(y_setpoint)

                        # Keep last 200 points
                        if len(time_data) > 200:
                            time_data.pop(0)
                            x_actual_data.pop(0)
                            y_actual_data.pop(0)
                            x_setpoint_data.pop(0)
                            y_setpoint_data.pop(0)

                        # Update plot data
                        line_x_actual.set_data(time_data, x_actual_data)
                        line_x_setpoint.set_data(time_data, x_setpoint_data)
                        line_y_actual.set_data(time_data, y_actual_data)
                        line_y_setpoint.set_data(time_data, y_setpoint_data)

                        # Update axes - show complete time range
                        for ax in [ax1, ax2]:
                            ax.relim()
                            ax.autoscale_view(scalex=False, scaley=False)  # Disable autoscaling
                            ax.set_xlim(min(time_data), max(time_data))  # Always show from min to current max time
                            ax.set_ylim(-320, 320)  # Updated Y-axis range

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
