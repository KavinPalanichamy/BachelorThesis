import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)

    # Create figure for XY plot with two lines
    fig, ax = plt.subplots(figsize=(8, 8))
    line_actual, = ax.plot([], [], 'b-', label='Actual Position')
    line_setpoint, = ax.plot([], [], 'r--', label='Setpoint')

    # Data storage for both actual and setpoint
    x_actual_data, y_actual_data = [], []
    x_setpoint_data, y_setpoint_data = [], []
    start_time = time.time()

    # Configure plot
    ax.set_title('Ball Position vs Setpoint - Infinity Trajectory')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.grid(True)
    ax.legend()
    ax.set_xlim(-400, 400)  # Updated range
    ax.set_ylim(-400, 400)  # Updated range
    ax.axis('equal')

    plt.tight_layout()
    plt.ion()
    plt.show()

    try:
        last_update = time.time()
        while True:
            current_time = time.time()
            
            # Only update every 0.25 seconds
            if current_time - last_update < 0.25:
                continue

            # Get only the last line from serial buffer
            ser.flushInput()
            data = ser.readline().decode('utf-8').strip()
            
            if data:
                try:
                    x_pos, y_pos, x_set, y_set = map(float, data.split(','))
                    
                    # Append actual position data
                    x_actual_data.append(x_pos - 500)  # Center the coordinates
                    y_actual_data.append(y_pos - 500)  # Center the coordinates
                    
                    # Append setpoint data
                    x_setpoint_data.append(x_set)
                    y_setpoint_data.append(y_set)

                    # Keep only last 100 points
                    if len(x_actual_data) > 100:
                        x_actual_data.pop(0)
                        y_actual_data.pop(0)
                        x_setpoint_data.pop(0)
                        y_setpoint_data.pop(0)

                    # Update both lines
                    line_actual.set_data(x_actual_data, y_actual_data)
                    line_setpoint.set_data(x_setpoint_data, y_setpoint_data)

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
