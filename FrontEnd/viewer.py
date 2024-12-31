import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    line_x, = ax1.plot([], [], 'r-', label='X Error')
    line_y, = ax2.plot([], [], 'b-', label='Y Error')
    
    time_data, error_x_data, error_y_data = [], [], []
    start_time = time.time()

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

    plt.tight_layout()
    plt.ion()
    plt.show()

    try:
        last_update = time.time()
        while True:
            current_time = time.time()
            
            # Only update every 25ms
            if current_time - last_update >= 0.025:
                ser.flushInput()
                data = ser.readline().decode('utf-8').strip()

                if data:
                    try:
                        millis, error_x, error_y = map(float, data.split(','))
                        # Convert milliseconds to seconds
                        time_sec = millis / 1000.0
                        
                        time_data.append(time_sec)
                        error_x_data.append(error_x)
                        error_y_data.append(error_y)
                        
                        # Keep last 200 points
                        if len(time_data) > 200:
                            time_data.pop(0)
                            error_x_data.pop(0)
                            error_y_data.pop(0)
                        
                        # Update plot data
                        line_x.set_data(time_data, error_x_data)
                        line_y.set_data(time_data, error_y_data)
                        
                        # Adjust plot limits to show full range
                        for ax in [ax1, ax2]:
                            ax.relim()
                            ax.autoscale_view()
                            ax.set_xlim(0, time_data[-1])  # Show from 0 to current time
                            
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
