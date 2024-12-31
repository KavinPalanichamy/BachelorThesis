import serial
import time
import matplotlib.pyplot as plt

def read_serial_plot():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    # Two lines per axis: raw (red) and filtered (green)
    line_x_raw, = ax1.plot([], [], 'r-', label='X Raw')
    line_x_filt, = ax1.plot([], [], 'g-', label='X Filtered')
    line_y_raw, = ax2.plot([], [], 'r-', label='Y Raw')
    line_y_filt, = ax2.plot([], [], 'g-', label='Y Filtered')

    time_data = []
    x_data, x_filt_data = [], []
    y_data, y_filt_data = [], []

    start_time = time.time()

    # Configure plots
    ax1.set_title('X Raw vs. Filtered')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('X Value')
    ax1.grid(True)
    ax1.legend()

    ax2.set_title('Y Raw vs. Filtered')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y Value')
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
                        # Parse rawX, rawY, xFiltered, yFiltered
                        pX, pY, xFilt, yFilt = map(float, data.split(','))
                        elapsed_s = current_time - start_time

                        # Append data
                        time_data.append(elapsed_s)
                        x_data.append(pX)
                        x_filt_data.append(xFilt)
                        y_data.append(pY)
                        y_filt_data.append(yFilt)

                        # Keep last 200 points
                        if len(time_data) > 200:
                            time_data.pop(0)
                            x_data.pop(0)
                            x_filt_data.pop(0)
                            y_data.pop(0)
                            y_filt_data.pop(0)

                        # Update plot data
                        line_x_raw.set_data(time_data, x_data)
                        line_x_filt.set_data(time_data, x_filt_data)
                        line_y_raw.set_data(time_data, y_data)
                        line_y_filt.set_data(time_data, y_filt_data)

                        for ax in [ax1, ax2]:
                            ax.relim()
                            ax.autoscale_view()
                            ax.set_xlim(0, time_data[-1])  # dynamic x-axis

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
