import serial
import csv
import time

def generate_csv():
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    time.sleep(2)
    with open('data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time","x", "y", "positions0", "positions1", "positions2"])
        try:
            while True:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    values = data.split(',')
                    if len(values) == 6:
                        writer.writerow(values)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

if __name__ == "__main__":
    generate_csv()
