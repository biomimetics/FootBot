import serial

with serial.Serial('/dev/ttyACM4', 115200, timeout=1.0) as ser:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
