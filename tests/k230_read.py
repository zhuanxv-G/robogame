import serial

ser = serial.Serial("/dev/ttyACM0", 115200)

while True:
    data = ser.readline()
    if data:
        print(data.decode(errors="ignore").strip())
