import serial

ser = serial.Serial(
    "/dev/ttyUSB1",
    115200,
    timeout=1
)

print("Listening...")

while True:
    data = ser.readline()

    if data:
        print("RAW :", repr(data))
        print("HEX :", data.hex(" "))
