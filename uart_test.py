from serial import Serial
from time import sleep


port = "/dev/serial0"
bus = Serial(port, baudrate = 115200)
for i in range(999999):
    print(bytes(str(i) + "\n", encoding = "utf-8"))
    bus.write(bytes(str(i) + "\n", encoding = "utf-8"))
    sleep(0.5)