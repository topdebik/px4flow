import sys
from serial import Serial
import matplotlib.pyplot as plt
from time import time


port = "COM7"
bus = Serial(port, baudrate = 115200)
"""
figX = plt.figure()
figY = plt.figure()
axX = figX.add_subplot()
axY = figY.add_subplot()

start = time()
"""
while True:
    try:
        print(bus.read())
        """
        xi, yi = list(map(float, bus.read().split()))
        print(xi, yi)
        axX.clear()
        axY.clear()

        axX.plot(start - time(), xi, 'r-', label='X')
        axY.plot(start - time(), yi, 'b-', label='Y')

        axX.legend()
        axY.legend()

        plt.pause(0.05)
        """
    except KeyboardInterrupt:
        sys.exit()

#plt.show()