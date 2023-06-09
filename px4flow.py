import sys
from smbus import SMBus
from serial import Serial
from numpy import median
import matplotlib.pyplot as plt
#import adafruit_vl53l1x
#import board
import RPi.GPIO as gpio
from time import time, sleep
from math import atan, degrees

def shift32(num):
    if num > 2147483648:
        return - 4294967296 + num
    return num


def shift16(num):
    if num > 32768:
        return -65536 + num
    return num


def shift8(num):
    if num > 128:
        return -256 + num
    return num


class PX4Flow:
    def __init__(self, bus=1):
        self.ADDRESS = 0x42
        self.BUS = SMBus(bus)

    def update(self):
        self.BUS.write_byte_data(self.ADDRESS, 0x00, 0x0)
        data = list(map(lambda x: hex(x)[2:], self.BUS.read_i2c_block_data(self.ADDRESS, 0x00, 22)))

        frame_count = shift16(int(data[1] + data[0], 16))
        pixel_flow_x_sum = shift16(int(data[3] + data[2], 16))
        pixel_flow_y_sum = shift16(int(data[5] + data[4], 16))
        flow_comp_m_x = shift16(int(data[7] + data[6], 16))
        flow_comp_m_y = shift16(int(data[9] + data[8], 16))
        qual = shift16(int(data[11] + data[10], 16))
        gyro_x_rate = shift16(int(data[13] + data[12], 16))
        gyro_y_rate = shift16(int(data[15] + data[14], 16))
        gyro_z_rate = shift16(int(data[17] + data[16], 16))
        gyro_range = shift8(int(data[18], 16))
        sonar_timestamp = shift8(int(data[19], 16))
        ground_distance = shift16(int(data[21] + data[20], 16))

        return frame_count, pixel_flow_x_sum, pixel_flow_y_sum, flow_comp_m_x, flow_comp_m_y, qual, gyro_x_rate, gyro_y_rate, gyro_z_rate, gyro_range, sonar_timestamp, ground_distance

    def update_integral(self):
        self.BUS.write_byte_data(self.ADDRESS, 0x00, 0x16)
        data = list(map(lambda x: hex(x)[2:], self.BUS.read_i2c_block_data(self.ADDRESS, 0x00, 26)))

        frame_count_since_last_readout = shift16(int(data[1] + data[0], 16))
        pixel_flow_x_integral = shift16(int(data[3] + data[2], 16))
        pixel_flow_y_integral = shift16(int(data[5] + data[4], 16))
        gyro_x_rate_integral = shift16(int(data[7] + data[6], 16))
        gyro_y_rate_integral = shift16(int(data[9] + data[8], 16))
        gyro_z_rate_integral = shift16(int(data[11] + data[10], 16))
        integration_timespan = shift32(int(data[15] + data[14] + data[13] + data[12], 16))
        sonar_timestamp = shift32(int(data[19] + data[18] + data[17] + data[16], 16))
        ground_distance = shift16(int(data[21] + data[20], 16))
        gyro_temperature = shift16(int(data[23] + data[22], 16))
        quality = shift8(int(data[24], 16))

        return frame_count_since_last_readout, pixel_flow_x_integral, pixel_flow_y_integral, gyro_x_rate_integral, gyro_y_rate_integral, gyro_z_rate_integral, integration_timespan, sonar_timestamp, ground_distance, gyro_temperature, quality

class VL53L1X:
    def __init__(self, distanceMode = 2):
        self.i2c = board.I2C()
        self.BUS = adafruit_vl53l1x.VL53L1X(self.i2c)
        self.BUS.distance_mode = distanceMode
        self.BUS.timing_budget = 50
        self.distance = 0


    def measureDistance(self):
        self.BUS.start_ranging()
        tries = 0
        while not self.BUS.data_ready:
            tries += 1
            if tries >= 6:
                return self.distance
            sleep(0.2)
        self.distance = self.BUS.distance
        self.BUS.clear_interrupt()
        self.BUS.stop_ranging()
        if not self.distance:
            return None
        self.distance = self.distance / 100
        return self.distance

class HCSR04:
    def __init__(self, trig = 20, echo = 16):
        self.trig = trig
        self.echo = echo
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)

    def distance(self):
        gpio.output(self.trig, gpio.HIGH)
        sleep(0.00001)
        gpio.output(self.trig, gpio.LOW)
        while gpio.input(self.echo) == gpio.LOW:
            start = time()
        while gpio.input(self.echo) == gpio.HIGH:
            stop = time()
        distance = (stop - start) * 17150
        return distance * 10 ** -2
    
    def read_distance_filtered(self):
        history = [self.distance() for _ in range(10)]
        return median(history)

def onExit():
    _, ax = plt.subplots()

    if sys.argv[1] == "speed":
        global plotSpeedX, plotSpeedYX, plotSpeedYY
        ax.plot(plotSpeedX, plotSpeedYX, color = "blue")
        ax.set_xlabel("Time")
        ax.set_ylabel("SpeedX", color = "blue")
        ax2 = ax.twinx()
        ax2.plot(plotSpeedX, plotSpeedYY, color = "red")
        ax2.set_ylabel("SpeedY", color = "red")
        plt.savefig(f"Speed_{time()}.png")
        return

    elif sys.argv[1] == "distance":
        global plotDistanceX, plotDistanceYX, plotDistanceYY
        ax.plot(plotDistanceX, plotDistanceYX, color = "blue")
        ax.set_xlabel("Time")
        ax.set_ylabel("DistanceX", color = "blue")
        ax2 = ax.twinx()
        ax2.plot(plotDistanceX, plotDistanceYY, color = "red")
        ax2.set_ylabel("DistanceY", color = "red")
        plt.savefig(f"Distance_{time()}.png")
        return
    return

if __name__ == "__main__":
    try:
        global plotSpeedX, plotSpeedYX, plotSpeedYY, plotDistanceX, plotDistanceYX, plotDistanceYY
        gpio.setmode(gpio.BCM)
        px4 = PX4Flow()
        distanceSensor = HCSR04()
        #uart = Serial("/dev/serial0", 115200)

        measureSpeed = 5 # sensor pollings per second

        matrixWidth = 758  #matrix width in pixels
        matrixHeight = 480  #matrix height in pixels
        matrixSizeX = 4.51 * 10 ** -3  #matrix size on X axis in meters
        matrixSizeY = 2.88 * 10 ** -3  #matrix size on Y axis in meters
        pixelSize = 6 * 10 ** -6  # pixel size in meters
        focalLength = 16 * 4 * 10 ** -3  # focal length in meters
        viewAngleX = (2 * degrees(atan(matrixSizeX / (2 * focalLength))))  # view angle of camera on X axis
        viewAngleY = (2 * degrees(atan(matrixSizeY / (2 * focalLength))))  # view angle of camera on Y axis

        altitude = 1  # altitude in meters

        count_x = 0
        count_y = 0
        count_time = 0
        plotSpeedX = []
        plotSpeedYX = []
        plotSpeedYY = []
        plotDistanceX = []
        plotDistanceYX = []
        plotDistanceYY = []

        while True:
                #getting the height from the sensor
                '''
                if count_time == measureSpeed:
                    altitude = round(distanceSensor.distance(), 3)
                    count_time = 0
                '''
                altitude = distanceSensor.read_distance_filtered()

                #gsd = ((matrixSizeX * 10 ** 3) * altitude * 100) / ((focalLength * 10 ** 3) * matrixWidth) #ground sampling distance in meters/pixel

                if len(sys.argv) != 2:
                    sys.exit()

                # speed measurement
                
                elif sys.argv[1] == "speed":
                    x, y = px4.update()[1:3]
                    speedXPixels = x #X speed in pixels
                    speedYPixels = y #Y speed in pixels
                    #speedXM = speedXPixels * (2 * altitude * tan(viewAngleX) + matrixWidth * pixelSize)
                    speedXM = speedXPixels / (16 / (4 * 6) * 1000) * altitude * -3 * measureSpeed
                    #speedYM = speedYPixels * (2 * altitude * tan(viewAngleY) + matrixHeight * pixelSize)
                    speedYM = speedYPixels / (16 / (4 * 6) * 1000) * altitude * -3.25 * measureSpeed
                    print("X:", round(speedXM, 3), "Y:", round(speedYM, 3))
                    print("Растояние:", altitude)
                    plotSpeedX.append(count_time)
                    plotSpeedYX.append(speedXM)
                    plotSpeedYY.append(speedYM)

                
                
                # distance measurement (prints distance between sensor pollings)
                elif sys.argv[1] == "distance":
                    x, y = px4.update()[1:3]
                    #distanceX = x * (2 * altitude * tan(viewAngleX) + matrixHeight * pixelSize)
                    distanceX = x / (16 / (4 * 6) * 1000) * altitude * -3
                    #distanceX = x * gsd
                    #distanceY = y * (2 * altitude * tan(viewAngleY) + matrixWidth * pixelSize)
                    distanceY = y / (16 / (4 * 6) * 1000) * altitude * -3.25
                    #distanceY = y * gsd
                    print(x, y)
                    #print("X:", round(distanceX, 3), "Y:", round(distanceY, 3))
                    count_x += distanceX
                    count_y += distanceY
                    print('Счётчик X', round(count_x, 3))
                    print('Счётчик Y', round(count_y, 3))
                    print("Растояние:", altitude)
                    plotDistanceX.append(count_time)
                    plotDistanceYX.append(distanceX)
                    plotDistanceYY.append(distanceY)

                else:
                    sys.exit()

                

                #print distance from sensor
                '''
                print(round(distanceSensor.distance(), 3))
                '''
                count_time += 1 / measureSpeed #This is the counter of the interval of polling the rangefinder sensor
                sleep(1 / measureSpeed)
    except KeyboardInterrupt:
        onExit()