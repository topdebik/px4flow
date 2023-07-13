import sys
import RPi.GPIO as gpio
from time import time, sleep
import rospy
from std_msgs.msg import String as msgString
from sensor_msgs.msg import Image as msgImage
from px4flow import PX4Flow
from HCSR04 import HCSR04
from kf import kf
from graph import graph


if __name__ == "__main__":
    global plotSpeedX, plotSpeedYX, plotSpeedYY, plotDistanceX, plotDistanceYX, plotDistanceYY
    gpio.setmode(gpio.BCM)
    px4 = PX4Flow()
    distanceSensor = HCSR04()
    graph = graph()
    rospy.init_node('my_ros_node')
    foo_pub1 = rospy.Publisher('/velocityString', msgString, queue_size=1)
    foo_pub = rospy.Publisher('/velocityImage', msgImage, queue_size=1)

    measureSpeed = 10 # sensor pollings per second
    measurementTime = time()
    count_time = 0

    count_x = 0
    count_y = 0

    kfx = kf()
    kfy = kf()

    while True:
            altitude = distanceSensor.get_distance_filtered()
            gsd = 4.51 * ((altitude * 10 ** 3) / (12 * 4)) / (758 / 4) * 10 ** -3 #ground sampling distance in meters/pixel

            if len(sys.argv) != 2:
                sys.exit()

            # speed measurement
            
            elif sys.argv[1] == "speed":
                lastMeasurementTime = measurementTime
                x, y = px4.update()[1:3]
                measurementTime = time()
                x = kfx.filter_value(x)
                y = kfy.filter_value(y)
                speedXPixels = x #X speed in pixels
                speedYPixels = y #Y speed in pixels
                speedXMgsd = speedXPixels * gsd / (measurementTime - lastMeasurementTime)
                speedXM = speedXPixels / (16 / (4 * 6) * 1000) * altitude * -3 / (measurementTime - lastMeasurementTime)
                speedYMgsd = speedYPixels * gsd / (measurementTime - lastMeasurementTime)
                speedYM = speedYPixels / (16 / (4 * 6) * 1000) * altitude * -3.25 / (measurementTime - lastMeasurementTime)
                
                """
                foo_pub.publish(graph.update(x, y, count_time))
                """

                data_as = f'X: {round(speedXMgsd, 3)}  Y: {round(speedYMgsd, 3)}'
                sys.stdout.write("\r" + "GSD  " + "X: " + "%.3f" % round(speedXMgsd, 3) + " Y: " + "%.3f" % round(speedYMgsd, 3) + "    FORM  " + "X: " + "%.3f" % round(speedXM, 3) + " Y: " + "%.3f" % round(speedYM, 3) + "    ALT  " + "%.3f" % round(altitude, 3) + " " * 4)
                sys.stdout.flush()
                foo_pub1.publish(data_as)
                
                
            # distance measurement (prints distance between sensor pollings)
            elif sys.argv[1] == "distance":
                x, y = px4.update()[1:3]
                x = kfx.filter_value(x)
                y = kfy.filter_value(y)
                distanceX = x / (16 / (4 * 6) * 1000) * altitude * -3
                #distanceX = x * gsd
                distanceY = y / (16 / (4 * 6) * 1000) * altitude * -3.25
                #distanceY = y * gsd
                print(x, y)
                #print("X:", round(distanceX, 3), "Y:", round(distanceY, 3))
                count_x += distanceX
                count_y += distanceY
                print('Счётчик X', round(count_x, 3))
                print('Счётчик Y', round(count_y, 3))
                print("Высота:", altitude)

            else:
                sys.exit()

            count_time += (measurementTime - lastMeasurementTime) #This is the counter of the interval of polling the rangefinder sensor
            sleep(1 / measureSpeed)