#! /usr/bin/env python3

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
    count_x_gsd = 0
    count_y_gsd = 0

    kfx = kf()
    kfy = kf()

    while True:
            altitude = distanceSensor.get_distance()
            gsd = 4.51 * ((altitude * 10 ** 3) / 12) / (758 / 4) * 10 ** -3 #ground sampling distance in meters/pixel

            # speed measurement
            
            if len(sys.argv) == 1:
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
                lastMeasurementTime = measurementTime
                x, y = px4.update()[1:3]
                measurementTime = time()
                x = kfx.filter_value(x)
                y = kfy.filter_value(y)
                distanceXgsd = x * gsd
                distanceX = x / (16 / (4 * 6) * 1000) * altitude * -3
                distanceYgsd = y * gsd
                distanceY = y / (16 / (4 * 6) * 1000) * altitude * -3.25
                count_x_gsd += distanceXgsd
                count_y_gsd += distanceYgsd
                count_x += distanceX
                count_y += distanceY
                sys.stdout.write("\r" + "GSD  " + "X: " + "%.3f" % round(distanceXgsd, 3) + " Y: " + "%.3f" % round(distanceYgsd, 3) + "    FORM  " + "X: " + "%.3f" % round(distanceX, 3) + " Y: " + "%.3f" % round(distanceY, 3) + "    COUNTER_GSD  " + "X: " + "%.3f" % round(count_x_gsd, 3) + " Y: " + "%.3f" % round(count_y_gsd, 3) + "    COUNTER_FORM  " + "X: " + "%.3f" % round(count_x, 3) + " Y: " + "%.3f" % round(count_y, 3) + "    ALT  " + "%.3f" % round(altitude, 3) + " " * 4)
                sys.stdout.flush()

            else:
                print("available args: distance")
                sys.exit()

            count_time += (measurementTime - lastMeasurementTime) #This is the counter of the interval of polling the rangefinder sensor
            sleep(1 / measureSpeed)