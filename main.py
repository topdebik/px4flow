#! /usr/bin/env python3

import sys
import RPi.GPIO as gpio
from time import time, sleep
import rospy
from std_msgs.msg import String as msgString
from sensor_msgs.msg import Image as msgImage
from px4flow import PX4Flow
from TF03 import TF03
from kf import kf
from graph import graph


if __name__ == "__main__":
    global plotSpeedX, plotSpeedYX, plotSpeedYY, plotDistanceX, plotDistanceYX, plotDistanceYY
    gpio.setmode(gpio.BCM)
    px4 = PX4Flow()
    distanceSensor = TF03()
    graph = graph()
    rospy.init_node('px4flow_node')
    foo_pub1 = rospy.Publisher('/px4flow/velocityString', msgString, queue_size=1)
    foo_pub = rospy.Publisher('/px4flow/velocityImage', msgImage, queue_size=1)

    measureSpeed = 10 # sensor pollings per second
    measurementTime = time()
    count_time = 0

    count_x_form = 0
    count_y_form = 0
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

                speedXgsd = x * gsd / (measurementTime - lastMeasurementTime)
                speedYgsd = y * gsd / (measurementTime - lastMeasurementTime)
                speedgsd = (speedXgsd ** 2 + speedYgsd ** 2) ** 0.5
                speedXform = x / (16 / (4 * 6) * 1000) * altitude * -3 / (measurementTime - lastMeasurementTime)
                speedYform = y / (16 / (4 * 6) * 1000) * altitude * -3.25 / (measurementTime - lastMeasurementTime)
                speedform = (speedXform ** 2 + speedYform ** 2) ** 0.5
                
                """
                foo_pub.publish(graph.update(x, y, count_time))
                """

                data_as = f'X: {round(speedXgsd, 3)}  Y: {round(speedYgsd, 3)}'
                sys.stdout.write("\r" + "GSD  " + "X: " + "%.3f" % round(speedXgsd, 3) + " Y: " + "%.3f" % round(speedYgsd, 3) + " TOTAL: " + "%.3f" % round(speedgsd, 3) + "    FORM  " + "X: " + "%.3f" % round(speedXform, 3) + " Y: " + "%.3f" % round(speedYform, 3) + " TOTAL: " + "%.3f" % round(speedform, 3) + "    ALT  " + "%.3f" % round(altitude, 3) + " " * 4)
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
                distanceYgsd = y * gsd
                distanceXform = x / (16 / (4 * 6) * 1000) * altitude * -3
                distanceYform = y / (16 / (4 * 6) * 1000) * altitude * -3.25
                count_x_gsd += distanceXgsd
                count_y_gsd += distanceYgsd
                count_x_form += distanceXform
                count_y_form += distanceYform

                sys.stdout.write("\r" + "GSD  " + "X: " + "%.3f" % round(distanceXgsd, 3) + " Y: " + "%.3f" % round(distanceYgsd, 3) + "    FORM  " + "X: " + "%.3f" % round(distanceXform, 3) + " Y: " + "%.3f" % round(distanceYform, 3) + "    COUNTER_GSD  " + "X: " + "%.3f" % round(count_x_gsd, 3) + " Y: " + "%.3f" % round(count_y_gsd, 3) + "    COUNTER_FORM  " + "X: " + "%.3f" % round(count_x_form, 3) + " Y: " + "%.3f" % round(count_y_form, 3) + "    ALT  " + "%.3f" % round(altitude, 3) + " " * 4)
                sys.stdout.flush()

            else:
                print("available args: distance")
                sys.exit()

            count_time += (measurementTime - lastMeasurementTime) # the counter of the interval of polling the rangefinder sensor
            sleep(1 / measureSpeed)