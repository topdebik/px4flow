#!/usr/bin/env python3

import sys
import RPi.GPIO as gpio
from time import time, sleep
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as msgImage
from geometry_msgs.msg import TwistStamped
from px4flow import PX4Flow
from TF03 import TF03
from kf import kf
from graph import graph
from Logger import Logger


if __name__ == "__main__":
    global plotSpeedX, plotSpeedYX, plotSpeedYY, plotDistanceX, plotDistanceYX, plotDistanceYY
    gpio.setmode(gpio.BCM)
    px4 = PX4Flow()
    distanceSensor = TF03()
    graph = graph()
    rospy.init_node("px4flow_node")
    textTopic = rospy.Publisher("/px4flow/velocityString", String, queue_size=1)
    graphTopic = rospy.Publisher("/px4flow/velocityImage", msgImage, queue_size=1)

    logger = Logger()

    measureSpeed = 10  # sensor pollings per second
    measurementTime = time()
    count_time = 0

    count_x_form = 0
    count_y_form = 0
    count_x_gsd = 0
    count_y_gsd = 0

    kfx = kf()
    kfy = kf()

    kfx_int = kf()
    kfy_int = kf()

    kfx_gps = kf()
    kfy_gps = kf()

    while True:
        altitude = distanceSensor.get_distance()
        gsd = (
            4.51 * ((altitude * 10**3) / 16) / (758 / 4) * 10**-3
        )  # ground sampling distance in meters/pixel

        # speed measurement

        if len(sys.argv) == 1:
            lastMeasurementTime = measurementTime

            # integral (compensated) measurement
            intData = px4.update_integral()
            if intData[6] == 0:
                speedXint = 0
                speedYint = 0
            else:
                speedXint = kfx_int.filter_value(
                    (intData[1] / 10 + intData[3] / 10)
                    * (altitude * 10**3)
                    / intData[6]
                )
                speedYint = kfy_int.filter_value(
                    (intData[2] / 10 + intData[4] / 10)
                    * (altitude * 10**3)
                    / intData[6]
                )
            speedint = (speedXint**2 + speedYint**2) ** 0.5

            frame = px4.update()
            x = frame[1] / 10
            y = frame[2] / 10
            timespan = frame[10] / 1000
            measurementTime = time()
            x = kfx.filter_value(x)
            y = kfy.filter_value(y)

            speedXgsd = x * gsd / timespan
            speedYgsd = (y * gsd / timespan) * -1
            speedgsd = (speedXgsd**2 + speedYgsd**2) ** 0.5
            speedXform = x / (16 / (4 * 6) * 1000) * altitude * 3 / timespan
            speedYform = y / (16 / (4 * 6) * 1000) * altitude * -3.25 / timespan
            speedform = (speedXform**2 + speedYform**2) ** 0.5

            """
                graphTopic.publish(graph.update(x, y, count_time))
                """

            try:
                gps = rospy.wait_for_message(
                    "/mavros/global_position/raw/gps_vel", TwistStamped, timeout=0.5
                ).twist.linear
                x_gps = kfx_gps.filter_value(gps.x)
                y_gps = kfy_gps.filter_value(gps.y)
            except rospy.exceptions.ROSException:
                x_gps = 0
                y_gps = 0

            data = [
                str(round(speedXint, 3)),
                str(round(speedYint, 3)),
                str(round(speedint, 3)),
                str(round(speedXgsd, 3)),
                str(round(speedYgsd, 3)),
                str(round(speedgsd, 3)),
                str(round(speedXform, 3)),
                str(round(speedYform, 3)),
                str(round(speedform, 3)),
                str(round(x_gps, 3)),
                str(round(y_gps, 3)),
                str(round((x_gps**2 + y_gps**2) ** 0.5, 3)),
                str(round(x_gps - speedXgsd, 3)),
                str(round(y_gps - speedYgsd, 3)),
                str(round((x_gps**2 + y_gps**2) ** 0.5 - speedgsd, 3)),
                str(round(x_gps - speedXform, 3)),
                str(round(y_gps - speedYform, 3)),
                str(round((x_gps**2 + y_gps**2) ** 0.5 - speedform, 3)),
                str(round(altitude, 3)),
            ]

            topicData = f"""\
{'INT':<5}\
{'X:':<3}{round(speedXint, 3):<8.3f}\
{'Y:':<3}{round(speedYint, 3):<8.3f}\
{'TOTAL:':<7}{round(speedint, 3):<9.3f}\
{'GSD':<5}\
{'X:':<3}{round(speedXgsd, 3):<8.3f}\
{'Y:':<3}{round(speedYgsd, 3):<8.3f}\
{'TOTAL:':<7}{round(speedgsd, 3):<9.3f}\
{'FORM':<6}\
{'X:':<3}{round(speedXform, 3):<8.3f}\
{'Y:':<3}{round(speedYform, 3):<8.3f}\
{'TOTAL:':<7}{round(speedform, 3):<9.3f}\
{'GPS':<5}\
{'X:':<3}{round(x_gps, 3):<8.3f}\
{'Y:':<3}{round(y_gps, 3):<8.3f}\
{'TOTAL:':<7}{round((x_gps ** 2 + y_gps ** 2) ** 0.5, 3):<9.3f}\
{'DIFF GPS/GSD':<14}\
{'X:':<3}{round(x_gps - speedXgsd, 3):<8.3f}\
{'Y:':<3}{round(y_gps - speedYgsd, 3):<8.3f}\
{'TOTAL:':<7}{round((x_gps ** 2 + y_gps ** 2) ** 0.5 - speedgsd, 3):<9.3f}\
{'DIFF GPS/FORM':<15}\
{'X:':<3}{round(x_gps - speedXform, 3):<8.3f}\
{'Y:':<3}{round(y_gps - speedYform, 3):<8.3f}\
{'TOTAL:':<7}{round((x_gps ** 2 + y_gps ** 2) ** 0.5 - speedform, 3):<9.3f}\
{'ALT':<5}\
{round(altitude, 3):<8.3f}\
\n"""

            sys.stdout.write("\r" + topicData[:-1])
            sys.stdout.flush()

            logger.log(data)
            textTopic.publish(topicData)

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

            sys.stdout.write(
                "\r"
                + "GSD  "
                + "X: "
                + "%.3f" % round(distanceXgsd, 3)
                + " Y: "
                + "%.3f" % round(distanceYgsd, 3)
                + "    FORM  "
                + "X: "
                + "%.3f" % round(distanceXform, 3)
                + " Y: "
                + "%.3f" % round(distanceYform, 3)
                + "    COUNTER_GSD  "
                + "X: "
                + "%.3f" % round(count_x_gsd, 3)
                + " Y: "
                + "%.3f" % round(count_y_gsd, 3)
                + "    COUNTER_FORM  "
                + "X: "
                + "%.3f" % round(count_x_form, 3)
                + " Y: "
                + "%.3f" % round(count_y_form, 3)
                + "    ALT  "
                + "%.3f" % round(altitude, 3)
                + " " * 4
            )
            sys.stdout.flush()

        else:
            print("available args: distance")
            sys.exit()

        # the counter of the interval of polling the rangefinder sensor
        count_time += measurementTime - lastMeasurementTime
        sleep(1 / measureSpeed)
