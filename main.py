import sys
import matplotlib.pyplot as plt
import RPi.GPIO as gpio
from time import time, sleep
from math import atan, degrees
from pykalman import KalmanFilter
import numpy as np
import rospy
from std_msgs.msg import String as msgString
from sensor_msgs.msg import Image as msgImage
from cv_bridge import CvBridge
import cv2 as cv
from px4flow import PX4Flow
from HCSR04 import HCSR04


if __name__ == "__main__":
    global plotSpeedX, plotSpeedYX, plotSpeedYY, plotDistanceX, plotDistanceYX, plotDistanceYY
    gpio.setmode(gpio.BCM)
    px4 = PX4Flow()
    distanceSensor = HCSR04()
    rospy.init_node('my_ros_node')
    foo_pub1 = rospy.Publisher('/velocityString', msgString, queue_size = 1)
    foo_pub = rospy.Publisher('/velocityImage', msgImage, queue_size = 1)
    bridge = CvBridge()
    fig, ax = plt.subplots()
    ax2 = ax.twinx()

    measureSpeed = 10 # sensor pollings per second
    measurementTime = time()

    matrixWidth = 758  #matrix width in pixels
    matrixHeight = 480  #matrix height in pixels
    matrixSizeX = 4.51 * 10 ** -3  #matrix size on X axis in meters
    matrixSizeY = 2.88 * 10 ** -3  #matrix size on Y axis in meters
    pixelSize = 6 * 10 ** -6  # pixel size in meters
    focalLength = 16 * 4 * 10 ** -3  # focal length in meters
    viewAngleX = (2 * degrees(atan(matrixSizeX / (2 * focalLength))))  # view angle of camera on X axis
    viewAngleY = (2 * degrees(atan(matrixSizeY / (2 * focalLength))))  # view angle of camera on Y axis

    count_x = 0
    count_y = 0
    count_time = 0
    plotSpeedX = []
    plotSpeedYX = []
    plotSpeedYY = []
    plotDistanceX = []
    plotDistanceYX = []
    plotDistanceYY = []

    kfx = KalmanFilter(transition_matrices=[1],
                observation_matrices=[1],
                initial_state_mean=0,
                initial_state_covariance=1,
                observation_covariance=1,
                transition_covariance=0.03)
    initial_state_x = 0
    initial_covariance_x = 1
    kfy = KalmanFilter(transition_matrices=[1],
                observation_matrices=[1],
                initial_state_mean=0,
                initial_state_covariance=1,
                observation_covariance=1,
                transition_covariance=0.03)
    initial_state_y = 0
    initial_covariance_y = 1

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

                filtered_state_mean_x, filtered_state_covariance_x = kfx.filter_update(
                    filtered_state_mean = initial_state_x,
                    filtered_state_covariance = initial_covariance_x,
                    observation = x
                )
                initial_state_x = filtered_state_mean_x
                initial_covariance_x = filtered_state_covariance_x
                x = filtered_state_mean_x.flatten()[0]

                filtered_state_mean_y, filtered_state_covariance_y = kfy.filter_update(
                    filtered_state_mean = initial_state_y,
                    filtered_state_covariance = initial_covariance_y,
                    observation = y
                )
                initial_state_y = filtered_state_mean_y
                initial_covariance_y = filtered_state_covariance_y
                y = filtered_state_mean_y.flatten()[0]

                speedXPixels = x #X speed in pixels
                speedYPixels = y #Y speed in pixels
                speedXMgsd = speedXPixels * gsd / (measurementTime - lastMeasurementTime)
                speedXM = speedXPixels / (16 / (4 * 6) * 1000) * altitude * -3 / (measurementTime - lastMeasurementTime)
                speedYMgsd = speedYPixels * gsd / (measurementTime - lastMeasurementTime)
                speedYM = speedYPixels / (16 / (4 * 6) * 1000) * altitude * -3.25 / (measurementTime - lastMeasurementTime)
                
                """
                plotSpeedX.append(count_time)
                plotSpeedYX.append(speedXM)
                plotSpeedYY.append(speedYM)

                ax.plot(plotSpeedX, plotSpeedYX, color = "blue")
                ax.set_xlabel("Time")
                ax.set_ylabel("SpeedX", color = "blue")
                ax2.plot(plotSpeedX, plotSpeedYY, color = "red")
                ax2.set_ylabel("SpeedY", color = "red")
                fig.tight_layout()
                fig.canvas.draw()
                img = cv.cvtColor(np.fromstring(fig.canvas.tostring_rgb(), dtype = np.uint8, sep = "").reshape(fig.canvas.get_width_height()[::-1] + (3,)), cv.COLOR_RGB2BGR)
                data_as = bridge.cv2_to_imgmsg(img, "bgr8")
                data_as.header.frame_id = "0"
                foo_pub.publish(data_as)
                """

                
                data_as = f'X: {round(speedXMgsd, 3)}  Y: {round(speedYMgsd, 3)}'
                sys.stdout.write("\r" + "GSD  " + "X: " + "%.3f" % round(speedXMgsd, 3) + " Y: " + "%.3f" % round(speedYMgsd, 3) + "    FORM  " + "X: " + "%.3f" % round(speedXM, 3) + " Y: " + "%.3f" % round(speedYM, 3) + "    ALT  " + "%.3f" % round(altitude, 3) + " " * 4)
                sys.stdout.flush()
                foo_pub1.publish(data_as)
                
                
            
            # distance measurement (prints distance between sensor pollings)
            elif sys.argv[1] == "distance":
                x, y = px4.update()[1:3]

                filtered_state_mean_x, filtered_state_covariance_x = kfx.filter_update(
                    filtered_state_mean = initial_state_x,
                    filtered_state_covariance = initial_covariance_x,
                    observation = x
                )
                initial_state_x = filtered_state_mean_x
                initial_covariance_x = filtered_state_covariance_x
                x = filtered_state_mean_x.flatten()[0]

                filtered_state_mean_y, filtered_state_covariance_y = kfy.filter_update(
                    filtered_state_mean = initial_state_y,
                    filtered_state_covariance = initial_covariance_y,
                    observation = y
                )
                initial_state_y = filtered_state_mean_y
                initial_covariance_y = filtered_state_covariance_y
                y = filtered_state_mean_y.flatten()[0]

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
                print("Высота:", altitude)
                plotDistanceX.append(count_time)
                plotDistanceYX.append(distanceX)
                plotDistanceYY.append(distanceY)

            else:
                sys.exit()

            count_time += (measurementTime - lastMeasurementTime) #This is the counter of the interval of polling the rangefinder sensor
            sleep(1 / measureSpeed)