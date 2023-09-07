import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge


class graph:
    def __init__(self):
        self.bridge = CvBridge()
        self.fig, self.ax = plt.subplots()
        self.ax2 = self.ax.twinx()
        self.count_x = 0
        self.count_y = 0
        self.count_time = 0
        self.plotSpeedX = []
        self.plotSpeedYX = []
        self.plotSpeedYY = []
        self.plotDistanceX = []
        self.plotDistanceYX = []
        self.plotDistanceYY = []

    def update(self, y1, y2, delta):
        self.count_time += delta
        self.plotSpeedX.append(self.count_time)
        self.plotSpeedYX.append(y1)
        self.plotSpeedYY.append(y2)
        self.ax.plot(self.plotSpeedX, self.plotSpeedYX, color="blue")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("SpeedX", color="blue")
        self.ax2.plot(self.plotSpeedX, self.plotSpeedYY, color="red")
        self.ax2.set_ylabel("SpeedY", color="red")
        self.fig.tight_layout()
        self.fig.canvas.draw()
        img = cv.cvtColor(
            np.fromstring(
                self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep=""
            ).reshape(self.fig.canvas.get_width_height()[::-1] + (3,)),
            cv.COLOR_RGB2BGR,
        )
        data = self.bridge.cv2_to_imgmsg(img, "bgr8")
        data.header.frame_id = "0"
        return data
