import RPi.GPIO as gpio
from pykalman import KalmanFilter
from time import time, sleep


class HCSR04:
    def __init__(self, trig = 20, echo = 16):
        self.distance = 0
        self.trig = trig
        self.echo = echo
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)

        self.kfd = KalmanFilter(transition_matrices=[1],
                  observation_matrices=[1],
                  initial_state_mean=0,
                  initial_state_covariance=1,
                  observation_covariance=1,
                  transition_covariance=0.03)
        self.initial_state_d = 0
        self.initial_covariance_d = 1

    def get_distance(self):
        gpio.output(self.trig, gpio.HIGH)
        sleep(0.00001)
        gpio.output(self.trig, gpio.LOW)
        trigStart = time()
        while gpio.input(self.echo) == gpio.LOW:
            start = time()
            if start - trigStart >= 0.07:
                return self.distance * 10 ** -2
        while gpio.input(self.echo) == gpio.HIGH:
            stop = time()
        self.distance = (stop - start) * 17150
        return self.distance * 10 ** -2
    
    def get_distance_filtered(self):
        filtered_state_mean_d, filtered_state_covariance_d = self.kfd.filter_update(
            filtered_state_mean = self.initial_state_d,
            filtered_state_covariance = self.initial_covariance_d,
            observation = self.get_distance()
        )
        self.initial_state_d = filtered_state_mean_d
        self.initial_covariance_d = filtered_state_covariance_d
        distance = filtered_state_mean_d.flatten()[0]
        return distance