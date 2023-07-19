import RPi.GPIO as gpio
from kf import kf
from time import time, sleep


class HCSR04:
    def __init__(self, trig=20, echo=16):
        self.trig = trig
        self.echo = echo
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)
        self.kfd = kf()

        self.distance = 0

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
        return self.kfd.filter_value(self.get_distance())