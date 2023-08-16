from serial import Serial
from kf import kf


class TF03:
    def __init__(self, port="/dev/ttyAMA0"):
        self.BUS = Serial(port, 115200, timeout=0.05)
        self.kfd = kf()

        self.distance = 0
    
    def get_distance(self):
        self.BUS.reset_input_buffer()
        self.BUS.read_until(b"\x59")
        data = self.BUS.read(8)

        if len(data) < 8:
            return self.distance * 10 ** -2

        if data[3] + data[4] * 2 ** 8 < 40: #bad data
            return self.distance * 10 ** -2
        self.distance = data[1] + data[2] * 2 ** 8
        return self.distance * 10 ** -2

    def get_distance_filtered(self):
        return self.kfd.filter_value(self.get_distance())