import adafruit_vl53l1x
import board


class VL53L1X:
    def __init__(self, distanceMode = 2):
        self.i2c = board.I2C()
        self.BUS = adafruit_vl53l1x.VL53L1X(self.i2c)
        self.BUS.distance_mode = distanceMode
        self.BUS.timing_budget = 50
        self.distance = 0

    def get_distance(self):
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