import csv
import time
import os
import datetime


class Logger:
    def __init__(self):
        self.headers = [
            "time",
            "INT: X",
            "INT: Y",
            "INT: TOTAL",
            "GSD: X",
            "GSD: Y",
            "GSD: TOTAL",
            "FORM: X",
            "FORM: Y",
            "FORM: TOTAL",
            "GPS: X",
            "GPS: Y",
            "GPS: TOTAL",
            "DIFF GPS/GSD: X",
            "DIFF GPS/GSD: Y",
            "DIFF GPS/GSD: TOTAL",
            "DIFF GPS/FORM: X",
            "DIFF GPS/FORM: Y",
            "DIFF GPS/FORM: TOTAL",
            "ALT",
        ]
        try:
            filename = str(int(list(filter(lambda x: x.endswith(".csv"), os.listdir()))[-1][:-4]) + 1)
        except:
            filename = "0"
        self.file = open(f"/home/pi/{filename}.csv", mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.headers)
        self.file.flush()
        os.fsync(self.file.fileno())

    def log(self, data):
        data.insert(0, self.timestamp())
        self.writer.writerow(data)
        self.file.flush()
        os.fsync(self.file.fileno())
        data = []

    def time_now(self):
        return str(time.time()).split(".")[0]

    def timestamp(self):
        local = datetime.datetime.now().isoformat(sep=" ")[11:-3]
        return local
