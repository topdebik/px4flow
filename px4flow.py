from smbus import SMBus
from c_int import shift8, shift16, shift32


class PX4Flow:
    def __init__(self, bus=1):
        self.ADDRESS = 0x42
        self.BUS = SMBus(bus)

    def update(self):
        self.BUS.write_byte_data(self.ADDRESS, 0x00, 0x0)
        data = list(map(lambda x: hex(x)[2:], self.BUS.read_i2c_block_data(self.ADDRESS, 0x00, 22)))

        frame_count = shift16(int(data[1] + data[0], 16))
        pixel_flow_x_sum = shift16(int(data[3] + data[2], 16))
        pixel_flow_y_sum = shift16(int(data[5] + data[4], 16))
        flow_comp_m_x = shift16(int(data[7] + data[6], 16))
        flow_comp_m_y = shift16(int(data[9] + data[8], 16))
        qual = shift16(int(data[11] + data[10], 16))
        gyro_x_rate = shift16(int(data[13] + data[12], 16))
        gyro_y_rate = shift16(int(data[15] + data[14], 16))
        gyro_z_rate = shift16(int(data[17] + data[16], 16))
        gyro_range = shift8(int(data[18], 16))
        sonar_timestamp = shift8(int(data[19], 16))
        ground_distance = shift16(int(data[21] + data[20], 16))

        return frame_count, pixel_flow_x_sum, pixel_flow_y_sum, flow_comp_m_x, flow_comp_m_y, qual, gyro_x_rate, gyro_y_rate, gyro_z_rate, gyro_range, sonar_timestamp, ground_distance

    def update_integral(self):
        self.BUS.write_byte_data(self.ADDRESS, 0x00, 0x16)
        data = list(map(lambda x: hex(x)[2:], self.BUS.read_i2c_block_data(self.ADDRESS, 0x00, 26)))

        frame_count_since_last_readout = shift16(int(data[1] + data[0], 16))
        pixel_flow_x_integral = shift16(int(data[3] + data[2], 16))
        pixel_flow_y_integral = shift16(int(data[5] + data[4], 16))
        gyro_x_rate_integral = shift16(int(data[7] + data[6], 16))
        gyro_y_rate_integral = shift16(int(data[9] + data[8], 16))
        gyro_z_rate_integral = shift16(int(data[11] + data[10], 16))
        integration_timespan = shift32(int(data[15] + data[14] + data[13] + data[12], 16))
        sonar_timestamp = shift32(int(data[19] + data[18] + data[17] + data[16], 16))
        ground_distance = shift16(int(data[21] + data[20], 16))
        gyro_temperature = shift16(int(data[23] + data[22], 16))
        quality = shift8(int(data[24], 16))

        return frame_count_since_last_readout, pixel_flow_x_integral, pixel_flow_y_integral, gyro_x_rate_integral, gyro_y_rate_integral, gyro_z_rate_integral, integration_timespan, sonar_timestamp, ground_distance, gyro_temperature, quality