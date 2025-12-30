import lgpio
import time


class HX711:
    def __init__(self, dout, pd_sck, gain=128):
        self.DOUT = dout
        self.PD_SCK = pd_sck

        self.chip = lgpio.gpiochip_open(4)

        lgpio.gpio_claim_input(self.chip, self.DOUT)
        lgpio.gpio_claim_output(self.chip, self.PD_SCK)

        self.GAIN = 1 if gain == 128 else 3 if gain == 64 else 2

        self.OFFSET = 0
        self.REFERENCE_UNIT = 1

        self.set_gain(gain)

    def is_ready(self):
        return lgpio.gpio_read(self.chip, self.DOUT) == 0

    def set_gain(self, gain):
        if gain == 128:
            self.GAIN = 1
        elif gain == 64:
            self.GAIN = 3
        elif gain == 32:
            self.GAIN = 2

        self.read_raw()

    def read_raw(self):
        while not self.is_ready():
            pass

        data = 0
        for _ in range(24):
            lgpio.gpio_write(self.chip, self.PD_SCK, 1)
            data = (data << 1) | lgpio.gpio_read(self.chip, self.DOUT)
            lgpio.gpio_write(self.chip, self.PD_SCK, 0)

        for _ in range(self.GAIN):
            lgpio.gpio_write(self.chip, self.PD_SCK, 1)
            lgpio.gpio_write(self.chip, self.PD_SCK, 0)

        if data & 0x800000:
            data -= 0x1000000

        return data

    def set_offset(self, offset):
        self.OFFSET = offset

    def set_reference_unit(self, value):
        self.REFERENCE_UNIT = value

    def get_value(self, times=5):
        return sum(self.read_raw() for _ in range(times)) / times

    def get_weight(self, times=5):
        value = self.get_value(times) - self.OFFSET
        return value / self.REFERENCE_UNIT

    def power_down(self):
        lgpio.gpio_write(self.chip, self.PD_SCK, 0)
        lgpio.gpio_write(self.chip, self.PD_SCK, 1)
        time.sleep(0.0001)

    def power_up(self):
        lgpio.gpio_write(self.chip, self.PD_SCK, 0)
        time.sleep(0.0001)
