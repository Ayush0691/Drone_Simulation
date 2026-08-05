import numpy as np


class Battery:

    """
    Simple LiPo Battery Model
    """

    def __init__(

        self,

        voltage=16.8,

        capacity=5200,

        internal_resistance=0.04,

    ):

        self.max_voltage = voltage

        self.voltage = voltage

        self.capacity = capacity

        self.remaining = capacity

        self.internal_resistance = internal_resistance

    def update(

        self,

        current,

        dt,

    ):

        used = (current * dt) / 3600 * 1000

        self.remaining -= used

        self.remaining = max(0, self.remaining)

        soc = self.remaining / self.capacity

        self.voltage = (

            13.2

            + 3.6 * soc

            - current * self.internal_resistance

        )

        return self.voltage

    def percentage(self):

        return 100 * self.remaining / self.capacity
