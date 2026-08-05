import numpy as np


class Motor:

    """
    ESC + Brushless Motor Model
    """

    def __init__(

        self,

        tau=0.05,

        max_speed=2500,

    ):

        self.speed = 0

        self.max_speed = max_speed

        self.tau = tau

    def update(

        self,

        command,

        dt,

    ):

        command = np.clip(

            command,

            0,

            self.max_speed,

        )

        self.speed += (

            command - self.speed

        ) * dt / self.tau

        return self.speed
