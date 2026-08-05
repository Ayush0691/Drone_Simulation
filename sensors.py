import numpy as np


class SensorSuite:

    def __init__(self):

        self.gyro_std=0.01

        self.accel_std=0.05

        self.gps_std=0.08

    def gyro(self,quad):

        true=quad.state[9:12]

        noise=np.random.normal(

            0,

            self.gyro_std,

            3

        )

        return true+noise

    def accelerometer(self,quad):

        accel=np.array([

            0,

            0,

            9.81

        ])

        noise=np.random.normal(

            0,

            self.accel_std,

            3

        )

        return accel+noise

    def gps(self,quad):

        pos=quad.state[:3]

        noise=np.random.normal(

            0,

            self.gps_std,

            3

        )

        return pos+noise
