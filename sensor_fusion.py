import numpy as np


class ComplementaryFilter:

    def __init__(self,alpha=0.98):

        self.alpha=alpha

        self.roll=0

        self.pitch=0

    def update(

        self,

        gyro,

        accel,

        dt

    ):

        gx,gy,gz=gyro

        ax,ay,az=accel

        accel_roll=np.arctan2(

            ay,

            az

        )

        accel_pitch=np.arctan2(

            -ax,

            np.sqrt(

                ay**2+az**2

            )

        )

        self.roll=self.alpha*(

            self.roll+gx*dt

        )+(1-self.alpha)*accel_roll

        self.pitch=self.alpha*(

            self.pitch+gy*dt

        )+(1-self.alpha)*accel_pitch

        return self.roll,self.pitch
