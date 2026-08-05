import numpy as np

from pid_controller import PIDController


class CascadedController:

    """
    Position PID
        ↓
    Attitude PID
        ↓
    Motor Mixer
    """

    def __init__(self, dt):

        self.dt = dt

        self.altitude = PIDController(
            8.5, 2.1, 4.8, dt,
            output_limits=(0,25)
        )

        self.roll = PIDController(
            4.0,0.0,1.2,dt,
            output_limits=(-5,5)
        )

        self.pitch = PIDController(
            4.0,0.0,1.2,dt,
            output_limits=(-5,5)
        )

        self.yaw = PIDController(
            2.0,0.0,0.4,dt,
            output_limits=(-2,2)
        )

    def update(self,target,current,quad):

        x,y,z=current[0:3]

        phi,theta,psi=quad.state[6:9]

        thrust=self.altitude.update(
            target[2],
            z
        )

        roll_cmd=self.roll.update(
            target[1],
            y
        )

        pitch_cmd=self.pitch.update(
            target[0],
            x
        )

        yaw_cmd=self.yaw.update(
            0,
            psi
        )

        return thrust,roll_cmd,pitch_cmd,yaw_cmd
