import numpy as np


class PIDController:
    """
    Industrial-style PID Controller

    Features
    --------
    ✔ Anti-Windup
    ✔ Output Saturation
    ✔ Derivative Filtering
    """

    def __init__(
        self,
        kp,
        ki,
        kd,
        dt,
        output_limits=(-np.inf, np.inf),
        derivative_filter=0.15
    ):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.dt = dt

        self.min_output = output_limits[0]
        self.max_output = output_limits[1]

        self.filter = derivative_filter

        self.integral = 0

        self.prev_error = 0

        self.filtered_derivative = 0

    def reset(self):

        self.integral = 0

        self.prev_error = 0

        self.filtered_derivative = 0

    def update(self, setpoint, measurement):

        error = setpoint - measurement

        proportional = self.kp * error

        derivative = (error - self.prev_error) / self.dt

        self.filtered_derivative = (

            self.filter * self.filtered_derivative

            + (1 - self.filter) * derivative

        )

        derivative_term = self.kd * self.filtered_derivative

        predicted = (

            proportional

            + derivative_term

            + self.ki * (self.integral + error * self.dt)

        )

        if self.min_output <= predicted <= self.max_output:

            self.integral += error * self.dt

        integral_term = self.ki * self.integral

        output = proportional + integral_term + derivative_term

        output = np.clip(

            output,

            self.min_output,

            self.max_output

        )

        self.prev_error = error

        return output
