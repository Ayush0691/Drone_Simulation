import numpy as np

from quadcopter import Quadcopter
from trajectory import TrajectoryGenerator

from controllers import CascadedController
from sensors import SensorSuite
from sensor_fusion import ComplementaryFilter

from visualization import (
    plot_altitude,
    plot_trajectory_3d,
    plot_motor_speed,
    plot_position_error,
)

from metrics import (
    rmse,
    overshoot,
    rise_time,
)


def main():

    dt = 0.01
    sim_time = 20.0

    time = np.arange(0, sim_time, dt)

    quad = Quadcopter()

    trajectory = TrajectoryGenerator()

    controller = CascadedController(dt)

    sensors = SensorSuite()

    fusion = ComplementaryFilter()

    altitude_history = []

    position_history = []

    desired_history = []

    motor_history = []

    position_error = []

    roll_history = []

    pitch_history = []

    print("Starting Simulation...\n")

    for t in time:

        #######################################################
        # Desired Flight Path
        #######################################################

        desired = trajectory.figure8(
            t,
            radius=5,
            altitude=10,
            omega=0.30,
        )

        desired_history.append(desired.copy())

        #######################################################
        # Read Sensors
        #######################################################

        gps = sensors.gps(quad)

        gyro = sensors.gyro(quad)

        accel = sensors.accelerometer(quad)

        #######################################################
        # Sensor Fusion
        #######################################################

        roll_est, pitch_est = fusion.update(
            gyro,
            accel,
            dt,
        )

        roll_history.append(roll_est)

        pitch_history.append(pitch_est)

        #######################################################
        # Cascaded Controller
        #######################################################

        thrust, roll_cmd, pitch_cmd, yaw_cmd = controller.update(
            desired,
            gps,
            quad,
        )

        #######################################################
        # Motor Mixer
        #######################################################

        base_speed = np.sqrt(
            max(thrust, 0.0)
            / (4 * quad.b)
        )

        motors = np.array(
            [
                base_speed - roll_cmd + pitch_cmd + yaw_cmd,
                base_speed + roll_cmd + pitch_cmd - yaw_cmd,
                base_speed + roll_cmd - pitch_cmd + yaw_cmd,
                base_speed - roll_cmd - pitch_cmd - yaw_cmd,
            ]
        )

        motors = np.clip(
            motors,
            0,
            2500,
        )

        motor_history.append(np.mean(motors))

        #######################################################
        # Wind Gust
        #######################################################

        if 4 <= t <= 6:

            wind = np.array(
                [0, 0, -3]
            )

        else:

            wind = np.zeros(3)

        #######################################################
        # Physics Update
        #######################################################

        quad.update(
            motors,
            external=wind,
            dt=dt,
        )

        #######################################################
        # Logging
        #######################################################

        current_position = quad.state[:3].copy()

        position_history.append(current_position)

        altitude_history.append(current_position[2])

        error = np.linalg.norm(
            desired - current_position
        )

        position_error.append(error)

    print("Simulation Complete.\n")

    ###########################################################
    # Plots
    ###########################################################

    plot_altitude(
        time,
        altitude_history,
        10,
    )

    plot_motor_speed(
        time,
        motor_history,
    )

    plot_position_error(
        time,
        position_error,
    )

    plot_trajectory_3d(
        position_history,
        desired_history,
    )

    ###########################################################
    # Performance Metrics
    ###########################################################

    desired_altitude = np.full(
        len(time),
        10,
    )

    altitude_rmse = rmse(
        desired_altitude,
        altitude_history,
    )

    position_rmse = rmse(
        np.array(desired_history),
        np.array(position_history),
    )

    altitude_overshoot = overshoot(
        desired_altitude,
        altitude_history,
    )

    altitude_rise = rise_time(
        time,
        altitude_history,
        10,
    )

    print("===================================")
    print("Drone Flight Performance")
    print("===================================")

    print(f"Altitude RMSE      : {altitude_rmse:.3f} m")

    print(f"Position RMSE      : {position_rmse:.3f} m")

    print(f"Overshoot          : {altitude_overshoot:.2f}%")

    print(f"Rise Time          : {altitude_rise:.2f} s")

    print(f"Maximum Altitude   : {max(altitude_history):.2f} m")

    print(f"Average Motor RPM  : {np.mean(motor_history):.2f}")

    print("===================================")


if __name__ == "__main__":

    main()
