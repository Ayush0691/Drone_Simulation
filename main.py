import numpy as np
import matplotlib.pyplot as plt

from pid_controller import PIDController
from quadcopter import Quadcopter
from trajectory import TrajectoryGenerator
from visualization import (
    plot_altitude,
    plot_trajectory_3d,
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

    traj = TrajectoryGenerator()

    altitude_pid = PIDController(
        kp=8.5,
        ki=2.1,
        kd=4.8,
        dt=dt,
        output_limits=(0, 25)
    )

    altitude_history = []

    position_history = []

    desired_history = []

    motor_speed_history = []

    target_altitude = 10.0

    for t in time:

        # Desired trajectory
        desired_position = traj.figure8(
            t,
            radius=5,
            altitude=target_altitude,
            omega=0.30
        )

        desired_history.append(desired_position)

        # Current state
        current_position = quad.state[0:3]

        current_altitude = current_position[2]

        # Altitude controller
        thrust = altitude_pid.update(
            target_altitude,
            current_altitude
        )

        # Convert thrust to motor speed
        motor_speed = np.sqrt(
            max(thrust, 0) /
            (4 * quad.b)
        )

        motors = np.array([
            motor_speed,
            motor_speed,
            motor_speed,
            motor_speed
        ])

        motor_speed_history.append(motor_speed)

        # Wind disturbance
        if 4 <= t <= 6:
            wind = np.array([0.0, 0.0, -3.0])
        else:
            wind = np.zeros(3)

        # Update quadcopter
        quad.update(
            motors,
            external=wind,
            dt=dt
        )

        altitude_history.append(
            quad.state[2]
        )

        position_history.append(
            quad.state[:3].copy()
        )

    # -----------------------------
    # Visualization
    # -----------------------------

    plot_altitude(
        time,
        altitude_history,
        target_altitude
    )

    plot_trajectory_3d(
        position_history,
        desired_history
    )

    # -----------------------------
    # Performance Metrics
    # -----------------------------

    desired_altitude = np.full(
        len(time),
        target_altitude
    )

    altitude_rmse = rmse(
        desired_altitude,
        altitude_history
    )

    altitude_overshoot = overshoot(
        desired_altitude,
        altitude_history
    )

    altitude_rise_time = rise_time(
        time,
        altitude_history,
        target_altitude
    )

    print("\n========== PERFORMANCE ==========")

    print(f"RMSE           : {altitude_rmse:.4f} m")

    print(f"Overshoot      : {altitude_overshoot:.2f} %")

    print(f"Rise Time      : {altitude_rise_time:.3f} s")

    print("=================================\n")


if __name__ == "__main__":
    main()
