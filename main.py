import numpy as np

from quadcopter import Quadcopter
from trajectory import TrajectoryGenerator
from telemetry import Telemetry
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

from battery import Battery
from motor import Motor
from logger import FlightLogger
from animation import animate


def main():

    dt = 0.01
    sim_time = 20.0

    time = np.arange(0, sim_time, dt)

    quad = Quadcopter()

    trajectory = TrajectoryGenerator()

    controller = CascadedController(dt)

    sensors = SensorSuite()

    fusion = ComplementaryFilter()

    battery = Battery()

    logger = FlightLogger("flight_log.csv")

    telemetry = Telemetry(
        save_csv=True,
        filename="telemetry.csv",
    )

    motor_objects = [
        Motor(),
        Motor(),
        Motor(),
        Motor()
    ]

    altitude_history = []

    position_history = []

    desired_history = []

    motor_history = []

    battery_history = []

    position_error = []

    roll_history = []

    pitch_history = []

    print("========== Drone Simulation ==========")

    for t in time:

        #################################################
        # Desired Trajectory
        #################################################

        desired = trajectory.figure8(
            t,
            radius=5,
            altitude=10,
            omega=0.30,
        )

        desired_history.append(desired.copy())

        #################################################
        # Sensor Readings
        #################################################

        gps = sensors.gps(quad)

        gyro = sensors.gyro(quad)

        accel = sensors.accelerometer(quad)

        #################################################
        # Complementary Filter
        #################################################

        roll_est, pitch_est = fusion.update(
            gyro,
            accel,
            dt,
        )

        roll_history.append(roll_est)

        pitch_history.append(pitch_est)

        #################################################
        # Flight Controller
        #################################################

        thrust, roll_cmd, pitch_cmd, yaw_cmd = controller.update(
            desired,
            gps,
            quad,
        )

        #################################################
        # Motor Mixer
        #################################################

        base_speed = np.sqrt(
            max(thrust, 0.0) /
            (4 * quad.b)
        )

        motor_commands = np.array([

            base_speed - roll_cmd + pitch_cmd + yaw_cmd,

            base_speed + roll_cmd + pitch_cmd - yaw_cmd,

            base_speed + roll_cmd - pitch_cmd + yaw_cmd,

            base_speed - roll_cmd - pitch_cmd - yaw_cmd,

        ])

        motor_commands = np.clip(
            motor_commands,
            0,
            2500,
        )

        #################################################
        # ESC + Motor Dynamics
        #################################################

        motor_speed = np.zeros(4)

        for i in range(4):

            motor_speed[i] = motor_objects[i].update(
                motor_commands[i],
                dt,
            )

        motor_history.append(
            np.mean(motor_speed)
        )

        #################################################
        # Battery Simulation
        #################################################

        estimated_current = np.mean(
            motor_speed
        ) / 100

        voltage = battery.update(
            estimated_current,
            dt,
        )

        battery_history.append(voltage)

        #################################################
        # Wind Gust
        #################################################

        if 4 <= t <= 6:

            wind = np.array([

                0,

                0,

                -3

            ])

        else:

            wind = np.zeros(3)

        #################################################
        # Physics Engine
        #################################################

        quad.update(

            motor_speed,

            external=wind,

            dt=dt,

        )

        #################################################
        # Data Logging
        #################################################

        logger.log(

            t,

            quad.state,

            voltage,

            np.mean(motor_speed),

        )

        #################################################
        # Live Telemetry
        #################################################

        if abs(t - round(t)) < dt / 2:

            telemetry.print_status(

                t,

                quad.state,

                battery.voltage,

                battery.percentage(),

                np.mean(motor_speed),

            )
    
        #################################################
        # Store Results
        #################################################

        current_position = quad.state[:3].copy()

        position_history.append(

            current_position

        )

        altitude_history.append(

            current_position[2]

        )

        position_error.append(

            np.linalg.norm(

                desired -

                current_position

            )

        )

    #################################################
    # Finish Logging
    #################################################


    #################################################
    # Visualization
    #################################################

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

    #################################################
    # Animation
    #################################################

    animate(

        position_history

    )

    #################################################
    # Metrics
    #################################################

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

    logger.close()
    #################################################
    # Summary
    #################################################

    #################################################
    # Telemetry Summary
    #################################################

    telemetry.print_summary(

        altitude_rmse,

        position_rmse,

        altitude_overshoot,

        altitude_rise,

        battery,

        sim_time,

    )

    telemetry.close()

print(f"Maximum Altitude     : {max(altitude_history):.2f} m")
print(f"Average Motor Speed  : {np.mean(motor_history):.2f}")
print("Flight Log Saved     : flight_log.csv")
print("Telemetry Saved      : telemetry.csv")

if __name__ == "__main__":
    main()
