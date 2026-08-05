import csv
from datetime import datetime
import numpy as np


class Telemetry:
    """
    Drone Telemetry Module

    Displays:
    - Time
    - Position
    - Velocity
    - Attitude
    - Battery
    - Motor Speed

    Optionally saves telemetry to CSV.
    """

    def __init__(self, save_csv=True, filename="telemetry.csv"):

        self.save_csv = save_csv

        if self.save_csv:

            self.file = open(filename, "w", newline="")

            self.writer = csv.writer(self.file)

            self.writer.writerow([
                "Time",
                "X",
                "Y",
                "Z",
                "VX",
                "VY",
                "VZ",
                "Roll",
                "Pitch",
                "Yaw",
                "Battery(V)",
                "Battery(%)",
                "Motor Speed"
            ])

    def print_status(
        self,
        t,
        state,
        battery_voltage,
        battery_percentage,
        motor_speed,
    ):

        x, y, z = state[0:3]

        vx, vy, vz = state[3:6]

        roll, pitch, yaw = np.degrees(state[6:9])

        print(
            f"[{t:6.2f}s] "
            f"Pos=({x:6.2f}, {y:6.2f}, {z:6.2f}) | "
            f"Vel=({vx:5.2f}, {vy:5.2f}, {vz:5.2f}) | "
            f"Att=({roll:6.2f}°, {pitch:6.2f}°, {yaw:6.2f}°) | "
            f"Battery={battery_voltage:5.2f}V "
            f"({battery_percentage:5.1f}%) | "
            f"Motor={motor_speed:7.1f}"
        )

        if self.save_csv:

            self.writer.writerow([
                round(t, 3),
                x,
                y,
                z,
                vx,
                vy,
                vz,
                roll,
                pitch,
                yaw,
                battery_voltage,
                battery_percentage,
                motor_speed,
            ])

    def print_summary(
        self,
        altitude_rmse,
        position_rmse,
        overshoot,
        rise_time,
        battery,
        flight_time,
    ):

        print("\n")
        print("=" * 70)
        print("                 FLIGHT SUMMARY")
        print("=" * 70)

        print(f"Flight Time           : {flight_time:.2f} sec")
        print(f"Altitude RMSE         : {altitude_rmse:.3f} m")
        print(f"Position RMSE         : {position_rmse:.3f} m")
        print(f"Overshoot             : {overshoot:.2f} %")
        print(f"Rise Time             : {rise_time:.2f} sec")

        print(f"Battery Voltage       : {battery.voltage:.2f} V")
        print(f"Battery Remaining     : {battery.percentage():.2f} %")

        print("=" * 70)

    def close(self):

        if self.save_csv:

            self.file.close()
