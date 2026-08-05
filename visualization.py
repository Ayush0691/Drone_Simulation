import matplotlib.pyplot as plt
import numpy as np


def plot_altitude(time, altitude, target):

    plt.figure(figsize=(10,5))

    plt.plot(time, altitude, label="Altitude")

    plt.axhline(target, linestyle="--", color="red", label="Target")

    plt.xlabel("Time (s)")

    plt.ylabel("Altitude (m)")

    plt.title("Altitude Response")

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.show()


def plot_motor_speed(time, motor_speed):

    plt.figure(figsize=(10,5))

    plt.plot(time, motor_speed)

    plt.xlabel("Time (s)")

    plt.ylabel("Motor Speed (rad/s)")

    plt.title("Motor Speed")

    plt.grid(True)

    plt.tight_layout()

    plt.show()


def plot_trajectory_3d(history, desired=None):

    history = np.array(history)

    fig = plt.figure(figsize=(8,7))

    ax = fig.add_subplot(111, projection="3d")

    ax.plot(

        history[:,0],

        history[:,1],

        history[:,2],

        linewidth=2,

        label="Drone"

    )

    if desired is not None:

        desired = np.array(desired)

        ax.plot(

            desired[:,0],

            desired[:,1],

            desired[:,2],

            '--',

            linewidth=2,

            label="Desired"

        )

    ax.set_xlabel("X")

    ax.set_ylabel("Y")

    ax.set_zlabel("Z")

    ax.set_title("3D Flight Path")

    ax.legend()

    plt.show()


def plot_position_error(time, error):

    plt.figure(figsize=(10,5))

    plt.plot(time, error)

    plt.xlabel("Time (s)")

    plt.ylabel("Error (m)")

    plt.title("Position Error")

    plt.grid(True)

    plt.tight_layout()

    plt.show()
