import numpy as np


class TrajectoryGenerator:
    """
    Generates different waypoint trajectories.
    """

    def __init__(self):

        pass

    def hover(self, t):

        return np.array([0.0, 0.0, 10.0])

    def circle(self, t, radius=5, altitude=10, omega=0.3):

        x = radius * np.cos(omega * t)

        y = radius * np.sin(omega * t)

        z = altitude

        return np.array([x, y, z])

    def figure8(self, t, radius=5, altitude=10, omega=0.3):

        x = radius * np.sin(omega * t)

        y = radius * np.sin(2 * omega * t) / 2

        z = altitude

        return np.array([x, y, z])

    def spiral(self, t, radius=0.3, altitude_rate=0.05):

        x = radius * t * np.cos(0.4 * t)

        y = radius * t * np.sin(0.4 * t)

        z = altitude_rate * t

        return np.array([x, y, z])
