import numpy as np
import matplotlib.pyplot as plt

from pid_controller import PIDController
from quadcopter import Quadcopter


quad = Quadcopter()

pid = PIDController(

    kp=8.5,

    ki=2.1,

    kd=4.8,

    dt=0.01,

    output_limits=(0,25)

)

dt = 0.01

time = np.arange(0,10,dt)

target = 10

history = []

for t in time:

    altitude = quad.state[2]

    thrust = pid.update(target, altitude)

    speed = np.sqrt(thrust/(4*quad.b))

    motors = np.ones(4)*speed

    wind = np.array([0,0,-3]) if 4<t<6 else np.zeros(3)

    quad.update(motors, wind, dt)

    history.append(quad.state[2])

plt.figure(figsize=(10,5))

plt.plot(time, history)

plt.axhline(target,color='r',linestyle='--')

plt.xlabel("Time (s)")

plt.ylabel("Altitude (m)")

plt.title("Drone Altitude PID Control")

plt.grid()

plt.show()
