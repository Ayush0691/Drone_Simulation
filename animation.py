import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

import numpy as np


def animate(history):

    history = np.array(history)

    fig = plt.figure()

    ax = fig.add_subplot(

        111,

        projection="3d",

    )

    ax.set_xlim(-6,6)

    ax.set_ylim(-6,6)

    ax.set_zlim(0,12)

    drone, = ax.plot(

        [],

        [],

        [],

        'bo',

        markersize=6,

    )

    def update(i):

        drone.set_data(

            history[i,0],

            history[i,1],

        )

        drone.set_3d_properties(

            history[i,2]

        )

        return drone,

    ani = FuncAnimation(

        fig,

        update,

        frames=len(history),

        interval=15,

        blit=True,

    )

    plt.show()
