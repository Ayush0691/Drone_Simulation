import numpy as np


class ObstacleAvoidance:

    def __init__(self):

        self.center=np.array([2.5,2.5,10])

        self.radius=1.2

    def modify(self,target,current):

        vec=current-self.center

        dist=np.linalg.norm(vec)

        if dist<self.radius:

            vec=vec/(dist+1e-6)

            target=current+vec*2

        return target
