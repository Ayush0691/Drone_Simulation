import numpy as np


class MissionPlanner:

    def __init__(self):

        self.points=[

            np.array([0,0,10]),

            np.array([5,0,10]),

            np.array([5,5,10]),

            np.array([0,5,10]),

            np.array([0,0,10])

        ]

        self.current=0

        self.threshold=0.5

    def update(self,current_pos):

        target=self.points[self.current]

        if np.linalg.norm(

            target-current_pos

        )<self.threshold:

            self.current=min(

                self.current+1,

                len(self.points)-1

            )

        return self.points[self.current]
