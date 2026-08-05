import numpy as np


class Quadcopter:

    """
    Simple 6DOF Quadcopter Physics Engine
    """

    def __init__(self):

        self.mass = 1.2

        self.gravity = 9.81

        self.arm = 0.22

        self.b = 3e-6

        self.d = 1e-7

        self.I = np.diag([0.015, 0.015, 0.03])

        self.I_inv = np.linalg.inv(self.I)

        self.state = np.zeros(12)

    def rotation_matrix(self):

        phi = self.state[6]

        theta = self.state[7]

        psi = self.state[8]

        Rx = np.array([

            [1,0,0],

            [0,np.cos(phi),-np.sin(phi)],

            [0,np.sin(phi),np.cos(phi)]

        ])

        Ry = np.array([

            [np.cos(theta),0,np.sin(theta)],

            [0,1,0],

            [-np.sin(theta),0,np.cos(theta)]

        ])

        Rz = np.array([

            [np.cos(psi),-np.sin(psi),0],

            [np.sin(psi),np.cos(psi),0],

            [0,0,1]

        ])

        return Rz @ Ry @ Rx

    def update(

        self,

        motor_speed,

        external=np.zeros(3),

        dt=0.01

    ):

        x,y,z,vx,vy,vz,phi,theta,psi,p,q,r = self.state

        motor_speed = np.array(motor_speed)

        thrust = self.b * motor_speed**2

        total = np.sum(thrust)

        R = self.rotation_matrix()

        accel = (

            R @ np.array([0,0,total])

            + np.array([0,0,-self.mass*self.gravity])

            -0.08*np.array([vx,vy,vz])

            + external

        )/self.mass

        torque = np.array([

            self.arm*self.b*(motor_speed[3]**2-motor_speed[1]**2),

            self.arm*self.b*(motor_speed[2]**2-motor_speed[0]**2),

            self.d*(motor_speed[0]**2-motor_speed[1]**2+

                    motor_speed[2]**2-motor_speed[3]**2)

        ])

        omega = np.array([p,q,r])

        omega_dot = self.I_inv @ (

            torque -

            np.cross(

                omega,

                self.I @ omega

            )

        )

        self.state[0:3] += self.state[3:6]*dt

        self.state[3:6] += accel*dt

        self.state[6:9] += self.state[9:12]*dt

        self.state[9:12] += omega_dot*dt

        return self.state
