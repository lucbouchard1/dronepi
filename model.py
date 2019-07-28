import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

class Drone:

    def __init__(self, time, position = np.zeros(3), principal_moments = np.ones(3),
            arm_length = 1, motor_torque_thrust_const = 1, mass = 1):

        self.time = 0
        self.ang_vel = np.zeros(3)
        self.orientation = Rotation.from_quat([0, 0, 0, 1])
        self.vel = np.zeros(3)
        self.motor_torques = np.zeros(3)
        self.position = position
        self.a = arm_length
        self.alpha = motor_torque_thrust_const
        self.principal_moments = principal_moments
        self.mass = 1

    def make_state_vector(self):

        quat = self.orientation.as_quat()

        return np.array(
            [
             self.motor_torques[0], self.motor_torques[1], self.motor_torques[2], self.motor_torques[3],
             self.ang_vel[0], self.ang_vel[1], self.ang_vel[2],
             quat[0], quat[1], quat[2], quat[3],
             self.vel[0], self.vel[1], self.vel[2]
            ]
        )

    def save_state_vector(self, state):

        self.motor_torques = state[0:4]
        self.ang_vel = state[4:7]
        self.orientation.from_quat(state[7:11])
        self.vel = state[11:]

    def step(self, time_step, motor_torques):

        self.motor_torques = motor_torques
        self.position = self.position + (self.vel * time_step)

        def state_deriv(t, x):
            c = self.a * self.alpha
            ang_vel_mat = np.array([
                [    0,  x[6], -x[5], x[4]]
                [-x[6],     0,  x[4], x[5]]
                [ x[5], -x[4],     0, x[6]]
                [-x[4], -x[5], -x[6],    0]
            ])
            orientation = Rotation(x[7:11])
            net_force = orientation.apply([0, 0, self.alpha * np.sum(x[0:4])])
            return np.array([
                0, 0, 0, 0,                                               # Motor torque derivatives
                (x[1] - x[3])*c/self.principal_moments[0],                # Angular accel x
                (x[2] - x[0])*c/self.principal_moments[1],                # Angular accel y
                (x[0] + x[2] - x[1] - x[3])/self.principal_moments[2]     # Angular accel z
                ] + (0.5 * ang_vel_mat @ orientation.as_quat())           # Quaternion derivative
                  + net_force / self.mass                                 # Acceleration
            )

        result = solve_ivp(state_deriv, (self.time, self.time + time_step), self.make_state_vector())
        self.save_state_vector(result.y)
        self.time += time_step
    

