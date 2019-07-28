import numpy as np

class DroneController:

    def __init__(self, drone, gains):

        self.gains = gains
        self.drone = drone

    def get_torques(self, desired_velocity):
        err = desired_velocity - self.drone.velocity_body
        kx = self.gains[0]
        ky = self.gains[1]
        kz = self.gains[2]

        z_ctl = kz*err[2]

        return np.array([
            z_ctl - (kx*err[0]),
            z_ctl - (ky*err[1]),
            z_ctl + (kx*err[0]),
            z_ctl + (ky*err[1]),
        ])