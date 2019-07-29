import numpy as np
from utils import PIDError

class DroneController:

    def __init__(self, drone, x_gains, y_gains, z_gains):

        self.drone = drone
        self.x_gains = x_gains
        self.y_gains = y_gains
        self.z_gains = z_gains
        self.x_error = PIDError(0, drone.time)
        self.y_error = PIDError(0, drone.time)
        self.z_error = PIDError(0, drone.time)

    def get_ctrl_val(self, error, pid_error, gains):

        d, p, i = pid_error.get_pid(error, self.drone.time)
        return gains[0]*d + gains[1]*p + gains[2]*i


    def get_torques(self, desired_velocity):
        err = desired_velocity - self.drone.velocity_body

        x_ctl = self.get_ctrl_val(err[0], self.x_error, self.x_gains)
        y_ctl = self.get_ctrl_val(err[1], self.y_error, self.y_gains)
        z_ctl = self.get_ctrl_val(err[2], self.z_error, self.z_gains)

        return np.array([
            z_ctl - x_ctl,
            z_ctl - y_ctl,
            z_ctl + x_ctl,
            z_ctl + y_ctl,
        ])