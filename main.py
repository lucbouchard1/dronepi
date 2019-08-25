import model, controller
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from ctypes import Structure, c_uint32, c_int16, c_uint8
import matplotlib.pyplot as plt
import ctypes, os

torques = np.zeros(4)
cmd_vel = np.zeros(3)
drone = model.Drone(time = 0)
gains = np.array([3, 0.2, 0.05])
controller = controller.DroneController(drone, x_gains=0.1*gains,
        y_gains=0.1*gains, z_gains=[0.2, 0.4, 1])

fig = plt.figure(figsize=(15, 9))
ax = p3.Axes3D(fig)

def get_plottable(drone):
    m0 = drone.motor_pos(0)
    m2 = drone.motor_pos(2)
    m1 = drone.motor_pos(1)
    m3 = drone.motor_pos(3)
    return (([m0[0], m2[0]], [m0[1], m2[1]], [m0[2], m2[2]]),
                ([m1[0], m3[0]], [m1[1], m3[1]], [m1[2], m3[2]]))

def update_drone(time, cmd_vel, drone, controller, arm_lines):
    global torques
    drone.step(time, torques)
    torques = controller.get_torques(cmd_vel)
    arm1, arm2 = get_plottable(drone)
    arm_lines[0].set_data(arm1[0:2])
    arm_lines[0].set_3d_properties(arm1[2])
    arm_lines[1].set_data(arm2[0:2])
    arm_lines[1].set_3d_properties(arm2[2])
    return arm_lines

arm1, arm2 = get_plottable(drone)
line1, = ax.plot(*arm1, 'o-', color='blue')
line2, = ax.plot(*arm2, 'o-', color='blue')

ax.view_init(elev=0, azim=90)
ax.set_xlim3d((-10, 20))
ax.set_ylim3d((-10, 10))
ax.set_zlim3d((-10, 10))

class JoystickEvent(Structure):
    _fields_ = [("time", c_uint32),
                ("value", c_int16),
                ("type", c_uint8),
                ("number", c_uint8)]

current_time = 0.1
TIME_STEP = 0.05
COMPUTE_TIME = 0.0003

fd = os.open("/dev/input/js0", os.O_RDONLY | os.O_NONBLOCK)
update_drone(current_time, cmd_vel, drone, controller, [line1, line2])
while plt.get_fignums():
    plt.pause(TIME_STEP - COMPUTE_TIME)
    current_time += TIME_STEP
    update_drone(current_time, cmd_vel, drone, controller, [line1, line2])
    done = 0
    while done == 0:
        try:
            data = os.read(fd, ctypes.sizeof(JoystickEvent))
        except:
            done = 1
            continue
        event = JoystickEvent.from_buffer_copy(data)
        if (event.number == 0):
            cmd_vel[0] = -1*event.value/3000
        if (event.number == 1):
            cmd_vel[1] = event.value/3000
        if (event.number == 0 and event.type == 1):
            if (event.value <= 1):
                cmd_vel[2] = -1*event.value*2
        if (event.number == 2 and event.type == 1):
            if (event.value <= 1):
                cmd_vel[2] = event.value*2
    