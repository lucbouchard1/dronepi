import model, controller
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

torques = np.zeros(4)
last_i = -1
drone = model.Drone(time = 0)
gains = np.array([3, 0.2, 0.05])
controller = controller.DroneController(drone, x_gains=0.1*gains,
        y_gains=0.1*gains, z_gains=[0.15, 0.3, 0.8])

fig = plt.figure(figsize=(15, 9))
ax = p3.Axes3D(fig)

def get_plottable(drone):
    m0 = drone.motor_pos(0)
    m2 = drone.motor_pos(2)
    m1 = drone.motor_pos(1)
    m3 = drone.motor_pos(3)
    return (([m0[0], m2[0]], [m0[1], m2[1]], [m0[2], m2[2]]),
                ([m1[0], m3[0]], [m1[1], m3[1]], [m1[2], m3[2]]))

def update_drone(i, times, ctrl_vels, drone, controller, arm_lines):
    global torques
    global last_i
    # TODO Animate function is just not meant to be used like this.
    # Look up interactive matplotlib shit
    if (last_i == i):
        return arm_lines
    drone.step(times[i], torques)
    torques = controller.get_torques(ctrl_vels[i])
    arm1, arm2 = get_plottable(drone)
    arm_lines[0].set_data(arm1[0:2])
    arm_lines[0].set_3d_properties(arm1[2])
    arm_lines[1].set_data(arm2[0:2])
    arm_lines[1].set_3d_properties(arm2[2])
    last_i = i
    return arm_lines

arm1, arm2 = get_plottable(drone)
line1, = ax.plot(*arm1, 'o-', color='blue')
line2, = ax.plot(*arm2, 'o-', color='blue')

ax.view_init(elev=0, azim=90)
ax.set_xlim3d((-10, 20))
ax.set_ylim3d((-10, 10))
ax.set_zlim3d((-10, 10))

times = np.arange(0.1, 100, 0.1)
ctrl_vels = np.array([[-5 + 10*np.ceil(np.sin(t/1)), 0, 0] for t in times])
print(ctrl_vels)
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_drone, len(times), fargs=(times, ctrl_vels, drone, controller, [line1, line2]),
                                   interval=100, blit=False)

plt.show()