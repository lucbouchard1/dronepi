import model, controller
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

drone = model.Drone(time = 0)
controller = controller.DroneController(drone, gains=np.array([1, 1, 1]))

fig = plt.figure()
ax = p3.Axes3D(fig)

def get_plottable(drone):
    m0 = drone.motor_pos(0)
    m2 = drone.motor_pos(2)
    m1 = drone.motor_pos(1)
    m3 = drone.motor_pos(3)
    return (([m0[0], m2[0]], [m0[1], m2[1]], [m0[2], m2[2]]),
                ([m1[0], m3[0]], [m1[1], m3[1]], [m1[2], m3[2]]))

def update_drone(i, times, ctrl_vels, drone, controller, arm_lines):
    drone.step(times[i], controller.get_torques(ctrl_vels[i]))
    arm1, arm2 = get_plottable(drone)
    arm_lines[0].set_data(arm1[0:2])
    arm_lines[0].set_3d_properties(arm1[2])
    arm_lines[1].set_data(arm2[0:2])
    arm_lines[1].set_3d_properties(arm2[2])
    return arm_lines

arm1, arm2 = get_plottable(drone)
line1, = ax.plot(*arm1, 'o-', color='blue')
line2, = ax.plot(*arm2, 'o-', color='blue')

ax.set_xlim3d((-10, 10))
ax.set_ylim3d((-10, 10))
ax.set_zlim3d((-10, 10))

times = np.arange(0, 100, 0.1)
ctrl_vels = np.array([[np.sin(t), np.sin(t), 0] for t in times])

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_drone, len(times), fargs=(times, ctrl_vels, drone, controller, [line1, line2]),
                                   interval=100, blit=False)

plt.show()