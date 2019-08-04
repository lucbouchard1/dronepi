from ctypes import Structure, c_uint32, c_int16, c_uint8
import matplotlib.pyplot as plt
import ctypes, os

fig = plt.figure()
b = plt.bar([0, 1], [0, 0])
plt.ylim(-30000, 30000)
fig.show()

class JoystickEvent(Structure):
    _fields_ = [("time", c_uint32),
                ("value", c_int16),
                ("type", c_uint8),
                ("number", c_uint8)]


fd = os.open("/dev/input/js0", os.O_RDONLY | os.O_NONBLOCK)
while plt.get_fignums():
    plt.pause(0.00005)
    try:
        data = os.read(fd, ctypes.sizeof(JoystickEvent))
    except:
        continue
    event = JoystickEvent.from_buffer_copy(data)
    if (event.number == 0):
        b[0].set_height(event.value)
    if (event.number == 1):
        b[1].set_height(event.value)