from ctypes import Structure, c_uint32, c_int16, c_uint8
import ctypes
import os

class JoystickEvent(Structure):
    _fields_ = [("time", c_uint32),
                ("value", c_int16),
                ("type", c_uint8),
                ("number", c_uint8)]


fd = os.open("/dev/input/js0", os.O_RDONLY)
while 1:
    data = os.read(fd, ctypes.sizeof(JoystickEvent))
    event = JoystickEvent.from_buffer_copy(data)
    print('\r', end='')
    print('Event at', event.time, 'type', event.type, 'number',
        event.number, 'value', event.value, end='                   ', flush=True)
