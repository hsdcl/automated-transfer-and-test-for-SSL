from ctypes import *
import os
from NTstatus import *

class NT_Motor_Core(object):
    NT_Motor = cdll.LoadLibrary("./sdk/Motor/Dll1.dll")
    dcforward = c_uint(0x2B)
    dcbackward = c_uint(0x2D)

    def __init__(self, index):
        self.sysid = c_uint32(index)
        self.location = 0.
        self.step = 0.

    @classmethod
    def find_systems(cls):
        openvalue = c_char_p(bytes("s1","utf-8"))
        sysindex = c_uint32(1000)
        cls.NT_Motor.NT_motor_findcom(byref(sysindex), openvalue)
        # print(status)
        return sysindex.value

    @staticmethod
    def assert_status(NT_STATUS):
        assert NT_STATUS == NT_OK, 'NTControl error code {}'.format(NT_STATUS)

    def set_step(self, step):
        self.step = step
        self.NT_Motor.STEP_motor_gear(self.sysid, c_uint(step))

    def move(self, direction):
        self.NT_Motor.STEP_motor_single(self.sysid, direction)
        if direction == self.dcforward:
            self.location -= self.step
        elif direction == self.dcbackward:
            self.location += self.step

if __name__ == "__main__":
    sysid = NT_Motor_Core.find_systems()
    print(sysid)
    Core = NT_Motor_Core(sysid)