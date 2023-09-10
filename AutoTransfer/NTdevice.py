from ctypes import *
from NTstatus import *
import time


nm, um, mm = 1, 1000, 1000000
X, Y, Z = c_int(0), c_int(1), c_int(2)

def precise_sleep(t):
    trg = time.perf_counter() + t
    while time.perf_counter() < trg:
        pass


class NTcore(object):
    NT = cdll.LoadLibrary('./sdk/lib64/NTControl.dll')
    
    def __init__(self, loc="usb:ix:0"):
        self.sysid = c_int()
        self.assert_status(self.NT.NT_OpenSystem(
            byref(self.sysid), c_char_p(loc.encode('utf-8')), c_wchar_p("sync")))
        _ = self.location()

    def location(self):
        return self.get_pos(X), self.get_pos(Y), self.get_pos(Z)

    @classmethod
    def find_systems(cls):
        outbuffer = c_char_p(('0'*100).encode('utf-8'))
        buffersize = c_int(100)
        cls.NT.NT_FindSystems('', outbuffer, byref(buffersize))
        return outbuffer.value.decode().split('\n')

    @staticmethod
    def assert_status(NT_STATUS):
        assert NT_STATUS == NT_OK, 'NTControl error code {}'.format(NT_STATUS)

    def close(self):
        try:
            self.NT.NT_CloseSystem(self.sysid)
        except:
            pass

    def move(self, axis, dist):
        if dist > 500:
            self.assert_status(self.NT.NT_SetAccumulateRelativePositions_S(self.sysid, axis, NT_NO_ACCUMULATE_RELATIVE_POSITIONS))
        self.assert_status(self.NT.NT_GotoPositionRelative_S(self.sysid, axis, dist, 0))
    

    def move_with_force(self, axis, dist, f, v, read_f):
        # i = 1
        # while dist:
            self.assert_status(self.NT.NT_GotoPositionAbsolute_S(self.sysid, axis, dist, 0))
            # if read_f() <= f or i*40 >= abs(dist):
            # time.sleep(1)
            # self.stop(Z)
                # break
            # i += 1

    def move_rel(self, axis, dist, wait=True):
        if wait:
            self.stop(axis)
            time.sleep(0.5)
        self.assert_status(self.NT.NT_GotoPositionRelative_S(self.sysid, axis, dist, 0))
        if wait:
            time.sleep(1)
            while self.get_status(axis) != 3:
                precise_sleep(0.1)
            # time.sleep(1)

    def move_abs_xy(self, x_dist, y_dist, wait=True):
        if wait:
            self.stop(X)
            self.stop(Y)
            time.sleep(0.5)
        self.assert_status(self.NT.NT_GotoPositionAbsolute_S(self.sysid, X, x_dist, 0))
        self.assert_status(self.NT.NT_GotoPositionAbsolute_S(self.sysid, Y, y_dist, 0))
        if wait:
            time.sleep(1)
            while self.get_status(X) != 3 or self.get_status(Y) != 3:
                precise_sleep(0.1)
            # print(self.get_status(X),self.get_status(Y), "over")


    def move_abs(self, axis, dist, wait=True):
        print(self.get_status(axis),"move_abs")
        if wait:
            self.stop(axis)
            time.sleep(0.5)
        self.assert_status(self.NT.NT_GotoPositionAbsolute_S(self.sysid, axis, dist, 0))
        if wait:
            time.sleep(1)
            while self.get_status(axis) != 3:
                print(self.get_status(axis), "move_abs")
                precise_sleep(0.1)

    def set_pos(self, axis, pos):
        self.assert_status(self.NT.NT_SetPosition_S(self.sysid, axis, pos))

    def get_pos(self, axis):
        pos = c_int()
        self.assert_status(self.NT.NT_GetPosition_S(self.sysid, axis, byref(pos)))
        return pos.value

    def get_status(self, axis):
        status = c_int()
        self.assert_status(self.NT.NT_GetStatus_S(self.sysid, axis, byref(status)))
        # time.sleep(0.1)
        return status.value

    def stop(self, axis):
        self.assert_status(self.NT.NT_Stop_S(self.sysid, axis))

    def __del__(self):
        self.close()

if __name__ == "__main__":
    device_ids = NTcore.find_systems()
    for _id in device_ids:
        try:
            macro = NTcore(_id)
            loc = macro.location()
        except Exception as err:
            if hasattr(macro, 'close'): macro.close()
            macro = None
    x_tar = -10000000
    y_tar = -10000000 
    z_tar = -1*mm
    # macro.move_abs_xy(x_tar, y_tar, wait=True)
    macro.move_rel(Z, z_tar)
    macro.close()
