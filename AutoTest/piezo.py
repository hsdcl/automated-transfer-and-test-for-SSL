import sys
import time
from pipython import GCSDevice
import numpy as np
import vision
import numpy as np
import threading

def precise_sleep(t):
    trg = time.perf_counter() + t
    while time.perf_counter() < trg:
        pass

# test x,y,z to self.axes
# 确认单位，应该是微米
class MicroCore(GCSDevice):
    def __init__(self, device, dll, usbid):
        GCSDevice.__init__(self, device, gcsdll=dll)
        self.gcsdevice.ConnectUSB(serialnum=usbid)
        self.axes_dict = {"x": self.axes[0], "y": self.axes[1], "z": self.axes[2]}
        self.position = {x: y for x,y in self.qPOS(self.axes).items()}
        self.SVO(self.axes, [True, True, True]) #闭环控制
        self.VCO(self.axes, [True, True, True]) #速度调节

        # self.qTCV
        # self.qVEL

    def close(self):
        self.gcsdevice.messages.interface.unregister_connection_status_changed_callback(self.connection_status_changed)
        self.gcsdevice.close()

    def recenter(self):
        # 确定是最后放在中间还是从0开始
        # rangemin = list(self.qTMN().values())
        # rangemax = list(self.qTMX().values())
        # targets = [(rangemax[i]+rangemin[i])/2 for i in range(len(self.axes))]
        targets = [50., 50., 40.]
        self.MOV(self.axes, targets)
        self.get_abs()

    def move(self, axis, value, velocity=5., absolute=False):
        axis = self.axes_dict[axis]
        cur_pos = self.position[axis]
        self.VEL(axis, velocity)
        if not absolute:
            self.MOV(axis, cur_pos+value)
            self.position[axis] = cur_pos+value
        else:
            self.MOV(axis, value)
            self.position[axis] = value

    def move_with_angle(self, angle, value, velocity):
        angle = float(angle)/180 * np.pi
        axes = self.axes[:2]
        values = [value*np.sin(angle), value*np.cos(angle)] #[value*np.cos(angle), value*np.sin(angle)]
        vels = [np.abs(velocity*np.sin(angle)), np.abs(velocity*np.cos(angle))] #[velocity*np.cos(angle), velocity*np.sin(angle)]
        tar_pos = [self.position[x]+values[i] for i,x in enumerate(axes)]
        # print(angle, tar_pos, vels)
        self.VEL(axes, vels)
        self.MOV(axes, tar_pos)
        return tar_pos

    def get_abs(self):
        self.position = {x: y for x,y in self.qPOS(self.axes).items()}
        return self.position


class MicroDevice(MicroCore):
    def __init__(self, device="E-727", dll="./sdk/PI_GCS2_DLL_x64.dll", usbid="0117025056"):
        super().__init__(device, dll, usbid)
        self.force = None
        self.event = threading.Event()

    def set_force(self, force):
        self.force = force

    def adjust_force_move(self, f, get_f, isDone):
        '''move til force is reached, f should be positive uN'''
        step = 0.01 if get_f() < f else -0.01
        while not isDone():
            force = get_f()
            if np.abs(force - f) < 20:
                break
            self.move('z', step, 0.5)
            self.event.wait(0.2)

    def move_with_force(self, axis, f, get_f, dist, v, isDone, emitter=None):
        '''move til force is reached, f should be negative uN'''
        i = 1
        if axis == 'z':
            step = 0.05 if dist > 0 else -0.05
        else:
            step = 0.5 if dist > 0 else -0.5
        v_c = v
        while not isDone():
            force = get_f()
            self.move(axis, step, v_c)
            self.event.wait(0.1/v_c)
            # emitter.emit(axis + ': %d' % self.position[axis])
            if axis == "z":
                v_c = max((1-0.9*np.abs(self.force/f)),0.01)*v
            else:
                v_c = v
            if force >= f or i * step >= abs(dist):
                break
            i += 1

    def move_with_height(self, axis, rel_hgt, dist, v, isDone, emitter):
        '''move til rel_hgt (positive) is reached'''
        i = 1
        step = 0.1 if dist > 0 else -0.1
        while not isDone():
            self.move(axis, step, v)
            emitter.emit(axis + ': %d' % self.position[axis])
            if abs(i * 0.1 - rel_hgt) < 0.04 or i * 0.1 >= abs(dist):
                break
            i += 1

    def push_island(self, f, dist, isDone, emitter, get_f, verbose=False):
        '''f and dist should be positive uN and whatever nm'''
        # 推岛动作代码，包括下降至力阈值、平移、抬升至原高度、平移回位四个动作
        # isDone是界面端的中断确认函数，emitter是界面端的显示信号，read_f是力传感器的读数
        # 仅在完整转岛动作中进行verbose和图像识别，单个推岛动作不进行

        success = False
        if not isDone() and verbose:
            img_before, loc = vision.get_island_shot(loc=vision.get_tip_loc())
            # emitter.emit(img_before)
        if not isDone(): hgt = self.get_abs()[self.axes_dict["z"]]
        if not isDone(): emitter.emit('Start pushing')
        if not isDone(): self.move_with_force('z', f, get_f, 25, 5, isDone, emitter)
        if not isDone(): self.move_with_force('x', 100000, get_f, -dist, 10, isDone, emitter)
        if not isDone(): self.move('z', hgt, absolute=True)
        if not isDone(): self.move('x', dist)
        if not isDone(): emitter.emit('Pushing over')
        if not isDone() and verbose:
            img_after = vision.get_island_shot(loc=loc)[0]
            diff = vision.get_diff_img(img_before, img_after)
            results = vision.predict(diff)  # 判断岛是否推出
            success = round(results)
            emitter.emit((diff, results))
        return success
    #
    def place_island(self, f_min, f_max, get_f, attempts, isDone, emitter, height=None, verbose=False):
        '''f and delay should be positive uN and ms. height is positive um'''
        # 放岛动作代码，包括下降至力阈值、抬升至原高度或指定高度、失败时提高阈值重新尝试三个动作
        # isDone是界面端的中断确认函数，emitter是界面端的显示信号，read_f是力传感器的读数
        # 仅在完整放岛动作中进行verbose和图像识别，单个放岛动作不进行
        dist = 0
        success = False
        if not isDone(): hgt = self.get_abs()[self.axes_dict["z"]]
        f_list = [f_min] + [f_min + (i + 1) * (f_max - f_min) / (attempts - 1) for i in range(attempts - 1)]
        if not isDone(): emitter.emit('Start placement')
        for f in f_list:
            if not isDone():
                if verbose:
                    img_before, loc = vision.get_island_shot(loc=vision.get_tip_loc())
                    before = vision.get_img()
                if not isDone(): self.move_with_force('z', f, get_f, 25, 5, isDone, emitter)
                if not isDone(): hgt2 = self.get_abs()[self.axes_dict["z"]]
                self.event.wait(0.5)
                # if not isDone(): self.move('z', 25)
                # if height is None:
                #     if not isDone(): self.move('z', int(hgt*1000))
                # else:
                if not isDone(): dist = hgt2 - 25  # this should be positive
                if not isDone() and verbose:
                    img_after = vision.get_island_shot(loc=loc)[0]
                    if not isDone(): self.move('x', 65)
                    # after = vision.get_img()
                    # emitter.emit(img_after)
                    diff = vision.get_diff_img(img_before, img_after)
                    results, confidence = vision.is_on_tip(diff)  # 判断岛是否还在针尖
                    before.save('cache/PLACE1/Place_before/' + str(time.time()) + '1.png')
                    if results:
                        emitter.emit('Success')
                        success = True
                        break
                    else:
                        emitter.emit('Failure')
            else:
                break
        if not isDone(): emitter.emit('Placement over')
        return success, dist
    #
    # def place_back(self, isDone):
    #     after = vision.get_img()
    #     after.save('cache/PLACE1/Place_after/' + str(time.time()) + '1.png')
    #     time.sleep(0.5)
    #     if not isDone(): self.move('x', int(50 * 1000))
    #
    def pick_island(self, f, dist, get_f, attempts, isDone, emitter, verbose=False):
        '''f and dist should be positive uN and positive nm, delay should be ms'''
        # 拾岛动作代码，包括下降至力阈值、平移、抬升至原高度、平移回位、失败时重新尝试五个动作
        # isDone是界面端的中断确认函数，emitter是界面端的显示信号，read_f是力传感器的读数
        # 仅在完整放岛动作中进行verbose和图像识别，单个拾岛动作不进行

        success = False
        if not isDone(): hgt = self.get_abs('z')
        if not isDone(): emitter.emit('Start picking')
        for i in range(attempts):
            if not isDone() and verbose:
                img_before, loc = vision.get_island_shot(loc=vision.get_tip_loc())
            if not isDone(): self.move_with_force('z', f, get_f, 25, 5, isDone, emitter)
            if not isDone(): n_hgt = self.get_abs('z')
            if not isDone(): self.move_with_force('x', -100000, get_f, -dist, 5, isDone, emitter)
            if not isDone(): self.move_with_height('z', abs(hgt - n_hgt), -15, 15, isDone, emitter)
            # if not isDone(): self.move('z', int(hgt*1000))
            if not isDone(): self.move('x', self.position['x'] + dist)
            if not isDone() and verbose:
                img_after = vision.get_island_shot(loc=loc)[0]
                diff = vision.get_diff_img(img_before, img_after)
                results, confidence = vision.is_on_tip2(diff)  # 判断岛是否还在针尖
                # emitter.emit((diff, confidence))
                if results:
                    emitter.emit('Success')
                    success = True
                    break
                else:
                    recover = vision.predict(diff)  # 判断岛是否推出，没退出可以再尝试拾岛
                    if round(recover):
                        continue
                    emitter.emit('Failure')
                    break
        if not isDone(): emitter.emit('Picking over')
        return success

    def estimate_dist(self, isDone, get_f, emitter):
        # 测算针尖到岛面距离
        dist = 0
        if not isDone(): hgt1 = self.get_abs()[self.axes_dict["z"]]
        if not isDone(): self.move_with_force('z', 50, get_f, 25, 4, isDone, emitter)
        if not isDone(): hgt2 = self.get_abs()[self.axes_dict["z"]]
        if not isDone(): self.move('z', hgt1)
        if not isDone(): dist = int(hgt2 - hgt1)  # this should be positive
        return dist

    def shear_island(self, angle, f, get_f, frequency, circ, amplitude, isDone, emitter):
        circ = int(circ)
        sleep_time = 0.5/frequency
        v = amplitude/sleep_time
        if f is not None:
            if not isDone(): self.move_with_force("z", f, get_f, 25, 5, isDone, emitter)
        for i in range(circ):
            emitter.emit("shear times {}".format(i))
            if not isDone(): self.move_with_angle(angle, amplitude, v)
            if not isDone(): self.event.wait(sleep_time)
            if not isDone(): self.move_with_angle(angle, -amplitude, v)
            if not isDone(): self.event.wait(sleep_time)

if __name__ == "__main__":
    micro = MicroDevice("E-727", "./sdk/PI_GCS2_DLL_x64.dll", "0117025056")
    micro.move_with_force()