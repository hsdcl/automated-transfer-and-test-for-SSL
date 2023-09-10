import serial
from serial.tools import list_ports
import time
import NTdevice as nt
import nplab.instrument.serial_instrument as si
import vision
from PIL import Image, ImageGrab
import numpy as np
from serial.tools import list_ports
from PyQt5.Qt import *
from com import *
from gui import Ui_MainWindow
from qsensor import Sensor
import NTdevice as nt
import vision
from clean import *
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import torch as T
import torch.nn as nn
from torchvision import transforms, models
import numpy as np


# 微动平台操作文件

nm, um, mm = 1, 1000, 1000000
# X, Y, Z = c_int(0), c_int(1), c_int(2)

macro = nt
def precise_sleep(t):
    trg = time.perf_counter() + t
    while time.perf_counter() < trg:
        pass

class PiezoCore(si.SerialInstrument):
    '''A simple class for the Piezo concept FOC100 nanopositioning system'''

    def __init__(self, port=None, reset=True):
        self.termination_character = '\n'
        self.timeout = 1
        self.port_settings = {
            'baudrate': 115200,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_NONE,
            'stopbits': serial.STOPBITS_ONE,
            'timeout': 1,  # wait at most one second for a response
            'writeTimeout':1,  #similarly, fail if writing takes >1s
            # 'xonxoff':False, 'rtscts':False, 'dsrdtr':False,
        }
        si.SerialInstrument.__init__(self, port=port)
        self.position = {'x': 0, 'y': 0, 'z': 0}
        # self.reset_position()
        self.recenter()        

    def close(self):
        """Release the serial port"""
        with self.communications_lock:
            try:
                self.ser.close()
            except:
                pass   
             
    def __del__(self):
        self.close()  
    
    def move_rel(self, axis, value, unit="n"):
        '''A command for relative movement, where the default units is nm. Response is Ok'''
        if unit == "n":
            multiplier = 1
        if unit == "u":
            multiplier = 1E3

        if axis != 'z' and \
            ((value * multiplier + self.position[axis]) >= 1E5 or (value * multiplier + self.position[axis]) < 0):
            raise ValueError('Movement beyond range.')
        if axis == 'z' and \
            ((value * multiplier + self.position[axis]) >= 5E4 or (value * multiplier + self.position[axis]) < 0):
            raise ValueError('Movement beyond range.')
        if axis == 'x':
            self.write("MOVRX " + str(value) + unit)
        # elif axis == 'y':
        #     self.write("MOVRY " + str(value) + unit)
        elif axis == 'z':
            self.write("MOVRZ " + str(value) + unit)
        self.position[axis] += value * multiplier

    def move(self, axis, value, unit="n"):
        '''An absolute movement command, will print an error to the console
        if you move outside of the range (xy:100um, z:50um) default unit is nm.
        Response is Ok'''
        if unit == "n":
            multiplier = 1
        if unit == "u":
            multiplier = 1E3

        if axis != 'z' and (value * multiplier >= 1E5 or value * multiplier < 0):
            raise ValueError('Movement beyond range.')
        if axis == 'z' and (value * multiplier >= 5E4 or value * multiplier < 0):
            raise ValueError('Movement beyond range.')

        if axis == 'x':
            self.write("MOVEX " + str(value) + unit)
        # elif axis == 'y':
        #     self.write("MOVEY " + str(value) + unit)
        elif axis == 'z':
            self.write("MOVEZ " + str(value) + unit)
        self.position[axis] = value * multiplier

    def get_abs(self, axis):
        '''Return xxx.xxx um'''
        # self.flush_input_buffer()
        # if axis == 'x':
        #     self.write("GET_X")
        # # elif axis == 'y':
        # #     self.write("GET_Y")
        # elif axis == 'z':
        #     self.write("GET_Z")
        # return float(self.read_multiline(termination_line="\n \n \n \n").split()[0])
        return self.position[axis]/1000

    def reset_position(self):
        pass
        # self.position['x'] = self.get_abs('x')*1000
        # self.position['y'] = self.get_abs('y')*1000
        # self.position['z'] = self.get_abs('z')*1000

    def recenter(self):
        '''Moves the stage to the center position'''
        self.move('x', 50, "u")
        # self.move('y', 50, "u")
        self.move('z', 25, "u")

    def INFO(self):
        '''Please specify the decode mode in SerialInstrument.readline to latin1'''
        self.flush_input_buffer()
        return self.query("INFOS", multiline=True, termination_line="\n \n \n \n", timeout=.1)

    def test_communications(self):
        return 'Piezoconcept' in self.INFO()


class PiezoDevice(PiezoCore):
    def __init__(self, port=None, reset=False):
        super().__init__(port=port, reset=reset)

    def move_with_force(self, axis, f, dist, v, isDone, read_f, emitter):
        '''move til force is reached, f should be negative uN'''
        i = 1
        step = 100 if dist > 0 else -100
        print(axis, f, dist, v, isDone, read_f)
        while not isDone():
            self.move(axis, self.position[axis]+step)
            emitter.emit(axis + ': %d' % self.position[axis])
            precise_sleep(100/v)
            if read_f() <= f or i*100 >= abs(dist):
                break
            i += 1

    def move_with_height(self, axis, rel_hgt, dist, v, isDone, emitter):
        '''move til rel_hgt (positive) is reached'''
        i = 1
        step = 100 if dist > 0 else -100
        while not isDone():
            self.move(axis, self.position[axis]+step)
            emitter.emit(axis + ': %d' % self.position[axis])
            precise_sleep(100/v*2)
            if abs(i*100-rel_hgt) < 40 or i*100 >= abs(dist):
                break
            i += 1

    def push_island(self, f, dist, isDone, read_f, emitter, verbose=False):
        '''f and dist should be positive uN and whatever nm'''
        # 推岛动作代码，包括下降至力阈值、平移、抬升至原高度、平移回位四个动作
        # isDone是界面端的中断确认函数，emitter是界面端的显示信号，read_f是力传感器的读数
        # 仅在完整转岛动作中进行verbose和图像识别，单个推岛动作不进行

        success = False
        if not isDone() and verbose:
            img_before, loc = vision.get_island_shot(loc=vision.get_tip_loc())
            emitter.emit(img_before)
        if not isDone(): hgt = self.get_abs('z')
        if not isDone(): emitter.emit('Start pushing')
        if not isDone(): self.move_with_force('z', -f, 25000, 4000, isDone, read_f, emitter)
        if not isDone(): self.move_with_force('x', -100000, -dist, 5000, isDone, read_f, emitter)
        if not isDone(): self.move('z', int(hgt*1000))
        if not isDone(): self.move('x', self.position['x']+dist)   
        if not isDone(): emitter.emit('Pushing over')
        if not isDone() and verbose: 
            img_after = vision.get_island_shot(loc=loc)[0]
            diff = vision.get_diff_img(img_before, img_after)
            results = vision.predict(diff)  # 判断岛是否推出
            success = round(results)
            emitter.emit((diff, results))
        return success


    def place_island(self, f_min, f_max, attempts, isDone, read_f, emitter, height=None, verbose=False):
        '''f and delay should be positive uN and ms. height is positive um'''
        # 放岛动作代码，包括下降至力阈值、抬升至原高度或指定高度、失败时提高阈值重新尝试三个动作
        # isDone是界面端的中断确认函数，emitter是界面端的显示信号，read_f是力传感器的读数
        # 仅在完整放岛动作中进行verbose和图像识别，单个放岛动作不进行
        dist = 0
        success = False
        if not isDone(): hgt = self.get_abs('z')
        img = vision.get_img()
        row,col = vision.get_relative_dist1(img)
        f_list = [f_min] + [f_min + (i+1)*(f_max-f_min)/(attempts-1) for i in range(attempts-1)]
        if not isDone(): emitter.emit('Start placement')
        for f in f_list:
            if not isDone():
                if not isDone():
                    self.move("x", int(50*1000))
                if verbose:
                    img_before, loc = vision.get_island_shot(loc=[row, col])
                    before = vision.get_img()
                if not isDone(): self.move_with_force('z', -f, 49900, 4000, isDone, read_f, emitter)
                if not isDone(): hgt2 = self.get_abs('z')
                time.sleep(0.5)
                if not isDone(): self.move('z', int(25*1000))
                # if height is None:
                #     if not isDone(): self.move('z', int(hgt*1000))
                # else:
                if not isDone(): dist = int((hgt2*1000)-(25*1000)) # this should be positive
                if not isDone() and verbose:
                    img_after = vision.get_island_shot(loc=loc)[0]

                    if not isDone(): self.move('x', int(65 * 1000))
                    after = vision.get_img()
                    # emitter.emit(img_after)
                    diff = vision.get_diff_img(img_before,img_after)
                    results,confidence = vision.is_on_tip(diff) # 判断岛是否还在针尖
                    emitter.emit((diff,confidence))
                    before.save('cache/PLACE1/Place_before/'+str(time.time())+'1.png')
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

    def place_back(self,isDone):
        after = vision.get_img()
        after.save('cache/PLACE1/Place_after/'+str(time.time())+'1.png')
        time.sleep(0.5)
        if not isDone(): self.move('x', int(50 * 1000))

    def pick_island(self, f, dist, attempts, isDone, read_f, emitter, verbose=False):
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
                emitter.emit(img_before)
            if not isDone(): self.move_with_force('z', -f, 49900, 2000, isDone, read_f, emitter)
            if not isDone(): n_hgt = self.get_abs('z')
            if not isDone(): self.move_with_force('x', -1000, -dist, 5000, isDone, read_f, emitter)
            if not isDone(): self.move_with_height('z', abs(hgt - n_hgt) * 1000, -15000, 15000, isDone, emitter)
            # if not isDone(): self.move('z', int(hgt*1000))
            if not isDone(): self.move('x', self.position['x'] + dist)
            if not isDone() and verbose:
                img_after = vision.get_island_shot(loc=loc)[0]
                diff = vision.get_diff_img(img_before, img_after)
                # results = vision.predict(img_before, img_after)
                # emitter.emit(results)
                # if round(results[1]):
                #     pass
                # else:
                results, confidence = vision.is_on_tip2(diff)  # 判断岛是否还在针尖
                emitter.emit((diff, confidence))
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
                emitter.emit('Failure')
        if not isDone(): emitter.emit('Picking over')
        return success    
        
    def estimate_dist(self, isDone, read_f, emitter):
        # 测算针尖到岛面距离
        dist = 0
        if not isDone(): hgt1 = 25
        if not isDone(): self.move_with_force('z', -80, 25000, 4000, isDone, read_f, emitter)
        if not isDone(): hgt2 = self.get_abs('z')
        if not isDone(): self.move('z', int(hgt1*1000))
        if not isDone(): dist = int(hgt2*1000-hgt1*1000) # this should be positive
        return dist

    def init_z(self, isDone):
        dist1 = 0
        if not isDone(): hgt = self.get_abs('z')
        if not isDone(): self.move('z', int(25 * 1000))
        if not isDone(): dist = 25 - hgt
        return dist1

    def estimate_mark_z(self, isDone, read_f, emitter):
        # 测算针尖到岛面距离
        if not isDone(): hgt1 = self.get_abs('z')
        if not isDone(): self.move_with_force('z', -80, 25000, 4000, isDone, read_f, emitter)

    def estimate_mark_z1(self, isDone, read_f, emitter):
            # 测算针尖到岛面距离
        if not isDone(): self.move('z', int(25 * 1000))

    def markloc(self,num):
        global macro
        try:
            n = num
            img = vision.get_img()
            y, x = vision.get_relative_dist1(img)
            x1, y1 = vision.get_sign(img, n)
            xx, yy, zz = macro.location()
            zz = micro.estimate_mark_dist(self.isDone, self.get_f, self.communicate)
            xx = int(((x - x1) * 78) + xx)
            yy = int(((y1 - y) * 78) + yy)
            self.target_x.setText(str(xx))
            self.target_y.setText(str(yy))
            self.target_z.setText(str(zz))
            self.xyz1 = xx, yy, zz
        except:
            pass