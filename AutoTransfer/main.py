import configparser
import math
import os
import sys
import traceback

import numpy as np
import serial
import torch
from PIL import Image, ImageQt
from PyQt5.Qt import *
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget
from modbus_tk import modbus_rtu
from serial.tools import list_ports
import cv2
import NTdevice as nt
import piezo
import vision
from clean import *
from com import *
from gui import Ui_MainWindow
from qsensor import Sensor
import copy
from PyQt5 import QtGui

cf = configparser.ConfigParser()
cf.read('./config.ini')

sensor = None
macro = None
micro = None
xyz1 = None
xyz2 = None
xyz3 = None
on = False
g = 0
angle = 0
angle1 = 0


class SingleActionThread(QThread):
    """
        推岛，拾岛，放岛单一动作
    """
    communicate = pyqtSignal(object)

    def __init__(self, action_name, params):
        super().__init__()
        self.action_name = action_name
        self.params = params
        self.done = False
        self.f = None

    def run(self):
        global micro, sensor
        time.sleep(0.5)
        # try:
        assert self.action_name in ['push_island', 'pick_island', 'place_island']
        # micro.reset_position()
        sensor.reset()
        time.sleep(0.5)
        action = getattr(micro, self.action_name)
        if self.action_name != 'place_island':
            action(**self.params,
                   isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
        else:
            _,dist = action(**self.params,
                            isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
            print(dist)
            dist1 = int(self.params["height"]) * 1000 - dist
            time.sleep(1)
            print(dist1)
            macro.move_rel(nt.Z, dist1)
        # except Exception as err:
        #     self.communicate.emit(str(err))

    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True

    def isDone(self):
        return self.done

class markActionThread(QThread,Ui_MainWindow,QWidget):
    communicate = pyqtSignal(object)

    def __init__(self, action_name,params):
        super(markActionThread, self).__init__()
        self.action_name = action_name
        self.params = params
        self.done = False
        self.f = None
        self.x = 0
        self.y = 0
        self.z = 0

    def run(self):
        global micro,xyz1,xyz2,xyz3
        time.sleep(0.5)
        # try:
        n = self.action_name
        time.sleep(0.5)
        img1 = vision.get_img()
        time.sleep(0.5)
        y, x = vision.get_relative_dist1(img1)
        x1,y1 = vision.get_sign(img1, n)
        micro.estimate_mark_z(self.isDone, self.get_f, self.communicate)
        xx, yy, zz = macro.location()
        micro.estimate_mark_z1(self.isDone, self.get_f, self.communicate)
        xx = int(xx + ((x1 - x) * 78))
        yy = int(yy + ((y1 - y) * 78))
        zz = int(zz - self.params['height'])
        macro.move_abs_xy(xx, yy)
        if n == "1":
            path = r'xyz/xyz1.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz2.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz3.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            path = r'xyz/xyz23.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz24.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz25.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            xyz1 = xx,yy,zz
        if n == "2":
            path = r'xyz/xyz26.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz27.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz28.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            xyz2 = xx,yy,zz
        if n == "3":
            path = r'xyz/xyz29.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz30.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz31.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            xyz3 = xx,yy,zz
        if n == "4":
            path = r'xyz/xyz32.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz33.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz34.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        self.x = xx
        self.y = yy
        self.z = zz
        # return xx,yy,zz
    # except Exception as err:
    #     self.communicate.emit(str(err))

    def loc(self):
        return self.x ,self.y ,self.z

    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True

    def isDone(self):
        return self.done

class gotoActionThread(QThread):
    communicate = pyqtSignal(object)

    def __init__(self, action_name,params):
        super(gotoActionThread, self).__init__()
        self.action_name = action_name
        self.p = params
        self.done = False
        self.f = None
        self.x = 0
        self.y = 0
        self.z = 0

    def run(self):
        global xyz1,xyz2,xyz3,macro, micro, sensor, angle
        time.sleep(0.5)
        if self.action_name == 'xyz':
            x_tar = self.p['x_0']
            y_tar = self.p['y_0']
            z_tar = self.p['z_0']
            if not self.isDone():macro.move_rel(nt.Z, 2*nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'X1Y1Z1':
            x_tar = self.p['x_1']
            y_tar = self.p['y_1']
            z_tar = self.p['z_1']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'Clean':
            x_tar = self.p['c_x']
            y_tar = self.p['c_y']
            z_tar = self.p['c_z']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
            ser = serial.Serial(port="COM11",baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
            assert ser.is_open
            master = modbus_rtu.RtuMaster(ser)
            master.set_timeout(5.0)
            set_controller(ser, "out")
            open_power(ser)
            set_current(ser, 160)
            set_voltage(ser, 1000)
            time.sleep(2)
            close_power(ser)
        if self.action_name == 'Target':
            x_tar = self.p['x_tar']
            y_tar = self.p['y_tar']
            z_tar = self.p['z_tar']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == '0':
            x_tar = int(0)
            y_tar = int(0)
            z_tar = int(0)
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'mark1':
            x_tar = self.p['mark1_x']
            y_tar = self.p['mark1_y']
            z_tar = self.p['mark1_z']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'mark2':
            x_tar = self.p['mark2_x']
            y_tar = self.p['mark2_y']
            z_tar = self.p['mark2_z']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'mark3':
            x_tar = self.p['mark3_x']
            y_tar = self.p['mark3_y']
            z_tar = self.p['mark3_z']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)
        if self.action_name == 'mark4':
            x_tar = self.p['mark4_x']
            y_tar = self.p['mark4_y']
            z_tar = self.p['mark4_z']
            if not self.isDone():macro.move_rel(nt.Z, 2 * nt.mm)
            if not self.isDone():macro.move_abs_xy(x_tar, y_tar)
            if not self.isDone():macro.move_abs(nt.Z, z_tar)


    def loc(self):
        return self.x ,self.y ,self.z

    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True
        try:
            macro.stop(nt.X)
            macro.stop(nt.Y)
            macro.stop(nt.Z)
        except:
            pass

    def isDone(self):
        return self.done

class ArrayActionThread(QThread):
    """
        阵列化转移
    """
    communicate = pyqtSignal(object)
    directions = ["right", "down", "up"]

    def __init__(self, array_params, push_params, pick_params, place_params):
        super().__init__()
        self.p = array_params
        self.push_params = push_params
        self.pick_params = pick_params
        self.place_params = place_params
        self.done = False
        self.f = None
        self.xyz = None

    def run(self):
        global macro, micro, sensor, angle1
        time.sleep(0.5)
        if angle1 == 0:
            angle1 = math.atan((self.p['y_p1_3'] - self.p['y_p2_3']) / (self.p['x_p1_3'] - self.p['x_p2_3']))
        try:
            direction_mat = vision.create_direction_matrix(self.p['rows'], self.p['cols'])
            # 设定下一个岛方向
            ctr = int(0)
            for col in range(self.p['cols']):
                if self.isDone(): break
                # micro.reset_position()
                for row in range(self.p['rows']):
                    if self.isDone(): break
                    if not self.isDone():
                        # 调整高度
                        self.communicate.emit('Adjusting height')
                        time.sleep(0.2)
                        sensor.reset()
                        time.sleep(0.2)
                        dist = micro.estimate_dist(self.isDone, self.get_f, self.communicate)
                        dist = int(self.place_params['height'])*1000 - dist    # positive == downward
                        if not self.isDone(): macro.move_rel(nt.Z, dist, wait=False)

                    isRecovered, isPicked = True, False
                    if not self.isDone(): xx, yy, zz = macro.location()
                    time.sleep(0.2)
                    sensor.reset()
                    time.sleep(0.2)
                    if not self.isDone() and not self.p['no_push']:
                        # 推岛
                        isRecovered = micro.push_island(**self.push_params, verbose=True,
                                                        isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                    if not self.isDone() and not self.p['push_only'] and isRecovered:
                        # 拾岛                     
                        isPicked = micro.pick_island(**self.pick_params, verbose=True,
                                                     isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                    if not self.isDone() and not self.p['push_only'] and isPicked:
                        # 移动到指定位置并放岛
                        cur_row = ctr // self.p['cols_tar']
                        cur_col = ctr % self.p['cols_tar']
                        gap = self.p['gap_tar']*1000
                        rel_x = int((self.p['cols_tar']-1)*gap/2-cur_col*gap)
                        rel_y = int((self.p['rows_tar']-1)*gap/2-cur_row*gap)
                        cur_x_tar = self.p['x_tar'] + int(rel_x*math.cos(angle1) - rel_y*math.sin(angle1))
                        cur_y_tar = self.p['y_tar'] + int(rel_x*math.sin(angle1) + rel_y*math.cos(angle1))
                        if not self.isDone(): macro.move_rel(nt.Z, 2*nt.mm)
                        if not self.isDone(): zz2 = int(macro.get_pos(nt.Z))
                        if not self.isDone(): macro.move_abs_xy(cur_x_tar, cur_y_tar)
                        if not self.isDone(): macro.move_abs(nt.Z, self.p['z_tar'])
                        if not self.isDone():
                            time.sleep(0.2)
                            sensor.reset()
                            time.sleep(0.2)
                            success,dist1 = micro.place_island(**self.place_params, verbose=True,
                                                               isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                            dist1 = int(self.place_params['height'])*1000 - dist1
                            if not self.isDone(): macro.move_rel(nt.Z, dist1, wait=False)
                            micro.place_back(isDone=self.isDone)
                            if not self.isDone(): macro.move_rel(nt.Z, -dist1, wait=False)
                        if not self.isDone() and success:
                            ctr += 1
                            self.communicate.emit('%d islands transfered' % ctr)
                            if ctr == self.p['cols_tar']*self.p['rows_tar']: self.set_done()
                        elif not self.isDone() and not success:
                            cx = self.p['c_x']
                            cy = self.p['c_y']
                            cz = self.p['c_z']
                            if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                            if not self.isDone(): macro.move_abs_xy(cx, cy)
                            if not self.isDone(): macro.move_abs(nt.Z, cz)
                            ser = serial.Serial(port="COM11",baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
                            assert ser.is_open
                            master = modbus_rtu.RtuMaster(ser)
                            master.set_timeout(5.0)
                            set_controller(ser, "out")
                            open_power(ser)
                            set_current(ser, 80)
                            set_voltage(ser, 500)
                            time.sleep(2)
                            close_power(ser)
                        if not self.isDone(): macro.move_abs(nt.Z, zz2)
                        if not self.isDone(): macro.move_abs_xy(xx, yy)
                        if not self.isDone(): macro.move_abs(nt.Z, zz)
                        if not self.isDone(): time.sleep(0.5)
                    if row == self.p['rows']-1 and col == self.p['cols']-1: self.set_done()
                    if not self.isDone():
                        # 移动到下一个岛的位置
                        img = vision.get_img()
                        direction = self.directions[int(direction_mat[row,col])]
                        time.sleep(1.0)
                        row_dist, col_dist, find_land = vision.get_relative_dist(img,
                                                                                 self.p['size'], self.p['gap'] , direction)
                        if not self.isDone():
                            macro.move_rel(nt.Z, 10*nt.um, wait=False)
                            time.sleep(0.2)
                            macro.move_rel(nt.X, col_dist, wait=False)
                            macro.move_rel(nt.Y, row_dist, wait=False)
                            time.sleep(0.2)
                            macro.move_rel(nt.Z, -10*nt.um, wait=False)
                            while not find_land:
                                img = vision.get_img()
                                direction = self.directions[int(direction_mat[row,col])]
                                time.sleep(1.0)
                                row_dist, col_dist, find_land = vision.get_relative_dist(img,
                                                                                         self.p['size'], self.p['gap'] , direction)
                                if not self.isDone():
                                    macro.move_rel(nt.Z, 10*nt.um, wait=False)
                                    time.sleep(0.2)
                                    macro.move_rel(nt.X, col_dist, wait=False)
                                    macro.move_rel(nt.Y, row_dist, wait=False)
                                    time.sleep(0.2)
                                    macro.move_rel(nt.Z, -10*nt.um, wait=False)
                                    self.p['rows'] = self.p['rows']-1
                                    self.p['cols'] = self.p['cols']-1
                            else:
                                continue

        except IndexError:
            self.communicate.emit('Failed to find the tip, or more than one tip are found')

        except Exception:
            self.communicate.emit(traceback.format_exc())

    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True
        try:
            macro.stop(nt.X)
            macro.stop(nt.Y)
            macro.stop(nt.Z)
        except:
            pass


    def isDone(self):
        return self.done

class FileActionThread(QThread):
    """
        自定义阵列化转移
        和阵列化转移基本一致，只不过目标位置从txt中读取
    """

    communicate = pyqtSignal(object)
    directions = ["right", "down", "up"]

    def __init__(self, array_params, push_params, pick_params, place_params, file_params):
        super().__init__()
        self.p = array_params
        self.push_params = push_params
        self.pick_params = pick_params
        self.place_params = place_params
        self.file_params = file_params
        self.startline = 0
        self.fileloc = []
        self.fileyesloc = []
        for line in self.file_params:
            if len(line) == 3: self.fileyesloc.append(line)
            else: self.fileloc.append(line)
        self.ctr = int(0)
        self.done = False
        self.f = None

    def run(self):
        global macro, micro, sensor, angle, xyz1, xyz2, xyz3, g
        time.sleep(0.5)
        n = 0
        if angle == 0:
            angle = math.atan((self.p['y_p1'] - self.p['y_p2']) / (self.p['x_p1'] - self.p['x_p2']))
        try:
            direction_mat = vision.create_direction_matrix(self.p['rows'], self.p['cols'])
            if self.ctr == len(self.fileloc):
                self.set_done()
            for col in range(self.p['cols']):
                if self.isDone(): break
                # micro.reset_position()
                for row in range(self.p['rows']):
                    if self.isDone(): break
                    if not self.isDone():
                        self.communicate.emit('Adjusting height')
                        time.sleep(0.2)
                        sensor.reset()
                        time.sleep(0.2)
                        dist = micro.estimate_dist(self.isDone, self.get_f, self.communicate)
                        dist = int(self.place_params['height'])*1000 - dist    # positive == downward
                        if not self.isDone(): macro.move_rel(nt.Z, dist, wait=False)
                    isRecovered, isPicked = True, False
                    if not self.isDone(): xx, yy, zz = macro.location()
                    time.sleep(0.2)
                    sensor.reset()
                    time.sleep(0.2)
                    image_be = vision.get_img()
                    if not self.isDone() and not self.p['no_push']:
                        isRecovered = micro.push_island(**self.push_params, verbose=True,
                                                        isDone=self.isDone, read_f=self.get_f,
                                                        emitter=self.communicate)
                    if not self.isDone() and isRecovered:
                        time.sleep(0.2)
                        sensor.reset()
                        time.sleep(0.2)
                        isPicked = micro.pick_island(**self.pick_params, verbose=True,isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                    if not self.isDone() and isPicked:
                        image_af = vision.get_img()
                        calibration = vision.get_target_calibration(image_be, image_af)

                        rel_x = int(self.fileloc[self.ctr][0])
                        rel_y = int(self.fileloc[self.ctr][1])
                        cur_x_tar = self.p['x_tar'] + int(rel_x*math.cos(angle) - rel_y*math.sin(angle))
                        cur_y_tar = self.p['y_tar'] + int(rel_x*math.sin(angle) + rel_y*math.cos(angle))
                        if not self.isDone(): macro.move_rel(nt.Z, 2*nt.mm)
                        if not self.isDone(): zz2 = int(macro.get_pos(nt.Z))
                        if not self.isDone(): macro.move_abs_xy(cur_x_tar, cur_y_tar)
                        if not self.isDone(): macro.move_abs(nt.Z, self.p['z_tar'])
                        if not self.isDone():
                            time.sleep(0.2)
                            sensor.reset()
                            time.sleep(0.2)
                            success, dist1 = micro.place_island(**self.place_params, verbose=True,
                                                                isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                            dist1 = int(self.place_params['height'])*1000 - dist1
                            if not self.isDone(): macro.move_rel(nt.Z, dist1)
                            micro.place_back(isDone=self.isDone)
                            if not self.isDone() and success:
                                self.fileyesloc.append(self.fileloc[self.ctr])
                                self.fileyesloc[-1].append('y')
                                self.ctr += 1
                                self.communicate.emit('%d islands transfered' % self.ctr)
                                if self.ctr == len(self.fileloc):
                                    self.set_done()
                            elif not self.isDone() and not success:
                                cx = self.p['c_x']
                                cy = self.p['c_y']
                                cz = self.p['c_z']
                                if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                                if not self.isDone(): macro.move_abs_xy(cx, cy)
                                if not self.isDone(): macro.move_abs(nt.Z, cz)
                                ser = serial.Serial(port="COM11",baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
                                assert ser.is_open
                                master = modbus_rtu.RtuMaster(ser)
                                master.set_timeout(5.0)
                                set_controller(ser, "out")
                                open_power(ser)
                                set_current(ser, 80)
                                set_voltage(ser, 500)
                                time.sleep(2)
                                close_power(ser)
                        if not self.isDone(): macro.move_abs(nt.Z, zz2)
                        if not self.isDone(): macro.move_abs_xy(xx, yy)
                        if not self.isDone(): macro.move_abs(nt.Z, zz)
                        if not self.isDone(): time.sleep(0.5)
                    if row == self.p['rows']-1 and col == self.p['cols']-1: self.set_done()
                    if not self.isDone():
                        img = vision.get_img()
                        direction = self.directions[int(direction_mat[row,col])]
                        row_dist, col_dist, find_land= vision.get_relative_dist(img,
                                                                                self.p['size'], self.p['gap'] , direction)
                        if not self.isDone():
                            macro.move_rel(nt.Z, 10*nt.um)
                            time.sleep(0.2)
                            macro.move_rel(nt.X, col_dist, wait=False)
                            macro.move_rel(nt.Y, row_dist, wait=False)
                            time.sleep(0.2)
                            macro.move_rel(nt.Z, -10*nt.um)
                            while not find_land:
                                img = vision.get_img()
                                direction = self.directions[int(direction_mat[row,col])]
                                time.sleep(1.0)
                                row_dist, col_dist, find_land = vision.get_relative_dist(img,
                                                                                         self.p['size'], self.p['gap'] , direction)
                                if not self.isDone():
                                    macro.move_rel(nt.Z, 10*nt.um)
                                    time.sleep(0.2)
                                    macro.move_rel(nt.X, col_dist, wait=False)
                                    macro.move_rel(nt.Y, row_dist, wait=False)
                                    time.sleep(0.2)
                                    macro.move_rel(nt.Z, -10*nt.um)
                                    self.p['rows'] = self.p['rows']-1
                                    self.p['cols'] = self.p['cols']-1
                            else:
                                continue

        except Exception:
            self.communicate.emit(traceback.format_exc())

        finally:
            self.save()

    def save(self):
        n = 0
        with open(time.strftime('./Array files/Temporary file/%d%M%S.txt'), 'w') as file:
            for line in self.fileyesloc:
                if line != []:
                    file.write('\t'.join(line)+'\n')
            for line in self.fileloc:
                if line != []:
                    if n >= self.ctr:
                        file.write('\t'.join(line) + '\n')
                        n += 1
                    else:
                        n += 1


    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True
        try:
            macro.stop(nt.X)
            macro.stop(nt.Y)
            macro.stop(nt.Z)
        except:
            pass

    def isDone(self):
        return self.done

class markfileActionThread(QThread):
    """
        预读取文件，并根据mark位置校准坐标系，自定义阵列化转移
        和阵列化转移基本一致，只不过目标位置从txt中读取
    """

    communicate = pyqtSignal(object)
    directions = ["right", "down", "up"]

    def __init__(self, array_params, push_params, pick_params, place_params, file_params):
        super().__init__()
        self.p = array_params
        self.push_params = push_params
        self.pick_params = pick_params
        self.place_params = place_params
        self.file_params = file_params
        self.estimate_params = copy.copy(place_params)
        self.estimate_params["attempts"] = 1
        # self.estimate_params['height'] = self.estimate_params['height']
        self.startline = 0
        self.fileloc = []
        self.fileyesloc = []
        self.markloc = []
        mark_ori = [[0,0,0], [-2.6e7,0,0], [-2.6e7, 2.6e7, 0]]
        mark_tar = []
        linenum = (-1)
        lennum = []
        for loc in ["1", "2", "3"]:
            mark_tar.append([])
            for dir in ["x", "y", "z"]:
                mark_tar[-1].append(self.p["mark{}_{}".format(loc, dir)])
        self.trans_mat, self.bias_ori, self.bias_tar = self.cal_trans_matrix(mark_ori, mark_tar)
        for line in self.file_params:
            if len(line) == 4:
                linenum += 1
                self.markloc.append([])
                self.markloc[linenum].append(line)
            else:
                self.markloc[linenum].append(line)
        for line in self.file_params:
            if len(line) == 2:
                self.fileloc.append(line)
                len(self.fileloc)

        self.ctr = int(0)
        self.ctr1 = int(0)
        self.ctr2 = int(0)
        self.done = False
        self.f = None
        self.clean = None
        self.befromloc = []
        self.markloc1 = []
        self.loc = []
    def run(self):
        global macro, micro, sensor, angle, g
        cx = 2000000
        cy = 2000000

        time.sleep(0.5)
        try:
            if not self.isDone():
                for i in self.markloc:
                    self.markloc1 = []
                    self.loc = []
                    col1 = 0
                    row1 = 0
                    self.ctr = int(0)
                    self.ctr2 += 1
                    pcol = self.p['cols'] - col1
                    prow = self.p['rows'] - row1
                    print(pcol,prow,"pcol")
                    if not self.isDone():
                        for q in i:
                            if len(q) == 4:
                                self.markloc1.append(q)
                                self.befromloc.append(q)
                                self.ctr2 += 1
                            if len(q) == 3:
                                self.befromloc.append(q)
                            if len(q) == 2:
                                self.loc.append(q)
                        shibie = 15000
                        if angle == 0:
                            angle = math.atan((self.p['y_p1'] - self.p['y_p2']) / (self.p['x_p1'] - self.p['x_p2']))
                        xx1, yy1, zz1 = macro.location()
                        if not self.isDone():mark_x, mark_y = int(self.markloc1[0][1]), int(self.markloc1[0][0])
                        if not self.isDone():mark_x, mark_y, mark_Z = self.cal_trans_results([mark_x, mark_y, 0])
                        if not self.isDone():mark_x1, mark_y1 = int(mark_x-shibie), int(mark_y-shibie)
                        if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                        if not self.isDone(): macro.move_abs_xy(int(mark_x1), int(mark_y1))
                        if not self.isDone(): macro.move_abs(nt.Z, int(mark_Z))
                        sensor.reset()
                        time.sleep(0.5)
                        _, dist = micro.place_island(**self.estimate_params, isDone=self.isDone, read_f=self.get_f, emitter=self.communicate)
                        dist1 = int(self.estimate_params["height"])*1000-dist
                        macro.move_rel(nt.Z, dist1)
                        time.sleep(1)

                        img = vision.get_img()
                        y, x = vision.get_relative_dist1(img)
                        x1,y1 = vision.get_sign(img, "5")
                        img = np.array(img)
                        cv2.circle(img,(x,y),3,(0,0,255),-1)
                        cv2.circle(img, (x1, y1), 3, (0, 0, 255), -1)
                        cv2.imwrite("./cache/mark/" + time.strftime("%m%d-%H-%M-%S") + ".png",
                                    img.astype(np.uint8))
                        micro.estimate_mark_z(self.isDone, self.get_f, self.communicate)
                        xx2, yy2, zz2 = macro.location()
                        micro.estimate_mark_z1(self.isDone, self.get_f, self.communicate)
                        xx3 = int(xx2 + ((x1 - x) * 78))
                        yy3 = int(yy2 + ((y1 - y) * 78))
                        zz3 = int(zz2 - self.place_params['height'])
                        macro.move_abs_xy(xx3, yy3)
                        xx4 = int(((x1 - x) * 78)-shibie)
                        yy4 = int(((y1 - y) * 78)-shibie)

                        #test
                        # rel_x = int(self.loc[self.ctr][1])
                        # rel_y = int(self.loc[self.ctr][0])
                        # cur_x_tar, cur_y_tar, z1 = self.cal_trans_results([rel_x, rel_y, 0])
                        # macro.move_abs_xy(int(cur_x_tar) + xx4, int(cur_y_tar) + yy4)

                        if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                        if not self.isDone(): macro.move_abs_xy(xx1, yy1)
                        if not self.isDone(): macro.move_abs(nt.Z, zz1)

                        direction_mat = vision.create_direction_matrix(prow, pcol)
                        if self.ctr1 == len(self.fileloc):
                            self.set_done()
                        for col in range(pcol):
                            if self.isDone(): break
                            # micro.reset_position()
                            for row in range(prow):
                                if self.isDone(): break
                                if not self.isDone():
                                    self.communicate.emit('Adjusting height')
                                    time.sleep(0.2)
                                    sensor.reset()
                                    time.sleep(0.2)
                                    dist = micro.estimate_dist(self.isDone, self.get_f, self.communicate)
                                    dist = int(self.place_params['height']) * 1000 - dist  # positive == downward
                                    if not self.isDone(): macro.move_rel(nt.Z, dist)
                                isRecovered, isPicked = True, False
                                if not self.isDone(): xx, yy, zz = macro.location()
                                time.sleep(0.2)
                                sensor.reset()
                                time.sleep(0.2)
                                if not self.isDone() and not self.p['no_push']:
                                    isRecovered = micro.push_island(**self.push_params, verbose=True,
                                                                    isDone=self.isDone, read_f=self.get_f,
                                                                    emitter=self.communicate)
                                if not self.isDone() and isRecovered:
                                    image_be = vision.get_img()
                                    sensor.reset()
                                    isPicked = micro.pick_island(**self.pick_params, verbose=True, isDone=self.isDone,
                                                                 read_f=self.get_f, emitter=self.communicate)
                                if not self.isDone() and isPicked:
                                    image_af = vision.get_img()
                                    #石墨岛中心和针尖中心的差距
                                    y1,x1 = vision.get_target_calibration(image_be, image_af)
                                    rel_x = int(self.loc[self.ctr][1])
                                    rel_y = int(self.loc[self.ctr][0])
                                    cur_x_tar, cur_y_tar, z1 = self.cal_trans_results([rel_x, rel_y, 0])
                                    if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                                    if not self.isDone(): zz2 = int(macro.get_pos(nt.Z))
                                    if not self.isDone(): macro.move_abs_xy(int(cur_x_tar) + int(x1*78) + xx4, int(cur_y_tar) + int(y1*78) + yy4)
                                    if not self.isDone(): macro.move_abs(nt.Z, int(z1))
                                    if not self.isDone():
                                        time.sleep(0.2)
                                        sensor.reset()
                                        time.sleep(0.2)
                                        success, dist1 = micro.place_island(**self.place_params, verbose=True,
                                                                            isDone=self.isDone, read_f=self.get_f,
                                                                         emitter=self.communicate)
                                        dist1 = int(self.estimate_params['height']) * 1000 - dist1
                                        if not self.isDone(): macro.move_rel(nt.Z, dist1)
                                        # place back == micro recenter
                                        micro.place_back(isDone=self.isDone)
                                        if not self.isDone() and success:
                                            self.befromloc.append(self.loc[self.ctr])
                                            self.befromloc[-1].append('y')
                                            self.ctr += 1
                                            self.ctr1 += 1
                                            self.communicate.emit('%d islands transfered' % self.ctr1)
                                            if self.ctr1 == len(self.fileloc):
                                                self.set_done()
                                        elif not self.isDone() and not success:
                                            self.set_done()
                                            # cx = self.p['c_x']
                                            # cy = self.p['c_y']
                                            # cz = self.p['c_z']
                                            # if not self.isDone(): macro.move_rel(nt.Z, 2 * nt.mm)
                                            # if not self.isDone(): macro.move_abs_xy(cx, cy)
                                            # if not self.isDone(): macro.move_abs(nt.Z, cz)
                                            # if self.clean is None:
                                            #     self.clean = serial.Serial(port="COM11",baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
                                            # assert self.clean.is_open
                                            # master = modbus_rtu.RtuMaster(self.clean)
                                            # master.set_timeout(5.0)
                                            # set_controller(self.clean, "out")
                                            # open_power(self.clean)
                                            # set_current(self.clean, 80)
                                            # set_voltage(self.clean, 500)
                                            # time.sleep(2)
                                            # close_power(self.clean)
                                    if not self.isDone(): macro.move_abs(nt.Z, zz2)
                                    if not self.isDone(): macro.move_abs_xy(xx, yy)
                                    if not self.isDone(): macro.move_abs(nt.Z, zz)
                                    if not self.isDone(): time.sleep(0.5)
                                    dist = micro.estimate_dist(self.isDone, self.get_f, self.communicate)
                                    dist = int(self.place_params['height']) * 1000 - dist  # positive == downward
                                    if not self.isDone(): macro.move_rel(nt.Z, dist)
                                if row == prow - 1 and col == pcol - 1: self.set_done()
                                if not self.isDone():
                                    img = vision.get_img()
                                    direction = self.directions[int(direction_mat[row, col])]
                                    row_dist, col_dist, find_land = vision.get_relative_dist(img,
                                                                                             self.p['size'], self.p['gap'] ,
                                                                                             direction)
                                    if not self.isDone():
                                        macro.move_rel(nt.Z, 10 * nt.um)
                                        time.sleep(0.2)
                                        macro.move_rel(nt.X, col_dist, wait=False)
                                        macro.move_rel(nt.Y, row_dist, wait=False)
                                        time.sleep(0.2)
                                        macro.move_rel(nt.Z, -10 * nt.um)
                                        while not find_land:
                                            img = vision.get_img()
                                            direction = self.directions[int(direction_mat[row,col])]
                                            print(direction)
                                            time.sleep(1.0)
                                            row_dist, col_dist, find_land = vision.get_relative_dist(img,
                                                                                                     self.p['size'], self.p['gap'] , direction)
                                            if not self.isDone():
                                                macro.move_rel(nt.Z, 10*nt.um)
                                                time.sleep(0.2)
                                                macro.move_rel(nt.X, col_dist, wait=False)
                                                macro.move_rel(nt.Y, row_dist, wait=False)
                                                time.sleep(0.2)
                                                macro.move_rel(nt.Z, -10*nt.um)
                                                time.sleep(0.5)
                                                prow = prow-1
                                                pcol = pcol-1
                                print(len(self.loc),self.ctr)
                                if len(self.loc) <= self.ctr: break
                                row1 = row
                                col1 = col
                            if len(self.loc) <= self.ctr: break
                            row1 = row
                            col1 = col
                        else:
                            continue



        except Exception:
            self.communicate.emit(traceback.format_exc())

        finally:
            # self.clean.flush()
            # self.clean.close()
            self.save()

    def cal_trans_matrix(self, cord_ori, cord_tar):
        bias_ori = np.array(cord_ori[0])
        bias_tar = np.array(cord_tar[0])
        Y_SUB_Y0 = np.array([np.array(y) - bias_tar for y in cord_tar[1:]])
        X_SUB_X0 = np.array([np.array(x) - bias_ori for x in cord_ori[1:]])
        # print(Y_SUB_Y0, np.cross(X_SUB_X0[0], X_SUB_X0[1])[None])
        X_add = np.concatenate((X_SUB_X0, np.cross(X_SUB_X0[0], X_SUB_X0[1])[None]), axis=0).T
        Y_add = np.concatenate((Y_SUB_Y0, np.cross(Y_SUB_Y0[0], Y_SUB_Y0[1])[None]), axis=0).T
        Trans = np.matmul(Y_add, np.linalg.inv(X_add))
        return Trans, bias_ori, bias_tar

    def cal_trans_results(self, cord_0):
        cord_0 = np.array(cord_0) - self.bias_ori
        results = np.matmul(self.trans_mat, cord_0).T + self.bias_tar
        return results

    def save(self):
        n = 0
        n1 = 0
        with open(time.strftime('./Array files/Temporary file/%d%M%S.txt'), 'w') as file:
            for line in self.befromloc:
                if line != []:
                    file.write('\t'.join(line) + '\n')
            for line in self.loc:
                if line != []:
                    if n1 < self.ctr:
                        n1 += 1
                    else:
                        file.write('\t'.join(line) + '\n')

            for line in self.markloc:
                if line != []:
                    n += 1
                    if n >= self.ctr2:
                        for i in line:
                            if i != []:
                                file.write('\t'.join(i) + '\n')


    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_done(self):
        self.done = True
        try:
            macro.stop(nt.X)
            macro.stop(nt.Y)
            macro.stop(nt.Z)
        except:
            pass

    def isDone(self):
        return self.done

class Main(Ui_MainWindow,QWidget):

    def __init__(self, MainWindow):
        super(Main, self).__init__()
        self.MainWindow = MainWindow
        self.setupUi(MainWindow)
        self.push_direction.lineEdit().setReadOnly(True)
        self.pick_direction.lineEdit().setReadOnly(True)
        self.img_viewer.setScaledContents(True)
        image = QtGui.QPixmap('./gui/bread_and_cheese.jpg')
        self.img_viewer.setPixmap(image)
        ports = [item[0] for item in list_ports.comports()]
        if len(ports) > 0:
            self.ports.addItems(ports)
            self.ports.setEditable(True)
            self.ports.lineEdit().setReadOnly(True)
        self.floatValidator = QDoubleValidator(0, 1000, 3)
        self.intValidator = QIntValidator(-99999999,99999999)
        self.array_size.setValidator(self.floatValidator)
        self.array_gap.setValidator(self.floatValidator)
        self.array_gap_tar.setValidator(self.floatValidator)
        self.push_force.setValidator(self.floatValidator)
        self.push_distance.setValidator(self.floatValidator)
        self.pick_force.setValidator(self.floatValidator)
        self.pick_distance.setValidator(self.floatValidator)
        self.place_fmin.setValidator(self.floatValidator)
        self.place_fmax.setValidator(self.floatValidator)
        self.place_height.setValidator(self.floatValidator)
        self.target_x.setValidator(self.intValidator)
        self.target_y.setValidator(self.intValidator)
        self.target_z.setValidator(self.intValidator)

        self.keystart.clicked.connect(self.grabMouse)
        self.keystart.clicked.connect(self.grabKeyboard)
        self.keyclose.clicked.connect(self.releaseKeyboard)
        self.connect_clean.clicked.connect(self.connect_clean_slot)
        self.connect_macro.clicked.connect(self.connect_macro_slot)
        self.connect_micro.clicked.connect(self.connect_micro_slot)
        self.connect_force.clicked.connect(self.connect_force_slot)
        self.reset_force.clicked.connect(self.reset_force_slot)
        self.get_xyz.clicked.connect(self.get_xyz_slot)
        self.get_xyz_3.clicked.connect(self.get_xyz_slot1)
        self.rezero_2.clicked.connect(self.goto_0a)
        self.goto_xyz.clicked.connect(self.goto_xyza)
        self.goto_xyz_2.clicked.connect(self.goto_X1Y1Z1a)
        self.goto_target.clicked.connect(self.goto_Targeta)
        # self.goto_target_3.clicked.connect(self.goto_Targeta)
        self.get_xyz_4.clicked.connect(self.get_clean)
        self.goto_target_2.clicked.connect(self.goto_Cleana)
        # self.goto_target_3.clicked.connect(self.goto_Targeta)
        self.get_xyz_2.clicked.connect(self.get_new_target)
        # self.get_xyz_5.clicked.connect(self.get_sign_target)
        # self.um4.clicked.connect(self.um4_slot)
        # self.um6.clicked.connect(self.um6_slot)
        # self.um8.clicked.connect(self.um8_slot)
        # self.um10.clicked.connect(self.um10_slot)
        self.push_only.clicked.connect(self.push_only_slot)
        self.no_push.clicked.connect(self.no_push_slot)
        self.start.clicked.connect(self.start_slot)
        self.start_2.clicked.connect(self.start_slot)
        self.start_3.clicked.connect(self.start_slot)
        self.start_4.clicked.connect(self.start_slot)
        self.rezero.clicked.connect(self.rezero_slot)
        self.get_xz.clicked.connect(self.get_xz_slot)
        self.goto_xz.clicked.connect(self.goto_xz_slot)
        self.recenter.clicked.connect(self.recenter_slot)
        self.calibrate.clicked.connect(self.calibrate_slot)
        self.point_1.clicked.connect(self.point_1_slot)
        self.point_2.clicked.connect(self.point_2_slot)
        self.point_5.clicked.connect(self.point_1_slot_3)
        self.point_6.clicked.connect(self.point_2_slot_3)
        self.zero_deg.clicked.connect(self.zero_deg_slot)
        self.zero_deg_3.clicked.connect(self.zero_deg_slot_3)
        self.openfile.clicked.connect(self.openfile_slot)
        self.markstart.clicked.connect(self.startmark_slot)
        self.startfile.clicked.connect(self.startfile_slot)
        self.get_mark1.clicked.connect(self.get_mark11)
        self.get_mark2.clicked.connect(self.get_mark21)
        self.get_mark3.clicked.connect(self.get_mark31)
        self.get_mark4.clicked.connect(self.get_mark41)
        self.mark1.clicked.connect(self.markloc1)
        self.mark2.clicked.connect(self.markloc2)
        self.mark3.clicked.connect(self.markloc3)
        self.mark4.clicked.connect(self.markloc4)
        self.up.clicked.connect(self.up1)
        self.down.clicked.connect(self.down1)
        self.left.clicked.connect(self.left1)
        self.right.clicked.connect(self.right1)
        self.up_1.clicked.connect(self.PageUp1)
        self.down_1.clicked.connect(self.Pagedown1)
        self.goto_mark_1.clicked.connect(self.goto_mark1a)
        self.goto_mark_2.clicked.connect(self.goto_mark2a)
        self.goto_mark_3.clicked.connect(self.goto_mark3a)
        self.goto_mark_4.clicked.connect(self.goto_mark4a)
        self.thread = None
        self.micro_port = None
        self.sensor_port = None
        self.file_name = None
        self.connect_macro_slot()
        self.connect_micro_slot()
        self.connect_force_slot()
        # self.connect_clean_slot()
        os.makedirs('./cache', exist_ok=True)

    def auto_size_slot(self):
        if int(self.array_size.text()) == 10:
            try:
                img = vision.get_img()
                size = vision.get_island_size(img)
                self.array_size.setText(str(size))
                self.array_gap.setText(str(size * 2))
            except:
                size = 10
                self.array_size.setText(str(size))
                self.array_gap.setText(str(size * 2))
            # push_force = 20 * size
            # self.push_force.setText(str(push_force))
            # self.push_distance.setText(str((size + 1) // 2))
            # pick_force = 20 * size
            # self.pick_force.setText(str(pick_force))
            # self.pick_distance.setText(str(size + 1))
        else:
            size = float(self.array_size.text())
        
        vision.set_crop_len(30 * size)

    def is_secure(self, secure):
        if not secure:
            try:
                self.thread.set_done()
            except:
                pass
            try:
                self.releaseKeyboard()
            except:
                pass
            try:
                macro.stop(nt.X)
                macro.stop(nt.Y)
                macro.stop(nt.Z)
                print("已停止")
            except:
                pass
            self.refresh_terminal("Maybe Collisions Happened")

    def mark_xyz(self,a,b,c):
        if a == "1":
            if b == "x":
                self.mark1_x.setText(str(c))

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            self.releaseKeyboard()
            self.releaseMouse()

    def keyReleaseEvent(self, QKeyEvent):
        if QKeyEvent.key() == Qt.Key_Up:
            macro.stop(nt.X)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        if QKeyEvent.key() == Qt.Key_Down:
            macro.stop(nt.X)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        if QKeyEvent.key() == Qt.Key_PageDown:
            macro.stop(nt.Z)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        if QKeyEvent.key() == Qt.Key_PageUp:
            macro.stop(nt.Z)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        if QKeyEvent.key() == Qt.Key_Right:
            macro.stop(nt.Y)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        if QKeyEvent.key() == Qt.Key_Left:
            macro.stop(nt.Y)
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            path = r'xyz/xyz38.txt'
            path1 = r'xyz/xyz35.txt'
            path2 = r'xyz/xyz36.txt'
            path3 = r'xyz/xyz37.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str((self.up_loc.text())))
            file.close()
            file = open(path1, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            file = open(path2, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
            file = open(path3, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()

    def keyPressEvent(self, QKeyEvent):
        if QKeyEvent.key() == Qt.Key_Up:
            try:
                macro.move(nt.Y, int(float(self.up_loc.text()) * 1000))
            except:
                pass

        if QKeyEvent.key() == Qt.Key_Down:
            try:
                macro.move(nt.Y, int(float(self.up_loc.text()) * -1000))
            except:
                pass

        if QKeyEvent.key() == Qt.Key_Left:
            try:
                macro.move(nt.X, int(float(self.up_loc.text()) * 1000))
            except:
                pass

        if QKeyEvent.key() == Qt.Key_Right:
            try:
                macro.move(nt.X, int(float(self.up_loc.text()) * -1000))
            except:
                pass

        if QKeyEvent.key() == Qt.Key_PageUp:
            try:
                macro.move(nt.Z, int(float(self.up_loc.text()) * -1000))
            except:
                pass

        if QKeyEvent.key() == Qt.Key_PageDown:
            try:
                macro.move(nt.Z, int(float(self.up_loc.text()) * 1000))
            except:
                pass

    def xyz(self):
        if xyz1:
            self.target_x.setText(str(xyz1[0]))
            self.target_y.setText(str(xyz1[1]))
            self.target_z.setText(str(xyz1[2]))
            self.mark1_x.setText(str(xyz1[0]))
            self.mark1_y.setText(str(xyz1[1]))
            self.mark1_z.setText(str(xyz1[2]))

    def up1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            x = int(int(self.up_loc.text())*1000) + int(xx)
            y = int(yy)
            macro.move_abs_xy(x, y)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass
    def down1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            x = int(int(self.up_loc.text())*1000) - int(xx)
            y = int(yy)
            macro.move_abs_xy(x, y)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass
    def right1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            x = int(xx)
            y = int(int(self.up_loc.text())*1000) + int(yy)
            macro.move_abs_xy(x, y)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass
    def left1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            x = int(xx)
            y = int(int(self.up_loc.text())*1000) - int(yy)
            macro.move_abs_xy(x, y)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass
    def PageUp1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            z_tar = int(int(self.up_loc.text())*1000) + int(zz)
            macro.move_abs(nt.Z, z_tar)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass
    def Pagedown1(self):
        try:
            xx, yy, zz = macro.location()
            self.cx.setText(str(xx))
            self.cy.setText(str(yy))
            self.cz.setText(str(zz))
            z_tar = int(int(self.up_loc.text()) * 1000) - int(zz)
            macro.move_abs(nt.Z, z_tar)
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(int(self.up_loc.text())))
            file.close()
        except:
            pass

    def gotomark1(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.mark1_x.text())
            y_tar = int(self.mark1_y.text())
            z_tar = int(self.mark1_z.text())
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def gotomark2(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.mark2_x.text())
            y_tar = int(self.mark2_y.text())
            z_tar = int(self.mark2_z.text())
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def gotomark3(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.mark3_x.text())
            y_tar = int(self.mark3_y.text())
            z_tar = int(self.mark3_z.text())
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def gotomark4(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.mark4_x.text())
            y_tar = int(self.mark4_y.text())
            z_tar = int(self.mark4_z.text())
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass


    def refresh_terminal(self, text):
        self.terminal.append(text)
        self.terminal.moveCursor(self.terminal.textCursor().End)

    def connect_macro_slot(self):
        # 连接宏动平台
        global macro
        try:
            macro.close()
        except:
            pass
        finally:
            macro = None
        device_ids = nt.NTcore.find_systems()
        if '' in device_ids:
            self.refresh_terminal('No macro device found')
        else:
            for _id in device_ids:
                try:
                    macro = nt.NTcore(_id)
                    loc = macro.location()
                    text = "Macro %s opened at X: %d, Y: %d, Z: %d" % (_id, loc[0], loc[1], loc[2])
                    self.refresh_terminal(text)
                    break
                except Exception as err:
                    if hasattr(macro, 'close'): macro.close()
                    macro = None
                    self.refresh_terminal(_id + ': ' + str(err))

    def connect_clean_slot(self):
        ser = serial.Serial(port="COM11", baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
        assert ser.is_open
        master = modbus_rtu.RtuMaster(ser)
        master.set_timeout(5.0)
        set_controller(ser, "out")
        open_power(ser)

    def connect_micro_slot(self):
        # 连接微动平台
        global micro
        try:
            micro.close()
        except:
            pass
        finally:
            micro = None
        port = 'COM6'
        try:
            assert port != '' and port != self.sensor_port, 'No port or reopened'
            micro = piezo.PiezoDevice(port=port, reset=cf.getint('basic', 'micro_rst'))
            xx, zz = micro.position['x'], micro.position['z']
            text = "Micro %s opened at X: %d, Z: %d" % (port, xx, zz)
            self.micro_port = port
        except Exception as err:
            if hasattr(micro, 'close'): micro.close()
            micro = None
            text = port + ' - ' + str(err)
        finally:
            self.refresh_terminal(text)

    def connect_force_slot(self):
        # 连接力传感器
        global sensor
        self.sensor_quit_slot()
        port = 'COM8'
        try:
            assert port != '' and port != self.micro_port, 'No port or reopened'
            sensor = Sensor(port)
            sensor.test()
            sensor.communicate.connect(self.F_display_slot)
            sensor.finished.connect(self.sensor_quit_slot)
            sensor.start()
            text = "Sensor %s opened" % (port)
            self.sensor_port = port
        except Exception as err:
            if hasattr(sensor, 'close_ser'): sensor.close_ser()
            sensor = None
            text = port + ' - ' + str(err)
        finally:
            self.refresh_terminal(text)

    def reset_force_slot(self):
        # 力传感器归零
        global sensor
        try:
            sensor.reset()
            self.refresh_terminal('Rezero Force successfully')
        except:
            pass

    def get_xyz_slot1(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.X_4.setText(str(xx))
            self.Y_2.setText(str(yy))
            self.Z_3.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz7.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz8.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz9.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_xyz_slot(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.X.setText(str(xx))
            self.Y.setText(str(yy))
            self.Z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz4.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz5.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz6.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def goto_xyz_slot(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.X.text())
            y_tar = int(self.Y.text())
            z_tar = int(self.Z.text())
            macro.move_rel(nt.Z, 2*nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def goto_xyz_slot1(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.X_4.text())
            y_tar = int(self.Y_2.text())
            z_tar = int(self.Z_3.text())
            macro.move_rel(nt.Z, 2*nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)

        except:
            pass

    def goto_zero(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(0)
            y_tar = int(0)
            z_tar = int(0)
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def get_clean(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.X_7.setText(str(xx))
            self.X_6.setText(str(yy))
            self.X_5.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz10.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz11.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz12.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def goto_clean(self):
        # 宏动前往输入位置
        global macro
        try:
            x_tar = int(self.X_7.text())
            y_tar = int(self.X_6.text())
            z_tar = int(self.X_5.text())
            macro.move_rel(nt.Z, 2 * nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
            ser = serial.Serial(port="COM11",baudrate=19200, bytesize=8, parity=serial.PARITY_EVEN, stopbits=1)
            assert ser.is_open
            master = modbus_rtu.RtuMaster(ser)
            master.set_timeout(5.0)
            set_controller(ser, "out")
            open_power(ser)
            set_current(ser, 80)
            set_voltage(ser, 500)
            time.sleep(2)
            close_power(ser)
        except:
            pass

    def get_new_target(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.target_x.setText(str(xx))
            self.target_y.setText(str(yy))
            self.target_z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz1.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz2.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz3.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_mark11(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.target_x.setText(str(xx))
            self.target_y.setText(str(yy))
            self.target_z.setText(str(zz))
            self.mark1_x.setText(str(xx))
            self.mark1_y.setText(str(yy))
            self.mark1_z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz23.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz24.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz25.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_mark21(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.mark2_x.setText(str(xx))
            self.mark2_y.setText(str(yy))
            self.mark2_z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz26.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz27.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz28.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_mark31(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.mark3_x.setText(str(xx))
            self.mark3_y.setText(str(yy))
            self.mark3_z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz29.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz30.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz31.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_mark41(self):
        # 宏动当前位置
        global macro
        try:
            xx, yy, zz = macro.location()
            self.mark4_x.setText(str(xx))
            self.mark4_y.setText(str(yy))
            self.mark4_z.setText(str(zz))
            self.refresh_terminal('Macro location received successfully')
            path = r'xyz/xyz32.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz33.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
            path = r'xyz/xyz34.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(zz))
            file.close()
        except:
            pass

    def get_sign_target(self):

        global macro
        try:
            img = vision.get_img()
            y,x = vision.get_relative_dist1(img)
            x1, y1 = vision.get_sign(img)
            xx, yy, zz = macro.location()
            xx = int(((x - x1)*78)+xx)
            yy = int(((y1 - y)*78)+yy)
            self.target_x.setText(str(xx))
            self.target_y.setText(str(yy))
            self.target_z.setText(str(zz))
            x1 = int(x1)
            y1 = int(y1)
            # image = img.convert("RGB").resize((1440, 1080), Image.NEAREST)
            # o_image = np.array(image.resize((1440, 1080), Image.NEAREST))
            # o_image[x1 - 4: x1 + 4, y1 - 4:y1 + 4, :] = np.array([0, 255, 0])
            # o_image[x - 4: x + 4, y - 4:y + 4, :] = np.array([255, 255, 0])
            # cv2.imwrite("./cache/o_img" + str(time.time()) + ".png", o_image[:, :, ::-1].astype(np.uint8))
            self.refresh_terminal('Macro location received successfully')
        except:
            pass

    def goto_target_slot(self):
        # 宏动前往目标栏的目标位置
        global macro
        try:
            x_tar = int(self.target_x.text())
            y_tar = int(self.target_y.text())
            z_tar = int(self.target_z.text())
            macro.move_rel(nt.Z, 2*nt.mm)
            macro.move_abs_xy(x_tar, y_tar)
            macro.move_abs(nt.Z, z_tar)
        except:
            pass

    def rezero_slot(self):
        # 宏动以当前位置为原点
        global macro
        try:
            macro.set_pos(nt.X, 0)
            macro.set_pos(nt.Y, 0)
            macro.set_pos(nt.Z, 0)
            self.refresh_terminal('Rezero Macro successfully')
        except:
            pass

    def get_xz_slot(self):
        # 微动当前位置
        global micro
        try:
            self.X_2.setText(str(micro.get_abs('x')))
            self.Z_2.setText(str(micro.get_abs('z')))
            path = r'xyz/xyz13.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(micro.get_abs('x')))
            file.close()
            path = r'xyz/xyz14.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(micro.get_abs('z')))
            file.close()
            self.refresh_terminal('Micro location received successfully')
        except:
            pass

    def goto_xz_slot(self):
        # 微动前往输入位置
        global micro
        try:
            x_tar = int(1000*float(self.X_2.text()))
            z_tar = int(1000*float(self.Z_2.text()))
            micro.move('x', x_tar)
            micro.move('z', z_tar)
        except:
            pass

    def recenter_slot(self):
        # 微动移动到归中位置
        global micro
        try:
            micro.recenter()
            self.refresh_terminal('Recenter Micro successfully')
        except:
            pass

    def calibrate_slot(self):
        # 校准针尖位置
        img = vision.get_img()
        vision.calibrate_tip_loc(img)
        self.X_3.setText(str(vision.get_tip_loc()[0]))
        self.Y_3.setText(str(vision.get_tip_loc()[1]))
        path = r'xyz/xyz20.txt'
        file = open(path, 'w+', encoding="utf-8")
        file.write(str(vision.get_tip_loc()[0]))
        file.close()
        path = r'xyz/xyz21.txt'
        file = open(path, 'w+', encoding="utf-8")
        file.write(str(vision.get_tip_loc()[1]))
        file.close()
        self.refresh_terminal('Calibrate tip location successfully')

    def point_1_slot(self):
        # 偏转角校准点1
        global macro, angle
        try:
            xx, yy, zz = macro.location()
            self.x_p1.setText(str(xx))
            self.y_p1.setText(str(yy))
            x_p2 = int(self.x_p2.text())
            y_p2 = int(self.y_p2.text())
            angle = math.atan((y_p2-yy)/(x_p2-xx))
            self.degree.setText(str(180*angle/math.pi))
            path = r'xyz/xyz19.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(180*angle/math.pi))
            file.close()
            path = r'xyz/xyz15.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz16.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        except:
            pass

    def point_1_slot_3(self):
        # 偏转角校准点1
        global macro, angle1
        try:
            xx, yy, zz = macro.location()
            self.x_p1_3.setText(str(xx))
            self.y_p1_3.setText(str(yy))
            x_p2_3 = int(self.x_p2_3.text())
            y_p2_3 = int(self.y_p2_3.text())
            angle1 = math.atan((y_p2_3-yy)/(x_p2_3-xx))
            self.degree_3.setText(str(180*angle1/math.pi))
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(180*angle1/math.pi))
            file.close()
            path = r'xyz/xyz39.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz40.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        except:
            pass


    def point_2_slot(self):
        # 偏转角校准点2
        global macro, angle
        try:
            xx, yy, zz = macro.location()
            self.x_p2.setText(str(xx))
            self.y_p2.setText(str(yy))
            x_p1 = int(self.x_p1.text())
            y_p1 = int(self.y_p1.text())
            angle = math.atan((y_p1-yy)/(x_p1-xx))
            self.degree.setText(str(180*angle/math.pi))
            path = r'xyz/xyz19.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(180*angle/math.pi))
            file.close()
            path = r'xyz/xyz17.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz18.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        except:
            pass

    def point_2_slot_3(self):
        # 偏转角校准点2
        global macro, angle1
        try:
            xx, yy, zz = macro.location()
            self.x_p2_3.setText(str(xx))
            self.y_p2_3.setText(str(yy))
            x_p1_3 = int(self.x_p1_3.text())
            y_p1_3 = int(self.y_p1_3.text())
            angle1 = math.atan((y_p1_3-yy)/(x_p1_3-xx))
            self.degree_3.setText(str(180*angle1/math.pi))
            path = r'xyz/xyz38.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(180*angle1/math.pi))
            file.close()
            path = r'xyz/xyz41.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(xx))
            file.close()
            path = r'xyz/xyz42.txt'
            file = open(path, 'w+', encoding="utf-8")
            file.write(str(yy))
            file.close()
        except:
            pass

    def zero_deg_slot(self):
        # 偏转角归零
        global angle
        self.x_p1.setText('0')
        self.y_p1.setText('0')
        self.x_p2.setText('0')
        self.y_p2.setText('0')
        self.degree.setText('0')
        angle = 0

    def zero_deg_slot_3(self):
        # 偏转角归零
        global angle1
        self.x_p1_3.setText('0')
        self.y_p1_3.setText('0')
        self.x_p2_3.setText('0')
        self.y_p2_3.setText('0')
        self.degree_3.setText('0')
        angle1 = 0

    def um4_slot(self):
        # 设置岛尺寸为4um
        self.array_size.setText('4')
        self.array_gap.setText('12')
        self.push_force.setText('100')
        self.push_distance.setText('2')
        self.pick_force.setText('80')
        self.pick_distance.setText('4.5')
        vision.set_crop_len(120)

    def um6_slot(self):
        self.array_size.setText('6')
        self.array_gap.setText('18')
        self.push_force.setText('200')
        self.push_distance.setText('3')
        self.pick_force.setText('200')
        self.pick_distance.setText('6.5')
        vision.set_crop_len(140)

    def um8_slot(self):
        self.array_size.setText('8')
        self.array_gap.setText('24')
        self.push_force.setText('300')
        self.push_distance.setText('4')
        self.pick_force.setText('250')
        self.pick_distance.setText('8.5')
        vision.set_crop_len(190)

    def um10_slot(self):
        self.array_size.setText('10')
        self.array_gap.setText('30')
        self.push_force.setText('350')
        self.push_distance.setText('5')
        self.pick_force.setText('300')
        self.pick_distance.setText('10.5')
        vision.set_crop_len(240)

    def push_only_slot(self):
        self.no_push.setChecked(False)

    def no_push_slot(self):
        self.push_only.setChecked(False)

    def F_display_slot(self, f):
        self.F.setText('%.4f' % f)

    def communication_slot(self, signal):
        # 界面显示信号槽
        if isinstance(signal, str):
            self.refresh_terminal(signal)
        elif isinstance(signal, Image.Image):
            img = ImageQt.toqpixmap(signal)
            self.img_viewer.setPixmap(img)
        elif torch.is_tensor(signal):
            self.refresh_terminal('Confidence on picking: %s' % str(signal.tolist()))
        elif isinstance(signal, tuple):
            if isinstance(signal[0], Image.Image):
                img = ImageQt.toqpixmap(signal[0])
                self.img_viewer.setPixmap(img)
                self.refresh_terminal('Confidence on recovery: %f' % signal[1])

    def finished_slot(self):
        # 动作完成或线程结束的信号
        self.thread.terminate()
        self.thread = None
        self.refresh_terminal('Finished/stopped')
        self.groupBox_force.setEnabled(True)
        self.groupBox_connect.setEnabled(True)
        # self.groupBox_island.setEnabled(True)
        self.groupBox_auto.setEnabled(True)
        self.groupBox_target.setEnabled(True)
        self.tabs_status.setEnabled(True)
        self.groupBox_action.setEnabled(True)
        self.start.setEnabled(True)
        self.stop.setEnabled(False)
        self.groupBox_action_2.setEnabled(True)
        self.groupBox_action_7.setEnabled(True)
        self.groupBox_action_11.setEnabled(True)
        self.groupBox_action_12.setEnabled(True)

    def start_slot(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        tab = self.groupBox_auto.currentIndex()
        tab = self.groupBox_auto.tabText(tab)
        if tab == 'Array' and macro is not None and micro is not None and sensor is not None:
            self.auto_size_slot()
            array_params = self.array_params()
            push_params = self.push_params()
            pick_params = self.pick_params()
            place_params = self.place_params()
            self.thread = ArrayActionThread(array_params, push_params, pick_params, place_params)
        elif tab == 'Push' and micro is not None and sensor is not None:
            params = self.push_params()
            self.thread = SingleActionThread('push_island', params)
        elif tab == 'Pick' and micro is not None and sensor is not None:
            params = self.pick_params()
            self.thread = SingleActionThread('pick_island', params)
        elif tab == 'Place' and micro is not None and sensor is not None:
            params = self.place_params()
            self.thread = SingleActionThread('place_island', params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            # sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.stop.setEnabled(True)
            self.start.setEnabled(False)


    def markloc1(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.place_params()
        self.thread = markActionThread("1",params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)


    def markloc2(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.place_params()
        self.thread = markActionThread("2",params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def markloc3(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.place_params()
        self.thread = markActionThread("3",params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def markloc4(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.place_params()
        self.thread = markActionThread("4",params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_xyza(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('xyz',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_xyza(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('xyz',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_X1Y1Z1a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('X1Y1Z1',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_Cleana(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('Clean',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_0a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('0',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_Targeta(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)



        self.thread = gotoActionThread('Target',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_mark1a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)
        self.thread = gotoActionThread('mark1',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_mark2a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)
        self.thread = gotoActionThread('mark2',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_mark3a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)
        self.thread = gotoActionThread('mark3',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)

    def goto_mark4a(self):
        # 创建线程并执行动作
        global macro, micro, sensor
        params = self.array_params()
        self.stop.setEnabled(True)


        self.thread = gotoActionThread('mark4',params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)



    def openfile_slot(self):
        # 指定自定义阵列化转移文件
        self.file_name = QFileDialog.getOpenFileName(None,'Select a file','./Array files','Text Files (*.txt)')[0]
        self.filename.setText(self.file_name.split('/')[-1])

    def startfile_slot(self):
        # 创建线程并执行自定义阵列化转移
        global micro, macro, sensor
        self.auto_size_slot()
        if micro is None or macro is None or sensor is None:
            pass
        elif self.file_name is None or self.file_name == '':
            pass
        else:
            array_params = self.array_params()
            push_params = self.push_params()
            pick_params = self.pick_params()
            place_params = self.place_params()
            file_params = self.file_params()
            self.thread = FileActionThread(array_params, push_params, pick_params, place_params, file_params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)
            self.stop.setEnabled(True)

    def startmark_slot(self):
        # 创建线程并执行自定义阵列化转移
        global micro, macro, sensor,xyz1,xyz2,xyz3
        try:
            self.auto_size_slot()
        except Exception:
            pass
        if micro is None or macro is None or sensor is None:
            pass
        elif self.file_name is None or self.file_name == '':
            pass
        if xyz1:
            self.target_x.setText(str(xyz1[0]))
            self.target_y.setText(str(xyz1[1]))
            self.target_z.setText(str(xyz1[2]))
            self.mark1_x.setText(str(xyz1[0]))
            self.mark1_y.setText(str(xyz1[1]))
            self.mark1_z.setText(str(xyz1[2]))
        if xyz2:
            self.mark2_x.setText(str(xyz2[0]))
            self.mark2_y.setText(str(xyz2[1]))
            self.mark2_z.setText(str(xyz2[2]))
        if xyz3:
            self.mark3_x.setText(str(xyz3[0]))
            self.mark3_y.setText(str(xyz3[1]))
            self.mark3_z.setText(str(xyz3[2]))
        else:

            array_params = self.array_params()
            push_params = self.push_params()
            pick_params = self.pick_params()
            place_params = self.place_params()
            file_params = self.file_params()
            self.thread = markfileActionThread(array_params, push_params, pick_params, place_params, file_params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.stop.clicked.connect(self.thread.set_done)
            sensor.comm_secure.connect(self.is_secure)
            sensor.communicate.connect(self.thread.set_f)
            self.thread.start()
            self.groupBox_force.setEnabled(False)
            self.groupBox_action_2.setEnabled(False)
            self.groupBox_action_7.setEnabled(False)
            self.groupBox_action_11.setEnabled(False)
            self.groupBox_action_12.setEnabled(False)
            self.groupBox_connect.setEnabled(False)
            # self.groupBox_island.setEnabled(False)
            self.groupBox_auto.setEnabled(False)
            self.groupBox_target.setEnabled(False)
            self.tabs_status.setEnabled(True)
            self.groupBox_action.setEnabled(True)
            self.start.setEnabled(False)
            self.stop.setEnabled(True)

    def array_params(self):
        return dict(
            rows = self.array_rows.value(),
            cols = self.array_cols.value(),
            size = float(self.array_size.text()),
            gap = float(self.array_gap.text()),
            rows_tar = int(self.array_rows_tar.value()),
            cols_tar = int(self.array_cols_tar.value()),
            gap_tar = float(self.array_gap_tar.text()),
            push_only = self.push_only.isChecked(),
            no_push = self.no_push.isChecked(),
            # material = 'metal' if self.metal.isChecked() else 'silicon',
            x_tar = int(self.target_x.text()),
            y_tar = int(self.target_y.text()),
            z_tar = int(self.target_z.text()),
            mark1_x = int(self.mark1_x.text()),
            mark1_y = int(self.mark1_y.text()),
            mark1_z = int(self.mark1_z.text()),
            mark2_x = int(self.mark2_x.text()),
            mark2_y = int(self.mark2_y.text()),
            mark2_z = int(self.mark2_z.text()),
            mark3_x = int(self.mark3_x.text()),
            mark3_y = int(self.mark3_y.text()),
            mark3_z = int(self.mark3_z.text()),
            mark4_x = int(self.mark4_x.text()),
            mark4_y = int(self.mark4_y.text()),
            mark4_z = int(self.mark4_z.text()),
            c_x = int(self.X_7.text()),
            c_y = int(self.X_6.text()),
            c_z = int(self.X_5.text()),
            x_0 = int(self.X.text()),
            y_0 = int(self.Y.text()),
            z_0 = int(self.Z.text()),
            x_1 = int(self.X_4.text()),
            y_1 = int(self.Y_2.text()),
            z_1 = int(self.Z_3.text()),
            x_p1 = int(self.x_p1.text()),
            x_p2 = int(self.x_p2.text()),
            y_p1 = int(self.y_p1.text()),
            y_p2 = int(self.y_p2.text()),
            x_p1_3 = int(self.x_p1_3.text()),
            x_p2_3 = int(self.x_p2_3.text()),
            y_p1_3 = int(self.y_p1_3.text()),
            y_p2_3 = int(self.y_p2_3.text())
        )

    def push_params(self):
        f = float(self.push_force.text())
        dist = float(self.push_distance.text())*1000
        return dict(f=f, dist=dist)

    def place_params(self):
        f_min = float(self.place_fmin.text())
        f_max = float(self.place_fmax.text())
        attempts = self.place_attempt.value()
        height = float(self.place_height.text())
        return dict(f_min=f_min, f_max=f_max, attempts=attempts, height=height)

    def pick_params(self):
        f = float(self.pick_force.text())
        dist = float(self.pick_distance.text())*1000
        attempts = self.pick_attempt.value()
        return dict(f=f, dist=dist, attempts=attempts)

    def file_params(self):
        with open(self.file_name, 'r') as file:
            lines = file.readlines()
            lines = [line.split() for line in lines]
        return lines

    def sensor_quit_slot(self):
        global sensor
        try:
            sensor.close()
            sensor.terminate()
        except:
            pass
        sensor = None

def loc(num):
    file1 = open("xyz/xyz" + str(num) + ".txt", 'r', encoding="utf-8")
    content1 = file1.read()
    file1.close()
    return content1

app = QApplication(sys.argv)
mainwindow = QMainWindow()
# w = win()
main = Main(mainwindow)
mainwindow.show()
# w.show()
sys.exit(app.exec_())



