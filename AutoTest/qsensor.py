# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal, QObject
import serial
import time
import numpy as np
from GUI.Sensors import Ui_Sensors
from PyQt5 import QtCore
import os
import ctypes

#力传感控制文件

class Sensor(QObject):
    '''Thread that runs a serial'''
    # 3.601E-2 不抖的力传感
    #  8.73583E-1  抖的力传感
    # 50: 35 66   1k: 35 6B
    dSensitivity = [7.684017e-2, 3.933847e-2]
    timeout = 1
    plot_data = pyqtSignal(list)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.port_settings = {
            'baudrate': 921600,
            'bytesize': 8,
            'parity': serial.PARITY_NONE,
            'stopbits': 1,
            'timeout': 0.02
        }
        self.status = 0
        self.value = None
        self.zero_value = None
        # self.ser = None
        self.ser = serial.Serial(port, **self.port_settings)
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        # self.ser.writable()
        # self.ser.flushInput()



    def run(self):
        # print(self.status, int(QThread.currentThreadId()))
        while True:
            if self.status == 0: # 正常读取
                self.value = self.read()
                self.plot_data.emit(self.value)
            elif self.status == 1: # 归零信号
                self.reset_zero()
                # print(self.ser.write(ctypes.c_char_p("5k\n".encode('utf-8'))))
                self.set_status(0)
            else:
                self.close_ser()
                break

    def read_raw(self):
        cur_time = time.time()
        while True:
            time.sleep(0.01)
            # x = [np.random.randn()*10, np.random.randn()*10]
            # # print(x)
            # return x
            # if self.ser.in_waiting >= 20:
            #     data = self.ser.read(self.ser.in_waiting)
            #     print(data)
            normal_data, lateral_data = None, None
            if self.ser.in_waiting >= 20:
                # print(self.ser.in_waiting)
                data = self.ser.read(self.ser.in_waiting)
                # decode_data = [x for x in data.decode().split("\r\n")]
                # # data = []
                # print(decode_data)
                # self.ser.flushInput()
                try:
                    decode_data = [x for x in data.decode().split("\r\n")]
                    # print(decode_data)
                #
                except Exception:
                    self.close_ser()
                # # print(decode_data)
                normal_data = [int(ddata[:-2], 16) for ddata in decode_data if ddata[-2:]=='0b' and len(ddata)==8]
                lateral_data = [int(ddata[:-2], 16) for ddata in decode_data if ddata[-2:]=='0d' and len(ddata)==8]
                # print(decode_data)
                # for ddata in decode_data:
                #     if ddata[-2:] == '0b':
                #         normal_data = int(ddata[:-2], 16)
                #     elif ddata[-2:] == '0d':
                #         lateral_data = int(ddata[:-2], 16)
                #     # print(data.decode().split("\r\n"))
                if len(normal_data) and len(lateral_data):
                    return [normal_data, lateral_data] # buffer里数据可能会积累，所以一定是从后往前取位，
                elif len(normal_data):
                    return [normal_data, [0.]*len(normal_data)]
                # 数据格式我忘了，可以print出来看看，print(data)
                else:
                    continue
                # return [[0.],[0.]]
            if time.time() - cur_time > self.timeout:
                # return np.random.randn()
                raise TimeoutError('Error: sensor failed to read value in %fs' % self.timeout)

    def read(self):
        raw_value = self.read_raw()
        normal_value = [(raw - self.zero_value[0])*self.dSensitivity[0] for raw in raw_value[0]]
        lateral_value = [(raw - self.zero_value[1])*self.dSensitivity[1] for raw in raw_value[1]]
        value = [normal_value, lateral_value]
        return value

    def reset_zero(self):
        # time.sleep(0.5)
        zero_raw = self.read_raw()
        self.zero_value = [np.mean(x) for x in zero_raw]

    def rezero_lateral(self):
        # time.sleep(0.5)
        zero_raw = self.read_raw()[1]
        self.zero_value = [self.zero_value[0], np.mean(zero_raw)]

    def test(self):
        # 这个test本身没有问题，但设备有时候抽风返回超时，多试几次或者重启设备就行
        
        self.reset_zero()
        for i in range(20):
            self.value = self.read()
        # self.ser.write(bytes.fromhex('35 6B'))

    def set_status(self, status):
        self.status = status

    def reset(self):
        self.set_status(1)

    def reset_lateral(self):
        self.set_status(2)

    def close(self):
        self.set_status(-1)
        # time.sleep(0.5)

    def close_ser(self):
        try:
            print("closed")
            self.ser.close()
        except:
            pass

    def __del__(self):
        self.close()
        self.close_ser()

    def resetting(self):
        self.reset_zero()
        self.set_status(0)

class SensorPlot(Ui_Sensors):
    info = pyqtSignal(str)
    def __init__(self, frequency, port):
        super(SensorPlot, self).__init__()
        self.frequency = frequency
        self.port = port
        self.timer_plot = QtCore.QTimer(self)
        self.timer_force = QtCore.QTimer(self)
        self.sensor_thread = QThread()
        self.sensor_object = None
        self.Connect.clicked.connect(self.try_plot)
        self.Disconnect.clicked.connect(self.sensor_quit_slot)
        self.Rezero.clicked.connect(self.rezero)
        self.Record.clicked.connect(self.record)
        self.time_data, self.normal_data, self.lateral_data = np.arange(-10, 0, 0.02), [], []
        ydata_normal, ydata_lateral = np.zeros(self.time_data.shape).tolist(), np.zeros(self.time_data.shape).tolist()
        self.ydata = {"sensor_normal": ydata_normal, "sensor_lateral": ydata_lateral}
        self.normal, self.lateral = [0.], [0.]
        self.record_flag = False
        self.memory_flag = False
        self.file = None
        self.memory = []

    def try_plot(self):
        try:
            self.plot()
            self.info.emit("connect force sensors succeed")
        except Exception as err:
            self.info.emit(str(err))

    def plot(self):
        self.sensor_object = Sensor(self.port)
        self.sensor_object.test()
        self.sensor_object.plot_data.connect(self.get_force)
        self.sensor_object.moveToThread(self.sensor_thread)
        self.sensor_thread.started.connect(self.sensor_object.run)
        # self.sensor_thread.finished.connect(self.sensor_quit_slot)
        self.sensor_thread.start()
        self.time0 = time.time()
        self.time1 = time.time()
        self.timer_plot.timeout.connect(self.plot_force)
        self.timer_force.timeout.connect(self.get_force_time)
        self.timer_plot.start(self.frequency)
        self.timer_force.start(20)

    def record(self):
        if not self.record_flag:
            os.makedirs("./force", exist_ok=True)
            name = os.path.join("./force", time.strftime("%m%d-%H%M%S")+".txt")
            self.file = open(name, "w")
        else:
            self.file.close()
            self.file = None
        self.record_flag = not self.record_flag

    def get_force(self, force):
        self.normal = force[0]
        self.lateral = force[1]

    def get_force_time(self):
        c_time = time.time()
        if (c_time-self.time1) > 1:
            self.time1 = c_time
            self.time_data = np.arange(c_time-self.time0-10,c_time-self.time0, 0.02)

        self.normal_data.extend(self.normal)
        self.lateral_data.extend(self.lateral)
        if self.record_flag and self.file is not None:
            self.file.write("{}\t{}\n".format(self.normal, self.lateral))
        else:
            pass

    def rezero(self):
        self.time0 = time.time()
        self.timer_plot.stop()
        self.timer_force.stop()
        self.time_data = np.arange(-10, 0, 0.02)
        ydata_normal, ydata_lateral = np.zeros(self.time_data.shape).tolist(), np.zeros(self.time_data.shape).tolist()
        self.ydata = {"sensor_normal": ydata_normal, "sensor_lateral": ydata_lateral}
        self.normal_data, self.lateral_data = [], []
        self.clear()
        self.sensor_object.reset()
        self.timer_plot.start(self.frequency)
        self.timer_force.start(20)
        # self.phase_fig = {"sensor_normal": phase_fig_normal, "sensor_lateral": phase_fig_lateral}

    def rezero_lateral(self):
        self.sensor_object.rezero_lateral()

    def start_memory(self):
        self.memory_flag = True

    def clear_memory(self):
        self.memory_flag = False
        self.memory = []

    def plot_force(self):
        length_normal = len(self.normal_data)
        length_lateral = len(self.lateral_data)
        if self.memory_flag:
            self.memory.extend(self.lateral)
        # print(length_normal, length_lateral)
        # print(self.time_data, self.normal_data, self.lateral_data)
        del self.ydata["sensor_normal"][:length_normal]
        self.ydata["sensor_normal"].extend(self.normal_data)
        del self.ydata["sensor_lateral"][:length_lateral]
        self.ydata["sensor_lateral"].extend(self.lateral_data)
        # print(len(self.ydata["sensor_normal"]), self.time_data.shape)
        try:
            self.phase_fig["sensor_normal"].add_line(self.time_data, self.ydata["sensor_normal"])
            self.phase_fig["sensor_normal"].draw()
            self.phase_fig["sensor_lateral"].add_line(self.time_data, self.ydata["sensor_lateral"])
            self.phase_fig["sensor_lateral"].draw()
        except Exception:
            pass

        self.normal_data, self.lateral_data = [], []

    def sensor_quit_slot(self):
        try:
            if self.sensor_object is not None:
                self.sensor_object.plot_data.disconnect()
                self.timer_plot.stop()
                self.timer_force.stop()
                self.timer_plot.disconnect()
                self.timer_force.disconnect()
                self.sensor_object.close()
                self.sensor_thread.quit()
                self.clear()
        except Exception as err:
            print(err)
        self.sensor_object = None
        self.time_data = np.arange(-10, 0, 0.02)
        ydata_normal, ydata_lateral = np.zeros(self.time_data.shape).tolist(), np.zeros(self.time_data.shape).tolist()
        self.ydata = {"sensor_normal": ydata_normal, "sensor_lateral": ydata_lateral}
        self.normal_data, self.lateral_data = [], []


