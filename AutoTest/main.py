# -*- coding: utf-8 -*-

import json
import pdb

import numpy as np
import logging
import os
import time
import sys
import configparser
from PyQt5.Qt import *
from PyQt5.QtCore import pyqtSignal, QThread, QObject
from PyQt5 import QtCore
import threading
import motors
import vision
from GUI.AutoTest_V2_1 import Ui_AutoTest
from GUI.MPL_Plot import MplPlot
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT as NavigationToolbar
from scipy.interpolate import make_interp_spline
from qsensor import SensorPlot
from serial.tools import list_ports
from scope import VideoBox, VideoTimer
from functools import partial
import NTdevice as nt
import piezo
# import vision
import traceback
import keyboard
from PIL import Image
import copy

macro = None
micro = None
focus = None
microscope = None

cf = configparser.ConfigParser()
cf.read('./config.ini')

QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)


class SingleActionThread(QThread):
    """
        对焦，推岛，拾岛，放岛，磨合，单一动作
    """
    communicate = pyqtSignal(object)

    def __init__(self, action_name, params):
        super().__init__()
        self.action_name = action_name
        self.params = params
        self.done = False
        self.f = None
        if "F_min" in self.params:
            self.F_min = int(self.params["F_min"])
            del self.params["F_min"]
        if "F_max" in self.params:
            self.F_max = int(self.params["F_max"])
            del self.params["F_max"]

    def run(self):
        global micro
        time.sleep(0.5)
        try:
            if self.action_name in ['shear_island', 'push_island', 'pick_island', 'place_island']:
                # micro.reset_position()
                action = getattr(micro, self.action_name)
                action(**self.params, read_f=self.get_f, isDone=self.isDone, emitter=self.communicate)
            elif self.action_name == "Grind":
                micro.shear_island(**self.params, read_f=self.get_f, isDone=self.isDone, emitter=self.communicate)
            elif self.action_name == 'F_coeff':
                for force in range(self.F_min, self.F_max, 100):
                    micro.shear_island(**self.params, f=force, read_f=self.get_f, isDone=self.isDone, emitter=self.communicate)
        except Exception as err:
            self.communicate.emit(str(err))

    def set_done(self):
        self.done = True

    def isDone(self):
        return self.done

    def set_f(self, force):
        self.f = np.mean(force[0])

    def get_f(self):
        return self.f


class AutoMeasureThread(QThread):
    """
        可以自动选择半自动动作，按照给定的动作进行测试
    """
    communicate = pyqtSignal(object)
    directions = ["right", "down", "up"]

    def __init__(self, action_names, array_params, params, sensors):
        """
        :param action_names: 动作名称
        :param array_params: 包含要测试的岛的阵列行列数，是否记录力
        """
        super().__init__()
        self.action_names = action_names
        self.params = params
        self.array_params = array_params
        self.done = False
        self.f = None
        self.sensors = sensors
        self.event = threading.Event()
        if "F_min" in self.params[action_names[1]]:
            self.F_min = self.params[action_names[1]]["F_min"]
            del self.params[action_names[1]]["F_min"]
        if "F_max" in self.params[action_names[1]]:
            self.F_max = self.params[action_names[1]]["F_max"]
            del self.params[action_names[1]]["F_max"]
        self.prow = self.array_params["prow"]
        self.pcol = self.array_params["pcol"]
        self.direction_mat = vision.create_direction_matrix(self.prow, self.pcol)
        self.save_path = "./later_force"
        os.makedirs(self.save_path, exist_ok=True)

    def run(self):
        global micro
        global microscope
        global macro
        time.sleep(0.5)
        try:
            for col in range(self.pcol):
                if self.isDone(): break
                for row in range(self.prow):
                    if self.isDone(): break
                    img = Image.fromarray(microscope.get_img())
                    # self.communicate.emit(str(img.shape))
                    direction = self.directions[int(self.direction_mat[row, col])]
                    print(direction, self.direction_mat)
                    row_dist, col_dist, find_land = vision.get_relative_dist(img, self.array_params["size"],
                                                                  40, direction=direction)
                    # if not find_land:
                    #     continue
                    if not self.isDone():
                        macro.move_rel(nt.Z, 5*nt.um, wait=True)
                        time.sleep(1)
                        macro.move_rel(nt.Y, row_dist, wait=True)
                        time.sleep(1)
                        macro.move_rel(nt.X, -col_dist, wait=True)
                        # time.sleep(10)
                        for _ in range(5):
                            macro.move_rel(nt.Z, -1 * nt.um, wait=False)
                            time.sleep(0.5)
                    # time.sleep(10)
                    if not self.isDone():
                        Flag_SSL = True
                        for action_name in self.action_names:
                            if self.isDone(): break
                            if action_name in ['shear_island', 'push_island', 'pick_island', 'place_island']:
                                action = getattr(micro, action_name)
                                action(**self.params, read_f=self.get_f, isDone=self.isDone, emitter=self.communicate)
                            elif action_name == 'Grind':
                                amplitude = self.params[action_name]["amplitude"]
                                angle = self.params[action_name]["angle"]
                                frequency = self.params[action_name]["frequency"]
                                f = self.params[action_name]["f"]
                                print(amplitude, f)
                                if not self.isDone(): micro.move_with_force('z', f, 25000, 5000, self.isDone, read_f=self.get_f,
                                                                            emitter=self.communicate)
                                if not self.isDone(): micro.adjust_force_move(f, self.get_f, self.isDone)
                                lateral_list = []
                                sleep_time = 0.5 / frequency
                                v = amplitude / sleep_time
                                # self.sensors.rezero_lateral()
                                # self.event.wait(0.5)
                                for circle in range(self.params[action_name]["circ"]):

                                    self.sensors.start_memory()
                                    if not self.isDone(): micro.move_with_angle(angle, amplitude, v)
                                    if not self.isDone(): self.event.wait(sleep_time)
                                    force = copy.copy(self.sensors.memory)[1:]
                                    print("max", force)
                                    force_max = np.mean(force)
                                    self.sensors.clear_memory()
                                    self.sensors.start_memory()
                                    if not self.isDone(): micro.move_with_angle(angle, -amplitude, v)
                                    if not self.isDone(): self.event.wait(sleep_time)
                                    force = copy.copy(self.sensors.memory)[1:]
                                    print("min", force)
                                    force_min = np.mean(force)
                                    self.sensors.clear_memory()
                                    force = np.abs(force_max - force_min)/2
                                    lateral_list.append(force)
                                    self.communicate.emit("current friction is {} uN".format(force))
                                    micro.adjust_force_move(f, self.get_f, self.isDone)
                                    if len(lateral_list) > 30:
                                        if self.check_finish(copy.copy(lateral_list[-30:])):
                                            break
                                with open(os.path.join(self.save_path, time.strftime("grind_%m%d-%H%M.txt")), "w") as f:
                                    for force in lateral_list:
                                        f.write(str(force) + "\n")
                                if np.mean(lateral_list[-30:]) > 4:
                                    Flag_SSL = False
                                    # print(lateral_list)
                                    # micro.move("z", -10, 10)
                                    # break

                            elif action_name == 'F_coeff':
                                F_min = self.F_min #if Flag_SSL else 500
                                for nforce in range(F_min, self.F_max, 100):
                                    amplitude = self.params[action_name]["amplitude"]
                                    angle = self.params[action_name]["angle"]
                                    frequency = self.params[action_name]["frequency"]
                                    print(nforce)
                                    micro.adjust_force_move(nforce, self.get_f, self.isDone)
                                    lateral_list = []
                                    sleep_time = 0.5 / frequency
                                    v = amplitude / sleep_time
                                    # self.sensors.rezero_lateral()
                                    # self.event.wait(0.5)
                                    for circle in range(self.params[action_name]["circ"]):
                                        #
                                        self.sensors.start_memory()
                                        if not self.isDone(): micro.move_with_angle(angle, amplitude, v)
                                        if not self.isDone(): self.event.wait(sleep_time)
                                        force = copy.copy(self.sensors.memory)[1:]
                                        print("max", force)
                                        force_max = np.mean(force)
                                        self.sensors.clear_memory()
                                        self.sensors.start_memory()
                                        if not self.isDone(): micro.move_with_angle(angle, -amplitude, v)
                                        if not self.isDone(): self.event.wait(sleep_time)
                                        force = copy.copy(self.sensors.memory)[1:]
                                        print("min", force)
                                        force_min = np.mean(force)
                                        self.sensors.clear_memory()
                                        force = np.abs(force_max - force_min) / 2
                                        lateral_list.append(force)
                                        self.communicate.emit("current friction is {} uN".format(force))
                                        micro.adjust_force_move(nforce, self.get_f, self.isDone)
                                    with open(os.path.join(self.save_path,
                                                           time.strftime("F_coeff_normal{}_%m%d-%H%M.txt".format(nforce))),
                                              "w") as f:
                                        for force in lateral_list:
                                            f.write(str(force) + "\n")
                                if not self.isDone(): micro.move("z", -15, 10)

        except Exception as err:
            self.communicate.emit(str(err))

    def set_done(self):
        self.done = True

    def isDone(self):
        return self.done


    def process_force(self, force):
        # length = len(force)//3
        force = np.array(force)
        force = (force.max()- force.min())/2
        return force

    def check_finish(self, last_force):
        last_force = np.array(last_force)
        if ((last_force.max() - last_force.min()) / last_force.mean()) < 0.05:
            return True
        else:
            return False

    def set_f(self, force):
        self.f = np.mean(force[0])

    def get_f(self):
        return self.f

class Window(QMainWindow, Ui_AutoTest):
    def __init__(self, app):
        super(QMainWindow, self).__init__()
        # super(Ui_Form, self).__init__()
        self.app = app
        self.setup_ui()
        self.data = []
        self.sensor_object = None
        self.time0 = 0.
        self.sensor_thread = QThread()

        self.set_enabled()
        self.timer_location = QTimer()
        self.timer_location.timeout.connect(self.connect_location)
        self.timer_location.start(500)
        self.releaseKeyboard()
        self.set_init()
        self.connect_signals()
        self.sensors = None
        self.thread = None
        # self.StartButton.setEnabled(True)
        # self.StopButton.setEnabled(False)


    def setup_ui(self):
        self.setupUi(self)


    def connect_signals(self):
        self.SelectMicro.toggled.connect(self.connect_radio_button)
        self.SelectMacro.toggled.connect(self.connect_radio_button)
        self.SelectFocus.toggled.connect(self.connect_radio_button)
        self.ConnectMacro.clicked.connect(self.connect_macro_slot)
        self.ConnectMicro.clicked.connect(self.connect_micro_slot)
        self.ConnectFocus.clicked.connect(self.connect_focus_slot)
        self.ConnectNorm.clicked.connect(self.start_sensor)
        self.ConnectAll.clicked.connect(self.connect_all)
        self.SemiButton.accepted.connect(self.start_slot)
        self.SemiButton.rejected.connect(self.quit_slot)
        self.AutoButton.accepted.connect(self.start_auto_slot)
        self.AutoButton.rejected.connect(self.quit_slot)
        self.SetTipLoc.clicked.connect(self.set_tip_location)

    def connect_all(self):
        event = threading.Event()
        global focus, micro, macro, microscope
        self.connect_focus_slot()
        if focus is None:
            print("connect focus failed")
        else:
            self.FocusCheck.setChecked(True)
        event.wait(0.2)

        self.connect_macro_slot()
        if macro is None:
            print("connect macro failed")
        else:
            self.MacroCheck.setChecked(True)
        event.wait(0.2)

        self.connect_micro_slot()
        if micro is None:
            print("connect micro failed")
        else:
            self.MicroCheck.setChecked(True)
        event.wait(0.2)

        try:
            microscope.show()
            microscope.show_video_images()
            microscope.re_play()
            self.ScopeCheck.setChecked(True)
        except Exception:
            print("connect microscope failed")
        event.wait(1)

        self.start_sensor()
        if self.sensors is None:
            print("connect force sensor failed")
        else:
            self.NormalCheck.setChecked(True)
            self.LateralCheck.setChecked(True)

    def set_init(self):
        self.VelMicroX.setText("%.2f" % 100.0)
        self.VelMicroY.setText("%.2f" % 100.0)
        self.VelMicroZ.setText("%.2f" % 100.0)
        self.StepMicroX.setText("%.2f" % 1.0)
        self.StepMicroY.setText("%.2f" % 1.0)
        self.StepMicroZ.setText("%.2f" % 1.0)

        self.VelMacroX.setText("%.2f" % 1.0)
        self.VelMacroY.setText("%.2f" % 1.0)
        self.VelMacroZ.setText("%.2f" % 1.0)
        self.StepMacroX.setText("%.2f" % 1.0)
        self.StepMacroY.setText("%.2f" % 1.0)
        self.StepMacroZ.setText("%.2f" % 1.0)

        self.VelFocusZ.setText("%.2f" % 1.0)
        self.StepFocusZ.setText("%.2f" % 1.0)

    def set_enabled(self):
        self.ValueMacroX.setEnabled(False)
        self.ValueMacroY.setEnabled(False)
        self.ValueMacroZ.setEnabled(False)
        self.ValueMicroX.setEnabled(False)
        self.ValueMicroY.setEnabled(False)
        self.ValueMicroZ.setEnabled(False)
        self.ValueFocusZ.setEnabled(False)
        self.VelMacroX.setEnabled(False)
        self.VelMacroY.setEnabled(False)
        self.VelMacroZ.setEnabled(False)
        self.VelFocusZ.setEnabled(False)
        self.MicroCheck.setEnabled(False)
        self.MacroCheck.setEnabled(False)
        self.FocusCheck.setEnabled(False)
        self.LateralCheck.setEnabled(False)
        self.ScopeCheck.setEnabled(False)
        self.NormalCheck.setEnabled(False)

    def connect_focus_slot(self):
        global focus
        try:
            focus.close()
        except:
            pass
        finally:
            focus = None
        focus_id = motors.NT_Motor_Core.find_systems()
        focus = motors.NT_Motor_Core(focus_id)
        print("open focus motor success !!!")
        # try:
        #     pass

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
        print(device_ids)
        if '' in device_ids:
            print('No macro device found')
        else:
            for _id in device_ids:
                try:
                    macro = nt.NTcore(_id)
                    loc = macro.location()
                    self.VelMacroX.textChanged.connect(partial(self.set_value_slot, macro, "MacroX"))
                    self.VelMacroY.textChanged.connect(partial(self.set_value_slot, macro, "MacroY"))
                    self.VelMacroZ.textChanged.connect(partial(self.set_value_slot, macro, "MacroZ"))
                    text = "Macro %s opened at X: %d, Y: %d, Z: %d" % (_id, loc[0], loc[1], loc[2])
                    print(text)
                    break
                except Exception as err:
                    if hasattr(macro, 'close'): macro.close()
                    macro = None
                    print(_id + ': ' + str(err))

    # micro part
    def connect_micro_slot(self):
        # 连接微动平台
        global micro
        try:
            micro.close()
        except:
            pass
        finally:
            micro = None
        try:
            micro = piezo.MicroDevice("E-727", "./sdk/PI_GCS2_DLL_x64.dll", "0117025056")
            print("connect micro stage success")
            micro.recenter()
            xx, zz = micro.position[micro.axes_dict['x']], micro.position[micro.axes_dict['z']]
            text = "Micro opened at X: %d, Z: %d" % (xx, zz)
            if self.sensors is not None:
                print("connect normal force sensor to micro stage, c_force:", micro.force)
        except Exception as err:
            if hasattr(micro, 'close'): micro.close()
            micro = None
            text = str(err)
        finally:
            print(text)

    def set_value_slot(self, device, name):
        pass

    def connect_location(self):
        global macro, micro, focus
        if macro is not None:
            pos_macro = macro.location()
        else:
            pos_macro = [0., 0., 0.]
        self.ValueMacroX.setText('%.2f' % pos_macro[0])
        self.ValueMacroY.setText('%.2f' % pos_macro[1])
        self.ValueMacroZ.setText('%.2f' % pos_macro[2])

        if micro is not None:
            pos_micro = list(micro.get_abs().values())
        else:
            pos_micro = [0., 0., 0.]
        self.ValueMicroX.setText('%.2f' % pos_micro[0])
        self.ValueMicroY.setText('%.2f' % pos_micro[1])
        self.ValueMicroZ.setText('%.2f' % pos_micro[2])

        if focus is not None:
            pos_focus = focus.location
        else:
            pos_focus = 0.
        self.ValueFocusZ.setText('%.2f' % pos_focus)

    def connect_radio_button(self):
        if self.sender().text()=="Micro":
            if self.sender().isChecked() == True:
                self.SelectMicro.clearFocus()
                keyboard.hook(partial(self.MyKeyPressEvent, "Micro"))
            else:
                keyboard.unhook_all()
        if self.sender().text()=="Macro":
            if self.sender().isChecked() == True:
                self.SelectMacro.clearFocus()
                keyboard.hook(partial(self.MyKeyPressEvent, "Macro"))
            else:
                keyboard.unhook_all()
        if self.sender().text() == "Focus":
            if self.sender().isChecked() == True:
                self.SelectFocus.clearFocus()
                keyboard.hook(partial(self.MyKeyPressEvent, "Focus"))
            else:
                keyboard.unhook_all()

    def MyKeyPressEvent(self, name, event):
        # unit of micro um; unit of macro nm
        global micro, macro, focus
        if name == "Micro":
            if event.name == "left" and event.event_type == "down":
                step = float(self.StepMicroX.text())
                vel = float(self.VelMicroX.text())
                micro.move("x", value=-step, velocity=vel)
            if event.name == "right" and event.event_type == "down":
                step = float(self.StepMicroX.text())
                vel = float(self.VelMicroX.text())
                micro.move("x", value=step, velocity=vel)
            if event.name == "up" and event.event_type == "down":
                step = float(self.StepMicroY.text())
                vel = float(self.VelMicroY.text())
                micro.move("y", value=-step, velocity=vel)
            if event.name == "down" and event.event_type == "down":
                print("down")
                step = float(self.StepMicroY.text())
                vel = float(self.VelMicroY.text())
                micro.move("y", value=step, velocity=vel)
            if event.name == "page up" and event.event_type == "down":
                print("page up")
                step = float(self.StepMicroZ.text())
                vel = float(self.VelMicroZ.text())
                micro.move("z", value=step, velocity=vel)
            if event.name == "page down" and event.event_type == "down":
                print("page down")
                step = float(self.StepMicroZ.text())
                vel = float(self.VelMicroZ.text())
                micro.move("z", value=-step, velocity=vel)
        if name == "Macro":
            if event.name == "left" and event.event_type == "down":
                step = float(self.StepMacroX.text())*1000
                macro.move(nt.X, dist=-int(step))
            if event.name == "right" and event.event_type == "down":
                step = float(self.StepMacroX.text()) * 1000
                macro.move(nt.X, dist=int(step))
            if event.name == "up" and event.event_type == "down":
                step = float(self.StepMacroY.text()) * 1000
                macro.move(nt.Y, dist=int(step))
            if event.name == "down" and event.event_type == "down":
                step = float(self.StepMacroY.text()) * 1000
                macro.move(nt.Y, dist=-int(step))
            if event.name == "page up" and event.event_type == "down":
                step = float(self.StepMacroZ.text()) * 1000
                macro.move(nt.Z, dist=int(step))
            if event.name == "page down" and event.event_type == "down":
                step = float(self.StepMacroZ.text()) * 1000
                macro.move(nt.Z, dist=-int(step))
        if name == "Focus":
            if event.name == "page up" and event.event_type == "down":
                step = int(float(self.StepFocusZ.text()))
                focus.set_step(step)
                focus.move(focus.dcbackward)
                self.ValueFocusZ.setText(str(focus.location))
            if event.name == "page down" and event.event_type == "down":
                step = int(float(self.StepFocusZ.text()))
                focus.set_step(step)
                focus.move(focus.dcforward)
                self.ValueFocusZ.setText(str(focus.location))

    def start_sensor(self):
        global micro
        port = "COM12"
        self.sensors = SensorPlot(50, port)
        self.sensors.info.connect(self.communication_slot)
        try:
            # self.sensors = SensorPlot(50, port)
            self.sensors.show()
            text = "Sensor %s opened" % (port)
            self.sensor_port = port
            self.sensors.plot()
            # self.sensors.n_f.connect(self.get_force)
            # if micro is not None:
            #     self.sensors.n_f.connect(micro.set_force)
        except Exception as err:
            if hasattr(self.sensor_object, 'close_ser'): self.sensor_object.close_ser()
            self.sensor_object = None
            text = port + ' - ' + str(err)
        finally:
            # globals()[name] = sensor
            self.communication_slot(text)

    def get_force(self, force):
        self.force = force

    def start_slot(self):
        global macro, micro, focus
        tab = self.tabWidget_2.currentIndex()
        tab = self.tabWidget_2.tabText(tab)
        self.refresh_terminal("test_terminal")
        print(tab)
        if tab == 'Push' and micro is not None and self.sensors is not None:
            f = float(self.push_force.text())
            dist = float(self.push_distance.text()) * 1000
            params = dict(f=f, dist=dist)
            self.thread = SingleActionThread('push_island', params)
        elif tab == 'Pick' and micro is not None and self.sensors is not None:
            f = float(self.pick_force.text())
            dist = float(self.pick_distance.text()) * 1000
            attempts = self.pick_attempt.value()
            params = dict(f=f, dist=dist, attempts=attempts)
            self.thread = SingleActionThread('pick_island', params)
        elif tab == 'Place' and micro is not None and self.sensors is not None:
            f_min = float(self.place_fmin.text())
            f_max = float(self.place_fmax.text())
            attempts = int(self.place_attempt.text())
            height = float(self.place_height.text())
            params = dict(f_min=f_min, f_max=f_max, attempts=attempts, height=height)
            self.thread = SingleActionThread('place_island', params)
        elif tab == 'Shear' and micro is not None:
            angle = float(self.AngleEdit.text())
            frequency = float(self.FreqEdit.text())
            amplitude = float(self.AmpEdit.text())
            circ = int(self.CircEdit.text())
            f = None
            params = dict(angle=angle, f=f, frequency=frequency, circ=circ, amplitude=amplitude)
            self.thread = SingleActionThread("shear_island", params)
        elif tab == "F_coeff" and micro is not None:
            angle = float(self.Coeff_Angle.text())
            frequency = float(self.Coeff_Freq.text())
            amplitude = float(self.Coeff_Amp.text())
            circ = int(self.Coeff_Circles.text())
            F_min = float(self.Coeff_Fmin.text())
            F_max = float(self.Coeff_Fmax.text())
            params = dict(angle=angle, F_min=F_min, F_max=F_max, frequency=frequency, circ=circ, amplitude=amplitude)
            self.thread = SingleActionThread("F_coeff", params)
        elif tab == "Grind" and micro is not None:
            angle = float(self.Grind_Angle.text())
            frequency = float(self.Grind_Freq.text())
            amplitude = float(self.Grind_Amp.text())
            circ = int(self.Grind_Circles.text())
            f = float(self.Grind_Force.text())
            params = dict(angle=angle, f=f, frequency=frequency, circ=circ, amplitude=amplitude)
            self.thread = SingleActionThread("Grind", params)

        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.SemiButton.rejected.connect(self.thread.set_done)
            if self.sensors is not None:
                self.sensors.sensor_object.plot_data.connect(self.thread.set_f)
            self.thread.start()

    def start_auto_slot(self):
        global macro, micro, focus
        tab = self.tabWidget_3.currentIndex()
        tab = self.tabWidget_3.tabText(tab)
        self.refresh_terminal("test_auto_terminal")
        if tab == 'AutoTest' and micro is not None and self.sensors is not None:
            try:
                angle = float(self.Grind_Angle.text())
                frequency = float(self.Grind_Freq.text())
                amplitude = float(self.Grind_Amp.text())
                circ = int(self.Grind_Circles.text())
                f = float(self.Grind_Force.text())
            except Exception:
                angle = 90
                frequency = 0.8
                amplitude = 5
                circ = 200
                f = 200
            grind_params = dict(angle=angle, f=f, frequency=frequency, circ=circ, amplitude=amplitude)
            try:
                angle = float(self.Coeff_Angle.text())
                frequency = float(self.Coeff_Freq.text())
                amplitude = float(self.Coeff_Amp.text())
                circ = int(self.Coeff_Circles.text())
                F_min = int(self.Coeff_Fmin.text())
                F_max = int(self.Coeff_Fmax.text())
            except Exception:
                angle = 90
                frequency = 0.5
                amplitude = 5
                circ = 50
                F_min = 500
                F_max = 600
            F_coeff_params = dict(angle=angle, F_min=F_min, F_max=F_max, frequency=frequency, circ=circ, amplitude=amplitude)
            try:
                column = int(self.Columns.text())
                row = int(self.Rows.text())
                size = float(self.Size.text())
            except Exception:
                column = 2
                row = 2
                size = 10
            actions = [self.Action1.text(), self.Action2.text()]
            params = dict(Grind=grind_params, F_coeff=F_coeff_params)
            for action in actions:
                params[action] = grind_params if action == 'Grind' else F_coeff_params
            array_params = dict(pcol=column, prow=row, size=size)
            self.thread = AutoMeasureThread(actions, array_params, params, sensors=self.sensors)
        elif tab == 'AutoTrans' and micro is not None and self.sensors is not None:
            f = float(self.pick_force.text())
            dist = float(self.pick_distance.text()) * 1000
            attempts = self.pick_attempt.value()
            params = dict(f=f, dist=dist, attempts=attempts)
            self.thread = SingleActionThread('pick_island', params)
        if self.thread is not None:
            self.thread.communicate.connect(self.communication_slot)
            self.thread.finished.connect(self.finished_slot)
            self.SemiButton.rejected.connect(self.thread.set_done)
            if self.sensors is not None:
                self.sensors.sensor_object.plot_data.connect(self.thread.set_f)
            self.thread.start()
        else:
            self.refresh_terminal("Thread is None")


    def quit_slot(self):
        if self.thread is not None:
            self.thread.set_done()

    def finished_slot(self):
        # 动作完成或线程结束的信号
        self.thread.terminate()
        self.thread = None
        self.refresh_terminal('Finished/stopped')

    def communication_slot(self, signal):
        # 界面显示信号槽
        if isinstance(signal, str):
            self.refresh_terminal(signal)
        else:
            pass

    def refresh_terminal(self, text):
        self.textBrowser.append(text)
        self.textBrowser.moveCursor(self.textBrowser.textCursor().End)
        self.textBrowser_2.append(text)
        self.textBrowser_2.moveCursor(self.textBrowser_2.textCursor().End)

    def set_tip_location(self):
        global microscope
        if microscope is not None:
            img = Image.fromarray(microscope.get_img())
            tip_loc = vision.calibrate_tip_loc(img)
            self.Tip_x.setText(str(tip_loc[0]))
            self.Tip_y.setText(str(tip_loc[1]))
        else:
            self.refresh_terminal("Microscope is None")


def sleep(times):
    print("times:",times)


def main():
    print("start")
    global microscope
    app = QApplication(sys.argv)
    mywindow = Window(app)

    microscope = VideoBox()
    mywindow.ShowImage.clicked.connect(microscope.show)
    mywindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    print("start")
    main()