from PyQt5.Qt import *
from PyQt5.QtCore import pyqtSignal, QThread, QObject
from PyQt5 import QtCore
from ctypes import *
import cv2

dcforward = c_uint(0x2B)
dcbackward = c_uint(0x2D)


class AutoFocus(QThread):
    info = pyqtSignal(str)

    def __int__(self, microscope, focus):
        focus.set_step(10)
        for _ in range(2):
            focus.move(dcbackward)

        self.focus = focus
        self.microscope = microscope

    def auto_focus(self):
        image = self.microscope.get_image()
        score = 0
        pass


