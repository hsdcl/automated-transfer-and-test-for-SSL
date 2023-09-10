import matplotlib
import numpy as np
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets
from matplotlib.lines import Line2D
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

class MplPlot(FigureCanvas):
    def __init__(self, parent=None, width=5, height=3, dpi=100):
        plt.rcParams["font.sans-serif"] = ["Arial"]
        plt.rcParams["axes.unicode_minus"] = False

        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(MplPlot, self).__init__(self.fig)
        self.setParent(parent)
        self.axes = self.fig.add_subplot(111)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def add_line(self, x_data, y_data):
        self.line = Line2D(x_data, y_data)
        self.axes.cla()
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel("Force (uN)")
        # self.axes.grid(True)
        self.axes.set_xlim(np.min(x_data), np.max(x_data))
        self.axes.set_ylim(np.min(y_data), np.max(y_data)+0.1)
        self.axes.add_line(self.line)

    def clear(self):
        self.axes.cla()
        self.draw()