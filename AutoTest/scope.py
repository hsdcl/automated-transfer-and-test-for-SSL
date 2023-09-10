import sys

import pymmcore
import os.path
import pdb
import cv2
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import time
import numpy as np
import tkinter as tk

mm_dir = "C:\\Program Files\\Micro-Manager-2.0"
root = tk.Tk()
pos = (root.winfo_screenwidth()//2, root.winfo_screenheight()//2)
# print(pos)
root.destroy()

# startContinuousSequenceAcquisition
# stopSequenceAcquisition
# snapImage

class VideoBox(QWidget):

    VIDEO_TYPE_OFFLINE = 0
    VIDEO_TYPE_REAL_TIME = 1

    STATUS_INIT = 0
    STATUS_PLAYING = 1
    STATUS_PAUSE = 2

    def __init__(self, video_type=VIDEO_TYPE_OFFLINE, auto_play=False, frequecy=20):
        QWidget.__init__(self)
        self.video_type = video_type  # 0: offline  1: realTime
        self.auto_play = auto_play
        self.status = self.STATUS_INIT  # 0: init 1:playing 2: pause
        self.mmc = pymmcore.CMMCore()
        self.mmc.setDeviceAdapterSearchPaths([mm_dir])
        self.mmc.loadSystemConfiguration(os.path.join(mm_dir, "MMConfig_GRYPHAX.cfg"))
        self.image = None

        # 组件展示
        # self.pictureLabel = QLabel()
        self.graphicsView = QGraphicsView()
        self.graphicsView.setStyleSheet("padding: 0px; border: 0px;")
        self.graphicsView.setSceneRect(0, 0, self.graphicsView.viewport().width(),
                                       self.graphicsView.height())
        self.pictureLabel = QGraphicsScene(self)
        self.graphicsView.setScene(self.pictureLabel)
        self.pixmap = QPixmap("./cat.jpeg").scaled(self.width(), self.height())
        self.pixmapItem = self.pictureLabel.addPixmap(self.pixmap)
        self.pictureLabel.mousePressEvent = self.scene_mousePressEvent
        self.pictureLabel.mouseMoveEvent = self.scene_mouseMoveEvent
        self.pictureLabel.wheelEvent = self.scene_wheelEvent
        self.ratio = 1.
        self.o_position = (-640,-300)
        self.position = (-640,-300)
        self.zoom_step = 0.1
        self.zoom_max = 3.
        self.zoom_min = 0.2

        self.playButton = QPushButton()
        self.playButton.setEnabled(True)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.switch_video)

        control_box = QHBoxLayout()
        control_box.setContentsMargins(0, 0, 0, 0)
        control_box.addWidget(self.playButton)

        layout = QVBoxLayout()
        layout.addWidget(self.graphicsView)
        layout.addLayout(control_box)

        self.setLayout(layout)

        # timer 设置
        self.timer = VideoTimer()
        self.timer.timeSignal.signal[str].connect(self.show_video_images)

        # video 初始设置
        self.mmc.startContinuousSequenceAcquisition(10)
        self.set_timer_fps(frequecy)

    def set_timer_fps(self, fps):
        self.timer.set_fps(fps)

    def stop(self):
        if self.mmc is not None:
            self.timer.stop()
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.status = VideoBox.STATUS_PAUSE

    def re_play(self):
        self.timer.start()
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        self.status = VideoBox.STATUS_PLAYING

    def close(self):
        self.mmc.reset()
        self.timer.stop()

    def show_video_images(self):
        if self.mmc.getRemainingImageCount() > 0:
            rgb32 = self.mmc.getLastImage()
            frame = rgb32.view(dtype=np.uint8).reshape(rgb32.shape[0], rgb32.shape[1], 4)[..., :3]
            self.image = frame
            height, width = frame.shape[:2]
            if frame.ndim == 3:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            elif frame.ndim == 2:
                rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            temp_image = QImage(rgb.flatten(), width, height, QImage.Format_RGB888)
            self.pictureLabel.clear()
            self.pixmapItem = self.pictureLabel.addPixmap(QPixmap.fromImage(temp_image))
            self.pixmapItem.setScale(self.ratio)
            if isinstance(self.position, tuple):
                self.pixmapItem.setPos(self.position[0],self.position[1])
            else:
                self.pixmapItem.setPos(self.position)
        # else:
        #     print("open file or capturing device error, init again")
        #     self.reset()
        if self.mmc.getRemainingImageCount() > 10:
            self.mmc.clearCircularBuffer()

    def get_img(self):
        return self.image

    def scene_mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:  # 左键按下
            self.preMousePosition = event.scenePos()  # 获取鼠标当前位置
            self.timer.stop()
        if event.button() == Qt.RightButton:  # 右键按下
            self.ratio = 1.
            self.pixmapItem.setScale(1.)
            self.pixmapItem.setPos(self.o_position)

    def scene_mouseMoveEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            # print("左键移动")  # 响应测试语句
            self.MouseMove = event.scenePos() - self.preMousePosition  # 鼠标当前位置-先前位置=单次偏移量
            self.preMousePosition = event.scenePos()  # 更新当前鼠标在窗口上的位置，下次移动用
            # self.pixmapItem.setPos(self.pixmapItem.pos() + self.MouseMove)  # 更新图元位置
            self.position = self.pixmapItem.pos() + self.MouseMove
            self.timer.start()

    def scene_wheelEvent(self, event):
        angle = event.delta() / 8  # 返回QPoint对象，为滚轮转过的数值，单位为1/8度
        if angle > 0:
            # print("滚轮上滚")
            self.ratio += self.zoom_step  # 缩放比例自加
            if self.ratio > self.zoom_max:
                self.ratio = self.zoom_max
            else:
                w = self.pixmap.size().width() * (self.ratio - self.zoom_step)
                h = self.pixmap.size().height() * (self.ratio - self.zoom_step)
                x1 = self.pixmapItem.pos().x()  # 图元左位置
                x2 = self.pixmapItem.pos().x() + w  # 图元右位置
                y1 = self.pixmapItem.pos().y()  # 图元上位置
                y2 = self.pixmapItem.pos().y() + h  # 图元下位置
                if x1 < event.scenePos().x() < x2 and y1 < event.scenePos().y() < y2:  # 判断鼠标悬停位置是否在图元中
                    # print('在内部')
                    self.pixmapItem.setScale(self.ratio)  # 缩放
                    a1 = event.scenePos() - self.pixmapItem.pos()  # 鼠标与图元左上角的差值
                    a2 = self.ratio / (self.ratio - self.zoom_step) - 1  # 对应比例
                    delta = a1 * a2
                    # self.pixmapItem.setPos(self.pixmapItem.pos() - delta)
                    self.position = self.pixmapItem.pos() - delta

                else:
                    # print('在外部')  # 以图元中心缩放
                    self.pixmapItem.setScale(self.ratio)  # 缩放
                    delta_x = (self.pixmap.size().width() * self.zoom_step) / 2  # 图元偏移量
                    delta_y = (self.pixmap.size().height() * self.zoom_step) / 2
                    self.position = (self.pixmapItem.pos().x() - delta_x, self.pixmapItem.pos().y() - delta_y)
                    # self.pixmapItem.setPos(self.pixmapItem.pos().x() - delta_x,
                    #                        self.pixmapItem.pos().y() - delta_y)  # 图元偏移
        else:
            # print("滚轮下滚")
            self.ratio -= self.zoom_step
            if self.ratio < 0.2:
                self.ratio = 0.2
            else:
                w = self.pixmap.size().width() * (self.ratio + self.zoom_step)
                h = self.pixmap.size().height() * (self.ratio + self.zoom_step)
                x1 = self.pixmapItem.pos().x()
                x2 = self.pixmapItem.pos().x() + w
                y1 = self.pixmapItem.pos().y()
                y2 = self.pixmapItem.pos().y() + h
                # print(x1, x2, y1, y2)
                if x1 < event.scenePos().x() < x2 and y1 < event.scenePos().y() < y2:
                    # print('在内部')
                    self.pixmapItem.setScale(self.ratio)  # 缩放
                    a1 = event.scenePos() - self.pixmapItem.pos()  # 鼠标与图元左上角的差值
                    a2 = self.ratio / (self.ratio + self.zoom_step) - 1  # 对应比例
                    delta = a1 * a2
                    self.position = self.pixmapItem.pos() - delta
                else:
                    # print('在外部')
                    self.pixmapItem.setScale(self.ratio)
                    delta_x = (self.pixmap.size().width() * self.zoom_step) / 2
                    delta_y = (self.pixmap.size().height() * self.zoom_step) / 2
                    self.position = (self.pixmapItem.pos().x() + delta_x, self.pixmapItem.pos().y() + delta_y)
                    # self.pixmapItem.setPos(self.pixmapItem.pos().x() + delta_x, self.pixmapItem.pos().y() + delta_y)

    def switch_video(self):
        if self.mmc is None:
            return
        if self.status is VideoBox.STATUS_INIT:
            self.timer.start()
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        elif self.status is VideoBox.STATUS_PLAYING:
            self.timer.stop()
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        elif self.status is VideoBox.STATUS_PAUSE:
            self.timer.start()
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))

        self.status = (VideoBox.STATUS_PLAYING,
                       VideoBox.STATUS_PAUSE,
                       VideoBox.STATUS_PLAYING)[self.status]

class Communicate(QObject):

    signal = pyqtSignal(str)


class VideoTimer(QThread):

    def __init__(self, frequent=20):
        QThread.__init__(self)
        self.stopped = False
        self.frequent = frequent
        self.timeSignal = Communicate()
        self.mutex = QMutex()

    def run(self):
        with QMutexLocker(self.mutex):
            self.stopped = False
        while True:
            if self.stopped:
                return
            self.timeSignal.signal.emit("1")
            time.sleep(1 / self.frequent)

    def stop(self):
        with QMutexLocker(self.mutex):
            self.stopped = True

    def is_stopped(self):
        with QMutexLocker(self.mutex):
            return self.stopped

    def set_fps(self, fps):
        self.frequent = fps

if __name__ == "__main__":
    mapp = QApplication(sys.argv)
    mw = VideoBox()
    mw.show()
    sys.exit(mapp.exec_())