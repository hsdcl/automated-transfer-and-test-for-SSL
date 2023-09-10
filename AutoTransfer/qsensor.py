from PyQt5.Qt import QThread, pyqtSignal
import serial
import time

#力传感控制文件

class Sensor(QThread):
    '''Thread that runs a serial'''
    # -3.601E-2 不抖的力传感
    #  -8.73583E-1  抖的力传感
    dSensitivity = 176.25E-2
    timeout = 1
    communicate = pyqtSignal(float)
    comm_secure = pyqtSignal(bool)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.port_settings = {
            'baudrate': 921600,
            'bytesize': 8,
            'parity': 'N',
            'stopbits': 1,
            'timeout': 0.01
        }
        self.status = 0
        self.value = None
        self.zero_value = None
        self.ser = serial.Serial(port, **self.port_settings)
        self.ser.flushInput()

    def run(self):
        # print(self.status, int(QThread.currentThreadId()))
        while True:
            if self.status == 0: # 正常读取
                self.value = self.read()
                self.comm_secure.emit(self.value > -10000)
                self.communicate.emit(self.value)
            elif self.status == 1: # 归零信号
                self.reset_zero()
                self.set_status(0)
            else:
                self.close_ser()
                break

    def read_raw(self):
        self.ser.flushInput()
        cur_time = time.time()
        while True:
            if self.ser.in_waiting >= 10:
                data = self.ser.read(self.ser.in_waiting)
                self.ser.flushInput()
                return int(data[-10:-4].decode(), 16) # buffer里数据可能会积累，所以一定是从后往前取位，
                # 数据格式我忘了，可以print出来看看，print(data)
            if time.time() - cur_time > self.timeout:
                raise TimeoutError('Error: sensor failed to read value in %fs' % self.timeout)

    def read(self):
        return (self.read_raw()-self.zero_value)*self.dSensitivity

    def reset_zero(self):
        time.sleep(0.5)
        self.zero_value = self.read_raw()

    def test(self):
        # 这个test本身没有问题，但设备有时候抽风返回超时，多试几次或者重启设备就行
        
        self.reset_zero()
        for i in range(20):
            self.value = self.read()

    def set_status(self, status):
        self.status = status

    def reset(self):
        self.set_status(1)
        time.sleep(0.5)

    def close(self):
        self.set_status(-1)
        time.sleep(0.5)

    def close_ser(self):
        try:
            self.ser.close()
        except:
            pass

    def __del__(self):
        self.close()
        self.close_ser()

    def resetting(self):
        self.reset_zero()
        self.set_status(0)