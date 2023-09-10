import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import time
import sys
import logging

def calc_crc16(string):
    data = bytearray.fromhex(string)
    # logging.info(type(data))
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if ((crc & 1) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return ((crc & 0xff) << 8) + (crc >> 8)

#读取电压
def read_voltage(ser):
    send_data = bytes.fromhex("01 04 00 00 00 02 71 CB")
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        str_return_data = str(return_data.hex())
        feedback_data = int(str_return_data[-12:-8], 16)
        print(feedback_data)


def open_power(ser):
    send_data = bytes.fromhex("01 05 00 00 FF 00 8C 3A")
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        # str_return_data = str(return_data.hex())
        #print(return_data, send_data)
        assert return_data == send_data


def close_power(ser):
    set_voltage(ser, 0) 
    send_data = bytes.fromhex("01 05 00 00 00 00 CD CA")
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        # str_return_data = str(return_data.hex())
        #print(str_return_data)
        assert return_data == send_data

#设置控制器
def set_controller(ser, controller):
    if controller == "out":
        send_data = bytes.fromhex("01 05 00 64 FF 00 CD E5")
    elif controller == "in":
        commond = "01 05 00 64 00 00"
        crc = calc_crc16(commond)
        crc = str("0x%04x" % crc)[2:]
        send_data = bytes.fromhex(commond+crc)
    else:
        print("no implement")
        return
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        assert return_data == send_data

# 设置电压
def set_voltage(ser, voltage):
    hex_v = str("0x%04x" % voltage)[2:]
    # hex_v =
    # print(hex_v)
    commond = "01 06 00 00" + hex_v
    crc = calc_crc16(commond)
    crc = str("0x%04x" % crc)[2:]
    send_data = bytes.fromhex(commond+crc)
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        str_return_data = str(return_data.hex())
        #print(str_return_data)
        assert return_data == send_data

#设置电流
def set_current(ser, current):
    hex_v = str("0x%04x" % current)[2:]
    # hex_v =
    # print(hex_v)
    commond = "01 06 00 0A" + hex_v
    crc = calc_crc16(commond)
    crc = str("0x%04x" % crc)[2:]
    send_data = bytes.fromhex(commond+crc)
    ser.write(send_data)
    time.sleep(1)
    len_return_data = ser.inWaiting()  # 获取缓冲数据（接收数据）长度
    if len_return_data:
        return_data = ser.read(len_return_data)  # 读取缓冲数据
        str_return_data = str(return_data.hex())
        #print(str_return_data)
        assert return_data == send_data

if __name__ == "__main__":
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
    # while True:
    #     commond = input("please input commond\n")
    #     assert commond in ["open", "set_c", "set_v", "read", "close", "controller"]
    #     if commond == "open":
    #         open_power(ser)
    #     elif commond == "controller":
    #         controller = input("please input controller\n")
    #         set_controller(ser, controller)
    #     elif commond == "set_v":
    #         vol = input("please input voltage\n")
    #         set_voltage(ser, int(vol))
    #     elif commond == "set_c":
    #         cur = input("please input current\n")
    #         set_current(ser, int(cur))
    #     elif commond == "read":
    #         read_voltage(ser)
    #     elif commond == "close":
    #         close_power(ser)
