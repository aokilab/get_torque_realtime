#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension

def set_serial_attributes(ser):
    ser.baudrate = 921600
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.1  # タイムアウト（秒）

def request_data(ser):
    ser.write(b'R')
    response = ser.read(27)
    return response

def decode_data(response):
    # バイトオーダーはリトルエンディアン、符号あり32ビット整数
    data = struct.unpack("<bhhhhhhhhhhhhbb", response)
    record_number = data[0]
    Fx_hex = data[1:2]
    Fy_hex = data[3:4]
    Fz_hex = data[5:6]
    Mx_hex = data[7:8]
    My_hex = data[9:10]
    Mz_hex = data[11:12]

    # 16進数を10進数に変換
    Fx = int(Fx_hex[0])
    Fy = int(Fy_hex[0])
    Fz = int(Fz_hex[0])
    Mx = int(Mx_hex[0])
    My = int(My_hex[0])
    Mz = int(Mz_hex[0])

    return Fx, Fy, Fz, Mx, My, Mz


def main():
    com_port = "/dev/ttyUSB0"
    ser = serial.Serial(com_port)
    set_serial_attributes(ser)

    rospy.init_node('sensor_data_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_data', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            response = request_data(ser)
            Fx, Fy, Fz, Mx, My, Mz = decode_data(response)

            rospy.loginfo("Fx: {}, Fy: {}, Fz: {}, Mx: {}, My: {}, Mz: {}".format(Fx, Fy, Fz, Mx, My, Mz))

            data_array = Int32MultiArray()
            data_array.layout.dim.append(MultiArrayDimension())
            data_array.layout.dim[0].label = "sensor_data"
            data_array.layout.dim[0].size = 6
            data_array.layout.dim[0].stride = 1
            data_array.data = [Fx, Fy, Fz, Mx, My, Mz]
            pub.publish(data_array)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("プログラムが中断されました。")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
