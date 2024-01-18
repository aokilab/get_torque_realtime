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

def calculate_calibration_data(ser, num_samples=100):
    calibration_data = [0, 0, 0, 0, 0, 0]  # Fx, Fy, Fz, Mx, My, Mz
    for _ in range(num_samples):
        ser.write(b'R')
        data = ser.readline()
        record_number = data[0]
        Fx_hex = data[1:5]
        Fy_hex = data[5:9]
        Fz_hex = data[9:13]
        Mx_hex = data[13:17]
        My_hex = data[17:21]
        Mz_hex = data[21:25]

        # 16進数を10進数に変換
        Fx = int(Fx_hex, 16)
        Fy = int(Fy_hex, 16)
        Fz = int(Fz_hex, 16)
        Mx = int(Mx_hex, 16)
        My = int(My_hex, 16)
        Mz = int(Mz_hex, 16)

        # キャリブレーションデータを累積
        calibration_data[0] += Fx
        calibration_data[1] += Fy
        calibration_data[2] += Fz
        calibration_data[3] += Mx
        calibration_data[4] += My
        calibration_data[5] += Mz

    # 平均値を計算
    calibration_data = [value / num_samples for value in calibration_data]
    return calibration_data

def decode_data(ser, calibration_data):
    ser.write(b'R')
    data = ser.readline()
    record_number = data[0]
    Fx_hex = data[1:5]
    Fy_hex = data[5:9]
    Fz_hex = data[9:13]
    Mx_hex = data[13:17]
    My_hex = data[17:21]
    Mz_hex = data[21:25]

    # 16進数を10進数に変換
    Fx = int(Fx_hex, 16) - calibration_data[0]
    Fy = int(Fy_hex, 16) - calibration_data[1]
    Fz = int(Fz_hex, 16) - calibration_data[2]
    Mx = int(Mx_hex, 16) - calibration_data[3]
    My = int(My_hex, 16) - calibration_data[4]
    Mz = int(Mz_hex, 16) - calibration_data[5]

    return Fx, Fy, Fz, Mx, My, Mz

def main():
    com_port = "/dev/ttyUSB1"
    ser = serial.Serial(com_port)
    set_serial_attributes(ser)

    rospy.init_node('sensor_data_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_data', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(100)

    # 最初の100回分のデータを用いてキャリブレーションデータを計算
    calibration_data = calculate_calibration_data(ser, num_samples=100)

    try:
        while not rospy.is_shutdown():
            Fx, Fy, Fz, Mx, My, Mz = decode_data(ser, calibration_data)

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
