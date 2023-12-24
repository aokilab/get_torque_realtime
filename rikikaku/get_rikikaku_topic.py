#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension



def set_serial_attributes(ser):
    ser.baudrate = 921600  # ボーレート
    ser.bytesize = serial.EIGHTBITS  # データビット数
    ser.parity = serial.PARITY_NONE  # パリティビット
    ser.stopbits = serial.STOPBITS_ONE  # ストップビット
    ser.timeout = 1  # タイムアウト（秒）

def request_data(ser):
    # データリクエストを行い、データを取得する関数
    ser.write(b'R')  # リクエストを送信

    # センサからのデータを読み込む
    response = ser.read(27)  # レコード番号(1バイト) + Fx～Mz(各4バイト) + 改行コード(2バイト) の合計27バイトを受信
    return response

def decode_data(response):
    # 受信したデータをデコードする関数
    record_number, Fx, Fy, Fz, Mx, My, Mz = struct.unpack("<Biiiiii", response[:25])
    return record_number, Fx, Fy, Fz, Mx, My, Mz

def main():
    # シリアルポートの設定
    com_port = "/dev/ttyUSB0"  # 接続されているCOMポートに合わせて変更
    ser = serial.Serial(com_port)
    set_serial_attributes(ser)

    # ROSノードの初期化
    rospy.init_node('sensor_data_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_data', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # サンプリング周期に合わせたROSトピックの発行レート

    try:
        # データリクエストと取得のメインループ
        while not rospy.is_shutdown():
            response = request_data(ser)
            record_number, Fx, Fy, Fz, Mx, My, Mz = decode_data(response)

            # データの処理や保存などをここで行う
            rospy.loginfo("Record Number: {}, Fx: {}, Fy: {}, Fz: {}, Mx: {}, My: {}, Mz: {}".format(record_number, Fx, Fy, Fz, Mx, My, Mz))

            # ROSトピックにデータをパブリッシュ
            data_array = Int32MultiArray()
            data_array.layout.dim.append(MultiArrayDimension())
            data_array.layout.dim[0].label = "sensor_data"
            data_array.layout.dim[0].size = 7  # データの要素数
            data_array.layout.dim[0].stride = 1
            data_array.data = [record_number, Fx, Fy, Fz, Mx, My, Mz]
            pub.publish(data_array)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("プログラムが中断されました。")

    finally:
        # 終了時にシリアルポートを閉じる
        ser.close()

if __name__ == "__main__":
    main()