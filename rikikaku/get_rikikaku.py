#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import time

def set_serial_attributes(ser):
    # シリアル通信の設定を行う関数
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

    try:
        # データリクエストと取得のメインループ
        while True:
            response = request_data(ser)
            record_number, Fx, Fy, Fz, Mx, My, Mz = decode_data(response)

            # データの処理や保存などをここで行う
            print(f"Record Number: {record_number}, Fx: {Fx}, Fy: {Fy}, Fz: {Fz}, Mx: {Mx}, My: {My}, Mz: {Mz}")

            # 任意の処理後、サンプリング周期に合わせて待つ
            time.sleep(0.1)  # 0.1秒待つ (10 Hz)

    except KeyboardInterrupt:
        print("プログラムが中断されました。")
if __name__ == "__main__":
    main()
