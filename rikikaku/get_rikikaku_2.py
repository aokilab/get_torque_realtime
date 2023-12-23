#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import termios
import time
import signal

def set_com_attr(fdc):
    term = termios.tcgetattr(fdc)
    termios.tcflush(fdc, termios.TCIFLUSH)

    term[2] = termios.IGNPAR
    term[6][termios.VTIME] = 0
    term[6][termios.VMIN] = 0

    term[0] = termios.B921600 | termios.CS8 | termios.CLOCAL | termios.CREAD
    term[3] = 0

    termios.tcsetattr(fdc, termios.TCSANOW, term)

def read_sensor_data(fdc):
    os.write(fdc, b"R")  # 単データリクエスト（次回分）

    str_data = os.read(fdc, 27)
    if len(str_data) < 27:
        return None

    tick, data0, data1, data2, data3, data4, data5 = struct.unpack("<1H4H", str_data)

    return tick, data0, data1, data2, data3, data4, data5

def signal_handler(signal, frame):
    global fdc
    if fdc >= 0:
        os.close(fdc)
    print("\nCOM port closed.")
    exit(0)

def main():
    global fdc
    fdc = -1

    try:
        signal.signal(signal.SIGINT, signal_handler)

        # COMポートをオープン
        com_no = int(input("Enter COM port > "))
        print("Open /dev/ttyUSB%d" % com_no)

        devname = "/dev/ttyUSB%d" % com_no
        fdc = os.open(devname, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if fdc < 0:
            print("Failed to open COM port.")
            return

        # COMポートのボーレート等を設定
        set_com_attr(fdc)

        print("=== Real-time sensor data ===")
        
        while True:
            sensor_data = read_sensor_data(fdc)
            if sensor_data:
                print("Tick: {}, Data: {}".format(sensor_data[0], sensor_data[1:]))

            # 1秒ごとにデータを取得し出力
            time.sleep(1)

    except Exception as e:
        print("Error:", str(e))
        if fdc >= 0:
            os.close(fdc)

if __name__ == "__main__":
    main()
