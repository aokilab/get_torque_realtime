#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

def main():
    # シリアルポートの設定（ポート名は環境に合わせて変更してください）
    ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1)

    try:
        while True:
            # シリアルポートからデータを読み込む
            ser.write(b'R')
            data = ser.readline()

            # データがあればそのままターミナルに表示
            if data:
                print(data.strip())

    except KeyboardInterrupt:
        print("プログラムを終了します.")
        ser.close()  # シリアルポートを閉じる

if __name__ == "__main__":
    main()
