#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import numpy as np

# グローバル変数（トルクデータと角度データを格納する行列および変数）
torque_matrix = np.zeros((1, 7))
ang1, ang2, ang3, ang4, ang5, ang6, ang7 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def callback(data):
    global torque_matrix, ang1, ang2, ang3, ang4, ang5, ang6, ang7
    # 最初の7つのトルクデータを行列に格納
    torque_matrix = np.array([data.effort[:7]])
    # 各関節の角度データを変数に代入
    ang1, ang2, ang3, ang4, ang5, ang6, ang7 = data.position[:7]

def main():
    rospy.init_node("joint_states_listener", anonymous=True)
    rospy.Subscriber("/torobo/joint_states", JointState, callback)

    # ここでトピックが更新されるたびにコールバック関数が呼ばれる
    # トルクデータと各関節の角度データは異なる変数に格納される

    while not rospy.is_shutdown():
        # トルクデータと各関節の角度データを表示
        print("Torque Matrix:")
        print(torque_matrix)
        print("Angle Joint 1:", ang1)
        print("Angle Joint 2:", ang2)
        print("Angle Joint 3:", ang3)
        print("Angle Joint 4:", ang4)
        print("Angle Joint 5:", ang5)
        print("Angle Joint 6:", ang6)
        print("Angle Joint 7:", ang7)
        rospy.sleep(0.1)  # ループの頻度を調整するために少し待つ

if __name__ == "__main__":
    main()
