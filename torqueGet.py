#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import numpy as np

# グローバル変数（トルクデータを格納する行列）
torque_matrix = np.zeros((1, 7))

def callback(data):
    global torque_matrix
    # 最初の7つのトルクデータを行列に格納
    torque_matrix = np.array([data.effort[:7]])

def main():
    rospy.init_node("joint_states_listener", anonymous=True)
    rospy.Subscriber("/torobo/joint_states", JointState, callback)

    # ここでトピックが更新されるたびにコールバック関数が呼ばれる
    # トルクデータは1行7列の行列に格納される

    while not rospy.is_shutdown():
        # トルクデータを表示
        print("Torque Matrix:")
        print(torque_matrix)
        rospy.sleep(0.01)  # ループの頻度を調整するために少し待つ

if __name__ == "__main__":
    main()
