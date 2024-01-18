#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import JointState
from numpy import *

# グローバル変数（トルクデータとセンサデータを格納する行列および変数）
sensor_data = (0, 0, 0, 0, 0, 0)  # Fx, Fy, Fz, Mx, My, Mz
torque_end  = (0, 0, 0, 0, 0, 0)

def torque_end_callback(data):
    global torque_end
    # トルクデータを受け取る
    torque_end = data.data

def sensor_data_callback(data):
    global sensor_data
    # センサーデータを受け取る
    sensor_data = data.data

def main():
    rospy.init_node("sync_data_node", anonymous=True)

    rospy.Subscriber("/torobo/torque_end", Float64MultiArray, torque_end_callback)
    rospy.Subscriber('sensor_data', Int32MultiArray, sensor_data_callback)

    pub = rospy.Publisher('/sync_data', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100)  # 250Hz

    while not rospy.is_shutdown():

        # 2つのデータを結合して新しいデータを生成
        sync_data = matrix([sensor_data + torque_end])

        pub.publish(Float64MultiArray(data=sync_data.flatten().tolist()[0]))

        print(sync_data)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
