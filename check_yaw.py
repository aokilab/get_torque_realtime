#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as *
from sensor_msgs.msg import JointState

angle1 ,angle2 ,angle3 ,angle4 ,angle5 ,angle6 ,angle7 = 0

def callback(data):
    global angle1, angle2, angle3, angle4, angle5, angle6, angle7, ang1, ang2, ang3, ang4, ang5, ang6, ang7
    # 各関節の角度データを変数に代入
    angle1, angle2, angle3, angle4, angle5, angle6, angle7 = data.position[:7]


while True:
    rospy.init_node("joint_states_listener", anonymous=True)
    rospy.Subscriber("/torobo/joint_states", JointState, callback)
    rate = rospy.Rate(10)  # 10Hz

    # ang1~ang7を変数として定義
    ang1 = radians(angle1)
    ang2 = radians(angle2)
    ang3 = radians(angle3)
    ang4 = radians(angle4)
    ang5 = radians(angle5)
    ang6 = radians(angle6)
    ang7 = radians(angle7)

    T1 = np.matrix([
        [cos(ang1), -sin(ang1), 0, 0],
        [sin(ang1), cos(ang1), 0, 0],
        [0, 0, 1, 215],
        [0, 0, 0, 1]
    ])

    T2 = np.matrix([
        [cos(ang2), -sin(ang2), 0, 0],
        [0, 0, 1, 0],
        [-sin(ang2), -cos(ang2), 0, 0],
        [0, 0, 0, 1]
    ])

    T3 = np.matrix([
        [cos(ang3), -sin(ang3), 0, 0],
        [0, 0, -1, -300],
        [sin(ang3), cos(ang3), 0, 0],
        [0, 0, 0, 1]
    ])

    T4 = np.matrix([
        [cos(ang4), -sin(ang4), 0, 0],
        [0, 0, 1, 0],
        [-sin(ang4), -cos(ang4), 0, 0],
        [0, 0, 0, 1]
    ])

    T5 = np.matrix([
        [cos(ang5), -sin(ang5), 0, 0],
        [0, 0, -1, -300],
        [sin(ang5), cos(ang5), 0, 0],
        [0, 0, 0, 1]
    ])

    T6 = np.matrix([
        [cos(ang6), -sin(ang6), 0, 0],
        [0, 0, 1, 0],
        [-sin(ang6), -cos(ang6), 0, 0],
        [0, 0, 0, 1]
    ])

    T7 = np.matrix([
        [cos(ang7), -sin(ang7), 0, 0],
        [0, 0, -1, -170],
        [sin(ang7), cos(ang7), 0, 0],
        [0, 0, 0, 1]
    ])

    T8 = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 150],
        [0, 0, 0, 1]
    ])

    T_all = T1 * T2 * T3 * T4 * T5 * T6 * T7 * T8

    #print(T_all)

    # 各要素を変数に代入
    R11, R12, R13, R14 = T_all[0, :4].tolist()[0]
    R21, R22, R23, R24 = T_all[1, :4].tolist()[0]
    R31, R32, R33, R34 = T_all[2, :4].tolist()[0]
    R41, R42, R43, R44 = T_all[3, :4].tolist()[0]


    # Roll, Pitch, Yawを計算
    R = atan2(R23, R13)
    P = atan2(sqrt(R13**2 + R23**2), R33)
    Y = atan2(R32, -R31)

    x = R13**2 + R23**2

    if x == 0:
        print("zero")
    else:
        print("not zero")