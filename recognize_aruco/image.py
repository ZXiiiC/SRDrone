#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from common_msgs.msg import Aruco
import rospy
import numpy as np




# 获取 ArUco 字典
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# 创建 ArUco 检测器
parameters = aruco.DetectorParameters_create()


msg_pub = None
def ImgCB(msg):
    global parameters
    global aruco_dict
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    # 检测标记
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    # 如果检测到了标记
    
    marker_length = 0.05 
    # 相机内参和畸变系数
    camera_matrix = np.array([[1, 0, 320], [0, 1, 240], [0, 0, 1]])
    dist_coeffs = np.array([0, 0, 0, 0, 0])

    if ids is not None:
        # 绘制检测到的标记
        # if len(ids) > 1:
        #     print("detect multi aruco")
        #     return
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        image = aruco.drawDetectedMarkers(image, corners, ids)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 0,255)
        font_thickness = 2
        aruco_msg = Aruco()
        aruco_msg.header = msg.header
        aruco_msg.id = int(ids[0])
        aruco_msg.cnt_x = int(corners[0][0][0][0])
        aruco_msg.cnt_y = int(corners[0][0][0][1])
        # 获取二维码在相机坐标系下的位置
        tvec = tvecs[0][0]
        rvec = rvecs[0][0]
        # 计算二维码在世界坐标系下的位置
        aruco_msg.position_x = tvec[0]  # 位置 x
        aruco_msg.position_y = tvec[1]  # 位置 y
        aruco_msg.position_z = tvec[2]  # 位置 z     
        aruco_msg.r_x = rvec[0]  # 位置 x
        aruco_msg.r_y = rvec[1]  # 位置 y
        aruco_msg.r_z = rvec[2]  # 位置 z         

        msg_pub.publish(aruco_msg)
        # 输出检测到的标记的 ID
        cv2.putText(image, str(ids[0]), tuple([50,50]), font, font_scale, font_color, font_thickness)
        # 显示图像
        cv2.imshow('ARuco Detection', image)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
    else:
        print("No ARuco markers detected.")

if __name__ == "__main__":
    rospy.init_node("aruco")
    msg_pub = rospy.Publisher("/Aruco", Aruco, queue_size = 10)
    img_sub = rospy.Subscriber("/rflysim/sensor2/img_rgb",Image,ImgCB)

    rospy.spin()
