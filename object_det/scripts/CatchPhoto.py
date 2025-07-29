#!C:\pythonCode
# -*- coding: utf-8 -*-
# @Time : 2023/9/1 10:11
# @Project : Yolo5_Detect
# @File : CatchPhoto.py
# @Author : YDYH
# @Software: PyCharm

import rospy
from sensor_msgs.msg import Image

import os
import cv2
import numpy as np

img = None


def image_callback(image_data):
    global img
    tmp = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
    img = cv2.cvtColor(tmp, cv2.COLOR_RGB2BGR)


if __name__ == '__main__':

    rospy.init_node('image_subscriber')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)  # 订阅无人机相机彩色图像话题
    rate = rospy.Rate(0.1)  # 设置话题发布频率为10Hz

    cnt = 0
    path_img = './Image_collect/'
    while True:
        rate.sleep()
        if img is not None:
            cv2.imshow("img", img)  # Show the processed image
            if not os.path.exists((path_img)):
                os.makedirs(path_img)
            cv2.imwrite(os.path.join(path_img, "{}.jpg".format(cnt)), img)
            print("采集图片数量：", cnt)
            cnt += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
