#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import sys
import cv2
import numpy as np
import rospy
import time
import threading
from sensor_msgs.msg import Image
from common_msgs.msg import Objects, Obj
from cv_bridge import CvBridge
from ObjectDetect import Yolo_Detect
from functools import partial

script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
sys.path.append(script_dir)

Model_path = script_dir + '/Model/best.pt'

class Depth_Estimate:
    def __init__(self, img_topic):
        self.yolo_detector = Yolo_Detect(Model_path)
        self.img_bridge = CvBridge()

        self.color_done = False
        self.color_img = None
        self.header = None
        self.objs = Objects()
        self.color_lock = threading.Lock()

        self.img_sub = rospy.Subscriber(img_topic, Image, self.img_cb)
        self.ret_pub = rospy.Publisher("/objects", Objects, queue_size=10)

    def img_cb(self, msg):
        self.color_lock.acquire()
        self.header = msg.header
        # 转为numpy图像，保持原格式和尺寸
        self.color_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.color_done = True
        self.color_lock.release()

    def run(self):
        self.color_lock.acquire()
        if not self.color_done:
            self.color_lock.release()
            return
        img_yolo, det, dt = self.yolo_detector(self.color_img)

        # 过滤置信度 >= 0.7 的检测框
        filtered_det = [d for d in det if d[4] >= 0.7]

        if len(filtered_det) != 0:
            filtered_det = np.array(filtered_det)
            scores = np.sqrt((filtered_det[:, 2] - filtered_det[:, 0]) * (filtered_det[:, 3] - filtered_det[:, 1])) * (filtered_det[:, 4] ** 2)
            frame_index = np.argmax(scores)
            xy = filtered_det[frame_index, :4].astype(int)

            obj = Obj()
            obj.class_name = "board"
            obj.left_top_x = xy[0]
            obj.left_top_y = xy[1]
            obj.right_bottom_x = xy[2]
            obj.right_bottom_y = xy[3]
            obj.score = filtered_det[frame_index, 4]

            self.objs.header = self.header
            self.objs.objects.append(obj)

            self.ret_pub.publish(self.objs)
            self.objs.objects = []

            # 这里用原始图像拷贝自己画框，避免img_yolo中所有框的影响
            img_show = self.color_img.copy()
            cv2.rectangle(img_show, tuple(xy[:2]), tuple(xy[2:4]), (0, 0, 0), thickness=3, lineType=cv2.LINE_AA)
        else:
            # 如果无检测，显示原图
            img_show = self.color_img.copy()

        if self.color_done:
            cv2.imshow('det', img_show)
            cv2.waitKey(1)

        self.color_done = False
        self.color_lock.release()


if __name__ == "__main__":
    rospy.init_node('det_node')
    topic1 = "/camera/color/image_raw"
    img_proc = Depth_Estimate(img_topic=topic1)

    while not rospy.is_shutdown():
        img_proc.run()
        cv2.waitKey(1)
        time.sleep(1)  # 1Hz频率执行

