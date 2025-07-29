#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import sys
import cv2
import numpy as np
import rospy
import time
import threading
from rospy import Time
from sensor_msgs.msg import Image
from common_msgs.msg import Objects
from common_msgs.msg import Obj
from cv_bridge import CvBridge
from ObjectDetect import Yolo_Detect
from functools import partial


script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
sys.path.append(script_dir)

# -----------------------------参数--------------------------------
# yolo模型路径
Model_path = script_dir + '/Model/det.pt'

class Depth_Estimate:
    def __init__(self,img1_topic,img2_topic):
        self.yolo_detector = Yolo_Detect(Model_path)
        self.img_bridge = CvBridge()
        #/home/nvidia/catkin_ws/src/Challege_ROS/object_det/scripts/test_node.py

        self.color_done = False
        # 是否显示中间结果图片
        self.show_img = True
        # self.task = None
        self.color_img1 = None
        self.color_img2 = None
        self.header = None

        self.color_lock = threading.Lock()

        self.img1_sub = rospy.Subscriber(img1_topic, Image, self.img_cb,1)
        self.img2_sub = rospy.Subscriber(img2_topic, Image, self.img_cb,2)
        # self.ret_pub = rospy.Publisher("/objects",Objects,queue_size=10)
        self.ret_pub = rospy.Publisher("/obj",Obj,queue_size=10)

    def img_cb(self, msg,idx):
        self.color_lock.acquire()
        self.header = msg.header
        #self.color_img = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.color_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
        if not self.color_done:
            self.color_done = True
        self.color_lock.release()

        
    def run(self):
        # 图片未更新，则返回False
        self.color_lock.acquire()
        # yolo识别圆环
        if( not self.color_done):
            self.color_lock.release()
            return
        img_yolo, det, dt = self.yolo_detector(self.color_img)
        if len(det) != 0:
            # 根据置信度和面积取最可靠目标
            scores = np.sqrt((det[:, 2] - det[:, 0]) * (det[:, 3] - det[:, 1])) * (det[:, 4]**2)
            frame_index = np.argmax(scores)
            xy = det[frame_index, :4].astype(int)
            # img_color_c = self.color_img[xy[1]:xy[3], xy[0]:xy[2], :]
            obj = Obj()
            obj.class_name="frame"
            obj.left_top_x = xy[0]
            obj.left_top_y = xy[1]
            obj.right_bottom_x = xy[2]
            obj.right_bottom_y = xy[3]
            obj.score = det[frame_index,4]
            # obj.class_id = det[frame_index,5]

            self.ret_pub.publish(obj)

            # objs = Objects()
            # objs.header = self.header
            # objs.objects.append(obj)
            
            # self.ret_pub.publish(objs)
            
        if self.show_img:
            if len(det) != 0:
                # 把最后选中的框用黑色框画出来
                cv2.rectangle(img_yolo, tuple(xy[:2]), tuple(xy[2:4]), (0,0,0), thickness=3, lineType=cv2.LINE_AA)
            cv2.imshow('img_yolo', img_yolo)

        self.color_done = False
        self.color_lock.release()
        


if __name__ == "__main__":
    rospy.init_node('frame_det_node')
    rgb_topic = "/rflysim/sensor1/img_rgb"
    img_proc = Depth_Estimate(rgb_topic = rgb_topic)
    while True:
        img_proc.run()
        if cv2.waitKey(10) == ord('q'):
            break
        time.sleep(1)