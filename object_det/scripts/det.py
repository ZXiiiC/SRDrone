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
from common_msgs.msg import MissionState
from cv_bridge import CvBridge
from ObjectDetect import Yolo_Detect
from functools import partial


script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
sys.path.append(script_dir)

# -----------------------------参数--------------------------------
# yolo模型路径
Model_path = script_dir + '/Model/det.pt'

# ----------------------------------------------------------------

class Depth_Estimate:
    def __init__(self,img1_topic,img2_topic):
        self.yolo_detector = Yolo_Detect(Model_path)
        self.img_bridge = CvBridge()

        self.color_done = False
        # 是否显示中间结果图片
        self.show_img = True
        self.color_img1 = None
        self.color_img2 = None
        self.color_img = None
        self.header = None
        self.task = 0
        self.objs = Objects()

        self.color_lock = threading.Lock()
        self.img1_sub = rospy.Subscriber(img1_topic, Image, partial(self.img_cb, idx=1))
        self.img2_sub = rospy.Subscriber(img2_topic, Image,  partial(self.img_cb, idx=2))
        self.task_sub = rospy.Subscriber("/task_state",MissionState,self.task_cb)
        self.ret_pub = rospy.Publisher("/objects",Objects,queue_size=10)

    def task_cb(self,msg):
        self.task = msg.task

    def img_cb(self, msg,idx):
        #self.color_img = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding) 
        if(self.task < 5 and idx == 2):
            return
        if(self.task >=5 and idx == 1):
            return
        
        #保证不用的数据不占用资源
        self.color_lock.acquire()
        self.objs.sensor_id = idx
        self.header = msg.header
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
            if(self.task >= 5):
                obj.class_name="land"
            obj.left_top_x = xy[0]
            obj.left_top_y = xy[1]
            obj.right_bottom_x = xy[2]
            obj.right_bottom_y = xy[3]
            obj.score = det[frame_index,4]
            # objs = Objects()
            self.objs.header = self.header
            self.objs.objects.append(obj)
            
            self.ret_pub.publish(self.objs)
            self.objs.objects=[]
            
        if self.show_img:
            if len(det) != 0:
                # 把最后选中的框用黑色框画出来
                cv2.rectangle(img_yolo, tuple(xy[:2]), tuple(xy[2:4]), (0,0,0), thickness=3, lineType=cv2.LINE_AA)
            cv2.imshow('det', img_yolo)

        self.color_done = False
        self.color_lock.release()
        


if __name__ == "__main__":
    rospy.init_node('det_node')
    # topic2 = "/rflysim/sensor2/img_rgb" #下视相机的图像
    topic2="/usb_cam/image_raw"
    # topic1 = "/rflysim/sensor1/img_rgb" #前视相机的图像
    topic1="/camera/color/image_raw"
    img_proc = Depth_Estimate(img1_topic = topic1,img2_topic = topic2)
    while True:
        img_proc.run()
        if cv2.waitKey(10) == ord('q'):
            break
        time.sleep(1)

