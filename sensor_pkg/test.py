import cv2
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


img_bridge = CvBridge()

img_path = "/home/zy/imgdata/"


idx = 0
count = 0

def img_cb(msg:Image):
    global count
    global idx
    if(count % 2 == 0):
        color_img = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height, msg.width,-1)
        cv2.imwrite(img_path + str(idx) + ".jpg",color_img)
        idx = idx +1
    count += 1
    
if __name__ == "__main__":
    rospy.init_node("test_node")
    rgb_topic = "/color"
    img_sub = rospy.Subscriber(rgb_topic,Image,img_cb)
    
    rospy.spin()