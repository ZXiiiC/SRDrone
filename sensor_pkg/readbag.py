import cv2
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


img_bridge = CvBridge()

img_path = "/home/zy/imgdata"



idx=0
count = 0
def img_cb(msg):
	color_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    cv2.imwrite(img_path + str(idx) + ".jpg",color_img)
    idx += 1
    count += 1



if __name__ == "__main__":
    rospy.init_node('save_image')
    rgb_topic = "/rflysim/sensor1/img_rgb"
    
    img_sub = rospy.Subscriber(rgb_topic, Image, img_cb)
    
    rospy.spin()
