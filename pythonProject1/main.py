# !/usr/bin/env python3
import rospy  # client library for ROS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import random as rng
import cv2
import numpy as np
import os
import time
import torch
import rospkg

# change absolute path to your best.pt path
path = "/home/user/trento_lab_home/prova/src/lego_recognizer_yolov5/nodes/usiamo_questi.pt"
model = torch.hub.load('/home/user/trento_lab_home/prova/src/lego_recognizer_yolov5/yolov5', 'custom', path=path, source='local')

def process_image(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        print(orig)
        print("scrivo su topic")
        save_images = 0
        model.conf = 0.6
        results = model(orig)
        pandino = results.pandas().xyxy[0].to_dict(orient="records")

        print(pandino)
        for pand in pandino:
            cs = float(pand['class'])
            x1 = float(pand['xmin'])
            y1 = float(pand['ymin'])
            x2 = float(pand['xmax'])
            y2 = float(pand['ymax'])
            xc = float(((x2 - x1) / 2) + x1)
            yc = float(((y2 - y1) / 2) + y1)

            print(cs, x1, y1, x2, y2, xc, yc)
        print("finito")

    except Exception as err:
        print(err)


def start_node():
    rospy.init_node('detect_topic')
    rospy.loginfo('Aspetto l\'immagine')
    rospy.Subscriber("/z_base_camera/camera/rgb/image_raw", Image, process_image)  # si iscrive al topic del kinectù
    rospy.spin()  # Continua a ciclare, evita la chiusura del nodo


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
