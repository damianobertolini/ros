from __future__ import print_function

import time

import numpy as np
import rospy
from sensor_msgs.msg import Image
import struct
from PIL import Image as pilimage
import cv2
import torch

# import ros_numpy
import numpy as np
import rospy as ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

# for locks
import threading

import time

import cv2
import torch
import numpy as np
import math

from imutils import contours as imcontours

# custom created messages
from my_vision_messages.msg import Pointxyz
from my_vision_messages.msg import Block
from my_vision_messages.msg import BlockList


# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image():

    to_delete = []

    block_list = BlockList()
    block_list_confs = []


    real_coord_x = 0.4444
    real_coord_y = 0.6202

    point = Pointxyz(real_coord_x, real_coord_y, 0.9)
    block = Block()

    conf = 100

    block.class_number = str(3)
    block.point = point
    block.rot_angle = 0.44    *360/(math.pi*2)
    block.confidence = conf
    block.world_point = point

    block.top_btm_l_r = ["", ""]
    block.up_dw_stand_lean = ""

    block_list_confs.append(conf)

    sorted_conf = sorted(enumerate(block_list_confs), key=lambda conf_num: conf_num[1])
    new_block_index = [i[0] for i in sorted_conf if i[1] == conf]

    block_list.blocks.insert(new_block_index[0], block)


        # when finished publish result but before sort BlockList by confidence
    global res_pub
    print("publishing")
    print(block_list.blocks)
    res_pub.publish(block_list.blocks)


#print("finito")

def image_depth_processing():

    process_image()


# TODO RENDERLA NON DIPENDENTE DALLA CALLBACK E METTERE VARIABILI 640 e 360



def processing_callback(event=None):

    image_depth_processing()


res_pub = None


def listener():
    rospy.init_node('feed_vision_node2', anonymous=True)

    global res_pub
    res_pub = rospy.Publisher('vision_results', BlockList, queue_size=1)

    # Create a ROS Timer for publishing data
    rospy.Timer(rospy.Duration(2), processing_callback)


    rospy.spin()


if __name__ == '__main__':
    listener()