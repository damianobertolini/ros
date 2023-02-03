# !/usr/bin/env python

from __future__ import print_function

# import ros_numpy
import time

import numpy as np
import rospy as ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)


# Resources

# native reading best explanation
# https: // answers.ros.org / question / 219876 / using - sensor_msgspointcloud2 - natively /
# https://medium.com/@jeehoahn/some-ideas-on-pointcloud2-data-type-1d1ae940ef9b
# https://answers.ros.org/question/373094/understanding-pointcloud2-data/
# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2


def receive_pointcloud(msg):
    # get one (or more) points from the pointcloud (unfortunately you need an iterator) These X,Y,Z values are in the
    # zed2_left_camera_optical_frame (X is seen as going to left (looking the camera) Y is top to bottom and Z
    # pointing out the camera). You can select the pixel in the image frame with the u,v variable, u = 0 ,
    # v = 0 is the top right corner, u = 640 , v = 360 is the middle of the image plane which corresponds to the
    # origin of the zed2_left_camera_optical_frame

    to_crop_depth = []
    eps = 10
    for y in range(int(637), int(704) + 1):
        for x in range(int(340), int(377) + 1):
            to_crop_depth.append([y, x])
            # print(y,x)

    points_list = []

    min = 2
    max = 0
    prev = 0
    for u, v in to_crop_depth:
        for data in point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(u, v)]):
            print(data, u, v)

            if data[2] < min:
                min = data[2]
            if data[2] > max:
                max = data[2]

            if prev == 0:
                prev = data[2]
            else:
                if data[2] - prev > 0.01:
                    prev = data[2]
                    continue

            points_list.append([data[0], data[1], data[2], 255])  # faking rgb values

    print(min, max)
    time.sleep(8)


if __name__ == '__main__':
    ros.init_node('custom_joint_pub_node', anonymous=True)
    sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2,
                                    callback=receive_pointcloud, queue_size=1)

    loop_rate = ros.Rate(1.)
    while True:
        loop_rate.sleep()
        pass
