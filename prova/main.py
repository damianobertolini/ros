import time

import numpy as np
import rospy
from sensor_msgs.msg import Image
import struct
from PIL import Image as pilimage


def callback(data):
    print(len(data.data))
    # tot_data = len(data.data) / 3
    tot_data = len(data.data)
    i = 0
    j = 0

    image_arr2 = [[] for l in range(int(tot_data/(640*3)))]
    k = 0
    m = 0
    while m < tot_data / (480*3):
        k = 0
        while k < tot_data / (640*3):
            image_arr2[k].append(struct.unpack_from('3B', data.data, int(k*3 + m * tot_data / 640)))
            k += 1

        m+=1

    print("creating asarray")

    arr = np.asarray(image_arr2, dtype='uint8')
    print("creating pilimage")

    img = pilimage.fromarray(arr)
    print("saving")

    img.save('new.png')
    print("done")


    image_arr = []
    while i < tot_data / (640*3):
        riga = []
        j = 0
        while j < tot_data / (480*3):
            riga.append(struct.unpack_from('3B', data.data, int(j*3 + i * tot_data / 480)))
            j += 1


        image_arr.append(riga)

        print(i)

        i += 1
    print(image_arr)


    print("creating asarray")

    arr = np.asarray(image_arr, dtype='uint8')
    print("creating pilimage")

    img = pilimage.fromarray(arr)
    print("saving")

    img.save('new.png')
    print("done")

    time.sleep(30)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/z_base_camera/camera/rgb/image_raw" , Image, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()