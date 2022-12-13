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



    with open("b.txt", "w") as fi:
        image_arr = []
        while i < tot_data / (1920*4):
            riga = []
            j = 0
            while j < tot_data / (1080*4):
                riga.append(struct.unpack_from('4B', data.data, int(j*4 + i * tot_data / 1080)))

                a = struct.unpack_from('4B', data.data, int(j*4 + i * tot_data / 1080))
                #print(a)
                #exit(1)
                fi.write(str(a))

                j += 1
                #print(a)
                #time.sleep(30)
                #print(j)
                riga = [a[0], a[1], a[3]]
            print("done")
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
    rospy.Subscriber("/ee_camera/camera/depth/image_raw" , Image, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()