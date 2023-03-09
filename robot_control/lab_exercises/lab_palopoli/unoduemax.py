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
def process_image(image):
    #model = torch.hub.load("../../../yolov5/", 'custom', path="../../../yolov5/best.pt", source='local')
    model = torch.hub.load("../../../yolov5/", 'custom', path="../../../yolov5/pesi yolo/GazebobestNoResizeGray.pt", source='local')

    # resize image for yolo detection (from 1920x1080 to 640x360)
    image_resized = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)

    model.conf = 0.6
    results = model(image_resized)
    result_dict = results.pandas().xyxy[0].to_dict(orient="records")

    to_delete = []

    # eliminare predictions vicine in base a conf maggiore
    for det1 in result_dict:
        x11 = float(det1['xmin'])
        y11 = float(det1['ymin'])
        x21 = float(det1['xmax'])
        y21 = float(det1['ymax'])
        conf1 = float(det1['confidence'])

        for det2 in result_dict:
            x12 = float(det2['xmin'])
            y12 = float(det2['ymin'])
            x22 = float(det2['xmax'])
            y22 = float(det2['ymax'])
            conf2 = float(det2['confidence'])

            if abs(x11 - x12) < 5 and abs(y11 - y12) < 5 and abs(x21 - x22) < 5 and abs(
                    y21 - y22) < 5:  # two very near labels
                if conf1 > conf2:  # take the one with best accuracy
                    to_delete.append(conf2)
                else:
                    to_delete.append(conf1)

    for to_del in to_delete:
        if to_del in result_dict:  # since the above part is not very efficient it will take every to_del twice
            # example: x1.conf > x2.conf, but later in the for loop: x2.conf < x1.conf -> counted twice
            result_dict.remove(to_del)

    block_list = BlockList()
    block_list_confs = []
    block_list_x_pos = []

    print("YOLO result dict:")
    print(result_dict)

    #--------------------------------------
    dictionary = {
        "X1-Y1-Z2": 0,
        "X1-Y2-Z2-CHAMFER":1,
        "X1-Y2-Z2":2,
        "X1-Y3-Z2":3,
        "X1-Y4-Z2":4,
        "X2-Y2-Z2":5,
        "X1-Y2-Z1":6,
        "X1-Y2-Z2-TWINFILLET":7,
        "X1-Y3-Z2-FILLET":8,
        "X1-Y4-Z1":9,
        "X2-Y2-Z2-FILLET":10,
    }

    #--------------------------------------



    for detected in result_dict:
        #cs = float(detected['name'])
        print(detected['name'])
        if detected['name'] in dictionary:
            cs = dictionary[detected['name']]
        else:
            cs = 12
        x1 = float(detected['xmin'])
        y1 = float(detected['ymin'])
        x2 = float(detected['xmax'])
        y2 = float(detected['ymax'])
        conf = float(detected['confidence'])

        # TODO this values might be changed if definition changes and on real robot
        # check if bounding box is inside table, else discard label
        if x1 < 280 or x2 > 514 or y1 < 140 or y2 > 305:
            continue  # not inside table

        # per template matching uso immagine 1280per 720 -> trasformo da 640 360
        x1 = 3 * x1
        x2 = 3 * x2
        y1 = 3 * y1
        y2 = 3 * y2

        # TODO aggiungere ricerca in base a conf maggiore e eliminare predictions vicine

        eps = 10
        crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]

        # new part, read cropped image, find contours (draw rectangle) and draw center point
        cannied_img = canny_img(crop_img, 3, str(int(cs)))

        rect_center, rect_width_height, rect_angle = min_area_rect(cannied_img, str(int(cs)))

        print("rotation of block: ", end="")
        print(rect_angle)
        # per successivo calcolo depth del centro

        print(f"center point coord for image with code {str(int(cs))}: ", end="")
        print(rect_center)

        # ritorno da immagine croppata a immagine yolo
        real_coord_x, real_coord_y = from_cropped_to_real_world_coord(rect_center, x1, y1, eps)

        print("full image coordinates for center point: ", end="")
        print(real_coord_x, real_coord_y)

        # calcolo depth punto centrale

        # print(center, real_coord_x, real_coord_y, image.shape, eps)
        center_depth = calculate_depth(real_coord_x, real_coord_y)

        print("Center point depth z: ", end="")
        print(center_depth[0][2])

        # print circle on image centre
        with_center_image = cv2.circle(image, (real_coord_x, real_coord_y), radius=1, color=(255, 255, 255),
                                       thickness=5)

        # return to original image sizes (find real coordinates of found centre)

        cv2.imwrite("final.jpg", with_center_image)


        print("-----------------------------------")
        point = Pointxyz(real_coord_x, real_coord_y, center_depth[0][2])

        w_R_c = np.matrix([[0., - 0.49948, 0.86632], [-1., 0., 0.], [-0., - 0.86632, - 0.49948]])
        w_c = np.array([-0.9, 0.24, -0.35])
        base_offset = np.array([0.5, 0.35, 1.75])

        real_coord_x = real_coord_x
        real_coord_y = real_coord_y

        points_list = calculate_depth(real_coord_x, real_coord_y)


        print("Data Optical frame: ", points_list)
        pointW = np.dot(w_R_c, points_list[0]) + w_c + base_offset
        print("Data World frame: ", pointW)
        #print("-----------------------------------")
        robo = np.array(
            [np.squeeze(np.asarray(pointW))[0], np.squeeze(np.asarray(pointW))[1], np.squeeze(np.asarray(pointW))[2]])
        #robo[0] = robo[0] - 0.5
        #robo[1] = 0.35 - robo[1]
        print("Data robot coord: ", robo)
        print("-----------------------------------")


        block = Block()
        block.class_number = str(cs)
        block.point = point
        block.rot_angle = rect_angle
        block.confidence = conf
        block.world_point = Pointxyz(robo[0],robo[1],robo[2])

        # this are not needed in this program
        block.top_btm_l_r = ["", ""]
        block.up_dw_stand_lean = ""

        # sort block list based on fifth component (confidence)
        block_list_confs.append(conf)
        block_list_x_pos.append(robo[0])

        sorted_conf = sorted(enumerate(block_list_x_pos), key=lambda conf_num: conf_num[1])
        new_block_index = [i[0] for i in sorted_conf if i[1] == robo[0]]

        block_list.blocks.insert(new_block_index[0], block)



        #-----------------------------------------------------------------------------------





        #-------------------------------------------------------------------------------------


        # when finished publish result but before sort BlockList by confidence
    global res_pub

    res_pub.publish(block_list.blocks)
    exit()


#print("finito")


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova
# quindi min_x min_y ecc del contorno poi trova anche il punto centrale e disegna un cerchio in quel punto salva
# tutto in edge.jpg
def canny_img(img, aperture_size, name):
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cannied_img = cv2.Canny(img_gray, 50, 150, apertureSize=aperture_size, L2gradient=True)

    cv2.imwrite(f'mio_cannied.jpg', cannied_img)

    return cannied_img


def draw_center(coordinates, image):
    xc = int((coordinates[2] - coordinates[0]) / 2 + coordinates[0])
    yc = int((coordinates[3] - coordinates[1]) / 2 + coordinates[1])

    image = cv2.circle(image, (xc, yc), radius=2, color=(255, 255, 255), thickness=10)

    # print(xc, yc, coordinates)
    return image, (xc, yc)


# trova minimo rettangolo che ingloba il contorno trovato con canny e lo salva  in minRect.jpg, utile anche per
# rotazioni sull'asse z
def min_area_rect(img, name):
    # print(img)

    # with open("pix.txt", "w") as f:
    #     for line in img:
    #         f.write(str(line) + "\n")

    ret, thresh = cv2.threshold(img, 200, 256, 0)

    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    # print(contours)

    # sort found contour in order to take the largest one
    (contours, _) = imcontours.sort_contours(contours)  # not strictly necessary
    contours = sorted(contours, key=lambda c: cv2.contourArea(c))

    assert len(contours) >= 1
    cnt = contours[-1]

    min_area_rectangle = cv2.minAreaRect(cnt)
    tmp2 = cv2.boxPoints(min_area_rectangle)

    box = np.int0(tmp2)

    # print(box)

    cv2.drawContours(img, [box], 0, (255, 255, 255), 2)
    # cv2.imwrite(f'{name}_minRect.jpg', img)

    # print(min_area_rectangle)

    min_area_rect_angle = min_area_rectangle[2]

    min_area_rect_width = min_area_rectangle[1][0]
    min_area_rect_height = min_area_rectangle[1][1]

    # print(name, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    if min_area_rect_width < min_area_rect_height:
        min_area_rect_angle = min_area_rect_angle + 90
    # else:
    #    min_area_rect_angle = min_area_rect_angle + 180

    # print(min_area_rect_angle)

    return min_area_rectangle[0], min_area_rectangle[1], min_area_rect_angle


def from_cropped_to_real_world_coord(point, x1, y1, eps):
    real_x = int(int(x1) - eps + point[0])
    real_y = int(int(y1) - eps + point[1])

    return real_x, real_y


# FOR DEPTH CALCULATION BASED ON A POINT IN CAMERA VIEW
def calculate_depth(x, y):
    global raw_depth

    # get one (or more) points from the pointcloud (unfortunately you need an iterator) These X,Y,Z values are in
    # the zed2_left_camera_optical_frame (X is seen as going to left (looking the camera) Y is top to bottom and
    # Z pointing out the camera). You can select the pixel in the image frame with the u,v variable, u = 0 ,
    # v = 0 is the top right corner, u = 640 , v = 360 is the middle of the image plane which corresponds to the
    # origin of the zed2_left_camera_optical_frame
    points_list = []
    for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=False,
                                         uvs=[(int(x), int(y))]):
        points_list.append([data[0], data[1], data[2]])

    return points_list

    # print("Data: ", points_list)


np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

# global variables
raw_image = None
raw_depth = None

image_lock = threading.Lock()
depth_lock = threading.Lock()


# takes raw image and creates jpg-compliant image
def from_raw_to_jpg_compliant():
    global raw_image

    tot_data = len(raw_image.data)
    i = 0

    image_arr = []
    while i < tot_data / (1920 * 3):
        riga = []
        j = 0
        while j < tot_data / (1080 * 3):
            riga.append(struct.unpack_from('3B', raw_image.data, int(j * 3 + i * tot_data / 1080)))
            j += 1

        image_arr.append(riga)

        i += 1

    arr = np.asarray(image_arr, dtype='uint8')

    # img = pilimage.fromarray(arr)

    # img.save('nt.png')

    return arr


def image_depth_processing():
    image = from_raw_to_jpg_compliant()
    global num
    print("image converted to jpg, starting processing")
    if not num:
        process_image(image)
    else:
        skipped
    num+=1
    print("finished image processing\n")


# TODO RENDERLA NON DIPENDENTE DALLA CALLBACK E METTERE VARIABILI 640 e 360
def receive_pointcloud(msg):
    if depth_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg
        # print("depth")

        global raw_depth

        raw_depth = msg

        depth_lock.release()


def receive_image(msg):
    # we need an if else it would continue with the thread without having the lock
    if image_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg
        # print("image")
        global raw_image

        raw_image = msg

        image_lock.release()


def processing_callback(event=None):
    global raw_image
    global raw_depth

    # after an execution I want to require the lock only if data is new,
    # otherwise a lot of processing_callbacks would enqueue and not allow image and depth
    # callbacks to update the data
    if raw_image is not None and raw_depth is not None:
        # here we want blocking locks as every tot seconds we MUST execute the processing part
        if image_lock.acquire(blocking=True) and depth_lock.acquire(blocking=True):  # by default blocking is True
            print("callback")

            image_depth_processing()

            image_lock.release()
            depth_lock.release()

            raw_image = None
            raw_depth = None


res_pub = None

def is_float(element: any) -> bool:
    #If you expect None to be passed:
    if element is None:
        return False
    try:
        float(element)
        return True
    except ValueError:
        print("not float")
        return False


def listener():
    rospy.init_node('vision_node', anonymous=True)
    global num
    num=0
    # registering to image raw (left)
    rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback=receive_image, queue_size=1)

    # registering to depth raw
    rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2,
                     callback=receive_pointcloud, queue_size=1)

    global res_pub
    res_pub = rospy.Publisher('vision_results', BlockList, queue_size=1)

    # Create a ROS Timer for publishing data
    rospy.Timer(rospy.Duration(2), processing_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()