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

# for locks
import threading

import time

import cv2
import torch
import numpy as np
import math

from imutils import contours as imcontours


# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image(image):
    model = torch.hub.load("../../../yolov5/", 'custom', path="../../../yolov5/best.pt", source='local')

    # resize image for yolo detection (from 1280x720 to 640x360)
    image_resized = cv2.resize(image, [640, 360], interpolation=cv2.INTER_AREA)

    model.conf = 0.6
    results = model(image_resized)
    result_dict = results.pandas().xyxy[0].to_dict(orient="records")

    print("result dict:")
    print(result_dict)
    for detected in result_dict:
        cs = float(detected['name'])
        x1 = float(detected['xmin'])
        y1 = float(detected['ymin'])
        x2 = float(detected['xmax'])
        y2 = float(detected['ymax'])
        conf = float(detected['confidence'])

        # check if bounding box is inside table, else discard label
        if x1 < 280 or x2 > 514 or y1 < 140 or y2 > 305:
            continue  # not inside table

        # per template matching uso immagine 1280per 720 -> trasformo da 640 360
        x1 = 2 * x1
        x2 = 2 * x2
        y1 = 2 * y1
        y2 = 2 * y2

        # TODO aggiungere ricerca in base a conf maggiore e eliminare predictions vicine

        eps = 10
        crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]

        # TODO delete
        global lamiadepth

        from cv_bridge import CvBridge

        bridge = CvBridge()

        depth = bridge.imgmsg_to_cv2(lamiadepth, "32FC1")

        depth_pass = depth[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]

        to_crop_depth = []

        for y in range(int(x1) + 5, int(x2) - 5 + 1):
            for x in range(int(y1) + 5, int(y2) - 5 + 1):
                to_crop_depth.append([y, x])
                # print(y,x)

        pca(to_crop_depth)

        return

        check_block(crop_img, depth_pass)

        # name = str(int(cs)) + "_cropped.jpg"

        # new part, read cropped image, find contours (draw rectangle) and draw center point
        cannied_img = canny_img(crop_img, 3, str(int(cs)))

        rect_center, rect_width_height, rect_angle = min_area_rect(cannied_img, str(int(cs)))

        # get points for depth sampling used in inclination calculation
        get_points_for_depth_sampling(rect_center, rect_width_height, rect_angle)

        line_angle = None

        # per successivo calcolo depth del centro

        print(f"center coord for image {str(int(cs))}: ", end="")
        print(rect_center)

        real_coord_x, real_coord_y = from_cropped_to_real_world_coord(rect_center, x1, y1, eps)

        # per calcolo inclination
        inclination = experimental_detect(crop_img, rect_center, str(int(cs)))

        print(inclination)

        # TODO ora manca la depth

        # print(center, real_coord_x, real_coord_y, image.shape, eps)
        center_depth = calculate_depth(real_coord_x, real_coord_y)

        print("Center point depth:")
        print(center_depth)

        # print circle on image centre
        with_center_image = cv2.circle(image, (real_coord_x, real_coord_y), radius=1, color=(255, 255, 255),
                                       thickness=5)

        # return to original image sizes (find real coordinates of found centre)

        cv2.imwrite("final.jpg", with_center_image)

    print("finito")


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova
# quindi min_x min_y ecc del contorno poi trova anche il punto centrale e disegna un cerchio in quel punto salva
# tutto in edge.jpg
def canny_img(img, aperture_size, name):
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cannied_img = cv2.Canny(img_gray, 50, 150, apertureSize=aperture_size, L2gradient=True)

    cv2.imwrite(f'{name}_cannied.jpg', cannied_img)

    return cannied_img

    # il resto da qui non serve ( fa un rettangolo dritto attorno al valore max e min dove trova un valore 255 (un
    # contorno)) -> rettangolo attorno al contorno più grande

    arr = []
    [[arr.append([row, column]) for column, val in enumerate(e) if val == 255] for row, e in enumerate(edges)]

    # print(arr)

    min_y = arr[0][0]
    max_y = arr[len(arr) - 1][0]

    min_x = min([val[1] for val in arr])
    max_x = max([val[1] for val in arr])

    # print("min and max values: ", end="")
    # print(min_y, max_y, min_x, max_x)

    cv2.rectangle(edges, (min_x, min_y), (max_x, max_y), (255, 255, 255), 2)

    # find and draw center
    edges, center = draw_center((min_x, min_y, max_x, max_y), edges)

    cv2.imwrite("edgeWithCenter.jpg", edges)

    # print("done")

    return (min_x, min_y, max_x, max_y), center


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
    cv2.imwrite(f'{name}_minRect.jpg', img)

    print(min_area_rectangle)

    min_area_rect_angle = min_area_rectangle[2]

    min_area_rect_width = min_area_rectangle[1][0]
    min_area_rect_height = min_area_rectangle[1][1]

    print(name, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    # if min_area_rect_width < min_area_rect_height:
    #    min_area_rect_angle = min_area_rect_angle + 90
    # else:
    #    min_area_rect_angle = min_area_rect_angle + 180

    # print(min_area_rect_angle)

    return min_area_rectangle


def experimental_detect(original_img, img_centre, name, no_blur=False):
    # img = cv2.imread(f'{img_number}_cannied.jpg', 0)

    img = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)

    # TODO set this as optional (some images might not detect anything if blurred
    if not no_blur:
        img = cv2.GaussianBlur(img, (7, 7), sigmaX=1.5, sigmaY=1.5)

    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        max_rad_circle = max(15, int(math.ceil(max(img.shape) / 4 / 2)))
        # print(max_rad_circle, img.shape)

        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            1,
            5,  # TODO Consider setting this parameter as variable (ex image.shape[mindistance] / tot)
            param1=50,
            param2=15,
            minRadius=5,
            maxRadius=max_rad_circle,
        )

        circles = np.uint16(np.around(circles))

        incl = get_inclination(circles, img_centre, img.shape)

        # print(incl)

        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

        # display that image
        # cv2.imshow('GFG', cimg)

        cv2.imwrite(f'{name}_det_circles.jpg', cimg)

    except Exception as e:
        print(e)

        canny_img(original_img, 5, name)

        incl = experimental_detect(original_img, img_centre, name, True)

    return incl




def get_inclination(circles, img_centre, image_shape):
    x_cr = 0
    y_cr = 0

    for circle in circles[0]:
        x_cr += circle[0]
        y_cr += circle[1]

    x_cr /= len(circles[0])
    y_cr /= len(circles[0])

    diff_x = abs(img_centre[0] - x_cr)
    diff_y = abs(img_centre[1] - y_cr)

    inclination = []

    if diff_x > diff_y:
        if x_cr < img_centre[0]:
            inclination.append("left")
        else:
            inclination.append("right")

        if y_cr < img_centre[1]:
            inclination.append("top")
        else:
            inclination.append("bottom")
    else:
        if y_cr < img_centre[1]:
            inclination.append("top")
        else:
            inclination.append("bottom")

        if x_cr < img_centre[0]:
            inclination.append("left")
        else:
            inclination.append("right")

    return inclination

    # if diff_x >= (image_shape[1] / 7):
    #     if img_centre[0] - x_cr > 0:
    #         inclination.append("left")
    #     else:
    #         inclination.append("right")
    #
    # if diff_y >= (image_shape[0] / 7):
    #     if img_centre[1] - y_cr > 0:
    #         inclination.append("top")
    #     else:
    #         inclination.append("bottom")

    # if inclination == "":
    #     return "None"
    # else:
    #     return inclination


# if __name__ == '__main__':
#     #process_image()
#     for i in range(30):
#         try:
#             canny_img(i, 3)
#             # posso mettere in houghcircles 16 invece di 15 e mettere aperturesize 5 e blur a 13 (ad es anche 15 ...)
#             centre = min_area_rect(i)
#             experimental_detect(i, centre)
#
#             print("\n")
#         except Exception as e:
#             from termcolor import colored
#             print(colored(e, 'red'))

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


def pca(to_crop):
    global raw_depth
    import pcl

    points_list = []

    min = 2
    max = 0
    prev = 0
    for u, v in to_crop:
        for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(u, v)]):
            # print(data, u, v)

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

    print("MIN MAX")
    print(min, max)
    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    from sklearn.decomposition import PCA

    # Perform PCA
    pca = PCA(n_components=2)
    pca.fit(pcl_data)

    # Print the explained variance ratio for each principal component
    print(pca.explained_variance_ratio_)
    print(pca.components_)


def get_points_for_depth_sampling(rect_center, rect_width_height, rect_angle):
    rect_width = rect_width_height[0]
    rect_height = rect_width_height[1]

    min_width = rect_center[0] - (rect_width / 2)
    max_width = rect_center[0] + (rect_width / 2)

    max_height = rect_center[1] + (rect_height / 2)

    # find line that passes through center and point of (max_width, 0)

    sample_points_x_translated = [max_width * 4 / 5, max_width * 3 / 5, max_width * 2 / 5]

    sample_points_y_translated = [x * math.tan(rect_angle) for x in sample_points_x_translated]

    sample_points_x = [max_width - x for x in sample_points_x_translated]

    sample_points_y = [max_height - y for y in sample_points_y_translated]

    sample_point_depths = [calculate_depth(x, y) for x, y in zip(sample_points_x, sample_points_y)]

    return sample_point_depths


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
    while i < tot_data / (1280 * 3):
        riga = []
        j = 0
        while j < tot_data / (720 * 3):
            riga.append(struct.unpack_from('3B', raw_image.data, int(j * 3 + i * tot_data / 720)))
            j += 1

        image_arr.append(riga)

        i += 1

    arr = np.asarray(image_arr, dtype='uint8')

    # img = pilimage.fromarray(arr)

    # img.save('nt.png')

    return arr


def image_depth_processing():
    image = from_raw_to_jpg_compliant()

    print("image converted to jpg, starting processing")
    process_image(image)
    print("finished image processing")


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


lamiadepth = None


def tada(msg):
    # we need an if else it would continue with the thread without having the lock
    if image_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg
        # print("lamiadepth")
        global lamiadepth

        lamiadepth = msg

        image_lock.release()


def processing_callback(event=None):
    global raw_image
    global raw_depth
    global lamiadepth

    # after an execution I want to require the lock only if data is new,
    # otherwise a lot of processing_callbacks would enqueue and not allow image and depth
    # callbacks to update the data
    if raw_image is not None and raw_depth is not None and lamiadepth is not None:
        # here we want blocking locks as every tot seconds we MUST execute the processing part
        if image_lock.acquire(blocking=True) and depth_lock.acquire(blocking=True):  # by default blocking is True
            print("callback")

            image_depth_processing()

            image_lock.release()
            depth_lock.release()

            raw_image = None
            raw_depth = None


# function to check if a block is standing or leaning
def check_block(image, depth):
    # convert depth image to grayscale

    gray_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # threshold the depth image to create a binary image
    ret, thresh = cv2.threshold(gray_rgb, 0, 255, cv2.THRESH_BINARY)

    # find contours in the binary image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    # calculate the moments of the largest contour
    moments = cv2.moments(max_contour)

    # calculate the center of mass of the contour
    if moments["m00"] != 0:
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
    else:
        center_x, center_y = 0, 0

    # draw the center of mass on the RGB image
    cv2.circle(rgb_image, (center_x, center_y), 5, (0, 255, 0), -1)

    cv2.imwrite("standing_leaning.jpg", rgb_image)

    # calculate the standard deviation of the depth values in the region of interest
    roi = depth[center_y - 10:center_y + 10, center_x - 10:center_x + 10]
    std_dev = np.std(roi)

    # set a threshold for determining if the block is standing or leaning
    threshold = 0.014
    print("center, std_dev, thres:", end="")
    print(center_x, center_y, std_dev, threshold)
    if std_dev > threshold:
        print("The block is standing.")
    else:
        print("The block is leaning.")


def listener():
    rospy.init_node('vision_node', anonymous=True)

    # registering to image raw (left)
    rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback=receive_image, queue_size=1)

    # registering to depth raw
    rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2,
                     callback=receive_pointcloud, queue_size=1)

    # registering to depth raw
    rospy.Subscriber("/ur5/zed_node/depth/depth_registered", Image,
                     callback=tada, queue_size=1)

    # Create a ROS Timer for publishing data
    rospy.Timer(rospy.Duration(2), processing_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
