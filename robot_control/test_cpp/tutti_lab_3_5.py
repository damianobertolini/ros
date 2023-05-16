# various useful libraries
from __future__ import print_function
import math
import random
import time
import numpy as np

# for locks
import threading

# for image manipulation and opencv algorithms
import cv2
from imutils import contours as imcontours
import struct


# for Yolo
import torch

# ROS libraries
import rospy
import ros_numpy

# default messages
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

# custom created messages
from my_vision_messages.msg import Pointxyz
from my_vision_messages.msg import Block
from my_vision_messages.msg import BlockList


def process_image(image):
    """Processes image executing the YOLOv5 detection on the specified image and gives as output the detected images cropped around the bounding boxes.
       The process is divided in steps:
       1. Resize the image in order to adapt it to YoloV5's training images resolution
       2. Execute YoloV5 image detection identifying blocks' classes and bounding boxes
       3. Remove blocks which are too close one to each other based on their confidence
       4. For each identified block:
            a. Discard guesses whose bounding boxes are outside the table (false positives)
            b. Crop the identified bounding box
            c. Apply Canny algorithm for edge detection (a filter which calculates the gradient and outputs 1 if this is above a certain threshold, 0 otherwise)
            d. Find the biggest contour from the image filtered by canny algorithm and draw the smallest rectangle which contains that contour.
               Return the rotation angle of that rectangle (rotation of block) and its 2-dimensional center
            e. Find circles in the block and establish a first 3D position (ex. mean of circles' centre is less than the block's center -> likely to be upside down or leaning with upper side towards the camera)
            f. Crop the Point Cloud given by the core points of the 2D identified block
            g. Apply Principal Component Analysis to the cropped Point Cloud; this will output how much the 3D directions (x, y, z) influence the 1st and 2nd principal component.
               This way we can distinguish if in the previous case the block upside down (x and y influence most the PCs) or leaning (x and z where z points towards the camera)
            h. Calculate the mean of the depths of the points nearby the center of the identified block
            i. Output the results

        Arguments
        ---------
        image : numpy array
            The image to execute the detection on

        Returns
        -------
            None

        Output
        -------
        block_list: BlockList()
            List of all detected blocks. Each block is identified by class, 3D position from the camera view, 3D position from the world frame, rotation angle,
            confidence of the block taken from Yolo, inclination based on position of circles identified in the block,
            final_incl (real 3D position based on Principal Component Analysis of the Point Cloud)cropped around their bounding boxes

        """

    # loading the model
    model = torch.hub.load("yolov5/", 'custom', path="yolov5/pesi_yolo/Realbest90.pt", source='local')

    # resize image for yolo detection (from 960x540 to 640x360)
    image_resized = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)

    model.conf = 0.6
    results = model(image_resized)  # executing YoloV5

    result_dict = results.pandas().xyxy[0].to_dict(orient="records")

    print("result dict:")
    print(result_dict)

    to_delete = []

    # delete close predictions based on greater confidence
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
    block_list_depths = []

    # dictionary for converting class numbers to class names provided in the task requirements
    dictionary = {
        "X1-Y1-Z2": 0,
        "X1-Y2-Z2-CHAMFER": 1,
        "X1-Y2-Z2": 2,
        "X1-Y3-Z2": 3,
        "X1-Y4-Z2": 4,
        "X2-Y2-Z2": 5,
        "X1-Y2-Z1": 6,
        "X1-Y2-Z2-TWINFILLET": 7,
        "X1-Y3-Z2-FILLET": 8,
        "X1-Y4-Z1": 9,
        "X2-Y2-Z2-FILLET": 10,
    }

    # main function start for every piece
    for detected in result_dict:
        # cs = float(detected['name'])
        print(detected['name'])
        if detected['name'] in dictionary:
            cs = dictionary[detected['name']]
        else:
            cs = 12  # workaround in case of an unidentified class number

        x1 = float(detected['xmin'])
        y1 = float(detected['ymin'])
        x2 = float(detected['xmax'])
        y2 = float(detected['ymax'])
        conf = float(detected['confidence'])

        # check if bounding box is inside table, else discard label
        # table_corners=[340//1.5,900//1.5,200//1.5,500//1.5] in 960p
        table_corners = [730, 1380, 250, 750]  # xxyy

        # up-scaling original image (interpolation and circles recognition show better results than in 960x540)
        image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)

        # re-scaling bounding box values from 640x360 to new image definition: 1920x1080
        molt = 3
        x1 = molt * x1
        x2 = molt * x2
        y1 = molt * y1
        y2 = molt * y2

        if x1 < table_corners[0] or x2 > table_corners[1] or y1 < table_corners[2] or y2 > table_corners[3]:
            # skipping this block since it is outside the table (false positive)
            continue

        # cropping the identified block
        eps = 10
        crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]
        if crop_img.shape[0] == 0 or crop_img.shape[1] == 0:
            continue


        siz = crop_img.shape
        low_res_mult = 2
        low_res_crop = cv2.resize(crop_img, (siz[0] // low_res_mult, siz[1] // low_res_mult), interpolation=cv2.INTER_CUBIC)

        # apply Canny for identifying edges and contours in image
        cannied_img = canny_img(low_res_crop, 3)

        #calculate block's rotation angle and position
        global rect_angle
        rect_center, rect_width_height, rect_angle = min_area_rect(cannied_img, str(int(cs)), low_res_mult)

        rect_center *= 2
        rect_width_height *= 2
        rect_angle *= 2

        # return from coordinates in cropped image to coordinates in original image
        real_coord_x, real_coord_y = from_cropped_to_real_world_coord(rect_center, x1, y1, eps)

        # calculate inclination based on position of circles identified in the block
        inclination = experimental_detect(crop_img, rect_center, str(int(cs)))
        print(inclination)

        # taking values for cropping depth map taken from cloud_registered topic
        to_crop_depth = []

        # I want the cropped point cloud to have the minimum between deltax and delta y equal to this value
        target_to_crop = 5

        if x2 - x1 > y2 - y1:
            value_to_crop = int((y2 - y1 - target_to_crop) / 2)
        else:
            value_to_crop = int((x2 - x1 - target_to_crop) / 2)

        # get values which will be used to get the cropped point cloud
        for y in range(int((x1 + value_to_crop) // molt), int((x2 - value_to_crop + 1) // molt)):
            for x in range(int((y1 + value_to_crop) // molt), int((y2 - value_to_crop + 1) // molt)):
                to_crop_depth.append([y, x])

        # apply Principal Component Analysis and return the final inclination
        final_incl = pca(to_crop_depth)

        print("")
        print(final_incl)
        print("")

        # get mean of depths of points around block's center
        center_depth = calculate_depth(real_coord_x, real_coord_y, 0.5)  # 0.5 from 1920x1080 to 960x540 for raw depth

        # return to original image sizes (find real coordinates of found centre)
        print("-----------------------------------")
        point = Pointxyz(real_coord_x, real_coord_y, center_depth[0][2])
        print(f"x:{real_coord_x},\ty:{real_coord_y}, \tdepth center: {center_depth[0][2]}")

        w_R_c = np.matrix([[0., - 0.49948, 0.86632], [-1., 0., 0.], [-0., - 0.86632, - 0.49948]])

        base_offset = np.array([-0.58, 0.49, 1.38])

        # points_list = calculate_depth(real_coord_x, real_coord_y,1/molt)
        points_list = calculate_depth(real_coord_x, real_coord_y, 0.5)
        print("Data Optical frame: ", points_list)

        # pointW = np.dot(w_R_c, points_list[0]) + w_c + base_offset
        pointW = np.dot(w_R_c, points_list[0]) + base_offset  # world frame coordinates without the robot
        print("Data World frame: ", pointW)

        robo = np.array([np.squeeze(np.asarray(pointW))[0], np.squeeze(np.asarray(pointW))[1], np.squeeze(np.asarray(pointW))[2]])
        print("Data robot coord: ", robo)
        print("-----------------------------------")

        # creating a block object and adding it to the list which will be the output of this function
        block = Block()
        block.class_number = str(cs)
        block.point = point
        block.rot_angle = rect_angle
        block.top_btm_l_r = inclination
        block.up_dw_stand_lean = final_incl
        block.confidence = conf
        block.world_point = Pointxyz(robo[0], robo[1], robo[2])

        # sort block list based on x component (depth), this will help later to decide which block to pick first
        block_list_depths.append(center_depth[0][0])

        block_list_depths.sort(reverse=True)
        new_block_index = [i[0] for i in enumerate(block_list_depths) if i[1] == center_depth[0][0]]

        block_list.blocks.insert(new_block_index[0], block)

    # when finished publish result
    global res_pub
    res_pub.publish(block_list.blocks)

    print("process image end")
    exit(0)



def canny_img(img, aperture_size):
    """Filters the image passes an input through Canny algorithm in order to perform edge detection

        Arguments
        ---------
        img : numpy array
            The image to execute the detection on

        aperture_size: Integer
            Parameter that represents the size of the kernel for detecting edges

        Returns
        -------
            image filtered through Canny algorithm

        """
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cannied_img = cv2.Canny(img_gray, 50, 150, apertureSize=aperture_size, L2gradient=True)

    return cannied_img


def min_area_rect(img, name, low_res_mult=1):
    """Takes the Canny-filtered image, identifies all the contours in it, sorts those contours based on their size, takes the largest contour as the block's contour.
       Then the function "draws" the smallest rectangle that contains that contour and finds therefore the block's rotation angle. It also outputs the rectangle's center and 2D-sizes

        Arguments
        ---------
        img : numpy array
            The image filtered through Canny algorithm

        name: String
            Class of the block

        low_res_mult: Float
            Parameter necessary due to image scaling

        Returns
        -------
            rect_center (x,y), rectangle width and height, angle of rotation of the block
        """

    ret, thresh = cv2.threshold(img, 180, 256, 0)

    contours, hierarchy = cv2.findContours(thresh, 1, 2)

    # sort found contour in order to take the largest one
    (contours, _) = imcontours.sort_contours(contours)  # not strictly necessary
    contours = sorted(contours, key=lambda c: cv2.contourArea(c))

    assert len(contours) >= 1
    cnt = contours[-1]  # take the largest contour

    min_area_rectangle = cv2.minAreaRect(cnt)   # find smallest rectangle that contains contour

    min_area_rect_angle = min_area_rectangle[2]

    min_area_rect_width = min_area_rectangle[1][0] * low_res_mult
    min_area_rect_height = min_area_rectangle[1][1] * low_res_mult

    # adjust found angle to world frame's angle
    if min_area_rect_width < min_area_rect_height:
        min_area_rect_angle = min_area_rect_angle + 90
    else:
        min_area_rect_angle = min_area_rect_angle + 180

    print(name, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    return (min_area_rectangle[0][0] * low_res_mult, min_area_rectangle[0][1] * low_res_mult), min_area_rectangle[1], min_area_rect_angle


def experimental_detect(original_img, img_centre, name, no_blur=False, redone_canny=False):
    """Searches for circles in the image passed as input using the cv2.HoughCircles() function, then tries to guess a first inclination based on those circle's position.
       In particular, after finding the circles, it finds the best line that represents all the center of the circles (least squares polynomial fit) and uses its parameters
       and the distance between the mean of the circles' centers and the block's center to determine whether the circles are mainly on top/bottom/right/left of the block.
       Since some blocks may have just one circle on (for example) the top-right corner, it returns a list where the first element is the "predominant" direction (easier to be right)
       and the second element is the other less dominant one.

        Arguments
        ---------
        original_img : numpy array
            The image cropped which is supposed to contain the block

        img_centre: Tuple
            Center of the rectangle found in the previous functions

        name: String
            Class of the block

        no_blur: Boolean
            Parameter used for deciding whether to apply a blur to the image before recognizing circles. In case no circles are detected at first, we can try removing the blur to make the image more clear

        redone_canny: Boolean
            Parameter that tells whether the detection has already been performed both with blur and without it (also with other improvements in the image quality). If still no circles are found,
            then image has a too bad quality or no circles are present. This could mean that the block is standing and pointing away from the camera (its circles cannot be seen by the camera view)

        Returns
        -------
            List containing the two most probable inclination, this can be "top", "bottom", "right" or "left"
        """

    img = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)

    # decide whether to apply blur or not. Doing it can remove false positives (works well if image has already a good quality)
    if not no_blur:
        img = cv2.GaussianBlur(img, (7, 7), sigmaX=1.5, sigmaY=1.5)

    try:
        # parameters that define some ranges where the detected circles' size should be contained (too big circles are unlikely to be the block's circle, the same applies to circles too close to each other)
        max_rad_circle = max(15, int(math.ceil(max(img.shape) / 4 / 2)))

        variable_min_dist = 1 / 5 * (max(original_img.shape))

        # main function that detects circles in passed image
        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            1,
            variable_min_dist,
            param1=50,
            param2=15 if redone_canny is False else 10,  # the less it is the more false positives circles
            minRadius=10,
            maxRadius=max_rad_circle,

        )

        circles = np.uint16(np.around(circles))

        # guess inclination based on circles position
        incl = get_inclination(circles, img_centre)


    except Exception as e:
        if redone_canny:    # if no circles found and also image quality improvements have been applied
            # then we assume the circles are located on the other side with respect to the camera view
            global rect_angle
            if 180 < rect_angle < 270:  # if rotated of an acute angle with respect to x-axis
                incl = ["top", "left"]
            else:
                incl = ["top", "right"]

            return incl

        print("Exception " + str(e))

        canny_img(original_img, 5)  # try again with better image quality and more precise circles detection

        incl = experimental_detect(original_img, img_centre, name, no_blur=True, redone_canny=True)

    return incl


def get_inclination(circles, img_centre):
    """Apply the least squares polynomial fit in order to find best line that represents the circles' centers. Based on that and on the distance between the image centre and the average position
       of the circles' centers we get a first estimation whether the block is for example upside-down or turned right

        Arguments
        ---------
        circles : numpy array
            Array of the detected circles

        img_centre: Tuple
            Center of the detected block

        Returns
        -------
            rect_center (x,y), rectangle width and height, angle of rotation of the block
        """

    x_cr = 0
    y_cr = 0

    # if more circles detected -> create line to fit circles center and detect better if the block is top/bottom or right/left
    x = []
    y = []
    priority = None
    if len(circles[0]) > 1:
        for circle in circles[0]:
            x.append(circle[0])
            y.append(circle[1])

        x = np.array(x)
        y = np.array(y)

        # find best line to fit points and return a and b values of line
        a, b = np.polyfit(x, y, 1)

        if abs(a) > 1:
            priority = "right_left"
        else:
            priority = "top_bottom"

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

    if priority is not None:
        if priority == "right_left" and inclination[0] != "right" and inclination[0] != "left":
            tmp = inclination[1]
            inclination[1] = inclination[0]
            inclination[0] = tmp
        else:
            if priority == "top_bottom" and inclination[0] != "top" and inclination[0] != "bottom":
                tmp = inclination[1]
                inclination[1] = inclination[0]
                inclination[0] = tmp

    return inclination


def from_cropped_to_real_world_coord(point, x1, y1, eps):
    """Simple function for converting a point's coordinates in a cropped image to the coordinates in the original image taken from the camera (eventually already rescaled)

        Arguments
        ---------
        point : tuple
            Coordinates of the point in the cropped image, to be converted back to the original image coordinates

        x1, y1: Integer
            Minimum 2D coordinate values (for x- and y-axis) in the original image

        eps: Integer
            Small delta that was applied before cropping the image in order to avoid losing some contours in Canny algorithm

        Returns
        -------
            2D coordinates in the original image (x,y)
        """

    real_x = int(int(x1) - eps + point[0])
    real_y = int(int(y1) - eps + point[1])

    return real_x, real_y


def calculate_depth(x, y, res_mult=0.5):
    """Calculates the depth value given a 2D point's coordinates. Since some depth values could be None a workaround has been implemented which just calculates the avergage of the depths of the points
       within a certain range from the given point

        Arguments
        ---------
        x,y : Integer
            Coordinates of the point

        res_mult: Float
            Parameter necessary due to image scaling

        Returns
        -------
            2D coordinates in the original image (x,y)
        """

    global raw_depth  # 960x540
    x = int(x * res_mult)
    y = int(y * res_mult)

    points_list = []

    inc = 0
    while inc < 20:

        non_nan = [0, 0, 0]
        summ = [0, 0, 0]
        for x_i in range(x - inc, x + inc):
            for y_i in range(y - inc, y + inc):

                for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=False,
                                                     uvs=[(int(x_i), int(y_i))]):

                    for i, d in enumerate(data):  # for each value in data[]
                        if d == d and not math.isinf(d):  # if not nan
                            summ[i] += d
                            non_nan[i] += 1

        if non_nan[0] > 0 and non_nan[1] > 0 and non_nan[2] > 0:  # some values found
            for i, val in enumerate(summ):
                summ[i] = val / non_nan[i]  # averaging
            data = summ  # summ contains the average
            break
        else:  # only nan found
            inc += 1

    points_list.append([-data[1], -data[2], data[0]])

    return points_list


def pca(to_crop):
    """Read depth values of the 2D points given as input. Transform the 3D points read to a PointCloud and then to a PCA-compatible structure.
        Apply Principal Component Analysis to the cropped Point Cloud; this will output how much the 3D directions (x, y, z) influence the 1st and 2nd principal component.
        Find which is the cartesian axis which influences the most the first and second Principal Components. Based on that return the final block's inclination

        Arguments
        ---------
        to_crop : Array of tuples
            Coordinates of the points we want to use in order to calculate the block's inclination

        Returns
        -------
            Final block's inclination
        """

    global raw_depth
    import pcl

    points_list = []

    min_pca = 2
    max_pca = 0
    prev = 0
    for u, v in to_crop:
        for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(u, v)]):
            if data[2] < min_pca:
                min_pca = data[2]
            if data[2] > max_pca:
                if math.isinf(data[2]):
                    continue
                else:
                    max_pca = data[2]

            if prev == 0:
                prev = data[2]
            else:
                if data[2] - prev > 0.01:
                    prev = data[2]
                    continue

            points_list.append([data[0], data[1], data[2], 255])  # faking rgb values (255 for each point)


    if len(points_list) <= 1:
        return "empty point list"


    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    from sklearn.decomposition import PCA

    # Perform PCA
    pca_conv = PCA(n_components=2)

    try:
        pca_conv.fit(pcl_data)
    except:
        print("Error while fitting PCA")

    # get index of most important feature for main component
    max_val = -1
    main_max_idx = 0
    for index, val in enumerate(np.array(pca_conv.components_[0])):
        if abs(val) > max_val:
            main_max_idx = index
            max_val = abs(val)

    # get index of most important feature for second component
    max_val = -1
    second_max_idx = 0
    second_second_val = 0
    for index, val in enumerate(np.array(pca_conv.components_[1])):
        if abs(val) > max_val and index != main_max_idx:
            second_second_val = max_val

            second_max_idx = index
            max_val = abs(val)

    third_idx = [0, 1, 2]
    third_idx.remove(main_max_idx)
    third_idx.remove(second_max_idx)

    diff_second_comp = max_val - second_second_val

    major_features_components = [main_max_idx, second_max_idx]

    # get real inclination
    if 1 not in major_features_components and diff_second_comp > 0.2:
        # print("leaning")
        incl = "leaning"
    else:
        if major_features_components[0] == 1:
            # print("standing")
            incl = "standing"
        else:
            # print("up_or_own")
            incl = "up_or_down"

    print(f"inclination: {incl}")

    return incl


# takes raw image and creates jpg-compliant image
def from_raw_to_jpg_compliant():
    """Transform raw bytes representing image rgb values into a structure which can be saved also as a jpg. The function assumes the input definition is 960x540 and that it is a rgb image,
       therefore reading 3 Bytes for each 2D point

        Returns
        -------
            image that can now be treated and used as a normal jpg image
        """

    global raw_image


    tot_data = len(raw_image.data)
    i = 0

    x = 960
    y = 540

    image_arr = []
    while i < tot_data / (x * 4):
        riga = []
        j = 0
        while j < tot_data / (y * 4):
            riga.append(struct.unpack_from('3B', raw_image.data, int(j * 4 + i * tot_data / y)))
            j += 1

        image_arr.append(riga)

        i += 1

    arr = np.asarray(image_arr, dtype='uint8')

    # now we could save the image for ex. as cv2.imwrite(f"image.jpg", arr)

    return arr


def image_depth_processing():
    """Main function called by the callback which transforms the global raw_image to jpg and calls process_image()
        """

    image = from_raw_to_jpg_compliant()

    print("image converted to jpg, starting processing")
    process_image(image)
    print("finished image processing")


def receive_pointcloud(msg):
    """Callback for the point cloud topic, updates the global variable raw_depth with the new values. It needs the lock for the depth to be acquired before updating it but just continues skipping the update
       if the lock is already taken from process_image()

        Arguments
        ---------
        msg : PointCloud2
            Entire Point Cloud read from depth camera
        """

    if depth_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg

        global raw_depth

        raw_depth = msg

        depth_lock.release()


def receive_image(msg):
    """Callback for the RGB image topic, updates the global variable raw_image with the new values. It needs the lock for the rgb image to be acquired before updating it but just continues skipping the update
       if the lock is already taken from process_image()

        Arguments
        ---------
        msg : Image
            Entire RGB image read from the camera (definition 960x540)
        """
    # we need an if, else it would continue with the thread without having the lock
    if image_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg
        global raw_image

        raw_image = msg

        image_lock.release()


def processing_callback():
    """Callback for the main processing part, it is being called every 2 seconds by a rospy Timer. The function waits to acquire both the locks (remaining idle until it has got all of them) and
       calls the functions responsible for the whole blocks' detection process

        """

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


# instruction used just for printing options, how certain variables (ex. Floats) are displayed
np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

# global variables
raw_image = None
raw_depth = None
rect_angle = None

res_pub = None

# define locks for blocking access to the global depth and rgb image in order to allow process_image() to maintain its integrity during its whole execution
image_lock = threading.Lock()
depth_lock = threading.Lock()


def listener():
    """First setup function for establishing the connection with the ROS middleware infrastructure, creating the vision_node, subscribing to the depth and image topics, setting up a publisher for sharing
       the results with the other ROS nodes (ex. the robot) and configuring all the other callbacks
        """
    rospy.init_node('vision_node', anonymous=True)

    # registering to image raw (left)
    # rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback=receive_image, queue_size=1)
    rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback=receive_image, queue_size=1)

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
