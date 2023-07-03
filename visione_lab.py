from __future__ import print_function

import math
import random
import struct
# for locks
import threading
import time

import cv2
import numpy as np
import rospy
import torch
from imutils import contours as imcontours
# import ros_numpy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

# custom created messages
from my_vision_messages.msg import Pointxyz
from my_vision_messages.msg import Block
from my_vision_messages.msg import BlockList


# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image(image):
    #model = torch.hub.load("yolov5/", 'custom', path="yolov5/pesi yolo/GazebobestNoResizeGray.pt", source='local')

    model = torch.hub.load("yolov5/", 'custom', path="yolov5/pesi_yolo/Realbest90.pt", source='local')

    # resize image for yolo detection (from 1280x720 to 640x360)
    image_resized = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)

    #cv2.imwrite("res.jpg", image_resized)
    #cv2.imwrite("img.jpg", image)


    model.conf = 0.6
    results = model(image_resized)
    result_dict = results.pandas().xyxy[0].to_dict(orient="records")

    print("result dict:")
    print(result_dict)

    to_delete = []
    id=0

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

            if abs(x11 - x12) < 5 and abs(y11 - y12) < 5 and abs(x21 - x22) < 5 and abs(y21 - y22) < 5:  # two very near labels
                if conf1 > conf2:   # take the one with best accuracy
                    to_delete.append(conf2)
                else:
                    to_delete.append(conf1)

    for to_del in to_delete:
        if to_del in result_dict:   # since the above part is not very efficient it will take every to_del twice
            # example: x1.conf > x2.conf, but later in the for loop: x2.conf < x1.conf -> counted twice
            result_dict.remove(to_del)

    block_list = BlockList()
    block_list_depths = []

        # --------------------------------------
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

    # --------------------------------------

    # main function start for every piece
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

        # check if bounding box is inside table, else discard label
        #table_corners=[340//1.5,900//1.5,200//1.5,500//1.5] in 960p
        table_corners=[730,1380,250,750]#xxyy
        #table_corners=[340,900,200,500]

        image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)

        # per template matching uso immagine 1280per 720 -> trasformo da 640 360
        # nel lab da 960*540 a 640*360
        molt = 3
        x1 = molt * x1
        x2 = molt * x2
        y1 = molt * y1
        y2 = molt * y2

        if x1 < table_corners[0] or x2 > table_corners[1] or y1 < table_corners[2] or y2 > table_corners[3]:
            print("saltato")
            print(x1, x2, y1, y2)
            continue  # not inside table

        # TODO occhio che potrebbe eseguire due volte cannied e experimental detect solo perche ci sono due
        # risultati vicini (magari anche della stessa classe)

        eps = 10
        crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]
        if crop_img.shape[0]==0 or crop_img.shape[1]==0:
            continue
        #cv2.imwrite('image.jpg', image)
        #cv2.imwrite('crop.jpg', crop_img)
        # new part, read cropped image, find contours (draw rectangle) and draw center point

        #cannied_img = canny_img(crop_img, 3, str(int(cs)))
        #print("crop shape: ",crop_img.shape)
        siz=crop_img.shape
        low_res_mult=2
        low_res_crop = cv2.resize(crop_img, (siz[0] // low_res_mult, siz[1] // low_res_mult), interpolation=cv2.INTER_CUBIC)
        cannied_img = canny_img(low_res_crop, 3, str(int(cs)))#low res funziona meglio per il rect
        cv2.imwrite(f'canny_{cs}.jpg', cannied_img)
        global rect_angle

        rect_center, rect_width_height, rect_angle = min_area_rect(cannied_img, str(int(cs)),low_res_mult)
        #print("rect center ",rect_center)
        rect_center*=2
        rect_width_height*=2
        rect_angle*=2
        print("rect center ",rect_center)
        # print(f"center coord for image {str(int(cs))}: ", end="")
        # print(rect_center)

        real_coord_x, real_coord_y = from_cropped_to_real_world_coord(rect_center, x1, y1, eps)

        # per calcolo inclination
        inclination = experimental_detect(crop_img, rect_center, str(int(cs)))

        print(inclination)

        # taking values for cropping depth map taken from cloud_registered topic
        to_crop_depth = []

        # TODO search for areas where more points are present -> finds circles of blocks but in 3d
        # i want the cropped point cloud to have the minimum between deltax and delta y equal to this value
        target_to_crop = 5

        if x2 - x1 > y2 - y1:
            value_to_crop = int((y2 - y1 - target_to_crop) / 2)
        else:
            value_to_crop = int((x2 - x1 - target_to_crop) / 2)

        # get values which will be used to get the cropped point cloud
        for y in range(int((x1+ value_to_crop)//molt), int((x2- value_to_crop + 1)//molt)):
            for x in range(int((y1+ value_to_crop)//molt), int((y2- value_to_crop + 1)//molt)):
                to_crop_depth.append([y, x])
                #print(y,x)
        """
        for y in range(0,540):
            for x in range(0,960):
                to_crop_depth.append([y, x])
                # print(y,x)
        todo: valori depth negativi
        """

        final_incl = pca(to_crop_depth)
        #final_incl=100

        print("")
        print(final_incl)
        print("")

        # print(center, real_coord_x, real_coord_y, image.shape, eps)
        center_depth = calculate_depth(real_coord_x, real_coord_y,0.5)#0.5 da 1920x1080 a 960x540 per la raw depth


        # print("Center point depth:")
        # print(center_depth[0][2])

        # print circle on image centre
        with_center_image = cv2.circle(image, (real_coord_x, real_coord_y), radius=1, color=(255, 255, 255),
                                       thickness=7)
        with_center_image = cv2.circle(with_center_image, (table_corners[0], table_corners[2]), radius=1, color=(255, 180, 120),
                                       thickness=7)
        with_center_image = cv2.circle(with_center_image, (table_corners[1], table_corners[3]), radius=1,
                                       color=(255, 180, 120),
                                       thickness=7)
        # return to original image sizes (find real coordinates of found centre)

        cv2.imwrite("final.jpg", with_center_image)


        print("-----------------------------------")
        point = Pointxyz(real_coord_x, real_coord_y, center_depth[0][2])
        print(f"x:{real_coord_x},\ty:{real_coord_y}, \tdepth denter: {center_depth[0][2]}")

        w_R_c = np.matrix([[0., - 0.49948, 0.86632], [-1., 0., 0.], [-0., - 0.86632, - 0.49948]])
        w_c = np.array([-0.9, 0.24, -0.35])
        base_offset = np.array([0.5, 0.35, 1.75])

        #w_R_c = np.matrix([[0.866,0.,0.5], [-0., - 1, 0.], [- 0.49948,0.,0.866]])
        #w_c = np.array([-0.9, 0.24, -0.35])
        base_offset = np.array([-0.58, 0.49, 1.38])

        real_coord_x = real_coord_x
        real_coord_y = real_coord_y

        #points_list = calculate_depth(real_coord_x, real_coord_y,1/molt)
        points_list = calculate_depth(real_coord_x, real_coord_y, 0.5)

        #points_list[0]=[0.4, 0.05, 0.71]

        #print(f"test depth angolo = ", calculate_depth(1600, 700, 0.5) )
        #print(f"x reale = {real_coord_x}, y reale = {real_coord_y}")
        print("Data Optical frame: ", points_list)
        #pointW = np.dot(w_R_c, points_list[0]) + w_c + base_offset
        pointW = np.dot(w_R_c, points_list[0]) + base_offset #coordinate nel world frame senza robot
        print("Data World frame: ", pointW)
        #print("Data World frame 2: ", pointW2)
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
        block.top_btm_l_r = inclination
        block.up_dw_stand_lean = final_incl
        block.confidence = conf
        block.world_point = Pointxyz(robo[0],robo[1],robo[2])


        # sort block list based on x component (depth)
        block_list_depths.append(center_depth[0][0])

        block_list_depths.sort(reverse=True)
        new_block_index = [i[0] for i in enumerate(block_list_depths) if i[1] == center_depth[0][0]]

        block_list.blocks.insert(new_block_index[0], block)

    # when finished publish result but before sort BlockList by depth
    global res_pub

    res_pub.publish(block_list.blocks)

    print("finito")
    #exit(0)


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova
# quindi min_x min_y ecc del contorno poi trova anche il punto centrale e disegna un cerchio in quel punto salva
# tutto in edge.jpg
def canny_img(img, aperture_size, name):
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)

    print(img.shape)
    #cv2.imwrite('pre-canny.jpg', img)
    #print("\nfatto")
    #time.sleep(1000)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cannied_img = cv2.Canny(img_gray, 50, 150, apertureSize=aperture_size, L2gradient=True)

    cv2.imwrite('post-canny.jpg', cannied_img)

    return cannied_img


def draw_center(coordinates, image):
    xc = int((coordinates[2] - coordinates[0]) / 2 + coordinates[0])
    yc = int((coordinates[3] - coordinates[1]) / 2 + coordinates[1])

    image = cv2.circle(image, (xc, yc), radius=2, color=(255, 255, 255), thickness=10)

    # print(xc, yc, coordinates)
    return image, (xc, yc)


# trova minimo rettangolo che ingloba il contorno trovato con canny e lo salva  in minRect.jpg, utile anche per
# rotazioni sull'asse z
def min_area_rect(img, name, low_res_mult=1):
    # print(img)

    # with open("pix.txt", "w") as f:
    #     for line in img:
    #         f.write(str(line) + "\n")

    ret, thresh = cv2.threshold(img, 180, 256, 0)

    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    #print(contours)

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

    # print(min_area_rectangle)

    min_area_rect_angle = min_area_rectangle[2]

    min_area_rect_width = min_area_rectangle[1][0]*low_res_mult
    min_area_rect_height = min_area_rectangle[1][1]*low_res_mult
    # print(name, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    if min_area_rect_width < min_area_rect_height:
        min_area_rect_angle = min_area_rect_angle + 90
    else:
        min_area_rect_angle = min_area_rect_angle + 180

    print(name, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    return (min_area_rectangle[0][0]*low_res_mult,min_area_rectangle[0][1]*low_res_mult), min_area_rectangle[1], min_area_rect_angle


def experimental_detect(original_img, img_centre, name, no_blur=False, redone_canny=False):
    # img = cv2.imread(f'{img_number}_cannied.jpg', 0)
    img = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)

    if not no_blur:
        img = cv2.GaussianBlur(img, (7, 7), sigmaX=1.5, sigmaY=1.5)

    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        max_rad_circle = max(15, int(math.ceil(max(img.shape) / 4 / 2)))
        # print("circle stats: ",max_rad_circle, img.shape)

        variable_min_dist = 1/5 * (max(original_img.shape))

        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            1,
            variable_min_dist,
            param1=50,
            param2=15 if redone_canny is False else 10,   # the less it is the more false positives circles
            minRadius=10,
            maxRadius=max_rad_circle,

        )

        circles = np.uint16(np.around(circles))

        # print("circles")
        # print(circles)

        incl = get_inclination(circles, img_centre, img.shape)

        # print(incl)

        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

        # display that image
        # cv2.imshow('GFG', cimg)
        import random

        #cv2.imwrite(f'{random.randint(1,500)}bb.jpg', cimg)
        cv2.imwrite(f'circles_{name}.jpg', cimg)
    except Exception as e:
        if redone_canny:
            # significa che anche con gli improvements non riconosce i cerchi, quindi è probabile
            # che siano dall'altra parte rispetto alla telecamera
            global rect_angle
            if 180 < rect_angle < 270:  # ruotato di un angolo acuto rispetto all'asse x (uscente dal robot)
                incl = ["top", "left"]
            else:
                incl = ["top", "right"]

            return incl

        print("Exception " + str(e))

        canny_img(original_img, 5, name)

        incl = experimental_detect(original_img, img_centre, name, no_blur=True, redone_canny=True)

    return incl


def get_inclination(circles, img_centre, image_shape):
    x_cr = 0
    y_cr = 0

    # # remove a circle if two circles are detected and one is too big compared to the other (probably false positive)
    # if len(circles[0]) == 2 and abs(int(circles[0][0][2]) - int(circles[0][1][2])) > 4:
    #     if circles[0][1][2] > circles[0][0][2]:
    #         circles = [np.array(circles[0][0]).tolist()]
    #     else:
    #         circles = [np.array(circles[0][1]).tolist()]
    #     print(circles)

    # more circles detected -> create line to fit circles center and detect better if the block is top/bottom or right/left
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
    real_x = int(int(x1) - eps + point[0])
    real_y = int(int(y1) - eps + point[1])

    return real_x, real_y


# FOR DEPTH CALCULATION BASED ON A POINT IN CAMERA VIEW
def calculate_depth(x, y, res_mult=0.5):
    global raw_depth#960x540
    x=int(x*res_mult)
    y=int(y*res_mult)

    #y=360-y
    #return[[1,2,3]]
    # get one (or more) points from the pointcloud (unfortunately you need an iterator) These X,Y,Z values are in
    # the zed2_left_camera_optical_frame (X is seen as going to left (looking the camera) Y is top to bottom and
    # Z pointing out the camera). You can select the pixel in the image frame with the u,v variable, u = 0 ,
    # v = 0 is the top right corner, u = 640 , v = 360 is the middle of the image plane which corresponds to the
    # origin of the zed2_left_camera_optical_frame
    points_list = []

    inc  = 0
    while(inc < 20):

        non_nan=[0,0,0]
        summ = [0,0,0]
        for x_i in range(x-inc,x+inc):
            for y_i in range(y-inc,y+inc):

                for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=False,
                                                     uvs=[(int(x_i), int(y_i))]):

                    for i, d in enumerate(data):#per ogni dato in data[]
                        if d==d and not math.isinf(d):#se non e' nan
                            summ[i]+=d
                            non_nan[i]+=1


        if non_nan[0]>0 and non_nan[1]>0 and non_nan[2]>0 :#trovati dei valori
            for i, val in enumerate(summ):
                summ[i] = val / non_nan[i]#faccio a media
            data = summ#in summ c'e' la media
            break
        else:#ho trovato solo nan
            inc+=1

    #points_list.append([data[0], data[1], data[2]])
    points_list.append([-data[1], -data[2], data[0]])

    ##cambio coordinate per zed reale
    #points_list[0] = [-points_list[0][1], -points_list[0][2], points_list[0][0]]
    ##

    return points_list

    # print("Data: ", points_list)


def pca(to_crop):
    global raw_depth
    import pcl

    #print(f"len pca: {len(to_crop)}")
    #print("to crop")
    #print(to_crop)

    #cv2.imwrite('to_crop.jpg', to_crop)
    points_list = []

    min_pca = 2
    max_pca = 0
    prev = 0
    for u, v in to_crop:
        for data in point_cloud2.read_points(raw_depth, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(u, v)]):
            #print(data, u, v)

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

            points_list.append([data[0], data[1], data[2], 255])  # faking rgb values

    # remove max depth values which might be point of the table
    # print(len(points_list))
    # for point in points_list:
    #     if point[2] >= max - (max - min) * 0.2:
    #         points_list.remove(point)
    #
    #print(len(points_list))
    if len(points_list) <=1:
        return "point list vuota"

    #print("MIN MAX")
    #print(min_pca, max_pca)
    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    from sklearn.decomposition import PCA

    # Perform PCA
    pca_conv = PCA(n_components=2)

    try:
        pca_conv.fit(pcl_data)
    except:
        print(to_crop)
        print(points_list)

    # Print the explained variance ratio for each principal component
    #print(pca_conv.explained_variance_ratio_)
    #print(pca_conv.components_)

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
    print(f"inclinazione: {incl}")
    return incl

    # first_comp_max = max(pca.components_[0])


np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

# global variables
raw_image = None
raw_depth = None
rect_angle = None

image_lock = threading.Lock()
depth_lock = threading.Lock()


# takes raw image and creates jpg-compliant image
def from_raw_to_jpg_compliant():
    global raw_image

    #cv2.imwrite("raw.jpg", raw_image)


    tot_data = len(raw_image.data)
    i = 0

    x=960
    y=540

    image_arr = []
    while i < tot_data / (x * 4 ):
        riga = []
        j = 0
        while j < tot_data / (y * 4):
            riga.append(struct.unpack_from('3B', raw_image.data, int(j * 4 + i * tot_data / y )))
            j += 1

        image_arr.append(riga)

        i += 1

    arr = np.asarray(image_arr, dtype='uint8')

    cv2.imwrite(f"raw_{random.random()*1000}.jpg", arr)
    # img = pilimage.fromarray(arr)

    # img.save('nt.png')

    return arr


def image_depth_processing():
    image = from_raw_to_jpg_compliant()

    print("image converted to jpg, starting processing")
    process_image(image)
    print("finished image processing")


def receive_pointcloud(msg):
    if depth_lock.acquire(blocking=False):  # don't want callback to wait indefinitely with an old msg
        # print("depth")

        global raw_depth

        with open("depth", "wb") as binary_file:
            # Write bytes to file
            binary_file.write(msg.data)
            uuiui=0
        tmp=b
        """
        try:
            with open("depth", "rb") as f:
                byte = f.read()
                msg.data = byte
                if msg.data == byte:
                    print("caricata depth")
        except IOError:
            print('Error While Opening the file!')
        """
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


def listener():
    rospy.init_node('vision_node', anonymous=True)

    # registering to image raw (left)
    #rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback=receive_image, queue_size=1)
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