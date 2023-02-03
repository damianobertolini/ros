import time

import numpy as np
import rospy
from sensor_msgs.msg import Image
import struct
from PIL import Image as pilimage
import cv2
import torch


def callback(data):
    # print(len(data.data))

    # tot_data = len(data.data) / 3
    tot_data = len(data.data)
    i = 0

    image_arr = []
    while i < tot_data / (1280 * 3):
        riga = []
        j = 0
        while j < tot_data / (720 * 3):
            riga.append(struct.unpack_from('3B', data.data, int(j * 3 + i * tot_data / 720)))
            j += 1

        image_arr.append(riga)

        #   print(i)

        i += 1
    # print(image_arr)

    # print("creating asarray")

    arr = np.asarray(image_arr, dtype='uint8')
    # print("creating pilimage")

    img = pilimage.fromarray(arr)
    # print("saving")

    # img.save('new.png')
    # print("done")
    print("now")
    process_image(arr)
    print("here")

    time.sleep(30)


# change absolute path to your best.pt path


# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image(image):
    try:
        model = torch.hub.load("../", 'custom', path="../best.pt", source='local')

        # image = cv2.imread("../../datasets/lego/test/images/untitled2.jpg",cv2.IMREAD_COLOR)
        print("ora")

        image = cv2.resize(image, [640, 360], interpolation=cv2.INTER_AREA)

        cv2.imwrite("dagazebo.jpg", image)

        print("salvato")


        time.sleep(5)
        print("scrivo su topic")
        model.conf = 0.7
        results = model(image)
        result_dict = results.pandas().xyxy[0].to_dict(orient="records")

        print(result_dict)
        for detected in result_dict:
            cs = float(detected['name'])
            x1 = float(detected['xmin'])
            y1 = float(detected['ymin'])
            x2 = float(detected['xmax'])
            y2 = float(detected['ymax'])
            conf = float(detected['confidence'])
            xc = float(((x2 - x1) / 2) + x1)
            yc = float(((y2 - y1) / 2) + y1)

            coordinate = [cs, x1, y1, x2, y2, conf, xc, yc]

            print(str(coordinate))

            eps = 10
            crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]

            name = str(int(cs)) + "_cropped.jpg"
            cv2.imwrite(name, crop_img)

        print("finito")

    except Exception as err:
        print(err)


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova quindi min_x min_y ecc del contorno
# poi trova anche il punto centrale e disegna un cerchio in quel punto
# salva tutto in edge.jpg
def canny_img():
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)
    template = cv2.imread('prova.jpg', cv2.IMREAD_GRAYSCALE)

    edges = cv2.Canny(template, 50, 150)

    arr = []
    [[arr.append([row, column]) for column, val in enumerate(e) if val == 255] for row, e in enumerate(edges)]

    # print(arr)

    min_y = arr[0][0]
    max_y = arr[len(arr) - 1][0]

    min_x = min([val[1] for val in arr])
    max_x = max([val[1] for val in arr])

    print("min and max values: ", end="")
    print(min_y, max_y, min_x, max_x)

    cv2.rectangle(edges, (min_x, min_y), (max_x, max_y), (255, 255, 255), 2)

    # find and draw center
    edges = drawCenter((min_x, min_y, max_x, max_y), edges)

    cv2.imwrite("edgeWithCenter.jpg", edges)

    print("done")

    return min_x, min_y, max_x, max_y


def drawCenter(coordinates, image):
    xc = int((coordinates[2] - coordinates[0]) / 2 + coordinates[0])
    yc = int((coordinates[3] - coordinates[1]) / 2 + coordinates[1])

    image = cv2.circle(image, (xc, yc), radius=2, color=(255, 255, 255), thickness=10)

    print(xc, yc, coordinates)
    return image


# trova minimo rettangolo che ingloba il contorno trovato con canny e lo salva  in minRect.jpg, utile anche per
# rotazioni sull'asse z
def minAreaRect():
    img = cv2.imread('2_cropped.jpg', 0)
    ret, thresh = cv2.threshold(img, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    cnt = contours[0]

    tmp = cv2.minAreaRect(cnt)
    tmp2 = cv2.boxPoints(tmp)

    box = np.int0(tmp2)

    print(box)

    cv2.drawContours(img, [box], 0, (255, 255, 255), 2)

    cv2.imwrite("minRect.jpg", img)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
