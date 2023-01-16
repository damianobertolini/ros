import time

import cv2
import torch
import numpy as np
import math

from imutils import contours as imcontours


# change absolute path to your best.pt path

# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image():
    try:
        model = torch.hub.load("../", 'custom', path="../best.pt", source='local')

        image = cv2.imread(
            "/Users/damiano/Desktop/UNIVERSITA'/TERZO ANNO TRENTO/ROBOTICA/copia di repo github/ros/yolov5/detect_primo/prove/18.jpg",
            cv2.IMREAD_COLOR)

        print("scrivo su topic")
        model.conf = 0.6
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

            # print(str(coordinate))

            eps = 10
            crop_img = image[int(y1) - eps:int(y2) + eps, int(x1) - eps:int(x2) + eps]

            name = str(int(cs)) + "_cropped.jpg"
            cv2.imwrite("prove/" + name, crop_img)
            time.sleep(3)

            # new part, read cropped image, find contours (draw rectangle) and draw center point
            canny_img(int(cs), 3)

            center = min_area_rect(int(cs))

            print(f"center coord for image {name}: ", end="")
            print(center)

            real_coord_x = int(int(x1) - eps + center[0])
            real_coord_y = int(int(y1) - eps + center[1])

            image = cv2.circle(image, (real_coord_x, real_coord_y), radius=1, color=(255, 255, 255), thickness=5)

            # return to original image sizes (find real coordinates of found centre)

        cv2.imwrite("final.jpg", image)

        print("finito")

    except Exception as err:
        print(err)


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova quindi min_x min_y ecc del contorno
# poi trova anche il punto centrale e disegna un cerchio in quel punto
# salva tutto in edge.jpg
def canny_img(img_number, aperture_size):
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)
    template = cv2.imread(f'{img_number}_cropped.jpg', cv2.IMREAD_GRAYSCALE)

    edges = cv2.Canny(template, 50, 150, apertureSize=aperture_size, L2gradient=True)

    cv2.imwrite(f'{img_number}_cannied.jpg', edges)

    return

    # il resto da qui non serve ( fa un rettangolo dritto attorno al valore max e min dove trova un valore 255 (un contorno)) -> rettangolo attorno al contorno più grande

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


# trova minimo rettangolo che ingloba il contorno trovato con canny e lo salva  in minRect.jpg, utile anche per rotazioni sull'asse z
def min_area_rect(img_number):
    img = cv2.imread(f'{img_number}_cannied.jpg', 0)
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

    cv2.imwrite(f'{img_number}_minRect.jpg', img)

    min_area_rect_angle = min_area_rectangle[2]

    min_area_rect_width = min_area_rectangle[1][0]
    min_area_rect_height = min_area_rectangle[1][1]

    print(img_number, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    # if min_area_rect_width < min_area_rect_height:
    #    min_area_rect_angle = min_area_rect_angle + 90
    # else:
    #    min_area_rect_angle = min_area_rect_angle + 180

    # print(min_area_rect_angle)

    return min_area_rectangle[0]


def experimental_detect(img_number, img_centre):
    img = cv2.imread(f'{img_number}_cannied.jpg', 0)

    img = cv2.GaussianBlur(img, (7, 7), sigmaX=1.5, sigmaY=1.5)

    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        max_rad_circle = max(15, int(math.ceil(max(img.shape) / 4 / 2)))
        #print(max_rad_circle, img.shape)


        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=15,
            minRadius=5,
            maxRadius=max_rad_circle,
        )

        circles = np.uint16(np.around(circles))

        incl = get_inclination(circles, img_centre, img.shape)

        print(incl)

        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

        # print(f'{a} funziona')
        # display that image
        cv2.imshow('GFG', cimg)

        cv2.imwrite(f'{img_number}_det_circles.jpg', cimg)

        cv2.waitKey(0)
    except Exception as e:
        print(e)

        canny_img(img_number, 5)
        experimental_detect(img_number, img_centre)


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

if __name__ == '__main__':
    #process_image()
    for i in range(30):
        try:
            canny_img(i, 3)
            # posso mettere in houghcircles 16 invece di 15 e mettere aperturesize 5 e blur a 13 (ad es anche 15 ...)
            centre = min_area_rect(i)
            experimental_detect(i, centre)

            print("\n")
        except:
            from termcolor import colored
            pass
            #print(colored("number not present", 'red'))
