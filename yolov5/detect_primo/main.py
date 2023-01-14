import cv2
import torch
import numpy as np

from imutils import contours as imcontours


# change absolute path to your best.pt path

# esegue yolo detect.py su immagine specificata nel path e da in output immagini trovate e ritagliate
def process_image():
    try:
        model = torch.hub.load("../", 'custom', path="../best.pt", source='local')

        image = cv2.imread(
            "/Users/damiano/Desktop/UNIVERSITA'/TERZO ANNO TRENTO/ROBOTICA/copia di repo github/ros/datasets/lego/test/images/untitled2.jpg",
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
            cv2.imwrite(name, crop_img)

            # new part, read cropped image, find contours (draw rectangle) and draw center point
            boundings, center = canny_img(name)

            print(f"center coord for image {name}: ", end="")
            print(center)

            real_coord_x = int(x1) - eps + center[0]
            real_coord_y = int(y1) - eps + center[1]

            image = cv2.circle(image, (real_coord_x, real_coord_y), radius=1, color=(255, 255, 255), thickness=5)

            # return to original image sizes (find real coordinates of found centre)

        cv2.imwrite("final.jpg", image)

        print("finito")

    except Exception as err:
        print(err)


# crea bounding box rettangolare NON ruotata attorno a contorno del blocco (usa canny per trovare edges) e trova quindi min_x min_y ecc del contorno
# poi trova anche il punto centrale e disegna un cerchio in quel punto
# salva tutto in edge.jpg
def canny_img(img_number):
    # img = cv2.imread("untitled2.jpg", cv2.IMREAD_GRAYSCALE)
    template = cv2.imread(f'{img_number}_cropped.jpg', cv2.IMREAD_GRAYSCALE)

    edges = cv2.Canny(template, 50, 150, apertureSize=3, L2gradient=True)

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

    min_area_rect = cv2.minAreaRect(cnt)
    tmp2 = cv2.boxPoints(min_area_rect)

    box = np.int0(tmp2)

    # print(box)

    cv2.drawContours(img, [box], 0, (255, 255, 255), 2)

    cv2.imwrite(f'{img_number}_minRect.jpg', img)

    min_area_rect_angle = min_area_rect[2]

    min_area_rect_width = min_area_rect[1][0]
    min_area_rect_height = min_area_rect[1][1]

    print(img_number, " : ", min_area_rect_angle, min_area_rect_width, min_area_rect_height)

    # if min_area_rect_width < min_area_rect_height:
    #    min_area_rect_angle = min_area_rect_angle + 90
    # else:
    #    min_area_rect_angle = min_area_rect_angle + 180

    # print(min_area_rect_angle)


def experimental_detect(img_number):
    img = cv2.imread(f'{img_number}_cannied.jpg', 0)

    img = cv2.GaussianBlur(img, (7, 7), sigmaX=1.5, sigmaY=1.5)

    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=15,
            minRadius=5,
            maxRadius=15,
        )

        circles = np.uint16(np.around(circles))

        print(circles)
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
        print('non va')


def get_inclination():
    pass


if __name__ == '__main__':
    # process_image()
    for i in [0, 2, 6, 7, 8]:
        canny_img(i)
        min_area_rect(i)
        # altro_detect("8_cropped.jpg", "cannied.jpg")
        experimental_detect(i)
