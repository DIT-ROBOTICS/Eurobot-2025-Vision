import cv2
import numpy as np

# 顏色範圍：HSV 空間
LOWER_COLOR = (10, 30, 100)
UPPER_COLOR = (60, 200, 255)

class CounterRecognition:
    def __init__(self):
        self.img = None
        self.contours = []
        self.hierarchy = []
        self.hsv_img = None
        self.binary_img = None

    def set_image(self, img):
        self.img = img

    def get_binary_img(self):
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(self.hsv_img, (3, 3), 0)
        self.binary_img = cv2.inRange(blurred, LOWER_COLOR, UPPER_COLOR)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_OPEN, kernel)
        return self.binary_img

    def get_contours(self):
        self.contours, self.hierarchy = cv2.findContours(self.binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def distinguish_contour(self, img):
        angle = 0.0
        for i, contour in enumerate(self.contours):
            if cv2.contourArea(contour) > 500:
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                box = np.intp(box)

                points = box.reshape(-1, 2).astype('float16')
                line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line[0][0], line[1][0], line[2][0], line[3][0]
                angle = cv2.fastAtan2(-vy, vx)
                if angle > 180:
                    angle -= 180

                cv2.line(
                    img,
                    (int(x0 - 100 * vx), int(y0 - 100 * vy)),
                    (int(x0 + 100 * vx), int(y0 + 100 * vy)),
                    (0, 0, 255), 2
                )
        return angle
        # return angle if angle is not None else 0.0
