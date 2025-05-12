import cv2
import numpy as np

# 顏色範圍：HSV 空間
LOWER_COLOR1 = (0, 0, 100)
UPPER_COLOR1 = (60, 255, 255)
LOWER_COLOR2 = (0, 100, 100)
UPPER_COLOR2 = (20, 210, 240)

class CounterRecognition:
    def __init__(self):
        self.img = None
        self.contours = []
        self.hierarchy = []
        self.hsv_img = None
        self.binary_img = None

    def set_image(self, img):
        self.img = img

    def get_binary_img_plat(self):
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(self.hsv_img, (3, 3), 0)
        self.binary_img = cv2.inRange(blurred, LOWER_COLOR1, UPPER_COLOR1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_OPEN, kernel)
        return self.binary_img

    def get_binary_img_colu(self):
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(self.hsv_img, (3, 3), 0)
        self.binary_img = cv2.inRange(blurred, LOWER_COLOR2, UPPER_COLOR2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_OPEN, kernel)
        return self.binary_img

    def get_contours(self):
        self.contours, self.hierarchy = cv2.findContours(self.binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def distinguish_contour(self, img , global_pose,material):
        angle = 0.0
        for i, contour in enumerate(self.contours):
            if material == "platform":
                gate = 600
            elif material == "overturn":
                gate = 100
                print("overturn check1")
            if cv2.contourArea(contour) > gate:
                
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                box = np.intp(box)

                points = box.reshape(-1, 2).astype('float16')
                line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line[0][0], line[1][0], line[2][0], line[3][0]
                angle = cv2.fastAtan2(-vy, vx)
                if angle > 180:
                    angle -= 180
                if(global_pose.position.x >= 2.85 or global_pose.position.x <= 0.15):
                    angle = 90.0
                if(global_pose.position.x <=0.15):
                    angle = 0.0
                cv2.line(
                    img,
                    (int(x0 - 100 * vx), int(y0 - 100 * vy)),
                    (int(x0 + 100 * vx), int(y0 + 100 * vy)),
                    (0, 0, 255), 2
                )
                # Degree to radian
                angle = angle * np.pi / 180
                if material == "overturn":
                    print(f"overturn angle: {angle}")
                if material == "platform":
                    print(f"platform angle: {angle}")
                
        return angle
        # return angle if angle is not None else 0.0