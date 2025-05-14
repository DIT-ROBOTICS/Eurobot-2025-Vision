import cv2
import numpy as np
LOWER_platform = (0, 0, 100)
UPPER_platform = (60, 255, 255)
LOWER_WHITE = (0, 0, 180) 
UPPER_WHITE = (180, 50, 255)
LOWER_RED1 = (0, 80, 80)   
UPPER_RED1 = (15, 255, 255)
LOWER_RED2 = (165, 80, 80)
UPPER_RED2 = (180, 255, 255)
LOWER_BLUE = (90, 80, 80)  
UPPER_BLUE = (140, 255, 255)
LOWER_GREEN = (35, 80, 80)
UPPER_GREEN = (85, 255, 255)
LOWER_YELLOW = (15, 80, 80)
UPPER_YELLOW = (45, 255, 255)
LOWER_BLACK = (0, 0, 0)
UPPER_BLACK = (180, 255, 50)
LOWER_DARK_BLUE = (90, 100, 0)
UPPER_DARK_BLUE = (130, 255, 80)
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
        self.binary_img = cv2.inRange(blurred, LOWER_platform, UPPER_platform)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_OPEN, kernel)
        return self.binary_img

    def get_binary_img_colu(self):
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.hsv_img = cv2.medianBlur(self.hsv_img, 3)
        mask_white = cv2.inRange(self.hsv_img, LOWER_WHITE, UPPER_WHITE)
        mask_red1 = cv2.inRange(self.hsv_img, LOWER_RED1, UPPER_RED1)
        mask_red2 = cv2.inRange(self.hsv_img, LOWER_RED2, UPPER_RED2)
        mask_blue = cv2.inRange(self.hsv_img, LOWER_BLUE, UPPER_BLUE)
        mask_green = cv2.inRange(self.hsv_img, LOWER_GREEN, UPPER_GREEN)
        mask_yellow = cv2.inRange(self.hsv_img, LOWER_YELLOW, UPPER_YELLOW)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_combined = cv2.bitwise_or(mask_white, mask_red)
        mask_combined = cv2.bitwise_or(mask_combined, mask_blue)
        mask_combined = cv2.bitwise_or(mask_combined, mask_green)
        mask_combined = cv2.bitwise_or(mask_combined, mask_yellow)
        mask_black = cv2.inRange(self.hsv_img, LOWER_BLACK, UPPER_BLACK)
        mask_dark_blue = cv2.inRange(self.hsv_img, LOWER_DARK_BLUE, UPPER_DARK_BLUE)
        mask_exclude = cv2.bitwise_or(mask_black, mask_dark_blue)
        self.binary_img = cv2.bitwise_and(mask_combined, cv2.bitwise_not(mask_exclude))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_CLOSE, kernel, iterations=3)
        return self.binary_img

    def get_contours(self):
        self.contours, self.hierarchy = cv2.findContours(self.binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def distinguish_contour(self, img , global_pose,material):
        angle = 0.0
        for i, contour in enumerate(self.contours):
            if material == "platform":
                gate = 600
            elif material == "overturn":
                gate = 80
            if cv2.contourArea(contour) > gate:
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                box = np.intp(box)
                points = box.reshape(-1, 2).astype('float16')
                line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line[0][0], line[1][0], line[2][0], line[3][0]
                angle = cv2.fastAtan2(-vy, vx)
                if angle > 180:
                    angle -= 180
                if(global_pose.position.x >= 2.90 or global_pose.position.x <= 0.10):
                    angle = 90.0
                if(global_pose.position.x <=0.15):
                    angle = 0.0
                cv2.line(
                    img,
                    (int(x0 - 100 * vx), int(y0 - 100 * vy)),
                    (int(x0 + 100 * vx), int(y0 + 100 * vy)),
                    (0, 0, 255), 2
                )
                angle = angle * np.pi / 180
                if material == "overturn":
                    print(f"overturn angle: {angle}")
                if material == "platform":
                    print(f"platform angle: {angle}")
        return angle