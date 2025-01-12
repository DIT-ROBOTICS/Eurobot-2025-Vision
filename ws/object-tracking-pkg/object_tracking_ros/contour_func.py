import cv2
import numpy as np
import os

class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

def Depth_Contour(depth_cv_image_path):
    depth_cv_image = cv2.imread(depth_cv_image_path)

    # normalize
    depth_normalized = cv2.normalize(depth_cv_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_normalized = np.uint8(depth_normalized)
    depth_gray = cv2.cvtColor(depth_normalized, cv2.COLOR_BGR2GRAY)
    
    binary = cv2.adaptiveThreshold(depth_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    kernel = np.ones((5,5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    depth_rec = Rectangle(x=0, y=0, w=0, h=0)

    if contours:
        # search all contour
        for contour in contours:
            # calculate all contours if too small then ignore
            area = cv2.contourArea(contour)
            if area > 500:
                # calculate rectangular
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(depth_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.drawContours(depth_cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.imshow('Depth_with_Contours', depth_cv_image)
                cv2.waitKey(1)
                depth_rec = Rectangle(x, y, w, h)
    
    return depth_rec

def Color_Contour(color_cv_image_path):
    color_cv_image = cv2.imread(color_cv_image_path)
    gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
    binary = cv2.erode(binary, None, iterations=2)
    binary = cv2.dilate(binary, None, iterations=2)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(binary, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.rectangle(color_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.drawContours(color_cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.imshow("Color", color_cv_image)
                cv2.waitKey(0)

    return contours