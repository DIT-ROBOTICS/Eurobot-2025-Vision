import cv2
import numpy as np

class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

def Depth_Contour(depth_cv_image_path):
    try:
        depth_cv_image = cv2.imread(depth_cv_image_path)

        if depth_cv_image is None:
            print("Cannot receive frame")
            return

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
                if area > 3000 and area <7000:
                    # calculate rectangular
                    x, y, w, h = cv2.boundingRect(contour)
                    # draw
                    cv2.rectangle(depth_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.drawContours(depth_cv_image, [contour], -1, (0, 255, 0), 2)
                    # area
                    cv2.putText(depth_cv_image, f'Area: {int(area)}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.imshow('Depth_with_Contours', depth_cv_image)
                    cv2.waitKey(1)
                    
        #             depth_rec = Rectangle(x, y, w, h)
        
        # return depth_rec

    except Exception as e:
        print(f"Error: {e}")

def Color_Contour(color_cv_image_path):
    try:
        color_cv_image = cv2.imread(color_cv_image_path)

        if color_cv_image is None:
            print("Cannot receive frame")
            return
        
        gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        kernel = np.ones((5,5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 3000 and area < 7000:
                    x, y, w, h = cv2.boundingRect(contour)
                    # draw
                    cv2.rectangle(color_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.drawContours(color_cv_image, [contour], -1, (0, 255, 0), 2)
                    # area
                    cv2.putText(color_cv_image, f'Area: {int(area)}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    cv2.imshow("Color_with_Contours", color_cv_image)
                    cv2.waitKey(1)
    
    except Exception as e:
        print(f"Error: {e}")

def ColorRecTrans(color_cv_image_path):
    try:
        color_cv_image = cv2.imread(color_cv_image_path)
        bbox = (0, 0, 0, 0)

        if color_cv_image is None:
            print('Cannot receive color frame')
            return
        
        gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        kernel = np.ones((5,5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 3000:
                    bbox = cv2.boundingRect(contour)
                    break
        
        return bbox

    except Exception as e:
        print(f"Error: {e}")

def DepthRecTrans(depth_cv_image_path):
    try:
        depth_cv_image = cv2.imread(depth_cv_image_path, cv2.IMREAD_GRAYSCALE)
        bbox = (0, 0, 0, 0)

        if depth_cv_image is None:
            print('Cannot receive depth frame')
            return
        
        binary = cv2.adaptiveThreshold(depth_cv_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        kernel = np.ones((5,5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 3000:
                    bbox = cv2.boundingRect(contour)
                    break
        
        return bbox

    except Exception as e:
        print(f"Error: {e}")

def ColorMultiRecTrans(color_cv_image_path):
    try:
        color_cv_image = cv2.imread(color_cv_image_path)
        bbox_list = [(0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0)]
        count = 0

        if color_cv_image is None:
            print('Cannot receive color frame')
            return
        
        gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        kernel = np.ones((5,5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if count == 3:
                    count = 0
                    break
                if area > 3000:
                    bbox_list[count] = cv2.boundingRect(contour)
                    count = count + 1
        
        return bbox_list

    except Exception as e:
        print(f"Error: {e}")