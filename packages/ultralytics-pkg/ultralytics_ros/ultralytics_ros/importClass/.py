import cv2 
import numpy as np

LOWER_COLOR = (10, 30, 100)
UPPER_COLOR = (60, 200, 255)

class CounterRecognition(object):
    def __init__(self, img):
        self.img = img
        self.contours = []
        self.hierarchy = []
        self.hsv_img = None
        self.binary_img = None

    def get_binary_img(self):
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(self.hsv_img, (5, 5), 0)
        self.binary_img = cv2.inRange(blurred, LOWER_COLOR, UPPER_COLOR)
        # self.binary_img = cv2.bitwise_not(self.binary_img)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        self.binary_img = cv2.morphologyEx(self.binary_img, cv2.MORPH_OPEN, kernel)
        return self.binary_img
    
    def get_contours(self):
        self.contours, self.hierarchy = cv2.findContours(self.binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return self.contours, self.hierarchy

    def distinguish_contour(self, img):
        for i, contour in enumerate(self.contours):
            if cv2.contourArea(contour) > 5000:
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                box = np.intp(box)
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

                points = box.reshape(-1, 2).astype('float16')
                line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line[0][0], line[1][0], line[2][0], line[3][0]
                print(f"Contour {i}: Line parameters: vx={vx}, vy={vy}, x0={x0}, y0={y0}")
                # Calculate the angle in degrees
                angle = cv2.fastAtan2(-vy, vx)
                if angle > 180:
                    angle -=180
                print(f"Contour {i}: Angle = {angle:.2f} degrees")
                # Draw the line
                cv2.line(img, (int(x0 - 100 * vx), int(y0 - 100 * vy)), (int(x0 + 100 * vx), int(y0 + 100 * vy)), (0, 0, 255), 2)
        return img

def main():
    img = cv2.imread("img/column4.png")
    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if img is None:
        print("Error: Image not found.")
        return
    
    counter_recognition = CounterRecognition(img)
    binary_img = counter_recognition.get_binary_img()
    contours, hierarchy = counter_recognition.get_contours()
    img = counter_recognition.distinguish_contour(img)

    # Display the contours
    cv2.imshow("Binary Image", binary_img)
    cv2.imshow("Contours", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()