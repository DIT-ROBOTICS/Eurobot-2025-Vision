# import cv2

#  def Depth_Contour(depth_cv_image):
#     ret, binary = cv2.adaptiveThreshold(depth_cv_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
#     contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if contours:
#         # search all contour
#         for contour in contours:
#             # calculate all contours if too small then ignore
#             area = cv2.contourArea(contour)
#             if area > 500:
#                 # calculate rectangular
#                 x, y, w, h = cv2.boundingRect(contour)
#                 cv2.rectangle(depth_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
#                 cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
#                 cv2.imshow("Depth", depth_cv_image)
#                 cv2.waitKey(1)
    
#     return contours

# def Color_Contour(color_cv_image):
#     gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
#     ret, binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
#     binary = cv2.erode(binary, None, iterations=2)
#     binary = cv2.dilate(binary, None, iterations=2)
#     contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if contours:
#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area > 500:
#                 x, y, w, h = cv2.boundingRect(contour)
#                 cv2.rectangle(binary, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 cv2.rectangle(color_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
#                 cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
#                 cv2.imshow("Color", color_cv_image)
#                 cv2.waitKey(1)
#                 cv2.imshow("Gray", binary)
#                 cv2.waitKey(1)

#     return contours