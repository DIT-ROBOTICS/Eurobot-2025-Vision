import rclpy
import cv2

def Select(image, tracker, tracking):
    frame = image
    if frame is None:
        print("Cannot receive frame")
        return
    
    cv2.imshow('frame', frame)
    keyName = cv2.waitKey(1)

    if not tracking:
        area = cv2.selectROI('Select', frame, showCrosshair=False, fromCenter=False)
        tracker.init(frame, area)

    return tracker, True

def Track(image, tracker, tracking):
    frame = image
    if tracking:
        success, point = tracker.update(frame)
        if success:
            p1 = [int(point[0]), int(point[1])]
            p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
            cv2.rectangle(frame, p1, p2, (0, 0, 255), 3)

        cv2.imshow('Tracking', frame)