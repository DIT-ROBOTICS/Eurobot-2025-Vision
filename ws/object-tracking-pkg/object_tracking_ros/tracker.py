import cv2

def Track(image, tracker, tracking):
    if image is None:
        print("Cannot receive frame")
        return

    if tracking[0] == False:
        area = cv2.selectROI('Select', image, False, False)
        tracker.init(image, area)
        tracking[0] = True

    if tracking[0] == True:
        success, point = tracker.update(image)
        if success:
            p1 = [int(point[0]), int(point[1])]
            p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
            cv2.rectangle(image, p1, p2, (0, 0, 255), 3)

        cv2.imshow('Tracking', image)

    return

def MultiTrack(image, multitracker, multitracking):
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (125, 125, 125)]

    if image is None:
        print("Cannot receive frame")
        return

    if multitracking[0] == False:
        for i in range(4):
            area = cv2.selectROI('Select', image, False, False)
            tracker = cv2.legacy.TrackerCSRT_create()
            multitracker.add(tracker, image, area)
        multitracking[0] = True
    
    if multitracking[0] == True:
        success, points = multitracker.update(image)
        a = 0

        if success:
            for i in points:
                p1 = [int(i[0]), int(i[1])]
                p2 = [int(i[0] + i[2]), int(i[1] + i[3])]
                cv2.rectangle(image, p1, p2, colors[a], 3)
                a = a + 1

        cv2.imshow('MultiTracking', image)

    return

def AutoTrack(image, autotracker, autotracking, bbox):
    try:
        if bbox is None:
            print('Cannot receive contour.')
            return

        if image is None:
            print('Cannot receive frame.')
            return
        
        if autotracking[0] == False:
            autotracker.init(image, bbox)
            autotracking[0] = True

        if autotracking[0] == True:
            success, point = autotracker.update(image)
            if success:
                p1 = [int(point[0]), int(point[1])]
                p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                cv2.rectangle(image, p1, p2, (0, 0, 255), 3)

            cv2.imshow('AutoTracking', image)
            cv2.waitKey(1)

        return p1, p2
    
    except Exception as e:
        print(f"Error: {e}")

def AutoMultiTrack(image, automultitracker, automultitracking, bbox_list):
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (125, 125, 125)]
    sima_points = []

    try:
        if bbox_list is None:
            print('Cannot receive contour.')
            return

        if image is None:
            print('Cannot receive frame.')
            return
        
        if automultitracking[0] == False:
            for bbox in bbox_list:
                tracker = cv2.legacy.TrackerCSRT_creat()
                automultitracker.add(tracker, image, bbox)
            automultitracking[0] = True

        if automultitracking[0] == True:
            success, points = automultitracker.update(image)
            a = 0
            if success:
                for i in points:
                    p1 = [int(i[0]), int(i[1])]
                    p2 = [int(i[0] + i[2]), int(i[1] + i[3])]
                    sima_points[a] = [(p1[0]+p2[0])/2, (p1[1]+p2[1])/2]
                    cv2.rectangle(image, p1, p2, colors[a], 3)
                    a = a + 1

            cv2.imshow('AutoMultiTracking', image)
            cv2.waitKey(1)

        return sima_points
    
    except Exception as e:
        print(f"Error: {e}")
