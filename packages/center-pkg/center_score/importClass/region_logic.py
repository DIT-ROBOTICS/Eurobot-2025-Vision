import cv2
import numpy as np

SIMA_REGION = {
    'sima_1': {
        'points': np.array([[1.8, 1.2], [1.65, 1.55], 
                            [1.95, 1.55], [1.95, 1.6], 
                            [2.3, 1.45], [2.3, 1.6]]),
    },
    'sima_2': {
        'points': np.array([[1.25, 1.55], [1.75, 1.55], 
                            [1.15, 1.2], [1.85, 1.2]]),
    },
    'sima_3': {
        'points': np.array([[1.05, 1.55], [1.35, 1.55], 
                            [1.05, 1.6], [0.75, 1.6],
                            [1.45, 1.6], [1.2, 1.2]]),
    },
}

HOME_REGION = {
    'x_min': 2.40,
    'x_max': 2.85,
    'y_min': 1.55,
    'y_max': 2.00,
}

BUILD_REGION = {
    'build_1': {
        'x_min': 0.00+0.1,
        'x_max': 0.45+0.1,
        'y_min': 0.65,
        'y_max': 1.10,
    },
    'build_2': {
        'x_min': 0.00+0.1,
        'x_max': 0.45+0.1,
        'y_min': 0.00+0.1,
        'y_max': 0.15+0.2,
    },
    'build_3': {
        'x_min': 1.55,
        'x_max': 2.00,
        'y_min': 0.00+0.1,
        'y_max': 0.45+0.2,
    },
    'build_4': {
        'x_min': 2.00,
        'x_max': 2.45,
        'y_min': 0.00+0.1,
        'y_max': 0.15+0.2,
    },
}

STAGE_REGION = {
    'x_min': 1.05,
    'x_max': 1.95,
    'y_min': 1.40,
    'y_max': 1.55,
}

class RegionLogic():
    def __init__(self, sensor_node):
        self.sensor_node = sensor_node

    def is_point_in_polygon(self, point, polygon_points):
        polygon_points = polygon_points.astype(np.float32)
        result = cv2.pointPolygonTest(polygon_points, point, False)
        return result >= 0

    def check_superstar_pose_in_region(self):
        superstar_point = 0
        data = self.sensor_node.get_sensor_data()
        superstar_pose = data['superstar_pose']

        if superstar_pose is None:
            return 0

        x = superstar_pose.pose.position.x
        y = superstar_pose.pose.position.y

        if STAGE_REGION['x_min'] <= x <= STAGE_REGION['x_max'] and \
           STAGE_REGION['y_min'] <= y <= STAGE_REGION['y_max']:
            superstar_point += 15

        return superstar_point

    def check_sima_pose_in_region(self):
        sima_point = 0
        region_hit_count = 0
        data = self.sensor_node.get_sensor_data()
        sima_pose = data['sima_pose_array']

        if sima_pose is None or not sima_pose.poses:
            return 0

        for sima_id, region in SIMA_REGION.items():
            polygon_points = region['points']
            for pose in sima_pose.poses:
                point = (pose.position.x, pose.position.y)
                if self.is_point_in_polygon(point, polygon_points):
                    sima_point += 5
                    region_hit_count += 1
                    break
        
        if region_hit_count == 3:
            sima_point += 10

        return sima_point

    def check_robot_pose_in_region(self):
        data = self.sensor_node.get_sensor_data()
        robot_pose = data['robot_pose']
        
        if robot_pose is None:
            return 0

        x = robot_pose.pose.position.x
        y = robot_pose.pose.position.y

        if HOME_REGION['x_min'] <= x <= HOME_REGION['x_max'] and \
           HOME_REGION['y_min'] <= y <= HOME_REGION['y_max']:
            return 10
        return 0

    def check_platform_pose_in_region(self):
        build_point = 0
        data = self.sensor_node.get_sensor_data()
        platform_pose = data['platform_pose_array']
        
        if platform_pose is None or not platform_pose.poses:
            return 0

        # for build_id, region in BUILD_REGION.items():
        for pose in platform_pose.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            if BUILD_REGION['build_1']['x_min'] <= x <= BUILD_REGION['build_1']['x_max'] and \
                BUILD_REGION['build_1']['y_min'] <= y <= BUILD_REGION['build_1']['y_max']:
                if z >=0.32:
                    build_point += 28
                elif z >=0.2:
                    build_point += 12
                elif z >0.08:
                    build_point += 4
        for pose in platform_pose.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            if BUILD_REGION['build_2']['x_min'] <= x <= BUILD_REGION['build_2']['x_max'] and \
                BUILD_REGION['build_2']['y_min'] <= y <= BUILD_REGION['build_2']['y_max']:
                if z >=0.28:
                    build_point += 28
                elif z >=0.18:
                    build_point += 12
                elif z >0.08:
                    build_point += 4
        for pose in platform_pose.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            if BUILD_REGION['build_3']['x_min'] <= x <= BUILD_REGION['build_3']['x_max'] and \
                BUILD_REGION['build_3']['y_min'] <= y <= BUILD_REGION['build_3']['y_max']:
                if z >=0.26:
                    build_point += 28
                elif z >=0.16:
                    build_point += 12
                elif z >0.06:
                    build_point += 4
        for pose in platform_pose.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            if BUILD_REGION['build_4']['x_min'] <= x <= BUILD_REGION['build_4']['x_max'] and \
                BUILD_REGION['build_4']['y_min'] <= y <= BUILD_REGION['build_4']['y_max']:
                if z >=0.26:
                    build_point += 28
                elif z >=0.16:
                    build_point += 12
                elif z >0.06:
                    build_point += 4

        return build_point
