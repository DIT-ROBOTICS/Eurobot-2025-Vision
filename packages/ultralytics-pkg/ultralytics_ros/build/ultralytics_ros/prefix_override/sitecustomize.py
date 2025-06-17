import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ultralytics/vision-ws/src/ultralytics-ros/ultralytics_ros/install/ultralytics_ros'
