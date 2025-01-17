import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(dict=cv2.aruco.DICT_4X4_100)

marker_size = 6
marker_id_range = range(0, 100)
dictionary_list = []

for marker_id in marker_id_range:
    marker_image = cv2.aruco.drawMarker(dictionary, marker_id, marker_size)
    hex_data = marker_image.tobytes().hex()
    odd_position_hex = hex_data[1::2] 
    odd_position_hex = odd_position_hex[6:-6]
    grouped_hex = [odd_position_hex[i:i+6] for i in range(0, len(odd_position_hex), 6)]
    grouped_hex_modified = [group[1:-1] for group in grouped_hex]
    modified_hex = [group.replace('f', '1') for group in grouped_hex_modified]
    binary_hex_data = ''.join([f'{int(b, 2):x}' for b in modified_hex])
    dictionary_list.append(f"0x{binary_hex_data}UL")

print(dictionary_list)