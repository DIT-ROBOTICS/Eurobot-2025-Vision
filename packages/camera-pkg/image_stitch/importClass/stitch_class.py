from rclpy.node import Node
import numpy as np 
import cv2

inv_homography = [
    np.array([
        [ 4.49513351e-01,  1.63480510e-02,  2.63303221e+01],
        [-2.23835724e-01,  6.82183468e-01,  8.68291550e+01],
        [-7.24774675e-04,  2.13018909e-05,  9.64446193e-01]
    ]),
    np.array([
        [ 1.11729775e+00, -1.27874995e-02, -6.72112101e+02],
        [ 3.17391523e-01,  1.05354360e+00, -2.09593535e+02],
        [ 1.07387610e-03, -7.85071157e-06,  3.53929865e-01]
        ])
]

class VideoStitcher(Node):
    def __init__(self):
        super().__init__('video_stitcher')
        self.coord_cache = {}

    def warp(self, images):
        # Expand the canvas to fit all images
        height, width = images[0].shape[:2]
        canvas = np.zeros((height, width * 3, 3), dtype=np.uint8)
        canvas[:, 1 * width:2 * width] = images[0]
       
        for i, inv_H in enumerate(inv_homography):
            canvas = self.warp_logic(canvas, images[i + 1], inv_H, i)
        
        return canvas

    def warp_logic(self, ref_img, src_img, inv_H, index):
        h_dst, w_dst = 640, 1080
        h_src, w_src = 640, 360

        cache_key = (h_dst, w_dst, inv_H.tobytes())

        if cache_key not in self.coord_cache:
            map_x, map_y = np.meshgrid(np.arange(w_dst), np.arange(h_dst))
            map_x = map_x.flatten()
            map_y = map_y.flatten()
            coords = np.vstack((map_x, map_y, np.ones_like(map_x)))

            mapped_coords = inv_H @ coords
            mapped_coords /= mapped_coords[2, :]

            x_src = mapped_coords[0, :].reshape(h_dst, w_dst).astype(np.float32)
            y_src = mapped_coords[1, :].reshape(h_dst, w_dst).astype(np.float32)
    
            self.coord_cache[cache_key] = (x_src, y_src)
        else:
            x_src, y_src = self.coord_cache[cache_key]
        
        remapped_img = cv2.remap(src_img, x_src, y_src, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        
        if index == 0:
            ref_img[:, :w_src] = remapped_img[:, :w_src]
        else:
            ref_img[:, -w_src:] = remapped_img[:, -w_src:]

        return ref_img
