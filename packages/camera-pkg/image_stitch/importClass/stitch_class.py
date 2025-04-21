import numpy as np 
import cv2

# === Calibration Parameters ===
PLANE1 = [-1.7524, 2545.5063]  # Plane 1 (slope, intercept)
PLANE2 = 1770.000              # Reference depth
PLANE3 = [2.0221, 571.8330]    # Plane 3 (slope, intercept)
PLANE1_SCALE = 1.8                  # Scaling from plane 1 → 2
PLANE3_SCALE = 1.8                  # Scaling from plane 3 → 2

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

class VideoStitcher():
    def __init__(self):
        self.coord_cache = {}
        self.image_shape_cache = {}

    def warp(self, images):
        cache_key = tuple(images[0].shape) + (images[0].dtype,)
        
        if cache_key in self.image_shape_cache:
            image_shape, dtype = self.image_shape_cache[cache_key]
        else:
            image_shape = images[0].shape
            dtype = images[0].dtype 
            self.image_shape_cache[cache_key] = (image_shape, dtype)

        if len(image_shape) == 3:
            canvas = np.zeros((image_shape[0], image_shape[1]*3, image_shape[2]), dtype=dtype)
        else: 
            canvas = np.zeros((image_shape[0], image_shape[1]*3), dtype=dtype)
        canvas[:, 1 * image_shape[1]:2 * image_shape[1]] = images[0]
       
        for i, inv_H in enumerate(inv_homography):
            canvas = self.warp_logic(canvas, images[i + 1], inv_H, i, image_shape[0], image_shape[1])
        
        return canvas

    def warp_logic(self, ref_img, src_img, inv_H, index, height, width):
        """
        Warp the source image to the reference image using the inverse homography matrix.
        Returns:
            ref_img (numpy.ndarray): The reference image with the warped source image.
        """
        h_dst, w_dst = height, width*3
        h_src, w_src = height, width

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
    
    def depth_cali(self, stitched_img):
        """
        Depth calibration for the stitched image.
        Args:
            stitched_img (numpy.ndarray): The stitched image to be calibrated.
        Returns:
            numpy.ndarray: The calibrated depth image.
        """
        height, width = stitched_img.shape
        x = np.tile(np.arange(width), (height, 1))

        z = stitched_img.copy()
        depth_cali_img = z.copy()

        # plane 1 → 2: x = 1~360 → x[:, 0:360]
        x1 = x[:, 0:360]
        z1 = z[:, 0:360]
        z_trans_1 = (z1 - (PLANE1[0] * x1 + PLANE1[1])) / np.sqrt(PLANE1[0]**2 + 1) + PLANE2
        depth_cali_img[:, 0:360] = (z_trans_1 - PLANE2) * PLANE1_SCALE + PLANE2

        # plane 3 → 2: x = 721~1080 → x[:, 721:1081]
        right_start = 721
        right_end = min(1081, width)  
        x3 = x[:, right_start:right_end]
        z3 = z[:, right_start:right_end]
        z_trans_3 = (z3 - (PLANE3[0] * x3 + PLANE3[1])) / np.sqrt(PLANE3[0]**2 + 1) + PLANE2
        depth_cali_img[:, right_start:right_end] = (z_trans_3 - PLANE2) * PLANE3_SCALE + PLANE2 - 20
        depth_cali_img[depth_cali_img < 0] = 0
        return depth_cali_img
