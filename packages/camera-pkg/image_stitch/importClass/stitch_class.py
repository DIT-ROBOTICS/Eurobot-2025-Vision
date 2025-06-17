import numpy as np 
import cv2
import cupy as cp
import time

gpu_available = True

# === Calibration Parameters ===
PLANE1 = [-0.2171,1830.5057]  # Plane 1 (slope, intercept)
PLANE2 = 1788.3972              # Reference depth
PLANE3 = [-0.0551,1621.6141]    # Plane 3 (slope, intercept)
PLANE1_SCALE = 1.0                  # Scaling from plane 1 → 2
PLANE3_SCALE = 1.2                  # Scaling from plane 3 → 2

inv_homography = [
    np.array([[ 3.67543648e-01, -2.39732498e-03,  3.48855044e+01],
              [-2.51014460e-01,  5.96460151e-01,  1.13341844e+02],
              [-7.64935836e-04, -1.41529366e-05,  9.22981675e-01]
    ]), 
    np.array([[ 1.12357928e+00,  3.47174904e-02, -7.05088792e+02],
              [ 3.17990404e-01,  1.07313688e+00, -2.16041850e+02],
              [ 1.14956673e-03,  4.30768603e-05,  2.78485899e-01]
    ]),
]

class VideoStitcher():
    def __init__(self):
        self.coord_cache = {}
        self.image_shape_cache = {}

    def warp(self, images):
        if gpu_available:
            # Ensure all images are CuPy arrays
            images = [cp.asarray(img) if not isinstance(img, cp.ndarray) else img for img in images]
        else:
            images = [np.asarray(img) if not isinstance(img, np.ndarray) else img for img in images]

        if gpu_available:
            process_start = time.time()
            cache_key = tuple(images[0].shape) + (images[0].dtype,)
            
            if cache_key in self.image_shape_cache:
                image_shape, dtype = self.image_shape_cache[cache_key]
            else:
                image_shape = images[0].shape
                dtype = images[0].dtype 
                self.image_shape_cache[cache_key] = (image_shape, dtype)

            if len(image_shape) == 3:
                canvas = cp.zeros((image_shape[0], image_shape[1]*3, image_shape[2]), dtype=dtype)
            else: 
                canvas = cp.zeros((image_shape[0], image_shape[1]*3), dtype=dtype)
            canvas[:, 1 * image_shape[1]:2 * image_shape[1]] = images[0]
        
            for i, inv_H in enumerate(inv_homography):
                canvas = self.warp_logic(canvas, images[i + 1], inv_H, i, image_shape[0], image_shape[1])
            process_end = time.time()
            #print(f"[DEBUG] Processing time: {process_end - process_start:.4f} seconds")
            canvas = cp.asnumpy(canvas)
            # print("Output image shape:", canvas.shape)
            return canvas
        else:
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

        cache_key = (h_dst, w_dst, hash(tuple(inv_H.flatten())))
        if gpu_available:
            if cache_key not in self.coord_cache:
                map_x, map_y = cp.meshgrid(cp.arange(w_dst), cp.arange(h_dst))
                map_x = map_x.flatten()
                map_y = map_y.flatten()
                coords = cp.vstack((map_x, map_y, cp.ones_like(map_x)))
                # Ensure coords is a cupy array before matrix multiplication
                inv_H = cp.asarray(inv_H, dtype=cp.float32) if not isinstance(inv_H, cp.ndarray) else inv_H
                mapped_coords = cp.matmul(inv_H, coords)
                mapped_coords /= mapped_coords[2, :]

                x_src = mapped_coords[0, :].reshape(h_dst, w_dst).astype(cp.float32)
                y_src = mapped_coords[1, :].reshape(h_dst, w_dst).astype(cp.float32)

                self.coord_cache[cache_key] = (x_src, y_src)
            else:
                x_src, y_src = self.coord_cache[cache_key]
                
            # Convert x_src and y_src to cv2.cuda.GpuMat
            gpu_x_src = cv2.cuda_GpuMat()
            gpu_y_src = cv2.cuda_GpuMat()
            gpu_x_src.upload(cp.asnumpy(x_src))
            gpu_y_src.upload(cp.asnumpy(y_src))

            # Ensure src_img is a cv2.cuda.GpuMat
            gpu_src_img = cv2.cuda_GpuMat()
            gpu_src_img.upload(cp.asnumpy(src_img))

            # Perform remap with cv2.cuda.GpuMat inputs
            process_start = time.time()
            remapped_img = cv2.cuda.remap(gpu_src_img, gpu_x_src, gpu_y_src, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            process_end = time.time()
            # print(f"[DEBUG] Processing time: {process_end - process_start:.4f} seconds")
            remapped_img = cp.asarray(remapped_img.download())
            
            if index == 0:
                ref_img[:, :w_src] = remapped_img[:, :w_src]
            else:
                ref_img[:, -w_src:] = remapped_img[:, -w_src:]

            return ref_img
        else:
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
        # x = np.tile(np.arange(width), (height, 1))
        x = np.arange(width, dtype=np.float32)[None, :]
        depth_cali_img = stitched_img.copy()

        # plane 1 → 2: x = 1~360 → x[:, 0:360]
        x1 = x[:, 0:360]
        z1 = depth_cali_img[:, 0:360]
        z_trans_1 = (z1 - (PLANE1[0] * x1 + PLANE1[1])) / np.sqrt(PLANE1[0]**2 + 1) + PLANE2
        depth_cali_img[:, 0:360] = (z_trans_1 - PLANE2) * PLANE1_SCALE + PLANE2

        # plane 3 → 2: x = 721~1080 → x[:, 721:1081]
        right_start = 721
        right_end = min(1081, width)  
        x3 = x[:, right_start:right_end]
        z3 = depth_cali_img[:, right_start:right_end]
        z_trans_3 = (z3 - (PLANE3[0] * x3 + PLANE3[1])) / np.sqrt(PLANE3[0]**2 + 1) + PLANE2
        depth_cali_img[:, right_start:right_end] = (z_trans_3 - PLANE2) * PLANE3_SCALE + PLANE2 - 20

        depth_cali_img[depth_cali_img < 0] = 0

        return depth_cali_img
