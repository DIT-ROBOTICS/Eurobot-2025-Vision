from rclpy.node import Node
import numpy as np 
import cv2

inv_homography = [
    np.array([
        [ 4.53804488e-01,  6.08931797e-03,  2.02711314e+01],
        [-2.18362987e-01,  6.74989251e-01,  1.24935563e+02],
        [-5.36737774e-04, -8.60158701e-06,  9.75746254e-01]
    ]), 
    np.array([
        [ 1.10186958e+00, -1.31769775e-02, -9.08180545e+02],
        [ 3.33107011e-01,  1.04094260e+00, -2.81665698e+02],
        [ 7.96767113e-04, -2.37587557e-05,  3.43387211e-01]
    ])
]

class VideoStitcher(Node):
    def __init__(self):
        super().__init__('video_stitcher')
        self.coord_cache = {}

    def rearrange_list(self, images):
        return [images[1], images[0], images[2]]

    def warp(self, images):
        images = self.rearrange_list(images)
        # Expand the canvas to fit all images
        height, width = images[0].shape[:2]
        canvas = np.zeros((height, width * 3, 3), dtype=np.uint8)
        canvas[:, 1 * width:2 * width] = images[0]
       
        for i, inv_H in enumerate(inv_homography):
            canvas = self.warp_logic(canvas, images[i + 1], inv_H, i)
        
        return canvas

    def warp_logic(self, ref_img, src_img, inv_H, index):
        h_dst, w_dst = 848, 1440
        h_src, w_src = 848, 480

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

# from rclpy.node import Node
# import numpy as np
# import cv2
# import threading
# import rclpy

# # Homography matrices for warping images
# inv_homography = [
#     np.array([
#         [4.53804488e-01, 6.08931797e-03, 2.02711314e+01],
#         [-2.18362987e-01, 6.74989251e-01, 1.24935563e+02],
#         [-5.36737774e-04, -8.60158701e-06, 9.75746254e-01]
#     ]),
#     np.array([
#         [1.10186958e+00, -1.31769775e-02, -9.08180545e+02],
#         [3.33107011e-01, 1.04094260e+00, -2.81665698e+02],
#         [7.96767113e-04, -2.37587557e-05, 3.43387211e-01]
#     ])
# ]

# class VideoStitcher(Node):
#     def __init__(self):
#         super().__init__('video_stitcher')
#         self.coord_cache = {}
#         self.lock = threading.Lock()

#     def rearrange_list(self, images):
#         return [images[1], images[0], images[2]]

#     def warp(self, images):
#         images = self.rearrange_list(images)

#         # Expand the canvas to fit all images
#         height, width = images[0].shape[:2]
#         canvas = np.zeros((height, width * 3, 3), dtype=np.uint8)
#         canvas[:, 1 * width:2 * width] = images[0]

#         # Start two threads to warp images concurrently
#         thread1 = threading.Thread(target=self.warp_thread, args=(canvas, images[1], inv_homography[0], 0))
#         thread2 = threading.Thread(target=self.warp_thread, args=(canvas, images[2], inv_homography[1], 1))

#         thread1.start()
#         thread2.start()

#         thread1.join()
#         thread2.join()

#         return canvas

#     def warp_thread(self, canvas, img, inv_H, index):
#         # This method performs the warping logic, same as in original warp_logic
#         h_dst, w_dst = 848, 1440
#         h_src, w_src = 848, 480

#         cache_key = (h_dst, w_dst, inv_H.tobytes())

#         # Cache the transformed coordinates for efficiency
#         if cache_key not in self.coord_cache:
#             map_x, map_y = np.meshgrid(np.arange(w_dst), np.arange(h_dst))
#             map_x = map_x.flatten()
#             map_y = map_y.flatten()
#             coords = np.vstack((map_x, map_y, np.ones_like(map_x)))

#             mapped_coords = inv_H @ coords
#             mapped_coords /= mapped_coords[2, :]

#             x_src = mapped_coords[0, :].reshape(h_dst, w_dst).astype(np.float32)
#             y_src = mapped_coords[1, :].reshape(h_dst, w_dst).astype(np.float32)

#             self.coord_cache[cache_key] = (x_src, y_src)
#         else:
#             x_src, y_src = self.coord_cache[cache_key]

#         remapped_img = cv2.remap(img, x_src, y_src, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)

#         # Merge the remapped image with the canvas based on the index
#         if index == 0:
#             canvas[:, :w_src] = remapped_img[:, :w_src]
#         else:
#             canvas[:, -w_src:] = remapped_img[:, -w_src:]

#         return canvas
