from setuptools import find_packages, setup

package_name = 'camera-ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'stitching', 'numpy'],
    zip_safe=True,
    maintainer='ohin',
    maintainer_email='ohin.kyuu@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_logger_node = camera_snap.sp_camera:main',
            'camera_merger_node = camera_snap.sp_multi_camera:main',
            'image_stitcher_node = image_stitch.sti_video:main',
            'get_node = image_stitch.get:main',
            'photo_node = photo_pkg.photo:main',
        ],
    },
)
