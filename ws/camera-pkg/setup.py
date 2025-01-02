from setuptools import find_packages, setup

package_name = 'camera-ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy'],
    zip_safe=True,
    maintainer='ohin',
    maintainer_email='ohin.kyuu@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_merger_node = camera_merger.oc_multi_camera_merged:main',
        ],
    },
)
