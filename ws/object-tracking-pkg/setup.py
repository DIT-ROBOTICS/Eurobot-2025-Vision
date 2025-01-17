from setuptools import find_packages, setup

package_name = 'object-tracking-ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aruco',
    maintainer_email='aruco@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = object_tracking_ros.main:main',
            'cam = object_tracking_ros.cam_capture:main',
            'tracker = object_tracking_ros.tracker:main'
        ],
    },
)
