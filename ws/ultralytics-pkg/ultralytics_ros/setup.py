from setuptools import find_packages, setup

package_name = 'ultralytics_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_tf_launch.py']),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='ultralytics',
    maintainer_email='kmes100210@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'script = ultralytics_ros.script:main',
            'yolo_node = ultralytics_ros.yolo_node:main',
            'tf_node= ultralytics_ros.tf_node:main',
            'yolo_node_nopub = ultralytics_ros.yolo_node_nopub:main',
        ],
    },
)
