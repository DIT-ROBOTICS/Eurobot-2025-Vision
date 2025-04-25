from setuptools import find_packages, setup

package_name = 'center_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='center',
    maintainer_email='ohin.kyuu@gmail.com',
    description='Vision Center ROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'score_service = center_score.score_service:main',
        ],
    },
)
