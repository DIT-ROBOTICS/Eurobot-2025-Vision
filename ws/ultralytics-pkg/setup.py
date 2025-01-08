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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ultralytics',
    maintainer_email='kmes100210@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'talker = ultralytics_ros.talker:main',
			'listener = ultralytics_ros.listener:main',
        ],
    },
)
