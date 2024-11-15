from setuptools import find_packages, setup

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning, module="setuptools")


package_name = 'uav_py'

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
    maintainer='chisom',
    maintainer_email='tgwilliamsc@gmail.com',
    description='This package is designed to support UAV-related operations within ROS 2 environment. It includes functionalities for computer vision processing, UAV tracking, and other related tasks. The package provides ROS 2 nodes, such as vision_node, which is intended to handle vision-based algorithms, including object detection and image streaming.',
    license='MIT',
#    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'vision_node = uav_py.vision_node:main'
        #'aruco_tracking_node = uav_py.aruco_tracking_node:main'
        ],
    },
)
