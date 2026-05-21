from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'helper_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiming',
    maintainer_email='seojimni@gmail.com',
    description=(
        'Perception package for helper robot sensor processing and obstacle '
        'detection nodes.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_filter_node = helper_perception.scan_filter_node:main',
            'obstacle_detector_node = helper_perception.obstacle_detector_node:main',
            'depth_obstacle_detector_node = helper_perception.depth_obstacle_detector_node:main',
            'obstacle_fusion_node = helper_perception.obstacle_fusion_node:main',
        ],
    },
)
