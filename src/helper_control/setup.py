from glob import glob

from setuptools import find_packages, setup

package_name = 'helper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiming',
    maintainer_email='seojimni@gmail.com',
    description='Control package for helper robot velocity commands and motion behavior nodes.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_odom = helper_control.fake_odom_node:main',
            'fake_scan = helper_control.fake_scan_node:main',
            'cmd_vel_safety_gate = helper_control.cmd_vel_safety_gate_node:main',
            'motor_driver = helper_control.motor_driver_node:main',
            'md200t_feedback_test = helper_control.md200t_feedback_test:main',
            'keyboard_teleop = helper_control.keyboard_teleop_node:main',
        ],
    },
)
