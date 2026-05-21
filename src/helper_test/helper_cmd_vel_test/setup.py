from setuptools import find_packages, setup

package_name = 'helper_cmd_vel_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/cmd_vel_constant.launch.py',
            'launch/cmd_vel_sequence.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiming',
    maintainer_email='seojimni@gmail.com',
    description='Test publisher for /control/cmd_vel motor bring-up.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_test = helper_cmd_vel_test.cmd_vel_test_node:main',
        ],
    },
)
