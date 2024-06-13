import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'team5_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Saimai Lau',
    author_email='s7lau@ucsd.edu',
    maintainer='Saimai Lau',
    maintainer_email='s7lau@ucsd.edu',
    description='MAE 148 Team 5 Final Project ROS2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_tennis_ball = team5_project.find_tennis_ball:main',
            'follow_ball = team5_project.follow_ball:main',
            'plot_stuff = team5_project.plot_stuff:main',
            'sim = team5_project.simulation:main',
            'odom = team5_project.odom:main',
        ],
    },
)
