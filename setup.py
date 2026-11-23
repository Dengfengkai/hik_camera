from setuptools import setup
import os
from glob import glob

package_name = 'hik_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='lys',
    maintainer_email='lys@todo.todo',
    description='Hikrobot camera ROS 2 driver using MVS SDK',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hik_camera_node = hik_camera.hik_camera_node:main',
        ],
    },
)
