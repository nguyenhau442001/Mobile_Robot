"""Setup for the mobile_robot_control package."""
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'mobile_robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['math_utils'],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hau Nguyen Ngoc',
    maintainer_email='haunguyenngoc442001@gmail.com',
    description='Motion controllers and teleoperation for the mobile robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = teleop.teleop_node:main',
        ],
    },
)
