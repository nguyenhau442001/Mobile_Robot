from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'mobile_robot_custom_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    py_modules=[
        'go_to_goal_server',
        'go_to_goal_client',
        'goal_bridge_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Hau Nguyen Ngoc',
    maintainer_email='haunguyenngoc442001@gmail.com',
    description='Custom go-to-goal navigation for the mobile robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_goal_server = go_to_goal_server:main',
            'go_to_goal_client = go_to_goal_client:main',
            'goal_bridge_node  = goal_bridge_node:main',
        ],
    },
)
