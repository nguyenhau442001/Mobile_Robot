"""Setup for the mobile_robot_monitor package."""
from setuptools import find_packages
from setuptools import setup

package_name = 'mobile_robot_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyqtgraph>=0.13', 'PyQt5>=5.15'],
    zip_safe=True,
    maintainer='Hau Nguyen Ngoc',
    maintainer_email='haunguyenngoc442001@gmail.com',
    description='Monitoring and plotting tools for the mobile robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'velocity_plotter = scripts.velocity_plotter:main',
        ],
    },
)
