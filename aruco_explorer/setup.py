import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='mateusz.stasiak@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_points_generator = aruco_explorer.map_points_generator:main',
            'map_points_navigation = aruco_explorer.map_points_navigation:main',
            'aruco_tracker = aruco_explorer.aruco_tracker:main',
        ],
    },
)
