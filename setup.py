from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'data_gathering'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luka Distelbrink',
    maintainer_email='luka-groot@hotmail.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = data_gathering.data_collector:main',
            f'test_coordinator = {package_name}.test_coordinator:main',
            f'rosbag_controller = {package_name}.rosbag_controller:main',
        ],
    },
)
