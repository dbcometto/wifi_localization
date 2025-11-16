from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wifi_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yaml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Albertson',
    maintainer_email='albertson.j@northeastern.edu',
    description='Node that publishes currently broadcasting Wi-Fi networks\' BSSIDs, RSSI values, and variance to /wifi topic at 0.2Hz',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_driver = wifi_driver.wifi_driver_node:main',
        ],
    },
)
