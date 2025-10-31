from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wifi_filter'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='dbcometto',
    maintainer_email='111076949+dbcometto@users.noreply.github.com',
    description='Filter nodes for wifi localization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kf_node = wifi_filter.wifi_kf_node:main',
            'lpf_node = wifi_filter.wifi_lpf_node:main',
            'driver_sim = wifi_filter.wifi_driver_sim:main',
            'estimate_sim = wifi_filter.wifi_estimate_sim:main',
        ],
    },
)
