from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wifi_launch'

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
    description='Launch files for the wifi localization system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
