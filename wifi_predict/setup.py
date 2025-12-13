from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wifi_predict'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package registration
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # Config files (if any)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

        # Python scripts
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'scipy'],
    zip_safe=True,
    maintainer='Ananda Sangli',
    maintainer_email='sangli.@northeastern.edu',
    description='WiFi Predictor Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_predictor = wifi_predict.wifi_predict_node:main',
        ],
    },
)