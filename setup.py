import os
from glob import glob
from setuptools import setup

package_name = 'ramabot_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kizuna',
    maintainer_email='kizuna@ramabot.local',
    description='Standalone mapping bringup for RAMABOT V3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_odom_bridge = ramabot_mapping.mapping_odom_bridge:main',
            'wsl_serial_mock = ramabot_mapping.wsl_serial_mock:main',
        ],
    },
)
