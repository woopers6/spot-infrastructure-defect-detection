from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'spot_eap_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaradar',
    maintainer_email='arunvaradarajan3@gmail.com',
    description='ROS 2 network bridge for Spot EAP LiDAR point clouds.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pointcloud_bridge = '
            'spot_eap_bridge.pointcloud_bridge:main',
        ],
    },
)
