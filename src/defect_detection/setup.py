from glob import glob
import os

from setuptools import find_namespace_packages, setup

package_name = 'defect_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(
        include=[package_name, package_name + '.*'],
        exclude=['test'],
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaradar',
    maintainer_email='arunvaradarajan3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = defect_detection.spot_cam_loading.image_publisher:main',
            'image_subscriber = defect_detection.spot_cam_loading.image_subscriber:main',
            'point_cloud_subscriber = '
            'defect_detection.pointcloud_loading.point_cloud_subscriber:main',
            'yolo_detector = defect_detection.defect_detection.yolo_detector:main',
            'fusion_node = defect_detection.defect_detection.fusion_node:main',
        ],
    },
)
