from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rock_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    package_data={
        package_name: ['models/*.pt'],
    },
    zip_safe=False,  
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Mars rock detection using YOLOv11',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'video_publisher = rock_detection.video_publisher:main',
            'rock_detector = rock_detection.rock_detector:main',
        ],
    },
)