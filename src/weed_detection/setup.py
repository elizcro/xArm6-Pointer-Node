from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'weed_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan',
    maintainer_email='evanfost@andrew.cmu.edu',
    description='Weed detection node using green segmentation and DBSCAN clustering',
    license='MIT',
    entry_points={
        'console_scripts': [
            'weed_detector = weed_detection.weed_detector_node:main',
            'weed_sequencer = weed_detection.weed_sequencer_node:main',
        ],
    },
)
