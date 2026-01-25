from setuptools import setup
import os
from glob import glob

package_name = 'tiago_exam'

setup(
    name=package_name,
    version='4.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giulio',
    description='Task esame TIAGo',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tuck_arm = tiago_exam.tuck_arm:main',
            'map_saver_watcher = tiago_exam.map_saver_watcher:main',
            'localization = tiago_exam.localization:main',
            'marker_searcher = tiago_exam.marker_searcher:main',
            'explorer = tiago_exam.naive_explorer:main',
            'aruco_noter = tiago_exam.aruco_noter:main',
        ],
    },
)