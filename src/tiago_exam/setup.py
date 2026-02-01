from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tiago_exam'

setup(
    name=package_name,
    version='4.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'temp'), glob('temp/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giulio',
    description='Exam Tasks TIAGo',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # General Nodes
            'tuck_arm = tiago_exam.tuck_arm:main',
            'map_navigator = tiago_exam.map_navigator:main',
            'head_motion = tiago_exam.head_motion:main',
            'localization = tiago_exam.localization:main',
            
            # Task 1 Nodes
            'map_saver_watcher = tiago_exam.task1.map_saver_watcher:main',

            # Task 2 Nodes
            'coordinator_task2 = tiago_exam.task2.coordinator_task2:main',
            'aruco_tracker = tiago_exam.task2.aruco_tracker:main',
            'aruco_noter = tiago_exam.task2.aruco_noter:main',
            'naive_explorer = tiago_exam.task2.naive_explorer:main',
            
            # Task 3 Nodes
            'coordinator_task3 = tiago_exam.task3.coordinator_task3:main',              
            'manipulator = tiago_exam.task3.manipulator:main',           
            
        ],
    },
)