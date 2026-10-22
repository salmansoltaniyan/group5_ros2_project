import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bug2_distributed'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='salman@todo.todo',
    description='Distributed 3-Node Architecture for Bug2 Maze Navigation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'go_to_goal_server = bug2_distributed.go_to_goal_server:main',
            'wall_follow_server = bug2_distributed.wall_follow_server:main',
            'bug2_coordinator = bug2_distributed.bug2_coordinator:main',
        ],
    },
)

