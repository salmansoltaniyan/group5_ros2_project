import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bug2_navigation_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='salman@todo.todo',
    description='Autonomous Bug2 Wall-Following and Navigation Package for iRobot Create 3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bug2_wall_follow_server = bug2_navigation_pkg.bug2_wall_follow_server:main',
            'line_follow_node = bug2_navigation_pkg.line_follow_node:main',
        ],
    },
)

