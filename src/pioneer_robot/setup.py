from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pioneer_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), #finds .py
    #files paths for shit that needs to copy into the install directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resources', 'worlds'),
            glob('resources/worlds/*.sdf') + glob('resources/worlds/*.png')),
        (os.path.join('share', package_name, 'resources', 'robots'),
            glob('resources/robots/*.urdf')),
        (os.path.join('share', package_name, 'resources', 'meshes'),
            [f for f in glob('resources/meshes/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'resources', 'meshes', 'p3at_meshes'),
            [f for f in glob('resources/meshes/p3at_meshes/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resources', 'rviz'),
            glob('resources/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='winton',
    maintainer_email='winton@todo.todo',
    description='Pioneer robot nav',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    #ros2 run pioneer_robot waypoint_follower
    entry_points={
        'console_scripts': [
            'explorer    = pioneer_robot.explorer:main',
            'perception  = pioneer_robot.perception_node:main',
        ],
    },
)

