import os
from glob import glob
from setuptools import setup

package_name = 'open_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'src'), glob('src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teemu',
    maintainer_email='teemu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'drone = open_project.drone_function:main',
                'robot = open_project.robot_function:main',
                'takeoff_client_node = open_project.takeoff_client_node:main',
        ],
    },
)
