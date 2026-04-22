from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cf_trajectory_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashu',
    maintainer_email='yashdadheech3o6@gmail.com',
    description='Modular trajectory tracking controllers for Crazyflie 2.1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_controller_node = cf_trajectory_controller.nodes.cf_controller_node:main',
            'cf_gazebo_force_node = cf_trajectory_controller.nodes.cf_gazebo_force_node:main',
            'cf_trajectory_plotter = cf_trajectory_controller.utils.cf_trajectory_plotter:main',
        ],
    },
)
