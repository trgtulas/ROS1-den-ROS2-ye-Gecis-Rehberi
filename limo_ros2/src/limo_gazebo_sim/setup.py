from setuptools import setup
import os
from glob import glob

package_name = 'limo_gazebo_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOU',
    maintainer_email='you@example.com',
    description='Teleop node for Limo robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'test_node = limo_gazebo_sim.test_node:main',
        ],
    },
)
