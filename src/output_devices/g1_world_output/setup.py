from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'g1_world_output'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'scipy>=1.8.0',
        'PyYAML',
        # pinocchio + casadi are intentionally NOT listed here: no PyPI `pin`
        # wheel ships `pinocchio.casadi` (verified across 4.1.0/4.0.0/3.8.0),
        # so a plain `pip install pin casadi` can never satisfy this import.
        # This package's Docker image (docker/Dockerfile) instead installs
        # the robotpkg apt build of Pinocchio+CasADi, which does include it.
    ],
    zip_safe=False,
    maintainer='Nathan Jew',
    maintainer_email='nathan.jew@berkeley.edu',
    description='Unitree G1 World Output - ROS REP 103 compliant output node for PICO teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'g1_world_output_node = g1_world_output.g1_world_output_node:main',
            # g1_joint_replay_node was folded into g1_world_output_node's
            # 'joint_replay' mode; see deprecated/g1_joint_replay_node.py.
        ],
    },
)
