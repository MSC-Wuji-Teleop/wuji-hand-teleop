from setuptools import find_packages, setup

package_name = 'replay'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Nathan Jew',
    maintainer_email='nathan.jew@berkeley.edu',
    description=(
        'Plays a prepared clip directory (clips/safe/<clip>/, written by tools/prepare_clip.py, or clips/home/<clip>/, written by tools/make_home_clip.py): waits for consumers, approaches frame 0, then named arm joint targets to g1_world_output in joint_replay mode and named hand joints to the starport_wuji_hand drivers at 100 Hz, once, then holds the last frame. Also the connection check that waits for state from those nodes before a run, and the one-shot arm pose capture the rehome command reads (docs/spec/spec1_1.md).'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_publisher = replay.replay_publisher:main',
            'replay_check = replay.replay_check:main',
            'capture_arm_pose = replay.capture_arm_pose:main',
        ],
    },
)
