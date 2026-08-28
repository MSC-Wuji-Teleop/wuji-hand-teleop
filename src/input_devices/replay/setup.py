from setuptools import setup, find_packages

package_name = 'replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates',
            ['templates/hardware_manifest_template.json']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',   # PCHIP retime in conditioning (hand q20 -> arm grid)
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Nathan Jew',
    maintainer_email='nathan.jew@berkeley.edu',
    description='SOT handoff bundle replay input device (arm joint targets + hand keypoints)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_publisher = replay.replay_publisher:main',
            'condition_clip = replay.condition_clip:main',
            'supervisor = replay.supervisor:main',
            'run_ctl = replay.run_ctl:main',
            'choose_first_clip = replay.choose_first_clip:main',
            'make_artifacts = replay.make_artifacts:main',
        ],
    },
)
