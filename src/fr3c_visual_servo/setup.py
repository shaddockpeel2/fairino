from glob import glob

from setuptools import find_packages, setup


package_name = 'fr3c_visual_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='FR3-C project',
    maintainer_email='developer@example.com',
    description='Leased 2-D color-target MoveIt Servo action server.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'track_2d_action_server = '
            'fr3c_visual_servo.track_2d_action_server:main',
        ],
    },
)
