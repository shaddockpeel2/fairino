from setuptools import find_packages, setup


package_name = 'fr3c_gripper_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/ydi_j22_ft232h.yaml']),
        ('share/' + package_name + '/launch', ['launch/gripper.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='FR3-C project',
    maintainer_email='developer@example.com',
    description='Guarded ROS 2 GripperCommand driver for YDI-J22 via FT232H PWM.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = fr3c_gripper_driver.node:main',
        ],
    },
)
