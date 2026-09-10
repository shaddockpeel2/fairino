from setuptools import setup


setup(
    name='fr3c_calibration',
    version='0.1.0',
    packages=['fr3c_calibration'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fr3c_calibration']),
        ('share/fr3c_calibration', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FR3-C project',
    maintainer_email='developer@example.com',
    description='Read-only hand-eye and TCP calibration tools for FR3-C.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'handeye_analyze = fr3c_calibration.analyze_session:main',
            'handeye_capture = fr3c_calibration.capture_once:main',
            'handeye_session = fr3c_calibration.handeye_session:main',
            'handeye_pose_crosscheck = fr3c_calibration.pose_crosscheck:main',
            'tcp_pivot_capture = fr3c_calibration.tcp_pivot:capture_main',
            'tcp_pivot_analyze = fr3c_calibration.tcp_pivot:analyze_main',
        ],
    },
)
