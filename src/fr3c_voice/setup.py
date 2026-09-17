from setuptools import find_packages, setup


setup(
    name='fr3c_voice',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fr3c_voice']),
        ('share/fr3c_voice', ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='FR3-C project',
    maintainer_email='developer@example.com',
    description='Offline Qwen3-VL service and RK3588 resource coordination',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qwen_service = fr3c_voice.service:main',
            'qwen_client = fr3c_voice.client:main',
        ],
    },
)
