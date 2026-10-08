from glob import glob
from setuptools import setup

setup(
    name='fr3c_target_follow', version='0.1.0', packages=['fr3c_target_follow'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fr3c_target_follow']),
        ('share/fr3c_target_follow', ['package.xml']),
        ('share/fr3c_target_follow/config', glob('config/*.yaml')),
        ('share/fr3c_target_follow/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='FR3-C project', maintainer_email='developer@example.com',
    description='Fiducial pose measurement and target following', license='Apache-2.0',
    entry_points={'console_scripts': [
        'tag_pose = fr3c_target_follow.pose_node:main',
        'pose_audit = fr3c_target_follow.audit:main',
        'pose_view = fr3c_target_follow.viewer:main',
        'follow_preview = fr3c_target_follow.preview_node:main',
        'preview_audit = fr3c_target_follow.preview_audit:main',
        'follow_control = fr3c_target_follow.control:main',
    ]},
)
