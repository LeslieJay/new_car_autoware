from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vehicle_video_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 添加 launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.xml')),

        # 添加 config 文件
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='byd',
    maintainer_email='704064616@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'recorder = vehicle_video_recorder.recorder_node:main',
        ],
    },
)
