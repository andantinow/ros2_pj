from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Launch 파일 설치
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # 2. Config 파일 설치 (이 부분이 중요합니다!)
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='misys',
    maintainer_email='misys@todo.todo',
    description='Main launcher for the Masterpiece Stack',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
