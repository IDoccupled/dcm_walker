from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dcm_walker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*rviz*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaoran',
    maintainer_email='xiaoran@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visual_node = dcm_walker.dcm_walker_visual_node:main',
        ],
    },
)
