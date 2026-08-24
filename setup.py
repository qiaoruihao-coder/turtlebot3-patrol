import os
from glob import glob
from setuptools import setup

package_name = 'tb3_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_tree'), glob('behavior_tree/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qiao',
    maintainer_email='qiaoruihao20061203@qq.com',
    description='TurtleBot3 自主巡检包：代码驱动的多目标点导航',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol = tb3_patrol.patrol_node:main',
            'spawn_obstacle = tb3_patrol.spawn_obstacle:main',
        ],
    },
)
