from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # ✅ THIS FIXES THE ERROR

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rainer',
    maintainer_email='rainerrodrigues16@gmail.com',
    description='ROS 2 pick-and-place task package',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'move_arm_node = pick_and_place.move_arm:main',
            'joint_recorder = pick_and_place.joint_recorder:main',
            'success_checker = pick_and_place.success_checker:main',
            'screenshotter = pick_and_place.screenshotter:main',
        ],
    },
)

