from setuptools import setup

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='intern',
    description='Pick and place node for UR5',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'move_arm = ur5_pick_place.move_arm:main'
        ],
    },
)
