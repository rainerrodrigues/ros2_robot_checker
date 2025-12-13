from setuptools import find_packages, setup

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rainer',
    maintainer_email='rainerrodrigues16@gmail.com',
    description='ROS 2 pick-and-place task package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This links the command 'move_arm_node' to the main function
            'move_arm_node = pick_and_place.move_arm:main',
        ],
    },
)
