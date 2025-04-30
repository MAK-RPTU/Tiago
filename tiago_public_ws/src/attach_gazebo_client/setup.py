from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'attach_gazebo_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv'))
    ],
    install_requires=['setuptools', 'rclpy', 'gazebo_msgs', 'std_msgs', 'control_msgs'],
    zip_safe=True,
    maintainer='mak',
    maintainer_email='meeranali-khan@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attach_gazebo = attach_gazebo_client.attach_gazebo:main'
        ],
    },
)
