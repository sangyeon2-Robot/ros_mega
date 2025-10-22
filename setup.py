from setuptools import setup
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'ros_mega'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resource'), ['resource/ros_mega']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sss',
    maintainer_email='sss@todo.todo',
    description='/cmd_vel to Arduino Mega (USB serial) bridge',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_cmdvel_bridge = ros_mega.serial_cmdvel_bridge:main',
        ],
    },
)
