import os
from glob import glob
from setuptools import setup
# for setuptools, use version 58.2.0 to avoid stderr msg
from setuptools import find_packages

package_name = 'motor_control'
submodules = 'motor_control/submodules'
maestro = 'motor_control/submodules/Maestro'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules, maestro],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jake',
    maintainer_email='tuckerjake11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_convert = motor_control.cmd_convert:main',
            'joy_listener = motor_control.joyListener:main',
            'motor_kill = motor_control.motorKill:main'
        ],
    },
)
