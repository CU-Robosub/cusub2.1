from setuptools import find_packages, setup

package_name = 'cv_detection_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Computer Vision Detection Package with YOLO and ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_detection_node = cv_detection_package.cv_detection_node:main',
            'cv_detection_subscriber = cv_detection_package.cv_detection_subscriber:main',
        ],
    },
)