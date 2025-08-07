import os
from glob import glob
from setuptools import setup

package_name = 'chimera_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        "opencv-python",
        "torch",
        "numpy",
        "ultralytics",
        "setuptools"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pt'))),
    ],
    zip_safe=True,
    maintainer='jaketucker',
    maintainer_email='tuckerjake11@gmail.com',
    description='Package to get camera data from Chimera',
    license='Apache-2.0',  
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'camera_node = chimera_camera.CameraPublisher:main',
        'client = chimera_camera.CameraPublisherTest:main',
        'yolo_node = chimera_camera.detect:main',
        'multi_camera_publisher = chimera_camera.multi_cam_test:main',
    ],
},

)
