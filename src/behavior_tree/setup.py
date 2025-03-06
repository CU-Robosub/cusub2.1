from setuptools import find_packages, setup

package_name = 'behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        f'behavior_tree.BehaviorTree', 
        f'behavior_tree.publisher_member_function',
        f'behavior_tree.subscriber_member_function'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owenr',
    maintainer_email='owen.r.welsh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = behavior_tree.publisher_member_function:main',
                'listener = behavior_tree.subscriber_member_function:main',
                'example_tree = behavior_tree.exampleTree:main'
        ],
    },
)
