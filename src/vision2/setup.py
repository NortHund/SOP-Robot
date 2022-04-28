from setuptools import setup
from glob import glob
import os

package_name = 'vision2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vagrant',
    maintainer_email='vagrant@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision2_node = vision2.vision2_node:main',
            'object_tracker_node = vision2.object_tracker_node:main',
            'expression_detection_node = vision2.expression_detection_node:main'
        ],
    },
)
