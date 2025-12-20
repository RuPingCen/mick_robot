from setuptools import setup
import os
from glob import glob

package_name = 'path_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Path tracking package with Pure Pursuit and Stanley controllers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_path_publisher = path_tracking.circle_path_publisher:main',
            'pure_pursuit_controller = path_tracking.pure_pursuit_controller:main',
            'stanley_controller = path_tracking.stanley_controller:main',
        ],
    },
)
