import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    package_data={
        package_name: ['chandrayaan_task_catalog.json'],
    },
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='user@todo.todo',
    description='Core rover control and communication package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_node = rover_core.rover_node:main',
            'earth_node = rover_core.earth_node:main',
            'space_link_node = rover_core.space_link_node:main',
            'telemetry_monitor = rover_core.telemetry_monitor:main',
            'fleet_manager = rover_core.fleet_manager:main',
        ],
    },
)
