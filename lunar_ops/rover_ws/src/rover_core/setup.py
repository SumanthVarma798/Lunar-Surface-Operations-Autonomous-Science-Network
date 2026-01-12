from setuptools import setup, find_packages

package_name = 'rover_core'

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
    maintainer='TODO',
    maintainer_email='user@todo.todo',
    description='Core rover control and communication package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_node = rover_core.rover_node:main',
            'earth_node = rover_core.earth_node:main',
        ],
    },
)
