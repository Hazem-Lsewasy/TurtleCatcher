from setuptools import find_packages, setup

package_name = 'turtlesim_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['publisher.py']),
        ('lib/' + package_name, ['spawner.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hazem',
    maintainer_email='hazem@todo.todo',
    description='ROS2 package for turtlesim simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = turtlesim_simulator.publisher:main',
            'spawner = turtlesim_simulator.spawner:main'
        ],
    },
)
