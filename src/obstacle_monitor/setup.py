from setuptools import find_packages, setup

package_name = 'obstacle_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flash',
    maintainer_email='flash@todo.todo',
    description='Forward ultrasonic obstacle detection (stop and wait)',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_monitor_node = obstacle_monitor.obstacle_monitor_node:main',
        ],
    },
)
