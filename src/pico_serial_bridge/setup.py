from setuptools import setup

package_name = 'pico_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mallu',
    maintainer_email='you@example.com',
    description='ROS 2 to Pico serial bridge for motor control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_serial_bridge = pico_serial_bridge.cmd_vel_serial_bridge:main',
        ],
    },
)
