import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dexi_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dennis Baldwin',
    maintainer_email='db@droneblocks.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service = dexi_py.led_service:main',
            'offboard_node = dexi_py.offboard:main',
            'led_mock_service = dexi_py.led_mock:main',
            'flight_mode_status = dexi_py.led_flight_mode_status:main',
            'gpio_writer_service = dexi_py.gpio_writer_service:main',
            'gpio_reader = dexi_py.gpio_reader:main',
            'servo_controller = dexi_py.servo_controller:main'
        ],
    },
)
