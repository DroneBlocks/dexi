from setuptools import find_packages, setup

package_name = 'dexi_led_strip'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hunter Baker',
    maintainer_email='hunterbaker@me.com',
    description='A simple node to control the led strip on the DEXI drone using SPI_1 on the CM4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_strip_node = dexi_led_strip.led_strip_node:main'
        ],
    },
)
