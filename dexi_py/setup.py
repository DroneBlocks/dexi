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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='droneblocks',
    maintainer_email='droneblocks@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service = dexi_py.led:main',
            'offboard_node = dexi_py.offboard:main',
            'processes = dexi_py.processes:main'
        ],
    },
)
