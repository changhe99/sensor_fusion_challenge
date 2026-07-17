from glob import glob

from setuptools import find_packages, setup

package_name = 'bmi088_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chang',
    maintainer_email='robert.he1999@gmail.com',
    description='ROS 2 driver for the Bosch BMI088 (Shuttle Board 3.0) IMU.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bmi088_node = bmi088_driver.bmi088_node:main',
        ],
    },
)
