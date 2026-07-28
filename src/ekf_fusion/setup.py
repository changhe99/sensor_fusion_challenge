from glob import glob

from setuptools import find_packages, setup

package_name = 'ekf_fusion'

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
    description='Custom error-state EKF fusing BMI088 IMU + ZED-F9P GNSS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = ekf_fusion.ekf_node:main',
        ],
    },
)
