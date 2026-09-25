from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tram_odometry_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'tram_map.pkl']),
    ],
    install_requires=['setuptools', 'numpy', 'pyproj'],
    zip_safe=True,
    maintainer='Hacker',
    maintainer_email='hacker@example.com',
    description='Fallback odometry node for tram without GNSS/IMU',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = tram_odometry_pkg.odometry_node:main'
        ],
    },
)
