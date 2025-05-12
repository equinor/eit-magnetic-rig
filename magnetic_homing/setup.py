import os  
from glob import glob
from setuptools import find_packages, setup

package_name = 'magnetic_homing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas',
    maintainer_email='jonas.peter.sorensen@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cnc_node = magnetic_homing.cnc_controller_node:main",
            "magnetic_sensor_node = magnetic_homing.magnetic_sensor_node:main",
            "cnc_tui = magnetic_homing.scripts.cnc_tui_node:main"
        ],
    },
)