from glob import glob

from setuptools import find_packages, setup

package_name = 'raspimouse_python_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='YusukeKato',
    maintainer_email='yusukekato.contact@gmail.com',
    description='ROS 2 node for Raspberry Pi Mouse built on raspimouse_python_driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'raspimouse_node = raspimouse_python_ros2.raspimouse_node:main',
        ],
    },
)
