from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylin',
    maintainer_email='kylingithubdev@gmail.com',
    description='This is a pkg to control PCar.',
    license='Commercial License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_reader = robot_control.arm_reader:main',
            'arm_writer = robot_control.arm_writer:main',
            'arm_keyboard = robot_control.arm_keyboard:main'
        ],
    },
)
