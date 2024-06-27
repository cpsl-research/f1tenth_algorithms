from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auto_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Spencer Hallyburton',
    maintainer_email='spencer@shally.dev',
    description='Python based control for autonomous vehicles',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_control = auto_py.straight:main',
            'wiggle_control = auto_py.wiggle:main',
        ],
    },
)
