from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gavin',
    maintainer_email='gavin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'sim = holoocean_ros.sim:main',
            'manual = holoocean_ros.manual:main',
            'migeran = holoocean_ros.migeran:main',
        ],
    },
)
