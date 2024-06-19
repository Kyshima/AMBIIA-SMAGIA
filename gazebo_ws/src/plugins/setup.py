import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'plugins'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.bash'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavo',
    maintainer_email='gustavo@todo.todo',
    description='Package hello',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = plugins.control_node:main',
            'humidity = plugins.humidity:main',
        ],
    },
)
