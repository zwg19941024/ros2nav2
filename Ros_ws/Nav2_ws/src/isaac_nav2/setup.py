from setuptools import find_packages, setup
from glob import glob
package_name = 'isaac_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob('launch/*.*')),
        ('share/' + package_name+'/config', glob('config/*.yaml')),
        ('share/' + package_name+'/maps', glob('maps/*.*')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='zwg',
    maintainer_email='zwg@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_robot_pose = isaac_nav2.init_robot_pose:main',
            'nav2_mqtt_client = isaac_nav2.nav2_mqtt_client:main',

        ],
    },
)
