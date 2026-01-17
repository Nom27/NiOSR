from setuptools import find_packages, setup

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='janek.horabik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'camera_node = camera_subscriber.camera_node:main',
        'turtlebot_rotate = camera_subscriber.turtlebot_rotate:main',
        'point_to_cmd_vel = camera_subscriber.point_to_cmd_vel:main',
    ],
},
)
