from setuptools import find_packages, setup

package_name = 'turtlebot_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Add package metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch files
        ('share/' + package_name + '/launch', ['launch/spawn_turtlebot3_custom.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_custom_house.launch.py']),

        # Include model files (SDF, config)
        ('share/' + package_name + '/models/custom_turtlebot3', [
            'models/custom_turtlebot3/model.sdf',
            'models/custom_turtlebot3/model.config',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='sergioromero48@outlook.com',
    description='A custom package for TurtleBot simulation and SLAM integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_slam = turtlebot_slam.turtlebot_slam:main'
        ],
    },
)
