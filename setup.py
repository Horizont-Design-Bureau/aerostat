from setuptools import setup

package_name = 'aerostat_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/flight_controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sensor_fusion.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Novikov',
    maintainer_email='chlensouza350@gmail.com',
    description='Control system for aerostat model in Gazebo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = aerostat_control.flight_controller_aerostate:main',
            'sensor_fusion = aerostat_control.sensors_fusion:main',
        ],
    },
)