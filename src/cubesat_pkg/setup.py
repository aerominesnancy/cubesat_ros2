from setuptools import find_packages, setup

package_name = 'cubesat_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_cubesat.py']),
    ],


    install_requires=['setuptools', 
                      'adafruit-bno055', 
                      'adafruit-extended-bus',
                      "RPi.GPIO",
                      "time"],


    zip_safe=True,
    maintainer='cubesat',
    maintainer_email='cubesat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            "imu_node = cubesat_pkg.IMU_node:main",
            "motor_node = cubesat_pkg.motor_node:main",
        ],
    },
)
