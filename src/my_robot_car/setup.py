from setuptools import setup

package_name = 'my_robot_car'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lihong',
    maintainer_email='leo930324@gmail.com',
    description='My Robot Car package',
    license='TODO: License declaration (e.g. MIT)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detection_control = my_robot_car.scripts.fire_detection_control_node:main',
            'fire_detector = my_robot_car.scripts.fire_detector_2:main',
            'fire_sample = my_robot_car.scripts.fire_sample:main',
        ],
    },

)