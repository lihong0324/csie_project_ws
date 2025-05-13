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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detection_control_node = my_robot_car.scripts.fire_detection_control_node:main',
            'ball_detector = my_robot_car.scripts.ball_detector:main',
            'fire_sample = my_robot_car.scripts.fire_sample:main',
            'patrol_node = my_robot_car.scripts.patrol_node:main',  # 新增這行
        ],
    },
)