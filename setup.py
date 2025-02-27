from setuptools import setup

package_name = 'px4_offboard_hover'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='PX4 Offboard Hovering using MAVROS in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_offboard_hover = px4_offboard_hover.px4_offboard_hover:main',
        ],
    },
)
