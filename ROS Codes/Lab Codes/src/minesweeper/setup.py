from setuptools import setup

package_name = 'minesweeper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Joystick Publisher and Subscriber using ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joystick_publisher = minesweeper.joystick_publisher:main',
            'joystick_subscriber = minesweeper.joystick_subscriber:main',
            'mine_detection_subscriber = minesweeper.mine_detection_subscriber:main',
            'mine_detection_publisher = minesweeper.mine_detection_publisher:main',
            'mine_mapping_subscriber = minesweeper.mine_mapping_subscriber:main',
            'mine_mapping_publisher = minesweeper.mine_mapping_publisher:main',
        ],
    },
)
