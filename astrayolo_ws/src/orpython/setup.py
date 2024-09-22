from setuptools import setup

package_name = 'orpython'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOv8 ROS2 Python package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolov8_node = orpython.yolov8_node:main',
            'yolov8_tracking_node = orpython.yolov8_tracking_node:main',
        ],
    },
)

