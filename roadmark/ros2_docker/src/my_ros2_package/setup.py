from setuptools import setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A description of your package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_image_publisher = my_ros2_package.camera_image_publisher:main',
        ],
    },
)
