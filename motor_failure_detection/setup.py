from setuptools import find_packages, setup

package_name = 'motor_failure_detection'

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
    maintainer='jpm',
    maintainer_email='xyz@xyz.com',
    description='Motor Failure detection package ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motor_failure_plugin_node = motor_failure_detection.motor_failure_detection:main',
        ],
    },
)
