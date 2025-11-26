from setuptools import setup

package_name = 'ap1_control_interface'  # ROS package name

setup(
    name=package_name,                  # Python distribution name
    version='0.2.0',
    # Python package in this repo
    packages=['control_interface'],
    package_dir={'control_interface': 'control_interface'},
    # Include style.css inside the installed control_interface package
    package_data={'control_interface': ['style.css']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='obaidmm',
    maintainer_email='you@example.com',
    description='AP1 control debug UI',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run ap1_control_interface system_interface
            'system_interface = control_interface.main:main',
        ],
    },
)
