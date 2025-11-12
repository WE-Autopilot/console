from setuptools import find_packages, setup

package_name = 'ap1_control_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aly',
    maintainer_email='alyashour1@gmail.com',
    description='Control & debug interface for AP1',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_interface = control_interface.main:main',
        ],
    },
)
