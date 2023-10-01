from setuptools import setup
from glob import glob


package_name = 'control_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edog',
    maintainer_email='gaultier.lecail@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_service = control_package.motion_service:main',
            'motion_complete_service = control_package.motion_complete_service:main',
            'lidar_filter = control_package.lidar_filter:main'
        ],
    },
)
