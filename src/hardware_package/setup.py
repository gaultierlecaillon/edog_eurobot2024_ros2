from setuptools import setup

package_name = 'hardware_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "reset1_publisher = hardware_package.reset1_publisher:main",
            "lidar_publisher = hardware_package.lidar_publisher:main",
            "tirette_publisher = hardware_package.tirette_publisher:main"
        ],
    },
)
