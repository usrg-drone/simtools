from setuptools import find_packages, setup

package_name = 'simtools'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_merge = simtools.lidar_merge:main',
            'lidar_repub = simtools.lidar_repub:main',
            'pose_relay = simtools.pose_relay:main',
            'tf_from_odom = simtools.tf_from_odom:main',
        ],
    },
)
